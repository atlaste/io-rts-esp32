#include "ramses_frame.h"

#include <cstdio>
#include <cstring>
#include <ostream>

namespace evohome
{

    namespace
    {
        // Header byte bit layout - matches evofw3 message.c.
        constexpr uint8_t kHdrTMask  = 0x30;
        constexpr uint8_t kHdrTShift = 4;
        constexpr uint8_t kHdrAMask  = 0x0C;
        constexpr uint8_t kHdrAShift = 2;
        constexpr uint8_t kHdrParam0 = 0x02;
        constexpr uint8_t kHdrParam1 = 0x01;

        // Address-presence table indexed by (header & kHdrAMask) >> kHdrAShift:
        //   0 -> ALL three (addr0, addr1, addr2)
        //   1 -> only addr2
        //   2 -> addr0 + addr2
        //   3 -> addr0 + addr1
        struct AddrPresence { bool a0, a1, a2; };
        constexpr AddrPresence kAddrPresence[4] = {
            {true,  true,  true},
            {false, false, true},
            {true,  false, true},
            {true,  true,  false},
        };

        // Map raw verb bits (0..3) to enum. Same numeric encoding as Verb.
        Verb verb_from_bits(uint8_t bits) { return static_cast<Verb>(bits & 0x03); }
        uint8_t bits_from_verb(Verb v)    { return static_cast<uint8_t>(v) & 0x03; }

        constexpr DeviceAddr kNullAddr = {63, 0x3FFFF};

        // Compose the header byte from a Frame's logical fields. Inverse of
        // get_hdr_flags() in evofw3.
        uint8_t build_header(const Frame &f)
        {
            // Find the encoding for the addr0/1/2 presence pattern.
            uint8_t pattern = 0;
            for (uint8_t i = 0; i < 4; ++i)
            {
                const auto &p = kAddrPresence[i];
                if (p.a0 == f.has_addr[0] && p.a1 == f.has_addr[1] && p.a2 == f.has_addr[2])
                {
                    pattern = i;
                    break;
                }
            }
            uint8_t h = 0;
            h |= (bits_from_verb(f.verb) << kHdrTShift) & kHdrTMask;
            h |= (pattern << kHdrAShift)                & kHdrAMask;
            if (f.param0) h |= kHdrParam0;
            if (f.param1) h |= kHdrParam1;
            return h;
        }
    } // namespace

    uint8_t checksum(std::span<const uint8_t> data)
    {
        uint8_t s = 0;
        for (uint8_t b : data) s = static_cast<uint8_t>(s + b);
        return static_cast<uint8_t>(-static_cast<int>(s) & 0xFF);
    }

    std::optional<Frame> parse_frame(std::span<const uint8_t> data)
    {
        // Minimum valid frame: header(1) + at least one addr(3) + opcode(2)
        //                    + len(1) + payload(0) + csum(1) = 8 bytes.
        if (data.size() < 8) return std::nullopt;

        // Verify checksum first - it is cheap and a wrong CSUM means there is
        // no point spending cycles parsing the rest.
        uint8_t total = 0;
        for (uint8_t b : data) total = static_cast<uint8_t>(total + b);
        if (total != 0) return std::nullopt;

        Frame f;
        const uint8_t *p   = data.data();
        const uint8_t *end = data.data() + data.size() - 1; // exclude csum byte

        f.header_byte = *p++;
        f.verb = verb_from_bits((f.header_byte & kHdrTMask) >> kHdrTShift);

        const auto &pres = kAddrPresence[(f.header_byte & kHdrAMask) >> kHdrAShift];
        const bool present[3] = {pres.a0, pres.a1, pres.a2};

        for (int i = 0; i < 3; ++i)
        {
            f.has_addr[i] = present[i];
            if (present[i])
            {
                if (p + 3 > end) return std::nullopt;
                uint8_t buf[3] = {p[0], p[1], p[2]};
                f.addr[i] = DeviceAddr::from_wire(buf);
                p += 3;
            }
            else
            {
                f.addr[i] = kNullAddr;
            }
        }

        if (f.header_byte & kHdrParam0)
        {
            if (p >= end) return std::nullopt;
            f.param0 = *p++;
        }
        if (f.header_byte & kHdrParam1)
        {
            if (p >= end) return std::nullopt;
            f.param1 = *p++;
        }

        if (p + 2 > end) return std::nullopt;
        f.opcode = static_cast<uint16_t>((p[0] << 8) | p[1]);
        p += 2;

        if (p >= end) return std::nullopt;
        f.payload_len = *p++;
        if (f.payload_len > kMaxFramePayload) return std::nullopt;
        if (p + f.payload_len != end) return std::nullopt; // payload must fit exactly
        std::memcpy(f.payload.data(), p, f.payload_len);

        return f;
    }

    size_t serialize_frame(const Frame &f, std::span<uint8_t> out)
    {
        // Compute exact length first so we can early-out on overflow.
        size_t need = 1; // header
        for (int i = 0; i < 3; ++i) if (f.has_addr[i]) need += 3;
        if (f.param0) need += 1;
        if (f.param1) need += 1;
        need += 3; // opcode + length
        need += f.payload_len;
        need += 1; // checksum
        if (out.size() < need) return 0;

        uint8_t *p = out.data();
        *p++ = build_header(f);
        for (int i = 0; i < 3; ++i)
        {
            if (!f.has_addr[i]) continue;
            f.addr[i].to_wire(p);
            p += 3;
        }
        if (f.param0) *p++ = *f.param0;
        if (f.param1) *p++ = *f.param1;
        *p++ = static_cast<uint8_t>((f.opcode >> 8) & 0xFF);
        *p++ = static_cast<uint8_t>(f.opcode & 0xFF);
        *p++ = f.payload_len;
        if (f.payload_len) std::memcpy(p, f.payload.data(), f.payload_len);
        p += f.payload_len;

        // Checksum over everything written so far (header..payload inclusive).
        const size_t body_len = static_cast<size_t>(p - out.data());
        *p++ = checksum({out.data(), body_len});

        return static_cast<size_t>(p - out.data());
    }

    void Frame::describe_header(std::ostream &os) const
    {
        // " I --- 01:145038 --:------ 01:145038 30C9 003 ..."
        // We omit the leading "RSSI" field (evofw3 leaves it "---" when the
        // payload was generated locally) so call sites can prepend their own.
        os << to_string(verb) << " --- ";
        for (int i = 0; i < 3; ++i)
        {
            addr[i].describe(os);
            os << ' ';
        }
        char buf[8];
        std::snprintf(buf, sizeof(buf), "%04X", static_cast<unsigned>(opcode));
        os << buf << ' ';
        std::snprintf(buf, sizeof(buf), "%03u", static_cast<unsigned>(payload_len));
        os << buf;
    }

    void Frame::describe_friendly(std::ostream &os) const
    {
        // Pick a sensible "source" address. Most well-formed frames carry
        // the actual sender in addr0; some controller broadcasts only fill
        // addr2 (their own ID, repeated) and leave addr0/1 NUL. We accept
        // both and don't print the third slot to keep the line short.
        int src_idx = -1;
        if (has_addr[0] && !addr[0].is_null()) src_idx = 0;
        else if (has_addr[2] && !addr[2].is_null()) src_idx = 2;

        // Destination: addr1 if present, else addr2 if it's different from
        // the source we already chose (addr2 is sometimes a duplicate of
        // addr0, in which case the message is a self-announcement and we
        // tag it as "(announces)").
        int dst_idx = -1;
        if (has_addr[1] && !addr[1].is_null()) dst_idx = 1;
        else if (has_addr[2] && !addr[2].is_null() && src_idx != 2 &&
                 !(src_idx >= 0 && addr[2] == addr[src_idx]))
            dst_idx = 2;

        auto print_addr = [&](int i) {
            os << device_class_long_name(addr[i].cls) << ' ';
            addr[i].describe(os);
        };

        if (src_idx < 0)
        {
            os << "(unknown source)";
        }
        else
        {
            print_addr(src_idx);
        }

        if (dst_idx < 0)
        {
            os << " (broadcasts)";
        }
        else if (src_idx >= 0 && addr[dst_idx] == addr[src_idx])
        {
            os << " (announces)";
        }
        else
        {
            os << " -> ";
            print_addr(dst_idx);
        }

        os << " : " << code_long_name(opcode) << " [" << verb_tag(verb) << "]";
    }

} // namespace evohome
