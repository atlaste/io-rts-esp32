#include "ramses_fields.h"

#include <cstdio>
#include <ostream>

namespace evohome::fields
{

    // -------------------------------------------------------------- ZoneIdx

    std::optional<ZoneIdx> ZoneIdx::deserialize(const uint8_t *&cur, const uint8_t *end)
    {
        if (!can_read(cur, end, 1)) return std::nullopt;
        ZoneIdx z{*cur++};
        return z;
    }

    void ZoneIdx::serialize(uint8_t *&cur, uint8_t *end) const
    {
        if (!can_write(cur, end, 1)) return;
        *cur++ = value;
    }

    void ZoneIdx::describe(std::ostream &os) const
    {
        char buf[12];
        if (value >= 0xF8)
            std::snprintf(buf, sizeof(buf), "domain=%02X", value);
        else
            std::snprintf(buf, sizeof(buf), "zone=%02X", value);
        os << buf;
    }

    // ------------------------------------------------------------- DomainId

    std::optional<DomainId> DomainId::deserialize(const uint8_t *&cur, const uint8_t *end)
    {
        if (!can_read(cur, end, 1)) return std::nullopt;
        if (*cur < 0xF8) return std::nullopt;
        DomainId d{*cur++};
        return d;
    }

    void DomainId::serialize(uint8_t *&cur, uint8_t *end) const
    {
        if (!can_write(cur, end, 1)) return;
        *cur++ = value;
    }

    void DomainId::describe(std::ostream &os) const
    {
        char buf[12];
        std::snprintf(buf, sizeof(buf), "domain=%02X", value);
        os << buf;
    }

    // ------------------------------------------------------------ TempCenti

    std::optional<TempCenti> TempCenti::deserialize(const uint8_t *&cur, const uint8_t *end)
    {
        if (!can_read(cur, end, 2)) return std::nullopt;
        TempCenti t;
        t.raw = static_cast<int16_t>((cur[0] << 8) | cur[1]);
        cur += 2;
        return t;
    }

    void TempCenti::serialize(uint8_t *&cur, uint8_t *end) const
    {
        if (!can_write(cur, end, 2)) return;
        const uint16_t u = static_cast<uint16_t>(raw);
        *cur++ = static_cast<uint8_t>((u >> 8) & 0xFF);
        *cur++ = static_cast<uint8_t>(u & 0xFF);
    }

    void TempCenti::describe(std::ostream &os) const
    {
        char buf[16];
        if (!is_valid())
            std::snprintf(buf, sizeof(buf), "T=--");
        else
            std::snprintf(buf, sizeof(buf), "T=%.2fC", static_cast<double>(celsius()));
        os << buf;
    }

    // -------------------------------------------------------------- Percent

    std::optional<Percent> Percent::deserialize(const uint8_t *&cur, const uint8_t *end)
    {
        if (!can_read(cur, end, 1)) return std::nullopt;
        Percent p{*cur++};
        return p;
    }

    void Percent::serialize(uint8_t *&cur, uint8_t *end) const
    {
        if (!can_write(cur, end, 1)) return;
        *cur++ = raw;
    }

    void Percent::describe(std::ostream &os) const
    {
        char buf[12];
        if (!is_valid())
            std::snprintf(buf, sizeof(buf), "%%=--");
        else
            std::snprintf(buf, sizeof(buf), "%%=%.1f", static_cast<double>(percent()));
        os << buf;
    }

    // --------------------------------------------------------------- Co2Ppm

    std::optional<Co2Ppm> Co2Ppm::deserialize(const uint8_t *&cur, const uint8_t *end)
    {
        if (!can_read(cur, end, 2)) return std::nullopt;
        Co2Ppm c;
        c.raw = static_cast<uint16_t>((cur[0] << 8) | cur[1]);
        cur += 2;
        return c;
    }

    void Co2Ppm::serialize(uint8_t *&cur, uint8_t *end) const
    {
        if (!can_write(cur, end, 2)) return;
        *cur++ = static_cast<uint8_t>((raw >> 8) & 0xFF);
        *cur++ = static_cast<uint8_t>(raw & 0xFF);
    }

    void Co2Ppm::describe(std::ostream &os) const
    {
        char buf[16];
        if (!is_valid())
            std::snprintf(buf, sizeof(buf), "co2=--");
        else
            std::snprintf(buf, sizeof(buf), "co2=%uppm", static_cast<unsigned>(raw));
        os << buf;
    }

    // ------------------------------------------------------------- Humidity

    std::optional<Humidity> Humidity::deserialize(const uint8_t *&cur, const uint8_t *end)
    {
        if (!can_read(cur, end, 1)) return std::nullopt;
        Humidity h{*cur++};
        return h;
    }

    void Humidity::serialize(uint8_t *&cur, uint8_t *end) const
    {
        if (!can_write(cur, end, 1)) return;
        *cur++ = raw;
    }

    void Humidity::describe(std::ostream &os) const
    {
        char buf[12];
        if (!is_valid())
            std::snprintf(buf, sizeof(buf), "rh=--");
        else
            std::snprintf(buf, sizeof(buf), "rh=%u%%", static_cast<unsigned>(raw));
        os << buf;
    }

    // -------------------------------------------------------------- Battery

    std::optional<Battery> Battery::deserialize(const uint8_t *&cur, const uint8_t *end)
    {
        if (!can_read(cur, end, 2)) return std::nullopt;
        Battery b;
        const uint8_t *p = cur;
        auto pct = Percent::deserialize(p, end);
        if (!pct) return std::nullopt;
        if (!can_read(p, end, 1)) return std::nullopt;
        b.percent = *pct;
        b.low_flag = *p++;
        cur = p;
        return b;
    }

    void Battery::serialize(uint8_t *&cur, uint8_t *end) const
    {
        if (!can_write(cur, end, 2)) return;
        percent.serialize(cur, end);
        *cur++ = low_flag;
    }

    void Battery::describe(std::ostream &os) const
    {
        percent.describe(os);
        os << (low_flag == 0x00 ? " LOW" : " OK");
    }

    // ------------------------------------------------------------ OpcodeBE16

    std::optional<OpcodeBE16> OpcodeBE16::deserialize(const uint8_t *&cur, const uint8_t *end)
    {
        if (!can_read(cur, end, 2)) return std::nullopt;
        OpcodeBE16 o;
        o.value = static_cast<uint16_t>((cur[0] << 8) | cur[1]);
        cur += 2;
        return o;
    }

    void OpcodeBE16::serialize(uint8_t *&cur, uint8_t *end) const
    {
        if (!can_write(cur, end, 2)) return;
        *cur++ = static_cast<uint8_t>((value >> 8) & 0xFF);
        *cur++ = static_cast<uint8_t>(value & 0xFF);
    }

    void OpcodeBE16::describe(std::ostream &os) const
    {
        char buf[8];
        std::snprintf(buf, sizeof(buf), "%04X", static_cast<unsigned>(value));
        os << "code=" << buf;
    }

    // ------------------------------------------------------------ DeviceId3

    std::optional<DeviceId3> DeviceId3::deserialize(const uint8_t *&cur, const uint8_t *end)
    {
        if (!can_read(cur, end, 3)) return std::nullopt;
        DeviceId3 d;
        d.bytes[0] = cur[0];
        d.bytes[1] = cur[1];
        d.bytes[2] = cur[2];
        cur += 3;
        return d;
    }

    void DeviceId3::serialize(uint8_t *&cur, uint8_t *end) const
    {
        if (!can_write(cur, end, 3)) return;
        *cur++ = bytes[0];
        *cur++ = bytes[1];
        *cur++ = bytes[2];
    }

    void DeviceId3::describe(std::ostream &os) const
    {
        // Decode the 3-byte address using the same bit layout as the header
        // address slots (class<<2 | id_top, then 16 bits of id).
        const unsigned cls = (bytes[0] & 0xFC) >> 2;
        const unsigned id  = (static_cast<unsigned>(bytes[0] & 0x03) << 16) |
                             (static_cast<unsigned>(bytes[1])        << 8 ) |
                              static_cast<unsigned>(bytes[2]);
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%02u:%06u", cls, id);
        os << buf;
    }

    // ----------------------------------------------------------- FixedBytes

    template<size_t N>
    std::optional<FixedBytes<N>> FixedBytes<N>::deserialize(const uint8_t *&cur, const uint8_t *end)
    {
        if (!can_read(cur, end, N)) return std::nullopt;
        FixedBytes<N> f;
        for (size_t i = 0; i < N; ++i) f.bytes[i] = cur[i];
        cur += N;
        return f;
    }

    template<size_t N>
    void FixedBytes<N>::serialize(uint8_t *&cur, uint8_t *end) const
    {
        if (!can_write(cur, end, N)) return;
        for (size_t i = 0; i < N; ++i) *cur++ = bytes[i];
    }

    template<size_t N>
    void FixedBytes<N>::describe(std::ostream &os) const
    {
        os << "raw=";
        for (size_t i = 0; i < N; ++i)
        {
            char buf[4];
            std::snprintf(buf, sizeof(buf), "%02X", static_cast<unsigned>(bytes[i]));
            os << buf;
        }
    }

    template struct FixedBytes<1>;
    template struct FixedBytes<2>;
    template struct FixedBytes<3>;
    template struct FixedBytes<4>;
    template struct FixedBytes<5>;
    template struct FixedBytes<6>;
    template struct FixedBytes<7>;
    template struct FixedBytes<8>;

    // -------------------------------------------------------------- HexBlob

    std::optional<HexBlob> HexBlob::deserialize(const uint8_t *&cur, const uint8_t *end)
    {
        if (cur > end) return std::nullopt;
        const size_t n = static_cast<size_t>(end - cur);
        HexBlob b;
        b.length = static_cast<uint8_t>(n > b.bytes.size() ? b.bytes.size() : n);
        for (size_t i = 0; i < b.length; ++i) b.bytes[i] = cur[i];
        cur = end; // greedy: consume all remaining payload
        return b;
    }

    void HexBlob::serialize(uint8_t *&cur, uint8_t *end) const
    {
        for (uint8_t i = 0; i < length; ++i)
        {
            if (!can_write(cur, end, 1)) return;
            *cur++ = bytes[i];
        }
    }

    void HexBlob::describe(std::ostream &os) const
    {
        if (length == 0)
        {
            os << "(empty)";
            return;
        }
        for (uint8_t i = 0; i < length; ++i)
        {
            char buf[4];
            std::snprintf(buf, sizeof(buf), "%02X", static_cast<unsigned>(bytes[i]));
            os << buf;
        }
    }

} // namespace evohome::fields
