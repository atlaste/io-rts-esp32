#pragma once

/**
 * @file ramses_frame.h
 * @brief RAMSES-II wire-level frame parser & serializer.
 *
 * Wire layout (after software Manchester decode, NOT including preamble /
 * sync / trailer / training):
 *
 *   header_byte               (1 byte)
 *     bits[5:4] = verb        (00=RQ, 01=I, 10=W, 11=RP)
 *     bits[3:2] = addr_combo  (00 = ALL three addrs, 01 = only addr2,
 *                              10 = addr0+addr2,    11 = addr0+addr1)
 *     bit [1]   = param0 present
 *     bit [0]   = param1 present
 *
 *   addr0 [if present]        (3 bytes)
 *   addr1 [if present]        (3 bytes)
 *   addr2 [if present]        (3 bytes)
 *   param0 [if present]       (1 byte)
 *   param1 [if present]       (1 byte)
 *
 *   opcode                    (2 bytes, big-endian)
 *   payload_length            (1 byte)
 *   payload                   (payload_length bytes)
 *   checksum                  (1 byte: -sum(prev) mod 256, so total sum = 0)
 *
 * Address bytes always come in groups of 3; absent address slots are filled
 * in by the parser as DeviceAddr{63, 0x3FFFF} (NUL marker, ramses style).
 *
 * Reference: evofw3 message.c (this is a near-line-for-line C++ port of the
 * RX state machine, matching evofw3's bit layout exactly so on-air
 * compatibility is byte-perfect).
 */

#include "ramses_types.h"

#include <array>
#include <cstdint>
#include <iosfwd>
#include <optional>
#include <span>

namespace evohome
{

    /// @brief Maximum protocol payload size we will ever consider valid. The
    ///        RAMSES wire format technically allows up to 255 (1-byte length
    ///        field) but legitimate evohome traffic stays well under 64.
    constexpr size_t kMaxFramePayload = 64;

    /// @brief A fully-decoded RAMSES-II frame ready for codec dispatch.
    struct Frame
    {
        uint8_t    header_byte = 0;        ///< raw header byte (for debug)
        Verb       verb = Verb::I;
        DeviceAddr addr[3]{};              ///< addr[0..2]; absent slots are NUL
        bool       has_addr[3]{false, false, false};
        std::optional<uint8_t> param0;
        std::optional<uint8_t> param1;
        uint16_t   opcode = 0;
        uint8_t    payload_len = 0;
        std::array<uint8_t, kMaxFramePayload> payload{};

        /// @brief Convenience: span over the meaningful payload bytes.
        std::span<const uint8_t> payload_view() const
        {
            return {payload.data(), payload_len};
        }

        /// @brief Print canonical evofw3 representation, e.g.
        ///         " I --- 01:145038 --:------ 01:145038 30C9 003 0008D9"
        ///        Note: we do NOT print the seqn (RAMSES does not put one on
        ///        the wire; evofw3 fakes "---" when not parsing locally).
        void describe_header(std::ostream &os) const;
    };

    /// @brief Parse a fully-decoded protocol-level byte stream into a Frame.
    ///        @p data must start at the header byte and end at (but not
    ///        include) the trailer byte; the checksum byte is the LAST byte
    ///        of @p data and is verified by this function.
    ///
    /// @return std::nullopt on any structural / checksum error. On success,
    ///         the returned Frame's payload span is valid for the lifetime
    ///         of the Frame itself.
    std::optional<Frame> parse_frame(std::span<const uint8_t> data);

    /// @brief Compute the RAMSES checksum over @p data: returns
    ///        (-(sum of bytes)) mod 256. Adding this byte to @p data makes
    ///        the total sum zero, which is the trivial verification rule.
    uint8_t checksum(std::span<const uint8_t> data);

    /// @brief Serialize a Frame back to wire bytes (header + addrs + params
    ///        + opcode + len + payload + checksum). Returns 0 on overflow.
    size_t serialize_frame(const Frame &f, std::span<uint8_t> out);

} // namespace evohome
