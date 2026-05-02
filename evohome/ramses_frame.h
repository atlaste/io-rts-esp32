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

        /// @brief Print a human-readable header instead of the cryptic evofw3
        ///        format. Examples of what gets produced:
        ///
        ///          OpenTherm Bridge 10:046700 (broadcasts)        : Boiler Status [I]
        ///          RFG100 Gateway 30:112365 -> Controller 01:067362 : Zone Setpoint Override [Q]
        ///          Controller 01:067362 (announces)               : System Sync [I]
        ///
        ///        The address/direction logic is evofw3-aware:
        ///          - addr0 is the source if present, else addr2.
        ///          - addr1 is the destination; if NUL we fall back to
        ///            addr2 only when it differs from addr0 (otherwise we
        ///            print "(broadcasts)" since the message has no
        ///            specific recipient).
        ///          - the verb is appended as a 1-char tag in [...].
        void describe_friendly(std::ostream &os) const;
    };

    /// @brief Why a frame parse failed, in enough detail that the caller can
    ///        tell radio-layer noise (the dominant case in practice) from a
    ///        real codec/structural bug.
    enum class ParseStatus : uint8_t
    {
        Ok = 0,                ///< parsed and checksum valid
        TooShort,              ///< < 8 bytes, can't even hold a minimal frame
        HeaderReservedBits,    ///< header bits 6/7 are RAMSES "reserved-zero"
                               ///  but are set; almost always a bit error
                               ///  on the very first decoded byte
        AddressTruncated,      ///< header demands more address bytes than we have
        ParamTruncated,        ///< param0/param1 byte indicated by header but missing
        OpcodeTruncated,       ///< not enough bytes left for opcode after addrs/params
        LengthTruncated,       ///< not enough bytes left for length byte
        PayloadTooLong,        ///< length > kMaxFramePayload (would overflow our buffer)
        LengthMismatch,        ///< length doesn't match remaining-bytes count
        BadChecksum,           ///< structurally fine, but byte-sum != 0
    };

    /// @brief Human-readable name for a ParseStatus (short, log-friendly).
    const char *to_string(ParseStatus s);

    /// @brief Result of a frame-parse attempt: success/failure status, the
    ///        Frame on success, plus a few diagnostic numbers callers can
    ///        log to help triage failures.
    struct ParseResult
    {
        ParseStatus            status = ParseStatus::TooShort;
        std::optional<Frame>   frame;          ///< populated only if status == Ok
        uint8_t                header_byte = 0;
        uint8_t                length_byte = 0;
        int                    sum_residual = 0;  ///< for BadChecksum: byte-sum mod 256 (1..255)
    };

    /// @brief Parse a fully-decoded protocol-level byte stream into a Frame.
    ///        @p data must start at the header byte and end at the checksum
    ///        byte (which IS verified by this function).
    ///
    ///        Returns a ParseResult with status==Ok plus the populated
    ///        frame on success, or a non-Ok status (and as much partial
    ///        diagnostic info as we managed to extract before bailing out)
    ///        on failure. The returned Frame's payload span is valid for
    ///        the lifetime of the Frame itself.
    ParseResult parse_frame(std::span<const uint8_t> data);

    /// @brief Compute the RAMSES checksum over @p data: returns
    ///        (-(sum of bytes)) mod 256. Adding this byte to @p data makes
    ///        the total sum zero, which is the trivial verification rule.
    uint8_t checksum(std::span<const uint8_t> data);

    /// @brief Serialize a Frame back to wire bytes (header + addrs + params
    ///        + opcode + len + payload + checksum). Returns 0 on overflow.
    size_t serialize_frame(const Frame &f, std::span<uint8_t> out);

} // namespace evohome
