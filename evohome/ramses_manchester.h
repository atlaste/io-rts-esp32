#pragma once

/**
 * @file ramses_manchester.h
 * @brief Software Manchester encode/decode for RAMSES-II.
 *
 * RAMSES-II encodes every protocol byte as TWO chip bytes: each 4-bit nibble
 * (big-endian: high nibble first) is mapped to one chip byte using the table
 * below. Only 16 of the 256 possible chip bytes are valid; everything else
 * indicates a bit error.
 *
 * Encoding table (high-nibble big-endian -> little-endian chip byte):
 *
 *   0x0 -> 0xAA   0x4 -> 0x9A   0x8 -> 0x6A   0xC -> 0x5A
 *   0x1 -> 0xA9   0x5 -> 0x99   0x9 -> 0x69   0xD -> 0x59
 *   0x2 -> 0xA6   0x6 -> 0x96   0xA -> 0x66   0xE -> 0x56
 *   0x3 -> 0xA5   0x7 -> 0x95   0xB -> 0x65   0xF -> 0x55
 *
 * Reference: evofw3/frame.c (man_encode / man_decode tables).
 */

#include <cstddef>
#include <cstdint>
#include <span>

namespace evohome::manchester
{

    /// @brief Encode a single 4-bit nibble (LSB-aligned) into one Manchester
    ///        chip byte. The input is masked with 0x0F.
    uint8_t encode_nibble(uint8_t nibble);

    /// @brief Decode a single Manchester chip byte to a 4-bit nibble. Returns
    ///        nullopt on an invalid Manchester code (bit error / collision).
    ///        The decoded nibble is in the low 4 bits of the return value.
    int decode_nibble(uint8_t chip);

    /// @brief Test whether @p chip encodes a valid Manchester pair (both
    ///        nibbles of the byte map to legal codes).
    bool is_valid_chip(uint8_t chip);

    /// @brief Encode @p data into Manchester chip bytes (2 chip bytes per
    ///        input byte). Returns the number of chip bytes written, or 0 on
    ///        overflow.
    size_t encode(std::span<const uint8_t> data, std::span<uint8_t> out);

    /// @brief Decode a stream of Manchester chip bytes into protocol bytes.
    ///        Stops at the first invalid chip byte. Returns the number of
    ///        protocol bytes successfully decoded; if @p invalid_at is not
    ///        null and decoding stopped early, it is set to the chip-byte
    ///        index that failed (otherwise left untouched).
    ///
    /// The caller is responsible for stripping the trailer byte (0x35) before
    /// invoking decode() - it is intentionally treated as an invalid chip
    /// byte by the table and will terminate decoding cleanly.
    size_t decode(std::span<const uint8_t> chips,
                  std::span<uint8_t> out,
                  size_t *invalid_at = nullptr);

    // ---------------------------------------------------------------- Constants

    /// @brief The trailer byte that marks "end of message" in RAMSES, just
    ///        after the Manchester-encoded payload. Not a valid chip byte.
    constexpr uint8_t kTrailerByte = 0x35;

    /// @brief The "training" / fill byte sent both before the preamble and
    ///        after the trailer. 0x55 is preamble-shaped (alternating bits)
    ///        so it is also the value of all chip-stream "noise" between
    ///        bursts.
    constexpr uint8_t kTrainingByte = 0x55;

} // namespace evohome::manchester
