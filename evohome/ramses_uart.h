#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace evohome::uart
{

    // ---------------------------------------------------------------
    // RAMSES-II on-air UART framing.
    //
    // evofw3 (the reference RAMSES gateway firmware) clocks every
    // protocol byte through the AVR's hardware UART before feeding the
    // CC1101 in async-serial mode. The CC1101 then FSK-modulates the
    // raw serial bit stream as-is. So on the air every protocol byte
    // shows up as a 10-bit UART frame:
    //
    //     <start=0> <bit0> <bit1> ... <bit7> <stop=1>     (LSB first)
    //
    // The SX1262 has no async-serial hook, so it captures the raw
    // FSK bit stream and packs it into bytes MSB-first (bit 0 of the
    // bit stream becomes the MSB of byte 0). To recover the original
    // protocol bytes we have to slide a UART decoder over those raw
    // chip bytes and validate each start/stop pair.
    //
    // Because the hardware sync detector can latch a few bits early
    // or late, the *correct* starting bit offset isn't known a-priori.
    // Callers should try each bit_offset in [0, 10) and keep the
    // decode that yields the longest valid byte run (mirrors what
    // RadioSX1262::findUartFrame() does for IO-Homecontrol).
    // ---------------------------------------------------------------

    constexpr size_t kUartBitsPerByte = 10; // start + 8 data LSB-first + stop

    // Returns the bit at `bit_pos` within `data`, treating each byte as
    // big-endian (bit 0 -> MSB of byte 0). Inline for hot-path use.
    inline uint8_t get_bit_msb(const uint8_t *data, size_t bit_pos)
    {
        return static_cast<uint8_t>((data[bit_pos / 8] >> (7 - (bit_pos % 8))) & 0x01);
    }

    // Decode raw chip bytes into protocol bytes by treating the input
    // as a continuous MSB-first bit stream of UART frames. Stops at
    // the first invalid start/stop bit (which is also the end-of-burst
    // marker - real frames are followed by the 0x35 trailer that
    // breaks UART framing on purpose).
    //
    // Returns the number of decoded bytes (0..out.size()).
    size_t decode(std::span<const uint8_t> raw,
                  size_t bit_offset,
                  std::span<uint8_t> out);

    // Pick the bit offset in [0, kUartBitsPerByte) that produces the
    // longest valid UART decode for `raw`, decode using that offset,
    // and return the number of bytes produced. Writes the chosen
    // offset to *best_offset (if non-null). Useful when the hardware
    // sync detector's bit alignment is unknown.
    size_t decode_best(std::span<const uint8_t> raw,
                       std::span<uint8_t> out,
                       size_t *best_offset);

} // namespace evohome::uart
