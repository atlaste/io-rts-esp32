#include "ramses_uart.h"

namespace evohome::uart
{

    size_t decode(std::span<const uint8_t> raw,
                  size_t bit_offset,
                  std::span<uint8_t> out)
    {
        if (raw.empty() || out.empty()) return 0;
        const size_t total_bits = raw.size() * 8;
        size_t bit_pos = bit_offset;
        size_t produced = 0;

        while (bit_pos + kUartBitsPerByte <= total_bits && produced < out.size())
        {
            // start bit must be 0, stop bit must be 1; anything else
            // means we've stepped off the UART framing - bail out and
            // let the caller decide whether to keep what we have so
            // far or try a different alignment.
            if (get_bit_msb(raw.data(), bit_pos) != 0 ||
                get_bit_msb(raw.data(), bit_pos + 9) != 1)
                break;

            uint8_t value = 0;
            for (uint8_t i = 0; i < 8; ++i)
            {
                if (get_bit_msb(raw.data(), bit_pos + 1 + i))
                    value = static_cast<uint8_t>(value | (1U << i));
            }
            out[produced++] = value;
            bit_pos += kUartBitsPerByte;
        }
        return produced;
    }

    namespace
    {
        // Length-only variant: counts how many UART frames decode cleanly
        // starting at `bit_offset` without writing anything. Used by
        // decode_best() to score each alignment without needing a
        // scratch buffer.
        size_t decode_length(std::span<const uint8_t> raw,
                             size_t bit_offset,
                             size_t out_max)
        {
            const size_t total_bits = raw.size() * 8;
            size_t bit_pos  = bit_offset;
            size_t produced = 0;
            while (bit_pos + kUartBitsPerByte <= total_bits && produced < out_max)
            {
                if (get_bit_msb(raw.data(), bit_pos) != 0 ||
                    get_bit_msb(raw.data(), bit_pos + 9) != 1)
                    break;
                ++produced;
                bit_pos += kUartBitsPerByte;
            }
            return produced;
        }
    }

    size_t decode_best(std::span<const uint8_t> raw,
                       std::span<uint8_t> out,
                       size_t *best_offset)
    {
        // Two-pass design to avoid a per-attempt scratch buffer (this
        // function is called from the radio GPIO task whose stack is
        // tight). First pass: score each of the 10 alignments by length
        // only. Second pass: actually decode the winner into `out`.
        size_t best_len  = 0;
        size_t best_offs = 0;
        for (size_t off = 0; off < kUartBitsPerByte; ++off)
        {
            const size_t n = decode_length(raw, off, out.size());
            if (n > best_len)
            {
                best_len  = n;
                best_offs = off;
            }
        }
        if (best_len > 0) decode(raw, best_offs, out);
        if (best_offset != nullptr) *best_offset = best_offs;
        return best_len;
    }

} // namespace evohome::uart
