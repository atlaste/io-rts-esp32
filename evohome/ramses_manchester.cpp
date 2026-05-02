#include "ramses_manchester.h"

namespace evohome::manchester
{

    namespace
    {
        // Encoding table: 4-bit (big-endian) data -> 8-bit chip byte. Indexed
        // by the input nibble.
        constexpr uint8_t kEncode[16] = {
            0xAA, 0xA9, 0xA6, 0xA5, 0x9A, 0x99, 0x96, 0x95,
            0x6A, 0x69, 0x66, 0x65, 0x5A, 0x59, 0x56, 0x55,
        };

        // Decoding table: chip-byte's low nibble -> 2 data bits (or 0xFF on
        // invalid). The chip bytes used by RAMSES only ever carry information
        // in their low/high nibbles individually, and only 4 of the 16 nibble
        // values are legal.
        constexpr uint8_t kDecode[16] = {
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x02, 0xFF,
            0xFF, 0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        };
    } // namespace

    uint8_t encode_nibble(uint8_t nibble)
    {
        return kEncode[nibble & 0x0F];
    }

    int decode_nibble(uint8_t chip)
    {
        // A full chip byte encodes 4 data bits as two 2-bit halves: low
        // nibble decodes to bits [1:0], high nibble decodes to bits [3:2].
        const uint8_t lo = kDecode[chip & 0x0F];
        const uint8_t hi = kDecode[(chip >> 4) & 0x0F];
        if (lo == 0xFF || hi == 0xFF) return -1;
        return static_cast<int>((hi << 2) | lo);
    }

    bool is_valid_chip(uint8_t chip)
    {
        return decode_nibble(chip) >= 0;
    }

    size_t encode(std::span<const uint8_t> data, std::span<uint8_t> out)
    {
        const size_t need = data.size() * 2;
        if (out.size() < need) return 0;
        for (size_t i = 0; i < data.size(); ++i)
        {
            const uint8_t b = data[i];
            out[2 * i + 0] = encode_nibble(static_cast<uint8_t>(b >> 4));
            out[2 * i + 1] = encode_nibble(static_cast<uint8_t>(b));
        }
        return need;
    }

    size_t decode(std::span<const uint8_t> chips,
                  std::span<uint8_t> out,
                  size_t *invalid_at)
    {
        // Process chip bytes in pairs - high nibble first, then low nibble.
        // Stop at the first invalid chip byte (caller can inspect invalid_at
        // to learn whether we hit the trailer or a real error).
        const size_t pairs = chips.size() / 2;
        size_t produced = 0;
        for (size_t i = 0; i < pairs && produced < out.size(); ++i)
        {
            const int hi = decode_nibble(chips[2 * i + 0]);
            if (hi < 0)
            {
                if (invalid_at) *invalid_at = 2 * i + 0;
                return produced;
            }
            const int lo = decode_nibble(chips[2 * i + 1]);
            if (lo < 0)
            {
                if (invalid_at) *invalid_at = 2 * i + 1;
                return produced;
            }
            out[produced++] = static_cast<uint8_t>((hi << 4) | lo);
        }
        return produced;
    }

} // namespace evohome::manchester
