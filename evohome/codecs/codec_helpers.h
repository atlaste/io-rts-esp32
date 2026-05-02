#pragma once

/**
 * @file codec_helpers.h
 * @brief Tiny, header-only formatting helpers shared by every codec
 *        translation unit (codecs_heat.cpp, codecs_hvac.cpp, ...).
 *
 * These exist purely so that each codec's describe() can produce
 * sentence-style output without dragging snprintf boilerplate into every
 * single struct. They are intentionally inline and namespaced in
 * `evohome::codecs::detail` to keep the codec files free of name
 * pollution.
 */

#include "../ramses_fields.h"

#include <cstdint>
#include <cstdio>
#include <ostream>

namespace evohome::codecs
{

    // -------------------------------------------------------------- zone/domain
    // 0x00..0xEF -> "zone N" (zero-indexed, displayed as N+1 in apps - we
    // stick to the wire value to keep cross-referencing easy).
    // 0xF8..0xFF -> "domain ID" with the well-known domain alias if any.

    inline void print_zone_or_domain(std::ostream &os, uint8_t v)
    {
        char buf[16];
        if (v >= 0xF8)
        {
            const char *name =
                v == 0xF8 ? "DHW heat-demand"
              : v == 0xF9 ? "DHW heat-demand 2"
              : v == 0xFA ? "DHW"
              : v == 0xFC ? "heating"
              : v == 0xFF ? "all"
              : nullptr;
            if (name) os << "domain " << name;
            else
            {
                std::snprintf(buf, sizeof(buf), "domain %02X", v);
                os << buf;
            }
        }
        else
        {
            std::snprintf(buf, sizeof(buf), "zone %u", static_cast<unsigned>(v));
            os << buf;
        }
    }

    // -------------------------------------------------------------- temperature
    // Centigrade with 0.01 resolution. "is_valid()" handles the various
    // sentinel values (0x7FFF, 0x8000, 0x7EFF) used by different vendors.

    inline void print_temp(std::ostream &os, const evohome::fields::TempCenti &t)
    {
        if (!t.is_valid()) { os << "--"; return; }
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%.2f\xC2\xB0""C", static_cast<double>(t.celsius()));
        os << buf;
    }

    // Same but for raw 2-byte signed BE int16 in centidegrees.
    inline void print_temp_raw(std::ostream &os, uint8_t hi, uint8_t lo)
    {
        const int16_t v = static_cast<int16_t>((hi << 8) | lo);
        if (hi == 0x7F && lo == 0xFF) { os << "--"; return; }
        if (v == static_cast<int16_t>(0x8000)) { os << "--"; return; }
        if (hi == 0xEF && lo == 0xEF) { os << "--"; return; }
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%.2f\xC2\xB0""C", v * 0.01);
        os << buf;
    }

    // -------------------------------------------------------------- percent
    // Percent in 0..200 raw -> 0..100% (0.5% resolution). 0xFF == "no value".

    inline void print_pct(std::ostream &os, const evohome::fields::Percent &p)
    {
        if (!p.is_valid()) { os << "--"; return; }
        char buf[12];
        std::snprintf(buf, sizeof(buf), "%.0f%%", static_cast<double>(p.percent()));
        os << buf;
    }

    // Raw percent byte (some codes use 0..100 directly, not the 0..200 scale).
    inline void print_pct100(std::ostream &os, uint8_t raw)
    {
        if (raw == 0xFF) { os << "--"; return; }
        char buf[12];
        std::snprintf(buf, sizeof(buf), "%u%%", static_cast<unsigned>(raw));
        os << buf;
    }

    // -------------------------------------------------------------- hex helpers

    inline void print_hex_byte(std::ostream &os, uint8_t b)
    {
        char buf[4];
        std::snprintf(buf, sizeof(buf), "%02X", static_cast<unsigned>(b));
        os << buf;
    }

    inline void print_hex_be16(std::ostream &os, uint8_t hi, uint8_t lo)
    {
        char buf[8];
        std::snprintf(buf, sizeof(buf), "%04X",
                      static_cast<unsigned>((hi << 8) | lo));
        os << buf;
    }

    inline void print_hex_buffer(std::ostream &os, const uint8_t *data, size_t n)
    {
        for (size_t i = 0; i < n; ++i)
        {
            char buf[4];
            std::snprintf(buf, sizeof(buf), "%02X", static_cast<unsigned>(data[i]));
            os << buf;
        }
    }

    // -------------------------------------------------------------- bool / on-off

    inline void print_on_off(std::ostream &os, uint8_t v)
    {
        os << (v == 0xC8 ? "ON"
             : v == 0x00 ? "OFF"
             : v == 0xFF ? "--"
                         : "?");
    }
} // namespace evohome::codecs
