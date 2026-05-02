/**
 * @file codecs_hvac.cpp
 * @brief Codecs for the HVAC half of RAMSES-II: Itho Daalderop, Orcon,
 *        Vasco, Nuaire, ClimaRad, Ventiline. CO2 / humidity / PIR
 *        sensors, fan units, remotes.
 *
 * Schemas come from `ramses_rf/src/ramses_tx/parsers.py`. HVAC payloads
 * vary a lot per manufacturer; where a field has multiple meanings we
 * pick the most common interpretation and surface the raw bytes when in
 * doubt (the [+NB unparsed] suffix from CodecAdapter helps spot drift).
 */

#include "../ramses_codec.h"
#include "../ramses_fields.h"
#include "../ramses_payload.h"
#include "codec_helpers.h"

#include <cstdio>
#include <ostream>

namespace evohome::codecs
{
    using namespace evohome;
    using namespace evohome::fields;

    // ============================================================ 31D9 FanState
    //
    // 3-byte mandatory header + optional 14-byte vendor extension.
    //   [0]    domain/zone idx (always 00)
    //   [1]    flags bitmap   (bit1=passive, bit2=damper-only, bit5=filter-dirty,
    //                          bit6=frost-cycle, bit7=has-fault)
    //   [2]    fan_speed (0..200 -> 0..100% for Itho/ClimaRad)
    //          *or* fan_mode (Orcon, Vasco)

    struct FanState_Inform
        : Payload<std::tuple<ZoneIdx, FixedBytes<2>, Optional<HexBlob>>>
    {
        void describe(std::ostream &os) const
        {
            const uint8_t flags = std::get<1>(fields).bytes[0];
            const uint8_t spd   = std::get<1>(fields).bytes[1];
            const char *mname =
                spd == 0x00 ? "off"
              : spd == 0x04 ? "low"
              : spd == 0x05 ? "medium"
              : spd == 0x06 ? "high"
              : spd == 0x07 ? "boost"
              : nullptr;
            print_zone_or_domain(os, std::get<0>(fields).value);
            os << " : fan ";
            if (mname) os << mname;
            else { char buf[12]; std::snprintf(buf, sizeof(buf), "mode=%02X", spd); os << buf; }
            char buf[16];
            std::snprintf(buf, sizeof(buf), " (%u%%)", static_cast<unsigned>(spd));
            os << buf;
            if (flags & 0x20) os << " FILTER";
            if (flags & 0x40) os << " FROST";
            if (flags & 0x80) os << " FAULT";
        }
    };

    // ============================================================ 31DA HvacState
    //
    // Long payload (typ. 30 bytes). Layout per ramses_rf parser_31da:
    //   [0]      hvac_id (zone)
    //   [1..2]   air_quality (12C8-style)         - manufacturer specific
    //   [3..4]   co2_level (1298-style, ppm BE)
    //   [5]      indoor humidity (12A0-style, %)
    //   [6]      outdoor humidity (%)
    //   [7..8]   exhaust temp (centi BE)
    //   [9..10]  supply temp  (centi BE)
    //   [11..12] indoor temp  (centi BE)
    //   [13..14] outdoor temp (centi BE)
    //   [15..16] capabilities (bitmap)
    //   [17]     bypass_position (% 0..200)
    //   [18]     fan_info / 22F3-ish
    //   [19]     exhaust_fan_speed
    //   [20]     supply_fan_speed
    //   [21..22] remaining_mins
    //   [23]     post_heater %
    //   [24]     pre_heater %
    //   [25..26] supply_flow (L/s, BE)
    //   [27..28] exhaust_flow
    //   [29]     trailing 00 (sometimes)
    //
    // We surface temps + co2 + humidity + speeds. Anything without a
    // sentinel value gets "--". The full hex follows via the unparsed
    // tail when the schema doesn't fully consume.

    struct HvacState_Inform
        : Payload<std::tuple<ZoneIdx, FixedBytes<2>, FixedBytes<2>, FixedBytes<1>,
                              FixedBytes<1>, FixedBytes<2>, FixedBytes<2>, FixedBytes<2>,
                              FixedBytes<2>, FixedBytes<2>, FixedBytes<1>, FixedBytes<1>,
                              FixedBytes<1>, FixedBytes<1>, FixedBytes<2>, FixedBytes<1>,
                              FixedBytes<1>, FixedBytes<2>, FixedBytes<2>, Optional<HexBlob>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);

            const auto &co2 = std::get<2>(fields).bytes;
            const uint16_t ppm = (co2[0] << 8) | co2[1];
            if (!(co2[0] == 0x7F && co2[1] == 0xFF)) {
                char buf[24];
                std::snprintf(buf, sizeof(buf), ", co2=%u ppm", static_cast<unsigned>(ppm));
                os << buf;
            }

            const uint8_t rh = std::get<3>(fields).bytes[0];
            if (rh != 0xFF) {
                char buf[16];
                std::snprintf(buf, sizeof(buf), ", rh=%u%%", static_cast<unsigned>(rh));
                os << buf;
            }

            const auto &exh = std::get<5>(fields).bytes;
            const auto &sup = std::get<6>(fields).bytes;
            const auto &ind = std::get<7>(fields).bytes;
            const auto &out = std::get<8>(fields).bytes;
            os << ", exhaust="; print_temp_raw(os, exh[0], exh[1]);
            os << ", supply=";  print_temp_raw(os, sup[0], sup[1]);
            os << ", indoor=";  print_temp_raw(os, ind[0], ind[1]);
            if (!(out[0] == 0x7F && out[1] == 0xFF) &&
                !(out[0] == 0xEF && out[1] == 0xEF)) {
                os << ", outdoor="; print_temp_raw(os, out[0], out[1]);
            }

            const uint8_t exhSpd = std::get<11>(fields).bytes[0];
            const uint8_t supSpd = std::get<12>(fields).bytes[0];
            if (exhSpd != 0xFF) {
                char buf[24];
                std::snprintf(buf, sizeof(buf), ", exhFan=%u%%",
                    static_cast<unsigned>(exhSpd / 2));
                os << buf;
            }
            if (supSpd != 0xFF) {
                char buf[24];
                std::snprintf(buf, sizeof(buf), ", supFan=%u%%",
                    static_cast<unsigned>(supSpd / 2));
                os << buf;
            }
        }
    };

    // ============================================================ 1298 Co2Level
    //   I/RP : zone(1) + co2_ppm(2B BE)

    struct Co2Level_Inform
        : Payload<std::tuple<ZoneIdx, Co2Ppm>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            const auto &c = std::get<1>(fields);
            char buf[16];
            std::snprintf(buf, sizeof(buf), " : %u ppm", static_cast<unsigned>(c.raw));
            if (c.is_valid()) os << buf;
            else              os << " : --";
        }
    };
    struct Co2Level_Reply : Co2Level_Inform {};

    // ============================================================ 12A0 IndoorHumidity
    //   I : zone(1) + humidity(1) + temp(2B centi) + optional dew(2B centi)

    struct IndoorHumidity_Inform
        : Payload<std::tuple<ZoneIdx, Humidity, TempCenti, Optional<TempCenti>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            const auto &h = std::get<1>(fields);
            const auto &t = std::get<2>(fields);
            const auto &dew = std::get<3>(fields);
            char buf[16];
            os << " : ";
            if (h.is_valid()) { std::snprintf(buf, sizeof(buf), "%u%% RH", static_cast<unsigned>(h.raw)); os << buf; }
            else              { os << "RH=--"; }
            os << ", ";
            print_temp(os, t);
            if (dew.value)
            {
                os << " (dew ";
                print_temp(os, *dew.value);
                os << ")";
            }
        }
    };

    // ============================================================ 1280 OutdoorHumidity
    //   I/RP : zone(1) + humidity(1)

    struct OutdoorHumidity_Reply
        : Payload<std::tuple<ZoneIdx, Humidity>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            const auto &h = std::get<1>(fields);
            os << " : ";
            if (h.is_valid()) {
                char buf[16];
                std::snprintf(buf, sizeof(buf), "%u%% RH", static_cast<unsigned>(h.raw));
                os << buf;
            } else { os << "--"; }
        }
    };
    struct OutdoorHumidity_Inform : OutdoorHumidity_Reply {};

    // ============================================================ 12C0 DisplayedTemp
    //
    //   I : zone(1) + temp_byte(1) + units_byte(1: 00=Fahr 1F-step, 01=Cels 0.5-step)
    // Used by TR87RF round thermostats bound to RFG100.

    struct DisplayedTemp_Inform
        : Payload<std::tuple<ZoneIdx, FixedBytes<1>, FixedBytes<1>, Optional<HexBlob>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            const uint8_t raw   = std::get<1>(fields).bytes[0];
            const uint8_t units = std::get<2>(fields).bytes[0];
            os << " = ";
            if (raw == 0x80) { os << "--"; return; }
            char buf[16];
            if (units == 0x01)      std::snprintf(buf, sizeof(buf), "%.1f\xC2\xB0""C", raw * 0.5);
            else                    std::snprintf(buf, sizeof(buf), "%u\xC2\xB0""F", static_cast<unsigned>(raw));
            os << buf;
        }
    };

    // ============================================================ 12C8 AirQuality
    //   I/RP : zone(1) + 2B air-quality bitmap

    struct AirQuality_Inform
        : Payload<std::tuple<ZoneIdx, FixedBytes<2>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            const auto &b = std::get<1>(fields).bytes;
            os << " : raw="; print_hex_be16(os, b[0], b[1]);
        }
    };
    struct AirQuality_Reply : AirQuality_Inform {};

    // ============================================================ 22F1 FanMode
    //
    //   I/W : 00 + mode_idx(1) + mode_max(1)
    // mode_max = 04(itho) / 0A(nuaire) / 06(vasco) / 07(orcon)

    struct FanMode_Inform
        : Payload<std::tuple<FixedBytes<1>, FixedBytes<1>, Optional<FixedBytes<1>>>>
    {
        void describe(std::ostream &os) const
        {
            const uint8_t idx = std::get<1>(fields).bytes[0];
            const uint8_t mx  = std::get<2>(fields).value
                              ? std::get<2>(fields).value->bytes[0] : 0;
            // Common Orcon mode mapping (most installs):
            const char *name = nullptr;
            if (mx == 0x07 || mx == 0x00 || mx == 0x04) {
                name =
                    idx == 0x00 ? "AWAY"
                  : idx == 0x01 ? "auto"
                  : idx == 0x02 ? "low"
                  : idx == 0x03 ? "medium"
                  : idx == 0x04 ? "high"
                  : idx == 0x05 ? "auto-night"
                  : idx == 0x06 ? "timer"
                  : idx == 0x07 ? "boost"
                  : nullptr;
            }
            os << "fan ";
            if (name) os << name;
            else { char buf[12]; std::snprintf(buf, sizeof(buf), "mode=%02X", idx); os << buf; }
            if (mx) {
                char buf[24];
                std::snprintf(buf, sizeof(buf), " (set max=%02X)", static_cast<unsigned>(mx));
                os << buf;
            }
        }
    };
    struct FanMode_Write : FanMode_Inform {};

    // ============================================================ 22F3 FanBoost
    //
    //   I/W : 00 + flags(1) + duration(1) + 4B options
    // Flags: bits 0-2 = new_speed_mode, bits 3-5 = fallback_speed_mode,
    //        bits 6-7 = units (00=mins, 40=hrs, 80=index).

    struct FanBoost_Inform
        : Payload<std::tuple<FixedBytes<1>, FixedBytes<1>, FixedBytes<1>, Optional<HexBlob>>>
    {
        void describe(std::ostream &os) const
        {
            const uint8_t flags = std::get<1>(fields).bytes[0];
            const uint8_t dur   = std::get<2>(fields).bytes[0];
            const char *units =
                (flags & 0xC0) == 0x00 ? "min"
              : (flags & 0xC0) == 0x40 ? "hr"
              : (flags & 0xC0) == 0x80 ? "idx"
              : "?";
            char buf[40];
            std::snprintf(buf, sizeof(buf), "boost %u%s (flags=0x%02X)",
                static_cast<unsigned>(dur), units,
                static_cast<unsigned>(flags));
            os << buf;
        }
    };
    struct FanBoost_Write : FanBoost_Inform {};

    // ============================================================ 22F7 BypassMode
    //
    //   I/W : 00 + mode_byte(1: 00=off, C8=on, FF=auto) + state_byte(1: 00/C8) + optional
    //
    // Bypass = the HRU damper that lets outside air bypass the heat
    // exchanger (useful at night in summer to dump heat).

    struct BypassMode_Inform
        : Payload<std::tuple<ZoneIdx, FixedBytes<1>, Optional<FixedBytes<1>>, Optional<HexBlob>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            const uint8_t mode = std::get<1>(fields).bytes[0];
            os << " : bypass " << (mode == 0x00 ? "off"
                                 : mode == 0xC8 ? "on"
                                 : mode == 0xFF ? "auto"
                                                : "?");
            if (std::get<2>(fields).value) {
                const uint8_t st = std::get<2>(fields).value->bytes[0];
                if (st != 0xEF)
                    os << ", state=" << (st == 0x00 ? "off"
                                       : st == 0xC8 ? "on"
                                                    : "?");
            }
        }
    };
    struct BypassMode_Write : BypassMode_Inform {};

    // ============================================================ 31E0 FanDemand
    //
    //   I : zone(1) + 1B flags + 1B percent(0..200) + optional 1B (00 or FF)
    //
    // Sent by motion sensors / CO2 sensors to *request* ventilation.

    struct FanDemand_Inform
        : Payload<std::tuple<ZoneIdx, FixedBytes<1>, Percent, Optional<FixedBytes<1>>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            os << " : demand=";
            print_pct(os, std::get<2>(fields));
            const uint8_t flags = std::get<1>(fields).bytes[0];
            if (flags) { char buf[16]; std::snprintf(buf, sizeof(buf), " (flags=0x%02X)", static_cast<unsigned>(flags)); os << buf; }
        }
    };

    // ============================================================ 3120 HvacStatus
    //
    //   I/RP : 7-byte fixed header. Mostly opaque vendor status.

    struct HvacStatus_Inform
        : Payload<std::tuple<ZoneIdx, FixedBytes<6>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            os << " : raw=";
            const auto &b = std::get<1>(fields).bytes;
            print_hex_buffer(os, b.data(), b.size());
        }
    };
    struct HvacStatus_Reply : HvacStatus_Inform {};

    // ============================================================ 2E10 PresenceDetected
    //
    // Sent by HVAC PIR / motion sensors and ClimaRad / Itho fans with
    // built-in presence detection.
    //   I 2E10 002 0001         -> presence=true   (ClimaRad Spider variant)
    //   I 2E10 003 000000       -> presence=false  (ClimaRad Ventura)
    //   I 2E10 003 000100       -> presence=true   (ClimaRad Ventura)

    struct PresenceDetected_Inform
        : Payload<std::tuple<ZoneIdx, FixedBytes<1>, Optional<FixedBytes<1>>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            const uint8_t flag = std::get<1>(fields).bytes[0];
            os << " : ";
            if      (flag == 0x00) os << "no presence";
            else if (flag == 0x01) os << "presence detected";
            else { char buf[20]; std::snprintf(buf, sizeof(buf), "presence=0x%02X", static_cast<unsigned>(flag)); os << buf; }
        }
    };

    // ----------------------------------------------------------------
    // Registration
    // ----------------------------------------------------------------

    void register_hvac_codecs(CodecRegistry &reg)
    {
        REGISTER_CODEC(reg, Code::FanState,  Verb::I, FanState_Inform);
        REGISTER_CODEC(reg, Code::HvacState, Verb::I, HvacState_Inform);

        REGISTER_CODEC(reg, Code::Co2Level, Verb::I,  Co2Level_Inform);
        REGISTER_CODEC(reg, Code::Co2Level, Verb::RP, Co2Level_Reply);

        REGISTER_CODEC(reg, Code::IndoorHumidity, Verb::I, IndoorHumidity_Inform);

        REGISTER_CODEC(reg, 0x1280, Verb::I,  OutdoorHumidity_Inform);
        REGISTER_CODEC(reg, 0x1280, Verb::RP, OutdoorHumidity_Reply);

        REGISTER_CODEC(reg, 0x12C0, Verb::I,  DisplayedTemp_Inform);

        REGISTER_CODEC(reg, 0x12C8, Verb::I,  AirQuality_Inform);
        REGISTER_CODEC(reg, 0x12C8, Verb::RP, AirQuality_Reply);

        REGISTER_CODEC(reg, 0x22F1, Verb::I,  FanMode_Inform);
        REGISTER_CODEC(reg, 0x22F1, Verb::W,  FanMode_Write);

        REGISTER_CODEC(reg, 0x22F3, Verb::I,  FanBoost_Inform);
        REGISTER_CODEC(reg, 0x22F3, Verb::W,  FanBoost_Write);

        REGISTER_CODEC(reg, 0x22F7, Verb::I,  BypassMode_Inform);
        REGISTER_CODEC(reg, 0x22F7, Verb::W,  BypassMode_Write);

        REGISTER_CODEC(reg, 0x31E0, Verb::I,  FanDemand_Inform);

        REGISTER_CODEC(reg, 0x3120, Verb::I,  HvacStatus_Inform);
        REGISTER_CODEC(reg, 0x3120, Verb::RP, HvacStatus_Reply);

        REGISTER_CODEC(reg, 0x2E10, Verb::I,  PresenceDetected_Inform);
    }

} // namespace evohome::codecs
