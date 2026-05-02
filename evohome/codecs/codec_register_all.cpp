/**
 * @file codec_register_all.cpp
 * @brief Schema declarations + REGISTER_CODEC calls for all codecs that
 *        ship with the initial RAMSES-II sniffer.
 *
 * Adding a new codec is a 3-step recipe:
 *
 *   1. Declare a struct that derives from Payload<std::tuple<Fields...>>.
 *      Field types live in evohome::fields. Use Repeated<> for arrays.
 *   2. Add ONE REGISTER_CODEC line in register_all_codecs() below.
 *   3. (Optional) Add the friendly name to the Code enum in ramses_types.h
 *      and add a long-form name to code_long_name() in ramses_types.cpp.
 *   4. (Optional) Override `describe(std::ostream&) const` on the struct to
 *      produce a friendly one-line description; otherwise the auto-generated
 *      "field=value, field=value" formatting is used.
 *
 * Schema sources are the regex tables in
 * https://github.com/ramses-rf/ramses_rf/blob/master/src/ramses_tx/ramses.py
 * (search for "RP" / "I_" / "RQ" / "W_" entries on the relevant 4-digit
 * code). Where the regex would map to several alternative shapes (e.g.
 * fixed length vs array) we register the most common one first; additional
 * verbs/lengths can always be added later.
 */

#include "../ramses_codec.h"
#include "../ramses_fields.h"
#include "../ramses_payload.h"

#include <cstdio>
#include <ostream>

namespace evohome::codecs
{
    using namespace evohome;
    using namespace evohome::fields;

    // -----------------------------------------------------------------
    // Tiny formatting helpers used by the friendly describe() overrides.
    // Kept inline here (not in a separate header) since they are all
    // short, callsite-local and only relevant inside this translation
    // unit.
    // -----------------------------------------------------------------

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

    inline void print_temp(std::ostream &os, const TempCenti &t)
    {
        char buf[16];
        if (!t.is_valid()) { os << "--"; return; }
        std::snprintf(buf, sizeof(buf), "%.2f\xC2\xB0""C", static_cast<double>(t.celsius()));
        os << buf;
    }

    inline void print_pct(std::ostream &os, const Percent &p)
    {
        char buf[12];
        if (!p.is_valid()) { os << "--"; return; }
        std::snprintf(buf, sizeof(buf), "%.0f%%", static_cast<double>(p.percent()));
        os << buf;
    }

    // ============================================================ 30C9 ZoneTemperature
    //
    // Spec (ramses_rf):
    //   I  : ^((?:0[0-9A-F])(?:[0-9A-F]{4}))+$  -> 1+ tuples of (zone_idx, temp)
    //   RQ : ^00$                               -> always exactly "00"
    //   RP : ^0[0-9A-F]{5}$                     -> exactly (zone_idx, temp)

    struct ZoneTemperature_Inform
        : Payload<std::tuple<Repeated<ZoneIdx, TempCenti>>>
    {
        void describe(std::ostream &os) const
        {
            const auto &arr = std::get<0>(fields);
            for (uint8_t i = 0; i < arr.count; ++i)
            {
                if (i) os << ", ";
                print_zone_or_domain(os, std::get<0>(arr.rows[i]).value);
                os << " = ";
                print_temp(os, std::get<1>(arr.rows[i]));
            }
        }
    };

    struct ZoneTemperature_Request
        : Payload<std::tuple<ZoneIdx>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
        }
    };

    struct ZoneTemperature_Reply
        : Payload<std::tuple<ZoneIdx, TempCenti>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            os << " = ";
            print_temp(os, std::get<1>(fields));
        }
    };

    // ============================================================ 2309 ZoneSetpoint
    //
    //   I  : array of (zone_idx, temp)  - same shape as 30C9 I
    //   RQ : "00" (or zone_idx for some controllers)
    //   RP : (zone_idx, temp)
    //   W  : (zone_idx, temp)

    struct ZoneSetpoint_Inform     : ZoneTemperature_Inform {};
    struct ZoneSetpoint_Request    : ZoneTemperature_Request {};
    struct ZoneSetpoint_Reply      : ZoneTemperature_Reply {};
    struct ZoneSetpoint_Write      : ZoneTemperature_Reply {};

    // ============================================================ 2349 ZoneSetpointOverride
    //
    //   RQ : zone_idx (1B)
    //   I/RP/W : zone_idx, setpoint (2B), mode (1B), [unused 0F00 (2B), datetime (7B)]

    struct ZoneSetpointOverride_Request
        : Payload<std::tuple<ZoneIdx>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
        }
    };

    struct ZoneSetpointOverride_Inform
        : Payload<std::tuple<ZoneIdx, TempCenti, FixedBytes<1>, Optional<HexBlob>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            os << " = ";
            print_temp(os, std::get<1>(fields));
            const uint8_t mode = std::get<2>(fields).bytes[0];
            const char *mname =
                mode == 0x00 ? "follow schedule"
              : mode == 0x01 ? "advance"
              : mode == 0x02 ? "permanent"
              : mode == 0x03 ? "countdown"
              : mode == 0x04 ? "until-tomorrow"
              : mode == 0x05 ? "temporary"
              : nullptr;
            os << " (";
            if (mname) os << mname;
            else os << "mode " << static_cast<unsigned>(mode);
            os << ")";
        }
    };
    struct ZoneSetpointOverride_Reply : ZoneSetpointOverride_Inform {};
    struct ZoneSetpointOverride_Write : ZoneSetpointOverride_Inform {};

    // ============================================================ 1FC9 RfBind
    //
    // Each tuple is (zone_or_domain_id, code, device_id). Bindings are
    // greedy / variable-length.
    //   I, RP, W : Repeated<ZoneIdx, OpcodeBE16, DeviceId3>
    //   RQ       : just an idx byte (often 00)

    struct RfBind_Inform
        : Payload<std::tuple<Repeated<ZoneIdx, OpcodeBE16, DeviceId3>>>
    {
        void describe(std::ostream &os) const
        {
            const auto &arr = std::get<0>(fields);
            for (uint8_t i = 0; i < arr.count; ++i)
            {
                if (i) os << " | ";
                print_zone_or_domain(os, std::get<0>(arr.rows[i]).value);
                os << " <-> ";
                std::get<2>(arr.rows[i]).describe(os);
                os << " for ";
                std::get<1>(arr.rows[i]).describe(os);
            }
        }
    };
    struct RfBind_Reply  : RfBind_Inform {};
    struct RfBind_Write  : RfBind_Inform {};

    struct RfBind_Request
        : Payload<std::tuple<ZoneIdx>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
        }
    };

    // ============================================================ 31D9 FanState
    //   I : <fan_mode:1B> <flags/percent:2B> ... (3 mandatory bytes,
    //       sometimes followed by 14 bytes of unknown extension fields)

    struct FanState_Inform
        : Payload<std::tuple<ZoneIdx, FixedBytes<2>, Optional<HexBlob>>>
    {
        void describe(std::ostream &os) const
        {
            const uint8_t mode  = std::get<1>(fields).bytes[0];
            const uint8_t pct   = std::get<1>(fields).bytes[1];
            const char *mname =
                mode == 0x00 ? "off"
              : mode == 0x04 ? "low"
              : mode == 0x05 ? "medium"
              : mode == 0x06 ? "high"
              : mode == 0x07 ? "boost"
              : nullptr;
            print_zone_or_domain(os, std::get<0>(fields).value);
            os << " : fan ";
            if (mname) os << mname;
            else { char buf[8]; std::snprintf(buf, sizeof(buf), "mode=%02X", mode); os << buf; }
            char buf[16];
            std::snprintf(buf, sizeof(buf), " (%u%%)", static_cast<unsigned>(pct));
            os << buf;
        }
    };

    // ============================================================ 31DA HvacState
    //
    // Long mixed payload (typically 30 bytes). Layout per ramses_rf is:
    //   [0]      domain_id  (00 for whole house)
    //   [1..2]   exhaust temp (centi)  - usually EF EF when absent
    //   [3..4]   supply temp  (centi)
    //   [5..6]   indoor temp  (centi)
    //   [7..8]   indoor humidity / co2 mixed (manufacturer specific)
    //   [9..]    flags + remaining hex blob
    //
    // The exact layout has many variants per manufacturer; we expose the
    // first 9 bytes as best-effort and dump the rest as hex so the user
    // can grep / refine.

    struct HvacState_Inform
        : Payload<std::tuple<ZoneIdx, FixedBytes<8>, Optional<HexBlob>>>
    {
        void describe(std::ostream &os) const
        {
            const auto &raw = std::get<1>(fields).bytes;
            print_zone_or_domain(os, std::get<0>(fields).value);
            // Best-effort temp parse: 0x7FFF / 0x8000 / 0xEFEF mean "no value"
            auto print_pair = [&](const char *label, uint8_t a, uint8_t b) {
                int16_t v = static_cast<int16_t>((a << 8) | b);
                if (a == 0xEF && b == 0xEF) { os << ", " << label << "=--"; return; }
                if (v == 0x7FFF || v == static_cast<int16_t>(0x8000)) { os << ", " << label << "=--"; return; }
                char buf[24];
                std::snprintf(buf, sizeof(buf), ", %s=%.1f\xC2\xB0""C", label, v * 0.01);
                os << buf;
            };
            print_pair("exhaust", raw[0], raw[1]);
            print_pair("supply",  raw[2], raw[3]);
            print_pair("indoor",  raw[4], raw[5]);
            // Bytes 6..7 are manufacturer-specific: skip in friendly view.
        }
    };

    // ============================================================ 1298 Co2Level
    //   I/RP : (zone_idx, co2_ppm)

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
    //
    //   I : (zone_idx, humidity, temp_centi, temp_centi)  - some sensors
    //       report (rh, indoor_temp, dewpoint)
    //   RP: same as I

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

    // ============================================================ 1060 DeviceBattery
    //   I : (zone_idx, percent, low_flag)

    struct DeviceBattery_Inform
        : Payload<std::tuple<ZoneIdx, Battery>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            const auto &b = std::get<1>(fields);
            os << " : ";
            print_pct(os, b.percent);
            os << (b.low_flag == 0x00 ? " (LOW)" : " (OK)");
        }
    };

    // ============================================================ 10E0 DeviceInfo
    //
    // First 2 bytes are the schema/version, rest is a manufacturer-specific
    // device-info blob (often includes a date and a 20-byte ASCII name).
    // Auto-described as a hex blob until somebody decodes the layout.

    struct DeviceInfo_Inform : Payload<std::tuple<HexBlob>> {};
    struct DeviceInfo_Reply  : Payload<std::tuple<HexBlob>> {};

    // ============================================================ 1F09 SystemSync
    //   I/RP : "FF" + 2-byte countdown (centiseconds, BE)
    //   RQ   : "00"

    struct SystemSync_Inform
        : Payload<std::tuple<ZoneIdx, FixedBytes<2>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            // Countdown in centiseconds (0.01s units) until the next 1F09
            // beacon. Typically ~17.8s on Evohome controllers (raw=06F4).
            const auto &raw = std::get<1>(fields).bytes;
            const unsigned cs = (raw[0] << 8) | raw[1];
            char buf[32];
            std::snprintf(buf, sizeof(buf), " : next sync in %.1fs",
                          static_cast<double>(cs) * 0.01);
            os << buf;
        }
    };
    struct SystemSync_Reply   : SystemSync_Inform {};
    struct SystemSync_Request : Payload<std::tuple<ZoneIdx>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
        }
    };

    // ============================================================ 1FD4 Boiler Heart-beat
    //   I : zone_idx (always 00) + 2-byte counter / state
    //
    // Used by OpenTherm bridges to broadcast a periodic "I'm alive"
    // heartbeat. The 2-byte field is mostly a monotonic counter.

    struct BoilerHeartbeat_Inform
        : Payload<std::tuple<ZoneIdx, FixedBytes<2>>>
    {
        void describe(std::ostream &os) const
        {
            const auto &raw = std::get<1>(fields).bytes;
            const unsigned ctr = (raw[0] << 8) | raw[1];
            char buf[24];
            std::snprintf(buf, sizeof(buf), "tick %u", ctr);
            os << buf;
        }
    };

    // ============================================================ 3EF0 ActuatorState (Boiler Status)
    //   RQ : zone_idx (1B)
    //   I  : zone_idx, modulation_level (1B), flags (1B), [4B extras]
    //   RP : same as I

    struct ActuatorState_Request
        : Payload<std::tuple<ZoneIdx>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
        }
    };

    struct ActuatorState_Inform
        : Payload<std::tuple<ZoneIdx, Percent, FixedBytes<1>, Optional<HexBlob>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            os << " : modulation ";
            print_pct(os, std::get<1>(fields));
            const uint8_t flags = std::get<2>(fields).bytes[0];
            char buf[24];
            std::snprintf(buf, sizeof(buf), " (flags=0x%02X)", static_cast<unsigned>(flags));
            os << buf;
        }
    };
    struct ActuatorState_Reply : ActuatorState_Inform {};

    // ============================================================ 2E10 PresenceDetected
    //
    // Sent by HVAC PIR / motion sensors (and some ClimaRad / Itho fans
    // that have presence detection on board) to inform the ventilation
    // unit whether anybody is home. Verified payload shapes from the
    // ramses_rf test logs:
    //   I 2E10 002 0001          -> presence=true   (ClimaRad Spider)
    //   I 2E10 003 000000        -> presence=false  (ClimaRad Ventura)
    //   I 2E10 003 000100        -> presence=true   (ClimaRad Ventura)
    // So byte[0] is a domain/zone idx (always 00), byte[1] is the
    // boolean presence flag, byte[2] (if present) is a vendor extra.

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
            else
            {
                char buf[16];
                std::snprintf(buf, sizeof(buf), "presence=0x%02X",
                              static_cast<unsigned>(flag));
                os << buf;
            }
        }
    };

} // namespace evohome::codecs

namespace evohome
{
    void register_all_codecs(CodecRegistry &reg)
    {
        using namespace evohome::codecs;

        REGISTER_CODEC(reg, Code::ZoneTemperature, Verb::I,  ZoneTemperature_Inform);
        REGISTER_CODEC(reg, Code::ZoneTemperature, Verb::RQ, ZoneTemperature_Request);
        REGISTER_CODEC(reg, Code::ZoneTemperature, Verb::RP, ZoneTemperature_Reply);

        REGISTER_CODEC(reg, Code::ZoneSetpoint, Verb::I,  ZoneSetpoint_Inform);
        REGISTER_CODEC(reg, Code::ZoneSetpoint, Verb::RQ, ZoneSetpoint_Request);
        REGISTER_CODEC(reg, Code::ZoneSetpoint, Verb::RP, ZoneSetpoint_Reply);
        REGISTER_CODEC(reg, Code::ZoneSetpoint, Verb::W,  ZoneSetpoint_Write);

        REGISTER_CODEC(reg, 0x2349, Verb::RQ, ZoneSetpointOverride_Request);
        REGISTER_CODEC(reg, 0x2349, Verb::I,  ZoneSetpointOverride_Inform);
        REGISTER_CODEC(reg, 0x2349, Verb::RP, ZoneSetpointOverride_Reply);
        REGISTER_CODEC(reg, 0x2349, Verb::W,  ZoneSetpointOverride_Write);

        REGISTER_CODEC(reg, Code::RfBind, Verb::I,  RfBind_Inform);
        REGISTER_CODEC(reg, Code::RfBind, Verb::RP, RfBind_Reply);
        REGISTER_CODEC(reg, Code::RfBind, Verb::W,  RfBind_Write);
        REGISTER_CODEC(reg, Code::RfBind, Verb::RQ, RfBind_Request);

        REGISTER_CODEC(reg, Code::FanState,  Verb::I, FanState_Inform);
        REGISTER_CODEC(reg, Code::HvacState, Verb::I, HvacState_Inform);

        REGISTER_CODEC(reg, Code::Co2Level, Verb::I,  Co2Level_Inform);
        REGISTER_CODEC(reg, Code::Co2Level, Verb::RP, Co2Level_Reply);

        REGISTER_CODEC(reg, Code::IndoorHumidity, Verb::I, IndoorHumidity_Inform);

        REGISTER_CODEC(reg, Code::DeviceBattery, Verb::I, DeviceBattery_Inform);

        REGISTER_CODEC(reg, Code::DeviceInfo, Verb::I,  DeviceInfo_Inform);
        REGISTER_CODEC(reg, Code::DeviceInfo, Verb::RP, DeviceInfo_Reply);

        REGISTER_CODEC(reg, Code::SystemSync, Verb::I,  SystemSync_Inform);
        REGISTER_CODEC(reg, Code::SystemSync, Verb::RP, SystemSync_Reply);
        REGISTER_CODEC(reg, Code::SystemSync, Verb::RQ, SystemSync_Request);

        REGISTER_CODEC(reg, 0x1FD4, Verb::I,  BoilerHeartbeat_Inform);

        REGISTER_CODEC(reg, 0x3EF0, Verb::RQ, ActuatorState_Request);
        REGISTER_CODEC(reg, 0x3EF0, Verb::I,  ActuatorState_Inform);
        REGISTER_CODEC(reg, 0x3EF0, Verb::RP, ActuatorState_Reply);

        REGISTER_CODEC(reg, 0x2E10, Verb::I,  PresenceDetected_Inform);
    }
} // namespace evohome
