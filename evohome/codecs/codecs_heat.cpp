/**
 * @file codecs_heat.cpp
 * @brief Codecs for the Honeywell/Resideo "heat" half of RAMSES-II:
 *        Evohome / Sundial / Hometronic / Chronotherm controllers, TRVs,
 *        BDR91 relays, OpenTherm bridges, sundry programmers.
 *
 * Schemas come from ramses_rf's `src/ramses_tx/parsers.py` (per-code
 * `parser_XXXX` functions) cross-referenced against `ramses.py`'s regex
 * tables. See README.md (in evohome/) for the porting protocol.
 *
 * Adding a new codec is the same 3-step recipe as the rest of this folder:
 *   1. Declare a struct deriving from Payload<std::tuple<Fields...>>.
 *   2. (Optional) override describe(std::ostream&) for friendly output.
 *   3. Register it in register_heat_codecs() at the bottom of this file.
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

    // ============================================================ 30C9 ZoneTemperature
    //
    // Spec (ramses_rf):
    //   I  : array of (zone_idx, temp_centi)  - 1+ entries
    //   RQ : "00"
    //   RP : (zone_idx, temp_centi)

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
    //   Same wire shape as 30C9 - just inherit.

    struct ZoneSetpoint_Inform     : ZoneTemperature_Inform {};
    struct ZoneSetpoint_Request    : ZoneTemperature_Request {};
    struct ZoneSetpoint_Reply      : ZoneTemperature_Reply {};
    struct ZoneSetpoint_Write      : ZoneTemperature_Reply {};

    // ============================================================ 2349 ZoneSetpointOverride
    //   RQ : zone_idx (1B)
    //   I/RP/W : zone_idx, setpoint_centi (2B), mode (1B), [unused 0xFFFFFF (3B), datetime (7B)]

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
    //   I/RP/W : Repeated<ZoneIdx, OpcodeBE16, DeviceId3>
    //   RQ     : zone_idx

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
    // Long manufacturer-specific blob. First byte is the schema version.
    // The remainder is too vendor-specific to decode here; we surface it
    // as a hex dump so the user can grep / extend later.

    struct DeviceInfo_Inform : Payload<std::tuple<HexBlob>> {};
    struct DeviceInfo_Reply  : Payload<std::tuple<HexBlob>> {};

    // ============================================================ 10E1 DeviceId
    //   I/RP : zone_idx + 3-byte device id reference

    struct DeviceId_Reply
        : Payload<std::tuple<ZoneIdx, DeviceId3>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            os << " : ";
            std::get<1>(fields).describe(os);
        }
    };

    // ============================================================ 1F09 SystemSync
    //   I/RP : <00|01|F8|FF> + 2-byte countdown (centiseconds, BE)
    //   RQ   : "00"

    struct SystemSync_Inform
        : Payload<std::tuple<ZoneIdx, FixedBytes<2>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            const auto &raw = std::get<1>(fields).bytes;
            const unsigned cs = (raw[0] << 8) | raw[1];
            char buf[32];
            std::snprintf(buf, sizeof(buf), " : next sync in %.1fs",
                          static_cast<double>(cs) * 0.1);
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

    // ============================================================ 1FD4 BoilerHeartbeat
    //   I : zone_idx + 2B counter

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
    //   I  : zone_idx, modulation_pct (1B), flags (1B), [extras]
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

    // ============================================================ 3EF1 ActuatorCycle
    //
    // Sent by BDR / OTB to report the current ON/OFF cycle position.
    //   I/RP : zone(1) + cycle_countdown(2B BE secs) + actuator_countdown(2B BE secs)
    //          + modulation_pct(1B 0..200) + trailing(1B 0xFF for BDR, varies for OTB)
    //
    // 0x7FFF on either countdown means "not applicable".

    struct ActuatorCycle_Inform
        : Payload<std::tuple<ZoneIdx, FixedBytes<2>, FixedBytes<2>, Percent, FixedBytes<1>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            const auto &cdn = std::get<1>(fields).bytes;
            const auto &act = std::get<2>(fields).bytes;
            const uint16_t cycle  = static_cast<uint16_t>((cdn[0] << 8) | cdn[1]);
            const uint16_t actrun = static_cast<uint16_t>((act[0] << 8) | act[1]);
            os << " : modulation ";
            print_pct(os, std::get<3>(fields));
            char buf[40];
            if (cycle != 0x7FFF)
            {
                std::snprintf(buf, sizeof(buf), ", cycle %us", static_cast<unsigned>(cycle));
                os << buf;
            }
            if (actrun != 0x7FFF)
            {
                std::snprintf(buf, sizeof(buf), ", running %us", static_cast<unsigned>(actrun));
                os << buf;
            }
        }
    };
    struct ActuatorCycle_Reply : ActuatorCycle_Inform {};

    // ============================================================ 3B00 ActuatorSync
    //   I : zone_idx + 0xC8 (always)
    //
    // Broadcast at TPI cycle boundaries (CTL/PRG send FCC8 at start of
    // cycle; BDR sends 00C8 at the end). Effectively "tick".

    struct ActuatorSync_Inform
        : Payload<std::tuple<ZoneIdx, FixedBytes<1>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            const uint8_t v = std::get<1>(fields).bytes[0];
            os << (v == 0xC8 ? " : tick" : " : raw ");
            if (v != 0xC8) print_hex_byte(os, v);
        }
    };

    // ============================================================ 0008 RelayDemand
    //   I/RP : zone_or_domain(1) + percent(1, 0..200 -> 0..100%)

    struct RelayDemand_Inform
        : Payload<std::tuple<ZoneIdx, Percent>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            os << " : ";
            print_pct(os, std::get<1>(fields));
        }
    };
    struct RelayDemand_Reply : RelayDemand_Inform {};

    // ============================================================ 0009 RelayFailsafe
    //   I : zone_or_domain(1) + failsafe_flag(1: 00=disabled, 01=enabled) + unknown(1)
    //
    // Failsafe defines what the relay does if comms are lost:
    //   disabled -> relay held OFF
    //   enabled  -> relay cycles 20% on / 80% off (frost protection)

    struct RelayFailsafe_Inform
        : Payload<std::tuple<ZoneIdx, FixedBytes<1>, Optional<FixedBytes<1>>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            const uint8_t f = std::get<1>(fields).bytes[0];
            os << " : failsafe " << (f == 0x01 ? "ON" : f == 0x00 ? "OFF" : "?");
        }
    };

    // ============================================================ 000A ZoneParams
    //
    //   I/RP : array of (zone_idx, bitmap, min_temp(2B BE centi), max_temp(2B BE centi))
    //   W    : single 6-byte entry (zone_idx + 5 bytes)
    //   RQ   : "0Z" (zone idx) or "0Z+entries"

    struct ZoneParams_Inform
        : Payload<std::tuple<Repeated<ZoneIdx, FixedBytes<1>, TempCenti, TempCenti>>>
    {
        void describe(std::ostream &os) const
        {
            const auto &arr = std::get<0>(fields);
            for (uint8_t i = 0; i < arr.count; ++i)
            {
                if (i) os << " | ";
                print_zone_or_domain(os, std::get<0>(arr.rows[i]).value);
                os << " : ";
                print_temp(os, std::get<2>(arr.rows[i]));
                os << "..";
                print_temp(os, std::get<3>(arr.rows[i]));
                const uint8_t bm = std::get<1>(arr.rows[i]).bytes[0];
                if (bm & 0x01) os << " noLocalOverride";
                if (bm & 0x02) os << " noOpenWindow";
                if (bm & 0x10) os << " noMultiroom";
            }
        }
    };
    struct ZoneParams_Reply : ZoneParams_Inform {};
    struct ZoneParams_Write
        : Payload<std::tuple<ZoneIdx, FixedBytes<1>, TempCenti, TempCenti>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            os << " : ";
            print_temp(os, std::get<2>(fields));
            os << "..";
            print_temp(os, std::get<3>(fields));
        }
    };

    // ============================================================ 0418 FaultLog
    //
    // Fault log entry. Length 22 bytes, dense layout per ramses_rf:
    //   [0:2]   "00" header
    //   [2:4]   fault_state  (B0 = active, C0 = restored, ...)
    //   [4:6]   log_idx (00 = newest)
    //   [6:8]   "00"
    //   [8:10]  fault_type
    //   [10:12] domain_idx (zone_idx or FC/FA/F9)
    //   [12:14] device_class
    //   [14:18] "0000"
    //   [18:30] timestamp (6-byte little-endian: SS MM HH DD MO YY)
    //   [30:38] device_id (4 bytes)
    //   [38:44] trailing (always 7000000001 or similar)
    //
    // We surface state, type, domain, device-id; rest as hex.

    // Schema is variable across firmware revisions (the timestamp/device-id
    // alignment moves around). Rather than reject frames whose layout
    // doesn't match an exact byte-for-byte template we surface the bytes
    // we can identify (state/log-idx/type/domain) and dump the rest.

    struct FaultLog_Inform
        : Payload<std::tuple<FixedBytes<1>, FixedBytes<1>, FixedBytes<1>, FixedBytes<1>,
                              FixedBytes<1>, FixedBytes<1>, HexBlob>>
    {
        void describe(std::ostream &os) const
        {
            // Byte layout per ramses_rf parser_0418 / parse_fault_log_entry:
            //   [0]=00  [1]=00  [2]=log_idx  [3]=fault_state  [4]=fault_type
            //   [5]=00  [6..]=domain + class + ts(6B) + device_id(3B) + tail
            const uint8_t log_idx = std::get<2>(fields).bytes[0];
            const uint8_t state   = std::get<3>(fields).bytes[0];
            const uint8_t type    = std::get<4>(fields).bytes[0];
            const auto   &rest    = std::get<6>(fields);

            const char *sname =
                state == 0xB0 ? "active"
              : state == 0xC0 ? "restored"
              : nullptr;
            os << "log[" << static_cast<unsigned>(log_idx) << "] ";
            if (sname) os << sname;
            else { os << "state=0x"; print_hex_byte(os, state); }
            os << " type=0x"; print_hex_byte(os, type);
            os << " raw=";
            print_hex_buffer(os, rest.bytes.data(), rest.length);
        }
    };
    struct FaultLog_Reply : FaultLog_Inform {};
    struct FaultLog_Request
        : Payload<std::tuple<FixedBytes<1>, FixedBytes<1>, FixedBytes<1>>>
    {
        void describe(std::ostream &os) const
        {
            const uint8_t log_idx = std::get<2>(fields).bytes[0];
            os << "log[" << static_cast<unsigned>(log_idx) << "]";
        }
    };

    // ============================================================ 1100 TpiParams
    //   I/RP : zone_or_domain + cycle_rate(1B/4) + min_on(1B/4) + min_off(1B/4) + 1B unk
    //          + optional 2B proportional_band_width(centi degrees)
    // For domain prefix only FC seen in the wild.

    struct TpiParams_Reply
        : Payload<std::tuple<ZoneIdx, FixedBytes<1>, FixedBytes<1>, FixedBytes<1>,
                              FixedBytes<1>, Optional<TempCenti>, Optional<HexBlob>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            const uint8_t cycle = std::get<1>(fields).bytes[0];
            const uint8_t monT  = std::get<2>(fields).bytes[0];
            const uint8_t mofT  = std::get<3>(fields).bytes[0];
            char buf[64];
            std::snprintf(buf, sizeof(buf),
                " : %u cycles/h, on>=%.2fmin, off>=%.2fmin",
                static_cast<unsigned>(cycle / 4),
                monT / 4.0,
                mofT / 4.0);
            os << buf;
            if (std::get<5>(fields).value)
            {
                os << ", PBW=";
                print_temp(os, *std::get<5>(fields).value);
            }
        }
    };
    struct TpiParams_Inform : TpiParams_Reply {};

    // ============================================================ 10A0 DhwParams
    //   RQ : "00" or 6-byte
    //   I/RP : zone(1) + setpoint(2B BE centi) [+ overrun(1B mins) + differential(2B BE centi)]

    struct DhwParams_Reply
        : Payload<std::tuple<ZoneIdx, TempCenti, Optional<FixedBytes<1>>, Optional<TempCenti>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            os << " : setpoint=";
            print_temp(os, std::get<1>(fields));
            if (std::get<2>(fields).value)
            {
                char buf[24];
                std::snprintf(buf, sizeof(buf), ", overrun=%umin",
                    static_cast<unsigned>(std::get<2>(fields).value->bytes[0]));
                os << buf;
            }
            if (std::get<3>(fields).value)
            {
                os << ", differential=";
                print_temp(os, *std::get<3>(fields).value);
            }
        }
    };
    struct DhwParams_Inform  : DhwParams_Reply {};
    struct DhwParams_Request
        : Payload<std::tuple<ZoneIdx>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
        }
    };

    // ============================================================ 1260 DhwTemperature
    //   I/RP : zone(1) + temp(2B BE centi)

    struct DhwTemperature_Reply
        : Payload<std::tuple<ZoneIdx, TempCenti>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            os << " = ";
            print_temp(os, std::get<1>(fields));
        }
    };
    struct DhwTemperature_Inform : DhwTemperature_Reply {};

    // ============================================================ 1290 OutdoorTemperature
    //   I/RP : zone(1) + temp(2B BE centi)

    struct OutdoorTemperature_Reply
        : Payload<std::tuple<ZoneIdx, TempCenti>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            os << " = ";
            print_temp(os, std::get<1>(fields));
        }
    };
    struct OutdoorTemperature_Inform : OutdoorTemperature_Reply {};

    // ============================================================ 12B0 WindowState
    //   I/RP : zone(1) + state(2B): 0000=closed, C800=open, FFFF=N/A

    struct WindowState_Inform
        : Payload<std::tuple<ZoneIdx, FixedBytes<2>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            const auto &b = std::get<1>(fields).bytes;
            os << " : window ";
            if      (b[0] == 0x00 && b[1] == 0x00) os << "closed";
            else if (b[0] == 0xC8 && b[1] == 0x00) os << "OPEN";
            else if (b[0] == 0xFF && b[1] == 0xFF) os << "--";
            else { os << "raw="; print_hex_be16(os, b[0], b[1]); }
        }
    };
    struct WindowState_Reply : WindowState_Inform {};

    // ============================================================ 1300 ChPressure
    //   I/RP : zone(1) + pressure(2B BE, centi-bar) ; 0x09F6 == "no value"

    struct ChPressure_Reply
        : Payload<std::tuple<ZoneIdx, FixedBytes<2>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            const auto &b = std::get<1>(fields).bytes;
            os << " : ";
            if (b[0] == 0x09 && b[1] == 0xF6) { os << "--"; return; }
            const unsigned raw = (b[0] << 8) | b[1];
            char buf[24];
            std::snprintf(buf, sizeof(buf), "%.2f bar", raw * 0.01);
            os << buf;
        }
    };
    struct ChPressure_Inform : ChPressure_Reply {};

    // ============================================================ 1F41 DhwMode
    //   RP/I/W : zone(1) + active_byte(1) + mode(1) + 0xFFFFFF + [12-byte datetime if mode=04]

    struct DhwMode_Reply
        : Payload<std::tuple<ZoneIdx, FixedBytes<1>, FixedBytes<1>, FixedBytes<3>,
                              Optional<HexBlob>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            const uint8_t active = std::get<1>(fields).bytes[0];
            const uint8_t mode   = std::get<2>(fields).bytes[0];
            os << " : "
               << (active == 0x00 ? "OFF"
                 : active == 0x01 ? "ON"
                 : "--");
            const char *mname =
                mode == 0x00 ? "follow schedule"
              : mode == 0x01 ? "advance"
              : mode == 0x02 ? "permanent"
              : mode == 0x04 ? "until-tomorrow"
              : mode == 0x05 ? "temporary"
              : nullptr;
            os << " (" << (mname ? mname : "mode=?") << ")";
        }
    };
    struct DhwMode_Inform  : DhwMode_Reply {};
    struct DhwMode_Write   : DhwMode_Reply {};

    // ============================================================ 22D9 BoilerSetpoint
    //   I/RP : zone(1) + temp(2B BE centi)
    //
    // OTB-side desired boiler flow temperature.

    struct BoilerSetpoint_Reply
        : Payload<std::tuple<ZoneIdx, TempCenti>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            os << " = ";
            print_temp(os, std::get<1>(fields));
        }
    };
    struct BoilerSetpoint_Inform : BoilerSetpoint_Reply {};

    // ============================================================ 2249 SetpointNowNext
    //   I : zone(1) + setpoint_now(2B centi) + setpoint_next(2B centi) + minutes_remaining(2B BE)

    struct SetpointNowNext_Inform
        : Payload<std::tuple<Repeated<ZoneIdx, TempCenti, TempCenti, FixedBytes<2>>>>
    {
        void describe(std::ostream &os) const
        {
            const auto &arr = std::get<0>(fields);
            for (uint8_t i = 0; i < arr.count; ++i)
            {
                if (i) os << " | ";
                print_zone_or_domain(os, std::get<0>(arr.rows[i]).value);
                os << " : now=";
                print_temp(os, std::get<1>(arr.rows[i]));
                os << ", next=";
                print_temp(os, std::get<2>(arr.rows[i]));
                const auto &mr = std::get<3>(arr.rows[i]).bytes;
                const unsigned mins = (mr[0] << 8) | mr[1];
                if (mr[0] != 0xFF || mr[1] != 0xFF)
                {
                    char buf[24];
                    std::snprintf(buf, sizeof(buf), " (in %u min)", mins);
                    os << buf;
                }
            }
        }
    };

    // ============================================================ 2389 AverageTemp
    //   I : zone(1) + temp(2B centi). Only seen from class 03 thermostats.

    struct AverageTemp_Inform
        : Payload<std::tuple<ZoneIdx, TempCenti>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            os << " = ";
            print_temp(os, std::get<1>(fields));
        }
    };

    // ============================================================ 2E04 SystemMode
    //
    // Evohome (8-byte form):
    //   mode(1B) + 6B until-time(SS MM HH DD MO YY) + 1B "is_permanent"(00=until,01=perm)
    //
    // Hometronic (16-byte form): same shell but with a 7-byte trailing block.
    // We expose the mode byte and (when applicable) a little hint of the
    // until-time. Modes per ramses_rf SYS_MODE_MAP:
    //   00=AUTO 01=HEAT_OFF 02=ECO 03=AWAY 04=DAY_OFF 05=DAY_OFF_ECO
    //   06=AUTO_RESET 07=CUSTOM FF=NONE

    struct SystemMode_Inform
        : Payload<std::tuple<FixedBytes<1>, FixedBytes<6>, FixedBytes<1>, Optional<HexBlob>>>
    {
        void describe(std::ostream &os) const
        {
            const uint8_t mode = std::get<0>(fields).bytes[0];
            const char *name =
                mode == 0x00 ? "AUTO"
              : mode == 0x01 ? "HEAT_OFF"
              : mode == 0x02 ? "ECO"
              : mode == 0x03 ? "AWAY"
              : mode == 0x04 ? "DAY_OFF"
              : mode == 0x05 ? "DAY_OFF_ECO"
              : mode == 0x06 ? "AUTO_RESET"
              : mode == 0x07 ? "CUSTOM"
              : mode == 0xFF ? "NONE"
              : nullptr;
            if (name) os << name;
            else { os << "mode=0x"; print_hex_byte(os, mode); }

            const auto &ts = std::get<1>(fields).bytes;
            const uint8_t perm = std::get<2>(fields).bytes[0];
            if (perm == 0x00 && (ts[0] | ts[1] | ts[2] | ts[3] | ts[4] | ts[5]) != 0xFF)
            {
                char buf[40];
                std::snprintf(buf, sizeof(buf),
                    ", until 20%02u-%02u-%02u %02u:%02u:%02u",
                    static_cast<unsigned>(ts[5]),
                    static_cast<unsigned>(ts[4]),
                    static_cast<unsigned>(ts[3]),
                    static_cast<unsigned>(ts[2]),
                    static_cast<unsigned>(ts[1]),
                    static_cast<unsigned>(ts[0]));
                os << buf;
            }
            else if (perm == 0x01)
            {
                os << ", permanent";
            }
        }
    };

    // ============================================================ 3150 HeatDemand
    //   I/RP : zone_or_domain(1) + percent(1, 0..200 -> 0..100%)
    //
    // FC domain = total of all zones (the heat demand the boiler should
    // honour). Per-zone variant comes from each TRV/UFC.

    struct HeatDemand_Inform
        : Payload<std::tuple<ZoneIdx, Percent>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            os << " : demand=";
            print_pct(os, std::get<1>(fields));
        }
    };
    struct HeatDemand_Reply : HeatDemand_Inform {};

    // ============================================================ 3200 BoilerOutputTemp
    //   I/RP : zone(1) + temp(2B BE centi). Output = supplied flow.

    struct BoilerOutputTemp_Reply
        : Payload<std::tuple<ZoneIdx, TempCenti>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            os << " : flow=";
            print_temp(os, std::get<1>(fields));
        }
    };
    struct BoilerOutputTemp_Inform : BoilerOutputTemp_Reply {};

    // ============================================================ 3210 BoilerReturnTemp
    //   I/RP : zone(1) + temp(2B BE centi).

    struct BoilerReturnTemp_Reply
        : Payload<std::tuple<ZoneIdx, TempCenti>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            os << " : return=";
            print_temp(os, std::get<1>(fields));
        }
    };
    struct BoilerReturnTemp_Inform : BoilerReturnTemp_Reply {};

    // ============================================================ 313F Datetime
    //   I/RP : zone(1) + 1B unknown + 7B (SS MM HH DD MO YY YY-low YY-high?)
    //
    // Per ramses_rf the date/time is a 7-byte LE block following the
    // "unknown" byte. Exact format: SS MM HH DD MO YY YY (year is BE 16-bit).

    struct Datetime_Reply
        : Payload<std::tuple<ZoneIdx, FixedBytes<1>, FixedBytes<7>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            const auto &t = std::get<2>(fields).bytes;
            const unsigned yy = (t[5] << 8) | t[6];
            char buf[40];
            std::snprintf(buf, sizeof(buf),
                " : %04u-%02u-%02u %02u:%02u:%02u",
                yy,
                static_cast<unsigned>(t[4]),
                static_cast<unsigned>(t[3]),
                static_cast<unsigned>(t[2]),
                static_cast<unsigned>(t[1]),
                static_cast<unsigned>(t[0]));
            os << buf;
        }
    };
    struct Datetime_Inform  : Datetime_Reply {};

    // ----------------------------------------------------------------
    // Registration
    // ----------------------------------------------------------------

    void register_heat_codecs(CodecRegistry &reg)
    {
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

        REGISTER_CODEC(reg, Code::DeviceBattery, Verb::I, DeviceBattery_Inform);

        REGISTER_CODEC(reg, Code::DeviceInfo, Verb::I,  DeviceInfo_Inform);
        REGISTER_CODEC(reg, Code::DeviceInfo, Verb::RP, DeviceInfo_Reply);

        REGISTER_CODEC(reg, 0x10E1, Verb::RP, DeviceId_Reply);
        REGISTER_CODEC(reg, 0x10E1, Verb::I,  DeviceId_Reply);

        REGISTER_CODEC(reg, Code::SystemSync, Verb::I,  SystemSync_Inform);
        REGISTER_CODEC(reg, Code::SystemSync, Verb::RP, SystemSync_Reply);
        REGISTER_CODEC(reg, Code::SystemSync, Verb::RQ, SystemSync_Request);

        REGISTER_CODEC(reg, 0x1FD4, Verb::I,  BoilerHeartbeat_Inform);

        REGISTER_CODEC(reg, Code::ActuatorState, Verb::RQ, ActuatorState_Request);
        REGISTER_CODEC(reg, Code::ActuatorState, Verb::I,  ActuatorState_Inform);
        REGISTER_CODEC(reg, Code::ActuatorState, Verb::RP, ActuatorState_Reply);

        REGISTER_CODEC(reg, Code::ActuatorCycle, Verb::I,  ActuatorCycle_Inform);
        REGISTER_CODEC(reg, Code::ActuatorCycle, Verb::RP, ActuatorCycle_Reply);

        REGISTER_CODEC(reg, 0x3B00, Verb::I,  ActuatorSync_Inform);

        REGISTER_CODEC(reg, 0x0008, Verb::I,  RelayDemand_Inform);
        REGISTER_CODEC(reg, 0x0008, Verb::RP, RelayDemand_Reply);

        REGISTER_CODEC(reg, 0x0009, Verb::I,  RelayFailsafe_Inform);

        REGISTER_CODEC(reg, 0x000A, Verb::I,  ZoneParams_Inform);
        REGISTER_CODEC(reg, 0x000A, Verb::RP, ZoneParams_Reply);
        REGISTER_CODEC(reg, 0x000A, Verb::W,  ZoneParams_Write);

        REGISTER_CODEC(reg, 0x0418, Verb::I,  FaultLog_Inform);
        REGISTER_CODEC(reg, 0x0418, Verb::RP, FaultLog_Reply);
        REGISTER_CODEC(reg, 0x0418, Verb::RQ, FaultLog_Request);

        REGISTER_CODEC(reg, 0x1100, Verb::I,  TpiParams_Inform);
        REGISTER_CODEC(reg, 0x1100, Verb::RP, TpiParams_Reply);

        REGISTER_CODEC(reg, 0x10A0, Verb::I,  DhwParams_Inform);
        REGISTER_CODEC(reg, 0x10A0, Verb::RP, DhwParams_Reply);
        REGISTER_CODEC(reg, 0x10A0, Verb::RQ, DhwParams_Request);

        REGISTER_CODEC(reg, Code::DhwTemperature, Verb::I,  DhwTemperature_Inform);
        REGISTER_CODEC(reg, Code::DhwTemperature, Verb::RP, DhwTemperature_Reply);

        REGISTER_CODEC(reg, 0x1290, Verb::I,  OutdoorTemperature_Inform);
        REGISTER_CODEC(reg, 0x1290, Verb::RP, OutdoorTemperature_Reply);

        REGISTER_CODEC(reg, 0x12B0, Verb::I,  WindowState_Inform);
        REGISTER_CODEC(reg, 0x12B0, Verb::RP, WindowState_Reply);

        REGISTER_CODEC(reg, 0x1300, Verb::I,  ChPressure_Inform);
        REGISTER_CODEC(reg, 0x1300, Verb::RP, ChPressure_Reply);

        REGISTER_CODEC(reg, 0x1F41, Verb::I,  DhwMode_Inform);
        REGISTER_CODEC(reg, 0x1F41, Verb::RP, DhwMode_Reply);
        REGISTER_CODEC(reg, 0x1F41, Verb::W,  DhwMode_Write);

        REGISTER_CODEC(reg, 0x22D9, Verb::I,  BoilerSetpoint_Inform);
        REGISTER_CODEC(reg, 0x22D9, Verb::RP, BoilerSetpoint_Reply);

        REGISTER_CODEC(reg, 0x2249, Verb::I,  SetpointNowNext_Inform);

        REGISTER_CODEC(reg, 0x2389, Verb::I,  AverageTemp_Inform);

        REGISTER_CODEC(reg, 0x2E04, Verb::I,  SystemMode_Inform);
        REGISTER_CODEC(reg, 0x2E04, Verb::RP, SystemMode_Inform);

        REGISTER_CODEC(reg, 0x3150, Verb::I,  HeatDemand_Inform);
        REGISTER_CODEC(reg, 0x3150, Verb::RP, HeatDemand_Reply);

        REGISTER_CODEC(reg, 0x3200, Verb::I,  BoilerOutputTemp_Inform);
        REGISTER_CODEC(reg, 0x3200, Verb::RP, BoilerOutputTemp_Reply);

        REGISTER_CODEC(reg, 0x3210, Verb::I,  BoilerReturnTemp_Inform);
        REGISTER_CODEC(reg, 0x3210, Verb::RP, BoilerReturnTemp_Reply);

        REGISTER_CODEC(reg, 0x313F, Verb::I,  Datetime_Inform);
        REGISTER_CODEC(reg, 0x313F, Verb::RP, Datetime_Reply);
    }

} // namespace evohome::codecs
