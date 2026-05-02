/**
 * @file codecs_opentherm.cpp
 * @brief Codec for the RAMSES `3220` OpenTherm wrapper.
 *
 * `3220` carries a single 4-byte OpenTherm wire frame from the OTB
 * (R8810A/R8820A "OpenTherm Bridge") between the controller and the
 * actual boiler. Layout:
 *
 *   payload[0] = 0x00            // domain idx (always 00)
 *   payload[1] = pTTTTSSSS       // p=parity, T=msg-type<<4, S=spare(0)
 *   payload[2] = data_id         // 0x00..0x7F
 *   payload[3] = data_value HB   // big-endian 16-bit value
 *   payload[4] = data_value LB
 *
 * msg-type bits (3): 0=Read-Data, 1=Write-Data, 2=Invalid-Data,
 *                    3=-reserved-, 4=Read-Ack, 5=Write-Ack,
 *                    6=Data-Invalid, 7=Unknown-DataId.
 *
 * Each data-id has its own value layout (see OPENTHERM_MESSAGES below).
 * Most are F8.8 (signed fixed-point, 1/256 precision); some are pairs
 * of u8/s8 in HB/LB; ID 0 (Status) is two bitmaps (master+slave).
 *
 * Sources:
 *   - ramses_rf/src/ramses_tx/opentherm.py   (master schema + decoder)
 *   - OpenTherm 2.2/2.3 protocol spec        (data-id documentation)
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

    // -------------------------------------------------------------- Helpers

    namespace ot
    {
        enum class ValueType : uint8_t {
            None,        // no value (status, msg-type 0/2/3/6/7)
            F8_8,        // signed 8.8 fixed-point (most temps, %, bar, L/min)
            F8_8_PCT,    // F8.8, 0..100 -> 0..100%
            U16,         // unsigned 16-bit (counters, hours)
            S16,         // signed 16-bit
            U8U8,        // HB+LB as two unsigned bytes
            U8S8,        // HB unsigned, LB signed
            S8S8,        // both signed
            FLAG_STATUS, // ID 0  : 8 master flags + 8 slave flags
            FLAG_FAULT,  // ID 5  : fault flags HB + OEM code LB
            FLAG_REMOTE, // ID 6  : remote enable flags
            FLAG_SLAVE,  // ID 3  : slave config flags HB + member id LB
            FLAG_MASTER, // ID 2  : master config flags HB + member id LB
            VERSION,     // ID 7E/7F : product type/version (2 bytes)
        };

        struct DataId {
            const char *name;
            ValueType   vt;
        };

        static constexpr DataId kIds[0x80] = {
            // 0x00
            {"Status",                                  ValueType::FLAG_STATUS},
            {"Control setpoint (°C)",                   ValueType::F8_8},
            {"Master configuration",                    ValueType::FLAG_MASTER},
            {"Slave configuration",                     ValueType::FLAG_SLAVE},
            {"Remote command",                          ValueType::U8U8},
            {"Fault flags & OEM code",                  ValueType::FLAG_FAULT},
            {"Remote parameter flags",                  ValueType::FLAG_REMOTE},
            {"Cooling control signal (%)",              ValueType::F8_8_PCT},
            {"CH2 control setpoint (°C)",               ValueType::F8_8},
            {"Remote override room setpoint (°C)",      ValueType::F8_8},
            {"Number of TSPs supported",                ValueType::U8U8},
            {"TSP entry",                               ValueType::U8U8},
            {"Size of fault history buffer",            ValueType::U8U8},
            {"FHB entry",                               ValueType::U8U8},
            {"Max. relative modulation level (%)",      ValueType::F8_8_PCT},
            {"Max boiler capacity / min mod (%)",       ValueType::U8U8},
            // 0x10
            {"Room setpoint (°C)",                      ValueType::F8_8},
            {"Relative modulation level (%)",           ValueType::F8_8_PCT},
            {"CH water pressure (bar)",                 ValueType::F8_8},
            {"DHW flow rate (L/min)",                   ValueType::F8_8},
            {"Day & time",                              ValueType::U8U8},
            {"Date",                                    ValueType::U8U8},
            {"Year",                                    ValueType::U16},
            {"CH2 room setpoint (°C)",                  ValueType::F8_8},
            {"Room temperature (°C)",                   ValueType::F8_8},
            {"Boiler flow water temp (°C)",             ValueType::F8_8},
            {"DHW temperature (°C)",                    ValueType::F8_8},
            {"Outside temperature (°C)",                ValueType::F8_8},
            {"Return water temperature (°C)",           ValueType::F8_8},
            {"Solar storage temperature (°C)",          ValueType::F8_8},
            {"Solar collector temperature (°C)",        ValueType::F8_8},
            {"CH2 flow water temperature (°C)",         ValueType::F8_8},
            // 0x20
            {"DHW2 temperature (°C)",                   ValueType::F8_8},
            {"Exhaust temperature (°C)",                ValueType::S16},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            // 0x30
            {"DHW setpoint bounds (°C)",                ValueType::S8S8},
            {"Max CH water setpoint bounds (°C)",       ValueType::S8S8},
            {"OTC heat curve ratio bounds",             ValueType::S8S8},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None},
            {"DHW setpoint (°C)",                       ValueType::F8_8},
            {"Max CH water setpoint (°C)",              ValueType::F8_8},
            {"OTC heat curve ratio",                    ValueType::F8_8},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None},
            // 0x40..0x6F (mostly unused / vendor)
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            {nullptr, ValueType::None}, {nullptr, ValueType::None},
            // 0x70
            {nullptr, ValueType::None},
            {"Number of un-successful burner starts",   ValueType::U16},
            {"Times flame signal too low",              ValueType::U16},
            {"OEM diagnostic code",                     ValueType::U16},
            {"Number of starts burner",                 ValueType::U16},
            {"Number of starts CH pump",                ValueType::U16},
            {"Number of starts DHW pump",               ValueType::U16},
            {"Starts burner during DHW mode",           ValueType::U16},
            {"Hours burner running",                    ValueType::U16},
            {"Hours CH pump running",                   ValueType::U16},
            {"Hours DHW pump running",                  ValueType::U16},
            {"Hours DHW burner running",                ValueType::U16},
            {"OpenTherm version Master",                ValueType::F8_8},
            {"OpenTherm version Slave",                 ValueType::F8_8},
            {"Master product version & type",           ValueType::VERSION},
            {"Slave product version & type",            ValueType::VERSION},
        };

        inline const char *msg_type_name(uint8_t t)
        {
            switch (t & 0x07) {
                case 0: return "Read-Data";
                case 1: return "Write-Data";
                case 2: return "Invalid-Data";
                case 3: return "-reserved-";
                case 4: return "Read-Ack";
                case 5: return "Write-Ack";
                case 6: return "Data-Invalid";
                case 7: return "Unknown-DataId";
            }
            return "?";
        }

        // Status bitmaps for ID 0x00.
        struct FlagBit { uint16_t bit; const char *name; };

        static constexpr FlagBit kStatusMaster[] = {
            {0x0100, "CH-enable"}, {0x0200, "DHW-enable"},
            {0x0400, "Cool-enable"}, {0x0800, "OTC-active"},
            {0x1000, "CH2-enable"}, {0x2000, "Summer/Winter"},
            {0x4000, "DHW-block"}, {0, nullptr}
        };
        static constexpr FlagBit kStatusSlave[] = {
            {0x0001, "Fault"}, {0x0002, "CH-active"},
            {0x0004, "DHW-active"}, {0x0008, "Flame-on"},
            {0x0010, "Cooling-on"}, {0x0020, "CH2-active"},
            {0x0040, "Diagnostic"}, {0, nullptr}
        };
        static constexpr FlagBit kFault[] = {
            {0x0100, "Service-req"}, {0x0200, "Lockout-reset"},
            {0x0400, "Low-water-pressure"}, {0x0800, "Gas/flame"},
            {0x1000, "Air-pressure"}, {0x2000, "Over-temp"},
            {0, nullptr}
        };
        static constexpr FlagBit kRemote[] = {
            {0x0100, "DHW-setpoint-en"}, {0x0200, "MaxCH-setpoint-en"},
            {0x0001, "DHW-setpoint-RW"}, {0x0002, "MaxCH-setpoint-RW"},
            {0, nullptr}
        };

        inline void print_flag_list(std::ostream &os, uint16_t bits, const FlagBit *table)
        {
            bool first = true;
            for (auto p = table; p->bit; ++p) {
                if (bits & p->bit) {
                    if (!first) os << ",";
                    os << p->name;
                    first = false;
                }
            }
            if (first) os << "(none)";
        }

        inline void print_value(std::ostream &os, ValueType vt, uint8_t hb, uint8_t lb, bool dataValid)
        {
            char buf[40];
            if (!dataValid) { os << "--"; return; }
            switch (vt) {
                case ValueType::None:
                    std::snprintf(buf, sizeof(buf), "0x%02X%02X",
                        static_cast<unsigned>(hb), static_cast<unsigned>(lb));
                    os << buf;
                    break;
                case ValueType::F8_8: {
                    const int16_t raw = static_cast<int16_t>((hb << 8) | lb);
                    std::snprintf(buf, sizeof(buf), "%.2f", raw / 256.0);
                    os << buf;
                    break;
                }
                case ValueType::F8_8_PCT: {
                    const int16_t raw = static_cast<int16_t>((hb << 8) | lb);
                    std::snprintf(buf, sizeof(buf), "%.1f%%", raw / 256.0);
                    os << buf;
                    break;
                }
                case ValueType::U16: {
                    const uint16_t raw = static_cast<uint16_t>((hb << 8) | lb);
                    std::snprintf(buf, sizeof(buf), "%u", static_cast<unsigned>(raw));
                    os << buf;
                    break;
                }
                case ValueType::S16: {
                    const int16_t raw = static_cast<int16_t>((hb << 8) | lb);
                    std::snprintf(buf, sizeof(buf), "%d", static_cast<int>(raw));
                    os << buf;
                    break;
                }
                case ValueType::U8U8:
                    std::snprintf(buf, sizeof(buf), "%u/%u",
                        static_cast<unsigned>(hb), static_cast<unsigned>(lb));
                    os << buf;
                    break;
                case ValueType::U8S8:
                    std::snprintf(buf, sizeof(buf), "%u/%d",
                        static_cast<unsigned>(hb), static_cast<int>(static_cast<int8_t>(lb)));
                    os << buf;
                    break;
                case ValueType::S8S8:
                    std::snprintf(buf, sizeof(buf), "%d..%d",
                        static_cast<int>(static_cast<int8_t>(hb)),
                        static_cast<int>(static_cast<int8_t>(lb)));
                    os << buf;
                    break;
                case ValueType::FLAG_STATUS:
                    os << "master=[";
                    print_flag_list(os, static_cast<uint16_t>(hb << 8), kStatusMaster);
                    os << "] slave=[";
                    print_flag_list(os, static_cast<uint16_t>(lb), kStatusSlave);
                    os << "]";
                    break;
                case ValueType::FLAG_FAULT:
                    os << "fault=[";
                    print_flag_list(os, static_cast<uint16_t>(hb << 8), kFault);
                    std::snprintf(buf, sizeof(buf), "] OEM=0x%02X", static_cast<unsigned>(lb));
                    os << buf;
                    break;
                case ValueType::FLAG_REMOTE:
                    os << "remote=[";
                    print_flag_list(os, static_cast<uint16_t>((hb << 8) | lb), kRemote);
                    os << "]";
                    break;
                case ValueType::FLAG_SLAVE:
                    os << "config=0x"; print_hex_byte(os, hb);
                    std::snprintf(buf, sizeof(buf), " member=%u", static_cast<unsigned>(lb));
                    os << buf;
                    break;
                case ValueType::FLAG_MASTER:
                    os << "config=0x"; print_hex_byte(os, hb);
                    std::snprintf(buf, sizeof(buf), " member=%u", static_cast<unsigned>(lb));
                    os << buf;
                    break;
                case ValueType::VERSION:
                    std::snprintf(buf, sizeof(buf), "type=%u version=%u",
                        static_cast<unsigned>(hb), static_cast<unsigned>(lb));
                    os << buf;
                    break;
            }
        }
    } // namespace ot

    // ============================================================ 3220 OpenTherm
    //
    // Single 5-byte payload regardless of verb.
    //   I/RP : reply (Read-Ack / Write-Ack)
    //   RQ   : request (Read-Data / Write-Data)
    //   W    : seldom used outside test gear

    struct OpenTherm_Inform
        : Payload<std::tuple<ZoneIdx, FixedBytes<1>, FixedBytes<1>, FixedBytes<2>>>
    {
        void describe(std::ostream &os) const
        {
            const uint8_t typeByte = std::get<1>(fields).bytes[0];
            const uint8_t dataId   = std::get<2>(fields).bytes[0];
            const auto   &v        = std::get<3>(fields).bytes;
            const uint8_t mt       = (typeByte >> 4) & 0x07;

            const auto &id = ot::kIds[dataId & 0x7F];
            const char *name = id.name ? id.name : "(unknown ID)";

            char buf[16];
            std::snprintf(buf, sizeof(buf), "[id=0x%02X] ", static_cast<unsigned>(dataId));
            os << buf << name << " | " << ot::msg_type_name(mt);

            // Only the *Ack and Read-Data carry meaningful data; types
            // 2/3/6/7 are envelopes for invalid/unknown id, no value.
            const bool dataValid =
                  mt == 0 /*Read-Data*/
               || mt == 1 /*Write-Data*/
               || mt == 4 /*Read-Ack*/
               || mt == 5 /*Write-Ack*/;
            os << " = ";
            ot::print_value(os, id.vt, v[0], v[1], dataValid && id.name != nullptr);
        }
    };
    struct OpenTherm_Reply   : OpenTherm_Inform {};
    struct OpenTherm_Request : OpenTherm_Inform {};
    struct OpenTherm_Write   : OpenTherm_Inform {};

    void register_opentherm_codecs(CodecRegistry &reg)
    {
        REGISTER_CODEC(reg, 0x3220, Verb::I,  OpenTherm_Inform);
        REGISTER_CODEC(reg, 0x3220, Verb::RP, OpenTherm_Reply);
        REGISTER_CODEC(reg, 0x3220, Verb::RQ, OpenTherm_Request);
        REGISTER_CODEC(reg, 0x3220, Verb::W,  OpenTherm_Write);
    }

} // namespace evohome::codecs
