/**
 * @file codecs_zoneconfig.cpp
 * @brief Codecs for zone metadata, schedules and topology.
 *
 * Schedules in particular are reassembled from many fragments by the
 * controller; this codec only decodes one *fragment header* at a time
 * and surfaces the fragment payload as a hex blob. A higher-level state
 * machine (not implemented here) would stitch them together.
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

    // ============================================================ 0004 ZoneName
    //
    //   I/RP/W : zone(1) + 1B (always 00) + 20B UTF-8 zone name
    //   RQ     : zone(1) + 1B (00)
    //
    // 7F * 20 == "no name set". Names are NUL-padded to 20 bytes.

    struct ZoneName_Reply
        : Payload<std::tuple<ZoneIdx, FixedBytes<1>, Optional<HexBlob>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            const auto &blob = std::get<2>(fields);
            os << " : ";
            if (!blob.value || blob.value->length == 0) { os << "(no name)"; return; }
            // 20-byte UTF-8 name; treat 7F-only as "no name"
            bool all_7f = true;
            for (uint8_t i = 0; i < blob.value->length; ++i)
                if (blob.value->bytes[i] != 0x7F) { all_7f = false; break; }
            if (all_7f) { os << "(unset)"; return; }
            os << "\"";
            for (uint8_t i = 0; i < blob.value->length; ++i) {
                const uint8_t c = blob.value->bytes[i];
                if (c == 0x00 || c == 0x7F) break;
                if (c >= 0x20 && c < 0x7F) os.put(static_cast<char>(c));
                else { char buf[8]; std::snprintf(buf, sizeof(buf), "\\x%02X", c); os << buf; }
            }
            os << "\"";
        }
    };
    struct ZoneName_Inform : ZoneName_Reply {};
    struct ZoneName_Write  : ZoneName_Reply {};
    struct ZoneName_Request
        : Payload<std::tuple<ZoneIdx, FixedBytes<1>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
        }
    };

    // ============================================================ 0005 SystemZones
    //
    //   I/RP : (00 + zone_type(1) + zone_mask_lo(1) + zone_mask_hi(1)){1..3}
    //   RQ   : 00 + zone_type(1)
    //
    // zone_mask is two LSB-first bitmaps; a "1" = zone is present in
    // the system. zone_type values come from DEV_ROLE_MAP (e.g. 0A=TRV,
    // 0F=DHW, 11=ELECTRIC, 22=MIXING).

    struct SystemZones_Inform
        : Payload<std::tuple<Repeated<FixedBytes<1>, FixedBytes<1>, FixedBytes<2>>>>
    {
        void describe(std::ostream &os) const
        {
            const auto &arr = std::get<0>(fields);
            for (uint8_t i = 0; i < arr.count; ++i)
            {
                if (i) os << " | ";
                const uint8_t ztype = std::get<1>(arr.rows[i]).bytes[0];
                const auto &mask = std::get<2>(arr.rows[i]).bytes;
                char buf[24];
                std::snprintf(buf, sizeof(buf), "type=0x%02X zones=", static_cast<unsigned>(ztype));
                os << buf;
                bool first = true;
                const uint16_t bm = static_cast<uint16_t>(mask[0]) | (static_cast<uint16_t>(mask[1]) << 8);
                for (int z = 0; z < 16; ++z) {
                    if (bm & (1u << z)) {
                        if (!first) os << ",";
                        os << z;
                        first = false;
                    }
                }
                if (first) os << "(none)";
            }
        }
    };
    struct SystemZones_Reply : SystemZones_Inform {};

    // ============================================================ 0006 ScheduleVersion
    //
    //   RQ : 00
    //   RP : 0005 + 2B BE counter (FFFF == "no schedule")
    //
    // Counter increments every time *any* zone schedule (or DHW) changes;
    // RFG100 polls it every minute and re-requests fragments on change.

    struct ScheduleVersion_Reply
        : Payload<std::tuple<FixedBytes<1>, FixedBytes<1>, FixedBytes<2>>>
    {
        void describe(std::ostream &os) const
        {
            const auto &b = std::get<2>(fields).bytes;
            if (b[0] == 0xFF && b[1] == 0xFF) { os << "no schedule"; return; }
            const unsigned ctr = (b[0] << 8) | b[1];
            char buf[24];
            std::snprintf(buf, sizeof(buf), "change_counter=%u", ctr);
            os << buf;
        }
    };

    // ============================================================ 000C ZoneDevices
    //
    //   I/RP : zone(1) + dev_role(1) + array of (1B + 1B + 3B device_id)
    //
    // Tells you which physical devices are bound to a logical zone /
    // role (e.g. all TRVs in zone 3, or all relays for the DHW domain).

    struct ZoneDevices_Reply
        : Payload<std::tuple<ZoneIdx, FixedBytes<1>, HexBlob>>
    {
        // 000C entries are either 5- or 6-byte rows depending on packet
        // length (long-form repeats the zone-idx prefix per entry, short-
        // form does not). The detection logic in ramses_rf is non-trivial
        // and we don't need it for a friendly description, so we just
        // surface the role and dump the raw entries as hex; users can
        // grep for `000C` and inspect manually if they need device ids.
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            const uint8_t role = std::get<1>(fields).bytes[0];
            char buf[24];
            std::snprintf(buf, sizeof(buf), " role=0x%02X raw=", static_cast<unsigned>(role));
            os << buf;
            const auto &blob = std::get<2>(fields);
            print_hex_buffer(os, blob.bytes.data(), blob.length);
        }
    };
    struct ZoneDevices_Inform : ZoneDevices_Reply {};
    struct ZoneDevices_Request
        : Payload<std::tuple<ZoneIdx, FixedBytes<1>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            const uint8_t role = std::get<1>(fields).bytes[0];
            char buf[24];
            std::snprintf(buf, sizeof(buf), " role=0x%02X", static_cast<unsigned>(role));
            os << buf;
        }
    };

    // ============================================================ 0404 ScheduleFragment
    //
    //   RQ : zone(1) + 200008(3B/heat 230008/dhw) + 1B "00" + frag_idx(1) + total_or_00(1)
    //   I  : same shell, no payload
    //   RP/W : same + frag_data(N B)

    struct ScheduleFragment_Reply
        : Payload<std::tuple<ZoneIdx, FixedBytes<3>, FixedBytes<1>, FixedBytes<1>,
                              FixedBytes<1>, Optional<HexBlob>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            const auto &kind = std::get<1>(fields).bytes;
            const uint8_t fragLen   = std::get<2>(fields).bytes[0];
            const uint8_t fragIdx   = std::get<3>(fields).bytes[0];
            const uint8_t fragTotal = std::get<4>(fields).bytes[0];
            char buf[40];
            std::snprintf(buf, sizeof(buf), " %s frag %u/%u (%uB)",
                (kind[0] == 0x23 ? "DHW" : "zone"),
                static_cast<unsigned>(fragIdx),
                static_cast<unsigned>(fragTotal),
                static_cast<unsigned>(fragLen));
            os << buf;
            if (std::get<5>(fields).value && std::get<5>(fields).value->length) {
                os << " data=";
                print_hex_buffer(os, std::get<5>(fields).value->bytes.data(),
                    std::get<5>(fields).value->length);
            }
        }
    };
    struct ScheduleFragment_Inform  : ScheduleFragment_Reply {};
    struct ScheduleFragment_Write   : ScheduleFragment_Reply {};
    struct ScheduleFragment_Request : ScheduleFragment_Reply {};

    // ============================================================ 313E ScheduleLastUpdated
    //
    //   I : zone(1) + 4B BE minutes-since-now? + 1B seconds + 5B fixed (003C800000)

    struct ScheduleLastUpdated_Inform
        : Payload<std::tuple<ZoneIdx, FixedBytes<4>, FixedBytes<1>, FixedBytes<5>>>
    {
        void describe(std::ostream &os) const
        {
            print_zone_or_domain(os, std::get<0>(fields).value);
            const auto &m = std::get<1>(fields).bytes;
            const uint8_t s = std::get<2>(fields).bytes[0];
            const uint32_t mins = (static_cast<uint32_t>(m[0]) << 24)
                                | (static_cast<uint32_t>(m[1]) << 16)
                                | (static_cast<uint32_t>(m[2]) << 8)
                                |  static_cast<uint32_t>(m[3]);
            char buf[40];
            std::snprintf(buf, sizeof(buf), " : updated %lu min %u s ago",
                static_cast<unsigned long>(mins), static_cast<unsigned>(s));
            os << buf;
        }
    };

    // ----------------------------------------------------------------
    // Registration
    // ----------------------------------------------------------------

    void register_zoneconfig_codecs(CodecRegistry &reg)
    {
        REGISTER_CODEC(reg, Code::ZoneName, Verb::I,  ZoneName_Inform);
        REGISTER_CODEC(reg, Code::ZoneName, Verb::RP, ZoneName_Reply);
        REGISTER_CODEC(reg, Code::ZoneName, Verb::W,  ZoneName_Write);
        REGISTER_CODEC(reg, Code::ZoneName, Verb::RQ, ZoneName_Request);

        REGISTER_CODEC(reg, 0x0005, Verb::I,  SystemZones_Inform);
        REGISTER_CODEC(reg, 0x0005, Verb::RP, SystemZones_Reply);

        REGISTER_CODEC(reg, 0x0006, Verb::RP, ScheduleVersion_Reply);

        REGISTER_CODEC(reg, 0x000C, Verb::I,  ZoneDevices_Inform);
        REGISTER_CODEC(reg, 0x000C, Verb::RP, ZoneDevices_Reply);
        REGISTER_CODEC(reg, 0x000C, Verb::RQ, ZoneDevices_Request);

        REGISTER_CODEC(reg, 0x0404, Verb::I,  ScheduleFragment_Inform);
        REGISTER_CODEC(reg, 0x0404, Verb::RP, ScheduleFragment_Reply);
        REGISTER_CODEC(reg, 0x0404, Verb::W,  ScheduleFragment_Write);
        REGISTER_CODEC(reg, 0x0404, Verb::RQ, ScheduleFragment_Request);

        REGISTER_CODEC(reg, 0x313E, Verb::I, ScheduleLastUpdated_Inform);
    }

} // namespace evohome::codecs
