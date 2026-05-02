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
 *   3. (Optional) Add the friendly name to the Code enum in ramses_types.h.
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

namespace evohome::codecs
{
    using namespace evohome;
    using namespace evohome::fields;

    // ============================================================ 30C9 ZoneTemperature
    //
    // Spec (ramses_rf):
    //   I  : ^((?:0[0-9A-F])(?:[0-9A-F]{4}))+$  -> 1+ tuples of (zone_idx, temp)
    //   RQ : ^00$                               -> always exactly "00"
    //   RP : ^0[0-9A-F]{5}$                     -> exactly (zone_idx, temp)

    struct ZoneTemperature_Inform
        : Payload<std::tuple<Repeated<ZoneIdx, TempCenti>>> {};
    struct ZoneTemperature_Request
        : Payload<std::tuple<ZoneIdx>> {};
    struct ZoneTemperature_Reply
        : Payload<std::tuple<ZoneIdx, TempCenti>> {};

    // ============================================================ 2309 ZoneSetpoint
    //
    //   I  : array of (zone_idx, temp)  - same shape as 30C9 I
    //   RQ : "00" (or zone_idx for some controllers)
    //   RP : (zone_idx, temp)
    //   W  : (zone_idx, temp)

    struct ZoneSetpoint_Inform
        : Payload<std::tuple<Repeated<ZoneIdx, TempCenti>>> {};
    struct ZoneSetpoint_Request
        : Payload<std::tuple<ZoneIdx>> {};
    struct ZoneSetpoint_Reply
        : Payload<std::tuple<ZoneIdx, TempCenti>> {};
    struct ZoneSetpoint_Write
        : Payload<std::tuple<ZoneIdx, TempCenti>> {};

    // ============================================================ 1FC9 RfBind
    //
    // Each tuple is (zone_or_domain_id, code, device_id). Bindings are
    // greedy / variable-length.
    //   I, RP, W : Repeated<ZoneIdx, OpcodeBE16, DeviceId3>
    //   RQ       : just an idx byte (often 00)

    struct RfBind_Inform
        : Payload<std::tuple<Repeated<ZoneIdx, OpcodeBE16, DeviceId3>>> {};
    struct RfBind_Reply
        : Payload<std::tuple<Repeated<ZoneIdx, OpcodeBE16, DeviceId3>>> {};
    struct RfBind_Write
        : Payload<std::tuple<Repeated<ZoneIdx, OpcodeBE16, DeviceId3>>> {};
    struct RfBind_Request
        : Payload<std::tuple<ZoneIdx>> {};

    // ============================================================ 31D9 FanState
    //
    //   I : <fan_mode:1B> <flags/percent:2B> ... (3 mandatory bytes,
    //       sometimes followed by 14 bytes of unknown extension fields)

    struct FanState_Inform
        : Payload<std::tuple<ZoneIdx, FixedBytes<2>, Optional<HexBlob>>> {};

    // ============================================================ 31DA HvacState
    //
    // Long mixed payload (typically 30 bytes). For now we expose:
    //   - zone_idx (always 00 for whole-house)
    //   - co2 (2B BE)
    //   - rh  (1B)
    //   - rest as opaque hex blob (29 bytes typical)
    //
    // The exact field layout has many variants per manufacturer; refining
    // this codec is one of the first follow-up tasks once we have real
    // captures from the user's HVAC system.

    struct HvacState_Inform
        : Payload<std::tuple<ZoneIdx, FixedBytes<3>, Optional<HexBlob>>> {};

    // ============================================================ 1298 Co2Level
    //   I/RP : (zone_idx, co2_ppm)

    struct Co2Level_Inform
        : Payload<std::tuple<ZoneIdx, Co2Ppm>> {};
    struct Co2Level_Reply
        : Payload<std::tuple<ZoneIdx, Co2Ppm>> {};

    // ============================================================ 12A0 IndoorHumidity
    //
    //   I : (zone_idx, humidity, temp_centi, temp_centi)  - some sensors
    //       report (rh, indoor_temp, dewpoint)
    //   RP: same as I

    struct IndoorHumidity_Inform
        : Payload<std::tuple<ZoneIdx, Humidity, TempCenti, Optional<TempCenti>>> {};

    // ============================================================ 1060 DeviceBattery
    //   I : (zone_idx, percent, low_flag)

    struct DeviceBattery_Inform
        : Payload<std::tuple<ZoneIdx, Battery>> {};

    // ============================================================ 10E0 DeviceInfo
    //
    // First 2 bytes are the schema/version, rest is a manufacturer-specific
    // device-info blob (often includes a date and a 20-byte ASCII name).
    // We keep it as a permissive hex blob until somebody needs more.

    struct DeviceInfo_Inform
        : Payload<std::tuple<HexBlob>> {};
    struct DeviceInfo_Reply
        : Payload<std::tuple<HexBlob>> {};

    // ============================================================ 1F09 SystemSync
    //   I/RP : "FF" + 2-byte countdown
    //   RQ   : "00"

    struct SystemSync_Inform
        : Payload<std::tuple<ZoneIdx, FixedBytes<2>>> {};
    struct SystemSync_Reply
        : Payload<std::tuple<ZoneIdx, FixedBytes<2>>> {};
    struct SystemSync_Request
        : Payload<std::tuple<ZoneIdx>> {};

    // ============================================================ Registration

    // Forward declare so EvohomeRamses can call it without pulling in this
    // file directly. Defined in ramses_codec.h as `register_all_codecs`.

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
    }
} // namespace evohome
