#pragma once

/**
 * @file ramses_types.h
 * @brief Common RAMSES-II protocol types: verb, code, device address.
 *
 * These types are intentionally minimal and have no dependency on the field /
 * payload / codec layers. They map directly to the wire-level header described
 * in evofw3 (https://github.com/ghoti57/evofw3/blob/master/message.c) and are
 * shared by the frame parser, every codec, and the dispatcher.
 */

#include <array>
#include <cstdint>
#include <iosfwd>
#include <optional>
#include <span>
#include <string_view>

namespace evohome
{

    /// @brief RAMSES-II message verb (encoded in the 2 high bits of the wire
    ///        header byte). The numeric values match the on-air order used by
    ///        evofw3: RQ=0, I=1, W=2, RP=3.
    enum class Verb : uint8_t
    {
        RQ = 0, ///< Request - asking another device for a value
        I  = 1, ///< Inform  - broadcast / unsolicited update
        W  = 2, ///< Write   - actively change a value on another device
        RP = 3, ///< Reply   - response to an RQ
    };

    /// @brief 2-character mnemonic ("RQ"/" I"/" W"/"RP") used by all common
    ///        evohome tooling and the ramses_rf project. The space padding
    ///        keeps log lines aligned with the canonical " I --- ..." format.
    const char *to_string(Verb v);

    /// @brief Try to parse a 2-character mnemonic into a Verb. Accepts both
    ///        "I"/"W" and the padded " I"/" W" variants.
    std::optional<Verb> parse_verb(std::string_view s);

    /// @brief A subset of the well-known RAMSES-II message codes. Values are
    ///        the on-air opcode (big-endian over the 2 opcode bytes). Codes
    ///        only need to live in this enum to get a friendly name in logs;
    ///        any 16-bit code can still be dispatched through the registry as
    ///        a raw uint16_t.
    enum class Code : uint16_t
    {
        SystemSync       = 0x1F09, ///< controller heartbeat / sync window
        ZoneName         = 0x0004, ///< 20-byte UTF-8 zone name
        DeviceInfo       = 0x10E0, ///< device manufacture date, fw, model
        DeviceBattery    = 0x1060, ///< per-device battery state + level
        ZoneTemperature  = 0x30C9, ///< zone temperature(s) - I/RQ/RP
        ZoneSetpoint     = 0x2309, ///< zone setpoint(s)    - I/RQ/RP/W
        RfBind           = 0x1FC9, ///< pairing / advertise bindings
        FanState         = 0x31D9, ///< HVAC fan state (Itho et al.)
        HvacState        = 0x31DA, ///< HVAC extended state (sensors + fan)
        Co2Level         = 0x1298, ///< indoor CO2 sensor reading
        IndoorHumidity   = 0x12A0, ///< indoor RH%/temperature sensor reading
        DhwTemperature   = 0x1260, ///< stored hot water temperature
        OutdoorSensor    = 0x0002, ///< outdoor temperature
        ActuatorState    = 0x3EF0, ///< on/off actuator state
        ActuatorCycle    = 0x3EF1, ///< on/off actuator duty cycle
    };

    /// @brief Friendly name (e.g. "ZoneTemperature") for known codes, or the
    ///        4-digit hex string (e.g. "30C9") for unknown ones. The returned
    ///        pointer is to a string with static lifetime.
    const char *to_string(Code c);

    /// @brief Format a 16-bit code as the canonical "30C9" 4-digit hex used
    ///        in logs, regardless of whether it appears in the Code enum.
    std::array<char, 5> code_to_hex4(uint16_t code);

    // ---------------------------------------------------------------- Address

    /// @brief 3-byte RAMSES device address as it appears on the wire.
    ///
    /// Wire layout (matches evofw3 msg_set_address / msg_get_address):
    ///   addr[0] = ((class << 2) & 0xFC) | ((id >> 16) & 0x03)
    ///   addr[1] = (id >>  8) & 0xFF
    ///   addr[2] = (id      ) & 0xFF
    ///
    /// `class` is the 6-bit device type (CTL=01, BDR=13, TRV=04, FAN=20, ...)
    /// and `id` is an 18-bit unique-per-class serial number. The textual
    /// representation used by evofw3 / ramses_rf is "TT:NNNNNN" (decimal).
    struct DeviceAddr
    {
        uint8_t  cls = 0;       ///< 6-bit device type (0..63). 63 = NUL/all
        uint32_t id  = 0;       ///< 18-bit serial; 0x3FFFF = NUL/broadcast

        /// @brief Decode a 3-byte wire address into class + id.
        static DeviceAddr from_wire(const uint8_t b[3]);

        /// @brief Encode class+id back into a 3-byte wire address.
        void to_wire(uint8_t b[3]) const;

        /// @brief True when this slot encodes the conventional "no device"
        ///        marker (class=63, id=0x3FFFF). Printed as "--:------".
        bool is_null() const { return cls == 63 && id == 0x3FFFF; }

        /// @brief True when this slot encodes the broadcast/all marker
        ///        (class=63, id=0x3FFFE). Printed as "63:262142".
        bool is_broadcast() const { return cls == 63 && id == 0x3FFFE; }

        /// @brief Print as "TT:NNNNNN" (decimal class/id, both zero-padded)
        ///        or "--:------" when is_null().
        void describe(std::ostream &os) const;

        bool operator==(const DeviceAddr &o) const { return cls == o.cls && id == o.id; }
    };

    /// @brief Friendly mnemonic for a device class (e.g. 01 -> "CTL"). Returns
    ///        the 2-digit decimal as a string for unknown classes.
    const char *device_class_name(uint8_t cls);

} // namespace evohome
