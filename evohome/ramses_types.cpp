#include "ramses_types.h"

#include <cstdio>
#include <cstring>
#include <ostream>

namespace evohome
{

    const char *to_string(Verb v)
    {
        switch (v)
        {
        case Verb::RQ: return "RQ";
        case Verb::I:  return " I";
        case Verb::W:  return " W";
        case Verb::RP: return "RP";
        }
        return "??";
    }

    const char *verb_tag(Verb v)
    {
        switch (v)
        {
        case Verb::I:  return "I"; // Inform / broadcast
        case Verb::RQ: return "Q"; // Question / request
        case Verb::RP: return "A"; // Answer / reply
        case Verb::W:  return "W"; // Write / set
        }
        return "?";
    }

    std::optional<Verb> parse_verb(std::string_view s)
    {
        if (s.size() == 2 && s[0] == ' ') s.remove_prefix(1);
        if (s == "RQ") return Verb::RQ;
        if (s == "I")  return Verb::I;
        if (s == "W")  return Verb::W;
        if (s == "RP") return Verb::RP;
        return std::nullopt;
    }

    const char *to_string(Code c)
    {
        switch (c)
        {
        case Code::SystemSync:      return "SystemSync";
        case Code::ZoneName:        return "ZoneName";
        case Code::DeviceInfo:      return "DeviceInfo";
        case Code::DeviceBattery:   return "DeviceBattery";
        case Code::ZoneTemperature: return "ZoneTemperature";
        case Code::ZoneSetpoint:    return "ZoneSetpoint";
        case Code::RfBind:          return "RfBind";
        case Code::FanState:        return "FanState";
        case Code::HvacState:       return "HvacState";
        case Code::Co2Level:        return "Co2Level";
        case Code::IndoorHumidity:  return "IndoorHumidity";
        case Code::DhwTemperature:  return "DhwTemperature";
        case Code::OutdoorSensor:   return "OutdoorSensor";
        case Code::ActuatorState:   return "ActuatorState";
        case Code::ActuatorCycle:   return "ActuatorCycle";
        }
        return "Unknown";
    }

    std::array<char, 5> code_to_hex4(uint16_t code)
    {
        std::array<char, 5> buf{};
        std::snprintf(buf.data(), buf.size(), "%04X", code);
        return buf;
    }

    // Long-form English names. Sources: ramses_rf RAMSES_CODES (master list)
    // plus assorted reverse-engineering forum posts. Codes that show up in
    // real-world traffic but have unclear semantics get a best-effort name
    // ("OEM Status") rather than a literal hex passthrough so the log line
    // still reads as a sentence. Codes the user has actually observed live
    // (1F09, 1FD4, 2349, 30C9, 31DA, 3EF0, ...) are prioritised.
    const char *code_long_name(uint16_t code)
    {
        switch (code)
        {
        // System / housekeeping
        case 0x0001: return "Rf Test";
        case 0x0002: return "Outdoor Sensor";
        case 0x0004: return "Zone Name";
        case 0x0005: return "System Zones";
        case 0x0006: return "Schedule Sync";
        case 0x0008: return "Relay Demand";
        case 0x0009: return "Relay Failsafe";
        case 0x000A: return "Zone Config";
        case 0x000C: return "Zone Devices";
        case 0x000E: return "Unknown 000E";
        case 0x0016: return "Rf Check";
        case 0x0100: return "Language";
        case 0x01D0: return "Bind Request";
        case 0x01E9: return "Bind Confirm";
        case 0x01FF: return "OpenTherm Read";
        case 0x0404: return "Schedule Fragment";
        case 0x0418: return "Fault Log";
        case 0x1030: return "Zone Mixing";
        case 0x1060: return "Battery State";
        case 0x10A0: return "DHW Params";
        case 0x10E0: return "Device Info";
        case 0x10E1: return "Device ID";
        case 0x1100: return "TPI Params";
        case 0x1260: return "DHW Temperature";
        case 0x1280: return "Outdoor Humidity";
        case 0x1290: return "Outdoor Temperature";
        case 0x1298: return "CO2 Level";
        case 0x12A0: return "Indoor Humidity";
        case 0x12B0: return "Window State";
        case 0x12C0: return "Display Brightness";
        case 0x12C8: return "Air Quality";
        case 0x1300: return "CH Pressure";
        case 0x1F09: return "System Sync";
        case 0x1F41: return "DHW Mode";
        case 0x1FC9: return "RF Bind";
        case 0x1FCA: return "RF Unbind";
        case 0x1FD0: return "Cooling Mode";
        case 0x1FD4: return "Boiler Heart-beat";
        case 0x2249: return "Setpoint Now-Next";
        case 0x22C9: return "Underfloor Setpoint";
        case 0x22D0: return "System Override";
        case 0x22D9: return "Boiler Setpoint";
        case 0x22F1: return "HVAC Speed";
        case 0x22F3: return "HVAC Boost";
        case 0x2309: return "Zone Setpoint";
        case 0x2349: return "Zone Setpoint Override";
        case 0x2389: return "Average Temperature";
        case 0x2400: return "OEM Code 2400";
        case 0x2401: return "OEM Code 2401";
        case 0x2410: return "OEM Code 2410";
        case 0x2420: return "OEM Code 2420";
        case 0x2D49: return "OEM Code 2D49";
        case 0x2E04: return "System Mode";
        case 0x2E10: return "Presence Detected";
        case 0x30C9: return "Zone Temperature";
        case 0x3110: return "Hometronic CO2";
        case 0x3120: return "HVAC Status";
        case 0x313E: return "Schedule Last-Updated";
        case 0x313F: return "Date / Time";
        case 0x3150: return "Heat Demand";
        case 0x31D9: return "Fan State";
        case 0x31DA: return "HVAC State";
        case 0x31E0: return "Fan Switch";
        case 0x3200: return "Boiler Output Temp";
        case 0x3210: return "Boiler Return Temp";
        case 0x3220: return "OpenTherm Message";
        case 0x3221: return "Boiler Setpoint Override";
        case 0x3223: return "DHW Setpoint Override";
        case 0x3B00: return "Actuator Sync";
        case 0x3EF0: return "Boiler Status";
        case 0x3EF1: return "Actuator Cycle";
        }
        // Fallback: 4-char hex in a small thread-local ring buffer so the
        // returned pointer stays valid for the lifetime of the log call.
        static thread_local char ring[8][16];
        static thread_local unsigned next = 0;
        char *out = ring[next++ & 7];
        std::snprintf(out, 16, "Unknown %04X", static_cast<unsigned>(code));
        return out;
    }

    // ------------------------------------------------------------ DeviceAddr

    DeviceAddr DeviceAddr::from_wire(const uint8_t b[3])
    {
        DeviceAddr a;
        a.cls = (b[0] & 0xFC) >> 2;
        a.id  = (static_cast<uint32_t>(b[0] & 0x03) << 16) |
                (static_cast<uint32_t>(b[1])       << 8 ) |
                 static_cast<uint32_t>(b[2]);
        return a;
    }

    void DeviceAddr::to_wire(uint8_t b[3]) const
    {
        b[0] = static_cast<uint8_t>(((cls << 2) & 0xFC) | ((id >> 16) & 0x03));
        b[1] = static_cast<uint8_t>((id >> 8) & 0xFF);
        b[2] = static_cast<uint8_t>(id & 0xFF);
    }

    void DeviceAddr::describe(std::ostream &os) const
    {
        char buf[16];
        if (is_null())
        {
            os << "--:------";
        }
        else
        {
            std::snprintf(buf, sizeof(buf), "%02u:%06lu",
                          static_cast<unsigned>(cls),
                          static_cast<unsigned long>(id));
            os << buf;
        }
    }

    const char *device_class_name(uint8_t cls)
    {
        // Subset taken from ramses_rf DEV_TYPE_MAP. Codes not in the map are
        // returned as a 2-digit decimal string (rotated in a small static
        // buffer so we do not need a separate allocation - good enough for
        // log output where the caller copies the string immediately).
        switch (cls)
        {
        case  1: return "CTL"; // controller
        case  2: return "UFC"; // underfloor heating controller
        case  3: return "STA"; // room thermostat
        case  4: return "TRV"; // radiator valve
        case  7: return "DHW"; // DHW sensor
        case 10: return "OTB"; // OpenTherm bridge
        case 12: return "DTS"; // sundial / programmer
        case 13: return "BDR"; // BDR91 actuator
        case 17: return "OUT"; // outdoor sensor
        case 18: return "HGI"; // gateway / sniffer
        case 20: return "FAN"; // HVAC fan unit
        case 22: return "DT2"; // dual-channel DTS
        case 23: return "PRG"; // programmer
        case 24: return "PIR"; // HVAC presence / motion sensor (Itho et al.)
        case 30: return "RFG"; // remote / HRU
        case 32: return "HUM"; // HVAC humidity / VOC sensor
        case 34: return "T87"; // T87RF round thermostat
        case 37: return "CO2"; // HVAC CO2 sensor / co2 controller
        case 39: return "SW2"; // 2-button switch
        case 42: return "SW4"; // 4-button switch
        case 49: return "REM"; // generic HVAC remote
        case 63: return "NUL"; // null / broadcast
        default: {
            static thread_local char ring[8][4];
            static thread_local unsigned next = 0;
            char *out = ring[next++ & 7];
            std::snprintf(out, 4, "%02u", static_cast<unsigned>(cls & 0x3F));
            return out;
        }
        }
    }

    const char *device_class_long_name(uint8_t cls)
    {
        // Long-form English. These overlap with device_class_name() short
        // codes; both come from ramses_rf DEV_TYPE_MAP plus the HVAC
        // sniffer-tooling notes. "OpenTherm Bridge" is the OTB / R8810,
        // "RFG" is the RFG100 mobile gateway, etc.
        switch (cls)
        {
        case  1: return "Controller";
        case  2: return "UFH Controller";
        case  3: return "Room Thermostat";
        case  4: return "Radiator Valve";
        case  7: return "DHW Sensor";
        case 10: return "OpenTherm Bridge";
        case 12: return "Sundial Programmer";
        case 13: return "Boiler Relay";
        case 17: return "Outdoor Sensor";
        case 18: return "Gateway / HGI";
        case 20: return "HVAC Fan Unit";
        case 22: return "DT2 Programmer";
        case 23: return "Programmer";
        case 24: return "HVAC PIR Sensor";
        case 30: return "RFG100 Gateway";
        case 32: return "HVAC Humidity Sensor";
        case 34: return "T87RF Round Stat";
        case 37: return "HVAC CO2 Sensor";
        case 39: return "2-Button Switch";
        case 42: return "4-Button Switch";
        case 49: return "HVAC Remote";
        case 63: return "(broadcast)";
        default: {
            static thread_local char ring[8][16];
            static thread_local unsigned next = 0;
            char *out = ring[next++ & 7];
            std::snprintf(out, 16, "Class %02u", static_cast<unsigned>(cls & 0x3F));
            return out;
        }
        }
    }

} // namespace evohome
