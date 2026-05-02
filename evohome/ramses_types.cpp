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

} // namespace evohome
