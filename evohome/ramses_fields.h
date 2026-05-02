#pragma once

/**
 * @file ramses_fields.h
 * @brief Reusable field types for RAMSES-II message payloads.
 *
 * Every Field type in this header conforms to the same trivial protocol:
 *
 *   struct Field {
 *       value_type value;                             // (or no member if pure marker)
 *       static std::optional<Field> deserialize(const uint8_t*& cur,
 *                                               const uint8_t* end);
 *       void serialize(uint8_t*& cur, uint8_t* end) const;
 *       void describe(std::ostream& os) const;
 *       static const char* name();
 *   };
 *
 * `cur` is advanced past consumed bytes; on failure cur is left untouched.
 * Fields never read or write past `end`; if there are not enough bytes
 * remaining they return std::nullopt (deserialize) or write nothing
 * (serialize). This is the only contract codec authors need to know about.
 */

#include <array>
#include <cstdint>
#include <iosfwd>
#include <optional>

namespace evohome::fields
{

    // ============================================================ Helpers
    //
    // Internal helpers for fields that consume a fixed number of bytes. These
    // keep the per-field deserialize() implementations short and error-safe.

    inline bool can_read(const uint8_t *cur, const uint8_t *end, size_t n)
    {
        return cur != nullptr && end != nullptr && cur + n <= end;
    }

    inline bool can_write(const uint8_t *cur, const uint8_t *end, size_t n)
    {
        return cur != nullptr && end != nullptr && cur + n <= end;
    }

    // ============================================================ ZoneIdx
    //
    // A 1-byte zone index. Values 0x00..0x0B are real zones (0-indexed -
    // displayed as 1-indexed in HA / evohome). Values 0xF8..0xFF are
    // "domain ids":
    //   0xF8/0xF9 - DHW heat-demand domain
    //   0xFA      - DHW
    //   0xFC      - heating (boiler control)
    //   0xFF      - all/any domain
    //
    // Higher-level codecs that only accept real zone ids should validate
    // against the value field before using it; this type accepts everything
    // so it is reusable for any zone-or-domain-prefixed payload.

    struct ZoneIdx
    {
        uint8_t value = 0;

        static constexpr const char *name() { return "zone_idx"; }
        static std::optional<ZoneIdx> deserialize(const uint8_t *&cur, const uint8_t *end);
        void serialize(uint8_t *&cur, uint8_t *end) const;
        void describe(std::ostream &os) const;

        bool is_domain() const { return value >= 0xF8; }
    };

    // ============================================================ DomainId
    //
    // Strict-domain variant of ZoneIdx: only accepts 0xF8..0xFF on parse.
    // Useful for codecs whose schema explicitly reserves the slot for a
    // domain id (e.g. boiler heat-demand on FC).

    struct DomainId
    {
        uint8_t value = 0xFF;

        static constexpr const char *name() { return "domain_id"; }
        static std::optional<DomainId> deserialize(const uint8_t *&cur, const uint8_t *end);
        void serialize(uint8_t *&cur, uint8_t *end) const;
        void describe(std::ostream &os) const;
    };

    // ============================================================ TempCenti
    //
    // Big-endian signed 16-bit temperature in centi-degrees Celsius.
    //   value 0x7FFF -> "no value" / sensor not present (printed as "--")
    //   value 0x8000 -> alternative unset marker (printed as "--")
    //   value 0x7EFF -> often used as "not implemented"     (printed as "--")
    //
    // 0x07D0 = 2000 -> 20.00 C
    // 0x0428 = 1064 -> 10.64 C
    // 0xFE0C = -500 -> -5.00 C

    struct TempCenti
    {
        int16_t raw = 0x7FFF;

        static constexpr const char *name() { return "temp"; }
        static std::optional<TempCenti> deserialize(const uint8_t *&cur, const uint8_t *end);
        void serialize(uint8_t *&cur, uint8_t *end) const;
        void describe(std::ostream &os) const;

        bool is_valid() const { return raw != 0x7FFF && raw != static_cast<int16_t>(0x8000); }
        float celsius() const { return raw * 0.01f; }
    };

    // ============================================================ Percent
    //
    // 1-byte percentage, scaled 0..200 -> 0..100 % (i.e. resolution 0.5%).
    //   0xFF -> "no value" (printed as "--")
    //
    // Used for setpoint demand, heat demand, valve position, fan speeds.

    struct Percent
    {
        uint8_t raw = 0xFF;

        static constexpr const char *name() { return "pct"; }
        static std::optional<Percent> deserialize(const uint8_t *&cur, const uint8_t *end);
        void serialize(uint8_t *&cur, uint8_t *end) const;
        void describe(std::ostream &os) const;

        bool is_valid() const { return raw != 0xFF; }
        float percent() const { return raw * 0.5f; }
    };

    // ============================================================ Co2Ppm
    //
    // Big-endian unsigned 16-bit ppm reading, 0..5000.

    struct Co2Ppm
    {
        uint16_t raw = 0x7FFF;

        static constexpr const char *name() { return "co2_ppm"; }
        static std::optional<Co2Ppm> deserialize(const uint8_t *&cur, const uint8_t *end);
        void serialize(uint8_t *&cur, uint8_t *end) const;
        void describe(std::ostream &os) const;

        bool is_valid() const { return raw != 0x7FFF; }
    };

    // ============================================================ Humidity
    //
    // Single-byte indoor relative humidity, 0..100 %.

    struct Humidity
    {
        uint8_t raw = 0x7F;

        static constexpr const char *name() { return "rh"; }
        static std::optional<Humidity> deserialize(const uint8_t *&cur, const uint8_t *end);
        void serialize(uint8_t *&cur, uint8_t *end) const;
        void describe(std::ostream &os) const;

        bool is_valid() const { return raw != 0xFF && raw <= 100; }
    };

    // ============================================================ Battery
    //
    // 1060-style battery descriptor: 1 percentage byte (0..200 -> 0..100%)
    // followed by a 1-byte "low battery" flag (0x01 = OK, 0x00 = LOW).

    struct Battery
    {
        Percent percent;
        uint8_t low_flag = 0x01;

        static constexpr const char *name() { return "battery"; }
        static std::optional<Battery> deserialize(const uint8_t *&cur, const uint8_t *end);
        void serialize(uint8_t *&cur, uint8_t *end) const;
        void describe(std::ostream &os) const;
    };

    // ============================================================ OpcodeBE16
    //
    // Big-endian 16-bit opcode field. Used inside RfBind tuples (each binding
    // entry references the code it is binding for).

    struct OpcodeBE16
    {
        uint16_t value = 0;

        static constexpr const char *name() { return "opcode"; }
        static std::optional<OpcodeBE16> deserialize(const uint8_t *&cur, const uint8_t *end);
        void serialize(uint8_t *&cur, uint8_t *end) const;
        void describe(std::ostream &os) const;
    };

    // ============================================================ DeviceId3
    //
    // 3-byte device address as it appears inline in payloads (e.g. inside
    // 1FC9 binding tuples). Not the same as the per-frame address slots,
    // which live in the frame header rather than the payload.

    struct DeviceId3
    {
        std::array<uint8_t, 3> bytes{};

        static constexpr const char *name() { return "device_id"; }
        static std::optional<DeviceId3> deserialize(const uint8_t *&cur, const uint8_t *end);
        void serialize(uint8_t *&cur, uint8_t *end) const;
        void describe(std::ostream &os) const;
    };

    // ============================================================ FixedBytes
    //
    // Fallback field for unknown/unparsed segments: consumes exactly N bytes
    // from the buffer and prints them as hex. Use as a placeholder while the
    // schema is being reverse-engineered.

    template<size_t N>
    struct FixedBytes
    {
        std::array<uint8_t, N> bytes{};

        static constexpr const char *name() { return "raw"; }
        static std::optional<FixedBytes> deserialize(const uint8_t *&cur, const uint8_t *end);
        void serialize(uint8_t *&cur, uint8_t *end) const;
        void describe(std::ostream &os) const;
    };

    // Common sizes are pre-instantiated in ramses_fields.cpp.
    extern template struct FixedBytes<1>;
    extern template struct FixedBytes<2>;
    extern template struct FixedBytes<3>;
    extern template struct FixedBytes<4>;
    extern template struct FixedBytes<5>;
    extern template struct FixedBytes<6>;
    extern template struct FixedBytes<7>;
    extern template struct FixedBytes<8>;

    // ============================================================ HexBlob
    //
    // Greedy "rest of the payload" field: consumes everything from cur up
    // to end and prints it as a hex string. Useful as a permissive fallback
    // for codecs whose schema we do not understand yet, or for debugging
    // unrecognised codes via the universal fallback codec.

    struct HexBlob
    {
        std::array<uint8_t, 64> bytes{};
        uint8_t length = 0;

        static constexpr const char *name() { return "blob"; }
        static std::optional<HexBlob> deserialize(const uint8_t *&cur, const uint8_t *end);
        void serialize(uint8_t *&cur, uint8_t *end) const;
        void describe(std::ostream &os) const;
    };

} // namespace evohome::fields
