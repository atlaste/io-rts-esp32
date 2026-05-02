#pragma once

/**
 * @file ramses_payload.h
 * @brief Header-only Payload<tuple<Fields...>> framework.
 *
 * Codec authors declare a payload schema by listing field types in a tuple:
 *
 *   struct ZoneTemperature_Reply : Payload<std::tuple<ZoneIdx, TempCenti>> {};
 *
 * That single line gives them serialize / deserialize / describe for free.
 *
 * Variable / repeated structures (e.g. an array of (zone_idx, temp) pairs)
 * are expressed by wrapping fields in Repeated<> as the LAST entry of the
 * tuple - Repeated greedily consumes all remaining bytes:
 *
 *   struct ZoneTemp_Inform_Array
 *       : Payload<std::tuple<Repeated<ZoneIdx, TempCenti>>> {};
 *
 * Optional<F> wraps a single field, accepting "absent if no bytes left".
 *
 * The whole machinery is implemented as constexpr + std::tuple folds; there
 * is exactly one virtual call (the codec adapter dispatched from the registry)
 * and zero allocation in the hot RX path.
 */

#include "ramses_fields.h"

#include <cstdint>
#include <iosfwd>
#include <optional>
#include <ostream>
#include <span>
#include <tuple>
#include <utility>

namespace evohome
{

    // ============================================================ Repeated
    //
    // A field that consumes all remaining bytes by repeatedly deserializing
    // the inner tuple of fields. Used as a "tail field" inside Payload.

    template<typename... Inner>
    struct Repeated
    {
        // Each "row" is a tuple of the inner field instances.
        using row_t = std::tuple<Inner...>;
        // Hard cap to avoid unbounded heap usage on a malformed payload. The
        // longest legitimate RAMSES-II payload is 64 bytes, so even with
        // single-byte rows we never exceed 64 entries.
        static constexpr size_t kMaxRows = 16;

        std::array<row_t, kMaxRows> rows{};
        uint8_t count = 0;

        static constexpr const char *name() { return "repeated"; }

        static std::optional<Repeated> deserialize(const uint8_t *&cur, const uint8_t *end);
        void serialize(uint8_t *&cur, uint8_t *end) const;
        void describe(std::ostream &os) const;
    };

    // ============================================================ Optional
    //
    // Wraps a single field; succeeds with std::nullopt if there are no bytes
    // left, otherwise delegates to the inner field. Useful for codecs whose
    // tail field is sometimes omitted.

    template<typename F>
    struct Optional
    {
        std::optional<F> value;

        static constexpr const char *name() { return F::name(); }

        static std::optional<Optional> deserialize(const uint8_t *&cur, const uint8_t *end)
        {
            Optional o;
            if (cur < end)
            {
                auto v = F::deserialize(cur, end);
                if (!v) return std::nullopt;
                o.value = std::move(v);
            }
            return o;
        }

        void serialize(uint8_t *&cur, uint8_t *end) const
        {
            if (value) value->serialize(cur, end);
        }

        void describe(std::ostream &os) const
        {
            if (value) value->describe(os);
            else       os << "(absent)";
        }
    };

    // ============================================================ Payload
    //
    // The variadic-tuple holder. Specialised for std::tuple<Fields...> so
    // codec definitions read like spec lines.

    template<typename TupleT>
    struct Payload;

    template<typename... Fs>
    struct Payload<std::tuple<Fs...>>
    {
        std::tuple<Fs...> fields;

        /// @brief Try to parse @p data into a fully-populated payload. Each
        ///        field consumes bytes from the front of @p data; when the
        ///        last field returns, all bytes should be consumed (otherwise
        ///        we still return success but log via the "trailing bytes"
        ///        path of the dispatcher).
        ///
        /// @return std::nullopt if any field returned nullopt.
        static std::optional<Payload> deserialize(std::span<const uint8_t> data);

        /// @brief Same as deserialize() but mutates *this in place. Used by
        ///        CodecAdapter so that per-codec `describe()` overrides in
        ///        derived structs are dispatched correctly: the alternative
        ///        (calling the static deserialize) returns a base Payload
        ///        and would always shadow-call the base describe(), making
        ///        per-codec friendly output impossible.
        ///
        /// @return std::nullopt if any field returned nullopt. Otherwise the
        ///         number of bytes the schema actually consumed. The caller
        ///         should sanity-check this against `data.size()`: a schema
        ///         that consumes fewer bytes than the payload is a hint the
        ///         schema is incomplete (Honeywell devices love adding
        ///         vendor-specific tail fields). The dispatcher logs the
        ///         delta so we notice and can extend the schema.
        std::optional<size_t> deserialize_into(std::span<const uint8_t> data);

        /// @brief Serialize each field in declaration order into @p out.
        /// @return Number of bytes written.
        size_t serialize(std::span<uint8_t> out) const;

        /// @brief Print "name=value" pairs separated by ", ". Codec structs
        ///        may shadow this in the derived type to emit a friendlier
        ///        single-sentence description (which CodecAdapter then sees
        ///        because it constructs the derived type before calling).
        void describe(std::ostream &os) const;
    };

    // ============================================================ Inline impls

    namespace detail
    {
        // Apply F over each tuple element, threading a boolean success flag
        // and the cur/end pair. Used by Payload::deserialize.
        template<typename Tuple, typename F, size_t... Is>
        bool fold_indexed(Tuple &t, F &&f, std::index_sequence<Is...>)
        {
            bool ok = true;
            ((ok = ok && f(std::get<Is>(t), Is)), ...);
            return ok;
        }
    } // namespace detail

    // ---- Payload<tuple<Fs...>>::deserialize -------------------------------

    template<typename... Fs>
    std::optional<Payload<std::tuple<Fs...>>>
    Payload<std::tuple<Fs...>>::deserialize(std::span<const uint8_t> data)
    {
        Payload p;
        if (!p.deserialize_into(data)) return std::nullopt;
        return p;
    }

    template<typename... Fs>
    std::optional<size_t>
    Payload<std::tuple<Fs...>>::deserialize_into(std::span<const uint8_t> data)
    {
        const uint8_t *cur = data.data();
        const uint8_t *end = data.data() + data.size();

        const bool ok = detail::fold_indexed(
            fields,
            [&](auto &field_inst, size_t /*idx*/) -> bool {
                using FieldT = std::remove_reference_t<decltype(field_inst)>;
                auto v = FieldT::deserialize(cur, end);
                if (!v) return false;
                field_inst = std::move(*v);
                return true;
            },
            std::index_sequence_for<Fs...>{});

        if (!ok) return std::nullopt;
        return static_cast<size_t>(cur - data.data());
    }

    // ---- Payload<tuple<Fs...>>::serialize ---------------------------------

    template<typename... Fs>
    size_t Payload<std::tuple<Fs...>>::serialize(std::span<uint8_t> out) const
    {
        uint8_t *cur = out.data();
        uint8_t *end = out.data() + out.size();
        std::apply([&](auto const &... f) { ((f.serialize(cur, end)), ...); }, fields);
        return static_cast<size_t>(cur - out.data());
    }

    // ---- Payload<tuple<Fs...>>::describe ----------------------------------

    template<typename... Fs>
    void Payload<std::tuple<Fs...>>::describe(std::ostream &os) const
    {
        bool first = true;
        std::apply(
            [&](auto const &... f) {
                ((first ? (first = false, (f.describe(os), 0))
                        : (os << ", ", f.describe(os), 0)),
                 ...);
            },
            fields);
    }

    // ---- Repeated<Inner...>::deserialize ----------------------------------

    template<typename... Inner>
    std::optional<Repeated<Inner...>>
    Repeated<Inner...>::deserialize(const uint8_t *&cur, const uint8_t *end)
    {
        Repeated out;
        while (cur < end && out.count < kMaxRows)
        {
            row_t row;
            const uint8_t *probe = cur;
            bool ok = std::apply(
                [&](auto &... f) {
                    bool acc = true;
                    auto step = [&](auto &fld) -> void {
                        if (!acc) return;
                        using FT = std::remove_reference_t<decltype(fld)>;
                        auto v = FT::deserialize(probe, end);
                        if (!v) { acc = false; return; }
                        fld = std::move(*v);
                    };
                    (step(f), ...);
                    return acc;
                },
                row);

            if (!ok)
            {
                // Stop here. We accept partial-trailing-bytes silently; the
                // dispatcher already logs raw bytes for any frame whose
                // payload doesn't match its schema cleanly.
                break;
            }
            out.rows[out.count++] = std::move(row);
            cur = probe;
        }
        return out;
    }

    template<typename... Inner>
    void Repeated<Inner...>::serialize(uint8_t *&cur, uint8_t *end) const
    {
        for (uint8_t i = 0; i < count; ++i)
        {
            std::apply([&](auto const &... f) { ((f.serialize(cur, end)), ...); },
                       rows[i]);
        }
    }

    template<typename... Inner>
    void Repeated<Inner...>::describe(std::ostream &os) const
    {
        os << "[";
        for (uint8_t i = 0; i < count; ++i)
        {
            if (i) os << " | ";
            bool first = true;
            std::apply(
                [&](auto const &... f) {
                    ((first ? (first = false, (f.describe(os), 0))
                            : (os << " ", f.describe(os), 0)),
                     ...);
                },
                rows[i]);
        }
        os << "]";
    }

} // namespace evohome
