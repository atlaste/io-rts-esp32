#pragma once

/**
 * @file ramses_codec.h
 * @brief Type-erasure layer between schema-declared codecs and the runtime
 *        dispatcher.
 *
 * Every concrete codec is a struct that derives from one of the Payload<>
 * specialisations in ramses_payload.h. To make a codec dispatch-able, the
 * registration macro REGISTER_CODEC wraps it in a CodecAdapter<T> that:
 *
 *   - exposes the (Code, Verb) key it claims,
 *   - holds a stable name string for logging,
 *   - implements parse() / describe_raw() in terms of T's static methods.
 *
 * The CodecRegistry is a thin wrapper over std::unordered_map; codec lookup
 * is O(1) hash on the (code << 2 | verb) tuple.
 */

#include "ramses_payload.h"
#include "ramses_types.h"

#include <cstdint>
#include <cstdio>
#include <functional>
#include <iosfwd>
#include <memory>
#include <ostream>
#include <span>
#include <string_view>
#include <unordered_map>

namespace evohome
{

    // ============================================================ CodecKey

    struct CodecKey
    {
        uint16_t code; // raw opcode (Code enum value or arbitrary)
        Verb     verb;

        bool operator==(const CodecKey &o) const noexcept
        {
            return code == o.code && verb == o.verb;
        }
    };

    struct CodecKeyHash
    {
        size_t operator()(const CodecKey &k) const noexcept
        {
            return (static_cast<size_t>(k.code) << 2) | static_cast<size_t>(k.verb);
        }
    };

    // ============================================================ ICodec

    /// @brief Type-erased view of a single codec, registered for a single
    ///        (Code, Verb) pair.
    class ICodec
    {
    public:
        virtual ~ICodec() = default;

        /// @brief The (Code, Verb) this codec is registered for.
        virtual CodecKey key() const = 0;

        /// @brief Stable, human-friendly name (e.g. "ZoneTemperature_Reply").
        virtual const char *name() const = 0;

        /// @brief Try to decode @p payload, writing a one-line description
        ///        to @p os. Returns true on a clean parse.
        virtual bool parse(std::span<const uint8_t> payload,
                           std::ostream &os) const = 0;

        /// @brief Universal "I don't know what this means" fallback: dump the
        ///        payload as hex. Default implementation hex-dumps; codecs
        ///        rarely need to override.
        virtual void describe_raw(std::span<const uint8_t> payload,
                                  std::ostream &os) const;
    };

    // ============================================================ CodecAdapter

    /// @brief Adapter that turns any Payload<T>-derived codec struct into an
    ///        ICodec. Constructed via REGISTER_CODEC.
    template<typename PayloadT>
    class CodecAdapter : public ICodec
    {
    public:
        CodecAdapter(CodecKey k, const char *n) : mKey(k), mName(n) {}

        CodecKey    key()  const override { return mKey;  }
        const char *name() const override { return mName; }

        bool parse(std::span<const uint8_t> payload, std::ostream &os) const override
        {
            // Construct PayloadT directly (not via the base Payload's static
            // deserialize) so that any per-codec describe() override in the
            // derived struct is dispatched. See PayloadT::deserialize_into
            // for the rationale.
            PayloadT p;
            const auto consumed = p.deserialize_into(payload);
            if (!consumed) return false;
            p.describe(os);
            // Sanity-check: did the schema actually consume the whole
            // payload? If not, the schema is incomplete (vendor-specific
            // tail bytes are common) and we surface the leftover so the
            // schema can be extended later. We still treat the parse as
            // successful so the friendly description survives.
            if (*consumed < payload.size())
            {
                const size_t extra = payload.size() - *consumed;
                os << " [+" << extra << "B unparsed: ";
                for (size_t i = *consumed; i < payload.size(); ++i)
                {
                    char buf[4];
                    std::snprintf(buf, sizeof(buf), "%02X", static_cast<unsigned>(payload[i]));
                    os << buf;
                }
                os << "]";
            }
            return true;
        }

    private:
        CodecKey    mKey;
        const char *mName; // pointer to a string literal
    };

    // ============================================================ Registry

    class CodecRegistry
    {
    public:
        /// @brief Register a codec. Last writer wins (returns false on
        ///        duplicate registration).
        bool add(std::unique_ptr<ICodec> codec);

        /// @brief Look up the codec for (code, verb), or nullptr if none.
        const ICodec *find(uint16_t code, Verb verb) const;

        /// @brief Convenience: register a Payload<>-derived schema struct.
        template<typename PayloadT>
        bool register_codec(uint16_t code, Verb verb, const char *name)
        {
            auto adapter = std::make_unique<CodecAdapter<PayloadT>>(CodecKey{code, verb}, name);
            return add(std::move(adapter));
        }

        size_t size() const { return mMap.size(); }

        /// @brief Iterate over all registered codecs (for "list codecs"
        ///        diagnostics from the CLI).
        void for_each(std::function<void(const ICodec &)> fn) const;

    private:
        std::unordered_map<CodecKey, std::unique_ptr<ICodec>, CodecKeyHash> mMap;
    };

    /// @brief Process-wide singleton. Codecs register themselves into this
    ///        instance via REGISTER_CODEC + register_all_codecs().
    CodecRegistry &global_codec_registry();

    /// @brief Wire all built-in codecs into the global registry. Idempotent.
    ///        Implemented in codecs/codec_register_all.cpp; declared here so
    ///        the entry point lives next to the registry it populates.
    void register_all_codecs(CodecRegistry &reg);

} // namespace evohome

// ============================================================ REGISTER_CODEC
//
// Registration macro. Usage:
//
//   struct ZoneTemperature_Reply
//       : Payload<std::tuple<ZoneIdx, TempCenti>> {};
//
//   void register_zone_temperature(CodecRegistry& reg) {
//       REGISTER_CODEC(reg, Code::ZoneTemperature, Verb::RP, ZoneTemperature_Reply);
//   }
//
// The third arg is the schema struct; its name (stringified) is used as the
// logger label, so prefer descriptive names (Verb suffix, etc.).
#define REGISTER_CODEC(reg, code_value, verb_value, payload_struct) \
    (reg).register_codec<payload_struct>(static_cast<uint16_t>(code_value), \
                                         (verb_value), #payload_struct)
