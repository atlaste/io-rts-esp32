#include "ramses_codec.h"

#include <cstdio>
#include <ostream>

namespace evohome
{

    void ICodec::describe_raw(std::span<const uint8_t> payload, std::ostream &os) const
    {
        os << name() << " { raw=";
        for (uint8_t b : payload)
        {
            char buf[4];
            std::snprintf(buf, sizeof(buf), "%02X", static_cast<unsigned>(b));
            os << buf;
        }
        os << " }";
    }

    bool CodecRegistry::add(std::unique_ptr<ICodec> codec)
    {
        if (!codec) return false;
        const auto k = codec->key();
        // emplace returns {iter, inserted}. We do NOT overwrite an existing
        // codec because codec definitions live in disparate translation units
        // and silently swapping them would be confusing.
        return mMap.emplace(k, std::move(codec)).second;
    }

    const ICodec *CodecRegistry::find(uint16_t code, Verb verb) const
    {
        auto it = mMap.find(CodecKey{code, verb});
        if (it == mMap.end()) return nullptr;
        return it->second.get();
    }

    void CodecRegistry::for_each(std::function<void(const ICodec &)> fn) const
    {
        for (const auto &kv : mMap) fn(*kv.second);
    }

    CodecRegistry &global_codec_registry()
    {
        static CodecRegistry reg;
        return reg;
    }

} // namespace evohome
