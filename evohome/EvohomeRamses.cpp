#include "EvohomeRamses.hpp"

#include "IoHomeControl.hpp"
#include "ramses_manchester.h"
#include "ramses_uart.h"

#include "esp_log.h"

#include <cstdio>
#include <cstring>
#include <sstream>

namespace evohome
{

    static const char *TAG = "evohome";

    // RAMSES-II radio profile (CC1101 reference values from evofw3, mapped
    // 1:1 to SX126x setters):
    //   Carrier      : 868.300 MHz
    //   Modulation   : 2-FSK / GFSK
    //   Bitrate      : 38.400 kbps
    //   Deviation    : ~ 50 kHz (CC1101 DEVIATN=0x50; we use 38kHz which is
    //                  inside the SX126x's wider acceptance range)
    //   Bandwidth    : ~ 270 kHz (CC1101 MDMCFG4=0x6A); SX126x's nearest is
    //                  234 kHz / 312 kHz - 312 kHz is the safe pick.
    //   Sync word    : The protocol-level sync is FF 00 33 55 53 (5 bytes
    //                  / 40 bits) BUT evofw3 clocks every byte through
    //                  the AVR's HW UART before pushing it to the CC1101
    //                  in async-serial mode. So on the air every byte is
    //                  a 10-bit UART frame: <start=0> <8 data bits LSB
    //                  first> <stop=1>. Concatenating the UART-encoded
    //                  bytes 0xFF 0x00 0x33 0x55 (4 of the 5 sync bytes)
    //                  and packing the resulting 40 bits MSB-first gives
    //                  the on-air sync we program here. We deliberately
    //                  drop the last protocol sync byte (0x53) from the
    //                  hardware sync to keep the chip-level sync exactly
    //                  40 bits / 5 bytes - it ends up in the captured
    //                  data and the UART decoder swallows it as the
    //                  first decoded byte.
    //   Preamble     : RAMSES senders emit ~5 bytes of UART-framed 0x55,
    //                  which are 50 bits of perfect 0101... alternation.
    //                  We ask the SX126x to lock after seeing 16 bits of
    //                  alternation (tightest robust setting).
    //   Payload len  : fixed 240 chip bytes (covers ~ 60 protocol bytes
    //                  after UART de-frame + Manchester decode, more
    //                  than enough for any legitimate evohome / Itho /
    //                  Honeywell HVAC traffic).
    constexpr uint32_t kFrequencyHz    = 868'300'000u;
    constexpr uint32_t kBitrateBps     = 38'400u;
    constexpr uint32_t kDeviationHz    = 38'000u;
    constexpr uint32_t kBandwidthHz    = 312'000u;
    constexpr uint16_t kPreambleBytes  = 2;     // -> 16 bits, see comment above
    constexpr uint8_t  kRxFixedLen     = 240;

    // On-air UART-encoded form of {0xFF, 0x00, 0x33, 0x55}. See the long
    // sync-word comment above for derivation. The 5th protocol sync byte
    // (0x53) is left in the captured data - the UART decoder treats it
    // as the first decoded byte and the message dispatch ignores the
    // first byte.
    constexpr uint8_t  kSyncWord[]    = {0x7F, 0xC0, 0x16, 0x65, 0x55};
    constexpr size_t   kSyncWordLen   = sizeof(kSyncWord);

    EvohomeRamses *EvohomeRamses::sActive = nullptr;

    EvohomeRamses::EvohomeRamses(RadioLinks::RadioModule *radio) : mRadio(radio)
    {
        // Populate the global registry once, lazily; the ctor is the obvious
        // place since EvohomeRamses is the only RAMSES entry point.
        auto &reg = global_codec_registry();
        if (reg.size() == 0) register_all_codecs(reg);
        mRegistry = &reg;
    }

    EvohomeRamses::~EvohomeRamses()
    {
        StopSniff();
    }

    bool EvohomeRamses::StartSniff()
    {
        if (mActive.load())
        {
            ESP_LOGW(TAG, "StartSniff: already sniffing");
            return true;
        }
        if (mRadio == nullptr)
        {
            ESP_LOGE(TAG, "StartSniff: no radio");
            return false;
        }

        ESP_LOGI(TAG, "StartSniff: configuring SX1262 for RAMSES-II "
                      "(%lu Hz, %lu bps, fdev=%lu Hz, BW=%lu Hz)",
                 (unsigned long)kFrequencyHz, (unsigned long)kBitrateBps,
                 (unsigned long)kDeviationHz, (unsigned long)kBandwidthHz);

        // The io-homecontrol stack normally drives the radio: it owns a
        // ~3 ms frequency-hopping task that calls SetFrequency() whenever
        // it sees nothing on the current channel, plus its own RX callback.
        // Both fight us if left running:
        //   - the hop task races with our SetFrequency() and parks the chip
        //     on 868.250 MHz, far away from RAMSES at 868.300 MHz;
        //   - it would also reset the preamble length back to the io-home
        //     8192-bit window (vs. our 16-bit window for RAMSES sync).
        // Pause the io-home receive path before reconfiguring; once paused
        // its hop task observes isReceiving()=false and stops touching the
        // radio. To swap back the simplest path is currently a reboot.
        if (mIoHome != nullptr)
        {
            ESP_LOGI(TAG, "StartSniff: pausing io-homecontrol receive "
                          "(reboot to switch back)");
            mIoHome->StopReceive();
        }

        // Order matters: the SX126x is happiest if we set modulation/bitrate
        // BEFORE sync word so that internal preamble-detection thresholds use
        // the right symbol rate. Each setter takes the radio mutex internally.
        if (mRadio->SetModulation(RadioLinks::Modulation::FSK) != RadioLinks::RADIO_ERR_NONE)
            { ESP_LOGE(TAG, "SetModulation failed"); return false; }
        if (mRadio->SetBandwidth(kBandwidthHz) != RadioLinks::RADIO_ERR_NONE)
            { ESP_LOGE(TAG, "SetBandwidth failed");  return false; }
        if (mRadio->SetBitRate(kBitrateBps) != RadioLinks::RADIO_ERR_NONE)
            { ESP_LOGE(TAG, "SetBitRate failed");    return false; }
        if (mRadio->SetFrequencyDeviation(kDeviationHz) != RadioLinks::RADIO_ERR_NONE)
            { ESP_LOGE(TAG, "SetFrequencyDeviation failed"); return false; }
        if (mRadio->SetFrequency(kFrequencyHz) != RadioLinks::RADIO_ERR_NONE)
            { ESP_LOGE(TAG, "SetFrequency failed");  return false; }
        if (mRadio->SetPreambleLength(kPreambleBytes) != RadioLinks::RADIO_ERR_NONE)
            { ESP_LOGE(TAG, "SetPreambleLength failed"); return false; }

        // SetSyncWord with the RAMSES sync clears the SX1262's internal
        // mIoHomeMode flag (only the magic 0x55,0xFF,0x33 enables that),
        // which in turn causes the radio's RX_DONE path to deliver raw
        // chip bytes to our callback - exactly what we need.
        uint8_t sw[kSyncWordLen];
        std::memcpy(sw, kSyncWord, kSyncWordLen);
        if (mRadio->SetSyncWord(kSyncWordLen, sw) != RadioLinks::RADIO_ERR_NONE)
            { ESP_LOGE(TAG, "SetSyncWord failed");  return false; }

        // Stretch the fixed-length capture window so we can fit a full
        // RAMSES message. Drivers that don't support runtime payload-length
        // resizing (SX1276 today) silently no-op and we'll have to live
        // with their default capture size.
        if (mRadio->SetRxFixedLen(kRxFixedLen) != RadioLinks::RADIO_ERR_NONE)
            ESP_LOGW(TAG, "SetRxFixedLen failed - using radio default");

        // Hand the radio our raw-bytes callback. We MUST do this last so any
        // bytes captured during the configuration steps above are discarded.
        sActive = this;
        mRadio->RegisterReceiveCallback(&EvohomeRamses::RadioRxThunk);

        if (mRadio->StartReceive() != RadioLinks::RADIO_ERR_NONE)
        {
            ESP_LOGE(TAG, "StartReceive failed");
            sActive = nullptr;
            return false;
        }

        mActive.store(true);
        std::memset(&mStats, 0, sizeof(mStats));
        ESP_LOGI(TAG, "StartSniff: listening for RAMSES-II traffic");
        return true;
    }

    void EvohomeRamses::StopSniff()
    {
        if (!mActive.exchange(false)) return;
        if (mRadio != nullptr) mRadio->StopReceive();
        sActive = nullptr;
        ESP_LOGI(TAG, "StopSniff: stopped after %llu raw bursts, %llu frames decoded "
                      "(%llu via codec, %llu unmapped)",
                 (unsigned long long)mStats.raw_bursts,
                 (unsigned long long)mStats.frames_decoded,
                 (unsigned long long)mStats.codec_hits,
                 (unsigned long long)mStats.codec_misses);
    }

    void EvohomeRamses::RadioRxThunk(uint8_t len, uint8_t buffer[], uint32_t frequency,
                                     float rssi, int64_t /*time_since_preamble*/)
    {
        EvohomeRamses *self = sActive;
        if (self == nullptr) return;
        self->ProcessChipBytes(buffer, len, frequency, rssi);
    }

    namespace
    {
        // Hex-encode @p data into @p out (NUL-terminated). Trims trailing
        // 0x00 bytes (typical demod tail) so the output stays focused on
        // the real signal. Returns the number of bytes shown (post-trim).
        size_t hex_dump(const uint8_t *data, size_t len, char *out, size_t out_max)
        {
            // Trim trailing 0x00 bytes (the SX126x demod produces long zero
            // tails after a real signal burst, just like the iohome sniffer
            // path does).
            while (len > 0 && data[len - 1] == 0x00) --len;
            char *w   = out;
            char *end = out + out_max - 1;
            for (size_t i = 0; i < len && w + 3 < end; ++i)
                w += std::snprintf(w, end - w, "%02X ", static_cast<unsigned>(data[i]));
            *w = '\0';
            return len;
        }
    } // namespace

    void EvohomeRamses::ProcessChipBytes(const uint8_t *raw, size_t len,
                                         uint32_t freq, float rssi)
    {
        ++mStats.raw_bursts;
        if (raw == nullptr || len == 0) return;

        // All scratch buffers live as members (mUartBuf / mProtoBuf /
        // mHexBuf) - this hot path runs on the radio GPIO task whose
        // stack is only 4 KB.

        // Step 1 of 2: UART de-frame.
        //
        // The captured `raw` is a packed bit stream (MSB-first per byte).
        // Each protocol byte is 10 bits on the wire (start + 8 data LSB-
        // first + stop). Slide a UART decoder across all 10 possible
        // bit alignments and keep the longest valid run - that handles
        // the ~few-bit slop in the chip's sync detector.
        size_t best_offset = 0;
        const size_t uart_len = uart::decode_best(
            {raw, len}, {mUartBuf, sizeof(mUartBuf)}, &best_offset);

        // The first protocol byte we decoded is the trailing 0x53 of the
        // RAMSES sync (we deliberately left it out of the chip-level
        // sync to keep that exactly 40 bits). Skip it; the rest is the
        // Manchester-encoded message proper.
        const size_t skip = (uart_len >= 1 && mUartBuf[0] == 0x53) ? 1u : 0u;

        // Step 2 of 2: Manchester decode the remaining UART-decoded
        // bytes. Stops at the 0x35 trailer (which is not a valid
        // Manchester code so the decoder returns when it sees it).
        size_t invalid_at = 0;
        const size_t proto_len = (uart_len >= 2)
            ? manchester::decode(
                {mUartBuf + skip, uart_len - skip},
                {mProtoBuf, sizeof(mProtoBuf)},
                &invalid_at)
            : 0u;

        // Decide what to log. We want clean log output when everything
        // works, and full triage information when it doesn't, so the
        // raw / UART / Manchester dumps are only emitted on failure (or
        // when the user explicitly turns on verbose mode for radio
        // debugging).
        bool decoded = false;
        if (proto_len > 0)
            decoded = DecodeAndDispatch({mProtoBuf, proto_len}, freq, rssi);

        if (!decoded)
        {
            if (uart_len < 2 || proto_len == 0) ++mStats.manchester_failures;
            else                                ++mStats.framing_failures;
        }

        if (mDumpRaw.load() || !decoded)
        {
            DumpStages(raw, len, freq, rssi,
                       uart_len, best_offset,
                       proto_len, invalid_at);
            if (!decoded && proto_len > 0)
                ESP_LOGI(TAG,
                    "RX RSSI=%.1f dBm: frame parse / csum failed (%u bytes)",
                    static_cast<double>(rssi),
                    static_cast<unsigned>(proto_len));
        }
    }

    void EvohomeRamses::DumpStages(const uint8_t *raw, size_t raw_len,
                                   uint32_t freq, float rssi,
                                   size_t uart_len, size_t bit_offset,
                                   size_t proto_len, size_t invalid_at)
    {
        const size_t shown = hex_dump(raw, raw_len, mHexBuf, sizeof(mHexBuf));
        ESP_LOGI(TAG, "RX raw  RSSI=%.1f dBm freq=%lu len=%u/%u: %s",
                 static_cast<double>(rssi),
                 static_cast<unsigned long>(freq),
                 static_cast<unsigned>(shown),
                 static_cast<unsigned>(raw_len),
                 mHexBuf);

        hex_dump(mUartBuf, uart_len, mHexBuf, sizeof(mHexBuf));
        ESP_LOGI(TAG, "RX uart bit_off=%u len=%u: %s",
                 static_cast<unsigned>(bit_offset),
                 static_cast<unsigned>(uart_len),
                 mHexBuf);

        if (proto_len > 0)
        {
            hex_dump(mProtoBuf, proto_len, mHexBuf, sizeof(mHexBuf));
            ESP_LOGI(TAG, "RX manc len=%u (stop@uart=%u): %s",
                     static_cast<unsigned>(proto_len),
                     static_cast<unsigned>(invalid_at),
                     mHexBuf);
        }
    }

    bool EvohomeRamses::DecodeAndDispatch(std::span<const uint8_t> proto_bytes,
                                          uint32_t freq, float rssi)
    {
        auto frame = parse_frame(proto_bytes);
        if (!frame) return false;

        ++mStats.frames_decoded;

        // Boot-relative timestamp, since the device usually has no NTP/RTC
        // when the user is hacking on this. Convert ms-since-boot to a
        // canonical HH:MM:SS.mmm string.
        const uint32_t now_ms = esp_log_timestamp();
        const unsigned ms = now_ms % 1000;
        const unsigned s  = (now_ms / 1000) % 60;
        const unsigned m  = (now_ms / 60000) % 60;
        const unsigned h  =  now_ms / 3600000;
        char ts[32];
        std::snprintf(ts, sizeof(ts), "%02u:%02u:%02u.%03u", h, m, s, ms);

        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss.precision(1);
        oss << "[" << ts << " " << rssi << "dBm @" << (freq / 1000000) << "MHz] ";
        frame->describe_friendly(oss);

        const ICodec *codec = (mRegistry != nullptr)
            ? mRegistry->find(frame->opcode, frame->verb)
            : nullptr;

        if (codec != nullptr)
        {
            ++mStats.codec_hits;
            oss << " - ";
            if (!codec->parse(frame->payload_view(), oss))
            {
                // Schema knows the code but parse failed - dump the bytes
                // so we can refine the schema.
                oss << "[parse failed] ";
                codec->describe_raw(frame->payload_view(), oss);
            }
        }
        else if (frame->payload_len > 0)
        {
            ++mStats.codec_misses;
            // Unknown opcode/verb: append a compact hex dump so the user
            // can copy/paste it into ramses_rf or add a codec.
            oss << " - raw=";
            for (uint8_t b : frame->payload_view())
            {
                char buf[4];
                std::snprintf(buf, sizeof(buf), "%02X", static_cast<unsigned>(b));
                oss << buf;
            }
        }
        else
        {
            ++mStats.codec_misses;
        }

        ESP_LOGI(TAG, "%s", oss.str().c_str());
        return true;
    }

} // namespace evohome
