#pragma once

/**
 * @file EvohomeRamses.hpp
 * @brief Top-level facade for the RAMSES-II stack: configures the shared
 *        SX1262 radio for RAMSES, decodes raw chip bytes (sync detect +
 *        Manchester decode + frame parse), then dispatches to the codec
 *        registry.
 *
 * Lifecycle
 * ---------
 * The class is a singleton owned by IoRtsManager. Construction does NOT
 * touch the radio; the io-homecontrol stack stays the active radio user
 * until somebody calls StartSniff().
 *
 * StartSniff() reconfigures the SX1262 for RAMSES (868.3 MHz, 38.4 kbps
 * GFSK, sync word = 0xFF 0x00 0x33 0x55 0x53) and registers a raw-bytes
 * receive callback. While sniffing, the io-homecontrol stack stays
 * suspended (its hop task finds no frames on the radio because the radio
 * is on a different sync word).
 *
 * StopSniff() puts the radio back to standby. To return to io-homecontrol
 * the easiest path is currently a reboot; a proper "swap profile" API can
 * be added once we need bidirectional control.
 */

#include "ramses_codec.h"
#include "ramses_frame.h"
#include "ramses_types.h"

#include "RadioModule.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <span>

namespace iohome
{
    class IoHomeControl;
}

namespace evohome
{

    /// @brief Statistics surfaced through the CLI / MQTT for monitoring the
    ///        sniffer health.
    struct SnifferStats
    {
        uint64_t raw_bursts          = 0;  ///< RX_DONE callbacks from radio
        uint64_t sync_misses         = 0;  ///< no RAMSES sync header in the chip capture
        uint64_t manchester_failures = 0;  ///< sync OK, Manchester decode failed
        uint64_t framing_failures    = 0;  ///< Manchester OK, frame parse / csum failed
        uint64_t frames_decoded      = 0;  ///< complete + checksum-valid frames
        uint64_t codec_hits          = 0;  ///< frames whose (code,verb) had a registered codec
        uint64_t codec_misses        = 0;  ///< frames whose codec was missing
    };

    class EvohomeRamses
    {
    public:
        explicit EvohomeRamses(RadioLinks::RadioModule *radio);
        ~EvohomeRamses();

        /// @brief Optional: hand a pointer to the IoHomeControl instance that
        ///        normally owns the radio. When set, StartSniff() will pause
        ///        the io-homecontrol receive path (and its frequency-hopping
        ///        task) before reconfiguring the radio for RAMSES; otherwise
        ///        the io-home hop task will immediately re-tune the chip away
        ///        from 868.300 MHz and the sniffer will see nothing.
        void SetIoHome(iohome::IoHomeControl *iohome) { mIoHome = iohome; }

        EvohomeRamses(const EvohomeRamses &)            = delete;
        EvohomeRamses &operator=(const EvohomeRamses &) = delete;

        /// @brief Reconfigure the radio for RAMSES and start receiving.
        ///        Returns false if the radio refused configuration.
        bool StartSniff();

        /// @brief Stop receiving. The radio is left in standby; callers
        ///        wishing to switch back to io-homecontrol should re-call
        ///        IoHomeControl::ConfigureRadio() (or just reboot).
        void StopSniff();

        bool IsSniffing() const { return mActive.load(); }

        const SnifferStats &Stats() const { return mStats; }

        /// @brief Override the registry used to dispatch frames (default is
        ///        the global registry populated by register_all_codecs()).
        ///        Mostly useful for unit tests; production code should leave
        ///        this alone.
        void SetRegistry(const CodecRegistry *reg) { mRegistry = reg; }

        /// @brief Toggle raw RX dumps. When true, every chip-level capture
        ///        is logged as hex (with the Manchester-decoded protocol
        ///        bytes alongside, if decode succeeded) regardless of
        ///        whether the frame parser is happy with it. Useful for
        ///        reverse-engineering new codes or diagnosing radio issues
        ///        when the dispatcher reports nothing.
        void SetDumpRaw(bool on) { mDumpRaw.store(on); }
        bool IsDumpingRaw() const { return mDumpRaw.load(); }

        /// @brief Process a single chip-level capture. Public for both unit
        ///        testing and the `ev_decode_hex` debug command.
        void ProcessChipBytes(const uint8_t *raw, size_t len, uint32_t freq, float rssi);

    private:
        // Radio glue
        RadioLinks::RadioModule *mRadio  = nullptr;
        iohome::IoHomeControl   *mIoHome = nullptr;
        std::atomic<bool>        mActive{false};
        std::atomic<bool>        mDumpRaw{true}; // ON by default during bring-up

        // Dispatch
        const CodecRegistry *mRegistry = nullptr;

        // Stats
        SnifferStats mStats{};

        // Singleton hand-off for the C-style radio callback. The radio API
        // takes a free-function pointer with no user-data slot, so we need
        // a static pointer to the active EvohomeRamses instance.
        static EvohomeRamses *sActive;
        static void RadioRxThunk(uint8_t len, uint8_t buffer[], uint32_t frequency,
                                 float rssi, int64_t time_since_preamble);

        // Hot-path internals
        bool DecodeAndDispatch(std::span<const uint8_t> proto_bytes,
                               uint32_t freq, float rssi);

        // Pre-allocated decode/log scratch buffers. They live as members
        // (not on the stack) because ProcessChipBytes() runs on the radio
        // GPIO task which only has 4 KB of stack: a 240-byte capture
        // expands into ~ 200B of UART-decoded bytes + ~ 100B of
        // Manchester-decoded bytes + 3x ~ 800B hex log strings, easily
        // blowing the stack. ProcessChipBytes() is single-threaded
        // (only ever called from the radio GPIO task) so no locking is
        // needed.
        //
        // Sizes derived from the chip-level capture window in
        // EvohomeRamses.cpp (kRxFixedLen = 240). Keep this header
        // independent of that constant by sizing for the worst case
        // the SX1262 can capture in fixed-length mode.
        static constexpr size_t kRawCaptureMax  = 240;
        static constexpr size_t kUartBufMax     = (kRawCaptureMax * 8) / 10;  // 192
        static constexpr size_t kProtoBufMax    = kUartBufMax / 2;            // 96
        static constexpr size_t kHexBufMax      = 3 * kRawCaptureMax + 8;     // 728
        uint8_t mUartBuf [kUartBufMax];
        uint8_t mProtoBuf[kProtoBufMax];
        char    mHexBuf  [kHexBufMax];
    };

} // namespace evohome
