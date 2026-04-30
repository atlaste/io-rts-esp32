#pragma once

#include "RadioModule.hpp"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include <atomic>
#include <stdint.h>

namespace RadioLinks
{
    /// @brief Driver for the Semtech SX1261 / SX1262 / SX1268 sub-GHz radio module, used as a drop-in replacement for RadioSX1276.
    ///
    /// The SX126x family does not natively support IO-Homecontrol packet handling like the SX1276 does
    /// (RF_PACKETCONFIG2_IOHOME_ON). On the SX1276 that hidden bit does three things at once:
    ///   1. UART-frames every byte on-air: start (0) + 8 LSB-first data bits + stop (1) = 10 chip bits
    ///      per protocol byte. The chip's sync detector internally matches the protocol-level
    ///      sync `55 FF 33` against the UART-encoded bit stream.
    ///   2. Computes/checks a CRC-CCITT KERMIT (poly 0x8408 reflected, init 0x0000) over the whole
    ///      frame including the length-encoded ctrl_byte_0.
    ///   3. Honours the IO-Homecontrol "length is in ctrl_byte_0[4:0]" framing rule.
    ///
    /// The SX126x sync detector only matches at the chip-bit level. The unique 24-bit window in the
    /// UART-encoded `55 FF 33` is `57 FD 99` (bits 6..29 of the 30-bit UART encoding); we program
    /// that as the chip-level sync. Everything past the sync is UART-encoded protocol bytes which
    /// this driver software-decodes in the RX_DONE path and software-encodes in Send().
    ///
    /// References:
    ///   - laberning/home_io_control: working SX1262 ESPHome implementation that proved the
    ///     UART hypothesis and the 0x57 0xFD 0x99 sync.
    ///   - Velocet/iown-homecontrol: protocol documentation.
    class RadioSX1262 : public RadioModule
    {
    public:
        /// @brief Constructor for SX1262 radio module interface
        /// @param spiHost SPI Host Controller ID to use
        /// @param cs SPI - GPIO connected to NSS (chip select) pin
        /// @param rst GPIO connected to RESET pin (active low)
        /// @param busy GPIO connected to BUSY pin (input)
        /// @param dio1 GPIO connected to DIO1 pin (used as IRQ line)
        /// @param useDio2RfSwitch If true, configure the chip to drive DIO2 as the RF antenna switch (default true, common on Lilygo / Heltec / Ebyte boards)
        /// @param useTCXO If true, the module has an external TCXO controlled through DIO3 (set true for Heltec V3 / Lilygo T3-S3 SX1262 boards)
        /// @param tcxoVoltage_mv TCXO control voltage in mV when useTCXO=true (1600..3300, default 1800)
        /// @param rxen GPIO connected to an external RX_EN pin of the antenna RF switch (driven HIGH while receiving). Set to GPIO_NUM_NC if not used.
        ///        Common on boards like the DFRobot ESP32-S3 LoRaWAN where DIO2 controls TX but RX is gated by a separate ESP32 GPIO.
        /// @param txen GPIO connected to an external TX_EN pin of the antenna RF switch (driven HIGH while transmitting). Set to GPIO_NUM_NC if not used.
        explicit RadioSX1262(spi_host_device_t spiHost, int cs, int rst, int busy, int dio1,
                             bool useDio2RfSwitch = true,
                             bool useTCXO = false,
                             uint32_t tcxoVoltage_mv = 1800,
                             int rxen = -1,
                             int txen = -1);

        /// @brief Dump a few useful registers / status to console (debug only)
        void DumpRegisters();

        /// @brief Called internally when the DIO1 GPIO is triggered, do not call directly
        /// @param gpio GPIO triggered
        void ManageInterrupt(int gpio);

        // RadioModule interface

        ~RadioSX1262() override;
        void Init(bool ioMode) override;
        void RegisterReceiveCallback(void (*func_ptr)(uint8_t, uint8_t[], uint32_t, float, int64_t)) override;
        RADIO_ERRCODE SetFrequency(uint32_t frequency) override;
        RADIO_ERRCODE SetModulation(Modulation modulation) override;
        RADIO_ERRCODE SetOutputPower(uint8_t txPower) override;
        RADIO_ERRCODE SetBitRate(uint32_t bitrate) override;
        RADIO_ERRCODE SetFrequencyDeviation(uint32_t deviation) override;
        RADIO_ERRCODE SetSyncWord(uint8_t size, uint8_t *syncWord) override;
        RADIO_ERRCODE SetBandwidth(uint32_t bandwidth) override;
        RADIO_ERRCODE SetPreambleLength(uint16_t preambleLen) override;
        RADIO_ERRCODE StartReceive() override;
        RADIO_ERRCODE StopReceive() override;
        RADIO_ERRCODE Send(uint8_t len, uint8_t *buffer, uint16_t preambleLen, uint32_t frequency) override;
        bool isPreambleDetected() override;

    private:
        static constexpr const char *TAG = "RadioSX1262";

        // Maximum protocol-level on-air payload (including software CRC). IO-Homecontrol frames are
        // at most 32 protocol bytes; we append 2 CRC bytes for a max protocol payload of 34 bytes.
        static constexpr uint8_t IOHOME_MAX_FRAME = 32;
        static constexpr uint8_t IOHOME_CRC_LEN   = 2;
        // After UART encoding (10 chip bits per protocol byte), 34 protocol bytes expand to 43
        // chip bytes. RX_IOHOME_LEN is the fixed chip-level capture size we program into the SX126x
        // packet engine. We round up to 32 to match laberning/home_io_control which empirically
        // covers typical 23-25-byte protocol frames after UART expansion (ceil(25*10/8) = 32). The
        // RX_DONE path always reads the full RX FIFO window (RX_BUFFER_READ_LEN) regardless, so we
        // pick up any tail past the chip-reported boundary too.
        static constexpr uint8_t RX_IOHOME_LEN       = 32;
        static constexpr uint8_t RX_BUFFER_READ_LEN  = 64;  // always read 64 chip bytes from FIFO
        static constexpr uint8_t RX_SNIFFER_LEN      = 240; // ~50 ms of air at 38.4 kbps GFSK
#ifdef CONFIG_IOHOMECONTROL_SX1262_DIAG_SNIFFER
        static constexpr uint8_t RX_FIXED_LEN        = RX_SNIFFER_LEN;
#else
        static constexpr uint8_t RX_FIXED_LEN        = RX_IOHOME_LEN;
#endif

        // UART-on-GFSK framing constants. See class header comment + applyIoHomeSync() for context.
        static constexpr uint8_t UART_BITS_PER_BYTE  = 10; // start + 8 data + stop
        static constexpr uint8_t UART_PROBE_MAX_BIT_OFFSET = 10; // try bit_offset 0..9 on RX

        // Hardware
        spi_host_device_t mSpiHost = SPI_HOST_MAX;
        int mSpiCS;                                   // SPI - CS pin
        spi_device_handle_t mSpiHandle = nullptr;
        int mIoRST;
        int mIoBUSY;
        int mIoDIO1;
        bool mUseDio2RfSwitch;
        bool mUseTCXO;
        uint32_t mTcxoVoltage_mv;
        int mIoRXEN; // optional external RX_EN GPIO for the antenna switch (-1 if not used)
        int mIoTXEN; // optional external TX_EN GPIO for the antenna switch (-1 if not used)

        // Cached configuration to be able to re-issue SetPacketParams whenever required
        bool mIoMode = true;
        uint32_t mBitrate = 38400;
        uint32_t mFrequencyDeviation = 19200;
        uint8_t mBandwidthReg = 0x19; // 312 kHz, closest above 250 kHz
        uint16_t mPreambleLenBits = 64;
        uint8_t mSyncWord[8] = {};
        uint8_t mSyncWordLenBits = 0;
        uint8_t mPayloadLen = RX_FIXED_LEN; // currently configured fixed payload length
        // True when SetSyncWord was called with the documented IO-Homecontrol sync 55 FF 33 and we
        // therefore overrode the chip-level sync to 0x57 0xFD 0x99. In that mode RX bytes are UART-
        // decoded in software and TX bytes are UART-encoded before being written to the FIFO.
        bool mIoHomeMode = false;

        void (*mCallback)(uint8_t len, uint8_t buffer[], uint32_t frequency, float rssi, int64_t time_since_preamble) = nullptr;

        bool mIsReceiveMode = false;
        std::atomic<int64_t> mLastPreambleDetectedTime; // us
        std::atomic<bool> mPreambleDetectedFlag;        // set by ManageInterrupt, consumed by isPreambleDetected()
        uint32_t mCurrentFrequency = 0;

        // ----- low level SPI / GPIO helpers -----

        void spiBegin();
        void spiEnd();
        void pinMode(int pin, gpio_mode_t mode, gpio_pullup_t pullup, gpio_pulldown_t pulldown);
        RADIO_ERRCODE digitalWrite(int pin, uint32_t value);
        uint32_t digitalRead(int pin);

        /// @brief Block until BUSY line is low, returning RADIO_ERR_BUSY on timeout.
        RADIO_ERRCODE waitBusy(uint32_t timeout_ms = 100);

        /// @brief Generic SX126x SPI transaction.
        /// @param tx Bytes clocked out on MOSI (must include opcode + parameters/NOPs).
        /// @param rx Bytes clocked in on MISO. nullptr to ignore.
        /// @param len Number of bytes to clock.
        RADIO_ERRCODE spiTransfer(const uint8_t *tx, uint8_t *rx, size_t len);

        // ----- SX126x command helpers -----

        RADIO_ERRCODE sendCommand(uint8_t opcode, const uint8_t *params = nullptr, size_t paramsLen = 0);
        RADIO_ERRCODE readCommand(uint8_t opcode, uint8_t *response, size_t responseLen);
        RADIO_ERRCODE writeRegister(uint16_t addr, const uint8_t *data, size_t len);
        RADIO_ERRCODE readRegister(uint16_t addr, uint8_t *data, size_t len);
        RADIO_ERRCODE writeBuffer(uint8_t offset, const uint8_t *data, size_t len);
        RADIO_ERRCODE readBuffer(uint8_t offset, uint8_t *data, size_t len);

        RADIO_ERRCODE setStandby(uint8_t mode);
        RADIO_ERRCODE setPacketType(uint8_t type);
        RADIO_ERRCODE setRegulatorMode(uint8_t mode);
        RADIO_ERRCODE setDio2AsRfSwitchCtrl(bool enable);
        RADIO_ERRCODE setDio3AsTcxoCtrl(uint8_t voltage, uint32_t delay_us);
        RADIO_ERRCODE calibrate(uint8_t calibParam);
        RADIO_ERRCODE calibrateImage(uint32_t frequency);
        RADIO_ERRCODE setPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel = 0x00, uint8_t paLut = 0x01);
        RADIO_ERRCODE setTxParams(uint8_t power, uint8_t rampTime);
        RADIO_ERRCODE setBufferBaseAddress(uint8_t txBase, uint8_t rxBase);
        RADIO_ERRCODE setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask = 0, uint16_t dio3Mask = 0);
        RADIO_ERRCODE clearIrqStatus(uint16_t irqMask);
        RADIO_ERRCODE getIrqStatus(uint16_t &irqStatus);
        RADIO_ERRCODE setRx(uint32_t timeout);
        RADIO_ERRCODE setTx(uint32_t timeout);
        RADIO_ERRCODE getRxBufferStatus(uint8_t &payloadLen, uint8_t &startPtr);
        RADIO_ERRCODE getPacketStatus(float &rssi);
        /// @brief Read the chip's instantaneous RSSI (returns dBm). Valid in RX mode.
        RADIO_ERRCODE getRssiInst(float &rssi);
        RADIO_ERRCODE applyModulationParams();
        RADIO_ERRCODE applyPacketParams(uint16_t preambleLenBits, uint8_t payloadLen);

        /// @brief Read SX126x chip status (Status command 0xC0). Bits [6:4] = chip mode, [3:1] = command status.
        RADIO_ERRCODE getStatus(uint8_t &status);

        /// @brief Read SX126x device errors. opErrors bit set means the chip detected the corresponding error.
        RADIO_ERRCODE getDeviceErrors(uint16_t &opErrors);

        /// @brief Clear all SX126x device errors.
        RADIO_ERRCODE clearDeviceErrors();

        /// @brief Format `getStatus()` byte into human-readable string ("STDBY_RC | DATA_AVAIL", ...). For debugging.
        static const char *statusModeName(uint8_t status);
        static const char *statusCmdName(uint8_t status);

        /// @brief Bring the radio in a known standby_RC state and apply the IO-Homecontrol register set.
        RADIO_ERRCODE InitRegisters(bool ioMode);

        /// @brief Convert a bandwidth in Hz to the closest SX126x register value (>= requested).
        static uint8_t bandwidthHzToReg(uint32_t bandwidth);

        /// @brief Compute the IO-Homecontrol CRC-CCITT KERMIT (poly 0x8408 reflected, init 0x0000)
        ///        over the given buffer. This is the on-air CRC variant used by IoHomeOn on SX1276
        ///        (verified against laberning/home_io_control which has a working SX1262 implementation).
        ///        Bytes are processed LSB-first; the result is appended low-byte-first by Send().
        static uint16_t crcKermit(const uint8_t *data, size_t len);

        /// @brief UART-encode a buffer of protocol bytes into a chip-level bit stream. Each protocol
        ///        byte becomes 10 chip bits: start (0) + 8 data bits LSB-first + stop (1). The output
        ///        is packed MSB-first into bytes (matching how the SX126x clocks bits onto the air).
        /// @return The number of chip bytes written, or 0 on overflow / invalid input.
        static size_t uartEncode(const uint8_t *data, size_t len, uint8_t *encoded, size_t encodedMaxLen);

        /// @brief UART-decode a chip-level capture into protocol bytes, starting at @p bitOffset
        ///        bits into the capture. Stops at the first byte whose start (0) or stop (1) bit
        ///        does not validate, so the caller can distinguish a clean decode from random noise.
        /// @return The number of protocol bytes successfully decoded.
        static size_t uartDecode(const uint8_t *raw, size_t rawLen, uint8_t bitOffset,
                                 uint8_t *decoded, size_t decodedMaxLen);

        /// @brief Search a chip-level capture for a plausible IO-Homecontrol frame.
        ///        Tries every bit_offset in [0..9), runs uartDecode at each, then slides through
        ///        candidate frame starts and validates with crcKermit. Returns true and copies the
        ///        validated protocol-level frame (without CRC) into @p frameOut if a clean match is
        ///        found.
        bool findUartFrame(const uint8_t *raw, size_t rawLen,
                           uint8_t *frameOut, size_t frameOutMax, size_t &frameOutLen) const;

        /// @brief Drive the optional external RX_EN / TX_EN GPIOs of the antenna RF switch.
        ///        Called whenever the radio transitions between standby / RX / TX. Safe no-op when both pins are unused.
        /// @param tx Set true when entering TX mode (drives TX_EN HIGH, RX_EN LOW).
        /// @param rx Set true when entering RX mode (drives RX_EN HIGH, TX_EN LOW).
        ///        When both are false the switch is parked (both pins LOW).
        void setRfSwitch(bool tx, bool rx);

        /// @brief Read & dispatch IRQ status. Called from the GPIO task on DIO1 rising edge.
        void handleDio1Irq();
    };
}
