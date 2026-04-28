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
    /// (RF_PACKETCONFIG2_IOHOME_ON). This driver therefore configures the SX126x packet engine in
    /// fixed-length GFSK mode with hardware CRC disabled and re-implements the IO-Homecontrol
    /// framing in software:
    ///   - the first byte (ctrl_byte_0) bits [4:0] are the actual frame length-1
    ///   - the CRC-CCITT (poly 0x1021, init 0xFFFF) is computed over the whole frame and appended
    ///     by Send(); on receive it is verified before delivering the payload to the upper layer.
    /// On the air this is wire-compatible with what an SX1276 in IoHomeOn mode would emit/receive.
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

        // Maximum on-air payload (including software CRC). IO-Homecontrol frames are at most 32 bytes.
        // We append 2 CRC bytes computed in software so the receiver always reads 34 bytes.
        static constexpr uint8_t IOHOME_MAX_FRAME = 32;
        static constexpr uint8_t IOHOME_CRC_LEN   = 2;
        static constexpr uint8_t RX_FIXED_LEN     = IOHOME_MAX_FRAME + IOHOME_CRC_LEN; // 34

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
        RADIO_ERRCODE applyModulationParams();
        RADIO_ERRCODE applyPacketParams(uint16_t preambleLenBits, uint8_t payloadLen);

        /// @brief Bring the radio in a known standby_RC state and apply the IO-Homecontrol register set.
        RADIO_ERRCODE InitRegisters(bool ioMode);

        /// @brief Convert a bandwidth in Hz to the closest SX126x register value (>= requested).
        static uint8_t bandwidthHzToReg(uint32_t bandwidth);

        /// @brief Compute the IO-Homecontrol CRC-CCITT (poly 0x1021, init 0xFFFF) over the given buffer.
        static uint16_t crc16Ccitt(const uint8_t *data, size_t len);

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
