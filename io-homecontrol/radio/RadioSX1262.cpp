#include "RadioSX1262.hpp"
#include "sx1262_commands.h"

#include <cstring>
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_log.h"

constexpr uint8_t  BOARD_READY_AFTER_POR_MS = 10;
constexpr TickType_t MUTEX_MAX_WAIT_TICKS    = pdMS_TO_TICKS(50);

static QueueHandle_t s1262GpioEvtQueue = nullptr; // queue containing GPIO triggered (uint32_t)
static SemaphoreHandle_t s1262Mutex = nullptr;    // mutex used to protect access to device on SPI bus
static StaticSemaphore_t s1262MutexBuffer;        // memory backing for s1262Mutex

/// @brief Called by the GPIO ISR when DIO1 fires.
static void IRAM_ATTR sx1262_gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    if (!xQueueSendFromISR(s1262GpioEvtQueue, &gpio_num, NULL))
    {
        ESP_DRAM_LOGI("RADIO", "sx1262_gpio_isr_handler can't add received frame to queue!");
    }
}

namespace RadioLinks
{
    /// @brief Task that drains the GPIO event queue and dispatches to the radio object.
    static void sx1262_gpio_task(void *arg)
    {
        uint32_t io_num;
        RadioSX1262 *radio = static_cast<RadioSX1262 *>(arg);
        for (;;)
        {
            if (xQueueReceive(s1262GpioEvtQueue, &io_num, portMAX_DELAY))
            {
                radio->ManageInterrupt(io_num);
            }
        }
    }

    // ====================================================================
    // Construction / destruction
    // ====================================================================

    RadioSX1262::RadioSX1262(spi_host_device_t spiHost, int cs, int rst, int busy, int dio1,
                             bool useDio2RfSwitch, bool useTCXO, uint32_t tcxoVoltage_mv,
                             int rxen, int txen)
        : mSpiHost(spiHost), mSpiCS(cs), mIoRST(rst), mIoBUSY(busy), mIoDIO1(dio1),
          mUseDio2RfSwitch(useDio2RfSwitch), mUseTCXO(useTCXO), mTcxoVoltage_mv(tcxoVoltage_mv),
          mIoRXEN(rxen), mIoTXEN(txen),
          mLastPreambleDetectedTime(0), mPreambleDetectedFlag(false)
    {
    }

    RadioSX1262::~RadioSX1262()
    {
        setStandby(SX126X_STANDBY_RC);
        setRfSwitch(false, false);
        spiEnd();
    }

    // ====================================================================
    // RadioModule interface
    // ====================================================================

    void RadioSX1262::Init(bool ioMode)
    {
        ESP_LOGI(TAG, "Init...");
        mIoMode = ioMode;

        // Configure GPIOs
        pinMode(mIoDIO1, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_ENABLE);
        pinMode(mIoBUSY, GPIO_MODE_INPUT, GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE);

        // Optional external RF switch GPIOs (driven by us; park them LOW = switch off until needed).
        if (mIoRXEN >= 0)
        {
            ESP_LOGI(TAG, "Init - using external RX_EN on GPIO%d", mIoRXEN);
            pinMode(mIoRXEN, GPIO_MODE_OUTPUT, GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE);
            digitalWrite(mIoRXEN, 0);
        }
        if (mIoTXEN >= 0)
        {
            ESP_LOGI(TAG, "Init - using external TX_EN on GPIO%d", mIoTXEN);
            pinMode(mIoTXEN, GPIO_MODE_OUTPUT, GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE);
            digitalWrite(mIoTXEN, 0);
        }

        // Reset sequence: hold RST low for >100us, then release.
        pinMode(mIoRST, GPIO_MODE_OUTPUT, GPIO_PULLUP_DISABLE, GPIO_PULLDOWN_DISABLE);
        digitalWrite(mIoRST, 0);
        vTaskDelay(pdMS_TO_TICKS(2));
        digitalWrite(mIoRST, 1);
        vTaskDelay(pdMS_TO_TICKS(BOARD_READY_AFTER_POR_MS));

        s1262Mutex = xSemaphoreCreateMutexStatic(&s1262MutexBuffer);

        spiBegin();

        // After reset SX126x is in cold-start sleep. Bring it to RC standby first.
        if (waitBusy(50) != RADIO_ERR_NONE)
        {
            ESP_LOGE(TAG, "Init - SX126x BUSY did not go low after reset");
        }
        setStandby(SX126X_STANDBY_RC);

        // Apply the rest of the configuration that does not depend on user-tunable parameters
        InitRegisters(ioMode);

        // Create FreeRTOS queue + GPIO task
        s1262GpioEvtQueue = xQueueCreate(10, sizeof(uint32_t));
        xTaskCreate(sx1262_gpio_task, "radio_sx1262_gpio_task", 4096, this, 10, NULL);

        // Hook the ISR for DIO1 (rising edge: chip drives the line high when an IRQ is pending)
        gpio_isr_handler_add(static_cast<gpio_num_t>(mIoDIO1), sx1262_gpio_isr_handler, (void *)mIoDIO1);
        gpio_set_intr_type(static_cast<gpio_num_t>(mIoDIO1), GPIO_INTR_POSEDGE);

        ESP_LOGI(TAG, "Init... end");
    }

    void RadioSX1262::RegisterReceiveCallback(void (*func_ptr)(uint8_t, uint8_t[], uint32_t, float, int64_t))
    {
        mCallback = func_ptr;
    }

    RADIO_ERRCODE RadioSX1262::SetFrequency(uint32_t frequency)
    {
        if (mCurrentFrequency == frequency)
            return RADIO_ERR_NONE;

        // Frf = (frequency * 2^25) / Fxtal
        uint64_t frf = (static_cast<uint64_t>(frequency) << 25) / SX126X_FXOSC;
        uint8_t out[4] = {
            static_cast<uint8_t>((frf >> 24) & 0xFF),
            static_cast<uint8_t>((frf >> 16) & 0xFF),
            static_cast<uint8_t>((frf >> 8) & 0xFF),
            static_cast<uint8_t>(frf & 0xFF),
        };

        if (xSemaphoreTake(s1262Mutex, MUTEX_MAX_WAIT_TICKS))
        {
            RADIO_ERRCODE ret = sendCommand(SX126X_CMD_SET_RF_FREQUENCY, out, sizeof(out));
            xSemaphoreGive(s1262Mutex);
            if (ret == RADIO_ERR_NONE)
                mCurrentFrequency = frequency;
            return ret;
        }
        // With MUTEX_MAX_WAIT_TICKS now at 50 ms, this branch is truly exceptional - it means
        // the SPI bus has been continuously held for >50 ms, which would only happen during a
        // real chip-level stall. Rate-limit very aggressively (one line per minute) so that if
        // it does happen we still see a breadcrumb without flooding the UART.
        static int64_t  sLastNoMutexUs = 0;
        static uint32_t sNoMutexCount  = 0;
        int64_t now = esp_timer_get_time();
        if ((now - sLastNoMutexUs) > 60000000)
        {
            ESP_LOGW(TAG, "SetFrequency - mutex held >50ms (+%u suppressed in last %lld s)",
                     (unsigned)sNoMutexCount, (long long)((now - sLastNoMutexUs) / 1000000));
            sLastNoMutexUs = now;
            sNoMutexCount = 0;
        }
        else
        {
            sNoMutexCount++;
        }
        return RADIO_ERR_BUSY;
    }

    RADIO_ERRCODE RadioSX1262::SetModulation(Modulation modulation)
    {
        switch (modulation)
        {
        case Modulation::FSK:
            return setPacketType(SX126X_PACKET_TYPE_GFSK);
        case Modulation::LoRa:
        case Modulation::OOK:
        default:
            return RADIO_ERR_INVALID_MODULATION;
        }
    }

    RADIO_ERRCODE RadioSX1262::SetOutputPower(uint8_t txPower)
    {
        // Datasheet table 13-21 - SX1262 high-power PA optimal settings.
        // We pick the closest setting based on requested power in dBm.
        uint8_t paDutyCycle = 0x04;
        uint8_t hpMax       = 0x07;
        int8_t  power       = static_cast<int8_t>(txPower);
        if (power > 22) power = 22;
        if (power < -9) power = -9;

        if (power >= 22)
        {
            paDutyCycle = 0x04;
            hpMax       = 0x07;
        }
        else if (power >= 20)
        {
            paDutyCycle = 0x03;
            hpMax       = 0x05;
        }
        else if (power >= 17)
        {
            paDutyCycle = 0x02;
            hpMax       = 0x03;
        }
        else
        {
            paDutyCycle = 0x02;
            hpMax       = 0x02;
        }

        if (xSemaphoreTake(s1262Mutex, MUTEX_MAX_WAIT_TICKS))
        {
            RADIO_ERRCODE err = setPaConfig(paDutyCycle, hpMax);
            if (err == RADIO_ERR_NONE)
                err = setTxParams(static_cast<uint8_t>(power), SX126X_PA_RAMP_200U);
            xSemaphoreGive(s1262Mutex);
            return err;
        }
        return RADIO_ERR_BUSY;
    }

    RADIO_ERRCODE RadioSX1262::SetBitRate(uint32_t bitrate)
    {
        if (bitrate == 0)
            return RADIO_ERR_INVALID_MODULATION;
        mBitrate = bitrate;
        if (xSemaphoreTake(s1262Mutex, MUTEX_MAX_WAIT_TICKS))
        {
            RADIO_ERRCODE err = applyModulationParams();
            xSemaphoreGive(s1262Mutex);
            return err;
        }
        return RADIO_ERR_BUSY;
    }

    RADIO_ERRCODE RadioSX1262::SetFrequencyDeviation(uint32_t deviation)
    {
        mFrequencyDeviation = deviation;
        if (xSemaphoreTake(s1262Mutex, MUTEX_MAX_WAIT_TICKS))
        {
            RADIO_ERRCODE err = applyModulationParams();
            xSemaphoreGive(s1262Mutex);
            return err;
        }
        return RADIO_ERR_BUSY;
    }

    RADIO_ERRCODE RadioSX1262::SetSyncWord(uint8_t size, uint8_t *syncWord)
    {
        if (size > 8 || size == 0)
            return RADIO_ERR_INVALID_SYNCWORD;

        // Cache - we reapply this whenever packet params change.
        memset(mSyncWord, 0, sizeof(mSyncWord));
        memcpy(mSyncWord, syncWord, size);
        mSyncWordLenBits = static_cast<uint8_t>(size * 8);

        // Special case: the iohome layer programs the documented IO-Homecontrol sync 0x55 0xFF 0x33
        // (3 bytes / 24 bits). On the SX1276 that protocol-level sync only matches because the
        // hidden IOHOME_ON bit also UART-frames every byte (start=0 + 8 LSB-first data bits +
        // stop=1) on the air, so the chip's sync detector internally hunts for the matching
        // bit pattern in the *UART-encoded* stream. The SX126x has no IOHOME_ON, and the protocol-
        // level pattern `01010101 11111111 00110011` simply never appears as such on the air -
        // what does appear is the UART-encoded version of those three bytes:
        //
        //   0x55 -> 0 10101010 1 = 0101010101
        //   0xFF -> 0 11111111 1 = 0111111111
        //   0x33 -> 0 11001100 1 = 0110011001
        //
        // Concatenated: 0101010101 0111111111 0110011001  (30 chip bits)
        //
        // The first 6 bits look exactly like preamble, but bits 6..29 are the unique 24-bit
        // window 01010111 11111101 10011001 = 0x57 0xFD 0x99. Programming THAT as the chip-level
        // sync makes the SX126x lock onto IO-Homecontrol traffic the same way SX1276 IOHOME_ON
        // would. After the sync match the FIFO contains UART-encoded protocol bytes which the
        // RX_DONE path then decodes in software (see findUartFrame()).
        //
        // Reference: laberning/home_io_control radio_sx1262.cpp (sync 0x57 0xFD 0x99 with the
        // same 24-bit hypothesis).
        if (size >= 3 && syncWord[0] == 0x55 && syncWord[1] == 0xFF && syncWord[2] == 0x33)
        {
            memset(mSyncWord, 0, sizeof(mSyncWord));
            mSyncWord[0] = 0x57;
            mSyncWord[1] = 0xFD;
            mSyncWord[2] = 0x99;
            mSyncWordLenBits = 24;
            mIoHomeMode = true;
            ESP_LOGI(TAG, "SetSyncWord: IO-Homecontrol detected (55 FF 33). SX126x has no IOHOME_ON, "
                          "programming chip-level sync 57 FD 99 = bits 6..29 of UART(55 FF 33). "
                          "RX/TX will UART-frame protocol bytes in software.");
        }
        else
        {
            mIoHomeMode = false;
        }

#ifdef CONFIG_IOHOMECONTROL_SX1262_DIAG_SNIFFER
        // Sniffer mode: relax to a 1-byte sync (0xFF) so we trigger on any 8-consecutive-1s window
        // in the bit stream. Combined with PREAMBLE_DETECT_OFF in applyPacketParams() this turns
        // the chip into a wide GFSK sniffer that catches IO-Home, Somfy IO, evohome, Itho, ...
        // bursts regardless of their specific preamble/sync layout. We use 0xFF because:
        //   - 0x55 is bit-wise indistinguishable from preamble (01010101 / 10101010) and matches
        //     inside the preamble with arbitrary phase, producing endless useless captures,
        //   - 0xFF (8 ones in a row) cannot occur inside a 0xAA/0x55 alternating preamble, so it
        //     genuinely fires on a transition out of preamble (or on real all-ones content like
        //     a Somfy IO leading burst).
        for (size_t i = 0; i < sizeof(mSyncWord); i++) mSyncWord[i] = 0x00;
        mSyncWord[0] = 0xFF;
        mSyncWordLenBits = 8;
        ESP_LOGW(TAG, "SetSyncWord: SNIFFER mode active - sync=0xFF, preamble detect OFF, capture %u bytes/trigger",
                 (unsigned)RX_SNIFFER_LEN);
#endif

        if (xSemaphoreTake(s1262Mutex, MUTEX_MAX_WAIT_TICKS))
        {
            // Sync word registers are 0x06C0 .. 0x06C7, MSB-first on the air.
            RADIO_ERRCODE err = writeRegister(SX126X_REG_SYNC_WORD_0, mSyncWord, 8);
            if (err == RADIO_ERR_NONE)
                err = applyPacketParams(mPreambleLenBits, mPayloadLen);

            // Read it back so we can prove the SPI write actually took effect.
            uint8_t readBack[8] = {};
            readRegister(SX126X_REG_SYNC_WORD_0, readBack, 8);
            xSemaphoreGive(s1262Mutex);

            ESP_LOGI(TAG, "SetSyncWord: programmed %u bits: %02X %02X %02X %02X %02X %02X %02X %02X",
                     mSyncWordLenBits,
                     mSyncWord[0], mSyncWord[1], mSyncWord[2], mSyncWord[3],
                     mSyncWord[4], mSyncWord[5], mSyncWord[6], mSyncWord[7]);
            ESP_LOGI(TAG, "SetSyncWord: read-back        : %02X %02X %02X %02X %02X %02X %02X %02X",
                     readBack[0], readBack[1], readBack[2], readBack[3],
                     readBack[4], readBack[5], readBack[6], readBack[7]);
            return err;
        }
        return RADIO_ERR_BUSY;
    }

    RADIO_ERRCODE RadioSX1262::SetBandwidth(uint32_t bandwidth)
    {
        mBandwidthReg = bandwidthHzToReg(bandwidth);
        if (xSemaphoreTake(s1262Mutex, MUTEX_MAX_WAIT_TICKS))
        {
            RADIO_ERRCODE err = applyModulationParams();
            xSemaphoreGive(s1262Mutex);
            return err;
        }
        return RADIO_ERR_BUSY;
    }

    RADIO_ERRCODE RadioSX1262::SetPreambleLength(uint16_t preambleLen)
    {
        // The RadioModule contract documents preambleLen "in bytes", matching how the SX1276 register works.
        // SX126x expects the preamble length in bits, so we convert here. 1024 bytes -> 8192 bits which fits in uint16_t.
        mPreambleLenBits = static_cast<uint16_t>(preambleLen * 8);
        if (xSemaphoreTake(s1262Mutex, MUTEX_MAX_WAIT_TICKS))
        {
            RADIO_ERRCODE err = applyPacketParams(mPreambleLenBits, mPayloadLen);
            xSemaphoreGive(s1262Mutex);
            return err;
        }
        return RADIO_ERR_BUSY;
    }

    RADIO_ERRCODE RadioSX1262::StartReceive()
    {
        if (!xSemaphoreTake(s1262Mutex, MUTEX_MAX_WAIT_TICKS))
        {
            ESP_LOGE(TAG, "StartReceive - No mutex available!");
            return RADIO_ERR_BUSY;
        }
        // Bring the chip back to a known state, re-arm the packet engine, then enter RX.
        // Important: turn the external RX_EN HIGH BEFORE issuing setRx() so the antenna is
        // already routed to the LNA when the demodulator starts looking for a preamble.
        // On boards like the DFRobot ESP32-S3 LoRaWAN the antenna is otherwise left
        // disconnected for ~1 ms after setRx(), which is more than enough to miss the
        // start of a short-preamble io-homecontrol response frame.
        // 
        // Use STDBY_XOSC (not STDBY_RC) so the TCXO stays alive across the standby->RX flip;
        // dropping to STDBY_RC would force a TCXO+PLL warm-up at the start of the next RX
        // window and we would miss the first few ms of actuator/controller replies.
        RADIO_ERRCODE err = setStandby(SX126X_STANDBY_XOSC);
        if (err == RADIO_ERR_NONE)
            err = applyPacketParams(mPreambleLenBits, RX_FIXED_LEN);
        if (err == RADIO_ERR_NONE)
            err = clearIrqStatus(SX126X_IRQ_ALL);
        if (err == RADIO_ERR_NONE)
            setRfSwitch(false, true);
        if (err == RADIO_ERR_NONE)
            err = setRx(SX126X_RX_TIMEOUT_CONTINUOUS);
        if (err != RADIO_ERR_NONE)
            setRfSwitch(false, false); // back-off: RX did not start, park the switch
        xSemaphoreGive(s1262Mutex);
        mPayloadLen = RX_FIXED_LEN;
        mPreambleDetectedFlag = false;
        mIsReceiveMode = (err == RADIO_ERR_NONE);
        if (err == RADIO_ERR_NONE)
        {
            uint8_t st = 0;
            if (getStatus(st) == RADIO_ERR_NONE)
                ESP_LOGI(TAG, "StartReceive OK (freq=%lu, preamble=%u bits, syncBits=%u, fixedLen=%u, status=0x%02X mode=%s)",
                         (unsigned long)mCurrentFrequency, mPreambleLenBits,
                         mSyncWordLenBits == 0 ? 24 : mSyncWordLenBits,
                         RX_FIXED_LEN, st, statusModeName(st));
            else
                ESP_LOGI(TAG, "StartReceive OK (freq=%lu, preamble=%u bits, syncBits=%u, fixedLen=%u)",
                         (unsigned long)mCurrentFrequency, mPreambleLenBits,
                         mSyncWordLenBits == 0 ? 24 : mSyncWordLenBits, RX_FIXED_LEN);
        }
        else
        {
            ESP_LOGE(TAG, "StartReceive FAILED (err=%d)", static_cast<int>(err));
        }
        return err;
    }

    RADIO_ERRCODE RadioSX1262::StopReceive()
    {
        if (!xSemaphoreTake(s1262Mutex, MUTEX_MAX_WAIT_TICKS))
        {
            ESP_LOGE(TAG, "StopReceive - No mutex available!");
            return RADIO_ERR_BUSY;
        }
        // STDBY_XOSC keeps the TCXO running so the next StartReceive() / Send() does not pay the
        // crystal warm-up cost.
        RADIO_ERRCODE err = setStandby(SX126X_STANDBY_XOSC);
        setRfSwitch(false, false);
        xSemaphoreGive(s1262Mutex);
        if (err == RADIO_ERR_NONE)
            mIsReceiveMode = false;
        return err;
    }

    RADIO_ERRCODE RadioSX1262::Send(uint8_t len, uint8_t *buffer, uint16_t preambleLen, uint32_t frequency)
    {
        if (len == 0 || len > IOHOME_MAX_FRAME)
            return RADIO_ERR_NULL_POINTER;

        // Build the protocol-level wire frame: [frame data] [CRC low] [CRC high]. The CRC is the
        // CRC-CCITT KERMIT of the frame data (poly 0x8408 reflected, init 0x0000). We append the
        // CRC low byte first to match what the SX1276 IoHomeOn mode emits and what
        // laberning/home_io_control's working implementation transmits.
        const uint16_t crc = crcKermit(buffer, len);
        uint8_t protoBuf[IOHOME_MAX_FRAME + IOHOME_CRC_LEN];
        memcpy(protoBuf, buffer, len);
        protoBuf[len]     = static_cast<uint8_t>(crc & 0xFF);
        protoBuf[len + 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);
        const uint8_t protoLen = static_cast<uint8_t>(len + IOHOME_CRC_LEN);

        // In IO-Homecontrol mode we UART-encode every protocol byte (start=0 + 8 LSB-first data
        // bits + stop=1) before clocking it onto the air. After encoding, every protocol byte
        // becomes 10 chip bits = 1.25 chip bytes; the chip-level FIFO transmit length is
        // ceil(protoLen * 10 / 8). Outside IoHome mode (e.g. raw GFSK use cases), we ship the
        // protocol bytes verbatim - no encoding.
        uint8_t txBuf[((IOHOME_MAX_FRAME + IOHOME_CRC_LEN) * UART_BITS_PER_BYTE + 7) / 8];
        size_t  txLen = 0;
        if (mIoHomeMode)
        {
            txLen = uartEncode(protoBuf, protoLen, txBuf, sizeof(txBuf));
            if (txLen == 0)
            {
                ESP_LOGE(TAG, "Send - uartEncode failed (protoLen=%u)", protoLen);
                return RADIO_ERR_NULL_POINTER;
            }
        }
        else
        {
            if (protoLen > sizeof(txBuf))
                return RADIO_ERR_NULL_POINTER;
            memcpy(txBuf, protoBuf, protoLen);
            txLen = protoLen;
        }

        if (!xSemaphoreTake(s1262Mutex, MUTEX_MAX_WAIT_TICKS))
        {
            ESP_LOGE(TAG, "Send - No mutex available!");
            return RADIO_ERR_BUSY;
        }

        // Stop everything, switch to the requested frequency, re-arm packet parameters with the exact tx length.
        // Park the RF switch while reconfiguring; flip to TX right before setTx.
        // Use STDBY_XOSC instead of STDBY_RC so the TCXO stays running while we reconfigure - the
        // PLL re-locks to the new frequency without having to wait for the crystal to start again,
        // which keeps frequency hopping snappy and avoids missing immediate replies after TX_DONE.
        setRfSwitch(false, false);
        RADIO_ERRCODE err = setStandby(SX126X_STANDBY_XOSC);
        if (err == RADIO_ERR_NONE && mCurrentFrequency != frequency)
        {
            uint64_t frf = (static_cast<uint64_t>(frequency) << 25) / SX126X_FXOSC;
            uint8_t out[4] = {
                static_cast<uint8_t>((frf >> 24) & 0xFF),
                static_cast<uint8_t>((frf >> 16) & 0xFF),
                static_cast<uint8_t>((frf >> 8) & 0xFF),
                static_cast<uint8_t>(frf & 0xFF),
            };
            err = sendCommand(SX126X_CMD_SET_RF_FREQUENCY, out, sizeof(out));
            if (err == RADIO_ERR_NONE)
                mCurrentFrequency = frequency;
        }
        mPreambleLenBits = static_cast<uint16_t>(preambleLen * 8);
        if (err == RADIO_ERR_NONE)
            err = applyPacketParams(mPreambleLenBits, static_cast<uint8_t>(txLen));
        if (err == RADIO_ERR_NONE)
            err = setBufferBaseAddress(0x00, 0x00);
        if (err == RADIO_ERR_NONE)
            err = writeBuffer(0x00, txBuf, txLen);
        if (err == RADIO_ERR_NONE)
            err = clearIrqStatus(SX126X_IRQ_ALL);
        if (err == RADIO_ERR_NONE)
        {
            setRfSwitch(true, false); // route antenna to PA before keying TX
            err = setTx(0);            // 0 = no timeout - the packet engine returns to standby on TX_DONE
            if (err != RADIO_ERR_NONE)
                setRfSwitch(false, false);
        }

        mPayloadLen = static_cast<uint8_t>(txLen);
        xSemaphoreGive(s1262Mutex);
        return err;
    }

    bool RadioSX1262::isPreambleDetected()
    {
        return mPreambleDetectedFlag.exchange(false);
    }

    void RadioSX1262::ManageInterrupt(int gpio)
    {
        if (gpio != mIoDIO1)
            return;
        handleDio1Irq();
    }

    void RadioSX1262::setRfSwitch(bool tx, bool rx)
    {
        if (mIoTXEN >= 0)
            digitalWrite(mIoTXEN, tx ? 1 : 0);
        if (mIoRXEN >= 0)
            digitalWrite(mIoRXEN, rx ? 1 : 0);
    }

    void RadioSX1262::handleDio1Irq()
    {
        // CRITICAL: unlike the SX1276 (short pulse / re-trigger-friendly DIO0), the SX126x holds
        // the DIO1 line HIGH until we send CLEAR_IRQ_STATUS. We register the GPIO pin for
        // GPIO_INTR_POSEDGE, so a permanently-high line never produces another rising edge - if
        // we ever return without clearing the IRQ, the radio stops generating interrupts entirely
        // and RX is dead until the next reset. Therefore we must NEVER drop a DIO1 event on the
        // floor: if MUTEX_MAX_WAIT_TICKS expires we requeue the GPIO event so the next loop
        // iteration retries.
        // The mutex is held by either:
        //   - the iohome process_radio_task running SetFrequency (bounded by waitBusy <= ~50 ms)
        //   - a Send/StartReceive/StopReceive call (bounded similarly)
        // so MUTEX_MAX_WAIT_TICKS (50 ms) is comfortably above worst-case contention.
        if (!xSemaphoreTake(s1262Mutex, MUTEX_MAX_WAIT_TICKS))
        {
            static int64_t  sLastBusyLogUs = 0;
            static uint32_t sBusySuppressed = 0;
            int64_t now = esp_timer_get_time();
            if ((now - sLastBusyLogUs) > 60000000)
            {
                ESP_LOGW(TAG, "ManageInterrupt - mutex held >%ums, requeueing (+%u suppressed in last %lld s)",
                         (unsigned)pdTICKS_TO_MS(MUTEX_MAX_WAIT_TICKS),
                         (unsigned)sBusySuppressed,
                         (long long)((now - sLastBusyLogUs) / 1000000));
                sLastBusyLogUs = now;
                sBusySuppressed = 0;
            }
            else
            {
                sBusySuppressed++;
            }
            // Requeue so we'll retry instead of leaving the chip's DIO1 line stuck high (which
            // would prevent any future POSEDGE IRQ from firing).
            uint32_t gpio = static_cast<uint32_t>(mIoDIO1);
            (void)xQueueSend(s1262GpioEvtQueue, &gpio, 0);
            // Yield briefly so the holder gets a chance to release before we spin around the
            // gpio_task loop again.
            vTaskDelay(pdMS_TO_TICKS(2));
            return;
        }

        uint16_t irqStatus = 0;
        bool getIrqOk = (getIrqStatus(irqStatus) == RADIO_ERR_NONE);
        if (!getIrqOk)
        {
            xSemaphoreGive(s1262Mutex);
            ESP_LOGE(TAG, "ManageInterrupt - getIrqStatus failed");
            return;
        }
        clearIrqStatus(irqStatus);

        // Rate-limit PREAMBLE-only IRQs (they fire constantly on RF noise at 312 kHz BW and would
        // drown out the actually interesting events). Anything beyond a bare PREAMBLE_DETECTED is
        // always logged because it is rare (real frames, sync match, errors, ...).
        constexpr uint16_t kPreambleOnly = SX126X_IRQ_PREAMBLE_DETECTED;
        bool isPreambleOnly = (irqStatus == kPreambleOnly);
        static int64_t  sLastPreambleLogUs = 0;
        static uint32_t sSuppressedPreambles = 0;
        static uint32_t sPreambleStreak     = 0;  // consecutive PREAMBLE-only IRQs without seeing SYNC/RX_DONE
        bool shouldLog = !isPreambleOnly;

#ifdef CONFIG_IOHOMECONTROL_SX1262_DIAG_SNIFFER
        // Sniffer mode: SYNC and RX_DONE fire very frequently (every 0xFF byte that walks past
        // the demodulator). The "DIO1 IRQ 0x... SYNC_WORD_VALID/RX_DONE" header lines would just
        // double up on the dedicated RX_DONE/hex-dump logs that follow. Suppress them when nothing
        // unexpected is set (CRC_ERROR, HEADER_ERROR, TIMEOUT, ... still get logged).
        constexpr uint16_t kBenignSnifferMask = SX126X_IRQ_PREAMBLE_DETECTED |
                                                SX126X_IRQ_SYNC_WORD_VALID |
                                                SX126X_IRQ_RX_DONE;
        if ((irqStatus & ~kBenignSnifferMask) == 0)
            shouldLog = false;
#endif

        // Decide whether we will emit either of the two LOGI lines below, and snapshot whatever
        // state they need, BEFORE we make a logging decision. The mutex must not be held while
        // ESP_LOGI runs - the default ESP-IDF console writes block on the UART (~87 us/char at
        // 115200 baud, so a 150-char line takes ~13 ms, and two stacked lines easily exceed
        // MUTEX_MAX_WAIT_TICKS, starving the iohome hop task).
        bool     logPreambleLine = false;
        uint32_t logPreambleSuppressed = 0;
        int64_t  logPreambleSinceMs    = 0;
        uint32_t logPreambleStreak     = 0;
        if (irqStatus & SX126X_IRQ_PREAMBLE_DETECTED)
        {
            mLastPreambleDetectedTime = esp_timer_get_time();
            mPreambleDetectedFlag = true;
        }
        if (isPreambleOnly)
        {
            sPreambleStreak++;
            int64_t now = esp_timer_get_time();
            if ((now - sLastPreambleLogUs) > 1000000) // one heartbeat per 1s
            {
                logPreambleLine       = true;
                logPreambleSuppressed = sSuppressedPreambles;
                logPreambleSinceMs    = (now - sLastPreambleLogUs) / 1000;
                logPreambleStreak     = sPreambleStreak;
                sLastPreambleLogUs    = now;
                sSuppressedPreambles  = 0;
            }
            else
            {
                sSuppressedPreambles++;
            }
        }
        else
        {
            sPreambleStreak = 0;
        }

        // RX_DONE / TX_DONE / TIMEOUT continue under the mutex (they need SPI access). All other
        // paths (PREAMBLE-only, SYNC_WORD_VALID, header errors with no RX_DONE, ...) just need
        // the IRQ-status read+clear we already did above, so release the mutex NOW and do the
        // rest (logging, flag updates) with the bus free.
        bool keepMutex = (irqStatus & (SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT)) != 0;
        if (!keepMutex)
        {
            xSemaphoreGive(s1262Mutex);
        }

        if (logPreambleLine)
        {
            ESP_LOGI(TAG, "DIO1 IRQ 0x%04X PREAMBLE (+%u suppressed in last %lld s, streak=%u)",
                     irqStatus, (unsigned)logPreambleSuppressed,
                     (long long)(logPreambleSinceMs / 1000),
                     (unsigned)logPreambleStreak);
        }
        if (shouldLog)
        {
            ESP_LOGI(TAG, "DIO1 IRQ 0x%04X%s%s%s%s%s%s%s%s%s%s",
                     irqStatus,
                     (irqStatus & SX126X_IRQ_TX_DONE)           ? " TX_DONE"           : "",
                     (irqStatus & SX126X_IRQ_RX_DONE)           ? " RX_DONE"           : "",
                     (irqStatus & SX126X_IRQ_PREAMBLE_DETECTED) ? " PREAMBLE"          : "",
                     (irqStatus & SX126X_IRQ_SYNC_WORD_VALID)   ? " SYNC_WORD_VALID"   : "",
                     (irqStatus & SX126X_IRQ_HEADER_VALID)      ? " HEADER_VALID"      : "",
                     (irqStatus & SX126X_IRQ_HEADER_ERROR)      ? " HEADER_ERROR"      : "",
                     (irqStatus & SX126X_IRQ_CRC_ERROR)         ? " CRC_ERROR"         : "",
                     (irqStatus & SX126X_IRQ_CAD_DONE)          ? " CAD_DONE"          : "",
                     (irqStatus & SX126X_IRQ_CAD_DETECTED)      ? " CAD_DETECTED"      : "",
                     (irqStatus & SX126X_IRQ_TIMEOUT)           ? " TIMEOUT"           : "");
        }

        if (!keepMutex)
        {
            // Mutex already released and all bookkeeping done.
            return;
        }

        if (irqStatus & SX126X_IRQ_RX_DONE)
        {
            // SX1262 RX engine has stopped, antenna switch can be parked while we read the buffer.
            setRfSwitch(false, false);
            uint8_t payloadLen = 0;
            uint8_t startPtr   = 0;
            // Read more than the configured fixed length: in IO-Homecontrol mode the SX126x
            // sometimes reports a short payload but the FIFO actually contains more chip bytes
            // past the reported boundary. laberning/home_io_control discovered this materially
            // improves post-auth response recovery on real hardware. Sized to RX_BUFFER_READ_LEN
            // (64) so we never under-read; in sniffer mode RX_FIXED_LEN is 240 and we still cap
            // to what the chip programmed.
            uint8_t rawBuf[RX_FIXED_LEN > RX_BUFFER_READ_LEN ? RX_FIXED_LEN : RX_BUFFER_READ_LEN];
            float rssi = -255.0f;
            getRxBufferStatus(payloadLen, startPtr);
            uint8_t readLen = payloadLen;
            // Snapshot the current RF frequency atomically while we still hold the chip mutex.
            // The radio task hops channels asynchronously: if we read mCurrentFrequency later
            // (e.g. after the SPI read, the hex dump and the UART decode) it could already
            // reflect the *next* channel, causing us to advertise the wrong frequency to the
            // upper layer - and TX our 0x29 reply on a channel Tahoma is no longer listening on.
            uint32_t rxFreq = mCurrentFrequency;
#ifdef CONFIG_IOHOMECONTROL_SX1262_DIAG_SNIFFER
            // Sniffer mode: trust the chip - we configured a long fixed length and want exactly
            // that many bytes (no overrun into stale FIFO data).
            if (readLen == 0 || readLen > RX_FIXED_LEN)
                readLen = RX_FIXED_LEN;
#else
            // IO-Homecontrol mode: pull RX_BUFFER_READ_LEN bytes if the chip reports less than the
            // configured fixed-length window, so we always have enough chip-level data for the
            // software UART decoder to scan.
            if (readLen == 0 || readLen < RX_FIXED_LEN)
                readLen = RX_BUFFER_READ_LEN;
            if (readLen > sizeof(rawBuf))
                readLen = sizeof(rawBuf);
#endif
            memset(rawBuf, 0, sizeof(rawBuf));
            RADIO_ERRCODE rdErr = readBuffer(startPtr, rawBuf, readLen);
            getPacketStatus(rssi);
            xSemaphoreGive(s1262Mutex);

            if (rdErr != RADIO_ERR_NONE)
            {
                // If we ever fall in here the captured byte buffer below is meaningless (still
                // memset to 0) and we'd otherwise log a stream of zeroes that looks identical to
                // a real "no signal" capture. Make this loud so it cannot get missed again.
                ESP_LOGE(TAG, "RX_DONE: readBuffer failed (err=%d, startPtr=%u, readLen=%u) - skipping",
                         (int)rdErr, (unsigned)startPtr, (unsigned)readLen);
            }

#ifdef CONFIG_IOHOMECONTROL_SX1262_DIAG_SNIFFER
            // RSSI gate: most sniffer captures are pure-noise false triggers from the 1-byte
            // sync. Logging each one (header + 15 hex lines = ~70 ms of UART time at 115200
            // baud) saturates the console, starves the radio mutex (-> "SetFrequency - No
            // mutex available!" spam), and eventually stalls RX entirely.
            constexpr float kSnifferRssiGate = (float)CONFIG_IOHOMECONTROL_SX1262_DIAG_SNIFFER_RSSI_GATE_DBM;
            static uint32_t sDroppedLow      = 0;
            static int64_t  sLastDropLogUs   = 0;
            bool snifferLog = (rssi >= kSnifferRssiGate);
            if (!snifferLog)
            {
                sDroppedLow++;
                int64_t now = esp_timer_get_time();
                if ((now - sLastDropLogUs) > 5000000) // one summary line every 5 s
                {
                    ESP_LOGD(TAG, "Sniffer: dropped %u low-RSSI capture(s) in last %lld ms (gate=%.0f dBm)",
                             (unsigned)sDroppedLow, (long long)((now - sLastDropLogUs) / 1000),
                             (double)kSnifferRssiGate);
                    sLastDropLogUs = now;
                    sDroppedLow = 0;
                }
            }
#else
            bool snifferLog = true;
#endif
            // If the SPI read failed the buffer is still zeroed and the dump would be misleading.
            if (rdErr != RADIO_ERR_NONE)
                snifferLog = false;

            if (snifferLog)
            {
                // Always log the raw FIFO content so we can post-mortem CRC failures or bad framing.
                ESP_LOGI(TAG, "RX_DONE: chipLen=%u, startPtr=%u, RSSI=%.1f dBm, freq=%lu",
                         payloadLen, startPtr, rssi, (unsigned long)rxFreq);
                // Trim trailing zero bytes from the dump - in sniffer mode the GFSK demod
                // typically outputs long zero tails after a brief real-signal burst, and they
                // contribute nothing useful to off-line analysis.
                uint8_t dumpLen = readLen;
                while (dumpLen > 16 && rawBuf[dumpLen - 1] == 0x00 && rawBuf[dumpLen - 2] == 0x00 &&
                       rawBuf[dumpLen - 3] == 0x00 && rawBuf[dumpLen - 4] == 0x00)
                {
                    dumpLen--;
                }
                ESP_LOG_BUFFER_HEX_LEVEL(TAG, rawBuf, dumpLen, ESP_LOG_INFO);
                if (dumpLen < readLen)
                {
                    ESP_LOGI(TAG, "  ... %u trailing zero byte(s) trimmed", (unsigned)(readLen - dumpLen));
                }
            }

#ifndef CONFIG_IOHOMECONTROL_SX1262_DIAG_SNIFFER
            // The chip captured `readLen` UART-encoded protocol bytes after matching the chip-level
            // sync (0x57 0xFD 0x99 if mIoHomeMode, see SetSyncWord). Software-decode them back to
            // protocol bytes and search for a CRC-valid IO-Homecontrol frame. findUartFrame() tries
            // every bit_offset in [0..UART_PROBE_MAX_BIT_OFFSET) and every plausible frame start;
            // CRC-CCITT KERMIT is the source of truth.
            if (mIoHomeMode)
            {
                uint8_t protoFrame[IOHOME_MAX_FRAME] = {0};
                size_t  protoLen = 0;
                if (findUartFrame(rawBuf, readLen, protoFrame, sizeof(protoFrame), protoLen))
                {
                    ESP_LOGI(TAG, "RX iohome OK: ctrl0=0x%02X len=%u RSSI=%.1f dBm freq=%lu",
                             protoFrame[0], (unsigned)protoLen, rssi,
                             (unsigned long)rxFreq);
                    if (mCallback != nullptr)
                    {
                        int64_t timeSincePreamble = esp_timer_get_time() - mLastPreambleDetectedTime;
                        mCallback(static_cast<uint8_t>(protoLen), protoFrame, rxFreq, rssi, timeSincePreamble);
                    }
                }
                else
                {
                    // No CRC-valid frame found in the capture. Rate-limit so weak / spurious sync
                    // matches do not flood the console while leaving useful debugging breadcrumbs.
                    static int64_t  sLastNoFrameLogUs = 0;
                    static uint32_t sNoFrameCount    = 0;
                    int64_t now = esp_timer_get_time();
                    if ((now - sLastNoFrameLogUs) > 1000000)
                    {
                        ESP_LOGW(TAG, "RX iohome: no CRC-valid frame in %u-byte capture (RSSI=%.1f dBm, +%u suppressed in last %lld ms)",
                                 (unsigned)readLen, rssi, (unsigned)sNoFrameCount,
                                 (long long)((now - sLastNoFrameLogUs) / 1000));
                        sLastNoFrameLogUs = now;
                        sNoFrameCount = 0;
                    }
                    else
                    {
                        sNoFrameCount++;
                    }
                }
            }
            else
            {
                // Generic / non-IoHome path: deliver chip-level bytes verbatim and let upper layers
                // sort it out. This is what the legacy code did and we keep it for non-IoHome users
                // (e.g. anyone using this driver as a generic SX126x GFSK transceiver).
                if (mCallback != nullptr)
                {
                    int64_t timeSincePreamble = esp_timer_get_time() - mLastPreambleDetectedTime;
                    mCallback(readLen, rawBuf, rxFreq, rssi, timeSincePreamble);
                }
            }
#endif

            // Restart receive automatically since IO-Homecontrol typically uses continuous listening.
            if (mIsReceiveMode)
            {
                // Re-arming is the critical path: if we don't reacquire the mutex here we leave
                // the chip in standby and RX is OFF until the next explicit StartReceive() call.
                // The only competitor for the mutex right now is the iohome hop task running
                // SetFrequency, which holds the mutex for ~1 ms; MUTEX_MAX_WAIT_TICKS (50 ms)
                // is several orders of magnitude more than necessary.
                if (xSemaphoreTake(s1262Mutex, MUTEX_MAX_WAIT_TICKS))
                {
                    applyPacketParams(mPreambleLenBits, RX_FIXED_LEN);
                    clearIrqStatus(SX126X_IRQ_ALL);
                    setRfSwitch(false, true); // antenna ON before setRx (see StartReceive)
                    if (setRx(SX126X_RX_TIMEOUT_CONTINUOUS) != RADIO_ERR_NONE)
                        setRfSwitch(false, false);
                    mPayloadLen = RX_FIXED_LEN;
                    xSemaphoreGive(s1262Mutex);
                }
                else
                {
                    // True stall - log loudly so the user notices. RX is now OFF; the upper
                    // layer's next StartReceive() (e.g. on a button press or timer kick) will
                    // recover us.
                    ESP_LOGE(TAG, "RX_DONE re-arm: mutex held >%ums, RX is STALLED",
                             (unsigned)pdTICKS_TO_MS(MUTEX_MAX_WAIT_TICKS));
                }
            }
            // Reset preamble flag so the upper layer does not see a stale preamble after a packet
            mPreambleDetectedFlag = false;
            return;
        }

        if (irqStatus & SX126X_IRQ_TX_DONE)
        {
            // PA is now off-air; park the RF switch before either re-arming RX or going to standby.
            setRfSwitch(false, false);
            if (mIsReceiveMode)
            {
                xSemaphoreGive(s1262Mutex);
                StartReceive();
            }
            else
            {
                // Park in STDBY_XOSC so the next TX/RX does not have to wait for TCXO startup.
                setStandby(SX126X_STANDBY_XOSC);
                xSemaphoreGive(s1262Mutex);
            }
            return;
        }

        if (irqStatus & SX126X_IRQ_TIMEOUT)
        {
            xSemaphoreGive(s1262Mutex);
            if (mIsReceiveMode)
                StartReceive();
            return;
        }

        // Default: just release the mutex. PREAMBLE-only and SYNC_WORD_VALID-only IRQs end up here.
        xSemaphoreGive(s1262Mutex);
    }

    // ====================================================================
    // SX126x command helpers
    // ====================================================================

    RADIO_ERRCODE RadioSX1262::sendCommand(uint8_t opcode, const uint8_t *params, size_t paramsLen)
    {
        if (waitBusy() != RADIO_ERR_NONE)
            return RADIO_ERR_BUSY;
        uint8_t txBuf[1 + 16];
        if (paramsLen > sizeof(txBuf) - 1)
            return RADIO_ERR_HARDWARE;
        txBuf[0] = opcode;
        if (paramsLen > 0 && params != nullptr)
            memcpy(txBuf + 1, params, paramsLen);
        return spiTransfer(txBuf, nullptr, paramsLen + 1);
    }

    RADIO_ERRCODE RadioSX1262::readCommand(uint8_t opcode, uint8_t *response, size_t responseLen)
    {
        if (waitBusy() != RADIO_ERR_NONE)
            return RADIO_ERR_BUSY;
        // SX126x read transactions: opcode | NOP (status) | NOP*N (data)
        uint8_t txBuf[1 + 1 + 16];
        uint8_t rxBuf[1 + 1 + 16];
        size_t total = 2 + responseLen;
        if (total > sizeof(txBuf))
            return RADIO_ERR_HARDWARE;
        memset(txBuf, 0x00, total);
        txBuf[0] = opcode;
        RADIO_ERRCODE err = spiTransfer(txBuf, rxBuf, total);
        if (err == RADIO_ERR_NONE && response != nullptr && responseLen > 0)
            memcpy(response, rxBuf + 2, responseLen);
        return err;
    }

    RADIO_ERRCODE RadioSX1262::writeRegister(uint16_t addr, const uint8_t *data, size_t len)
    {
        if (waitBusy() != RADIO_ERR_NONE)
            return RADIO_ERR_BUSY;
        uint8_t txBuf[3 + 16];
        if (len > sizeof(txBuf) - 3)
            return RADIO_ERR_HARDWARE;
        txBuf[0] = SX126X_CMD_WRITE_REGISTER;
        txBuf[1] = static_cast<uint8_t>((addr >> 8) & 0xFF);
        txBuf[2] = static_cast<uint8_t>(addr & 0xFF);
        memcpy(txBuf + 3, data, len);
        return spiTransfer(txBuf, nullptr, 3 + len);
    }

    RADIO_ERRCODE RadioSX1262::readRegister(uint16_t addr, uint8_t *data, size_t len)
    {
        if (waitBusy() != RADIO_ERR_NONE)
            return RADIO_ERR_BUSY;
        uint8_t txBuf[4 + 16];
        uint8_t rxBuf[4 + 16];
        size_t total = 4 + len;
        if (total > sizeof(txBuf))
            return RADIO_ERR_HARDWARE;
        memset(txBuf, 0x00, total);
        txBuf[0] = SX126X_CMD_READ_REGISTER;
        txBuf[1] = static_cast<uint8_t>((addr >> 8) & 0xFF);
        txBuf[2] = static_cast<uint8_t>(addr & 0xFF);
        RADIO_ERRCODE err = spiTransfer(txBuf, rxBuf, total);
        if (err == RADIO_ERR_NONE && data != nullptr && len > 0)
            memcpy(data, rxBuf + 4, len);
        return err;
    }

    // The SX126x has a 256-byte RX/TX FIFO, so any single READ_BUFFER / WRITE_BUFFER request
    // can ask for up to 256 bytes of payload. The stack buffers below are sized to cover the
    // full FIFO + the 2-3 bytes of opcode/offset/status framing the SX126x prepends. Earlier
    // versions of this driver hard-coded 64-byte payloads (a hangover from the LoRa driver),
    // which silently rejected the 240-byte sniffer reads with RADIO_ERR_HARDWARE and made every
    // capture appear as a stream of zeroes. Do NOT shrink these without updating RX_FIXED_LEN.
    static constexpr size_t kSx126xMaxBuffer = 256;

    RADIO_ERRCODE RadioSX1262::writeBuffer(uint8_t offset, const uint8_t *data, size_t len)
    {
        if (waitBusy() != RADIO_ERR_NONE)
            return RADIO_ERR_BUSY;
        if (len > kSx126xMaxBuffer)
        {
            ESP_LOGE(TAG, "writeBuffer - request too large: %u bytes (max %u)",
                     (unsigned)len, (unsigned)kSx126xMaxBuffer);
            return RADIO_ERR_HARDWARE;
        }
        uint8_t txBuf[2 + kSx126xMaxBuffer];
        txBuf[0] = SX126X_CMD_WRITE_BUFFER;
        txBuf[1] = offset;
        memcpy(txBuf + 2, data, len);
        return spiTransfer(txBuf, nullptr, 2 + len);
    }

    RADIO_ERRCODE RadioSX1262::readBuffer(uint8_t offset, uint8_t *data, size_t len)
    {
        if (waitBusy() != RADIO_ERR_NONE)
            return RADIO_ERR_BUSY;
        if (len > kSx126xMaxBuffer)
        {
            ESP_LOGE(TAG, "readBuffer - request too large: %u bytes (max %u)",
                     (unsigned)len, (unsigned)kSx126xMaxBuffer);
            return RADIO_ERR_HARDWARE;
        }
        uint8_t txBuf[3 + kSx126xMaxBuffer];
        uint8_t rxBuf[3 + kSx126xMaxBuffer];
        size_t total = 3 + len;
        memset(txBuf, 0x00, total);
        txBuf[0] = SX126X_CMD_READ_BUFFER;
        txBuf[1] = offset;
        RADIO_ERRCODE err = spiTransfer(txBuf, rxBuf, total);
        if (err == RADIO_ERR_NONE && data != nullptr && len > 0)
            memcpy(data, rxBuf + 3, len);
        return err;
    }

    // ----- typed wrappers -----

    RADIO_ERRCODE RadioSX1262::setStandby(uint8_t mode)
    {
        return sendCommand(SX126X_CMD_SET_STANDBY, &mode, 1);
    }

    RADIO_ERRCODE RadioSX1262::setPacketType(uint8_t type)
    {
        return sendCommand(SX126X_CMD_SET_PACKET_TYPE, &type, 1);
    }

    RADIO_ERRCODE RadioSX1262::setRegulatorMode(uint8_t mode)
    {
        return sendCommand(SX126X_CMD_SET_REGULATOR_MODE, &mode, 1);
    }

    RADIO_ERRCODE RadioSX1262::setRxTxFallbackMode(uint8_t fallbackMode)
    {
        return sendCommand(SX126X_CMD_SET_RX_TX_FALLBACK_MODE, &fallbackMode, 1);
    }

    RADIO_ERRCODE RadioSX1262::setDio2AsRfSwitchCtrl(bool enable)
    {
        uint8_t param = enable ? 0x01 : 0x00;
        return sendCommand(SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, &param, 1);
    }

    RADIO_ERRCODE RadioSX1262::setDio3AsTcxoCtrl(uint8_t voltage, uint32_t delay_us)
    {
        // Delay register is in 15.625 us steps. Compute (delay_us / 15.625) using only integer math:
        // delay_us / 15.625 == delay_us * 64 / 1000.
        uint32_t delay = (delay_us * 64ULL) / 1000ULL;
        uint8_t params[4] = {
            voltage,
            static_cast<uint8_t>((delay >> 16) & 0xFF),
            static_cast<uint8_t>((delay >> 8) & 0xFF),
            static_cast<uint8_t>(delay & 0xFF),
        };
        return sendCommand(SX126X_CMD_SET_DIO3_AS_TCXO_CTRL, params, sizeof(params));
    }

    RADIO_ERRCODE RadioSX1262::calibrate(uint8_t calibParam)
    {
        return sendCommand(SX126X_CMD_CALIBRATE, &calibParam, 1);
    }

    RADIO_ERRCODE RadioSX1262::calibrateImage(uint32_t frequency)
    {
        // Datasheet 13.1.12 - precomputed (freq1, freq2) pairs covering common ISM bands
        uint8_t freq1, freq2;
        if (frequency >= 902000000U)      { freq1 = 0xE1; freq2 = 0xE9; }
        else if (frequency >= 863000000U) { freq1 = 0xD7; freq2 = 0xDB; }
        else if (frequency >= 779000000U) { freq1 = 0xC1; freq2 = 0xC5; }
        else if (frequency >= 470000000U) { freq1 = 0x75; freq2 = 0x81; }
        else                              { freq1 = 0x6B; freq2 = 0x6F; }
        uint8_t params[2] = {freq1, freq2};
        return sendCommand(SX126X_CMD_CALIBRATE_IMAGE, params, sizeof(params));
    }

    RADIO_ERRCODE RadioSX1262::setPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut)
    {
        uint8_t params[4] = {paDutyCycle, hpMax, deviceSel, paLut};
        return sendCommand(SX126X_CMD_SET_PA_CONFIG, params, sizeof(params));
    }

    RADIO_ERRCODE RadioSX1262::setTxParams(uint8_t power, uint8_t rampTime)
    {
        uint8_t params[2] = {power, rampTime};
        return sendCommand(SX126X_CMD_SET_TX_PARAMS, params, sizeof(params));
    }

    RADIO_ERRCODE RadioSX1262::setBufferBaseAddress(uint8_t txBase, uint8_t rxBase)
    {
        uint8_t params[2] = {txBase, rxBase};
        return sendCommand(SX126X_CMD_SET_BUFFER_BASE_ADDRESS, params, sizeof(params));
    }

    RADIO_ERRCODE RadioSX1262::setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
    {
        uint8_t params[8] = {
            static_cast<uint8_t>((irqMask >> 8) & 0xFF), static_cast<uint8_t>(irqMask & 0xFF),
            static_cast<uint8_t>((dio1Mask >> 8) & 0xFF), static_cast<uint8_t>(dio1Mask & 0xFF),
            static_cast<uint8_t>((dio2Mask >> 8) & 0xFF), static_cast<uint8_t>(dio2Mask & 0xFF),
            static_cast<uint8_t>((dio3Mask >> 8) & 0xFF), static_cast<uint8_t>(dio3Mask & 0xFF),
        };
        return sendCommand(SX126X_CMD_SET_DIO_IRQ_PARAMS, params, sizeof(params));
    }

    RADIO_ERRCODE RadioSX1262::clearIrqStatus(uint16_t irqMask)
    {
        uint8_t params[2] = {
            static_cast<uint8_t>((irqMask >> 8) & 0xFF),
            static_cast<uint8_t>(irqMask & 0xFF),
        };
        return sendCommand(SX126X_CMD_CLEAR_IRQ_STATUS, params, sizeof(params));
    }

    RADIO_ERRCODE RadioSX1262::getIrqStatus(uint16_t &irqStatus)
    {
        uint8_t resp[2] = {};
        RADIO_ERRCODE err = readCommand(SX126X_CMD_GET_IRQ_STATUS, resp, 2);
        irqStatus = (static_cast<uint16_t>(resp[0]) << 8) | resp[1];
        return err;
    }

    RADIO_ERRCODE RadioSX1262::setRx(uint32_t timeout)
    {
        uint8_t params[3] = {
            static_cast<uint8_t>((timeout >> 16) & 0xFF),
            static_cast<uint8_t>((timeout >> 8) & 0xFF),
            static_cast<uint8_t>(timeout & 0xFF),
        };
        return sendCommand(SX126X_CMD_SET_RX, params, sizeof(params));
    }

    RADIO_ERRCODE RadioSX1262::setTx(uint32_t timeout)
    {
        uint8_t params[3] = {
            static_cast<uint8_t>((timeout >> 16) & 0xFF),
            static_cast<uint8_t>((timeout >> 8) & 0xFF),
            static_cast<uint8_t>(timeout & 0xFF),
        };
        return sendCommand(SX126X_CMD_SET_TX, params, sizeof(params));
    }

    RADIO_ERRCODE RadioSX1262::getRxBufferStatus(uint8_t &payloadLen, uint8_t &startPtr)
    {
        uint8_t resp[2] = {};
        RADIO_ERRCODE err = readCommand(SX126X_CMD_GET_RX_BUFFER_STATUS, resp, 2);
        payloadLen = resp[0];
        startPtr   = resp[1];
        return err;
    }

    RADIO_ERRCODE RadioSX1262::getPacketStatus(float &rssi)
    {
        // GFSK packet status: 3 bytes - RxStatus, RssiSync, RssiAvg
        uint8_t resp[3] = {};
        RADIO_ERRCODE err = readCommand(SX126X_CMD_GET_PACKET_STATUS, resp, 3);
        // RssiSync (resp[1]) - the actual RSSI of the sync word, expressed as -val/2 dBm.
        rssi = -static_cast<float>(resp[1]) / 2.0f;
        return err;
    }

    RADIO_ERRCODE RadioSX1262::getRssiInst(float &rssi)
    {
        // 1 byte response, value is -dBm/2.
        uint8_t resp = 0;
        RADIO_ERRCODE err = readCommand(SX126X_CMD_GET_RSSI_INST, &resp, 1);
        rssi = -static_cast<float>(resp) / 2.0f;
        return err;
    }

    RADIO_ERRCODE RadioSX1262::applyModulationParams()
    {
        // GFSK ModulationParams: br[23:16], br[15:8], br[7:0], pulseShape, bandwidth, fdev[23:16], fdev[15:8], fdev[7:0]
        uint32_t br = (32U * SX126X_FXOSC) / mBitrate;                                                // datasheet 13.4.4
        uint64_t fdev = (static_cast<uint64_t>(mFrequencyDeviation) << 25) / SX126X_FXOSC;            // datasheet 13.4.4
        uint8_t params[8] = {
            static_cast<uint8_t>((br >> 16) & 0xFF),
            static_cast<uint8_t>((br >> 8) & 0xFF),
            static_cast<uint8_t>(br & 0xFF),
            SX126X_GFSK_PULSE_NO_FILTER,
            mBandwidthReg,
            static_cast<uint8_t>((fdev >> 16) & 0xFF),
            static_cast<uint8_t>((fdev >> 8) & 0xFF),
            static_cast<uint8_t>(fdev & 0xFF),
        };
        return sendCommand(SX126X_CMD_SET_MODULATION_PARAMS, params, sizeof(params));
    }

    RADIO_ERRCODE RadioSX1262::applyPacketParams(uint16_t preambleLenBits, uint8_t payloadLen)
    {
        if (preambleLenBits == 0)
            preambleLenBits = 64; // safety
        if (payloadLen == 0)
            payloadLen = RX_FIXED_LEN;

        // Sync word size has to fit in [0..64] bits, in increments of 8
        uint8_t syncBits = mSyncWordLenBits == 0 ? 24 : mSyncWordLenBits;

#ifdef CONFIG_IOHOMECONTROL_SX1262_DIAG_SNIFFER
        // Sniffer mode: no preamble detection, capture as many bytes as we can per trigger.
        constexpr uint8_t preambleDetect = SX126X_GFSK_PREAMBLE_DETECT_OFF;
        if (payloadLen < RX_SNIFFER_LEN) payloadLen = RX_SNIFFER_LEN;
#else
        constexpr uint8_t preambleDetect = SX126X_GFSK_PREAMBLE_DETECT_16;
#endif

        // GFSK PacketParams: preamble[15:8], preamble[7:0], detect, syncWordLen, addrComp, packetType, payloadLen, crcType, whitening
        uint8_t params[9] = {
            static_cast<uint8_t>((preambleLenBits >> 8) & 0xFF),
            static_cast<uint8_t>(preambleLenBits & 0xFF),
            preambleDetect,
            syncBits,
            SX126X_GFSK_ADDRESS_FILT_OFF,
            SX126X_GFSK_PACKET_FIXED, // Fixed length: IO-Homecontrol uses ctrl_byte_0 lower 5 bits as length, so chip cannot decode it
            payloadLen,
            SX126X_GFSK_CRC_OFF, // CRC is implemented in software because the SX126x packet engine does not include the length byte in the CRC
            SX126X_GFSK_WHITENING_OFF,
        };
        return sendCommand(SX126X_CMD_SET_PACKET_PARAMS, params, sizeof(params));
    }

    // ====================================================================
    // Init details
    // ====================================================================

    RADIO_ERRCODE RadioSX1262::InitRegisters(bool /*ioMode*/)
    {
        RADIO_ERRCODE err = setStandby(SX126X_STANDBY_RC);
        if (err != RADIO_ERR_NONE)
            return err;

        if (mUseTCXO)
        {
            uint8_t voltage;
            if      (mTcxoVoltage_mv >= 3300) voltage = SX126X_DIO3_OUTPUT_3_3;
            else if (mTcxoVoltage_mv >= 3000) voltage = SX126X_DIO3_OUTPUT_3_0;
            else if (mTcxoVoltage_mv >= 2700) voltage = SX126X_DIO3_OUTPUT_2_7;
            else if (mTcxoVoltage_mv >= 2400) voltage = SX126X_DIO3_OUTPUT_2_4;
            else if (mTcxoVoltage_mv >= 2200) voltage = SX126X_DIO3_OUTPUT_2_2;
            else if (mTcxoVoltage_mv >= 1800) voltage = SX126X_DIO3_OUTPUT_1_8;
            else if (mTcxoVoltage_mv >= 1700) voltage = SX126X_DIO3_OUTPUT_1_7;
            else                              voltage = SX126X_DIO3_OUTPUT_1_6;

            // Datasheet recommends 5 ms startup, but a number of common modules (DFRobot Beetle ESP32-S3
            // LoRaWAN, some Heltec V3 revisions, ...) use cheap TCXOs that need closer to 10 ms before
            // they are stable enough to clock the PLL. We start with 10 ms and retry with progressively
            // larger delays if the chip reports XOSC_START_ERR.
            // After SetDio3AsTcxoCtrl the chip drives DIO3 high to power the TCXO and arms a guard
            // timer; the next mode change that requires XOSC (calibrate / setRx / setTx / SetStandby
            // STDBY_XOSC) will fail-fast with XOSC_START_ERR if the TCXO is still not stable.
            const uint32_t kStartupDelaysUs[] = { 10000U, 20000U, 50000U };
            bool tcxoOk = false;
            for (uint32_t delay_us : kStartupDelaysUs)
            {
                clearDeviceErrors();
                setDio3AsTcxoCtrl(voltage, delay_us);
                // Allow the chip to actually run its guard timer before we calibrate.
                vTaskDelay(pdMS_TO_TICKS((delay_us / 1000U) + 2U));
                calibrate(SX126X_CALIBRATE_ALL); // also exercises the XOSC, will set XOSC_START_ERR if it fails

                uint16_t devErrors = 0;
                if (getDeviceErrors(devErrors) == RADIO_ERR_NONE && (devErrors & 0x0020) == 0)
                {
                    ESP_LOGI(TAG, "TCXO startup OK (voltage=0x%02X, delay=%u us, devErrors=0x%04X)",
                             voltage, (unsigned)delay_us, devErrors);
                    tcxoOk = true;
                    break;
                }
                ESP_LOGW(TAG, "TCXO startup FAILED (voltage=0x%02X, delay=%u us, devErrors=0x%04X) - retrying with longer delay",
                         voltage, (unsigned)delay_us, devErrors);
            }
            if (!tcxoOk)
            {
                ESP_LOGE(TAG, "TCXO never became stable - the chip will keep using the internal RC oscillator");
                ESP_LOGE(TAG, "If your module does NOT have a TCXO, disable CONFIG_IOHOMECONTROL_SX1262_USE_TCXO");
                ESP_LOGE(TAG, "If it does, double check CONFIG_IOHOMECONTROL_SX1262_TCXO_VOLTAGE_MV (often 1800 or 3300)");
            }
        }

        if (mUseDio2RfSwitch)
            setDio2AsRfSwitchCtrl(true);

        // DC-DC regulator if available - safer default for SX1262 modules
        setRegulatorMode(SX126X_REGULATOR_DC_DC);

        // Keep the crystal alive after every TX_DONE / RX_DONE instead of falling back to STDBY_RC
        // (the chip's POR default). io-homecontrol response windows are only a few ms, well below
        // the TCXO startup time we configure for cold-start - if the chip dropped to STDBY_RC after
        // each TX it would have to re-warm the TCXO and relock the PLL on every TX->RX flip, which
        // wipes out the start of the actuator's reply. STDBY_XOSC also matches what laberning's
        // working SX1262 driver does (SET_RX_TX_FALLBACK_MODE = 0x30).
        setRxTxFallbackMode(SX126X_FALLBACK_STDBY_XOSC);

        // Default packet type GFSK; the IoHomeControl layer will call SetModulation(FSK) which is a no-op since we are already there.
        setPacketType(SX126X_PACKET_TYPE_GFSK);

        // Image calibration for the 868 MHz ISM band - this is what IO-Homecontrol uses anyway.
        calibrateImage(868000000);

        // Boost the LNA gain (slightly higher current consumption, much better sensitivity)
        uint8_t gain = SX126X_RX_GAIN_BOOSTED;
        writeRegister(SX126X_REG_RX_GAIN, &gain, 1);

        // Buffer base addresses
        setBufferBaseAddress(0x00, 0x00);

        // IRQ routing - we want everything that matters on DIO1.
        // SYNC_WORD_VALID and CRC_ERROR are not strictly needed for io-home (CRC is done in software)
        // but we route them to DIO1 too so the IRQ logging tells us whether the chip is locking on
        // the sync word at all - very useful when debugging "I see preambles but never RX_DONE".
        constexpr uint16_t irqMask = SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE
                                   | SX126X_IRQ_PREAMBLE_DETECTED | SX126X_IRQ_SYNC_WORD_VALID
                                   | SX126X_IRQ_CRC_ERROR | SX126X_IRQ_TIMEOUT;
        setDioIrqParams(irqMask, irqMask);
        clearIrqStatus(SX126X_IRQ_ALL);

        // Clear and log any device errors accumulated during the init sequence (XOSC start, PLL lock, ...).
        // Especially important on TCXO boards where a wrong DIO3 voltage / startup time leaves the chip
        // in a permanent XOSC_START_ERR state where no RF will ever work.
        uint16_t devErrors = 0;
        if (getDeviceErrors(devErrors) == RADIO_ERR_NONE)
        {
            ESP_LOGI(TAG, "Device errors after init: 0x%04X%s%s%s%s%s%s%s%s%s",
                     devErrors,
                     (devErrors & 0x0001) ? " RC64K_CALIB_ERR"   : "",
                     (devErrors & 0x0002) ? " RC13M_CALIB_ERR"   : "",
                     (devErrors & 0x0004) ? " PLL_CALIB_ERR"     : "",
                     (devErrors & 0x0008) ? " ADC_CALIB_ERR"     : "",
                     (devErrors & 0x0010) ? " IMG_CALIB_ERR"     : "",
                     (devErrors & 0x0020) ? " XOSC_START_ERR"    : "",
                     (devErrors & 0x0040) ? " PLL_LOCK_ERR"      : "",
                     (devErrors & 0x0100) ? " PA_RAMP_ERR"       : "",
                     (devErrors == 0)     ? " (none)"            : "");
            if (devErrors != 0)
                clearDeviceErrors();
        }

        // Read chip status to confirm we are sitting in STDBY_RC after init.
        uint8_t st = 0;
        if (getStatus(st) == RADIO_ERR_NONE)
            ESP_LOGI(TAG, "Chip status after init: 0x%02X (mode=%s, cmd=%s)", st, statusModeName(st), statusCmdName(st));

        return RADIO_ERR_NONE;
    }

    void RadioSX1262::DumpRegisters()
    {
        uint16_t irq = 0;
        getIrqStatus(irq);
        ESP_LOGI(TAG, "IRQ status: 0x%04X", irq);
        uint8_t sync[8];
        readRegister(SX126X_REG_SYNC_WORD_0, sync, 8);
        ESP_LOGI(TAG, "Sync word: %02X %02X %02X %02X %02X %02X %02X %02X (configured length: %u bits)",
                 sync[0], sync[1], sync[2], sync[3], sync[4], sync[5], sync[6], sync[7],
                 mSyncWordLenBits == 0 ? 24 : mSyncWordLenBits);
        uint8_t gain = 0;
        readRegister(SX126X_REG_RX_GAIN, &gain, 1);
        ESP_LOGI(TAG, "RX gain: 0x%02X", gain);
        uint8_t status = 0;
        if (getStatus(status) == RADIO_ERR_NONE)
            ESP_LOGI(TAG, "Status: 0x%02X (mode=%s, cmd=%s)", status, statusModeName(status), statusCmdName(status));
        uint16_t devErrors = 0;
        if (getDeviceErrors(devErrors) == RADIO_ERR_NONE)
            ESP_LOGI(TAG, "Device errors: 0x%04X", devErrors);
        ESP_LOGI(TAG, "Cached: freq=%lu, bitrate=%lu, fdev=%lu, bwReg=0x%02X, preambleBits=%u, fixedLen=%u",
                 (unsigned long)mCurrentFrequency, (unsigned long)mBitrate,
                 (unsigned long)mFrequencyDeviation, mBandwidthReg,
                 mPreambleLenBits, mPayloadLen);
    }

    RADIO_ERRCODE RadioSX1262::getStatus(uint8_t &status)
    {
        // GetStatus is special: 1 NOP byte returns the status. Use a generic 1-byte read.
        if (waitBusy() != RADIO_ERR_NONE)
            return RADIO_ERR_BUSY;
        uint8_t txBuf[2] = {SX126X_CMD_GET_STATUS, 0x00};
        uint8_t rxBuf[2] = {};
        RADIO_ERRCODE err = spiTransfer(txBuf, rxBuf, sizeof(txBuf));
        if (err == RADIO_ERR_NONE)
            status = rxBuf[1];
        return err;
    }

    RADIO_ERRCODE RadioSX1262::getDeviceErrors(uint16_t &opErrors)
    {
        uint8_t resp[2] = {};
        RADIO_ERRCODE err = readCommand(SX126X_CMD_GET_DEVICE_ERRORS, resp, 2);
        opErrors = (static_cast<uint16_t>(resp[0]) << 8) | resp[1];
        return err;
    }

    RADIO_ERRCODE RadioSX1262::clearDeviceErrors()
    {
        uint8_t params[2] = {0x00, 0x00};
        return sendCommand(SX126X_CMD_CLEAR_DEVICE_ERRORS, params, sizeof(params));
    }

    const char *RadioSX1262::statusModeName(uint8_t status)
    {
        switch ((status >> 4) & 0x07)
        {
        case 0x02: return "STDBY_RC";
        case 0x03: return "STDBY_XOSC";
        case 0x04: return "FS";
        case 0x05: return "RX";
        case 0x06: return "TX";
        default:   return "???";
        }
    }

    const char *RadioSX1262::statusCmdName(uint8_t status)
    {
        switch ((status >> 1) & 0x07)
        {
        case 0x02: return "DATA_AVAILABLE";
        case 0x03: return "CMD_TIMEOUT";
        case 0x04: return "CMD_ERROR";
        case 0x05: return "EXEC_FAIL";
        case 0x06: return "TX_DONE";
        default:   return "RESERVED";
        }
    }

    // ====================================================================
    // GPIO / SPI helpers
    // ====================================================================

    void RadioSX1262::pinMode(int pin, gpio_mode_t mode, gpio_pullup_t pullup, gpio_pulldown_t pulldown)
    {
        if (pin == GPIO_NUM_NC)
            return;
        gpio_config_t cfg = {};
        cfg.pin_bit_mask = (1ULL << pin);
        cfg.mode = mode;
        cfg.pull_up_en = pullup;
        cfg.pull_down_en = pulldown;
        cfg.intr_type = GPIO_INTR_DISABLE;
        esp_err_t err = gpio_config(&cfg);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "pinMode - gpio_config failed: %s", esp_err_to_name(err));
        }
    }

    RADIO_ERRCODE RadioSX1262::digitalWrite(int pin, uint32_t value)
    {
        if (pin == GPIO_NUM_NC)
            return RADIO_ERR_INVALID_GPIO;
        return (gpio_set_level(static_cast<gpio_num_t>(pin), value != 0) != ESP_OK) ? RADIO_ERR_HARDWARE : RADIO_ERR_NONE;
    }

    uint32_t RadioSX1262::digitalRead(int pin)
    {
        if (pin == GPIO_NUM_NC)
            return 0;
        return static_cast<uint32_t>(gpio_get_level(static_cast<gpio_num_t>(pin)));
    }

    RADIO_ERRCODE RadioSX1262::waitBusy(uint32_t timeout_ms)
    {
        if (mIoBUSY == GPIO_NUM_NC)
            return RADIO_ERR_NONE;
        uint32_t elapsed = 0;
        while (digitalRead(mIoBUSY) != 0)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
            elapsed++;
            if (elapsed >= timeout_ms)
            {
                ESP_LOGW(TAG, "waitBusy - timeout");
                return RADIO_ERR_BUSY;
            }
        }
        return RADIO_ERR_NONE;
    }

    void RadioSX1262::spiBegin()
    {
        spi_device_interface_config_t dev = {};
        dev.command_bits   = 0;
        dev.address_bits   = 0;
        dev.mode           = 0;
        dev.clock_speed_hz = SPI_MASTER_FREQ_8M; // SX126x supports up to 16 MHz, 8 MHz is a comfortable conservative choice
        dev.spics_io_num   = mSpiCS;
        dev.queue_size     = 1;

        esp_err_t err = spi_bus_add_device(mSpiHost, &dev, &mSpiHandle);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "spi_bus_add_device: %s", esp_err_to_name(err));
        }
    }

    void RadioSX1262::spiEnd()
    {
        if (mSpiHandle)
        {
            (void)spi_bus_remove_device(mSpiHandle);
            mSpiHandle = nullptr;
        }
    }

    RADIO_ERRCODE RadioSX1262::spiTransfer(const uint8_t *tx, uint8_t *rx, size_t len)
    {
        if (mSpiHandle == nullptr)
            return RADIO_ERR_NOT_INITIALIZED;
        if (len == 0)
            return RADIO_ERR_NONE;

        esp_err_t err = spi_device_acquire_bus(mSpiHandle, portMAX_DELAY);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "spiTransfer - acquire failed: %s", esp_err_to_name(err));
            return RADIO_ERR_HARDWARE;
        }

        spi_transaction_t t = {};
        t.length    = len * 8; // bits
        t.rxlength  = (rx != nullptr) ? len * 8 : 0;
        t.tx_buffer = tx;
        t.rx_buffer = rx;
        err = spi_device_polling_transmit(mSpiHandle, &t);

        spi_device_release_bus(mSpiHandle);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "spiTransfer - transmit failed: %s", esp_err_to_name(err));
            return RADIO_ERR_HARDWARE;
        }
        return RADIO_ERR_NONE;
    }

    // ====================================================================
    // Static helpers
    // ====================================================================

    uint8_t RadioSX1262::bandwidthHzToReg(uint32_t bandwidth)
    {
        // Pick the smallest SX126x bandwidth that is >= requested value, so that we never under-specify.
        struct Entry
        {
            uint32_t hz;
            uint8_t reg;
        };
        static const Entry table[] = {
            {4800, SX126X_GFSK_RX_BW_4800},
            {5800, SX126X_GFSK_RX_BW_5800},
            {7300, SX126X_GFSK_RX_BW_7300},
            {9700, SX126X_GFSK_RX_BW_9700},
            {11700, SX126X_GFSK_RX_BW_11700},
            {14600, SX126X_GFSK_RX_BW_14600},
            {19500, SX126X_GFSK_RX_BW_19500},
            {23400, SX126X_GFSK_RX_BW_23400},
            {29300, SX126X_GFSK_RX_BW_29300},
            {39000, SX126X_GFSK_RX_BW_39000},
            {46900, SX126X_GFSK_RX_BW_46900},
            {58600, SX126X_GFSK_RX_BW_58600},
            {78200, SX126X_GFSK_RX_BW_78200},
            {93800, SX126X_GFSK_RX_BW_93800},
            {117300, SX126X_GFSK_RX_BW_117300},
            {156200, SX126X_GFSK_RX_BW_156200},
            {187200, SX126X_GFSK_RX_BW_187200},
            {234300, SX126X_GFSK_RX_BW_234300},
            {312000, SX126X_GFSK_RX_BW_312000},
            {373600, SX126X_GFSK_RX_BW_373600},
            {467000, SX126X_GFSK_RX_BW_467000},
        };
        for (const auto &e : table)
        {
            if (e.hz >= bandwidth)
                return e.reg;
        }
        return SX126X_GFSK_RX_BW_467000;
    }

    uint16_t RadioSX1262::crcKermit(const uint8_t *data, size_t len)
    {
        // CRC-CCITT KERMIT: poly 0x8408 (= bit-reversed 0x1021), init 0x0000, reflected/LSB-first
        // input, no XOR-out. This matches what an SX1276 in IoHomeOn mode produces on-air and what
        // laberning/home_io_control's working SX1262 driver (proto_frame.cpp) computes in software.
        uint16_t crc = 0x0000;
        for (size_t i = 0; i < len; i++)
        {
            crc ^= data[i];
            for (int b = 0; b < 8; b++)
                crc = (crc & 0x0001) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408) : static_cast<uint16_t>(crc >> 1);
        }
        return crc;
    }

    // ====================================================================
    // UART-on-GFSK encoding (RX/TX) for IO-Homecontrol
    // ====================================================================
    //
    // The IO-Homecontrol PHY wraps every protocol byte in a 10-bit UART frame on top of GFSK:
    //   start (0) | data bit 0 (LSB) | data bit 1 | ... | data bit 7 (MSB) | stop (1)
    //
    // The chip's bit clock runs at 38.4 kbps and clocks chip bits MSB-first into bytes. Hence one
    // protocol byte takes 10 chip bits = 1.25 chip bytes, and protocol/chip byte boundaries do not
    // line up. On TX we pack the encoded bit stream MSB-first into the SX126x FIFO (matching how
    // the chip will then clock those bits out on the air). On RX we have to undo that: the chip
    // dumps its captured bit stream as MSB-first chip bytes, and we slide a UART decoder over it
    // until start/stop bits validate cleanly.
    //
    // Because preamble lock can land us anywhere within the first UART byte, RX tries every bit
    // offset in [0..UART_PROBE_MAX_BIT_OFFSET) and keeps the longest valid decode.

    static inline uint8_t getBitMsb(const uint8_t *data, size_t bitPos)
    {
        return static_cast<uint8_t>((data[bitPos / 8] >> (7 - (bitPos % 8))) & 0x01);
    }

    static inline void setBitMsb(uint8_t *data, size_t bitPos)
    {
        data[bitPos / 8] |= static_cast<uint8_t>(1U << (7 - (bitPos % 8)));
    }

    size_t RadioSX1262::uartEncode(const uint8_t *data, size_t len, uint8_t *encoded, size_t encodedMaxLen)
    {
        if (data == nullptr || encoded == nullptr || len == 0 || encodedMaxLen == 0)
            return 0;
        const size_t totalBits = len * UART_BITS_PER_BYTE;
        const size_t totalBytes = (totalBits + 7) / 8;
        if (totalBytes > encodedMaxLen)
            return 0;

        memset(encoded, 0, totalBytes);

        size_t bitPos = 0;
        for (size_t i = 0; i < len; i++)
        {
            const uint8_t value = data[i];
            // start bit = 0 (skip)
            bitPos++;
            for (uint8_t b = 0; b < 8; b++)
            {
                if (value & (1U << b))
                    setBitMsb(encoded, bitPos);
                bitPos++;
            }
            // stop bit = 1
            setBitMsb(encoded, bitPos);
            bitPos++;
        }
        return totalBytes;
    }

    size_t RadioSX1262::uartDecode(const uint8_t *raw, size_t rawLen, uint8_t bitOffset,
                                   uint8_t *decoded, size_t decodedMaxLen)
    {
        if (raw == nullptr || decoded == nullptr || rawLen == 0 || decodedMaxLen == 0)
            return 0;
        size_t bitPos = bitOffset;
        const size_t totalBits = rawLen * 8;
        size_t decodedLen = 0;

        while (bitPos + UART_BITS_PER_BYTE <= totalBits && decodedLen < decodedMaxLen)
        {
            // start must be 0, stop must be 1; anything else means we stepped off the framing
            if (getBitMsb(raw, bitPos) != 0 || getBitMsb(raw, bitPos + 9) != 1)
                break;

            uint8_t value = 0;
            for (uint8_t i = 0; i < 8; i++)
                value = static_cast<uint8_t>(value | (getBitMsb(raw, bitPos + 1 + i) << i));

            decoded[decodedLen++] = value;
            bitPos += UART_BITS_PER_BYTE;
        }

        return decodedLen;
    }

    bool RadioSX1262::findUartFrame(const uint8_t *raw, size_t rawLen,
                                    uint8_t *frameOut, size_t frameOutMax, size_t &frameOutLen) const
    {
        frameOutLen = 0;
        if (raw == nullptr || frameOut == nullptr || rawLen == 0)
            return false;

        for (uint8_t bitOffset = 0; bitOffset < UART_PROBE_MAX_BIT_OFFSET; bitOffset++)
        {
            uint8_t decoded[IOHOME_MAX_FRAME + IOHOME_CRC_LEN + 8] = {0};
            const size_t decodedLen = uartDecode(raw, rawLen, bitOffset, decoded, sizeof(decoded));
            // Need at least minimum frame (9 header bytes) + 2 CRC bytes to make a CRC check meaningful.
            if (decodedLen < 9 + IOHOME_CRC_LEN)
                continue;

            // Slide a candidate frame start through the decoded buffer. The IO-Homecontrol sync
            // 55 FF 33 occasionally surfaces in the decoded stream when capture starts before the
            // sync (e.g. on a low-RSSI re-trigger); accept frames at any start position and use the
            // CRC as the source of truth.
            for (size_t start = 0; start + 9 + IOHOME_CRC_LEN <= decodedLen; start++)
            {
                // Skip the sync bytes themselves so a UART decode that includes the trailing 0x33
                // of the sync (because the chip-level capture began a few bits earlier) doesn't
                // get parsed as the frame header.
                const uint8_t ctrl0 = decoded[start];
                const uint8_t lengthFromCtrl0 = static_cast<uint8_t>((ctrl0 & 0x1F) + 1);
                if (lengthFromCtrl0 < 9 || lengthFromCtrl0 > IOHOME_MAX_FRAME)
                    continue;
                if (start + lengthFromCtrl0 + IOHOME_CRC_LEN > decodedLen)
                    continue;

                const uint16_t crcRx = static_cast<uint16_t>(decoded[start + lengthFromCtrl0]) |
                                       static_cast<uint16_t>(decoded[start + lengthFromCtrl0 + 1] << 8);
                const uint16_t crcCalc = crcKermit(decoded + start, lengthFromCtrl0);
                if (crcRx != crcCalc)
                    continue;

                if (lengthFromCtrl0 > frameOutMax)
                    return false;
                memcpy(frameOut, decoded + start, lengthFromCtrl0);
                frameOutLen = lengthFromCtrl0;
                return true;
            }
        }
        return false;
    }
}
