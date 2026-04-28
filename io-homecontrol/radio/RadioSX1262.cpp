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
constexpr TickType_t MUTEX_MAX_WAIT_TICKS    = ((TickType_t)2 * portTICK_PERIOD_MS); // 2 ms - same value as SX1276 driver

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
        ESP_LOGE(TAG, "SetFrequency - No mutex available!");
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

        // Cache - we will need it whenever we reapply packet parameters
        memset(mSyncWord, 0, sizeof(mSyncWord));
        memcpy(mSyncWord, syncWord, size);
        mSyncWordLenBits = static_cast<uint8_t>(size * 8);

        if (xSemaphoreTake(s1262Mutex, MUTEX_MAX_WAIT_TICKS))
        {
            // Sync word registers are 0x06C0 .. 0x06C7, MSB first on the air.
            // The IoHomeControl layer fills the buffer so that buffer[0] is the first byte sent on the air,
            // matching the order we feed to the SX126x sync registers.
            RADIO_ERRCODE err = writeRegister(SX126X_REG_SYNC_WORD_0, mSyncWord, 8);
            if (err == RADIO_ERR_NONE)
                err = applyPacketParams(mPreambleLenBits, mPayloadLen);
            xSemaphoreGive(s1262Mutex);
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
        // Park RF switch while we re-arm the chip; flip to RX once setRx is issued.
        setRfSwitch(false, false);
        RADIO_ERRCODE err = setStandby(SX126X_STANDBY_RC);
        if (err == RADIO_ERR_NONE)
            err = applyPacketParams(mPreambleLenBits, RX_FIXED_LEN);
        if (err == RADIO_ERR_NONE)
            err = clearIrqStatus(SX126X_IRQ_ALL);
        if (err == RADIO_ERR_NONE)
            err = setRx(SX126X_RX_TIMEOUT_CONTINUOUS);
        if (err == RADIO_ERR_NONE)
            setRfSwitch(false, true);
        xSemaphoreGive(s1262Mutex);
        mPayloadLen = RX_FIXED_LEN;
        mPreambleDetectedFlag = false;
        mIsReceiveMode = (err == RADIO_ERR_NONE);
        return err;
    }

    RADIO_ERRCODE RadioSX1262::StopReceive()
    {
        if (!xSemaphoreTake(s1262Mutex, MUTEX_MAX_WAIT_TICKS))
        {
            ESP_LOGE(TAG, "StopReceive - No mutex available!");
            return RADIO_ERR_BUSY;
        }
        RADIO_ERRCODE err = setStandby(SX126X_STANDBY_RC);
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

        // Software-compute IO-Homecontrol CRC over the whole payload (mimicking what an SX1276 in IoHomeOn mode would
        // produce in hardware). The result is appended MSB first.
        uint16_t crc = crc16Ccitt(buffer, len);

        uint8_t txBuf[IOHOME_MAX_FRAME + IOHOME_CRC_LEN];
        memcpy(txBuf, buffer, len);
        txBuf[len]     = static_cast<uint8_t>((crc >> 8) & 0xFF);
        txBuf[len + 1] = static_cast<uint8_t>(crc & 0xFF);
        uint8_t totalLen = len + IOHOME_CRC_LEN;

        if (!xSemaphoreTake(s1262Mutex, MUTEX_MAX_WAIT_TICKS))
        {
            ESP_LOGE(TAG, "Send - No mutex available!");
            return RADIO_ERR_BUSY;
        }

        // Stop everything, switch to the requested frequency, re-arm packet parameters with the exact tx length.
        // Park the RF switch while reconfiguring; flip to TX right before setTx.
        setRfSwitch(false, false);
        RADIO_ERRCODE err = setStandby(SX126X_STANDBY_RC);
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
            err = applyPacketParams(mPreambleLenBits, totalLen);
        if (err == RADIO_ERR_NONE)
            err = setBufferBaseAddress(0x00, 0x00);
        if (err == RADIO_ERR_NONE)
            err = writeBuffer(0x00, txBuf, totalLen);
        if (err == RADIO_ERR_NONE)
            err = clearIrqStatus(SX126X_IRQ_ALL);
        if (err == RADIO_ERR_NONE)
        {
            setRfSwitch(true, false); // route antenna to PA before keying TX
            err = setTx(0);            // 0 = no timeout - the packet engine returns to standby on TX_DONE
            if (err != RADIO_ERR_NONE)
                setRfSwitch(false, false);
        }

        mPayloadLen = totalLen;
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
        if (!xSemaphoreTake(s1262Mutex, MUTEX_MAX_WAIT_TICKS))
        {
            ESP_LOGE(TAG, "ManageInterrupt - Busy");
            return;
        }

        uint16_t irqStatus = 0;
        if (getIrqStatus(irqStatus) != RADIO_ERR_NONE)
        {
            xSemaphoreGive(s1262Mutex);
            return;
        }
        clearIrqStatus(irqStatus);

        if (irqStatus & SX126X_IRQ_PREAMBLE_DETECTED)
        {
            mLastPreambleDetectedTime = esp_timer_get_time();
            mPreambleDetectedFlag = true;
        }

        if (irqStatus & SX126X_IRQ_RX_DONE)
        {
            // SX1262 RX engine has stopped, antenna switch can be parked while we read the buffer.
            setRfSwitch(false, false);
            uint8_t payloadLen = 0;
            uint8_t startPtr   = 0;
            uint8_t rawBuf[RX_FIXED_LEN];
            float rssi = -255.0f;
            getRxBufferStatus(payloadLen, startPtr);
            if (payloadLen > sizeof(rawBuf))
                payloadLen = sizeof(rawBuf);
            readBuffer(startPtr, rawBuf, payloadLen);
            getPacketStatus(rssi);
            xSemaphoreGive(s1262Mutex);

            // IO-Homecontrol framing: ctrl_byte_0 lower 5 bits == frame length minus 1.
            if (payloadLen >= 9 + IOHOME_CRC_LEN)
            {
                uint8_t actualLen = (rawBuf[0] & 0x1F) + 1;
                if (actualLen >= 9 && actualLen <= IOHOME_MAX_FRAME && static_cast<size_t>(actualLen + IOHOME_CRC_LEN) <= payloadLen)
                {
                    uint16_t computedCrc = crc16Ccitt(rawBuf, actualLen);
                    uint16_t receivedCrc = (static_cast<uint16_t>(rawBuf[actualLen]) << 8) | rawBuf[actualLen + 1];
                    if (computedCrc == receivedCrc)
                    {
                        if (mCallback != nullptr)
                        {
                            int64_t timeSincePreamble = esp_timer_get_time() - mLastPreambleDetectedTime;
                            mCallback(actualLen, rawBuf, mCurrentFrequency, rssi, timeSincePreamble);
                        }
                    }
                    else
                    {
                        ESP_LOGD(TAG, "RX CRC mismatch (len=%u, computed=%04X, received=%04X)", actualLen, computedCrc, receivedCrc);
                    }
                }
                else
                {
                    ESP_LOGD(TAG, "RX invalid IO-Homecontrol length 0x%02X", rawBuf[0]);
                }
            }

            // Restart receive automatically since IO-Homecontrol typically uses continuous listening.
            if (mIsReceiveMode)
            {
                if (xSemaphoreTake(s1262Mutex, MUTEX_MAX_WAIT_TICKS))
                {
                    applyPacketParams(mPreambleLenBits, RX_FIXED_LEN);
                    clearIrqStatus(SX126X_IRQ_ALL);
                    if (setRx(SX126X_RX_TIMEOUT_CONTINUOUS) == RADIO_ERR_NONE)
                        setRfSwitch(false, true);
                    mPayloadLen = RX_FIXED_LEN;
                    xSemaphoreGive(s1262Mutex);
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
                setStandby(SX126X_STANDBY_RC);
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

        // Default: just release the mutex. PREAMBLE-only IRQs end up here.
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

    RADIO_ERRCODE RadioSX1262::writeBuffer(uint8_t offset, const uint8_t *data, size_t len)
    {
        if (waitBusy() != RADIO_ERR_NONE)
            return RADIO_ERR_BUSY;
        uint8_t txBuf[2 + 64];
        if (len > sizeof(txBuf) - 2)
            return RADIO_ERR_HARDWARE;
        txBuf[0] = SX126X_CMD_WRITE_BUFFER;
        txBuf[1] = offset;
        memcpy(txBuf + 2, data, len);
        return spiTransfer(txBuf, nullptr, 2 + len);
    }

    RADIO_ERRCODE RadioSX1262::readBuffer(uint8_t offset, uint8_t *data, size_t len)
    {
        if (waitBusy() != RADIO_ERR_NONE)
            return RADIO_ERR_BUSY;
        uint8_t txBuf[3 + 64];
        uint8_t rxBuf[3 + 64];
        size_t total = 3 + len;
        if (total > sizeof(txBuf))
            return RADIO_ERR_HARDWARE;
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

    RADIO_ERRCODE RadioSX1262::setDio2AsRfSwitchCtrl(bool enable)
    {
        uint8_t param = enable ? 0x01 : 0x00;
        return sendCommand(SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, &param, 1);
    }

    RADIO_ERRCODE RadioSX1262::setDio3AsTcxoCtrl(uint8_t voltage, uint32_t delay_us)
    {
        // Delay register is in 15.625us steps
        uint32_t delay = delay_us / 16; // approximate, the chip rounds anyway
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

        // GFSK PacketParams: preamble[15:8], preamble[7:0], detect, syncWordLen, addrComp, packetType, payloadLen, crcType, whitening
        uint8_t params[9] = {
            static_cast<uint8_t>((preambleLenBits >> 8) & 0xFF),
            static_cast<uint8_t>(preambleLenBits & 0xFF),
            SX126X_GFSK_PREAMBLE_DETECT_16,
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
            setDio3AsTcxoCtrl(voltage, 5000); // 5 ms TCXO startup time
            calibrate(SX126X_CALIBRATE_ALL);  // recommended after enabling TCXO
        }

        if (mUseDio2RfSwitch)
            setDio2AsRfSwitchCtrl(true);

        // DC-DC regulator if available - safer default for SX1262 modules
        setRegulatorMode(SX126X_REGULATOR_DC_DC);

        // Default packet type GFSK; the IoHomeControl layer will call SetModulation(FSK) which is a no-op since we are already there.
        setPacketType(SX126X_PACKET_TYPE_GFSK);

        // Image calibration for the 868 MHz ISM band - this is what IO-Homecontrol uses anyway.
        calibrateImage(868000000);

        // Boost the LNA gain (slightly higher current consumption, much better sensitivity)
        uint8_t gain = SX126X_RX_GAIN_BOOSTED;
        writeRegister(SX126X_REG_RX_GAIN, &gain, 1);

        // Buffer base addresses
        setBufferBaseAddress(0x00, 0x00);

        // IRQ routing - we want everything that matters on DIO1
        constexpr uint16_t irqMask = SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_PREAMBLE_DETECTED | SX126X_IRQ_TIMEOUT;
        setDioIrqParams(irqMask, irqMask);
        clearIrqStatus(SX126X_IRQ_ALL);

        return RADIO_ERR_NONE;
    }

    void RadioSX1262::DumpRegisters()
    {
        uint16_t irq = 0;
        getIrqStatus(irq);
        ESP_LOGI(TAG, "IRQ status: 0x%04X", irq);
        uint8_t sync[8];
        readRegister(SX126X_REG_SYNC_WORD_0, sync, 8);
        ESP_LOGI(TAG, "Sync word: %02X %02X %02X %02X %02X %02X %02X %02X",
                 sync[0], sync[1], sync[2], sync[3], sync[4], sync[5], sync[6], sync[7]);
        uint8_t gain = 0;
        readRegister(SX126X_REG_RX_GAIN, &gain, 1);
        ESP_LOGI(TAG, "RX gain: 0x%02X", gain);
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

    uint16_t RadioSX1262::crc16Ccitt(const uint8_t *data, size_t len)
    {
        // CRC-CCITT used by SX1276 IoHomeOn mode: poly 0x1021, seed 0x1D0F, no XOR-out (as documented in the SX1276 datasheet).
        // The bytes are processed MSB first.
        uint16_t crc = 0x1D0F;
        for (size_t i = 0; i < len; i++)
        {
            crc ^= static_cast<uint16_t>(data[i]) << 8;
            for (int b = 0; b < 8; b++)
            {
                if (crc & 0x8000)
                    crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
                else
                    crc = static_cast<uint16_t>(crc << 1);
            }
        }
        return crc;
    }
}
