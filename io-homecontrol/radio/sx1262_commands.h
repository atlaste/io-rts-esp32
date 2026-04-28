/*
   SX1262 / SX126x command opcodes, IRQ flags and register addresses needed by RadioSX1262.

   References: Semtech SX1261/2 Datasheet (rev. 2.1, sections 11-13).
*/
#pragma once

#include <stdint.h>

namespace RadioLinks
{
    // ------------------------------------------------------------------
    // SX1262 SPI command opcodes
    // ------------------------------------------------------------------

    // Operational modes
    constexpr uint8_t SX126X_CMD_SET_SLEEP                 = 0x84;
    constexpr uint8_t SX126X_CMD_SET_STANDBY               = 0x80;
    constexpr uint8_t SX126X_CMD_SET_FS                    = 0xC1;
    constexpr uint8_t SX126X_CMD_SET_TX                    = 0x83;
    constexpr uint8_t SX126X_CMD_SET_RX                    = 0x82;
    constexpr uint8_t SX126X_CMD_STOP_TIMER_ON_PREAMBLE    = 0x9F;
    constexpr uint8_t SX126X_CMD_SET_RX_DUTY_CYCLE         = 0x94;
    constexpr uint8_t SX126X_CMD_SET_CAD                   = 0xC5;
    constexpr uint8_t SX126X_CMD_SET_TX_CONTINUOUS_WAVE    = 0xD1;
    constexpr uint8_t SX126X_CMD_SET_TX_INFINITE_PREAMBLE  = 0xD2;
    constexpr uint8_t SX126X_CMD_SET_REGULATOR_MODE        = 0x96;
    constexpr uint8_t SX126X_CMD_CALIBRATE                 = 0x89;
    constexpr uint8_t SX126X_CMD_CALIBRATE_IMAGE           = 0x98;
    constexpr uint8_t SX126X_CMD_SET_PA_CONFIG             = 0x95;
    constexpr uint8_t SX126X_CMD_SET_RX_TX_FALLBACK_MODE   = 0x93;

    // Registers and buffer
    constexpr uint8_t SX126X_CMD_WRITE_REGISTER            = 0x0D;
    constexpr uint8_t SX126X_CMD_READ_REGISTER             = 0x1D;
    constexpr uint8_t SX126X_CMD_WRITE_BUFFER              = 0x0E;
    constexpr uint8_t SX126X_CMD_READ_BUFFER               = 0x1E;

    // DIO and IRQ control
    constexpr uint8_t SX126X_CMD_SET_DIO_IRQ_PARAMS        = 0x08;
    constexpr uint8_t SX126X_CMD_GET_IRQ_STATUS            = 0x12;
    constexpr uint8_t SX126X_CMD_CLEAR_IRQ_STATUS          = 0x02;
    constexpr uint8_t SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL = 0x9D;
    constexpr uint8_t SX126X_CMD_SET_DIO3_AS_TCXO_CTRL     = 0x97;

    // RF parameters
    constexpr uint8_t SX126X_CMD_SET_RF_FREQUENCY          = 0x86;
    constexpr uint8_t SX126X_CMD_SET_PACKET_TYPE           = 0x8A;
    constexpr uint8_t SX126X_CMD_GET_PACKET_TYPE           = 0x11;
    constexpr uint8_t SX126X_CMD_SET_TX_PARAMS             = 0x8E;
    constexpr uint8_t SX126X_CMD_SET_MODULATION_PARAMS     = 0x8B;
    constexpr uint8_t SX126X_CMD_SET_PACKET_PARAMS         = 0x8C;
    constexpr uint8_t SX126X_CMD_SET_BUFFER_BASE_ADDRESS   = 0x8F;
    constexpr uint8_t SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT = 0xA0;

    // Status
    constexpr uint8_t SX126X_CMD_GET_STATUS                = 0xC0;
    constexpr uint8_t SX126X_CMD_GET_RX_BUFFER_STATUS      = 0x13;
    constexpr uint8_t SX126X_CMD_GET_PACKET_STATUS         = 0x14;
    constexpr uint8_t SX126X_CMD_GET_RSSI_INST             = 0x15;
    constexpr uint8_t SX126X_CMD_GET_STATS                 = 0x10;
    constexpr uint8_t SX126X_CMD_RESET_STATS               = 0x00;
    constexpr uint8_t SX126X_CMD_GET_DEVICE_ERRORS         = 0x17;
    constexpr uint8_t SX126X_CMD_CLEAR_DEVICE_ERRORS       = 0x07;

    // ------------------------------------------------------------------
    // Standby / packet type / regulator
    // ------------------------------------------------------------------

    constexpr uint8_t SX126X_STANDBY_RC                    = 0x00;
    constexpr uint8_t SX126X_STANDBY_XOSC                  = 0x01;

    constexpr uint8_t SX126X_PACKET_TYPE_GFSK              = 0x00;
    constexpr uint8_t SX126X_PACKET_TYPE_LORA              = 0x01;

    constexpr uint8_t SX126X_REGULATOR_LDO                 = 0x00;
    constexpr uint8_t SX126X_REGULATOR_DC_DC               = 0x01;

    // ------------------------------------------------------------------
    // Calibration bit-mask (CMD_CALIBRATE)
    // ------------------------------------------------------------------

    constexpr uint8_t SX126X_CALIBRATE_RC64K               = 0x01;
    constexpr uint8_t SX126X_CALIBRATE_RC13M               = 0x02;
    constexpr uint8_t SX126X_CALIBRATE_PLL                 = 0x04;
    constexpr uint8_t SX126X_CALIBRATE_ADC_PULSE           = 0x08;
    constexpr uint8_t SX126X_CALIBRATE_ADC_BULK_N          = 0x10;
    constexpr uint8_t SX126X_CALIBRATE_ADC_BULK_P          = 0x20;
    constexpr uint8_t SX126X_CALIBRATE_IMAGE               = 0x40;
    constexpr uint8_t SX126X_CALIBRATE_ALL                 = 0x7F;

    // ------------------------------------------------------------------
    // TCXO control voltage values for SetDio3AsTcxoCtrl
    // ------------------------------------------------------------------

    constexpr uint8_t SX126X_DIO3_OUTPUT_1_6               = 0x00;
    constexpr uint8_t SX126X_DIO3_OUTPUT_1_7               = 0x01;
    constexpr uint8_t SX126X_DIO3_OUTPUT_1_8               = 0x02;
    constexpr uint8_t SX126X_DIO3_OUTPUT_2_2               = 0x03;
    constexpr uint8_t SX126X_DIO3_OUTPUT_2_4               = 0x04;
    constexpr uint8_t SX126X_DIO3_OUTPUT_2_7               = 0x05;
    constexpr uint8_t SX126X_DIO3_OUTPUT_3_0               = 0x06;
    constexpr uint8_t SX126X_DIO3_OUTPUT_3_3               = 0x07;

    // ------------------------------------------------------------------
    // PA configuration helpers (SX1262 - High Power)
    // From datasheet table 13-21 "PA optimal settings":
    // For TX power = +22 dBm: paDutyCycle=0x04, hpMax=0x07
    // For TX power = +20 dBm: paDutyCycle=0x03, hpMax=0x05
    // For TX power = +17 dBm: paDutyCycle=0x02, hpMax=0x03
    // For TX power = +14 dBm: paDutyCycle=0x02, hpMax=0x02
    // ------------------------------------------------------------------

    constexpr uint8_t SX126X_PA_CONFIG_DEVICE_SX1262       = 0x00;
    constexpr uint8_t SX126X_PA_CONFIG_DEVICE_SX1261       = 0x01;
    constexpr uint8_t SX126X_PA_CONFIG_PA_LUT              = 0x01; // reserved, must be 0x01

    // ------------------------------------------------------------------
    // Ramp times (CMD_SET_TX_PARAMS)
    // ------------------------------------------------------------------

    constexpr uint8_t SX126X_PA_RAMP_10U                   = 0x00;
    constexpr uint8_t SX126X_PA_RAMP_20U                   = 0x01;
    constexpr uint8_t SX126X_PA_RAMP_40U                   = 0x02;
    constexpr uint8_t SX126X_PA_RAMP_80U                   = 0x03;
    constexpr uint8_t SX126X_PA_RAMP_200U                  = 0x04;
    constexpr uint8_t SX126X_PA_RAMP_800U                  = 0x05;
    constexpr uint8_t SX126X_PA_RAMP_1700U                 = 0x06;
    constexpr uint8_t SX126X_PA_RAMP_3400U                 = 0x07;

    // ------------------------------------------------------------------
    // GFSK Modulation parameters
    // ------------------------------------------------------------------

    constexpr uint8_t SX126X_GFSK_PULSE_NO_FILTER          = 0x00;
    constexpr uint8_t SX126X_GFSK_PULSE_BT_0_3             = 0x08;
    constexpr uint8_t SX126X_GFSK_PULSE_BT_0_5             = 0x09;
    constexpr uint8_t SX126X_GFSK_PULSE_BT_0_7             = 0x0A;
    constexpr uint8_t SX126X_GFSK_PULSE_BT_1_0             = 0x0B;

    // RX bandwidths (DSB) - as per SX1261/2 datasheet table 13-39
    constexpr uint8_t SX126X_GFSK_RX_BW_4800               = 0x1F;
    constexpr uint8_t SX126X_GFSK_RX_BW_5800               = 0x17;
    constexpr uint8_t SX126X_GFSK_RX_BW_7300               = 0x0F;
    constexpr uint8_t SX126X_GFSK_RX_BW_9700               = 0x1E;
    constexpr uint8_t SX126X_GFSK_RX_BW_11700              = 0x16;
    constexpr uint8_t SX126X_GFSK_RX_BW_14600              = 0x0E;
    constexpr uint8_t SX126X_GFSK_RX_BW_19500              = 0x1D;
    constexpr uint8_t SX126X_GFSK_RX_BW_23400              = 0x15;
    constexpr uint8_t SX126X_GFSK_RX_BW_29300              = 0x0D;
    constexpr uint8_t SX126X_GFSK_RX_BW_39000              = 0x1C;
    constexpr uint8_t SX126X_GFSK_RX_BW_46900              = 0x14;
    constexpr uint8_t SX126X_GFSK_RX_BW_58600              = 0x0C;
    constexpr uint8_t SX126X_GFSK_RX_BW_78200              = 0x1B;
    constexpr uint8_t SX126X_GFSK_RX_BW_93800              = 0x13;
    constexpr uint8_t SX126X_GFSK_RX_BW_117300             = 0x0B;
    constexpr uint8_t SX126X_GFSK_RX_BW_156200             = 0x1A;
    constexpr uint8_t SX126X_GFSK_RX_BW_187200             = 0x12;
    constexpr uint8_t SX126X_GFSK_RX_BW_234300             = 0x0A;
    constexpr uint8_t SX126X_GFSK_RX_BW_312000             = 0x19;
    constexpr uint8_t SX126X_GFSK_RX_BW_373600             = 0x11;
    constexpr uint8_t SX126X_GFSK_RX_BW_467000             = 0x09;

    // ------------------------------------------------------------------
    // GFSK Packet parameters
    // ------------------------------------------------------------------

    constexpr uint8_t SX126X_GFSK_PREAMBLE_DETECT_OFF      = 0x00;
    constexpr uint8_t SX126X_GFSK_PREAMBLE_DETECT_8        = 0x04;
    constexpr uint8_t SX126X_GFSK_PREAMBLE_DETECT_16       = 0x05;
    constexpr uint8_t SX126X_GFSK_PREAMBLE_DETECT_24       = 0x06;
    constexpr uint8_t SX126X_GFSK_PREAMBLE_DETECT_32       = 0x07;

    constexpr uint8_t SX126X_GFSK_ADDRESS_FILT_OFF         = 0x00;
    constexpr uint8_t SX126X_GFSK_ADDRESS_FILT_NODE        = 0x01;
    constexpr uint8_t SX126X_GFSK_ADDRESS_FILT_NODE_BROAD  = 0x02;

    constexpr uint8_t SX126X_GFSK_PACKET_FIXED             = 0x00;
    constexpr uint8_t SX126X_GFSK_PACKET_VARIABLE          = 0x01;

    constexpr uint8_t SX126X_GFSK_CRC_OFF                  = 0x01;
    constexpr uint8_t SX126X_GFSK_CRC_1_BYTE               = 0x00;
    constexpr uint8_t SX126X_GFSK_CRC_2_BYTE               = 0x02;
    constexpr uint8_t SX126X_GFSK_CRC_1_BYTE_INV           = 0x04;
    constexpr uint8_t SX126X_GFSK_CRC_2_BYTE_INV           = 0x06;

    constexpr uint8_t SX126X_GFSK_WHITENING_OFF            = 0x00;
    constexpr uint8_t SX126X_GFSK_WHITENING_ON             = 0x01;

    // ------------------------------------------------------------------
    // IRQ flags (16 bits)
    // ------------------------------------------------------------------

    constexpr uint16_t SX126X_IRQ_TX_DONE                  = (1U << 0);
    constexpr uint16_t SX126X_IRQ_RX_DONE                  = (1U << 1);
    constexpr uint16_t SX126X_IRQ_PREAMBLE_DETECTED        = (1U << 2);
    constexpr uint16_t SX126X_IRQ_SYNC_WORD_VALID          = (1U << 3);
    constexpr uint16_t SX126X_IRQ_HEADER_VALID             = (1U << 4);
    constexpr uint16_t SX126X_IRQ_HEADER_ERROR             = (1U << 5);
    constexpr uint16_t SX126X_IRQ_CRC_ERROR                = (1U << 6);
    constexpr uint16_t SX126X_IRQ_CAD_DONE                 = (1U << 7);
    constexpr uint16_t SX126X_IRQ_CAD_DETECTED             = (1U << 8);
    constexpr uint16_t SX126X_IRQ_TIMEOUT                  = (1U << 9);
    constexpr uint16_t SX126X_IRQ_ALL                      = 0x03FF;

    // ------------------------------------------------------------------
    // Useful registers (selection only)
    // ------------------------------------------------------------------

    constexpr uint16_t SX126X_REG_HOPPING_ENABLE           = 0x0385;
    constexpr uint16_t SX126X_REG_RX_GAIN                  = 0x08AC;
    constexpr uint16_t SX126X_REG_OCP_CONFIG               = 0x08E7;
    constexpr uint16_t SX126X_REG_XTA_TRIM                 = 0x0911;
    constexpr uint16_t SX126X_REG_XTB_TRIM                 = 0x0912;
    constexpr uint16_t SX126X_REG_SYNC_WORD_0              = 0x06C0;

    // ------------------------------------------------------------------
    // RX_GAIN values
    // ------------------------------------------------------------------

    constexpr uint8_t SX126X_RX_GAIN_BOOSTED               = 0x96;
    constexpr uint8_t SX126X_RX_GAIN_POWER_SAVING          = 0x94;

    // ------------------------------------------------------------------
    // Crystal frequency / Frf math constants
    // SX126x: Frf = (RF_FREQ * Fxtal) / 2^25, with Fxtal = 32 MHz
    // SX126x: Br_reg = 32 * Fxtal / Bitrate = 32 * 32e6 / Bitrate
    // SX126x: Fdev_reg = (Fdev_hz * 2^25) / Fxtal
    // ------------------------------------------------------------------

    constexpr uint32_t SX126X_FXOSC                        = 32000000U;
    constexpr uint32_t SX126X_RX_TIMEOUT_CONTINUOUS        = 0xFFFFFFU; // continuous receive
}
