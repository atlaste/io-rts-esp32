#pragma once

#include "driver/spi_master.h"
#include "esp_err.h"

namespace Config
{
    /// @brief Initialize hardware depending on config: NVS, LittleFS, GPIO ISR, SPI bus
    /// @return ESP_OK if no error
    esp_err_t InitHardware();

    /// @brief Get SPI host used by the radio module (SX1276 or SX1262) from configuration
    /// @return SPI host
    spi_host_device_t GetRadioSpiHost();

    /// @brief Get SPI host used by SX1276 from configuration
    /// @return SPI host
    /// @deprecated Use GetRadioSpiHost() instead - kept for backward compatibility.
    inline spi_host_device_t GetSX1276SpiHost() { return GetRadioSpiHost(); }

#ifdef CONFIG_CONNECTIVITY_CHOICE_ETH
    /// @brief Get SPI host used by W5500 Ethernet module from configuration
    /// @return SPI host
    spi_host_device_t GetEthernetSpiHost();
#endif
}