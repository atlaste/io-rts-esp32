#include <stdio.h>
#include <string.h>

#include "HardwareConfig.hpp"
#include "NetworkHelpers.hpp"
#include "IoRtsManager.hpp"
#include "CmdLineManagement.hpp"

#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_console.h"

using namespace Helpers;

// static const char *TAG = "io-rts-esp32";

extern "C" void app_main(void)
{
    // Initialize Hardware: NVS, LittleFS, GPIO ISR, SPI bus
    esp_err_t err = Config::InitHardware();
    ESP_ERROR_CHECK(err);

    // Initialize network: Ethernet/Wifi + DHCP/Static IP + SNTP
    NetworkHelpers::InitNetwork();
    vTaskDelay(pdMS_TO_TICKS(5000));

    // Initialize Manager
    IoRts::IoRtsManager ioRtsManager = IoRts::IoRtsManager();

    // Initialize commands line tools
    init_cmdline(&ioRtsManager);

    while (true)
        vTaskDelay(pdMS_TO_TICKS(60000));
}