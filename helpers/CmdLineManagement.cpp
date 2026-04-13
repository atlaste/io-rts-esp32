#include "CmdLineManagement.hpp"
#include "NetworkConfig.hpp"

#include <string.h>
#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_log.h"

using namespace Config;

// static const char *TAG = "cmdline_mngt";
static IoRts::IoRtsManager *sIoRtsManager;

// ******************* INIT ********************

void init_cmdline_tools(IoRts::IoRtsManager *io_rts_manager)
{
    sIoRtsManager = io_rts_manager;
    // Initialize Command line tools
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "io-rts-esp32>";

    // install console REPL environment
#if CONFIG_ESP_CONSOLE_UART
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_CDC
    esp_console_dev_usb_cdc_config_t cdc_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&cdc_config, &repl_config, &repl));
#elif CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t usbjtag_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&usbjtag_config, &repl_config, &repl));
#endif
    register_io_cmdline_tools(sIoRtsManager);
    register_misc_cmdline_tools();
    register_network_config_cmdline_tools();
    register_mqtt_config_cmdline_tools();

    printf("\n ==============================================================\n");
    printf(" |            Steps to Use io-rts-esp32                       |\n");
    printf(" |                                                            |\n");
    printf(" |  Try 'help' to check all supported commands                |\n");
    printf(" |  Try TAB for commands auto-completion                      |\n");
    printf(" |                                                            |\n");
    printf(" ==============================================================\n\n");

    // start console REPL
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}

// ******************* REBOOT ********************

static int do_reboot_cmd(int argc, char **argv)
{
    esp_restart();
    return 0;
}

void register_reboot(void)
{
    const esp_console_cmd_t reboot_cmd = {
        .command = "reboot",
        .help = "Reboot ESP32",
        .hint = NULL,
        .func = &do_reboot_cmd,
        .argtable = NULL,
        .func_w_context = NULL,
        .context = NULL};

    ESP_ERROR_CHECK(esp_console_cmd_register(&reboot_cmd));
}

// ******************* Misc Register commands ********************

void register_misc_cmdline_tools()
{
    register_reboot();
}
