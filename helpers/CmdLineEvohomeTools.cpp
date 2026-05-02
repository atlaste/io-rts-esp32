/**
 * @file CmdLineEvohomeTools.cpp
 * @brief Console commands for the RAMSES-II (Evohome / Itho) sniffer.
 *
 * Initial commands:
 *   ev_sniff start         - reconfigure radio for RAMSES, start listening
 *   ev_sniff stop          - stop listening (radio is left in standby)
 *   ev_status              - print sniffer counters + registered codecs
 *
 * Switching back to io-homecontrol after a sniff session currently requires
 * a `reboot`. Hot-swap is a follow-up task.
 */

#include "CmdLineManagement.hpp"
#include "EvohomeRamses.hpp"

#include "argtable3/argtable3.h"
#include "esp_console.h"
#include "esp_log.h"

#include <string.h>

static const char *TAG = "cmdline_ev";

static IoRts::IoRtsManager *sIoRts;
static evohome::EvohomeRamses *sEvohome;

// ********************** ev_sniff start | stop **********************

static struct
{
    struct arg_str *action; // positional: "start" or "stop"
    struct arg_end *end;
} sniff_args;

static int do_evsniff_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&sniff_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, sniff_args.end, argv[0]);
        return 1;
    }
    if (sEvohome == nullptr)
    {
        ESP_LOGE(TAG, "ev_sniff: EvohomeRamses not initialised");
        return 1;
    }
    if (sniff_args.action->count == 0)
    {
        ESP_LOGI(TAG, "ev_sniff: state=%s", sEvohome->IsSniffing() ? "ON" : "OFF");
        return 0;
    }
    const char *act = sniff_args.action->sval[0];
    if (strcmp(act, "start") == 0)
    {
        if (!sEvohome->StartSniff())
        {
            ESP_LOGE(TAG, "ev_sniff: failed to start (see prior log lines)");
            return 1;
        }
    }
    else if (strcmp(act, "stop") == 0)
    {
        sEvohome->StopSniff();
    }
    else
    {
        ESP_LOGE(TAG, "ev_sniff: unknown action '%s' (expected start|stop)", act);
        return 1;
    }
    return 0;
}

static void register_evsniff(void)
{
    sniff_args.action = arg_str0(NULL, NULL, "<start|stop>",
                                 "Action to perform; omit to query current state");
    sniff_args.end    = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command       = "ev_sniff",
        .help          = "Start/stop the RAMSES-II sniffer (Evohome / Itho on the shared radio)",
        .hint          = NULL,
        .func          = &do_evsniff_cmd,
        .argtable      = &sniff_args,
        .func_w_context = NULL,
        .context       = NULL};

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

// ********************** ev_status **********************

static int do_evstatus_cmd(int /*argc*/, char ** /*argv*/)
{
    if (sEvohome == nullptr)
    {
        printf("evohome: not initialised\n");
        return 1;
    }
    const auto &s = sEvohome->Stats();
    printf("evohome: state=%s\n", sEvohome->IsSniffing() ? "SNIFFING" : "OFF");
    printf("  raw_bursts          = %llu\n", (unsigned long long)s.raw_bursts);
    printf("  frames_decoded      = %llu\n", (unsigned long long)s.frames_decoded);
    printf("  codec_hits          = %llu\n", (unsigned long long)s.codec_hits);
    printf("  codec_misses        = %llu\n", (unsigned long long)s.codec_misses);
    printf("  failures by category:\n");
    printf("    manc_too_short    = %llu  (Manchester output < 8 bytes - severe noise)\n",
           (unsigned long long)s.manc_too_short);
    printf("    manc_truncated    = %llu  (Manchester bailed mid-frame - radio bit error)\n",
           (unsigned long long)s.manc_truncated);
    printf("    header_reserved   = %llu  (header bits 6/7 set - bit error in first byte)\n",
           (unsigned long long)s.header_reserved);
    printf("    length_mismatch   = %llu  (length byte != bytes left after header)\n",
           (unsigned long long)s.length_mismatch);
    printf("    bad_checksum      = %llu  (structurally fine, sum != 0 - bit error in payload)\n",
           (unsigned long long)s.bad_checksum);
    auto &reg = evohome::global_codec_registry();
    printf("registry: %u codec(s)\n", static_cast<unsigned>(reg.size()));
    return 0;
}

static void register_evstatus(void)
{
    const esp_console_cmd_t cmd = {
        .command       = "ev_status",
        .help          = "Print RAMSES-II sniffer counters and registry size",
        .hint          = NULL,
        .func          = &do_evstatus_cmd,
        .argtable      = NULL,
        .func_w_context = NULL,
        .context       = NULL};

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

// ********************** ev_dump on|off **********************
//
// Toggles the per-burst raw chip dump. Default is ON during bring-up so
// every captured burst is logged as hex (chip bytes + Manchester-decoded
// protocol bytes) before the frame parser ever sees it. Turn OFF once the
// codec layer is trustworthy and the volume becomes too noisy.

static struct
{
    struct arg_str *action;
    struct arg_end *end;
} dump_args;

static int do_evdump_cmd(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&dump_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, dump_args.end, argv[0]);
        return 1;
    }
    if (sEvohome == nullptr)
    {
        ESP_LOGE(TAG, "ev_dump: EvohomeRamses not initialised");
        return 1;
    }
    if (dump_args.action->count == 0)
    {
        ESP_LOGI(TAG, "ev_dump: state=%s", sEvohome->IsDumpingRaw() ? "ON" : "OFF");
        return 0;
    }
    const char *act = dump_args.action->sval[0];
    if (strcmp(act, "on") == 0)        sEvohome->SetDumpRaw(true);
    else if (strcmp(act, "off") == 0)  sEvohome->SetDumpRaw(false);
    else { ESP_LOGE(TAG, "ev_dump: unknown action '%s'", act); return 1; }
    ESP_LOGI(TAG, "ev_dump: %s", sEvohome->IsDumpingRaw() ? "ON" : "OFF");
    return 0;
}

static void register_evdump(void)
{
    dump_args.action = arg_str0(NULL, NULL, "<on|off>",
                                "Enable/disable per-burst raw RX hex dump (default ON)");
    dump_args.end    = arg_end(2);

    const esp_console_cmd_t cmd = {
        .command       = "ev_dump",
        .help          = "Toggle raw chip-byte dump from the RAMSES sniffer",
        .hint          = NULL,
        .func          = &do_evdump_cmd,
        .argtable      = &dump_args,
        .func_w_context = NULL,
        .context       = NULL};

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

// ********************** ev_codecs **********************

static int do_evcodecs_cmd(int /*argc*/, char ** /*argv*/)
{
    auto &reg = evohome::global_codec_registry();
    printf("Registered codecs (%u):\n", static_cast<unsigned>(reg.size()));
    reg.for_each([](const evohome::ICodec &c) {
        const auto k = c.key();
        printf("  %04X / %s  -> %s\n",
               static_cast<unsigned>(k.code),
               evohome::to_string(k.verb),
               c.name());
    });
    return 0;
}

static void register_evcodecs(void)
{
    const esp_console_cmd_t cmd = {
        .command       = "ev_codecs",
        .help          = "List all registered RAMSES-II codecs",
        .hint          = NULL,
        .func          = &do_evcodecs_cmd,
        .argtable      = NULL,
        .func_w_context = NULL,
        .context       = NULL};

    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

// ********************** Public init **********************

void register_evohome_cmdline_tools(IoRts::IoRtsManager *io_rts_manager)
{
    sIoRts   = io_rts_manager;
    sEvohome = io_rts_manager ? io_rts_manager->mEvohome : nullptr;
    register_evsniff();
    register_evstatus();
    register_evdump();
    register_evcodecs();
}
