#include <stdint.h>

#include "mxos_board.h"
#include "mxos_hal/mxos_gpio.h"
#include "mxos_hal/mxos_uart.h"
#include "mxos_hal/mxos_flash.h"
#include "mos.h"
#include "command_console/mxos_cli.h"
#include "qc_test.h"

#define STAT_PIN 11
#define BOOT_PIN 12
#define EASL_PIN 13

typedef struct
{
    int argc;
    char **argv;
    bool cli_enable;
} kinit_t;

const kinit_t kinit =
    {
        .cli_enable = true,
};

int mxos_debug_enabled;
mos_mutex_id_t stdio_tx_mutex;

int32_t vfs_init(void);
void cli_service_init(const kinit_t *kinit);
void ulog_init(const char host_name[8]);
void vfs_device_init(void);
void aos_loop_init(void);
void aos_loop_run(void);
int main(int argc, const char *argv[]);

void handle_kv_cmd(char *pwbuf, int blen, int argc, char **argv);
static const struct cli_command kv_cmd = {"kv", "kv [set key value | get key | del key | seti key int_val | geti key | list]", handle_kv_cmd};

static void usrapp_thread(void *arg)
{
    mxos_debug_enabled = 1;
    stdio_tx_mutex = mos_mutex_new();

    mhal_gpio_open(STAT_PIN, INPUT_PULL_UP);
    mhal_gpio_open(BOOT_PIN, INPUT_PULL_UP);

    if (mhal_gpio_value(STAT_PIN) == false && mhal_gpio_value(BOOT_PIN) == false)
    {
        mxos_system_qc_test();
    }
    else
    {
        main(0, NULL);
    }

    mos_thread_delete(NULL);
}

void usr_main(void)
{
    vfs_init();
    cli_service_init(&kinit);
    ulog_init("A");
    vfs_device_init();
    aos_loop_init();
    cli_register_command(&kv_cmd);

    mos_thread_new(32, "user app thread", usrapp_thread, 4096, NULL);

    aos_loop_run();
}

char *mxos_get_bootloader_ver(void)
{
    return "bootloader";
}

typedef struct
{
    uint32_t dst_adr;
    uint32_t src_adr;
    uint32_t siz;
    uint16_t crc;
} ota_hdr_t;

merr_t mxos_ota_switch(uint32_t ota_len, uint16_t ota_crc)
{
    uint32_t addr = 0;
    ota_hdr_t ota_hdr =
    {
        .dst_adr = 0xA000,
        .src_adr = 0x100000,
        .siz = ota_len,
        .crc = ota_crc,
    };

    mhal_flash_write(MXOS_PARTITION_PARAMETER_1, &addr, (uint8_t *)&ota_hdr, sizeof(ota_hdr));

    return kNoErr;
}