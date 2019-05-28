#include <stdint.h>

#include "mxos_rtos.h"

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
void cli_service_init(kinit_t *kinit);
void ulog_init(const char host_name[8]);
void vfs_device_init(void);
void aos_loop_init(void);
void aos_loop_run(void);
int main(int argc, const char *argv[]);

static void usrapp_thread(void *arg)
{
    mxos_debug_enabled = 1;
    stdio_tx_mutex = mos_mutex_new();

    main(0, NULL);

    mos_thread_delete(NULL);
}

void usr_main(void)
{
    vfs_init();
    cli_service_init(&kinit);
    ulog_init("A");
    vfs_device_init();
    aos_loop_init();

    mos_thread_new(32, "user app thread", usrapp_thread, 2048, NULL);

    aos_loop_run();
}