#include <stdint.h>

#include "mxos_rtos.h"

int mxos_debug_enabled;
mos_mutex_id_t stdio_tx_mutex;

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
    mos_thread_new(32, "user app thread", usrapp_thread, 2048, NULL);

    aos_loop_run();
}