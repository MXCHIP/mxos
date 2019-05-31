#ifndef __MOS_WORKER_H__
#define __MOS_WORKER_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "merr.h"

typedef merr_t (*event_handler_t)(void *arg);

typedef struct
{
    mos_thread_id_t thread;
    mos_queue_id_t event_queue;
} mos_worker_thread_id_t;

typedef struct
{
    event_handler_t function;
    void *arg;
    mos_timer_id_t timer;
    mos_worker_thread_id_t *thread;
} mos_worker_timed_event_t;

#define MOS_HARDWARE_IO_WORKER_THREAD ((mos_worker_thread_id_t *)&mxos_hardware_io_worker_thread)
#define MOS_NETWORKING_WORKER_THREAD ((mos_worker_thread_id_t *)&mxos_worker_thread)
#define MOS_WORKER_THREAD ((mos_worker_thread_id_t *)&mxos_worker_thread)

extern mos_worker_thread_id_t mxos_hardware_io_worker_thread;
extern mos_worker_thread_id_t mxos_worker_thread;

merr_t mos_worker_thread_new(mos_worker_thread_id_t *worker_thread, uint8_t priority, uint32_t stack_size, uint32_t event_queue_size);
merr_t mos_worker_thread_delete(mos_worker_thread_id_t *worker_thread);
merr_t mos_worker_send_async_event( mos_worker_thread_id_t* worker_thread, event_handler_t function, void* arg );
merr_t mos_worker_register_timed_event( mos_worker_timed_event_t* event_object, mos_worker_thread_id_t* worker_thread, event_handler_t function, uint32_t time_ms, void* arg );
merr_t mos_worker_deregister_timed_event( mos_worker_timed_event_t* event_object );



#endif