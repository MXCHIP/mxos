#ifndef __MOS_H__
#define __MOS_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "merr.h"

typedef void *mos_semphr_id_t;
typedef void *mos_mutex_id_t;
typedef void *mos_thread_id_t;
typedef void *mos_queue_id_t;
typedef void *mos_timer_id_t;
typedef void (*mos_thread_func_t)(void *arg);
typedef void (*mos_timer_handler_t)(void *arg);

#define MOS_NEVER_TIMEOUT   (0xFFFFFFFF)
#define MOS_WAIT_FOREVER    (0xFFFFFFFF)
#define MOS_NO_WAIT         (0)

typedef struct
{
    int total;    /* total space */
    int free;     /* total free space */
    int chunks;   /* number of free chunks */
    int min_free; /* maximum allocated space */
} mos_mallinfo_t;

typedef struct
{
    mos_thread_id_t thread; /* caller thread id */
    void *caller;           /* caller function address */
    uint32_t size;          /* allocate size */
    uint32_t addr;          /* allocated space address */
} mos_mallrecord_t;

// thread
mos_thread_id_t mos_thread_new(uint8_t priority, const char *name, mos_thread_func_t function, uint32_t stack_size, void *arg);
void mos_thread_delete(mos_thread_id_t id);
void mos_thread_suspend(mos_thread_id_t id);
void mos_thread_resume(mos_thread_id_t id);
void mos_thread_yield(void);
void mos_thread_join(mos_thread_id_t id);
void mos_thread_awake(mos_thread_id_t id);
mos_thread_id_t mos_thread_get_id(void);
char *mos_thread_get_name(mos_thread_id_t id);

// semaphore
mos_semphr_id_t mos_semphr_new(uint32_t count);
merr_t mos_semphr_acquire(mos_semphr_id_t id, uint32_t timeout);
merr_t mos_semphr_release(mos_semphr_id_t id);
void mos_semphr_delete(mos_semphr_id_t id);

// mutex
mos_mutex_id_t mos_mutex_new(void);
void mos_mutex_lock(mos_mutex_id_t id);
void mos_mutex_unlock(mos_mutex_id_t id);
void mos_mutex_delete(mos_mutex_id_t id);

// queue
mos_queue_id_t mos_queue_new(uint32_t message_size, uint32_t number_of_messages);
merr_t mos_queue_push(mos_queue_id_t id, void *message, uint32_t timeout_ms);
merr_t mos_queue_pop(mos_queue_id_t id, void *message, uint32_t timeout_ms);
void mos_queue_delete(mos_queue_id_t id);
uint32_t mos_queue_get_total(mos_queue_id_t id);
uint32_t mos_queue_get_free(mos_queue_id_t id);

// timer
mos_timer_id_t mos_timer_new(uint32_t time_ms, mos_timer_handler_t function, bool autoreload, void *arg);
void mos_timer_delete(mos_timer_id_t id);
void mos_timer_stop(mos_timer_id_t id);
void mos_timer_change(mos_timer_id_t id, uint32_t time_ms);
void mxos_timer_reset(mos_timer_id_t id);
bool mos_timer_is_runing(mos_timer_id_t id);

// others
uint32_t mos_time(void);
void mos_sleep(float seconds);
mos_mallinfo_t *mos_mallinfo(void);
void mos_mallrecord_show(void);

#endif