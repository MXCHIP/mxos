/**
 ******************************************************************************
 * @file    mxos_rtos.c
 * @author  William Xu
 * @version V1.0.0
 * @date    05-Aug-2018
 * @brief   This file provide the MXOS RTOS abstract layer functions.
 ******************************************************************************
 *
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#include <string.h>
#include <stdlib.h>

#include "mxos_common.h"
#include "moc_api.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

extern const mxos_api_t *lib_api_p;

static mxos_time_t mxos_time_offset = 0;

/******************************************************
 *               Function Definitions
 ******************************************************/

/* OS Layer*/
mos_thread_id_t mos_thread_new(uint8_t priority, const char *name, mos_thread_func_t function, uint32_t stack_size, void *arg)
{
    mos_thread_id_t id;
    if (lib_api_p->mos_thread_new(&id, priority, name, function, stack_size, arg) == kNoErr)
        return id;
    else
        return NULL;
}

void mos_thread_delete(mos_thread_id_t thread)
{
    lib_api_p->mos_thread_delete(thread == NULL ? NULL : &thread);
}

void mos_thread_suspend(mos_thread_id_t thread)
{
    lib_api_p->mos_thread_suspend(&thread);
}

void mos_thread_yield(void)
{
    mos_sleep(0);
}

void mxos_rtos_suspend_all_thread(void)
{
    lib_api_p->mxos_rtos_suspend_all_thread();
}

long mxos_rtos_resume_all_thread(void)
{
    return lib_api_p->mxos_rtos_resume_all_thread();
}

void mos_thread_join(mos_thread_id_t id)
{
    lib_api_p->mos_thread_join(&id);
}

void mos_thread_awake(mos_thread_id_t id)
{
    lib_api_p->mos_thread_awake(&id);
}

bool mxos_rtos_is_current_thread(mos_thread_id_t *thread)
{
    return lib_api_p->mxos_rtos_is_current_thread(thread);
}

mos_semphr_id_t mos_semphr_new(uint32_t count)
{
    mos_semphr_id_t id = NULL;
    lib_api_p->mos_semphr_new(&id, count);
    return id;
}

merr_t mos_semphr_release(mos_semphr_id_t id)
{
    return lib_api_p->mos_semphr_release(&id);
}

merr_t mos_semphr_acquire(mos_semphr_id_t id, uint32_t timeout)
{
    return lib_api_p->mos_semphr_acquire(&id, timeout);
}

void mos_semphr_delete(mos_semphr_id_t id)
{
    lib_api_p->mos_semphr_delete(&id);
}

mos_mutex_id_t mos_mutex_new(void)
{
    mos_mutex_id_t id = NULL;
    lib_api_p->mos_mutex_new(&id);
    return id;
}

void mos_mutex_lock(mos_mutex_id_t id)
{
    lib_api_p->mos_mutex_lock(&id);
}

void mos_mutex_unlock(mos_mutex_id_t id)
{
    lib_api_p->mos_mutex_unlock(&id);
}

void mos_mutex_delete(mos_mutex_id_t id)
{
    lib_api_p->mos_mutex_delete(&id);
}

mos_queue_id_t mos_queue_new(uint32_t message_size, uint32_t number_of_messages)
{
    mos_queue_id_t id = NULL;
    lib_api_p->mos_queue_new(&id, NULL, message_size, number_of_messages);
    return id;
}

merr_t mos_queue_push(mos_queue_id_t id, void *message, uint32_t timeout)
{
    return lib_api_p->mos_queue_push(&id, message, timeout);
}

merr_t mos_queue_pop(mos_queue_id_t id, void *message, uint32_t timeout)
{
    return lib_api_p->mos_queue_pop(&id, message, timeout);
}

void mos_queue_delete(mos_queue_id_t id)
{
    lib_api_p->mos_queue_delete(&id);
}

typedef struct
{
    void *handle;
    mos_timer_handler_t function;
    void *arg;
} mico_timer_t;

mos_timer_id_t mos_timer_new(uint32_t timeout, mos_timer_handler_t function, bool repeat, void *arg)
{
    mos_timer_id_t timer;
    if ((timer = malloc(sizeof(mico_timer_t))) == NULL)
    {
        return NULL;
    }
    if (lib_api_p->mxos_init_timer(timer, timeout, function, arg) != kNoErr)
    {
        free(timer);
        return NULL;
    }
    return timer;
}

void mos_timer_start(mos_timer_id_t id)
{
    lib_api_p->mxos_start_timer(id);
}

void mos_timer_stop(mos_timer_id_t id)
{
    lib_api_p->mxos_stop_timer(id);
}

void mos_timer_delete(mos_timer_id_t id)
{
    lib_api_p->mos_timer_delete(id);
    free(id);
}

bool mos_timer_is_runing(mos_timer_id_t id)
{
    return lib_api_p->mxos_is_timer_running(id);
}

int mos_event_fd_new(mos_event_id_t handle)
{
    return lib_api_p->mos_event_fd_new(handle);
}

int mos_event_fd_delete(int fd)
{
    return lib_api_p->mos_event_fd_delete(fd);
}

/**
 * Gets time in milliseconds since RTOS start
 *
 * @Note: since this is only 32 bits, it will roll over every 49 days, 17 hours.
 *
 * @returns Time in milliseconds since RTOS started.
 */
mxos_time_t mos_time(void)
{
    return lib_api_p->mxos_get_time();
}

merr_t mxos_time_get_time(mxos_time_t *time_ptr)
{
    *time_ptr = lib_api_p->mxos_get_time() + mxos_time_offset;
    return kNoErr;
}

merr_t mxos_time_set_time(const mxos_time_t *time_ptr)
{
    mxos_time_offset = *time_ptr - lib_api_p->mxos_get_time();
    return kNoErr;
}

/**
 * Delay for a number of milliseconds
 *
 * Processing of this function depends on the minimum sleep
 * time resolution of the RTOS.
 * The current thread sleeps for the longest period possible which
 * is less than the delay required, then makes up the difference
 * with a tight loop
 *
 * @return merr_t : kNoErr if delay was successful
 *
 */
void mos_msleep(uint32_t num_ms)
{
    lib_api_p->mos_msleep(num_ms);
}

void mos_sleep(float seconds)
{
    mos_msleep(seconds * 1000);
}

void *mxos_malloc(size_t xWantedSize)
{
    return lib_api_p->malloc(xWantedSize);
}

void mxos_free(void *pv)
{
    lib_api_p->free(pv);
}

void *mxos_realloc(void *pv, size_t xWantedSize)
{
    return lib_api_p->realloc(pv, xWantedSize);
}
