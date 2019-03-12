/**
 ******************************************************************************
 * @file    mxos_rtos_common.c
 * @author  William Xu
 * @version V1.0.0
 * @date    22-Aug-2016
 * @brief   Definitions of the MXOS Common RTOS abstraction layer functions
 ******************************************************************************
 *
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 *
 ******************************************************************************
 */

/** @file
 *
 */

#include <string.h>
#include <stdlib.h>
#include "mxos_debug.h"
#include "mxos_rtos_internal.h"
#include "platform_core.h"
#include "mxos_rtos.h"
#include "mxos_common.h"
#include "mxos_rtos_common.h"
#include "mxos_board_conf.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define CPU_CYCLES_PER_MICROSECOND    ( MCU_CLOCK_HZ / 1000000 )


/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    event_handler_t function;
    void* arg;
} mxos_event_message_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static void timed_event_handler( void* arg );

/******************************************************
 *               Variable Definitions
 ******************************************************/

mos_worker_thread_id_t mxos_hardware_io_worker_thread;
mos_worker_thread_id_t mxos_worker_thread;

/******************************************************
 *               Function Definitions
 ******************************************************/

/* Entry point for user Application */
extern int application_start( void );

merr_t mxos_rtos_init( void )
{
    merr_t result = kNoErr;

    rtos_log("Started MXOS RTOS interface for %s %s", RTOS_NAME, RTOS_VERSION );

    /*
    result = mos_worker_thread_new( MXOS_HARDWARE_IO_WORKER_THREAD, MXOS_DEFAULT_WORKER_PRIORITY, HARDWARE_IO_WORKER_THREAD_STACK_SIZE, HARDWARE_IO_WORKER_THREAD_QUEUE_SIZE );
    if ( result != kNoErr )
    {
        rtos_log("Failed to create MXOS_NETWORKING_WORKER_THREAD");
    }
    */

    result = mos_worker_thread_new( MXOS_NETWORKING_WORKER_THREAD, MXOS_NETWORK_WORKER_PRIORITY, NETWORKING_WORKER_THREAD_STACK_SIZE, NETWORKING_WORKER_THREAD_QUEUE_SIZE );
    if ( result != kNoErr )
    {
        rtos_log("Failed to create MXOS_NETWORKING_WORKER_THREAD");
    }

    return result;
}


merr_t mxos_rtos_deinit( void )
{
    merr_t result = mos_worker_thread_delete( MXOS_HARDWARE_IO_WORKER_THREAD );

    if ( result == kNoErr )
    {
        result = mos_worker_thread_delete( MXOS_NETWORKING_WORKER_THREAD );
    }

    return result;
}


merr_t mxos_rtos_delay_microseconds( uint32_t microseconds )
{
    uint32_t current_time;
    uint32_t duration;
    uint32_t elapsed_time = 0;

    current_time = platform_get_cycle_count( );
    duration     = ( microseconds * CPU_CYCLES_PER_MICROSECOND );
    while ( elapsed_time < duration )
    {
        elapsed_time = platform_get_cycle_count( ) - current_time;
    }

    return kNoErr;
}

static void worker_thread_main( void *arg )
{
    mos_worker_thread_id_t* worker_thread = (mos_worker_thread_id_t*) arg;

    while ( 1 )
    {
        mxos_event_message_t message;

        if ( mxos_rtos_pop_from_queue( &worker_thread->event_queue, &message, MXOS_WAIT_FOREVER ) == kNoErr )
        {
            message.function( message.arg );
        }
    }
}


merr_t mos_worker_thread_new( mos_worker_thread_id_t* worker_thread, uint8_t priority, uint32_t stack_size, uint32_t event_queue_size )
{
    memset( worker_thread, 0, sizeof( *worker_thread ) );

    if ( mxos_rtos_init_queue( &worker_thread->event_queue, "worker queue", sizeof(mxos_event_message_t), event_queue_size ) != kNoErr )
    {
        return kGeneralErr;
    }

    if ((worker_thread->thread = mos_thread_new( priority , "worker thread", worker_thread_main, stack_size, worker_thread )) == NULL )
    {
        mxos_rtos_deinit_queue( &worker_thread->event_queue );
        return kGeneralErr;
    }

    return kNoErr;
}

merr_t mos_worker_thread_delete( mos_worker_thread_id_t* worker_thread )
{
    if ( mos_thread_delete( worker_thread->thread ) != kNoErr )
    {
        return kGeneralErr;
    }

    if ( mxos_rtos_deinit_queue( &worker_thread->event_queue ) != kNoErr )
    {
        return kGeneralErr;
    }

    return kNoErr;
}

merr_t mxos_rtos_register_timed_event( mxos_timed_event_t* event_object, mos_worker_thread_id_t* worker_thread, event_handler_t function, uint32_t time_ms, void* arg )
{
    if( worker_thread->thread == NULL )
        return kNotInitializedErr;

    if ( mxos_rtos_init_timer( &event_object->timer, time_ms, timed_event_handler, (void*) event_object ) != kNoErr )
    {
        return kGeneralErr;
    }

    event_object->function = function;
    event_object->thread = worker_thread;
    event_object->arg = arg;

    if ( mxos_rtos_start_timer( &event_object->timer ) != kNoErr )
    {
        mxos_rtos_deinit_timer( &event_object->timer );
        return kGeneralErr;
    }

    return kNoErr;
}

merr_t mxos_rtos_deregister_timed_event( mxos_timed_event_t* event_object )
{
    if ( mxos_rtos_deinit_timer( &event_object->timer ) != kNoErr )
    {
        return kGeneralErr;
    }


    return kNoErr;
}

merr_t mxos_rtos_send_asynchronous_event( mos_worker_thread_id_t* worker_thread, event_handler_t function, void* arg )
{
    mxos_event_message_t message;

    if( worker_thread->thread == NULL )
        return kNotInitializedErr;

    message.function = function;
    message.arg = arg;

    return mxos_rtos_push_to_queue( &worker_thread->event_queue, &message, MXOS_NO_WAIT );
}

static void timed_event_handler( void* arg )
{
    mxos_timed_event_t* event_object = (mxos_timed_event_t*) arg;
    mxos_event_message_t message;

    message.function = event_object->function;
    message.arg = event_object->arg;

    mxos_rtos_push_to_queue( &event_object->thread->event_queue, &message, MXOS_NO_WAIT );
}

void mxos_rtos_thread_sleep(uint32_t seconds)
{
    mos_thread_delay(seconds*1000);

}
void mxos_rtos_thread_msleep(uint32_t mseconds)
{
    mos_thread_delay(mseconds);
}

void sleep(uint32_t seconds)
{
    mos_thread_delay(seconds*1000);
}

void msleep(uint32_t mseconds)
{
    mos_thread_delay(mseconds);
}



/////////////////////////
///
#if 1
struct mxchip_timer {
    uint32_t timeout;
    void (*handler)(void);
    struct mxchip_timer *next;
    int valid;
};
struct mxchip_timer *timer_head = NULL;
static mxos_semaphore_t timer_sem;
static int mxchip_timer_inited = 0;
static uint32_t timer_thread_wait = MXOS_NEVER_TIMEOUT;

static void timer_thread_func(void* arg);

int mxchip_timer_init(void)
{
    int ret;

    if (mxchip_timer_inited)
        return 0;
    ret = mxos_rtos_init_semaphore(&timer_sem, 1);
    if (ret != 0)
        return -1;

    if (mos_thread_new(MXOS_DEFAULT_WORKER_PRIORITY, "mxchipTimer", (mos_thread_func_t)timer_thread_func, 2048, NULL) == NULL)
        return -1;

    mxchip_timer_inited = 1;
    return 0;
}

int SetTimer(unsigned long ms, void (*psysTimerHandler)(void))
{
	struct mxchip_timer *timer, *p;

    if (mxchip_timer_inited == 0) {
        if (mxchip_timer_init() != 0)
            return -1;
    }
	timer = (struct mxchip_timer *)malloc(sizeof(struct mxchip_timer));
	if (timer == NULL)
		return -1;

	timer->timeout = mxos_rtos_get_time() + ms;
	timer->handler = psysTimerHandler;
    timer->valid = 1;
	timer->next = NULL;
    if (timer_head == NULL)
        timer_head = timer;
    else {
        p = timer_head;
        while(p->next != NULL)
            p = p->next;
        p->next = timer;
    }
    mxos_rtos_set_semaphore(&timer_sem);
    return 0;
}

/* find in the timer list, if find the same handler ignore, else create new timer */
int SetTimer_uniq(unsigned long ms, void (*psysTimerHandler)(void))
{
	struct mxchip_timer *p;

    p = timer_head;
    while(p != NULL) {
        if (p->handler == psysTimerHandler) {
            p->timeout = mxos_rtos_get_time() + ms; // update time
            p->valid = 1;
            return 0;
        } else
            p = p->next;
    }

	return SetTimer(ms, psysTimerHandler);
}

/* Remove all timers which handler is psysTimerHandler */
int UnSetTimer(void (*psysTimerHandler)(void))
{
	struct mxchip_timer *p;

    p = timer_head;
    while (p != NULL) {
        if (p->handler == psysTimerHandler) {
            p->valid = 0;
		}
        p = p->next;
    }

    return 0;
}

static void mxchip_timer_tick(void)
{
	struct mxchip_timer *p, *q;
    uint32_t next_time = MXOS_NEVER_TIMEOUT, cur_time;

    q = timer_head;
    p = timer_head;
	while (p != NULL) {
        if (next_time > p->timeout)
            next_time = p->timeout;
		if (p->timeout < mxos_rtos_get_time()) {
            if (p == timer_head) {
                timer_head = timer_head->next;
                if (p->valid == 1)
                    p->handler();
                free(p);
                p = timer_head; // time_head may be changed by handler().
                continue;
            } else {
                q->next = p->next;
                if (p->valid == 1)
                    p->handler();
    			free(p);
                break;
            }
		}
        q = p;
		p = p->next;
	}

    cur_time = mxos_rtos_get_time();
    if (next_time <= cur_time)
        timer_thread_wait = 1;
    else
        timer_thread_wait = next_time - cur_time;
	return;

}

static void timer_thread_func(void* arg)
{
    while(1) {
        mxos_rtos_get_semaphore(&timer_sem, timer_thread_wait);
        mxchip_timer_tick();
    }
}
#endif
