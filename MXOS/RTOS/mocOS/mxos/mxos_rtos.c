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
OSStatus mxos_rtos_create_thread( mxos_thread_t* thread, uint8_t priority, const char* name, mxos_thread_function_t function, uint32_t stack_size, mxos_thread_arg_t arg )
{
    return lib_api_p->mxos_rtos_create_thread( thread, priority, name, function, stack_size, (void *)arg );
}

OSStatus mxos_rtos_delete_thread( mxos_thread_t* thread )
{
    return lib_api_p->mxos_rtos_delete_thread( thread );
}

void mxos_rtos_suspend_thread(mxos_thread_t* thread)
{
    lib_api_p->mxos_rtos_suspend_thread(thread);
}

void mxos_rtos_thread_yield(void)
{
    mxos_rtos_delay_milliseconds(0);
}

void mxos_rtos_suspend_all_thread(void)
{
    lib_api_p->mxos_rtos_suspend_all_thread();
}

long mxos_rtos_resume_all_thread(void)
{
    return lib_api_p->mxos_rtos_resume_all_thread();
}

OSStatus mxos_rtos_thread_join( mxos_thread_t* thread )
{
    return lib_api_p->mxos_rtos_thread_join(thread);
}

OSStatus mxos_rtos_thread_force_awake( mxos_thread_t* thread )
{
    return lib_api_p->mxos_rtos_thread_force_awake(thread);
}

bool mxos_rtos_is_current_thread( mxos_thread_t* thread )
{
    return lib_api_p->mxos_rtos_is_current_thread(thread);
}

OSStatus mxos_rtos_init_semaphore( mxos_semaphore_t* semaphore, int count )
{
    return lib_api_p->mxos_rtos_init_semaphore(semaphore, count);
}
OSStatus mxos_rtos_set_semaphore( mxos_semaphore_t* semaphore )
{
    return lib_api_p->mxos_rtos_set_semaphore(semaphore);
}
OSStatus mxos_rtos_get_semaphore( mxos_semaphore_t* semaphore, uint32_t timeout_ms )
{
    return lib_api_p->mxos_rtos_get_semaphore(semaphore, timeout_ms);
}
OSStatus mxos_rtos_deinit_semaphore( mxos_semaphore_t* semaphore )
{
    return lib_api_p->mxos_rtos_deinit_semaphore(semaphore);
}
OSStatus mxos_rtos_init_mutex( mxos_mutex_t* mutex )
{
    return lib_api_p->mxos_rtos_init_mutex( mutex );
}
OSStatus mxos_rtos_lock_mutex( mxos_mutex_t* mutex )
{
    return lib_api_p->mxos_rtos_lock_mutex( mutex );
}
OSStatus mxos_rtos_unlock_mutex( mxos_mutex_t* mutex )
{
    return lib_api_p->mxos_rtos_unlock_mutex( mutex );
}
OSStatus mxos_rtos_deinit_mutex( mxos_mutex_t* mutex )
{
    return lib_api_p->mxos_rtos_deinit_mutex( mutex );
}
OSStatus mxos_rtos_init_queue( mxos_queue_t* queue, const char* name, uint32_t message_size, uint32_t number_of_messages )
{
    return lib_api_p->mxos_rtos_init_queue( queue, name, message_size, number_of_messages );
}
OSStatus mxos_rtos_push_to_queue( mxos_queue_t* queue, void* message, uint32_t timeout_ms )
{
    return lib_api_p->mxos_rtos_push_to_queue( queue, message, timeout_ms );
}
OSStatus mxos_rtos_pop_from_queue( mxos_queue_t* queue, void* message, uint32_t timeout_ms )
{
    return lib_api_p->mxos_rtos_pop_from_queue( queue, message, timeout_ms );
}
OSStatus mxos_rtos_deinit_queue( mxos_queue_t* queue )
{
    return lib_api_p->mxos_rtos_deinit_queue( queue );
}
bool mxos_rtos_is_queue_empty( mxos_queue_t* queue )
{
    return lib_api_p->mxos_rtos_is_queue_empty( queue );
}
bool mxos_rtos_is_queue_full( mxos_queue_t* queue )
{
    return lib_api_p->mxos_rtos_is_queue_full( queue );
}

OSStatus mxos_rtos_init_timer( mxos_timer_t* timer, uint32_t time_ms, timer_handler_t function, void* arg )
{
    return lib_api_p->mxos_init_timer( timer, time_ms, function, arg );
}
OSStatus mxos_rtos_start_timer( mxos_timer_t* timer )
{
    return lib_api_p->mxos_start_timer( timer );
}
OSStatus mxos_rtos_stop_timer( mxos_timer_t* timer )
{
    return lib_api_p->mxos_stop_timer( timer );
}
OSStatus mxos_rtos_reload_timer( mxos_timer_t* timer )
{
    return lib_api_p->mxos_reload_timer( timer );
}
OSStatus mxos_rtos_deinit_timer( mxos_timer_t* timer )
{
    return lib_api_p->mxos_deinit_timer( timer );
}
bool mxos_is_timer_running( mxos_timer_t* timer )
{
    return lib_api_p->mxos_is_timer_running( timer );
}
int mxos_create_event_fd(mxos_event_t handle)
{
    return lib_api_p->mxos_create_event_fd(handle);
}
int mxos_delete_event_fd(int fd)
{
    return lib_api_p->mxos_delete_event_fd(fd);
}

/**
 * Gets time in milliseconds since RTOS start
 *
 * @Note: since this is only 32 bits, it will roll over every 49 days, 17 hours.
 *
 * @returns Time in milliseconds since RTOS started.
 */
mxos_time_t mxos_rtos_get_time( void )
{
    return lib_api_p->mxos_get_time();
}

OSStatus mxos_time_get_time( mxos_time_t* time_ptr )
{
    *time_ptr = lib_api_p->mxos_get_time( ) + mxos_time_offset;
    return kNoErr;
}

OSStatus mxos_time_set_time( const mxos_time_t* time_ptr )
{
    mxos_time_offset = *time_ptr - lib_api_p->mxos_get_time( );
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
 * @return OSStatus : kNoErr if delay was successful
 *
 */
OSStatus mxos_rtos_delay_milliseconds( uint32_t num_ms )
{
    lib_api_p->mxos_thread_msleep(num_ms);
    return kNoErr;
}

void *mxos_malloc( size_t xWantedSize )
{
	return lib_api_p->malloc(xWantedSize);
}

void mxos_free( void *pv )
{
	lib_api_p->free(pv);
}

void *mxos_realloc( void *pv, size_t xWantedSize )
{
	return lib_api_p->realloc(pv, xWantedSize);
}

