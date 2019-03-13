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
mos_thread_id_t mos_thread_new( uint8_t priority, const char* name, mos_thread_func_t function, uint32_t stack_size, void *arg )
{
    mos_thread_id_t id;
    if (lib_api_p->mos_thread_new( &id, priority, name, function, stack_size, arg ) == kNoErr)
        return id;
    else
        return NULL;
}

merr_t mos_thread_delete( mos_thread_id_t thread )
{
    return lib_api_p->mos_thread_delete( &thread );
}

void mos_thread_suspend(mos_thread_id_t thread)
{
    lib_api_p->mos_thread_suspend(&thread);
}

void mos_thread_yield(void)
{
    mos_thread_delay(0);
}

void mxos_rtos_suspend_all_thread(void)
{
    lib_api_p->mxos_rtos_suspend_all_thread();
}

long mxos_rtos_resume_all_thread(void)
{
    return lib_api_p->mxos_rtos_resume_all_thread();
}

merr_t mos_thread_join( mos_thread_id_t id )
{
    return lib_api_p->mos_thread_join(&id);
}

merr_t mxos_rtos_thread_force_awake( mos_thread_id_t* thread )
{
    return lib_api_p->mxos_rtos_thread_force_awake(thread);
}

bool mxos_rtos_is_current_thread( mos_thread_id_t* thread )
{
    return lib_api_p->mxos_rtos_is_current_thread(thread);
}

mos_semphr_id_t mos_semphr_new( uint32_t count )
{
    mos_semphr_id_t id = NULL;
    lib_api_p->mos_semphr_new(&id, count);
    return id;
}
merr_t mos_semphr_release( mos_semphr_id_t id )
{
    return lib_api_p->mos_semphr_release(&id);
}
merr_t mos_semphr_acquire( mos_semphr_id_t id, uint32_t timeout )
{
    return lib_api_p->mos_semphr_acquire(&id, timeout);
}
merr_t mos_semphr_delete( mos_semphr_id_t id )
{
    return lib_api_p->mos_semphr_delete(&id);
}
mos_mutex_id_t mos_mutex_new( void )
{
    mos_mutex_id_t id = NULL;
    lib_api_p->mos_mutex_new( &id );
    return id;
}
merr_t mos_mutex_lock( mos_mutex_id_t id )
{
    return lib_api_p->mos_mutex_lock( &id );
}
merr_t mos_mutex_unlock( mos_mutex_id_t id )
{
    return lib_api_p->mos_mutex_unlock( &id );
}
merr_t mos_mutex_delete( mos_mutex_id_t id )
{
    return lib_api_p->mos_mutex_delete( &id );
}
mos_queue_id_t mos_queue_new( uint32_t message_size, uint32_t number_of_messages )
{
    mos_queue_id_t id = NULL;
    lib_api_p->mos_queue_new( &id, name, message_size, number_of_messages );
    return id;
}
merr_t mos_queue_push( mos_queue_id_t id, void* message, uint32_t timeout )
{
    return lib_api_p->mos_queue_push( &id, message, timeout );
}
merr_t mos_queue_pop( mos_queue_id_t id, void* message, uint32_t timeout )
{
    return lib_api_p->mos_queue_pop( &id, message, timeout );
}
merr_t mos_queue_delete( mos_queue_id_t id )
{
    return lib_api_p->mos_queue_delete( &id );
}
bool mxos_rtos_is_queue_empty( mos_queue_id_t* queue )
{
    return lib_api_p->mxos_rtos_is_queue_empty( queue );
}
bool mxos_rtos_is_queue_full( mos_queue_id_t* queue )
{
    return lib_api_p->mxos_rtos_is_queue_full( queue );
}

merr_t mxos_rtos_init_timer( mxos_timer_t* timer, uint32_t time_ms, timer_handler_t function, void* arg )
{
    return lib_api_p->mxos_init_timer( timer, time_ms, function, arg );
}
merr_t mxos_rtos_start_timer( mxos_timer_t* timer )
{
    return lib_api_p->mxos_start_timer( timer );
}
merr_t mxos_rtos_stop_timer( mxos_timer_t* timer )
{
    return lib_api_p->mxos_stop_timer( timer );
}
merr_t mxos_rtos_reload_timer( mxos_timer_t* timer )
{
    return lib_api_p->mxos_reload_timer( timer );
}
merr_t mxos_rtos_deinit_timer( mxos_timer_t* timer )
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

merr_t mxos_time_get_time( mxos_time_t* time_ptr )
{
    *time_ptr = lib_api_p->mxos_get_time( ) + mxos_time_offset;
    return kNoErr;
}

merr_t mxos_time_set_time( const mxos_time_t* time_ptr )
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
 * @return merr_t : kNoErr if delay was successful
 *
 */
merr_t mos_thread_delay( uint32_t num_ms )
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

