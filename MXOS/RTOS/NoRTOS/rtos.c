/**
 ******************************************************************************
 * @file    rtos.c
 * @author  William Xu
 * @version V1.0.0
 * @date    25-Aug-2016
 * @brief   Definitions of the MXOS RTOS abstraction layer for the special case
 *          of having no RTOS
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

#include "mxos_common.h"
#include "mxos_rtos.h"
#include "platform_peripheral.h"

#include "portmacro.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define SEMAPHORE_POOL_NUM      8
#define MUTEX_POOL_NUM          8

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef volatile struct _noos_semaphore_t
{
    uint8_t used;
    uint8_t count;
} noos_semaphore_t;

typedef volatile struct _noos_mutex_t
{
    uint8_t used;
    uint8_t reversed;
} noos_mutex_t;

/******************************************************
 *                    Structures
 ******************************************************/


/******************************************************
 *               Function Declarations
 ******************************************************/
extern void mxos_main(void);
extern int __real_main(void);

/******************************************************
 *               Variable Definitions
 ******************************************************/


static mxos_time_t mxos_time_offset = 0;

#ifdef  MXOS_DEFAULT_TICK_RATE_HZ
uint32_t  ms_to_tick_ratio = (uint32_t)( 1000 / MXOS_DEFAULT_TICK_RATE_HZ );
#else
uint32_t  ms_to_tick_ratio = 1; // Default OS tick is 1000Hz
#endif


uint8_t semaphore_pool_init = 0;
noos_semaphore_t semaphore_pool[SEMAPHORE_POOL_NUM];

uint8_t mutex_pool_init = 0;
noos_mutex_t mutex_pool[MUTEX_POOL_NUM];

/******************************************************
 *               Function Definitions
 ******************************************************/

mos_thread_id_t mos_thread_new( uint8_t priority, const char* name, mos_thread_func_t function, uint32_t stack_size, void *arg )
{
    return NULL;
}

merr_t mos_thread_delete( mos_thread_id_t id )
{
    return kUnsupportedErr;
}

merr_t mos_thread_join( mos_thread_id_t id )
{
    return kUnsupportedErr;
}

bool mxos_rtos_is_current_thread( mos_thread_id_t* thread )
{
    return kUnsupportedErr;
}

merr_t mxos_rtos_thread_force_awake( mos_thread_id_t* thread )
{
    return kUnsupportedErr;
}


merr_t mxos_rtos_print_thread_status( char* pcWriteBuffer, int xWriteBufferLen )
{
    return kNoErr;
}

merr_t semaphore_pool_alloc( noos_semaphore_t **semaphore )
{
	if(semaphore_pool_init == 0)
	{
		for(uint8_t i=0; i<SEMAPHORE_POOL_NUM; i++)
		{
			semaphore_pool[i].used = 0;
		}
		semaphore_pool_init = 1;
	}

	for(uint8_t i=0; i<SEMAPHORE_POOL_NUM; i++)
	{
		if(semaphore_pool[i].used == 0)
		{
			semaphore_pool[i].used = 1;
			*semaphore = &semaphore_pool[i];
			return kNoErr;
		}
	}
	return kGeneralErr;
}

merr_t semaphore_pool_free( noos_semaphore_t **semaphore )
{
	if(semaphore != NULL)
	{
		if((*semaphore)->used == 1)
		{
			(*semaphore)->used = 0;
			return kNoErr;
		}
	}
	return kGeneralErr;
}

merr_t mutex_pool_alloc( noos_mutex_t **mutex )
{
	if(mutex_pool_init == 0)
	{
		for(uint8_t i=0; i<MUTEX_POOL_NUM; i++)
		{
			mutex_pool[i].used = 0;
		}
		mutex_pool_init = 1;
	}

	for(uint8_t i=0; i<MUTEX_POOL_NUM; i++)
	{
		if(mutex_pool[i].used == 0)
		{
			mutex_pool[i].used = 1;
			*mutex = &mutex_pool[i];
			return kNoErr;
		}
	}
	return kGeneralErr;
}

merr_t mutex_pool_free( noos_mutex_t **mutex )
{
	if(mutex != NULL)
	{
		if((*mutex)->used == 1)
		{
			(*mutex)->used = 0;
			return kNoErr;
		}
	}
	return kGeneralErr;
}

mos_semphr_id_t mos_semphr_new( uint32_t count )
{
    noos_semaphore_t *noos_semaphore;
    UNUSED_PARAMETER( count );
    semaphore_pool_alloc(&noos_semaphore);
    noos_semaphore->count = 0;
    return (mos_semphr_id_t)noos_semaphore;
}

merr_t mos_semphr_acquire( mos_semphr_id_t id, uint32_t timeout )
{
    noos_semaphore_t *noos_semaphore = (noos_semaphore_t *)id;
    int delay_start;

    if( noos_semaphore == NULL)
        return kNotInitializedErr;

    delay_start = mos_time();
    while( noos_semaphore->count == 0){
      if(mos_time() >= delay_start + timeout && timeout != MXOS_NEVER_TIMEOUT){
        return kTimeoutErr;
      }
    }

    DISABLE_INTERRUPTS();
    noos_semaphore->count--;
    ENABLE_INTERRUPTS();

    return kNoErr;
}

merr_t mos_semphr_release( mos_semphr_id_t id )
{
    noos_semaphore_t *noos_semaphore = (noos_semaphore_t *)id;

    if( noos_semaphore == NULL)
        return kNotInitializedErr;

    DISABLE_INTERRUPTS();
    noos_semaphore->count++;
    ENABLE_INTERRUPTS();

    return kNoErr;
}

merr_t mos_semphr_delete( mos_semphr_id_t id )
{
    noos_semaphore_t *noos_semaphore = (noos_semaphore_t *)id;

    if( noos_semaphore == NULL)
        return kNotInitializedErr;

    semaphore_pool_free(&noos_semaphore);

    return kNoErr;
}


mos_mutex_id_t mos_mutex_new( void )
{
    noos_mutex_t *noos_mutex;
    mutex_pool_alloc(&noos_mutex);
    noos_mutex->reversed = 0;
    return (mos_mutex_id_t)noos_mutex;
}


merr_t mos_mutex_lock( mos_mutex_id_t id )
{
    UNUSED_PARAMETER(id);
    return kNoErr;
}

merr_t mos_mutex_unlock( mos_mutex_id_t id )
{
    UNUSED_PARAMETER(id);
    return kNoErr;
}

merr_t mos_mutex_delete( mos_mutex_id_t id )
{
    noos_mutex_t *noos_mutex = (noos_mutex_t *)id;

    if( noos_mutex == NULL)
        return kNotInitializedErr;

    mutex_pool_free(&noos_mutex);
    return kNoErr;
}

/**
 * Delay for a number of milliseconds
 *
 * Simply implemented with a tight loop
 *
 * @return merr_t : kNoErr if delay was successful
 *
 */
merr_t mos_thread_delay( uint32_t num_ms )
{
    mxos_time_t start = mos_time( );

    while ( ( mos_time( ) - start ) < num_ms )
    {
        /* do nothing */
    }

    return kNoErr;
}

void mxos_rtos_enter_critical( void )
{
}

void mxos_rtos_exit_critical( void )
{
}

mos_queue_id_t mos_queue_new( uint32_t message_size, uint32_t number_of_messages )
{
    return NULL;
}

merr_t mos_queue_push( mos_queue_id_t id, void* message, uint32_t timeout_ms )
{
    return kUnsupportedErr;
}


merr_t mos_queue_push_front( mos_queue_id_t* queue, void* message, uint32_t timeout_ms )
{
    return kUnsupportedErr;
}


merr_t mos_queue_pop( mos_queue_id_t id, void* message, uint32_t timeout_ms )
{
    return kUnsupportedErr;
}


merr_t mos_queue_delete( mos_queue_id_t id )
{
    return kUnsupportedErr;
}

bool mxos_rtos_is_queue_empty( mos_queue_id_t* queue )
{
    return kUnsupportedErr;
}

bool mxos_rtos_is_queue_full( mos_queue_id_t* queue )
{
    return kUnsupportedErr;
}


merr_t mos_timer_new( mxos_timer_t* timer, uint32_t time_ms, timer_handler_t function, void* arg )
{
    return kUnsupportedErr;
}

merr_t mos_timer_start( mxos_timer_t* timer )
{
    return kUnsupportedErr;
}

merr_t mos_timer_stop( mxos_timer_t* timer )
{
    return kUnsupportedErr;
}

merr_t mxos_rtos_reload_timer( mxos_timer_t* timer )
{
    return kUnsupportedErr;
}

merr_t mos_timer_delete( mxos_timer_t* timer )
{
    return kUnsupportedErr;
}

merr_t mxos_time_get_time(mxos_time_t* time_ptr)
{
    mxos_time_t _time = mos_time();
    *time_ptr = _time + mxos_time_offset;
    return kNoErr;
}

merr_t mxos_time_set_time(mxos_time_t* time_ptr)
{
    mxos_time_offset = *time_ptr - mos_time();
    return kNoErr;
}


