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

OSStatus mxos_rtos_create_thread( mxos_thread_t* thread, uint8_t priority, const char* name, mxos_thread_function_t function, uint32_t stack_size, uint32_t arg )
{
    return kNoErr;
}

OSStatus mxos_rtos_delete_thread( mxos_thread_t* thread )
{
    return kUnsupportedErr;
}

OSStatus mxos_rtos_thread_join( mxos_thread_t* thread )
{
    return kUnsupportedErr;
}

bool mxos_rtos_is_current_thread( mxos_thread_t* thread )
{
    return kUnsupportedErr;
}

OSStatus mxos_rtos_thread_force_awake( mxos_thread_t* thread )
{
    return kUnsupportedErr;
}


OSStatus mxos_rtos_print_thread_status( char* pcWriteBuffer, int xWriteBufferLen )
{
    return kNoErr;
}

OSStatus semaphore_pool_alloc( noos_semaphore_t **semaphore )
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

OSStatus semaphore_pool_free( noos_semaphore_t **semaphore )
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

OSStatus mutex_pool_alloc( noos_mutex_t **mutex )
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

OSStatus mutex_pool_free( noos_mutex_t **mutex )
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

OSStatus mxos_rtos_init_semaphore( mxos_semaphore_t* semaphore, int count )
{
    noos_semaphore_t *noos_semaphore;
    UNUSED_PARAMETER( count );
    semaphore_pool_alloc(&noos_semaphore);
    noos_semaphore->count = 0;
    *semaphore = (void *)noos_semaphore;
    return kNoErr;
}

OSStatus mxos_rtos_get_semaphore( mxos_semaphore_t* semaphore, uint32_t timeout_ms )
{
    noos_semaphore_t *noos_semaphore = (noos_semaphore_t *)*semaphore;
    int delay_start;

    if( noos_semaphore == NULL)
        return kNotInitializedErr;

    delay_start = mxos_rtos_get_time();
    while( noos_semaphore->count == 0){
      if(mxos_rtos_get_time() >= delay_start + timeout_ms && timeout_ms != MXOS_NEVER_TIMEOUT){
        return kTimeoutErr;
      }
    }

    DISABLE_INTERRUPTS();
    noos_semaphore->count--;
    ENABLE_INTERRUPTS();

    return kNoErr;
}

OSStatus mxos_rtos_set_semaphore( mxos_semaphore_t* semaphore )
{
    noos_semaphore_t *noos_semaphore = (noos_semaphore_t *)*semaphore;

    if( noos_semaphore == NULL)
        return kNotInitializedErr;

    DISABLE_INTERRUPTS();
    noos_semaphore->count++;
    ENABLE_INTERRUPTS();

    return kNoErr;
}

OSStatus mxos_rtos_deinit_semaphore( mxos_semaphore_t* semaphore )
{
    noos_semaphore_t *noos_semaphore = (noos_semaphore_t *)*semaphore;

    if( noos_semaphore == NULL)
        return kNotInitializedErr;

    semaphore_pool_free(&noos_semaphore);
    *semaphore = NULL;

    return kNoErr;
}


OSStatus mxos_rtos_init_mutex( mxos_mutex_t* mutex )
{
    noos_mutex_t *noos_mutex;
    mutex_pool_alloc(&noos_mutex);
    noos_mutex->reversed = 0;
    *mutex = (void *)noos_mutex;
    return kNoErr;
}


OSStatus mxos_rtos_lock_mutex( mxos_mutex_t* mutex )
{
    UNUSED_PARAMETER(mutex);
    return kNoErr;
}

OSStatus mxos_rtos_unlock_mutex( mxos_mutex_t* mutex )
{
    UNUSED_PARAMETER(mutex);
    return kNoErr;
}

OSStatus mxos_rtos_deinit_mutex( mxos_mutex_t* mutex )
{
    noos_mutex_t *noos_mutex = (noos_mutex_t *)*mutex;

    if( noos_mutex == NULL)
        return kNotInitializedErr;

    mutex_pool_free(&noos_mutex);
    *mutex = NULL;
    return kNoErr;
}

/**
 * Delay for a number of milliseconds
 *
 * Simply implemented with a tight loop
 *
 * @return OSStatus : kNoErr if delay was successful
 *
 */
OSStatus mxos_rtos_delay_milliseconds( uint32_t num_ms )
{
    mxos_time_t start = mxos_rtos_get_time( );

    while ( ( mxos_rtos_get_time( ) - start ) < num_ms )
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

OSStatus mxos_rtos_init_queue( mxos_queue_t* queue, const char* name, uint32_t message_size, uint32_t number_of_messages )
{
    return kNoErr;
}

OSStatus mxos_rtos_push_to_queue( mxos_queue_t* queue, void* message, uint32_t timeout_ms )
{
    return kUnsupportedErr;
}


OSStatus mxos_rtos_push_to_queue_front( mxos_queue_t* queue, void* message, uint32_t timeout_ms )
{
    return kUnsupportedErr;
}


OSStatus mxos_rtos_pop_from_queue( mxos_queue_t* queue, void* message, uint32_t timeout_ms )
{
    return kUnsupportedErr;
}


OSStatus mxos_rtos_deinit_queue( mxos_queue_t* queue )
{
    return kUnsupportedErr;
}

bool mxos_rtos_is_queue_empty( mxos_queue_t* queue )
{
    return kUnsupportedErr;
}

bool mxos_rtos_is_queue_full( mxos_queue_t* queue )
{
    return kUnsupportedErr;
}


OSStatus mxos_rtos_init_timer( mxos_timer_t* timer, uint32_t time_ms, timer_handler_t function, void* arg )
{
    return kUnsupportedErr;
}

OSStatus mxos_rtos_start_timer( mxos_timer_t* timer )
{
    return kUnsupportedErr;
}

OSStatus mxos_rtos_stop_timer( mxos_timer_t* timer )
{
    return kUnsupportedErr;
}

OSStatus mxos_rtos_reload_timer( mxos_timer_t* timer )
{
    return kUnsupportedErr;
}

OSStatus mxos_rtos_deinit_timer( mxos_timer_t* timer )
{
    return kUnsupportedErr;
}

OSStatus mxos_time_get_time(mxos_time_t* time_ptr)
{
    mxos_time_t _time = mxos_rtos_get_time();
    *time_ptr = _time + mxos_time_offset;
    return kNoErr;
}

OSStatus mxos_time_set_time(mxos_time_t* time_ptr)
{
    mxos_time_offset = *time_ptr - mxos_rtos_get_time();
    return kNoErr;
}


