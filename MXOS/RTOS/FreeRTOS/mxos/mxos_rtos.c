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
#include "mxos_rtos.h"
//#include "qc_test.h"
#include "mxos_rtos_internal.h"
#include "mxos_rtos_common.h"
#include "FreeRTOS.h"
#include "projdefs.h"
#include "mxos_debug.h"
#include "platform_core.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "mxos_FreeRTOS_systick.h"
#include "FreeRTOSConfig.h"
#include "timers.h"


/******************************************************
 *                      Macros
 ******************************************************/

#if FreeRTOS_VERSION_MAJOR == 7
#define _xTaskCreate( pvTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask ) xTaskCreate( pvTaskCode, (signed char*)pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask )
#define _xTimerCreate( pcTimerName, xTimerPeriodInTicks, uxAutoReload, pvTimerID, pxCallbackFunction ) xTimerCreate( (signed char*)pcTimerName, xTimerPeriodInTicks, uxAutoReload, pvTimerID, pxCallbackFunction )
#else
#define _xTaskCreate( pvTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask ) xTaskCreate( pvTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask )
#define _xTimerCreate( pcTimerName, xTimerPeriodInTicks, uxAutoReload, pvTimerID, pxCallbackFunction ) xTimerCreate( pcTimerName, xTimerPeriodInTicks, uxAutoReload, pvTimerID, pxCallbackFunction )
#endif

#define cmd_printf(...) do{\
                                if (xWriteBufferLen > 0) {\
                                    snprintf(pcWriteBuffer, xWriteBufferLen, __VA_ARGS__);\
                                    xWriteBufferLen-=strlen(pcWriteBuffer);\
                                    pcWriteBuffer+=strlen(pcWriteBuffer);\
                                }\
                             }while(0)

/******************************************************
 *                    Constants
 ******************************************************/

#ifndef TIMER_THREAD_STACK_SIZE
#define TIMER_THREAD_STACK_SIZE      1024 + 4*1024
#endif
#define TIMER_QUEUE_LENGTH  5

/*
 * Macros used by vListTask to indicate which state a task is in.
 */
#define tskBLOCKED_CHAR     ( 'B' )
#define tskREADY_CHAR       ( 'R' )
#define tskDELETED_CHAR     ( 'D' )
#define tskSUSPENDED_CHAR   ( 'S' )

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef tmrTIMER_CALLBACK native_timer_handler_t;
typedef pdTASK_CODE       native_thread_t;

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    event_handler_t function;
    void*           arg;
} timer_queue_message_t;

typedef struct
{
    event_handler_t function;
    void*           arg;
} mxos_event_message_t;


/******************************************************
 *               Function Declarations
 ******************************************************/

extern void __libc_fini_array(void);
extern void __libc_init_array (void);
extern int main(int argc, char **argv);

extern void mxos_rtos_stack_overflow(char *taskname);
extern void mxos_board_init( void );
extern void mxos_system_qc_test( void );
extern bool mxos_should_enter_mfg_mode(void);

extern void mxos_main(void);
extern void pre_main (void);
extern int __real_main(void);

#ifdef __GNUC__
void __malloc_lock(struct _reent *ptr);
void __malloc_unlock(struct _reent *ptr);
#endif /* ifdef __GNUC__ */

/******************************************************
 *               Variables Definitions
 ******************************************************/

static xTaskHandle  app_thread_handle;

static mxos_time_t mxos_time_offset = 0;

extern const uint32_t mxos_tick_rate_hz;
extern uint32_t       app_stack_size;
extern const int CFG_PRIO_BITS;

uint32_t  max_syscall_int_prio;
uint32_t  ms_to_tick_ratio = 1;

uint32_t mxos_rtos_max_priorities = RTOS_HIGHEST_PRIORITY - RTOS_LOWEST_PRIORITY + 1;

/******************************************************
 *               Function Definitions
 ******************************************************/

/**
 *  Main function - starts FreeRTOS
 *  Called from the crt0 _start function
 *
 */


#if __MBED__
#include "cmsis.h"
#include "cmsis_nvic.h"

extern void vPortSVCHandler     ( void );
extern void xPortPendSVHandler     ( void );
extern void xPortSysTickHandler     ( void );
#endif

/* software_init_hook_rtos -> pre_main -> __wrap_main -> mbed_main/mxos_main -> __real_main */

void software_init_hook_rtos (void)
{
    max_syscall_int_prio = (1 << (8 - CFG_PRIO_BITS));
    ms_to_tick_ratio = (uint32_t)( 1000 / mxos_tick_rate_hz );

#if __MBED__
    NVIC_SetVector(SVCall_IRQn, (uint32_t)&vPortSVCHandler); //SVCall_IRQn
    NVIC_SetVector(PendSV_IRQn, (uint32_t)&xPortPendSVHandler); //PendSV_IRQn
    NVIC_SetVector(SysTick_IRQn, (uint32_t)&xPortSysTickHandler); //SysTick_IRQn
#endif

    /* Create an initial thread */
    _xTaskCreate( (TaskFunction_t)pre_main, "app_thread", (unsigned short)(app_stack_size/sizeof( portSTACK_TYPE )), NULL, MXOS_PRIORITY_TO_NATIVE_PRIORITY(MXOS_APPLICATION_PRIORITY), &app_thread_handle);

    /* Start the FreeRTOS scheduler - this call should never return */
    vTaskStartScheduler( );

}

/* case: use mbed hal, mbed_main is called by platform_regarget.cpp at mbed-os */
void mbed_main( void )
{
    /* Initialize MXOS_STDIO_UART and Printf Mutex. */
    mxos_main();
}

void pre_main( void )
{
    __libc_init_array();

    /* Jump to __wrap_main in platform_retarget.c */
    main(0, NULL);

    vTaskDelete( NULL );
}

mos_thread_id_t mos_thread_new( uint8_t priority, const char* name, mos_thread_func_t function, uint32_t stack_size, void *arg )
{
    mos_thread_id_t id;
    /* Limit priority to default lib priority */
    if ( priority > RTOS_HIGHEST_PRIORITY )
    {
        priority = RTOS_HIGHEST_PRIORITY;
    }

    if( pdPASS == _xTaskCreate( (native_thread_t)function, name, (unsigned short) (stack_size/sizeof( portSTACK_TYPE )), (void *)arg, MXOS_PRIORITY_TO_NATIVE_PRIORITY(priority), &id ) )
    {
        return id;
    }
    else
    {
        return NULL;
    }
}

merr_t mos_thread_delete( mos_thread_id_t thread )
{
    if ( thread == NULL )
    {
        vTaskDelete( NULL );
    }
    else if ( xTaskIsTaskFinished( thread ) != pdTRUE )
    {
        vTaskDelete( thread );
    }
    return kNoErr;
}

void mos_thread_yield( void )
{
    vTaskDelay( (portTickType) 0 );
}

merr_t mos_thread_join( mos_thread_id_t id )
{
    mos_thread_id_t tmp = id;

    if ( (id == NULL) || (tmp == NULL) )
        return kNoErr;

    while ( xTaskIsTaskFinished( tmp ) != pdTRUE )
    {
        mos_thread_delay( 10 );
    }
    return kNoErr;
}

bool mxos_rtos_is_current_thread( mos_thread_id_t* thread )
{
    if ( xTaskGetCurrentTaskHandle( ) == *thread )
    {
        return true;
    }
    else
    {
        return false;
    }
}

mos_thread_id_t mos_thread_get_id( void )
{
    return (mos_thread_id_t)xTaskGetCurrentTaskHandle();
}

#if FreeRTOS_VERSION_MAJOR == 7
/* Old deployment, may has some problem */
merr_t mxos_rtos_print_thread_status( char* pcWriteBuffer, int xWriteBufferLen )
{
    cmd_printf("%-30s Status  Prio    Stack   TCB\r\n", "Name");
    cmd_printf("------------------------------------------------------------");
    if (xWriteBufferLen >= 256)
        vTaskList((signed char *)pcWriteBuffer);
    else {
        char buf[256];

        vTaskList((signed char *)buf);
        strncpy(pcWriteBuffer, buf, xWriteBufferLen);
    }
    return kNoErr;
}

#else
static char *prvWriteNameToBuffer( char *pcBuffer, const char *pcTaskName )
{
    long x;

    /* Start by copying the entire string. */
    strcpy( pcBuffer, pcTaskName );

    /* Pad the end of the string with spaces to ensure columns line up when
    printed out. */
    for( x = strlen( pcBuffer ); x < ( configMAX_TASK_NAME_LEN - 1 ); x++ )
    {
        pcBuffer[ x ] = ' ';
    }

    /* Terminate. */
    pcBuffer[ x ] = 0x00;

    /* Return the new end of string. */
    return &( pcBuffer[ x ] );
}

/* Re-write vTaskList to add a buffer size parameter */
merr_t mxos_rtos_print_thread_status( char* pcWriteBuffer, int xWriteBufferLen )
{
    TaskStatus_t *pxTaskStatusArray;
    unsigned portBASE_TYPE uxCurrentNumberOfTasks = uxTaskGetNumberOfTasks();
    volatile UBaseType_t uxArraySize, x;
    char cStatus;
    char pcTaskStatusStr[48];
    char *pcTaskStatusStrTmp;

    /* Make sure the write buffer does not contain a string. */
    *pcWriteBuffer = 0x00;

    /* Take a snapshot of the number of tasks in case it changes while this
    function is executing. */
    uxArraySize = uxCurrentNumberOfTasks;

    /* Allocate an array index for each task.  NOTE!  if
     configSUPPORT_DYNAMIC_ALLOCATION is set to 0 then pvPortMalloc() will
     equate to NULL. */
    pxTaskStatusArray = pvPortMalloc( uxCurrentNumberOfTasks * sizeof(TaskStatus_t) );

    cmd_printf("%-12s Status     Prio    Stack   TCB\r\n", "Name");
    cmd_printf("-------------------------------------------\r\n");

    xWriteBufferLen-=strlen(pcWriteBuffer);

    if ( pxTaskStatusArray != NULL )
    {
        /* Generate the (binary) data. */
        uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, NULL );

        /* Create a human readable table from the binary data. */
        for ( x = 0; x < uxArraySize; x++ )
        {
            switch ( pxTaskStatusArray[x].eCurrentState )
            {
                case eReady:
                    cStatus = tskREADY_CHAR;
                    break;

                case eBlocked:
                    cStatus = tskBLOCKED_CHAR;
                    break;

                case eSuspended:
                    cStatus = tskSUSPENDED_CHAR;
                    break;

                case eDeleted:
                    cStatus = tskDELETED_CHAR;
                    break;

                default: /* Should not get here, but it is included
                 to prevent static checking errors. */
                    cStatus = 0x00;
                    break;
            }

            /* Write the task name to the string, padding with spaces so it
             can be printed in tabular form more easily. */
            pcTaskStatusStrTmp = pcTaskStatusStr;
            pcTaskStatusStrTmp = prvWriteNameToBuffer( pcTaskStatusStrTmp, pxTaskStatusArray[x].pcTaskName );
            //pcWriteBuffer = prvWriteNameToBuffer( pcWriteBuffer, pxTaskStatusArray[x].pcTaskName );

            /* Write the rest of the string. */
            sprintf( pcTaskStatusStrTmp, "\t%c\t%u\t%u\t%u\r\n", cStatus,
                     MXOS_PRIORITY_TO_NATIVE_PRIORITY((unsigned int) pxTaskStatusArray[x].uxCurrentPriority),
                     (unsigned int) pxTaskStatusArray[x].usStackHighWaterMark,
                     (unsigned int) pxTaskStatusArray[x].xTaskNumber );

            if( xWriteBufferLen < strlen( pcTaskStatusStr ) )
            {
                for ( x = 0; x < xWriteBufferLen; x++ )
                {
                    *(pcWriteBuffer+x) = '.';
                }
                break;
            }
            else
            {
                strncpy( pcWriteBuffer, pcTaskStatusStr, xWriteBufferLen);
                xWriteBufferLen -= strlen( pcTaskStatusStr );
                pcWriteBuffer += strlen( pcWriteBuffer );
            }
        }

        /* Free the array again.  NOTE!  If configSUPPORT_DYNAMIC_ALLOCATION
         is 0 then vPortFree() will be #defined to nothing. */
        vPortFree( pxTaskStatusArray );
    }
    else
    {
        mtCOVERAGE_TEST_MARKER( );
    }

    return kNoErr;
}
#endif

merr_t mxos_rtos_check_stack( void )
{
    // TODO: Add stack checking here.

    return kNoErr;
}

merr_t mxos_rtos_thread_force_awake( mos_thread_id_t* thread )
{
#if FreeRTOS_VERSION_MAJOR < 9
    vTaskForceAwake(*thread);
#else
    xTaskAbortDelay(*thread);
#endif
    return kNoErr;
}

merr_t mxos_time_get_time(mxos_time_t* time_ptr)
{
    *time_ptr = (mxos_time_t) ( xTaskGetTickCount( ) * ms_to_tick_ratio ) + mxos_time_offset;
    return kNoErr;
}

merr_t mxos_time_set_time(mxos_time_t* time_ptr)
{
    mxos_time_offset = *time_ptr - (mxos_time_t) ( xTaskGetTickCount( ) * ms_to_tick_ratio );
    return kNoErr;
}

mos_semphr_id_t mos_semphr_new( uint32_t count )
{
    return xSemaphoreCreateCounting( (unsigned portBASE_TYPE) count, (unsigned portBASE_TYPE) 0 );
}

merr_t mos_semphr_acquire( mos_semphr_id_t id, uint32_t timeout )
{
    if ( pdTRUE == xSemaphoreTake( id, (portTickType) ( timeout / ms_to_tick_ratio ) ) )
    {
        return kNoErr;
    }
    else
    {
        return kTimeoutErr;
    }
}

merr_t mos_semphr_release( mos_semphr_id_t id )
{
    signed portBASE_TYPE result;

    if ( platform_is_in_interrupt_context( ) == MXOS_TRUE )
    {
        signed portBASE_TYPE xHigherPriorityTaskWoken;
        result = xSemaphoreGiveFromISR( id, &xHigherPriorityTaskWoken );

        //check_string( result == pdTRUE, "Unable to set semaphore" );

        /* If xSemaphoreGiveFromISR() unblocked a task, and the unblocked task has
         * a higher priority than the currently executing task, then
         * xHigherPriorityTaskWoken will have been set to pdTRUE and this ISR should
         * return directly to the higher priority unblocked task.
         */
        portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    }
    else
    {
        result = xSemaphoreGive( id );
        //check_string( result == pdTRUE, "Unable to set semaphore" );
    }

    return ( result == pdPASS )? kNoErr : kGeneralErr;
}

merr_t mos_semphr_delete( mos_semphr_id_t id )
{
    if (id != NULL)
    {
        vQueueDelete( id );
    }
    return kNoErr;
}

void mxos_rtos_enter_critical( void )
{
    vPortEnterCritical();
}

void mxos_rtos_exit_critical( void )
{
    vPortExitCritical();
}


mos_mutex_id_t mos_mutex_new( void )
{
    /* Mutex uses priority inheritance */
    return xSemaphoreCreateMutex( );
}

merr_t mos_mutex_lock( mos_mutex_id_t id )
{
    check_string(id != NULL, "Bad args");

    if ( xSemaphoreTake( id, MXOS_WAIT_FOREVER ) != pdPASS )
    {
        return kGeneralErr;
    }

    return kNoErr;
}


merr_t mos_mutex_unlock( mos_mutex_id_t id )
{
    check_string(id != NULL, "Bad args");

    if ( xSemaphoreGive( id ) != pdPASS )
    {
        return kGeneralErr;
    }

    return kNoErr;
}

merr_t mos_mutex_delete( mos_mutex_id_t id )
{
    check_string(id != NULL, "Bad args");

    vSemaphoreDelete( id );
    return kNoErr;
}

mos_queue_id_t mos_queue_new( uint32_t message_size, uint32_t number_of_messages )
{
    return xQueueCreate( number_of_messages, message_size );
}

merr_t mos_queue_push( mos_queue_id_t id, void* message, uint32_t timeout_ms )
{
    signed portBASE_TYPE result;

    if ( platform_is_in_interrupt_context( ) == MXOS_TRUE )
    {
        signed portBASE_TYPE xHigherPriorityTaskWoken;
        result = xQueueSendToBackFromISR( id, message, &xHigherPriorityTaskWoken );

        /* If xQueueSendToBackFromISR() unblocked a task, and the unblocked task has
         * a higher priority than the currently executing task, then
         * xHigherPriorityTaskWoken will have been set to pdTRUE and this ISR should
         * return directly to the higher priority unblocked task.
         */
        portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    }
    else
    {
        result = xQueueSendToBack( id, message, (portTickType) ( timeout_ms / ms_to_tick_ratio ) );
    }

    return ( result == pdPASS )? kNoErr : kGeneralErr;
}


merr_t mos_queue_push_front( mos_queue_id_t* queue, void* message, uint32_t timeout_ms )
{
    signed portBASE_TYPE result;

    if ( platform_is_in_interrupt_context( ) == MXOS_TRUE )
    {
        signed portBASE_TYPE xHigherPriorityTaskWoken;
        result = xQueueSendToFrontFromISR( *queue, message, &xHigherPriorityTaskWoken );

        /* If xQueueSendToBackFromISR() unblocked a task, and the unblocked task has
         * a higher priority than the currently executing task, then
         * xHigherPriorityTaskWoken will have been set to pdTRUE and this ISR should
         * return directly to the higher priority unblocked task.
         */
        portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    }
    else
    {
        result = xQueueSendToFront( *queue, message, (portTickType) ( timeout_ms / ms_to_tick_ratio ) );
    }

    return ( result == pdPASS )? kNoErr : kGeneralErr;
}


merr_t mos_queue_pop( mos_queue_id_t id, void* message, uint32_t timeout_ms )
{
    if ( xQueueReceive( id, message, ( timeout_ms / ms_to_tick_ratio ) ) != pdPASS )
    {
        return kGeneralErr;
    }

    return kNoErr;
}


merr_t mos_queue_delete( mos_queue_id_t id )
{
    vQueueDelete( id );
    return kNoErr;
}

bool mxos_rtos_is_queue_empty( mos_queue_id_t* queue )
{
    signed portBASE_TYPE result;

    taskENTER_CRITICAL();
    result = xQueueIsQueueEmptyFromISR( *queue );
    taskEXIT_CRITICAL();

    return ( result != 0 ) ? true : false;
}

bool mxos_rtos_is_queue_full( mos_queue_id_t* queue )
{
    signed portBASE_TYPE result;

    taskENTER_CRITICAL();
    result = xQueueIsQueueFullFromISR( *queue );
    taskEXIT_CRITICAL();

    return ( result != 0 ) ? true : false;
}

static void timer_callback( xTimerHandle handle )
{
    mxos_timer_t* timer = (mxos_timer_t*) pvTimerGetTimerID( handle );

    if ( timer->function )
    {
        timer->function( timer->arg );
    }
}

merr_t mxos_rtos_init_timer( mxos_timer_t* timer, uint32_t time_ms, timer_handler_t function, void* arg )
{
    check_string(timer != NULL, "Bad args");

    timer->function = function;
    timer->arg      = arg;

    timer->handle = _xTimerCreate( "", (portTickType)( time_ms / ms_to_tick_ratio ), pdTRUE, timer, (native_timer_handler_t) timer_callback );
    if ( timer->handle == NULL )
    {
        return kGeneralErr;
    }

    return kNoErr;
}


merr_t mxos_rtos_start_timer( mxos_timer_t* timer )
{
    signed portBASE_TYPE result;

    if ( platform_is_in_interrupt_context( ) == MXOS_TRUE ) {
        signed portBASE_TYPE xHigherPriorityTaskWoken;
        result = xTimerStartFromISR(timer->handle, &xHigherPriorityTaskWoken );
        portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    } else
        result = xTimerStart( timer->handle, MXOS_WAIT_FOREVER );

    if ( result != pdPASS )
    {
        return kGeneralErr;
    }

    return kNoErr;
}

merr_t mxos_rtos_stop_timer( mxos_timer_t* timer )
{
    signed portBASE_TYPE result;

    if ( platform_is_in_interrupt_context( ) == MXOS_TRUE ) {
        signed portBASE_TYPE xHigherPriorityTaskWoken;
        result = xTimerStopFromISR(timer->handle, &xHigherPriorityTaskWoken );
        portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    } else
        result = xTimerStop( timer->handle, MXOS_WAIT_FOREVER );

    if ( result != pdPASS )
    {
        return kGeneralErr;
    }

    return kNoErr;
}

merr_t mxos_rtos_reload_timer( mxos_timer_t* timer )
{
    signed portBASE_TYPE result;

    if ( platform_is_in_interrupt_context( ) == MXOS_TRUE ) {
        signed portBASE_TYPE xHigherPriorityTaskWoken;
        result = xTimerResetFromISR(timer->handle, &xHigherPriorityTaskWoken );
        portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    } else
        result = xTimerReset( timer->handle, MXOS_WAIT_FOREVER );

    if ( result != pdPASS )
    {
        return kGeneralErr;
    }

    return kNoErr;
}

merr_t mxos_rtos_deinit_timer( mxos_timer_t* timer )
{
    if ( xTimerDelete( timer->handle, MXOS_WAIT_FOREVER ) != pdPASS )
    {
        return kGeneralErr;
    }

    return kNoErr;
}


bool mxos_rtos_is_timer_running( mxos_timer_t* timer )
{
    return ( xTimerIsTimerActive( timer->handle ) != 0 ) ? true : false;
}

merr_t mxos_rtos_init_event_flags( mxos_event_flags_t* event_flags )
{
    UNUSED_PARAMETER( event_flags );
    check_string( 0!=0, "Unsupported\r\n" );
    return kUnsupportedErr;
}

merr_t mxos_rtos_wait_for_event_flags( mxos_event_flags_t* event_flags, uint32_t flags_to_wait_for, uint32_t* flags_set, mxos_bool_t clear_set_flags, mxos_event_flags_wait_option_t wait_option, uint32_t timeout_ms )
{
    UNUSED_PARAMETER( event_flags );
    UNUSED_PARAMETER( flags_to_wait_for );
    UNUSED_PARAMETER( flags_set );
    UNUSED_PARAMETER( clear_set_flags );
    UNUSED_PARAMETER( wait_option );
    UNUSED_PARAMETER( timeout_ms );
    check_string( 0!=0, "Unsupported\r\n" );
    return kUnsupportedErr;
}

merr_t mxos_rtos_set_event_flags( mxos_event_flags_t* event_flags, uint32_t flags_to_set )
{
    UNUSED_PARAMETER( event_flags );
    UNUSED_PARAMETER( flags_to_set );
    check_string( 0!=0, "Unsupported\r\n" );
    return kUnsupportedErr;
}

merr_t mxos_rtos_deinit_event_flags( mxos_event_flags_t* event_flags )
{
    UNUSED_PARAMETER( event_flags );
    check_string( 0!=0, "Unsupported\r\n" );
    return kUnsupportedErr;
}

#ifdef __GNUC__

void __rtos_malloc_lock(struct _reent *ptr)
{
    UNUSED_PARAMETER( ptr );
    vTaskSuspendAll();
    return;
}

void __rtos_malloc_unlock(struct _reent *ptr)
{
    UNUSED_PARAMETER( ptr );
    xTaskResumeAll();
}

#endif /* __GNUC__ */


void mos_thread_suspend(mos_thread_id_t thread)
{
    if (thread == NULL)
        vTaskSuspend(NULL);
    else
        vTaskSuspend(thread);
}

void mos_thread_resume(mos_thread_id_t id)
{
    if (id == NULL)
        vTaskResume(NULL);
    else
        vTaskResume(id);
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
    return (mxos_time_t) ( xTaskGetTickCount( ) * ms_to_tick_ratio );
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
    uint32_t ticks;

    ticks = num_ms / ms_to_tick_ratio;
    if (ticks == 0)
        ticks = 1;

    vTaskDelay( (portTickType) ticks );

    return kNoErr;
}

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName )
{
    /*@-noeffect@*/
    UNUSED_PARAMETER( pxTask );
    UNUSED_PARAMETER( pcTaskName ); /* unused parameter in release build */
    /*@+noeffect@*/

    mxos_rtos_stack_overflow((char*)pcTaskName);
}

void vApplicationMallocFailedHook( void )
{
    //WPRINT_RTOS_DEBUG(("Heap is out of memory during malloc\r\n"));
}


#if FreeRTOS_VERSION_MAJOR > 7
void rtos_suppress_and_sleep( unsigned long sleep_ms )
{
    TickType_t missed_ticks = 0;
    extern uint32_t platform_power_down_hook( unsigned long sleep_ms );

    missed_ticks = platform_power_down_hook( sleep_ms );

    vTaskStepTick( missed_ticks );
}

#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void *mxos_malloc( size_t xWantedSize )
{
	return pvPortMalloc(xWantedSize);
}

void mxos_free( void *pv )
{
	vPortFree(pv);
}

void *mxos_realloc( void *pv, size_t xWantedSize )
{
	return pvPortRealloc(pv, xWantedSize);
}

//#endif

