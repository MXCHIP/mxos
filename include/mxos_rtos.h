/**
 ******************************************************************************
 * @file    mxos_rtos.h
 * @author  William Xu
 * @version V1.0.0
 * @date    16-Sep-2014
 * @brief   This file provides all the headers of RTOS operation provided by MXOS.
 ******************************************************************************
 *
 *  The MIT License
 *  Copyright (c) 2014 MXCHIP Inc.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is furnished
 *  to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 *  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ******************************************************************************
 */

#ifndef __MXOSRTOS_H__
#define __MXOSRTOS_H__

#include "mxos_opt.h"
#include "mxos_common.h"
#include "mos.h"
#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup MXOS_Core_APIs
  * @{
  */

/** @defgroup MXOS_RTOS MXOS RTOS Operations
  * @brief Provide management APIs for Thread, Mutex, Timer, Semaphore and FIFO
  * @{
  */

#define MXOS_HARDWARE_IO_WORKER_THREAD     ( (mos_worker_thread_id_t*)&mxos_hardware_io_worker_thread)
#define MXOS_NETWORKING_WORKER_THREAD      ( (mos_worker_thread_id_t*)&mxos_worker_thread )
#define MXOS_WORKER_THREAD                 ( (mos_worker_thread_id_t*)&mxos_worker_thread )

#define MXOS_RTOS_HIGEST_PRIORITY         (0)
#define MXOS_NETWORK_WORKER_PRIORITY      (3)
#define MXOS_DEFAULT_WORKER_PRIORITY      (5)
#define MXOS_DEFAULT_LIBRARY_PRIORITY     (5)
#define MXOS_APPLICATION_PRIORITY         (7)

#define kNanosecondsPerSecond   1000000000UUL
#define kMicrosecondsPerSecond  1000000UL
#define kMillisecondsPerSecond  1000

#define MXOS_NEVER_TIMEOUT   (0xFFFFFFFF)
#define MXOS_WAIT_FOREVER    (0xFFFFFFFF)
#define MXOS_NO_WAIT         (0)

typedef enum
{
    WAIT_FOR_ANY_EVENT,
    WAIT_FOR_ALL_EVENTS,
} mxos_event_flags_wait_option_t;

typedef uint32_t  mxos_event_flags_t;
typedef void (*timer_handler_t)( void* arg );
typedef merr_t (*event_handler_t)( void* arg );

typedef struct
{
    mos_thread_id_t thread;
    mos_queue_id_t  event_queue;
} mos_worker_thread_id_t;

typedef struct
{
    event_handler_t        function;
    void*                  arg;
    mos_timer_id_t         timer;
    mos_worker_thread_id_t*  thread;
} mxos_timed_event_t;

typedef void (*mos_thread_func_t)( void * arg );

extern mos_worker_thread_id_t mxos_hardware_io_worker_thread;
extern mos_worker_thread_id_t mxos_worker_thread;



/** @brief   Creates a worker thread
 *
 * Creates a worker thread
 * A worker thread is a thread in whose context timed and asynchronous events
 * execute.
 *
 * @param worker_thread    : a pointer to the worker thread to be created
 * @param priority         : thread priority
 * @param stack_size       : thread's stack size in number of bytes
 * @param event_queue_size : number of events can be pushed into the queue
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred
 */
merr_t mos_worker_thread_new( mos_worker_thread_id_t* worker_thread, uint8_t priority, uint32_t stack_size, uint32_t event_queue_size );


/** @brief   Deletes a worker thread
 *
 * @param worker_thread : a pointer to the worker thread to be created
 *
 * @return    kNoErr : on success.
 * @return    kGeneralErr   : if an error occurred
 */
merr_t mos_worker_thread_delete( mos_worker_thread_id_t* worker_thread );


/** @brief    Print Thread status into buffer
  *
  * @param    buffer, point to buffer to store thread status
  * @param    length, length of the buffer
  *
  * @return   none
  */
merr_t mxos_rtos_print_thread_status( char* buffer, int length );

/**
  * @}
  */


/** @defgroup MXOS_RTOS_EVENT MXOS RTOS Event Functions
  * @{
  */

/**
  * @brief    Sends an asynchronous event to the associated worker thread
  *
  * @param worker_thread :the worker thread in which context the callback should execute from
  * @param function      : the callback function to be called from the worker thread
  * @param arg           : the argument to be passed to the callback function
  *
  * @return    kNoErr        : on success.
  * @return    kGeneralErr   : if an error occurred
  */
merr_t mxos_rtos_send_asynchronous_event( mos_worker_thread_id_t* worker_thread, event_handler_t function, void* arg );

/** Requests a function be called at a regular interval
 *
 * This function registers a function that will be called at a regular
 * interval. Since this is based on the RTOS time-slice scheduling, the
 * accuracy is not high, and is affected by processor load.
 *
 * @param event_object  : pointer to a event handle which will be initialised
 * @param worker_thread : pointer to the worker thread in whose context the
 *                        callback function runs on
 * @param function      : the callback function that is to be called regularly
 * @param time_ms       : the time period between function calls in milliseconds
 * @param arg           : an argument that will be supplied to the function when
 *                        it is called
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred
 */
merr_t mxos_rtos_register_timed_event( mxos_timed_event_t* event_object, mos_worker_thread_id_t* worker_thread, event_handler_t function, uint32_t time_ms, void* arg );


/** Removes a request for a regular function execution
 *
 * This function de-registers a function that has previously been set-up
 * with @ref mxos_rtos_register_timed_event.
 *
 * @param event_object : the event handle used with @ref mxos_rtos_register_timed_event
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred
 */
merr_t mxos_rtos_deregister_timed_event( mxos_timed_event_t* event_object );


/**
  * @}
  */

#ifdef __cplusplus
} /*"C" */
#endif

#endif

/**
  * @}
  */

/**
  * @}
  */

