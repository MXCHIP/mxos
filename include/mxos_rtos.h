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
typedef void * mos_semphr_id_t;
typedef void * mxos_mutex_t;
typedef void * mos_thread_id_t;
typedef void * mxos_queue_t;
typedef void * mxos_event_t;// MXOS OS event: mos_semphr_id_t, mxos_mutex_t or mxos_queue_t
typedef void (*timer_handler_t)( void* arg );
typedef merr_t (*event_handler_t)( void* arg );

typedef struct
{
    void *          handle;
    timer_handler_t function;
    void *          arg;
}mxos_timer_t;

typedef struct
{
    mos_thread_id_t thread;
    mxos_queue_t  event_queue;
} mos_worker_thread_id_t;

typedef struct
{
    event_handler_t        function;
    void*                  arg;
    mxos_timer_t           timer;
    mos_worker_thread_id_t*  thread;
} mxos_timed_event_t;

typedef void (*mos_thread_func_t)( void * arg );

extern mos_worker_thread_id_t mxos_hardware_io_worker_thread;
extern mos_worker_thread_id_t mxos_worker_thread;

/* Legacy definitions */
#define mxos_thread_sleep                 mxos_rtos_thread_sleep
#define mxos_thread_msleep                mxos_rtos_thread_msleep
#define mxos_rtos_reload_timer            mxos_reload_timer
#define mxos_rtos_deinit_timer            mxos_deinit_timer
#define mxos_rtos_is_timer_running        mxos_is_timer_running
#define mxos_rtos_init_event_fd           mxos_create_event_fd
#define mxos_rtos_deinit_event_fd         mxos_delete_event_fd

/** @addtogroup MXOS_Core_APIs
  * @{
  */

/** @defgroup MXOS_RTOS MXOS RTOS Operations
  * @{
  */


/** @defgroup MXOS_RTOS_COMMON MXOS RTOS Common Functions
  * @brief Provide Generic RTOS Functions.
  * @{
  */

/** @brief Enter a critical session, all interrupts are disabled
  *
  * @return    none
  */
void mxos_rtos_enter_critical( void );

/** @brief Exit a critical session, all interrupts are enabled
  *
  * @return    none
  */
void mxos_rtos_exit_critical( void );

/**
  * @}
  */

/** @defgroup MXOS_RTOS_Thread MXOS RTOS Thread Management Functions
 *  @brief Provide thread creation, delete, suspend, resume, and other RTOS management API
 *  @verbatim   
 *   MXOS thread priority table
 *
 * +----------+-----------------+
 * | Priority |      Thread     |
 * |----------|-----------------|
 * |     0    |      MXOS       |   Highest priority
 * |     1    |     Network     |
 * |     2    |                 |
 * |     3    | Network worker  |
 * |     4    |                 |
 * |     5    | Default Library |
 * |          | Default worker  |
 * |     6    |                 |
 * |     7    |   Application   |
 * |     8    |                 |
 * |     9    |      Idle       |   Lowest priority
 * +----------+-----------------+ 
 *  @endverbatim
 * @{
 */


/** @brief Creates and starts a new thread
  *
  * @param thread     : Pointer to variable that will receive the thread handle (can be null)
  * @param priority   : A priority number.
  * @param name       : a text name for the thread (can be null)
  * @param function   : the main thread function
  * @param stack_size : stack size for this thread
  * @param arg        : argument which will be passed to thread function
  *
  * @return    kNoErr          : on success.
  * @return    kGeneralErr     : if an error occurred
  */
mos_thread_id_t mos_thread_new( uint8_t priority, const char* name, mos_thread_func_t function, uint32_t stack_size, void *arg );

/** @brief   Deletes a terminated thread
  *
  * @param   thread     : the handle of the thread to delete, , NULL is the current thread
  *
  * @return  kNoErr        : on success.
  * @return  kGeneralErr   : if an error occurred
  */
merr_t mos_thread_delete( mos_thread_id_t id );

/** @brief   Current thread is forced yield
  * @return  None
  */
void mos_thread_yield(void);

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


/** @brief    Suspend a thread
  *
  * @param    thread     : the handle of the thread to suspend, NULL is the current thread
  *
  * @return   kNoErr        : on success.
  * @return   kGeneralErr   : if an error occurred
  */
void mos_thread_suspend(mos_thread_id_t id);



/** @brief    Suspend all other thread
  *
  * @param    none
  *
  * @return   none
  */
void mxos_rtos_suspend_all_thread(void);


/** @brief    Rresume all other thread
  *
  * @param    none
  *
  * @return   none
  */
long mxos_rtos_resume_all_thread(void);


/** @brief    Sleeps until another thread has terminated
  *
  * @Details  Causes the current thread to sleep until the specified other thread
  *           has terminated. If the processor is heavily loaded with higher priority
  *           tasks, this thread may not wake until significantly after the thread termination.
  *
  * @param    thread : the handle of the other thread which will terminate
  *
  * @return   kNoErr        : on success.
  * @return   kGeneralErr   : if an error occurred
  */
merr_t mos_thread_join( mos_thread_id_t id );


/** @brief    Forcibly wakes another thread
  *
  * @Details  Causes the specified thread to wake from suspension. This will usually
  *           cause an error or timeout in that thread, since the task it was waiting on
  *           is not complete.
  *
  * @param    thread : the handle of the other thread which will be woken
  *
  * @return   kNoErr        : on success.
  * @return   kGeneralErr   : if an error occurred
  */
merr_t mxos_rtos_thread_force_awake( mos_thread_id_t* thread );


/** @brief    Checks if a thread is the current thread
  *
  * @Details  Checks if a specified thread is the currently running thread
  *
  * @param    thread : the handle of the other thread against which the current thread 
  *                    will be compared
  *
  * @return   true   : specified thread is the current thread
  * @return   false  : specified thread is not currently running
  */
bool mxos_rtos_is_current_thread( mos_thread_id_t* thread );

/** @brief    Get current thread handler
  *
  * @return   Current MXOS RTOS thread handler
  */
mos_thread_id_t mos_thread_get_id( void );

/** @brief    Suspend current thread for a specific time
  *
  * @param    seconds : A time interval (Unit: seconds)
  *
  * @return   None.
  */
void mxos_rtos_thread_sleep(uint32_t seconds);

/** @brief    Suspend current thread for a specific time
 *
 * @param     milliseconds : A time interval (Unit: millisecond)
 *
 * @return    None.
 */
void mxos_rtos_thread_msleep(uint32_t milliseconds);

/** @brief    Suspend current thread for a specific time
 *
 * @param     num_ms : A time interval (Unit: millisecond)
 *
 * @return    kNoErr.
 */
merr_t mos_thread_delay( uint32_t num_ms );


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

/** @defgroup MXOS_RTOS_SEM MXOS RTOS Semaphore Functions
  * @brief Provide management APIs for semaphore such as init,set,get and dinit. 
  * @{
  */

/** @brief    Initialises a counting semaphore and set count to 0
  *
  * @param    semaphore : a pointer to the semaphore handle to be initialised
  * @param    count     : the max count number of this semaphore
  *
  * @return   kNoErr        : on success.
  * @return   kGeneralErr   : if an error occurred
  */
mos_semphr_id_t mos_semphr_new( uint32_t count );


/** @brief    Set (post/put/increment) a semaphore
  *
  * @param    semaphore : a pointer to the semaphore handle to be set
  *
  * @return   kNoErr        : on success.
  * @return   kGeneralErr   : if an error occurred
  */
merr_t mos_semphr_release( mos_semphr_id_t id );


/** @brief    Get (wait/decrement) a semaphore
  *
  * @Details  Attempts to get (wait/decrement) a semaphore. If semaphore is at zero already,
  *           then the calling thread will be suspended until another thread sets the
  *           semaphore with @ref mos_semphr_release
  *
  * @param    semaphore : a pointer to the semaphore handle
  * @param    timeout_ms: the number of milliseconds to wait before returning
  *
  * @return   kNoErr        : on success.
  * @return   kGeneralErr   : if an error occurred
  */
merr_t mos_semphr_acquire( mos_semphr_id_t id, uint32_t timeout );


/** @brief    De-initialise a semaphore
  *
  * @Details  Deletes a semaphore created with @ref mos_semphr_new
  *
  * @param    semaphore : a pointer to the semaphore handle
  *
  * @return   kNoErr        : on success.
  * @return   kGeneralErr   : if an error occurred
  */
merr_t mos_semphr_delete( mos_semphr_id_t id );
/**
  * @}
  */

/** @defgroup MXOS_RTOS_MUTEX MXOS RTOS Mutex Functions
  * @brief Provide management APIs for Mutex such as init,lock,unlock and dinit.
  * @{
  */

/** @brief    Initialises a mutex
  *
  * @Details  A mutex is different to a semaphore in that a thread that already holds
  *           the lock on the mutex can request the lock again (nested) without causing
  *           it to be suspended.
  *
  * @param    mutex : a pointer to the mutex handle to be initialised
  *
  * @return   kNoErr        : on success.
  * @return   kGeneralErr   : if an error occurred
  */
merr_t mxos_rtos_init_mutex( mxos_mutex_t* mutex );


/** @brief    Obtains the lock on a mutex
  *
  * @Details  Attempts to obtain the lock on a mutex. If the lock is already held
  *           by another thead, the calling thread will be suspended until the mutex 
  *           lock is released by the other thread.
  *
  * @param    mutex : a pointer to the mutex handle to be locked
  *
  * @return   kNoErr        : on success.
  * @return   kGeneralErr   : if an error occurred
  */
merr_t mxos_rtos_lock_mutex( mxos_mutex_t* mutex );


/** @brief    Releases the lock on a mutex
  *
  * @Details  Releases a currently held lock on a mutex. If another thread
  *           is waiting on the mutex lock, then it will be resumed.
  *
  * @param    mutex : a pointer to the mutex handle to be unlocked
  *
  * @return   kNoErr        : on success.
  * @return   kGeneralErr   : if an error occurred
  */
merr_t mxos_rtos_unlock_mutex( mxos_mutex_t* mutex );


/** @brief    De-initialise a mutex
  *
  * @Details  Deletes a mutex created with @ref mxos_rtos_init_mutex
  *
  * @param    mutex : a pointer to the mutex handle
  *
  * @return   kNoErr        : on success.
  * @return   kGeneralErr   : if an error occurred
  */
merr_t mxos_rtos_deinit_mutex( mxos_mutex_t* mutex );
/**
  * @}
  */

/** @defgroup MXOS_RTOS_QUEUE MXOS RTOS FIFO Queue Functions
  * @brief Provide management APIs for FIFO such as init,push,pop and dinit.
  * @{
  */

/** @brief    Initialises a FIFO queue
  *
  * @param    queue : a pointer to the queue handle to be initialised
  * @param    name  : a text string name for the queue (NULL is allowed)
  * @param    message_size : size in bytes of objects that will be held in the queue
  * @param    number_of_messages : depth of the queue - i.e. max number of objects in the queue
  *
  * @return   kNoErr        : on success.
  * @return   kGeneralErr   : if an error occurred
  */
merr_t mxos_rtos_init_queue( mxos_queue_t* queue, const char* name, uint32_t message_size, uint32_t number_of_messages );


/** @brief    Pushes an object onto a queue
  *
  * @param    queue : a pointer to the queue handle
  * @param    message : the object to be added to the queue. Size is assumed to be
  *                  the size specified in @ref mxos_rtos_init_queue
  * @param    timeout_ms: the number of milliseconds to wait before returning
  *
  * @return   kNoErr        : on success.
  * @return   kGeneralErr   : if an error or timeout occurred
  */
merr_t mxos_rtos_push_to_queue( mxos_queue_t* queue, void* message, uint32_t timeout_ms );


/** @brief    Pops an object off a queue
  *
  * @param    queue : a pointer to the queue handle
  * @param    message : pointer to a buffer that will receive the object being
  *                     popped off the queue. Size is assumed to be
  *                     the size specified in @ref mxos_rtos_init_queue , hence
  *                     you must ensure the buffer is long enough or memory
  *                     corruption will result
  * @param    timeout_ms: the number of milliseconds to wait before returning
  *
  * @return   kNoErr        : on success.
  * @return   kGeneralErr   : if an error or timeout occurred
  */
merr_t mxos_rtos_pop_from_queue( mxos_queue_t* queue, void* message, uint32_t timeout_ms );


/** @brief    De-initialise a queue created with @ref mxos_rtos_init_queue
  *
  * @param    queue : a pointer to the queue handle
  *
  * @return   kNoErr        : on success.
  * @return   kGeneralErr   : if an error occurred
  */
merr_t mxos_rtos_deinit_queue( mxos_queue_t* queue );


/** @brief    Check if a queue is empty
  *
  * @param    queue : a pointer to the queue handle
  *
  * @return   true  : queue is empty.
  * @return   false : queue is not empty.
  */
bool mxos_rtos_is_queue_empty( mxos_queue_t* queue );


/** @brief    Check if a queue is full
  *
  * @param    queue : a pointer to the queue handle
  *
  * @return   true  : queue is empty.
  * @return   false : queue is not empty.
  */
bool mxos_rtos_is_queue_full( mxos_queue_t* queue );

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

/** @defgroup MXOS_RTOS_TIMER MXOS RTOS Timer Functions
  * @brief Provide management APIs for timer such as init,start,stop,reload and dinit.
  * @{
  */

/**
  * @brief    Gets time in miiliseconds since RTOS start
  *
  * @note:    Since this is only 32 bits, it will roll over every 49 days, 17 hours.
  *
  * @returns  Time in milliseconds since RTOS started.
  */
uint32_t mxos_rtos_get_time(void);


/** 
  * @brief     Initialize a RTOS timer
  *
  * @note      Timer does not start running until @ref mxos_start_timer is called
  *
  * @param     timer    : a pointer to the timer handle to be initialised
  * @param     time_ms  : Timer period in milliseconds
  * @param     function : the callback handler function that is called each time the 
  *                       timer expires
  * @param     arg      : an argument that will be passed to the callback function
  *
  * @return    kNoErr        : on success.
  * @return    kGeneralErr   : if an error occurred
  */
merr_t mxos_rtos_init_timer( mxos_timer_t* timer, uint32_t time_ms, timer_handler_t function, void* arg );


/** @brief    Starts a RTOS timer running
  *
  * @note     Timer must have been previously initialised with @ref mxos_rtos_init_timer
  *
  * @param    timer    : a pointer to the timer handle to start
  *
  * @return   kNoErr        : on success.
  * @return   kGeneralErr   : if an error occurred
  */
merr_t mxos_rtos_start_timer( mxos_timer_t* timer );


/** @brief    Stops a running RTOS timer
  *
  * @note     Timer must have been previously started with @ref mxos_rtos_init_timer
  *
  * @param    timer    : a pointer to the timer handle to stop
  *
  * @return   kNoErr        : on success.
  * @return   kGeneralErr   : if an error occurred
  */
merr_t mxos_rtos_stop_timer( mxos_timer_t* timer );


/** @brief    Reloads a RTOS timer that has expired
  *
  * @note     This is usually called in the timer callback handler, to
  *           reschedule the timer for the next period.
  *
  * @param    timer    : a pointer to the timer handle to reload
  *
  * @return   kNoErr        : on success.
  * @return   kGeneralErr   : if an error occurred
  */
merr_t mxos_rtos_reload_timer( mxos_timer_t* timer );


/** @brief    De-initialise a RTOS timer
  *
  * @note     Deletes a RTOS timer created with @ref mxos_rtos_init_timer
  *
  * @param    timer : a pointer to the RTOS timer handle
  *
  * @return   kNoErr        : on success.
  * @return   kGeneralErr   : if an error occurred
  */
merr_t mxos_rtos_deinit_timer( mxos_timer_t* timer );


/** @brief    Check if an RTOS timer is running
  *
  * @param    timer : a pointer to the RTOS timer handle
  *
  * @return   true        : if running.
  * @return   false       : if not running
  */
bool mxos_rtos_is_timer_running( mxos_timer_t* timer );

int SetTimer(unsigned long ms, void (*psysTimerHandler)(void));
int SetTimer_uniq(unsigned long ms, void (*psysTimerHandler)(void));
int UnSetTimer(void (*psysTimerHandler)(void));

/** @brief    Initialize an endpoint for a RTOS event, a file descriptor
  *           will be created, can be used for select
  *
  * @param    event_handle : mos_semphr_id_t, mxos_mutex_t or mxos_queue_t
  *
  * @retval   On success, a file descriptor for RTOS event is returned.
  *           On error, -1 is returned.
  */
int mxos_rtos_init_event_fd(mxos_event_t event_handle);

/** @brief    De-initialise an endpoint created from a RTOS event
  *
  * @param    fd : file descriptor for RTOS event
  *
  * @retval   0 for success. On error, -1 is returned.
  */
int mxos_rtos_deinit_event_fd(int fd);

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

