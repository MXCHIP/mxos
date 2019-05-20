#include <string.h>
#include <stdio.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include "mos.h"

static bool _mos_in_isr(void);

// NOTE: thread functions
// -----------------------------------------------------------------------------------------------------------------------------
mos_thread_id_t mos_thread_new(uint8_t priority, const char *name, mos_thread_func_t function, uint32_t stack_size, void *arg)
{
    mos_thread_id_t id;

    return pdPASS == xTaskCreate((pdTASK_CODE)function, name, (unsigned short)(stack_size / sizeof(portSTACK_TYPE)), arg, priority, &id) ? id : NULL;
}

void mos_thread_delete(mos_thread_id_t id)
{
    vTaskDelete(id);
}

void mos_thread_suspend(mos_thread_id_t id)
{
    vTaskSuspend(id);
}

void mos_thread_resume(mos_thread_id_t id)
{
    vTaskResume(id);
}

void mos_thread_yield(void)
{
    vTaskDelay(0);
}

void mos_thread_join(mos_thread_id_t id)
{
    // TODO:
}

void mos_thread_awake(mos_thread_id_t id)
{
    // TODO:
}

mos_thread_id_t mos_thread_get_id(void)
{
    return xTaskGetCurrentTaskHandle();
}

char *mos_thread_get_name(mos_thread_id_t id)
{
    return pcTaskGetTaskName(id);
}

// NOTE: semaphore functions
// -----------------------------------------------------------------------------------------------------------------------------
mos_semphr_id_t mos_semphr_new(uint32_t count)
{
    return xSemaphoreCreateCounting(count, 0);
}

merr_t mos_semphr_acquire(mos_semphr_id_t id, uint32_t timeout)
{
    return pdTRUE == xSemaphoreTake(id, (portTickType)(timeout / portTICK_PERIOD_MS)) ? kNoErr : kTimeoutErr;
}

merr_t mos_semphr_release(mos_semphr_id_t id)
{
    BaseType_t result;

    if (_mos_in_isr() == true)
    {
        BaseType_t xHigherPriorityTaskWoken;
        result = xSemaphoreGiveFromISR(id, &xHigherPriorityTaskWoken);
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        result = xSemaphoreGive(id);
    }

    return result == pdPASS ? kNoErr : kGeneralErr;
}

void mos_semphr_delete(mos_semphr_id_t id)
{
    vQueueDelete(id);
}

// NOTE: mutex functions
// -----------------------------------------------------------------------------------------------------------------------------
mos_mutex_id_t mos_mutex_new(void)
{
    return xSemaphoreCreateMutex();
}

void mos_mutex_lock(mos_mutex_id_t id)
{
    xSemaphoreTake(id, portMAX_DELAY);
}

void mos_mutex_unlock(mos_mutex_id_t id)
{
    xSemaphoreGive(id);
}

void mos_mutex_delete(mos_mutex_id_t id)
{
    vSemaphoreDelete(id);
}

// NOTE: queue functions
// -----------------------------------------------------------------------------------------------------------------------------
mos_queue_id_t mos_queue_new(uint32_t message_size, uint32_t number_of_messages)
{
    return xQueueCreate(number_of_messages, message_size);
}

merr_t mos_queue_push(mos_queue_id_t id, void *message, uint32_t timeout_ms)
{
    BaseType_t result;

    if (_mos_in_isr() == true)
    {
        BaseType_t xHigherPriorityTaskWoken;
        result = xQueueSendToBackFromISR(id, message, &xHigherPriorityTaskWoken);
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        result = xQueueSendToBack(id, message, timeout_ms / portTICK_PERIOD_MS);
    }

    return result == pdPASS ? kNoErr : kGeneralErr;
}

merr_t mos_queue_pop(mos_queue_id_t id, void *message, uint32_t timeout_ms)
{
    return pdPASS == xQueueReceive(id, message, (timeout_ms / portTICK_PERIOD_MS)) ? kNoErr : kGeneralErr;
}

void mos_queue_delete(mos_queue_id_t id)
{
    vQueueDelete(id);
}

uint32_t mos_queue_get_total(mos_queue_id_t id)
{
    // TODO:
    return 0;
}

uint32_t mos_queue_get_free(mos_queue_id_t id)
{
    return uxQueueSpacesAvailable(id);
}

// NOTE: timer functions
// -----------------------------------------------------------------------------------------------------------------------------
typedef struct
{
    void *arg;
    mos_timer_handler_t func;
} _mos_timer_adapter_t;

static void _mos_timer_callback(TimerHandle_t handle)
{
    _mos_timer_adapter_t *adapter = (_mos_timer_adapter_t *)pvTimerGetTimerID(handle);
    adapter->func(adapter->arg);
}

mos_timer_id_t mos_timer_new(uint32_t time_ms, mos_timer_handler_t function, bool autoreload, void *arg)
{
    mos_timer_id_t id;
    _mos_timer_adapter_t *adapter;

    adapter = pvPortMalloc(sizeof(_mos_timer_adapter_t));
    adapter->arg = arg;
    adapter->func = function;

    id = xTimerCreate("", time_ms / portTICK_PERIOD_MS, autoreload, adapter, _mos_timer_callback);
    if (id != NULL)
        xTimerStart(id, portMAX_DELAY);
    return id;
}

void mos_timer_start(mos_timer_id_t id)
{
    if (_mos_in_isr() == true)
    {
        BaseType_t xHigherPriorityTaskWoken;
        xTimerStartFromISR(id, &xHigherPriorityTaskWoken);
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
    else
        xTimerStart(id, portMAX_DELAY);
}

void mos_timer_stop(mos_timer_id_t id)
{
    if (_mos_in_isr() == true)
    {
        BaseType_t xHigherPriorityTaskWoken;
        xTimerStopFromISR(id, &xHigherPriorityTaskWoken);
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
    else
        xTimerStop(id, portMAX_DELAY);
}

void mos_timer_change(mos_timer_id_t id, uint32_t time_ms)
{
    if (_mos_in_isr() == true)
    {
        BaseType_t xHigherPriorityTaskWoken;
        xTimerChangePeriodFromISR(id, time_ms / portTICK_PERIOD_MS, &xHigherPriorityTaskWoken);
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
    else
        xTimerChangePeriod(id, time_ms / portTICK_PERIOD_MS, portMAX_DELAY);
}

void mxos_timer_reset(mos_timer_id_t id)
{
    if (_mos_in_isr() == true)
    {
        BaseType_t xHigherPriorityTaskWoken;
        xTimerResetFromISR(id, &xHigherPriorityTaskWoken);
        portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
    }
    else
        xTimerReset(id, portMAX_DELAY);
}

void mos_timer_delete(mos_timer_id_t id)
{
    _mos_timer_adapter_t *adapter = (_mos_timer_adapter_t *)pvTimerGetTimerID(id);
    xTimerDelete(id, portMAX_DELAY);
    vPortFree(adapter);
}

bool mos_timer_is_runing(mos_timer_id_t id)
{
    return xTimerIsTimerActive(id);
}

// NOTE: other functions
// -----------------------------------------------------------------------------------------------------------------------------
uint32_t mos_time(void)
{
    if (_mos_in_isr() == true)
    {
        return xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
    }
    else
    {
        return xTaskGetTickCount() * portTICK_PERIOD_MS;
    }
}

size_t xPortGetTotalHeapSize(void);
int vPortGetBlocks(void);

void mos_sleep(float seconds)
{
    uint32_t ticks = seconds * 1000;

    ticks = ticks / portTICK_PERIOD_MS;

    vTaskDelay(ticks == 0 ? 1 : ticks);
}

void mos_msleep(int ms)
{
    uint32_t ticks;

    ticks = ms / portTICK_PERIOD_MS;

    vTaskDelay(ticks == 0 ? 1 : ticks);
}

mos_mallinfo_t *mos_mallinfo(void)
{
    static mos_mallinfo_t info;

    info.total = xPortGetTotalHeapSize();
    info.free = xPortGetFreeHeapSize();
    info.chunks = vPortGetBlocks();
    info.min_free = xPortGetMinimumEverFreeHeapSize();

    return &info;
}

// NOTE: heap functions
// -----------------------------------------------------------------------------------------------------------------------------
#define RECORD_LIST_NUM 200

#define HEAD_GUARD 0xDEADDEAD
#define TAIL_GUARD 0xBEEFBEEF

static mos_mallrecord_t record_list[RECORD_LIST_NUM];
static int record_index;
static mos_mutex_id_t record_mutex;

void mos_mallrecord_show(void)
{
    uint32_t *pv;
    uint32_t word_sz;
    printf("#\tthread id\tthread name\tcaller address\tmemory address\tmemory size\thead guard\ttail guard\t\r\n");
    printf("------------------------------------------------------------------------------------------------------------------\r\n");
    for (int i = 0; i < record_index; i++)
    {
        pv = (uint32_t *)record_list[i].addr - 1;
        word_sz = ((record_list[i].size + 3) >> 2) + 2;
        printf("%d\t%p\t%-15s\t%p\t0x%lx\t0x%-8lx\t0x%lx\t0x%lx\r\n",
               i,
               record_list[i].thread,
               mos_thread_get_name(record_list[i].thread),
               record_list[i].caller,
               record_list[i].addr,
               record_list[i].size,
               pv[0],
               pv[word_sz - 1]);
    }
}

/*
-------------------------------------------------
| 1 word     | N words             | 1 word     |
|------------+---------------------+------------|
| HEAD_GUARD | user memory | align | TAIL_GUARD |
-------------------------------------------------
*/

void *__wrap__malloc_r(void *p, size_t size)
{
    uint32_t *pv;
    size_t word_sz;

    word_sz = ((size + 3) >> 2) + 2;
    if ((pv = pvPortMalloc(word_sz * 4)) == NULL)
    {
        return NULL;
    }

    pv[0] = HEAD_GUARD;
    pv[word_sz - 1] = TAIL_GUARD;

    if (record_mutex == NULL)
    {
        record_mutex = mos_mutex_new();
    }

    mos_mutex_lock(record_mutex);
    if (record_index < RECORD_LIST_NUM)
    {
        record_list[record_index].thread = mos_thread_get_id();
        record_list[record_index].caller = __builtin_return_address(0);
        record_list[record_index].size = size;
        record_list[record_index].addr = (uint32_t)&pv[1];
        record_index++;
    }
    mos_mutex_unlock(record_mutex);

    return &pv[1];
}

void __wrap__free_r(void *p, void *x)
{
    size_t word_sz;
    uint32_t *pv = (uint32_t *)x;

    if (pv == NULL)
        return;

    pv--;

    mos_mutex_lock(record_mutex);
    for (int i = 0; i < record_index; i++)
    {
        if (record_list[i].addr == (uint32_t)x)
        {
            word_sz = ((record_list[i].size + 3) >> 2) + 2;
            if (pv[0] != HEAD_GUARD || pv[word_sz - 1] != TAIL_GUARD)
            {
                printf("WARNING! MEMORY CORRUPTED!\r\n");
                printf("#\tthread id\tthread name\tcaller address\tmemory address\tmemory size\thead guard\ttail guard\t\r\n");
                printf("------------------------------------------------------------------------------------------------------------------\r\n");
                printf("0\t%p\t%-15s\t%p\t0x%lx\t0x%-8lx\t0x%lx\t0x%lx\r\n",
                       record_list[i].thread,
                       mos_thread_get_name(record_list[i].thread),
                       record_list[i].caller,
                       record_list[i].addr,
                       record_list[i].size,
                       pv[0],
                       pv[word_sz - 1]);

                while (1)
                    ;
            }
            record_index--;
            memcpy(&record_list[i], &record_list[record_index], sizeof(mos_mallrecord_t));
            break;
        }
    }
    mos_mutex_unlock(record_mutex);

    vPortFree(pv);
}

void *__wrap__calloc_r(void *p, size_t a, size_t b)
{
    void *pv;
    size_t sz = a * b;

    if ((pv = __wrap__malloc_r(p, sz)) == NULL)
    {
        return NULL;
    }
    memset(pv, 0, sz);

    return pv;
}

void *__wrap__realloc_r(void *p, void *x, size_t sz)
{
    void *pv;

    if ((pv = __wrap__malloc_r(p, sz)) == NULL)
    {
        return NULL;
    }
    memcpy(pv, x, sz);
    __wrap__free_r(p, x);

    return pv;
}

// NOTE: bsp functions
// -----------------------------------------------------------------------------------------------------------------------------
// TODO:
// void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed portCHAR *pcTaskName)
// {
// }

// TODO:
// void vApplicationMallocFailedHook(void)
// {
// }

static bool _mos_in_isr(void)
{
    /* From the ARM Cortex-M3 Techinical Reference Manual
     * 0xE000ED04   ICSR    RW [a]  Privileged  0x00000000  Interrupt Control and State Register */
    return (*(volatile uint32_t *)0xE000ED04 & 0x1FF) != 0 ? true : false;
}

/*
 * Macros used by vListTask to indicate which state a task is in.
 */
#define tskBLOCKED_CHAR		( 'B' )
#define tskREADY_CHAR		( 'R' )
#define tskDELETED_CHAR		( 'D' )
#define tskSUSPENDED_CHAR	( 'S' )

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

#define cmd_printf(...) do{\
                                if (xWriteBufferLen > 0) {\
                                    snprintf(pcWriteBuffer, xWriteBufferLen, __VA_ARGS__);\
                                    xWriteBufferLen-=strlen(pcWriteBuffer);\
                                    pcWriteBuffer+=strlen(pcWriteBuffer);\
                                }\
                             }while(0)

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
                     (unsigned int) pxTaskStatusArray[x].uxCurrentPriority,
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

