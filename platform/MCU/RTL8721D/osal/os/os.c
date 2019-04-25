#include <string.h>

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

#include "osal/os/os.h"

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

size_t xPortGetTotalHeapSize( void );
int vPortGetBlocks(void);

void mos_sleep(float seconds)
{
    uint32_t ticks = seconds * 1000;

    ticks = ticks / portTICK_PERIOD_MS;

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

// NOTE:
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

void *__wrap__malloc_r(void *p, size_t size)
{
    return pvPortMalloc(size);
}

void __wrap__free_r(void *p, void *x)
{
    vPortFree(x);
}

void *__wrap__calloc_r(void *p, size_t a, size_t b)
{
    void *pvReturn;

    pvReturn = pvPortMalloc(a * b);
    if (pvReturn)
    {
        memset(pvReturn, 0, a * b);
    }

    return pvReturn;
}

void *__wrap__realloc_r(void *p, void *x, size_t sz)
{
    // TODO:
    return NULL;
}