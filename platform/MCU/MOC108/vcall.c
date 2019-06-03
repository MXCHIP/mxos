#include "mxos.h"

merr_t mico_rtos_create_thread(mos_thread_id_t *thread, uint8_t priority, const char *name, mos_thread_func_t function, uint32_t stack_size, void * arg)
{
    *thread = mos_thread_new( priority, name, function, stack_size, arg);
    return *thread == NULL ? kGeneralErr : kNoErr;
}

merr_t mico_rtos_delay_milliseconds(uint32_t num_ms)
{
    return mos_msleep(num_ms);
}

merr_t mico_rtos_delete_thread(mos_thread_id_t *thread)
{
    mos_thread_delete(*thread);
    return kNoErr;
}

merr_t mico_rtos_get_semaphore(mos_semphr_id_t *id, uint32_t timeout)
{
    return mos_semphr_acquire(*id, timeout);
}

merr_t mico_rtos_deinit_semaphore(mos_semphr_id_t *id)
{
    mos_semphr_delete(*id);
    return kNoErr;
}

uint32_t mico_rtos_get_time(void)
{
    return mos_time();
}

merr_t mico_rtos_init_mutex(mos_mutex_id_t *mutex)
{
    *mutex = mos_mutex_new();
    return *mutex == NULL ? kGeneralErr : kNoErr;
}

merr_t mico_rtos_init_semaphore(mos_semphr_id_t *id, int count)
{
    *id = mos_semphr_new(count);
    return *id == NULL ? kGeneralErr : kNoErr;
}

bool mico_rtos_is_current_thread(mos_thread_id_t *thread)
{
    return mxos_rtos_is_current_thread(thread);
}

merr_t mico_rtos_lock_mutex(mos_mutex_id_t *mutex)
{
    mos_mutex_lock(*mutex);
    return kNoErr;
}

merr_t mico_rtos_push_to_queue(mos_queue_id_t *id, void *message, uint32_t timeout)
{
    return mos_queue_push(*id, message, timeout);
}

merr_t mico_rtos_set_semaphore(mos_semphr_id_t *id)
{
    return mos_semphr_release(*id);
}

merr_t mico_rtos_thread_join(mos_thread_id_t *id)
{
    mos_thread_join(*id);
    return kNoErr;
}

void mico_rtos_thread_msleep(uint32_t milliseconds)
{
    mos_msleep(milliseconds);
}

merr_t mico_rtos_unlock_mutex(mos_mutex_id_t *id)
{
    mos_mutex_unlock(*id);
    return kNoErr;
}

int mos_event_fd_new(mos_event_id_t event_handle)
{
    return mico_create_event_fd(event_handle);
}

mos_mallinfo_legacy_t *mos_mallinfo_legacy(void)
{
    return mico_memory_info();
}

merr_t mxos_network_init(void)
{
    return mxchipInit();
}

char *mxos_system_lib_version(void)
{
    return system_lib_version();
}

int mxos_wlan_driver_version(char *outVersion, uint8_t inLength)
{
    return wlan_driver_version(outVersion, inLength);
}

int mxos_wlan_monitor_no_easylink(void)
{
    return mico_wlan_monitor_no_easylink();
}

merr_t mwifi_monitor_set_channel(uint8_t channel)
{
    return mico_wlan_monitor_set_channel(channel);
}

void mwifi_monitor_reg_cb(monitor_cb_t fn)
{
    mico_wlan_register_monitor_cb(fn);
}

int mwifi_monitor_start(void)
{
    return mico_wlan_start_monitor();
}

int mwifi_monitor_stop(void)
{
    return mico_wlan_stop_monitor();
}

merr_t MicoFlashRead(mxos_partition_t inPartition, volatile uint32_t *off_set, uint8_t *outBuffer, uint32_t inBufferLength)
{
    return mhal_flash_read(inPartition, off_set, outBuffer, inBufferLength);
}