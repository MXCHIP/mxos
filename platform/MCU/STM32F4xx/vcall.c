#include "mxos.h"

merr_t mxos_platform_init(void);
int mico_create_event_fd(mxos_event_t event_handle);
merr_t mxchipInit(void);
char *system_lib_version(void);
int wlan_driver_version(char *outVersion, uint8_t inLength);
int mico_wlan_monitor_no_easylink(void);
merr_t mico_wlan_monitor_set_channel(uint8_t channel);
void mico_wlan_register_monitor_cb(monitor_cb_t fn);
int mico_wlan_start_monitor(void);
int mico_wlan_stop_monitor(void);

merr_t mico_rtos_create_thread(mos_thread_id_t *thread, uint8_t priority, const char *name, mos_thread_func_t function, uint32_t stack_size, void * arg)
{
    *thread = mos_thread_new( priority, name, function, stack_size, arg);
    return *thread == NULL ? kGeneralErr : kNoErr;
}

merr_t mico_rtos_delay_milliseconds(uint32_t num_ms)
{
	return mos_thread_delay(num_ms);
}

merr_t mico_rtos_delete_thread(mos_thread_id_t *thread)
{
	return mos_thread_delete(&thread);
}

merr_t mico_rtos_get_semaphore(mos_semphr_id_t *id, uint32_t timeout)
{
    return mos_semphr_acquire(*id, timeout);
}

merr_t mico_rtos_deinit_semaphore(mos_semphr_id_t *id)
{
	return mos_semphr_delete(*id);
}

uint32_t mico_rtos_get_time(void)
{
	return mxos_rtos_get_time();
}

merr_t mico_rtos_init_mutex(mos_mutex_id_t *mutex)
{
    *mutex = mos_mutex_new();
    return *mutex == NULL ? kGeneralErr : kNoErr;
}

merr_t mico_rtos_init_semaphore(mos_semphr_id_t *semaphore, int count)
{
    *semaphore = mos_semphr_new(count);
    return *semaphore == NULL ? kGeneralErr : kNoErr;
}

bool mico_rtos_is_current_thread(mos_thread_id_t *thread)
{
	return mxos_rtos_is_current_thread(thread);
}

merr_t mico_rtos_lock_mutex(mos_mutex_id_t *mutex)
{
	return mos_mutex_lock(*mutex);
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
	return mos_thread_join(*id);
}

void mico_rtos_thread_msleep(uint32_t milliseconds)
{
	mxos_rtos_thread_msleep(milliseconds);
}

merr_t mico_rtos_unlock_mutex(mos_mutex_id_t *id)
{
    return mos_mutex_unlock(*id);
}

mxosMemInfo_t *mico_memory_info(void)
{
	return mxos_memory_info();
}

const uint32_t mico_nmode_enable = true;

merr_t mico_platform_init(void)
{
	return mxos_platform_init();
}

merr_t mico_rtos_deinit_queue(mos_queue_id_t *id)
{
	return mos_queue_delete(*id);
}

void mico_rtos_enter_critical(void)
{
	mxos_rtos_enter_critical();
}

void mico_rtos_exit_critical(void)
{
	mxos_rtos_exit_critical();
}

merr_t mico_rtos_init_queue(mos_queue_id_t *id, const char *name, uint32_t message_size, uint32_t number_of_messages)
{
	*id = mos_queue_new(message_size, number_of_messages);
	return *id == NULL ? kGeneralErr : kNoErr;
}

bool mico_rtos_is_queue_empty(mos_queue_id_t *queue)
{
	return mxos_rtos_is_queue_empty(queue);
}

merr_t mico_rtos_pop_from_queue(mos_queue_id_t *id, void *message, uint32_t timeout_ms)
{
	return mos_queue_pop(*id, message, timeout_ms);
}

merr_t mico_rtos_thread_force_awake(mos_thread_id_t *thread)
{
	return mxos_rtos_thread_force_awake(thread);
}

int mxos_create_event_fd(mxos_event_t event_handle)
{
	return mico_create_event_fd(event_handle);
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

merr_t mxos_wlan_monitor_set_channel(uint8_t channel)
{
	return mico_wlan_monitor_set_channel(channel);
}

void mxos_wlan_register_monitor_cb(monitor_cb_t fn)
{
	mico_wlan_register_monitor_cb(fn);
}

int mxos_wlan_start_monitor(void)
{
	return mico_wlan_start_monitor();
}

int mxos_wlan_stop_monitor(void)
{
	return mico_wlan_stop_monitor();
}

merr_t MicoFlashRead(mxos_partition_t inPartition, volatile uint32_t *off_set, uint8_t *outBuffer, uint32_t inBufferLength)
{
	return mxos_flash_read(inPartition, off_set, outBuffer, inBufferLength);
}