#include "mxos.h"

OSStatus mxos_platform_init(void);
int mico_create_event_fd(mxos_event_t event_handle);
OSStatus mxchipInit(void);
char *system_lib_version(void);
int wlan_driver_version(char *outVersion, uint8_t inLength);
int mico_wlan_monitor_no_easylink(void);
OSStatus mico_wlan_monitor_set_channel(uint8_t channel);
void mico_wlan_register_monitor_cb(monitor_cb_t fn);
int mico_wlan_start_monitor(void);
int mico_wlan_stop_monitor(void);

OSStatus mico_rtos_create_thread(mxos_thread_t *thread, uint8_t priority, const char *name, mxos_thread_function_t function, uint32_t stack_size, mxos_thread_arg_t arg)
{
	return mxos_rtos_create_thread(thread, priority, name, function, stack_size, arg);
}

OSStatus mico_rtos_delay_milliseconds(uint32_t num_ms)
{
	return mxos_rtos_delay_milliseconds(num_ms);
}

OSStatus mico_rtos_delete_thread(mxos_thread_t *thread)
{
	return mxos_rtos_delete_thread(thread);
}

OSStatus mico_rtos_get_semaphore(mxos_semaphore_t *semaphore, uint32_t timeout_ms)
{
	return mxos_rtos_get_semaphore(semaphore, timeout_ms);
}

OSStatus mico_rtos_deinit_semaphore(mxos_semaphore_t *semaphore)
{
	return mxos_rtos_deinit_semaphore(semaphore);
}

uint32_t mico_rtos_get_time(void)
{
	return mxos_rtos_get_time();
}

OSStatus mico_rtos_init_mutex(mxos_mutex_t *mutex)
{
	return mxos_rtos_init_mutex(mutex);
}

OSStatus mico_rtos_init_semaphore(mxos_semaphore_t *semaphore, int count)
{
	return mxos_rtos_init_semaphore(semaphore, count);
}

bool mico_rtos_is_current_thread(mxos_thread_t *thread)
{
	return mxos_rtos_is_current_thread(thread);
}

OSStatus mico_rtos_lock_mutex(mxos_mutex_t *mutex)
{
	return mxos_rtos_lock_mutex(mutex);
}

OSStatus mico_rtos_push_to_queue(mxos_queue_t *queue, void *message, uint32_t timeout_ms)
{
	return mxos_rtos_push_to_queue(queue, message, timeout_ms);
}

OSStatus mico_rtos_set_semaphore(mxos_semaphore_t *semaphore)
{
	return mxos_rtos_set_semaphore(semaphore);
}

OSStatus mico_rtos_thread_join(mxos_thread_t *thread)
{
	return mxos_rtos_thread_join(thread);
}

void mico_rtos_thread_msleep(uint32_t milliseconds)
{
	mxos_rtos_thread_msleep(milliseconds);
}

OSStatus mico_rtos_unlock_mutex(mxos_mutex_t *mutex)
{
	return mxos_rtos_unlock_mutex(mutex);
}

mxosMemInfo_t *mico_memory_info(void)
{
	return mxos_memory_info();
}

const uint32_t mico_nmode_enable = true;

OSStatus mico_platform_init(void)
{
	return mxos_platform_init();
}

OSStatus mico_rtos_deinit_queue(mxos_queue_t *queue)
{
	return mxos_rtos_deinit_queue(queue);
}

void mico_rtos_enter_critical(void)
{
	mxos_rtos_enter_critical();
}

void mico_rtos_exit_critical(void)
{
	mxos_rtos_exit_critical();
}

OSStatus mico_rtos_init_queue(mxos_queue_t *queue, const char *name, uint32_t message_size, uint32_t number_of_messages)
{
	return mxos_rtos_init_queue(queue, name, message_size, number_of_messages);
}

bool mico_rtos_is_queue_empty(mxos_queue_t *queue)
{
	return mxos_rtos_is_queue_empty(queue);
}

OSStatus mico_rtos_pop_from_queue(mxos_queue_t *queue, void *message, uint32_t timeout_ms)
{
	return mxos_rtos_pop_from_queue(queue, message, timeout_ms);
}

OSStatus mico_rtos_thread_force_awake(mxos_thread_t *thread)
{
	return mxos_rtos_thread_force_awake(thread);
}

int mxos_create_event_fd(mxos_event_t event_handle)
{
	return mico_create_event_fd(event_handle);
}

OSStatus mxos_network_init(void)
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

OSStatus mxos_wlan_monitor_set_channel(uint8_t channel)
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

OSStatus MicoFlashRead(mxos_partition_t inPartition, volatile uint32_t *off_set, uint8_t *outBuffer, uint32_t inBufferLength)
{
	return mxos_flash_read(inPartition, off_set, outBuffer, inBufferLength);
}