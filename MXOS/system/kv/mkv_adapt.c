/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#include "mkv.h"
#include "mxos.h"

void *mkv_queue_new(uint32_t n)
{
    return mos_queue_new(sizeof(void *), n);
}

void mkv_queue_pop(void *queue, void *msg)
{
    mos_queue_pop(queue, msg, MXOS_WAIT_FOREVER);
}

void mkv_queue_push(void *queue, void *msg)
{
    mos_queue_push(queue, msg, MXOS_WAIT_FOREVER);
}

void *mkv_sem_new(void)
{
    return mos_semphr_new(1);
}

void mkv_sem_acquire(void *sem)
{
    mos_semphr_acquire(sem, MXOS_WAIT_FOREVER);
}

void mkv_sem_release(void *sem)
{
    mos_semphr_release(sem);
}

void mkv_sem_delete(void *sem)
{
    mos_semphr_delete(sem);
}

int mkv_thread_new(void (*func)(void))
{
    return mos_thread_new(MXOS_APPLICATION_PRIORITY, "mkv deamon", (void (*)(void *))func, 2048, NULL) == NULL ? -1 : 0;
}

int32_t kv_flash_read(uint32_t offset, void *buf, uint32_t nbytes)
{
    return mhal_flash_read(MXOS_PARTITION_KV, &offset, buf, nbytes);
}

int32_t kv_flash_write(uint32_t offset, void *buf, uint32_t nbytes)
{
    return mhal_flash_write(MXOS_PARTITION_KV, &offset, buf, nbytes);
}

int32_t kv_flash_erase(uint32_t offset, uint32_t size)
{
    return mhal_flash_erase(MXOS_PARTITION_KV, offset, size);
}

void *kv_malloc(uint32_t size)
{
    return malloc(size);
}

void kv_free(void *ptr)
{
    free(ptr);
}
