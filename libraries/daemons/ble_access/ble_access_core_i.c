/**
 ******************************************************************************
 * @file    ble_access_core_i.c
 * @author  Jian Zhang
 * @version V1.2.1
 * @date    26-Dec-2016
 * @file    BLE ACCESS Protocol Components
 * ******************************************************************************
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
 *
 ******************************************************************************
 *  BLE Vendor Specific Device
 *
 * Features demonstrated
 *  - Implement BLE_ACCESS Center Protocol developed by MXCHIP on MXOSKit-3239.
 *  - You should see details about this protocol on mxos.io. 
 *
 ******************************************************************************
 **/

#include <string.h>
#include <stdio.h>

#include "mxos.h"
#include "mxos_bt.h"
#include "mxos_bt_cfg.h"
#include "mxos_bt_smart_interface.h"
#include "mxos_bt_smartbridge.h"
#include "mxos_bt_smartbridge_gatt.h"
#include "sdpdefs.h"
#include "gattdefs.h"

#include "StringUtils.h"
#include "LinkListUtils.h"

#include "ble_access_core.h"
#include "ble_access_core_i.h"

/*
 *-------------------------------------------------------------------------------------------------
 *
 *  Configurations & Constants 
 *
 *-------------------------------------------------------------------------------------------------
 */

/*
 *-------------------------------------------------------------------------------------------------
 *
 *  Predefine Type
 *
 *-------------------------------------------------------------------------------------------------
 */

/*
 *-------------------------------------------------------------------------------------------------
 *
 *  Local Function Prototype 
 *
 *-------------------------------------------------------------------------------------------------
 */

/*
 *-------------------------------------------------------------------------------------------------
 *
 *  Local Variables 
 *
 *-------------------------------------------------------------------------------------------------
 */

static ble_access_device_t  ble_access_devices[MAX_CONCURRENT_CONNECTIONS];  
static mos_mutex_id_t         ble_access_dev_mutex;

static linked_list_t        ble_access_connecting_device_list; 
static mos_mutex_id_t         ble_access_conn_dev_list_mutex;

static mos_worker_thread_id_t ble_access_worker_thread;
static event_handler_t      ble_access_timer_evt;

/* Prefix Address for valid device MAC Address */
static uint8_t prefix_addr[][BLE_ACCESS_PREFIX_LEN] = { 
    [0] = {
        BLE_ACCESS_PREFIX_D0BAE4
    }, 
    [1] = {
        BLE_ACCESS_PREFIX_C89346
    },
};

/*
 *-------------------------------------------------------------------------------------------------
 *
 *  Global Function Definition
 *
 *-------------------------------------------------------------------------------------------------
 */

/*
 *  Worker thread
 */

merr_t ble_access_create_worker_thread(void)
{
    return mos_worker_thread_new(&ble_access_worker_thread, 
                                          MXOS_DEFAULT_WORKER_PRIORITY,
                                          2048,
                                          20);
}

merr_t ble_access_send_aync_event(event_handler_t event_handle, void *arg)
{
    return mxos_rtos_send_asynchronous_event(&ble_access_worker_thread, event_handle, arg);
}

/*
 * Timer Management
 */

static void ble_access_timer_callback(void *arg)
{
    if (ble_access_timer_evt) {
        ble_access_send_aync_event(ble_access_timer_evt, arg);
    }
}

merr_t ble_access_start_timer(ble_access_device_t *dev, event_handler_t timer_event_handle, void *arg)
{
    merr_t err = kNoErr;

    require_action(dev != NULL && timer_event_handle != NULL, exit, err = kParamErr);

    err = mxos_rtos_init_timer(&dev->timer, 10000, ble_access_timer_callback, arg);
    require_noerr_string(err, exit, "Initialize a timer failed");

    err = mxos_rtos_start_timer(&dev->timer);
    require_noerr_action_string(err, exit, mxos_rtos_deinit_timer(&dev->timer), "Start a timer failed");

    ble_access_timer_evt = timer_event_handle;

exit:
    return err;
}

merr_t ble_access_stop_timer(ble_access_device_t *dev)
{
    merr_t err = kNoErr;

    require_action(dev != NULL, exit, err = kParamErr);

    if (mxos_rtos_is_timer_running(&dev->timer)) {
        mxos_rtos_stop_timer(&dev->timer);
    }
    err = mxos_rtos_deinit_timer(&dev->timer);

exit:
    return err;
}

/*
 * Local Device Pool Management
 */

/* Initialize a devices pool */
void ble_access_initialize_devices(void)
{
    merr_t err = kNoErr;
    uint8_t  idx = 0;

    for (idx = 0; idx < ble_access_array_size(ble_access_devices); idx++) {
        ble_access_devices[idx].used = FALSE;
        ble_access_devices[idx].device_id = 0;
        err = mxos_bt_smartbridge_create_socket(&ble_access_devices[idx].socket);
        require_noerr_string(err, exit, "Create Sockets failed");
    }
    ble_access_dev_mutex = mos_mutex_new();
    
exit:
    return;
}

/* De-initialize a devices pool */
void ble_access_deinit_devices(void)
{
    uint8_t idx = 0;

    for (idx = 0; idx < ble_access_array_size(ble_access_devices); idx++) {
        mxos_bt_smartbridge_delete_socket(&ble_access_devices[idx].socket);
    }
    mos_mutex_delete(ble_access_dev_mutex);
}

ble_access_device_t *ble_access_find_device_by_address(const mxos_bt_device_address_t address)
{
    uint8_t                  idx = 0;
    ble_access_device_t     *dev = NULL;
    uint32_t                 device_id = ble_access_calculate_device_id(address);

    mos_mutex_lock(ble_access_dev_mutex);
    for (idx = 0; idx < ble_access_array_size(ble_access_devices); idx++) {
        if (ble_access_devices[idx].used && device_id == ble_access_devices[idx].device_id) {
            break;
        }
    }
    if (idx < ble_access_array_size(ble_access_devices)) {
        dev = &ble_access_devices[idx];
    }
    mos_mutex_unlock(ble_access_dev_mutex);
    
    return dev;
}

/* Get a free socket from 'sockets' */
ble_access_device_t *ble_access_get_free_device(void)
{
    uint8_t idx = 0;

    mos_mutex_lock(ble_access_dev_mutex);
    for (idx = 0; idx < ble_access_array_size(ble_access_devices); idx++) {
        if (!ble_access_devices[idx].used) {
            ble_access_devices[idx].used = TRUE;
            mos_mutex_unlock(ble_access_dev_mutex);
            return &ble_access_devices[idx];
        }
    }
    mos_mutex_unlock(ble_access_dev_mutex);

    return NULL;
}

void ble_access_release_device(mxos_bool_t free, const ble_access_device_t *device)
{
    uint8_t idx = 0;

    mos_mutex_lock(ble_access_dev_mutex);
    if (device != NULL) {
        for (idx = 0; 
             idx < ble_access_array_size(ble_access_devices) && device != &ble_access_devices[idx]; 
             idx++);
        if (idx < ble_access_array_size(ble_access_devices) && free) {
            ble_access_devices[idx].device_id = 0;
            ble_access_devices[idx].used = FALSE;
        }
    }
    mos_mutex_unlock(ble_access_dev_mutex);
}

/*
 * Generate a BT device address by device_id and prefix.
 *
 * @param[output] addr - device address
 * @param[input]  prefix_addr - a prefix address array.
 * @param[input] device_id - a device id.
 *
 * addr[0 - 2] = prefix_addr[0 - 2]
 * addr[3] = device_id[bit23:bit16]
 * addr[4] = device_id[bit15:bit8]
 * addr[5] = device_id[bit7:bit0]
 *
 * example:
 *      prefix - { 0x20, 0x73, 0x6a }, device_id = 0x12112233,
 *  ---->
 *      addr = { 0x20, 0x73, 0x6a, 0x11, 0x22, 0x33 }
 */
merr_t ble_access_generate_device_address(mxos_bt_device_address_t addr, uint32_t device_id)
{
    uint8_t idx_addr_type = 0;

    /* Check Parameters */
    if (addr == NULL) {
        return kParamErr;
    }

    idx_addr_type = (uint8_t)((device_id & 0xff000000) >> 24);
    if (idx_addr_type >= ble_access_array_size(prefix_addr)) {
        return kParamErr;
    }

    /* Fill prefix address part */
    memcpy(addr, prefix_addr[idx_addr_type], 3);
    /* Fill the device id part (little endian) */
    addr[3] = (uint8_t)((device_id & 0x00ffffff) >> 16);
    addr[4] = (uint8_t)((device_id & 0x00ffffff) >> 8);
    addr[5] = (uint8_t)((device_id & 0x00ffffff));
    return kNoErr;
}

/*
 * Calculate the device ID according device address.
 *
 * @param[input] addr - the device address
 *
 * return device ID.
 *
 * example:
 *  addr = { 0x20, 0x73, 0x6a, 0x11, 0x22, 0x33 };
 *  so, device_id = 0x112233.
 */
uint32_t ble_access_calculate_device_id(const mxos_bt_device_address_t addr)
{
    uint32_t id = 0x00000000;
    uint8_t  idx;

    /* check parameters */
    if (addr == NULL) {
        /* No allow device address. */
        goto exit;
    }

    for (idx = 0; idx < ble_access_array_size(prefix_addr); idx++) {
        if (memcmp(&addr[0], prefix_addr[idx], 3) == 0) {
            break;
        }
    }
    if (idx >= ble_access_array_size(prefix_addr)) {
        goto exit;
    }

    /* Calculate Device ID. */
    id |= (((uint32_t)idx) << 24);
    id |= (((uint32_t)addr[3]) << 16);
    id |= (((uint32_t)addr[4]) << 8);
    id |= (((uint32_t)addr[5]));

exit:
    return id;
}

/*
 * Get and parse manufacturer data from the Peer Device Advertisement data.
 */
merr_t ble_access_get_manufactor_adv_data(uint8_t *eir_data,
                                            uint8_t  eir_data_length,
                                            ble_access_manufactor_data_t *manufactor_data)
{
    if (eir_data == NULL || eir_data_length < 14 || manufactor_data == NULL) {
        return kParamErr;
    }

    /* Manufacture data
     *
     * ------------------------------------------------------------------------------------
     * | MXCHIP (6 bytes) | advType (1 byte) | directAddr (6 bytes) | alertState (1 byte) |
     * ------------------------------------------------------------------------------------
     *
     * advType: MXCHIP_ADV_TYPE_INIT or MXCHIP_ADV_TYPE_RECONN
     * directAddr: Directing device address. (used in MXCHIP_ADV_TYPE_RECONN)
     * alertState: Alert state. 00 or 01
     *
     */
    memcpy(manufactor_data->mxchip, &eir_data[0], 6);
    manufactor_data->adv_type = eir_data[6];
    manufactor_data->alert_state = eir_data[13];
    memcpy(manufactor_data->direct_addr, &eir_data[7], 6);
    return kNoErr;
}

/* Check wheter a ADV packet is valid type */
merr_t ble_access_check_adv_type(const uint8_t *adv_data,
                                   uint8_t length,
                                   uint8_t adv_type,
                                   ble_access_manufactor_data_t *manu_data)
{
    merr_t                         err = kNoErr;
    uint8_t                         *packet = NULL;
    uint8_t                          data_length = 0;
    ble_access_manufactor_data_t     manufactor_data;

    if (adv_data == NULL
            || length == 0
            || (adv_type != BLE_ACCESS_ADV_TYPE_INIT && adv_type != BLE_ACCESS_ADV_TYPE_RECONN)) {
        return MXOS_FALSE;
    }

    packet = mxos_bt_ble_check_advertising_data((uint8_t *)adv_data,
                                                BTM_BLE_ADVERT_TYPE_MANUFACTURER,
                                                &data_length);
    if (packet == NULL || data_length == 0) {
        err = kUnknownErr;
        goto exit;
    }

    err = ble_access_get_manufactor_adv_data(packet,
                                             data_length,
                                             &manufactor_data);
    if (err != kNoErr) goto exit;

    if (memcmp(manufactor_data.mxchip, BLE_ACCESS_MXCHIP_FLAG, 6) == 0) {
        if (manufactor_data.adv_type != adv_type) {
            err = kParamErr;
        } else {
            if (manu_data != NULL) {
                memcpy(manu_data, &manufactor_data, sizeof(ble_access_manufactor_data_t));
            }
        }
    } else {
        err = kGeneralErr;
    }
exit:
    return err;
}

void ble_access_set_scan_cfg(mxos_bt_smart_scan_settings_t *scan_cfg, mxos_bool_t is_auto_scanning)
{
    require_string(scan_cfg != NULL, exit, "invalid parameters");
    
    if (is_auto_scanning) {
        scan_cfg->interval = 2048;
        scan_cfg->window = 48;
        scan_cfg->duration_second = 0xffff;
        scan_cfg->type = BT_SMART_PASSIVE_SCAN;
        scan_cfg->filter_policy = FILTER_POLICY_WHITE_LIST;
        scan_cfg->filter_duplicates = DUPLICATES_FILTER_DISABLED;
    } else {
        scan_cfg->interval = 256;
        scan_cfg->window = 48;
        scan_cfg->duration_second = 30;
        scan_cfg->type = BT_SMART_ACTIVE_SCAN;
        scan_cfg->filter_policy = FILTER_POLICY_NONE;
        scan_cfg->filter_duplicates = DUPLICATES_FILTER_DISABLED;
    }
    
exit:
    return;
}

mxos_bool_t ble_access_uuid_compare(const ble_access_uuid_t *uuid1, const ble_access_uuid_t *uuid2)
{
    if (!uuid1 || !uuid2) {
        return MXOS_FALSE;
    }

    if (uuid1->len == uuid2->len 
        && memcmp(&uuid1->uu, &uuid2->uu, uuid1->len) == 0) {

        return MXOS_TRUE;
    }

    return MXOS_FALSE;
}

merr_t ble_access_connect_list_init( void )
{
    ble_access_conn_dev_list_mutex = mos_mutex_new();
    return linked_list_init(&ble_access_connecting_device_list);
}

merr_t ble_access_connect_list_deinit(void)
{
    merr_t                err = kNoErr;
    mxos_bt_smart_device_t *dev = NULL;

    for(; ;) {
        err = ble_access_connect_list_get(&dev, NULL);
        if (err == kNoErr) {
            ble_access_connect_list_remove(dev);
        } else {
            break;
        }
    }
    
    return mos_mutex_delete(ble_access_conn_dev_list_mutex);
}

mxos_bool_t compare_device_by_address(linked_list_node_t* node_to_compare, void* user_data)
{
    ble_access_connecting_device_t* device = (ble_access_connecting_device_t* )node_to_compare;
    mxos_bt_device_address_t* device_address  = (mxos_bt_device_address_t *)user_data;

    if (memcmp(device->device.address, device_address, BD_ADDR_LEN) == 0) {
        return TRUE;
    } else {
        return FALSE;
    }
}

merr_t ble_access_connect_list_add(const mxos_bt_smart_device_t *remote_device, mxos_bool_t is_reported)
{
    merr_t err = kNoErr;
    ble_access_connecting_device_t *device_found, *new_device;

    require_action(remote_device != NULL, exit, err = kParamErr);

    mos_mutex_lock(ble_access_conn_dev_list_mutex);
    err = linked_list_find_node(&ble_access_connecting_device_list,
                                 (linked_list_compare_callback_t)compare_device_by_address,
                                 (void *)remote_device->address,
                                 (linked_list_node_t**)&device_found);
    mos_mutex_unlock(ble_access_conn_dev_list_mutex);
    
    if (err != kNotFoundErr) {
        err = kAlreadyInUseErr;
        goto exit;
    }

    new_device = malloc(sizeof(ble_access_device_t));
    if (!new_device) {
        err = kNoMemoryErr;
        goto exit;
    }

    new_device->reported = is_reported;
    memcpy(&new_device->device, remote_device, sizeof(mxos_bt_smart_device_t));

    mos_mutex_lock(ble_access_conn_dev_list_mutex);
    err = linked_list_insert_node_at_rear(&ble_access_connecting_device_list, &new_device->this_node);
    mos_mutex_unlock(ble_access_conn_dev_list_mutex);

exit:
    return err;
}

merr_t ble_access_connect_list_set_report(const mxos_bt_smart_device_t *device, mxos_bool_t is_reported)
{
    merr_t err = kNoErr;
    ble_access_connecting_device_t *device_found;

    require_action(device != NULL, exit, err = kParamErr);

    mos_mutex_lock(ble_access_conn_dev_list_mutex);
    err = linked_list_find_node(&ble_access_connecting_device_list,
                                (linked_list_compare_callback_t)compare_device_by_address,
                                (void *)device->address,
                                (linked_list_node_t **)&device_found);
    if (err == kNoErr) {
        device_found->reported = is_reported;
    }
    mos_mutex_unlock(ble_access_conn_dev_list_mutex);

exit:
    return err;
}

merr_t ble_access_connect_list_get(mxos_bt_smart_device_t** device, mxos_bool_t *reported)
{
    merr_t err = kNoErr;
    ble_access_connecting_device_t* current_device;

    require_action(device != NULL, exit, err = kParamErr);

    mos_mutex_lock(ble_access_conn_dev_list_mutex);
    err = linked_list_get_front_node(&ble_access_connecting_device_list, (linked_list_node_t **)&current_device);
    mos_mutex_unlock(ble_access_conn_dev_list_mutex);
    
    if (err != kNoErr) goto exit;

    *device = &current_device->device;
    if (reported) *reported = current_device->reported;

exit:
    return err;
}

merr_t ble_access_connect_list_get_by_address(mxos_bt_smart_device_t **device, mxos_bool_t *reported, const mxos_bt_device_address_t address)
{
    merr_t err = kNoErr;
    ble_access_connecting_device_t *current_device;

    require_action(device != NULL && address != NULL, exit, err = kParamErr);

    mos_mutex_lock(ble_access_conn_dev_list_mutex);
    err = linked_list_find_node(&ble_access_connecting_device_list,
                                (linked_list_compare_callback_t)compare_device_by_address,
                                (void *)address,
                                (linked_list_node_t **)&current_device);
    mos_mutex_unlock(ble_access_conn_dev_list_mutex);
    
    if (err != kNoErr) goto exit;

    *device = &current_device->device;
    if (reported) *reported = current_device->reported;

exit:
    return err;
}

merr_t ble_access_connect_list_find_by_address(const mxos_bt_device_address_t address)
{
    merr_t err = kNoErr;
    ble_access_connecting_device_t *current_device;

    mos_mutex_lock(ble_access_conn_dev_list_mutex);
    err = linked_list_find_node(&ble_access_connecting_device_list,
                                (linked_list_compare_callback_t)compare_device_by_address,
                                (void *)address,
                                (linked_list_node_t **)&current_device);
    mos_mutex_unlock(ble_access_conn_dev_list_mutex);
    return err;
}

merr_t ble_access_connect_list_remove(mxos_bt_smart_device_t *device)
{
    merr_t err = kNoErr;
    ble_access_connecting_device_t* current_device;

    mos_mutex_lock(ble_access_conn_dev_list_mutex);
    err = linked_list_find_node(&ble_access_connecting_device_list,
                                 (linked_list_compare_callback_t)compare_device_by_address,
                                 device->address,
                                 (linked_list_node_t**)&current_device);
    if (err != kNoErr) goto exit;

    err = linked_list_remove_node(&ble_access_connecting_device_list, &current_device->this_node);
    if (err != kNoErr) goto exit;

    free(current_device);
exit:
    mos_mutex_unlock(ble_access_conn_dev_list_mutex);
    return err;
}

const char *print_request_str(uint8_t request)
{
    switch (request) {
    case BLE_ACCESS_REQ_DEV_SCAN: 
        return "SCAN_REQ";
    case BLE_ACCESS_REQ_DEV_ADD: 
        return "ADD_REQ";
    case BLE_ACCESS_REQ_DEV_DISC: 
        return "DISC_REQ";
    case BLE_ACCESS_REQ_DEV_REMOVE: 
        return "REMOVE_REQ";
    case BLE_ACCESS_REQ_DEV_START_AUTO: 
        return "START_AUTO_REQ";
    case BLE_ACCESS_REQ_DEV_STOP_AUTO: 
        return "STOP_AUTO_REQ";
    default: 
        return "Unknown request";
    }
}