/**
 ******************************************************************************
 * @file    ble_access_core_i.h
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

#ifndef __BLE_ACCESS_CORE_I_H__
#define __BLE_ACCESS_CORE_I_H__

#include "mxos.h"
#include "mxos_bt.h"

/*
 *-------------------------------------------------------------------------------------------------
 *
 *  Configurations & Constants 
 *
 *-------------------------------------------------------------------------------------------------
 */

/* Debug Logout */
#define BLE_ACCESS_DEBUG                             0 

/* The maximum number of remote connections */
#define MAX_CONCURRENT_CONNECTIONS                   10

/* Protocal Flags */
#define BLE_ACCESS_MXCHIP_FLAG                      "MXCHIP"

/*
 * Array index of Command Packet
 *
 * ----------------------------------------
 * | code | count | length | len1 | data1 |
 * ----------------------------------------
 *
 */
#define BLE_ACCESS_AUTH_IDX_CODE                    0x00
#define BLE_ACCESS_AUTH_IDX_COUNT                   0x01
#define BLE_ACCESS_AUTH_IDX_LENGTH                  0x02
#define BLE_ACCESS_AUTH_IDX_PARM1_LEN               0x04
#define BLE_ACCESS_AUTH_IDX_PARM1_DAT               0x05

/*
 * Code Value
 */
#define BLE_ACCESS_CODE_START_AUTH                  0x11
#define BLE_ACCESS_CODE_AUTH_DATA                   0x12
#define BLE_ACCESS_CODE_AUTH_RESULT                 0x13

/*
 * Res Value
 */
#define BLE_ACCESS_RES_START                        0x21
#define BLE_ACCESS_RES_AUTH                         0x22

#define BLE_ACCESS_CODE_DISC                        0x31
#define BLE_ACCESS_RES_DISC                         0x41

/* Response to Peer */
//#define BLE_ACCESS_RSP_ALLOW_CONNECT       0x01
//#define BLE_ACCESS_RSP_NOT_ALLOW_CONNECT   0x02
/* Alert code to peer */
#define BLE_ACCESS_RSP_ALERTING                     0x03

/* Slave command to authentication */
#define BLE_ACCESS_SLAVE_CMD_AUTH_START             0x11
#define BLE_ACCESS_SLAVE_CMD_AUTH_DATA              0x12
#define BLE_ACCESS_SLAVE_CMD_AUTH_RESULT            0x13

/* Master response to authentication */
#define BLE_ACCESS_MASTER_RSP_AUTH_ACK              0x21
#define BLE_ACCESS_MASTER_RSP_AUTH_DATA             0x22

/* EVENT Error Code */
#define BLE_ACCESS_ERR_NO_ERR                       0x00
#define BLE_ACCESS_ERR_ENCRY_FAILED                 0x01
#define BLE_ACCESS_ERR_BUSY                         0x02
#define BLE_ACCESS_ERR_AUTH_FAILED                  0x03
#define BLE_ACCESS_ERR_TIMEOUT                      0x04
#define BLE_ACCESS_ERR_REJECT                       0x05

#define BLE_ACCESS_ERR_ALERTING                     0x10
#define BLE_ACCESS_ERR_NO_ALERT                     0x11

/* Control Point Service */
// {7A414C35-605A-4319-AB3C-E945A9A42A70}
#define UUID_MXCHIP_CONTROL_POINT_SERVICE           0x70,0x2a,0xa4,0xa9,0x45,0xe9,0x3c,0xab,0x19,0x43,0x5a,0x60,0x35,0x4c,0x41,0x7a
// {B1CADCA2-9E80-4D24-9313-A45125A5F846}
#define UUID_MXCHIP_CONTROL_POINT_CMD_CHAR          0x46,0xf8,0xa5,0x25,0x51,0xa4,0x13,0x93,0x24,0x4d,0x80,0x9e,0xa2,0xdc,0xca,0xb1
// {E36945B5-30EE-4ABD-9445-4122CB40D175}
#define UUID_MXCHIP_CONTROL_POINT_EVT_CHAR          0x75,0xd1,0x40,0xcb,0x22,0x41,0x45,0x94,0xbd,0x4a,0xee,0x30,0xb5,0x45,0x69,0xe3


/*
 *-------------------------------------------------------------------------------------------------
 *
 *  Predefine Type
 *
 *-------------------------------------------------------------------------------------------------
 */

/* Device info ready to connect */
typedef struct {
    linked_list_node_t          this_node;     /* Linked-list node of this device */
    mxos_bt_smart_device_t      device;        /* Remote BT device */
    mxos_bool_t                 reported;      /* Is reported? */
} ble_access_connecting_device_t;

/* The Peer device Advertisement data */
typedef struct {
    uint8_t                     mxchip[6];
    uint8_t                     adv_type;
    uint8_t                     alert_state;
    mxos_bt_device_address_t    direct_addr;
} ble_access_manufactor_data_t;

/*
 * A BLE Access Core BLE Device 
 */
typedef struct {
    // Device ID 
    uint32_t                        device_id;
    // BT Socket for Every BLE Connection
    mxos_bt_smartbridge_socket_t    socket;
    // A timer for Authentication Process 
    mxos_timer_t                    timer;
    // Authentication State 
    uint8_t                         auth_state;
    // Is used ?
    mxos_bool_t                     used;
    // Some handles for a service of a BLE Connection.
    struct {
        // Notify Characteristic Value handle in MXCHIP Service
        uint16_t                    notify_char_value_handle;
        // Control Point Characteristic Value handle in MXCHIP Service.
        uint16_t                    ctrl_evt_char_value_handle;
    } service;
} ble_access_device_t;

/*
 *-------------------------------------------------------------------------------------------------
 *
 *  Local Function Prototype 
 *
 *-------------------------------------------------------------------------------------------------
 */

/* Log output */
#define ble_access_log(M, ...)          custom_log("BLE_ACCESS", M, ##__VA_ARGS__)

/* Calculate An array size */
#define ble_access_array_size(array)    (sizeof(array)/sizeof(array[0]))

/* 
 * Remote device list management 
 */
extern mret_t ble_access_connect_list_init           (void);
extern mret_t ble_access_connect_list_deinit         (void);

extern mret_t ble_access_connect_list_add            (const mxos_bt_smart_device_t *remote_device, mxos_bool_t is_reported);
extern mret_t ble_access_connect_list_get            (mxos_bt_smart_device_t **address, mxos_bool_t *reported);
extern mret_t ble_access_connect_list_get_by_address (mxos_bt_smart_device_t **device, mxos_bool_t *reported, const mxos_bt_device_address_t address);
extern mret_t ble_access_connect_list_find_by_address(const mxos_bt_device_address_t address);
extern mret_t ble_access_connect_list_remove         (mxos_bt_smart_device_t* device);
extern mret_t ble_access_connect_list_set_report     (const mxos_bt_smart_device_t *device, mxos_bool_t is_reported);

/*
 * Utils function
 */

const char *print_request_str(uint8_t request);

extern mret_t ble_access_get_manufactor_adv_data(uint8_t *eir_data,
                                                     uint8_t  eir_data_length,
                                                     ble_access_manufactor_data_t *manufactor_data);

extern mret_t ble_access_check_adv_type         (const uint8_t *adv_data,
                                                   uint8_t length,
                                                   uint8_t adv_type,
                                                   ble_access_manufactor_data_t *manu_data);

extern void ble_access_set_scan_cfg               (mxos_bt_smart_scan_settings_t *scan_cfg, 
                                                   mxos_bool_t is_auto_scanning);

extern mret_t ble_access_start_timer            (ble_access_device_t *dev, event_handler_t timer_event_handle, void *arg);
extern mret_t ble_access_stop_timer             (ble_access_device_t *dev);

/*
 * Local Devices Pool
 */
extern void                 ble_access_initialize_devices     (void);
extern void                 ble_access_deinit_devices         (void);
extern ble_access_device_t *ble_access_get_free_device        (void);
extern void                 ble_access_release_device         (mxos_bool_t free, const ble_access_device_t *device);
//extern ble_access_device_t *ble_access_find_device_by_socket  (const mxos_bt_smartbridge_socket_t *socket);
extern ble_access_device_t *ble_access_find_device_by_address (const mxos_bt_device_address_t address);

extern mret_t ble_access_create_worker_thread(void);
extern mret_t ble_access_send_aync_event(event_handler_t event_handle, void *arg);

#endif /* __BLE_ACCESS_CORE_I_H__ */
