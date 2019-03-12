/**
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 *
 */

#pragma once

/** @file
 *  Smartbridge's Helper Function Headers
 */

//#include "mxos_utilities.h"
#include "mxos_bt_smartbridge.h"
#include "mxos_bt_smart_interface.h"

#include "mxos_bt_peripheral.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

//#define bt_smartbridge_log(M, ...) custom_log("Smartbridge", M, ##__VA_ARGS__)
#define bt_smartbridge_log(M, ...)

//#define bt_peripheral_log(M, ...) custom_log("Peripheral", M, ##__VA_ARGS__)
#define bt_peripheral_log(M, ...)
  
//#define bt_manager_log(M, ...) custom_log("Manager", M, ##__VA_ARGS__)
#define bt_manager_log(M, ...)

/******************************************************
 *                    Constants
 ******************************************************/

#define SOCKET_STATE_DISCONNECTED              ( 0 )
#define SOCKET_STATE_LINK_CONNECTED            ( 1 )
#define SOCKET_STATE_LINK_ENCRYPTED            ( 2 )
#define SOCKET_STATE_LINK_CONNECTING           ( 3 )

#define SOCKET_ACTION_HOST_CONNECT             ( 1 << 0 )
#define SOCKET_ACTION_HOST_DISCONNECT          ( 1 << 2 )
#define SOCKET_ACTION_INITIATE_PAIRING         ( 1 << 3 )
#define SOCKET_ACTION_ENCRYPT_USING_BOND_INFO  ( 1 << 4 )

/******************************************************
 *                   Enumerations
 ******************************************************/

/* GATT Feature Sub-Procedures (SPEC v4.0 Part G Section 4) */
typedef enum
{
    GATT_SUBPROCEDURE_NONE,                         /* Default Value                                       */
    GATT_EXCHANGE_MTU,                              /* GATT Feature: Server Configuration                  */
    GATT_DISCOVER_ALL_PRIMARY_SERVICES,             /* GATT Feature: Primary Service Discovery             */
    GATT_DISCOVER_PRIMARY_SERVICE_BY_SERVICE_UUID,  /* GATT Feature: Primary Service Discovery             */
    GATT_FIND_INCLUDED_SERVICES,                    /* GATT Feature: Relationship Discovery                */
    GATT_DISCOVER_ALL_CHARACTERISTICS_OF_A_SERVICE, /* GATT Feature: Characteristic Discovery              */
    GATT_DISCOVER_CHARACTERISTIC_BY_UUID,           /* GATT Feature: Characteristic Discovery              */
    GATT_DISCOVER_ALL_CHARACTERISTICS_DESCRIPTORS,  /* GATT Feature: Characteristic Descriptor Discovery   */
    GATT_READ_CHARACTERISTIC_VALUE,                 /* GATT Feature: Characteristic Value Read             */
    GATT_READ_USING_CHARACTERISTIC_UUID,            /* GATT Feature: Characteristic Value Read             */
    GATT_READ_LONG_CHARACTERISTIC_VALUES,           /* GATT Feature: Characteristic Value Read             */
    GATT_READ_MULTIPLE_CHARACTERISTIC_VALUES,       /* GATT Feature: Characteristic Value Read             */
    GATT_WRITE_WITHOUT_RESPONSE,                    /* GATT Feature: Characteristic Value Write            */
    GATT_SIGNED_WRITE_WITHOUT_RESPONSE,             /* GATT Feature: Characteristic Value Write            */
    GATT_WRITE_CHARACTERISTIC_VALUE,                /* GATT Feature: Characteristic Value Write            */
    GATT_WRITE_LONG_CHARACTERISTIC_VALUE,           /* GATT Feature: Characteristic Value Write            */
    GATT_CHARACTERISTIC_VALUE_RELIABLE_WRITES,      /* GATT Feature: Characteristic Value Write            */
    GATT_NOTIFICATIONS,                             /* GATT Feature: Characteristic Value Notification     */
    GATT_INDICATIONS,                               /* GATT Feature: Characteristic Value Indication       */
    GATT_READ_CHARACTERISTIC_DESCRIPTORS,           /* GATT Feature: Characteristic Descriptor Value Read  */
    GATT_READ_LONG_CHARACTERISTIC_DESCRIPTORS,      /* GATT Feature: Characteristic Descriptor Value Read  */
    GATT_WRITE_CHARACTERISTIC_DESCRIPTORS,          /* GATT Feature: Characteristic Descriptor Value Write */
    GATT_WRITE_LONG_CHARACTERISTIC_DESCRIPTORS,     /* GATT Feature: Characteristic Descriptor Value Write */
} bt_smart_gatt_subprocedure_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    bt_smart_gatt_subprocedure_t subprocedure;
    mxos_mutex_t                 mutex;
    mxos_semaphore_t             done_semaphore;
    merr_t                     result;
    mxos_bt_uuid_t               uuid;
    mxos_bt_smart_attribute_t*  attr_head;
    mxos_bt_smart_attribute_t*  attr_tail;
    uint32_t                     attr_count;
    uint16_t                     server_mtu;
    uint16_t                     start_handle;
    uint16_t                     end_handle;
    uint16_t                     length;
    uint16_t                     offset;
    //bt_smart_att_pdu_t*          pdu;
    uint16_t                     connection_handle;
} gatt_subprocedure_t;

typedef struct 
{
    mxos_timer_t        timer;      // MXOS timer object
    mxos_bool_t         is_started; // is it started?
    mxos_bool_t         one_shot;   // one-shot?
    timer_handler_t     handler;    // timer expired handler
    void               *context;    // user context
} smartbridge_helper_timer_t;
/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

merr_t  smartbridge_helper_delete_scan_result_list          ( void );
merr_t  smartbridge_helper_add_scan_result_to_list          ( mxos_bt_smart_scan_result_t* result );
merr_t  smartbridge_helper_find_device_in_scan_result_list  ( mxos_bt_device_address_t* address, mxos_bt_smart_address_type_t type,  mxos_bt_smart_scan_result_t** result );
merr_t  smartbridge_helper_get_scan_results                 ( mxos_bt_smart_scan_result_t** result_list, uint32_t* count );

mxos_bool_t    smartbridge_helper_socket_check_actions_enabled      ( mxos_bt_smartbridge_socket_t* socket, uint8_t action_bits );
mxos_bool_t    smartbridge_helper_socket_check_actions_disabled     ( mxos_bt_smartbridge_socket_t* socket, uint8_t action_bits );
void           smartbridge_helper_socket_set_actions               ( mxos_bt_smartbridge_socket_t* socket, uint8_t action_bits );
void           smartbridge_helper_socket_clear_actions             ( mxos_bt_smartbridge_socket_t* socket, uint8_t action_bits );



mxos_bool_t    peripheral_helper_socket_check_actions_enabled      ( mxos_bt_peripheral_socket_t* socket, uint8_t action_bits );
mxos_bool_t    peripheral_helper_socket_check_actions_disabled     ( mxos_bt_peripheral_socket_t* socket, uint8_t action_bits );
void           peripheral_helper_socket_set_actions                ( mxos_bt_peripheral_socket_t* socket, uint8_t action_bits );
void           peripheral_helper_socket_clear_actions              ( mxos_bt_peripheral_socket_t* socket, uint8_t action_bits );

merr_t  subprocedure_notify_complete               ( gatt_subprocedure_t* subprocedure );
merr_t  subprocedure_unlock                        ( gatt_subprocedure_t* subprocedure );
merr_t  subprocedure_lock                          ( gatt_subprocedure_t* subprocedure );
merr_t  subprocedure_reset                         ( gatt_subprocedure_t* subprocedure );
merr_t  subprocedure_wait_for_completion           ( gatt_subprocedure_t* subprocedure );
merr_t  subprocedure_wait_clear_semaphore          ( gatt_subprocedure_t* subprocedure );

/* This function is used to stop a active timer.
 * return MXOS_TRUE if successful, otherwise, return MXOS_FALSE
 */
mxos_bool_t smartbridge_helper_timer_stop(smartbridge_helper_timer_t *timer);

/* This function is used to start or restart a timer.
 * The 'timer' will be restarted if it is active.
 */
mxos_bool_t smartbridge_helper_timer_start(smartbridge_helper_timer_t *timer, mxos_bool_t one_shot, uint32_t ms, timer_handler_t handler, void *arg);

#ifdef __cplusplus
} /* extern "C" */
#endif
