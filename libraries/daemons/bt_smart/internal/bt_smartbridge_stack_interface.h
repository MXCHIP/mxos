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
 *  Smartbridge's Interface Header with Bluetooth Stack
 */

//#include "mxos_utilities.h"
#include "mxos_bt_smartbridge.h"
#include "mxos_bt_smart_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define ATT_DEFAULT_MTU           (23)
#define ATT_STANDARD_VALUE_LENGTH (ATT_DEFAULT_MTU - 3)
#define ATT_STANDARD_TIMEOUT      (500)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/** 
 * SmartBridge background connection type 
 */
typedef enum
{
    SMARTBRIDGE_CONN_NONE,                          /**< No background connection */
    SMARTBRIDGE_CONN_AUTO,                          /**< Auto connection */
    SMARTBRIDGE_CONN_SELECTIVE                      /**< Selective connection */
} mxos_bt_smartbridge_auto_connection_type_t;


/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

mret_t smartbridge_bt_interface_initialize( void );
mret_t smartbridge_bt_interface_deinitialize( void );

mret_t  smartbridge_bt_interface_stop_scan( void  );

mret_t  smartbridge_bt_interface_start_scan( const mxos_bt_smart_scan_settings_t* setting, mxos_bt_smart_scan_complete_callback_t complete_callback, mxos_bt_smart_advertising_report_callback_t advertising_report_callback );
mxos_bool_t    smartbridge_bt_interface_is_scanning( void );

mret_t  smartbridge_bt_interface_connect( const mxos_bt_smart_device_t* remote_device, const mxos_bt_smart_connection_settings_t* settings, mxos_bt_smartbridge_disconnection_callback_t disconnection_callback, mxos_bt_smartbridge_notification_callback_t notification_callback );

mret_t  smartbridge_bt_interface_cancel_last_connect( mxos_bt_device_address_t address );

mret_t  smartbridge_bt_interface_disconnect( uint16_t connection_handle );

mret_t  smartbridge_bt_interface_update_background_connection_device( mxos_bool_t add, mxos_bt_device_address_t device_address );

mret_t  smartbridge_bt_interface_get_background_connection_device_size( uint8_t *size );

mret_t  smartbridge_bt_interface_set_background_connection_type(mxos_bt_smartbridge_auto_connection_type_t type, const mxos_bt_smart_scan_settings_t* settings, mxos_bt_smartbridge_auto_connection_parms_cback_t );

mret_t  smartbridge_bt_interface_set_attribute_timeout( uint32_t timeout_seconds );

mret_t  smartbridge_bt_interface_set_connection_tx_power( uint16_t connection_handle, int8_t transmit_power_dbm );

mret_t  smartbridge_bt_interface_set_max_concurrent_connections( uint8_t count );


mret_t smartbridge_bt_interface_discover_all_primary_services( uint16_t connection_handle, mxos_bt_smart_attribute_list_t* service_list );
mret_t smartbridge_bt_interface_discover_all_characteristics_in_a_service( uint16_t connection_handle, uint16_t start_handle, uint16_t end_handle, mxos_bt_smart_attribute_list_t* characteristic_list );
mret_t smartbridge_bt_interface_discover_all_characteristic_descriptors( uint16_t connection_handle, uint16_t start_handle, uint16_t end_handle, mxos_bt_smart_attribute_list_t* no_value_descriptor_list );
mret_t smartbridge_bt_interface_discover_primary_services_by_uuid( uint16_t connection_handle, const mxos_bt_uuid_t* uuid, mxos_bt_smart_attribute_list_t* service_list );
mret_t smartbridge_bt_interface_find_included_services( uint16_t connection_handle, uint16_t start_handle, uint16_t end_handle, mxos_bt_smart_attribute_list_t* include_list );
mret_t smartbridge_bt_interface_discover_characteristic_by_uuid( uint16_t connection_handle, const mxos_bt_uuid_t* uuid, uint16_t start_handle, uint16_t end_handle, mxos_bt_smart_attribute_list_t* characteristic_list );

mret_t smartbridge_bt_interface_read_characteristic_value( uint16_t connection_handle, uint16_t handle, const mxos_bt_uuid_t* type, mxos_bt_smart_attribute_t** characteristic_value );
mret_t smartbridge_bt_interface_read_long_characteristic_value( uint16_t connection_handle, uint16_t handle, const mxos_bt_uuid_t* type, mxos_bt_smart_attribute_t** characteristic_value );
mret_t smartbridge_bt_interface_read_characteristic_descriptor( uint16_t connection_handle, uint16_t handle, const mxos_bt_uuid_t* uuid, mxos_bt_smart_attribute_t** descriptor );
mret_t smartbridge_bt_interface_read_long_characteristic_descriptor( uint16_t connection_handle, uint16_t handle, const mxos_bt_uuid_t* uuid, mxos_bt_smart_attribute_t** descriptor );
mret_t smartbridge_bt_interface_read_characteristic_values_using_uuid( uint16_t connection_handle, const mxos_bt_uuid_t* uuid, mxos_bt_smart_attribute_list_t* characteristic_value_list );

mret_t smartbridge_bt_interface_write_characteristic_value( uint16_t connection_handle, mxos_bt_smart_attribute_t* attribute );
mret_t smartbridge_bt_interface_write_long_characteristic_value( uint16_t connection_handle, mxos_bt_smart_attribute_t* attribute );
mret_t smartbridge_bt_interface_write_long_characteristic_descriptor( uint16_t connection_handle, const mxos_bt_smart_attribute_t* descriptor );
mret_t smartbridge_bt_interface_write_characteristic_descriptor(  uint16_t connection_handle, mxos_bt_smart_attribute_t* attribute );

#ifdef __cplusplus
} /* extern "C" */
#endif
