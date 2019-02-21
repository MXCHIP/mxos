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

//#include "mxos_utilities.h"
#include "mxos_bt_smartbridge.h"
#include "LinkListUtils.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

OSStatus bt_smartbridge_socket_manager_init( void );

OSStatus bt_smartbridge_socket_manager_deinit( void );

OSStatus bt_smartbridge_socket_manager_set_max_concurrent_connections( uint8_t count );

mxos_bool_t   bt_smartbridge_socket_manager_is_full( void );

OSStatus bt_smartbridge_socket_manager_insert_socket( mxos_bt_smartbridge_socket_t* socket );

OSStatus bt_smartbridge_socket_manager_remove_socket( uint16_t connection_handle, mxos_bt_smartbridge_socket_t** socket );

OSStatus bt_smartbridge_socket_manager_find_socket_by_handle( uint16_t connection_handle, mxos_bt_smartbridge_socket_t** socket );

OSStatus bt_smartbridge_socket_manager_find_socket_by_address( const mxos_bt_device_address_t* address, mxos_bt_smartbridge_socket_t** socket );

#ifdef __cplusplus
} /* extern "C" */
#endif
