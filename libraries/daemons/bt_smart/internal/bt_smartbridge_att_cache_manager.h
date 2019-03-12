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
#include "mxos_bt_smart_interface.h"
#include "bt_smartbridge_att_cache_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define MAX_CACHED_ATTRIBUTES_INSTANCES ( MAX_BT_SMART_CONNECTIONS )

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct bt_smartbridge_att_cache bt_smartbridge_att_cache_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

mret_t bt_smartbridge_att_cache_enable( uint32_t cache_count, mxos_bt_uuid_t cache_services[], uint32_t service_count );

mret_t bt_smartbridge_att_cache_disable( void );

mxos_bool_t   bt_smartbridge_att_cache_is_enabled( void );

mxos_bool_t   bt_smartbridge_att_cache_is_discovering( const bt_smartbridge_att_cache_t* cache );

mxos_bool_t   bt_smartbridge_att_cache_get_active_state( const bt_smartbridge_att_cache_t* cache );

mret_t bt_smartbridge_att_cache_set_active_state( bt_smartbridge_att_cache_t* cache, mxos_bool_t is_active );

mret_t bt_smartbridge_att_cache_find( const mxos_bt_smart_device_t* remote_device, bt_smartbridge_att_cache_t** cache );

mret_t bt_smartbridge_att_cache_generate( const mxos_bt_smart_device_t* remote_device, uint16_t connection_handle, bt_smartbridge_att_cache_t** cache );

mret_t bt_smartbridge_att_cache_release( bt_smartbridge_att_cache_t* cache );

mret_t bt_smartbridge_att_cache_get_list( bt_smartbridge_att_cache_t* cache, mxos_bt_smart_attribute_list_t** list );

mret_t bt_smartbridge_att_cache_lock( bt_smartbridge_att_cache_t* cache );

mret_t bt_smartbridge_att_cache_unlock( bt_smartbridge_att_cache_t* cache );

#ifdef __cplusplus
} /* extern "C" */
#endif
