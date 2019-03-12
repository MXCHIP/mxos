/**
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 *
 */

/** @file
 *
 */

#include "mxos.h"
#include "LinkListUtils.h"
#include "bt_smartbridge_socket_manager.h"

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
 *               Static Function Declarations
 ******************************************************/

static bool smartbridge_socket_manager_find_socket_by_handle_callback  ( linked_list_node_t* node_to_compare, void* user_data );
static bool smartbridge_socket_manager_find_socket_by_address_callback ( linked_list_node_t* node_to_compare, void* user_data );

/******************************************************
 *               Variable Definitions
 ******************************************************/

/* Socket Management Globals */
static linked_list_t connected_socket_list;
static mxos_mutex_t  connected_socket_list_mutex;
static uint8_t       max_number_of_connections = 0;

/******************************************************
 *               Function Definitions
 ******************************************************/

mret_t bt_smartbridge_socket_manager_init( void )
{
    mret_t result;

    result = linked_list_init( &connected_socket_list );
    if ( result != kNoErr )
    {
        WPRINT_LIB_INFO( ( "Error creating linked list\n" ) );
        return result;
    }

    result = mxos_rtos_init_mutex( &connected_socket_list_mutex );
    if ( result != kNoErr )
    {
        WPRINT_LIB_INFO( ( "Error creating mutex\n" ) );
        return result;
    }

    max_number_of_connections = 1;

    return MXOS_BT_SUCCESS;
}

mret_t bt_smartbridge_socket_manager_deinit( void )
{
    mxos_rtos_deinit_mutex( &connected_socket_list_mutex );
    linked_list_deinit( &connected_socket_list );
    max_number_of_connections = 0;
    return MXOS_BT_SUCCESS;
}

mret_t bt_smartbridge_socket_manager_set_max_concurrent_connections( uint8_t count )
{
    max_number_of_connections = count;
    return MXOS_BT_SUCCESS;
}

mxos_bool_t   bt_smartbridge_socket_manager_is_full( void )
{
    uint32_t active_connection_count;

    linked_list_get_count( &connected_socket_list, &active_connection_count );

    return ( active_connection_count == max_number_of_connections ) ? MXOS_TRUE : MXOS_FALSE;
}

mret_t bt_smartbridge_socket_manager_insert_socket( mxos_bt_smartbridge_socket_t* socket )
{
    mret_t  result;
    uint32_t  count;

    linked_list_get_count( &connected_socket_list, &count );

    if ( count == max_number_of_connections )
    {
        return MXOS_BT_MAX_CONNECTIONS_REACHED;
    }

    /* Lock protection */
    mxos_rtos_lock_mutex( &connected_socket_list_mutex );

    result = linked_list_insert_node_at_rear( &connected_socket_list, &socket->node );

    /* Unlock protection */
    mxos_rtos_unlock_mutex( &connected_socket_list_mutex );

    return result;
}

mret_t bt_smartbridge_socket_manager_remove_socket( uint16_t connection_handle, mxos_bt_smartbridge_socket_t** socket )
{
    mret_t            result;
    uint32_t            count;
    linked_list_node_t* node_found;
    uint32_t            user_data = connection_handle;

    linked_list_get_count( &connected_socket_list, &count );

    if ( count == 0 )
    {
        return MXOS_BT_SOCKET_NOT_CONNECTED;
    }

    /* Lock protection */
    mxos_rtos_lock_mutex( &connected_socket_list_mutex );

    result = linked_list_find_node( &connected_socket_list, smartbridge_socket_manager_find_socket_by_handle_callback, (void*)user_data, &node_found );
    if ( result == MXOS_BT_SUCCESS )
    {
        result = linked_list_remove_node( &connected_socket_list, node_found );

        if ( result == MXOS_BT_SUCCESS )
        {
            *socket = (mxos_bt_smartbridge_socket_t*)node_found->data;
        }
    }

    /* Unlock protection */
    mxos_rtos_unlock_mutex( &connected_socket_list_mutex );

    return result;
}

mret_t bt_smartbridge_socket_manager_find_socket_by_handle( uint16_t connection_handle, mxos_bt_smartbridge_socket_t** socket )
{
    mret_t            result;
    uint32_t            count;
    linked_list_node_t* node_found;
    uint32_t            user_data = connection_handle;

    linked_list_get_count( &connected_socket_list, &count );

    if ( count == 0 )
    {
        return MXOS_BT_SOCKET_NOT_CONNECTED;
    }

    /* Lock protection */
    mxos_rtos_lock_mutex( &connected_socket_list_mutex );

    result = linked_list_find_node( &connected_socket_list, smartbridge_socket_manager_find_socket_by_handle_callback, (void*)user_data, &node_found );

    if ( result == MXOS_BT_SUCCESS )
    {
        *socket = (mxos_bt_smartbridge_socket_t*)node_found->data;
    }

    /* Unlock protection */
    mxos_rtos_unlock_mutex( &connected_socket_list_mutex );

    return result;
}

mret_t bt_smartbridge_socket_manager_find_socket_by_address( const mxos_bt_device_address_t* address, mxos_bt_smartbridge_socket_t** socket )
{
    mret_t            result;
    uint32_t            count;
    linked_list_node_t* node_found;

    linked_list_get_count( &connected_socket_list, &count );

    if ( count == 0 )
    {
        return MXOS_BT_SOCKET_NOT_CONNECTED;
    }

    /* Lock protection */
    mxos_rtos_lock_mutex( &connected_socket_list_mutex );

    result = linked_list_find_node( &connected_socket_list, smartbridge_socket_manager_find_socket_by_address_callback, (void*)address, &node_found );

    if ( result == MXOS_BT_SUCCESS )
    {
        *socket = (mxos_bt_smartbridge_socket_t*)node_found->data;
    }

    /* Unlock protection */
    mxos_rtos_unlock_mutex( &connected_socket_list_mutex );

    return result;
}

static bool smartbridge_socket_manager_find_socket_by_handle_callback( linked_list_node_t* node_to_compare, void* user_data )
{
    mxos_bt_smartbridge_socket_t* socket = (mxos_bt_smartbridge_socket_t*)node_to_compare->data;
    uint32_t connection_handle = (uint32_t)user_data;

    return ( socket->connection_handle == connection_handle ) ? true : false;
}

static bool smartbridge_socket_manager_find_socket_by_address_callback( linked_list_node_t* node_to_compare, void* user_data )
{
    mxos_bt_smartbridge_socket_t* socket = (mxos_bt_smartbridge_socket_t*)node_to_compare->data;
    mxos_bt_device_address_t*      address = (mxos_bt_device_address_t*)user_data;

    return ( memcmp( socket->remote_device.address, address, sizeof( *address ) ) == 0 ) ? true : false;
}
