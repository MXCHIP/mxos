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
#include "mxos_bt_smartbridge.h"
#include "mxos_bt_smartbridge_gatt.h"
#include "bt_smartbridge_stack_interface.h"
#include "bt_smartbridge_helper.h"

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

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

mret_t mxos_bt_smartbridge_gatt_discover_all_primary_services( const mxos_bt_smartbridge_socket_t* socket, mxos_bt_smart_attribute_list_t* service_list )
{
    mxos_bt_smartbridge_socket_status_t status;

    if ( socket == NULL || service_list == NULL )
    {
        return MXOS_BT_BADARG;
    }

    mxos_bt_smartbridge_get_socket_status( (mxos_bt_smartbridge_socket_t*)socket, &status );

    if ( status != SMARTBRIDGE_SOCKET_CONNECTED )
    {
        return MXOS_BT_SOCKET_NOT_CONNECTED;
    }

    return smartbridge_bt_interface_discover_all_primary_services( socket->connection_handle, service_list );
}

mret_t mxos_bt_smartbridge_gatt_discover_primary_services_by_uuid( const mxos_bt_smartbridge_socket_t* socket, const mxos_bt_uuid_t* uuid, mxos_bt_smart_attribute_list_t* service_list )
{
    mxos_bt_smartbridge_socket_status_t status;

    if ( socket == NULL || uuid == NULL || service_list == NULL )
    {
        return MXOS_BT_BADARG;
    }

    mxos_bt_smartbridge_get_socket_status( (mxos_bt_smartbridge_socket_t*)socket, &status );
    if ( status != SMARTBRIDGE_SOCKET_CONNECTED )
    {
        return MXOS_BT_SOCKET_NOT_CONNECTED;
    }

    return smartbridge_bt_interface_discover_primary_services_by_uuid( socket->connection_handle, uuid, service_list );
}

mret_t mxos_bt_smartbridge_gatt_find_included_services( const mxos_bt_smartbridge_socket_t* socket, uint16_t start_handle, uint16_t end_handle, mxos_bt_smart_attribute_list_t* include_list )
{
    mxos_bt_smartbridge_socket_status_t status;

    if ( socket == NULL || include_list == NULL )
    {
        return MXOS_BT_BADARG;
    }

    mxos_bt_smartbridge_get_socket_status( (mxos_bt_smartbridge_socket_t*)socket, &status );
    if ( status != SMARTBRIDGE_SOCKET_CONNECTED )
    {
        return MXOS_BT_SOCKET_NOT_CONNECTED;
    }

    return smartbridge_bt_interface_find_included_services( socket->connection_handle, start_handle, end_handle, include_list );
}

mret_t mxos_bt_smartbridge_gatt_discover_all_characteristics_in_a_service( const mxos_bt_smartbridge_socket_t* socket, uint16_t start_handle, uint16_t end_handle, mxos_bt_smart_attribute_list_t* characteristic_list )
{
    mxos_bt_smartbridge_socket_status_t status;

    if ( socket == NULL || characteristic_list == NULL )
    {
        return MXOS_BT_BADARG;
    }

    mxos_bt_smartbridge_get_socket_status( (mxos_bt_smartbridge_socket_t*)socket, &status );
    if ( status != SMARTBRIDGE_SOCKET_CONNECTED )
    {
        return MXOS_BT_SOCKET_NOT_CONNECTED;
    }
    return smartbridge_bt_interface_discover_all_characteristics_in_a_service( socket->connection_handle, start_handle, end_handle, characteristic_list );
}

mret_t mxos_bt_smartbridge_gatt_discover_characteristic_by_uuid( const mxos_bt_smartbridge_socket_t* socket, const mxos_bt_uuid_t* uuid, uint16_t start_handle, uint16_t end_handle, mxos_bt_smart_attribute_list_t* characteristic_list )
{
    mxos_bt_smartbridge_socket_status_t status;

    if ( socket == NULL || uuid == NULL || characteristic_list == NULL )
    {
        return MXOS_BT_BADARG;
    }

    mxos_bt_smartbridge_get_socket_status( (mxos_bt_smartbridge_socket_t*)socket, &status );
    if ( status != SMARTBRIDGE_SOCKET_CONNECTED )
    {
        return MXOS_BT_SOCKET_NOT_CONNECTED;
    }

    return smartbridge_bt_interface_discover_characteristic_by_uuid( socket->connection_handle, uuid, start_handle, end_handle, characteristic_list );
}

mret_t mxos_bt_smartbridge_gatt_discover_handle_and_type_of_all_characteristic_descriptors( const mxos_bt_smartbridge_socket_t* socket, uint16_t start_handle, uint16_t end_handle, mxos_bt_smart_attribute_list_t* descriptor_list )
{
    mxos_bt_smartbridge_socket_status_t status;

    if ( socket == NULL || descriptor_list == NULL )
    {
        return MXOS_BT_BADARG;
    }

    mxos_bt_smartbridge_get_socket_status( (mxos_bt_smartbridge_socket_t*)socket, &status );
    if ( status != SMARTBRIDGE_SOCKET_CONNECTED )
    {
        return MXOS_BT_SOCKET_NOT_CONNECTED;
    }

    return smartbridge_bt_interface_discover_all_characteristic_descriptors( socket->connection_handle, start_handle, end_handle, descriptor_list );
}

mret_t mxos_bt_smartbridge_gatt_read_characteristic_descriptor( const mxos_bt_smartbridge_socket_t* socket, uint16_t handle, const mxos_bt_uuid_t* uuid, mxos_bt_smart_attribute_t** descriptor )
{
    mxos_bt_smartbridge_socket_status_t status;

    if ( socket == NULL || uuid == NULL || descriptor == NULL )
    {
        return MXOS_BT_BADARG;
    }

    mxos_bt_smartbridge_get_socket_status( (mxos_bt_smartbridge_socket_t*)socket, &status );
    if ( status != SMARTBRIDGE_SOCKET_CONNECTED )
    {
        return MXOS_BT_SOCKET_NOT_CONNECTED;
    }

    return smartbridge_bt_interface_read_characteristic_descriptor( socket->connection_handle, handle, uuid, descriptor );
}

mret_t mxos_bt_smartbridge_gatt_read_long_characteristic_descriptor( const mxos_bt_smartbridge_socket_t* socket, uint16_t handle, const mxos_bt_uuid_t* uuid, mxos_bt_smart_attribute_t** descriptor )
{
    mxos_bt_smartbridge_socket_status_t status;

    if ( socket == NULL || uuid == NULL || descriptor == NULL )
    {
        return MXOS_BT_BADARG;
    }

    mxos_bt_smartbridge_get_socket_status( (mxos_bt_smartbridge_socket_t*)socket, &status );
    if ( status != SMARTBRIDGE_SOCKET_CONNECTED )
    {
        return MXOS_BT_SOCKET_NOT_CONNECTED;
    }

    return smartbridge_bt_interface_read_long_characteristic_descriptor( socket->connection_handle, handle, uuid, descriptor );
}

mret_t mxos_bt_smartbridge_gatt_write_characteristic_descriptor( const mxos_bt_smartbridge_socket_t* socket, const mxos_bt_smart_attribute_t* descriptor )
{
    mxos_bt_smartbridge_socket_status_t status;

    if ( socket == NULL || descriptor == NULL )
    {
        return MXOS_BT_BADARG;
    }

    mxos_bt_smartbridge_get_socket_status( (mxos_bt_smartbridge_socket_t*)socket, &status );
    if ( status != SMARTBRIDGE_SOCKET_CONNECTED )
    {
        return MXOS_BT_SOCKET_NOT_CONNECTED;
    }

    return smartbridge_bt_interface_write_characteristic_descriptor( socket->connection_handle, (mxos_bt_smart_attribute_t*)descriptor );
}

mret_t mxos_bt_smartbridge_gatt_write_long_characteristic_descriptor( const mxos_bt_smartbridge_socket_t* socket, const mxos_bt_smart_attribute_t* descriptor )
{
    mxos_bt_smartbridge_socket_status_t status;

    if ( socket == NULL || descriptor == NULL )
    {
        return MXOS_BT_BADARG;
    }

    mxos_bt_smartbridge_get_socket_status( (mxos_bt_smartbridge_socket_t*)socket, &status );
    if ( status != SMARTBRIDGE_SOCKET_CONNECTED )
    {
        return MXOS_BT_SOCKET_NOT_CONNECTED;
    }

    return smartbridge_bt_interface_write_long_characteristic_descriptor( socket->connection_handle, (mxos_bt_smart_attribute_t*)descriptor );
}

mret_t mxos_bt_smartbridge_gatt_read_characteristic_value( const mxos_bt_smartbridge_socket_t* socket, uint16_t handle, const mxos_bt_uuid_t* uuid, mxos_bt_smart_attribute_t** characteristic_value )
{
    mxos_bt_smartbridge_socket_status_t status;

    if ( socket == NULL || uuid == NULL || characteristic_value == NULL )
    {
        return MXOS_BT_BADARG;
    }

    mxos_bt_smartbridge_get_socket_status( (mxos_bt_smartbridge_socket_t*)socket, &status );
    if ( status != SMARTBRIDGE_SOCKET_CONNECTED )
    {
        return MXOS_BT_SOCKET_NOT_CONNECTED;
    }

    return smartbridge_bt_interface_read_characteristic_value( socket->connection_handle, handle, uuid, characteristic_value );
}

mret_t mxos_bt_smartbridge_gatt_read_characteristic_values_using_uuid( const mxos_bt_smartbridge_socket_t* socket,  const mxos_bt_uuid_t* uuid, mxos_bt_smart_attribute_list_t* characteristic_value_list )
{
    mxos_bt_smartbridge_socket_status_t status;

    if ( socket == NULL || uuid == NULL || characteristic_value_list == NULL )
    {
        return MXOS_BT_BADARG;
    }

    mxos_bt_smartbridge_get_socket_status( (mxos_bt_smartbridge_socket_t*)socket, &status );
    if ( status != SMARTBRIDGE_SOCKET_CONNECTED )
    {
        return MXOS_BT_SOCKET_NOT_CONNECTED;
    }

    return smartbridge_bt_interface_read_characteristic_values_using_uuid( socket->connection_handle, uuid, characteristic_value_list );
}

mret_t mxos_bt_smartbridge_gatt_read_long_characteristic_value( const mxos_bt_smartbridge_socket_t* socket, uint16_t handle, const mxos_bt_uuid_t* uuid, mxos_bt_smart_attribute_t** characteristic_value )
{
    mxos_bt_smartbridge_socket_status_t status;

    if ( socket == NULL || uuid == NULL || characteristic_value == NULL )
    {
        return MXOS_BT_BADARG;
    }

    mxos_bt_smartbridge_get_socket_status( (mxos_bt_smartbridge_socket_t*)socket, &status );
    if ( status != SMARTBRIDGE_SOCKET_CONNECTED )
    {
        return MXOS_BT_SOCKET_NOT_CONNECTED;
    }

    return smartbridge_bt_interface_read_long_characteristic_value( socket->connection_handle, handle, uuid, characteristic_value );
}

mret_t mxos_bt_smartbridge_gatt_write_characteristic_value_without_response( const mxos_bt_smartbridge_socket_t* socket, const mxos_bt_smart_attribute_t* characteristic_value )
{
    UNUSED_PARAMETER( socket );
    UNUSED_PARAMETER( characteristic_value );
    return MXOS_BT_UNSUPPORTED;
}

mret_t mxos_bt_smartbridge_gatt_signed_write_characteristic_value_without_response( const mxos_bt_smartbridge_socket_t* socket, const mxos_bt_smart_attribute_t* characteristic_value )
{
    UNUSED_PARAMETER( socket );
    UNUSED_PARAMETER( characteristic_value );
    return MXOS_BT_UNSUPPORTED;
}

mret_t mxos_bt_smartbridge_gatt_write_characteristic_value( const mxos_bt_smartbridge_socket_t* socket, const mxos_bt_smart_attribute_t* characteristic_value )
{
    mxos_bt_smartbridge_socket_status_t status;

    if ( socket == NULL || characteristic_value == NULL )
    {
        return MXOS_BT_BADARG;
    }

    mxos_bt_smartbridge_get_socket_status( (mxos_bt_smartbridge_socket_t*)socket, &status );
    if ( status != SMARTBRIDGE_SOCKET_CONNECTED )
    {
        return MXOS_BT_SOCKET_NOT_CONNECTED;
    }
    return smartbridge_bt_interface_write_characteristic_value( socket->connection_handle, (mxos_bt_smart_attribute_t*)characteristic_value );
}

mret_t mxos_bt_smartbridge_gatt_write_long_characteristic_value( const mxos_bt_smartbridge_socket_t* socket, const mxos_bt_smart_attribute_t* characteristic_value )
{
    mxos_bt_smartbridge_socket_status_t status;

    if ( socket == NULL || characteristic_value == NULL )
    {
        return MXOS_BT_BADARG;
    }

    mxos_bt_smartbridge_get_socket_status( (mxos_bt_smartbridge_socket_t*)socket, &status );
    if ( status != SMARTBRIDGE_SOCKET_CONNECTED )
    {
        return MXOS_BT_SOCKET_NOT_CONNECTED;
    }

    return smartbridge_bt_interface_write_long_characteristic_value( socket->connection_handle, (mxos_bt_smart_attribute_t*)characteristic_value );
}

mret_t mxos_bt_smartbridge_gatt_reliable_write_characteristic_value( const mxos_bt_smartbridge_socket_t* socket, const mxos_bt_smart_attribute_t* characteristic_value )
{
    UNUSED_PARAMETER( socket );
    UNUSED_PARAMETER( characteristic_value );    
    return MXOS_BT_UNSUPPORTED;
}
