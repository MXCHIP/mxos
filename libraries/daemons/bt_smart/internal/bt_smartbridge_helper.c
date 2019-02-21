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
#include "mxos_bt_peripheral.h"

#include "mxos_bt_gatt.h"
#include "mxos_bt_ble.h"
#include "mxos_bt_cfg.h"

#include "bt_smartbridge_socket_manager.h"
#include "bt_smartbridge_att_cache_manager.h"
#include "bt_smartbridge_helper.h"
#include "bt_smartbridge_stack_interface.h"



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

static mxos_bt_smart_scan_result_t*                scan_result_head        = NULL;
static mxos_bt_smart_scan_result_t*                scan_result_tail        = NULL;
static uint32_t                                     scan_result_count       = 0;

/******************************************************
 *               Function Definitions
 ******************************************************/

OSStatus smartbridge_helper_get_scan_results( mxos_bt_smart_scan_result_t** result_list, uint32_t* count )
{
    if ( smartbridge_bt_interface_is_scanning() == MXOS_TRUE )
    {
        bt_smartbridge_log("Can't Fetch Scan-Results [ Scan in-progress ? ]");
        return MXOS_BT_SCAN_IN_PROGRESS;
    }

    *result_list = scan_result_head;
    *count       = scan_result_count;
    return MXOS_BT_SUCCESS;
}

OSStatus smartbridge_helper_delete_scan_result_list( void )
{
    mxos_bt_smart_scan_result_t* curr;

    if ( scan_result_count == 0 )
    {
        return MXOS_BT_LIST_EMPTY;
    }

    curr = scan_result_head;

    /* Traverse through the list and delete all attributes */
    while ( curr != NULL )
    {
        /* Store pointer to next because curr is about to be deleted */
        mxos_bt_smart_scan_result_t* next = curr->next;

        /* Detach result from the list and free memory */
        curr->next = NULL;
        free( curr );

        /* Update curr */
        curr = next;
    }

    scan_result_count = 0;
    scan_result_head  = NULL;
    scan_result_tail  = NULL;
    return MXOS_BT_SUCCESS;
}

OSStatus smartbridge_helper_add_scan_result_to_list( mxos_bt_smart_scan_result_t* result )
{
    if ( scan_result_count == 0 )
    {
        scan_result_head = result;
        scan_result_tail = result;
    }
    else
    {
        scan_result_tail->next = result;
        scan_result_tail       = result;
    }

    scan_result_count++;
    //bt_smartbridge_log("New scan-result-count:%d", (int)scan_result_count);
    result->next = NULL;

    return MXOS_BT_SUCCESS;
}

OSStatus smartbridge_helper_find_device_in_scan_result_list( mxos_bt_device_address_t* address, mxos_bt_smart_address_type_t type,  mxos_bt_smart_scan_result_t** result )
{
    mxos_bt_smart_scan_result_t* iterator = scan_result_head;

    while( iterator != NULL )
    {
        if ( ( memcmp( &iterator->remote_device.address, address, sizeof( *address ) ) == 0 ) && ( iterator->remote_device.address_type == type ) )
        {
            *result = iterator;
            return MXOS_BT_SUCCESS;
        }

        iterator = iterator->next;
    }

    return MXOS_BT_ITEM_NOT_IN_LIST;
}

/******************************************************
 *            Socket Action Helper Functions
 ******************************************************/

mxos_bool_t smartbridge_helper_socket_check_actions_enabled( mxos_bt_smartbridge_socket_t* socket, uint8_t action_bits )
{
    return ( ( socket->actions & action_bits ) == action_bits ) ? MXOS_TRUE : MXOS_FALSE;
}

mxos_bool_t smartbridge_helper_socket_check_actions_disabled( mxos_bt_smartbridge_socket_t* socket, uint8_t action_bits )
{
    return ( ( socket->actions | ~action_bits ) == ~action_bits ) ? MXOS_TRUE : MXOS_FALSE;
}

void smartbridge_helper_socket_set_actions( mxos_bt_smartbridge_socket_t* socket, uint8_t action_bits )
{
    socket->actions |= action_bits;
}

void smartbridge_helper_socket_clear_actions( mxos_bt_smartbridge_socket_t* socket, uint8_t action_bits )
{
    socket->actions &= ~action_bits;
}

mxos_bool_t peripheral_helper_socket_check_actions_enabled( mxos_bt_peripheral_socket_t* socket, uint8_t action_bits )
{
    return ( ( socket->actions & action_bits ) == action_bits ) ? MXOS_TRUE : MXOS_FALSE;
}

mxos_bool_t peripheral_helper_socket_check_actions_disabled( mxos_bt_peripheral_socket_t* socket, uint8_t action_bits )
{
    return ( ( socket->actions | ~action_bits ) == ~action_bits ) ? MXOS_TRUE : MXOS_FALSE;
}

void peripheral_helper_socket_set_actions( mxos_bt_peripheral_socket_t* socket, uint8_t action_bits )
{
    socket->actions |= action_bits;
}

void peripheral_helper_socket_clear_actions( mxos_bt_peripheral_socket_t* socket, uint8_t action_bits )
{
    socket->actions &= ~action_bits;
}

/******************************************************
 *         GATT/GAP subprocedure Functions
 ******************************************************/

#define GATT_MAX_PROCEDURE_TIMEOUT (10000)

OSStatus subprocedure_lock( gatt_subprocedure_t* subprocedure )
{
    return mxos_rtos_lock_mutex( &subprocedure->mutex );
}

OSStatus subprocedure_unlock( gatt_subprocedure_t* subprocedure )
{
    return mxos_rtos_unlock_mutex( &subprocedure->mutex );
}

OSStatus subprocedure_reset( gatt_subprocedure_t* subprocedure )
{
    subprocedure->subprocedure      = GATT_SUBPROCEDURE_NONE;
    subprocedure->attr_head         = NULL;
    subprocedure->attr_tail         = NULL;
    subprocedure->attr_count        = 0;
    subprocedure->result            = MXOS_BT_SUCCESS;
    subprocedure->start_handle      = 0;
    subprocedure->end_handle        = 0;
    //subprocedure.pdu               = 0;
    subprocedure->length            = 0;
    subprocedure->offset            = 0;
    subprocedure->connection_handle = 0;
    memset( &subprocedure->uuid, 0, sizeof( subprocedure->uuid ) );
    subprocedure_wait_clear_semaphore( subprocedure );
    return MXOS_BT_SUCCESS;
}

OSStatus subprocedure_wait_for_completion( gatt_subprocedure_t* subprocedure )
{
    if( kNoErr != mxos_rtos_get_semaphore( &subprocedure->done_semaphore, GATT_MAX_PROCEDURE_TIMEOUT ) )
    {
        subprocedure->result = MXOS_BT_TIMEOUT;
    }
    return subprocedure->result;
}

OSStatus subprocedure_wait_clear_semaphore( gatt_subprocedure_t* subprocedure )
{
    while ( mxos_rtos_get_semaphore( &subprocedure->done_semaphore, MXOS_NO_WAIT ) == kNoErr )
    {
    }
    return MXOS_BT_SUCCESS;
}

OSStatus subprocedure_notify_complete( gatt_subprocedure_t* subprocedure )
{
    return mxos_rtos_set_semaphore( &subprocedure->done_semaphore );
}

/******************************************************
 *         Smartbridge timer Functions
 ******************************************************/

/* Smartbridge timer handler */
static void smartbridge_helper_timer_handler(void *arg)
{
    smartbridge_helper_timer_t *timer = (smartbridge_helper_timer_t *)arg;
    
    bt_smartbridge_log("smartbridge timer expired.");

    /* Call user timer handler */
    timer->handler(timer->context);

    /* Reload if not one-shot timer. */
    if (timer->is_started && !timer->one_shot) 
    {
        bt_smartbridge_log("smartbridge timer reload");
        mxos_rtos_reload_timer(&timer->timer);
    }
}

/* This function is used to stop a active timer.
 * return MXOS_TRUE if successful, otherwise, return MXOS_FALSE.
 */
mxos_bool_t smartbridge_helper_timer_stop(smartbridge_helper_timer_t *timer)
{    
    if (timer == (void *)0) 
        return MXOS_FALSE;

    if (!timer->is_started) 
        return MXOS_FALSE;

    if (mxos_rtos_is_timer_running(&timer->timer)) 
        mxos_rtos_stop_timer(&timer->timer);
    
    mxos_rtos_deinit_timer(&timer->timer);
    
    timer->is_started = MXOS_FALSE;
    timer->one_shot = MXOS_FALSE;

    return MXOS_TRUE;
}

/* This function is used to start or restart a timer.
 * The 'timer' will be restarted if it is active.
 */
mxos_bool_t smartbridge_helper_timer_start(smartbridge_helper_timer_t *timer, mxos_bool_t one_shot, uint32_t ms, timer_handler_t handler, void *arg)
{
    if (timer == (void *)0) 
        return MXOS_FALSE;

    if (timer->is_started) 
        smartbridge_helper_timer_stop(timer);

    if (mxos_rtos_init_timer(&timer->timer, ms, smartbridge_helper_timer_handler, (void *)timer) != kNoErr)
        return MXOS_FALSE;

    if (mxos_rtos_start_timer(&timer->timer) != kNoErr)
    {
        mxos_rtos_deinit_timer(&timer->timer);
        return MXOS_FALSE;
    }

    timer->is_started = MXOS_TRUE;
    timer->one_shot = one_shot;
    timer->handler = handler;
    timer->context = arg;
    
    return MXOS_TRUE;
}
