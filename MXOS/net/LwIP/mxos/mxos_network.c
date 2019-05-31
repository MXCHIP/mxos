/**
 ******************************************************************************
 * @file    mxos_socket.c
 * @author  William Xu
 * @version V1.0.0
 * @date    05-Aug-2018
 * @brief   This file provide the MXOS Socket abstract layer convert functions.
 ******************************************************************************
 *
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */


#include <string.h>
#include <stdlib.h>

#include "mxos_common.h"
#include "mdebug.h"

#include "mos.h"
#include "mos_worker.h"
#include "mxos_eth.h"
#include "mwifi.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define net_log(M, ...) MXOS_LOG(CONFIG_NET_DEBUG, "NET", M, ##__VA_ARGS__)

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
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

netif_t netif_prioritys[INTERFACE_MAX] = { INTERFACE_ETH, INTERFACE_STA, INTERFACE_UAP };

/******************************************************
 *               Function Definitions
 ******************************************************/

static netif_t default_if = INTERFACE_NONE;
static mxos_bool_t auto_switch = MXOS_TRUE;

MXOS_WEAK void mxos_eth_set_default_interface(void)
{

}

MXOS_WEAK void mxos_wlan_set_default_interface(uint8_t interface)
{

}

static merr_t set_default_interface( void* arg)
{
    if ( default_if == INTERFACE_ETH ) {
        net_log(" INTERFACE_ETH SET DEFAULT.........");
        mxos_eth_set_default_interface();
    }
    else if( default_if == INTERFACE_STA ) {
        net_log(" INTERFACE_STA SET DEFAULT.........");
        mxos_wlan_set_default_interface( (netif_t)default_if );
    }
    return kNoErr;
}


void mxos_network_set_interface_priority( netif_t prioritys[INTERFACE_MAX] )
{
    auto_switch = MXOS_TRUE;

    memcpy( netif_prioritys, prioritys, INTERFACE_MAX );
}


merr_t mxos_network_switch_interface_auto( void )
{
    merr_t err = kNoErr;

    require_action_quiet( auto_switch == MXOS_TRUE, exit, err = kUnsupportedErr );

    for ( uint8_t i = 0; i < INTERFACE_MAX; i++ ) {
        if ( IS_INTERFACE_UP( netif_prioritys[i] ) ) {
            if( netif_prioritys[i] != default_if ) {
                default_if = netif_prioritys[i];
            }
            break;
        }
    }

exit:
    err = mos_worker_send_async_event( MOS_NETWORKING_WORKER_THREAD, set_default_interface, NULL );
    return err;
}


merr_t mxos_network_switch_interface_manual( netif_t interface )
{
    merr_t err = kNoErr;

    auto_switch = MXOS_FALSE;

    default_if = interface;
    err = mos_worker_send_async_event( MOS_NETWORKING_WORKER_THREAD, set_default_interface, NULL );

    return err;


}

void mxos_network_set_default_interface( netif_t prioritys[] )
{
    memcpy( netif_prioritys, prioritys, INTERFACE_MAX );
}

