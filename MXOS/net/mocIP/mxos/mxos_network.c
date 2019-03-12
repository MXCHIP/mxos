/**
 ******************************************************************************
 * @file    mxos_network.c
 * @author  William Xu
 * @version V1.0.0
 * @date    06-Nov-2017
 * @brief   This file provide the MXOS network management abstract layer functions.
 ******************************************************************************
 *
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2017 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */


#include <string.h>
#include <stdlib.h>

#include "mxos_common.h"
#include "mxos_debug.h"
#include "mxos_network.h"

#include "mxos_rtos.h"
#include "mxos_eth.h"
#include "mxos_wlan.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define net_log(M, ...) MXOS_LOG(MXOS_DEBUG_ON, "ETH", M, ##__VA_ARGS__)

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


void mxos_network_set_interface_priority( netif_t prioritys[INTERFACE_MAX] )
{
    return;
}


mret_t mxos_network_switch_interface_auto( void )
{
    return kUnsupportedErr;
}


mret_t mxos_network_switch_interface_manual( netif_t interface )
{
    return kUnsupportedErr;
}

void mxos_network_set_default_interface( netif_t prioritys[] )
{
    return;
}

