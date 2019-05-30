/**
 ******************************************************************************
 * @file    platform_init.c
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This file provide functions called by MICO to drive stm32f2xx
 *          platform: - e.g. power save, reboot, platform initialize
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#include "mos.h"
#include "mos_worker.h"

#include "mxos_board.h"
#include "mxos_board_conf.h"

#include "platform_peripheral.h"
#include "platform_logging.h"


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
*               Function Declarations
******************************************************/
extern merr_t host_platform_init( void );

/******************************************************
*               Variables Definitions
******************************************************/

/******************************************************
*               Function Definitions
******************************************************/

/******************************************************
*            NO-OS Functions
******************************************************/


