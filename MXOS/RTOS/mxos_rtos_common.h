/**
 ******************************************************************************
 * @file    mxos_rtos_common.h
 * @author  William Xu
 * @version V1.0.0
 * @date    22-Aug-2016
 * @brief   Definitions of the MXOS Common RTOS abstraction layer function headers
 ******************************************************************************
 *
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 *
 ******************************************************************************
 */


#pragma once

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define rtos_log(M, ...) custom_log("RTOS", M, ##__VA_ARGS__)
#define rtos_log_trace() custom_log_trace("RTOS")

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

/* MXOS <-> RTOS API */
extern OSStatus mxos_rtos_init  ( void );
extern OSStatus mxos_rtos_deinit( void );


#ifdef __cplusplus
} /* extern "C" */
#endif
