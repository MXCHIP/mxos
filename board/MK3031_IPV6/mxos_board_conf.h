/**
******************************************************************************
* @file    platform_config.h
* @author  William Xu
* @version V1.0.0
* @date    05-Oct-2016
* @brief   This file provides common configuration for current platform.
******************************************************************************
*
*  The MIT License
*  Copyright (c) 2014 MXCHIP Inc.
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy 
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights 
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is furnished
*  to do so, subject to the following conditions:
*
*  The above copyright notice and this permission notice shall be included in
*  all copies or substantial portions of the Software.
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
*  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR 
*  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
******************************************************************************
*/ 

#pragma once

#ifndef __PLATFORM_COMMON_CONFIG_H__
#define __PLATFORM_COMMON_CONFIG_H__


/******************************************************
*                      Macros
******************************************************/

/******************************************************
*                    Constants
******************************************************/

#define HARDWARE_REVISION   "3031"
#define DEFAULT_NAME        "EMW3031 Module"
#define MODEL               "EMW3031"

/* MXOS RTOS tick rate in Hz */
#define MXOS_DEFAULT_TICK_RATE_HZ                   (1000) 
/************************************************************************
 * Uncomment to disable watchdog. For debugging only */
#define MXOS_DISABLE_WATCHDOG

/************************************************************************
 * Uncomment to disable standard IO, i.e. printf(), etc. */
//#define MXOS_DISABLE_STDIO

/************************************************************************
 * Uncomment to disable MCU powersave API functions */
//#define MXOS_DISABLE_MCU_POWERSAVE

/************************************************************************
 * Uncomment to enable MCU real time clock */
#define MXOS_ENABLE_MCU_RTC

/************************************************************************
 * Restore default and start easylink after press down EasyLink button for 3 seconds. */
#define RestoreDefault_TimeOut                      (3000)

/************************************************************************
 * CPU clock */
#define MCU_CLOCK_HZ            (100000000)

/************************************************************************
 * How many bits are used in NVIC priority configuration */
#define CORTEX_NVIC_PRIO_BITS   (4)

/************************************************************************
 * Enable write protection to write-disabled embedded flash sectors */
//#define MCU_EBANLE_FLASH_PROTECT 

/************************************************************************
 * Platform provide OTA temporary partition as secondary application partition */
#define MXOS_ENABLE_SECONDARY_APPLICATION

/************************************************************************
 * TCPIP stack in MOC kernel support IPV6 functions */
#define MOCIP_CONFIG_IPV6

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
#endif

