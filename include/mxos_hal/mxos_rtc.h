/**
 ******************************************************************************
 * @file    MxosDriverRtc.h
 * @author  William Xu
 * @version V1.0.0
 * @date    16-Sep-2014
 * @brief   This file provides all the headers of RTC operation functions.
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

#ifndef __MXOSDRIVERRTC_H__
#define __MXOSDRIVERRTC_H__

#pragma once
#include "mxos_common.h"
#ifdef ALIOS_SUPPORT
#include "aos_peripheral.h"
#else
#include "platform_peripheral.h"
#endif

/* Legacy definitions */
#define MxosRtcInitialize mxos_rtc_init
#define MxosRtcGetTime mxos_rtc_get_time
#define MxosRtcSetTime mxos_rtc_set_time

/** @addtogroup MXOS_PLATFORM
* @{
*/

/** @defgroup MXOS_RTC MXOS RTC Driver
* @brief  Real-time clock (RTC) Functions
* @{
*/

/******************************************************
 *                   Macros
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
 *                     Variables
 ******************************************************/

/******************************************************
                Function Declarations
 ******************************************************/

/** Initialize the RTC peripheral
 *
 */
OSStatus mxos_rtc_init(void);

/** Get the current time from the RTC peripheral
 *
 * @return The current time
 */
OSStatus mxos_rtc_get_time(time_t *t);

/** Set the current time to the RTC peripheral
 *
 * @param t The current time to be set
 */
OSStatus mxos_rtc_set_time(time_t t);

/** @} */
/** @} */

#endif


