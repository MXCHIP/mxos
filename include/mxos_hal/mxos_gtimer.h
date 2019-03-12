/**
 ******************************************************************************
 * @file    MxosDriverTimer.h
 * @author  SWYANG
 * @version V1.0.0
 * @date    2017-07-25
 * @brief   This file provides all the headers of timer operation functions.
 ******************************************************************************
 *
 *  The MIT License
 *  Copyright (c) 2017 MXCHIP Inc.
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

#ifndef __MXOSDRIVERTIMER_H__
#define __MXOSDRIVERTIMER_H__

#pragma once
#include "mxos_common.h"
#include "platform_peripheral.h"

/* Legacy definitions */
#define MxosGtimerInitialize mxos_gtimer_init
#define MxosGtimerStart mxos_gtimer_start
#define MxosGtimerStop mxos_gtimer_stop

/** @addtogroup MXOS_PLATFORM
  * @{
  */

/** @defgroup MXOS_timer MXOS timer Driver
  * @brief  Pulse-Width Modulation (timer) Functions
  * @{
  */

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef int8_t mxos_gtimer_t;

 typedef platform_gtimer_mode_t                  mxos_gtimer_mode_t;

 typedef platform_gtimer_irq_callback_t          mxos_gtimer_irq_callback_t;

 /******************************************************
 *                 Function Declarations
 ******************************************************/

merr_t mxos_gtimer_init(mxos_gtimer_t timer);

merr_t mxos_gtimer_start(mxos_gtimer_t timer, mxos_gtimer_mode_t mode, uint32_t time, mxos_gtimer_irq_callback_t function, void *arg);

merr_t mxos_gtimer_stop(mxos_gtimer_t timer);

/** @} */
/** @} */

#endif
