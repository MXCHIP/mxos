/**
 ******************************************************************************
 * @file    MxosDriverPwm.h
 * @author  William Xu
 * @version V1.0.0
 * @date    16-Sep-2014
 * @brief   This file provides all the headers of PWM operation functions.
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

#ifndef __MXOSDRIVERPWM_H__
#define __MXOSDRIVERPWM_H__

#pragma once
#include "mxos_common.h"
#ifdef ALIOS_SUPPORT
#include "aos_peripheral.h"
#else
#include "platform_peripheral.h"
#endif

/** @addtogroup MXOS_PLATFORM
  * @{
  */

/** @defgroup MXOS_PWM MXOS PWM Driver
  * @brief  Pulse-Width Modulation (PWM) Functions
  * @{
  */

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef int8_t mxos_pwm_t;   /**< MXOS PWM peripheral handle, MXOS_PWM_XX define by board/<board_name>/mxos_board.h. */

 /******************************************************
 *                 Function Declarations
 ******************************************************/



/**@brief Initialises a PWM pin
 *
 * @note  Prepares a Pulse-Width Modulation pin for use.
 * Does not start the PWM output (use @ref MxosPwmStart).
 *
 * @param pwm        : the PWM interface which should be initialised
 * @param frequency  : Output signal frequency in Hertz
 * @param duty_cycle : Duty cycle of signal as a floating-point percentage (0.0 to 100.0)
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus MxosPwmInitialize(mxos_pwm_t pwm, uint32_t frequency, float duty_cycle);


/**@brief Starts PWM output on a PWM interface
 *
 * @note  Starts Pulse-Width Modulation signal output on a PWM pin
 *
 * @param pwm        : the PWM interface which should be started
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus MxosPwmStart(mxos_pwm_t pwm);


/**@brief Stops output on a PWM pin
 *
 * @note  Stops Pulse-Width Modulation signal output on a PWM pin
 *
 * @param pwm        : the PWM interface which should be stopped
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
OSStatus MxosPwmStop(mxos_pwm_t pwm);

/** @} */
/** @} */

#endif
