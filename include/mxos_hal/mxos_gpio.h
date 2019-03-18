/**
 ******************************************************************************
 * @file    MxosDriverGpio.h
 * @author  William Xu
 * @version V1.0.0
 * @date    16-Sep-2014
 * @brief   This file provides all the headers of GPIO operation functions.
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

#ifndef __MXOSDRIVERGPIO_H__
#define __MXOSDRIVERGPIO_H__

#pragma once
#include "mxos_common.h"
#include "platform_peripheral.h"

/* Legacy definitions */
#define MxosGpioInitialize      mhal_gpio_open
#define MxosGpioFinalize        mhal_gpio_close
#define MxosGpioOutputHigh      mhal_gpio_high
#define MxosGpioOutputLow       mhal_gpio_low
#define MxosGpioOutputTrigger   mhal_gpio_toggle
#define MxosGpioInputGet        mhal_gpio_value
#define MxosGpioEnableIRQ       mhal_gpio_int_on
#define MxosGpioDisableIRQ      mhal_gpio_int_off

/** @addtogroup MXOS_PLATFORM
* @{
*/


/** @defgroup MXOS_GPIO MXOS GPIO Driver
* @brief  General Purpose Input/Output pin (GPIO) Functions
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

typedef int8_t mxos_gpio_t;  /**< MXOS GPIO peripheral handle, MXOS_GPIO_XX define by board/<board_name>/mxos_board.h. */

typedef platform_pin_config_t                   mxos_gpio_config_t;
typedef platform_gpio_irq_trigger_t             mxos_gpio_irq_trigger_t;
typedef platform_gpio_irq_callback_t            mxos_gpio_irq_handler_t;


 /******************************************************
 *                 Function Declarations
 ******************************************************/


/**@brief Initialises a GPIO pin
 *
 * @note  Prepares a GPIO pin for use.
 *
 * @param gpio          : the gpio pin which should be initialised
 * @param configuration : A structure containing the required
 *                        gpio configuration
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
merr_t mhal_gpio_open( mxos_gpio_t gpio, mxos_gpio_config_t configuration );


/**@brief DeInitialises a GPIO pin
 *
 * @note  Set a GPIO pin in default state.
 *
 * @param  gpio          : the gpio pin which should be deinitialised
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
merr_t mhal_gpio_close( mxos_gpio_t gpio );


/**@brief Sets an output GPIO pin high
 *
 * @note  Using this function on a gpio pin which is set to input mode is undefined.
 *
 * @param gpio          : the gpio pin which should be set high
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
merr_t mhal_gpio_high( mxos_gpio_t gpio );


/**@brief Sets an output GPIO pin low
 *
 * @note  Using this function on a gpio pin which is set to input mode is undefined.
 *
 * @param gpio          : the gpio pin which should be set low
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
merr_t mhal_gpio_low( mxos_gpio_t gpio );

/** Trigger an output GPIO pin 
 *
 * Trigger an output GPIO pin's output. Using this function on a
 * gpio pin which is set to input mode is undefined.
 *
 * @param gpio          : the gpio pin which should be set low
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
merr_t mhal_gpio_toggle( mxos_gpio_t gpio );



/**@brief Get the state of an input GPIO pin
 *
 * @note Get the state of an input GPIO pin. Using this function on a
 * gpio pin which is set to output mode will return an undefined value.
 *
 * @param gpio          : the gpio pin which should be read
 *
 * @return    true  : if high
 * @return    fasle : if low
 */
bool mhal_gpio_value( mxos_gpio_t gpio );


/**@brief Enables an interrupt trigger for an input GPIO pin
 *
 * @note Enables an interrupt trigger for an input GPIO pin.
 * Using this function on a gpio pin which is set to
 * output mode is undefined.
 *
 * @param gpio    : the gpio pin which will provide the interrupt trigger
 * @param trigger : the type of trigger (rising/falling edge)
 * @param handler : a function pointer to the interrupt handler
 * @param arg     : an argument that will be passed to the
 *                  interrupt handler
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
merr_t mhal_gpio_int_on( mxos_gpio_t gpio, mxos_gpio_irq_trigger_t trigger, mxos_gpio_irq_handler_t handler, void* arg );


/**@brief Disables an interrupt trigger for an input GPIO pin
 *
 * @note Disables an interrupt trigger for an input GPIO pin.
 * Using this function on a gpio pin which has not been set up
 * using @ref mxos_gpio_input_irq_enable is undefined.
 *
 * @param gpio    : the gpio pin which provided the interrupt trigger
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
merr_t mhal_gpio_int_off( mxos_gpio_t gpio );

/** @} */
/** @} */

#endif


