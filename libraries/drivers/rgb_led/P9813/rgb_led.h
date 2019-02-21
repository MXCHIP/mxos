/**
 ******************************************************************************
 * @file    rgb_led.h
 * @author  Eshen Wang
 * @version V1.0.0
 * @date    17-Mar-2015
 * @brief   rgb led controller.
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#ifndef __RGB_LED_H_
#define __RGB_LED_H_

#include "mxos.h"
#include "hsb2rgb_led.h"
#include "mxos_board.h"


/** @addtogroup MXOS_Drivers_interface
  * @{
  */

/** @addtogroup MXOS_LED_Driver
  * @{
  */

/** @defgroup MXOS_RGBLED_Driver MXOS RGBLED Driver
  * @brief Provide driver interface for RGBLED
  * @{
  */

#ifndef P9813_PIN_CIN
#define P9813_PIN_CIN       (MXOS_GPIO_NONE)
#endif

#ifndef P9813_PIN_DIN
#define P9813_PIN_DIN       (MXOS_GPIO_NONE)
#endif

#define P9813_PIN_CIN_Clr()        MxosGpioOutputLow(P9813_PIN_CIN)  
#define P9813_PIN_CIN_Set()        MxosGpioOutputHigh(P9813_PIN_CIN)

#define P9813_PIN_DIN_Clr()        MxosGpioOutputLow(P9813_PIN_DIN) 
#define P9813_PIN_DIN_Set()        MxosGpioOutputHigh(P9813_PIN_DIN)

//-------------------- user interfaces ---------------------------

/**
 * @brief Initialize RGB LED device.
 *
 * @return none
 */
void rgb_led_init(void);


/**
 * @brief Set light parameters for RGB LED
 *
 * @param red:    Red light parameter
 * @param green:  Green light parameter
 * @param blue:   Blue light parameter
 *
 * @return none
 */
void rgb_led_open(uint8_t red, uint8_t green, uint8_t blue);


/**
 * @brief Close RGB LED
 *
 * @return none
 */
void rgb_led_close(void);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */


#endif  // __RGB_LED_H_
