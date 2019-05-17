/**
 ******************************************************************************
 * @file    button.h
 * @author  Eshen Wang
 * @version V1.0.0
 * @date    1-May-2015
 * @brief   user key operation.
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#ifndef __BUTTON_H_
#define __BUTTON_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mxos.h"


/** @addtogroup MXOS_Drivers_interface
  * @{
  */
    
/** @defgroup MXOS_keypad_Driver MXOS keypad Driver
  * @brief Provide driver interface for keypad devices
  * @{
  */


/** @addtogroup MXOS_keypad_Driver
  * @{
  */
/** @defgroup MXOS_Button_Driver MXOS Button Driver
  * @brief Provide driver interface for button
  * @{
  */


//--------------------------------  pin defines --------------------------------

enum _button_idle_state_e{
    IOBUTTON_IDLE_STATE_LOW = 0,
    IOBUTTON_IDLE_STATE_HIGH,
};

typedef uint8_t btn_idle_state;

typedef void (*button_pressed_cb)(void) ;
typedef void (*button_long_pressed_cb)(void) ;

typedef struct _button_context_t {
  mxos_gpio_t gpio;
  btn_idle_state idle;
  uint32_t long_pressed_timeout;
  button_pressed_cb pressed_func;
  button_long_pressed_cb long_pressed_func;
  /* Use by driver, do not initialze */
  mos_timer_id_t _user_button_timer;
  uint32_t start_time;
#ifdef CONFIG_MX108
  uint32_t ignore;
#endif
} button_context_t;

//------------------------------ user interfaces -------------------------------


/**
 * @brief Initialize button device.
 *
 * @param btn_context: button driver context data, should be persist at button's life time
 *
 * @return none
 */
void button_init( button_context_t *btn_context );

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  // __BUTTON_H_
