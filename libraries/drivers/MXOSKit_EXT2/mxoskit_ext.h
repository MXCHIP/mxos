/**
 ******************************************************************************
 * @file    mxoskit_ext.h
 * @author  Eshen Wang
 * @version V1.0.0
 * @date    8-May-2015
 * @brief   mxoskit extension board peripherals operations..
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#ifndef __MXOSKIT_EXT_H_
#define __MXOSKIT_EXT_H_

#include "mxos_common.h"

//------------------------- MxosKit-EXT board modules drivers ------------------
#include "rgb_led/P9813/rgb_led.h"
#include "display/VGM128064/oled.h"
#include "motor/dc_motor/dc_motor.h"
#include "keypad/gpio_button/button.h"

#include "sensor/light_adc/light_sensor.h"
#include "sensor/APDS9930/APDS9930.h"
#include "sensor/infrared_adc/infrared_reflective.h"


#include "motion_sensor.h"
#include "temp_hum_sensor.h"


/** @addtogroup MXOS_Drivers_interface
  * @{
  */

/** @defgroup MXOSKIT_EXT_Driver MXOSKit Ext Driver
  * @brief Provide driver interface for MXOSKit Ext devices
  * @{
  */
   
/** @addtogroup MXOSKIT_EXT_Driver
  * @{
  */

/** @defgroup MXOSKIT_EXT_Driver MXOSKit Ext Driver
  * @brief Provide device init driver interface for MXOSKit Ext or modules
  * @{
  */

//--------------------------- MxosKit-EXT board info ---------------------------
#define DEV_KIT_MANUFACTURER    "MXCHIP"
#define DEV_KIT_NAME            "MXOSKit3288"

#define MFG_TEST_MAX_MODULE_NUM      3


/**
 * @brief    MxosKit-EXT board init
 *
 * @return   kNoErr        : on success.
 * @return   kGeneralErr   : if an error occurred
 */
mret_t mxoskit_ext_init(void);    


/**
 * @brief    Init modules on MxosKit-EXT board.
 *
 * @return   kNoErr        : on success.
 * @return   kGeneralErr   : if an error occurred
 */
mret_t user_modules_init(void);   
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
#endif  // __MXOSKIT_EXT_H_
