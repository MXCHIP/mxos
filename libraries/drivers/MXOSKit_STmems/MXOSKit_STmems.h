/**
 ******************************************************************************
 * @file    mxoskit_ext.h
 * @author  William Xu
 * @version V1.0.0
 * @date    8-May-2015
 * @brief   mxoskit st mems extension board peripherals operations..
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#ifndef __MXOSKIT_STMEMS_H_
#define __MXOSKIT_STMEMS_H_

#include "mxos_common.h"

//------------------------- MXOSKit STmems board modules drivers ------------------
#include "rgb_led/P9813/rgb_led.h"
#include "display/VGM128064/oled.h"
#include "motor/dc_motor/dc_motor.h"
#include "keypad/gpio_button/button.h"

#include "sensor/HTS221/hts221.h"
#include "sensor/LPS25HB/lps25hb.h"
#include "sensor/UVIS25/uvis25.h"
#include "sensor/LSM9DS1/lsm9ds1.h"
#include "sensor/light_adc/light_sensor.h"


/** @addtogroup MXOS_Drivers_interface
  * @{
  */


/** @defgroup MXOSKit_STmems_Driver MXOSKit STmems Driver 
  * @brief Provide device driver interface for MXOSKitSTmems board
  * @{
  */

/** @addtogroup MXOSKit_STmems_Driver
  * @{
  */

/** @defgroup MXOSKit_STmems_Init MXOSKit STmems Init 
  * @brief Provide init api for MXOSKit STmems board
  * @{
  */

//--------------------------- MXOSKit STmems board info ---------------------------
#define MXOSKIT_STMEMS_MANUFACTURER    "MXCHIP"
#define MXOSKIT_STMEMS_NAME            "MXOSKit-STmems"

#define MFG_TEST_MAX_MODULE_NUM      8



/**
 * @brief MXOSKit STmems board initialization
 *
 * @return   kNoErr        : on success.
 * @return   kGeneralErr   : if an error occurred
 */
mret_t mxoskit_STmems_init(void);   

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

#endif  // __MXOSKIT_STMEMS_H_
