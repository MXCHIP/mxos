/**
 ******************************************************************************
 * @file    mxoskit_stmems.h
 * @author  Willian Xu
 * @version V1.0.0
 * @date    20-May-2015
 * @brief   mxoskit st mems extension board peripherals pin defines.
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#pragma once

#ifndef __MXOSKIT_STMEMS_DEF_H_
#define __MXOSKIT_STMEMS_DEF_H_

//-------------------------- MxosKit-EXT board pin define ----------------------

#define SSD1106_USE_I2C
#define OLED_I2C_PORT       (Arduino_I2C)


#define P9813_PIN_CIN       (Arduino_D9)
#define P9813_PIN_DIN       (Arduino_D8)

#define DC_MOTOR            (Arduino_A3)

#define MXOSKIT_STMEMS_KEY1                (Arduino_D5)
#define MXOSKIT_STMEMS_KEY2                (Arduino_D6)

#define HTS221_I2C_PORT              (Arduino_I2C)
#define LPS25HB_I2C_PORT             (Arduino_I2C)
#define UVIS25_I2C_PORT              (Arduino_I2C)
#define LSM9DS1_I2C_PORT             (Arduino_I2C)

#define LIGHT_SENSOR_ADC             (Arduino_A2)

#endif  // __MXOSKIT_EXT_DEF_H_
