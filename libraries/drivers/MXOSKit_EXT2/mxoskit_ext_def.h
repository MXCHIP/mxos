/**
 ******************************************************************************
 * @file    mxoskit_ext_def.h
 * @author  Eshen Wang
 * @version V1.0.0
 * @date    20-May-2015
 * @brief   mxoskit extension board peripherals pin defines.
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#ifndef __MXOSKIT_EXT_DEF_H_
#define __MXOSKIT_EXT_DEF_H_

//-------------------------- MxosKit-EXT board pin define ----------------------
#ifdef CONFIG_CPU_MX1290
#define SSD1106_USE_I2C
#define OLED_I2C_PORT       (Arduino_I2C)
#else
#define OLED_SPI_PORT       (Arduino_SPI)
#define OLED_SPI_SCK        (Arduino_SCK)
#define OLED_SPI_DIN        (Arduino_SI)
#define OLED_SPI_DC         (Arduino_SO)
#define OLED_SPI_CS         (Arduino_CS)
#endif

#define P9813_PIN_CIN       (Arduino_SCL)
#define P9813_PIN_DIN       (Arduino_SDA)

#define DC_MOTOR            (Arduino_D9)

#define MXOS_EXT_KEY1                (Arduino_D4)
#define MXOS_EXT_KEY2                (Arduino_D5)

#define BME280_I2C_DEVICE            (Arduino_I2C)
#define DHT11_DATA                   (Arduino_D8)

#define APDS9930_I2C_DEVICE          (Arduino_I2C)

#define LIGHT_SENSOR_ADC             (Arduino_A2)
#define INFARAED_REFLECTIVE_ADC      (Arduino_A3)


#endif  // __MXOSKIT_EXT_DEF_H_
