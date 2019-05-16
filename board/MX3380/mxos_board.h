/**
 ******************************************************************************
 * @file    platform.h
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This file provides all MXOS Peripherals defined for current platform.
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


#ifndef __MXOS_BOARD_H_
#define __MXOS_BOARD_H_

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
  

   
/******************************************************
 *                   Enumerations
 ******************************************************/

enum
{
    //MXOS_GPIO_1, // VDD
    //MXOS_GPIO_2, // GND
    //MXOS_GPIO_3, // VBATM
    MXOS_GPIO_4,
    MXOS_GPIO_5,
    MXOS_GPIO_6,
    MXOS_GPIO_7,
    MXOS_GPIO_8,
    MXOS_GPIO_9,
#if 0 // not used, for BR262
    MXOS_GPIO_10,
    MXOS_GPIO_11,
    MXOS_GPIO_12,
    MXOS_GPIO_13,
#endif    
    //MXOS_GPIO_14, // VREG
    MXOS_GPIO_15,
    MXOS_GPIO_16,
    //MXOS_GPIO_17, // GND
    //MXOS_GPIO_18, // RESET
    MXOS_GPIO_19,
    MXOS_GPIO_20,
    //MXOS_GPIO_21, // GND
    //MXOS_GPIO_22, // GND
    MXOS_GPIO_23,
    MXOS_GPIO_24,
    MXOS_GPIO_25,
    MXOS_GPIO_26,
    MXOS_GPIO_27,
    MXOS_GPIO_28,
    MXOS_GPIO_29,
    MXOS_GPIO_30,
    MXOS_GPIO_31,
    MXOS_GPIO_32,
    MXOS_GPIO_33,
    MXOS_GPIO_34,
    MXOS_GPIO_35,
    MXOS_GPIO_36,
    MXOS_GPIO_37,
    MXOS_GPIO_38,
    MXOS_GPIO_39,
    MXOS_GPIO_40,
    MXOS_GPIO_41,
    MXOS_GPIO_42,
    MXOS_GPIO_MAX, /* Denotes the total number of GPIO port aliases. Not a valid GPIO alias */
    MXOS_GPIO_NONE,
    
    BOOT_SEL = MXOS_GPIO_NONE, 
    MFG_SEL = MXOS_GPIO_NONE, 
    EasyLink_BUTTON = MXOS_GPIO_NONE,
    MXOS_SYS_LED = MXOS_GPIO_NONE,
};


enum
{
  MXOS_SPI_1,
  MXOS_SPI_MAX, /* Denotes the total number of SPI port aliases. Not a valid SPI alias */
  MXOS_SPI_NONE,
};

enum
{
    MXOS_I2C_1,
    MXOS_I2C_MAX, /* Denotes the total number of I2C port aliases. Not a valid I2C alias */
    MXOS_I2C_NONE,
};

enum
{
    MXOS_PWM_1,
    MXOS_PWM_2,
    MXOS_PWM_3,
    MXOS_PWM_4,
    MXOS_PWM_5,
    MXOS_PWM_6,
    MXOS_PWM_7,
    MXOS_PWM_8,
    MXOS_PWM_9,
    MXOS_PWM_10,
    MXOS_PWM_11,
    MXOS_PWM_MAX, /* Denotes the total number of PWM port aliases. Not a valid PWM alias */
    MXOS_PWM_NONE,
};

enum
{
    MXOS_ADC_1,
    MXOS_ADC_2,
    MXOS_ADC_MAX, /* Denotes the total number of ADC port aliases. Not a valid ADC alias */
    MXOS_ADC_NONE,
};

enum
{
    MXOS_UART_1,
    MXOS_UART_2,
    MXOS_UART_3,
    MXOS_UART_MAX, /* Denotes the total number of UART port aliases. Not a valid UART alias */
    MXOS_UART_NONE,
};

enum
{
  MXOS_FLASH_EMBEDDED,
  MXOS_FLASH_SPI,
  MXOS_FLASH_MAX,
  MXOS_FLASH_NONE,
};

enum
{
  MXOS_PARTITION_FILESYS,
  MXOS_PARTITION_USER_MAX
};

enum
{
    MXOS_PARTITION_ERROR = -1,
    MXOS_PARTITION_BOOTLOADER = MXOS_PARTITION_USER_MAX,
    MXOS_PARTITION_APPLICATION,
    MXOS_PARTITION_OTA_TEMP,
    MXOS_PARTITION_KV,
    MXOS_PARTITION_USER,
    MXOS_PARTITION_PARAMETER_1,
    MXOS_PARTITION_PARAMETER_2,
    MXOS_PARTITION_MAX,
    MXOS_PARTITION_NONE,
};
enum
{
    PHY_PARTITION_BOOTLOADER = 0, // first 24K
    PHY_PARTITION_APPLICATION1,
    PHY_PARTITION_APPLICATION2,
    PHY_PARTITION_KV,
    PHY_PARTITION_USER,
    PHY_PARTITION_MAX,
    PHY_PARTITION_NONE,
};

#ifdef BOOTLOADER
#define MXOS_STDIO_UART             (MXOS_UART_1)
#define MXOS_STDIO_UART_BAUDRATE    (115200)
#else
#define MXOS_STDIO_UART             (MXOS_UART_1)
#define MXOS_STDIO_UART_BAUDRATE    (115200)
#endif

#define MXOS_UART_FOR_APP     (MXOS_UART_2)
#define MXOS_MFG_TEST         (MXOS_UART_2)
#define MXOS_CLI_UART         (MXOS_UART_1)

/* Components connected to external I/Os*/
#define USE_SPI_FLASH
#define SFLASH_SUPPORT_MACRONIX_PARTS 
//#define SFLASH_SUPPORT_SST_PARTS
//#define SFLASH_SUPPORT_WINBOND_PARTS

/* Arduino extention connector */
#define Arduino_RXD         (MXOS_GPIO_29)
#define Arduino_TXD         (MXOS_GPIO_30)
#define Arduino_D2          (MXOS_GPIO_NONE)
#define Arduino_D3          (MXOS_GPIO_NONE)
#define Arduino_D4          (MXOS_GPIO_19) 
#define Arduino_D5          (MXOS_GPIO_16)  
#define Arduino_D6          (MXOS_GPIO_14) 
#define Arduino_D7          (MXOS_GPIO_NONE)

#define Arduino_D8          (MXOS_GPIO_35)
#define Arduino_D9          (MXOS_GPIO_27)
#define Arduino_CS          (MXOS_GPIO_2)
#define Arduino_SI          (FLASH_PIN_SPI_MOSI)
#define Arduino_SO          (FLASH_PIN_SPI_MISO)
#define Arduino_SCK         (FLASH_PIN_SPI_CLK)
#define Arduino_SDA         (MXOS_GPIO_18)
#define Arduino_SCL         (MXOS_GPIO_17)

#define Arduino_A0          (MXOS_ADC_NONE)
#define Arduino_A1          (MXOS_ADC_NONE)
#define Arduino_A2          (MXOS_ADC_1)
#define Arduino_A3          (MXOS_ADC_2)
#define Arduino_A4          (MXOS_ADC_NONE)
#define Arduino_A5          (MXOS_ADC_NONE)

#define Arduino_I2C         (MXOS_I2C_1)
#define Arduino_SPI         (MXOS_SPI_1)
#define Arduino_UART        (MXOS_UART_2)

#ifdef USE_MXOSKit_EXT
#define MXOS_I2C_CP         (Arduino_I2C)
#include "mxoskit_ext_def.h"
#else
#define MXOS_I2C_CP         (MXOS_I2C_NONE)
#endif //USE_MXOSKit_EXT

void mxos_board_init( void );

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif
