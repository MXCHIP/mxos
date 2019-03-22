/**
******************************************************************************
* @file    platform.h
* @author  William Xu
* @version V1.0.0
* @date    05-Oct-2016
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



#ifndef __MXOS_BOARD_H__
#define __MXOS_BOARD_H__

#include <stdint.h>

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
    MXOS_GPIO_1,
    MXOS_GPIO_2,
    MXOS_GPIO_7,
    MXOS_GPIO_8,
    MXOS_GPIO_9,
    MXOS_GPIO_10,
    MXOS_GPIO_12,
    MXOS_GPIO_13,
    MXOS_GPIO_14,
    MXOS_GPIO_19,
    MXOS_GPIO_21,
    MXOS_GPIO_22,
    MXOS_GPIO_23,
    MXOS_GPIO_MAX, /* Denotes the total number of GPIO port aliases. Not a valid GPIO alias */
    MXOS_GPIO_NONE,
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
    MXOS_I2C_2,
    MXOS_I2C_MAX, /* Denotes the total number of I2C port aliases. Not a valid I2C alias */
    MXOS_I2C_NONE,
};

enum
{
    MXOS_IIS_MAX, /* Denotes the total number of IIS port aliases. Not a valid IIS alias */
    MXOS_IIS_NONE,
};

enum
{
    MXOS_PWM_1,
    MXOS_PWM_2,
    MXOS_PWM_3,
    MXOS_PWM_4,
    MXOS_PWM_5,
    MXOS_PWM_6,
    MXOS_PWM_MAX, /* Denotes the total number of PWM port aliases. Not a valid PWM alias */
    MXOS_PWM_NONE,
};

enum
{
    MXOS_GTIMER_1,
    MXOS_GTIMER_2,
    MXOS_GTIMER_3,
    MXOS_GTIMER_4,
    MXOS_GTIMER_5,
    MXOS_GTIMER_MAX, /* Denotes the total number of GTIMER port aliases. Not a valid GTIMER alias */
    MXOS_GTIMER_NONE,
};

enum
{
    MXOS_ADC_1,
    MXOS_ADC_MAX, /* Denotes the total number of ADC port aliases. Not a valid ADC alias */
    MXOS_ADC_NONE,
};

enum
{
    MXOS_UART_1,
    MXOS_UART_2,
    MXOS_UART_MAX, /* Denotes the total number of UART port aliases. Not a valid UART alias */
    MXOS_UART_NONE,
};

enum
{
  MXOS_FLASH_SPI,
  MXOS_FLASH_MAX,
  MXOS_FLASH_NONE,
};

/* Donot change MXOS_PARTITION_USER_MAX!! */
enum
{
    MXOS_PARTITION_ERROR = -1,
    MXOS_PARTITION_USER_MAX   = 0,
    MXOS_PARTITION_BOOTLOADER = MXOS_PARTITION_USER_MAX,
    MXOS_PARTITION_APPLICATION,
    MXOS_PARTITION_ATE,
    MXOS_PARTITION_OTA_TEMP,
    MXOS_PARTITION_RF_FIRMWARE,
    MXOS_PARTITION_PARAMETER_1,
    MXOS_PARTITION_PARAMETER_2,
    MXOS_PARTITION_USER = 7,
    MXOS_PARTITION_KV = MXOS_PARTITION_USER,
    MXOS_PARTITION_SDS,
    MXOS_PARTITION_MAX,
    MXOS_PARTITION_NONE,
};


typedef struct
{
    int8_t pin;
    unsigned char config; /* @ref mxos_gpio_config_t */
    unsigned char out; /* 0: low, 1: high */
}mhal_gpio_open_t;

typedef struct
{
    int8_t pin;
} mxos_pwm_pinmap_t;

typedef struct
{
    int8_t mosi;
    int8_t miso;
    int8_t sclk;
    int8_t ssel;
} mxos_spi_pinmap_t;

typedef struct
{
    int8_t tx;
    int8_t rx;
    int8_t rts;
    int8_t cts;
} mxos_uart_pinmap_t;

typedef struct
{
    int8_t sda;
    int8_t scl;
} mxos_i2c_pinmap_t;
    
typedef struct
{
  const mxos_pwm_pinmap_t *pwm_pinmap;
  const mxos_spi_pinmap_t *spi_pinmap;
  const mxos_uart_pinmap_t *uart_pinmap;
  const mxos_i2c_pinmap_t *i2c_pinmap;
} platform_peripherals_pinmap_t;

#define MXOS_STDIO_UART      MXOS_UART_2
#define MXOS_STDIO_UART_BAUDRATE (115200)

#define MXOS_UART_FOR_APP    MXOS_UART_1
#define MXOS_MFG_TEST        MXOS_UART_1
#define MXOS_CLI_UART        MXOS_UART_2

/* Components connected to external I/Os*/
#define Standby_SEL      (MXOS_GPIO_29)

/* I/O connection <-> Peripheral Connections */
#define BOOT_SEL        MXOS_GPIO_19
#define MFG_SEL         MXOS_GPIO_NONE
#define MXOS_RF_LED     MXOS_GPIO_NONE
#define MXOS_SYS_LED    MXOS_GPIO_19
#define EasyLink_BUTTON MXOS_GPIO_23

#define MXOS_GPIO_NC    0xFF

typedef struct {
	int country_code;
	int enable_healthmon;
	int dhcp_arp_check;
} mxos_system_config_t;

/* Arduino extention connector */
#define Arduino_RXD         (MXOS_GPIO_23)
#define Arduino_TXD         (MXOS_GPIO_18)
#define Arduino_D2          (MXOS_GPIO_NONE)
#define Arduino_D3          (MXOS_GPIO_NONE)
#define Arduino_D4          (MXOS_GPIO_NONE) 
#define Arduino_D5          (MXOS_GPIO_14)
#define Arduino_D6          (MXOS_GPIO_NONE) 
#define Arduino_D7          (MXOS_GPIO_NONE)

#define Arduino_D8          (MXOS_GPIO_13)
#define Arduino_D9          (MXOS_GPIO_12)
#define Arduino_CS          (MXOS_GPIO_8)
#define Arduino_SI          (MXOS_GPIO_9)
#define Arduino_SO          (MXOS_GPIO_7)
#define Arduino_SCK         (MXOS_GPIO_10)
#define Arduino_SDA         (MXOS_GPIO_8)
#define Arduino_SCL         (MXOS_GPIO_7)

#define Arduino_A0          (MXOS_ADC_NONE)
#define Arduino_A1          (MXOS_ADC_NONE)
#define Arduino_A2          (MXOS_ADC_NONE)
#define Arduino_A3          (MXOS_ADC_NONE)
#define Arduino_A4          (MXOS_ADC_NONE)
#define Arduino_A5          (MXOS_ADC_NONE)

#define Arduino_I2C         (MXOS_I2C_1)
#define Arduino_SPI         (MXOS_SPI_1)
#define Arduino_UART        (MXOS_UART_1)

#ifdef USE_MXOSKit_EXT
#define MXOS_I2C_CP         (Arduino_I2C)
#include "mxoskit_ext_def.h"
#else
#define MXOS_I2C_CP         (MXOS_I2C_NONE)
#endif //USE_MXOSKit_EXT

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif
