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

/*
EMW3165 on EMB-3165-A platform pin definitions ...
+-------------------------------------------------------------------------+
| Enum ID       |Pin | STM32| Peripheral  |    Board     |   Peripheral   |
|               | #  | Port | Available   |  Connection  |     Alias      |
|---------------+----+------+-------------+--------------+----------------|
|               | 1  | NC   |             |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MXOS_GPIO_2   | 2  | B  2 |   GPIO      |              |                |
|---------------+----+------+-------------+--------------+----------------|
|               | 3  |  NC  |             |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MXOS_GPIO_4   | 4  | A  7 | TIM1_CH1N   |              |                |
|               |    |      | TIM3_CH2    |              |                |
|               |    |      | SPI1_MOSI   |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MXOS_GPIO_5   | 5  | A  15| JTDI        |              |                |
|               |    |      | TIM2_CH1    |              |                |        
|               |    |      | TIM2_ETR    |              |                |
|               |    |      | SPI1_NSS    |              |                |
|               |    |      | SPI3_NSS    |              |                |
|               |    |      | USART1_TX   |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MXOS_GPIO_6   | 6  | B  3 | TIM2_CH2    |              |                |
|               |    |      | GPIO        |              |                |        
|               |    |      | SPI1_SCK    |              |                |
|               |    |      | USART1_RX   |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MXOS_GPIO_7   | 7  | B  4 | JTRST       |              |                |
|               |    |      | GPIO        |              |                |
|               |    |      | SDIO_D0     |              |                |
|               |    |      | TIM3_CH1    |              |                |
|               |    |      | SPI1_MISO   |              |                |
|               |    |      | SPI3_MISO   |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MXOS_GPIO_8   | 8  | A  2 | TIM2_CH3    |              | MXOS_UART_1_TX |
|               |    |      | TIM5_CH3    |              |                |
|               |    |      | TIM9_CH1    |              |                |
|               |    |      | USART2_TX   |              |                |
|               |    |      | GPIO        |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MXOS_GPIO_9   | 9  | A  1 | TIM2_CH2    |EasyLink_BUTTON|               |
|               |    |      | TIM5_CH2    |              |                |
|               |    |      | USART2_RTS  |              |                |
|               |    |      | GPIO        |              |                |
|---------------+----+------+-------------+--------------+----------------|
|               | 10 | VBAT |             |
|---------------+----+------+-------------+--------------+----------------|
|               | 11 | NC   |             |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MXOS_GPIO_12  | 12 | A  3 | TIM2_CH4    |              | MXOS_UART_1_RX |
|               |    |      | TIM5_CH4    |              |                |
|               |    |      | TIM9_CH2    |              |                |
|               |    |      | USART2_RX   |              |                |
|               |    |      | GPIO        |              |                |
|---------------+----+------+-------------+--------------+----------------|
|               | 13 | NRST |             |              |  MICRO_RST_N   |
|---------------+----+------+-------------+--------------+----------------|
| MXOS_GPIO_14  | 14 | A 0  | WAKE_UP     |              |                |
|---------------+----+------+-------------+--------------+----------------|
|               | 15 | NC   |             |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MXOS_GPIO_16  | 16 | C 13 |     -       |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MXOS_GPIO_17  | 17 | B 10 |  TIM2_CH3   | MXOS_SYS_LED |                |
|               |    |      |  I2C2_SCL   |              |                |
|               |    |      |  GPIO       |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MXOS_GPIO_18  | 18 | B  9 | TIM4_CH4    |              |                |
|               |    |      | TIM11_CH1   |              |                |
|               |    |      | I2C1_SDA    |              |                |
|               |    |      | GPIO        |              |                |
+---------------+----+--------------------+--------------+----------------+
| MXOS_GPIO_19  | 19 | B 12 | GPIO        |              |                |
+---------------+----+--------------------+--------------+----------------+
|               | 20 | GND  |             |              |                |
+---------------+----+--------------------+--------------+----------------+
|               | 21 | GND  |             |              |                |
+---------------+----+--------------------+--------------+----------------+
| MXOS_GPIO_22  | 22 | B  3 |             |              |                |
+---------------+----+--------------------+--------------+----------------+
| MXOS_GPIO_23  | 23 | A 15 | GPIO        |              |  JTAG_TDI      |
|               |    |      | USART1_TX   |STDIO_UART_RX |  SPI1_SSN      |
|               |    |      | TIM2_CH1    |              |                |
|               |    |      | TIM2_ETR    |              |                |
+---------------+----+--------------------+--------------+----------------+
| MXOS_GPIO_24  | 24 | B  4 |             |              |                |
+---------------+----+--------------------+--------------+----------------+
| MXOS_GPIO_25  | 25 | A 14 | JTCK-SWCLK  |  SWCLK       |                |
|               |    |      |  GPIO       |              |                |  
+---------------+----+--------------------+--------------+----------------+
|MXOS_GPIO_26   | 26 | A 13 | JTMS-SWDIO  |  SWDIO       |                |
|               |    |      |  GPIO       |              |                |    
+---------------+----+--------------------+--------------+----------------+
|MXOS_GPIO_27   | 27 | A 12 | TIM1_ETR    |              | USART1_RTS     |
|               |    |      | USART1_RTS  |              |                |    
|               |    |      | USART6_RX   |              |                |    
|               |    |      | USB_FS_DP   |              |                |    
|               |    |      | GPIO        |              |                |    
+---------------+----+--------------------+--------------+----------------+
|               | 28 | NC   |             |              |                |
+---------------+----+--------------------+--------------+----------------+
| MXOS_GPIO_29  | 29 | A 10 | GPIO        |              |                |
|               |    |      | TIM1_CH3    |              |                |
|               |    |      | USART1_RX   |              |                |
|               |    |      | USB_FS_ID   |              |                |
+---------------+----+--------------------+--------------+----------------+
| MXOS_GPIO_30  | 30 | B  6 | GPIO        |              |                |
|               |    |      | TIM4_CH1    |              |                |
|               |    |      | USART1_TX   |              |                |
|               |    |      | I2C1_SCL    |              |                |
+---------------+----+--------------------+--------------+----------------+
| MXOS_SYS_LED  | 31 | B  8 | GPIO        |              |                |
|               |    |      | TIM4_CH3    |              |                |
|               |    |      | TIM10_CH1   |              |                |  
+---------------+----+--------------------+--------------+----------------+
|               | 32 |  NC  |             |              |                | 
+---------------+----+--------------------+--------------+----------------+  
| MXOS_GPIO_33  | 33 | B 13 | TIM1_CH1N   |              |                |  
|               |    |      | SPI2_SCK    |              |                |  
|               |    |      | GPIO        |              |                | 
+---------------+----+--------------------+--------------+----------------+  
| MXOS_GPIO_34  | 34 | A  5 | TIM2_CH1    |              |                |  
|               |    |      | GPIO        |              |                | 
+---------------+----+--------------------+--------------+----------------+  
| MXOS_GPIO_35  | 35 | A  11| TIM1_CH4    |              |  USART1_CTS    |  
|               |    |      | SPI4_MISO   |              |                |  
|               |    |      | USART1_CTS  |              |                |  
|               |    |      | USART6_TX   |              |                |  
|               |    |      | USB_FS_DM   |              |                |  
|               |    |      | GPIO        |              |                |
+---------------+----+--------------------+--------------+----------------+  
| MXOS_GPIO_36  | 36 | B  1 | TIM1_CH3N   | BOOT_SEL     |                |  
|               |    |      | TIM3_CH4    |              |                |  
|               |    |      | GPIO        |              |                | 
+---------------+----+--------------------+--------------+----------------+  
| MXOS_GPIO_37  | 37 | B  0 | TIM1_CH2N   | MFG_SEL      |                |  
|               |    |      | TIM3_CH3    |              |                | 
|               |    |      | GPIO        |              |                | 
+---------------+----+--------------------+--------------+----------------+  
| MXOS_GPIO_38  | 38 | A  4 | USART2_CK   | MXOS_RF_LED |                | 
|               |    |      | GPIO        |              |                | 
+---------------+----+--------------------+--------------+----------------+  
|               | 39 | VDD  |             |              |                | 
+---------------+----+--------------------+--------------+----------------+  
|               | 40 | VDD  |             |              |                |
+---------------+----+--------------------+--------------+----------------+  
|               | 41 | ANT  |             |              |                |
+---------------+----+--------------------+--------------+----------------+ 
*/

enum
{
    MXOS_SYS_LED,
    MXOS_RF_LED,
    BOOT_SEL,
    MFG_SEL,
    EasyLink_BUTTON,
    STDIO_UART_RX,
    STDIO_UART_TX,
    FLASH_PIN_SPI_CS,
    FLASH_PIN_SPI_CLK,
    FLASH_PIN_SPI_MOSI,
    FLASH_PIN_SPI_MISO,

    MXOS_GPIO_2,
    MXOS_GPIO_8,
    MXOS_GPIO_9,
    MXOS_GPIO_12,
    MXOS_GPIO_14,
    MXOS_GPIO_16,
    MXOS_GPIO_17,
    MXOS_GPIO_18,
    MXOS_GPIO_19,
    MXOS_GPIO_27,  
    MXOS_GPIO_29,
    MXOS_GPIO_30,
    MXOS_GPIO_31,
    MXOS_GPIO_33,
    MXOS_GPIO_34,
    MXOS_GPIO_35,
    MXOS_GPIO_36,
    MXOS_GPIO_37,
    MXOS_GPIO_38,
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
    MXOS_I2C_MAX, /* Denotes the total number of I2C port aliases. Not a valid I2C alias */
    MXOS_I2C_NONE,
};

enum
{
    MXOS_PWM_1,
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
    MXOS_PARTITION_ATE,
    MXOS_PARTITION_OTA_TEMP,
    MXOS_PARTITION_RF_FIRMWARE,
    MXOS_PARTITION_PARAMETER_1,
    MXOS_PARTITION_PARAMETER_2,
    MXOS_PARTITION_KV,
    MXOS_PARTITION_MAX,
    MXOS_PARTITION_NONE,
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
