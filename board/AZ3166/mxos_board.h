/* MiCO Team
 * Copyright (c) 2017 MXCHIP Information Tech. Co.,Ltd
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __MXOS_BOARD_H__
#define __MXOS_BOARD_H__

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
EMW3239 platform pin definitions ...
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
| MXOS_GPIO_4   | 4  | B 15 | TIM1_CH3N   |              |                |
|               |    |      | TIM8_CH3N   |              |                |
|               |    |      | SPI2_MOSI   |              |                |
|               |    |      | SDIO_CK     |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MXOS_GPIO_5   | 5  | B 12 | SPI2_NSS    |              |                |
|               |    |      | SPI4_NSS    |              |                |        
|---------------+----+------+-------------+--------------+----------------|
| MXOS_GPIO_6   | 6  | B 13 | TIM1_CH1N   |              |                |
|               |    |      | GPIO        |              |                |        
|               |    |      | SPI2_SCK    |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MXOS_GPIO_7   | 7  | B 14 | GPIO        |              |                |
|               |    |      | SDIO_D6     |              |                |
|               |    |      | TIM1_CH2N   |              |                |
|               |    |      | SPI2_MISO   |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MXOS_GPIO_8   | 8  | C  6 | TIM3_CH1    | STDIO_UART_TX| MXOS_UART_1_TX |
|               |    |      | TIM8_CH1    |              |                |
|               |    |      | USART6_TX   |              |                |
|               |    |      | GPIO        |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MXOS_GPIO_9   | 9  | A 15 | TIM2_CH1    |EasyLink_BUTTON|               |
|               |    |      | JTDI        |              |                |
|               |    |      | USART1_TX   |              |                |
|               |    |      | GPIO        |              |                |
|---------------+----+------+-------------+--------------+----------------|
|               | 10 | VBAT |             |
|---------------+----+------+-------------+--------------+----------------|
|               | 11 | NC   |             |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MXOS_GPIO_12  | 12 | C  7 | TIM3_CH2    | STDIO_UART_RX| MXOS_UART_1_RX |
|               |    |      | TIM8_CH2    |              |                |
|               |    |      | SPI2_SCK    |              |                |
|               |    |      | SDIO_D7     |              |                |
|               |    |      | USART6_RX   |              |                |
|               |    |      | GPIO        |              |                |
|---------------+----+------+-------------+--------------+----------------|
|               | 13 | NRST |             |              |  MICRO_RST_N   |
|---------------+----+------+-------------+--------------+----------------|
| MXOS_GPIO_14  | 14 | C 0  | WAKE_UP     |              |                |
|---------------+----+------+-------------+--------------+----------------|
|               | 15 | NC   |             |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MXOS_GPIO_16  | 16 | C 13 |     -       |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MXOS_SYS_LED  | 17 | B  8 |  TIM4_CH3   |              |                |
|               |    |      |  I2C2_SCL   |              |                |
|               |    |      |  GPIO       |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MXOS_GPIO_18  | 18 | B  9 | TIM4_CH3    |              |                |
|               |    |      | TIM10_CH1   |              |                |
|               |    |      | I2C1_SCL    |              |                |
|               |    |      | SDIO_D4     |              |                |
|               |    |      | GPIO        |              |                |
+---------------+----+--------------------+--------------+----------------+
| MXOS_GPIO_19  | 19 | B 10 | GPIO        |              |                |
+---------------+----+--------------------+--------------+----------------+
|               | 20 | GND  |             |              |                |
+---------------+----+--------------------+--------------+----------------+
|               | 21 | GND  |             |              |                |
+---------------+----+--------------------+--------------+----------------+
| MXOS_GPIO_22  | 22 | B  3 |             |              |                |
+---------------+----+--------------------+--------------+----------------+
| MXOS_GPIO_23  | 23 | A 15 | GPIO        |              |  JTAG_TDI      |
|               |    |      | USART1_TX   |              |  SPI1_SSN      |
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
|MXOS_GPIO_27   | 27 | B  3 | TIM1_ETR    |              |                |
|               |    |      | USART1_RX   |              |                |         
|               |    |      | GPIO        |              |                |    
+---------------+----+--------------------+--------------+----------------+
|               | 28 | NC   |             |              |                |
+---------------+----+--------------------+--------------+----------------+
| MXOS_GPIO_29  | 29 | B  7 | GPIO        |              | MXOS_UART_2_RX |
|               |    |      | TIM4_CH2    |              |                |
|               |    |      | USART1_RX   |              |                |
|               |    |      | I2C1_SDA    |              |                |
+---------------+----+--------------------+--------------+----------------+
| MXOS_GPIO_30  | 30 | B  6 | GPIO        |              | MXOS_UART_2_TX |
|               |    |      | TIM4_CH1    |              |                |
|               |    |      | USART1_TX   |              |                |
|               |    |      | I2C1_SCL    |              |                |
+---------------+----+--------------------+--------------+----------------+
| MXOS_GPIO_31  | 31 | B  4 | GPIO        | MXOS_RF_LED  |                |
|               |    |      | TIM3_CH1    |              |                |
|               |    |      | SDIO_D0     |              |                |  
+---------------+----+--------------------+--------------+----------------+
|               | 32 |  NC  |             |              |                | 
+---------------+----+--------------------+--------------+----------------+  
| MXOS_GPIO_33  | 33 | A 10 | TIM1_CH3    | MXOS_SYS_LED |                |  
|               |    |      | SPI5_MOSI   |              |                |  
|               |    |      | USB_FS_ID   |              |                | 
|               |    |      | GPIO        |              |                | 
+---------------+----+--------------------+--------------+----------------+  
| MXOS_GPIO_34  | 34 | A 12 | TIM1_ETR    |              |                |  
|               |    |      | USART1_RTS  |              |                |
|               |    |      | USB_FS_DP   |              |                |  
|               |    |      | GPIO        |              |                |
+---------------+----+--------------------+--------------+----------------+  
| MXOS_GPIO_35  | 35 | A 11 | TIM1_CH4    |              |                | 
|               |    |      | SPI4_MISO   |              |                |  
|               |    |      | USART1_CTS  |              |                |  
|               |    |      | USART6_TX   |              |                |  
|               |    |      | USB_FS_DM   |              |                |  
|               |    |      | GPIO        |              |                |
+---------------+----+--------------------+--------------+----------------+  
| MXOS_GPIO_36  | 36 | A  5 | TIM2_CH1    | BOOT_SEL     |                |  
|               |    |      | TIM2_ETR    |              |                |
|               |    |      | TIM8_CH1N   |              |                |  
|               |    |      | SPI1_SCK    |              |                |
|               |    |      | GPIO        |              |                | 
+---------------+----+--------------------+--------------+----------------+  
| MXOS_GPIO_37  | 37 | B  0 | TIM1_CH2N   | MFG_SEL      |                |  
|               |    |      | TIM3_CH3    |              |                |
|               |    |      | TIM8_CH2N   |              |                |  
|               |    |      | GPIO        |              |                | 
+---------------+----+--------------------+--------------+----------------+  
| MXOS_GPIO_38  | 38 | A  4 | USART2_CK   |              |                | 
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
    FLASH_PIN_QSPI_CS,
    FLASH_PIN_QSPI_CLK,
    FLASH_PIN_QSPI_D0,
    FLASH_PIN_QSPI_D1,
    FLASH_PIN_QSPI_D2,
    FLASH_PIN_QSPI_D3,
    
    MXOS_GPIO_2,
    MXOS_GPIO_4,
    MXOS_GPIO_5,
    MXOS_GPIO_6,
    MXOS_GPIO_7,
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

typedef enum
{
  MXOS_QSPI_1,
  MXOS_QSPI_MAX,/* Denotes the total number of QSPI port aliases. Not a valid QSPI alias */
  MXOS_QSPI_NONE,
}mxos_qspi_t;

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
  MXOS_FLASH_QSPI,
  MXOS_FLASH_MAX,
  MXOS_FLASH_NONE,
};

enum
{
    MXOS_PARTITION_ERROR = -1,
    MXOS_PARTITION_FILESYS,
    MXOS_PARTITION_BOOTLOADER,
    MXOS_PARTITION_APPLICATION,
    MXOS_PARTITION_ATE,
    MXOS_PARTITION_OTA_TEMP,
    MXOS_PARTITION_RF_FIRMWARE,
    MXOS_PARTITION_PARAMETER_1,
    MXOS_PARTITION_PARAMETER_2,
    MXOS_PARTITION_KV,
#ifdef MXOS_USE_BT_PARTITION
    MXOS_PARTITION_BT_FIRMWARE,
#endif
    MXOS_PARTITION_MAX,
    MXOS_PARTITION_NONE,
};


#ifdef BOOTLOADER
#define MXOS_STDIO_UART          (MXOS_UART_1)
#define MXOS_STDIO_UART_BAUDRATE (115200)
#else
#define MXOS_STDIO_UART          (MXOS_UART_1)
#define MXOS_STDIO_UART_BAUDRATE (115200)
#endif

#define MXOS_UART_FOR_APP        (MXOS_UART_1)
#define MXOS_MFG_TEST            (MXOS_UART_1)
#define MXOS_CLI_UART            (MXOS_UART_1)

/* Components connected to external I/Os*/
#define USE_QUAD_SPI_FLASH
//#define USE_QUAD_SPI_DMA

#define BOOT_SEL            (MXOS_GPIO_NONE)
#define MFG_SEL             (MXOS_GPIO_NONE)
#define EasyLink_BUTTON     (MXOS_GPIO_NONE)
#define MXOS_SYS_LED        (MXOS_GPIO_16)
#define MXOS_RF_LED         (MXOS_GPIO_NONE)

/* Arduino extention connector */
#define Arduino_RXD         (MXOS_GPIO_29)
#define Arduino_TXD         (MXOS_GPIO_30)
#define Arduino_D2          (MXOS_GPIO_NONE)
#define Arduino_D3          (MXOS_GPIO_NONE)
#define Arduino_D4          (MXOS_GPIO_19)
#define Arduino_D5          (MXOS_GPIO_16)
#define Arduino_D6          (MXOS_GPIO_14)
#define Arduino_D7          (MXOS_GPIO_NONE)

#define Arduino_D8          (MXOS_GPIO_2)
#define Arduino_D9          (MXOS_GPIO_27)
#define Arduino_CS          (MXOS_GPIO_5)
#define Arduino_SI          (MXOS_GPIO_4)
#define Arduino_SO          (MXOS_GPIO_7)
#define Arduino_SCK         (MXOS_GPIO_6)
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

// #define USE_MiCOKit_EXT

// #ifdef USE_MiCOKit_EXT
// #define MXOS_I2C_CP         (Arduino_I2C)
// #include "micokit_ext_def.h"
// #else
// #define MXOS_I2C_CP         (MXOS_I2C_NONE)
// #endif //USE_MiCOKit_EXT

#define MXOS_I2C_CP         (MXOS_I2C_NONE)

void mxos_board_init(void);


#ifdef __cplusplus
} /*extern "C" */
#endif

#endif
