/**
******************************************************************************
* @file    mxos_board.h
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
/* EMW3031 PIN
PIN	FUN1	FUN2	FUN3	FUN4
1/3	SWD_CLK	GPIO_7		
2/4	SWD_DIO	GPIO_8		
5	WAKE_UP0	GPIO_22		
6	WAKE_UP1	GPIO_23		
7	UART2_RTS	GPIO_47	SPI2_CS	ADC0_5
8	UART2_CTS	GPIO_46	SPI2_CLK	ADC0_4
9	UART2_TXD	GPIO_48	SPI2_TXD	ADC0_6
10	UART2_RXD	GPIO_49	SPI2_RXD	ADC0_7
11	RESET			
12	SPI0_CS	GPIO_1	PWM0_1	UART0_RTS
13	SPI0_MISO	GPIO_3	PWM0_3	UART0_RXD
14	SPI0_MOSI	GPIO_2	PWM0_2	UART0_TXD
15	SPI0_CLK	GPIO_0	PWM0_0	UART0_CTS
16	VCC_3V3			
17	GND			
18	GPIO_41			
19	I2C0_SDA	GPIO_4	PWM0_4	
20	I2C0_CLK	GPIO_5	PWM0_5	
21/24	UART1_TXD	GPIO_44		ADC0_2
22/25	UART1_RXD	GPIO_45		ADC0_3
23	GPIO_6			
26A1	I2C1_SDA	GPIO_9		
27A2	I2C1_SCL	GPIO_10		
28A3	GPIO_26			32K_OUT
29A4	GND			
30B1	GPIO_43			ADC0_1
31B2	GPIO_42			ADC0_0
32B3	GPIO_40			
33B4	GPIO_39		
MXOS_SYS_LED GPIO_16 
*/

enum
{
    MXOS_GPIO_1, 
    MXOS_GPIO_2,
    MXOS_GPIO_3,
    MXOS_GPIO_4,
    MXOS_GPIO_5, 
    MXOS_GPIO_6, 
    MXOS_GPIO_7,
    MXOS_GPIO_8,
    MXOS_GPIO_9,
    MXOS_GPIO_10,
    MXOS_GPIO_11,
    MXOS_GPIO_12,
    MXOS_GPIO_13,
    MXOS_GPIO_14,
    MXOS_GPIO_15,
    MXOS_GPIO_16,
    MXOS_GPIO_17,
    MXOS_GPIO_18,
    MXOS_GPIO_19,
    MXOS_GPIO_20,
    MXOS_GPIO_21,
    MXOS_GPIO_22,
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
    MXOS_SYS_LED,
    MXOS_GPIO_MAX, /* Denotes the total number of GPIO port aliases. Not a valid GPIO alias */
    MXOS_GPIO_NONE,
};

enum
{
    MXOS_SPI_1,
    MXOS_SPI_2,
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
} ;

enum
{
    MXOS_ADC_1,
    MXOS_ADC_2,
    MXOS_ADC_3,
    MXOS_ADC_4,
    MXOS_ADC_5,
    MXOS_ADC_6,
    MXOS_ADC_7,
    MXOS_ADC_8,
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
    MXOS_PARTITION_MAX,
    MXOS_PARTITION_NONE,
} ;


#define MXOS_STDIO_UART          MXOS_UART_1
#define MXOS_STDIO_UART_BAUDRATE (115200)

#define MXOS_UART_FOR_APP     MXOS_UART_2
#define MXOS_MFG_TEST         MXOS_UART_2
#define MXOS_CLI_UART         MXOS_UART_1

/* Components connected to external I/Os*/
#define Standby_SEL      (MXOS_GPIO_29)

/* I/O connection <-> Peripheral Connections */
#define BOOT_SEL        MXOS_GPIO_19
#define MFG_SEL         MXOS_GPIO_20
#define MXOS_RF_LED     MXOS_GPIO_30
#define EasyLink_BUTTON MXOS_GPIO_23

typedef struct {
	int country_code;
	int enable_healthmon;
	int dhcp_arp_check;
} mxos_system_config_t;

/* Arduino extention connector */
#define Arduino_RXD         (MXOS_GPIO_10)
#define Arduino_TXD         (MXOS_GPIO_9)
#define Arduino_D2          (MXOS_GPIO_NONE)
#define Arduino_D3          (MXOS_GPIO_18)
#define Arduino_D4          (MXOS_GPIO_5) 
#define Arduino_D5          (MXOS_GPIO_6)  
#define Arduino_D6          (MXOS_GPIO_32) 
#define Arduino_D7          (MXOS_GPIO_NONE)

#define Arduino_D8          (MXOS_GPIO_28)
#define Arduino_D9          (MXOS_GPIO_33)
#define Arduino_CS          (MXOS_GPIO_12)
#define Arduino_SI          (MXOS_GPIO_14)
#define Arduino_SO          (MXOS_GPIO_13)
#define Arduino_SCK         (MXOS_GPIO_15)
#define Arduino_SDA         (MXOS_GPIO_26)
#define Arduino_SCL         (MXOS_GPIO_27)

#define Arduino_A0          (MXOS_ADC_NONE)
#define Arduino_A1          (MXOS_ADC_NONE)
#define Arduino_A2          (MXOS_ADC_6)
#define Arduino_A3          (MXOS_ADC_5)
#define Arduino_A4          (MXOS_ADC_NONE)
#define Arduino_A5          (MXOS_ADC_NONE)

#define Arduino_I2C         (MXOS_I2C_2)
#define Arduino_SPI         (MXOS_SPI_1)
#define Arduino_UART        (MXOS_UART_2)

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
