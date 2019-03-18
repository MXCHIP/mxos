/**
 ******************************************************************************
 * @file    platform.c
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This file provides all MXOS Peripherals mapping table and platform
 *          specific functions.
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

#include "mxos_platform.h"
#include "mxos_board.h"
#include "mxos_board_conf.h"
#include "platform_peripheral.h"
#include "platform_logging.h"
#include "CheckSumUtils.h"

/******************************************************
*                      Macros
******************************************************/

/******************************************************
*                    Constants
******************************************************/

/******************************************************
*                   Enumerations
******************************************************/

/******************************************************
*                 Type Definitions
******************************************************/

/******************************************************
*                    Structures
******************************************************/

/******************************************************
*               Function Declarations
******************************************************/

/******************************************************
*               Variables Definitions
******************************************************/

const platform_gpio_t platform_gpio_pins[] =
{
    [MXOS_GPIO_4]  = { 4},
    [MXOS_GPIO_22] = {22},
    [MXOS_GPIO_23] = {23},
    [MXOS_GPIO_20] = {20},
    [MXOS_GPIO_21] = {21},
    [MXOS_GPIO_31] = { 0},
    [MXOS_GPIO_0]  = { 0},
    [MXOS_GPIO_1]  = { 1}, 
    [MXOS_GPIO_15] = {15},
    [MXOS_GPIO_17] = {17},
    [MXOS_GPIO_16] = {16},
    [MXOS_GPIO_14] = {14},
    [MXOS_GPIO_30] = {30},
    [MXOS_GPIO_29] = {29},
};

const platform_pwm_t *platform_pwm_peripherals = NULL;

const platform_i2c_t platform_i2c_peripherals[] =
{
    [MXOS_I2C_1] =
    {
        .pin_scl = &platform_gpio_pins[MXOS_GPIO_20],
        .pin_sda = &platform_gpio_pins[MXOS_GPIO_21],
    },
    [MXOS_I2C_2] =
    {
        .pin_scl = &platform_gpio_pins[MXOS_GPIO_0],
        .pin_sda = &platform_gpio_pins[MXOS_GPIO_1],
    },
};
platform_i2c_driver_t platform_i2c_drivers[MXOS_I2C_MAX];

const platform_uart_t platform_uart_peripherals[] = 
{
    [MXOS_UART_1] = {MX_UART_1},
    [MXOS_UART_2] = {MX_UART_2}, 
};

platform_uart_driver_t platform_uart_drivers[MXOS_UART_MAX];

const platform_spi_t platform_spi_peripherals[] = {
    [MXOS_SPI_1] = {0},
};
platform_spi_driver_t platform_spi_drivers[MXOS_SPI_MAX];

/* Flash memory devices */
const platform_flash_t platform_flash_peripherals[] = {};
platform_flash_driver_t platform_flash_drivers[MXOS_FLASH_MAX];

/* Logic partition on flash devices */
const mxos_logic_partition_t mxos_partitions[] = 
{
    [MXOS_PARTITION_BOOTLOADER] =
    {
        .partition_owner            = MXOS_FLASH_EMBEDDED,
        .partition_description      = "Bootloader",
        .partition_start_addr       = 0x0,
        .partition_length           = 0x10000,    //64k bytes
        .partition_options          = PAR_OPT_READ_EN | PAR_OPT_WRITE_DIS,
    },
    [MXOS_PARTITION_PARAMETER_1] =
    {
        .partition_owner            = MXOS_FLASH_EMBEDDED,
        .partition_description      = "PARAMETER1",
        .partition_start_addr       = 0x10000,
        .partition_length           = 0x1000, // 4k bytes
        .partition_options          = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
    },
    [MXOS_PARTITION_PARAMETER_2] =
    {
        .partition_owner            = MXOS_FLASH_EMBEDDED,
        .partition_description      = "PARAMETER2",
        .partition_start_addr       = 0x11000,
        .partition_length           = 0x1000, //4k bytes
        .partition_options          = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
    },
    [MXOS_PARTITION_APPLICATION] =
    {
        .partition_owner            = MXOS_FLASH_EMBEDDED,
        .partition_description      = "Application",
        .partition_start_addr       = 0x13000,
        .partition_length           = 0xED000, //948k bytes
        .partition_options          = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
    },
    [MXOS_PARTITION_OTA_TEMP] =
    {
        .partition_owner           = MXOS_FLASH_EMBEDDED,
        .partition_description     = "OTA Storage",
        .partition_start_addr      = 0x100000,
        .partition_length          = 0xA5E66, //664k bytes
        .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
    },
    [MXOS_PARTITION_PARAMETER_3] =
    {
        .partition_owner            = MXOS_FLASH_EMBEDDED,
        .partition_description      = "PARAMETER3",
        .partition_start_addr       = 0x12000,
        .partition_length           = 0x1000, //4k bytes
        .partition_options          = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
    },
    [MXOS_PARTITION_PARAMETER_4] =
    {
        .partition_owner            = MXOS_FLASH_EMBEDDED,
        .partition_description      = "PARAMETER4",
        .partition_start_addr       = 0xD000,
        .partition_length           = 0x1000, //4k bytes
        .partition_options          = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
    },
    [MXOS_PARTITION_SYSTEM_DATA] = 
    {
        .partition_owner            = MXOS_FLASH_EMBEDDED,
        .partition_description      = "System data",
        .partition_start_addr       = 0xE000,
        .partition_length           = 0x2000,    //8k bytes
        .partition_options          = PAR_OPT_READ_EN | PAR_OPT_WRITE_DIS,
    },
};

const platform_adc_t platform_adc_peripherals[] = {};

/******************************************************
*               Function Definitions
******************************************************/

void mxos_board_init( void )
{
  mhal_gpio_open( (mxos_gpio_t)MXOS_SYS_LED, OUTPUT_PUSH_PULL );
  mhal_gpio_low( (mxos_gpio_t)MXOS_SYS_LED );
  mhal_gpio_open( (mxos_gpio_t)MXOS_RF_LED, OUTPUT_OPEN_DRAIN_NO_PULL );
  mhal_gpio_high( (mxos_gpio_t)MXOS_RF_LED );
  
  mhal_gpio_open((mxos_gpio_t)BOOT_SEL, INPUT_PULL_UP);
  mhal_gpio_open((mxos_gpio_t)MFG_SEL, INPUT_PULL_UP);
}

void mxos_sys_led(bool onoff)
{
  if (onoff) {
    mhal_gpio_low( (mxos_gpio_t)MXOS_SYS_LED );
  } else {
    mhal_gpio_high( (mxos_gpio_t)MXOS_SYS_LED );
  }
}

void mxos_rf_led(bool onoff)
{
  if (onoff) {
    mhal_gpio_low( (mxos_gpio_t)MXOS_RF_LED );
  } else {
    mhal_gpio_high( (mxos_gpio_t)MXOS_RF_LED );
  }
}

#define BOOT_MODE_REG (*(uint32_t *)0x40001C)

#define BOOT_MODE_APP   0
#define BOOT_MODE_ATE   1
#define BOOT_MODE_QC    2

bool mxos_should_enter_mfg_mode(void)
{
    return BOOT_MODE_REG == BOOT_MODE_QC ? true : false;
}


