/**
******************************************************************************
* @file    platform.c 
* @author  William Xu
* @version V1.0.0
* @date    05-May-2014
* @brief   This file provides all MICO Peripherals mapping table and platform
*          specific funcgtions.
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
#include "mxos_board.h"
#include "mxos_platform.h"
#include "platform_peripheral.h"
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
extern WEAK void PlatformEasyLinkButtonClickedCallback(void);
extern WEAK void PlatformStandbyButtonClickedCallback(void);
extern WEAK void PlatformEasyLinkButtonLongPressedCallback(void);
extern WEAK void bootloader_start(void);

/******************************************************
*               Variables Definitions
******************************************************/


platform_gpio_t platform_gpio_pins[] =
{
    [MXOS_GPIO_4 ]  = {.pin = PB_22,},
    [MXOS_GPIO_5 ]  = {.pin = PB_23,},
    [MXOS_GPIO_6 ]  = {.pin = PB_26,},
    [MXOS_GPIO_7 ]  = {.pin = PA_0 ,},
    [MXOS_GPIO_8 ]  = {.pin = PA_4,},
    [MXOS_GPIO_9 ]  = {.pin = PA_2,},
    [MXOS_GPIO_15 ] = {.pin = PB_29,},
    [MXOS_GPIO_16 ] = {.pin = PB_31,},
    [MXOS_GPIO_19 ] = {.pin = PA_7,},
    [MXOS_GPIO_20 ] = {.pin = PA_8,},
    [MXOS_GPIO_23 ] = {.pin = PA_12,},
    [MXOS_GPIO_24 ] = {.pin = PA_13,},
    [MXOS_GPIO_25 ] = {.pin = PA_14,},
    [MXOS_GPIO_26 ] = {.pin = PA_15,},
    [MXOS_GPIO_27 ] = {.pin = PA_16,},
    [MXOS_GPIO_28 ] = {.pin = PA_17,},
    [MXOS_GPIO_29 ] = {.pin = PA_18,},
    [MXOS_GPIO_30 ] = {.pin = PA_19,},
    [MXOS_GPIO_31 ] = {.pin = PA_27,}, // SWD_DATA
    [MXOS_GPIO_32 ] = {.pin = PB_3,},  // SWD_CLK
    [MXOS_GPIO_33 ] = {.pin = PA_30,},
    [MXOS_GPIO_34 ] = {.pin = PA_28,},
    [MXOS_GPIO_35 ] = {.pin = PA_26,},
    [MXOS_GPIO_36 ] = {.pin = PA_25,},
    [MXOS_GPIO_37 ] = {.pin = PB_1,},
    [MXOS_GPIO_38 ] = {.pin = PB_7,},
    [MXOS_GPIO_39 ] = {.pin = PB_6,},
    [MXOS_GPIO_40 ] = {.pin = PB_5,},
    [MXOS_GPIO_41 ] = {.pin = PB_4,},
    [MXOS_GPIO_42 ] = {.pin = PB_2,},
};

/*
* Possible compile time inputs:
* - Set which ADC peripheral to use for each ADC. All on one ADC allows sequential conversion on all inputs. All on separate ADCs allows concurrent conversion.
*/
/* TODO : These need fixing */
platform_adc_t platform_adc_peripherals[] =
{
  [MXOS_ADC_1] = {.pin = AD_1,},
};

/* PWM mappings */
platform_pwm_t platform_pwm_peripherals[] =
{  
  [MXOS_PWM_1] = {.pin = PA_12, .init=false,},
  [MXOS_PWM_2] = {.pin = PA_13, .init=false,},
  [MXOS_PWM_3] = {.pin = PA_25, .init=false,},
  [MXOS_PWM_4] = {.pin = PA_26, .init=false,},
  [MXOS_PWM_5] = {.pin = PA_28, .init=false,},
  [MXOS_PWM_6] = {.pin = PA_30, .init=false,},
  [MXOS_PWM_7] = {.pin = PB_4, .init=false,},
  [MXOS_PWM_8] = {.pin = PB_5, .init=false,},
  [MXOS_PWM_9] = {.pin = PB_7, .init=false,},
  [MXOS_PWM_10] = {.pin = PB_22, .init=false,},
  [MXOS_PWM_11] = {.pin = PB_23, .init=false,},
};

#if (defined EMW3080B) || (defined EMW3080C) || (defined EMW5080)
platform_spi_t platform_spi_peripherals[] =
{ 
  [MXOS_SPI_1]  =
  {
    .spi_obj.spi_idx = MBED_SPI1,
    .mosi = PA_23,
    .miso = PA_22,
    .sclk = PA_18,
    .ssel = PA_19,
  },	
};
platform_spi_driver_t platform_spi_drivers[MXOS_SPI_MAX];
#endif

platform_spi_t platform_spi_peripherals[] =
{ 
  [MXOS_SPI_1]  =
  {
    .spi_obj.spi_idx = MBED_SPI1,
    .mosi = PA_16,
    .miso = PA_17,
    .sclk = PA_18,
    .ssel = PA_19,
  },	
};
platform_spi_driver_t platform_spi_drivers[MXOS_SPI_MAX];


platform_uart_t platform_uart_peripherals[] =
{
  [MXOS_UART_1] =
  {
    .tx = PA_7,
    .rx = PA_8,
    .dma_enabled = 1,
#if (defined EMW3080B) || (defined EMW3080C) || (defined EMW5080)
    .rts = PA_22,
    .cts = PA_19, 
#endif
    .wake_lock = PMU_UART0_DEVICE,
  },
  [MXOS_UART_2] =
  {
    .tx = PB_1,
    .rx = PB_2,
    .dma_enabled = 1,
    .wake_lock = PMU_UART1_DEVICE,
  },
  [MXOS_UART_3] =
  {
    .tx = PA_18,
    .rx = PA_19,
    .dma_enabled = 1,
    .wake_lock = PMU_UART1_DEVICE,
  },
};
platform_uart_driver_t platform_uart_drivers[MXOS_UART_MAX];

platform_i2c_t platform_i2c_peripherals[] =
{
  [MXOS_I2C_1] =
  {
    .sda = PA_19,
    .scl = PA_22,
  },
};


/* Flash memory devices */
platform_flash_t platform_flash_peripherals[] =
{
  [MXOS_FLASH_SPI] =
  {
    .flash_type                   = FLASH_TYPE_SPI,
    .flash_start_addr             = 0x00000000,
    .flash_length                 = 0x200000,
  },
};

platform_flash_driver_t platform_flash_drivers[MXOS_FLASH_MAX];


/* Logic partition on flash devices */
const mxos_logic_partition_t mico_partitions[] =
{
  [MXOS_PARTITION_BOOTLOADER] =
  {
    .partition_owner           = MXOS_FLASH_SPI,
    .partition_description     = "Bootloader",
    .partition_start_addr      = 0x0000B000,
    .partition_length          =     0x8000,    //16k bytes
    .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_DIS,
  },
  [MXOS_PARTITION_APPLICATION] =
  {
    .partition_owner           = MXOS_FLASH_SPI,
    .partition_description     = "Application",
    .partition_start_addr      = 0x00013000,
    .partition_length          =    0x75000,   //468k bytes
    .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
  },
  
  [MXOS_PARTITION_PARAMETER_1] =
  {
    .partition_owner           = MXOS_FLASH_SPI,
    .partition_description     = "PARAMETER1",
    .partition_start_addr      = 0x000C8000,
    .partition_length          = 0x4000, // 16k bytes
    .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
  },
  [MXOS_PARTITION_PARAMETER_2] =
  {
    .partition_owner           = MXOS_FLASH_SPI,
    .partition_description     = "PARAMETER2",
    .partition_start_addr      = 0x000CC000,
    .partition_length          = 0x4000, //16k bytes
    .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
  },
  [MXOS_PARTITION_ATE] =
  {
    .partition_owner           = MXOS_FLASH_SPI,
    .partition_description     = "ATE",
    .partition_start_addr      = 0x000D0000,
    .partition_length          = 0x40000, //256k bytes
    .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
  },
  /*
  [MXOS_PARTITION_RDP] =
  {
    .partition_owner           = MXOS_FLASH_SPI,
    .partition_description     = "RDP code",
    .partition_start_addr      = 0x0087000,
    .partition_length          = 0x1000, //4k bytes
    .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
  },
  */
  [MXOS_PARTITION_OTA_TEMP] =
  {
    .partition_owner           = MXOS_FLASH_SPI,
    .partition_description     = "OTA Storage",
    .partition_start_addr      = 0x00110000,
    .partition_length          = 0xB5000, //724k bytes
    .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
  },
  [MXOS_PARTITION_USER] =
  {
    .partition_owner           = MXOS_FLASH_SPI,
    .partition_description     = "USER Storage",
    .partition_start_addr      = 0x001C5000,
    .partition_length          = 0x3A000, //236k bytes
    .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
  },
  [MXOS_PARTITION_SDS] =
  {
    .partition_owner           = MXOS_FLASH_SPI,
    .partition_description     = "SDS Storage",
    .partition_start_addr      = 0x001FF000,
    .partition_length          = 0x1000, //1k bytes
    .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN ,
  },
};

/******************************************************
*           Interrupt Handler Definitions
******************************************************/
/******************************************************
*               Function Definitions
******************************************************/
bool watchdog_check_last_reset( void )
{
  return false;
}

void platform_init_peripheral_irq_priorities( void )
{
}

void init_platform( void )
{
}

void init_platform_bootloader( void )
{
}

void MicoSysLed(bool onoff)
{
}

void MicoRfLed(bool onoff)
{
}


void platform_jtag_off(void)
{
  static uint8_t jtag_on = 1;
  if(jtag_on){
    sys_jtag_off();
    jtag_on = 0;
  }
}
