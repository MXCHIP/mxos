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

//#include "mico.h"
#include "mxos_board.h"
#include "platform_peripheral.h"
#include "wlan_platform_common.h"

#include "button.h"

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

extern merr_t host_platform_init( void );

/******************************************************
*               Variables Definitions
******************************************************/

const platform_gpio_t platform_gpio_pins[] =
{
  /* Common GPIOs for internal use */
    [FLASH_PIN_SPI_CS  ]                = { SFLASH_CS },

  /* GPIOs for external use */
    [MXOS_GPIO_2]                       = { MBED_GPIO_2 },
    [MXOS_GPIO_8]                       = { MBED_GPIO_8 },   // UART_TXD_DEBUG
    [MXOS_GPIO_9]                       = { MBED_GPIO_9 },   // EASY LINK
    [MXOS_GPIO_12]                      = { MBED_GPIO_12 },   // UART_RXD_DEBUG
    [MXOS_GPIO_14]                      = { MBED_GPIO_14 },   // WL_HOST_WAKE
    [MXOS_GPIO_16]                      = { MBED_GPIO_16 },
    [MXOS_GPIO_17]                      = { MBED_GPIO_17 },   // I2C_SCL
    [MXOS_GPIO_18]                      = { MBED_GPIO_18 },   // I2C_SDA
    [MXOS_GPIO_19]                      = { MBED_GPIO_19 },
    [MXOS_GPIO_27]                      = { MBED_GPIO_27 },   //
    [MXOS_GPIO_29]                      = { MBED_GPIO_29 },   // UART_RXD_USER
    [MXOS_GPIO_30]                      = { MBED_GPIO_30 },   // UART_TXD_USER
    [MXOS_GPIO_31]                      = { MBED_GPIO_31 },   // RF LED
    [MXOS_GPIO_33]                      = { MBED_GPIO_33 },   // SYS LED
    [MXOS_GPIO_34]                      = { MBED_GPIO_34 },
    [MXOS_GPIO_35]                      = { MBED_GPIO_35 },
    [MXOS_GPIO_36]                      = { MBED_GPIO_36 },
    [MXOS_GPIO_37]                      = { MBED_GPIO_37 },
    [MXOS_GPIO_38]                      = { MBED_GPIO_38 },   // ADC_1
};



// const platform_pwm_t *platform_pwm_peripherals = NULL;

const platform_adc_t platform_adc_peripherals[] =
{
    [MXOS_ADC_1] = { MBED_GPIO_38 },
    [MXOS_ADC_2] = { MBED_GPIO_34 },
};

const platform_i2c_t platform_i2c_peripherals[] = {
    [MXOS_I2C_1] = {
        .mbed_scl_pin = I2C_SCL,
        .mbed_sda_pin = I2C_SDA,
    }
};


const platform_uart_t platform_uart_peripherals[] = {
    [MXOS_UART_1] =
    {
        .mbed_tx_pin = STDIO_UART_TX,
        .mbed_rx_pin = STDIO_UART_RX,
        .mbed_rts_pin = NC,
        .mbed_cts_pin = NC,
    },
    [MXOS_UART_2] =
    {
        .mbed_tx_pin = MBED_GPIO_30,
        .mbed_rx_pin = MBED_GPIO_29,
        .mbed_rts_pin =  MBED_GPIO_27,
        .mbed_cts_pin =  MBED_GPIO_35 ,
    }
};

const platform_spi_t platform_spi_peripherals[] =
{
    [MXOS_SPI_1]  =
    {
        .mbed_sclk_pin  = SFLASH_SCL,
        .mbed_mosi_pin  = SFLASH_MOSI,
        .mbed_miso_pin  = SFLASH_MISO,
    }
};


/* Flash memory devices */
const platform_flash_t platform_flash_peripherals[] =
{
  [MXOS_FLASH_EMBEDDED] =
  {
    .flash_type                   = FLASH_TYPE_EMBEDDED,
    .flash_start_addr             = 0x08000000,
    .flash_length                 = 0x80000,
  },
  [MXOS_FLASH_SPI] =
  {
    .flash_type                   = FLASH_TYPE_SPI,
    .flash_start_addr             = 0x000000,
    .flash_length                 = 0x200000,
  },
};


/* Logic partition on flash devices */
const mxos_logic_partition_t mxos_partitions[] =
{
  [MXOS_PARTITION_BOOTLOADER] =
  {
    .partition_owner           = MXOS_FLASH_EMBEDDED,
    .partition_description     = "Bootloader",
    .partition_start_addr      = 0x08000000,
    .partition_length          =     0xC000,    //32k bytes
    .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_DIS,
  },
  [MXOS_PARTITION_APPLICATION] =
  {
    .partition_owner           = MXOS_FLASH_EMBEDDED,
    .partition_description     = "Application",
    .partition_start_addr      = 0x0800C000,
    .partition_length          =    0x74000,   //464k bytes
    .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_DIS,
  },
  [MXOS_PARTITION_PARAMETER_1] =
  {
    .partition_owner           = MXOS_FLASH_SPI,
    .partition_description     = "PARAMETER1",
    .partition_start_addr      = 0x0,
    .partition_length          = 0x1000, // 4k bytes
    .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
  },
  [MXOS_PARTITION_PARAMETER_2] =
  {
    .partition_owner           = MXOS_FLASH_SPI,
    .partition_description     = "PARAMETER1",
    .partition_start_addr      = 0x1000,
    .partition_length          = 0x1000, //4k bytes
    .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
  },
  [MXOS_PARTITION_RF_FIRMWARE] =
  {
    .partition_owner           = MXOS_FLASH_SPI,
    .partition_description     = "RF Firmware",
    .partition_start_addr      = 0x2000,
    .partition_length          = 0x3E000,  //248k bytes
    .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_DIS,
  },
  [MXOS_PARTITION_OTA_TEMP] =
  {
    .partition_owner           = MXOS_FLASH_SPI,
    .partition_description     = "OTA Storage",
    .partition_start_addr      = 0x40000,
    .partition_length          = 0x70000, //448k bytes
    .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
  },
  [MXOS_PARTITION_KV] =
  {
      .partition_owner           = MXOS_FLASH_SPI,
      .partition_description     = "KV",
      .partition_start_addr      = 0xE0000,
      .partition_length          = 0x20000,//64k bytes
      .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
  },
  [MXOS_PARTITION_FILESYS] =
  {
    .partition_owner           = MXOS_FLASH_SPI,
    .partition_description     = "FILESYS",
    .partition_start_addr      = 0x100000,
    .partition_length          = 0x100000, //1M bytes
    .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
  }
};


#if defined ( USE_SPI_FLASH )
const mxos_spi_device_t mxos_spi_flash =
{
  .port         = MXOS_SPI_1,
  .chip_select  = FLASH_PIN_SPI_CS,
  .speed        = 40000000,
  .mode         = (SPI_CLOCK_RISING_EDGE | SPI_CLOCK_IDLE_HIGH | SPI_USE_DMA | SPI_MSB_FIRST),
  .bits         = 8
};
#endif

const platform_gpio_t wifi_control_pins[] =
{
  [WIFI_PIN_RESET]        = { PB_14 },
};


/* Wi-Fi SDIO bus pins. Used by platform/MCU/STM32F2xx/EMW1062_driver/wlan_SDIO.c */
const platform_gpio_t wifi_sdio_pins[] =
{
  [WIFI_PIN_SDIO_OOB_IRQ] = { SDIO_OOB_IRQ },
  [WIFI_PIN_SDIO_CLK    ] = { SDIO_CLK     },
  [WIFI_PIN_SDIO_CMD    ] = { SDIO_CMD     },
  [WIFI_PIN_SDIO_D0     ] = { SDIO_D0      },
  [WIFI_PIN_SDIO_D1     ] = { SDIO_D1      },
  [WIFI_PIN_SDIO_D2     ] = { SDIO_D2      },
  [WIFI_PIN_SDIO_D3     ] = { SDIO_D3      },
};

// /******************************************************
// *               Function Definitions
// ******************************************************/
void platform_init_peripheral_irq_priorities( void )
{
    NVIC_SetPriority( RTC_WKUP_IRQn,        1 );   /* RTC Wake-up event   */
    NVIC_SetPriority( SDIO_IRQn,            2 );   /* WLAN SDIO           */
    NVIC_SetPriority( DMA2_Stream3_IRQn,    3 );   /* WLAN SDIO DMA       */
    NVIC_SetPriority( USART6_IRQn,          6 );   /* MXOS_UART_1         */
    NVIC_SetPriority( DMA2_Stream6_IRQn,    7 );   /* MXOS_UART_1 TX DMA  */
    NVIC_SetPriority( DMA2_Stream1_IRQn,    7 );   /* MXOS_UART_1 RX DMA  */
    NVIC_SetPriority( USART2_IRQn,          6 );   /* BT UART             */
    NVIC_SetPriority( DMA1_Stream5_IRQn,    7 );   /* BT UART RX DMA      */
    NVIC_SetPriority( DMA1_Stream6_IRQn,    7 );   /* BT UART TX DMA      */
    NVIC_SetPriority( EXTI0_IRQn,           14 );  /* GPIO                */
    NVIC_SetPriority( EXTI1_IRQn,           14 );  /* GPIO                */
    NVIC_SetPriority( EXTI2_IRQn,           14 );  /* GPIO                */
    NVIC_SetPriority( EXTI3_IRQn,           14 );  /* GPIO                */
    NVIC_SetPriority( EXTI4_IRQn,           14 );  /* GPIO                */
    NVIC_SetPriority( EXTI9_5_IRQn,         14 );  /* GPIO                */
    NVIC_SetPriority( EXTI15_10_IRQn,       14 );  /* GPIO                */
}

void mxos_board_init( void )
{

    platform_init_peripheral_irq_priorities();

    /* Ensure 802.11 device is in reset. */
    host_platform_init( );

#ifndef BOOTLOADER
    RTC_HandleTypeDef RtcHandle;
    RtcHandle.Instance = RTC;

    /* Check Backup domain, and initialze RTC. */
    __PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    if (HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_DR0) != USE_RTC_BKP) {
        mxos_rtc_init();
        mxos_rtc_set_time(0);
        HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR0, USE_RTC_BKP);
    }
#endif

    /* Initialise leds */
    mhal_gpio_open       ( (mxos_gpio_t) MXOS_SYS_LED, OUTPUT_PUSH_PULL );
    mhal_gpio_low        ( (mxos_gpio_t) MXOS_SYS_LED );
    mhal_gpio_open       ( (mxos_gpio_t) MXOS_RF_LED, OUTPUT_OPEN_DRAIN_NO_PULL );
    mhal_gpio_high       ( (mxos_gpio_t) MXOS_RF_LED );

    /* Initialise selection IOs */
    mhal_gpio_open( (mxos_gpio_t) BOOT_SEL, INPUT_PULL_UP );
    mhal_gpio_open( (mxos_gpio_t) MFG_SEL, INPUT_PULL_UP );

    /* Initialise RTC */
    //platform_rtc_init( );
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

bool MicoShouldEnterMFGMode( void )
{
    if ( mhal_gpio_value( (mxos_gpio_t) BOOT_SEL ) == false && mhal_gpio_value( (mxos_gpio_t) MFG_SEL ) == false )
        return true;
    else
        return false;
}

bool MicoShouldEnterBootloader( void )
{
    if ( mhal_gpio_value( (mxos_gpio_t) BOOT_SEL ) == false && mhal_gpio_value( (mxos_gpio_t) MFG_SEL ) == true )
        return true;
    else
        return false;
}


