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
#include "platform_peripheral.h"
#include "mxos_board_conf.h"
#include "platform_logging.h"
#include "spi_flash_platform_interface.h"
#include "wlan_platform_common.h"
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

extern merr_t host_platform_init( void );

/******************************************************
*               Variables Definitions
******************************************************/

const platform_gpio_t platform_gpio_pins[] =
{
  /* Common GPIOs for internal use */
  [MXOS_SYS_LED]                      = { GPIOB,  13 }, 
  [MXOS_RF_LED]                       = { GPIOB,  8 }, 
  [BOOT_SEL]                          = { GPIOB,  1 }, 
  [MFG_SEL]                           = { GPIOB,  0 }, 
  [EasyLink_BUTTON]                   = { GPIOA,  1 }, 
  [STDIO_UART_RX]                     = { GPIOA,  3 },  
  [STDIO_UART_TX]                     = { GPIOA,  2 },  
  [FLASH_PIN_SPI_CS  ]                = { GPIOA, 15 },
  [FLASH_PIN_SPI_CLK ]                = { GPIOB,  3 },
  [FLASH_PIN_SPI_MOSI]                = { GPIOA,  7 },
  [FLASH_PIN_SPI_MISO]                = { GPIOB,  4 },

  /* GPIOs for external use */
  [MXOS_GPIO_2]                       = { GPIOB,  2 },
  [MXOS_GPIO_8]                       = { GPIOA , 2 },
  [MXOS_GPIO_9]                       = { GPIOA,  1 },
  [MXOS_GPIO_12]                      = { GPIOA,  3 },
  [MXOS_GPIO_14]                      = { GPIOA,  0 },
  [MXOS_GPIO_16]                      = { GPIOC, 13 },
  [MXOS_GPIO_17]                      = { GPIOB, 10 },
  [MXOS_GPIO_18]                      = { GPIOB,  9 },
  [MXOS_GPIO_19]                      = { GPIOB, 12 },
  [MXOS_GPIO_27]                      = { GPIOA, 12 },  
  [MXOS_GPIO_29]                      = { GPIOA, 10 },
  [MXOS_GPIO_30]                      = { GPIOB,  6 },
  [MXOS_GPIO_31]                      = { GPIOB,  8 },
  [MXOS_GPIO_33]                      = { GPIOB, 13 },
  [MXOS_GPIO_34]                      = { GPIOA,  5 },
  [MXOS_GPIO_35]                      = { GPIOA, 11 },
  [MXOS_GPIO_36]                      = { GPIOB,  1 },
  [MXOS_GPIO_37]                      = { GPIOB,  0 },
  [MXOS_GPIO_38]                      = { GPIOA,  4 },
};

const platform_pwm_t platform_pwm_peripherals[] =
{
    [MXOS_PWM_1] =
    {
        .tim = TIM1,
        .channel = 4,
        .tim_peripheral_clock = RCC_APB2Periph_TIM1,
        .gpio_af = GPIO_AF_TIM1,
        .pin = &platform_gpio_pins[MXOS_GPIO_35],
    },
};

const platform_i2c_t platform_i2c_peripherals[] =
{
  [MXOS_I2C_1] =
  {
    .port                         = I2C2,
    .pin_scl                      = &platform_gpio_pins[MXOS_GPIO_17],
    .pin_sda                      = &platform_gpio_pins[MXOS_GPIO_18],
    .peripheral_clock_reg         = RCC_APB1Periph_I2C2,
    .tx_dma                       = DMA1,
    .tx_dma_peripheral_clock      = RCC_AHB1Periph_DMA1,
    .tx_dma_stream                = DMA1_Stream7,
    .rx_dma_stream                = DMA1_Stream5,
    .tx_dma_stream_id             = 7,
    .rx_dma_stream_id             = 5,
    .tx_dma_channel               = DMA_Channel_1,
    .rx_dma_channel               = DMA_Channel_1,
    .gpio_af_scl                  = GPIO_AF_I2C2,
    .gpio_af_sda                  = GPIO_AF9_I2C2
  },
};

platform_i2c_driver_t platform_i2c_drivers[MXOS_I2C_MAX];

const platform_uart_t platform_uart_peripherals[] =
{
  [MXOS_UART_1] =
  {
    .port                         = USART2,
    .pin_tx                       = &platform_gpio_pins[STDIO_UART_TX],
    .pin_rx                       = &platform_gpio_pins[STDIO_UART_RX],
    .pin_cts                      = NULL,
    .pin_rts                      = NULL,
    .tx_dma_config =
    {
      .controller                 = DMA1,
      .stream                     = DMA1_Stream6,
      .channel                    = DMA_Channel_4,
      .irq_vector                 = DMA1_Stream6_IRQn,
      .complete_flags             = DMA_HISR_TCIF6,
      .error_flags                = ( DMA_HISR_TEIF6 | DMA_HISR_FEIF6 ),
    },
    .rx_dma_config =
    {
      .controller                 = DMA1,
      .stream                     = DMA1_Stream5,
      .channel                    = DMA_Channel_4,
      .irq_vector                 = DMA1_Stream5_IRQn,
      .complete_flags             = DMA_HISR_TCIF5,
      .error_flags                = ( DMA_HISR_TEIF5 | DMA_HISR_FEIF5 | DMA_HISR_DMEIF5 ),
    },
  },
  [MXOS_UART_2] =
  {
    .port                         = USART1,
    .pin_tx                       = &platform_gpio_pins[MXOS_GPIO_30],
    .pin_rx                       = &platform_gpio_pins[MXOS_GPIO_29],
    .pin_cts                      = &platform_gpio_pins[MXOS_GPIO_35],
    .pin_rts                      = &platform_gpio_pins[MXOS_GPIO_27],
    .tx_dma_config =
    {
      .controller                 = DMA2,
      .stream                     = DMA2_Stream7,
      .channel                    = DMA_Channel_4,
      .irq_vector                 = DMA2_Stream7_IRQn,
      .complete_flags             = DMA_HISR_TCIF7,
      .error_flags                = ( DMA_HISR_TEIF7 | DMA_HISR_FEIF7 ),
    },
    .rx_dma_config =
    {
      .controller                 = DMA2,
      .stream                     = DMA2_Stream2,
      .channel                    = DMA_Channel_4,
      .irq_vector                 = DMA2_Stream2_IRQn,
      .complete_flags             = DMA_LISR_TCIF2,
      .error_flags                = ( DMA_LISR_TEIF2 | DMA_LISR_FEIF2 | DMA_LISR_DMEIF2 ),
    },
  },
};
platform_uart_driver_t platform_uart_drivers[MXOS_UART_MAX];

const platform_spi_t platform_spi_peripherals[] =
{
  [MXOS_SPI_1]  =
  {
    .port                         = SPI1,
    .gpio_af                      = GPIO_AF_SPI1,
    .peripheral_clock_reg         = RCC_APB2Periph_SPI1,
    .peripheral_clock_func        = RCC_APB2PeriphClockCmd,
    .pin_mosi                     = &platform_gpio_pins[FLASH_PIN_SPI_MOSI],
    .pin_miso                     = &platform_gpio_pins[FLASH_PIN_SPI_MISO],
    .pin_clock                    = &platform_gpio_pins[FLASH_PIN_SPI_CLK],
    .tx_dma =
    {
      .controller                 = DMA2,
      .stream                     = DMA2_Stream5,
      .channel                    = DMA_Channel_3,
      .irq_vector                 = DMA2_Stream5_IRQn,
      .complete_flags             = DMA_HISR_TCIF5,
      .error_flags                = ( DMA_HISR_TEIF5 | DMA_HISR_FEIF5 ),
    },
    .rx_dma =
    {
      .controller                 = DMA2,
      .stream                     = DMA2_Stream0,
      .channel                    = DMA_Channel_3,
      .irq_vector                 = DMA2_Stream0_IRQn,
      .complete_flags             = DMA_LISR_TCIF0,
      .error_flags                = ( DMA_LISR_TEIF0 | DMA_LISR_FEIF0 | DMA_LISR_DMEIF0 ),
    },
  }
};

platform_spi_driver_t platform_spi_drivers[MXOS_SPI_MAX];

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

platform_flash_driver_t platform_flash_drivers[MXOS_FLASH_MAX];

/* Logic partition on flash devices */
const mxos_logic_partition_t mxos_partitions[] =
{
  [MXOS_PARTITION_BOOTLOADER] =
  {
    .partition_owner           = MXOS_FLASH_EMBEDDED,
    .partition_description     = "Bootloader",
    .partition_start_addr      = 0x08000000,
    .partition_length          =     0x8000,    //32k bytes
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
    .partition_description     = "PARAMETER2",
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
    .partition_length          = 0x74000, //464k bytes
    .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
  },
  [MXOS_PARTITION_KV] =
  {
    .partition_owner           = MXOS_FLASH_SPI,
    .partition_description     = "KV",
    .partition_start_addr      = 0xB4000,
    .partition_length          = 0x4000,//16k bytes
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
  .port        = MXOS_SPI_1,
  .chip_select = FLASH_PIN_SPI_CS,
  .speed       = 40000000,
  .mode        = (SPI_CLOCK_RISING_EDGE | SPI_CLOCK_IDLE_HIGH | SPI_USE_DMA | SPI_MSB_FIRST ),
  .bits        = 8
};
#endif

const platform_adc_t platform_adc_peripherals[] =
{
  [MXOS_ADC_1] = { ADC1, ADC_Channel_4, RCC_APB2Periph_ADC1, 1, (platform_gpio_t*)&platform_gpio_pins[MXOS_GPIO_38] },
  [MXOS_ADC_2] = { ADC1, ADC_Channel_5, RCC_APB2Periph_ADC1, 1, (platform_gpio_t*)&platform_gpio_pins[MXOS_GPIO_34] },
};

/* Wi-Fi control pins. Used by platform/MCU/wlan_platform_common.c
* SDIO: EMW1062_PIN_BOOTSTRAP[1:0] = b'00
* gSPI: EMW1062_PIN_BOOTSTRAP[1:0] = b'01
*/
const platform_gpio_t wifi_control_pins[] =
{
  [WIFI_PIN_RESET]           = { GPIOB, 14 },
};

/* Wi-Fi SDIO bus pins. Used by platform/MCU/STM32F2xx/EMW1062_driver/wlan_SDIO.c */
const platform_gpio_t wifi_sdio_pins[] =
{
  [WIFI_PIN_SDIO_OOB_IRQ] = { GPIOA,  0 },
  [WIFI_PIN_SDIO_CLK    ] = { GPIOB, 15 },
  [WIFI_PIN_SDIO_CMD    ] = { GPIOA,  6 },
  [WIFI_PIN_SDIO_D0     ] = { GPIOB,  7 },
  [WIFI_PIN_SDIO_D1     ] = { GPIOA,  8 },
  [WIFI_PIN_SDIO_D2     ] = { GPIOA,  9 },
  [WIFI_PIN_SDIO_D3     ] = { GPIOB,  5 },
};


/******************************************************
*           Interrupt Handler Definitions
******************************************************/

MXOS_RTOS_DEFINE_ISR( USART1_IRQHandler )
{
  platform_uart_irq( &platform_uart_drivers[MXOS_UART_2] );
}

MXOS_RTOS_DEFINE_ISR( USART2_IRQHandler )
{
  platform_uart_irq( &platform_uart_drivers[MXOS_UART_1] );
}

MXOS_RTOS_DEFINE_ISR( DMA1_Stream6_IRQHandler )
{
  platform_uart_tx_dma_irq( &platform_uart_drivers[MXOS_UART_1] );
}

MXOS_RTOS_DEFINE_ISR( DMA2_Stream7_IRQHandler )
{
  platform_uart_tx_dma_irq( &platform_uart_drivers[MXOS_UART_2] );
}

MXOS_RTOS_DEFINE_ISR( DMA1_Stream5_IRQHandler )
{
  platform_uart_rx_dma_irq( &platform_uart_drivers[MXOS_UART_1] );
}

MXOS_RTOS_DEFINE_ISR( DMA2_Stream2_IRQHandler )
{
  platform_uart_rx_dma_irq( &platform_uart_drivers[MXOS_UART_2] );
}


/******************************************************
*               Function Definitions
******************************************************/

void platform_init_peripheral_irq_priorities( void )
{
  /* Interrupt priority setup. Called by MXOS/platform/MCU/STM32F2xx/platform_init.c */
  NVIC_SetPriority( RTC_WKUP_IRQn    ,  1 ); /* RTC Wake-up event   */
  NVIC_SetPriority( SDIO_IRQn        ,  2 ); /* WLAN SDIO           */
  NVIC_SetPriority( DMA2_Stream3_IRQn,  3 ); /* WLAN SDIO DMA       */
  //NVIC_SetPriority( DMA1_Stream3_IRQn,  3 ); /* WLAN SPI DMA        */
  NVIC_SetPriority( USART1_IRQn      ,  6 ); /* MXOS_UART_1         */
  NVIC_SetPriority( USART2_IRQn      ,  6 ); /* MXOS_UART_2         */
  NVIC_SetPriority( DMA1_Stream6_IRQn,  7 ); /* MXOS_UART_1 TX DMA  */
  NVIC_SetPriority( DMA1_Stream5_IRQn,  7 ); /* MXOS_UART_1 RX DMA  */
  NVIC_SetPriority( DMA2_Stream7_IRQn,  7 ); /* MXOS_UART_2 TX DMA  */
  NVIC_SetPriority( DMA2_Stream2_IRQn,  7 ); /* MXOS_UART_2 RX DMA  */
  NVIC_SetPriority( EXTI0_IRQn       , 14 ); /* GPIO                */
  NVIC_SetPriority( EXTI1_IRQn       , 14 ); /* GPIO                */
  NVIC_SetPriority( EXTI2_IRQn       , 14 ); /* GPIO                */
  NVIC_SetPriority( EXTI3_IRQn       , 14 ); /* GPIO                */
  NVIC_SetPriority( EXTI4_IRQn       , 14 ); /* GPIO                */
  NVIC_SetPriority( EXTI9_5_IRQn     , 14 ); /* GPIO                */
  NVIC_SetPriority( EXTI15_10_IRQn   , 14 ); /* GPIO                */
}

void mxos_board_init( void )
{
    /* Ensure 802.11 device is in reset. */
    host_platform_init( );

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

bool mxos_should_enter_mfg_mode(void)
{
  if(mhal_gpio_value((mxos_gpio_t)BOOT_SEL)==false && mhal_gpio_value((mxos_gpio_t)MFG_SEL)==false)
    return true;
  else
    return false;
}

bool mxos_should_enter_bootloader(void)
{
  if(mhal_gpio_value((mxos_gpio_t)BOOT_SEL)==false && mhal_gpio_value((mxos_gpio_t)MFG_SEL)==true)
    return true;
  else
    return false;
}

void mxos_eth_set_default_interface(void)
{
    
}

void platform_eth_mac_address(char *mac) {
}
void *mxos_eth_get_if_handle(void)
{
    return NULL;
}
