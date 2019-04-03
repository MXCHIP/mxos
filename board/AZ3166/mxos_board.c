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

#include "mxos.h"
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

/******************************************************
*               Variables Definitions
******************************************************/

const platform_gpio_t platform_gpio_pins[] =
{
  /* Common GPIOs for internal use */
   [FLASH_PIN_QSPI_CS ]                = { QSPI_CS },
   [FLASH_PIN_QSPI_CLK]                = { QSPI_CLK },
   [FLASH_PIN_QSPI_D0]                 = { QSPI_D0 },
   [FLASH_PIN_QSPI_D1]                 = { QSPI_D1 },
   [FLASH_PIN_QSPI_D2]                 = { QSPI_D2 },
   [FLASH_PIN_QSPI_D3]                 = { QSPI_D3 },

  /* GPIOs for external use */
  [MXOS_GPIO_2]                       = { MBED_GPIO_2 },
  [MXOS_GPIO_4]                       = { MBED_GPIO_4 },   // SPI_MOSI
  [MXOS_GPIO_5]                       = { MBED_GPIO_5 },   // SPI_CS
  [MXOS_GPIO_6]                       = { MBED_GPIO_6 },   // SPI_SCK
  [MXOS_GPIO_7]                       = { MBED_GPIO_7 },   // SPI_MISO
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


const platform_pwm_t platform_pwm_peripherals[] ={
     [MXOS_PWM_1] = {
         .mbed_pwm_pin = MBED_GPIO_27,
     },
     [MXOS_PWM_2] = {
         .mbed_pwm_pin = MBED_GPIO_31,
     },
     [MXOS_PWM_3] = {
         .mbed_pwm_pin = MBED_GPIO_12,
     },
 };

// const platform_adc_t platform_adc_peripherals[] =
// {
//   [MXOS_ADC_1] = { ADC1, ADC_Channel_4, RCC_APB2Periph_ADC1, 1, (platform_gpio_t*)&platform_gpio_pins[MXOS_GPIO_38] },
//   [MXOS_ADC_2] = { ADC1, ADC_Channel_5, RCC_APB2Periph_ADC1, 1, (platform_gpio_t*)&platform_gpio_pins[MXOS_GPIO_34] },
// };

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
    }
};

// const platform_spi_t platform_spi_peripherals[] =
// {
//   [MXOS_SPI_1]  =
//   {
//     .port                         = SPI2,
//     .gpio_af                      = GPIO_AF_SPI2,
//     .peripheral_clock_reg         = RCC_APB1Periph_SPI2,
//     .peripheral_clock_func        = RCC_APB1PeriphClockCmd,
//     .pin_mosi                     = &platform_gpio_pins[MXOS_GPIO_4],
//     .pin_miso                     = &platform_gpio_pins[MXOS_GPIO_7],
//     .pin_clock                    = &platform_gpio_pins[MXOS_GPIO_6],
//     .tx_dma =
//     {
//       .controller                 = DMA1,
//       .stream                     = DMA1_Stream4,
//       .channel                    = DMA_Channel_0,
//       .irq_vector                 = DMA1_Stream4_IRQn,
//       .complete_flags             = DMA_HISR_TCIF4,
//       .error_flags                = ( DMA_HISR_TEIF4 | DMA_HISR_FEIF4 ),
//     },
//     .rx_dma =
//     {
//       .controller                 = DMA1,
//       .stream                     = DMA1_Stream3,
//       .channel                    = DMA_Channel_0,
//       .irq_vector                 = DMA1_Stream3_IRQn,
//       .complete_flags             = DMA_LISR_TCIF3,
//       .error_flags                = ( DMA_LISR_TEIF3 | DMA_LISR_FEIF3 | DMA_LISR_DMEIF3 ),
//     },
//   }
// };

// platform_spi_driver_t platform_spi_drivers[MXOS_SPI_MAX];

const platform_qspi_t platform_qspi_peripherals[] = {
    [MXOS_QSPI_1] =
    {
        .port                   = QUADSPI,
        .FlashID                = QSPI_FLASH_ID_2,
        .pin_d0                 = &platform_gpio_pins[FLASH_PIN_QSPI_D0],
        .pin_d1                 = &platform_gpio_pins[FLASH_PIN_QSPI_D1],
        .pin_d2                 = &platform_gpio_pins[FLASH_PIN_QSPI_D2],
        .pin_d3                 = &platform_gpio_pins[FLASH_PIN_QSPI_D3],
        .pin_clock              = &platform_gpio_pins[FLASH_PIN_QSPI_CLK],
        .pin_cs                 = &platform_gpio_pins[FLASH_PIN_QSPI_CS],
#ifdef USE_QUAD_SPI_DMA
        .dma =
        {
            .controller         = DMA2,
            .stream             = DMA2_Stream7,
            .channel            = DMA_Channel_3,
            .complete_flags     = DMA_FLAG_TCIF7,
        },
#endif
    }
};

// //platform_qspi_driver_t platform_qspi_drivers[MXOS_QSPI_MAX];

/* Flash memory devices */
const platform_flash_t platform_flash_peripherals[] =
{
   [MXOS_FLASH_EMBEDDED] =
   {
     .flash_type                   = FLASH_TYPE_EMBEDDED,
     .flash_start_addr             = 0x08000000,
     .flash_length                 = 0x100000,
   },
   [MXOS_FLASH_QSPI] =
   {
     .flash_type                   = FLASH_TYPE_QSPI,
     .flash_start_addr             = 0x000000,
     .flash_length                 = 0x200000,
   },
};


/* Logic partition on flash devices */
const platform_logic_partition_t mxos_partitions[] =
{
    [MXOS_PARTITION_BOOTLOADER] =
    {
        .partition_owner = MXOS_FLASH_EMBEDDED,
        .partition_description = "Bootloader",
        .partition_start_addr = 0x08000000,
        .partition_length = 0x8000,    //32k bytes
        .partition_options = PAR_OPT_READ_EN | PAR_OPT_WRITE_DIS,
    },
    [MXOS_PARTITION_APPLICATION] =
    {
        .partition_owner = MXOS_FLASH_EMBEDDED,
        .partition_description = "Application",
        .partition_start_addr = 0x08008000,
        .partition_length = 0xF8000,   //992k bytes
        .partition_options = PAR_OPT_READ_EN | PAR_OPT_WRITE_DIS,
    },
    [MXOS_PARTITION_ATE] =
    {
        .partition_owner = MXOS_FLASH_EMBEDDED,
        .partition_description = "ATEFirmware",
        .partition_start_addr = 0x080A0000,
        .partition_length = 0x60000, //384k bytes
        .partition_options = PAR_OPT_READ_EN | PAR_OPT_WRITE_DIS,
    },
    [MXOS_PARTITION_PARAMETER_1] =
    {
        .partition_owner = MXOS_FLASH_QSPI,
        .partition_description = "PARAMETER1",
        .partition_start_addr = 0x0,
        .partition_length = 0x1000, // 4k bytes
        .partition_options = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
    },
    [MXOS_PARTITION_PARAMETER_2] =
    {
        .partition_owner = MXOS_FLASH_QSPI,
        .partition_description = "PARAMETER2",
        .partition_start_addr = 0x1000,
        .partition_length = 0x1000, //4k bytes
        .partition_options = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
    },
    [MXOS_PARTITION_RF_FIRMWARE] =
    {
        .partition_owner = MXOS_FLASH_QSPI,
        .partition_description = "RF Firmware",
        .partition_start_addr = 0x2000,
        .partition_length = 0x3E000,  //248k bytes
        .partition_options = PAR_OPT_READ_EN | PAR_OPT_WRITE_DIS,
    },
    [MXOS_PARTITION_OTA_TEMP] =
    {
        .partition_owner = MXOS_FLASH_QSPI,
        .partition_description = "OTA Storage",
        .partition_start_addr = 0x40000,
        .partition_length = 0xB0000, //704k bytes
        .partition_options = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
    },
    [MXOS_PARTITION_KV] =
    {
        .partition_owner           = MXOS_FLASH_QSPI,
        .partition_description     = "KV",
        .partition_start_addr      = 0xF0000,
        .partition_length          = 0x10000,//64k bytes
        .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
    },
    [MXOS_PARTITION_FILESYS] =
    {
        .partition_owner = MXOS_FLASH_QSPI,
        .partition_description = "FILESYS",
        .partition_start_addr = 0x100000,
        .partition_length = 0x100000, //1M bytes
        .partition_options = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
    }
};

// #if defined ( USE_MXOS_SPI_FLASH )
// const mxos_spi_device_t mxos_spi_flash =
// {
//   .port        = MXOS_SPI_1,
//   .chip_select = FLASH_PIN_SPI_CS,
//   .speed       = 40000000,
//   .mode        = (SPI_CLOCK_RISING_EDGE | SPI_CLOCK_IDLE_HIGH | SPI_USE_DMA | SPI_MSB_FIRST ),
//   .bits        = 8
// };
// #endif

/** Wi-Fi control pins. Used by platform/MCU/wlan_platform_common.c
 */
const platform_gpio_t wifi_control_pins[] =
{
    [WIFI_PIN_RESET      ] = { PA_9 },
    [WIFI_PIN_32K_CLK    ] = { PA_8 },
};

platform_gpio_driver_t      wifi_control_pin_drivers[WIFI_PIN_CONTROL_MAX];

/* Wi-Fi SDIO bus pins. Used by platform/MCU/STM32F4xx/wlan_bus_driver/wlan_SDIO.c */
const platform_gpio_t wifi_sdio_pins[] =
{
  [WIFI_PIN_SDIO_OOB_IRQ] = { SDIO_OOB_IRQ },
  [WIFI_PIN_SDIO_CLK    ] = { SDIO_CLK },
  [WIFI_PIN_SDIO_CMD    ] = { SDIO_CMD },
  [WIFI_PIN_SDIO_D0     ] = { SDIO_D0 },
  [WIFI_PIN_SDIO_D1     ] = { SDIO_D1 },
  [WIFI_PIN_SDIO_D2     ] = { SDIO_D2 },
  [WIFI_PIN_SDIO_D3     ] = { SDIO_D3 },
};

platform_gpio_driver_t      wifi_sdio_pin_drivers[WIFI_PIN_SDIO_MAX];


// /* Bluetooth control pins.*/
// static const platform_gpio_t internal_bt_control_pins[] =
// {
//     /* Reset pin unavailable */
//     [MXOS_BT_PIN_POWER      ] = { GPIOC,  3 },
//     [MXOS_BT_PIN_HOST_WAKE  ] = { GPIOC,  2 },
//     [MXOS_BT_PIN_DEVICE_WAKE] = { GPIOC,  1 }
// };

// const platform_gpio_t* mxos_bt_control_pins[] =
// {
//     /* Reset pin unavailable */
//     [MXOS_BT_PIN_POWER      ] = &internal_bt_control_pins[MXOS_BT_PIN_POWER      ],
//     [MXOS_BT_PIN_HOST_WAKE  ] = &internal_bt_control_pins[MXOS_BT_PIN_HOST_WAKE  ],
//     [MXOS_BT_PIN_DEVICE_WAKE] = &internal_bt_control_pins[MXOS_BT_PIN_DEVICE_WAKE],
//     [MXOS_BT_PIN_RESET      ] = NULL,
// };

// /* Bluetooth UART pins.*/
// static const platform_gpio_t internal_bt_uart_pins[] =
// {
//     [MXOS_BT_PIN_UART_TX ] = { GPIOA,  2 },
//     [MXOS_BT_PIN_UART_RX ] = { GPIOA,  3 },
//     [MXOS_BT_PIN_UART_CTS] = { GPIOA,  0 },
//     [MXOS_BT_PIN_UART_RTS] = { GPIOA,  1 },
// };

// const platform_gpio_t* mxos_bt_uart_pins[] =
// {
//     [MXOS_BT_PIN_UART_TX ] = &internal_bt_uart_pins[MXOS_BT_PIN_UART_TX ],
//     [MXOS_BT_PIN_UART_RX ] = &internal_bt_uart_pins[MXOS_BT_PIN_UART_RX ],
//     [MXOS_BT_PIN_UART_CTS] = &internal_bt_uart_pins[MXOS_BT_PIN_UART_CTS],
//     [MXOS_BT_PIN_UART_RTS] = &internal_bt_uart_pins[MXOS_BT_PIN_UART_RTS],
// };

// static const platform_uart_t internal_bt_uart_peripheral =
// {
//   .port                         = USART2,
//   .pin_tx                       = &internal_bt_uart_pins[MXOS_BT_PIN_UART_TX ],
//   .pin_rx                       = &internal_bt_uart_pins[MXOS_BT_PIN_UART_RX ],
//   .pin_cts                      = &internal_bt_uart_pins[MXOS_BT_PIN_UART_CTS ],
//   .pin_rts                      = &internal_bt_uart_pins[MXOS_BT_PIN_UART_RTS ],
//   .tx_dma_config =
//   {
//     .controller                 = DMA1,
//     .stream                     = DMA1_Stream6,
//     .channel                    = DMA_Channel_4,
//     .irq_vector                 = DMA1_Stream6_IRQn,
//     .complete_flags             = DMA_HISR_TCIF6,
//     .error_flags                = ( DMA_HISR_TEIF6 | DMA_HISR_FEIF6 ),
//   },
//   .rx_dma_config =
//   {
//     .controller                 = DMA1,
//     .stream                     = DMA1_Stream5,
//     .channel                    = DMA_Channel_4,
//     .irq_vector                 = DMA1_Stream5_IRQn,
//     .complete_flags             = DMA_HISR_TCIF5,
//     .error_flags                = ( DMA_HISR_TEIF5 | DMA_HISR_FEIF5 | DMA_HISR_DMEIF5 ),
//   },
// };

// static platform_uart_driver_t internal_bt_uart_driver;
// const platform_uart_t*        mxos_bt_uart_peripheral = &internal_bt_uart_peripheral;
// platform_uart_driver_t*       mxos_bt_uart_driver     = &internal_bt_uart_driver;


// /* Bluetooth UART configuration. Used by libraries/bluetooth/internal/bus/UART/bt_bus.c */
// const platform_uart_config_t mxos_bt_uart_config =
// {
//     .baud_rate    = 115200,
//     .data_width   = DATA_WIDTH_8BIT,
//     .parity       = NO_PARITY,
//     .stop_bits    = STOP_BITS_1,
//     .flow_control = FLOW_CONTROL_CTS_RTS, //FLOW_CONTROL_DISABLED,
// };

// /*BT chip specific configuration information*/
// const platform_bluetooth_config_t mxos_bt_config =
// {
//     .patchram_download_mode      = PATCHRAM_DOWNLOAD_MODE_MINIDRV_CMD,
//     .patchram_download_baud_rate = 115200,
//     .featured_baud_rate          = 3000000
// };


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

    mhal_gpio_open( MXOS_SYS_LED, OUTPUT_PUSH_PULL );
    mhal_gpio_low( MXOS_SYS_LED );
    mhal_gpio_open( MXOS_RF_LED, OUTPUT_OPEN_DRAIN_NO_PULL );
    mhal_gpio_high( MXOS_RF_LED );

    mhal_gpio_open( BOOT_SEL, INPUT_PULL_UP );
    mhal_gpio_open ( MFG_SEL, INPUT_PULL_UP );
}

void mxos_sys_led( bool onoff )
{
    if ( onoff ) {
        mhal_gpio_low( MXOS_SYS_LED );
    } else {
        mhal_gpio_high( MXOS_SYS_LED );
    }
}

void mxos_rf_led( bool onoff )
{
    if ( onoff ) {
        mhal_gpio_low( MXOS_RF_LED );
    } else {
        mhal_gpio_high( MXOS_RF_LED );
    }
}

bool MicoShouldEnterMFGMode( void )
{
    if ( mhal_gpio_value( BOOT_SEL ) == false && mhal_gpio_value( MFG_SEL ) == false )
        return true;
    else
        return false;
}

bool MicoShouldEnterBootloader( void )
{
    if ( mhal_gpio_value( BOOT_SEL ) == false && mhal_gpio_value( MFG_SEL ) == true )
        return true;
    else
        return false;
}


