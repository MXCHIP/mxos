/**
******************************************************************************
* @file    platform.c
* @author  William Xu
* @version V1.0.0
* @date    05-Oct-2016
* @brief   This file provides all MXOS Peripherals mapping table and platform
*          specific functions.
******************************************************************************
*
*  The MIT License
*  Copyright (c) 2016 MXCHIP Inc.
*
*  Permission is hereby gra nted, free of charge, to any person obtaining a copy
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

#include "mxos_common.h"
#include "mxos_platform.h"

#include "mxos_board.h"
#include "button.h"
#include "moc_api.h"


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
const mhal_gpio_open_t gpio_init[] =
{
  {MXOS_GPIO_12, INPUT_PULL_UP, 0},
  {MXOS_GPIO_13, INPUT_PULL_UP, 0},
  {MXOS_GPIO_14, INPUT_PULL_UP, 0},
  {MXOS_GPIO_NC, 0, 0}
};

const mxos_pwm_pinmap_t pwm_pinmap[] = 
{
  [MXOS_PWM_1] = {.pin = MXOS_GPIO_1, },
  [MXOS_PWM_2] = {.pin = MXOS_GPIO_2, },
  [MXOS_PWM_3] = {.pin = MXOS_GPIO_12,},
  [MXOS_PWM_4] = {.pin = MXOS_GPIO_13,},
  [MXOS_PWM_5] = {.pin = MXOS_GPIO_14,},
  [MXOS_PWM_6] = {.pin = MXOS_GPIO_7, },
};

const mxos_spi_pinmap_t spi_pinmap[] =
{ 
  [MXOS_SPI_1]  =
  {
    .mosi = MXOS_GPIO_9,
    .miso = MXOS_GPIO_7,
    .sclk = MXOS_GPIO_10,
    .ssel = MXOS_GPIO_8,
  },  
};

const mxos_uart_pinmap_t uart_pinmap[] =
{
  [MXOS_UART_1] =
  {
    .tx   = MXOS_GPIO_9,
    .rx   = MXOS_GPIO_10,
    .rts  = MXOS_GPIO_7,
    .cts  = MXOS_GPIO_8, 
  },
  [MXOS_UART_2] =
  {
    .tx   = MXOS_GPIO_21,
    .rx   = MXOS_GPIO_22,
    .rts  = MXOS_GPIO_NONE,
    .cts  = MXOS_GPIO_NONE, 
  },
};

const mxos_i2c_pinmap_t i2c_pinmap[] =
{
  [MXOS_I2C_1] =
  {
    .sda = MXOS_GPIO_8,
    .scl = MXOS_GPIO_7,
  },
  [MXOS_I2C_2] =
  {
    .sda = MXOS_GPIO_9,
    .scl = MXOS_GPIO_10,
  },  
};

const platform_peripherals_pinmap_t peripherals_pinmap = 
{
  .pwm_pinmap   = pwm_pinmap,
  .spi_pinmap   = spi_pinmap,
  .uart_pinmap  = uart_pinmap,
  .i2c_pinmap   = i2c_pinmap, 
};
/******************************************************
*               Function Definitions
******************************************************/
void mxos_board_init( void )
{
#if defined (MOC) && (MOC == 1)
    extern int get_last_reset_reason(void);

    if ( get_last_reset_reason() & LAST_RST_CAUSE_WDT )
    {
       platform_log( "WARNING: Watchdog reset occured previously." );
    }

    mhal_gpio_open( (mxos_gpio_t)MXOS_SYS_LED, OUTPUT_PUSH_PULL );
    mhal_gpio_low( (mxos_gpio_t)MXOS_SYS_LED );

    mhal_gpio_open( (mxos_gpio_t)MXOS_RF_LED, OUTPUT_PUSH_PULL );
    mhal_gpio_high( (mxos_gpio_t)MXOS_RF_LED );
#endif
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



