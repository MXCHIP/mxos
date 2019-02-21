/**
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 *
 */

#pragma once

#ifndef MXOS_PREBUILT_LIBS
#include "platform_peripheral.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/
#ifndef UART_RX_FIFO_SIZE
#define UART_RX_FIFO_SIZE (3000)
#endif
/******************************************************
 *                    Constants
 ******************************************************/
/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    MXOS_BT_PIN_POWER,
    MXOS_BT_PIN_RESET,
    MXOS_BT_PIN_HOST_WAKE,
    MXOS_BT_PIN_DEVICE_WAKE,
    MXOS_BT_PIN_MAX,
} mxos_bt_control_pin_t;

typedef enum
{
    MXOS_BT_PIN_UART_TX,
    MXOS_BT_PIN_UART_RX,
    MXOS_BT_PIN_UART_CTS,
    MXOS_BT_PIN_UART_RTS,
} mxos_bt_uart_pin_t;

typedef enum
{
    PATCHRAM_DOWNLOAD_MODE_NO_MINIDRV_CMD,
    PATCHRAM_DOWNLOAD_MODE_MINIDRV_CMD,
} mxos_bt_patchram_download_mode_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    uint32_t                          patchram_download_baud_rate;
    mxos_bt_patchram_download_mode_t  patchram_download_mode;
    uint32_t                          featured_baud_rate;
} platform_bluetooth_config_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

extern const platform_bluetooth_config_t    mxos_bt_config;

#ifndef MXOS_PREBUILT_LIBS
/* Variables to be defined by the Bluetooth supporting platform */
extern const platform_gpio_t*               mxos_bt_control_pins[];
extern const platform_gpio_t*               mxos_bt_uart_pins[];
extern const platform_uart_t*               mxos_bt_uart_peripheral;
extern       platform_uart_driver_t*        mxos_bt_uart_driver;
extern const platform_uart_config_t         mxos_bt_uart_config;
#endif

/******************************************************
 *               Function Declarations
 ******************************************************/

#ifdef __cplusplus
} /* extern "C" */
#endif
