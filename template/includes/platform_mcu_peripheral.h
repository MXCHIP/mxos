/**
 ******************************************************************************
 * @file    platform_mcu_peripheral.h
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This file provide all the headers of functions for stm32f2xx platform
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#pragma once
#include "mxos_rtos.h"
#include "RingBufferUtils.h"

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

/******************************************************
 *                 Type Definitions
 ******************************************************/


/**
 * UART flow control for driver
 */
typedef enum
{
    FLOW_CONTROL_DISABLED_DRV,
    FLOW_CONTROL_CTS_DRV,
    FLOW_CONTROL_RTS_DRV,
    FLOW_CONTROL_CTS_RTS_DRV
} platform_uart_driver_flow_control_t;

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    uint8_t unimplemented;
} platform_gpio_t;

typedef struct
{
    uint8_t unimplemented;
} platform_adc_t;

typedef struct
{
    uint8_t unimplemented;
} platform_pwm_t;


/* DMA can be enabled by setting SPI_USE_DMA */
typedef struct
{
    uint8_t unimplemented;
} platform_spi_t;

typedef struct
{
    platform_spi_t*     peripheral;
    mxos_mutex_t        spi_mutex;
    mxos_bool_t         initialized;
} platform_spi_driver_t;

typedef struct
{
    uint8_t unimplemented;
} platform_spi_slave_driver_t;

typedef struct
{
    uint8_t unimplemented;
} platform_i2c_t;

typedef struct
{
    mxos_mutex_t              i2c_mutex;
} platform_i2c_driver_t;

typedef void (* wakeup_irq_handler_t)(void *arg);

typedef struct
{
    uint32_t  port_id;
} platform_uart_t;

typedef struct
{
    uint8_t                    id;
    ring_buffer_t*             rx_ring_buffer;
    mxos_semaphore_t           rx_complete;
    mxos_semaphore_t           tx_complete;
    mxos_mutex_t               tx_mutex;
    mxos_semaphore_t           sem_wakeup;
    volatile uint32_t          tx_size;
    volatile uint32_t          rx_size;
    volatile OSStatus          last_receive_result;
    volatile OSStatus          last_transmit_result;
    platform_uart_driver_flow_control_t   flow_control;
} platform_uart_driver_t;


typedef struct
{
    uint32_t                   flash_type;
    uint32_t                   flash_start_addr;
    uint32_t                   flash_length;
} platform_flash_t;

typedef struct
{
    const platform_flash_t*    peripheral;
    mxos_mutex_t               flash_mutex;
    volatile bool              initialized;
} platform_flash_driver_t;

/******************************************************
 *                 Global Variables
 ******************************************************/


/******************************************************
 *               Function Declarations
 ******************************************************/


#ifdef __cplusplus
} /* extern "C" */
#endif





