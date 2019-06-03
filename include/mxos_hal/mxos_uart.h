/**
 ******************************************************************************
 * @file    MxosDriverUart.h
 * @author  William Xu
 * @version V1.0.0
 * @date    16-Sep-2014
 * @brief   This file provides all the headers of UART operation functions.
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

#pragma once

#include "stdint.h"
#include "merr.h"
#include "RingBufferUtils.h"

/* Legacy definitions */
#define MxosUartInitialize          mhal_uart_open
#define MxosUartFinalize            mhal_uart_close
#define MxosUartSend                mhal_uart_write
#define MxosUartRecv                mhal_uart_read
#define MxosUartGetLengthInBuffer   mhal_uart_readd_data_len
#define MxosStdioUartInitialize     mxos_stdio_uart_init

/** @addtogroup MXOS_PLATFORM
* @{
*/

/** @defgroup MXOS_UART MXOS UART Driver
* @brief  Universal Asynchronous Receiver Transmitter (UART) Functions
* @{
*/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef int8_t mxos_uart_t;  /**< MXOS UART peripheral handle, MXOS_UART_XX define by board/<board_name>/mxos_board.h. */


#define UART_WAKEUP_MASK_POSN   0
#define UART_WAKEUP_DISABLE    (0 << UART_WAKEUP_MASK_POSN) /**< UART can not wakeup MCU from stop mode */
#define UART_WAKEUP_ENABLE     (1 << UART_WAKEUP_MASK_POSN) /**< UART can wake up MCU from stop mode */

/**
 * UART data width
 */
typedef enum
{
    DATA_WIDTH_5BIT,
    DATA_WIDTH_6BIT,
    DATA_WIDTH_7BIT,
    DATA_WIDTH_8BIT,
    DATA_WIDTH_9BIT
} platform_uart_data_width_t;

/**
 * UART stop bits
 */
typedef enum
{
    STOP_BITS_1,
    STOP_BITS_2,
} platform_uart_stop_bits_t;

/**
 * UART flow control
 */
typedef enum
{
    FLOW_CONTROL_DISABLED,
    FLOW_CONTROL_CTS,
    FLOW_CONTROL_RTS,
    FLOW_CONTROL_CTS_RTS
} platform_uart_flow_control_t;

/**
 * UART parity
 */
typedef enum
{
    NO_PARITY,
    ODD_PARITY,
    EVEN_PARITY,
} platform_uart_parity_t;


/**
 * UART configuration
 */
typedef struct
{
    uint32_t                     baud_rate;
    platform_uart_data_width_t   data_width;
    platform_uart_parity_t       parity;
    platform_uart_stop_bits_t    stop_bits;
    platform_uart_flow_control_t flow_control;
    uint8_t                      flags;          /**< if set, UART can wake up MCU from stop mode, reference: @ref UART_WAKEUP_DISABLE and @ref UART_WAKEUP_ENABLE*/
} mxos_uart_config_t;

/******************************************************
 *                 Function Declarations
 ******************************************************/



/**@brief Initialises a UART interface
 *
 * @note Prepares an UART hardware interface for communications
 *
 * @param  uart     : the interface which should be initialised
 * @param  config   : UART configuration structure
 * @param  optional_rx_buffer : Pointer to an optional RX ring buffer
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
merr_t mhal_uart_open( mxos_uart_t uart, const mxos_uart_config_t* config, ring_buffer_t* optional_rx_buffer );


/**@brief Initialises STDIO UART interface
 *
 * @note Make sure you want oto use STDIO UART other than printf
 *
 * @param  config   : UART configuration structure
 * @param  optional_rx_buffer : Pointer to an optional RX ring buffer
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
merr_t mxos_stdio_uart_init( const mxos_uart_config_t* config, ring_buffer_t* optional_rx_buffer );



/**@brief Initialises a STDIO UART interface, internal use only
 *
 * @note Prepares an UART hardware interface for stdio communications
 *
 * @param  config   : UART configuration structure
 * @param  optional_rx_buffer : Pointer to an optional RX ring buffer
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
merr_t mxos_stdio_uart_init( const mxos_uart_config_t* config, ring_buffer_t* optional_rx_buffer );


/**@brief Deinitialises a UART interface
 *
 * @param  uart : the interface which should be deinitialised
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
merr_t mhal_uart_close( mxos_uart_t uart );


/**@brief Transmit data on a UART interface
 *
 * @param  uart     : the UART interface
 * @param  data     : pointer to the start of data
 * @param  size     : number of bytes to transmit
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
merr_t mhal_uart_write( mxos_uart_t uart, const void* data, uint32_t size );


/**@brief Receive data on a UART interface
 *
 * @param  uart     : the UART interface
 * @param  data     : pointer to the buffer which will store incoming data
 * @param  size     : number of bytes to receive
 * @param  timeout  : timeout in millisecond
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
merr_t mhal_uart_read( mxos_uart_t uart, void* data, uint32_t size, uint32_t timeout );

/**@brief Read the length of the data that is already recived by uart driver and stored in buffer
 *
 * @param uart     : the UART interface
 *
 * @return    Data length
 */
uint32_t mhal_uart_readd_data_len( mxos_uart_t uart );

/** @} */
/** @} */

