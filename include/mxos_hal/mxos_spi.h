/**
 ******************************************************************************
 * @file    MxosDriverSpi.h
 * @author  William Xu
 * @version V1.0.0
 * @date    16-Sep-2014
 * @brief   This file provides all the headers of SPi operation functions.
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


/** @addtogroup MXOS_PLATFORM
* @{
*/
/** @defgroup MXOS_SPI MXOS SPI Driver
* @brief  Serial Peripheral Interface (SPI) Functions
* @{
*/
/******************************************************
 *                    Constants
 ******************************************************/
/* SPI mode constants */
#define SPI_CLOCK_RISING_EDGE  ( 1 << 0 )
#define SPI_CLOCK_FALLING_EDGE ( 0 << 0 )
#define SPI_CLOCK_IDLE_HIGH    ( 1 << 1 )
#define SPI_CLOCK_IDLE_LOW     ( 0 << 1 )
#define SPI_USE_DMA            ( 1 << 2 )
#define SPI_NO_DMA             ( 0 << 2 )
#define SPI_MSB_FIRST          ( 1 << 3 )
#define SPI_LSB_FIRST          ( 0 << 3 )


/******************************************************
 *                   Enumerations
 ******************************************************/


/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef int8_t mxos_spi_t;   /**< MXOS SPI peripheral handle, MXOS_SPI_XX define by board/<board_name>/mxos_board.h. */


/**
 * SPI message segment
 */
typedef struct
{
    const void* tx_buffer;
    void*       rx_buffer;
    uint32_t    length;
} mxos_spi_message_segment_t;

typedef struct
{
    mxos_spi_t  port;
    mxos_gpio_t chip_select;
    uint32_t     speed;
    uint8_t      mode;
    uint8_t      bits;
} mxos_spi_device_t;

/******************************************************
 *                     Variables
 ******************************************************/
 
/******************************************************
 *                 Function Declarations
 ******************************************************/



/**@brief Initialises the SPI interface for a given SPI device
 *
 * @note  Prepares a SPI hardware interface for communication as a master
 *
 * @param  spi : the SPI device to be initialised
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if the SPI device could not be initialised
 */
merr_t mxos_spi_init( const mxos_spi_device_t* spi );


/**@brief Transmits and/or receives data from a SPI device
 *
 * @param  spi      : the SPI device to be initialised
 * @param  segments : a pointer to an array of segments
 * @param  number_of_segments : the number of segments to transfer
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred
 */
merr_t mxos_spi_transfer( const mxos_spi_device_t* spi, const mxos_spi_message_segment_t* segments, uint16_t number_of_segments );


/**@brief De-initialises a SPI interface
 *
 * @note Turns off a SPI hardware interface
 *
 * @param  spi : the SPI device to be de-initialised
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred
 */
merr_t mxos_spi_deinit( const mxos_spi_device_t* spi );

/** @} */
/** @} */

