/**
 ******************************************************************************
 * @file    MxosDriverI2C.h
 * @author  William Xu
 * @version V1.0.0
 * @date    16-Sep-2014
 * @brief   This file provides all the headers of I2C operation functions.
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

/** @defgroup MXOS_I2C MXOS I2C Driver
* @brief  Inter-IC bus (I2C) Functions
* @{
*/

/******************************************************
 *                   Macros
 ******************************************************/  

/******************************************************
 *                   Enumerations
 ******************************************************/
/* I2C flags constants */
#define I2C_DEVICE_DMA_MASK_POSN ( 0 )
#define I2C_DEVICE_NO_DMA        ( 0 << I2C_DEVICE_DMA_MASK_POSN )
#define I2C_DEVICE_USE_DMA       ( 1 << I2C_DEVICE_DMA_MASK_POSN )



/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef int8_t mxos_i2c_t;   /**< MXOS I2C peripheral handle, MXOS_I2C_XX define by board/<board_name>/mxos_board.h. */

/**
 * I2C address width
 */
typedef enum
{
    I2C_ADDRESS_WIDTH_7BIT,
    I2C_ADDRESS_WIDTH_10BIT,
    I2C_ADDRESS_WIDTH_16BIT,
} mxos_i2c_bus_address_width_t;

/**
 * I2C speed mode
 */
typedef enum
{
    I2C_LOW_SPEED_MODE,         /* 10Khz devices */
    I2C_STANDARD_SPEED_MODE,    /* 100Khz devices */
    I2C_HIGH_SPEED_MODE         /* 400Khz devices */
} mxos_i2c_speed_mode_t;


/**
 * I2C message
 */
typedef struct
{
    const void*  tx_buffer;
    void*        rx_buffer;
    uint16_t     tx_length;
    uint16_t     rx_length;
    uint16_t     retries;    /* Number of times to retry the message */
    bool combined;           /**< If set, this message is used for both tx and rx. */
    //uint8_t      flags;      /* MESSAGE_DISABLE_DMA : if set, this flag disables use of DMA for the message */
} mxos_i2c_message_t;

typedef struct
{
   mxos_i2c_t                    port;           /**< Platform I2C port that is connected to the target I2C device, - e.g. MXOS_I2C_1 */
   uint16_t                      address;        /**< The address of the device on the I2C bus */
   mxos_i2c_bus_address_width_t  address_width;  /**< I2C device's address length */
   mxos_i2c_speed_mode_t         speed_mode;     /**< Speed mode the device operates in */
} mxos_i2c_device_t;

/**
 * I2C configuration
 */
typedef struct
{
    uint16_t                         address;       /* the address of the device on the i2c bus */
    mxos_i2c_bus_address_width_t address_width;
    uint8_t                          flags;
    mxos_i2c_speed_mode_t        speed_mode;    /* speed mode the device operates in */
} mxos_i2c_config_t;

/******************************************************
 *                 Function Declarations
 ******************************************************/



/**@brief Initialises an I2C interface
 *
 * @note Prepares an I2C hardware interface for communication as a master
 *
 * @param  device : the device for which the i2c port should be initialised
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred during initialisation
 */
merr_t mxos_i2c_init( mxos_i2c_device_t* device );


/**@brief Checks whether the device is available on a bus or not
 *
 * @param  device : the i2c device to be probed
 * @param  retries    : the number of times to attempt to probe the device
 *
 * @return    true : device is found.
 * @return    false: device is not found
 */
bool mxos_i2c_probe_dev( mxos_i2c_device_t* device, int retries );


/**@brief Initialize the mxos_i2c_message_t structure for i2c tx transaction
 *
 * @param message : pointer to a message structure, this should be a valid pointer
 * @param tx_buffer : pointer to a tx buffer that is already allocated
 * @param tx_buffer_length : number of bytes to transmit
 * @param retries    : the number of times to attempt send a message in case it can't not be sent
 *
 * @return    kNoErr    : message structure was initialised properly.
 * @return    kParamErr : one of the arguments is given incorrectly
 */
merr_t mxos_i2c_build_tx_msg(mxos_i2c_message_t* message, const void* tx_buffer, uint16_t  tx_buffer_length, uint16_t retries);

/**@brief Initialize the mxos_i2c_message_t structure for i2c rx transaction
 *
 * @param message : pointer to a message structure, this should be a valid pointer
 * @param rx_buffer : pointer to an rx buffer that is already allocated
 * @param rx_buffer_length : number of bytes to receive
 * @param retries    : the number of times to attempt receive a message in case device doesnt respond
 *
 * @return    kNoErr    : message structure was initialised properly.
 * @return    kParamErr : one of the arguments is given incorrectly
 */
merr_t mxos_i2c_build_rx_msg(mxos_i2c_message_t* message, void* rx_buffer, uint16_t rx_buffer_length, uint16_t retries);


/**@brief Initialize the mxos_i2c_message_t structure for i2c combined transaction
 *
 * @param  message : pointer to a message structure, this should be a valid pointer
 * @param tx_buffer: pointer to a tx buffer that is already allocated
 * @param rx_buffer: pointer to an rx buffer that is already allocated
 * @param tx_buffer_length: number of bytes to transmit
 * @param rx_buffer_length: number of bytes to receive
 * @param  retries    : the number of times to attempt receive a message in case device doesnt respond
 *
 * @return    kNoErr    : message structure was initialised properly.
 * @return    kParamErr : one of the arguments is given incorrectly
 */
merr_t mxos_i2c_build_comb_msg(mxos_i2c_message_t* message, const void* tx_buffer, void* rx_buffer, uint16_t tx_buffer_length, uint16_t rx_buffer_length, uint16_t retries);


/**@brief Transmits and/or receives data over an I2C interface
 *
 * @param  device             : the i2c device to communicate with
 * @param  message            : a pointer to a message (or an array of messages) to be transmitted/received
 * @param  number_of_messages : the number of messages to transfer. [1 .. N] messages
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred during message transfer
 */
merr_t mxos_i2c_transfer( mxos_i2c_device_t* device, mxos_i2c_message_t* message, uint16_t number_of_messages );


/**@brief Deinitialises an I2C device
 *
 * @param  device : the device for which the i2c port should be deinitialised
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred during deinitialisation
 */
merr_t mxos_i2c_deinit( mxos_i2c_device_t* device );


/** @} */
/** @} */



