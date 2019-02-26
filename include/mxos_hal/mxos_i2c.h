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

#ifndef __MXOSDRIVERI2C_H__
#define __MXOSDRIVERI2C_H__

#pragma once
#include "mxos_common.h"
#include "platform_peripheral.h"

/* Legacy definitions */
#define MxosI2cInitialize mxos_i2c_init
#define MxosI2cProbeDevice mxos_i2c_probe_dev 
#define MxosI2cBuildTxMessage mxos_i2c_build_tx_msg
#define MxosI2cBuildRxMessage mxos_i2c_build_rx_msg
#define MxosI2cBuildCombinedMessage mxos_i2c_build_comb_msg
#define MxosI2cTransfer mxos_i2c_transfer
#define MxosI2cFinalize mxos_i2c_deinit

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



/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef int8_t mxos_i2c_t;   /**< MXOS I2C peripheral handle, MXOS_I2C_XX define by board/<board_name>/mxos_board.h. */

typedef platform_i2c_bus_address_width_t        mxos_i2c_bus_address_width_t;
typedef platform_i2c_speed_mode_t               mxos_i2c_speed_mode_t;
typedef platform_i2c_message_t                  mxos_i2c_message_t;

typedef struct
{
   mxos_i2c_t                    port;           /**< Platform I2C port that is connected to the target I2C device, - e.g. MXOS_I2C_1 */
   uint16_t                      address;        /**< The address of the device on the I2C bus */
   mxos_i2c_bus_address_width_t  address_width;  /**< I2C device's address length */
   mxos_i2c_speed_mode_t         speed_mode;     /**< Speed mode the device operates in */
} mxos_i2c_device_t;

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
OSStatus mxos_i2c_init( mxos_i2c_device_t* device );


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
OSStatus mxos_i2c_build_tx_msg(mxos_i2c_message_t* message, const void* tx_buffer, uint16_t  tx_buffer_length, uint16_t retries);

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
OSStatus mxos_i2c_build_rx_msg(mxos_i2c_message_t* message, void* rx_buffer, uint16_t rx_buffer_length, uint16_t retries);


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
OSStatus mxos_i2c_build_comb_msg(mxos_i2c_message_t* message, const void* tx_buffer, void* rx_buffer, uint16_t tx_buffer_length, uint16_t rx_buffer_length, uint16_t retries);


/**@brief Transmits and/or receives data over an I2C interface
 *
 * @param  device             : the i2c device to communicate with
 * @param  message            : a pointer to a message (or an array of messages) to be transmitted/received
 * @param  number_of_messages : the number of messages to transfer. [1 .. N] messages
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred during message transfer
 */
OSStatus mxos_i2c_transfer( mxos_i2c_device_t* device, mxos_i2c_message_t* message, uint16_t number_of_messages );


/**@brief Deinitialises an I2C device
 *
 * @param  device : the device for which the i2c port should be deinitialised
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred during deinitialisation
 */
OSStatus mxos_i2c_deinit( mxos_i2c_device_t* device );


/** @} */
/** @} */

#endif


