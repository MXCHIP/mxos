/**
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 *
 */

/** @file
 *  Defines functions for using the MXOS Bluetooth Framework
 */

#pragma once

#include "mxos.h"
#include "mxos_bt_types.h"
#include "mxos_bt_dev.h"
#include "mxos_bt_smart_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                     Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/** MXOS Bluetooth Framework mode of operation
 */
typedef enum
{
    MXOS_BT_MPAF_MODE, /**< The framework uses Multi-Profile Application Framework (MPAF). The entire Bluetooth stack runs on the controller. The host controls the controller using remote procedure calls (RPC) */
    MXOS_BT_HCI_MODE,  /**< The framework uses standard Host Controller Interface (HCI). The upper stack runs on the host and the lower stack runs on the controller                                                         */
} mxos_bt_mode_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/** MXOS Bluetooth packet.
 *  An opaque data type representing a generic, zero-copy packet used for transporting data to/from the Bluetooth controller.
 */
typedef struct bt_packet mxos_bt_packet_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

extern mxos_worker_thread_t mxos_bt_worker_thread;
extern mxos_worker_thread_t mxos_bt_evt_worker_thread;

#define MXOS_BT_WORKER_THREAD       ((mxos_worker_thread_t *)&mxos_bt_worker_thread)
#define MXOS_BT_EVT_WORKER_THREAD   ((mxos_worker_thread_t *)&mxos_bt_evt_worker_thread)

/******************************************************
 *               Function Declarations
 ******************************************************/

/*****************************************************************************/
/** @defgroup mxosbt      Bluetooth
 *
 *  MXOS Bluetooth Framework Functions
 */
/*****************************************************************************/

/*****************************************************************************/
/** @addtogroup btfwmgmt  Framework
 *  @ingroup mxosbt
 *
 *  Bluetooth Framework Management Functions
 *
 *
 *  @{
 */
/*****************************************************************************/

/** Initialise the MXOS Bluetooth Framework
 *
 * This function initialises the local Bluetooth device and the framework core
 * components to operate in the mode specified. Upon return, the device is
 * powered.
 *
 * @note To switch mode, invoke @ref mxos_bt_deinit to tear down the current
 *       operating mode, and call this function with the desired mode.
 *
 * @param mode        : The framework mode of operation, only MXOS_BT_HCI_MODE is
 *                      supported now
 * @param device_name : A user-friendly name of the local Bluetooth device. A
 *                      name longer than 21 characters will be truncated.
 * @param client_links :  Max cocurrent connections as a BLE client
 * @param server_links : 
 *
 * @return    MXOS_SUCCESS : on success;
 *            MXOS_ERROR   : if an error occurred
 */

OSStatus mxos_bt_init( mxos_bt_mode_t mode, const char* device_name, uint8_t client_links, uint8_t server_links );

/** Deinitialise the MXOS Bluetooth Framework
 *
 * This function tears down all active framework components. Depending on the
 * hardware platform used, it may also power down the Bluetooth device.
 *
 * @return    MXOS_SUCCESS : on success;
 *            MXOS_ERROR   : if an error occurred
 */
OSStatus mxos_bt_deinit( void );

/** Initialise the device address of the local Bluetooth device
 *
 * This function provides users with a option to overwrite the default address of
 * the local Bluetooth device with the address provided. Once called, @ref mxos_bt_init()
 * overwrites the default address with the address provided. Users can selectively
 * overwrite bits of the default address by setting the correspondping bits in the
 * 'mask' argument to 1.
 *
 * @warning When used, this function *MUST* be called before mxos_bt_init()
 *
 * @param[in] address : new address
 * @param[in] mask    : masking bits
 *
 * @return    MXOS_SUCCESS : on success;
 *            MXOS_ERROR   : if an error occurred
 */
OSStatus mxos_bt_init_address( const mxos_bt_device_address_t* address, const mxos_bt_device_address_t* mask );


/** Start manufacturing test mode
 *
 * @param[in] config : Configuration of the UART peripheral that connects to the host PC
 *
 * @return    MXOS_SUCCESS : on success;
 *            MXOS_ERROR   : if an error occurred
 */
OSStatus mxos_bt_start_mfgtest_mode( const mxos_uart_config_t* config );

/** @} */

/*****************************************************************************/
/** @addtogroup btdevmgmt  Device
 *  @ingroup mxosbt
 *
 *  Bluetooth Device Management Functions
 *
 *
 *  @{
 */
/*****************************************************************************/

/** Retrieve the device address of the local Bluetooth device
 *
 * @param[out] address : device address
 *
 * @return    MXOS_TRUE  : is device address successfully retrieved;
 *            MXOS_FALSE : if not or if an error occurred
 */
OSStatus mxos_bt_device_get_address( mxos_bt_device_address_t* address );

/** Retrieve the user-friendly name of the local Bluetooth device
 *
 * @return pointer to the device name string
 */
const char*    mxos_bt_device_get_name( void );

/** Check if the local Bluetooth device is powered
 *
 * @return    MXOS_TRUE  : if powered on;
 *            MXOS_FALSE : if not powered or if an error occurred
 */
mxos_bool_t   mxos_bt_device_is_on( void );

/** Check if the local Bluetooth device is connectable state
 *
 * @return    MXOS_TRUE  : is in connectable state;
 *            MXOS_FALSE : if not or if an error occurred
 */
mxos_bool_t   mxos_bt_device_is_connectable( void );

/** Check if the local Bluetooth device is discoverable state
 *
 * @return    MXOS_TRUE  : is in discoverable state;
 *            MXOS_FALSE : if not or if an error occurred
 */
mxos_bool_t   mxos_bt_device_is_discoverable( void );


/** @} */

/*****************************************************************************/
/** @addtogroup btdevmgmt  Device
 *  @ingroup mxosbt
 *
 *  Bluetooth Pairing Management Functions
 *
 *
 *  @{
 */
/*****************************************************************************/

/** Start a MXOS bluetooth LE pairing procedure
 *
 *
 * @param address 	: The remote device address
 * @param type 		: The remote device address type
 * @param settings 	: Security settings used in pairing procedure
 *
 * @return 	MXOS_BT_PENDING if successfully initiated,
 *          MXOS_BT_SUCCESS if already paired to the device, else
 *          error code
 */
OSStatus mxos_bt_start_pairing( mxos_bt_device_address_t address, mxos_bt_smart_address_type_t type, const mxos_bt_smart_security_settings_t* settings );

/** Stop a MXOS bluetooth LE pairing procedure
 *
 * @param address 	: The remote device address
 *
 * @return 	 MXOS_BT_PENDING if cancel initiated,
 *           MXOS_BT_SUCCESS if cancel has completed already, else error code.
 */
OSStatus mxos_bt_stop_pairing( mxos_bt_device_address_t address );

/** Satrt a MXOS bluetooth LE encryption procedure
 *
 * @param address 	: The remote device address
 *
 * @return 	 MXOS_BT_SUCCESS            : already encrypted
 *           MXOS_BT_PENDING            : command will be returned in the callback
 *           MXOS_BT_WRONG_MODE         : connection not up.
 *           MXOS_BT_BUSY               : security procedures are currently active
 */
OSStatus mxos_bt_start_encryption( mxos_bt_device_address_t* address );

/** @} */

/*****************************************************************************/
/** @addtogroup btpktmgmt  Packet
 *  @ingroup mxosbt
 *
 *  Bluetooth Packet Management Functions
 *
 *
 *  @{
 */
/*****************************************************************************/

/** Delete a MXOS Bluetooth packet
 *
 * This function returns the packet's memory space back to the source, allowing
 * for reuse.
 *
 * @param packet : The pointer to the packet to delete

 * @return    MXOS_SUCCESS : on success;
 *            MXOS_BADARG  : if bad argument(s) are inserted;
 *            MXOS_ERROR   : if an error occurred.
 */
OSStatus mxos_bt_packet_delete( mxos_bt_packet_t* packet );

/** Get a pointer to the packet data
 *
 * This function retrieves a pointer to the start of the data section in the
 * packet. It also returns the current data size and the remaining data space
 * in the packet.
 *
 * @param packet            : The pointer to the packet
 * @param data              : A pointer that will receive the pointer to the
 *                            start of the data section in the packet
 * @param current_data_size : A pointer that will receive the size of the data
 *                            in the packet in bytes
 * @param available_space   : A pointer that will receive the available data
 *                            space in the packet in bytes
 *
 * @return    MXOS_SUCCESS : on success;
 *            MXOS_BADARG  : if bad argument(s) are inserted;
 *            MXOS_ERROR   : if an error occurred.
 */
OSStatus mxos_bt_packet_get_data( const mxos_bt_packet_t* packet, uint8_t** data, uint32_t* current_data_size, uint32_t* available_space );

/** Set the end of the packet data
 *
 * This function updates the end of the data section and the data size in the
 * packet.
 *
 * @param packet   : The pointer to the packet
 * @param data_end : The pointer to the end of the data section in the packet

 * @return    MXOS_SUCCESS : on success;
 *            MXOS_BADARG  : if bad argument(s) are inserted;
 *            MXOS_ERROR   : if an error occurred
 */
OSStatus mxos_bt_packet_set_data_end( mxos_bt_packet_t* packet, const uint8_t* data_end );

/** @} */


/*****************************************************************************/
/** @addtogroup sbwhitelist SmartBridge Whitelist Filter
 *  @ingroup smartbridge
 *
 *  SmartBridge Whitelist Filter Functions
 *
 *
 *  @{
 */
/*****************************************************************************/


/** Get the maximum number of devices supported by the whitelist
 *
 * @note
 * This function retrieves the maximum number of Bluetooth Smart devices which can
 * be added to the whitelist.
 *
 * @param[out]  size : device count
 *
 * @return @ref OSStatus
 */
OSStatus mxos_bt_get_whitelist_capability( uint8_t* size );


/** Clear the whitelist
 *
 * @note
 * This function instructs the Bluetooth Controller to remove all devices from the
 * whitelist
 *
 * @return @ref OSStatus
 */
OSStatus mxos_bt_clear_whitelist( void );

/** @} */

#ifdef __cplusplus
} /* extern "C" */
#endif
