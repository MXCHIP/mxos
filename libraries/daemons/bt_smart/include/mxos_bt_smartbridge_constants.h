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
 *  Defines common constants and types for the MXOS Bluetooth Framework
 */

#pragma once

#include "mxos_bt_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                     Macros
 ******************************************************/


/******************************************************
 *                    Constants
 ******************************************************/

/** @cond !ADDTHIS*/
#define MXOS_BT_ADDRESS_BYTE_SIZE 6
/** @endcond */

/******************************************************
 *                   Enumerations
 ******************************************************/

/**
 * UUID size
 */
typedef enum
{
    UUID_16BIT  = 0x02, /**< 16-bit  */
    UUID_32BIT  = 0x04, /**< 32-bit  */
    UUID_128BIT = 0x10  /**< 128-bit */
} mxos_bt_uuid_size_t;


/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

#if 0
/**
 * Bluetooth Device Address (BD_ADDR)
 * A 48-bit address that uniquely identifies a Bluetooth device
 */
typedef struct
{
    uint8_t address[MXOS_BT_ADDRESS_BYTE_SIZE]; /**< Address. 48-bit Bluetooth device address in a little-endian format */
} mxos_bt_device_address_t;


/**
 * Universally Unique Identifier (UUID)
 * A standardised format of string ID that uniquely identifies a Bluetooth service
 */
typedef struct
{
    union
    {
        uint16_t value_16_bit;                  /**< 16-bit UUID value.     */
        uint16_t value_128_bit[UUID_128BIT/2];  /**< 128-bit UUID value.    */
    } value;                                    /**< A union of UUID values */

    mxos_bt_uuid_size_t size;                  /**< UUID size              */

} mxos_bt_uuid_t;
#endif

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

#ifdef __cplusplus
} /* extern "C" */
#endif
