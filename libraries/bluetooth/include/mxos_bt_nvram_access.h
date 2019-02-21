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
 *
 * MXOS Bluetooth Low Energy (BLE) Functions
 *
 */
#pragma once

#ifndef OFFSETOF
#define OFFSETOF( type, member )  ( (uintptr_t)&((type *)0)->member )
#endif /* OFFSETOF */

#define MXOS_BT_PARA_LOCAL_KEY_DATA  65  /* BTM_SECURITY_LOCAL_KEY_DATA_LEN */

#define MXOS_BT_DCT_NAME            249
#define MXOS_BT_DCT_MAX_KEYBLOBS    146   /* Maximum size of key blobs to be stored :=  size of BR-EDR link keys +  size of BLE keys*/
#define MXOS_BT_DCT_ADDR_FIELD      6
#define MXOS_BT_DCT_LENGTH_FIELD    2
#ifndef MXOS_BT_DCT_MAX_DEVICES
#define MXOS_BT_DCT_MAX_DEVICES     10    /* Maximum number of device records stored in nvram */
#endif
#define MXOS_BT_DCT_ADDR_TYPE       1
#define MXOS_BT_DCT_DEVICE_TYPE     1

/* Length of BD_ADDR + 2bytes length field */
#define MXOS_BT_DCT_ENTRY_HDR_LENGTH  (MXOS_BT_DCT_ADDR_FIELD + MXOS_BT_DCT_LENGTH_FIELD + MXOS_BT_DCT_ADDR_TYPE + MXOS_BT_DCT_DEVICE_TYPE)

#define MXOS_BT_DCT_LOCAL_KEY_OFFSET  OFFSETOF( mxos_bt_config_t, bluetooth_local_key )
#define MXOS_BT_DCT_REMOTE_KEY_OFFSET OFFSETOF( mxos_bt_config_t, bluetooth_remote_key )



#ifdef __cplusplus
extern "C" {
#endif



#ifdef __cplusplus
}
#endif
