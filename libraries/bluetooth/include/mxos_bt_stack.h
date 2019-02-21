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
 * Bluetooth Management (BTM) Application Programming Interface
 *
 * The BTM consists of several management entities:
 *      1. Device Control - controls the local device
 *      2. Device Discovery - manages inquiries, discover database
 *      3. ACL Channels - manages ACL connections (BR/EDR and LE)
 *      4. SCO Channels - manages SCO connections
 *      5. Security - manages all security functionality
 *      6. Power Management - manages park, sniff, hold, etc.
 *
 * @defgroup mxosbt      Bluetooth
 *
 * MXOS Bluetooth Framework Functions
 */

#pragma once

#include "mxos_bt_dev.h"
#include "mxos_bt_cfg.h"

/******************************************************
 *               Function Declarations
 ******************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************/
/**
 * Framework Management Functions
 *
 * @addtogroup  mxosbt_Framework   Framework
 * @ingroup     mxosbt
 *
 * @{
 */
/****************************************************************************/

/**
 * Function         mxos_bt_stack_init
 *
 *                  Initialize the Bluetooth controller and stack; register
 *                  callback for Bluetooth event notification.
 *
 * @param[in] p_bt_management_cback     : Callback for receiving Bluetooth management events
 * @param[in] p_bt_cfg_settings         : Bluetooth stack configuration
 * @param[in] mxos_bt_cfg_buf_pools    : Buffer pool configuration
 *
 * @return    MXOS_BT_SUCCESS : on success;
 *            MXOS_BT_FAILED : if an error occurred
 */
mxos_bt_result_t mxos_bt_stack_init(mxos_bt_management_cback_t *p_bt_management_cback,
                                   const mxos_bt_cfg_settings_t     *p_bt_cfg_settings,
                                   const mxos_bt_cfg_buf_pool_t     mxos_bt_cfg_buf_pools[MXOS_BT_CFG_NUM_BUF_POOLS]);



/**
 * Function         mxos_bt_stack_deinit
 *
 *                  De-initialize the Bluetooth controller and stack.
 *
 * @return    MXOS_BT_SUCCESS : on success;
 *            MXOS_BT_ERROR   : if an error occurred
 */
OSStatus mxos_bt_stack_deinit(void);


/**@} mxosbt_Framework */


#ifdef __cplusplus
}
#endif


