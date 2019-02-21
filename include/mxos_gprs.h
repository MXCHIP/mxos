/**
 ******************************************************************************
 * @file    mxos_gprs.h
 * @author  Snow Yang
 * @version V1.0.0
 * @date    2018-01-26
 * @brief   This file provides all the headers of wlan connectivity functions.
 ******************************************************************************
 *
 *  The MIT License
 *  Copyright (c) 2018 MXCHIP Inc.
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

#ifndef __MXOSGPRS_H__
#define __MXOSGPRS_H__

#include "mxos_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct 
{
  char    ip[16];     /**< Local IP address on the target wlan interface: @ref wlanInterfaceTypedef.*/
  char    gate[16];   /**< Router IP address on the target wlan interface: @ref wlanInterfaceTypedef.*/
  char    mask[16];   /**< Netmask on the target wlan interface: @ref wlanInterfaceTypedef.*/
} mxos_gprs_net_addr_t;

void mxos_gprs_status_handler(notify_netif_status_t status, const mxos_gprs_net_addr_t *net_addr);

OSStatus mxos_gprs_open(void);
OSStatus mxos_gprs_close(void);

#ifdef __cplusplus
    }
#endif

#endif //__MXOSGPRS_H__

/**
  * @}
  */

/**
  * @}
  */



