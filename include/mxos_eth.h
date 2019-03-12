/**
 ******************************************************************************
 * @file    mxos_eth.h
 * @author  William Xu
 * @version V1.0.0
 * @date    7-Nov-2017
 * @brief   This file provides all the headers of ethernet connectivity functions.
 ******************************************************************************
 *
 *  The MIT License
 *  Copyright (c) 2017 MXCHIP Inc.
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

#ifndef __MXOS_ETH_H__
#define __MXOS_ETH_H__


#include "mxos_opt.h"

#include "mxos_common.h"
#include "mxos_network.h"



#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief  Check TCPIP stack is initialized or not, TCPIP can be initialized by mxos_init()
 *
 * @param  None
 *
 * @return MXOS_TRUE: Initialized, MXOS_FALSE: Uninitialized
 */
mxos_bool_t  mxos_tcpip_stack_is_inited(void);

/**
 * @brief  Get mac address string on ethernet interface.
 *
 * @return MAC address string, like xx:xx:xx:xx:xx:xx
 */
const char *mxos_eth_get_mac_address(void);

/**
 *  @brief  Enable ethernet interface, according to provided ip addesss
 *
 *  @param  dhcp: enable DHCP client or not
 *  @param  ip: Static IP configuration, Local IP address
 *  @param  netmask: Static IP configuration, netmask
 *  @param  char: Static IP configuration, gateway ip address
 *
 *  @return kNoErr
 */
mret_t mxos_eth_bringup(bool dhcp, const char *ip, const char *netmask, const char *gw);

/**
 *  @brief  Power on ethernet mac and PHY
 *
 *  @return none
 */
void mxos_eth_power_up(void);


/**
 *  @brief  Set ethernet as the default network interface
 *
 *  @return none
 */
void mxos_eth_set_default_interface(void);


#ifdef __cplusplus
    }
#endif

#endif //__MXOS_ETH_H__

/**
  * @}
  */

/**
  * @}
  */



