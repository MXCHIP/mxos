/**
 ******************************************************************************
 * @file    mxos_lwip_ethif_logging.h
 * @author  William Xu
 * @version V1.0.0
 * @date    3-Nov-2017
 * @brief   This file provide the lwip ethernet layer logging
 ******************************************************************************
 *
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2017 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#ifndef MXOS_LWIP_ETHIF_LOGGING_H_
#define MXOS_LWIP_ETHIF_LOGGING_H_

#include "mxos_opt.h"
#include "mxos_debug.h"

#ifdef __cplusplus
extern "C" {
#endif

#define eth_log(M, ...) MXOS_LOG(CONFIG_ETH_DEBUG, "ETH", M, ##__VA_ARGS__)

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif /* ifndef MXOS_LWIP_ETHIF_LOGGING_H_ */
