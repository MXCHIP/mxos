/**
 ******************************************************************************
 * @file    MXOSBonjour.c
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   Zero-configuration protocol compatible with Bonjour from Apple
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#include "mxos.h"
#include "system_internal.h"

#include "mxos_board_conf.h"
#include "StringUtils.h"
#include "mdns.h"

#define SYS_TXT_LEN  256

static struct mdns_service mxos_system_service;
static char keyvals[SYS_TXT_LEN];

OSStatus system_discovery_init(system_context_t * const inContext)
{
    char *val;

    memset(&mxos_system_service, 0x0, sizeof(struct mdns_service));
    mxos_system_service.servname = inContext->flashContentInRam.mxosSystemConfig.name;
    mxos_system_service.servtype = "easylink";
    mxos_system_service.port = MXOS_SYSTEM_DISCOVERY_PORT;
    mxos_system_service.proto = MDNS_PROTO_TCP;

    val = __strdup_trans_dot(inContext->mxosStatus.mac);
    snprintf(keyvals, SYS_TXT_LEN, "MAC=%s.", val);
    free(val);

    val = __strdup_trans_dot(FIRMWARE_REVISION);
    snprintf(keyvals, SYS_TXT_LEN, "%sFirmware Rev=%s.", keyvals, val);
    free(val);

    val = __strdup_trans_dot(HARDWARE_REVISION);
    snprintf(keyvals, SYS_TXT_LEN, "%sHardware Rev=%s.", keyvals, val);
    free(val);

    val = __strdup_trans_dot(MxosGetVer());
    snprintf(keyvals, SYS_TXT_LEN, "%sMXOS OS Rev=%s.", keyvals, val);
    free(val);

    val = __strdup_trans_dot(MODEL);
    snprintf(keyvals, SYS_TXT_LEN, "%sModel=%s.", keyvals, val);
    free(val);

    val = __strdup_trans_dot(PROTOCOL);
    snprintf(keyvals, SYS_TXT_LEN, "%sProtocol=%s.", keyvals, val);
    free(val);

    val = __strdup_trans_dot(MANUFACTURER);
    snprintf(keyvals, SYS_TXT_LEN, "%sManufacturer=%s.", keyvals, val);
    free(val);

    snprintf(keyvals, SYS_TXT_LEN, "%sSeed=%lu.", keyvals, inContext->flashContentInRam.mxosSystemConfig.seed);

    mdns_set_txt_rec(&mxos_system_service, keyvals, '.');

    mdns_start(NULL, DEFAULT_NAME);
    mdns_announce_service(&mxos_system_service, INTERFACE_STA);
    mdns_announce_service(&mxos_system_service, INTERFACE_UAP);
#if PLATFORM_ETH_ENABLE    
    mdns_announce_service(&mxos_system_service, INTERFACE_ETH);
#endif
    return kNoErr;
}
