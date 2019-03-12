/**
 ******************************************************************************
 * @file    system.h
 * @author  William Xu
 * @version V1.0.0
 * @date    22-July-2015
 * @brief   This file provide internal function prototypes for for mxos system.
 ******************************************************************************
 *
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#ifndef _SYSTEM_INTERNAL_H_
#define _SYSTEM_INTERNAL_H_


#include "system.h"

#ifdef __cplusplus
extern "C" {
#endif


#define system_log(M, ...)       MXOS_LOG(CONFIG_SYSTEM_DEBUG, "SYSTEM", M, ##__VA_ARGS__)

/* Define MXOS service thread stack size */
#define STACK_SIZE_LOCAL_CONFIG_SERVER_THREAD   0x400
#define STACK_SIZE_LOCAL_CONFIG_CLIENT_THREAD   0x1650
#define STACK_SIZE_NTP_CLIENT_THREAD            0x450
#define STACK_SIZE_mxos_system_MONITOR_THREAD   0x300

#define EASYLINK_BYPASS_NO                      (0)
#define EASYLINK_BYPASS                         (1)
#define EASYLINK_SOFT_AP_BYPASS                 (2)



#define SYS_MAGIC_NUMBR     (0xA43E2165)

typedef struct _mxos_Context_t
{
  /*Flash content*/
  system_config_t           flashContentInRam;
  mxos_mutex_t              flashContentInRam_mutex;

  void *                    user_config_data;
  uint32_t                  user_config_data_size;

  /*Running status*/
  system_status_wlan_t      mxosStatus;
} system_context_t;

typedef void (*config_server_uap_configured_cb) (uint32_t id);

merr_t system_notification_init( system_context_t * const inContext);

merr_t system_network_daemen_start( system_context_t * const inContext );

merr_t system_discovery_init( system_context_t * const inContext );

void system_connect_wifi_normal( system_context_t * const inContext );

void system_connect_wifi_fast( system_context_t * const inContext);

void system_easylink_btn_init( int8_t btn, uint32_t long_pressed_timeout );

merr_t MXOSRestoreMFG                 ( void );

merr_t MXOSReadConfiguration          ( system_context_t * const inContext );

system_context_t *system_context( void );
void config_server_set_uap_cb( config_server_uap_configured_cb callback );


int mxos_station_status_monitor(char *ssid, char*key, int trigger_seconds);

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif

