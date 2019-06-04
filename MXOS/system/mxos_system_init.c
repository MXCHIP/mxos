/**
 ******************************************************************************
 * @file    mxos_system_init.c
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This file provide the mxos system initialize function.
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

#include <time.h>

#include "mxos.h"

#include "mkv.h"
#include "system_internal.h"

#if MXOS_WLAN_FORCE_OTA_ENABLE
#include "tftp_ota.h"
#endif

#ifndef  EasyLink_Needs_Reboot
static mos_worker_thread_id_t wlan_autoconf_worker_thread;
#endif

extern system_context_t* sys_context;


/******************************************************
 *               Variables Definitions
 ******************************************************/

static merr_t system_config_mode_worker( void *arg )
{
    merr_t err = kNoErr;
    mxos_Context_t* in_context = mxos_system_context_get();
    require( in_context, exit );

    mwifi_on();
#if ( WIFI_CONFIG_MODE == WIFI_CONFIG_MODE_SOFTAP)
    err = mxos_easylink_softap( in_context, MXOS_TRUE );
#elif ( WIFI_CONFIG_MODE == WIFI_CONFIG_MODE_MONITOR)
    err = mxos_easylink_monitor( in_context, MXOS_TRUE );
#elif ( WIFI_CONFIG_MODE == WIFI_CONFIG_MODE_WAC)
    err = mxos_easylink_wac( in_context, MXOS_TRUE );
#elif ( WIFI_CONFIG_MODE == WIFI_CONFIG_MODE_AWS)
    err = mxos_easylink_aws( in_context, MXOS_TRUE );
#elif ( WIFI_CONFIG_MODE == WIFI_CONFIG_MODE_NONE)
#else
    #error "Wi-Fi configuration mode is not defined"
#endif
    require_noerr( err, exit );
exit:
    return err;
}

merr_t mxos_system_wlan_start_autoconf( void )
{
  /* Enter auto-conf mode only once in reboot mode, use MOS_NETWORKING_WORKER_THREAD to save ram */
#ifdef  EasyLink_Needs_Reboot
    return mos_worker_send_async_event( MOS_NETWORKING_WORKER_THREAD, system_config_mode_worker, NULL );
#else
    return mos_worker_send_async_event( &wlan_autoconf_worker_thread, system_config_mode_worker, NULL );
#endif
}

merr_t mxos_system_init( void )
{
  merr_t err = kNoErr;

  /* Create mxos system context */
  system_config_t* mxos_context = system_context_init();
  require_action( mxos_context, exit, err = kNoMemoryErr );

  /* Initialize mxos notify system */
  err = system_notification_init( sys_context );
  require_noerr( err, exit ); 

#if MXOS_SYSTEM_MONITOR_ENABLE
  /* MXOS system monitor */
  err = mxos_system_monitor_daemen_start( );
  require_noerr( err, exit ); 
#endif


#if MXOS_CONFIG_EASYLINK_BTN_ENABLE
  system_easylink_btn_init( EasyLink_BUTTON, MXOS_CONFIG_EASYLINK_BTN_LONG_PRESS_TIMEOUT );
#endif

#if MXOS_CLI_ENABLE
  /* MXOS command line interface */
  cli_init();
#endif

  /* Network PHY driver and tcp/ip stack init */
  err = system_network_daemen_start( sys_context );
  require_noerr( err, exit ); 

#if MXOS_WLAN_CONNECTION_ENABLE

#if MXOS_WLAN_FORCE_OTA_ENABLE
	err = start_forceota_check();
	require_noerr( err, exit );
#endif

#ifndef  EasyLink_Needs_Reboot
  /* Create a worker thread for user handling wlan auto-conf event, this worker thread only has
     one event on queue, avoid some unwanted operation */
  err = mos_worker_thread_new( &wlan_autoconf_worker_thread, MOS_APPLICATION_PRIORITY, 0x500, 1 );
  require_noerr_string( err, exit, "ERROR: Unable to start the autoconf worker thread." );
#endif

  if( sys_context->flashContentInRam.mxos_config.configured == unConfigured){
#if MXOS_WLAN_AUTO_CONFIG
    system_log("Empty configuration. Starting configuration mode...");
    err = mxos_system_wlan_start_autoconf( );
    require_noerr( err, exit );
#endif
  }
#ifdef EasyLink_Needs_Reboot
  else if( sys_context->flashContentInRam.mxos_config.configured == wLanUnConfigured ){
      system_log("Re-config wlan configuration. Starting configuration mode...");
      err = mxos_system_wlan_start_autoconf( );
      require_noerr( err, exit );
  }
#endif

#ifdef MFG_MODE_AUTO
  else if( sys_context->flashContentInRam.mxos_config.configured == mfgConfigured ){
    system_log( "Enter MFG mode automatically" );
    mxos_mfg_test( in_context );
    mos_msleep( MOS_NEVER_TIMEOUT );
  }
#endif
  else{
    system_log("Available configuration. Starting Wi-Fi connection...");
    system_connect_wifi_fast( sys_context );
#if MXOS_WLAN_AUTO_SOFTAP_WHEN_DISCONNECTED
    mxos_station_status_monitor("testsoftap", "", 30);
#endif
  }
#endif
  
  /* System discovery */
#if MXOS_SYSTEM_DISCOVERY_ENABLE
  system_discovery_init( sys_context );
#endif

  /*Local configuration server*/
#if MXOS_CONFIG_SERVER_ENABLE
  config_server_start( );
#endif
  
#ifdef AIRKISS_DISCOVERY_ENABLE
  err = airkiss_discovery_start( AIRKISS_APP_ID, AIRKISS_DEVICE_ID );
  require_noerr( err, exit );
#endif

exit:
  return err;
}


