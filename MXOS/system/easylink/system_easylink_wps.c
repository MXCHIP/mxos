/**
 ******************************************************************************
 * @file    system_easylink.c
 * @author  William Xu
 * @version V1.0.0
 * @date    20-July-2015
 * @brief   This file provide the easylink function for quick provisioning and
 *          first time configuration.
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

#include "mxos.h"

#include "system_internal.h"
#include "easylink_internal.h"

#include "StringUtils.h"


/******************************************************
 *               Function Declarations
 ******************************************************/
/* EasyLink event callback functions*/
static void easylink_wifi_status_cb( WiFiEvent event, system_context_t * const inContext );
static void easylink_complete_cb( network_InitTypeDef_st *nwkpara, system_context_t * const inContext );

/* Thread perform wps and connect to wlan */
static void easylink_wps_thread( void *inContext ); /* Perform easylink and connect to wlan */

/******************************************************
 *               Variables Definitions
 ******************************************************/
static mos_semphr_id_t easylink_sem;         /**< Used to suspend thread while easylink. */
static mos_semphr_id_t easylink_connect_sem; /**< Used to suspend thread while connection. */
static bool easylink_success = false;         /**< true: connect to wlan, false: start soft ap mode or roll back to previous settings */
static uint32_t easylinkIndentifier = 0;      /**< Unique for an easylink instance. */
static mos_thread_id_t easylink_wps_thread_handler = NULL;
static bool easylink_thread_force_exit = false;

static mxos_config_source_t source = CONFIG_BY_NONE;

static mxos_wps_device_detail_t wps_config =
{
    .device_name     = DEFAULT_NAME,
    .manufacturer    = MANUFACTURER,
    .model_name      = MODEL,
    .model_number    = HARDWARE_REVISION,
    .serial_number   = SERIAL_NUMBER,
    .device_category = MXOS_WPS_DEVICE_COMPUTER,
    .sub_category    = 07,//7,
    .config_methods  = WPS_CONFIG_PHYSICAL_PUSH_BUTTON,
};

/******************************************************
 *               Function Definitions
 ******************************************************/

/* MXOS callback when WiFi status is changed */
static void easylink_wifi_status_cb( WiFiEvent event, system_context_t * const inContext )
{
    switch ( event )
    {
        case NOTIFY_STATION_UP:
            inContext->flashContentInRam.mxosSystemConfig.configured = allConfigured;
            mxos_system_context_update( &inContext->flashContentInRam ); //Update Flash content
            mos_semphr_release(easylink_connect_sem ); //Notify Easylink thread
            break;
        default:
            break;
    }
    return;
}

/* MXOS callback when EasyLink is finished step 1, return SSID and KEY */
static void easylink_complete_cb( network_InitTypeDef_st *nwkpara, system_context_t * const inContext )
{
    merr_t err = kNoErr;

    require_action_string( nwkpara, exit, err = kTimeoutErr, "EasyLink Timeout or terminated" );

    /* Store SSID and KEY*/
    mos_mutex_lock(inContext->flashContentInRam_mutex );
    memset( inContext->flashContentInRam.mxosSystemConfig.bssid, 0x0, 6 );
    memset( inContext->flashContentInRam.mxosSystemConfig.ssid, 0x0, maxSsidLen );
    memset( inContext->flashContentInRam.mxosSystemConfig.user_key, 0x0, maxKeyLen );

    memcpy( inContext->flashContentInRam.mxosSystemConfig.ssid, nwkpara->wifi_ssid, nwkpara->local_ip_addr[0] );
    memcpy( inContext->flashContentInRam.mxosSystemConfig.user_key, nwkpara->wifi_key, nwkpara->local_ip_addr[1] );
    inContext->flashContentInRam.mxosSystemConfig.user_keyLength = nwkpara->local_ip_addr[1];
    inContext->flashContentInRam.mxosSystemConfig.dhcpEnable = true;
    mos_mutex_unlock(inContext->flashContentInRam_mutex );
    system_log("Get SSID: %s, Key: %s", inContext->flashContentInRam.mxosSystemConfig.ssid,
                                                    inContext->flashContentInRam.mxosSystemConfig.user_key);

    source = CONFIG_BY_WPS;

exit:
    if ( err != kNoErr )
    {
        /*EasyLink timeout or error*/
        easylink_success = false;
        mos_semphr_release(easylink_sem );
    } else {
        easylink_success = true;
        mos_semphr_release(easylink_sem );
    }

    return;
}


static void easylink_remove_bonjour_from_sta(void)
{
    easylink_remove_bonjour(INTERFACE_STA);
}

static void easylink_wps_thread( void *arg )
{
    merr_t err = kNoErr;
    system_context_t *context = (system_context_t *) arg;

    easylinkIndentifier = 0x0;
    easylink_success = false;
    easylink_thread_force_exit = false;

    source = CONFIG_BY_NONE;
    mxos_system_notify_register( mxos_notify_EASYLINK_WPS_COMPLETED,    (void *) easylink_complete_cb,      context );
    mxos_system_notify_register( mxos_notify_WIFI_STATUS_CHANGED,       (void *) easylink_wifi_status_cb,   context );

    easylink_sem = mos_semphr_new( 1 );
    easylink_connect_sem = mos_semphr_new( 1 );

restart:
    mxos_system_delegate_config_will_start( );
    system_log("Start easylink Wi-Fi protected setup mode(WPS) mode");
    mxos_wlan_start_wps(&wps_config, EasyLink_TimeOut / 1000);
    while( mos_semphr_acquire(easylink_sem, 0 ) == kNoErr );
    err = mos_semphr_acquire(easylink_sem, MXOS_WAIT_FOREVER );

    /* Easylink force exit by user, clean and exit */
    if( err != kNoErr && easylink_thread_force_exit )
    {
        system_log("EasyLink waiting for terminate");
        mxos_wlan_stop_wps( );
        mos_semphr_acquire(easylink_sem, 3000 );
        system_log("EasyLink canceled by user");
        goto exit;
    }

    /* EasyLink Success */
    if ( easylink_success == true )
    {
        mxos_system_delegate_config_recv_ssid( context->flashContentInRam.mxosSystemConfig.ssid,
                                               context->flashContentInRam.mxosSystemConfig.user_key );
        system_connect_wifi_normal( context );

        /* Wait for station connection */
        while( mos_semphr_acquire(easylink_connect_sem, 0 ) == kNoErr );
        err = mos_semphr_acquire(easylink_connect_sem, EasyLink_ConnectWlan_Timeout );
        /* Easylink force exit by user, clean and exit */
        if( err != kNoErr && easylink_thread_force_exit )
        {
            mxosWlanSuspend();
            system_log("EasyLink connection canceled by user");
            goto exit;
        }

        /*SSID or Password is not correct, module cannot connect to wlan, so restart EasyLink again*/
        require_noerr_action_string( err, restart, mxosWlanSuspend(), "Re-start easylink combo mode" );
        mxos_system_delegate_config_success( source );

        /* Start bonjour service for new device discovery */
        err = easylink_bonjour_start( Station, easylinkIndentifier, context );
        require_noerr( err, exit );
        SetTimer( 60 * 1000, easylink_remove_bonjour_from_sta );

        goto exit;
    }
    else /* EasyLink failed */
    {
        /* so roll back to previous settings  (if it has) and connect */
        if ( context->flashContentInRam.mxosSystemConfig.configured != unConfigured ) {
            system_log("Roll back to previous settings");
            MXOSReadConfiguration( context );
            system_connect_wifi_normal( context );
        }
        else {
            /*module should power down in default setting*/
            system_log("Wi-Fi power off");
            mwifi_off();
        }
    }

exit:
    easylink_thread_force_exit = false;

    mxos_system_delegate_config_will_stop( );

    mxos_system_notify_remove( mxos_notify_WIFI_STATUS_CHANGED, (void *)easylink_wifi_status_cb );
    mxos_system_notify_remove( mxos_notify_EASYLINK_WPS_COMPLETED, (void *)easylink_complete_cb );

    mos_semphr_delete(easylink_sem );
    mos_semphr_delete(easylink_connect_sem );
    easylink_wps_thread_handler = NULL;
    mos_thread_delete( NULL );
}

merr_t mxos_easylink_wps( mxos_Context_t * const in_context, mxos_bool_t enable )
{
    merr_t err = kUnknownErr;

    require_action( in_context, exit, err = kNotPreparedErr );

    easylink_remove_bonjour( INTERFACE_STA );

    /* easylink thread existed? stop! */
    if ( easylink_wps_thread_handler ) {
        system_log("WPS processing, force stop..");
        easylink_thread_force_exit = true;
        mxos_rtos_thread_force_awake( &easylink_wps_thread_handler );
        mos_thread_join( easylink_wps_thread_handler );
    }

    if ( enable == MXOS_TRUE ) {
        easylink_wps_thread_handler = mos_thread_new( MXOS_APPLICATION_PRIORITY, "WPS", easylink_wps_thread,
                                       0x1000, (void *) in_context );
        require_action_string( easylink_wps_thread_handler != NULL, exit, err = kGeneralErr, "ERROR: Unable to start the WPS thread." );

        /* Make sure easylink is already running, and waiting for sem trigger */
        mos_thread_delay( 100 );
    }

    exit:
    return err;
}



