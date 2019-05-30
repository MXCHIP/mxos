/**
 ******************************************************************************
 * @file    system_easylink_softap.c
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
#include "StringUtils.h"
#include "HTTPUtils.h"
#include "SocketUtils.h"

#include "system.h"
#include "easylink_internal.h"

int SetTimer(unsigned long ms, void (*psysTimerHandler)(void));
int SetTimer_uniq(unsigned long ms, void (*psysTimerHandler)(void));
int UnSetTimer(void (*psysTimerHandler)(void));

/* Internal vars and functions */
static mos_semphr_id_t easylink_sem;         /**< Used to suspend thread while easylink. */
static mos_semphr_id_t easylink_connect_sem; /**< Used to suspend thread while connection. */
static bool easylink_success = false;         /**< true: connect to wlan, false: start soft ap mode or roll back to previous settings */
static uint32_t easylinkIndentifier = 0;      /**< Unique for an easylink instance. */
static mos_thread_id_t easylink_softap_thread_handler = NULL;
static bool easylink_thread_force_exit = false;

/* Perform easylink and connect to wlan */
static void easylink_softap_thread( void *inContext );

/* MXOS callback when WiFi status is changed */
static void easylink_wifi_status_cb( WiFiEvent event, system_context_t * const inContext )
{
    switch ( event )
    {
        case NOTIFY_STATION_UP:
            /* Connected to AP, means that the wlan configuration is right, update configuration in flash and update
             bongjour txt record with new "easylinkIndentifier" */
            easylink_bonjour_update( Station, easylinkIndentifier, inContext );
            inContext->flashContentInRam.mxos_config.configured = allConfigured;
            mxos_system_context_update( &inContext->flashContentInRam ); //Update Flash content
            mos_semphr_release(easylink_connect_sem ); //Notify Easylink thread
            break;
        case NOTIFY_AP_DOWN:
            /* Remove bonjour service under soft ap interface */
            easylink_remove_bonjour( INTERFACE_UAP );
            break;
        default:
            break;
    }
}

void easylink_uap_configured_cd(uint32_t id)
{
    easylinkIndentifier = id;
    easylink_success = true;
    mwifi_softap_stop();
    mos_semphr_release(easylink_sem );
}

static void easylink_remove_bonjour_from_uap(void)
{
    easylink_remove_bonjour(INTERFACE_UAP);
}

void easylink_softap_thread( void *inContext )
{
    merr_t err = kNoErr;
    system_context_t *context = (system_context_t *) inContext;
    mwifi_softap_attr_t wNetConfig;

    easylinkIndentifier = 0x0;
    easylink_success = false;

    easylink_thread_force_exit = false;

    mxos_system_notify_register( mxos_notify_WIFI_STATUS_CHANGED, (void *) easylink_wifi_status_cb, (void *) inContext );

    easylink_sem = mos_semphr_new( 1 );
    easylink_connect_sem = mos_semphr_new( 1 );

restart:
    mxosWlanSuspend( );
    mos_sleep_ms( 20 );

    mxos_system_delegate_config_will_start( );

    memset( &wNetConfig, 0, sizeof(mwifi_softap_attr_t) );
    snprintf( wNetConfig.wifi_ssid, 32, "EasyLink_%c%c%c%c%c%c",
              context->mxosStatus.mac[9], context->mxosStatus.mac[10], context->mxosStatus.mac[12],
              context->mxosStatus.mac[13], context->mxosStatus.mac[15], context->mxosStatus.mac[16] );
    strcpy( (char*) wNetConfig.wifi_key, "" );
    strcpy( (char*) wNetConfig.local_ip_addr, "10.10.10.1" );
    strcpy( (char*) wNetConfig.net_mask, "255.255.255.0" );
    strcpy( (char*) wNetConfig.gateway_ip_addr, "10.10.10.1" );
    mwifi_softap_start( &wNetConfig );
    system_log("Establish soft ap: %s.....", wNetConfig.wifi_ssid);

    /* Start bonjour service for device discovery under soft ap mode */
    err = easylink_bonjour_start( Soft_AP, 0, context );
    require_noerr( err, exit );

    while( mos_semphr_acquire(easylink_sem, 0 ) == kNoErr );
    err = mos_semphr_acquire(easylink_sem, MOS_WAIT_FOREVER );

    mwifi_softap_stop();

    /* Easylink force exit by user, clean and exit */
    if( err != kNoErr && easylink_thread_force_exit )
    {
        system_log("EasyLink canceled by user");
        goto exit;
    }

    /* EasyLink Success */
    if ( easylink_success == true ) {
        mxos_system_delegate_config_recv_ssid( context->flashContentInRam.mxos_config.ssid,
                                               context->flashContentInRam.mxos_config.user_key );

        mos_sleep_ms(1);
        system_connect_wifi_normal( context );

        /* Wait for station connection */
        while ( mos_semphr_acquire(easylink_connect_sem, 0 ) == kNoErr );
        err = mos_semphr_acquire(easylink_connect_sem, EasyLink_ConnectWlan_Timeout );
        /* Easylink force exit by user, clean and exit */
        if ( err != kNoErr && easylink_thread_force_exit )
        {
            mxosWlanSuspend( );
            system_log("EasyLink connection canceled by user");
            goto exit;
        }

        /*SSID or Password is not correct, module cannot connect to wlan, so restart EasyLink again*/
        require_noerr_action_string( err, restart, mxosWlanSuspend(), "Re-start easylink softap mode" );
        mxos_system_delegate_config_success( CONFIG_BY_SOFT_AP );

        /* Start bonjour service for new device discovery */
        err = easylink_bonjour_start( Station, easylinkIndentifier, context );
        require_noerr( err, exit );
        SetTimer( 60 * 1000, easylink_remove_bonjour_from_uap );

        goto exit;
    }
    else /* EasyLink failed */
    {
        /*so roll back to previous settings  (if it has) and connect*/
        if(context->flashContentInRam.mxos_config.configured != unConfigured)
        {
            system_log("Roll back to previous settings");
            MXOSReadConfiguration( context );
#ifdef EasyLink_Needs_Reboot
            context->flashContentInRam.mxos_config.configured = allConfigured;
            mxos_system_context_update( &context->flashContentInRam );
#endif
            system_connect_wifi_normal( context );
        }
        else {
            /*module should power down in default setting*/
            system_log("Wi-Fi power off");
            mwifi_off( );
        }

    }

exit:
    easylink_thread_force_exit = false;

    mxos_system_delegate_config_will_stop( );

    mxos_system_notify_remove( mxos_notify_WIFI_STATUS_CHANGED, (void *) easylink_wifi_status_cb );

#if MXOS_CONFIG_SERVER_ENABLE
    config_server_stop( );
#endif 

    mos_semphr_delete(easylink_sem );
    mos_semphr_delete(easylink_connect_sem );
    easylink_softap_thread_handler = NULL;
    mos_thread_delete( NULL );
}

merr_t mxos_easylink_softap( mxos_Context_t * const in_context, mxos_bool_t enable )
{
    merr_t err = kUnknownErr;

    require_action( in_context, exit, err = kNotPreparedErr );

    easylink_remove_bonjour( INTERFACE_UAP );

    /* easylink soft thread existed? stop! */
    if ( easylink_softap_thread_handler ) {
        system_log("EasyLink SoftAP processing, force stop..");
        easylink_thread_force_exit = true;
        mos_thread_awake(easylink_softap_thread_handler );
        mos_thread_join( easylink_softap_thread_handler );
    }

    if ( enable == MXOS_TRUE ) {
        /* Start config server */
        err = config_server_start( );
        require_noerr( err, exit );

        config_server_set_uap_cb( easylink_uap_configured_cd );

        easylink_softap_thread_handler = mos_thread_new( MOS_APPLICATION_PRIORITY, "EASYLINK AP",
                                       easylink_softap_thread, 0x1000, (void *) in_context );
        require_action_string( easylink_softap_thread_handler != NULL, exit, err = kGeneralErr, "ERROR: Unable to start the EasyLink thread." );

        /* Make sure easylink softap is already running, and waiting for sem trigger */
        mos_sleep_ms( 1000 );
    }

    exit:
    return err;
}

//#endif

