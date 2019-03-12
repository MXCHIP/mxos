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

/* Internal vars and functions */
static mxos_semaphore_t easylink_sem;         /**< Used to suspend thread while easylink. */
static mxos_semaphore_t easylink_connect_sem; /**< Used to suspend thread while connection. */
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
            inContext->flashContentInRam.mxosSystemConfig.configured = allConfigured;
            mxos_system_context_update( &inContext->flashContentInRam ); //Update Flash content
            mxos_rtos_set_semaphore( &easylink_connect_sem ); //Notify Easylink thread
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
    mxosWlanSuspendSoftAP();
    mxos_rtos_set_semaphore( &easylink_sem );
}

static void easylink_remove_bonjour_from_uap(void)
{
    easylink_remove_bonjour(INTERFACE_UAP);
}

void easylink_softap_thread( void *inContext )
{
    mret_t err = kNoErr;
    system_context_t *context = (system_context_t *) inContext;
    network_InitTypeDef_st wNetConfig;

    easylinkIndentifier = 0x0;
    easylink_success = false;

    easylink_thread_force_exit = false;

    mxos_system_notify_register( mxos_notify_WIFI_STATUS_CHANGED, (void *) easylink_wifi_status_cb, (void *) inContext );

    mxos_rtos_init_semaphore( &easylink_sem, 1 );
    mxos_rtos_init_semaphore( &easylink_connect_sem, 1 );

restart:
    mxosWlanSuspend( );
    mxos_thread_msleep( 20 );

    mxos_system_delegate_config_will_start( );

    memset( &wNetConfig, 0, sizeof(network_InitTypeDef_st) );
    wNetConfig.wifi_mode = Soft_AP;
    snprintf( wNetConfig.wifi_ssid, 32, "EasyLink_%c%c%c%c%c%c",
              context->mxosStatus.mac[9], context->mxosStatus.mac[10], context->mxosStatus.mac[12],
              context->mxosStatus.mac[13], context->mxosStatus.mac[15], context->mxosStatus.mac[16] );
    strcpy( (char*) wNetConfig.wifi_key, "" );
    strcpy( (char*) wNetConfig.local_ip_addr, "10.10.10.1" );
    strcpy( (char*) wNetConfig.net_mask, "255.255.255.0" );
    strcpy( (char*) wNetConfig.gateway_ip_addr, "10.10.10.1" );
    wNetConfig.dhcpMode = DHCP_Server;
    mxosWlanStart( &wNetConfig );
    system_log("Establish soft ap: %s.....", wNetConfig.wifi_ssid);

    /* Start bonjour service for device discovery under soft ap mode */
    err = easylink_bonjour_start( Soft_AP, 0, context );
    require_noerr( err, exit );

    while( mxos_rtos_get_semaphore( &easylink_sem, 0 ) == kNoErr );
    err = mxos_rtos_get_semaphore( &easylink_sem, MXOS_WAIT_FOREVER );

    mxosWlanSuspendSoftAP();

    /* Easylink force exit by user, clean and exit */
    if( err != kNoErr && easylink_thread_force_exit )
    {
        system_log("EasyLink canceled by user");
        goto exit;
    }

    /* EasyLink Success */
    if ( easylink_success == true ) {
        mxos_system_delegate_config_recv_ssid( context->flashContentInRam.mxosSystemConfig.ssid,
                                               context->flashContentInRam.mxosSystemConfig.user_key );

        mxos_thread_sleep(1);
        system_connect_wifi_normal( context );

        /* Wait for station connection */
        while ( mxos_rtos_get_semaphore( &easylink_connect_sem, 0 ) == kNoErr );
        err = mxos_rtos_get_semaphore( &easylink_connect_sem, EasyLink_ConnectWlan_Timeout );
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
        if(context->flashContentInRam.mxosSystemConfig.configured != unConfigured)
        {
            system_log("Roll back to previous settings");
            MXOSReadConfiguration( context );
#ifdef EasyLink_Needs_Reboot
            context->flashContentInRam.mxosSystemConfig.configured = allConfigured;
            mxos_system_context_update( &context->flashContentInRam );
#endif
            system_connect_wifi_normal( context );
        }
        else {
            /*module should power down in default setting*/
            system_log("Wi-Fi power off");
            mxosWlanPowerOff( );
        }

    }

exit:
    easylink_thread_force_exit = false;

    mxos_system_delegate_config_will_stop( );

    mxos_system_notify_remove( mxos_notify_WIFI_STATUS_CHANGED, (void *) easylink_wifi_status_cb );

#if MXOS_CONFIG_SERVER_ENABLE
    config_server_stop( );
#endif 

    mxos_rtos_deinit_semaphore( &easylink_sem );
    mxos_rtos_deinit_semaphore( &easylink_connect_sem );
    easylink_softap_thread_handler = NULL;
    mos_thread_delete( NULL );
}

mret_t mxos_easylink_softap( mxos_Context_t * const in_context, mxos_bool_t enable )
{
    mret_t err = kUnknownErr;

    require_action( in_context, exit, err = kNotPreparedErr );

    easylink_remove_bonjour( INTERFACE_UAP );

    /* easylink soft thread existed? stop! */
    if ( easylink_softap_thread_handler ) {
        system_log("EasyLink SoftAP processing, force stop..");
        easylink_thread_force_exit = true;
        mxos_rtos_thread_force_awake( &easylink_softap_thread_handler );
        mos_thread_join( easylink_softap_thread_handler );
    }

    if ( enable == MXOS_TRUE ) {
        /* Start config server */
        err = config_server_start( );
        require_noerr( err, exit );

        config_server_set_uap_cb( easylink_uap_configured_cd );

        easylink_softap_thread_handler = mos_thread_new( MXOS_APPLICATION_PRIORITY, "EASYLINK AP",
                                       easylink_softap_thread, 0x1000, (void *) in_context );
        require_action_string( easylink_softap_thread_handler != NULL, exit, err = kGeneralErr, "ERROR: Unable to start the EasyLink thread." );

        /* Make sure easylink softap is already running, and waiting for sem trigger */
        mos_thread_delay( 1000 );
    }

    exit:
    return err;
}

//#endif

