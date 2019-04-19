/**
 ******************************************************************************
 * @file    system_easylink_monitor.c
 * @author  William Xu
 * @version V1.0.0
 * @date    20-May-2017
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

//#if (WIFI_CONFIG_MODE == WIFI_CONFIG_MODE_EASYLINK) || (WIFI_CONFIG_MODE == WIFI_CONFIG_MODE_EASYLINK_WITH_SOFTAP)

/******************************************************
 *               Function Declarations
 ******************************************************/
/* EasyLink event callback functions*/
static void easylink_wifi_status_cb( WiFiEvent event, system_context_t * const inContext );
static void easylink_complete_cb( char *ssid, char *key, int mode, system_context_t * const inContext );
static void easylink_extra_data_cb( int datalen, char* data, system_context_t * const inContext );

/* Thread perform easylink and connect to wlan */
static void easylink_monitor_thread( void *inContext ); /* Perform easylink and connect to wlan */

extern void mxos_wlan_monitor_no_easylink(void);

/******************************************************
 *               Variables Definitions
 ******************************************************/

static uint8_t wlan_channel = 1;
static mxos_bool_t wlan_channel_walker = MXOS_TRUE;
static uint32_t wlan_channel_walker_interval = 100;

static mos_semphr_id_t easylink_sem;         /**< Used to suspend thread while easylink. */
static mos_semphr_id_t easylink_connect_sem; /**< Used to suspend thread while connection. */
static bool easylink_success = false;         /**< true: connect to wlan, false: start soft ap mode or roll back to previous settings */
static uint32_t easylink_id = 0;      /**< Unique for an easylink instance. */
static mos_thread_id_t easylink_monitor_thread_handler = NULL;
static mos_thread_id_t switch_channel_thread_handler = NULL;
static bool easylink_thread_force_exit = false;
static bool switch_channel_flag = true;

static mxos_config_source_t source = CONFIG_BY_NONE;
static mxos_connect_fail_config_t connect_fail_config = EXIT_EASYLINK;

/******************************************************
 *               Function Definitions
 ******************************************************/

/* MXOS callback when WiFi status is changed */
static void easylink_wifi_status_cb( WiFiEvent event, system_context_t * const inContext )
{
    switch ( event )
    {
        case NOTIFY_STATION_UP:
            inContext->flashContentInRam.mxos_config.configured = allConfigured;
            mxos_system_context_update( &inContext->flashContentInRam ); //Update Flash content
            mos_semphr_release(easylink_connect_sem ); //Notify Easylink thread
            break;
        default:
            break;
    }
    return;
}

/* MXOS callback when EasyLink is finished step 1, return SSID and KEY */
static void easylink_complete_cb( char *ssid, char *key, int mode, system_context_t * const inContext )
{
    merr_t err = kNoErr;

    require_action_string( ssid, exit, err = kTimeoutErr, "EasyLink Timeout or terminated" );

    /* Store SSID and KEY*/
    mos_mutex_lock(inContext->flashContentInRam_mutex );
    memcpy( inContext->flashContentInRam.mxos_config.ssid, ssid, maxSsidLen );
    memset( inContext->flashContentInRam.mxos_config.bssid, 0x0, 6 );
    memcpy( inContext->flashContentInRam.mxos_config.user_key, key, maxKeyLen );
    inContext->flashContentInRam.mxos_config.user_keyLength = strlen( key );
    memcpy( inContext->flashContentInRam.mxos_config.key, key, maxKeyLen );
    inContext->flashContentInRam.mxos_config.keyLength = strlen( key );
    inContext->flashContentInRam.mxos_config.dhcpEnable = true;
    mos_mutex_unlock(inContext->flashContentInRam_mutex );
    system_log("Get SSID: %s, Key: %s", inContext->flashContentInRam.mxos_config.ssid, inContext->flashContentInRam.mxos_config.user_key);

    source = mode;
    exit:
    if ( err != kNoErr )
    {
        /*EasyLink timeout or error*/
        easylink_success = false;
        mos_semphr_release(easylink_sem );
    }
    return;
}

/* MXOS callback when EasyLink is finished step 2, return extra data 
 data format: [AuthData#Identifier]<localIp/netMask/gateWay/dnsServer>
 Auth data: Provide to application, application will decide if this is a proter configuration for currnet device
 Identifier: Unique id for every easylink instance send by easylink mobile app
 localIp/netMask/gateWay/dnsServer: Device static ip address, use DHCP if not exist
 */
static void easylink_extra_data_cb( int datalen, char* data, system_context_t * const inContext )
{
    merr_t err = kNoErr;
    int index;
    uint32_t *identifier, ipInfoCount;
    char *debugString;
    struct in_addr ipv4_addr;

    debugString = DataToHexStringWithSpaces( (const uint8_t *) data, datalen );
    system_log("Get user info: %s", debugString);
    free( debugString );

    /* Find '#' that separate authdata and identifier*/
    for ( index = datalen - 1; index >= 0; index-- )
    {
        if ( data[index] == '#' && ((datalen - index) == 5 || (datalen - index) == 25) )
            break;
    }
    require_action( index >= 0, exit, err = kParamErr );

    /* Check auth data by device */
    data[index++] = 0x0;
    err = mxos_system_delegate_config_recv_auth_data( data );
    require_noerr( err, exit );

    /* Read identifier */
    identifier = (uint32_t *) &data[index];
    easylink_id = *identifier;

    /* Identifier: 1 x uint32_t or Identifier/localIp/netMask/gateWay/dnsServer: 5 x uint32_t */
    ipInfoCount = (datalen - index) / sizeof(uint32_t);
    require_action( ipInfoCount >= 1, exit, err = kParamErr );

    mos_mutex_lock(inContext->flashContentInRam_mutex );

    if ( ipInfoCount == 1 )
    { //Use DHCP to obtain local ip address
        inContext->flashContentInRam.mxos_config.dhcpEnable = true;
        system_log("Get auth info: %s, EasyLink identifier: %lx", data, easylink_id);
    } else
    { //Use static ip address
        inContext->flashContentInRam.mxos_config.dhcpEnable = false;
        ipv4_addr.s_addr = *(identifier+1);
        strcpy( (char *) inContext->mxosStatus.localIp, inet_ntoa( ipv4_addr ) );
        ipv4_addr.s_addr = *(identifier+2);
        strcpy( (char *) inContext->mxosStatus.netMask, inet_ntoa( ipv4_addr ) );
        ipv4_addr.s_addr = *(identifier+3);
        strcpy( (char *) inContext->mxosStatus.gateWay, inet_ntoa( ipv4_addr ) );
        ipv4_addr.s_addr = *(identifier+4);
        strcpy( (char *) inContext->mxosStatus.dnsServer, inet_ntoa( ipv4_addr ) );

        system_log("Get auth info: %s, EasyLink identifier: %lx, local IP info:%s %s %s %s ", data, easylink_id, inContext->flashContentInRam.mxos_config.localIp,
            inContext->flashContentInRam.mxos_config.netMask, inContext->flashContentInRam.mxos_config.gateWay,inContext->flashContentInRam.mxos_config.dnsServer);
    }
    mos_mutex_unlock(inContext->flashContentInRam_mutex );
    source = CONFIG_BY_EASYLINK_V2;

    exit:
    if ( err != kNoErr )
    {
        /*EasyLink error*/
        system_log("EasyLink step 2 ERROR, err: %d", err);
        easylink_success = false;
    } else
        /* Easylink success after step 1 and step 2 */
        easylink_success = true;

    mos_semphr_release(easylink_sem );
    return;
}

static void switch_channel_thread(void * arg)
{
    mxos_time_t current;

    while(switch_channel_flag)
    {
        mxos_time_get_time( &current );
        if ( current > (mxos_time_t) arg ) {
            easylink_success = false;
            mos_semphr_release(easylink_sem );
            break;
        }

        if( wlan_channel_walker == MXOS_TRUE){
            mwifi_monitor_set_channel( wlan_channel );
            mxos_easylink_monitor_delegate_channel_changed( wlan_channel );
            wlan_channel++;
            if ( wlan_channel >= 14 ) wlan_channel = 1;
            mos_thread_delay(wlan_channel_walker_interval);
        }
    }

    switch_channel_thread_handler = NULL;
    mos_thread_delete(NULL);
}

static void monitor_cb( uint8_t * frame, int len )
{
    mxos_easylink_monitor_delegate_package_recved( frame, len );
}

static void easylink_remove_bonjour_from_sta(void)
{
    easylink_remove_bonjour(INTERFACE_STA);
}

static void easylink_monitor_thread( void *arg )
{
    merr_t err = kNoErr;
    system_context_t *context = (system_context_t *) arg;

    mxos_time_t current;
    easylink_id = 0x0;
    easylink_success = false;
    easylink_thread_force_exit = false;

    source = CONFIG_BY_NONE;
    mxos_system_notify_register( mxos_notify_EASYLINK_WPS_COMPLETED,    (void *) easylink_complete_cb,      context );
    mxos_system_notify_register( mxos_notify_EASYLINK_GET_EXTRA_DATA,   (void *) easylink_extra_data_cb,    context );
    mxos_system_notify_register( mxos_notify_WIFI_STATUS_CHANGED,       (void *) easylink_wifi_status_cb,   context );

    easylink_sem = mos_semphr_new( 1 );
    easylink_connect_sem = mos_semphr_new( 1 );

    mwifi_monitor_reg_cb( monitor_cb );

restart:
    mxos_system_delegate_config_will_start( );
    system_log("Start easylink monitor mode");
    mxos_easylink_monitor_delegate_will_start( );
    mxosWlanSuspend();
    mwifi_monitor_start( );

    wlan_channel_walker = MXOS_TRUE;
    mxos_time_get_time( &current );
    switch_channel_flag = true;
    switch_channel_thread_handler = mos_thread_new( MXOS_DEFAULT_WORKER_PRIORITY, "sw_channel",
                            switch_channel_thread, 0x1000, (void *)(current + EasyLink_TimeOut));

    while( mos_semphr_acquire(easylink_sem, 0 ) == kNoErr );
    err = mos_semphr_acquire(easylink_sem, MXOS_WAIT_FOREVER );

    switch_channel_flag = false;
    mwifi_monitor_stop();
    mxos_easylink_monitor_delegate_stoped();

    /* Easylink force exit by user, clean and exit */
    if( err != kNoErr && easylink_thread_force_exit )
    {
        system_log("EasyLink canceled by user");
        goto exit;
    }

    /* EasyLink Success */
    if ( easylink_success == true )
    {
        mxos_system_delegate_config_recv_ssid( context->flashContentInRam.mxos_config.ssid,
                                               context->flashContentInRam.mxos_config.user_key );
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

        source = (source == CONFIG_BY_NONE) ? CONFIG_BY_MONITOR : source;

        /* Easylink connect result */
        if ( err != kNoErr )
        {
            connect_fail_config = mxos_system_delegate_config_result( source, MXOS_FALSE );
            if ( RESTART_EASYLINK == connect_fail_config ) {
                system_log("Re-start easylink combo mode");
                mxosWlanSuspend( );
                goto restart;
            } else {
                system_log("exit easylink combo mode");
                mwifi_disconnect( );
                goto exit;
            }
        }
        else
        {
            mxos_system_delegate_config_result( source, MXOS_TRUE );
            mxos_easylink_monitor_delegate_connect_success( source );

            /* Start bonjour service for new device discovery */
            err = easylink_bonjour_start( Station, easylink_id, context );
            require_noerr( err, exit );
            SetTimer( 60 * 1000, easylink_remove_bonjour_from_sta );
        }
    }
    else /* EasyLink failed */
    {
        mxos_system_delegate_easylink_timeout( context );
    }

exit:
    easylink_thread_force_exit = false;

    mxos_system_delegate_config_will_stop( );

    mxos_system_notify_remove( mxos_notify_WIFI_STATUS_CHANGED, (void *)easylink_wifi_status_cb );
    mxos_system_notify_remove( mxos_notify_EASYLINK_WPS_COMPLETED, (void *)easylink_complete_cb );
    mxos_system_notify_remove( mxos_notify_EASYLINK_GET_EXTRA_DATA, (void *)easylink_extra_data_cb );

    mos_semphr_delete(easylink_sem );
    mos_semphr_delete(easylink_connect_sem );
    easylink_monitor_thread_handler = NULL;
    mos_thread_delete( NULL );
}

merr_t mxos_easylink_monitor_channel_walker( mxos_bool_t enable, uint32_t interval )
{
    wlan_channel_walker = enable;

    if( enable == MXOS_TRUE ) wlan_channel_walker_interval = interval;

    return kNoErr;
}

merr_t mxos_easylink_monitor_save_result( mwifi_softap_attr_t *nwkpara )
{
    system_context_t * context = system_context( );

    if( context == NULL ) return kNotPreparedErr;

    memcpy( context->flashContentInRam.mxos_config.ssid, nwkpara->wifi_ssid, maxSsidLen );
    memset( context->flashContentInRam.mxos_config.bssid, 0x0, 6 );
    memcpy( context->flashContentInRam.mxos_config.user_key, nwkpara->wifi_key, maxKeyLen );
    context->flashContentInRam.mxos_config.user_keyLength = strlen( nwkpara->wifi_key );
    context->flashContentInRam.mxos_config.dhcpEnable = true;

    system_log("Get SSID: %s, Key: %s", context->flashContentInRam.mxos_config.ssid, context->flashContentInRam.mxos_config.user_key);

    easylink_success = true;
    mos_semphr_release(easylink_sem );
    return kNoErr;
}

merr_t mxos_easylink_monitor_with_easylink( mxos_Context_t * const in_context, mxos_bool_t enable )
{
    merr_t err = kNoErr;

    require_action( in_context, exit, err = kNotPreparedErr );

    easylink_remove_bonjour( INTERFACE_STA );

    /* easylink thread existed? stop! */
    if ( easylink_monitor_thread_handler ) {
        system_log("EasyLink monitor processing, force stop..");
        easylink_thread_force_exit = true;
        mxos_rtos_thread_force_awake( &easylink_monitor_thread_handler );
        mos_thread_join( easylink_monitor_thread_handler );
    }

    if ( enable == MXOS_TRUE ) {
        easylink_monitor_thread_handler = mos_thread_new( MXOS_DEFAULT_LIBRARY_PRIORITY, "EASYLINK",
                                      easylink_monitor_thread, 0x1000, (void *)in_context);
        require_action_string( easylink_monitor_thread_handler != NULL, exit, err = kGeneralErr, "ERROR: Unable to start the EasyLink monitor thread." );

        /* Make sure easylink is already running, and waiting for sem trigger */
        mos_thread_delay( 1000 );
    }

    exit:
    return err;
}


merr_t mxos_easylink_monitor( mxos_Context_t * const in_context, mxos_bool_t enable )
{
    mxos_wlan_monitor_no_easylink();
    return mxos_easylink_monitor_with_easylink( in_context, enable );
}



//#endif

