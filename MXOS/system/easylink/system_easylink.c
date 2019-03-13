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

//#if (MXOS_WLAN_CONFIG_MODE == CONFIG_MODE_EASYLINK) || (MXOS_WLAN_CONFIG_MODE == CONFIG_MODE_EASYLINK_WITH_SOFTAP)

/******************************************************
 *               Function Declarations
 ******************************************************/
/* EasyLink event callback functions*/
static void easylink_wifi_status_cb( WiFiEvent event, system_context_t * const inContext );
static void easylink_complete_cb( network_InitTypeDef_st *nwkpara, system_context_t * const inContext );
static void easylink_extra_data_cb( int datalen, char* data, system_context_t * const inContext );

/* Thread perform easylink and connect to wlan */
static void easylink_thread( void *inContext ); /* Perform easylink and connect to wlan */

/******************************************************
 *               Variables Definitions
 ******************************************************/
static mos_semphr_id_t easylink_sem;         /**< Used to suspend thread while easylink. */
static mos_semphr_id_t easylink_connect_sem; /**< Used to suspend thread while connection. */
static bool easylink_success = false;         /**< true: connect to wlan, false: start soft ap mode or roll back to previous settings */
static uint32_t easylinkIndentifier = 0;      /**< Unique for an easylink instance. */
static mos_thread_id_t easylink_thread_handler = NULL;
static bool easylink_thread_force_exit = false;

static mxos_config_source_t source = CONFIG_BY_NONE;

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
    memcpy( inContext->flashContentInRam.mxosSystemConfig.ssid, nwkpara->wifi_ssid, maxSsidLen );
    memset( inContext->flashContentInRam.mxosSystemConfig.bssid, 0x0, 6 );
    memcpy( inContext->flashContentInRam.mxosSystemConfig.user_key, nwkpara->wifi_key, maxKeyLen );
    inContext->flashContentInRam.mxosSystemConfig.user_keyLength = strlen( nwkpara->wifi_key );
    memcpy( inContext->flashContentInRam.mxosSystemConfig.key, nwkpara->wifi_key, maxKeyLen );
    inContext->flashContentInRam.mxosSystemConfig.keyLength = strlen( nwkpara->wifi_key );
    inContext->flashContentInRam.mxosSystemConfig.dhcpEnable = true;
    mos_mutex_unlock(inContext->flashContentInRam_mutex );
    system_log("Get SSID: %s, Key: %s", inContext->flashContentInRam.mxosSystemConfig.ssid, inContext->flashContentInRam.mxosSystemConfig.user_key);

    source = (mxos_config_source_t) nwkpara->wifi_retry_interval;
    exit:
    if ( err != kNoErr )
    {
        /*EasyLink timeout or error*/
        easylink_success = false;
        mos_semphr_release(easylink_sem );
    }
    return;
}

#if PLATFORM_CONFIG_EASYLINK_SOFTAP_COEXISTENCE
static void easylink_uap_configured_cd(uint32_t id)
{
    easylinkIndentifier = id;
    easylink_success = true;
    mxosWlanSuspendSoftAP();
    mos_semphr_release(easylink_sem );
}
#endif

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
    uint32_t ipInfoCount;
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
	memcpy(&easylinkIndentifier, &data[index], 4);
    /* Identifier: 1 x uint32_t or Identifier/localIp/netMask/gateWay/dnsServer: 5 x uint32_t */
    ipInfoCount = (datalen - index) / sizeof(uint32_t);
    require_action( ipInfoCount >= 1, exit, err = kParamErr );

    mos_mutex_lock(inContext->flashContentInRam_mutex );

    if ( ipInfoCount == 1 )
    { //Use DHCP to obtain local ip address
        inContext->flashContentInRam.mxosSystemConfig.dhcpEnable = true;
        system_log("Get auth info: %s, EasyLink identifier: %lx", data, easylinkIndentifier);
    } else
    { //Use static ip address
        inContext->flashContentInRam.mxosSystemConfig.dhcpEnable = false;
		memcpy(&ipv4_addr.s_addr, &data[index+4], 4);
		ipv4_addr.s_addr = hton32( ipv4_addr.s_addr );
        strcpy( (char *) inContext->flashContentInRam.mxosSystemConfig.localIp, inet_ntoa( ipv4_addr ) );
		memcpy(&ipv4_addr.s_addr, &data[index+8], 4);
		ipv4_addr.s_addr = hton32( ipv4_addr.s_addr );
        strcpy( (char *) inContext->flashContentInRam.mxosSystemConfig.netMask, inet_ntoa( ipv4_addr ) );
		memcpy(&ipv4_addr.s_addr, &data[index+12], 4);
		ipv4_addr.s_addr = hton32( ipv4_addr.s_addr );
        strcpy( (char *) inContext->flashContentInRam.mxosSystemConfig.gateWay, inet_ntoa( ipv4_addr ) );
		memcpy(&ipv4_addr.s_addr, &data[index+16], 4);
		ipv4_addr.s_addr = hton32( ipv4_addr.s_addr );
        strcpy( (char *) inContext->flashContentInRam.mxosSystemConfig.dnsServer, inet_ntoa( ipv4_addr ) );

        system_log("Get auth info: %s, EasyLink identifier: %lx, local IP info:%s %s %s %s ", data, easylinkIndentifier, inContext->flashContentInRam.mxosSystemConfig.localIp,
            inContext->flashContentInRam.mxosSystemConfig.netMask, inContext->flashContentInRam.mxosSystemConfig.gateWay,inContext->flashContentInRam.mxosSystemConfig.dnsServer);
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

static void easylink_remove_bonjour_from_sta(void)
{
    easylink_remove_bonjour(INTERFACE_STA);
}

static void easylink_thread( void *arg )
{
    merr_t err = kNoErr;
    system_context_t *context = (system_context_t *) arg;

    easylinkIndentifier = 0x0;
    easylink_success = false;
    easylink_thread_force_exit = false;

    source = CONFIG_BY_NONE;
    mxos_system_notify_register( mxos_notify_EASYLINK_WPS_COMPLETED,    (void *) easylink_complete_cb,      context );
    mxos_system_notify_register( mxos_notify_EASYLINK_GET_EXTRA_DATA,   (void *) easylink_extra_data_cb,    context );
    mxos_system_notify_register( mxos_notify_WIFI_STATUS_CHANGED,       (void *) easylink_wifi_status_cb,   context );

    easylink_sem = mos_semphr_new( 1 );
    easylink_connect_sem = mos_semphr_new( 1 );

restart:
    mxos_system_delegate_config_will_start( );
    system_log("Start easylink combo mode");
#if PLATFORM_CONFIG_EASYLINK_SOFTAP_COEXISTENCE
    char wifi_ssid[32];
    sprintf( wifi_ssid, "EasyLink_%c%c%c%c%c%c",
        context->mxosStatus.mac[9], context->mxosStatus.mac[10], context->mxosStatus.mac[12],
        context->mxosStatus.mac[13], context->mxosStatus.mac[15], context->mxosStatus.mac[16]);

    system_log("Enable softap %s in easylink", wifi_ssid);
    mxos_wlan_easylink_uap_start( EasyLink_TimeOut / 1000, wifi_ssid, NULL, 6 );
    /* Start config server */
    config_server_start( );
    config_server_set_uap_cb( easylink_uap_configured_cd );
    easylink_bonjour_start( Soft_AP, 0, context );
    while( mos_semphr_acquire(easylink_sem, 0 ) == kNoErr );
    err = mos_semphr_acquire(easylink_sem, EasyLink_TimeOut );
#else
    mxosWlanStartEasyLinkPlus( EasyLink_TimeOut / 1000 );
    while( mos_semphr_acquire(easylink_sem, 0 ) == kNoErr );
    err = mos_semphr_acquire(easylink_sem, MXOS_WAIT_FOREVER );
#endif



    /* Easylink force exit by user, clean and exit */
    if( err != kNoErr && easylink_thread_force_exit )
    {
        system_log("EasyLink waiting for terminate");
        mxosWlanStopEasyLinkPlus( );
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
            mxosWlanPowerOff();
        }
    }

exit:
    easylink_thread_force_exit = false;

    mxos_system_delegate_config_will_stop( );

    mxos_system_notify_remove( mxos_notify_WIFI_STATUS_CHANGED, (void *)easylink_wifi_status_cb );
    mxos_system_notify_remove( mxos_notify_EASYLINK_WPS_COMPLETED, (void *)easylink_complete_cb );
    mxos_system_notify_remove( mxos_notify_EASYLINK_GET_EXTRA_DATA, (void *)easylink_extra_data_cb );

    mos_semphr_delete(easylink_sem );
    mos_semphr_delete(easylink_connect_sem );
    easylink_thread_handler = NULL;
    mos_thread_delete( NULL );
}

merr_t mxos_easylink( mxos_Context_t * const in_context, mxos_bool_t enable )
{
    merr_t err = kUnknownErr;

    require_action( in_context, exit, err = kNotPreparedErr );

    easylink_remove_bonjour( INTERFACE_STA );

    /* easylink thread existed? stop! */
    if ( easylink_thread_handler ) {
        system_log("EasyLink processing, force stop..");
        easylink_thread_force_exit = true;
        mxos_rtos_thread_force_awake( &easylink_thread_handler );
        mos_thread_join( easylink_thread_handler );
    }

    if ( enable == MXOS_TRUE ) {
        easylink_thread_handler = mos_thread_new( MXOS_APPLICATION_PRIORITY, "EASYLINK", easylink_thread,
                                       0x1000, (void *) in_context );
        require_action_string( easylink_thread_handler != NULL, exit, err = kGeneralErr, "ERROR: Unable to start the EasyLink thread." );

        /* Make sure easylink is already running, and waiting for sem trigger */
        mos_thread_delay( 1000 );
    }

    exit:
    return err;
}


//#endif

