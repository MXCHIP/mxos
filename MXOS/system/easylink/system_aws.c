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

#define AWS_NOTIFY_INTERVAL (20*1000)
#define AWS_NOTIFY_TIMES    500
//#if (WIFI_CONFIG_MODE == WIFI_CONFIG_MODE_EASYLINK) || (WIFI_CONFIG_MODE == WIFI_CONFIG_MODE_EASYLINK_WITH_SOFTAP)

/******************************************************
 *               Function Declarations
 ******************************************************/
/* EasyLink event callback functions*/
static void aws_wifi_status_cb( WiFiEvent event, system_context_t * const inContext );
static void aws_complete_cb( char *ssid, char *key, int mode, system_context_t * const inContext );

/* Thread perform easylink and connect to wlan */
static void aws_thread( void *inContext ); /* Perform easylink and connect to wlan */
char* aws_notify_msg_create(system_context_t *context);

/******************************************************
 *               Variables Definitions
 ******************************************************/
static mos_semphr_id_t aws_sem;         /**< Used to suspend thread while easylink. */
static mos_semphr_id_t aws_connect_sem; /**< Used to suspend thread while connection. */
static bool aws_success = false;         /**< true: connect to wlan, false: start soft ap mode or roll back to previous settings */
static mos_thread_id_t aws_thread_handler = NULL;
static bool aws_thread_force_exit = false;


/******************************************************
 *               Function Definitions
 ******************************************************/

/* MXOS callback when WiFi status is changed */
static void aws_wifi_status_cb( WiFiEvent event, system_context_t * const inContext )
{
    switch ( event )
    {
        case NOTIFY_STATION_UP:
            inContext->flashContentInRam.mxosSystemConfig.configured = allConfigured;
            mxos_system_context_update( &inContext->flashContentInRam ); //Update Flash content
            mos_semphr_release(aws_connect_sem ); //Notify Easylink thread
            break;
        default:
            break;
    }
    return;
}

/* MXOS callback when EasyLink is finished step 1, return SSID and KEY */
static void aws_complete_cb( char *ssid, char *key, int mode, system_context_t * const inContext )
{
    merr_t err = kNoErr;

    require_action_string( ssid, exit, err = kTimeoutErr, "AWS Timeout or terminated" );

    /* Store SSID and KEY*/
    mos_mutex_lock(inContext->flashContentInRam_mutex );
    memcpy( inContext->flashContentInRam.mxosSystemConfig.ssid, ssid, maxSsidLen );
    memset( inContext->flashContentInRam.mxosSystemConfig.bssid, 0x0, 6 );
    memcpy( inContext->flashContentInRam.mxosSystemConfig.user_key, key, maxKeyLen );
    inContext->flashContentInRam.mxosSystemConfig.user_keyLength = strlen( key );
    memcpy( inContext->flashContentInRam.mxosSystemConfig.key, key, maxKeyLen );
    inContext->flashContentInRam.mxosSystemConfig.keyLength = strlen( key );
    inContext->flashContentInRam.mxosSystemConfig.dhcpEnable = true;
    mos_mutex_unlock(inContext->flashContentInRam_mutex );
    system_log("Get SSID: %s, Key: %s", inContext->flashContentInRam.mxosSystemConfig.ssid, inContext->flashContentInRam.mxosSystemConfig.user_key);
    aws_success = true;
    exit:
    if ( err != kNoErr )
    {
        /*EasyLink timeout or error*/
        aws_success = false;
    }
    mos_semphr_release(aws_sem );
    return;
}

#define UDP_TX_PORT         (65123)
#define UDP_RX_PORT         (65126)

static int aws_broadcast_notification(char *msg, int msg_num)
{
    int i, ret, result = 0;
    int fd;
    socklen_t addrlen;
    fd_set readfds;
    struct timeval t;
    struct sockaddr_in s_addr;
    int buf_len = 1024;
    char *buf = malloc(buf_len);
    uint8_t stop = 0xEE;
    
    memset(buf, 0, buf_len);
    fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0)
    {
        system_log("CREATE UDP SOCKET ERROR!!!\n");
        return -1;
    }
    memset(&s_addr, 0, sizeof(struct sockaddr_in));
    s_addr.sin_family = AF_INET;
    s_addr.sin_addr.s_addr = INADDR_ANY;
    s_addr.sin_port = htons(UDP_RX_PORT);
    if (bind(fd, (struct sockaddr*)&s_addr, sizeof(s_addr)) < 0)
    {
        system_log("BIND UDP SOCKET ERROR!!!\n");
        return -2;
    }
    system_log("UDP SOCKET initialized!\n");

    memset(&s_addr, 0, sizeof(struct sockaddr_in));
    s_addr.sin_family = AF_INET;
    s_addr.sin_addr.s_addr = INADDR_BROADCAST;
    s_addr.sin_port = htons(UDP_TX_PORT);
    //stop APP broadcast aws packets.
    for(i=0; i<10; i++) {
        ret = sendto(fd, &stop, 1, 0, (struct sockaddr *)&s_addr, sizeof(s_addr));
        if (ret > 0) {
            i++; // sendto fail don't i++, max sending times is 10, min sending times is 5.
        }
        mxos_rtos_thread_msleep(20);
    }
    
    //send notification
    for (i = 0; i < msg_num; i++) {
        if (aws_thread_force_exit == true) {
            break;
        }
        ret = sendto(fd, msg, strlen(msg), 0, (struct sockaddr *)&s_addr, sizeof(s_addr));
        if (ret < 0) {
            system_log("awss send notify msg ERROR!\r\n");
        } 

        FD_ZERO(&readfds);
        t.tv_sec = 0;
        t.tv_usec = AWS_NOTIFY_INTERVAL;
        FD_SET(fd, &readfds);
        ret = select(fd+1, &readfds, NULL, NULL, &t);
        if (ret > 0) {
            addrlen = sizeof(s_addr);
            ret = recvfrom(fd, buf, buf_len, 0, (struct sockaddr *)&s_addr, &addrlen);
            system_log("rx len %d\n", ret);
            if (ret > 0) {
                //buf[ret] = '\0';
                system_log("rx: %s\n", buf);
                buf[strlen(buf)-1] = '\0';
                sprintf(buf, "%s,\"IP\":\"%s\",\"PORT\":%d}", buf, (char *)inet_ntoa(s_addr.sin_addr), hton16(s_addr.sin_port));
                mxos_easylink_aws_delegate_recv_notify_msg(buf);
                result = 1;
                break;
            }
        }
    }

    free(buf);
    close(fd);
    if (result == 0) {
        system_log("awss notify %d times, no response\r\n", msg_num);
    }
    return result;
}

static void aws_thread( void *arg )
{
    merr_t err = kNoErr;
    system_context_t *context = (system_context_t *) arg;
    char *aws_msg = aws_notify_msg_create(context);

    if (aws_msg == NULL) {
        system_log("Not enough memory!!");
        goto exit;
    }

    aws_success = false;
    aws_thread_force_exit = false;

    mxos_system_notify_register( mxos_notify_EASYLINK_WPS_COMPLETED,    (void *) aws_complete_cb,      context );
    mxos_system_notify_register( mxos_notify_WIFI_STATUS_CHANGED,       (void *) aws_wifi_status_cb,   context );

    aws_sem = mos_semphr_new( 1 );
    aws_connect_sem = mos_semphr_new( 1 );

restart:
    mxos_system_delegate_config_will_start( );
    system_log("Start AWS mode");

    mwifi_softap_startAws( EasyLink_TimeOut / 1000 );
    while( mos_semphr_acquire(aws_sem, 0 ) == kNoErr );
    err = mos_semphr_acquire(aws_sem, MXOS_WAIT_FOREVER );

    /* Easylink force exit by user, clean and exit */
    if( err != kNoErr && aws_thread_force_exit )
    {
        system_log("AWS waiting for terminate");
        mwifi_aws_stop( );
        mos_semphr_acquire(aws_sem, 3000 );
        system_log("AWS canceled by user");
        goto exit;
    }

    /* AWS Success */
    if ( aws_success == true )
    {
        mxos_system_delegate_config_recv_ssid( context->flashContentInRam.mxosSystemConfig.ssid,
                                               context->flashContentInRam.mxosSystemConfig.user_key );
        system_connect_wifi_normal( context );

        /* Wait for station connection */
        while( mos_semphr_acquire(aws_connect_sem, 0 ) == kNoErr );
        err = mos_semphr_acquire(aws_connect_sem, EasyLink_ConnectWlan_Timeout );
        /* AWS force exit by user, clean and exit */
        if( err != kNoErr && aws_thread_force_exit )
        {
            mxosWlanSuspend();
            system_log("AWS connection canceled by user");
            goto exit;
        }

        /*SSID or Password is not correct, module cannot connect to wlan, so restart AWS again*/
        require_noerr_action_string( err, restart, mxosWlanSuspend(), "Re-start AWS mode" );
        mxos_system_delegate_config_success( CONFIG_BY_AWS );

/* mxos_config.h can define MXOS_AWS_NOTIFY_DISABLE to disable send aws notification */
#ifndef MXOS_AWS_NOTIFY_DISABLE
        /* Start AWS udp notify */
        aws_broadcast_notification(aws_msg, AWS_NOTIFY_TIMES);
#endif
        goto exit;
    }
    else /* EasyLink failed */
    {
        mxos_system_delegate_easylink_timeout(context);
    }

exit:
    aws_thread_force_exit = false;
    if (aws_msg) {
        free(aws_msg);
    }
    mxos_system_delegate_config_will_stop( );

    mxos_system_notify_remove( mxos_notify_WIFI_STATUS_CHANGED, (void *)aws_wifi_status_cb );
    mxos_system_notify_remove( mxos_notify_EASYLINK_WPS_COMPLETED, (void *)aws_complete_cb );

    mos_semphr_delete(aws_sem );
    mos_semphr_delete(aws_connect_sem );
    aws_thread_handler = NULL;
    mos_thread_delete( NULL );
}

merr_t mxos_easylink_aws( mxos_Context_t * const in_context, mxos_bool_t enable )
{
    merr_t err = kUnknownErr;

    require_action( in_context, exit, err = kNotPreparedErr );

    /* easylink thread existed? stop! */
    if ( aws_thread_handler ) {
        system_log("EasyLink processing, force stop..");
        aws_thread_force_exit = true;
        mxos_rtos_thread_force_awake( &aws_thread_handler );
        mos_thread_join( aws_thread_handler );
    }

    if ( enable == MXOS_TRUE ) {
        aws_thread_handler = mos_thread_new( MXOS_APPLICATION_PRIORITY, "aws", aws_thread,
                                       0x1000, (void *) in_context );
        require_action_string( aws_thread_handler != NULL, exit, err = kGeneralErr, "ERROR: Unable to start the EasyLink thread." );

        err = kNoErr;

        /* Make sure easylink is already running, and waiting for sem trigger */
        mos_thread_delay( 1000 );
    }

    exit:
    return err;
}

#define AWS_NOTIFY_MSG_LEN 512
char* aws_notify_msg_create(system_context_t *context)
{
    char *aws_notify_msg = (char*)malloc(AWS_NOTIFY_MSG_LEN);
    char sn[64];
    uint8_t mac[6];

    mwifi_get_mac(mac);
    sprintf(sn, "%02X0%02X0%02X0%02X0%02X0%02X", 
        mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    
    if (aws_notify_msg == NULL) {
        
        goto exit;
    }
    memset(aws_notify_msg, 0, AWS_NOTIFY_MSG_LEN);
#if 1
    snprintf(aws_notify_msg, AWS_NOTIFY_MSG_LEN,
             "{\"FW\":\"%s\",\"HD\":\"%s\",\"PO\":\"%s\",\"RF\":\"%s\",\"MAC\":\"%s\",\"OS\":\"%s\",\"MD\":\"%s\",\"MF\":\"%s\"",
             FIRMWARE_REVISION, HARDWARE_REVISION, PROTOCOL,
             context->mxosStatus.rf_version, context->mxosStatus.mac,
             mxos_system_lib_version( ), MODEL, MANUFACTURER );

    sprintf(aws_notify_msg, "%s,\"wlan unconfigured\":\"F\"", aws_notify_msg);

#ifdef MXOS_CONFIG_SERVER_ENABLE
    sprintf(aws_notify_msg, "%s,\"FTC\":\"T\",\"PORT\":%d", aws_notify_msg,MXOS_CONFIG_SERVER_PORT);
#else
    sprintf(aws_notify_msg, "%s,\"FTC\":\"F\"", aws_notify_msg);
#endif
#else
    sprintf(aws_notify_msg, "{\"version\":\"1.6\",\"model\":\"%s\",\"sn\":\"%s\"}",
        "ALINKTEST_LIVING_LIGHT_ALINK_TEST", sn);
#endif
    sprintf(aws_notify_msg, "%s,\"ExtraData\":\"", aws_notify_msg);

    mxos_easylink_aws_delegate_send_notify_msg(aws_notify_msg);

     sprintf(aws_notify_msg, "%s\"}", aws_notify_msg);
exit:
    return aws_notify_msg;
}

//#endif

