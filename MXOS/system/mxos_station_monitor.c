/**
 ******************************************************************************
 * @file    mxos_station_monitor.c
 * @author  Yang Haibo
 * @version V1.0.0
 * @date    01-11-2017
 * @brief   This file provide the function to monitor station status. 
            Start softap when station disconnected too long.
            Stop softap when station connected.
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

/** @file
 *  Setup softap if station is disconnect from AP
 */

#include "mxos.h"

static mos_semphr_id_t sem;
static int station_up = 0, softap_up = 0;
static char softap_ssid[33], softap_key[64];
static uint32_t softap_wait_seconds;

#define station_m_log(format, ...)  custom_log("mxos", format, ##__VA_ARGS__)

static void mxosNotify_WifiStatusHandler(WiFiEvent event,  void* inContext)
{
  switch (event) 
  {
  case NOTIFY_STATION_UP:
    station_up = 1;
    mos_semphr_release(sem);
    break;
  case NOTIFY_STATION_DOWN:
    station_up = 0;
    mos_semphr_release(sem);
    break;
  default:
    break;
  }
}

static void station_monitro_func( void * arg )
{
    mwifi_ip_attr_t ip_attr;
  
    while(1) {
        mos_semphr_acquire(sem, MOS_WAIT_FOREVER);
        if (station_up == 1) {
            if (softap_up) {
                station_m_log("Stop softap");
                mwifi_softap_stop();
                softap_up = 0;
            }
        } else if (softap_up == 0) {
            mwifi_softap_attr_t wNetConfig;

            mos_semphr_acquire(sem, softap_wait_seconds*1000);
            if (station_up == 1)
                continue;

            station_m_log("Establish SofAP, SSID:%s and KEY:%s", softap_ssid, softap_key);

            strcpy( ip_attr.localip, "10.10.10.1" );
            strcpy( ip_attr.netmask, "255.255.255.0" );
            strcpy( ip_attr.gateway, "10.10.10.1" );
            strcpy( ip_attr.dnserver, "10.10.10.1" );
            mwifi_softap_start( softap_ssid, softap_key, 6, &ip_attr);
            softap_up = 1;
        }
    }

    mos_thread_delete(NULL);
}

/* Start a softap if station is disconnected more than trigger_seconds.
 * Stop softap if station is connected.
 * The softap's ssid and passphres is setted as <ssid> <key>
 */
int mxos_station_status_monitor(char *ssid, char*key, int trigger_seconds)
{
    int err = kNoErr;
    
    sem = mos_semphr_new(1);
    mos_semphr_release(sem);
    /* Register user function when wlan connection status is changed */
    err = mxos_system_notify_register( mxos_notify_WIFI_STATUS_CHANGED, (void *)mxosNotify_WifiStatusHandler, NULL );
    require_noerr( err, exit );

    strncpy(softap_ssid, ssid, 33);
    strncpy(softap_key, key, 64);
    softap_wait_seconds = trigger_seconds;
    
    mos_thread_new( MOS_APPLICATION_PRIORITY, "station monitor",
        station_monitro_func, 1024, NULL);
exit:
    return err;
}

