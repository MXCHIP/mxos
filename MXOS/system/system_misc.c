/**
 ******************************************************************************
 * @file    system_misc.c
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This file provide the system mics functions for internal usage
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
#include "StringUtils.h"
#include "time.h"

#include "system_internal.h"

extern system_context_t* sys_context;

system_context_t *system_context( void )
{
    return sys_context;
}

static void mxosNotify_ConnectFailedHandler(merr_t err, system_context_t * const inContext)
{
  (void)inContext;
  system_log("Wlan Connection Err %d", err);
}

static void mxosNotify_WlanFatalErrHandler(system_context_t * const inContext)
{
  (void)inContext;
  system_log("Wlan Fatal Err!");
  mxos_sys_reboot();
}

static void mxosNotify_StackOverflowErrHandler(char *taskname, system_context_t * const inContext)
{
  (void)inContext;
  system_log("Thread %s overflow, system rebooting", taskname);
  mxos_sys_reboot();
}

static void mxosNotify_WifiStatusHandler(WiFiEvent event, system_context_t * const inContext)
{
  mwifi_ip_attr_t attr;
  
  (void)inContext;
  switch (event) {
  case NOTIFY_STATION_UP:
    system_log("Station up");
    mxos_rf_led(true);
    mwifi_get_ip(&attr, STATION_INTERFACE);
    mos_mutex_lock(inContext->flashContentInRam_mutex);
    strcpy((char *)inContext->mxosStatus.localIp, attr.localip);
    strcpy((char *)inContext->mxosStatus.netMask, attr.netmask);
    strcpy((char *)inContext->mxosStatus.gateWay, attr.gateway);
    strcpy((char *)inContext->mxosStatus.dnsServer, attr.dnserver);
    mos_mutex_unlock(inContext->flashContentInRam_mutex);
    break;
  case NOTIFY_STATION_DOWN:
    system_log("Station down");
    mxos_rf_led(false);
    break;
  case NOTIFY_AP_UP:
    system_log("uAP established");
    mxos_rf_led(true);
    break;
  case NOTIFY_AP_DOWN:
    system_log("uAP deleted");
    mxos_rf_led(false);
    break;
  case NOTIFY_ETH_UP:
    system_log("ETH up");
    break;
  case NOTIFY_ETH_DOWN:
    system_log("ETH down");
    break;
  default:
    break;
  }
  return;
}

static void mxosNotify_WiFIParaChangedHandler(mwifi_link_info_t *ap_info, char *key, int key_len, system_context_t * const inContext)
{
  bool _needsUpdate = false;
  require(inContext, exit);
  mos_mutex_lock(inContext->flashContentInRam_mutex);
  if(strncmp(inContext->flashContentInRam.mxos_config.ssid, ap_info->ssid, maxSsidLen)!=0){
    strncpy(inContext->flashContentInRam.mxos_config.ssid, ap_info->ssid, maxSsidLen);
    _needsUpdate = true;
  }

  if(memcmp(inContext->flashContentInRam.mxos_config.bssid, ap_info->bssid, 6)!=0){
    memcpy(inContext->flashContentInRam.mxos_config.bssid, ap_info->bssid, 6);
    _needsUpdate = true;
  }

  if(inContext->flashContentInRam.mxos_config.channel != ap_info->channel){
    inContext->flashContentInRam.mxos_config.channel = ap_info->channel;
    _needsUpdate = true;
  }
  
  if(inContext->flashContentInRam.mxos_config.security != ap_info->security){
    inContext->flashContentInRam.mxos_config.security = ap_info->security;
    _needsUpdate = true;
  }

  if (key_len == maxKeyLen) {//  maxKeyLen is the PSK. using PSK replace passphrase. 
    if(memcmp(inContext->flashContentInRam.mxos_config.key, key, maxKeyLen)!=0){
      memcpy(inContext->flashContentInRam.mxos_config.key, key, maxKeyLen);
      _needsUpdate = true;
    }
  
    if(inContext->flashContentInRam.mxos_config.keyLength != key_len){
      inContext->flashContentInRam.mxos_config.keyLength = key_len;
      _needsUpdate = true;
    }
  }
  
  if(_needsUpdate== true)  
    mxos_system_context_update( &inContext->flashContentInRam );
  mos_mutex_unlock(inContext->flashContentInRam_mutex);
  
exit:
  return;
}

merr_t system_notification_init( system_context_t * const inContext )
{
  merr_t err = kNoErr;

  err = mxos_system_notify_register( mxos_notify_WIFI_CONNECT_FAILED, (void *)mxosNotify_ConnectFailedHandler, inContext );
  require_noerr( err, exit );

  err = mxos_system_notify_register( mxos_notify_WIFI_Fatal_ERROR, (void *)mxosNotify_WlanFatalErrHandler, inContext );
  require_noerr( err, exit ); 

  err = mxos_system_notify_register( mxos_notify_Stack_Overflow_ERROR, (void *)mxosNotify_StackOverflowErrHandler, inContext );
  require_noerr( err, exit );

  err = mxos_system_notify_register( mxos_notify_WIFI_STATUS_CHANGED, (void *)mxosNotify_WifiStatusHandler, inContext );
  require_noerr( err, exit );

  err = mxos_system_notify_register( mxos_notify_WiFI_PARA_CHANGED, (void *)mxosNotify_WiFIParaChangedHandler, inContext );
  require_noerr( err, exit ); 

exit:
  return err;
}

void system_connect_wifi_normal( system_context_t * const inContext)
{
  system_log("connect to %s.....", inContext->flashContentInRam.mxos_config.ssid);
  mwifi_connect(inContext->flashContentInRam.mxos_config.ssid, inContext->flashContentInRam.mxos_config.user_key,
    inContext->flashContentInRam.mxos_config.user_keyLength, NULL, NULL);
}

void system_connect_wifi_fast( system_context_t * const inContext)
{
  char *ssid, *key;
  int security, key_len;
  mwifi_ip_attr_t attr, *pip_attr=NULL;
  mwifi_connect_attr_t ap_attr;
  
  mos_mutex_lock(inContext->flashContentInRam_mutex);
  ssid = inContext->flashContentInRam.mxos_config.ssid;
  memcpy(ap_attr.bssid, inContext->flashContentInRam.mxos_config.bssid, 6);
  ap_attr.channel = inContext->flashContentInRam.mxos_config.channel;
  ap_attr.security = inContext->flashContentInRam.mxos_config.security;
  key = inContext->flashContentInRam.mxos_config.key;
  key_len = inContext->flashContentInRam.mxos_config.keyLength;
  if(inContext->flashContentInRam.mxos_config.dhcpEnable != true){
      strncpy(attr.localip, inContext->flashContentInRam.mxos_config.localIp, maxIpLen);
      strncpy(attr.netmask, inContext->flashContentInRam.mxos_config.netMask, maxIpLen);
      strncpy(attr.gateway, inContext->flashContentInRam.mxos_config.gateWay, maxIpLen);
      strncpy(attr.dnserver, inContext->flashContentInRam.mxos_config.dnsServer, maxIpLen);
      pip_attr = &attr;
  }
  mos_mutex_unlock(inContext->flashContentInRam_mutex);

  system_log("Connect to %s.....", ssid);
  mwifi_connect(ssid, key, key_len, &ap_attr, pip_attr);
}

merr_t system_network_daemen_start( system_context_t * const inContext )
{
  uint8_t major, minor, revision;
  uint8_t mac[6];

  mxos_network_init();
  mxos_sys_led(true);
  mwifi_get_mac(mac);
  sprintf(inContext->mxosStatus.mac, "%02X:%02X:%02X:%02X:%02X:%02X", 
    mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  mxos_wlan_driver_version(inContext->mxosStatus.rf_version, sizeof(inContext->mxosStatus.rf_version));
  inContext->mxosStatus.rf_version[49] = 0x0;

  system_log("Author: %s", MXOS_OS_USER_NAME );
  system_log("Email: %s", MXOS_OS_USER_EMAIL );
  system_log("GCC version: %s", CC_VERSION );
  
  system_log("MXOS version: %s", MXOS_OS_VERSION );
  system_log("Kernel version: %s", mxos_system_lib_version());
  system_log("Wi-Fi driver version %s", inContext->mxosStatus.rf_version);
  system_log("Wi-Fi mac address: %s", inContext->mxosStatus.mac);

  system_log("Free memory %d bytes", mos_mallinfo()->free); 

#if PLATFORM_ETH_ENABLE
  mxos_eth_bringup(true, NULL, NULL, NULL);
  system_log("Ethernet mac address %s", mxos_eth_get_mac_address());
#endif

  if(inContext->flashContentInRam.mxos_config.rfPowerSaveEnable == true){
    mwifi_ps_on();
  }

  if(inContext->flashContentInRam.mxos_config.mcuPowerSaveEnable == true){
    mxos_mcu_powersave_config(true);
  }  
  return kNoErr;
}

#if MXOS_CONFIG_EASYLINK_BTN_ENABLE
#include "button.h"

button_context_t easylink_btn;

static void PlatformEasyLinkButtonClickedCallback(void)
{
  require_quiet( sys_context, exit );

#ifdef EasyLink_Needs_Reboot
  /* Enter easylink mode temporary in configed mode */
  if(sys_context->flashContentInRam.mxos_config.configured == allConfigured){
      sys_context->flashContentInRam.mxos_config.configured = wLanUnConfigured;
    needs_update = true;
  }

  mxos_system_power_perform( &sys_context->flashContentInRam, eState_Software_Reset );
#else
  mxos_system_wlan_start_autoconf( );
#endif

exit:
  return;
}

static void PlatformEasyLinkButtonLongPressedCallback(void)
{
  mxos_Context_t* context = NULL;
  mxos_logic_partition_t *partition = NULL;

    return;
  context = mxos_system_context_get( );
  require( context, exit );

  partition = mhal_flash_get_info( MXOS_PARTITION_KV );
  mhal_flash_erase( MXOS_PARTITION_KV ,0x0, partition->partition_length );

  mxos_system_power_perform( context, eState_Software_Reset );

exit:
  return;
}

void system_easylink_btn_init( mxos_gpio_t btn, uint32_t long_pressed_timeout )
{
    easylink_btn.gpio = btn;
    easylink_btn.idle = MXOS_CONFIG_EASYLINK_BTN_IDLE_STATE;
    easylink_btn.pressed_func = PlatformEasyLinkButtonClickedCallback;
    easylink_btn.long_pressed_func = PlatformEasyLinkButtonLongPressedCallback;
    easylink_btn.long_pressed_timeout = long_pressed_timeout;

    button_init( &easylink_btn );
}

#endif

void mxos_sdk_version( uint8_t *major, uint8_t *minor, uint8_t *revision )
{
  *major = MXOS_SDK_VERSION_MAJOR;
  *minor = MXOS_SDK_VERSION_MINOR;
  *revision = MXOS_SDK_VERSION_REVISION;
}

merr_t mxos_system_get_status_wlan( system_status_wlan_t** status )
{
    if( sys_context == NULL )
    {
        *status = NULL;
        return kNotPreparedErr;
    }
    else
    {
        *status = &sys_context->mxosStatus;
        return kNotPreparedErr;
    }
}

merr_t mxos_system_wlan_get_status( mxos_system_status_wlan_t** status )
{
    return mxos_system_get_status_wlan( status );
}

void mxos_app_info(char *str, int len)
{
  snprintf( str, len, "%s %s, build at %s %s", APP_INFO, FIRMWARE_REVISION, __TIME__, __DATE__);
}


