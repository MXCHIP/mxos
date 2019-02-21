/**
 ******************************************************************************
 * @file    mxos_opt.h
 * @author  William Xu
 * @version V1.0.0
 * @date    22-July-2015
 * @brief   This file provide MXOS default configurations
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

#ifndef __MXOS_OPT_H
#define __MXOS_OPT_H

#ifndef MXOS_PREBUILT_LIBS
#ifdef ALIOS_NATIVE_APP
#include "alios_native_app_config.h"
#else
#include "mxos_config.h"
#endif
#include "mxos_board_conf.h"
#endif


#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 *                            MXOS OS and APP VERSION
 ******************************************************************************/

#if !defined APP_INFO
#define APP_INFO                                 "MXOS BASIC Demo"
#endif

#if !defined FIRMWARE_REVISION
#define FIRMWARE_REVISION                       "MXOS_BASIC_1_0"
#endif

#if !defined MANUFACTURER
#define MANUFACTURER                            "MXCHIP Inc."
#endif

#if !defined SERIAL_NUMBER
#define SERIAL_NUMBER                           "20170101"
#endif

#if !defined PROTOCOL
#define PROTOCOL                                "com.mxchip.basic"
#endif

/**
 *  MXOS_SDK_VERSION_XXX, should be defined in MakeFile
 */
#ifndef MXOS_SDK_VERSION_MAJOR
#define MXOS_SDK_VERSION_MAJOR                  (4)
#endif

#ifndef MXOS_SDK_VERSION_MINOR
#define MXOS_SDK_VERSION_MINOR                  (0)
#endif

#ifndef MXOS_SDK_VERSION_REVISION
#define MXOS_SDK_VERSION_REVISION               (0)
#endif

/******************************************************************************
 *                             MXOS Debug Enabler
 ******************************************************************************/

#if !defined MXOS_DEBUG_MIN_LEVEL
#define MXOS_DEBUG_MIN_LEVEL                    MXOS_DEBUG_LEVEL_ALL
#endif

#if !defined MXOS_DEBUG_TYPES_ON
#define MXOS_DEBUG_TYPES_ON                     MXOS_DEBUG_ON
#endif

/******************************************************************************
 *                             MXOS Main Application
 ******************************************************************************/

/**
 *  MXOS_DEFAULT_APPLICATION_STACK_SIZE: Application thread stack size, Default: 1500 bytes
 */
#if !defined MXOS_DEFAULT_APPLICATION_STACK_SIZE
#define MXOS_DEFAULT_APPLICATION_STACK_SIZE     1500
#endif

/**
 *  Do some MXOS initializing before main, like mxos_board_init, stdio uart init...
 */
#if !defined MXOS_APPLICATION
#define MXOS_APPLICATION                        1
#endif

/**
 *  Start standard QC test function other than application
 */
#if !defined MXOS_QUALITY_CONTROL_ENABLE
#define MXOS_QUALITY_CONTROL_ENABLE             0
#endif

/******************************************************************************
 *                             Wlan Configuration
 ******************************************************************************/

#define CONFIG_MODE_NONE                        (1)
#define CONFIG_MODE_USER                        (2)
#define CONFIG_MODE_WAC                         (3)
#define CONFIG_MODE_EASYLINK                    (4)
#define CONFIG_MODE_EASYLINK_WITH_SOFTAP        (4)  //Legacy definition, not supported any more
#define CONFIG_MODE_SOFTAP                      (5)
#define CONFIG_MODE_MONITOR                     (6)
#define CONFIG_MODE_MONITOR_EASYLINK            (7)
#define CONFIG_MODE_WPS                         (8)
#define CONFIG_MODE_AWS                         (9)

/**
 *  MXOS_WLAN_CONFIG_MODE: wlan configuration mode, Default: EasyLink
 */
#if !defined MXOS_WLAN_CONFIG_MODE
#define MXOS_WLAN_CONFIG_MODE                   CONFIG_MODE_AWS
#endif

#if MXOS_WLAN_CONFIG_MODE == CONFIG_MODE_WAC
#define EasyLink_Needs_Reboot
#endif

#if !defined MXOS_WLAN_FORCE_OTA_ENABLE
#define MXOS_WLAN_FORCE_OTA_ENABLE               1
#endif

#if !defined MXOS_WLAN_AUTO_CONFIG
#define MXOS_WLAN_AUTO_CONFIG                    1
#endif


#if !defined MXOS_WLAN_AUTO_SOFTAP_WHEN_DISCONNECTED
#define MXOS_WLAN_AUTO_SOFTAP_WHEN_DISCONNECTED  0
#endif

/**
 *  EasyLink_TimeOut: Easylink configuration timeout, Default: 60 secs
 */
#if !defined EasyLink_TimeOut
#define EasyLink_TimeOut                        60000
#endif

/**
 *  EasyLink_ConnectWlan_Timeout: Connect to wlan after wlan is configured
 *  Restart wlan configuration mode after timeout. Default: 20 seconds.
 */
#if !defined EasyLink_ConnectWlan_Timeout
#define EasyLink_ConnectWlan_Timeout            20000 
#endif

/******************************************************************************
 *                             TCPIP Stack Options
 ******************************************************************************/

/**
 *  MXOS_CONFIG_IP_VER_PREF: On dual stack configuration how long wait for preferred stack
 *  4 or 6
 */

#if !defined MXOS_CONFIG_IP_VER_PREF
#define MXOS_CONFIG_IP_VER_PREF                4
#endif

/**
 *  MXOS_CONFIG_IPV6: Enable IPv4 and IPv6 dual stack apis, Default: disabled
 */
#if !defined MXOS_CONFIG_IPV6
#define MXOS_CONFIG_IPV6                        0
#endif

/**
 *  MXOS_IPV6_NUM_ADDRESSES: Number of IPv6 addresses per interface. Default: 3
 */
#if !defined MXOS_IPV6_NUM_ADDRESSES
#define MXOS_IPV6_NUM_ADDRESSES                 3
#endif

/******************************************************************************
 *        MXOS System Functions, established by mxos_system_init()
 ******************************************************************************/

 /**
 *  MXOS_WLAN_CONNECTION_ENABLE: Start wlan connection when MXOS system starts, 
 *  Default: Enable
 */
#if !defined MXOS_WLAN_CONNECTION_ENABLE
#define MXOS_WLAN_CONNECTION_ENABLE             1
#endif

/**
 *  MXOS_CONFIG_EASYLINK_BTN_ENABLE: Enable EasyLink Button,
 *  - Press to start easylink
 *  - Long pressed  @ref MXOS_CONFIG_EASYLINK_BTN_LONG_PRESS_TIMEOUT milliseconds
 *    to clear all settings
 *  Default: Enable
 */
#if !defined MXOS_CONFIG_EASYLINK_BTN_ENABLE
#define MXOS_CONFIG_EASYLINK_BTN_ENABLE         1
#endif

#if !defined MXOS_CONFIG_EASYLINK_BTN_IDLE_STATE
#define MXOS_CONFIG_EASYLINK_BTN_IDLE_STATE     1
#endif


#if !defined MXOS_CONFIG_EASYLINK_BTN_LONG_PRESS_TIMEOUT
#define MXOS_CONFIG_EASYLINK_BTN_LONG_PRESS_TIMEOUT         5000
#endif

/**
 * Command line interface
 */
#if !defined MXOS_CLI_ENABLE
#define MXOS_CLI_ENABLE                         1
#endif

/**
 * Start a system monitor daemon, application can register some monitor
 * points, If one of these points is not executed in a predefined period,
 * a watchdog reset will occur.
 */
#if !defined MXOS_SYSTEM_MONITOR_ENABLE
#define MXOS_SYSTEM_MONITOR_ENABLE              1
#endif

/**
 * Add service _easylink._tcp._local. for discovery
 */
#if !defined MXOS_SYSTEM_DISCOVERY_ENABLE
#define MXOS_SYSTEM_DISCOVERY_ENABLE              1
#endif

/**
 * _easylink._tcp._local. service port
 */
#if !defined MXOS_SYSTEM_DISCOVERY_PORT
#define MXOS_SYSTEM_DISCOVERY_PORT               8000
#endif

/**
  * MXOS TCP server used for configuration and ota.
  */
#if !defined MXOS_CONFIG_SERVER_ENABLE
#define MXOS_CONFIG_SERVER_ENABLE                0
#endif

#if !defined MXOS_CONFIG_SERVER_PORT
#define MXOS_CONFIG_SERVER_PORT                 8000
#endif

#if !defined MXOS_CONFIG_SERVER_REPORT_SYSTEM_DATA
#define MXOS_CONFIG_SERVER_REPORT_SYSTEM_DATA   MXOS_CONFIG_SERVER_ENABLE
#endif

/******************************************************************************
 *                            Debug and Log
 ******************************************************************************/

#if !defined CONFIG_APP_DEBUG
#define CONFIG_APP_DEBUG                       MXOS_DEBUG_ON
#endif

#if !defined CONFIG_SYSTEM_DEBUG
#define CONFIG_SYSTEM_DEBUG                    MXOS_DEBUG_ON
#endif

#if !defined CONFIG_MDNS_DEBUG
#define CONFIG_MDNS_DEBUG                      MXOS_DEBUG_OFF
#endif

#if !defined CONFIG_LWS_DEBUG
#define CONFIG_LWS_DEBUG                       MXOS_DEBUG_OFF
#endif

#if !defined CONFIG_ETH_DEBUG
#define CONFIG_ETH_DEBUG                       MXOS_DEBUG_OFF
#endif

#if !defined CONFIG_FORCTOTA_DEBUG
#define CONFIG_FORCTOTA_DEBUG                  MXOS_DEBUG_OFF
#endif



/******************************************************************************
 *                            Platform
 ******************************************************************************/

#if !defined PLATFORM_ETH_ENABLE
#define PLATFORM_ETH_ENABLE                                0
#endif

#if !defined PLATFORM_CONFIG_EASYLINK_SOFTAP_COEXISTENCE
#define PLATFORM_CONFIG_EASYLINK_SOFTAP_COEXISTENCE        0
#endif

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif //__MXOS_OPT_H
