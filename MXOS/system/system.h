/**
 ******************************************************************************
 * @file    system.h
 * @author  William Xu
 * @version V1.0.0
 * @date    22-July-2015
 * @brief   This file provide internal function prototypes for for mxos system.
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

#pragma once

#include "mxos_opt.h"
#include "mxos_common.h"
#include "mxos_rtos.h"
#include "mxos_wlan.h"


#ifndef MXOS_PREBUILT_LIBS
#include "mxos_board_conf.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define maxSsidLen          32
#define maxKeyLen           64
#define maxNameLen          32
#define maxIpLen            16

typedef enum
{
  eState_Normal,
  eState_Software_Reset,
  eState_Wlan_Powerdown,
  eState_Restore_default,
  eState_Standby,
} system_state_t;

enum  config_state_type_e{
  /*All settings are in default state, module will enter easylink mode if WIFI_CONFIG_MODE_TRIGGER_AUTO is selected.
  Press down Easyink button for 3 seconds (defined by RestoreDefault_TimeOut) to enter this mode */
  unConfigured,
#ifdef EasyLink_Needs_Reboot
  /*Module will enter easylink mode temporally when powered on, and go back to allConfigured
    mode after time out (Defined by EasyLink_TimeOut), This mode is used for changing wlan
    settings if module is moved to a new wlan environment. Press down Easyink button to
    enter this mode */                
  wLanUnConfigured,
#endif
  /*Normal working mode, module use the configured settings to connect to wlan, and run
    user's threads*/
  allConfigured,
  /*If MFG_MODE_AUTO is enabled and MXOS settings are erased (maybe a fresh device just has 
    been programmed or MXOS settings is damaged), module will enter MFG mode when powered on. */
  mfgConfigured,
};
typedef uint8_t config_state_type_t;

/* OTA should save this table to flash */
typedef struct  _boot_table_t {
#ifdef CONFIG_MX108
  uint32_t dst_adr;
  uint32_t src_adr;
  uint32_t siz;
  uint16_t crc;
#else
  uint32_t start_address; // the address of the bin saved on flash.
  uint32_t length; // file real length
  uint8_t version[8];
  uint8_t type; // B:bootloader, P:boot_table, A:application, D: 8782 driver
  uint8_t upgrade_type; //u:upgrade, 
  uint16_t crc;
  uint8_t reserved[4];
#endif
}boot_table_t;

typedef struct _mxos_config_t
{
  /*Device identification*/
  char            name[maxNameLen];

  /*Wi-Fi configuration*/
  char            ssid[maxSsidLen];
  char            user_key[maxKeyLen]; 
  int             user_keyLength;
  char            key[maxKeyLen]; 
  int             keyLength;
  char            bssid[6];
  int             channel;
  wlan_sec_type_t security;

  /*Power save configuration*/
  bool            rfPowerSaveEnable;
  bool            mcuPowerSaveEnable;

  /*Local IP configuration*/
  bool            dhcpEnable;
  char            localIp[maxIpLen];
  char            netMask[maxIpLen];
  char            gateWay[maxIpLen];
  char            dnsServer[maxIpLen];

  /*EasyLink configuration*/
  config_state_type_t   configured;
  uint32_t              reserved;

  /*Services in MXOS system*/
  uint32_t        magic_number;

  /*Update seed number when configuration is changed*/
  int32_t         seed;
} mxos_config_t;


#ifdef MXOS_BLUETOOTH_ENABLE
#define MXOS_BT_PARA_LOCAL_KEY_DATA  65  /* BTM_SECURITY_LOCAL_KEY_DATA_LEN */

#define MXOS_BT_DCT_NAME            249
#define MXOS_BT_DCT_MAX_KEYBLOBS    146   /* Maximum size of key blobs to be stored :=  size of BR-EDR link keys +  size of BLE keys*/
#define MXOS_BT_DCT_ADDR_FIELD      6
#define MXOS_BT_DCT_LENGTH_FIELD    2
#ifndef MXOS_BT_DCT_MAX_DEVICES
#define MXOS_BT_DCT_MAX_DEVICES     10    /* Maximum number of device records stored in nvram */
#endif
#define MXOS_BT_DCT_ADDR_TYPE       1
#define MXOS_BT_DCT_DEVICE_TYPE     1

/* Length of BD_ADDR + 2bytes length field */
#define MXOS_BT_DCT_ENTRY_HDR_LENGTH  (MXOS_BT_DCT_ADDR_FIELD + MXOS_BT_DCT_LENGTH_FIELD + MXOS_BT_DCT_ADDR_TYPE + MXOS_BT_DCT_DEVICE_TYPE)

#define MXOS_BT_DCT_LOCAL_KEY_OFFSET  OFFSETOF( mxos_bt_config_t, bluetooth_local_key )
#define MXOS_BT_DCT_REMOTE_KEY_OFFSET OFFSETOF( mxos_bt_config_t, bluetooth_remote_key )

#pragma pack(1)
typedef struct
{
//    uint8_t bluetooth_local_addeess[6];
//    uint8_t bluetooth_local_name[249]; /* including null termination */
    uint8_t bluetooth_local_key[MXOS_BT_PARA_LOCAL_KEY_DATA];
    uint8_t bluetooth_remote_key[MXOS_BT_DCT_ENTRY_HDR_LENGTH + MXOS_BT_DCT_MAX_KEYBLOBS][MXOS_BT_DCT_MAX_DEVICES];
    uint8_t padding[1];   /* to ensure 32-bit aligned size */
} mxos_bt_config_t;
#pragma pack()
#endif


typedef struct {

  /*OTA options*/
  boot_table_t             bootTable;

  /*MXOS system core configuration*/
  mxos_config_t            mxos_config;

#ifdef MXOS_BLUETOOTH_ENABLE
  mxos_bt_config_t         bt_config; 
#endif
} system_config_t;

typedef struct
{
  system_state_t        current_sys_state;
  mos_semphr_id_t      sys_state_change_sem;
  /*MXOS system Running status*/
  char                  localIp[maxIpLen];
  char                  netMask[maxIpLen];
  char                  gateWay[maxIpLen];
  char                  dnsServer[maxIpLen];
  char                  mac[18];
  char                  rf_version[50];
} system_status_wlan_t;


#ifdef __cplusplus
} /*extern "C" */
#endif

