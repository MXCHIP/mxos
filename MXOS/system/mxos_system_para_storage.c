/**
 ******************************************************************************
 * @file    mxos_system_para_storage.c
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This file provide functions to read and write configuration data on
 *          nonvolatile memory.
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
#include "mxos_system.h"
#include "system_internal.h"
#include "CheckSumUtils.h"
#include "mkv.h"

/* Update seed number every time*/
static int32_t seedNum = 0;

#define SYS_CONFIG_SIZE     ( sizeof(system_config_t) - sizeof( boot_table_t ) )
#define CRC_SIZE      ( 2 )

system_context_t* sys_context = NULL;
static mos_mutex_id_t para_flash_mutex = NULL;
//#define para_log(M, ...) custom_log("MXOS Settting", M, ##__VA_ARGS__)

#define para_log(M, ...)

WEAK void appRestoreDefault_callback(void *user_data, uint32_t size)
{

}

#ifndef OFFSETOF
#define OFFSETOF( type, member )  ( (uintptr_t)&((type *)0)->member )
#endif /* OFFSETOF */

static const uint32_t mxos_context_section_offsets[ ] =
{
    [PARA_BOOT_TABLE_SECTION]            = OFFSETOF( system_config_t, bootTable ),
    [PARA_MXOS_DATA_SECTION]             = OFFSETOF( system_config_t, mxosSystemConfig ),
#ifdef MXOS_BLUETOOTH_ENABLE
    [PARA_BT_DATA_SECTION]               = OFFSETOF( system_config_t, bt_config ),
#endif
    [PARA_SYS_END_SECTION]               = sizeof( system_config_t ),
    [PARA_APP_DATA_SECTION]              = sizeof( system_config_t ),
};

mxos_Context_t* mxos_system_context_init( void )
{
  void *user_config_data = NULL;

  if (mkv_init() != 0)
    return NULL;

  if( sys_context !=  NULL) {
    free( sys_context );
    sys_context = NULL;
  }

  sys_context = calloc( 1, sizeof(system_context_t) );
  require( sys_context, exit );

  para_log( "Init context: len=%d", sizeof(system_context_t));

  sys_context->flashContentInRam_mutex = mos_mutex_new( );
  para_flash_mutex = mos_mutex_new( );
  MXOSReadConfiguration( sys_context );

  if(sys_context->flashContentInRam.mxosSystemConfig.magic_number != SYS_MAGIC_NUMBR){
    para_log("Magic number error, restore to default");
#ifdef MFG_MODE_AUTO
    MICORestoreMFG( );
#else
    mxos_system_context_restore( (mxos_Context_t *)sys_context );
#endif
  }

exit:
  return &sys_context->flashContentInRam;
}

mxos_Context_t* mxos_system_context_get( void )
{
  return &sys_context->flashContentInRam;
}

#define MKV_ITEM_SET(name) \
do \
{ \
  require(mkv_item_set(#name, &sys_config->name, sizeof(sys_config->name)) == 0, exit); \
} while (0)

static merr_t internal_update_config( system_context_t * const inContext )
{
  merr_t err = kGeneralErr;
  mxos_sys_config_t *sys_config = &inContext->flashContentInRam.mxosSystemConfig;

  /*Device identification*/
  MKV_ITEM_SET(name);
  /*Wi-Fi configuration*/
  MKV_ITEM_SET(ssid);
  MKV_ITEM_SET(user_key);
  MKV_ITEM_SET(user_keyLength);
  MKV_ITEM_SET(key); 
  MKV_ITEM_SET(keyLength);
  MKV_ITEM_SET(bssid);
  MKV_ITEM_SET(channel);
  MKV_ITEM_SET(security);
  /*Power save configuration*/
  MKV_ITEM_SET(rfPowerSaveEnable);
  MKV_ITEM_SET(mcuPowerSaveEnable);
  /*Local IP configuration*/
  MKV_ITEM_SET(dhcpEnable);
  MKV_ITEM_SET(localIp);
  MKV_ITEM_SET(netMask);
  MKV_ITEM_SET(gateWay);
  MKV_ITEM_SET(dnsServer);
  /*EasyLink configuration*/
  MKV_ITEM_SET(configured);
  MKV_ITEM_SET(easyLinkByPass);
  /*Services in MXOS system*/
  MKV_ITEM_SET(magic_number);
  /*Update seed number when configuration is changed*/
  MKV_ITEM_SET(seed);

  err = kNoErr;
  
exit:
  return err;
}

merr_t mxos_system_context_restore( mxos_Context_t * const inContext )
{ 
  merr_t err = kNoErr;
  require_action( inContext, exit, err = kNotPreparedErr );

  /*wlan configration is not need to change to a default state, use easylink to do that*/
  memset(&sys_context->flashContentInRam, 0x0, sizeof(system_config_t));
  sprintf(sys_context->flashContentInRam.mxosSystemConfig.name, DEFAULT_NAME);
  sys_context->flashContentInRam.mxosSystemConfig.configured = unConfigured;
  sys_context->flashContentInRam.mxosSystemConfig.easyLinkByPass = EASYLINK_BYPASS_NO;
  sys_context->flashContentInRam.mxosSystemConfig.rfPowerSaveEnable = false;
  sys_context->flashContentInRam.mxosSystemConfig.mcuPowerSaveEnable = false;
  sys_context->flashContentInRam.mxosSystemConfig.magic_number = SYS_MAGIC_NUMBR;
  sys_context->flashContentInRam.mxosSystemConfig.seed = seedNum;
#ifdef MXOS_BLUETOOTH_ENABLE
  memset(&sys_context->flashContentInRam.bt_config, 0xFF, sizeof(mxos_bt_config_t));
#endif

  para_log("Restore to default");

exit:
  return err;
}

#ifdef MFG_MODE_AUTO
merr_t MXOSRestoreMFG( void )
{ 
  merr_t err = kNoErr;
  require_action( sys_context, exit, err = kNotPreparedErr );

  /*wlan configration is not need to change to a default state, use easylink to do that*/
  sprintf(sys_context->flashContentInRam.mxosSystemConfig.name, DEFAULT_NAME);
  sys_context->flashContentInRam.mxosSystemConfig.configured = mfgConfigured;
  sys_context->flashContentInRam.mxosSystemConfig.magic_number = SYS_MAGIC_NUMBR;

  /*Application's default configuration*/
  appRestoreDefault_callback();

  err = internal_update_config( sys_context );
  require_noerr(err, exit);

exit:
  return err;
}
#endif

#define MKV_ITEM_GET(name) \
do \
{ \
  n = sizeof(sys_config->name); \
  mkv_item_get(#name, &sys_config->name, &n); \
} while (0)

merr_t MXOSReadConfiguration(system_context_t *inContext)
{
  int n;
  mxos_sys_config_t *sys_config = &inContext->flashContentInRam.mxosSystemConfig;

  /*Device identification*/
  MKV_ITEM_GET(name);
  /*Wi-Fi configuration*/
  MKV_ITEM_GET(ssid);
  MKV_ITEM_GET(user_key);
  MKV_ITEM_GET(user_keyLength);
  MKV_ITEM_GET(key); 
  MKV_ITEM_GET(keyLength);
  MKV_ITEM_GET(bssid);
  MKV_ITEM_GET(channel);
  MKV_ITEM_GET(security);
  /*Power save configuration*/
  MKV_ITEM_GET(rfPowerSaveEnable);
  MKV_ITEM_GET(mcuPowerSaveEnable);
  /*Local IP configuration*/
  MKV_ITEM_GET(dhcpEnable);
  MKV_ITEM_GET(localIp);
  MKV_ITEM_GET(netMask);
  MKV_ITEM_GET(gateWay);
  MKV_ITEM_GET(dnsServer);
  /*EasyLink configuration*/
  MKV_ITEM_GET(configured);
  MKV_ITEM_GET(easyLinkByPass);
  /*Services in MXOS system*/
  MKV_ITEM_GET(magic_number);
  /*Update seed number when configuration is changed*/
  MKV_ITEM_GET(seed);

  return kNoErr;
}

merr_t mxos_system_context_update( mxos_Context_t *in_context )
{
  merr_t err = kNoErr;
  require_action( in_context, exit, err = kNotPreparedErr );

  sys_context->flashContentInRam.mxosSystemConfig.seed = ++seedNum;

  err = internal_update_config( sys_context );
  require_noerr(err, exit);

exit:
  return err;
}





//static mxos_Context_t * inContext=0;
/******************************************************
 *               Function Definitions
 ******************************************************/

merr_t mxos_ota_switch_to_new_fw( int ota_data_len, uint16_t ota_data_crc )
{
    mxos_Context_t *mxos_context = mxos_system_context_get();
#ifdef MXOS_ENABLE_SECONDARY_APPLICATION
    UNUSED_PARAMETER( ota_data_len );
    UNUSED_PARAMETER( ota_data_crc );
    extern int switch_active_firmware(void);
    switch_active_firmware();
#else
    mxos_logic_partition_t* ota_partition = mhal_flash_get_info( MXOS_PARTITION_OTA_TEMP );

    memset( &sys_context->flashContentInRam.bootTable, 0, sizeof(boot_table_t) );
#ifdef CONFIG_MX108
    sys_context->flashContentInRam.bootTable.dst_adr = 0x13200;
    sys_context->flashContentInRam.bootTable.src_adr = ota_partition->partition_start_addr;
    sys_context->flashContentInRam.bootTable.siz = ota_data_len;
    sys_context->flashContentInRam.bootTable.crc = ota_data_crc;
#else
    sys_context->flashContentInRam.bootTable.length = ota_data_len;
    sys_context->flashContentInRam.bootTable.start_address = ota_partition->partition_start_addr;
    sys_context->flashContentInRam.bootTable.type = 'A';
    sys_context->flashContentInRam.bootTable.upgrade_type = 'U';
    sys_context->flashContentInRam.bootTable.crc = ota_data_crc;
#endif
    mxos_system_context_update( mxos_context );
#endif
    return kNoErr;
}
