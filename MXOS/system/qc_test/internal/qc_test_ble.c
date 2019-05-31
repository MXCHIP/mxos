/**
 ******************************************************************************
 * @file    qc_test_ble.c
 * @author  William Xu
 * @version V1.0.0
 * @date    18-Dec-2016
 * @brief   This file provide the BLE QC test function.
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

#ifdef QC_TEST_BLUETOOTH_ENABLE

#include "qc_test_internal.h"

#include "mxos_bt.h"
#include "mxos_bt_cfg.h"
#include "mxos_bt_dev.h"
#include "mxos_bt_smart_interface.h"
#include "mxos_bt_smartbridge.h"
#include "mxos_bt_smartbridge_gatt.h"

#include "StringUtils.h"

/******************************************************
 *               Function Declarations
 ******************************************************/

static void ble_scan( void );

/******************************************************
 *               Function Definitions
 ******************************************************/

void qc_test_ble( void )
{
    char str[128];
    uint8_t mac[6];

    mxos_bt_init( MXOS_BT_HCI_MODE, "SmartBridge Device", 0, 0 );  //Client + server connections
    mxos_bt_smartbridge_init( 0 );

    mxos_bt_dev_read_local_addr( mac );
    sprintf( str, "%02X-%02X-%02X-%02X-%02X-%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5] );
    QC_TEST_PRINT_STRING( "Local Bluetooth Address:", str );

    ble_scan( );
}

/* Scan complete handler. Scan complete event reported via this callback.
 * It runs on the MXOS_BT_WORKER_THREAD context.
 */
static merr_t scan_complete_handler( void *arg )
{
    UNUSED_PARAMETER( arg );
    merr_t err = kNoErr;
    uint32_t count = 0;
    mxos_bt_smart_scan_result_t *scan_result = NULL;

    mf_printf( "BLE scan complete\r\n" );
    err = mxos_bt_smartbridge_get_scan_result_list( &scan_result, &count );
    require_noerr( err, exit );

    if ( count == 0 )
    {
        mf_printf( "No ble device found\r\n" );
        err = kNotFoundErr;
        goto exit;
    }
    mf_printf( "\r\n" );
exit:
    /* Scan duration is complete */
    return err;
}

static merr_t ble_scan_handler( const mxos_bt_smart_advertising_report_t* result )
{
    merr_t err = kNoErr;
    char* bd_addr_str = NULL;
    char str[128];

    bd_addr_str = DataToHexStringWithColons( (uint8_t *) result->remote_device.address, 6 );
    snprintf( str, 128, "  ADDR: %s, RSSI: %d", bd_addr_str, result->signal_strength );
    mf_printf( str );
    free( bd_addr_str );
    mf_printf( "\r\n" );
    
    /* Scan duration is complete */
    return err;
}

void ble_scan( void )
{
    uint32_t count = 0;
    mxos_bt_smart_scan_result_t *scan_result = NULL;
    /* Scan settings */
    mxos_bt_smart_scan_settings_t scan_settings;

    scan_settings.type = BT_SMART_PASSIVE_SCAN;
    scan_settings.filter_policy = FILTER_POLICY_NONE;
    scan_settings.filter_duplicates = DUPLICATES_FILTER_ENABLED;
    scan_settings.interval = MXOS_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_INTERVAL;
    scan_settings.window = MXOS_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_WINDOW;
    scan_settings.duration_second = 2;
    scan_settings.type = BT_SMART_PASSIVE_SCAN;

    /* Start scan */
    mxos_bt_smartbridge_start_scan( &scan_settings, scan_complete_handler, ble_scan_handler );

    mos_sleep_ms( 2 * 1000 );

    mxos_bt_smartbridge_stop_scan( );

    mf_printf( "BLE scan complete\r\n" );
    mxos_bt_smartbridge_get_scan_result_list( &scan_result, &count );

    if ( count == 0 )
    {
        mf_printf( "No BLE device found\r\n" );
    }

    mf_printf( "\r\n" );
}

#endif
