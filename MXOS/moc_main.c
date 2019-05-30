/**
 ******************************************************************************
 * @file    mxos_main.c
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This file provide the application main function for MOC platform.
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
#include "mxos_common.h"
#include "moc_api.h"
#include "mxos_platform.h"

/******************************************************
*                      Macros
******************************************************/

#define moc_main_log(M, ...) custom_log("MOC MAIN", M, ##__VA_ARGS__)
#define moc_main_log_trace() custom_log_trace("MOC MAIN")

/******************************************************
*                    Constants
******************************************************/
#ifndef DEV_MODEL
#define DEV_MODEL "MXOS_TEST_PID"
#endif
#ifndef MXOS_SN
#define MXOS_SN  "0000.0000.0004"
#endif

/******************************************************
*                   Enumerations
******************************************************/

/******************************************************
*                 Type Definitions
******************************************************/

/******************************************************
*                    Structures
******************************************************/

/******************************************************
*               Function Declarations
******************************************************/
/* MXOS callback functions, called by MOC kernel */
extern void init_platform( void );
extern int application_start( void );
extern void mxos_rtos_init( void );
extern void ApListCallback( ScanResult *pApList );
extern void ApListAdvCallback( ScanResult_adv *pApAdvList );
extern void WifiStatusHandler( WiFiEvent status );
extern void connected_ap_info( apinfo_adv_t *ap_info, char *key, int key_len );
extern void NetCallback( IPStatusTypedef *pnet );
extern void RptConfigmodeRslt( mwifi_softap_attr_t *nwkpara );
extern void easylink_user_data_result( int datalen, char*data );
extern void socket_connected( int fd );
extern void dns_ip_set( uint8_t *hostname, uint32_t ip );
extern void join_fail( merr_t err );
extern void wifi_reboot_event( void );
extern void mxos_rtos_stack_overflow( char *taskname );

/* MOC main function, called by MOC kernel */
void moc_app_main( const mxos_api_t *lib_api_t );

/******************************************************
*               Variables Definitions
******************************************************/

extern uint32_t app_stack_size;
extern mos_mutex_id_t stdio_tx_mutex;
const mxos_api_t *lib_api_p = NULL;
extern uint32_t _ram_end_;
#ifdef CONFIG_CPU_MX1290
extern const platform_peripherals_pinmap_t peripherals_pinmap;
extern const mhal_gpio_open_t gpio_init[];
#endif

#if defined ( __ICCARM__ )
#pragma location = "user_header_section"
#endif
USED const user_api_t user_handler = {
    .len = 0xFFFFFFFF,
    .reserved = 0xFFFF,
    .crc16 = 0xFFFF,
    .magic_num = 0xC89346,
    .app_stack_size = 4096,
    .interface_version = INTERFACE_VERSION,
    .version = FIRMWARE_REVISION,
    .PID = DEV_MODEL,
    .SN = MXOS_SN,
#ifndef MXOS_DISABLE_STDIO
    .debug_uart = MXOS_STDIO_UART,
    .debug_baudrate = MXOS_STDIO_UART_BAUDRATE,
#else
    .debug_uart = MXOS_UART_NONE,
    .debug_baudrate = 115200,
#endif

    .user_app_in = moc_app_main,
    .init_platform = NULL,
    .application_start = NULL,

    .ApListCallback = ApListCallback,
    .ApListAdvCallback = ApListAdvCallback,
    .WifiStatusHandler = WifiStatusHandler,
    .connected_ap_info = connected_ap_info,
    .NetCallback = NetCallback,
    .RptConfigmodeRslt = RptConfigmodeRslt,
    .easylink_user_data_result = easylink_user_data_result,
    .socket_connected = socket_connected,
    .dns_ip_set = dns_ip_set,
    .join_fail = join_fail,
    .wifi_reboot_event = wifi_reboot_event,
    .mxos_rtos_stack_overflow = mxos_rtos_stack_overflow,
#ifdef CONFIG_CPU_MX1290
    .pinmaps = &peripherals_pinmap,
    .gpio_init = gpio_init,
    .stdio_break_in = 1, // 1=enable: bootloader use user uart to enter boot mode.
#endif
};

#if defined ( __ICCARM__ )
#pragma location=".ram_data_end"
static uint32_t heap_start;

extern void __iar_data_init3(void);
#else
extern uint32_t link_bss_end;
extern void _memory_init( void );
#endif

/******************************************************
*               Function Definitions
******************************************************/
static void pre_main( void )
{
    main( );
    mos_thread_delete( NULL );
}

#ifdef CONFIG_CPU_MX1290
#include "moc_api_sep.h"
extern void init_debug_uart(void);
#endif

void moc_app_main( const mxos_api_t *moc_kernel_apis )
{
#if defined ( __ICCARM__ )
    uint32_t heap_begin = (uint32_t)&heap_start;
    uint32_t size = (uint32_t)(&_ram_end_) - heap_begin;

    __iar_data_init3();
#else
    uint32_t heap_begin = (uint32_t) &link_bss_end;
    uint32_t size = (uint32_t) (&_ram_end_) - heap_begin;

    _memory_init( );
#endif

#ifdef CONFIG_CPU_MX1290
	lib_api_p = moc_adapter((new_mxos_api_t *)moc_kernel_apis);
#else
	lib_api_p = moc_kernel_apis;
#endif
 
    stdio_tx_mutex = mos_mutex_new( );
#ifdef CONFIG_CPU_MX1290
 	init_debug_uart();
#endif
    moc_main_log( "Lib version %s. APP built time %s", lib_api_p->library_version, __TIME__ );
    moc_main_log( "heap reuse from %p, %ld bytes", (void *) heap_begin, size );
    lib_api_p->heap_insert( (uint8_t*) heap_begin, (int) size );

    /* Init nano second clock counter */
    platform_init_nanosecond_clock();

    mos_thread_new( MOS_APPLICATION_PRIORITY, "app_thread", (mos_thread_func_t)pre_main, app_stack_size, NULL );
   
    return;

}



