/**
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 *
 */

#include "mxos.h"
#include "tftp.h"
#include "CheckSumUtils.h"
#include "mxos_system.h"


#define DEFAULT_OTA_AP "MXOS_OTA_AP"
#define DEFAULT_OTA_NETMASK "255.0.0.0"
#define DEFAULT_OTA_SERVER "10.0.0.2"
#define UDP_PORT 20000
#define TCP_PORT 30000

#define FORCE_OTA_SUEECSS 0x00000001
#define FORCE_OTA_NEED    0xfffffffe


#define fota_log(M, ...)       MXOS_LOG(CONFIG_FORCTOTA_DEBUG, "Force OTA", M, ##__VA_ARGS__)


static int wifi_up = 0;
extern void mxos_write_ota_tbl(int len, uint16_t crc);


void wlan_get_mac_address( uint8_t *mac );

enum {
    OTA_SUCCESS = 0,
    OTA_NO_AP = -1,
    OTA_NO_FILE = -2,
    OTA_MD5_FAIL = -3,
    OTA_NO_MEM = -4,
};
/* Call back for OTA finished */
WEAK void mxos_ota_finished(int result, uint8_t *reserved)
{
    switch(result) {
    case OTA_SUCCESS:
        printf("OTA SUCCESS. Rebooting...\r\n");
		mxos_system_context_get( )->mxosSystemConfig.reserved |= FORCE_OTA_SUEECSS;
        mxos_system_context_update(mxos_system_context_get( ));
        mxos_sys_reboot();
        break;
    case OTA_NO_AP:
        printf("OTA FAIL. Can't find the OTA AP\r\n");
        break;
    case OTA_NO_FILE:
        printf("OTA FAIL. Can't find the OTA image\r\n");
        break;
    case OTA_MD5_FAIL:
        printf("OTA FAIL. MD5 check failed\r\n");
        break;
    case OTA_NO_MEM:
        printf("OTA FAIL. Don't have enough memory\r\n");
    default:
        break;
    }
}

static void FOTA_WifiStatusHandler(WiFiEvent event, void * arg)
{
  UNUSED_PARAMETER(arg);
  switch (event) {
  case NOTIFY_STATION_UP:
    wifi_up = 1;
    break;
  case NOTIFY_STATION_DOWN:
    wifi_up = 0;
    break;
 
  default:
    break;
  }

  return;
}

/* connect to AP: ssid="mxos_ota_ap", security=OPEN.
  * Broadcast to find OTA server
  * Connect to OTA server, request to OTA.
  */
void tftp_ota(void)
{
    network_InitTypeDef_st conf;
    tftp_file_info_t fileinfo;
    uint32_t ipaddr = inet_addr(DEFAULT_OTA_SERVER), flashaddr;
    int filelen, maxretry = 5, len, left, i = 0;
    uint8_t md5_recv[16];
    uint8_t md5_calc[16];
    uint8_t *tmpbuf;
    md5_context ctx;
    uint8_t mac[6], sta_ip_addr[16];
    mxos_logic_partition_t* ota_partition = mxos_flash_get_info( MXOS_PARTITION_OTA_TEMP );
    uint16_t crc = 0;
    CRC16_Context contex;
    
#define TMP_BUF_LEN 1024

    fota_log("Start OTA");
    mxos_system_notify_remove_all(mxos_notify_WIFI_STATUS_CHANGED);
    mxos_system_notify_remove_all(mxos_notify_WiFI_PARA_CHANGED);
    mxos_system_notify_remove_all(mxos_notify_DHCP_COMPLETED);
    mxos_system_notify_remove_all(mxos_notify_WIFI_CONNECT_FAILED);
	  mxos_system_notify_remove_all(mxos_notify_EASYLINK_WPS_COMPLETED);
    mxos_system_notify_register( mxos_notify_WIFI_STATUS_CHANGED, (void *)FOTA_WifiStatusHandler, NULL );
    mxosWlanStopEasyLink();
	  mxosWlanStopEasyLinkPlus();
    mxosWlanStopAirkiss();
    mxosWlanSuspendStation();
	mxos_rtos_thread_msleep(10);
		
    tmpbuf = (uint8_t*)malloc(TMP_BUF_LEN);
    if (tmpbuf == NULL) {
        fota_log("ERROR!! Can't get enough memory");
        mxos_ota_finished(OTA_NO_MEM, NULL);
        return;
    }
    
    wlan_get_mac_address(mac);
    
    sprintf((char *)sta_ip_addr, "10.%d.%d.%d", 
        mac[3], mac[4], mac[5]);
        
    fota_log("Staic IP = %s", sta_ip_addr);  
    
    memset(&conf, 0, sizeof(network_InitTypeDef_st));
    
    conf.wifi_mode = Station;
    strcpy(conf.wifi_ssid, DEFAULT_OTA_AP);
    
    conf.dhcpMode = DHCP_Disable;
    strcpy(conf.net_mask, DEFAULT_OTA_NETMASK);
    strcpy(conf.local_ip_addr, (char *)sta_ip_addr);
    
    wifi_up = 0;
    fota_log("Connect to AP %s...", DEFAULT_OTA_AP);
    mxosWlanStart(&conf);

    while(wifi_up == 0) {
        mxos_rtos_thread_msleep(100);
        i++;
        if (i > 100) {
            fota_log("ERROR!! Can't find the OTA AP");
            mxos_ota_finished(OTA_NO_AP, NULL);
            return;
        }
    }
    fota_log("AP connected, tftp download image... to 0x%lx", ota_partition->partition_start_addr);

    fileinfo.filelen = ota_partition->partition_length;
    fileinfo.flashaddr = 0;
    fileinfo.flashtype = MXOS_PARTITION_OTA_TEMP;
    strcpy(fileinfo.filename, "mxos_ota.bin");

    while((filelen = tget (&fileinfo, ipaddr)) < 0) {
        fota_log("tget return filelen %d, maxretry %d", filelen, maxretry);
        maxretry--;
        if (maxretry < 0) {
            fota_log("ERROR!! Can't get OTA image.");
            free(tmpbuf);
            mxos_ota_finished(OTA_NO_FILE, NULL);
            return;
        }
    }

    filelen -= 16; // remove md5.
    fota_log("tftp download image finished, OTA bin len %d", filelen);
    flashaddr = filelen;
    mxos_flash_read(MXOS_PARTITION_OTA_TEMP, &flashaddr, (uint8_t *)md5_recv, 16);
    InitMd5( &ctx );
    CRC16_Init( &contex );
    flashaddr = 0;
    left = filelen;
    while(left > 0) {
        if (left > TMP_BUF_LEN) {
            len = TMP_BUF_LEN;
        } else {
            len = left;
        }
        left -= len;
        mxos_flash_read(MXOS_PARTITION_OTA_TEMP, &flashaddr, (uint8_t *)tmpbuf, len);
        Md5Update( &ctx, (uint8_t *)tmpbuf, len);
        CRC16_Update( &contex, tmpbuf, len );
    }
    Md5Final( &ctx, md5_calc );
    CRC16_Final( &contex, &crc );
    
    if(memcmp(md5_calc, md5_recv, 16) != 0) {
        fota_log("ERROR!! MD5 Error.");
        fota_log("RX:   %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                 md5_recv[0],md5_recv[1],md5_recv[2],md5_recv[3],
                 md5_recv[4],md5_recv[5],md5_recv[6],md5_recv[7],
                 md5_recv[8],md5_recv[9],md5_recv[10],md5_recv[11],
                 md5_recv[12],md5_recv[13],md5_recv[14],md5_recv[15]);
        fota_log("Need: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                 md5_calc[0],md5_calc[1],md5_calc[2],md5_calc[3],
                 md5_calc[4],md5_calc[5],md5_calc[6],md5_calc[7],
                 md5_calc[8],md5_calc[9],md5_calc[10],md5_calc[11],
                 md5_calc[12],md5_calc[13],md5_calc[14],md5_calc[15]);
        mxos_ota_finished(OTA_MD5_FAIL, NULL);
        return;
    }

    fota_log("OTA bin md5 check success, CRC %x. upgrading...", crc);
    mxos_ota_switch_to_new_fw( filelen, crc );
    mxos_ota_finished(OTA_SUCCESS, NULL);
    while(1)
        mxos_rtos_thread_sleep(100);
}

/******************************************************
 *              MXOS_FORCE_OTA
 *****************************************************/
mxos_semaphore_t force_ota_sem;

static void force_thread(void * arg){
    extern void tftp_ota();
    tftp_ota();
}

merr_t start_force_ota()
{
   merr_t err;

   require_action_string( mos_thread_new( MXOS_APPLICATION_PRIORITY, "Force OTA", force_thread, 0x1000, NULL ) != NULL, 
   exit, err = kGeneralErr, "ERROR: Unable to start the  force ota thread." );

   exit:
           return err;

}
static void mxosNotify_ApListCallback(ScanResult *pApList, mxos_Context_t * const inContext)
{
	fota_log("ota notify");
    (void)inContext;

    if(pApList->ApNum == 0){
        if(NULL != force_ota_sem)
        {
        	fota_log("set force_ota_sem");
            mxos_rtos_set_semaphore(&force_ota_sem);
        }
    }else{
    	fota_log("num = %d,ssid = %s",pApList->ApNum,pApList->ApList->ssid);
    	fota_log("start_force_ota");
        start_force_ota();
    }
}

merr_t start_forceota_check()
{
	merr_t err = kNoErr;
	if((mxos_system_context_get( )->mxosSystemConfig.reserved & FORCE_OTA_SUEECSS)==0)
	{
		#define FORCE_OTA_AP "MXOS_OTA_AP"
		fota_log("force ota ssid :%s",FORCE_OTA_AP);
		/* Register user function when wlan scan is completed */
		err =  mxos_system_notify_register( mxos_notify_WIFI_SCAN_COMPLETED, (void *)mxosNotify_ApListCallback, NULL );
		require_noerr( err, exit );
		fota_log("Start scan");
		mxos_rtos_init_semaphore(&force_ota_sem,1);
		mxchip_active_scan(FORCE_OTA_AP,0);
		err = mxos_rtos_get_semaphore(&force_ota_sem,MXOS_WAIT_FOREVER);
		if(NULL != force_ota_sem)
		mxos_rtos_deinit_semaphore(&force_ota_sem);
		err = mxos_system_notify_remove( mxos_notify_WIFI_SCAN_COMPLETED, (void *)mxosNotify_ApListCallback);
		require_noerr( err, exit );
	}
	else
	{
		mxos_system_context_get( )->mxosSystemConfig.reserved &= FORCE_OTA_NEED;
		mxos_system_context_update(mxos_system_context_get( ));
	}
	exit:
	return err;
}



