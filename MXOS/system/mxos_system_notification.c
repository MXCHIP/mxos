/**
 ******************************************************************************
 * @file    mxos_system_notification.c
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This file provide functions for add or remove custom functions to MXOS
 *          notifucations
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
#include "mxos.h"


typedef struct _Notify_list{
  void  *function;
  void  *arg;
  struct _Notify_list *next;
  void  *contex;
} _Notify_list_t;

_Notify_list_t* Notify_list[mxos_notify_MAX] = {NULL};

/* MXOS system defined notifications */
typedef void (*mxos_notify_WIFI_SCAN_COMPLETE_function)           ( int num, mwifi_ap_info_t *ap_list, void * inContext );
typedef void (*mxos_notify_WIFI_STATUS_CHANGED_function)          ( uint8_t status, void * inContext );
typedef void (*mxos_notify_WiFI_PARA_CHANGED_function)            ( mwifi_link_info_t *info, char *key, int key_len, void * inContext );
typedef void (*mxos_notify_EASYLINK_COMPLETE_function)            ( char *ssid, char *key, int mode, void * inContext );
typedef void (*mxos_notify_EASYLINK_GET_EXTRA_DATA_function)      ( int datalen, char*data, void * inContext );
typedef void (*mxos_notify_TCP_CLIENT_CONNECTED_function)         ( int fd, void * inContext );
typedef void (*mxos_notify_DNS_RESOLVE_COMPLETED_function)        ( uint8_t *hostname, uint32_t ip, void * inContext );
typedef void (*mxos_notify_READ_APP_INFO_function)                ( char *str, int len, void * inContext );
typedef void (*mxos_notify_SYS_WILL_POWER_OFF_function)           ( void * inContext );
typedef void (*mxos_notify_WIFI_CONNECT_FAILED_function)          ( merr_t err, void * inContext );
typedef void (*mxos_notify_WIFI_FATAL_ERROR_function)             ( void * inContext );
typedef void (*mxos_notify_STACK_OVERFLOW_ERROR_function)         ( char *taskname, void * const inContext );
typedef void (*mxos_notify_GPRS_STATUS_CHANGED_function)          ( notify_netif_status_t status, const mxos_gprs_net_addr_t* pnet );

netif_status_t netif_status[INTERFACE_MAX] = {INTERFACE_STATUS_DOWN, INTERFACE_STATUS_DOWN, INTERFACE_STATUS_DOWN};

/* User defined notifications */
void mwifi_scan_results_cb(int num, mwifi_ap_info_t *ap_list)
{
  _Notify_list_t *temp =  Notify_list[mxos_notify_WIFI_SCAN_COMPLETED];
  if(temp == NULL)
    return;
  else{
    do{
      ((mxos_notify_WIFI_SCAN_COMPLETE_function)(temp->function))(num, ap_list, temp->arg);
      temp = temp->next;
    }while(temp!=NULL);
  }
}

void mwifi_status_cb(uint8_t status)
{
    switch ( status )
    {
        case NOTIFY_STATION_UP:
            netif_status[INTERFACE_STA] = INTERFACE_STATUS_UP;
            break;
        case NOTIFY_STATION_DOWN:
            netif_status[INTERFACE_STA] = INTERFACE_STATUS_DOWN;
            break;
        case NOTIFY_AP_UP:
            netif_status[INTERFACE_UAP] = INTERFACE_STATUS_UP;
            break;
        case NOTIFY_AP_DOWN:
            netif_status[INTERFACE_UAP] = INTERFACE_STATUS_DOWN;
            break;
        case NOTIFY_ETH_UP:
            netif_status[INTERFACE_ETH] = INTERFACE_STATUS_UP;
            break;
        case NOTIFY_ETH_DOWN:
            netif_status[INTERFACE_ETH] = INTERFACE_STATUS_DOWN;
            break;
        default:
            break;
    }

    mxos_network_switch_interface_auto( );

  _Notify_list_t *temp =  Notify_list[mxos_notify_WIFI_STATUS_CHANGED];
  if(temp == NULL)
    return;
  else{
    do{
      ((mxos_notify_WIFI_STATUS_CHANGED_function)(temp->function))(status, temp->arg);
      temp = temp->next;
    }while(temp!=NULL);
  }
}

void mwifi_connected_ap_info_cb(mwifi_link_info_t *info, char *key, int key_len)
{
  _Notify_list_t *temp =  Notify_list[mxos_notify_WiFI_PARA_CHANGED];
  if(temp == NULL)
    return;
  else{
    do{
      ((mxos_notify_WiFI_PARA_CHANGED_function)(temp->function))(info, key, key_len, temp->arg);
      temp = temp->next;
    }while(temp!=NULL);
  }
}

void RptConfigmodeRslt(char *ssid, char *key, int mode)
{
  _Notify_list_t *temp =  Notify_list[mxos_notify_EASYLINK_WPS_COMPLETED];
  if(temp == NULL)
    return;
  else{
    do{
      ((mxos_notify_EASYLINK_COMPLETE_function)(temp->function))(ssid, key, mode, temp->arg);
      temp = temp->next;
    }while(temp!=NULL);
  }
}

void easylink_user_data_result(int datalen, char*data)
{
  _Notify_list_t *temp =  Notify_list[mxos_notify_EASYLINK_GET_EXTRA_DATA];
  if(temp == NULL)
    return;
  else{
    do{
      ((mxos_notify_EASYLINK_GET_EXTRA_DATA_function)(temp->function))(datalen, data, temp->arg);
      temp = temp->next;
    }while(temp!=NULL);
  }  
}

void socket_connected(int fd)
{
  _Notify_list_t *temp =  Notify_list[mxos_notify_TCP_CLIENT_CONNECTED];
  if(temp == NULL)
    return;
  else{
    do{
      ((mxos_notify_TCP_CLIENT_CONNECTED_function)(temp->function))(fd, temp->arg);
      temp = temp->next;
    }while(temp!=NULL);
  }    
}

void dns_ip_set(uint8_t *hostname, uint32_t ip)
{
  _Notify_list_t *temp =  Notify_list[mxos_notify_DNS_RESOLVE_COMPLETED];
  if(temp == NULL)
    return;
  else{
    do{
      ((mxos_notify_DNS_RESOLVE_COMPLETED_function)(temp->function))(hostname, ip, temp->arg);
      temp = temp->next;
    }while(temp!=NULL);
  }    
}

void sendNotifySYSWillPowerOff(void)
{
  _Notify_list_t *temp =  Notify_list[mxos_notify_SYS_WILL_POWER_OFF];
  if(temp == NULL)
    return;
  else{
    do{
      ((mxos_notify_SYS_WILL_POWER_OFF_function)(temp->function))(temp->arg);
      temp = temp->next;
    }while(temp!=NULL);
  }    
}

void join_fail(merr_t err)
{
  _Notify_list_t *temp =  Notify_list[mxos_notify_WIFI_CONNECT_FAILED];
  if(temp == NULL)
    return;
  else{
    do{
      ((mxos_notify_WIFI_CONNECT_FAILED_function)(temp->function))(err, temp->arg);
      temp = temp->next;
    }while(temp!=NULL);
  }    
}

void wifi_reboot_event(void)
{
  _Notify_list_t *temp =  Notify_list[mxos_notify_WIFI_Fatal_ERROR];
  if(temp == NULL)
    return;
  else{
    do{
      ((mxos_notify_WIFI_FATAL_ERROR_function)(temp->function))(temp->arg);
      temp = temp->next;
    }while(temp!=NULL);
  }    
}

void mxos_rtos_stack_overflow(char *taskname)
{
  _Notify_list_t *temp =  Notify_list[mxos_notify_Stack_Overflow_ERROR];
  if(temp == NULL)
    return;
  else{
    do{
      ((mxos_notify_STACK_OVERFLOW_ERROR_function)(temp->function))(taskname, temp->arg);
      temp = temp->next;
    }while(temp!=NULL);
  }    
}

void mxos_gprs_status_handler(notify_netif_status_t status, const mxos_gprs_net_addr_t *net_addr)
{
  _Notify_list_t *temp =  Notify_list[mxos_notify_GPRS_STATUS_CHANGED];
  if(temp == NULL)
    return;
  else{
    do{
      ((mxos_notify_GPRS_STATUS_CHANGED_function)(temp->function))(status, net_addr);
      temp = temp->next;
    }while(temp!=NULL);
  }  
}

merr_t mxos_system_notify_register( mxos_notify_types_t notify_type, void* functionAddress, void* arg )
{
  merr_t err = kNoErr;
  _Notify_list_t *temp =  Notify_list[notify_type];
  _Notify_list_t *notify = (_Notify_list_t *)malloc(sizeof(_Notify_list_t));
  require_action(notify, exit, err = kNoMemoryErr);
  notify->function = functionAddress;
  notify->arg = arg;
  notify->next = NULL;
  if(Notify_list[notify_type] == NULL){
    Notify_list[notify_type] = notify;
    notify->next = NULL;
  }else{
    if(temp->function == functionAddress)
        return kNoErr;   //Nodify already exist
    while(temp->next!=NULL){
      temp = temp->next;
      if(temp->function == functionAddress)
        return kNoErr;   //Nodify already exist
    }
    temp->next = notify;
  }
exit:
  return err;
}

merr_t mxos_system_notify_remove( mxos_notify_types_t notify_type, void *functionAddress )
{
  merr_t err = kNoErr;
  _Notify_list_t *temp = Notify_list[notify_type];
  _Notify_list_t *temp2 = temp;
  require_action(Notify_list[notify_type], exit, err = kDeletedErr);
  do{
    if(temp->function == functionAddress){
      if(temp == Notify_list[notify_type]){  //first element
        Notify_list[notify_type] = Notify_list[notify_type]->next;
        free(temp);
      }else{
        temp2->next = temp->next;
        free(temp);
      }
       break;
    }
    require_action(temp->next!=NULL, exit, err = kNotFoundErr);
    temp2 = temp;
    temp = temp->next;
  }while(temp != NULL);

exit:
  return err;
}

merr_t mxos_system_notify_remove_all( mxos_notify_types_t notify_type)
{
    _Notify_list_t *temp = Notify_list[notify_type];;

    while(temp) {
        Notify_list[notify_type] = Notify_list[notify_type]->next;
        free(temp);
        temp = Notify_list[notify_type];
    }

    return kNoErr;
}
