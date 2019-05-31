#include <string.h>

#include "merr.h"
#include "mwifi.h"

#define DHCP_Disable (0) /**< Disable DHCP service. */
#define DHCP_Client (1)  /**< Enable DHCP client which get IP address from DHCP server automatically, \
                              reset Wi-Fi connection if failed. */
#define DHCP_Server (2)  /**< Enable DHCP server, needs assign a static address as local address. */

/** 
 *  @brief  wlan network interface enumeration definition.
 */
typedef enum
{
  Soft_AP, /**< Act as an access point, and other station can connect, 4 stations Max*/
  Station  /**< Act as a station which can connect to an access point*/
} wlanInterfaceTypedef;

typedef uint8_t uint8_t;

typedef uint8_t wlan_sec_type_t;

/** 
 *  @brief  wlan local IP information structure definition.  
 */
typedef struct
{
  uint8_t dhcp;  /**< DHCP mode: @ref DHCP_Disable, @ref DHCP_Client, @ref DHCP_Server.*/
  char ip[16];   /**< Local IP address on the target wlan interface: @ref wlanInterfaceTypedef.*/
  char gate[16]; /**< Router IP address on the target wlan interface: @ref wlanInterfaceTypedef.*/
  char mask[16]; /**< Netmask on the target wlan interface: @ref wlanInterfaceTypedef.*/
  char dns[16];  /**< DNS server IP address.*/
  char mac[16];  /**< MAC address, example: "C89346112233".*/
  char broadcastip[16];
} IPStatusTypedef;

/** 
 *  @brief  Scan result using advanced scan.  
 */
typedef struct _ScanResult_adv
{
  char ApNum; /**< The number of access points found in scanning.*/
  struct
  {
    char ssid[32];            /**< The SSID of an access point.*/
    char bssid[6];            /**< The BSSID of an access point.*/
    char channel;             /**< The RF frequency, 1-13*/
    wlan_sec_type_t security; /**< Security type, @ref wlan_sec_type_t*/
    int16_t rssi;             /**< Signal strength*/
  } * ApList;
} ScanResult_adv;

/** 
 *  @brief  Scan result using normal scan.  
 */
typedef struct _ScanResult
{
  char ApNum; /**< The number of access points found in scanning. */
  struct
  {
    char ssid[32]; /**< The SSID of an access point. */
    int16_t rssi;  /**< Signal strength*/
  } * ApList;
} ScanResult;

/** 
 *  @brief  Input network paras, used in micoWlanStart function.  
 */
typedef struct _network_InitTypeDef_st
{
  char wifi_mode;             /**< DHCP mode: @ref wlanInterfaceTypedef.*/
  char wifi_ssid[32];         /**< SSID of the wlan needs to be connected.*/
  char wifi_key[64];          /**< Security key of the wlan needs to be connected, ignored in an open system.*/
  char local_ip_addr[16];     /**< Static IP configuration, Local IP address. */
  char net_mask[16];          /**< Static IP configuration, Netmask. */
  char gateway_ip_addr[16];   /**< Static IP configuration, Router IP address. */
  char dnsServer_ip_addr[16]; /**< Static IP configuration, DNS server IP address. */
  char dhcpMode;              /**< DHCP mode, @ref DHCP_Disable, @ref DHCP_Client and @ref DHCP_Server. */
  char reserved[32];
  int wifi_retry_interval; /**< Retry interval if an error is occured when connecting an access point, 
                                     time unit is millisecond. */
} network_InitTypeDef_st;

/** 
 *  @brief  Advanced precise wlan parameters, used in @ref network_InitTypeDef_adv_st.  
 */
typedef struct
{
  char ssid[32];   /**< SSID of the wlan that needs to be connected. Example: "SSID String". */
  char bssid[6];   /**< BSSID of the wlan needs to be connected. Example: {0xC8 0x93 0x46 0x11 0x22 0x33}. */
  uint8_t channel; /**< Wlan's RF frequency, channel 0-13. 1-13 means a fixed channel
                            that can speed up a connection procedure, 0 is not a fixed input
                            means all channels are possible*/
  wlan_sec_type_t security;
} apinfo_adv_t;

/** 
 *  @brief  Input network precise paras in micoWlanStartAdv function.  
 */
typedef struct _network_InitTypeDef_adv_st
{
  apinfo_adv_t ap_info;       /**< @ref apinfo_adv_t. */
  char key[64];               /**< Security key or PMK of the wlan. */
  int key_len;                /**< The length of the key. */
  char local_ip_addr[16];     /**< Static IP configuration, Local IP address. */
  char net_mask[16];          /**< Static IP configuration, Netmask. */
  char gateway_ip_addr[16];   /**< Static IP configuration, Router IP address. */
  char dnsServer_ip_addr[16]; /**< Static IP configuration, DNS server IP address. */
  char dhcpMode;              /**< DHCP mode, @ref DHCP_Disable, @ref DHCP_Client and @ref DHCP_Server. */
  char reserved[32];
  int wifi_retry_interval; /**< Retry interval if an error is occured when connecting an access point, 
                                  time unit is millisecond. */
} network_InitTypeDef_adv_st;

/** 
 *  @brief  Current link status in station mode.  
 */
typedef struct _linkStatus_t
{
  int is_connected; /**< The link to wlan is established or not, 0: disconnected, 1: connected. */
  int rssi;         /**< Signal strength of the current connected AP */
  uint8_t ssid[32]; /**< SSID of the current connected wlan */
  uint8_t bssid[6]; /**< BSSID of the current connected wlan */
  int channel;      /**< Channel of the current connected wlan */
} LinkStatusTypeDef;

typedef struct
{
  /* WIFI MGR */
  int (*wlan_get_mac_address)(unsigned char *dest);
  int (*wlan_get_mac_address_by_interface)(uint8_t wlan_if, unsigned char *dest);
  int (*mxos_wlan_driver_version)(char *version, int length);
  merr_t (*mwifi_softap_start)(void *attr);
  merr_t (*mwifi_connect)(void *attr);
  merr_t (*mwifi_get_ip)(void *outNetpara, uint8_t inInterface);
  merr_t (*mwifi_get_link_info)(void *outStatus);
  void (*mwifi_softap_startScan)(void);
  void (*mwifi_softap_startScanAdv)(void);
  merr_t (*mwifi_off)(void);
  merr_t (*mwifi_on)(void);
  merr_t (*mxosWlanSuspend)(void);
  merr_t (*mwifi_disconnect)(void);
  merr_t (*mwifi_softap_stop)(void);
  merr_t (*mwifi_softap_startEasyLink)(int inTimeout);
  merr_t (*mxosWlanStopEasyLink)(void);
  void (*mwifi_ps_on)(void);
  void (*mwifi_ps_off)(void);
  void (*wifimgr_debug_enable)(bool enable);
  int (*mxos_wlan_monitor_rx_type)(int type);
  int (*mwifi_monitor_start)(void);
  int (*mwifi_monitor_stop)(void);
  int (*mwifi_monitor_set_channel)(int channel);
  void (*mwifi_monitor_reg_cb)(void *fn);
  void (*wlan_set_channel)(int channel);
  int (*mxchip_active_scan)(char *ssid, int is_adv);
  int (*send_easylink_minus)(uint32_t ip, char *ssid, char *key);
  int (*mxos_wlan_get_channel)(void);
  merr_t (*wifi_manage_custom_ie_add)(uint8_t wlan_if, uint8_t *custom_ie, uint32_t len);
  merr_t (*wifi_manage_custom_ie_delete)(uint8_t wlan_if);
  int (*wlan_inject_frame)(const uint8_t *buff, size_t len);
  int (*mxos_wlan_monitor_no_easylink)(void);
  int (*wifi_set_country)(int country_code);
  int (*wlan_rx_mgnt_set)(int enable, void *cb);
  void (*autoconfig_start)(int seconds, int mode);
  void (*wlan_set_softap_tdma)(int value);
  int (*wifi_off_fastly)(void);
  int (*OpenEasylink_softap)(int timeout, char *ssid, char *key, int channel);
  int (*mxos_wlan_monitor_with_easylink)(void);
} wifi_api_t;

extern wifi_api_t *wifi_apis;

// station
merr_t mwifi_connect(const char *ssid, char *key, int key_len, mwifi_connect_attr_t *attr, mwifi_ip_attr_t *ip)
{
  network_InitTypeDef_adv_st wNetConfig;

  memset(&wNetConfig, 0x0, sizeof(network_InitTypeDef_adv_st));

  strncpy(wNetConfig.ap_info.ssid, ssid, 32 - 1);
  memcpy(wNetConfig.key, key, key_len);
  wNetConfig.key_len = key_len;

  if (attr != NULL)
  {
    memcpy(wNetConfig.ap_info.bssid, attr->bssid, 6);
    wNetConfig.ap_info.channel = attr->channel;
    wNetConfig.ap_info.security = attr->security;
  }

  if (ip != NULL)
  {
    wNetConfig.dhcpMode = DHCP_Disable;
    strcpy(wNetConfig.local_ip_addr, ip->localip);
    strcpy(wNetConfig.net_mask, ip->netmask);
    strcpy(wNetConfig.gateway_ip_addr, ip->gateway);
    strcpy(wNetConfig.dnsServer_ip_addr, ip->dnserver);
  }
  else
  {
    wNetConfig.dhcpMode = DHCP_Client;
  }

  wNetConfig.wifi_retry_interval = 100;

  return wifi_apis->mwifi_connect(&wNetConfig);
}

merr_t mwifi_ap_add(const char *ssid, char *key, int key_len, mwifi_ip_attr_t *attr)
{
  return kUnsupportedErr;
}

merr_t mwifi_disconnect(void)
{
  return wifi_apis->mxosWlanSuspend();
}

void mwifi_set_reconnect_interval(uint32_t ms)
{
}

// softap
merr_t mwifi_softap_start(const char *ssid, char *key, int channel, mwifi_ip_attr_t *attr)
{
  network_InitTypeDef_st wNetConfig;

  memset(&wNetConfig, 0x0, sizeof(network_InitTypeDef_st));

  strcpy(wNetConfig.wifi_ssid, ssid);
  strcpy(wNetConfig.wifi_key, key);
  wNetConfig.wifi_mode = Soft_AP;
  wNetConfig.dhcpMode = DHCP_Server;
  if (attr != NULL)
  {
    strcpy(wNetConfig.local_ip_addr, attr->localip);
    strcpy(wNetConfig.net_mask, attr->netmask);
    strcpy(wNetConfig.dnsServer_ip_addr, attr->dnserver);
  }
  else
  {
    strcpy(wNetConfig.local_ip_addr, "10.0.0.1");
    strcpy(wNetConfig.net_mask, "255.255.255.0");
    strcpy(wNetConfig.dnsServer_ip_addr, "10.0.0.1");
  }

  wifi_apis->wlan_set_channel(channel);

  return wifi_apis->mwifi_softap_start(&wNetConfig);
}

merr_t mwifi_softap_stop(void)
{
  return wifi_apis->mwifi_softap_stop();
}

// utilities
void mwifi_scan(const char *ssid)
{
  return wifi_apis->mwifi_softap_startScanAdv();
}

void mwifi_get_mac(uint8_t mac[6])
{
  wifi_apis->wlan_get_mac_address(mac);
}

merr_t mwifi_get_ip(mwifi_ip_attr_t *attr, mwifi_if_t iface)
{
  merr_t err;
  IPStatusTypedef ipstat;

  err = wifi_apis->mwifi_get_ip(&ipstat, iface);

  memset(attr, 0, sizeof(mwifi_ip_attr_t));
  memcpy(attr->localip, ipstat.ip, 16);
  memcpy(attr->netmask, ipstat.mask, 16);
  memcpy(attr->gateway, ipstat.gate, 16);
  memcpy(attr->dnserver, ipstat.dns, 16);

  return err;
}

merr_t mwifi_get_link_info(mwifi_link_info_t *info)
{
  merr_t err;
  LinkStatusTypeDef linkstat;

  err = wifi_apis->mwifi_get_link_info(&linkstat);

  memset(info, 0, sizeof(mwifi_link_info_t));
  info->is_connected = linkstat.is_connected;
  info->rssi = linkstat.rssi;
  memcpy(info->ssid, linkstat.ssid, 32);
  memcpy(info->bssid, linkstat.bssid, 6);
  info->channel = linkstat.channel;

  return err;
}

// power
merr_t mwifi_on(void)
{
  return wifi_apis->mwifi_on();
}

merr_t mwifi_off(void)
{
  return wifi_apis->mwifi_off();
}

void mwifi_ps_on(void)
{
  wifi_apis->mwifi_ps_on();
}

void mwifi_ps_off(void)
{
  wifi_apis->mwifi_ps_off();
}

// monitor
merr_t mwifi_monitor_start(void)
{
  return wifi_apis->mwifi_monitor_start();
}

merr_t mwifi_monitor_stop(void)
{
  return wifi_apis->mwifi_monitor_stop();
}

merr_t mwifi_monitor_set_channel(uint8_t channel)
{
  return wifi_apis->mwifi_monitor_set_channel(channel);
}

uint8_t mwifi_monitor_get_channel(void)
{
  return wifi_apis->mxos_wlan_get_channel();
}

void mwifi_monitor_reg_cb(mwifi_monitor_cb_t func)
{
  wifi_apis->mwifi_monitor_reg_cb(func);
}

merr_t mwifi_monitor_send_frame(uint8_t *data, uint32_t size)
{
  return wifi_apis->wlan_inject_frame(data, size);
}

merr_t mwifi_reg_mgnt_cb(mwifi_monitor_cb_t func)
{
  wifi_apis->mwifi_monitor_reg_cb(func);
  return kNoErr;
}

merr_t mwifi_custom_ie_add(mwifi_if_t iface, uint8_t *data, uint32_t size)
{
  return wifi_apis->wifi_manage_custom_ie_add(iface, data, size);
}

merr_t mwifi_custom_ie_remove(mwifi_if_t iface)
{
  return wifi_apis->wifi_manage_custom_ie_delete(iface);
}

merr_t mwifi_monitor_start_with_softap(char *ssid, char *key, int channel, mwifi_ip_attr_t *attr, asso_event_handler_t fn)
{
  return kUnsupportedErr;
}
