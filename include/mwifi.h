#ifndef __MWIFI_H__
#define __MWIFI_H__

#include <stdint.h>
#include <stdbool.h>

#include "merr.h"

typedef uint8_t mwifi_if_t;

enum
{
  SECURITY_TYPE_NONE,       /**< Open system. */
  SECURITY_TYPE_WEP,        /**< Wired Equivalent Privacy. WEP security. */
  SECURITY_TYPE_WPA_TKIP,   /**< WPA /w TKIP */
  SECURITY_TYPE_WPA_AES,    /**< WPA /w AES */
  SECURITY_TYPE_WPA2_TKIP,  /**< WPA2 /w TKIP */
  SECURITY_TYPE_WPA2_AES,   /**< WPA2 /w AES */
  SECURITY_TYPE_WPA2_MIXED, /**< WPA2 /w AES or TKIP */
  SECURITY_TYPE_AUTO,       /**< It is used when calling @ref mwifi_connect, MXOS read security type from scan result. */
};
typedef uint8_t mwifi_security_t;

typedef struct
{
  char localip[16];
  char netmask[16];
  char gateway[16];
  char dnserver[16];
} mwifi_ip_attr_t;

typedef struct
{
  uint8_t bssid[6];
  uint8_t channel;
  mwifi_security_t security;
} mwifi_connect_attr_t;

typedef struct
{
  uint8_t channel;
  mwifi_ip_attr_t *ip_attr; /* NULL - use default */
} mwifi_softap_attr_t;

typedef struct
{
  int is_connected; /**< The link to wlan is established or not, 0: disconnected, 1: connected. */
  int rssi;         /**< Signal strength of the current connected AP */
  char ssid[33];    /**< SSID of the current connected wlan */
  uint8_t bssid[6]; /**< BSSID of the current connected wlan */
  char key[65];     /**< The passphrase/PSK of the connected AP */
  int channel;      /**< Channel of the current connected wlan */
  mwifi_security_t security;
} mwifi_link_info_t;

typedef struct
{
  int rssi;                  /**< Signal strength of the AP */
  char ssid[33];             /**< SSID of the AP */
  uint8_t bssid[6];          /**< BSSID of the AP */
  int channel;               /**< Channel of the AP */
  mwifi_security_t security; /**< security of the AP */
} mwifi_ap_info_t;

typedef void (*mwifi_monitor_cb_t)(uint8_t *data, int len);

enum custom_ie_delete_op_e
{
  CUSTOM_IE_DELETE_ALL = 0,
  CUSTOM_IE_DELETE_BY_OUI = 1,
};
typedef uint8_t mwifi_custom_ie_remove_type_t;

typedef uint8_t mwifi_notify_t;

typedef void (*asso_event_handler_t)(char *buf, int buf_len, int flags, void *handler_user_data);

// station
merr_t mwifi_connect(const char *ssid, char *key, int key_len, mwifi_connect_attr_t *attr, mwifi_ip_attr_t *ip);
merr_t mwifi_ap_add(const char *ssid, char *key, int key_len, mwifi_ip_attr_t *attr);
merr_t mwifi_disconnect(void);
void mwifi_set_reconnect_interval(uint32_t ms); /* 0xFFFFFFFF=don't reconnect */

// softap
merr_t mwifi_softap_start(const char *ssid, char *key, int channel, mwifi_ip_attr_t *attr);
merr_t mwifi_softap_stop(void);

// utilities
void mwifi_scan(const char *ssid);
void mwifi_get_mac(uint8_t mac[6]);
merr_t mwifi_get_ip(mwifi_ip_attr_t *attr, mwifi_if_t iface);
merr_t mwifi_get_link_info(mwifi_link_info_t *info);

// power
merr_t mwifi_on(void);
merr_t mwifi_off(void);
void mwifi_ps_on(void);
void mwifi_ps_off(void);

// monitor
merr_t mwifi_monitor_start(void);
merr_t mwifi_monitor_stop(void);
merr_t mwifi_monitor_set_channel(uint8_t channel);
uint8_t mwifi_monitor_get_channel(void);
void mwifi_monitor_reg_cb(mwifi_monitor_cb_t func);
merr_t mwifi_monitor_send_frame(uint8_t *data, uint32_t size);
merr_t mwifi_reg_mgnt_cb(mwifi_monitor_cb_t func);
merr_t mwifi_custom_ie_add(mwifi_if_t iface, uint8_t *data, uint32_t size);
merr_t mwifi_custom_ie_remove(mwifi_if_t iface);
merr_t mwifi_monitor_start_with_softap(char *ssid, char *key, int channel, mwifi_ip_attr_t *attr, asso_event_handler_t fn);

//
merr_t mwifi_start_aws(int timeout);
merr_t mwifi_aws_stop(void);

#endif