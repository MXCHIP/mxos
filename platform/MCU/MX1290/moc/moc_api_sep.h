#include "mxos_security.h"

enum {
	API_VERSION_V1 = 1,
	API_VERSION_MAX,
};

typedef void* mxos_event;
typedef void (*ssl_log_cb)(const int logLevel, const char *const logMessage);

#define EXTRA_CRYPTO_FLAG 0xEC000001

typedef struct {
    int  (*InitRng)(CyaSSL_RNG*);
    int  (*RNG_GenerateBlock)(CyaSSL_RNG*, byte*, word32 sz);
    int  (*RNG_GenerateByte)(CyaSSL_RNG*, byte*);
    int  (*FreeRng)(CyaSSL_RNG*);

    int  (*RsaPublicKeyDecode)(const byte* input, word32* inOutIdx, RsaKey*, word32);
    int  (*InitRsaKey)(RsaKey* key, void*);
    int  (*FreeRsaKey)(RsaKey* key);
    int  (*RsaPublicEncrypt)(const byte* in, word32 inLen, byte* out,
                             word32 outLen, RsaKey* key, CyaSSL_RNG* rng);
    int  (*RsaSSL_Verify)(const byte* in, word32 inLen, byte* out,
                          word32 outLen, RsaKey* key);
    int  (*RsaEncryptSize)(RsaKey* key);

    int (*InitSha256)(Sha256*);
    int (*Sha256Update)(Sha256*, const byte*, word32);
    int (*Sha256Final)(Sha256*, byte*);

    int (*InitSha)(Sha*);
    int (*ShaUpdate)(Sha*, const byte*, word32);
    int (*ShaFinal)(Sha*, byte*);

    int (*HmacSetKey)(Hmac*, int type, const byte* key, word32 keySz);
    int (*HmacUpdate)(Hmac*, const byte*, word32);
    int (*HmacFinal)(Hmac*, byte*);

}extra_crypto_api_t;

typedef void (*mgnt_handler_t)(char *buf, int buf_len);

typedef struct {
	/* OS Layer*/
	int (*system_config)(int type, void *value);/* system configuration */
	int (*mxos_network_init)();
	merr_t (*mos_thread_new)( mos_thread_id_t* thread, uint8_t priority, const char* name, mos_thread_func_t function, uint32_t stack_size, uint32_t arg );
	merr_t (*mos_thread_delete)( mos_thread_id_t* thread );
	void (*mos_thread_suspend)(mos_thread_id_t* thread);
	void (*mxos_rtos_suspend_all_thread)(void);
	long (*mxos_rtos_resume_all_thread)(void);
	merr_t (*mos_thread_join)( mos_thread_id_t* thread );
	merr_t (*mos_thread_awake)( mos_thread_id_t* thread );
	bool (*mxos_rtos_is_current_thread)( mos_thread_id_t* thread );
	void (*mos_sleep)(uint32_t seconds);
	void (*mos_sleep_ms)(uint32_t milliseconds);
	merr_t (*mos_semphr_new)( mos_semphr_id_t* semaphore, int count );
	merr_t (*mos_semphr_release)( mos_semphr_id_t* semaphore );
	merr_t (*mos_semphr_acquire)( mos_semphr_id_t* semaphore, uint32_t timeout_ms );
	merr_t (*mos_semphr_delete)( mos_semphr_id_t* semaphore );
	merr_t (*mos_mutex_new)( mos_mutex_id_t* mutex );
	merr_t (*mos_mutex_lock)( mos_mutex_id_t* mutex );
	merr_t (*mos_mutex_unlock)( mos_mutex_id_t* mutex );
	merr_t (*mos_mutex_delete)( mos_mutex_id_t* mutex );
	merr_t (*mos_queue_new)( mos_queue_id_t* queue, const char* name, uint32_t message_size, uint32_t number_of_messages );
	merr_t (*mos_queue_push)( mos_queue_id_t* queue, void* message, uint32_t timeout_ms );
	merr_t (*mos_queue_pop)( mos_queue_id_t* queue, void* message, uint32_t timeout_ms );
	merr_t (*mos_queue_delete)( mos_queue_id_t* queue );
	bool (*mxos_rtos_is_queue_empty)( mos_queue_id_t* queue );
	bool (*mxos_rtos_is_queue_full)( mos_queue_id_t* queue );
	uint32_t (*mxos_get_time)(void);
	merr_t (*mxos_init_timer)( mos_timer_id_t* timer, uint32_t time_ms, mos_timer_handler_t function, void* arg );
	merr_t (*mxos_start_timer)( mos_timer_id_t* timer );
	merr_t (*mxos_stop_timer)( mos_timer_id_t* timer );
	merr_t (*mxos_reload_timer)( mos_timer_id_t* timer );
	merr_t (*mos_timer_delete)( mos_timer_id_t* timer );
	bool (*mxos_is_timer_running)( mos_timer_id_t* timer );
	int (*mos_event_fd_new)(mxos_event handle);
	int (*mos_event_fd_delete)(int fd);

	/* memory management*/
	struct mxchip_mallinfo* (*mos_mallinfo_legacy)(void);
	void* (*malloc)(size_t size); // malloc
	void* (*realloc)(void* pv, size_t size); // realloc
	void (*free)(void* pv);     //free
	void* (*calloc)(size_t nmemb, size_t size);     // calloc
	void (*heap_insert)(uint8_t *pv, int len);

	void (*get_random_sequence)(unsigned char *buf, unsigned int size);
	int (*last_reset_reason)(void);
	int (*aon_write)( uint32_t offset, uint8_t* in ,uint32_t len);
	int (*aon_read )( uint32_t offset, uint8_t* out, uint32_t len);

	/* uitls */
	int (*debug_putchar)(char *ch, int len);
	int (*debug_getchar)(char *ch);
	void (*mxos_sys_reboot)( void );

	struct tm* (*localtime)(const time_t * time);
	char * (*asctime)(const struct tm *tm);

    void (*mos_thread_resume)(mos_thread_id_t* thread);
    int (*hardfault_get)(char *msg, int len);
    int (*mxos_init_once_timer)( mos_timer_id_t* timer, uint32_t time_ms, mos_timer_handler_t function, void* arg );
    int (*mxos_change_timer_period)( mos_timer_id_t* timer, uint32_t new_period );
} os_api_v1_t;

typedef struct {
	/* SSL */
	void (*ssl_set_cert)(const char *_cert_pem, const char *private_key_pem);
	void* (*ssl_connect)(int fd, int calen, char*ca, int *errno); 
	void* (*ssl_accept)(int fd); 
	int (*ssl_send)(void* ssl, char *data, int len);
	int (*ssl_recv)(void* ssl, char *data, int len);
	int (*ssl_close)(void* ssl);
	void (*set_ssl_client_version)(int version);

	int (*ssl_pending)(void* ssl);
	int (*ssl_get_error)(void* ssl, int ret);
	void (*ssl_set_using_nonblock)(void* ssl, int nonblock);
	int (*ssl_get_fd)(const void* ssl);
	int (*ssl_loggingcb)(ssl_log_cb f);
	
	/*crypto*/
	void (*InitMd5)(md5_context*md5);
	void (*Md5Update)(md5_context* md5, const uint8_t* data, uint32_t len);
	void (*Md5Final)(md5_context* md5, uint8_t* hash);
	int (*Md5Hash)(const uint8_t* data, uint32_t len, uint8_t* hash);
	void (*AesEncryptDirect)(Aes* aes, uint8_t* out, const uint8_t* in);
	void (*AesDecryptDirect)(Aes* aes, uint8_t* out, const uint8_t* in);
	int (*AesSetKeyDirect)(Aes* aes, const uint8_t* key, uint32_t len,
                                const uint8_t* iv, int dir);
	int (*aes_encrypt)(int sz, const char * key, const char * in, char * out);
	int (*aes_decrypt)(int sz, const char * key, const char * in, char * out);
	int  (*AesSetKey)(Aes* aes, const uint8_t* key, uint32_t len,
                              const uint8_t* iv, int dir);
	int  (*AesSetIV)(Aes* aes, const uint8_t* iv);
	int  (*AesCbcEncrypt)(Aes* aes, uint8_t* out,
                                  const uint8_t* in, uint32_t sz);
	int  (*AesCbcDecrypt)(Aes* aes, uint8_t* out,
                                  const uint8_t* in, uint32_t sz);
	void* (*ssl_nonblock_connect)(int fd, int calen, char*ca, int *errno, int timeout);
	void (*ssl_set_client_cert)(const char *_cert_pem, const char *private_key_pem);
	void* (*ssl_connect_sni)(int fd, int calen, char*ca, char *sni_servername, int *errno);
    void (*ssl_set_ecc)(int enable);

    uint32_t extra_crypto_flag;
    extra_crypto_api_t *extra_crypto_apis; 

    void* (*ssl_connect_dtls)(int fd, int calen, char*ca, int *errno);
    void (*ssl_set_alpn_list)(char*list);
} ssl_crypto_api_v1_t;

typedef struct {
	/* WIFI MGR */
	int (*wlan_get_mac_address)(unsigned char *dest);
	int (*wlan_get_mac_address_by_interface)(wlan_if_t wlan_if, unsigned char *dest);
	int (*mxos_wlan_driver_version)( char* version, int length );
	merr_t (*mwifi_softap_start)(mwifi_softap_attr_t* attr);
	merr_t (*mwifi_connect)(wifi_connect_attr_t* attr);
	merr_t (*mwifi_get_ip)(IPStatusTypedef *outNetpara, WiFi_Interface inInterface);
	merr_t (*mwifi_get_link_info)(LinkStatusTypeDef *outStatus);
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
	void (*mwifi_monitor_reg_cb)(monitor_cb_t fn);
	void (*wlan_set_channel)(int channel);
	int (*mxchip_active_scan)(char*ssid, int is_adv);
	int (*send_easylink_minus)(uint32_t ip, char *ssid, char *key)	;
	int (*mxos_wlan_get_channel)(void);
    merr_t (*wifi_manage_custom_ie_add)(wlan_if_t wlan_if, uint8_t *custom_ie, uint32_t len);
    merr_t (*wifi_manage_custom_ie_delete)(wlan_if_t wlan_if);
    int (*wlan_inject_frame)(const uint8_t *buff, size_t len);
	int (*mxos_wlan_monitor_no_easylink)(void);
	int (*wifi_set_country)(int country_code);
	int (*wlan_rx_mgnt_set)(int enable, mgnt_handler_t cb);
	void (*autoconfig_start)(int seconds, int mode);
    void (*wlan_set_softap_tdma)(int value);
    int (*wifi_off_fastly)(void);
    int (*OpenEasylink_softap)(int timeout, char *ssid, char*key, int channel);
    int (*mxos_wlan_monitor_with_easylink)(void);
} wifi_api_v1_t;

typedef struct {
	/* CLI APIs */
	int (*cli_init)(void);
	int (*cli_register_command)(const struct cli_command *command);
	int (*cli_unregister_command)(const struct cli_command *command);
	void (*wifistate_Command)(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
	void (*wifidebug_Command)(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
	void (*wifiscan_Command)(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
	void (*ifconfig_Command)(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
	void (*arp_Command)(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
	void (*ping_Command)(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
	void (*dns_Command)(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
	void (*task_Command)(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
	void (*socket_show_Command)(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
	void (*memory_show_Command)(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
	void (*memory_dump_Command)(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
	void (*memory_set_Command)(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
	void (*memp_dump_Command)(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
	void (*driver_state_Command)(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
	void (*iperf_Command)(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv);
} cli_api_v1_t;


typedef struct {
	mxos_logic_partition_t* (*mhal_flash_get_info)( mxos_partition_t inPartition );
	merr_t (*mhal_flash_erase)(mxos_partition_t inPartition, uint32_t off_set, uint32_t size);
	merr_t (*mhal_flash_write)( mxos_partition_t inPartition, volatile uint32_t* off_set, uint8_t* inBuffer ,uint32_t inBufferLength);
	merr_t (*mhal_flash_read)( mxos_partition_t inPartition, volatile uint32_t* off_set, uint8_t* outBuffer, uint32_t inBufferLength);
	merr_t (*mxos_flash_enable_security)( mxos_partition_t partition, uint32_t off_set, uint32_t size );
} flash_api_t;

typedef struct {
	merr_t (*mhal_gpio_open)( mxos_gpio_t gpio, mxos_gpio_config_t configuration );
	merr_t (*mhal_gpio_close)( mxos_gpio_t gpio );
	merr_t (*mhal_gpio_high)( mxos_gpio_t gpio );
	merr_t (*mhal_gpio_low)( mxos_gpio_t gpio );
	merr_t (*mhal_gpio_toggle)( mxos_gpio_t gpio );
	bool (*mhal_gpio_value)( mxos_gpio_t gpio );
	merr_t (*mhal_gpio_int_on)( mxos_gpio_t gpio, mxos_gpio_irq_trigger_t trigger, mxos_gpio_irq_handler_t handler, void* arg );
	merr_t (*mhal_gpio_int_off)( mxos_gpio_t gpio );
} gpio_api_t;

typedef struct {
	merr_t (*mhal_uart_open)( mxos_uart_t uart, const mxos_uart_config_t* config, ring_buffer_t* optional_rx_buffer );
	merr_t (*mhal_uart_close)( mxos_uart_t uart );
	merr_t (*mhal_uart_write)( mxos_uart_t uart, const void* data, uint32_t size );
	merr_t (*mhal_uart_read)( mxos_uart_t uart, void* data, uint32_t size, uint32_t timeout );
	uint32_t (*mhal_uart_readd_data_len)( mxos_uart_t uart ); 
	void     (*MxosUartPinRedirect)(mxos_uart_t uart);
    int (*disable_log_uart)(void);
} uart_api_t;

typedef void (*rtc_irq_handler)(void);

typedef struct {
	void (*mxos_rtc_init)(void);
	merr_t (*mxos_rtc_get_time)(time_t *time);
	merr_t (*mxos_rtc_set_time)(time_t time);
    merr_t (*MxosRtcSetalarm)(time_t *time, rtc_irq_handler handler);
} rtc_api_t;

typedef struct {
	/* Power management*/
	int (*pm_mcu_state)(power_state_t state, uint32_t time_dur);
	int (*pm_wakeup_source)(uint8_t wake_source);
	void (*wifi_off_mcu_standby)(uint32_t seconds);
	void (*mxos_mcu_powersave_config)( int enable );
} power_save_api_t;

typedef os_api_v1_t os_api_t;
typedef ssl_crypto_api_v1_t ssl_crypto_api_t;
typedef wifi_api_v1_t wifi_api_t;
typedef cli_api_v1_t cli_api_t;

/* API type define */
typedef struct 
{
	os_api_t *os_apis;
	lwip_api_t *lwip_apis;
	ssl_crypto_api_t *ssl_crypto_apis;
	wifi_api_t *wifi_apis;
	cli_api_t *cli_apis;

    flash_api_t *flash_apis;
	gpio_api_t *gpio_apis;
	uart_api_t *uart_apis;
	i2c_api_t *i2c_apis;
	spi_api_t *spi_apis;
	pwm_api_t *pwm_apis;
	rtc_api_t *rtc_apis;
	wdg_api_t *wdg_apis;
	adc_api_t *adc_apis;
	power_save_api_t *ps_apis;
	gtimer_api_t *gtimer_apis;
} kernel_api_v1_t;

typedef kernel_api_v1_t kernel_api_t;

typedef struct new_mxos_api_struct
{
	char *library_version;

	int (*mxos_api_get)(int version, void *kernel_apis);
} new_mxos_api_t;

mxos_api_t *moc_adapter(new_mxos_api_t *new_mxos_api);
