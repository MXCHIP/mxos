#ifndef _MXOS_API_H_
#define _MXOS_API_H_

#include "lwip_api_define.h"
#include "mxos.h"

#define INTERFACE_VERSION 3

typedef void (*ssl_Logging_cb)( const int logLevel,
                                const char * const logMessage );

#ifndef BIT
#define BIT(__n)       (1<<(__n))
#endif

#define DSLEEP_WAKEUP_BY_TIMER		BIT(0)
#define DSLEEP_WAKEUP_BY_GPIO		BIT(2)    // GPIO Port(PA_18, PA_5, PA_22, PA_23)

typedef enum
{
    /** US FCC */
    COUNTRY_US = 1,
    /** IC Canada */
    COUNTRY_CA,
    /** Singapore */
    COUNTRY_SG,
    /** ETSI */
    COUNTRY_EU,
    /** Australia */
    COUNTRY_AU,
    /** Republic Of Korea */
    COUNTRY_KR,
    /** France */
    COUNTRY_FR,
    /** Japan */
    COUNTRY_JP,
    /** China */
    COUNTRY_CN,
} country_code_t;
enum wlan_bss_type
{
    WLAN_BSS_TYPE_STA = 0,
    WLAN_BSS_TYPE_UAP = 1,
    WLAN_BSS_TYPE_WIFIDIRECT = 2,
    WLAN_BSS_TYPE_ANY = 0xff,
};
typedef enum
{
    ASSOC_REQ_FRAME = 0x00,
    ASSOC_RESP_FRAME = 0x10,
    REASSOC_REQ_FRAME = 0x20,
    REASSOC_RESP_FRAME = 0x30,
    PROBE_REQ_FRAME = 0x40,
    PROBE_RESP_FRAME = 0x50,
    BEACON_FRAME = 0x80,
    DISASSOC_FRAME = 0xA0,
    AUTH_FRAME = 0xB0,
    DEAUTH_FRAME = 0xC0,
    ACTION_FRAME = 0xD0,
    DATA_FRAME = 0x08,
    QOS_DATA_FRAME = 0x88,
} wifi_frame_type_t;

/** 802_11_header packet */
typedef struct _wifi_mgmt_frame_t
{
    /** Packet Length */
    uint16_t frm_len;
    /** Frame Type */
    wifi_frame_type_t frame_type;
    /** Frame Control flags */
    uint8_t frame_ctrl_flags;
    /** Duration ID */
    uint16_t duration_id;
    /** Address1 */
    uint8_t addr1[6];
    /** Address2 */
    uint8_t addr2[6];
    /** Address3 */
    uint8_t addr3[6];
    /** Sequence Control */
    uint16_t seq_ctl;
    /** Address4 */
    uint8_t addr4[6];
    /** Frame payload */
    uint8_t payload[0];
} wlan_mgmt_frame_t;

typedef struct
{
    merr_t (*pwm_init)( mxos_pwm_t pwm, uint32_t frequency, float duty_cycle );
    merr_t (*pwm_start)( mxos_pwm_t pwm );
    merr_t (*pwm_stop)( mxos_pwm_t pwm );
} pwm_api_t;

typedef struct
{
    merr_t (*wdg_init)( uint32_t timeout );
    void (*wdg_reload)( void );
    merr_t (*wdg_stop)( void );
} wdg_api_t;

#define LAST_RST_CAUSE_VBAT    (1<<0)
#define LAST_RST_CAUSE_AV12    (1<<1)
#define LAST_RST_CAUSE_AV18    (1<<2)
#define LAST_RST_CAUSE_SOFTRST (1<<3)
#define LAST_RST_CAUSE_LOCKUP  (1<<4)
#define LAST_RST_CAUSE_WDT     (1<<5)

#define USER_APP_ADDR 0x1f064000 /* 400KB offset */
#define USER_MAGIC_NUM 0xC89346

/** Power States of MCU */
typedef enum
{

    /** (Active Mode): This is the full power state of MCU.
     *  Instruction execution takes place only in PM0.
     */
    PM0,
    /** (Idle Mode): In this mode Cortex M3 core function
     *  clocks are stopped until the occurrence of any interrupt.
     *  This consumes lower power than PM0. */
    PM1,

    /** (Standby Mode):In this mode, the Cortex M3,
     *  most of the peripherals & SRAM arrays are in
     *  low-power mode.The PMU and RTC are operational.
     *  A wakeup can happen by timeout (RTC based) or by asserting the
     *  WAKEUP 0/1 lines.This consumes much lower power than PM1.
     */
    PM2,

    /**(Sleep Mode): This mode further aggressively conserves power.
     * Only 192 KB (160 KB in SRAM0  and 32 KB in SRAM1)
     * out of 512 KB of SRAM is alive. All peripherals
     * are turned off and register config is lost.
     * Application should restore the peripheral config
     * after exit form PM3. This consumes lower power
     * than in PM2. A wakeup can happen by timeout (RTC based)
     * or by asserting the WAKEUP 0/1 lines.
     */
    PM3,

    /** (Shutoff Mode): This simulates a shutdown condition.
     * A wakeup can happen by timeout (RTC based) or by
     * asserting the WAKEUP 0/1 lines.
     * This is the lowest power state of MCU.
     * On wakeup execution begins from bootrom as
     * if a fresh bootup has occurred.
     */
    PM4
} power_state_t;

typedef struct
{
    merr_t (*mxos_adc_init)( mxos_adc_t adc, uint32_t sampling_cycle );
    merr_t (*mxos_adc_take_sample)( mxos_adc_t adc, uint16_t* output );
    merr_t (*mxos_adc_take_sampleStreram)( mxos_adc_t adc, void* buffer, uint16_t buffer_length );
    merr_t (*mxos_adc_deinit)( mxos_adc_t adc );
} adc_api_t;

typedef struct
{
    merr_t (*i2c_init)( mxos_i2c_device_t* device );
    merr_t (*i2c_deinit)( mxos_i2c_device_t* device );
    bool (*i2c_probe_device)( mxos_i2c_device_t* device, int retries );
    merr_t (*i2c_build_tx_msg)( mxos_i2c_message_t* message, const void* tx_buffer, uint16_t tx_buffer_length,
                                  uint16_t retries );
    merr_t (*i2c_build_rx_msg)( mxos_i2c_message_t* message, void* rx_buffer, uint16_t rx_buffer_length,
                                  uint16_t retries );
    merr_t (*i2c_build_combined_msg)( mxos_i2c_message_t* message, const void* tx_buffer, void* rx_buffer,
                                        uint16_t tx_buffer_length, uint16_t rx_buffer_length, uint16_t retries );
    merr_t (*i2c_transfer)( mxos_i2c_device_t* device, mxos_i2c_message_t* messages, uint16_t number_of_messages );
} i2c_api_t;

typedef struct
{
    merr_t (*spi_init)( const mxos_spi_device_t* spi );
    merr_t (*spi_transfer)( const mxos_spi_device_t* spi, const mxos_spi_message_segment_t* segments,
                              uint16_t number_of_segments );
    merr_t (*spi_finalize)( const mxos_spi_device_t* spi );
} spi_api_t;

typedef struct {
	merr_t (*mxos_gtimer_init)(mxos_gtimer_t gtimer);
	merr_t (*mxos_gtimer_start)(mxos_gtimer_t timer, mxos_gtimer_mode_t mode, uint32_t time, mxos_gtimer_irq_callback_t function, void *arg);
	merr_t (*mxos_gtimer_stop)(mxos_gtimer_t timer);
} gtimer_api_t;

/* API type define */
typedef struct mxos_api_struct
{
    char *library_version;

    /* OS Layer*/
    mxos_system_config_t* (*system_config_get)( void );
    void (*system_config_set)( mxos_system_config_t *cfg );
    void (*mxos_network_init)( );
    merr_t (*mos_thread_new)( mos_thread_id_t* thread, uint8_t priority, const char* name,
                                         mos_thread_func_t function, uint32_t stack_size, void* arg );
    merr_t (*mos_thread_delete)( mos_thread_id_t* thread );
    void (*mos_thread_yield)( void );
    void (*mos_thread_suspend)( mos_thread_id_t* thread );
    void (*mxos_rtos_suspend_all_thread)( void );
    long (*mxos_rtos_resume_all_thread)( void );
    merr_t (*mos_thread_join)( mos_thread_id_t* thread );
    merr_t (*mos_thread_awake)( mos_thread_id_t* thread );
    bool (*mxos_rtos_is_current_thread)( mos_thread_id_t* thread );
    void (*mos_sleep)( uint32_t seconds );
    void (*mos_sleep_ms)( uint32_t milliseconds );
    merr_t (*mos_semphr_new)( mos_semphr_id_t* semaphore, int count );
    merr_t (*mos_semphr_release)( mos_semphr_id_t* semaphore );
    merr_t (*mos_semphr_acquire)( mos_semphr_id_t* semaphore, uint32_t timeout_ms );
    merr_t (*mos_semphr_delete)( mos_semphr_id_t* semaphore );
    merr_t (*mos_mutex_new)( mos_mutex_id_t* mutex );
    merr_t (*mos_mutex_lock)( mos_mutex_id_t* mutex );
    merr_t (*mos_mutex_unlock)( mos_mutex_id_t* mutex );
    merr_t (*mos_mutex_delete)( mos_mutex_id_t* mutex );
    merr_t (*mos_queue_new)( mos_queue_id_t* queue, const char* name, uint32_t message_size,
                                      uint32_t number_of_messages );
    merr_t (*mos_queue_push)( mos_queue_id_t* queue, void* message, uint32_t timeout_ms );
    merr_t (*mos_queue_pop)( mos_queue_id_t* queue, void* message, uint32_t timeout_ms );
    merr_t (*mos_queue_delete)( mos_queue_id_t* queue );
    bool (*mxos_rtos_is_queue_empty)( mos_queue_id_t* queue );
    merr_t (*mxos_rtos_is_queue_full)( mos_queue_id_t* queue );
    uint32_t (*mxos_get_time)( void );
    merr_t (*mxos_init_timer)( mos_timer_id_t* timer, uint32_t time_ms, mos_timer_handler_t function, void* arg );
    merr_t (*mxos_start_timer)( mos_timer_id_t* timer );
    merr_t (*mxos_stop_timer)( mos_timer_id_t* timer );
    merr_t (*mxos_reload_timer)( mos_timer_id_t* timer );
    merr_t (*mos_timer_delete)( mos_timer_id_t* timer );
    bool (*mxos_is_timer_running)( mos_timer_id_t* timer );
    int (*mos_event_fd_new)( mos_event_id_t handle );
    int (*mos_event_fd_delete)( int fd );
    int (*SetTimer)( unsigned long ms, void (*psysTimerHandler)( void ) );
    int (*SetTimer_uniq)( unsigned long ms, void (*psysTimerHandler)( void ) );
    int (*UnSetTimer)( void (*psysTimerHandler)( void ) );

    /* memory management*/
    mxosMemInfo_t* (*mxos_memory_info)( void );
    void* (*malloc)( size_t size ); // malloc
    void* (*realloc)( void* pv, size_t size ); // realloc
    void (*free)( void* pv );     //free
    void* (*calloc)( int a, int b );     // calloc
    void (*heap_insert)( uint8_t *pv, int len );

    /* Socket */
    int (*socket)( int domain, int type, int protocol );
    int (*setsockopt)( int sockfd, int level, int optname, const void *optval, socklen_t optlen );
    int (*getsockopt)( int sockfd, int level, int optname, const void *optval, socklen_t *optlen );
    int (*bind)( int sockfd, const struct sockaddr *addr, socklen_t addrlen );
    int (*connect)( int sockfd, const struct sockaddr *addr, socklen_t addrlen );
    int (*listen)( int sockfd, int backlog );
    int (*accept)( int sockfd, struct sockaddr *addr, socklen_t *addrlen );
    int (*select)( int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, struct timeval *timeout );
    ssize_t (*send)( int sockfd, const void *buf, size_t len, int flags );
    int (*write)( int sockfd, void *buf, size_t len );
    ssize_t (*sendto)( int sockfd, const void *buf, size_t len, int flags,
                       const struct sockaddr *dest_addr,
                       socklen_t addrlen );
    ssize_t (*recv)( int sockfd, void *buf, size_t len, int flags );
    int (*read)( int sockfd, void *buf, size_t len );
    ssize_t (*recvfrom)( int sockfd, void *buf, size_t len, int flags,
                         struct sockaddr *src_addr,
                         socklen_t *addrlen );
    int (*close)( int fd );
    uint32_t (*inet_addr)( char *s );
    char* (*inet_ntoa)( char *s, uint32_t x );
    int (*gethostbyname)( const char * name, uint8_t * addr, uint8_t addrLen );
    void (*set_tcp_keepalive)( int inMaxErrNum, int inSeconds );
    void (*get_tcp_keepalive)( int *outMaxErrNum, int *outSeconds );

    /* SSL */
    void (*ssl_set_cert)( const char *_cert_pem, const char *private_key_pem );
    void* (*ssl_connect)( int fd, int calen, char*ca, int *ssl_errno );
    void* (*ssl_accept)( int fd );
    int (*ssl_send)( void* ssl, char *data, int len );
    int (*ssl_recv)( void* ssl, char *data, int len );
    int (*ssl_close)( void* ssl );
    void (*set_ssl_client_version)( int version );

    /*crypto*/
    void (*InitMd5)( md5_context*md5 );
    void (*Md5Update)( md5_context* md5, unsigned char *input, int ilen );
    void (*Md5Final)( md5_context* md5, uint8_t* hash );
    int (*Md5Hash)( const uint8_t* data, uint32_t len, uint8_t* hash );
    void (*AesEncryptDirect)( Aes* aes, uint8_t* out, const uint8_t* in );
    void (*AesDecryptDirect)( Aes* aes, uint8_t* out, const uint8_t* in );
    int (*AesSetKeyDirect)( Aes* aes, const uint8_t* key, uint32_t len,
                            const uint8_t* iv,
                            int dir );
    int (*aes_encrypt)( int sz, const char * key, const char * in, char * out );
    int (*aes_decrypt)( int sz, const char * key, const char * in, char * out );
    int (*AesSetKey)( Aes* aes, const uint8_t* key, uint32_t len,
                      const uint8_t* iv,
                      int dir );
    int (*AesSetIV)( Aes* aes, const uint8_t* iv );
    int (*AesCbcEncrypt)( Aes* aes, uint8_t* out,
                          const uint8_t* in,
                          uint32_t sz );
    int (*AesCbcDecrypt)( Aes* aes, uint8_t* out,
                          const uint8_t* in,
                          uint32_t sz );

    /* WIFI MGR */
    int (*wlan_get_mac_address)( unsigned char *dest );
    int (*wlan_get_mac_address_by_interface)(wlan_if_t wlan_if, unsigned char *dest);
    int (*mxos_wlan_driver_version)( char* version, int length );
    merr_t (*mwifi_softap_start)( mwifi_softap_attr_t* attr );
    merr_t (*mwifi_connect)( wifi_connect_attr_t* attr );
    merr_t (*mwifi_get_ip)( IPStatusTypedef *outNetpara, WiFi_Interface inInterface );
    merr_t (*mwifi_get_link_info)( LinkStatusTypeDef *outStatus );
    merr_t (*mwifi_softap_startScan)( void );
    merr_t (*mwifi_softap_startScanAdv)( void );
    merr_t (*mwifi_off)( void );
    merr_t (*mwifi_on)( void );
    merr_t (*mxosWlanSuspend)( void );
    merr_t (*mwifi_disconnect)( void );
    merr_t (*mwifi_softap_stop)( void );
    merr_t (*mwifi_softap_startEasyLink)( int inTimeout );
    merr_t (*mwifi_softap_startEasyLinkPlus)( int inTimeout );
    merr_t (*mxosWlanStopEasyLink)( void );
    merr_t (*mxosWlanStopEasyLinkPlus)( void );
    merr_t (*mwifi_softap_startWPS)( int inTimeout );
    merr_t (*mxosWlanStopWPS)( void );
    merr_t (*mwifi_softap_startAirkiss)( int inTimeout );
    merr_t (*mwifi_airkiss_stop)( void );
    void (*mwifi_ps_on)( void );
    void (*mwifi_ps_off)( void );
    void (*wifimgr_debug_enable)( bool enable );
    int (*mxos_wlan_monitor_rx_type)( int type );
    int (*mwifi_monitor_start)( void );
    int (*mwifi_monitor_stop)( void );
    int (*mwifi_monitor_set_channel)( int channel );
    void (*mwifi_monitor_reg_cb)( monitor_cb_t fn );
    void (*wlan_set_channel)( int channel );
    int (*mxchip_active_scan)( char*ssid, int is_adv );
    merr_t (*wifi_manage_custom_ie_add)(wlan_if_t wlan_if, uint8_t *custom_ie, uint32_t len);
    merr_t (*wifi_manage_custom_ie_delete)(wlan_if_t wlan_if);

    /* CLI APIs */
    int (*cli_init)(void);
    int (*cli_register_command)(const struct cli_command *command);
    int (*cli_unregister_command)(const struct cli_command *command);
    void (*wifistate_Command)( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv );
    void (*wifidebug_Command)( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv );
    void (*wifiscan_Command)( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv );
    void (*ifconfig_Command)( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv );
    void (*arp_Command)( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv );
    void (*ping_Command)( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv );
    void (*dns_Command)( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv );
    void (*task_Command)( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv );
    void (*socket_show_Command)( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv );
    void (*memory_show_Command)( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv );
    void (*memory_dump_Command)( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv );
    void (*memory_set_Command)( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv );
    void (*memp_dump_Command)( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv );
    void (*driver_state_Command)( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv );
    void (*iperf_Command)( char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv );

    /* HAL: GPIO; FLASH; UART */
    mxos_logic_partition_t* (*mhal_flash_get_info)( mxos_partition_t inPartition );
    merr_t (*mhal_flash_erase)( mxos_partition_t inPartition, uint32_t off_set, uint32_t size );
    merr_t (*mhal_flash_write)( mxos_partition_t inPartition, volatile uint32_t* off_set, uint8_t* inBuffer,
                                uint32_t inBufferLength );
    merr_t (*mhal_flash_read)( mxos_partition_t inPartition, volatile uint32_t* off_set, uint8_t* outBuffer,
                               uint32_t inBufferLength );
    merr_t (*mxos_flash_enable_security)( mxos_partition_t partition, uint32_t off_set, uint32_t size );

    merr_t (*mhal_gpio_open)( mxos_gpio_t gpio, mxos_gpio_config_t configuration );
    merr_t (*mhal_gpio_close)( mxos_gpio_t gpio );
    merr_t (*mhal_gpio_high)( mxos_gpio_t gpio );
    merr_t (*mhal_gpio_low)( mxos_gpio_t gpio );
    merr_t (*mhal_gpio_toggle)( mxos_gpio_t gpio );
    bool (*mhal_gpio_value)( mxos_gpio_t gpio );
    merr_t (*mhal_gpio_int_on)( mxos_gpio_t gpio, mxos_gpio_irq_trigger_t trigger, mxos_gpio_irq_handler_t handler,
                                   void* arg );
    merr_t (*mhal_gpio_int_off)( mxos_gpio_t gpio );

    merr_t (*mhal_uart_open)( mxos_uart_t uart, const mxos_uart_config_t* config,
                                    ring_buffer_t* optional_rx_buffer );
    merr_t (*mhal_uart_close)( mxos_uart_t uart );
    merr_t (*mhal_uart_write)( mxos_uart_t uart, const void* data, uint32_t size );
    merr_t (*mhal_uart_read)( mxos_uart_t uart, void* data, uint32_t size, uint32_t timeout );
    uint32_t (*mhal_uart_readd_data_len)( mxos_uart_t uart );
    void (*MxosUartPinRedirect)( mxos_uart_t uart );

    /* Power management*/
    int (*pm_mcu_state)( power_state_t state, uint32_t time_dur );
    int (*pm_wakeup_source)( uint8_t wake_source );
    void (*wifi_off_mcu_standby)( int seconds );
    void (*mxos_mcu_powersave_config)( int enable );

    /* uitls */
    int (*debug_putchar)( char *ch, int len );
    void (*mxos_sys_reboot)( void );

    /* ALI APIs */
    char* (*get_ali_key)( void );
    char* (*get_ali_secret)( void );

    /* RTC */
    void (*mxos_rtc_init)( void );
    merr_t (*mxos_rtc_get_time)( time_t *t );
    merr_t (*mxos_rtc_set_time)( time_t t );
    struct tm* (*localtime)( const time_t * time );
    char * (*asctime)( const struct tm *tm );

    int (*wifi_set_country)( int country );
    int (*switch_active_firmrware)( void );
    int (*last_reset_reason)( void );
    int (*aon_write)( uint32_t offset, uint8_t* in, uint32_t len );
    int (*aon_read)( uint32_t offset, uint8_t* out, uint32_t len );

    /* LwIP */
    lwip_api_t *lwip_apis;

    /* FreeRTOS */

    /* PWM */
    pwm_api_t *pwm_apis;

    /* WDG */
    wdg_api_t *wdg_apis;

    int (*ssl_get_fd)( const void* ssl );

    void (*get_random_sequence)( unsigned char *buf, unsigned int size );
    adc_api_t *adc_apis;
    i2c_api_t *i2c_apis;
    spi_api_t *spi_apis;
    gtimer_api_t *gtimer_apis;

    int (*ssl_set_loggingcb)( ssl_Logging_cb f );
    int (*wlan_inject_frame)( const uint8_t *buff, size_t len );
    int (*wlan_rx_mgmt_indication)( const enum wlan_bss_type bss_type,
                                    const uint32_t mgmt_subtype_mask,
                                    void (*rx_mgmt_callback)( const enum wlan_bss_type
                                                              bss_type,
                                                              const uint8_t *frame,
                                                              const uint16_t len ) );
    int (*wlan_remain_on_channel)( const bool status, const uint8_t channel,
                                   const uint32_t duration );

    int (*wifi_bridge_mode_enable)( bool hidden_ssid );
    int (*wifi_bridge_mode_disable)( void );

    int (*send_easylink_minus)( uint32_t ip, char *ssid, char *key );

    int (*ssl_socket)( void* ssl );
	int (*mxos_wlan_get_channel)(void);
	
	int (*ssl_pending)(void* ssl);
	int (*ssl_get_error)(void* ssl, int ret);
	void (*ssl_set_using_nonblock)(void* ssl, int nonblock);
	void* (*ssl_nonblock_connect)(int fd, int calen, char*ca, int *errno, int timeout);
	void (*ssl_set_client_cert)(const char *_cert_pem, const char *private_key_pem);
	void* (*ssl_connect_sni)(int fd, int calen, char*ca, char *sni_servername, int *errno);
} mxos_api_t;

typedef struct user_api_struct
{
    uint32_t len;
    uint16_t reserved;
    uint16_t crc16;
    uint32_t magic_num;
    uint32_t app_stack_size;
    uint32_t interface_version;
    char * version;
    char * user_app_version;
    char * PID;
    char * SN;
    mxos_uart_t debug_uart;
    int debug_baudrate;

    void (*user_app_in)( const mxos_api_t *lib_api_t );
    void (*init_platform)( void );
    int (*application_start)( void );

    /* callback functions */
    void (*ApListCallback)( ScanResult *pApList );
    void (*ApListAdvCallback)( ScanResult_adv *pApAdvList );
    void (*WifiStatusHandler)( WiFiEvent status );
    void (*connected_ap_info)( apinfo_adv_t *ap_info, char *key, int key_len );
    void (*NetCallback)( IPStatusTypedef *pnet );
    void (*RptConfigmodeRslt)( mwifi_softap_attr_t *nwkpara );
    void (*easylink_user_data_result)( int datalen, char*data );
    void (*socket_connected)( int fd );
    void (*dns_ip_set)( uint8_t *hostname, uint32_t ip );
    void (*join_fail)( merr_t err );
    void (*wifi_reboot_event)( void );
    void (*mxos_rtos_stack_overflow)( char *taskname );
    const platform_peripherals_pinmap_t *pinmaps;
    const mhal_gpio_open_t *gpio_init;
    const uint8_t stdio_break_in;
} user_api_t;

typedef enum {
  /* CHANNEL PLAN */
	MXOS_COUNTRY_WORLD1,  // 0x20
	MXOS_COUNTRY_ETSI1,   // 0x21
	MXOS_COUNTRY_FCC1,    // 0x22
	MXOS_COUNTRY_MKK1,    // 0x23
	MXOS_COUNTRY_ETSI2,   // 0x24
	MXOS_COUNTRY_FCC2,    // 0x2A
	MXOS_COUNTRY_WORLD2,  // 0x47
	MXOS_COUNTRY_MKK2,    // 0x58

  /* SPECIAL */
	MXOS_COUNTRY_WORLD,  // WORLD1
	MXOS_COUNTRY_EU,     // ETSI1

  /* JAPANESE */
	MXOS_COUNTRY_JP,     // MKK1

  /* FCC , 19 countries*/
	MXOS_COUNTRY_AS,     // FCC2
	MXOS_COUNTRY_BM,
	MXOS_COUNTRY_CA,
	MXOS_COUNTRY_DM,
	MXOS_COUNTRY_DO,
	MXOS_COUNTRY_FM,
	MXOS_COUNTRY_GD,
	MXOS_COUNTRY_GT,
	MXOS_COUNTRY_GU,
	MXOS_COUNTRY_HT,
	MXOS_COUNTRY_MH,
	MXOS_COUNTRY_MP,
	MXOS_COUNTRY_NI,
	MXOS_COUNTRY_PA,
	MXOS_COUNTRY_PR,
	MXOS_COUNTRY_PW,
	MXOS_COUNTRY_TW,
	MXOS_COUNTRY_US,
	MXOS_COUNTRY_VI,

  /* others,  ETSI */
	MXOS_COUNTRY_AD,    // ETSI1
	MXOS_COUNTRY_AE,
	MXOS_COUNTRY_AF,
	MXOS_COUNTRY_AI,
	MXOS_COUNTRY_AL,
	MXOS_COUNTRY_AM,
	MXOS_COUNTRY_AN,
	MXOS_COUNTRY_AR,
	MXOS_COUNTRY_AT,
	MXOS_COUNTRY_AU,
	MXOS_COUNTRY_AW,
	MXOS_COUNTRY_AZ,
	MXOS_COUNTRY_BA,
	MXOS_COUNTRY_BB,
	MXOS_COUNTRY_BD,
	MXOS_COUNTRY_BE,
	MXOS_COUNTRY_BF,
	MXOS_COUNTRY_BG,
	MXOS_COUNTRY_BH,
	MXOS_COUNTRY_BL,
	MXOS_COUNTRY_BN,
	MXOS_COUNTRY_BO,
	MXOS_COUNTRY_BR,
	MXOS_COUNTRY_BS,
	MXOS_COUNTRY_BT,
	MXOS_COUNTRY_BY,
	MXOS_COUNTRY_BZ,
	MXOS_COUNTRY_CF,
	MXOS_COUNTRY_CH,
	MXOS_COUNTRY_CI,
	MXOS_COUNTRY_CL,
	MXOS_COUNTRY_CN,
	MXOS_COUNTRY_CO,
	MXOS_COUNTRY_CR,
	MXOS_COUNTRY_CX,
	MXOS_COUNTRY_CY,
	MXOS_COUNTRY_CZ,
	MXOS_COUNTRY_DE,
	MXOS_COUNTRY_DK,
	MXOS_COUNTRY_DZ,
	MXOS_COUNTRY_EC,
	MXOS_COUNTRY_EE,
	MXOS_COUNTRY_EG,
	MXOS_COUNTRY_ES,
	MXOS_COUNTRY_ET,
	MXOS_COUNTRY_FI,
	MXOS_COUNTRY_FR,
	MXOS_COUNTRY_GB,
	MXOS_COUNTRY_GE,
	MXOS_COUNTRY_GF,
	MXOS_COUNTRY_GH,
	MXOS_COUNTRY_GL,
	MXOS_COUNTRY_GP,
	MXOS_COUNTRY_GR,
	MXOS_COUNTRY_GY,
	MXOS_COUNTRY_HK,
	MXOS_COUNTRY_HN,
	MXOS_COUNTRY_HR,
	MXOS_COUNTRY_HU,
	MXOS_COUNTRY_ID,
	MXOS_COUNTRY_IE,
	MXOS_COUNTRY_IL,
	MXOS_COUNTRY_IN,
	MXOS_COUNTRY_IQ,
	MXOS_COUNTRY_IR,
	MXOS_COUNTRY_IS,
	MXOS_COUNTRY_IT,
	MXOS_COUNTRY_JM,
	MXOS_COUNTRY_JO,
	MXOS_COUNTRY_KE,
	MXOS_COUNTRY_KH,
	MXOS_COUNTRY_KN,
	MXOS_COUNTRY_KP,
	MXOS_COUNTRY_KR,
	MXOS_COUNTRY_KW,
	MXOS_COUNTRY_KY,
	MXOS_COUNTRY_KZ,
	MXOS_COUNTRY_LA,
	MXOS_COUNTRY_LB,
	MXOS_COUNTRY_LC,
	MXOS_COUNTRY_LI,
	MXOS_COUNTRY_LK,
	MXOS_COUNTRY_LR,
	MXOS_COUNTRY_LS,
	MXOS_COUNTRY_LT,
	MXOS_COUNTRY_LU,
	MXOS_COUNTRY_LV,
	MXOS_COUNTRY_MA,
	MXOS_COUNTRY_MC,
	MXOS_COUNTRY_MD,
	MXOS_COUNTRY_ME,
	MXOS_COUNTRY_MF,
	MXOS_COUNTRY_MK,
	MXOS_COUNTRY_MN,
	MXOS_COUNTRY_MO,
	MXOS_COUNTRY_MQ,
	MXOS_COUNTRY_MR,
	MXOS_COUNTRY_MT,
	MXOS_COUNTRY_MU,
	MXOS_COUNTRY_MV,
	MXOS_COUNTRY_MW,
	MXOS_COUNTRY_MX,
	MXOS_COUNTRY_MY,
	MXOS_COUNTRY_NG,
	MXOS_COUNTRY_NL,
	MXOS_COUNTRY_NO,
	MXOS_COUNTRY_NP,
	MXOS_COUNTRY_NZ,
	MXOS_COUNTRY_OM,
	MXOS_COUNTRY_PE,
	MXOS_COUNTRY_PF,
	MXOS_COUNTRY_PG,
	MXOS_COUNTRY_PH,
	MXOS_COUNTRY_PK,
	MXOS_COUNTRY_PL,
	MXOS_COUNTRY_PM,
	MXOS_COUNTRY_PT,
	MXOS_COUNTRY_PY,
	MXOS_COUNTRY_QA,
	MXOS_COUNTRY_RS,
	MXOS_COUNTRY_RU,
	MXOS_COUNTRY_RW,
	MXOS_COUNTRY_SA,
	MXOS_COUNTRY_SE,
	MXOS_COUNTRY_SG,
	MXOS_COUNTRY_SI,
	MXOS_COUNTRY_SK,
	MXOS_COUNTRY_SN,
	MXOS_COUNTRY_SR,
	MXOS_COUNTRY_SV,
	MXOS_COUNTRY_SY,
	MXOS_COUNTRY_TC,
	MXOS_COUNTRY_TD,
	MXOS_COUNTRY_TG,
	MXOS_COUNTRY_TH,
	MXOS_COUNTRY_TN,
	MXOS_COUNTRY_TR,
	MXOS_COUNTRY_TT,
	MXOS_COUNTRY_TZ,
	MXOS_COUNTRY_UA,
	MXOS_COUNTRY_UG,
	MXOS_COUNTRY_UY,
	MXOS_COUNTRY_UZ,
	MXOS_COUNTRY_VC,
	MXOS_COUNTRY_VE,
	MXOS_COUNTRY_VN,
	MXOS_COUNTRY_VU,
	MXOS_COUNTRY_WF,
	MXOS_COUNTRY_WS,
	MXOS_COUNTRY_YE,
	MXOS_COUNTRY_YT,
	MXOS_COUNTRY_ZA,
	MXOS_COUNTRY_ZW,

	MXOS_COUNTRY_MAX

}mxos_country_code_t;

#endif
