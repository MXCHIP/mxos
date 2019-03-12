#include "mxos_common.h"
#include "moc_api.h"

extern const mxos_api_t *lib_api_p;

extern platform_spi_driver_t            platform_spi_drivers[];
extern platform_i2c_driver_t            platform_i2c_drivers[];


/* WIFI MGR */
merr_t StartNetwork(network_InitTypeDef_st* inNetworkInitPara)
{
	return lib_api_p->mxosWlanStart(inNetworkInitPara);
}
merr_t StartAdvNetwork(network_InitTypeDef_adv_st* inNetworkInitParaAdv)
{
	return lib_api_p->mxosWlanStartAdv(inNetworkInitParaAdv);
}

merr_t mxosWlanStartEnt(network_Enterprise_st* inNetworkInitPara)
{
	return lib_api_p->mxosWlanStartEnt(inNetworkInitPara);
}

merr_t getNetPara(IPStatusTypedef *outNetpara, WiFi_Interface inInterface)
{
	return lib_api_p->mxosWlanGetIPStatus(outNetpara, inInterface);
}
merr_t mxosWlanGetIP6Status(ipv6_addr_t ipv6_addr[], uint8_t ipv6_addr_num, WiFi_Interface inInterface)
{
#ifdef MOCIP_CONFIG_IPV6
    return lib_api_p->mxosWlanGetIP6Status(ipv6_addr, ipv6_addr_num, inInterface);
#else
    return kUnsupportedErr;
#endif
}
merr_t CheckNetLink(LinkStatusTypeDef *outStatus)
{
	return lib_api_p->mxosWlanGetLinkStatus(outStatus);
}
void mxchipStartScan(void)
{
	lib_api_p->mxosWlanStartScan();
}
void mxchipStartAdvScan(void)
{
	lib_api_p->mxosWlanStartScanAdv();
}
merr_t wifi_power_down(void)
{
	return lib_api_p->mxosWlanPowerOff();
}
merr_t wifi_power_up(void)
{
	return lib_api_p->mxosWlanPowerOn();
}
merr_t wlan_disconnect(void)
{
	return lib_api_p->mxosWlanSuspend();
}
merr_t sta_disconnect(void)
{
	return lib_api_p->mxosWlanSuspendStation();
}
merr_t uap_stop(void)
{
	return lib_api_p->mxosWlanSuspendSoftAP();
}
merr_t OpenEasylink2_withdata(int inTimeout)
{
	return lib_api_p->mxosWlanStartEasyLink(inTimeout);
}
merr_t OpenEasylink(int inTimeout)
{
	return lib_api_p->mxosWlanStartEasyLinkPlus(inTimeout);
}
merr_t CloseEasylink2(void)
{
	return lib_api_p->mxosWlanStopEasyLink();
}
merr_t CloseEasylink(void)
{
	return lib_api_p->mxosWlanStopEasyLinkPlus();
}
merr_t OpenConfigmodeWPS(int inTimeout)
{
	return lib_api_p->mxosWlanStartWPS(inTimeout);
}
merr_t CloseConfigmodeWPS(void)
{
	return lib_api_p->mxosWlanStopWPS();
}
merr_t OpenAirkiss(int inTimeout)
{
	return lib_api_p->mxosWlanStartAirkiss(inTimeout);
}
merr_t CloseAirkiss(void)
{
	return lib_api_p->mxosWlanStopAirkiss();
}
void ps_enable(void)
{
	lib_api_p->mxosWlanEnablePowerSave();
}
void ps_disable(void)
{
	lib_api_p->mxosWlanDisablePowerSave();
}
void wifimgr_debug_enable(bool enable)
{
	lib_api_p->wifimgr_debug_enable(enable);
}

int mxos_wlan_monitor_no_easylink(void)
{
	return lib_api_p->mxos_wlan_monitor_no_easylink();
}


int mxos_wlan_monitor_rx_type(int type)
{
	return lib_api_p->mxos_wlan_monitor_rx_type(type);
}
int mxos_wlan_start_monitor(void)
{
	return lib_api_p->mxos_wlan_start_monitor();
}
int mxos_wlan_stop_monitor(void)
{
	return lib_api_p->mxos_wlan_stop_monitor();
}
int mxos_wlan_monitor_set_channel(uint8_t channel)
{
	return lib_api_p->mxos_wlan_monitor_set_channel((int)channel);
}
void mxos_wlan_register_monitor_cb(monitor_cb_t fn)
{
	lib_api_p->mxos_wlan_register_monitor_cb(fn);
}

int mxchip_active_scan(char*ssid, int is_adv)
{
	return lib_api_p->mxchip_active_scan(ssid, is_adv);
}

void wlan_set_channel(int channel)
{
    lib_api_p->wlan_set_channel(channel);
}


/* HAL: GPIO
{
	return lib_api_p->;
} FLASH
{
	return lib_api_p->;
} UART */
mxos_logic_partition_t* mxos_flash_get_info( mxos_partition_t inPartition )
{
	return lib_api_p->mxos_flash_get_info(inPartition);
}
merr_t mxos_flash_erase(mxos_partition_t inPartition, uint32_t off_set, uint32_t size)
{
	return lib_api_p->mxos_flash_erase(inPartition, off_set, size);
}
merr_t mxos_flash_write( mxos_partition_t inPartition, volatile uint32_t* off_set, uint8_t* inBuffer ,uint32_t inBufferLength)
{
	 lib_api_p->mxos_flash_write(inPartition, off_set, inBuffer, inBufferLength);
	 return 0;
}
merr_t mxos_flash_read( mxos_partition_t inPartition, volatile uint32_t* off_set, uint8_t* outBuffer, uint32_t inBufferLength)
{
	return lib_api_p->mxos_flash_read(inPartition, off_set, outBuffer, inBufferLength);
}
merr_t mxos_flash_enable_security( mxos_partition_t partition, uint32_t off_set, uint32_t size )
{
	return lib_api_p->mxos_flash_enable_security(partition, off_set, size );
}

merr_t mxos_gpio_init( mxos_gpio_t gpio, mxos_gpio_config_t configuration )
{
	return lib_api_p->mxos_gpio_init(gpio, configuration );
}
merr_t mxos_gpio_deinit( mxos_gpio_t gpio )
{
	return lib_api_p->mxos_gpio_deinit(gpio);
}
merr_t mxos_gpio_output_high( mxos_gpio_t gpio )
{
	return lib_api_p->mxos_gpio_output_high(gpio);
}
merr_t mxos_gpio_output_low( mxos_gpio_t gpio )
{
	return lib_api_p->mxos_gpio_output_low(gpio);
}
merr_t mxos_gpio_output_toggle( mxos_gpio_t gpio )
{
	return lib_api_p->mxos_gpio_output_toggle(gpio);
}
bool mxos_gpio_input_get( mxos_gpio_t gpio )
{
	return lib_api_p->mxos_gpio_input_get(gpio);
}
merr_t mxos_gpio_enable_irq( mxos_gpio_t gpio, mxos_gpio_irq_trigger_t trigger, mxos_gpio_irq_handler_t handler, void* arg )
{
	return lib_api_p->mxos_gpio_enable_irq(gpio, trigger, handler, arg );
}
merr_t mxos_gpio_disable_irq( mxos_gpio_t gpio )
{
	return lib_api_p->mxos_gpio_disable_irq(gpio);
}

merr_t mxos_uart_init( mxos_uart_t uart, const mxos_uart_config_t* config, ring_buffer_t* optional_rx_buffer )
{
	return lib_api_p->mxos_uart_init(uart, config, optional_rx_buffer );
}

merr_t mxos_stdio_uart_init( const mxos_uart_config_t* config, ring_buffer_t* optional_rx_buffer )
{
    return lib_api_p->mxos_uart_init(MXOS_STDIO_UART, config, optional_rx_buffer );
}

merr_t mxos_uart_deinit( mxos_uart_t uart )
{
	return lib_api_p->mxos_uart_deinit(uart);
}
merr_t mxos_uart_send( mxos_uart_t uart, const void* data, uint32_t size )
{
	return lib_api_p->mxos_uart_send(uart, data, size );
}
merr_t mxos_uart_recv( mxos_uart_t uart, void* data, uint32_t size, uint32_t timeout )
{
	return lib_api_p->mxos_uart_recv(uart, data, size, timeout );
}
uint32_t mxos_uart_recvd_data_len( mxos_uart_t uart )
{
	return lib_api_p->mxos_uart_recvd_data_len(uart);
}
merr_t MxosUartPinRedirect( mxos_uart_t uart )
{
	lib_api_p->MxosUartPinRedirect(uart);
	return kNoErr;
}

void wifistate_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	lib_api_p->wifistate_Command(pcWriteBuffer, xWriteBufferLen, argc, argv);
}

void wifidebug_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	lib_api_p->wifidebug_Command(pcWriteBuffer, xWriteBufferLen, argc, argv);
}
void wifiscan_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	lib_api_p->wifiscan_Command(pcWriteBuffer, xWriteBufferLen, argc, argv);
}

void ifconfig_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	lib_api_p->ifconfig_Command(pcWriteBuffer, xWriteBufferLen, argc, argv);
}
void arp_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	lib_api_p->arp_Command(pcWriteBuffer, xWriteBufferLen, argc, argv);
}
void ping_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	lib_api_p->ping_Command(pcWriteBuffer, xWriteBufferLen, argc, argv);
}
void dns_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	lib_api_p->dns_Command(pcWriteBuffer, xWriteBufferLen, argc, argv);
}
void task_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	lib_api_p->task_Command(pcWriteBuffer, xWriteBufferLen, argc, argv);
}
void socket_show_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	lib_api_p->socket_show_Command(pcWriteBuffer, xWriteBufferLen, argc, argv);
}
void memory_show_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	lib_api_p->memory_show_Command(pcWriteBuffer, xWriteBufferLen, argc, argv);
}
void memory_dump_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	lib_api_p->memory_dump_Command(pcWriteBuffer, xWriteBufferLen, argc, argv);
}
void memory_set_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	lib_api_p->memory_set_Command(pcWriteBuffer, xWriteBufferLen, argc, argv);
}
void memp_dump_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	lib_api_p->memp_dump_Command(pcWriteBuffer, xWriteBufferLen, argc, argv);
}
void driver_state_Command(char *pcWriteBuffer, int xWriteBufferLen, int argc, char **argv)
{
	lib_api_p->driver_state_Command(pcWriteBuffer, xWriteBufferLen, argc, argv);
}

int mxos_wlan_driver_version( char* outVersion, uint8_t inLength )
{
	return lib_api_p->mxos_wlan_driver_version(outVersion, inLength);
}

void mxos_wlan_get_mac_address( uint8_t *mac )
{
	lib_api_p->wlan_get_mac_address(mac);
}

void InitMd5(md5_context*md5)
{
	lib_api_p->InitMd5(md5);
}
void Md5Update(md5_context* md5, unsigned char *input, int ilen)
{
	lib_api_p->Md5Update(md5, input, ilen);
}
void Md5Final(md5_context* md5, uint8_t* hash)
{
	lib_api_p->Md5Final(md5, hash);
}
int Md5Hash(const uint8_t* data, uint32_t len, uint8_t* hash)
{
	return lib_api_p->Md5Hash(data, len, hash);
}
void AesEncryptDirect(Aes* aes, uint8_t* out, const uint8_t* in)
{
	lib_api_p->AesEncryptDirect(aes, out, in);
}
void AesDecryptDirect(Aes* aes, uint8_t* out, const uint8_t* in)
{
	lib_api_p->AesDecryptDirect(aes, out, in);
}
//int  AesSetKeyDirect(Aes* aes, const byte* key, word32 ilen,const byte* out, int dir);
int AesSetKeyDirect(Aes* aes, const unsigned char* key, unsigned int len, const unsigned char* iv, int dir)
{
	return lib_api_p->AesSetKeyDirect(aes, key, len, iv, dir);
}
int aes_encrypt(int sz, const char * key, const char * in, char * out)
{
	return lib_api_p->aes_encrypt(sz, key, in, out);
}
int aes_decrypt(int sz, const char * key, const char * in, char * out)
{
	return lib_api_p->aes_decrypt(sz, key, in, out);
}
int  AesSetKey(Aes* aes, const uint8_t* key, unsigned int  len,
                          const uint8_t* iv, int dir)
{
	return lib_api_p->AesSetKey(aes, key, len, iv, dir);
}
int  AesSetIV(Aes* aes, const uint8_t* iv)
{
	return lib_api_p->AesSetIV(aes, iv);
}
int  AesCbcEncrypt(Aes* aes, uint8_t* out,
                              const uint8_t* in, unsigned int sz)
{
	return lib_api_p->AesCbcEncrypt(aes, out, in, sz);
}
int  AesCbcDecrypt(Aes* aes, uint8_t* out,
                              const uint8_t* in, unsigned int sz){
	return lib_api_p->AesCbcDecrypt(aes, out, in, sz);
}

merr_t mxos_network_init(void)
{
	lib_api_p->mxos_network_init();
	return kNoErr;
}

mxosMemInfo_t* mxos_memory_info(void)
{
	return lib_api_p->mxos_memory_info();
}
char* mxos_system_lib_version(void)
{
    return lib_api_p->library_version;
}

void mxos_sys_reboot(void)
{
	lib_api_p->mxos_sys_reboot();
}

char *mxos_get_bootloader_ver(void)
{
	return "bootloader";
}

void mxos_sys_standby( uint32_t secondsToWakeup )
{
  lib_api_p->wifi_off_mcu_standby(secondsToWakeup);
}

char *get_ali_key(void)
{
	return lib_api_p->get_ali_key();
}

char *get_ali_secret(void)
{
	return lib_api_p->get_ali_secret();
}

merr_t mxos_rtc_init(void)
{
	lib_api_p->mxos_rtc_init();
	return kNoErr;
}

merr_t mxos_rtc_get_time(time_t* time)
{
	return lib_api_p->mxos_rtc_get_time(time);
}

merr_t mxos_rtc_set_time(time_t time)
{
	return lib_api_p->mxos_rtc_set_time(time);
}

#if defined ( __ICCARM__ )
struct tm *localtime(const time_t * time)
{
	return lib_api_p->localtime(time);
}
#endif

char *asctime(const struct tm *tm)
{
	return lib_api_p->asctime(tm);
}

int wifi_set_country(int country)
{
	return lib_api_p->wifi_set_country(country);
}

int switch_active_firmware(void)
{
	return lib_api_p->switch_active_firmrware();
}

int get_last_reset_reason(void)
{
	return lib_api_p->last_reset_reason();
}

void system_config_set(mxos_system_config_t *cfg)
{
	lib_api_p->system_config_set(cfg);
}

mxos_system_config_t *system_config_get(void)
{
	return lib_api_p->system_config_get();
}

int aon_write( uint32_t offset, uint8_t* in ,uint32_t len)
{
	return lib_api_p->aon_write(offset, in, len);
}

int aon_read( uint32_t offset, uint8_t* out, uint32_t len)
{
	return lib_api_p->aon_read(offset, out, len);
}


int lwip_ioctl(int s, long cmd, void *argp)
{
	return lib_api_p->lwip_apis->lwip_ioctl(s, cmd, argp);
}
int lwip_fcntl(int s, int cmd, int val)
{
	return lib_api_p->lwip_apis->lwip_fcntl(s, cmd, val);
}

void lwip_freeaddrinfo(struct addrinfo *ai)
{
	lib_api_p->lwip_apis->lwip_freeaddrinfo(ai);
}

int lwip_getaddrinfo(const char *nodename,
	   const char *servname,
	   const struct addrinfo *hints,
	   struct addrinfo **res)
{
	return lib_api_p->lwip_apis->lwip_getaddrinfo(nodename,
	   			servname, hints, res);
}

char * ipaddr_ntoa(const ip_addr_t *addr)
{
	return lib_api_p->lwip_apis->ipaddr_ntoa(addr);
}
uint32_t ipaddr_addr(const char *cp)
{
	return lib_api_p->lwip_apis->ipaddr_addr(cp);
}

uint16_t
lwip_htons(uint16_t n)
{
  return ((n & 0xff) << 8) | ((n & 0xff00) >> 8);
}

/**
 * Convert an uint16_t from network- to host byte order.
 *
 * @param n uint16_t in network byte order
 * @return n in host byte order
 */
uint16_t
lwip_ntohs(uint16_t n)
{
  return lwip_htons(n);
}

/**
 * Convert an uint32_t from host- to network byte order.
 *
 * @param n uint32_t in host byte order
 * @return n in network byte order
 */
uint32_t
lwip_htonl(uint32_t n)
{
  return ((n & 0xff) << 24) |
    ((n & 0xff00) << 8) |
    ((n & 0xff0000UL) >> 8) |
    ((n & 0xff000000UL) >> 24);
}

/**
 * Convert an uint32_t from network- to host byte order.
 *
 * @param n uint32_t in network byte order
 * @return n in host byte order
 */
uint32_t
lwip_ntohl(uint32_t n)
{
  return lwip_htonl(n);
}

merr_t mxos_pwm_init(mxos_pwm_t pwm, uint32_t freequency, float duty_cycle)
{
	return lib_api_p->pwm_apis->pwm_init(pwm, freequency, duty_cycle);
}

merr_t mxos_pwm_start(mxos_pwm_t pwm)
{
	return lib_api_p->pwm_apis->pwm_start(pwm);
}

merr_t mxos_pwm_stop(mxos_pwm_t pwm)
{
	return lib_api_p->pwm_apis->pwm_stop(pwm);
}

merr_t mxos_wdg_init( uint32_t timeout )
{
	return lib_api_p->wdg_apis->wdg_init(timeout);
}

void mxos_wdg_reload( void )
{
	lib_api_p->wdg_apis->wdg_reload();
}

merr_t mxos_wdg_deinit( void )
{
	return lib_api_p->wdg_apis->wdg_stop();
}

int Cyassl_get_fd(const void *ssl)
{
	return lib_api_p->ssl_get_fd(ssl);
}

merr_t mxos_adc_init( mxos_adc_t adc, uint32_t sampling_cycle )
{
	return lib_api_p->adc_apis->mxos_adc_init(adc, sampling_cycle);
}

merr_t  mxos_adc_deinit( mxos_adc_t adc )
{
    return lib_api_p->adc_apis->mxos_adc_deinit(adc);
}

merr_t mxos_adc_take_sample( mxos_adc_t adc, uint16_t* output )
{
    return lib_api_p->adc_apis->mxos_adc_take_sample(adc, output);
}

merr_t mxos_adc_take_sampleStreram( mxos_adc_t adc, void* buffer, uint16_t buffer_length )
{
    return lib_api_p->adc_apis->mxos_adc_take_sampleStreram(adc, buffer, buffer_length);
}

merr_t mxos_i2c_init( mxos_i2c_device_t* device )
{
    merr_t result;

    if( platform_i2c_drivers[device->port].i2c_mutex == NULL)
      mxos_rtos_init_mutex( &platform_i2c_drivers[device->port].i2c_mutex );

    mxos_rtos_lock_mutex( &platform_i2c_drivers[device->port].i2c_mutex );
    result = lib_api_p->i2c_apis->i2c_init(device);
    mxos_rtos_unlock_mutex( &platform_i2c_drivers[device->port].i2c_mutex );
    return result;
}

merr_t mxos_i2c_deinit( mxos_i2c_device_t* device )
{
    if( platform_i2c_drivers[device->port].i2c_mutex != NULL){
      mxos_rtos_deinit_mutex( &platform_i2c_drivers[device->port].i2c_mutex );
      platform_i2c_drivers[device->port].i2c_mutex = NULL;
    }

    return lib_api_p->i2c_apis->i2c_deinit(device);
}

bool mxos_i2c_probe_dev( mxos_i2c_device_t* device, int retries )
{
    bool ret;

    mxos_rtos_lock_mutex( &platform_i2c_drivers[device->port].i2c_mutex );
    ret = lib_api_p->i2c_apis->i2c_probe_device(device, retries);
    mxos_rtos_unlock_mutex( &platform_i2c_drivers[device->port].i2c_mutex );
    return ret;
}

merr_t mxos_i2c_build_tx_msg( mxos_i2c_message_t* message, const void* tx_buffer, uint16_t  tx_buffer_length, uint16_t retries )
{
    return lib_api_p->i2c_apis->i2c_build_tx_msg(message, tx_buffer, tx_buffer_length, retries );
}

merr_t mxos_i2c_build_rx_msg( mxos_i2c_message_t* message, void* rx_buffer, uint16_t rx_buffer_length, uint16_t retries )
{
	return lib_api_p->i2c_apis->i2c_build_rx_msg(message, rx_buffer, rx_buffer_length, retries );
}

merr_t mxos_i2c_build_comb_msg( mxos_i2c_message_t* message, const void* tx_buffer, void* rx_buffer, uint16_t tx_buffer_length, uint16_t rx_buffer_length, uint16_t retries )
{
	return lib_api_p->i2c_apis->i2c_build_combined_msg(message, tx_buffer, rx_buffer,
										tx_buffer_length, rx_buffer_length, retries );
}

merr_t mxos_i2c_transfer( mxos_i2c_device_t* device, mxos_i2c_message_t* messages, uint16_t number_of_messages )
{
    merr_t err = kNoErr;

    mxos_rtos_lock_mutex( &platform_i2c_drivers[device->port].i2c_mutex );
    err = lib_api_p->i2c_apis->i2c_transfer(device, messages, number_of_messages);
    mxos_rtos_unlock_mutex( &platform_i2c_drivers[device->port].i2c_mutex );

    return err;
}

merr_t mxos_spi_init( const mxos_spi_device_t* spi )
{
    // if ( platform_spi_drivers[spi->port].initialized == MXOS_TRUE )
    //     return kNoErr;
    lib_api_p->spi_apis->spi_init(spi);
    // platform_spi_drivers[spi->port].initialized = MXOS_TRUE;
    return kNoErr;
}

merr_t mxos_spi_deinit( const mxos_spi_device_t* spi )
{
    return lib_api_p->spi_apis->spi_finalize(spi);
}

merr_t mxos_spi_transfer( const mxos_spi_device_t* spi, const mxos_spi_message_segment_t* segments, uint16_t number_of_segments )
{
	return lib_api_p->spi_apis->spi_transfer(spi, segments, number_of_segments );
}


merr_t MxosRandomNumberRead( void *inBuffer, int inByteCount )
{
    lib_api_p->get_random_sequence(inBuffer, inByteCount);
	return kNoErr;
}

uint32_t RNG_GetRandomNumber(void)
{
    uint32_t d;
    MxosRandomNumberRead((unsigned char*)&d, 4);
    return d;
}

int wlan_inject_frame(const uint8_t *buff, size_t len)
{
	return lib_api_p->wlan_inject_frame(buff, len);
}

merr_t mxos_wlan_send_mgnt(uint8_t *buffer, uint32_t length)
{
	// I don't know the return value;
	lib_api_p->wlan_inject_frame(buffer, length);
	return kNoErr;
}

/* 3031 MCU power save mode set to PM2 */
void mxos_mcu_powersave_config(int enable)
{
    if (enable) {
        lib_api_p->pm_mcu_cfg(true, PM2, 10);// auto go to sleep mode if sleep time bigger than 10ms.
    } else {
        lib_api_p->pm_mcu_cfg(false, PM2, 10);
    }
}

/**
 * This API can be used to start/stop the management frame forwards
 * to host through datapath.
 *
 * \param[in] bss_type The interface from which management frame needs to be
 *	   collected.
 * \param[in] mgmt_subtype_mask     Management Subtype Mask
 *	      If Bit X is set in mask, it means that IEEE Management Frame
 *	      SubTyoe X is to be filtered and passed through to host.
 *            Bit                   Description
 *	      [31:14]               Reserved
 *	      [13]                  Action frame
 *	      [12:9]                Reserved
 *	      [8]                   Beacon
 *	      [7:6]                 Reserved
 *	      [5]                   Probe response
 *	      [4]                   Probe request
 *	      [3]                   Reassociation response
 *	      [2]                   Reassociation request
 *	      [1]                   Association response
 *	      [0]                   Association request
 *	      Support multiple bits set.
 *	      0 = stop forward frame
 *	      1 = start forward frame
 *
 *\param[in] rx_mgmt_callback The receive callback where the received management
 *	  frames are passed.

 * \return WM_SUCCESS if operation is successful.
 * \return -WM_FAIL if command fails.
 *
 * \note Pass Management Subtype Mask all zero to disable all the management
 * 	 frame forward to host.
 */
int wlan_rx_mgmt_indication(const enum wlan_bss_type bss_type,
			const uint32_t mgmt_subtype_mask,
			void (*rx_mgmt_callback)(const enum wlan_bss_type
				bss_type, const uint8_t *frame,
				const uint16_t len))
{
	return lib_api_p->wlan_rx_mgmt_indication(
		bss_type, mgmt_subtype_mask, rx_mgmt_callback);
}


int wlan_remain_on_channel(const bool status, const uint8_t channel,
				const uint32_t ms)
{
	return lib_api_p->wlan_remain_on_channel(status, channel, ms);
}

int wifi_bridge_mode_enable(bool hidden_ssid)
{
	return lib_api_p->wifi_bridge_mode_enable(hidden_ssid);
}

int wifi_bridge_mode_disable(void)
{
	return lib_api_p->wifi_bridge_mode_disable();
}

int send_easylink_minus(uint32_t ip, char *ssid, char *key)
{
	return lib_api_p->send_easylink_minus(ip, ssid, key);
}

merr_t mxos_i2s_init( const mxos_iis_device_t* iis )
{
  	return lib_api_p->iis_apis->iis_init(iis);
}

merr_t mxos_i2s_deinit( const mxos_iis_device_t* iis )
{
    return lib_api_p->iis_apis->iis_finalize(iis);
}

merr_t mxos_i2s_transfer( const mxos_iis_device_t* iis, const mxos_iis_message_segment_t* segments, uint16_t number_of_segments )
{
	return lib_api_p->iis_apis->iis_transfer(iis,segments,number_of_segments);
}

merr_t mxos_i2s_write( const mxos_iis_device_t* iis, uint8_t *p_buf, uint32_t size )
{
	return lib_api_p->iis_apis->iis_write(iis, p_buf, size);
}

merr_t mxos_i2s_read( const mxos_iis_device_t* iis, uint8_t *p_buf, uint32_t size )
{
	return lib_api_p->iis_apis->iis_read(iis, p_buf, size);
}


merr_t mxos_wlan_get_channel( uint8_t *channel )
{
  *channel = lib_api_p->mxos_wlan_get_channel();
  return kNoErr;
}
