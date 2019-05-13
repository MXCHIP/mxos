/**
 ******************************************************************************
 * @file    mxos_platform_common.c
 * @author  William Xu
 * @version V1.0.0
 * @date    20-March-2015
 * @brief   This file provide MXOS driver functions.
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#include <stdint.h>
#include <string.h>
#include "mxos_common.h"
#include "platform_peripheral.h"
#include "mxos_platform.h"
#include "mxos_board_conf.h"
#include "platform_logging.h"

#ifndef BOOTLOADER

#ifdef USE_MXOSKit_EXT
#include "MXOSKit_EXT/mxoskit_ext.h"   // extension sensor board.
#endif

#ifdef USE_MXOSKit_STMEMS
#include "MXOSKit_STmems/MXOSKit_STmems.h"   // extension sensor board.
#endif

#endif

/******************************************************
*                      Macros
******************************************************/

/******************************************************
*                    Constants
******************************************************/

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
*               Static Function Declarations
******************************************************/

extern merr_t mxos_platform_init      ( void );

/******************************************************
*               Variable Definitions
******************************************************/

/* Externed from platforms/<Platform>/platform.c */
extern const mxos_gpio_t            platform_gpio_pins[];
extern const mxos_adc_t             platform_adc_peripherals[];
extern const mxos_i2c_t             platform_i2c_peripherals[];
extern platform_i2c_driver_t            platform_i2c_drivers[];
extern const mxos_pwm_t             platform_pwm_peripherals[];
extern const mxos_spi_t             platform_spi_peripherals[];
extern platform_spi_driver_t            platform_spi_drivers[];
extern const mxos_uart_t            platform_uart_peripherals[];
extern platform_uart_driver_t           platform_uart_drivers[];
extern WEAK platform_spi_slave_driver_t platform_spi_slave_drivers[];
extern const mxos_flash_t           platform_flash_peripherals[];
extern platform_flash_driver_t          platform_flash_drivers[];
extern const mxos_logic_partition_t     mxos_partitions[];

#ifdef MXOS_WIFI_SHARE_SPI_BUS
extern const mxos_spi_t wifi_spi;
#endif

/******************************************************
*               Function Definitions
******************************************************/

merr_t mxos_platform_init( void )
{
#if defined(__CC_ARM)
  platform_log("Platform initialised, build by RVMDK");
#elif defined (__IAR_SYSTEMS_ICC__)
  platform_log("Platform initialised, build by IAR");
#elif defined (__GNUC__)
  platform_log("Platform initialised, build by GNUC");
#endif
  
  if ( true == platform_watchdog_check_last_reset() )
  {
    platform_log( "WARNING: Watchdog reset occured previously. Please see platform_watchdog.c for debugging instructions." );
  }

#ifdef USES_RESOURCE_FILESYSTEM
  platform_filesystem_init();
#endif
 
#ifndef BOOTLOADER
  
#ifdef USE_MXOSKit_EXT
  mxoskit_ext_init();
#endif
  
#ifdef USE_MXOSKit_STMEMS
  mxoskit_STmems_init();
#endif
  
#endif
  
  return kNoErr;
}

merr_t mxos_adc_init( mxos_adc_t adc, uint32_t sampling_cycle )
{
  if ( adc >= MXOS_ADC_NONE )
    return kUnsupportedErr;
  return (merr_t) platform_adc_init( &platform_adc_peripherals[adc], sampling_cycle );
}

merr_t  mxos_adc_deinit( mxos_adc_t adc )
{
  if ( adc >= MXOS_ADC_NONE )
    return kUnsupportedErr;
  return (merr_t) platform_adc_deinit( &platform_adc_peripherals[adc] );
}

uint16_t mxos_adc_get_bit_range( mxos_adc_t adc )
{
    if ( adc >= MXOS_ADC_NONE )
      return kUnsupportedErr;
    return platform_adc_get_bit_range( &platform_adc_peripherals[adc] );
}

merr_t mxos_adc_take_sample( mxos_adc_t adc, uint16_t* output )
{
  if ( adc >= MXOS_ADC_NONE )
    return kUnsupportedErr;
  return (merr_t) platform_adc_take_sample( &platform_adc_peripherals[adc], output );
}

merr_t mxos_adc_take_sampleStream( mxos_adc_t adc, void* buffer, uint16_t buffer_length )
{
  if ( adc >= MXOS_ADC_NONE )
    return kUnsupportedErr;
  return (merr_t) platform_adc_take_sample_stream( &platform_adc_peripherals[adc], buffer, buffer_length );
}

merr_t mhal_gpio_open( mxos_gpio_t gpio, mxos_gpio_config_t configuration )
{
  if ( gpio >= MXOS_GPIO_NONE )
    return kUnsupportedErr;
  return (merr_t) platform_gpio_init( &platform_gpio_pins[gpio], configuration );
}

merr_t mhal_gpio_high( mxos_gpio_t gpio )
{
  if ( gpio >= MXOS_GPIO_NONE )
    return kUnsupportedErr;
  return (merr_t) platform_gpio_output_high( &platform_gpio_pins[gpio] );
}

merr_t mhal_gpio_low( mxos_gpio_t gpio )
{
  if ( gpio >= MXOS_GPIO_NONE )
    return kUnsupportedErr;
  return (merr_t) platform_gpio_output_low( &platform_gpio_pins[gpio] );
}

merr_t mhal_gpio_toggle( mxos_gpio_t gpio )
{
  if ( gpio >= MXOS_GPIO_NONE )
    return kUnsupportedErr;
  return (merr_t) platform_gpio_output_trigger( &platform_gpio_pins[gpio] );
}

merr_t mhal_gpio_close( mxos_gpio_t gpio )
{
  if ( gpio >= MXOS_GPIO_NONE )
    return kUnsupportedErr;
  return (merr_t) platform_gpio_deinit( &platform_gpio_pins[gpio] );
}

bool mhal_gpio_value( mxos_gpio_t gpio )
{
  if ( gpio >= MXOS_GPIO_NONE )
    return kUnsupportedErr;
  return platform_gpio_input_get( &platform_gpio_pins[gpio] );
}

merr_t mhal_gpio_int_on( mxos_gpio_t gpio, mxos_gpio_irq_trigger_t trigger, mxos_gpio_irq_handler_t handler, void* arg )
{
  if ( gpio >= MXOS_GPIO_NONE )
    return kUnsupportedErr;
  return (merr_t) platform_gpio_irq_enable( &platform_gpio_pins[gpio], trigger, handler, arg );
}

merr_t mhal_gpio_int_off( mxos_gpio_t gpio )
{
  if ( gpio >= MXOS_GPIO_NONE )
    return kUnsupportedErr;
  return (merr_t) platform_gpio_irq_disable( &platform_gpio_pins[gpio] );
}

merr_t mxos_i2c_init( mxos_i2c_device_t* device )
{
  mxos_i2c_config_t config;
  merr_t result;

  if ( device->port >= MXOS_I2C_NONE )
    return kUnsupportedErr;
 
  config.address       = device->address;
  config.address_width = device->address_width;
  config.flags         &= ~I2C_DEVICE_USE_DMA ;
  config.speed_mode    = device->speed_mode;

  if( platform_i2c_drivers[device->port].i2c_mutex == NULL)
    platform_i2c_drivers[device->port].i2c_mutex = mos_mutex_new( );
  
  mos_mutex_lock(platform_i2c_drivers[device->port].i2c_mutex );
  result = (merr_t) platform_i2c_init( &platform_i2c_peripherals[device->port], &config );
  mos_mutex_unlock(platform_i2c_drivers[device->port].i2c_mutex );

  return result;
}

merr_t mxos_i2c_deinit( mxos_i2c_device_t* device )
{
  mxos_i2c_config_t config;

  if ( device->port >= MXOS_I2C_NONE )
    return kUnsupportedErr;
  
  config.address       = device->address;
  config.address_width = device->address_width;
  config.flags         &= ~I2C_DEVICE_USE_DMA ;
  config.speed_mode    = device->speed_mode;

  if( platform_i2c_drivers[device->port].i2c_mutex != NULL){
    mos_mutex_delete( platform_i2c_drivers[device->port].i2c_mutex );
    platform_i2c_drivers[device->port].i2c_mutex = NULL;
  }
    
  return (merr_t) platform_i2c_deinit( &platform_i2c_peripherals[device->port], &config );
}

bool mxos_i2c_probe_dev( mxos_i2c_device_t* device, int retries )
{
  bool ret;
  mxos_i2c_config_t config;

  if ( device->port >= MXOS_I2C_NONE )
    return kUnsupportedErr;
  
  config.address       = device->address;
  config.address_width = device->address_width;
  config.flags         &= ~I2C_DEVICE_USE_DMA ;
  config.speed_mode    = device->speed_mode;

  mos_mutex_lock(platform_i2c_drivers[device->port].i2c_mutex );
  ret = platform_i2c_probe_device( &platform_i2c_peripherals[device->port], &config, retries );
  mos_mutex_unlock(platform_i2c_drivers[device->port].i2c_mutex );

  return ret;
}

merr_t mxos_i2c_build_tx_msg( mxos_i2c_message_t* message, const void* tx_buffer, uint16_t  tx_buffer_length, uint16_t retries )
{
  return (merr_t) platform_i2c_init_tx_message( message, tx_buffer, tx_buffer_length, retries );
}

merr_t mxos_i2c_build_rx_msg( mxos_i2c_message_t* message, void* rx_buffer, uint16_t rx_buffer_length, uint16_t retries )
{
  return (merr_t) platform_i2c_init_rx_message( message, rx_buffer, rx_buffer_length, retries );
}

merr_t mxos_i2c_build_comb_msg( mxos_i2c_message_t* message, const void* tx_buffer, void* rx_buffer, uint16_t tx_buffer_length, uint16_t rx_buffer_length, uint16_t retries )
{
  return (merr_t) platform_i2c_init_combined_message( message, tx_buffer, rx_buffer, tx_buffer_length, rx_buffer_length, retries );
}

merr_t mxos_i2c_transfer( mxos_i2c_device_t* device, mxos_i2c_message_t* messages, uint16_t number_of_messages )
{
  merr_t err = kNoErr;
  mxos_i2c_config_t config;
  
  if ( device->port >= MXOS_I2C_NONE )
    return kUnsupportedErr;

  config.address       = device->address;
  config.address_width = device->address_width;
  config.flags         &= ~I2C_DEVICE_USE_DMA ;
  config.speed_mode    = device->speed_mode;
  
  mos_mutex_lock(platform_i2c_drivers[device->port].i2c_mutex );
  err = platform_i2c_transfer( &platform_i2c_peripherals[device->port], &config, messages, number_of_messages );
  mos_mutex_unlock(platform_i2c_drivers[device->port].i2c_mutex );

  return err;
}

void mxos_mcu_powersave_config( int enable )
{
  if (enable == 1)
    platform_mcu_powersave_enable( );
  else
    platform_mcu_powersave_disable( );
}

void mxos_sys_standby( uint32_t secondsToWakeup )
{
  platform_mcu_enter_standby( secondsToWakeup );
}


merr_t mxos_pwm_init(mxos_pwm_t pwm, uint32_t frequency, float duty_cycle)
{
  if ( pwm >= MXOS_PWM_NONE )
    return kUnsupportedErr;
  return (merr_t) platform_pwm_init( &platform_pwm_peripherals[pwm], frequency, duty_cycle );
}

merr_t mxos_pwm_start( mxos_pwm_t pwm )
{
  if ( pwm >= MXOS_PWM_NONE )
    return kUnsupportedErr;
  return (merr_t) platform_pwm_start( &platform_pwm_peripherals[pwm] );
}

merr_t mxos_pwm_stop( mxos_pwm_t pwm )
{
  if ( pwm >= MXOS_PWM_NONE )
    return kUnsupportedErr;
  return (merr_t) platform_pwm_stop( &platform_pwm_peripherals[pwm] );
}

merr_t mxos_rtc_init(void)
{
    return platform_rtc_init();
}

merr_t mxos_rtc_get_time(time_t *t)
{
    return platform_rtc_get_time(t);
}

merr_t mxos_rtc_set_time(time_t t)
{
    return platform_rtc_set_time(t);
}

merr_t mxos_spi_init( const mxos_spi_device_t* spi )
{
  platform_spi_config_t config;
  merr_t              err = kNoErr;

  if ( spi->port >= MXOS_SPI_NONE )
    return kUnsupportedErr;

#ifdef MXOS_WIFI_SHARE_SPI_BUS
  if( platform_spi_peripherals[spi->port].port == wifi_spi.port )
  {
    return platform_wlan_spi_init( &platform_gpio_pins[spi->chip_select] );
  }
#endif

  if( platform_spi_drivers[spi->port].spi_mutex == NULL)
    platform_spi_drivers[spi->port].spi_mutex = mos_mutex_new( );
  
  config.chip_select = spi->chip_select == MXOS_GPIO_NONE ? NULL : &platform_gpio_pins[spi->chip_select];
  config.speed       = spi->speed;
  config.mode        = spi->mode;
  config.bits        = spi->bits;

  mos_mutex_lock(platform_spi_drivers[spi->port].spi_mutex );
  err = platform_spi_init( &platform_spi_drivers[spi->port], &platform_spi_peripherals[spi->port], &config );
  mos_mutex_unlock(platform_spi_drivers[spi->port].spi_mutex );

  return err;
}

merr_t mxos_spi_deinit( const mxos_spi_device_t* spi )
{
  merr_t err = kNoErr;

  if ( spi->port >= MXOS_SPI_NONE )
    return kUnsupportedErr;

#ifdef MXOS_WIFI_SHARE_SPI_BUS
  if( platform_spi_peripherals[spi->port].port == wifi_spi.port )
  {
      return kUnsupportedErr;
    //return platform_wlan_spi_deinit( &platform_gpio_pins[spi->chip_select] );
  }
#endif

  if( platform_spi_drivers[spi->port].spi_mutex == NULL)
    platform_spi_drivers[spi->port].spi_mutex = mos_mutex_new( );
  
  mos_mutex_lock(platform_spi_drivers[spi->port].spi_mutex );
  err = platform_spi_deinit( &platform_spi_drivers[spi->port] );
  mos_mutex_unlock(platform_spi_drivers[spi->port].spi_mutex );

  return err;
}

merr_t mxos_spi_transfer( const mxos_spi_device_t* spi, const mxos_spi_message_segment_t* segments, uint16_t number_of_segments )
{
  platform_spi_config_t config;
  merr_t err = kNoErr;

  if ( spi->port >= MXOS_SPI_NONE )
    return kUnsupportedErr;

#ifdef MXOS_WIFI_SHARE_SPI_BUS
  if( platform_spi_peripherals[spi->port].port == wifi_spi.port )
  {
    return platform_wlan_spi_transfer( &platform_gpio_pins[spi->chip_select], segments, number_of_segments );
  }
#endif

  if( platform_spi_drivers[spi->port].spi_mutex == NULL)
    platform_spi_drivers[spi->port].spi_mutex = mos_mutex_new( );

  config.chip_select = spi->chip_select == MXOS_GPIO_NONE ? NULL : &platform_gpio_pins[spi->chip_select];
  config.speed       = spi->speed;
  config.mode        = spi->mode;
  config.bits        = spi->bits;

  mos_mutex_lock(platform_spi_drivers[spi->port].spi_mutex );
  err = platform_spi_init( &platform_spi_drivers[spi->port], &platform_spi_peripherals[spi->port], &config );
  err = platform_spi_transfer( &platform_spi_drivers[spi->port], &config, segments, number_of_segments );
  mos_mutex_unlock(platform_spi_drivers[spi->port].spi_mutex );

  return err;
}

#if 0
merr_t MxosSpiSlaveInitialize( mxos_spi_t spi, const mxos_spi_slave_config_t* config )
{
  if ( spi >= MXOS_SPI_NONE )
    return kUnsupportedErr;

  return (merr_t) platform_spi_slave_init( &platform_spi_slave_drivers[spi], &platform_spi_peripherals[spi], config );
}

merr_t MxosSpiSlaveFinalize( mxos_spi_t spi )
{
  if ( spi >= MXOS_SPI_NONE )
    return kUnsupportedErr;

  return (merr_t) platform_spi_slave_deinit( &platform_spi_slave_drivers[spi] );
}

merr_t  MxosSpiSlaveSendErrorStatus( mxos_spi_t spi, mxos_spi_slave_transfer_status_t error_status )
{
  if ( spi >= MXOS_SPI_NONE )
    return kUnsupportedErr;

  return (merr_t) platform_spi_slave_send_error_status( &platform_spi_slave_drivers[spi], error_status );
}

merr_t MxosSpiSlaveReceiveCommand( mxos_spi_t spi, mxos_spi_slave_command_t* command, uint32_t timeout_ms )
{
  if ( spi >= MXOS_SPI_NONE )
    return kUnsupportedErr;  

  return (merr_t) platform_spi_slave_receive_command( &platform_spi_slave_drivers[spi], command, timeout_ms );
}

merr_t MxosSpiSlaveTransferData( mxos_spi_t spi, mxos_spi_slave_transfer_direction_t direction, mxos_spi_slave_data_buffer_t* buffer, uint32_t timeout_ms )
{
  if ( spi >= MXOS_SPI_NONE )
    return kUnsupportedErr;  

  return (merr_t) platform_spi_slave_transfer_data( &platform_spi_slave_drivers[spi], direction, buffer, timeout_ms );
}

merr_t MxosSpiSlaveGenerateInterrupt( mxos_spi_t spi, uint32_t pulse_duration_ms )
{
  if ( spi >= MXOS_SPI_NONE )
    return kUnsupportedErr;

  return (merr_t) platform_spi_slave_generate_interrupt( &platform_spi_slave_drivers[spi], pulse_duration_ms );
}
#endif
merr_t mhal_uart_open( mxos_uart_t uart, const mxos_uart_config_t* config, ring_buffer_t* optional_rx_buffer )
{
  if ( uart >= MXOS_UART_NONE )
    return kUnsupportedErr;

#ifndef MXOS_DISABLE_STDIO
  /* Interface is used by STDIO. Uncomment MXOS_DISABLE_STDIO to overcome this */
  if ( uart == MXOS_STDIO_UART )
  {
    return kGeneralErr;
  }
#endif
  
  return (merr_t) platform_uart_init( &platform_uart_drivers[uart], &platform_uart_peripherals[uart], config, optional_rx_buffer );
}

merr_t mxos_stdio_uart_init( const mxos_uart_config_t* config, ring_buffer_t* optional_rx_buffer )
{

    return (merr_t) platform_uart_init( &platform_uart_drivers[MXOS_STDIO_UART],
                                          &platform_uart_peripherals[MXOS_STDIO_UART],
                                          config, optional_rx_buffer );
}

merr_t mhal_uart_close( mxos_uart_t uart )
{
  if ( uart >= MXOS_UART_NONE )
    return kUnsupportedErr;

  return (merr_t) platform_uart_deinit( &platform_uart_drivers[uart] );
}

merr_t mhal_uart_write( mxos_uart_t uart, const void* data, uint32_t size )
{
  if ( uart >= MXOS_UART_NONE )
    return kUnsupportedErr;

  return (merr_t) platform_uart_transmit_bytes( &platform_uart_drivers[uart], (const uint8_t*) data, size );
}

merr_t mhal_uart_read( mxos_uart_t uart, void* data, uint32_t size, uint32_t timeout )
{
  if ( uart >= MXOS_UART_NONE )
    return kUnsupportedErr;

  return (merr_t) platform_uart_receive_bytes( &platform_uart_drivers[uart], (uint8_t*)data, size, timeout );
}

uint32_t mhal_uart_readd_data_len( mxos_uart_t uart )
{
  if ( uart >= MXOS_UART_NONE )
    return 0;
  
  return (merr_t) platform_uart_get_length_in_buffer( &platform_uart_drivers[uart] );
}

merr_t MxosRandomNumberRead( void *inBuffer, int inByteCount )
{
  return (merr_t) platform_random_number_read( inBuffer, inByteCount );
}

void mxos_sys_reboot( void )
{
  platform_mcu_reset();
}

merr_t mxos_wdg_init( uint32_t timeout )
{
    return (merr_t) platform_watchdog_init( timeout );
}

void mxos_wdg_reload( void )
{
    platform_watchdog_kick( );
}

mxos_logic_partition_t* mhal_flash_get_info( mxos_partition_t inPartition )
{
    mxos_logic_partition_t *logic_partition = NULL;
    require( inPartition >= 0 && inPartition < MXOS_PARTITION_MAX, exit );

#ifdef MXOS_ENABLE_SECONDARY_APPLICATION
extern platform_logic_partition_t* paltform_flash_get_info(int inPartition);
    logic_partition = paltform_flash_get_info( inPartition );
#else
    logic_partition = (mxos_logic_partition_t *)&mxos_partitions[ inPartition ];
#endif

exit:
    return logic_partition;
}


static merr_t MxosFlashInitialize( mxos_partition_t partition )
{
  merr_t err = kNoErr;
  mxos_logic_partition_t *partition_info;
  
  require_action_quiet( partition > MXOS_PARTITION_ERROR, exit, err = kParamErr );
  require_action_quiet( partition < MXOS_PARTITION_MAX, exit, err = kParamErr );

  partition_info = mhal_flash_get_info( partition );
  require_action_quiet( partition_info->partition_owner != MXOS_FLASH_NONE, exit, err = kNotFoundErr );
  
  if( platform_flash_drivers[ partition_info->partition_owner ].flash_mutex == NULL){
    platform_flash_drivers[ partition_info->partition_owner ].flash_mutex = mos_mutex_new( );
    require_action( platform_flash_drivers[ partition_info->partition_owner ].flash_mutex != NULL, exit, err = kGeneralErr );
  }
  
  mos_mutex_lock(platform_flash_drivers[ partition_info->partition_owner ].flash_mutex );
  
  err = platform_flash_init( &platform_flash_peripherals[ partition_info->partition_owner ] );
  platform_flash_drivers[ partition_info->partition_owner ].peripheral = (mxos_flash_t *)&platform_flash_peripherals[ partition_info->partition_owner ];
  platform_flash_drivers[ partition_info->partition_owner ].initialized = true;
  mos_mutex_unlock(platform_flash_drivers[ partition_info->partition_owner ].flash_mutex );
  
exit: 
  return err;
}

merr_t mhal_flash_erase(mxos_partition_t partition, uint32_t off_set, uint32_t size)
{
  merr_t err = kNoErr;
  uint32_t start_addr, end_addr;
  mxos_logic_partition_t *partition_info;

  require_quiet( size != 0, exit);
  require_action_quiet( partition > MXOS_PARTITION_ERROR && partition < MXOS_PARTITION_MAX, exit, err = kParamErr );
  require_action_quiet( partition < MXOS_PARTITION_MAX, exit, err = kParamErr );

  partition_info = mhal_flash_get_info( partition );
  require_action_quiet( partition_info->partition_owner != MXOS_FLASH_NONE, exit, err = kNotFoundErr );
#if (!defined BOOTLOADER) && (!defined FIRMWARE_DOWNLOAD)
  require_action_quiet( ( partition_info->partition_options & PAR_OPT_WRITE_MASK ) == PAR_OPT_WRITE_EN, exit, err = kPermissionErr );
#endif

  start_addr = partition_info->partition_start_addr + off_set;
  end_addr = partition_info->partition_start_addr + off_set + size - 1;

  require_action_quiet( end_addr < partition_info->partition_start_addr + partition_info->partition_length, exit, err = kParamErr );

  if( platform_flash_drivers[ partition_info->partition_owner ].initialized == false )
  {
    err =  MxosFlashInitialize( partition );
    require_noerr_quiet( err, exit );
  }

  mos_mutex_lock(platform_flash_drivers[ partition_info->partition_owner ].flash_mutex );
  err = platform_flash_erase( &platform_flash_peripherals[ partition_info->partition_owner ], start_addr, end_addr );
  mos_mutex_unlock(platform_flash_drivers[ partition_info->partition_owner ].flash_mutex );

exit:
  return err;
}

merr_t mhal_flash_write( mxos_partition_t partition, volatile uint32_t* off_set, uint8_t* inBuffer ,uint32_t inBufferLength)
{
  merr_t err = kNoErr;
  uint32_t start_addr, end_addr;
  mxos_logic_partition_t *partition_info;

  require_quiet( inBufferLength != 0, exit);
  require_action_quiet( partition > MXOS_PARTITION_ERROR && partition < MXOS_PARTITION_MAX, exit, err = kParamErr );
  
  partition_info = mhal_flash_get_info( partition );
  require_action_quiet( partition_info->partition_owner != MXOS_FLASH_NONE, exit, err = kNotFoundErr );
#if (!defined BOOTLOADER) && (!defined FIRMWARE_DOWNLOAD)
  require_action_quiet( ( partition_info->partition_options & PAR_OPT_WRITE_MASK ) == PAR_OPT_WRITE_EN, exit, err = kPermissionErr );
#endif

  start_addr = partition_info->partition_start_addr + *off_set;
  end_addr = partition_info->partition_start_addr + *off_set + inBufferLength - 1;

  require_action_quiet( end_addr < partition_info->partition_start_addr + partition_info->partition_length , exit, err = kParamErr );
  
  if( platform_flash_drivers[ partition_info->partition_owner ].initialized == false )
  {
    err =  MxosFlashInitialize( partition );
    require_noerr_quiet( err, exit );
  }

  mos_mutex_lock(platform_flash_drivers[ partition_info->partition_owner ].flash_mutex );
  err = platform_flash_write( &platform_flash_peripherals[ partition_info->partition_owner ], &start_addr, inBuffer, inBufferLength );
  *off_set = start_addr - partition_info->partition_start_addr;
  mos_mutex_unlock(platform_flash_drivers[ partition_info->partition_owner ].flash_mutex );
  
exit:
  return err;
}

merr_t mhal_flash_read( mxos_partition_t partition, volatile uint32_t* off_set, uint8_t* outBuffer ,uint32_t inBufferLength)
{
  merr_t err = kNoErr;
  uint32_t start_addr, end_addr;
  mxos_logic_partition_t *partition_info;

  require_quiet( inBufferLength != 0, exit);
  require_action_quiet( partition > MXOS_PARTITION_ERROR && partition < MXOS_PARTITION_MAX, exit, err = kParamErr );

  partition_info = mhal_flash_get_info( partition );
  require_action_quiet( partition_info->partition_owner != MXOS_FLASH_NONE, exit, err = kNotFoundErr );
#if (!defined BOOTLOADER) && (!defined FIRMWARE_DOWNLOAD)
  require_action_quiet( ( partition_info->partition_options & PAR_OPT_READ_MASK ) == PAR_OPT_READ_EN, exit, err = kPermissionErr );
#endif

  start_addr = partition_info->partition_start_addr + *off_set;
  end_addr = partition_info->partition_start_addr + *off_set + inBufferLength - 1;
  require_action_quiet( end_addr < partition_info->partition_start_addr + partition_info->partition_length , exit, err = kParamErr );

  if( platform_flash_drivers[ partition_info->partition_owner ].initialized == false )
  {
    err =  MxosFlashInitialize( partition );
    require_noerr_quiet( err, exit );
  }

  mos_mutex_lock(platform_flash_drivers[ partition_info->partition_owner ].flash_mutex );
  err = platform_flash_read( &platform_flash_peripherals[ partition_info->partition_owner ], &start_addr, outBuffer, inBufferLength );
  *off_set = start_addr - partition_info->partition_start_addr;
  mos_mutex_unlock(platform_flash_drivers[ partition_info->partition_owner ].flash_mutex );

exit:
  return err;
}

merr_t mxos_flash_enable_security( mxos_partition_t partition, uint32_t off_set, uint32_t size )
{
  merr_t err = kNoErr;
  uint32_t start_addr, end_addr;
  mxos_logic_partition_t *partition_info;

  require_quiet( size != 0, exit);
  require_action_quiet( partition > MXOS_PARTITION_ERROR && partition < MXOS_PARTITION_MAX, exit, err = kParamErr );

  partition_info = mhal_flash_get_info( partition );
  require_action_quiet( partition_info->partition_owner != MXOS_FLASH_NONE, exit, err = kNotFoundErr );

  start_addr = partition_info->partition_start_addr + off_set;
  end_addr = partition_info->partition_start_addr + off_set + size - 1;
  require_action_quiet( end_addr < partition_info->partition_start_addr + partition_info->partition_length, exit, err = kParamErr );

  if( platform_flash_drivers[ partition_info->partition_owner ].initialized == false )
  {
    err =  MxosFlashInitialize( partition );
    require_noerr_quiet( err, exit );
  }

  mos_mutex_lock(platform_flash_drivers[ partition_info->partition_owner ].flash_mutex );
  err = platform_flash_enable_protect( &platform_flash_peripherals[ partition_info->partition_owner ], start_addr, end_addr);
  mos_mutex_unlock(platform_flash_drivers[ partition_info->partition_owner ].flash_mutex );
  
exit:
  return err;
}

char *mxos_get_bootloader_ver(void)
{
  #ifdef CONFIG_MX108
  static char boot_ver[64];
  uint16_t reg_status;
  uint8_t blocks;

  strncpy(boot_ver, (const char *)0x00000020, sizeof(boot_ver) - 3);

  flash_ctrl(0xe240000 + 6, &reg_status);
  blocks = (reg_status >> 2) & 0x0F;
  if(blocks != 1)
  {
    strcat(boot_ver, "-x");
  }
  return boot_ver;
  #else
  static char ver[33];
  const mxos_logic_partition_t* bootloader_partition = &mxos_partitions[ MXOS_PARTITION_BOOTLOADER ];
  uint32_t version_offset = bootloader_partition->partition_length - 0x20;

  memset(ver, 0, sizeof(ver));
  mhal_flash_read( MXOS_PARTITION_BOOTLOADER, &version_offset, (uint8_t *)ver , 32);
  return ver;
  #endif
}

#ifdef BOOTLOADER 
#include "bootloader.h"

void mxos_set_bootload_ver(void)
{
   uint8_t ver[33];
   mxos_logic_partition_t *boot_partition = mhal_flash_get_info( MXOS_PARTITION_BOOTLOADER );
   uint32_t flashaddr =  boot_partition->partition_length - 0x20;
   int i;

   memset(ver, 0, sizeof(ver));
   mhal_flash_read( MXOS_PARTITION_BOOTLOADER, &flashaddr, (uint8_t *)ver , 32);
   for(i=0;i<32;i++) {
       if (ver[i] != 0xFF)
           return;
   }
   snprintf((char *)ver, 33, "%s %s %d", MODEL, Bootloader_REVISION , MXOS_STDIO_UART_BAUDRATE);
   flashaddr =  boot_partition->partition_length - 0x20;
   mxos_flash_disable_security( MXOS_PARTITION_BOOTLOADER, 0x0, boot_partition->partition_length );
   mhal_flash_write( MXOS_PARTITION_BOOTLOADER, &flashaddr, ver , 32);
}

merr_t mxos_flash_disable_security( mxos_partition_t partition, uint32_t off_set, uint32_t size )
{
  merr_t err = kNoErr;
  uint32_t start_addr, end_addr;
  mxos_logic_partition_t *partition_info;

  require_quiet( size != 0, exit);
  require_action_quiet( partition > MXOS_PARTITION_ERROR && partition < MXOS_PARTITION_MAX, exit, err = kParamErr );

  partition_info = mhal_flash_get_info( partition );
  require_action_quiet( partition_info->partition_owner != MXOS_FLASH_NONE, exit, err = kNotFoundErr );

  start_addr = partition_info->partition_start_addr + off_set;
  end_addr = partition_info->partition_start_addr + off_set + size - 1;
  require_action_quiet( end_addr < partition_info->partition_start_addr + partition_info->partition_length, exit, err = kParamErr );

  if( platform_flash_drivers[ partition_info->partition_owner ].initialized == false )
  {
    err =  MxosFlashInitialize( partition );
    require_noerr_quiet( err, exit );
  }

  mos_mutex_lock(platform_flash_drivers[ partition_info->partition_owner ].flash_mutex );
  err = platform_flash_disable_protect( &platform_flash_peripherals[ partition_info->partition_owner ], start_addr, end_addr);
  mos_mutex_unlock(platform_flash_drivers[ partition_info->partition_owner ].flash_mutex );
  
exit:
  return err;
}

#endif

uint64_t mxos_nanosecond_clock_value( void )
{
    return platform_get_nanosecond_clock_value( );
}

void mxos_nanosecond_deinit( void )
{
    platform_deinit_nanosecond_clock( );
}

void mxos_nanosecond_reset( void )
{
    platform_reset_nanosecond_clock( );
}

void mxos_nanosecond_init( void )
{
    platform_init_nanosecond_clock( );
}

void mxos_nanosecond_delay( uint64_t delayns )
{
  platform_nanosecond_delay( delayns );
}


