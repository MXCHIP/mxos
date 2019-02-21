/**
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 *
 */

/*******************************************************************************
 **  Name:       userial_bby.c
 **
 **  Description:
 **
 **  This file contains the universal driver wrapper for the BTE-QC serial
 **  drivers
 *******************************************************************************/


#include "mxos.h"
#include "mxos_bt_types.h"

#include "platform_bluetooth.h"
#include "platform_config.h"


/* Macro for checking of bus is initialised */
#define IS_BUS_INITIALISED( ) \
do \
{ \
    if ( bus_initialised == false ) \
    { \
        printf( "bus uninitialised" ); \
        return -1; \
    } \
}while ( 0 )

/* Macro for checking if bus is ready */
#define BT_BUS_IS_READY( ) \
do \
{ \
    if ( bt_bus_is_ready( ) == false ) \
    { \
        printf( "bus not ready" ) \
        return kGeneralErr; \
    } \
}while ( 0 )

/* Macro for waiting until bus is ready */
#define BT_BUS_WAIT_UNTIL_READY( ) \
do \
{ \
    while ( bt_bus_is_ready( ) == false ) \
    { \
        mxos_thread_msleep( 10 ); \
    } \
} while ( 0 )

/* TODO: bring in bt_bus code to remove BTE dependency on MXOS bluetooth library */
extern int bt_bus_init( void );
extern int bt_bus_deinit( void );
extern int bt_bus_transmit( const uint8_t* data_out, uint32_t size );
extern int bt_bus_receive( uint8_t* data_in, uint32_t size, uint32_t timeout_ms );
extern int bt_bus_uart_reset( void );
extern int bt_bus_uart_reconifig_baud(uint32_t baud);
extern bool bt_bus_is_ready( void );
extern OSStatus platform_uart_reconfig( platform_uart_driver_t* driver, const platform_uart_config_t* config );


/******************************************************
 *                   Enumerations
 ******************************************************/


/*****************************************************************************
 * Platform UART interface, taken from
 * ../Library/bluetooth/internal/bus/UART/bt_bus.c
 * (audio/2.4.x-bluetooth branch)
 *****************************************************************************/

#ifndef USERIAL_RX_FIFO_SIZE
#define USERIAL_RX_FIFO_SIZE (3000)
#endif

static volatile bool bus_initialised = false;
static volatile bool device_powered  = false;

/* RX ring buffer. Bluetooth chip UART receive can be asynchronous, therefore a ring buffer is required */
static ring_buffer_t rx_ring_buffer;
static uint8_t             rx_data[USERIAL_RX_FIFO_SIZE];

int bt_bus_init( void )
{
    //USART_OverSampling8Cmd(USART1, ENABLE);

    if ( bus_initialised == false )
    {
        if( mxos_bt_control_pins[MXOS_BT_PIN_HOST_WAKE] != NULL)
            require_noerr( platform_gpio_init( mxos_bt_control_pins[MXOS_BT_PIN_HOST_WAKE], INPUT_HIGH_IMPEDANCE ), exit );

        if( mxos_bt_control_pins[MXOS_BT_PIN_DEVICE_WAKE] != NULL)
        {
            require_noerr( platform_gpio_init( mxos_bt_control_pins[MXOS_BT_PIN_DEVICE_WAKE], OUTPUT_OPEN_DRAIN_PULL_UP ), exit );
            require_noerr( platform_gpio_output_high( mxos_bt_control_pins[MXOS_BT_PIN_DEVICE_WAKE] ), exit );
            mxos_thread_msleep( 100 );            
        }

        /* Configure Reg Enable pin to output. Set to HIGH */
        if( mxos_bt_control_pins[MXOS_BT_PIN_POWER] != NULL)
        {
            require_noerr( platform_gpio_init( mxos_bt_control_pins[MXOS_BT_PIN_POWER], OUTPUT_OPEN_DRAIN_PULL_UP ), exit );
            require_noerr( platform_gpio_output_high( mxos_bt_control_pins[MXOS_BT_PIN_POWER] ), exit );            
        }
        device_powered = true;

        if ( mxos_bt_uart_config.flow_control == FLOW_CONTROL_DISABLED )
        {
            /* Configure RTS pin to output. Set to HIGH */
            require_noerr( platform_gpio_init( mxos_bt_uart_pins[MXOS_BT_PIN_UART_RTS], OUTPUT_OPEN_DRAIN_PULL_UP ), exit );
            require_noerr( platform_gpio_output_high( mxos_bt_uart_pins[MXOS_BT_PIN_UART_RTS] ), exit ); //William, working wrong if set high, so donot use FLOW_CONTROL_DISABLED in 43438A1

            /* Configure CTS pin to input pull-up */
            require_noerr( platform_gpio_init( mxos_bt_uart_pins[MXOS_BT_PIN_UART_CTS], INPUT_PULL_UP ), exit );
        }

        /* Configure Reset pin to output. Set to HIGH */
        if( mxos_bt_control_pins[MXOS_BT_PIN_RESET] != NULL)
        {
            require_noerr( platform_gpio_init( mxos_bt_control_pins[MXOS_BT_PIN_RESET], OUTPUT_OPEN_DRAIN_PULL_UP ), exit );
            require_noerr( platform_gpio_output_high( mxos_bt_control_pins[MXOS_BT_PIN_RESET] ), exit );
        }

        /* Initialise RX ring buffer */
        ring_buffer_init( &rx_ring_buffer, (uint8_t*) rx_data, sizeof( rx_data ) );

        /* Configure USART comms */
        require_noerr( platform_uart_init( mxos_bt_uart_driver, mxos_bt_uart_peripheral, &mxos_bt_uart_config, &rx_ring_buffer ), exit );

#ifdef  MXOS_USE_BT_RESET_PIN
        /* Reset bluetooth chip. Delay momentarily. */
        require_noerr( platform_gpio_output_low( &bt_control_pins[BT_PIN_RESET] ), exit );
        mxos_thread_msleep( 10 );
        require_noerr( platform_gpio_output_high( &bt_control_pins[BT_PIN_RESET] ), exit );
#endif

        /* Wait until the Bluetooth chip stabilizes.  */
        mxos_thread_msleep( 100 );

        /* Bluetooth chip is ready. Pull host's RTS low */
        if ( mxos_bt_uart_config.flow_control == FLOW_CONTROL_DISABLED )
        {
            /* Bluetooth chip is ready. Pull host's RTS low */
            require_noerr( platform_gpio_output_low( mxos_bt_uart_pins[MXOS_BT_PIN_UART_RTS] ), exit );
        }

        bus_initialised = true;

        /* Wait for bluetooth chip to pull its RTS (host's CTS) low. From observation using CRO, it takes the bluetooth chip > 170ms to pull its RTS low after CTS low */
        BT_BUS_WAIT_UNTIL_READY();
    }

exit:
    return kNoErr;
}

int bt_bus_deinit( void )
{
    require( bus_initialised, exit);

    if( mxos_bt_control_pins[MXOS_BT_PIN_RESET] != NULL)
        require_noerr( platform_gpio_output_low( mxos_bt_control_pins[MXOS_BT_PIN_RESET] ), exit );

    require_noerr( platform_gpio_output_high( mxos_bt_uart_pins[MXOS_BT_PIN_UART_RTS] ), exit ); // RTS deasserted

    if( mxos_bt_control_pins[MXOS_BT_PIN_POWER] != NULL)
        require_noerr( platform_gpio_output_low ( mxos_bt_control_pins[MXOS_BT_PIN_POWER] ), exit ); // Bluetooth chip regulator off
    
    device_powered = false;

    /* Deinitialise UART */
    require_noerr( platform_uart_deinit( mxos_bt_uart_driver ), exit );
    bus_initialised = false;

    return kNoErr;
exit:
    return kGeneralErr;
}

int bt_bus_transmit( const uint8_t* data_out, uint32_t size )
{
    IS_BUS_INITIALISED();

    BT_BUS_WAIT_UNTIL_READY();

    require_noerr( platform_uart_transmit_bytes( mxos_bt_uart_driver, data_out, size ), exit );

exit:
    return kNoErr;
}

int bt_bus_receive( uint8_t* data_in, uint32_t size, uint32_t timeout_ms )
{
    IS_BUS_INITIALISED();

    return platform_uart_receive_bytes( mxos_bt_uart_driver, (void*)data_in, size, timeout_ms );
}

bool bt_bus_is_ready( void )
{
    return ( bus_initialised == false ) ? false : ( ( platform_gpio_input_get( mxos_bt_uart_pins[MXOS_BT_PIN_UART_CTS] ) == true ) ? false : true );
}

int bt_bus_uart_reconifig_baud(uint32_t newBaudRate)
{
    platform_uart_config_t bt_uart_config;
    uint32_t last_rx_size = mxos_bt_uart_driver->rx_size; 
    memcpy( &bt_uart_config, &mxos_bt_uart_config, sizeof( mxos_bt_uart_config ) );
       
    if ( bus_initialised == true )
    {
        bt_uart_config.baud_rate = newBaudRate;
        
        /* Initialise RX ring buffer */
        ring_buffer_init( &rx_ring_buffer, (uint8_t*) rx_data, sizeof( rx_data ) );

        /* Configure USART comms */
        platform_uart_init( mxos_bt_uart_driver, mxos_bt_uart_peripheral, &bt_uart_config, &rx_ring_buffer );
        
        /* UART receive function may on the pending, but init will clear the rx size, recover here */
        mxos_bt_uart_driver->rx_size = last_rx_size;
        
        return kNoErr;
    }
    else
    {
        return kGeneralErr;
    }
}






