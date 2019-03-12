/**
 ******************************************************************************
 * @file    mxos_main.c
 * @author  William Xu
 * @version V1.0.0
 * @date    22-Aug-2016
 * @brief   MXOS initialize before main application
 ******************************************************************************
 *
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2017 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 *
 ******************************************************************************
 */

/** @file
 *
 */

#include <string.h>
#include <stdlib.h>
#include "mxos.h"
#include "mxos_board_conf.h"
#include "mxos_rtos_common.h"
#if MXOS_QUALITY_CONTROL_ENABLE
#include "qc_test.h"
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#ifndef STDIO_BUFFER_SIZE
#define STDIO_BUFFER_SIZE   64
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
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/


#ifndef MXOS_DISABLE_STDIO
static const mxos_uart_config_t stdio_uart_config =
{
  .baud_rate    = MXOS_STDIO_UART_BAUDRATE,
  .data_width   = DATA_WIDTH_8BIT,
  .parity       = NO_PARITY,
  .stop_bits    = STOP_BITS_1,
  .flow_control = FLOW_CONTROL_DISABLED,
  .flags        = 0,
};

static volatile ring_buffer_t stdio_rx_buffer;
static volatile uint8_t       stdio_rx_data[STDIO_BUFFER_SIZE];

mxos_mutex_t MXOS_WEAK stdio_tx_mutex = NULL;
#endif /* #ifndef MXOS_DISABLE_STDIO */


extern int mxos_debug_enabled;

/******************************************************
 *               Function Definitions
 ******************************************************/

/* mxos_main is executed after rtos is start up and before real main*/
void mxos_main( void )
{
#if MXOS_APPLICATION

#ifdef DEBUG
    mxos_debug_enabled = 1;
#else
    mxos_debug_enabled = 0;
#endif

    /* Customized board configuration. */
    mxos_board_init( );

#ifndef MXOS_DISABLE_STDIO
    if( stdio_tx_mutex == NULL )
        mxos_rtos_init_mutex( &stdio_tx_mutex );

    ring_buffer_init( (ring_buffer_t*) &stdio_rx_buffer, (uint8_t*) stdio_rx_data, STDIO_BUFFER_SIZE );
    mxos_stdio_uart_init( &stdio_uart_config, (ring_buffer_t*) &stdio_rx_buffer );
#endif

    mxos_rtos_init( );

#if MXOS_QUALITY_CONTROL_ENABLE
#ifndef RTOS_mocOS
    if ( mxos_should_enter_mfg_mode( ) ) {
        mxos_system_qc_test( );
        mos_thread_delete(NULL);
        mos_thread_yield();
    }
#endif
#endif

#endif
}

