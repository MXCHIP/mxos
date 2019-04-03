/**
 ******************************************************************************
 * @file    rtos.c
 * @author  William Xu
 * @version V1.0.0
 * @date    25-Aug-2016
 * @brief   Definitions of the MXOS RTOS abstraction layer for the special case
 *          of having no RTOS
 ******************************************************************************
 *
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 *
 ******************************************************************************
 */


#include "mbed.h"
#include "mxos.h"

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
 *               Function Declarations
 ******************************************************/
extern "C" void mxos_main(void);

/******************************************************
 *               Variable Definitions
 ******************************************************/
static volatile uint32_t no_os_tick = 0;

/******************************************************
 *               Function Definitions
 ******************************************************/

Timer noos_timer;

extern "C" void mbed_main( void )
{
    noos_timer.start();

#if MXOS_APPLICATION
    /* Initialize MXOS_STDIO_UART and board peripherals. */
    mxos_main();
#endif
}

mxos_time_t mos_time(void)
{
    return noos_timer.read_ms();
}



