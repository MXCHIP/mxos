/**
 ******************************************************************************
 * @file    platform_core.c
 * @author  William Xu
 * @version V1.0.0
 * @date    09-Aug-2016
 * @brief   This file provide functions called by RTOS
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#include "platform_cmsis.h"
#include "platform_core.h"

uint32_t platform_get_cycle_count( void )
{
    /* m0plus not support this */
    return 0;
}

mxos_bool_t platform_is_in_interrupt_context( void )
{
    /* From the ARM Cortex-M3 Techinical Reference Manual
     * 0xE000ED04   ICSR    RW [a]  Privileged  0x00000000  Interrupt Control and State Register */
    return ( ( SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk ) != 0 ) ? MXOS_TRUE : MXOS_FALSE;
}
