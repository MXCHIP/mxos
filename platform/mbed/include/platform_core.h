/**
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 *
 */


#ifndef __PLATFORM_CORE_h__
#define __PLATFORM_CORE_h__

#include "mxos_common.h"

/**
 * Returns the current CPU cycle count.
 *
 * This function is used to accurately calculate sub-systick timing.
 */
extern uint32_t platform_get_cycle_count( void );

/**
 * Returns TRUE if the CPU is currently running in interrupt context
 *
 */
extern mxos_bool_t platform_is_in_interrupt_context( void );

#endif // __PLATFORM_CORE_h__

