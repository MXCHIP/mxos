/**
 ******************************************************************************
 * @file    mxos_system_monitor.c
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   System monitor functions, create a system monitor thread, and developer
 *          can add own monitor items.
 ******************************************************************************
 *
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#include "mxos.h"
#include "system_internal.h"

#define DEFAULT_SYSTEM_MONITOR_PERIOD   (2000)

#ifndef MAXIMUM_NUMBER_OF_SYSTEM_MONITORS
#define MAXIMUM_NUMBER_OF_SYSTEM_MONITORS    (5)
#endif

#if defined (CONFIG_CPU_MX1290) || defined (CONFIG_MX108)
#define APPLICATION_WATCHDOG_TIMEOUT_SECONDS  25 /**< Monitor point defined by mxos system
                                                     25 seconds to reload. */
#else
#define APPLICATION_WATCHDOG_TIMEOUT_SECONDS  5 /**< Monitor point defined by mxos system
                                                     5 seconds to reload. */
#endif

static mxos_system_monitor_t* system_monitors[MAXIMUM_NUMBER_OF_SYSTEM_MONITORS];
void mxos_system_monitor_thread_main( void *arg );

merr_t MXOSStartSystemMonitor ( void )
{
  merr_t err = kNoErr;
  require_noerr(mxos_wdg_init( DEFAULT_SYSTEM_MONITOR_PERIOD + 1000 ), exit);
  memset(system_monitors, 0, sizeof(system_monitors));

  require_action(mos_thread_new( 0, "SYS MONITOR", mxos_system_monitor_thread_main, STACK_SIZE_mxos_system_MONITOR_THREAD, NULL ) != NULL, exit, err = kGeneralErr);
exit:
  return err;
}

void mxos_system_monitor_thread_main( void *arg )
{
  (void)arg;
  
  while (1)
  {
    int a;
    uint32_t current_time = mos_time();
    
    for (a = 0; a < MAXIMUM_NUMBER_OF_SYSTEM_MONITORS; ++a)
    {
      if (system_monitors[a] != NULL)
      {
        if ((current_time - system_monitors[a]->last_update) > system_monitors[a]->longest_permitted_delay)
        {
          /* A system monitor update period has been missed */
          mxos_sys_reboot();
          while(1);
        }
      }
    }
    
    mxos_wdg_reload();
    mxos_thread_msleep(DEFAULT_SYSTEM_MONITOR_PERIOD);
  }
}

merr_t mxos_system_monitor_register(mxos_system_monitor_t* system_monitor, uint32_t initial_permitted_delay)
{
  int a;
  
  /* Find spare entry and add the new system monitor */
  for ( a = 0; a < MAXIMUM_NUMBER_OF_SYSTEM_MONITORS; ++a )
  {
    if (system_monitors[a] == NULL)
    {
      system_monitor->last_update = mos_time();
      system_monitor->longest_permitted_delay = initial_permitted_delay;
      system_monitors[a] = system_monitor;
      return kNoErr;
    }
  }
  
  return kUnknownErr;
}

merr_t mxos_system_monitor_update(mxos_system_monitor_t* system_monitor, uint32_t permitted_delay)
{
  uint32_t current_time = mos_time();
  /* Update the system monitor if it hasn't already passed it's permitted delay */
  if ((current_time - system_monitor->last_update) <= system_monitor->longest_permitted_delay)
  {
    system_monitor->last_update             = current_time;
    system_monitor->longest_permitted_delay = permitted_delay;
  }
  
  return kNoErr;
}

static mxos_timer_t _watchdog_reload_timer;

static mxos_system_monitor_t mxos_monitor;

static void _watchdog_reload_timer_handler( void* arg )
{
  (void)(arg);
  mxos_system_monitor_update(&mxos_monitor, APPLICATION_WATCHDOG_TIMEOUT_SECONDS*1000);
}


merr_t mxos_system_monitor_daemen_start( void )
{
  merr_t err = kNoErr;
  /*Start system monotor thread*/
  err = MXOSStartSystemMonitor( );
  require_noerr_string( err, exit, "ERROR: Unable to start the system monitor." );

  /* Register first monitor */
  err = mxos_system_monitor_register(&mxos_monitor, APPLICATION_WATCHDOG_TIMEOUT_SECONDS*1000);
  require_noerr( err, exit );
  mos_timer_new(&_watchdog_reload_timer,APPLICATION_WATCHDOG_TIMEOUT_SECONDS*1000/2, _watchdog_reload_timer_handler, NULL);
  mos_timer_start(&_watchdog_reload_timer);
exit:
  return err;
}








