/**
 ******************************************************************************
 * @file    mxos_system_power_daemon.c
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This file provide the power management daemon and provide a safety
 *          power operation to developer.
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

static bool                     needs_update          = false;
static mxos_timed_event_t _timed_sys_power_state_change;

extern system_context_t* sys_context;

extern void sendNotifySYSWillPowerOff(void);

USED void PlatformStandbyButtonClickedCallback(void)
{
  mxos_Context_t* context = NULL;
  
  context = mxos_system_context_get( );
  require( context, exit );
  
  mxos_system_power_perform( context, eState_Standby );

exit: 
  return;
}

static mret_t _sys_power_state_change_handler(void *arg)
{  
    
  switch( sys_context->mxosStatus.current_sys_state )
  {
    case eState_Normal:
      break;
    case eState_Software_Reset:
      mxos_sys_reboot( );
      break;
    case eState_Wlan_Powerdown:
      mxosWlanPowerOff( );
      break;
    case eState_Standby:
      mxosWlanPowerOff( );
      mxos_sys_standby( MXOS_WAIT_FOREVER );
      break;
    default:
      break;
  }
  return kNoErr;
}

static mret_t _sys_will_power_off_handler(void *arg)
{
  mret_t err = kNoErr;
  
  require_action( sys_context, exit, err = kNotPreparedErr );

  if(needs_update == true)
  {
    mxos_system_context_update( &sys_context->flashContentInRam );
    needs_update = false;
  }

  mxos_rtos_register_timed_event( &_timed_sys_power_state_change, MXOS_NETWORKING_WORKER_THREAD, _sys_power_state_change_handler, 500, 0 );
  sendNotifySYSWillPowerOff();

exit:
  return err;
}


mret_t mxos_system_power_perform( mxos_Context_t* const in_context, mxos_system_state_t new_state )
{
  mret_t err = kNoErr;

  require_action( sys_context, exit, err = kNotPreparedErr );

  sys_context->mxosStatus.current_sys_state = new_state;

  /* Send an envent to do some operation before power off, and wait some time to perform power change */
  mxos_rtos_send_asynchronous_event( MXOS_NETWORKING_WORKER_THREAD, _sys_will_power_off_handler, NULL );

exit:
  return err; 
}




