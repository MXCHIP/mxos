/**
 ******************************************************************************
 * @file    mxos_platform.h
 * @author  William Xu
 * @version V1.0.0
 * @date    16-Sep-2014
 * @brief   This file provides all the headers of MXOS peripheral operations.
 ******************************************************************************
 *
 *  The MIT License
 *  Copyright (c) 2014 MXCHIP Inc.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is furnished
 *  to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 *  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ******************************************************************************
 */


#ifndef __MXOS_PLATFORM_H__
#define __MXOS_PLATFORM_H__

#include "mxos_opt.h"
#include "mxos_common.h"

#ifdef __cplusplus
extern "C" {
#endif


#include "mxos_hal/mxos_gpio.h"
#include "mxos_hal/mxos_wdg.h"
#include "mxos_hal/mxos_uart.h"
#include "mxos_hal/mxos_adc.h"

#include "mxos_hal/mxos_i2c.h"
#include "mxos_hal/mxos_spi.h"
#include "mxos_hal/mxos_pwm.h"
#include "mxos_hal/mxos_rtc.h"
#include "mxos_hal/mxos_rng.h"
#include "mxos_hal/mxos_flash.h"
#include "mxos_hal/mxos_mfi_auth.h"
#include "mxos_hal/mxos_i2s.h"
#ifdef CONFIG_CPU_MX1290
#include "mxos_hal/mxos_gtimer.h"
#endif

#define MxosMcuPowerSaveConfig mxos_mcu_powersave_config

#define MxosSystemReboot mxos_system_reboot


/** @defgroup MXOS_PLATFORM  MXOS Hardware Abstract Layer APIs
*   @brief Control hardware peripherals on different platfroms using standard HAL API functions
* 
*/
/** @addtogroup MXOS_PLATFORM
  * @{
  */

/** @defgroup platform_misc Task switching, reboot, and standby
  * @brief Provide task switching,reboot and standby functions
  * @{
  */

#define ENABLE_INTERRUPTS   __asm("CPSIE i")  /**< Enable interrupts to start task switching in MXOS RTOS. */
#define DISABLE_INTERRUPTS  __asm("CPSID i")  /**< Disable interrupts to stop task switching in MXOS RTOS. */


/** @brief    Software reboot the MXOS hardware
  *
  * @param    none
  * @return   none
  */
void mxos_system_reboot(void);


/** @brief    Software reboot the MXOS hardware
  *
  * @param    timeToWakeup: MXOS will wakeup after secondsToWakeup (seconds)
  * @return   none
  */
void MxosSystemStandBy(uint32_t secondsToWakeup);


/** @brief    Enables the MCU to enter deep sleep mode when all threads are suspended.
  *
  * @note:    When all threads are suspended, mcu can shut down some peripherals to 
  *           save power. For example, STM32 enters STOP mode in this condition, 
  *           its peripherals are not working and needs to be wake up by an external
  *           interrupt or MXOS core's internal timer. So if you are using a peripherals,  
  *           you should disable this function temporarily.
  *           To make this function works, you should not disable the macro in MxosDefault.h: 
  *           MXOS_DISABLE_MCU_POWERSAVE
  *
  * @param    enable : 1 = enable MCU powersave, 0 = disable MCU powersave
  * @return   none
  */
void mxos_mcu_powersave_config( int enable );


/** @brief    Set MXOS system led on/off state
  *
  * @param    onff: State of syetem LED,0: OFF,1:ON
  * @return   none
  */
void MxosSysLed(bool onoff);

/** @brief     Set MXOS RF led on/off state
  *
  * @param    onff: State of RF LED,0: OFF,1:ON
  * @return   none
  */
void MxosRfLed(bool onoff);

/** @brief    add
  *
  * @param    add
  * @return   none
  */
bool MxosShouldEnterMFGMode(void);


/** @brief    add
  *
  * @param    add
  * @return   none
  */
bool MxosShouldEnterATEMode(void);


/** @brief    add
  *
  * @param    add
  * @return   none
  */
bool MxosShouldEnterBootloader(void);


/** @brief    add
  *
  * @param    add
  * @return   none
  */
char *mxos_get_bootloader_ver(void);


void mxos_board_init( void );


#ifdef BOOTLOADER 
void mxos_set_bootload_ver(void);
#endif
/**
  * @}
  */


/**
  * @}
  */
#ifdef __cplusplus
} /*extern "C" */
#endif


#endif /*__MXOS_PLATFORM_H__*/

