/**
 ******************************************************************************
 * @file    MXOS.h
 * @author  William Xu
 * @version V1.0.0
 * @date    16-Sep-2014
 * @brief   This file provides other MXOS API header file and some basic APIs.
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


/** @mainpage MXOS 

    This documentation describes the MXOS APIs.
    It consists of:
     - MXOS Core APIs   
     - MXOS Hardware Abstract Layer APIs    
     - MXOS Algorithm APIs        
     - MXOS System APIs        
     - MXOS Middleware APIs
     - MXOS Drivers interface
 */

#ifndef __MXOS_H_
#define __MXOS_H_



/* MXOS SDK APIs */
#include "mxos_opt.h"
#include "mxos_debug.h"
#include "mxos_common.h"
#include "mxos_rtos.h"
#include "mxos_wlan.h"
#include "mxos_eth.h"
#include "mxos_socket.h"
#include "mxos_security.h"
#include "mxos_platform.h"
#include "mxos_system.h"
#include "mxos_gprs.h"

#ifdef ALIOS_SUPPORT
#include <aos/aos.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif


#define MxosGetRfVer                wlan_driver_version
#define MxosGetVer                  system_lib_version
#define MxosInit                    mxchipInit

/** @defgroup MXOS_Core_APIs MXOS Core APIs
  * @brief MXOS Initialization, RTOS, TCP/IP stack, and Network Management
  */

/** @addtogroup MXOS_Core_APIs
  * @{
  */

/** \defgroup MXOS_Init_Info Initialization and Tools
  * @brief Get MXOS version or RF version, flash usage information or init MXOS TCPIP stack
  * @{
 */

 /******************************************************
 *                    Structures
 ******************************************************/



/******************************************************
 *              Function Declarations
 ******************************************************/

/**
  * @brief  Get RF driver's version.
  *
  * @note   Create a memery buffer to store the version characters.
  *         THe input buffer length should be 40 bytes at least.
  * @note   This must be executed after mxosInit().
  * @param  inVersion: Buffer address to store the RF driver. 
  * @param  inLength: Buffer size. 
  *
  * @return int
  */
int MxosGetRfVer( char* outVersion, uint8_t inLength );

/**
  * @brief  Get MXOS's version.
  *
  * @param  None 
  *
  * @return Point to the MXOS's version string.
  */
char* MxosGetVer( void );

/**
  * @brief  Initialize the TCPIP stack thread, RF driver thread, and other
            supporting threads needed for wlan connection. Do some necessary
            initialization
  *
  * @param  None
  *
  * @return kNoErr: success, kGeneralErr: fail
  */
OSStatus MxosInit( void );


/**
  * @brief  Get an identifier id from device, every id is unique and will not change in life-time
  *
  * @param  identifier length 
  *
  * @return Point to the identifier 
  */
const uint8_t* mxos_generate_cid( uint8_t *length );

/* Entry point for user Application */
int main( void );

/**
 *  Start the application at the given address. This function does
 *  not return. It is the applications responsibility for flushing to
 *  or powering down external components such as filesystems or
 *  socket connections before calling this function. For Cortex-M
 *  devices this function powers down generic system components such as
 *  the NVIC and set the vector table to that of the new image followed
 *  by jumping to the reset handler of the new image.
 *
 *  @param address    Starting address of next application to run
 */
void mxos_start_application(uintptr_t address);

#ifdef __cplusplus
} /*"C" */
#endif

#endif /* __MXOS_H_ */

/**
  * @}
  */

/**
  * @}
  */

