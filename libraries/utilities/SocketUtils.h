/**
 ******************************************************************************
 * @file    SocketUtils.h
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This header contains function prototypes called by socket operation
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

#ifndef __SocketUtils_h__
#define __SocketUtils_h__

#include "mxos_common.h"

/** @addtogroup MXOS_Middleware_Interface
  * @{
  */

/** @defgroup MXOS_Socket_Tool MXOS Socket Tool
  * @brief Provide APIs for socket tool
  * @{
  */


/**
 * @brief ?
 *
 * @param fd:   ?
 * @param inBuf:   ?
 * @param inBufLen:   ?
 *
 * @return   kNoErr        : on success.
 * @return   kGeneralErr   : if an error occurred
 */

mret_t SocketSend( int fd, const uint8_t *inBuf, size_t inBufLen );


/**
 * @brief ?
 *
 * @param fd:   ?
 *
 * @return knone
 */
void SocketClose(int* fd);


/**
 * @brief ?
 *
 * @param fd:   ?
 *
 * @return none
 */
void SocketCloseForOSEvent(int* fd);



/**
 * @brief ?
 *
 * @param fd:   ?
 *
 * @return none
 */
void SocketAccept(int *plocalTcpClientsPool, int maxClientsNum, int newFd);


/**
  * @}
  */

/**
  * @}
  */


#endif // __SocketUtils_h__


