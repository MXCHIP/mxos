/**
 ******************************************************************************
 * @file    SecurityUtils.h
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This header contains function prototypes that aid in security on
 *          the platform.
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


#ifndef __SecurityUtils_h__
#define __SecurityUtils_h__

#include "mxos_common.h"

//===========================================================================================================================
//  memcmp_constant_time
//
//  Compares memory so that the time it takes does not depend on the data being compared.
//  This is needed to avoid certain timing attacks in cryptographic software.
//===========================================================================================================================

/** @addtogroup MXOS_Middleware_Interface
  * @{
  */

/** @defgroup MXOS_Security_Utils MXOS Security Utils
  * @brief Provide APIs for Security Utils
  * @{
  */

/**
 * @brief ?
 *
 * @param inA  :   ?
 * @param inB  :   ?
 * @param inLen:   ?
 *
 * @return          ?
 */
int memcmp_constant_time( const void *inA, const void *inB, size_t inLen );
/**
  * @}
  */

/**
  * @}
  */
#endif // __SecurityUtils_h__


