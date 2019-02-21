/**
 ******************************************************************************
 * @file    CheckSumUtils.h
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This header contains function prototypes which aid in checksum calculations.
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


#ifndef __CheckSumUtils_h__
#define __CheckSumUtils_h__

#include "mxos_common.h"


/** \defgroup MXOS_Algorithm_APIs MXOS Algorithm APIs
  * @brief Provide commonly used Algorithm APIs
  * @{
  */

/** @addtogroup MXOS_Algorithm_APIs
  * @{
  */

/** @defgroup MXOS_CRC8_Check MXOS CRC8 Check
  * @brief    MXOS CRC8 Check Function
  * @{
  */

/*********************  CRC8 MXOS Check **************************************

******************CRC-8 XMODEM       x8+x5+x4+1******************************

*******************************************************************************/

typedef struct
{
  uint8_t crc;
} CRC8_Context;

/**
 * @brief             initialize the CRC8 Context
 *
 * @param inContext   holds CRC8 result
 *
 * @retval            none
 */
void CRC8_Init( CRC8_Context *inContext );


/**
 * @brief             Caculate the CRC8 result
 *
 * @param inContext   holds CRC8 result during caculation process
 * @param inSrc       input data
 * @param inLen       length of input data
 *
 * @retval            none
 */
void CRC8_Update( CRC8_Context *inContext, const void *inSrc, size_t inLen );


/**
 * @brief             output CRC8 result
 *
 * @param inContext   holds CRC8 result
 * @param outResutl   holds CRC8 final result
 *
 * @retval            none
 */
void CRC8_Final( CRC8_Context *inContext, uint8_t *outResult );
/**
  * @}
  */




/*********************  CRC16 MXOS Check  **************************************

******************CRC-16/XMODEM       x16+x12+x5+1******************************

*******************************************************************************/

/** @defgroup MXOS_CRC16_Check MXOS CRC16 Check
  * @brief    MXOS CRC16 Check Functions
  * @{
  */

/**
 * @brief             caculate the CRC16 result for each byte 
 *
 * @param crcIn       The context to save CRC16 result.
 * @param byte        each byte of input data.
 *
 * @retval            return the CRC16 result
 */


typedef struct
{
  uint16_t crc;
} CRC16_Context;

/**
 * @brief             initialize the CRC16 Context
 *
 * @param inContext   holds CRC16 result
 *
 * @retval            none
 */
void CRC16_Init( CRC16_Context *inContext );


/**
 * @brief             Caculate the CRC16 result
 *
 * @param inContext   holds CRC16 result during caculation process
 * @param inSrc       input data
 * @param inLen       length of input data
 *
 * @retval            none
 */
void CRC16_Update( CRC16_Context *inContext, const void *inSrc, size_t inLen );


/**
 * @brief             output CRC16 result
 *
 * @param inContext   holds CRC16 result
 * @param outResutl   holds CRC16 final result
 *
 * @retval            none
 */
void CRC16_Final( CRC16_Context*inContext, uint16_t *outResult );

/**
  * @}
  */

/**
  * @}
  */
#endif //__CheckSumUtils_h__


