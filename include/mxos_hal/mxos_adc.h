/**
 ******************************************************************************
 * @file    mxos_adc
 * @author  William Xu
 * @version V1.0.0
 * @date    16-Sep-2014
 * @brief   This file provides all the headers of ADC operation functions.
 ******************************************************************************
 *
 *  The MIT License
 *  Copyright (c) 2018 MXCHIP Inc.
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

#ifndef __MXOS_ADC_H__
#define __MXOS_ADC_H__

#include "mxos_common.h"
#include "platform_peripheral.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Legacy definitions */
#define MxosAdcInitialize           mxos_adc_init
#define MxosAdcFinalize             mxos_adc_deinit
#define MxosAdcTakeSample           mxos_adc_take_sample
#define MxosAdcTakeSampleStreram    mxos_adc_take_sample_streram

/** @addtogroup MXOS_PLATFORM
* @{
*/

/** @defgroup MXOS_ADC MXOS ADC Driver
  * @brief  Analog to Digital Converter (ADC) Functions
  * @{
  */

/******************************************************
 *                   Macros
 ******************************************************/  

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef int8_t mxos_adc_t;   /**< MXOS ADC peripheral handle, MXOS_ADC_XX define by board/<board_name>/mxos_board.h. */

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                     Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/**@biref Initialises an ADC interface
 *
 * Prepares an ADC hardware interface for sampling
 *
 * @param adc            : the interface which should be initialised
 * @param sampling_cycle : sampling period in number of ADC clock cycles. If the
 *                         MCU does not support the value provided, the closest
 *                         supported value is used.
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
merr_t mxos_adc_init( mxos_adc_t adc, uint32_t sampling_cycle );

/**@biref Get ADC sample bit ranges
 *
 * @param adc            : interface
 *
 * @return    Bit ranges mask, for example, 0xFFF means adc sample result has 12 bits resolution
 */
uint16_t mxos_adc_get_bit_range( mxos_adc_t adc );


/**@biref Takes a single sample from an ADC interface
 *
 * Takes a single sample from an ADC interface
 *
 * @param adc    : the interface which should be sampled
 * @param output : pointer to a variable which will receive the sample
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
merr_t mxos_adc_take_sample( mxos_adc_t adc, uint16_t* output );


/**@biref Takes multiple samples from an ADC interface
 *
 * Takes multiple samples from an ADC interface and stores them in
 * a memory buffer
 *
 * @param adc           : the interface which should be sampled
 * @param buffer        : a memory buffer which will receive the samples
 *                        Each sample will be uint16_t little endian.
 * @param buffer_length : length in bytes of the memory buffer.
 *
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
merr_t mxos_adc_take_sample_streram( mxos_adc_t adc, void* buffer, uint16_t buffer_length );


/**@biref     De-initialises an ADC interface
 *
 * @abstract Turns off an ADC hardware interface
 *
 * @param  adc : the interface which should be de-initialised
 *
 * @return    kNoErr        : on success.
 * @return    kGeneralErr   : if an error occurred with any step
 */
merr_t  mxos_adc_deinit( mxos_adc_t adc );

/** @} */
/** @} */

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif
