/**
 ******************************************************************************
 * @file    bmm050_user.c
 * @author  William Xu
 * @version V1.0.0
 * @date    21-May-2015
 * @brief   bmm050 sensor control operations.
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#ifndef __BMM050_USER_H_
#define __BMM050_USER_H_

#include "mxos_platform.h"
#include "mxos_board.h"


/** @addtogroup MXOS_Drivers_interface
  * @{
  */

/** @addtogroup MXOS_Sensor_Driver
  * @{
  */

/** @defgroup MXOS_BMM050_Driver MXOS BMM050 Driver
  * @brief Provide driver interface for BMM050 Sensor 
  * @{
  */

#define s32 int32_t
#define u32 uint32_t

#ifndef BMM050_I2C_DEVICE
  #define BMM050_I2C_DEVICE      MXOS_I2C_NONE
#endif



/**
 * @brief Initialize BMM050 sensor device.
 *
 * @return   kNoErr        : on success.
 * @return   kGeneralErr   : if an error occurred
 */
merr_t bmm050_sensor_init(void);


/**
 * @brief Read data from BMM050 sensor device.
 *
 * @param   v_mag_datax_s16 : mag X data
 * @param   v_mag_datay_s16 : mag Y data
 * @param   v_mag_datay_s16 : mag Z data
 *
 * @return   kNoErr        : on success.
 * @return   kGeneralErr   : if an error occurred
 */
merr_t bmm050_data_readout(s16 *v_mag_datax_s16, s16 *v_mag_datay_s16, s16 *v_mag_dataz_s16);


/**
 * @brief Deinitialize BMM050 sensor device.
 *
 * @return   kNoErr        : on success.
 * @return   kGeneralErr   : if an error occurred
 */
merr_t bmm050_sensor_deinit(void);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif



