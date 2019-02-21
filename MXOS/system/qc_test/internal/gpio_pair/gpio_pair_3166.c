/**
 ******************************************************************************
 * @file    gpio_pair_3081.c
 * @author  William Xu
 * @version V1.0.0
 * @date    18-Dec-2016
 * @brief   This file provide the definition of GPIO pairs used for GPIO QC test.
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


#include "qc_test_internal.h"
#include "mxos_board.h"

const qc_test_gpio_pair_t qc_test_gpio_pairs[] =
{
    {MXOS_GPIO_2,  MXOS_GPIO_4},
    {MXOS_GPIO_5,  MXOS_GPIO_6},
    {MXOS_GPIO_6,  MXOS_GPIO_7},
    {MXOS_GPIO_16,  MXOS_GPIO_17},
    {MXOS_GPIO_18,  MXOS_GPIO_19},
    {MXOS_GPIO_27,  MXOS_GPIO_31},
    {MXOS_GPIO_33,  MXOS_GPIO_34},
    {MXOS_GPIO_35,  MXOS_GPIO_38},
   // {MXOS_GPIO_35,  MXOS_GPIO_38}
};

const int qc_test_gpio_pairs_num = sizeof(qc_test_gpio_pairs) / sizeof(qc_test_gpio_pair_t);









