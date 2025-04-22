/**
 * \file apm_pwr_mgr_utils.c
 *
 * \brief
 *     This file contains Stubbed Implementation of APM Power
 *     Manager Utilities
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/****************************************************************************
 * INCLUDE HEADER FILES                                                     *
 ****************************************************************************/

#include "apm_internal.h"

/****************************************************************************
 * Function Definitions
 ****************************************************************************/

ar_result_t apm_pwr_mgr_utils_init(apm_t *apm_info_ptr)
{
   apm_info_ptr->ext_utils.pwr_mgr_vtbl_ptr = NULL;

   return AR_EOK;
}
