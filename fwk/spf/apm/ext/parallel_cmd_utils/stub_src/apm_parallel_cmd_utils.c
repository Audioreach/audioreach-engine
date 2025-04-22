/**
 * \file apm_parallel_cmd_utils.c
 *
 * \brief
 *     This file contains stubbed implementation for
 *     APM parallel command handler utility functions
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
#include "apm_ext_cmn.h"

/****************************************************************************
 * Function Definitions
 ****************************************************************************/

ar_result_t apm_parallel_cmd_utils_init(apm_t *apm_info_ptr)
{
   apm_info_ptr->ext_utils.parallel_cmd_utils_vtbl_ptr = NULL;

   return AR_EOK;
}
