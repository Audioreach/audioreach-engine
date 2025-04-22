/**
 * \file apm_sys_util_stub.c
 *
 * \brief
 *
 *     This file contains stubbed implementation of
        APM Sys utils.
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

ar_result_t apm_sys_util_init(apm_t *apm_info_ptr)
{
   apm_info_ptr->ext_utils.sys_util_vtbl_ptr = NULL;

   return AR_EOK;
}

ar_result_t apm_sys_util_deinit()
{
   return AR_EOK;
}
