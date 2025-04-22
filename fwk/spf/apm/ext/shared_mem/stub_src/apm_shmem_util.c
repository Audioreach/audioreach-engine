/**
 * \file apm_shmem_util.c
 * \brief
 *     This file contains stubbed implementation of APM
 *     shared mem utils
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "apm_internal.h"

ar_result_t apm_shmem_utils_init(apm_t *apm_info_ptr)
{
   apm_info_ptr->ext_utils.shmem_vtbl_ptr = NULL;

   return AR_EOK;
}

