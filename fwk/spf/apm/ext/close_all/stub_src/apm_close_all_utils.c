/**
 * \file apm_close_all_utils.c
 *  
 * \brief
 *  
 *     This file contains stubbed implementaiton of
        APM_CMD_CLOSE_ALL processing utilities.
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

ar_result_t apm_close_all_utils_init(apm_t *apm_info_ptr)
{
   apm_info_ptr->ext_utils.close_all_vtbl_ptr = NULL;

   return AR_EOK;
}
