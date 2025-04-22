/**
 * \file apm_err_hdlr_utils.c
 *
 * \brief
 *     This file contains stubbed implementation of utility
 *     functions for error handling during command processing
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

ar_result_t apm_err_hdlr_utils_init(apm_t *apm_info_ptr)
{
   apm_info_ptr->ext_utils.err_hdlr_vtbl_ptr = NULL;

   return AR_EOK;
}


