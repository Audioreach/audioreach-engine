/**
 * \file spf_minidump.c
 * \brief
 *     This file contains the callback for minidump in 
 *         SPF framework
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "posal.h"
#include "spf_utils.h"
#include "spf_main.h"
#include "apm_debug_info_dump.h"

/* =======================================================================
**                          Macro definitions
** ======================================================================= */

/* =======================================================================
**                          Function Definitions
** ======================================================================= */

/* =======================================================================
**                          Global Variable Definitions
** ======================================================================= */

void spf_debug_info_dump(void *callback_data,int8_t *start_address,uint32_t max_size)
{
    apm_cntr_dump_debug_info(NULL,start_address,max_size);
    return;
}