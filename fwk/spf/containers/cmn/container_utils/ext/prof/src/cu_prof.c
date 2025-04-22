/**
 * \file cu_prof.c
 * \brief
 *     This file contains container utility functions for profiling.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "cu_i.h"
#include "cu_prof.h"
/* =======================================================================
Static Function Definitions
========================================================================== */

/* ==========================================================================  */

ar_result_t cu_cntr_get_prof_info(cu_base_t *base_ptr, int8_t *param_payload_ptr, uint32_t *param_size_ptr)
{
   ar_result_t result = AR_EOK;

   result = base_ptr->topo_vtbl_ptr->get_prof_info((void *)base_ptr->topo_ptr, param_payload_ptr, param_size_ptr);
   return result;
}
