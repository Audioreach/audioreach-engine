/**
 * \file spl_cntr_proc_dur_fwk_ext.c
 *
 * \brief
 *     Stub file for container proc duration fwk extn implementation
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spl_cntr_proc_dur_fwk_ext.h"

ar_result_t spl_cntr_fwk_extn_set_cntr_proc_duration_per_module(spl_cntr_t *       me_ptr,
                                                                spl_topo_module_t *module_ptr,
                                                                uint32_t           cont_proc_dur_us)
{
   return AR_EOK;
}

/* Iterates over all modules in the sg list and tries to set proc delay if it supports this extension*/
ar_result_t spl_cntr_fwk_extn_set_cntr_proc_duration(spl_cntr_t *me_ptr, uint32_t cont_proc_dur_us)
{
   return AR_EOK;
}
