/**
 * \file spl_topo_dm_fwk_ext.c
 *
 * \brief
 *
 *     Topo 2 functions for managing dm extension stubs
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spl_topo_trigger_policy_fwk_ext.h"

/* =======================================================================
Public Function Definitions
========================================================================== */
//stub implementation
bool_t spl_topo_fwk_ext_is_dm_enabled(spl_topo_module_t *module_ptr)
{
   return FALSE;
}

bool_t spl_topo_fwk_ext_is_module_dm_fixed_out(spl_topo_module_t *module_ptr)
{
   return FALSE;
}

bool_t spl_topo_fwk_ext_is_module_dm_fixed_in(spl_topo_module_t *module_ptr)
{
   return FALSE;
}

bool_t spl_topo_fwk_ext_any_dm_module_present(spl_topo_dm_info_t dm_info)
{
   return FALSE;
}

bool_t spl_topo_fwk_ext_is_fixed_in_dm_module_present(spl_topo_dm_info_t dm_info)
{
   return FALSE;
}

bool_t spl_topo_fwk_ext_is_fixed_out_dm_module_present(spl_topo_dm_info_t dm_info)
{
   return FALSE;
}

ar_result_t spl_topo_fwk_ext_update_dm_modes(spl_topo_t *topo_ptr)
{
   return AR_EOK;
}

void spl_topo_fwk_ext_free_dm_req_samples(spl_topo_t *topo_ptr)
{
}

ar_result_t spl_topo_fwk_ext_handle_dm_report_samples_event(spl_topo_t *                      topo_ptr,
                                                            spl_topo_module_t *               module_ptr,
                                                            capi_event_data_to_dsp_service_t *event_data_ptr,
                                                            bool_t                            is_max)
{
   return AR_EOK;
}

ar_result_t spl_topo_fwk_ext_handle_dm_disable_event(spl_topo_t *                      topo_ptr,
                                                     spl_topo_module_t *               module_ptr,
                                                     capi_event_data_to_dsp_service_t *event_data_ptr)
{
   return AR_EOK;
}

ar_result_t spl_topo_fwk_ext_set_dm_samples_per_module(spl_topo_t *                        topo_ptr,
                                                       spl_topo_module_t *                 module_ptr,
                                                       fwk_extn_dm_param_id_req_samples_t *dm_req_samples_ptr,
                                                       bool_t                              is_max)
{
   return AR_EOK;
}
