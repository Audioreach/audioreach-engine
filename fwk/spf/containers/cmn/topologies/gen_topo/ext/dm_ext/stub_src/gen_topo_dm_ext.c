/**
 * \file gen_topo_dm_extn.c
 * \brief
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_topo.h"

bool_t gen_topo_is_in_port_in_dm_variable_nblc(gen_topo_t *me_ptr, gen_topo_input_port_t *in_port_ptr)
{
   return FALSE;
}

bool_t gen_topo_is_out_port_in_dm_variable_nblc(gen_topo_t *me_ptr, gen_topo_output_port_t *out_port_ptr)
{
   return FALSE;
}

uint32_t gen_topo_compute_if_input_needs_addtional_bytes_for_dm(gen_topo_t *me_ptr, gen_topo_input_port_t *in_port_ptr)
{
   return 0;
}

uint32_t gen_topo_compute_if_output_needs_addtional_bytes_for_dm(gen_topo_t *            me_ptr,
                                                                 gen_topo_output_port_t *out_port_ptr)
{
   return 0;
}

ar_result_t gen_topo_update_dm_modes(gen_topo_t *topo_ptr)
{
   return AR_EOK;
}

ar_result_t gen_topo_updated_expected_samples_for_dm_modules(gen_topo_t *topo_ptr, gen_topo_module_t *module_ptr)
{
   return AR_EOK;
}

ar_result_t gen_topo_handle_dm_report_mode_event(gen_topo_t *                      topo_ptr,
                                                 gen_topo_module_t *               module_ptr,
                                                 capi_event_data_to_dsp_service_t *event_data_ptr)
{

   return AR_EOK;
}

ar_result_t gen_topo_send_dm_consume_partial_input(gen_topo_t *topo_ptr, gen_topo_module_t* module_ptr, bool_t should_consume_partial_input)
{
   return AR_EOK;
}
