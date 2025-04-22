/**
 * \file gen_topo_island.c
 *  
 * \brief
 *  
 *     Basic topology implementation.
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_topo.h"
#include "gen_topo_capi.h"

// clang-format off
const gen_topo_vtable_t gen_topo_vtable =
{
   .capi_get_required_fmwk_extensions    = gen_topo_capi_get_required_fmwk_extensions,
   .input_port_is_trigger_present        = gen_topo_input_port_is_trigger_present,
   .output_port_is_trigger_present       = gen_topo_output_port_is_trigger_present,
   .input_port_is_trigger_absent         = gen_topo_input_port_is_trigger_absent,
   .output_port_is_trigger_absent        = gen_topo_output_port_is_trigger_absent,
   .output_port_is_size_known            = gen_topo_output_port_is_size_known,
   .get_out_port_data_len                = gen_topo_get_out_port_data_len,  
   .is_port_at_nblc_end					     = gen_topo_is_port_at_nblc_end,
   .update_module_info				        = NULL,
};
// clang-format on

const topo_cu_island_vtable_t gen_topo_cu_island_vtable =
{
   .handle_incoming_ctrl_intent         = gen_topo_handle_incoming_ctrl_intent
};

/**
 * index by capi data format (index cannot be MAX_FORMAT_TYPE)
 */
data_format_t gen_topo_convert_spf_data_format_to_capi_data_format(spf_data_format_t spf_data_fmt)
{
   if (spf_data_fmt > SPF_DEINTERLEAVED_RAW_COMPRESSED)
   {
      return CAPI_MAX_FORMAT_TYPE;
   }
   else
   {
      uint32_t index = 32 - s32_cl0_s32(spf_data_fmt); // SPF_FIXED_POINT = 1<<0. Leading zeros is 31. index is 1.
      const data_format_t mapper[] = { CAPI_MAX_FORMAT_TYPE,
                                       CAPI_FIXED_POINT,
                                       CAPI_FLOATING_POINT,
                                       CAPI_RAW_COMPRESSED,
                                       CAPI_IEC61937_PACKETIZED,
                                       CAPI_DSD_DOP_PACKETIZED,
                                       CAPI_COMPR_OVER_PCM_PACKETIZED,
                                       CAPI_GENERIC_COMPRESSED,
                                       CAPI_IEC60958_PACKETIZED,
                                       CAPI_IEC60958_PACKETIZED_NON_LINEAR,
                                       CAPI_DEINTERLEAVED_RAW_COMPRESSED};
      if (index < SIZE_OF_ARRAY(mapper))
      {
         return mapper[index];
      }
      else
      {
         return CAPI_MAX_FORMAT_TYPE;
      }
   }
}

/** island exit function **/
void gen_topo_exit_island_temporarily(gen_topo_t *topo_ptr)
{
   if (POSAL_IS_ISLAND_HEAP_ID(topo_ptr->heap_id) &&
       ((PM_ISLAND_VOTE_ENTRY == topo_ptr->flags.aggregated_island_vote) ||
        (PM_ISLAND_VOTE_DONT_CARE == topo_ptr->flags.aggregated_island_vote)))
   {
      if (posal_island_get_island_status_inline())
      {
         TOPO_MSG_ISLAND(topo_ptr->gu.log_id, DBG_LOW_PRIO, "exiting island");
      }
      posal_island_trigger_island_exit();
   }
}
