/**
 * \file cu_offload_util.c
 * \brief
 *     This file contains container utility functions for external port handling (input and output).
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "cu_i.h"
#include "rd_sh_mem_ep_api.h"
#include "wr_sh_mem_ep_api.h"

// utility function to send ext output port property update to downstream peer port.
ar_result_t cu_send_upstream_state_to_parent_container(cu_base_t *        me_ptr,
                                                       gu_ext_out_port_t *gu_ext_out_port_ptr,
                                                       topo_port_state_t  ds_state)
{
   return AR_EOK;
}

// utility function to send ext output port property update to downstream peer port over GPR.
// This function is used in offload context, where the WR/RD_SHMEM_EP modules have to communicate
// the event from the satellite graph to the OLC in the master graph
ar_result_t cu_propagate_to_parent_container_ext_port(cu_base_t *      me_ptr,
                                                      spf_list_node_t *event_list_ptr,
                                                      uint32_t         event_id,
                                                      uint32_t         src_miid,
                                                      int8_t *         prop_ptr)
{
   ar_result_t result = AR_EOK;

   return result;
}

ar_result_t cu_offload_process_peer_port_property_propagation(cu_base_t *base_ptr, bool_t need_to_update_states)
{
   ar_result_t result = AR_EOK;

   return result;
}

ar_result_t cu_create_offload_info(cu_base_t *me_ptr, apm_prop_data_t *cntr_prop_ptr)
{
   ar_result_t result = AR_EOK;

   return result;
}

void cu_destroy_offload_info(cu_base_t *me_ptr)
{
}

ar_result_t cu_offload_ds_propagation_init(cu_base_t *base_ptr, cu_ext_out_port_t *cu_ext_out_port_ptr)
{
   return AR_EOK;
}

ar_result_t cu_offload_us_propagation_init(cu_base_t *base_ptr, cu_ext_in_port_t *cu_ext_in_port_ptr)
{
   return AR_EOK;
}

ar_result_t cu_raise_event_get_path_delay(cu_base_t *base_ptr,
                                          uint32_t   prev_delay_in_us,
                                          uint32_t   curr_delay_in_us,
                                          uint32_t   path_id)
{
   ar_result_t result = AR_EOK;
   return result;
}

ar_result_t cu_offload_handle_gpr_cmd(cu_base_t *me_ptr, bool_t *switch_case_found_ptr)
{
   *switch_case_found_ptr = FALSE;
   return AR_EUNSUPPORTED;
}
