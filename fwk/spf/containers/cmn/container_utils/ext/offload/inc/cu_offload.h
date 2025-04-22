#ifndef CU_OFFLOAD_H
#define CU_OFFLOAD_H

/**
 * \file cu_offload.h
 *  
 * \brief
 *  
 *     Common container framework code.
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "posal.h"
#include "topo_utils.h"
#include "spf_list_utils.h"

#include "offload_apm_api.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

typedef struct cu_base_t         cu_base_t;
typedef struct cu_ext_out_port_t cu_ext_out_port_t;
typedef struct cu_ext_in_port_t  cu_ext_in_port_t;

// clang-format off
ar_result_t cu_send_upstream_state_to_parent_container(cu_base_t *        me_ptr,
                                                       gu_ext_out_port_t *ext_out_port_ptr,
                                                       topo_port_state_t  ds_state);
ar_result_t cu_propagate_to_parent_container_ext_port(cu_base_t *     me_ptr,
                                                      spf_list_node_t *event_list_ptr,
                                                      uint32_t        event_id,
                                                      uint32_t        src_miid,
                                                      int8_t *        prop_ptr);
ar_result_t cu_offload_process_peer_port_property_propagation(cu_base_t *base_ptr, bool_t need_to_update_states);
ar_result_t cu_create_offload_info(cu_base_t *me_ptr, apm_prop_data_t *cntr_prop_ptr);
void cu_destroy_offload_info(cu_base_t *me_ptr);
ar_result_t cu_offload_ds_propagation_init(cu_base_t *base_ptr, cu_ext_out_port_t *cu_ext_out_port_ptr);
ar_result_t cu_offload_us_propagation_init(cu_base_t *base_ptr, cu_ext_in_port_t *cu_ext_in_port_ptr);
ar_result_t cu_raise_event_get_path_delay(cu_base_t *base_ptr,
                                          uint32_t   prev_delay_in_us,
                                          uint32_t   curr_delay_in_us,
                                          uint32_t   path_id);
ar_result_t cu_offload_handle_gpr_cmd(cu_base_t *me_ptr, bool_t *switch_case_found_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef CU_OFFLOAD_H
