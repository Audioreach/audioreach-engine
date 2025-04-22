#ifndef CU_PATH_DELAY_H
#define CU_PATH_DELAY_H

/**
 * \file cu_path_delay.h
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
#include "apm_cntr_path_delay_if.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

typedef struct cu_base_t cu_base_t;

// clang-format off
#define CU_PATH_ID_ALL_PATHS 0
typedef enum cu_path_delay_op_t { CU_PATH_DELAY_OP_REMOVE = 1, CU_PATH_DELAY_OP_UPDATE = 2 } cu_path_delay_op_t;

uint32_t cu_aggregate_ext_in_port_delay(cu_base_t *base_ptr, gu_ext_in_port_t *gu_ext_in_port_ptr);
uint32_t cu_aggregate_ext_out_port_delay(cu_base_t *base_ptr, gu_ext_out_port_t *ext_out_port_ptr);
ar_result_t cu_operate_on_delay_paths(cu_base_t *base_ptr, uint32_t path_id, cu_path_delay_op_t op);
ar_result_t cu_update_path_delay(cu_base_t *base_ptr, uint32_t path_id);
ar_result_t cu_path_delay_cfg(cu_base_t *base_ptr, int8_t *   param_payload_ptr,
      uint32_t * param_size_ptr);
ar_result_t cu_destroy_delay_path_cfg(cu_base_t *base_ptr, int8_t *param_payload_ptr, uint32_t *param_size_ptr);
ar_result_t cu_cfg_src_mod_delay_list(cu_base_t *base_ptr, int8_t *param_payload_ptr, uint32_t *param_size_ptr);
ar_result_t cu_destroy_src_mod_delay_list(cu_base_t *base_ptr, int8_t *param_payload_ptr, uint32_t *param_size_ptr);
ar_result_t cu_handle_event_to_dsp_service_topo_cb_for_path_delay(cu_base_t *        cu_ptr,
                                                                  gu_module_t *      module_ptr,
                                                                  capi_event_info_t *event_info_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef CU_PATH_DELAY_H
