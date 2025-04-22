#ifndef _APM_DATA_PATH_UTILS_H_
#define _APM_DATA_PATH_UTILS_H_

/**
 * \file apm_data_path_utils.h
 *
 * \brief
 *     This file contains declarations for APM data path hander utility functions.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "apm_i.h"
#include "apm_graph_db.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/**------------------------------------------------------------------------------
 *  Structure Definition
 *----------------------------------------------------------------------------*/

#define APM_DATA_PATH_ID_INVALID (0)

typedef struct apm_data_path_utils_vtable
{
   ar_result_t (*apm_data_path_cfg_cmn_seqncer_fptr)(apm_t *apm_info_ptr, bool_t data_path_create);

   ar_result_t (*apm_close_data_path_list_fptr)(apm_t *apm_info_ptr);

   ar_result_t (*apm_update_get_data_path_cfg_rsp_payload_fptr)(apm_t *                  apm_info_ptr,
                                                                spf_msg_header_t *       msg_hdr_ptr,
                                                                apm_module_param_data_t *cont_param_data_hdr_ptr);

   ar_result_t (*apm_cont_path_delay_msg_rsp_hdlr_fptr)(apm_t *apm_info_ptr);

   ar_result_t (*apm_path_delay_event_hdlr_fptr)(apm_t *apm_info_ptr, spf_msg_t *msg_ptr);

   ar_result_t (*apm_process_get_cfg_path_delay_fptr)(apm_t *                  apm_info_ptr,
                                                      apm_module_param_data_t *get_cfg_rsp_payload_ptr);

   ar_result_t (*apm_destroy_one_time_data_paths_fptr)(apm_t *apm_info_ptr);

   ar_result_t (*apm_graph_open_cmd_cache_data_link_fptr)(apm_module_conn_cfg_t *data_link_cfg_ptr,
                                                          apm_module_t **        module_node_pptr);

   ar_result_t (*apm_compute_cntr_path_delay_param_payload_size_fptr)(uint32_t                    container_id,
                                                                      apm_cont_set_get_cfg_hdr_t *set_get_cfg_hdr_ptr,
                                                                      uint32_t *                  msg_payload_size_ptr);

   ar_result_t (*apm_populate_cntr_path_delay_params_fptr)(uint32_t                    container_id,
                                                           apm_cont_set_get_cfg_hdr_t *set_get_cfg_hdr_ptr,
                                                           apm_module_param_data_t *   param_data_ptr);

   ar_result_t (*apm_data_path_clear_cached_cont_cfg_params_fptr)(spf_list_node_t *param_node_ptr);

   ar_result_t (*apm_clear_module_data_port_conn_fptr)(apm_t *apm_info_ptr, apm_module_t *self_module_node_ptr);

   ar_result_t (*apm_clear_module_single_port_conn_fptr)(apm_t *                 apm_info_ptr,
                                                         spf_module_port_conn_t *module_port_conn_ptr);

   ar_result_t (*apm_clear_closed_cntr_from_data_paths_fptr)(apm_t *          apm_info_ptr,
                                                             apm_container_t *closing_container_node_ptr);

} apm_data_path_utils_vtable_t;

/**------------------------------------------------------------------------------
 *  Function Declaration
 *----------------------------------------------------------------------------*/

ar_result_t apm_data_path_utils_init(apm_t *apm_info_ptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /** #ifdef _APM_DATA_PATH_UTILS_H_ */
