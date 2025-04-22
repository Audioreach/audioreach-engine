#ifndef _APM_PROXY_VCPM_UTILS_H_
#define _APM_PROXY_VCPM_UTILS_H_

/**
 * \file apm_proxy_vcpm_utils.h
 * \brief
 *     This file contains APM proxy manager private function declarations
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spf_cmn_if.h"
#include "apm_msg_utils.h"
#include "apm_proxy_def.h"
#include "apm_internal.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/****************************************************************************
 * Macro Definition                                                         *
 ****************************************************************************/

/**********************************************************************

         Proxy utility function declarations

**********************************************************************/
bool_t apm_proxy_util_find_vcpm_proxy_mgr(apm_graph_info_t *       apm_graph_info_ptr,
                                          apm_proxy_manager_t **   proxy_mgr_pptr,
                                          apm_module_param_data_t *param_data);

ar_result_t apm_proxy_util_add_sg_to_proxy_cmd_ctrl(apm_cmd_ctrl_t *  apm_cmd_ctrl_ptr,
                                                    spf_list_node_t **proxy_manager_list_pptr,
                                                    uint32_t *        num_proxy_mgrs_ptr,
                                                    apm_sub_graph_t * sg_node_ptr);

ar_result_t apm_proxy_util_check_if_proxy_required(apm_cmd_ctrl_t *  apm_cmd_ctrl_ptr,
                                                   apm_graph_info_t *graph_info_ptr,
                                                   apm_sub_graph_t * sg_node_ptr);

ar_result_t apm_proxy_util_sort_graph_mgmt_sg_lists(apm_t *apm_info_ptr);

ar_result_t apm_proxy_graph_open_cmn_sequencer(apm_t *apm_info_ptr);

ar_result_t apm_proxy_graph_mgmt_sequencer(apm_t *apm_info_ptr);

ar_result_t apm_proxy_set_cfg_sequencer(apm_t *apm_info_ptr);

ar_result_t apm_proxy_util_send_graph_info_to_proxy_managers(apm_cmd_ctrl_t *apm_cmd_ctrl_ptr,
                                                             spf_handle_t *  apm_handle_ptr);

ar_result_t apm_proxy_util_send_mgmt_command_to_proxy_managers(apm_cmd_ctrl_t *apm_cmd_ctrl_ptr,
                                                               spf_handle_t *  apm_handle_ptr);

ar_result_t apm_proxy_util_add_sg_to_proxy_cmd_ctrl(apm_cmd_ctrl_t *  apm_cmd_ctrl_ptr,
                                                    spf_list_node_t **proxy_manager_list_pptr,
                                                    uint32_t *        num_proxy_mgrs_ptr,
                                                    apm_sub_graph_t * sg_node_ptr);

ar_result_t apm_populate_proxy_mgr_cmd_seq(apm_cmd_ctrl_t *apm_cmd_ctrl_ptr);

bool_t apm_proxy_util_find_proxy_manager(spf_list_node_t *     proxy_mgr_list_ptr,
                                         apm_proxy_manager_t **proxy_mgr_pptr,
                                         apm_sub_graph_t *     sg_node_ptr);

ar_result_t apm_proxy_util_validate_input_sg_list(spf_list_node_t *   proxy_manager_list_ptr,
                                                  apm_sub_graph_id_t *sg_array_ptr,
                                                  uint32_t            num_sgs);

ar_result_t apm_proxy_util_send_cfg_to_proxy_mgrs(spf_handle_t *    apm_handle_ptr,
                                                  apm_cmd_ctrl_t *  apm_cmd_ctrl_ptr,
                                                  apm_graph_info_t *graph_info_ptr);

ar_result_t apm_gm_cmd_handle_proxy_sub_graph_list(apm_t *apm_info_ptr);

ar_result_t apm_proxy_util_remove_closed_subgraphs_from_proxy_mgrs(apm_t *apm_info_ptr);

ar_result_t apm_clear_permitted_proxy_list(apm_cmd_ctrl_t *apm_cmd_ctrl_ptr);

ar_result_t apm_proxy_util_clear_vcpm_active_or_inactive_proxy(apm_t *apm_info_ptr, apm_cmd_ctrl_t *apm_cmd_ctrl_ptr);

// Response handlers
ar_result_t apm_proxy_graph_mgmt_cmd_hdlr(apm_t *apm_info_ptr, spf_msg_t *msg_ptr, bool_t *cmd_def_check_reqd_ptr);
ar_result_t apm_graph_proxy_permission_cmd_rsp(spf_msg_rsp_proxy_permission_t *proxy_response,
                                               apm_cmd_ctrl_t *                apm_curr_cmd_ctrl,
                                               apm_proxy_cmd_ctrl_t *          proxy_cmd_ctrl);
ar_result_t apm_graph_proxy_permission_rsp_hndlr(apm_t *apm_info_ptr, spf_msg_t *rsp_msg_ptr);
ar_result_t apm_graph_proxy_graph_info_rsp_hdlr(apm_t *apm_info_ptr, spf_msg_t *rsp_msg_ptr);
ar_result_t apm_graph_proxy_mgmt_cmd_rsp_hdlr(apm_t *apm_info_ptr, spf_msg_t *rsp_msg_ptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* _APM_PROXY_VCPM_UTILS_H_ */
