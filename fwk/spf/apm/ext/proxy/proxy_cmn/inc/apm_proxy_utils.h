#ifndef _APM_PROXY_UTILS_H_
#define _APM_PROXY_UTILS_H_

/**
 * \file apm_proxy_utils.h
 * \brief
 *     This file contains APM proxy manager utility functions.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "apm_graph_db.h"
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
ar_result_t apm_free_proxy_mgr_from_id(apm_t *apm_info_ptr, uint32_t instance_id);

ar_result_t apm_handle_proxy_mgr_cfg_params(apm_t *                  apm_info_ptr,
                                            apm_module_param_data_t *mod_data_ptr,
                                            bool_t                   use_sys_q);

ar_result_t apm_proxy_manager_response_handler(apm_t *apm_info_ptr, spf_msg_t *rsp_msg_ptr);

ar_result_t apm_proxy_util_get_allocated_cmd_ctrl_obj(apm_proxy_manager_t *  proxy_mgr_ptr,
                                                      apm_cmd_ctrl_t *       apm_cmd_ctrl_ptr,
                                                      apm_proxy_cmd_ctrl_t **proxy_cmd_ctrl_pptr);

ar_result_t apm_proxy_util_get_cmd_ctrl_obj(apm_proxy_manager_t *  proxy_mgr_ptr,
                                            apm_cmd_ctrl_t *       apm_cmd_ctrl_ptr,
                                            apm_proxy_cmd_ctrl_t **proxy_cmd_ctrl_pptr);

ar_result_t apm_proxy_util_update_proxy_manager(apm_cmd_ctrl_t *apm_cmd_ctrl_ptr);

ar_result_t apm_aggregate_proxy_manager_cmd_response(apm_t *apm_info_ptr, spf_msg_t *rsp_msg_ptr);

ar_result_t apm_clear_active_proxy_list(apm_t *apm_info_ptr, apm_cmd_ctrl_t *apm_cmd_ctrl_ptr);

ar_result_t apm_proxy_util_release_cmd_ctrl_obj(apm_proxy_manager_t * proxy_mgr_ptr,
                                                apm_proxy_cmd_ctrl_t *proxy_cmd_ctrl_ptr);

ar_result_t apm_move_proxies_to_active_list(apm_t *apm_info_ptr);

ar_result_t apm_move_proxy_to_active_list_by_id(apm_t *apm_info_ptr, uint32_t instance_id);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* _APM_PROXY_UTILS_H_ */
