#ifndef _APM_CMD_UTILS_H_
#define _APM_CMD_UTILS_H_

/**
 * \file apm_cmd_utils.h
 *
 * \brief
 *     This file contains declarations for APM command handling utility functions.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "apm_internal.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

ar_result_t apm_allocate_cmd_hdlr_resources(apm_t *apm_info_ptr, spf_msg_t *msg_ptr);

ar_result_t apm_deallocate_cmd_hdlr_resources(apm_t *apm_info_ptr, apm_cmd_ctrl_t *apm_cmd_ctrl_ptr);

ar_result_t apm_clear_graph_open_cfg(apm_t *apm_info_ptr);

ar_result_t apm_clear_pspc_cont_mod_list(apm_t *              apm_info_ptr,
                                         apm_container_t *    container_node_ptr,
                                         apm_cont_cmd_ctrl_t *cont_cmd_ctrl_ptr,
                                         spf_list_node_t *    sub_graph_list_ptr);

ar_result_t apm_handle_cmd_parse_failure(apm_t *apm_info_ptr, ar_result_t cmd_cfg_parse_result);

ar_result_t apm_end_cmd(apm_t *apm_info_ptr);

ar_result_t apm_validate_module_instance_pair(apm_graph_info_t *graph_info_ptr,
                                              uint32_t          peer_1_module_iid,
                                              uint32_t          peer_2_module_iid,
                                              apm_module_t **   module_node_pptr,
                                              bool_t            dangling_link_allowed);

bool_t apm_gm_cmd_is_sg_id_present(apm_cmd_ctrl_t *apm_cmd_ctrl_ptr, uint32_t sg_id);

ar_result_t apm_validate_sg_state_and_add_to_process_list(apm_cmd_ctrl_t * apm_cmd_ctrl_ptr,
                                                          apm_sub_graph_t *sub_graph_node_ptr,
                                                          bool_t           is_gm_cmd_sg_id);

ar_result_t apm_clear_graph_mgmt_cmd_info(apm_t *apm_info_ptr, apm_cmd_ctrl_t *apm_cmd_ctrl_ptr);

ar_result_t apm_validate_and_cache_link_for_state_mgmt(apm_t *                    apm_info_ptr,
                                                       apm_graph_mgmt_cmd_ctrl_t *gm_cmd_ctrl_ptr,
                                                       apm_module_t **            module_node_list_pptr);

apm_sub_graph_state_t apm_gm_cmd_get_default_sg_list_state(uint32_t cmd_opcode);

ar_result_t apm_gm_cmd_cfg_sg_list_to_process(apm_t *apm_info_ptr);

ar_result_t apm_gm_cmd_validate_sg_list(apm_t *             apm_info_ptr,
                                        uint32_t            num_sub_graphs,
                                        apm_sub_graph_id_t *sg_id_list_ptr,
                                        uint32_t            cmd_opcode);

ar_result_t apm_parse_cmd_payload(apm_t *apm_info_ptr);

ar_result_t apm_parse_graph_mgmt_sg_id_list(apm_t *             apm_info_ptr,
                                            apm_sub_graph_id_t *sg_id_list_ptr,
                                            uint32_t            num_sg,
                                            uint32_t            cmd_opcode);

uint32_t apm_get_cmd_opcode_from_msg_payload(spf_msg_t *msg_ptr);

ar_result_t apm_check_sg_id_overlap_across_parallel_cmds(apm_t *         apm_info_ptr,
                                                         apm_cmd_ctrl_t *curr_cmd_ctrl_ptr,
                                                         bool_t *        sg_list_overlap_ptr);

/****************************************************************************
 *  Inline Function Definitions                                             *
 ****************************************************************************/

static inline ar_result_t apm_cmd_ctrl_clear_cmd_pending_status(apm_cmd_ctrl_t *apm_cmd_ctrl_ptr, ar_result_t result)
{
   /** Update overall command status  */
   apm_cmd_ctrl_ptr->cmd_status = result;

   /** Set the command pending status to FALSE */
   apm_cmd_ctrl_ptr->cmd_pending = FALSE;

   return AR_EOK;
}

static inline apm_cmd_ctrl_t *apm_get_nth_cmd_ctrl_obj(apm_t *apm_info_ptr, uint32_t n)
{
   return (&(apm_info_ptr->cmd_ctrl_list[n]));
}

static inline bool_t apm_is_module_static_inst_id(uint32_t instance_id)
{
   return (instance_id > AR_SPF_STATIC_INSTANCE_ID_RANGE_BEGIN && instance_id <= AR_SPF_STATIC_INSTANCE_ID_RANGE_END);
}

static inline bool_t apm_defer_close_all_cmd(apm_t *apm_info_ptr)
{
   return (apm_info_ptr->active_cmd_mask && (APM_CMD_CLOSE_ALL == apm_info_ptr->curr_cmd_ctrl_ptr->cmd_opcode));
}

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* _APM_CMD_UTILS_H_ */
