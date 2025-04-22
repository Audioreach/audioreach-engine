#ifndef _APM_GRAPH_UTILS_H__
#define _APM_GRAPH_UTILS_H__

/**
 * \file apm_graph_utils.h
 *  
 * \brief
 *     This file contains function declaration for APM graph managemeent utilities
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

/*------------------------------------------------------------------------------
 *  Constants/Macros
 *----------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 *  Function Declaration
 *----------------------------------------------------------------------------*/

ar_result_t apm_update_cont_graph(apm_graph_info_t *graph_info_ptr,
                                  apm_container_t * src_cont_node_ptr,
                                  apm_container_t * dst_cont_node_ptr,
                                  bool_t            retain_sorted_cont_graph);

ar_result_t apm_update_cont_graph_list(apm_graph_info_t *graph_info_ptr);

ar_result_t apm_remove_cont_from_graph(apm_graph_info_t *graph_info_ptr, apm_container_t *container_node_ptr);

ar_result_t apm_remove_sg_from_cont_graph(apm_graph_info_t *graph_info_ptr, apm_sub_graph_t *sub_graph_node_ptr);

ar_result_t apm_gm_cmd_get_cont_graph_node(apm_cmd_ctrl_t *apm_cmd_ctrl_ptr, spf_list_node_t **graph_node_pptr);

ar_result_t apm_set_up_graph_list_traversal(apm_graph_info_t *graph_info_ptr, apm_cmd_ctrl_t *apm_cmd_ctrl_ptr);

ar_result_t apm_gm_cmd_get_next_cont_in_sorted_list(apm_cmd_ctrl_t *  apm_cmd_ctrl_ptr,
                                                    apm_container_t **container_node_pptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* _APM_GRAPH_UTILS_H__ */
