#ifndef _APM_CNTR_PEER_HEAP_UTILS_H__
#define _APM_CNTR_PEER_HEAP_UTILS_H__

/**
 * \file apm_cntr_peer_heap_utils.h
 *
 * \brief
 *     This file contains function declaration for APM utilities for Container Peer Heap ID
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "apm_i.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/**------------------------------------------------------------------------------
 *  Structure Definition
 *----------------------------------------------------------------------------*/

typedef struct apm_cntr_peer_heap_utils_vtable
{
   ar_result_t (*apm_cache_mxd_heap_id_cntr_link_in_db_fptr)(apm_cont_cmd_ctrl_t *cont_cmd_ctrl_ptr,
                                                             void *               curr_conn_cfg_ptr,
                                                             uint32_t             peer_heap_id,
                                                             bool_t               is_data_link);

   void (*apm_check_and_handle_mixed_heap_id_cntr_links_fptr)(apm_module_t **module_node_ptr_list,
                                                              uint32_t *     cont_heap_id_ptr_arr,
                                                              bool_t *       is_mixed_heap_data_link_ptr,
                                                              bool_t         is_data_link);

} apm_cntr_peer_heap_utils_vtable_t;

/**------------------------------------------------------------------------------
 *  Function Declaration
 *----------------------------------------------------------------------------*/

ar_result_t apm_cntr_peer_heap_utils_init(apm_t *apm_info_ptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* _APM_CNTR_PEER_HEAP_UTILS_H__ */
