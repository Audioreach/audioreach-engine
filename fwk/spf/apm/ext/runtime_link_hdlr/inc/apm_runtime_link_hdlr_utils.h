#ifndef _APM_RUNTIME_LINK_HDLR_H__
#define _APM_RUNTIME_LINK_HDLR_H__

/**
 * \file apm_runtime_link_hdlr_utils.h
 *
 * \brief
 *     This file contains function declaration for APM Link Open Handling across started subgraphs
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

typedef struct apm_runtime_link_hdlr_utils_vtable
{
   ar_result_t (*apm_check_and_cache_link_to_start_fptr)(apm_t *                apm_info_ptr,
                                                         apm_module_t **        module_node_ptr_list,
                                                         void *                 link_cfg_ptr,
                                                         spf_module_link_type_t link_type,
                                                         bool_t *               link_start_reqd_ptr);

   ar_result_t (*apm_graph_open_handle_link_start_fptr)(apm_t *apm_info_ptr);

} apm_runtime_link_hdlr_utils_vtable_t;

/**------------------------------------------------------------------------------
 *  Function Declaration
 *----------------------------------------------------------------------------*/

ar_result_t apm_runtime_link_hdlr_utils_init(apm_t *apm_info_ptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* _APM_RUNTIME_LINK_HDLR_H__ */
