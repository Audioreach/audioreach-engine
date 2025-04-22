#ifndef _APM_DEBUG_INFO_CFG_H__
#define _APM_DEBUG_INFO_CFG_H__

/**
 * \file apm_debug_info_cfg.h
 *
 * \brief
 *     This file contains function declaration for APM framework parameters Apm Debug Info Config utilities
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "apm_debug_info.h"
#include "ar_osal_error.h"
#include "ar_error_codes.h"
//#include "comdef.h"
#include "apm_api.h"
#include "apm_i.h"
#include "ar_msg.h"
#include "apm_graph_db.h"
#include "rtm_logging_api.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/**------------------------------------------------------------------------------
 *  Structure Definition
 *----------------------------------------------------------------------------*/

typedef struct apm_debug_info_utils_vtable_t
{
   ar_result_t (*apm_parse_debug_info_fptr)(apm_t *apm_info_ptr, apm_module_param_data_t *mod_data_ptr);
   ar_result_t (*apm_log_sg_state_change_fptr)(spf_list_node_t *processed_sg_list_ptr, uint32_t num_processed_sg);

} apm_debug_info_utils_vtable_t;

/**------------------------------------------------------------------------------
 *  Function Declaration
 *----------------------------------------------------------------------------*/

ar_result_t apm_debug_info_init(apm_t *apm_info_ptr);
//ar_result_t apm_sg_state_change(spf_list_node_t *processed_sg_list_ptr, uint32_t num_processed_sg);
ar_result_t apm_populate_port_mf_cntr_param_payload_size(uint32_t                    container_id,
                                                         apm_cont_set_get_cfg_hdr_t *set_get_cfg_hdr_ptr,
                                                         apm_module_param_data_t *   param_ptr);
ar_result_t apm_compute_port_mf_cntr_msg_payload_size(uint32_t *cntr_msg_payload_size_ptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* _APM_DEBUG_INFO_CFG_H__ */
