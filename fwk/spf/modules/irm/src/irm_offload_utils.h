#ifndef _IRM_OFFLOAD_UTILS_H_
#define _IRM_OFFLOAD_UTILS_H_

/**
 * \file irm_offload_utils.h
 * \brief
 *     This file contains utility functions for IRM command handling
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/****************************************************************************
 * INCLUDE HEADER FILES                                                     *
 ****************************************************************************/
#include "spf_cmn_if.h"
#include "gpr_api_inline.h"
#include "irm_i.h"
#include "spf_list_utils.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus



/****************************************************************************
 * Structure Definitions                                                    *
 ****************************************************************************/

ar_result_t irm_clean_up_proc_id_cmd_ctrl(irm_t *irm_ptr, uint32_t proc_domain_id);

ar_result_t irm_set_cmd_ctrl(irm_t *          irm_ptr,
                             spf_msg_t *      msg_ptr,
                             void *           master_payload_ptr,
                             bool_t           is_out_of_band,
                             irm_cmd_ctrl_t **curr_cmd_ctrl_pptr,
                             uint32_t         dst_domain_id);

ar_result_t irm_clear_cmd_ctrl(irm_t *irm_ptr, irm_cmd_ctrl_t *irm_cmd_ctrl_ptr);

/* Returns TRUE if any irm_cmd_ctrl_t in the irm structure's cmd_ctrl_list are pending a resp from the Sat PD */
bool_t irm_offload_any_cmd_pending(irm_t* irm_ptr);

ar_result_t irm_route_cmd_to_satellite(irm_t *    irm_ptr,
                                       spf_msg_t *msg_ptr,
                                       uint32_t   dst_domain_id,
                                       uint8_t *  irm_payload_ptr);

ar_result_t irm_route_basic_rsp_to_client(irm_t *irm_ptr, spf_msg_t *msg_ptr);

ar_result_t irm_route_apm_cmd_to_satellite(irm_t *irm_ptr, spf_msg_t *msg_ptr, uint32_t dst_domain_id);

ar_result_t irm_route_load_rsp_to_client(irm_t *irm_ptr, spf_msg_t *msg_ptr);


ar_result_t irm_send_get_cfg_cmd_to_satellite(irm_t *    irm_ptr,
                                       uint32_t opcode,
                                       uint32_t param_id,
                                       uint32_t host_domain_id,
                                       uint32_t   dst_domain_id,
                                       uint32_t payload_size,
                                       uint8_t *  resp_ptr,
                                       uint8_t* payload_ptr,
                                       irm_cmd_ctrl_t* curr_cmd_ctrl_ptr);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _IRM_OFFLOAD_UTILS_H_ */
