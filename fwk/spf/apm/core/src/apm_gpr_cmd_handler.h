#ifndef _APM_GPR_CMD_HDLR_H_
#define _APM_GPR_CMD_HDLR_H_

/**
 * \file apm_gpr_cmd_handler.h
 *
 * \brief
 *     This file contains private declarations of the Graphite server service.
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

/**
  APM Service get command Q.
  param[out] p_cmd_q: Pointer to cmdQ pointer

  return: None
 */
ar_result_t apm_cmdq_gpr_cmd_handler(apm_t *apm_info_ptr, spf_msg_t *msg_ptr);

ar_result_t apm_cmdq_spf_cmd_handler(apm_t *apm_info_ptr, spf_msg_t *msg_ptr);

ar_result_t apm_allocate_cmd_rsp_payload(uint32_t       log_id,
                                         gpr_packet_t * gpr_pkt_ptr,
                                         gpr_packet_t **gpr_rsp_pkt_pptr,
                                         uint32_t       cmd_rsp_opcode,
                                         uint32_t       cmd_rsp_payload_size);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* _APM_GPR_CMD_HDLR_H_ */
