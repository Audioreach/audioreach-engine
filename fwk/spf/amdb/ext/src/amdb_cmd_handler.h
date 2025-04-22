#ifndef _AMDB_CMD_HANDLER_H_
#define _AMDB_CMD_HANDLER_H_
/**
 * \file amdb_cmd_handler.h
 * \brief
 *     This file contains command handler for AMDB
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spf_cmn_if.h"
#include "amdb_thread_i.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/**
  AMDB Service get command Q.
  param[out] p_cmd_q: Pointer to cmdQ pointer

  return: None
 */
ar_result_t amdb_cmdq_gpr_cmd_handler(amdb_thread_t *amdb_info_ptr, spf_msg_t *msg_ptr);

/**
  AMDB Service get command Q.
  param[out] p_rsp_q: Pointer to rspQ pointer

  return: None
 */
ar_result_t amdb_rspq_gpr_rsp_handler(amdb_thread_t *amdb_info_ptr, spf_msg_t *msg_ptr);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _AMDB_CMD_HANDLER_H_ */
