#ifndef _DLS_GPR_CMD_HANDLER_H_
#define _DLS_GPR_CMD_HANDLER_H_
/**
 * \file dls_gpr_cmd_handler.h
 * \brief
 *  	This file contains the DLS service GPR command handler public function
 *    declaration
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* ----------------------------------------------------------------------------
 * Include Files
 * ------------------------------------------------------------------------- */
#include "dls_i.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/**
  DLS Service get command Q.
  param[out] p_cmd_q: Pointer to cmdQ pointer

  return: None
 */
ar_result_t dls_cmdq_gpr_cmd_handler(dls_t *dls_info_ptr, spf_msg_t *msg_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //_DLS_GPR_CMD_HANDLER_H_
