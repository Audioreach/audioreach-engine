#ifndef _APM_GPR_CMD_RSP_HDLR_H_
#define _APM_GPR_CMD_RSP_HDLR_H_

/**
 * \file apm_gpr_cmd_rsp_hdlr.h
 *
 * \brief
 *     This file contains function declaration for APM GPR Command Response Handler utilities
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

/**
  param[in]  : apm_info_ptr - Pointer to APM global info struct
  param[in] : msg_ptr: Pointer to rsp message payload

  Return: MM Error Code
 */

ar_result_t apm_rspq_gpr_cmd_rsp_handler(apm_t *apm_info_ptr, spf_msg_t *msg_ptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* _APM_GPR_CMD_RSP_HDLR_H_ */
