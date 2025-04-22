#ifndef _APM_SPF_CMD_HDLR_H_
#define _APM_SPF_CMD_HDLR_H_

/**
 * \file apm_spf_cmd_hdlr.h
 *
 * \brief
 *     This file contains function declaration for APM SPF Command Handler utilities
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

ar_result_t apm_cmdq_spf_cmd_handler(apm_t *apm_info_ptr, spf_msg_t *msg_ptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* _APM_SPF_CMD_HDLR_H_ */
