#ifndef _APM_PROXY_VCPM_UTILS_I_H_
#define _APM_PROXY_VCPM_UTILS_I_H_

/**
 * \file apm_proxy_utils_i.h
 * \brief
 *     This file contains APM proxy manager private function declarations
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

#include "apm_internal.h"
#include "apm_proxy_def.h"

/****************************************************************************
 * Macro Definition                                                         *
 ****************************************************************************/

/**********************************************************************

         Proxy utility function declarations

**********************************************************************/

ar_result_t apm_proxy_util_process_pending_subgraphs(apm_t *apm_info_ptr, apm_cmd_ctrl_t *apm_curr_cmd_ctrl);

ar_result_t apm_gm_cmd_clear_proxy_mgr_sg_info(apm_t *apm_info_ptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* _APM_PROXY_VCPM_UTILS_I_H_ */
