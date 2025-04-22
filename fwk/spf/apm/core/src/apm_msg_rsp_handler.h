/**
 * \file apm_msg_rsp_handler.h
 *  
 * \brief
 *     This header file contains function declarations for response handlers for APM Response queue messages
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef _APM_MSG_RSP_HANDLER_H_
#define _APM_MSG_RSP_HANDLER_H_

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */

#include "apm_internal.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/* -----------------------------------------------------------------------
** Function declarations
** ----------------------------------------------------------------------- */

ar_result_t apm_rsp_q_msg_handler(apm_t *apm_info_ptr, spf_msg_t *rsp_msg_ptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* #ifndef _APM_MSG_RSP_HANDLER_H_ */
