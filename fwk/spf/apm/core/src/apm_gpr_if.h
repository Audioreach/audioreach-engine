/**
 * \file apm_gpr_if.h
 * \brief
 *    This file contains APM GPR handler Implementation
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef _APM_GPR_IF_H_
#define _APM_GPR_IF_H_

/*==========================================================================
  Include files
  ========================================================================== */

#include "gpr_api_inline.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/**
 * GPR callback function for sending cmd to APM.
 *
 * @param[in] APR cmd packet
 * @param[in] callback context pointer
 * @return return code
 */
uint32_t apm_gpr_call_back_f(gpr_packet_t *gpr_pkt_ptr, void *cb_ctxt_ptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* _APM_GPR_IF_H_ */
