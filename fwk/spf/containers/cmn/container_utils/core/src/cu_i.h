#ifndef CU_I_H
#define CU_I_H
/**
 * \file cu_i.h
 * \brief
 *     This file contains internal structures and declarations for the container utility.
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spf_svc_utils.h"
#include "spf_svc_calib.h"
#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#include "container_utils.h"

#include "graph_utils.h"
#include "posal_power_mgr.h"
#include "capi.h"
#include "capi_mm_error_code_converter.h"
#include "posal.h"
#include "posal_internal_inline.h"
#include "spf_macros.h"
#include "shared_lib_api.h"
#include "gpr_packet.h"
#include "gpr_api_inline.h"
#include "ar_guids.h"

ar_result_t cu_workloop_entry(void *instance_ptr);
ar_result_t cu_workloop(cu_base_t *me_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef CU_I_H
