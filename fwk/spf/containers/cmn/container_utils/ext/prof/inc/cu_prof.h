#ifndef CU_PROF_H
#define CU_PROF_H

/**
 * \file cu_prof.h
 *  
 * \brief
 *  
 *     Common container framework code.
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "posal.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus
ar_result_t cu_cntr_get_prof_info(cu_base_t *base_ptr, int8_t *param_payload_ptr, uint32_t *param_size_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef CU_CTRL_PORT_H
