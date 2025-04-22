/**
 * \file posal_thread_profiling.h
 * \brief 
 *  	 This file contains PUBLIC utilities for thread profiling.
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef _POSAL_THREAD_PROFILING_H_
#define _POSAL_THREAD_PROFILING_H_


#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#include "ar_error_codes.h"
#include "posal_types.h"


ar_result_t posal_thread_profiling_get_stack_info(uint32_t tid, uint32_t* current_stack_usage_ptr, uint32_t* stack_size_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _POSAL_THREAD_PROFILING_H_
