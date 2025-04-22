/**========================================================================
 @file hw_rs_lib.h

 @brief This file contains API's for allocating static variables needed for hw resampler

/*=========================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
========================================================================= */

#ifndef _HW_RS_LIB_H_
#define _HW_RS_LIB_H_

#include "ar_error_codes.h"
#include "posal.h"


#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

// Mutex Info Structure
typedef struct hw_resampler_mutex_t
{
   posal_mutex_t       mutex_lock; // Mutex lock for Hw Resampler api call.
} hw_resampler_mutex_t;

ar_result_t hw_rs_lib_static_init();

void hw_rs_lib_init(posal_atomic_word_t **phwrs_resource_state,
					posal_atomic_word_t **estimated_time_of_all_suspended_session,
					hw_resampler_mutex_t **mutex_info);
					
ar_result_t hw_rs_lib_static_deinit();

#ifdef __cplusplus
}
#endif //__cplusplus

#endif /* _HW_RS_LIB_H_ */
