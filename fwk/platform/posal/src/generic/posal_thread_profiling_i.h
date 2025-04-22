/**
 * \file posal_thread_profiling_i.h
 * \brief
 *  	 This file contains utilities for thread profiling.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */


/*===========================================================================
NOTE: The @brief description above does not appear in the PDF.
      The weakgroup description below displays in the PDF.
===========================================================================*/

/** @weakgroup weakf_posal_thread_intro
Threads must be joined to avoid memory leaks. The following thread functions
are used to create and destroy threads, and to change thread priorities.
 - posal_thread_profile_at_join()
 - posal_thread_profile_at_exit()
 - posal_thread_profile_at_launch()
 */

#ifndef POSAL_THREAD_PROFILING_I_H_
#define POSAL_THREAD_PROFILING_I_H_

#include "posal.h"

#ifdef SIM
/* Enable this macro for thread profiling*/
#define POSAL_ENABLE_THREAD_PROFILING  1
#endif //SIM

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus


ar_result_t    posal_thread_profile_at_join( void *thread_profile_obj_ptr, int *pStatus);

ar_result_t    posal_thread_profile_at_exit( void *thread_profile_obj_ptr, uint32_t tid, int32_t exit_status );

ar_result_t    posal_thread_profile_at_launch(  void **thread_list_ptr_ptr, char* stack_ptr, uint32_t stack_size,
										  char *threadname, POSAL_HEAP_ID heap_id);

void posal_thread_profile_set_tid(   void **thread_list_ptr_ptr,
						uint32_t tid);

#ifdef __cplusplus
}
#endif //__cplusplus



#endif /* POSAL_THREAD_PROFILING_I_H_ */
