#ifndef _APM_SYS_UTIL_H__
#define _APM_SYS_UTIL_H__

/**
 * \file apm_sys.h
 *
 * \brief
 *     This file contains function declaration for APM utilities for sys util handling
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "apm_i.h"
#define APM_MAX_SYS_Q_ELEMENTS 8
#define APM_PREALLOC_SYS_Q_ELEMENTS 8
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/**------------------------------------------------------------------------------
 *  Structure Definition
 *----------------------------------------------------------------------------*/

typedef struct apm_sys_util_vtable_t
{
   ar_result_t (*apm_sys_util_process_fptr)(apm_t *apm_info_ptr);
   ar_result_t (*apm_sys_util_register_fptr)(uint32_t num_proc_domain_ids, uint32_t *proc_domain_list);
   void *(*apm_sys_util_get_sys_q_handle_fptr)();
   bool_t (*apm_sys_util_is_pd_info_available_fptr)();
} apm_sys_util_vtable_t;

/**------------------------------------------------------------------------------
 *  Function Declaration
 *----------------------------------------------------------------------------*/

ar_result_t apm_sys_util_init(apm_t *apm_info_ptr);

ar_result_t apm_sys_util_deinit();

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* _APM_SYS_UTIL_H__ */
