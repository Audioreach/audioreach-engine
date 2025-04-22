#ifndef _APM_PARALLEL_CMD_UTILS_H__
#define _APM_PARALLEL_CMD_UTILS_H__

/**
 * \file apm_parallel_cmd_utils.h
 *
 * \brief
 *     This file contains utility function declaration for APM
 *     parallel command handling
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "apm_i.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/**------------------------------------------------------------------------------
 *  Structure Definition
 *----------------------------------------------------------------------------*/

typedef struct apm_parallel_cmd_utils_vtable
{
   ar_result_t (*apm_check_and_defer_cmd_processing_fptr)(apm_t *apm_info_ptr, bool_t *cmd_proc_deferred_ptr);

   ar_result_t (*apm_check_def_cmd_is_ready_to_process_fptr)(apm_t *apm_info_ptr);
   
   ar_result_t (*apm_update_deferred_gm_cmd_fptr)(apm_t *apm_info_ptr, apm_sub_graph_t *closed_sg_ptr);


} apm_parallel_cmd_utils_vtable_t;

/**------------------------------------------------------------------------------
 *  Function Declaration
 *----------------------------------------------------------------------------*/

ar_result_t apm_parallel_cmd_utils_init(apm_t *apm_info_ptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* _APM_PARALLEL_CMD_UTILS_H__ */
