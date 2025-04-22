#ifndef _APM_CLOSE_ALL_UTILS_H__
#define _APM_CLOSE_ALL_UTILS_H__

/**
 * \file apm_close_all_utils.h
 *  
 * \brief
 *     This file contains function declaration for APM utilities for APM_CMD_CLOSE_ALL handling
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

typedef struct apm_close_all_utils_vtable
{
   ar_result_t (*apm_cmd_close_all_seqncer_fptr)(apm_t *apm_info_ptr);
   ar_result_t (*apm_cmd_set_cfg_close_all_seq_fptr)(apm_t *apm_info_ptr);
}apm_close_all_utils_vtable_t;

/**------------------------------------------------------------------------------
 *  Function Declaration
 *----------------------------------------------------------------------------*/

ar_result_t apm_close_all_utils_init(apm_t *apm_info_ptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* _APM_CLOSE_ALL_UTILS_H__ */
