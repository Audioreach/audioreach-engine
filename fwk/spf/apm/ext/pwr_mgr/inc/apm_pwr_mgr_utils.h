#ifndef _APM_PWR_MGR_UTILS_H_
#define _APM_PWR_MGR_UTILS_H_

/**
 * \file apm_pwr_mgr_utils.h
 *
 * \brief
 *     This file contains declarations for APM Power Manager utility functions.
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

typedef struct apm_pwr_mgr_utils_vtable
{
   ar_result_t (*apm_pwr_mgr_vote_fptr)(apm_t *apm_info_ptr);

   ar_result_t (*apm_pwr_mgr_devote_fptr)(apm_t *apm_info_ptr);

} apm_pwr_mgr_utils_vtable_t;

/**------------------------------------------------------------------------------
 *  Function Declaration
 *----------------------------------------------------------------------------*/

ar_result_t apm_pwr_mgr_utils_init(apm_t *apm_info_ptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /** #ifdef _APM_PWR_MGR_UTILS_H_ */
