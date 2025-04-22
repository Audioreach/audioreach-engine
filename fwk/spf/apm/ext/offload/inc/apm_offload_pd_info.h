#ifndef _APM_OFFLOAD_PD_INFO_H_
#define _APM_OFFLOAD_PD_INFO_H_

/**
 * \file apm_offload_pd_info.h
 *
 * \brief
 *     This file declares utility functions to to get and validate sat pd_id
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/
#include "ar_error_codes.h"
#include "posal_types.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

typedef struct apm_offload_utils_sat_pd_info_t
{
   uint32_t master_proc_domain;
   /**< Proc domain id of the master */

   uint32_t num_proc_domain_ids;
   /**< Number of satellite proc domain ids */

   uint32_t *proc_domain_list;
   /**< List of satellite proc domain ids */
} apm_offload_utils_sat_pd_info_t;
/*----------------------------------------------------------------------------
 * Type Declarations
 * -------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 * Function Declarations and Documentation
 * -------------------------------------------------------------------------*/

ar_result_t apm_offload_utils_get_sat_proc_domain_list(apm_offload_utils_sat_pd_info_t **sat_pd_info_pptr);

bool_t apm_offload_utils_is_valid_sat_pd(uint32_t sat_proc_domain_id);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _APM_OFFLOAD_PD_INFO_H_
