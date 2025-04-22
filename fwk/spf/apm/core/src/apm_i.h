#ifndef _APM_I_H_
#define _APM_I_H_

/*==============================================================================
@file apm_i.h

@brief
   This file contains private declarations of the APM Module

================================================================================
@copyright
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/

#include "posal.h"
#include "spf_cmn_if.h"
#include "apm_graph_db.h"
#include "gpr_api_inline.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

#define APM_INTERNAL_STATIC_HEAP_ID (MODIFY_STATIC_MODULE_HEAP_ID_FOR_MEM_TRACKING(APM_MODULE_INSTANCE_ID, POSAL_HEAP_DEFAULT))

/****************************************************************************
 * Structure Forward Declaration                                            *
 ****************************************************************************/

typedef struct apm_t                   apm_t;
typedef struct apm_cmd_ctrl_t          apm_cmd_ctrl_t;
typedef struct apm_deferred_cmd_list_t apm_deferred_cmd_list_t;
typedef struct apm_ext_utils_t         apm_ext_utils_t;

/****************************************************************************
 * Function Declaration                                                     *
 ****************************************************************************/
apm_ext_utils_t *apm_get_ext_utils_ptr();

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* _APM_I_H_ */
