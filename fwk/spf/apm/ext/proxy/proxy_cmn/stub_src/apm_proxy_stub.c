/**
 * \file apm_proxy_stub.c
 *
 * \brief
 *
 *     This file contains APM proxy manager utility function stub implementation
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/****************************************************************************
 * INCLUDE HEADER FILES                                                     *
 ****************************************************************************/

#include "apm_graph_db.h"
#include "apm_msg_utils.h"
#include "apm_proxy_def.h"
#include "apm_internal.h"
#include "apm_proxy_utils.h"

/****************************************************************************
 * GLOBALS                                                                  *
 ****************************************************************************/

/****************************************************************************
 * Function Declarations
 ****************************************************************************/

ar_result_t apm_handle_proxy_mgr_cfg_params(apm_t *                  apm_info_ptr,
                                            apm_module_param_data_t *mod_data_ptr,
                                            bool_t                   use_sys_q)
{

   return AR_EOK;
}

ar_result_t apm_proxy_manager_response_handler(apm_t *apm_info_ptr, spf_msg_t *rsp_msg_ptr)
{

   return AR_EOK;
}

ar_result_t apm_free_proxy_mgr_from_id(apm_t *apm_info_ptr, uint32_t instance_id)
{
   return AR_EOK;
}


ar_result_t apm_move_proxy_to_active_list_by_id(apm_t *apm_info_ptr, uint32_t instance_id)
{
   return AR_EOK;
}