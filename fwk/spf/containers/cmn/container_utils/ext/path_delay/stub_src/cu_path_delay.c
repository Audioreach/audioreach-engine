/**
 * \file cu_path_delay.c
 * \brief
 *     This file contains container utility functions for path delay handling
 *  
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "cu_i.h"
#include "apm.h"
#include "capi_intf_extn_path_delay.h"

ar_result_t cu_handle_event_to_dsp_service_topo_cb_for_path_delay(cu_base_t *        cu_ptr,
                                                                  gu_module_t *      module_ptr,
                                                                  capi_event_info_t *event_info_ptr)
{
   return AR_EUNSUPPORTED;
}

uint32_t cu_aggregate_ext_out_port_delay(cu_base_t *base_ptr, gu_ext_out_port_t *gu_ext_out_port_ptr)
{
   return 0;
}

/**
 * APM get param for CNTR_PARAM_ID_PATH_DELAY_CFG
 */
ar_result_t cu_path_delay_cfg(cu_base_t *base_ptr, int8_t *param_payload_ptr, uint32_t *param_size_ptr)
{
   ar_result_t result = AR_EOK;

   return result;
}

/**
 * APM SET param for CNTR_PARAM_ID_CFG_SRC_MOD_DELAY_LIST
 *
 * here source module means the source of event SPF_EVT_TO_APM_FOR_PATH_DELAY
 * not necessarily the one with zero inputs.
 */
ar_result_t cu_cfg_src_mod_delay_list(cu_base_t *base_ptr, int8_t *param_payload_ptr, uint32_t *param_size_ptr)
{
   ar_result_t result = AR_EOK;

   return result;
}

/**
 * APM Set param for CNTR_PARAM_ID_DESTROY_SRC_MOD_DELAY_LIST
 *
 * here source module means the source of event SPF_EVT_TO_APM_FOR_PATH_DELAY
 * not necessarily the one with zero inputs.
 */
ar_result_t cu_destroy_src_mod_delay_list(cu_base_t *base_ptr, int8_t *param_payload_ptr, uint32_t *param_size_ptr)
{
   ar_result_t result = AR_EOK;

   return result;
}

/**
 * path-id = 0 => all paths
 *
 * used for destroy or update
 */
ar_result_t cu_operate_on_delay_paths(cu_base_t *base_ptr, uint32_t path_id, cu_path_delay_op_t op)
{
   ar_result_t result = AR_EOK;

   return result;
}

ar_result_t cu_update_path_delay(cu_base_t *base_ptr, uint32_t path_id)
{
   return AR_EOK;
}
/**
 * called by APM through set param: CNTR_PARAM_ID_PATH_DESTROY
 */
ar_result_t cu_destroy_delay_path_cfg(cu_base_t *base_ptr, int8_t *param_payload_ptr, uint32_t *param_size_ptr)
{
   ar_result_t result = AR_EOK;

   return result;
}
