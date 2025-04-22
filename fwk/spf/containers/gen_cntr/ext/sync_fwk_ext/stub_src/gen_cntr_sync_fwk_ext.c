/**
 * \file gen_cntr_sync_fwk_ext.c
 *
 * \brief
 *
 *     Implementation of stub utilities for FWK_EXTN_SYNC in gen cntr
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */


#include "gen_cntr_sync_fwk_ext.h"

ar_result_t gen_cntr_fwk_extn_sync_handle_toggle_threshold_buffering_event(
   gen_cntr_t *                      me_ptr,
   gen_topo_module_t *               module_ptr,
   capi_buf_t *                      payload_ptr,
   capi_event_data_to_dsp_service_t *dsp_event_ptr)
{
   return AR_EOK;
}

void gen_cntr_fwk_ext_sync_update_ext_in_tgp(gen_cntr_t *me_ptr)
{
  return;
}

bool_t gen_cntr_fwk_ext_sync_requires_data(gen_cntr_t *me_ptr, gen_topo_input_port_t *in_port_ptr)
{
	return FALSE;
}