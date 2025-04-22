/**
 * \file spl_cntr_sync_fwk_ext.c
 *
 * \brief
 *
 *     Implementation of stub utilities for FWK_EXTN_SYNC in spl cntr
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spl_cntr_sync_fwk_ext.h"

ar_result_t spl_cntr_fwk_extn_sync_handle_toggle_threshold_buffering_event(
   spl_cntr_t *                      me_ptr,
   spl_topo_module_t *               module_ptr,
   capi_buf_t *                      payload_ptr,
   capi_event_data_to_dsp_service_t *dsp_event_ptr)
{
   return AR_EOK;
}

