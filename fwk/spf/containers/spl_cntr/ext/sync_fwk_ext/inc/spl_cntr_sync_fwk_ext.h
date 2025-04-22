#ifndef SPL_CNTR_SYNC_FWK_H
#define SPL_CNTR_SYNC_FWK_H
/**
 * // clang-format off
 * \file spl_cntr_sync_fwk_ext.h
 * \brief
 *  This file contains utility functions for FWK_EXTN_SYNC
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */
// clang-format on

#include "spl_cntr_i.h"
#include "spl_topo.h"

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

typedef struct spl_cntr_t spl_cntr_t;
typedef struct spl_cntr_ext_in_port_t spl_cntr_ext_in_port_t;

ar_result_t spl_cntr_fwk_extn_sync_handle_toggle_threshold_buffering_event(
   spl_cntr_t *                      me_ptr,
   spl_topo_module_t *               module_ptr,
   capi_buf_t *                      payload_ptr,
   capi_event_data_to_dsp_service_t *dsp_event_ptr);

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif /* SPL_CNTR_SYNC_FWK_H */
