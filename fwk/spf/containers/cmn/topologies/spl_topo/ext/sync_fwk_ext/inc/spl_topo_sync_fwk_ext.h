#ifndef SPL_TOPO_SYNC_FWK_H
#define SPL_TOPO_SYNC_FWK_H
/**
 * \file spl_topo_sync_fwk_ext.h
 * \brief
 *     This file contains function definitions for FWK_EXTN_SYNC at the topo layer

 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "topo_utils.h"
#include "ar_defs.h"

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

typedef struct spl_topo_t             spl_topo_t;
typedef struct spl_topo_module_t      spl_topo_module_t;

ar_result_t spl_topo_handle_data_port_activity_sync_event_cb(spl_topo_t *topo_ptr, spl_topo_module_t *module_ptr, capi_buf_t *payload_ptr);

ar_result_t spl_topo_sync_handle_inactive_port_events(spl_topo_t *topo_ptr, bool_t *is_event_handled_ptr);

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif //SPL_TOPO_SYNC_FWK_H
