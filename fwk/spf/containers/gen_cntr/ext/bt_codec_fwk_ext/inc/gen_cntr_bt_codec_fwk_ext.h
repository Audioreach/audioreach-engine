#ifndef GEN_CNTR_BT_CODEC_FWK_EXT_H
#define GEN_CNTR_BT_CODEC_FWK_EXT_H
/**
 * \file gen_cntr_bt_codec_fwk_ext.h
 * \brief
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_cntr.h"
#include "gen_cntr_cmn_utils.h"
#include "gen_topo.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

typedef struct gen_topo_module_t gen_topo_module_t;
typedef struct capi_event_info_t event_info_ptr;

ar_result_t gen_cntr_handle_bt_codec_ext_event(gen_topo_module_t *module_ptr, capi_event_info_t *event_info_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef GEN_CNTR_BT_CODEC_FWK_EXT_H
