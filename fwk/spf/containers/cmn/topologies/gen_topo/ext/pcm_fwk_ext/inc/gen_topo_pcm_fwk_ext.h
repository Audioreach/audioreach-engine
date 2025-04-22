#ifndef GEN_TOPO_PCM_FWK_H
#define GEN_TOPO_PCM_FWK_H
/**
 * \file gen_topo_pcm_fwk_ext.h
 * \brief
 *     This file contains utility functions FWK_EXTN_PARAM_ID_CONTAINER_PROC_DELAY
 *  
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "topo_utils.h"
#include "ar_defs.h"

// clang-format off

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

typedef struct gen_topo_t gen_topo_t;
typedef struct gen_topo_output_port_t gen_topo_output_port_t;
typedef struct gen_topo_module_t gen_topo_module_t;
typedef struct topo_media_fmt_t topo_media_fmt_t;


void gen_topo_capi_get_fwk_ext_media_fmt(gen_topo_t *            topo_ptr,
                                         gen_topo_module_t *     module_ptr,
                                         bool_t                  is_input_mf,
                                         uint32_t                port_index,
                                         gen_topo_output_port_t *out_port_ptr,
                                         topo_media_fmt_t *      fmt_ptr);

void gen_topo_capi_set_fwk_extn_media_fmt(gen_topo_t *       topo_ptr,
                                          gen_topo_module_t *module_ptr,
                                          bool_t             is_input_mf,
                                          uint16_t           port_index,
                                          topo_media_fmt_t * media_fmt_ptr);


#if defined(__cplusplus)
}
#endif // __cplusplus

#endif /* GEN_TOPO_PCM_FWK_H */
