#ifndef GEN_TOPO_CAPI_UTIL_H
#define GEN_TOPO_CAPI_UTIL_H
/**
 * \file gen_topo_capi.h
 * \brief
 *     This file contains functions for common topology capi v2
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus
#include "gen_topo.h"

/**
 * returns true of x is not 0 and -1
 */
#define IS_VALID_CAPI_VALUE(x) ((CAPI_DATA_FORMAT_INVALID_VAL != x) && (0 != x))

#define CAPI_INPUT_PORT 0x0001
#define CAPI_OUTPUT_PORT 0x0000

typedef struct gen_topo_deint_raw_fmt_t
{
   capi_deinterleaved_raw_compressed_data_format_t deint_raw;
   capi_channel_mask_t                             ch_mask[TOPO_MAX_NUM_BUFS_DEINT_RAW_COMPR];
} gen_topo_deint_raw_fmt_t;

typedef struct gen_topo_capi_media_fmt_t
{
   capi_set_get_media_format_t main;
   union
   {
      capi_standard_data_format_t       std;
      capi_raw_compressed_data_format_t raw_fmt;
      // raw payload follows raw_fmt
      gen_topo_deint_raw_fmt_t deint_raw;
   };
} gen_topo_capi_media_fmt_t;

typedef struct gen_topo_capi_std_fmt_v2_t
{
   capi_standard_data_format_v2_t fmt;
   uint16_t                       channel_type[CAPI_MAX_CHANNELS_V2];
} gen_topo_capi_std_fmt_v2_t;

typedef struct gen_topo_capi_media_fmt_v2_t
{
   capi_set_get_media_format_t main;
   union
   {
      gen_topo_capi_std_fmt_v2_t        std;
      capi_raw_compressed_data_format_t raw_fmt;
      // raw payload follows raw_fmt
      gen_topo_deint_raw_fmt_t deint_raw;
   };
} gen_topo_capi_media_fmt_v2_t;

ar_result_t gen_topo_capi_get_port_thresh(uint32_t  module_instance_id,
                                          uint32_t  log_id,
                                          capi_t *  capi_ptr,
                                          bool_t    is_input,
                                          uint16_t  port_index,
                                          uint32_t *thresh);

ar_result_t gen_topo_capi_set_media_fmt(gen_topo_t *       topo_ptr,
                                        gen_topo_module_t *module_ptr,
                                        topo_media_fmt_t * media_fmt_ptr,
                                        bool_t             is_input_mf,
                                        uint16_t           port_index);

ar_result_t gen_topo_capi_algorithmic_reset(uint32_t log_id,
                                            capi_t * capi_ptr,
                                            bool_t   is_port_valid,
                                            bool_t   is_input,
                                            uint16_t port_index);

ar_result_t gen_topo_capi_get_param(uint32_t  log_id,
                                    capi_t *  capi_ptr,
                                    uint32_t  param_id,
                                    int8_t *  payload,
                                    uint32_t *size_ptr);

ar_result_t gen_topo_capi_set_param(uint32_t log_id,
                                    capi_t * capi_ptr,
                                    uint32_t param_id,
                                    int8_t * payload,
                                    uint32_t size);

ar_result_t gen_topo_capi_set_persistence_prop(uint32_t            log_id,
                                               gen_topo_module_t * module_ptr,
                                               uint32_t            param_id,
                                               bool_t              is_deregister,
                                               spf_cfg_data_type_t cfg_type);

ar_result_t gen_topo_capi_create_from_amdb(gen_topo_module_t *    module_ptr,
                                           gen_topo_t *           topo_ptr,
                                           void *                 amdb_handle,
                                           POSAL_HEAP_ID          heap_id,
                                           gen_topo_graph_init_t *graph_init_ptr);

ar_result_t gen_topo_capi_get_required_fmwk_extensions(void *           topo_ctx_ptr,
                                                       void *           module_ctx_ptr,
                                                       void *           amdb_handle,
                                                       capi_proplist_t *init_proplist_ptr);

void gen_topo_get_dl_info(uint32_t log_id, void *amdb_handle, bool_t *is_dl, uint32_t **start_addr, uint32_t *so_size);

ar_result_t gen_topo_validate_client_pcm_media_format(const payload_media_fmt_pcm_t *pcm_fmt_ptr);
ar_result_t gen_topo_validate_client_pcm_float_media_format(const payload_media_fmt_pcm_t *pcm_fmt_ptr);
ar_result_t gen_topo_validate_client_pcm_output_cfg(const payload_pcm_output_format_cfg_t *pcm_cfg_ptr);

ar_result_t gen_topo_capi_get_out_media_fmt(gen_topo_t *            topo_ptr,
                                            gen_topo_module_t *     module_ptr,
                                            gen_topo_output_port_t *out_port_ptr);

capi_err_t gen_topo_handle_output_media_format_event(void *             ctxt_ptr,
                                                     void *             module_ctxt_ptr,
                                                     capi_event_info_t *event_info_ptr,
                                                     bool_t             is_std_fmt_v2,
                                                     bool_t             is_pending_data_valid);

/* Returns Max stack size required for all the opened modules */
ar_result_t gen_topo_get_aggregated_capi_stack_size(gen_topo_t *topo_ptr, uint32_t *max_stack_size);

/**------------------------------ gen_topo_capi ------------------------------*/

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif /* GEN_TOPO_CAPI_UTIL_H */
