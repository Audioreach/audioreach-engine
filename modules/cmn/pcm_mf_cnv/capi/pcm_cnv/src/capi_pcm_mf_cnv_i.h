/* =========================================================================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 * @file capi_pcm_mf_cnv_i.h
 */

#include "capi_pcm_mf_cnv_utils.h"

capi_vtbl_t *capi_pcm_mf_cnv_get_vtable();

capi_err_t capi_pcm_mf_cnv_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[]);

capi_err_t capi_pcm_mf_cnv_end(capi_t *_pif);

capi_err_t capi_pcm_mf_cnv_set_param(capi_t *                _pif,
                                     uint32_t                param_id,
                                     const capi_port_info_t *port_info_ptr,
                                     capi_buf_t *            params_ptr);

capi_err_t capi_pcm_mf_cnv_get_param(capi_t *                _pif,
                                     uint32_t                param_id,
                                     const capi_port_info_t *port_info_ptr,
                                     capi_buf_t *            params_ptr);

capi_err_t capi_pcm_mf_cnv_set_properties(capi_t *me_ptr, capi_proplist_t *props_ptr);

capi_err_t capi_pcm_mf_cnv_get_properties(capi_t *me_ptr, capi_proplist_t *props_ptr);

capi_err_t capi_pcm_mf_cnv_check_buffers(capi_pcm_mf_cnv_t *me_ptr,
                                         capi_buf_t *       src_buf_ptr,
                                         capi_buf_t *       dest_buf_ptr,
                                         uint32_t           port,
                                         uint32_t           eos_flag);

capi_err_t capi_pcm_mf_cnv_check_scratch_buffer(capi_pcm_mf_cnv_t *me_ptr,
                                                capi_buf_t *       src_buf_ptr,
                                                capi_buf_t *       dest_buf_ptr,
                                                uint32_t           port);

static inline void pcm_to_capi_interleaved_with_native(capi_interleaving_t *capi_value,
                                                       int16_t              cfg_value,
                                                       capi_interleaving_t  inp_value)
{
   switch (cfg_value)
   {
      case PCM_INTERLEAVED:
      {
         *capi_value = CAPI_INTERLEAVED;
         break;
      }
      case PCM_DEINTERLEAVED_PACKED:
      {
         *capi_value = CAPI_DEINTERLEAVED_PACKED;
         break;
      }
      case PCM_DEINTERLEAVED_UNPACKED:
      {
         *capi_value = CAPI_DEINTERLEAVED_UNPACKED_V2;
         break;
      }
      case PARAM_VAL_NATIVE:
      {
         *capi_value = inp_value;
         break;
      }
      default:
         break;
   }
}
