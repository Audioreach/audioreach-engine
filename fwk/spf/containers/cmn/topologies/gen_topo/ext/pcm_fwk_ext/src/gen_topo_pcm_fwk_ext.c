/**
 * \file gen_topo_pcm_fwk_ext.c
 *
 * \brief
 *
 *     Implementation of path delay aspects of topology interface functions.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_topo.h"
#include "gen_topo_capi.h"

void gen_topo_capi_get_fwk_ext_media_fmt(gen_topo_t *            topo_ptr,
                                         gen_topo_module_t *     module_ptr,
                                         bool_t                  is_input_mf,
                                         uint32_t                port_index,
                                         gen_topo_output_port_t *out_port_ptr,
                                         topo_media_fmt_t *      fmt_ptr)
{
   capi_err_t                             err_code     = CAPI_EOK;
   capi_t *                               capi_ptr     = (capi_t *)module_ptr->capi_ptr;
   fwk_extn_pcm_param_id_media_fmt_extn_t payload      = { 0 };
   gen_topo_common_port_t *               cmn_port_ptr = &out_port_ptr->common;

   capi_port_info_t port_info;
   capi_buf_t       buf;
   port_info.is_valid      = TRUE;
   port_info.is_input_port = is_input_mf;
   port_info.port_index    = port_index;
   buf.actual_data_len     = sizeof(fwk_extn_pcm_param_id_media_fmt_extn_t);
   buf.max_data_len        = sizeof(fwk_extn_pcm_param_id_media_fmt_extn_t);
   buf.data_ptr            = (int8_t *)&payload;

   err_code = capi_ptr->vtbl_ptr->get_param(capi_ptr, FWK_EXTN_PCM_PARAM_ID_MEDIA_FORMAT_EXTN, &port_info, &buf);
   if (CAPI_FAILED(err_code))
   {
      TOPO_MSG(topo_ptr->gu.log_id,
               DBG_ERROR_PRIO,
               "Getting Framework extn failed 0x%x",
               FWK_EXTN_PCM_PARAM_ID_MEDIA_FORMAT_EXTN);
   }
   else
   {
      // for endianness, no need to check else case like in COMPARE_SET_VALUE because, endianness is already assigned
      // with proper value in prev func
      if (IS_VALID_CAPI_VALUE(payload.endianness))
      {
         topo_endianness_t in_gen_topo_endianness =
            gen_topo_convert_public_endianness_to_gen_topo_endianness(payload.endianness);
         if (in_gen_topo_endianness != fmt_ptr->pcm.endianness)
         {
            cmn_port_ptr->flags.media_fmt_event = TRUE;
            fmt_ptr->pcm.endianness             = in_gen_topo_endianness;
         }
      }

      // if bits-per-sample was valid, then bit width is also valid. hence no need to check else case like in
      // COMPARE_SET_VALUE
      if (IS_VALID_CAPI_VALUE(payload.bit_width))
      {
         if (payload.bit_width != fmt_ptr->pcm.bit_width)
         {
            cmn_port_ptr->flags.media_fmt_event = TRUE;
            fmt_ptr->pcm.bit_width              = payload.bit_width;
         }
      }
   }
}

void gen_topo_capi_set_fwk_extn_media_fmt(gen_topo_t *       topo_ptr,
                                          gen_topo_module_t *module_ptr,
                                          bool_t             is_input_mf,
                                          uint16_t           port_index,
                                          topo_media_fmt_t * media_fmt_ptr)
{
   capi_err_t                             err_code = CAPI_EOK;
   capi_t *                               capi_ptr = (capi_t *)module_ptr->capi_ptr;
   fwk_extn_pcm_param_id_media_fmt_extn_t payload  = { 0 };

   capi_port_info_t port_info;
   capi_buf_t       buf;
   port_info.is_valid      = TRUE;
   port_info.is_input_port = is_input_mf;
   port_info.port_index    = port_index;
   payload.alignment       = PCM_LSB_ALIGNED;
   if (0 == media_fmt_ptr->pcm.endianness)
   {
      payload.endianness = CAPI_DATA_FORMAT_INVALID_VAL;
   }
   else
   {
      payload.endianness = gen_topo_convert_gen_topo_endianness_to_public_endianness(media_fmt_ptr->pcm.endianness);
   }

   if (0 == media_fmt_ptr->pcm.bit_width)
   {
      payload.bit_width = CAPI_DATA_FORMAT_INVALID_VAL;
   }
   else
   {
      payload.bit_width = media_fmt_ptr->pcm.bit_width;
   }
   buf.actual_data_len = sizeof(fwk_extn_pcm_param_id_media_fmt_extn_t);
   buf.max_data_len    = sizeof(fwk_extn_pcm_param_id_media_fmt_extn_t);
   buf.data_ptr        = (int8_t *)&payload;

   err_code = capi_ptr->vtbl_ptr->set_param(capi_ptr, FWK_EXTN_PCM_PARAM_ID_MEDIA_FORMAT_EXTN, &port_info, &buf);
   if (CAPI_FAILED(err_code))
   {
      TOPO_MSG(topo_ptr->gu.log_id,
               DBG_ERROR_PRIO,
               "Setting Framework extn failed 0x%x",
               FWK_EXTN_PCM_PARAM_ID_MEDIA_FORMAT_EXTN);
   }
}
