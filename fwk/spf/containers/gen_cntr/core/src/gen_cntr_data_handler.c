/**
 * \file gen_cntr_data_handler_island.c
 * \brief
 *     This file contains functions for data handling of GEN_CNTR service
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_cntr_i.h"

void gen_cntr_handle_st_overrun_at_post_process(gen_cntr_t *me_ptr, gen_cntr_ext_out_port_t *ext_out_port_ptr)
{
   gen_topo_output_port_t *out_port_ptr = (gen_topo_output_port_t *)ext_out_port_ptr->gu.int_out_port_ptr;
   ext_out_port_ptr->err_msg_overrun_err_count++;

   if (gen_cntr_check_for_err_print(&me_ptr->topo))
   {
      GEN_CNTR_MSG_ISLAND(me_ptr->topo.gu.log_id,
                          DBG_ERROR_PRIO,
                          "postprocess: Ext output Port 0x%lx of Module 0x%lX, dropping data %lu per ch"
                          "md_list0x%p. No of overrun after last print: %u.",
                          out_port_ptr->gu.cmn.id,
                          out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                          out_port_ptr->common.bufs_ptr[0].actual_data_len,
                          out_port_ptr->common.sdata.metadata_list_ptr,
                          ext_out_port_ptr->err_msg_overrun_err_count);
      ext_out_port_ptr->err_msg_overrun_err_count = 0;
   }

   // drop data if buffer is not present at the output [this helps GEN_CNTR, but in GEN_CNTR buffer will always be
   // there]
   gen_topo_set_all_bufs_len_to_zero(&out_port_ptr->common);

   // also drop metadata if any
   if (out_port_ptr->common.sdata.metadata_list_ptr)
   {
      // If md lib is markep nlpi, no need to exit island here since we should be in nlpi
      // by now due to exiting to propagate md in process context
      gen_topo_destroy_all_metadata(me_ptr->topo.gu.log_id,
                                    (void *)out_port_ptr->gu.cmn.module_ptr,
                                    &out_port_ptr->common.sdata.metadata_list_ptr,
                                    TRUE /*is_dropped*/);
   }
}

ar_result_t gen_cntr_mv_data_from_topo_to_ext_out_buf_npli_(gen_cntr_t *             me_ptr,
                                                            gen_cntr_ext_out_port_t *ext_out_port_ptr,
                                                            uint32_t                 total_len,
                                                            uint32_t                 max_empty_space)
{
   gen_topo_output_port_t *    out_port_ptr = (gen_topo_output_port_t *)ext_out_port_ptr->gu.int_out_port_ptr;
   topo_media_fmt_t *          module_out_media_fmt_ptr = out_port_ptr->common.media_fmt_ptr;

   topo_interleaving_t interleaving   = module_out_media_fmt_ptr->pcm.interleaving;
   uint32_t            num_channels   = module_out_media_fmt_ptr->pcm.num_channels;
   uint32_t            src_actual_len = total_len;
   uint32_t            dst_actual_len = ext_out_port_ptr->buf.actual_data_len;
   int8_t *            src_ptr        = out_port_ptr->common.bufs_ptr[0].data_ptr;
   int8_t *            dst_ptr        = ext_out_port_ptr->buf.data_ptr;
   uint32_t            num_out_bytes  = MIN(src_actual_len, max_empty_space);

   // always assume data starts from beginning in src_ptr. when ext out buf is not large enough to hold,
   // what's left of input has to be memmoved to beginning.

   if (SPF_IS_PCM_DATA_FORMAT(module_out_media_fmt_ptr->data_format) && (TOPO_DEINTERLEAVED_PACKED == interleaving))
   {
      // only one buf exists.
      GEN_CNTR_SIM_DEBUG(me_ptr->topo.gu.log_id, "Warning: postprocess: deint-packed handling. this is suboptimal");

      uint32_t num_out_bytes_per_ch          = capi_cmn_divide(num_out_bytes, num_channels);
      uint32_t src_channel_spacing_bytes     = capi_cmn_divide(src_actual_len, num_channels);
      uint32_t new_dst_channel_spacing_bytes = capi_cmn_divide((dst_actual_len + num_out_bytes), num_channels);
      uint32_t old_dst_channel_spacing_bytes = capi_cmn_divide(dst_actual_len, num_channels);

      // move bytes already in the ext out away (deint packed)
      if (ext_out_port_ptr->buf.actual_data_len)
      {
         for (uint32_t ch = num_channels; ch > 0; ch--)
         {
            TOPO_MEMSMOV_NO_RET(dst_ptr + (ch - 1) * new_dst_channel_spacing_bytes,
                                old_dst_channel_spacing_bytes,
                                dst_ptr + (ch - 1) * old_dst_channel_spacing_bytes,
                                old_dst_channel_spacing_bytes,
                                me_ptr->topo.gu.log_id,
                                "POST: (0x%lX, 0x%lX)",
                                out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                                out_port_ptr->gu.cmn.id);
         }
      }

      for (uint32_t ch = 0; ch < num_channels; ch++)
      {
         TOPO_MEMSCPY_NO_RET(dst_ptr + old_dst_channel_spacing_bytes + ch * new_dst_channel_spacing_bytes,
                             num_out_bytes_per_ch,
                             src_ptr + ch * src_channel_spacing_bytes,
                             num_out_bytes_per_ch,
                             me_ptr->topo.gu.log_id,
                             "POST: (0x%lX, 0x%lX)",
                             out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                             out_port_ptr->gu.cmn.id);
      }

      out_port_ptr->common.bufs_ptr[0].actual_data_len -= num_out_bytes;
      ext_out_port_ptr->buf.actual_data_len += num_out_bytes;

      if (out_port_ptr->common.bufs_ptr[0].actual_data_len) // if data is left, move it up
      {
         uint32_t new_ch_spacing = capi_cmn_divide(out_port_ptr->common.bufs_ptr[0].actual_data_len , num_channels);
         for (uint32_t ch = 0; ch < num_channels; ch++)
         {
            TOPO_MEMSMOV_NO_RET(src_ptr + ch * new_ch_spacing,
                                new_ch_spacing,
                                src_ptr + num_out_bytes_per_ch + src_channel_spacing_bytes * ch,
                                new_ch_spacing,
                                me_ptr->topo.gu.log_id,
                                "POST: (0x%lX, 0x%lX)",
                                out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                                out_port_ptr->gu.cmn.id);
         }
      }

      (void)src_channel_spacing_bytes;
      (void)new_dst_channel_spacing_bytes;
      (void)old_dst_channel_spacing_bytes;
   }
   else if (out_port_ptr->common.flags.is_pcm_unpacked)
   {
      // unpacked v1/v2 cases are handled within island, before calling this function. (check caller context)
   }
   else if (SPF_DEINTERLEAVED_RAW_COMPRESSED == out_port_ptr->common.media_fmt_ptr->data_format)
   {
	  /* For deinterleaved raw compressed case we use the spf message data buf v2 and hence we later populate bufs_num and bufs_ptr instead of bufs */
      for (uint32_t b = 0; b < out_port_ptr->common.sdata.bufs_num; b++)
      {
         uint32_t max_empty =
            ext_out_port_ptr->bufs_ptr[b].max_data_len - ext_out_port_ptr->bufs_ptr[b].actual_data_len;

         uint32_t num_output_bytes = MIN(out_port_ptr->common.bufs_ptr[b].actual_data_len, max_empty);

         TOPO_MEMSMOV_NO_RET(ext_out_port_ptr->bufs_ptr[b].data_ptr + ext_out_port_ptr->bufs_ptr[b].actual_data_len,
                             max_empty,
                             out_port_ptr->common.bufs_ptr[b].data_ptr,
                             num_output_bytes,
                             me_ptr->topo.gu.log_id,
                             "POST: (0x%lX, 0x%lX)",
                             out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                             out_port_ptr->gu.cmn.id);

         out_port_ptr->common.bufs_ptr[b].actual_data_len -= num_output_bytes;
         ext_out_port_ptr->bufs_ptr[b].actual_data_len += num_output_bytes;

         if (out_port_ptr->common.bufs_ptr[b].actual_data_len)
         {
            TOPO_MEMSMOV_NO_RET(out_port_ptr->common.bufs_ptr[b].data_ptr,
                                out_port_ptr->common.bufs_ptr[b].actual_data_len,
                                out_port_ptr->common.bufs_ptr[b].data_ptr + num_output_bytes,
                                out_port_ptr->common.bufs_ptr[b].actual_data_len,
                                me_ptr->topo.gu.log_id,
                                "POST: (0x%lX, 0x%lX)",
                                out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                                out_port_ptr->gu.cmn.id);
         }
      }
   }
   else
   {
      TOPO_MEMSCPY_NO_RET(dst_ptr + dst_actual_len,
                          max_empty_space,
                          src_ptr,
                          num_out_bytes,
                          me_ptr->topo.gu.log_id,
                          "POST: (0x%lX, 0x%lX)",
                          out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                          out_port_ptr->gu.cmn.id);

      ext_out_port_ptr->buf.actual_data_len += num_out_bytes;
      out_port_ptr->common.bufs_ptr[0].actual_data_len -= num_out_bytes;

      if (out_port_ptr->common.bufs_ptr[0].actual_data_len) // if data is left, move it up
      {
         TOPO_MEMSMOV_NO_RET(src_ptr,
                             out_port_ptr->common.bufs_ptr[0].actual_data_len,
                             src_ptr + num_out_bytes,
                             out_port_ptr->common.bufs_ptr[0].actual_data_len,
                             me_ptr->topo.gu.log_id,
                             "POST: (0x%lX, 0x%lX)",
                             out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                             out_port_ptr->gu.cmn.id);
      }
   }

   (void)src_ptr;
   (void)dst_ptr;

   return AR_EOK;
}


