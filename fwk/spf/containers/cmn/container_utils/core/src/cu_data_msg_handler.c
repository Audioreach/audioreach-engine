/**
 * \file cu_data_msg_handler.c
 * \brief
 *     This file contains container utility functions for data handling.
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "cu_i.h"
#include "wr_sh_mem_ep_api.h"
#include "wr_sh_mem_ep_ext_api.h"

/* =======================================================================
Public Function Definitions
========================================================================== */
ar_result_t cu_create_media_fmt_msg_for_downstream(cu_base_t *        base_ptr,
                                                   gu_ext_out_port_t *gu_ext_out_port_ptr,
                                                   spf_msg_t *        msg,
                                                   uint32_t           media_fmt_opcode)
{
   ar_result_t result      = AR_EOK;
   msg->payload_ptr        = NULL;
   uint32_t size = 0, out_port_thresh = 0;

   cu_ext_out_port_t *ext_out_port_ptr =
      (cu_ext_out_port_t *)((uint8_t *)gu_ext_out_port_ptr + base_ptr->ext_out_port_cu_offset);
   topo_media_fmt_t *out_actual_media_fmt = &ext_out_port_ptr->media_fmt;

   if (gu_ext_out_port_ptr->downstream_handle.spf_handle_ptr)
   {
      if (SPF_IS_PACKETIZED_OR_PCM(out_actual_media_fmt->data_format))
      {
         size = GET_SPF_STD_MEDIA_FMT_SIZE;
      }
      else if (SPF_RAW_COMPRESSED == out_actual_media_fmt->data_format)
      {
         size = GET_SPF_RAW_MEDIA_FMT_SIZE(out_actual_media_fmt->raw.buf_size - (TOPO_MIN_SIZE_OF_RAW_MEDIA_FMT));
      }
      else if (SPF_DEINTERLEAVED_RAW_COMPRESSED == out_actual_media_fmt->data_format)
      {
         size =
            GET_SPF_MSG_REQ_SIZE(sizeof(spf_msg_media_format_t) - sizeof(uint64_t)) + sizeof(topo_deint_raw_med_fmt_t);
      }
      else
      {
         CU_MSG(base_ptr->gu_ptr->log_id,
                DBG_ERROR_PRIO,
                "Data format not supported %lx",
                out_actual_media_fmt->data_format);
         return AR_EFAILED;
      }

      if (AR_DID_FAIL(spf_msg_create_msg(msg,
                                         &size,
                                         media_fmt_opcode,
                                         NULL,
                                         NULL,
                                         gu_ext_out_port_ptr->downstream_handle.spf_handle_ptr,
                                         base_ptr->heap_id)))
      {
         CU_MSG(base_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Fail to create media fmt message");
         return AR_EFAILED;
      }

      spf_msg_header_t *      header         = (spf_msg_header_t *)msg->payload_ptr;
      spf_msg_media_format_t *pCmdMsgPayload = (spf_msg_media_format_t *)(&header->payload_start);
      // pCmdMsgPayload->actual_size;
      pCmdMsgPayload->df     = out_actual_media_fmt->data_format;
      pCmdMsgPayload->fmt_id = out_actual_media_fmt->fmt_id;

      // Populate fields for upstream frame len depending on mf
      memset((void *)&pCmdMsgPayload->upstream_frame_len, 0, sizeof(spf_msg_frame_length_t));

      if (SPF_IS_PACKETIZED_OR_PCM(out_actual_media_fmt->data_format))
      {
         // For pcm we populate this based on icb frame len
         pCmdMsgPayload->upstream_frame_len.frame_len_samples = base_ptr->cntr_frame_len.frame_len_samples;
         pCmdMsgPayload->upstream_frame_len.frame_len_us      = base_ptr->cntr_frame_len.frame_len_us;
         pCmdMsgPayload->upstream_frame_len.sample_rate       = base_ptr->cntr_frame_len.sample_rate;

         spf_std_media_format_t *std_ptr = (spf_std_media_format_t *)&pCmdMsgPayload->payload_start;
         std_ptr->num_channels           = out_actual_media_fmt->pcm.num_channels;
         std_ptr->bits_per_sample        = out_actual_media_fmt->pcm.bits_per_sample;
         std_ptr->sample_rate            = out_actual_media_fmt->pcm.sample_rate;
         std_ptr->q_format               = out_actual_media_fmt->pcm.q_factor;
         std_ptr->interleaving           = (SPF_IS_PACKETIZED(out_actual_media_fmt->data_format) ||
                                            (TOPO_INTERLEAVED == out_actual_media_fmt->pcm.interleaving))
                                             ? SPF_INTERLEAVED
                                             : SPF_DEINTERLEAVED_PACKED;
         memscpy(std_ptr->channel_map,
                 sizeof(std_ptr->channel_map),
                 out_actual_media_fmt->pcm.chan_map,
                 out_actual_media_fmt->pcm.num_channels);

         // print an error for quicker debugging. For packetized format interleaved is used by default.
         if (SPF_IS_PCM_DATA_FORMAT(out_actual_media_fmt->data_format) &&
             ((TOPO_DEINTERLEAVED_PACKED != out_actual_media_fmt->pcm.interleaving) &&
              (TOPO_INTERLEAVED != out_actual_media_fmt->pcm.interleaving)))
         {
            CU_MSG(base_ptr->gu_ptr->log_id,
                   DBG_ERROR_PRIO,
                   "container output media format: must be interleaved/deinterleaved packed for PCM, but it is %u",
                   out_actual_media_fmt->pcm.interleaving);
         }

         TOPO_PRINT_PCM_MEDIA_FMT(base_ptr->gu_ptr->log_id, out_actual_media_fmt, "container output");
         CU_MSG(base_ptr->gu_ptr->log_id,
                DBG_HIGH_PRIO,
                "container output media format: Upstream max frame len (%lu samples, %lu Hz, %lu us)",
                pCmdMsgPayload->upstream_frame_len.frame_len_samples,
                pCmdMsgPayload->upstream_frame_len.sample_rate,
                pCmdMsgPayload->upstream_frame_len.frame_len_us);
      }
      else if (SPF_RAW_COMPRESSED == out_actual_media_fmt->data_format)
      {
         if (base_ptr->topo_vtbl_ptr->get_port_threshold)
         {
            out_port_thresh =
               base_ptr->topo_vtbl_ptr->get_port_threshold((void *)gu_ext_out_port_ptr->int_out_port_ptr);
         }

         pCmdMsgPayload->upstream_frame_len.frame_len_samples = base_ptr->cntr_frame_len.frame_len_samples;
         pCmdMsgPayload->upstream_frame_len.frame_len_us      = base_ptr->cntr_frame_len.frame_len_us;
         pCmdMsgPayload->upstream_frame_len.sample_rate       = base_ptr->cntr_frame_len.sample_rate;
         pCmdMsgPayload->upstream_frame_len.frame_len_bytes   = out_port_thresh;

         if (out_actual_media_fmt->raw.buf_ptr)
         {
            int8_t *raw_dst_ptr         = (int8_t *)&pCmdMsgPayload->payload_start;
            pCmdMsgPayload->actual_size = out_actual_media_fmt->raw.buf_size - (TOPO_MIN_SIZE_OF_RAW_MEDIA_FMT);
            int8_t * raw_src_ptr        = (int8_t *)out_actual_media_fmt->raw.buf_ptr + TOPO_MIN_SIZE_OF_RAW_MEDIA_FMT;
            uint32_t size_to_copy       = out_actual_media_fmt->raw.buf_size - (TOPO_MIN_SIZE_OF_RAW_MEDIA_FMT);
            memscpy(raw_dst_ptr, size_to_copy, raw_src_ptr, size_to_copy);
         }
         else
         {
            pCmdMsgPayload->actual_size = 0; // enc path just has the df and format
         }
         tu_capi_destroy_raw_compr_med_fmt(&out_actual_media_fmt->raw);
         CU_MSG(base_ptr->gu_ptr->log_id,
                DBG_HIGH_PRIO,
                "container output media format: Raw compressed. format-id 0x%lx, upstream max len in bytes %d",
                out_actual_media_fmt->fmt_id,
                pCmdMsgPayload->upstream_frame_len.frame_len_bytes);
      }
      else if (SPF_DEINTERLEAVED_RAW_COMPRESSED == out_actual_media_fmt->data_format)
      {
         if (base_ptr->topo_vtbl_ptr->get_port_threshold)
         {
            out_port_thresh =
               base_ptr->topo_vtbl_ptr->get_port_threshold((void *)gu_ext_out_port_ptr->int_out_port_ptr);
         }

         pCmdMsgPayload->upstream_frame_len.frame_len_samples = base_ptr->cntr_frame_len.frame_len_samples;
         pCmdMsgPayload->upstream_frame_len.frame_len_us      = base_ptr->cntr_frame_len.frame_len_us;
         pCmdMsgPayload->upstream_frame_len.sample_rate       = base_ptr->cntr_frame_len.sample_rate;
         pCmdMsgPayload->upstream_frame_len.frame_len_bytes   = out_port_thresh;

         topo_deint_raw_med_fmt_t *raw_dst_ptr = (topo_deint_raw_med_fmt_t *)&pCmdMsgPayload->payload_start;
         pCmdMsgPayload->actual_size           = sizeof(topo_deint_raw_med_fmt_t);
         raw_dst_ptr->bufs_num                 = out_actual_media_fmt->deint_raw.bufs_num;
         for (uint32_t i = 0; i < out_actual_media_fmt->deint_raw.bufs_num; i++)
         {
            raw_dst_ptr->ch_mask[i].channel_mask_lsw = out_actual_media_fmt->deint_raw.ch_mask[i].channel_mask_lsw;
            raw_dst_ptr->ch_mask[i].channel_mask_msw = out_actual_media_fmt->deint_raw.ch_mask[i].channel_mask_msw;
         }

         CU_MSG(base_ptr->gu_ptr->log_id,
                DBG_HIGH_PRIO,
                "container output media format: Deinterleaved raw compressed. format-id 0x%lx, num bufs %d, upstream "
                "max len in bytes %d",
                out_actual_media_fmt->fmt_id,
                raw_dst_ptr->bufs_num,
                pCmdMsgPayload->upstream_frame_len.frame_len_bytes);
      }
   }

   return result;
}

ar_result_t cu_send_media_fmt_update_to_downstream(cu_base_t *        base_ptr,
                                                   gu_ext_out_port_t *gu_ext_out_port_ptr,
                                                   spf_msg_t *        msg,
                                                   posal_queue_t *    q_ptr)
{
   ar_result_t result = AR_EOK;

   if (NULL == msg)
   {
      return result;
   }

   if (gu_ext_out_port_ptr->downstream_handle.spf_handle_ptr)
   {
      result = posal_queue_push_back(q_ptr, (posal_queue_element_t *)msg);
      if (AR_DID_FAIL(result))
      {
         spf_msg_return_msg(msg);
      }

      CU_MSG(base_ptr->gu_ptr->log_id, DBG_MED_PRIO, "Sent media fmt update with result 0x%lx", result);
   }
   else
   {
      CU_MSG(base_ptr->gu_ptr->log_id, DBG_MED_PRIO, "Dropping media format message as downstream is not connected");
      spf_msg_return_msg(msg);
      return AR_EFAILED;
   }

   return result;
}
