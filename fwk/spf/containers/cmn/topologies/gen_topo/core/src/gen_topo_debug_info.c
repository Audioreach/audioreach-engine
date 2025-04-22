/**
 * \file gen_topo_debug_info.c
 *
 * \brief
 *
 *     Basic topology implementation.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_topo.h"
#include "apm_debug_info.h"
#include "topo_utils.h"
#include "capi_types.h"
#include "gen_topo_capi.h"
#include "gen_topo_buf_mgr.h"
#include "gen_topo_prof.h"
#include "gen_topo_ctrl_port.h"
#include "apm_cntr_debug_if.h"
// clang-format on

static ar_result_t gen_topo_fill_rtm_payload_with_mf(gen_topo_common_port_t *          cmn_port_ptr,
                                                     apm_param_id_port_media_format_t *mf_payload_ptr,
                                                     uint32_t                          module_id,
                                                     uint32_t                          port_id,
                                                     uint32_t                           num_of_reserved_bytes)
{
   ar_result_t result = AR_EOK;

   mf_payload_ptr->module_instance_id       = module_id;
   mf_payload_ptr->port_id                  = port_id;
   apm_media_format_t *payload_data_fmt_ptr = (apm_media_format_t *)(mf_payload_ptr + 1);

   payload_data_fmt_ptr->data_format =
      gen_topo_convert_spf_data_fmt_public_data_format(cmn_port_ptr->media_fmt_ptr->data_format);
   payload_data_fmt_ptr->fmt_id = cmn_port_ptr->media_fmt_ptr->fmt_id;

   if ((SPF_UNKNOWN_DATA_FORMAT == cmn_port_ptr->media_fmt_ptr->data_format) ||
       (SPF_RAW_COMPRESSED == cmn_port_ptr->media_fmt_ptr->data_format))
   {
   }
   else if (SPF_DEINTERLEAVED_RAW_COMPRESSED == cmn_port_ptr->media_fmt_ptr->data_format)
   {
      apm_deint_raw_med_fmt_t *deint_raw_ptr = (apm_deint_raw_med_fmt_t *)(payload_data_fmt_ptr + 1);

      deint_raw_ptr->num_buffers           = cmn_port_ptr->media_fmt_ptr->deint_raw.bufs_num;
      apm_channel_mask_t *channel_mask_ptr = (apm_channel_mask_t *)(deint_raw_ptr + 1);

      for (uint32_t i = 0; i < deint_raw_ptr->num_buffers; i++)
      {
         channel_mask_ptr->channel_mask_lsw = cmn_port_ptr->media_fmt_ptr->deint_raw.ch_mask[i].channel_mask_lsw;
         channel_mask_ptr->channel_mask_msw = cmn_port_ptr->media_fmt_ptr->deint_raw.ch_mask[i].channel_mask_msw;
         channel_mask_ptr++;
      }
   }

   else
   {
      apm_pcm_pack_med_fmt_t *pcm_ptr = (apm_pcm_pack_med_fmt_t *)(payload_data_fmt_ptr + 1);
      pcm_ptr->sample_rate            = cmn_port_ptr->media_fmt_ptr->pcm.sample_rate;
      pcm_ptr->bit_width              = cmn_port_ptr->media_fmt_ptr->pcm.bit_width;
      pcm_ptr->bits_per_sample        = cmn_port_ptr->media_fmt_ptr->pcm.bits_per_sample;
      pcm_ptr->q_factor               = cmn_port_ptr->media_fmt_ptr->pcm.q_factor;
      pcm_ptr->num_channels           = cmn_port_ptr->media_fmt_ptr->pcm.num_channels;
      pcm_ptr->interleaving =
         gen_topo_convert_gen_topo_interleaving_to_public_interleaving(cmn_port_ptr->media_fmt_ptr->pcm.interleaving);
      pcm_ptr->endianness =
         gen_topo_convert_gen_topo_endianness_to_public_endianness(cmn_port_ptr->media_fmt_ptr->pcm.endianness);
	  /*actual size of apm_pcm_pack_med_fmt_t is 10 bytes but sizeof(apm_pcm_pack_med_fmt_t) is 12 bytes so reducing
	    2 bytes to get correct size of apm_pcm_pack_med_fmt_t*/
      uint8_t *chan_map_ptr = (uint8_t *)((uint8_t *)pcm_ptr + (sizeof(apm_pcm_pack_med_fmt_t)- 2));
      for (uint32_t i = 0; i < pcm_ptr->num_channels; i++)
      {
         chan_map_ptr[i] = cmn_port_ptr->media_fmt_ptr->pcm.chan_map[i];
      }
      chan_map_ptr += pcm_ptr->num_channels;
      for(uint32_t j = 0; j < num_of_reserved_bytes; j++)
      {
         chan_map_ptr[j] = 0;
      }
   }
   return result;
}

static void gen_topo_calculate_rtm_dump_mf_size_per_port(gen_topo_common_port_t *cmn_port_ptr,uint32_t * size_of_port, uint8_t *num_of_reserved_bytes)
{
   //uint32_t size = 0;
   *size_of_port += sizeof(apm_param_id_port_media_format_t) + sizeof(apm_media_format_t);

   if ((cmn_port_ptr->media_fmt_ptr->data_format == SPF_UNKNOWN_DATA_FORMAT) ||
       (cmn_port_ptr->media_fmt_ptr->data_format == SPF_RAW_COMPRESSED))
   {
   }
   else if (cmn_port_ptr->media_fmt_ptr->data_format == SPF_DEINTERLEAVED_RAW_COMPRESSED)
   {
      *size_of_port += sizeof(apm_deint_raw_med_fmt_t) +
              (sizeof(apm_channel_mask_t) * (cmn_port_ptr->media_fmt_ptr->deint_raw.bufs_num));
   }
   else
   {
      uint32_t actual_payload_size =
         (sizeof(apm_pcm_pack_med_fmt_t)-2) + (sizeof(uint8_t) * cmn_port_ptr->media_fmt_ptr->pcm.num_channels);
      uint32_t size_of_current_port = ALIGN_4_BYTES(actual_payload_size);
      //*size_of_port += ALIGN_4_BYTES(actual_payload_size);
      *size_of_port += size_of_current_port;
      *num_of_reserved_bytes = size_of_current_port - actual_payload_size;
   }
   //return size;
}

static uint32_t gen_topo_calculate_total_size_of_port_mf_rtm_dump(gu_t *    gu_ptr)

{
	uint32_t size_of_output_ports = 0;
	uint32_t size_of_input_ports  = 0;
	uint8_t num_of_reserved_bytes = 0;
   for (gu_sg_list_t *sg_list_ptr = gu_ptr->sg_list_ptr; (NULL != sg_list_ptr); LIST_ADVANCE(sg_list_ptr))
   {
      for (gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != module_list_ptr);
           LIST_ADVANCE(module_list_ptr))
      {
         for (gu_input_port_list_t *input_port_list_ptr = module_list_ptr->module_ptr->input_port_list_ptr;
              (NULL != input_port_list_ptr);
              LIST_ADVANCE(input_port_list_ptr))
         {
            gen_topo_common_port_t *cmn_port_ptr = &((gen_topo_input_port_t *)input_port_list_ptr->ip_port_ptr)->common;
            gen_topo_calculate_rtm_dump_mf_size_per_port(cmn_port_ptr, &size_of_input_ports,  &num_of_reserved_bytes);
         }

         for (gu_output_port_list_t *output_port_list_ptr = module_list_ptr->module_ptr->output_port_list_ptr;
              (NULL != output_port_list_ptr);
              LIST_ADVANCE(output_port_list_ptr))
         {
            gen_topo_common_port_t *cmn_port_ptr =
                          &((gen_topo_output_port_t *)output_port_list_ptr->op_port_ptr)->common;
            gen_topo_calculate_rtm_dump_mf_size_per_port(cmn_port_ptr, &size_of_output_ports, &num_of_reserved_bytes);
         }
      }
   }
   return (size_of_input_ports + size_of_output_ports);
}

typedef struct gen_topo_rtm_dump_header_t
{
   rtm_logging_header_t               rtm_header;
   rtm_logging_param_data_t           rtm_param_data;
   apm_param_id_port_media_fmt_list_t apm_cntr;
} gen_topo_rtm_dump_header_t;

static ar_result_t gen_topo_populate_rtm_dump_header(uint8_t *log_ptr,
                                                     uint32_t total_size_to_dump,
                                                     uint32_t num_of_port_media_formats_of_cntr,gen_topo_t *topo_ptr)
{
   ar_result_t                 result  = AR_EOK;
   gen_topo_rtm_dump_header_t *rtm_ptr = (gen_topo_rtm_dump_header_t *)log_ptr;
   rtm_ptr->rtm_header.version                  = RTM_HEADER_VERSION_0;
   rtm_ptr->rtm_param_data.module_instance_id       = APM_MODULE_INSTANCE_ID;
   rtm_ptr->rtm_param_data.param_id                 = APM_PARAM_ID_PORT_MEDIA_FMT_REPORT_CFG;
   rtm_ptr->rtm_param_data.reserved                 = 0;
   rtm_ptr->rtm_param_data.param_size               = sizeof(apm_param_id_port_media_fmt_list_t) + total_size_to_dump;
   rtm_ptr->apm_cntr.container_id                      = topo_ptr->gu.container_instance_id;
   rtm_ptr->apm_cntr.num_port_media_format              = num_of_port_media_formats_of_cntr;
   return result;
}

static void gen_topo_rtm_dump_commit(gen_topo_t *topo_ptr, uint8_t **log_pptr, uint32_t max_allowed_payload_size)
{
   posal_data_log_info_t posal_log_info;
   posal_log_info.buf_size   = max_allowed_payload_size;
   posal_log_info.data_fmt   = LOG_DATA_FMT_BITSTREAM;
   posal_log_info.log_code   = (uint32_t)DEBUG_INFO_LOG_CODE;
   posal_log_info.log_tap_id = DEBUG_INFO_TAP_ID;
   posal_log_info.session_id = 0;
   posal_data_log_commit(*log_pptr, &posal_log_info);
   posal_log_info.seq_number_ptr = &(topo_ptr->port_mf_rtm_dump_seq_num);
   topo_ptr->port_mf_rtm_dump_seq_num += 1;
}

static ar_result_t gen_topo_rtm_dump_data_port_mf_for_each_port(gen_topo_t *topo_ptr,
                                                                gen_topo_common_port_t *cmn_port_ptr,
                                                                uint32_t *remaining_log_pkt_size_ptr,
                                                                uint8_t **log_pptr,
                                                                uint32_t *total_size_to_dump_ptr,
                                                                uint32_t *number_of_mfs_counter_ptr,
                                                                uint32_t *payload_size_for_commit_ptr,
                                                                apm_param_id_port_media_format_t **mf_payload_pptr,
                                                                uint32_t module_id,
                                                                uint32_t port_id)
{
   ar_result_t           result = AR_EOK;
   uint8_t num_of_reserved_bytes = 0;
   uint32_t size_for_port = 0;
    gen_topo_calculate_rtm_dump_mf_size_per_port(cmn_port_ptr,&size_for_port, &num_of_reserved_bytes);
   uint32_t header_size   = sizeof(gen_topo_rtm_dump_header_t);
   if (*remaining_log_pkt_size_ptr < size_for_port)
   {

      if (*log_pptr)
      {
         ((gen_topo_rtm_dump_header_t *)*log_pptr)->apm_cntr.num_port_media_format = *number_of_mfs_counter_ptr;
         gen_topo_rtm_dump_commit(topo_ptr, log_pptr, *payload_size_for_commit_ptr);
         *payload_size_for_commit_ptr = 0;
         *remaining_log_pkt_size_ptr = 0;
         *log_pptr = NULL; // to prevent commit call outside the loop not to fail.
      }
      // create new packet
      uint32_t max_payload_size             = header_size + *total_size_to_dump_ptr;
      uint32_t max_allowed_payload_size     = MIN(max_payload_size, MAX_LOG_PKT_SIZE_RTM);
      *log_pptr                      = posal_data_log_alloc(max_allowed_payload_size, DEBUG_INFO_LOG_CODE, LOG_DATA_FMT_BITSTREAM);

      if (NULL == *log_pptr)
      {
         TOPO_MSG(topo_ptr->gu.log_id,DBG_HIGH_PRIO,"gen_topo_rtm_dump_data_port_mf_for_all_ports() :Failed to allocate log packet");
         return result;
      }

      gen_topo_populate_rtm_dump_header(*log_pptr, *total_size_to_dump_ptr, *number_of_mfs_counter_ptr, topo_ptr);

      *mf_payload_pptr              = (apm_param_id_port_media_format_t *)(*log_pptr + header_size);
      *remaining_log_pkt_size_ptr = (max_allowed_payload_size - header_size);
      *payload_size_for_commit_ptr = max_allowed_payload_size;
   }

   gen_topo_fill_rtm_payload_with_mf(cmn_port_ptr, *mf_payload_pptr, module_id, port_id, num_of_reserved_bytes);
   *mf_payload_pptr = (apm_param_id_port_media_format_t *)(((uint8_t *)*mf_payload_pptr) + size_for_port);
   *remaining_log_pkt_size_ptr -= size_for_port;
   *total_size_to_dump_ptr -= size_for_port;
   *number_of_mfs_counter_ptr += 1;
   return           result;
}

ar_result_t gen_topo_rtm_dump_data_port_mf_for_all_ports(void *   vtopo_ptr,
                                                         uint32_t container_instance_id,
                                                         uint32_t port_media_fmt_report_enable)
{
   ar_result_t result                      = AR_EOK;
   gen_topo_t *topo_ptr                    = (gen_topo_t *)vtopo_ptr;

   if (topo_ptr->flags.port_mf_rtm_dump_enable == port_media_fmt_report_enable)
   {
      return result;
   }
   topo_ptr->flags.port_mf_rtm_dump_enable = port_media_fmt_report_enable;
   if (FALSE == port_media_fmt_report_enable)
   {
      return result;
   }

   uint32_t total_size_to_dump     = 0; // doesn't include header size
   uint32_t remaining_log_pkt_size = 0;
   uint32_t number_of_mfs_counter  = 0;
   uint32_t payload_size_for_commit = 0;
   uint8_t *log_ptr                = NULL;

   apm_param_id_port_media_format_t *mf_payload_ptr = NULL;

   total_size_to_dump = gen_topo_calculate_total_size_of_port_mf_rtm_dump(&(topo_ptr->gu));
    for (gu_sg_list_t *sg_list_ptr = topo_ptr->gu.sg_list_ptr; (NULL != sg_list_ptr); LIST_ADVANCE(sg_list_ptr))
   {
      for (gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != module_list_ptr);
           LIST_ADVANCE(module_list_ptr))
      {
         for (gu_input_port_list_t *input_port_list_ptr = module_list_ptr->module_ptr->input_port_list_ptr;
              (NULL != input_port_list_ptr);
              LIST_ADVANCE(input_port_list_ptr))
         {
            gen_topo_common_port_t *cmn_port_ptr = &((gen_topo_input_port_t *)input_port_list_ptr->ip_port_ptr)->common;
            gen_topo_rtm_dump_data_port_mf_for_each_port(topo_ptr,
                                                         cmn_port_ptr,
                                                         &remaining_log_pkt_size,
                                                         &log_ptr,
                                                         &total_size_to_dump,
                                                         &number_of_mfs_counter,
                                                         &payload_size_for_commit,
                                                         &mf_payload_ptr,
                                                         module_list_ptr->module_ptr->module_instance_id,
                                                         input_port_list_ptr->ip_port_ptr->cmn.id);
         }

         for (gu_output_port_list_t *output_port_list_ptr = module_list_ptr->module_ptr->output_port_list_ptr;
              (NULL != output_port_list_ptr);
              LIST_ADVANCE(output_port_list_ptr))
         {
            gen_topo_common_port_t *cmn_out_port_ptr =
               &((gen_topo_output_port_t *)output_port_list_ptr->op_port_ptr)->common;
            gen_topo_rtm_dump_data_port_mf_for_each_port(topo_ptr,
                                                         cmn_out_port_ptr,
                                                         &remaining_log_pkt_size,
                                                         &log_ptr,
                                                         &total_size_to_dump,
                                                         &number_of_mfs_counter,
                                                         &payload_size_for_commit,
                                                         &mf_payload_ptr,
                                                         module_list_ptr->module_ptr->module_instance_id,
                                                         output_port_list_ptr->op_port_ptr->cmn.id);
         }
      }
   }
   if (log_ptr)
   {
      ((gen_topo_rtm_dump_header_t *)log_ptr)->apm_cntr.num_port_media_format = number_of_mfs_counter;
      gen_topo_rtm_dump_commit(topo_ptr, &log_ptr, payload_size_for_commit);
   }
   return result;
}

ar_result_t gen_topo_rtm_dump_change_in_port_mf(gen_topo_t *            topo_ptr,
                                                gen_topo_module_t *     module_ptr,
                                                gen_topo_common_port_t *cmn_port_ptr,
                                                uint32_t                port_id)
{
   ar_result_t result                            = AR_EOK;
   uint32_t    num_of_port_media_formats_of_cntr = 1;
   uint32_t    size_for_port                     = 0;
   uint8_t    num_of_reserved_bytes             = 0;
   gen_topo_calculate_rtm_dump_mf_size_per_port(cmn_port_ptr, &size_for_port, &num_of_reserved_bytes);
   uint32_t header_size = sizeof(gen_topo_rtm_dump_header_t);
   uint32_t max_payload_size = header_size + size_for_port;
   uint8_t *log_ptr          = posal_data_log_alloc(max_payload_size, DEBUG_INFO_LOG_CODE, LOG_DATA_FMT_BITSTREAM);
   if (NULL == log_ptr)
   {
      TOPO_MSG(topo_ptr->gu.log_id,
               DBG_HIGH_PRIO,
               "gen_topo_rtm_dump_change_in_port_mf() :Failed to allocate log packet");
      return result;
   }
   gen_topo_populate_rtm_dump_header(log_ptr, size_for_port, num_of_port_media_formats_of_cntr, topo_ptr);
   apm_param_id_port_media_format_t *mf_payload_ptr = (apm_param_id_port_media_format_t *)(log_ptr + header_size);
   gen_topo_fill_rtm_payload_with_mf(cmn_port_ptr,
                                     mf_payload_ptr,
                                     module_ptr->gu.module_instance_id,
                                     port_id,
                                     num_of_reserved_bytes);
   gen_topo_rtm_dump_commit(topo_ptr, &log_ptr, max_payload_size);
   return result;
}
