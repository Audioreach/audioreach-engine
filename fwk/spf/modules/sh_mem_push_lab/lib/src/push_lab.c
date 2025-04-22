/*========================================================================

 */ /**
This file contains function definitions internal to CAPI
Push lab module

  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear

  */
/*====================================================================== */

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */

#include "sh_mem_push_lab_api.h"
#include "push_lab.h"
#include "ar_msg.h"
#include "apm.h"

#ifndef PM_UNIT_TEST
#include "posal_timer.h"
#include "posal_memorymap.h"
#include "posal_memory.h"
#include "spf_interleaver.h"
#else
#include "test_util.h"
#endif


//#define DEBUG_DATA_DUMP_PUSH_LAB
//#define DEBUG_PUSH_LAB

static capi_err_t push_lab_check_send_watermark_event(capi_push_lab_t *pm_ptr);

capi_err_t push_lab_init(push_lab_t *pm_ptr, sh_mem_push_lab_cfg_t *init_ptr)
{
   capi_err_t result = CAPI_EOK;

   if ((NULL == pm_ptr) || (NULL == init_ptr))
   {
      AR_MSG(DBG_ERROR_PRIO, "push_lab_init: Invalid params!");
      return CAPI_EBADPARAM;
   }

   if (0 != pm_ptr->circ_buf_mem_map_handle)
   {
      AR_MSG(DBG_ERROR_PRIO, "push_lab_init: already configured with circ buf or pos buf memories");
      return CAPI_EBADPARAM;
   }

   pm_ptr->mem_map_client = apm_get_mem_map_client();

   pm_ptr->circ_buf_mem_map_handle = init_ptr->circ_buf_mem_map_handle;

   if (CAPI_FAILED(
          result =
             posal_memorymap_get_virtual_addr_from_shm_handle(pm_ptr->mem_map_client,
                                                              pm_ptr->circ_buf_mem_map_handle,
                                                              init_ptr->shared_circ_buf_addr_lsw,
                                                              init_ptr->shared_circ_buf_addr_msw,
                                                              init_ptr->shared_circ_buf_size,
                                                              TRUE, // Need to decrement ref count to release the handle
                                                              (uint32_t *)&(pm_ptr->shared_circ_buf_start_ptr))))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "push_lab_init: Circ Buf Phy to Virt Failed, physical addr: %lx%lx, result: %d \n",
             init_ptr->shared_circ_buf_addr_msw,
             init_ptr->shared_circ_buf_addr_lsw,
             result);
      return CAPI_EFAILED;
   }

   pm_ptr->shared_circ_buf_size = init_ptr->shared_circ_buf_size;

   AR_MSG(DBG_HIGH_PRIO, "push_lab_init: circ buf addr: %lx", (uint32_t *)pm_ptr->shared_circ_buf_start_ptr);

   AR_MSG(DBG_HIGH_PRIO, "push_lab_init: circ buf size: %lu", (uint32_t *)pm_ptr->shared_circ_buf_size);

   pm_ptr->num_water_mark_levels = 0;

   if (CAPI_FAILED(result = push_lab_check_buf_size(pm_ptr, &pm_ptr->media_fmt)))
   {
      AR_MSG(DBG_ERROR_PRIO, "push_lab_deinit: releasing shared buffer.");
      push_lab_deinit(pm_ptr);
      return result;
   }

   if (result == CAPI_EOK)
   {
      pm_ptr->circ_buf_allocated = 1;
      if ((pm_ptr->resize_in_us != 0) && (pm_ptr->resize_in_bytes == 0))
      {
         pm_ptr->resize_in_bytes =
            ((pm_ptr->media_fmt.sample_rate * ((pm_ptr->media_fmt.bits_per_sample) / 8)) * pm_ptr->resize_in_us) /
            1000000;
      }
   }

   return result;
}

capi_err_t push_lab_set_fwk_ext_inp_media_fmt(push_lab_t *pm_ptr, fwk_extn_pcm_param_id_media_fmt_extn_t *extn_ptr)
{

   if ((16 != extn_ptr->bit_width) && (24 != extn_ptr->bit_width) && (32 != extn_ptr->bit_width))
   {
      AR_MSG(DBG_ERROR_PRIO, "push_lab: set param received bad param for bit_width %d", extn_ptr->bit_width);
      return CAPI_EBADPARAM;
   }
   if ((PCM_LSB_ALIGNED != extn_ptr->alignment) && (PCM_MSB_ALIGNED != extn_ptr->alignment))
   {
      AR_MSG(DBG_ERROR_PRIO, "push_lab: set param received bad param for alignment");
      return CAPI_EBADPARAM;
   }

   if ((PCM_LITTLE_ENDIAN != extn_ptr->endianness) && (PCM_BIG_ENDIAN != extn_ptr->endianness))
   {
      AR_MSG(DBG_ERROR_PRIO, "push_lab: set param received bad param for endianness");
      return CAPI_EBADPARAM;
   }

   pm_ptr->media_fmt.bit_width  = extn_ptr->bit_width;
   pm_ptr->media_fmt.alignment  = extn_ptr->alignment;
   pm_ptr->media_fmt.endianness = extn_ptr->endianness;

   return CAPI_EOK;
}

capi_err_t push_lab_check_buf_size(push_lab_t *pm_ptr, push_lab_media_fmt_t *media_fmt_ptr)
{
   uint32_t unit_size = (media_fmt_ptr->bits_per_sample >> 3) * media_fmt_ptr->num_channels;
   // if media format is not received then unit_size will be zero.
   // if shared buffer info is not received that buffer size will be zero.
   if ((0 != unit_size) && (0 != pm_ptr->shared_circ_buf_size) && (pm_ptr->shared_circ_buf_size % unit_size))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "push_lab: Circular buffer size %lu is not a multiple of unit size %lu ",
             pm_ptr->shared_circ_buf_size,
             unit_size);
      pm_ptr->is_disabled = TRUE;
      return CAPI_EFAILED;
   }

   return CAPI_EOK;
}

capi_err_t push_lab_set_inp_media_fmt(push_lab_t *          pm_ptr,
                                      media_format_t *      media_fmt_header,
                                      push_lab_media_fmt_t *dst_media_fmt_ptr)
{
   payload_media_fmt_pcm_t *media_fmt = (payload_media_fmt_pcm_t *)(media_fmt_header + 1);

   if (CAPI_FAILED(capi_cmn_validate_client_pcm_media_format(media_fmt)))
   {
      AR_MSG(DBG_ERROR_PRIO, "push_lab: input media format has errors");
      return CAPI_EBADPARAM;
   }

   dst_media_fmt_ptr->fmt_id          = media_fmt_header->fmt_id;
   dst_media_fmt_ptr->num_channels    = media_fmt->num_channels;
   dst_media_fmt_ptr->sample_rate     = media_fmt->sample_rate;
   dst_media_fmt_ptr->Q_format        = media_fmt->q_factor;
   dst_media_fmt_ptr->bit_width       = media_fmt->bit_width;
   dst_media_fmt_ptr->bits_per_sample = media_fmt->bits_per_sample;
   if (CAPI_FAILED(push_lab_check_buf_size(pm_ptr, dst_media_fmt_ptr)))
   {
      return CAPI_EFAILED;
   }
   dst_media_fmt_ptr->endianness = media_fmt->endianness;
   dst_media_fmt_ptr->alignment  = media_fmt->alignment;

   AR_MSG(DBG_HIGH_PRIO,
          "push_lab_set_inp_media_fmt: num_channels:%d  sample_rate:%d",
          dst_media_fmt_ptr->num_channels,
          dst_media_fmt_ptr->sample_rate);

   return CAPI_EOK;
}

bool_t push_lab_check_media_fmt_validity(push_lab_t *pm_ptr)
{
   push_lab_media_fmt_t *media_fmt = &pm_ptr->media_fmt;
   if (((0 == media_fmt->num_channels) || (PUSH_LAB_MAX_CHANNELS < media_fmt->num_channels)))
   {
      AR_MSG(DBG_ERROR_PRIO, "push_lab: Check media fmt validity Fail, num_channels %d", media_fmt->num_channels);
      return FALSE;
   }

   if ((0 == media_fmt->sample_rate) || (SAMPLE_RATE_384K < media_fmt->sample_rate))
   {
      AR_MSG(DBG_ERROR_PRIO, "push_lab: Check media fmt validity Fail sample rate %d", media_fmt->sample_rate);
      return FALSE;
   }

   if (((16 != media_fmt->bit_width) && (24 != media_fmt->bit_width) && (32 != media_fmt->bit_width)))
   {
      AR_MSG(DBG_ERROR_PRIO, "push_lab: Check media fmt validity Fail bits_per_sample %d", media_fmt->bit_width);
      return FALSE;
   }

   if ((PCM_LITTLE_ENDIAN != media_fmt->endianness) && (PCM_BIG_ENDIAN != media_fmt->endianness))
   {
      AR_MSG(DBG_ERROR_PRIO, "push_lab: Check media fmt validity Fail endianness %d", media_fmt->endianness);
      return FALSE;
   }

   if ((16 != media_fmt->bits_per_sample) && (24 != media_fmt->bits_per_sample) && (32 != media_fmt->bits_per_sample))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "push_lab: Check media fmt validity Fail sample_word_size %d",
             media_fmt->bits_per_sample);
      return FALSE;
   }

   if ((PCM_LSB_ALIGNED != media_fmt->alignment) && (PCM_MSB_ALIGNED != media_fmt->alignment))
   {
      AR_MSG(DBG_ERROR_PRIO, "push_lab: Check media fmt validity Fail alignment  %d ", media_fmt->alignment);
      return FALSE;
   }

   if (media_fmt->num_channels > 1 && CAPI_INTERLEAVED == media_fmt->data_interleaving)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "push_lab: Check media fmt validity Fail for data_interleaving :%d (0:CAPI_INTERLEAVED) for "
             "num_channels:%d is not supported ",
			 media_fmt->data_interleaving,
			 media_fmt->num_channels);
      return FALSE;
   }

   AR_MSG(DBG_HIGH_PRIO, "push_lab: Check media fmt validity Pass");
   return TRUE;
}

void push_lab_deinit(push_lab_t *pm_ptr)
{
   if (NULL != pm_ptr)
   {
      // Release the shm by decrementing the ref count

      if (0 != pm_ptr->mem_map_client)
      {
         posal_memorymap_shm_decr_refcount(pm_ptr->mem_map_client, pm_ptr->circ_buf_mem_map_handle);
         pm_ptr->circ_buf_mem_map_handle = 0;
         pm_ptr->mem_map_client          = 0;
      }
   }
}

capi_err_t push_lab_update_dam_output_ch_cfg(capi_push_lab_t *capi_ptr)
{
   push_lab_t *me_ptr = &capi_ptr->push_lab_info;
   if (NULL == me_ptr->dam_output_ch_cfg_received || !me_ptr->is_media_fmt_populated)
   {
      return CAPI_EOK;
   }

   uint32_t num_channels              = me_ptr->dam_output_ch_cfg_received->num_channels;
   uint32_t num_channels_from_med_fmt = me_ptr->media_fmt.num_channels;

   if (num_channels > num_channels_from_med_fmt)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "push_lab: IMC num_channels:%u > me_ptr->media_fmt.num_channels :%u which is not supported",
             num_channels,
             num_channels_from_med_fmt);

      num_channels = num_channels_from_med_fmt;
   }

   param_id_audio_dam_output_ch_cfg_t *src_cfg = me_ptr->dam_output_ch_cfg_received;
   param_id_audio_dam_output_ch_cfg_t *dst_cfg = me_ptr->dam_output_ch_cfg;

   if (NULL == dst_cfg)
   {
      uint32_t out_cfg_size = sizeof(param_id_audio_dam_output_ch_cfg_t) + (sizeof(uint32_t) * src_cfg->num_channels);

      dst_cfg = (param_id_audio_dam_output_ch_cfg_t *)posal_memory_malloc(out_cfg_size,
                                                                          (POSAL_HEAP_ID)(capi_ptr->heap_mem.heap_id));
      if (NULL == dst_cfg)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "push_lab: Param id 0x%lx, memory couldn't be allocated for the internal "
                "struts.");
         return CAPI_ENOMEMORY;
      }
      me_ptr->dam_output_ch_cfg = dst_cfg;
   }

   dst_cfg->num_channels = num_channels;

   AR_MSG(DBG_HIGH_PRIO, "push_lab: IMC num_channels:%u", dst_cfg->num_channels);

   for (uint32_t idx = 0; idx < num_channels; idx++)
   {
      dst_cfg->channels_ids[idx] =
         ((src_cfg->channels_ids[idx] - 1) < num_channels_from_med_fmt) ? (src_cfg->channels_ids[idx] - 1) : idx;

      AR_MSG(DBG_HIGH_PRIO,
             "push_lab: IMC channel map output ch idx:%u input ch idx : %u received ch idx:%u",
             idx,
             dst_cfg->channels_ids[idx],
             src_cfg->channels_ids[idx]);
   }
   return CAPI_EOK;
}



capi_err_t push_lab_get_num_out_channels_to_write(push_lab_t *me_ptr)
{
   uint32_t num_channels_to_write = 0;
   if (NULL == me_ptr->dam_output_ch_cfg)
   {
      num_channels_to_write = me_ptr->media_fmt.num_channels;
      AR_MSG(DBG_HIGH_PRIO,
             "push_lab_get_num_out_channels_to_write: PARAM_ID_AUDIO_DAM_OUTPUT_CH_CFG is not yet recieved falling back to "
             "Mediaformat "
             "info num_ch:%d",
             num_channels_to_write);
   }
   else
   {
      num_channels_to_write = (me_ptr->dam_output_ch_cfg)->num_channels;
   }
   return num_channels_to_write;
}

capi_err_t push_lab_write_output(capi_push_lab_t *capi_ptr, capi_buf_t *module_buf_ptr, uint64_t timestamp)
{
   capi_err_t  result = CAPI_EOK;
   uint32_t    rem_lin_size;
   int8_t *    write_ptr = NULL, *read_ptr = NULL;
   uint32_t    bytes_to_copy, bytes_copied = 0, bytes_copied_per_channel = 0, bytes_copied_per_channel_now = 0;
   push_lab_t *me_ptr = &(capi_ptr->push_lab_info);
   capi_buf_t  output_scratch_buf;

   if (NULL == me_ptr->shared_circ_buf_start_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "push_lab_write_output: circ buf is not set-up");
      return CAPI_EFAILED;
   }
   uint32_t write_index     = me_ptr->pos_buf_write_index, temp_wr_ind;
   uint32_t word_size       = me_ptr->media_fmt.bits_per_sample;
   uint32_t input_copy_size = 0;
   uint32_t num_channels_to_write = push_lab_get_num_out_channels_to_write(me_ptr);


   if (me_ptr->is_gate_opened)
   {
      me_ptr->acc_data += module_buf_ptr[0].actual_data_len;
   }
   while (module_buf_ptr[0].actual_data_len > 0)
   {
      write_ptr    = (int8_t *)(me_ptr->shared_circ_buf_start_ptr + write_index);
      rem_lin_size = me_ptr->shared_circ_buf_size - write_index;
      if (CAPI_INTERLEAVED == me_ptr->media_fmt.data_interleaving)
      {
         if (me_ptr->media_fmt.num_channels > 1)
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "push_lab_write_output : data_interleaving :%d (0:CAPI_INTERLEAVED) for "
                   "num_channels:%d is not supported ",
				   me_ptr->media_fmt.data_interleaving,
				   me_ptr->media_fmt.num_channels);
            return CAPI_EFAILED;
         }

         bytes_to_copy = MIN(module_buf_ptr[0].actual_data_len, rem_lin_size);

         read_ptr = module_buf_ptr[0].data_ptr + bytes_copied;

         memscpy(write_ptr, bytes_to_copy, read_ptr, bytes_to_copy);
         posal_cache_flush((uint32_t)write_ptr, bytes_to_copy);

         module_buf_ptr[0].actual_data_len -= bytes_to_copy;

         bytes_copied += bytes_to_copy;
      }
      else
      {
         input_copy_size = (module_buf_ptr[0].actual_data_len * num_channels_to_write);
         bytes_to_copy   = MIN(input_copy_size, rem_lin_size);

#ifdef DEBUG_PUSH_LAB
         AR_MSG(DBG_LOW_PRIO,
                "push_lab_write_output: CAPI_DEINTERLEAVED_PACKED input_copy_size:%d num_channels_to_write:%d bytes_to_copy:%d, "
                "med_num_chan:%d",
                input_copy_size,
                num_channels_to_write,
                bytes_to_copy,
                me_ptr->media_fmt.num_channels);
#endif

         for (uint32_t i = 0; i < num_channels_to_write; i++)
         {
            uint32_t inp_ch_index                  = (me_ptr->dam_output_ch_cfg) ? (me_ptr->dam_output_ch_cfg)->channels_ids[i] : i;
            me_ptr->scratch_buf[i].data_ptr        = module_buf_ptr[inp_ch_index].data_ptr + bytes_copied_per_channel;
            me_ptr->scratch_buf[i].actual_data_len = module_buf_ptr[inp_ch_index].actual_data_len;
         }

         for (int32_t i = num_channels_to_write; i < me_ptr->media_fmt.num_channels; i++)
         {
            me_ptr->scratch_buf[i].data_ptr        = module_buf_ptr[i].data_ptr + bytes_copied_per_channel;
            me_ptr->scratch_buf[i].actual_data_len = module_buf_ptr[i].actual_data_len;
         }
         output_scratch_buf.max_data_len    = rem_lin_size;
         output_scratch_buf.actual_data_len = 0;
         output_scratch_buf.data_ptr        = write_ptr;
         if (CAPI_FAILED(result = spf_deintlv_to_intlv(me_ptr->scratch_buf,
                                                       &output_scratch_buf,
                                                       num_channels_to_write, // considering only
                                                                              // me_ptr->num_output_channels and
                                                                              // ignoring the media format info
                                                       word_size)))
         {
            AR_MSG(DBG_ERROR_PRIO, "push_lab_write_output: push_mode_write_output: De-Int - Int conversion failed");
            return result;
         }

         posal_cache_flush((uint32_t)write_ptr, bytes_to_copy);

#ifdef DEBUG_DATA_DUMP_PUSH_LAB
         static int first_check_push_lab = 1;
         FILE *     fout        = NULL;
         FILE *     fin         = NULL;
         // AR_MSG(DBG_ERROR_PRIO, "push_lab:(*output)->buf_ptr[0].actual_data_len %d ",
         // (*output)->buf_ptr[0].actual_data_len);
         if (first_check_push_lab)
         {
        	 first_check_push_lab = 0;
            fin         = fopen("push_lab_in.pcm", "w");
            fout        = fopen("push_lab_out.pcm", "w");
         }
         else
         {
            fin  = fopen("push_lab_in.pcm", "a");
            fout = fopen("push_lab_out.pcm", "a");
         }
         if (!fout || !fin)
         {
            AR_MSG(DBG_ERROR_PRIO, "push_lab: cannot open the file fin:%d fout:%d", (int)fin, (int)fout);
         }
         else
         {
            fwrite(module_buf_ptr[0].data_ptr + bytes_copied_per_channel, 1, module_buf_ptr[0].actual_data_len, fin);
            fwrite(write_ptr, 1, bytes_to_copy, fout);
            fclose(fin);
            fclose(fout);
         }
#endif

         bytes_copied_per_channel_now = bytes_to_copy / num_channels_to_write;

         for (int32_t i = 0; i < me_ptr->media_fmt.num_channels; i++)
         {
            module_buf_ptr[i].actual_data_len -= bytes_copied_per_channel_now;
         }

         bytes_copied_per_channel += bytes_copied_per_channel_now;
      }

      temp_wr_ind = write_index;
      write_index += bytes_to_copy;

      push_lab_check_send_watermark_event(capi_ptr);

      if (write_index >= me_ptr->shared_circ_buf_size)
      {
         AR_MSG(DBG_ERROR_PRIO, "push_lab: write_index is greater than shared_circ_buf_size, being set to zero");
         write_index = 0;
      }

      me_ptr->prev_write_index    = temp_wr_ind;
      me_ptr->current_write_index = write_index;
   }

   // in CAPI we need to report number of bytes consumed (when we return)
   if (CAPI_INTERLEAVED == me_ptr->media_fmt.data_interleaving)
   {
      module_buf_ptr[0].actual_data_len = bytes_copied;
   }
   else
   {
      for (int32_t i = 0; i < me_ptr->media_fmt.num_channels; i++)
      {

         module_buf_ptr[i].actual_data_len = bytes_copied_per_channel;
      }
   }

   me_ptr->pos_buf_write_index = write_index;

   return result;
}

static capi_err_t push_lab_check_send_watermark_event(capi_push_lab_t *capi_ptr)
{

   // check detection success -
   capi_err_t  result = CAPI_EOK;
   push_lab_t *pm_ptr = &(capi_ptr->push_lab_info);

   if (pm_ptr->is_gate_opened != 1)
   {
      // no detection case
      return result;
   }
   else
   {
   		AR_MSG(DBG_HIGH_PRIO, "prev_write_index: %d, current_write_index: %d",
		pm_ptr->prev_write_index, pm_ptr->current_write_index);
		AR_MSG(DBG_HIGH_PRIO, "acc_data: %d, watermark_interval_in_bytes: %d",
		pm_ptr->acc_data, pm_ptr->watermark_interval_in_bytes);
		if (pm_ptr->acc_data >= pm_ptr->watermark_interval_in_bytes)
		{
			result = capi_push_lab_populate_payload_raise_watermark_event(capi_ptr);
			AR_MSG(DBG_HIGH_PRIO, "push_lab: Raised Watermark event");
			pm_ptr->acc_data -= pm_ptr->watermark_interval_in_bytes;
		}
   }
/*    else
   {
	   	AR_MSG(DBG_HIGH_PRIO, "push_lab: prev_write_index: %d, current_write_index: %d, last_watermark_level_index: %d, watermark_interval_in_bytes: %d",
	pm_ptr->prev_write_index, pm_ptr->current_write_index, pm_ptr->last_watermark_level_index, pm_ptr->watermark_interval_in_bytes);
      if ((pm_ptr->prev_write_index <= (pm_ptr->last_watermark_level_index + pm_ptr->watermark_interval_in_bytes)) &&
          (pm_ptr->current_write_index >= (pm_ptr->last_watermark_level_index + pm_ptr->watermark_interval_in_bytes)))
      {
         result = capi_push_lab_populate_payload_raise_watermark_event(capi_ptr);
         pm_ptr->last_watermark_level_index += pm_ptr->watermark_interval_in_bytes;
      }
   } */
   return result;
}
