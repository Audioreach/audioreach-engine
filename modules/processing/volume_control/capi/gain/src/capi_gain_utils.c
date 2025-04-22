/* =========================================================================
  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 * @file capi_gain_utils.cpp
 *
 * C source file to implement the Audio Post Processor Interface for
 * Gain Module
 */

#include "capi_gain_utils.h"


static bool_t gain_is_supported_media_type(const capi_media_fmt_v2_t *format_ptr);

static bool_t gain_is_supported_media_type(const capi_media_fmt_v2_t *format_ptr)
{
   if (CAPI_FIXED_POINT != format_ptr->header.format_header.data_format)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "CAPI GAIN: unsupported data format %lu",
             (uint32_t)format_ptr->header.format_header.data_format);
      return FALSE;
   }

   if ((16 != format_ptr->format.bits_per_sample) && (32 != format_ptr->format.bits_per_sample))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "CAPI GAIN: only supports 16 and 32 bit data. Received %lu.",
             format_ptr->format.bits_per_sample);
      return FALSE;
   }

   if ((CAPI_DEINTERLEAVED_UNPACKED_V2 != format_ptr->format.data_interleaving) &&
       (CAPI_INTERLEAVED != format_ptr->format.data_interleaving) &&
       (format_ptr->format.num_channels != 1))
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI GAIN : Interleaved data not supported.");
      return FALSE;
   }

   if (!format_ptr->format.data_is_signed)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI GAIN: Unsigned data not supported.");
      return FALSE;
   }

   if ((format_ptr->format.num_channels == 0) || (format_ptr->format.num_channels > CAPI_MAX_CHANNELS_V2))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "CAPIv2 Gain: Only upto %lu channels supported."
             "Received %lu.",
			 CAPI_MAX_CHANNELS_V2,
             format_ptr->format.num_channels);
      return FALSE;
   }

   return TRUE;
}

void capi_gain_init_config(capi_gain_t *me_ptr)
{
   me_ptr->events_config.enable = TRUE;
   me_ptr->lib_config.enable    = TRUE;

   uint16_t apply_gain         = 0x2000;
   me_ptr->gain_q12            = apply_gain >> 1;
   me_ptr->lib_config.gain_q13 = apply_gain;
}

/* =========================================================================
 * FUNCTION : capi_gain_raise_process_event
 * DESCRIPTION: Function to send the output media format using the
 *              callback function
 * =========================================================================*/
capi_err_t capi_gain_raise_process_event(capi_gain_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   uint32_t      enable         = TRUE;
   if (Q13_UNITY == me_ptr->gain_q12 << 1)
   {
      enable = FALSE;
   }
   enable = enable && ((me_ptr->lib_config.enable == 0) ? 0 : 1);

   if (me_ptr->events_config.enable != enable)
   {
      capi_result = capi_cmn_update_process_check_event(&me_ptr->cb_info, enable);
      if (CAPI_EOK == capi_result)
      {
         me_ptr->events_config.enable = enable;
      }
   }
   return capi_result;
}

/* =========================================================================
 * FUNCTION : capi_gain_update_event_states
 * DESCRIPTION: function to get KPPS numbers
 * =========================================================================*/
static uint32_t capi_gain_get_kpps(capi_gain_t *me_ptr)
{
   uint32_t kpps = 0;

   if ((me_ptr->inp_media_fmt.format.sampling_rate != CAPI_DATA_FORMAT_INVALID_VAL) &&
       (me_ptr->inp_media_fmt.format.num_channels != CAPI_DATA_FORMAT_INVALID_VAL))
   {
      uint32_t kpps_8khz_mono = CAPI_GAIN_KPPS_8KHZ_MONO_CH_16BIT;
      if (BIT_WIDTH_32 == me_ptr->inp_media_fmt.format.bits_per_sample)
      {
         if ((uint16_t)(me_ptr->gain_q12 >> 12) > 0)
         {
            // kpps for gain greater than 1
            kpps_8khz_mono = CAPI_GAIN_KPPS_8KHZ_MONO_CH_32BIT_G1;
         }
         else
         {
            // kpps for gain less than 1
            kpps_8khz_mono = CAPI_GAIN_KPPS_8KHZ_MONO_CH_32BIT_L1;
         }
      }
      // scale KPPS based on num channels and sampling rate.
      kpps = kpps_8khz_mono * me_ptr->inp_media_fmt.format.num_channels *
             (me_ptr->inp_media_fmt.format.sampling_rate / 8000);
   }
   return kpps;
}

/* =========================================================================
 * FUNCTION : capi_gain_raise_event
 * DESCRIPTION: Function to raise various events of the gain module
 * =========================================================================*/
capi_err_t capi_gain_raise_event(capi_gain_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr->cb_info.event_cb)
   {
      GAIN_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Event callback is not set. Unable to raise events!");
      CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
      return capi_result;
   }

   me_ptr->events_config.kpps        = capi_gain_get_kpps(me_ptr);
   me_ptr->events_config.delay_in_us = 0;

   capi_result |= capi_gain_raise_process_event(me_ptr);
   capi_result |= capi_cmn_update_kpps_event(&me_ptr->cb_info, me_ptr->events_config.kpps);
   capi_result |= capi_cmn_update_bandwidth_event(&me_ptr->cb_info,
                                                        me_ptr->events_config.code_bw,
                                                        me_ptr->events_config.data_bw);
   capi_result |= capi_cmn_update_algo_delay_event(&me_ptr->cb_info, me_ptr->events_config.delay_in_us);

   return capi_result;
}

/*------------------------------------------------------------------------
  Function name: gain_process
  Processes an input buffer and generates an output buffer.
 * -----------------------------------------------------------------------*/
capi_err_t gain_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[])
{
   capi_err_t   capi_result = CAPI_EOK;
   capi_gain_t *me_ptr         = (capi_gain_t *)_pif;

   uint32_t nSampleCnt;
   int32_t  byte_sample_convert = (BIT_WIDTH_16 == me_ptr->inp_media_fmt.format.bits_per_sample) ? 1 : 2;
   nSampleCnt                   = s32_min_s32_s32(input[0]->buf_ptr[0].actual_data_len >> byte_sample_convert,
                                output[0]->buf_ptr[0].max_data_len >> byte_sample_convert);

   uint32_t num_bytes_processed = 0;
   for (uint32_t ch = 0; ch < input[0]->bufs_num; ch++)
   {
      if (BIT_WIDTH_16 == me_ptr->inp_media_fmt.format.bits_per_sample)
      {
         audpp_volume_apply_gain_16((int16 *)output[0]->buf_ptr[ch].data_ptr,
                                    (int16 *)input[0]->buf_ptr[ch].data_ptr,
                                    me_ptr->gain_q12,
                                    nSampleCnt);
         num_bytes_processed = nSampleCnt << 1;
      }
      else
      {
         if ((uint16)(me_ptr->gain_q12 >> 12) > 0)
         {
            // for gain greater than 1
            audpp_volume_apply_gain_32_G1((int32 *)output[0]->buf_ptr[ch].data_ptr,
                                          (int32 *)input[0]->buf_ptr[ch].data_ptr,
                                          me_ptr->gain_q12,
                                          nSampleCnt);
         }
         else
         {
            // for gain less than 1
            audpp_volume_apply_gain_32_L1((int32 *)output[0]->buf_ptr[ch].data_ptr,
                                          (int32 *)input[0]->buf_ptr[ch].data_ptr,
                                          me_ptr->gain_q12,
                                          nSampleCnt);
         }
         num_bytes_processed = nSampleCnt << 2;
      }
   }

   // update rest of the channels since module supports only CAPI_DEINTERLEAVED_UNPACKED_V2
   output[0]->buf_ptr[0].actual_data_len = num_bytes_processed;
   input[0]->buf_ptr[0].actual_data_len  = num_bytes_processed;

   return capi_result;
}

capi_err_t capi_gain_process_set_properties(capi_gain_t *me_ptr, capi_proplist_t *proplist_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == me_ptr)
   {
      GAIN_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "Set common property received null ptr");
      return CAPI_EBADPARAM;
   }

   capi_result |= capi_cmn_set_basic_properties(proplist_ptr, &me_ptr->heap_info, &me_ptr->cb_info, TRUE);
   uint32_t miid = me_ptr ? me_ptr->miid : MIID_UNKNOWN;
   if (CAPI_EOK != capi_result)
   {
      GAIN_MSG(miid, DBG_ERROR_PRIO, "Set basic properties failed with result %lu", capi_result);
   }

   capi_prop_t *prop_array = proplist_ptr->prop_ptr;
   uint32_t        i          = 0;

   for (i = 0; i < proplist_ptr->props_num; i++)
   {
      capi_buf_t *payload_ptr = &(prop_array[i].payload);
	  miid = me_ptr ? me_ptr->miid : MIID_UNKNOWN;

      switch (prop_array[i].id)
      {
         case CAPI_EVENT_CALLBACK_INFO:
         case CAPI_HEAP_ID:
         case CAPI_ALGORITHMIC_RESET:
         case CAPI_CUSTOM_INIT_DATA:
         case CAPI_PORT_NUM_INFO:
         case CAPI_INTERFACE_EXTENSIONS:
         case CAPI_OUTPUT_MEDIA_FORMAT_V2:
         {
            break;
         }
         case CAPI_INPUT_MEDIA_FORMAT_V2:
         {
            if (payload_ptr->actual_data_len >= sizeof(capi_media_fmt_v2_t))
            {
               GAIN_MSG(miid, DBG_HIGH_PRIO, "Received Input media format");

               capi_media_fmt_v2_t *data_ptr = (capi_media_fmt_v2_t *)(payload_ptr->data_ptr);
               if (!gain_is_supported_media_type(data_ptr))
               {
                  CAPI_SET_ERROR(capi_result, CAPI_EFAILED);
                  return capi_result;
               }

               // copy and save the input media fmt
               me_ptr->inp_media_fmt.header.format_header.data_format = data_ptr->header.format_header.data_format;
               me_ptr->inp_media_fmt.format                           = data_ptr->format;

               for (uint32_t ch = 0; ch < me_ptr->inp_media_fmt.format.num_channels; ch++)
               {
                  me_ptr->inp_media_fmt.channel_type[ch] = data_ptr->channel_type[ch];
               }

               // raise event for output media format
               capi_result |= capi_gain_raise_event(me_ptr);
               capi_result |=
                  capi_cmn_output_media_fmt_event_v2(&me_ptr->cb_info, &me_ptr->inp_media_fmt, FALSE, 0);
            }
            else
            {
               GAIN_MSG(miid, DBG_ERROR_PRIO,
                      "Set property id 0x%lx Bad param size %lu",
                      (uint32_t)prop_array[i].id,
                      payload_ptr->actual_data_len);
               CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
            }
            break;
         }
		  case CAPI_MODULE_INSTANCE_ID:
          {
              if (payload_ptr->actual_data_len >= sizeof(capi_module_instance_id_t))
              {
                 capi_module_instance_id_t *data_ptr = (capi_module_instance_id_t *)payload_ptr->data_ptr;
                 me_ptr->miid                        = data_ptr->module_instance_id;
                 GAIN_MSG(miid, DBG_LOW_PRIO,
                        "This module-id 0x%08lX, instance-id 0x%08lX",
                        data_ptr->module_id,
                        me_ptr->miid);
              }
              else
              {
                 GAIN_MSG(miid, DBG_ERROR_PRIO,
                        "Set property id 0x%lx, Bad param size %lu",
                        prop_array[i].id,
                        payload_ptr->max_data_len);
                 capi_result |= CAPI_ENEEDMORE;
              }
              break;
           } // CAPI_MODULE_INSTANCE_ID
         default:
         {
            capi_result |= CAPI_EUNSUPPORTED;
            break;
         }
      }
      if (CAPI_FAILED(capi_result))
      {
         GAIN_MSG(miid, DBG_HIGH_PRIO,
                "Set property for %#x failed with opcode %lu",
                prop_array[i].id,
                capi_result);
      }
   }

   return capi_result;
}

capi_err_t capi_gain_process_get_properties(capi_gain_t *me_ptr, capi_proplist_t *proplist_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   capi_basic_prop_t mod_prop;
   mod_prop.init_memory_req    = align_to_8_byte(sizeof(capi_gain_t));
   mod_prop.stack_size         = GAIN_STACK_SIZE;
   mod_prop.num_fwk_extns      = 0;
   mod_prop.fwk_extn_ids_arr   = NULL;
   mod_prop.is_inplace         = TRUE;
   mod_prop.req_data_buffering = FALSE;
   mod_prop.max_metadata_size  = 0;

   uint32_t miid = me_ptr ? me_ptr->miid : MIID_UNKNOWN;

   capi_result |= capi_cmn_get_basic_properties(proplist_ptr, &mod_prop);
   if (CAPI_EOK != capi_result)
   {
      GAIN_MSG(miid, DBG_ERROR_PRIO, "Get common basic properties failed with result %lu", capi_result);
   }

   capi_prop_t *prop_array = proplist_ptr->prop_ptr;

   for (uint32_t i = 0; i < proplist_ptr->props_num; i++)
   {
	  miid = me_ptr ? me_ptr->miid : MIID_UNKNOWN;
      switch (prop_array[i].id)
      {
         case CAPI_INIT_MEMORY_REQUIREMENT:
         case CAPI_STACK_SIZE:
         case CAPI_IS_INPLACE:
         case CAPI_REQUIRES_DATA_BUFFERING:
         case CAPI_OUTPUT_MEDIA_FORMAT_SIZE:
         case CAPI_NUM_NEEDED_FRAMEWORK_EXTENSIONS:
         {
            break;
         }
         case CAPI_IS_ELEMENTARY:
         {
            capi_buf_t *payload_ptr = &prop_array[i].payload;
            if (payload_ptr->max_data_len >= sizeof(capi_is_elementary_t))
            {
               capi_is_elementary_t *data_ptr = (capi_is_elementary_t *)payload_ptr->data_ptr;
               data_ptr->is_elementary        = TRUE;
               payload_ptr->actual_data_len   = sizeof(capi_is_elementary_t);
            }
            else
            {
               GAIN_MSG(miid, DBG_ERROR_PRIO,
                      "Get basic property id 0x%lx Bad param size %lu",
                      (uint32_t)prop_array[i].id,
                      payload_ptr->max_data_len);
               CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
            }
            break;
         }
         case CAPI_OUTPUT_MEDIA_FORMAT_V2:
         {
            if (NULL == me_ptr)
            {
               GAIN_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "null ptr while querying output mf");
               return CAPI_EBADPARAM;
            }
            capi_result = capi_cmn_handle_get_output_media_fmt_v2(&prop_array[i], &me_ptr->inp_media_fmt);
            break;
         }
         case CAPI_PORT_DATA_THRESHOLD:
         {
            if (NULL == me_ptr)
            {
               GAIN_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "null ptr while querying threshold");
               return CAPI_EBADPARAM;
            }
            uint32_t threshold_in_bytes = 1; // default
            capi_result              = capi_cmn_handle_get_port_threshold(&prop_array[i], threshold_in_bytes);
            break;
         }
         default:
         {
            capi_result |= CAPI_EUNSUPPORTED;
            break;
         }
      }
      if (CAPI_FAILED(capi_result))
      {
         GAIN_MSG(miid, DBG_HIGH_PRIO,
                "Get property for %#x failed with opcode %lu",
                prop_array[i].id,
                capi_result);
      }
   }
   return capi_result;
}
