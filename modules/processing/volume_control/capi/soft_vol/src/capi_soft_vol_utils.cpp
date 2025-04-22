/* =========================================================================
  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 * @file capi_soft_vol_utils.cpp
 *
 * Utility functions for soft_vol module.
 */

#include "ar_defs.h"
#include "audpp_util.h"
#include "capi_soft_vol_utils.h"
#include "audio_basic_op_ext.h"
#include "audio_exp10.h"

static uint32_t mapping_to_mask(const uint32_t numChannels, const uint16_t mapping[]);
static bool_t is_valid_channel_type(const uint16_t channel_type);

//#define SOFT_VOL_DEBUG 1

/* TODO: check for correct value based on nch and fs */
static const uint32_t SOFT_VOL_STACK_SIZE = 2000;
/*this is for mono, 48K, 16/24bit case */
static const uint32_t SOFT_VOL_KPPS = 125;

capi_err_t capi_vol_imc_set_param_handler(capi_soft_vol *me_ptr, capi_buf_t *intent_buf_ptr)
{
	capi_err_t result = CAPI_EOK;

    if (NULL == intent_buf_ptr->data_ptr)
    {
       SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO, "IMC set param handler received null buffer");
       result |= CAPI_EBADPARAM;
       return result;
    }

    // Level 1 check
    if (intent_buf_ptr->actual_data_len < MIN_INCOMING_IMCL_PARAM_SIZE_P_EQ_VOL)
    {
       SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Invalid payload size for incoming data %d", intent_buf_ptr->actual_data_len);
       return CAPI_ENEEDMORE;
    }

   // accessing the wrong payload.. need to do + sizeof(incoming payload struct to access the actual data)
   int8_t * payload_ptr  = intent_buf_ptr->data_ptr + sizeof(intf_extn_param_id_imcl_incoming_data_t);
   uint32_t payload_size = intent_buf_ptr->actual_data_len - sizeof(intf_extn_param_id_imcl_incoming_data_t);

   while (payload_size >= sizeof(vol_imcl_header_t))
   {
      vol_imcl_header_t *header_ptr = (vol_imcl_header_t *)payload_ptr;

      payload_ptr += sizeof(vol_imcl_header_t);
      payload_size -= sizeof(vol_imcl_header_t);//TODO: move them to bottom
      switch (header_ptr->opcode)
      {
      case PARAM_ID_P_EQ_VOL_HEADROOM:
      {
          if (header_ptr->actual_data_len < sizeof(p_eq_vol_headroom_param_t))
          {
             SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                    "IMC Param id 0x%lx Invalid payload size for incoming data %d",
                    header_ptr->opcode,
                    header_ptr->actual_data_len);
             return CAPI_ENEEDMORE;
          }

          p_eq_vol_headroom_param_t *cfg_ptr = (p_eq_vol_headroom_param_t *)payload_ptr;
          int32_t headroom_mB = (int32_t)cfg_ptr->headroom_in_millibels;
          SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO,
                              "IMC Param for headroom incoming data %d",
                              headroom_mB);
          if (me_ptr->soft_vol_lib.headroom_mB != headroom_mB)
          {
             me_ptr->soft_vol_lib.headroom_mB = headroom_mB;
             me_ptr->adjust_volume_based_on_headroom = (int8_t)TRUE;
          }
          else
          {
              me_ptr->adjust_volume_based_on_headroom = (int8_t)FALSE;
          }
          break;
      }
      case PARAM_ID_IMCL_MUTE:
      {
         if (header_ptr->actual_data_len < sizeof(param_id_imcl_mute_t))
         {
            SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                   "IMC Param id 0x%lx Invalid payload size for incoming data %d",
                   header_ptr->opcode,
                   header_ptr->actual_data_len);
            return CAPI_ENEEDMORE;
         }

         param_id_imcl_mute_t *mute_cfg_ptr = (param_id_imcl_mute_t *)payload_ptr;

         if (TRUE == mute_cfg_ptr->mute_flag)
         {
            capi_soft_vol_set_mute_for_all_channels(me_ptr, TRUE);
            SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Mute received through IMC, so muting channels");
         }
         else
         {
            capi_soft_vol_set_mute_for_all_channels(me_ptr, FALSE);
            SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Un-mute received through IMC, so un-muting channels");
         }
         break;
      }
      default:
      {
         SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Unsupported opcode for incoming data over IMCL %d", header_ptr->opcode);
         return CAPI_EUNSUPPORTED;
      }

         SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO,
                "IMC Set param 0x%x done. payload size = %lu",
                header_ptr->opcode,
                header_ptr->actual_data_len);
      }

      payload_ptr += header_ptr->actual_data_len;
      payload_size -= header_ptr->actual_data_len;
   }

   return result;
}


bool_t capi_soft_vol_is_supported_media_type_v2(capi_soft_vol_t *me_ptr, capi_media_fmt_v2_t *format_ptr)
{
   if (CAPI_FIXED_POINT != format_ptr->header.format_header.data_format)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "CAPI SOFT_VOL: unsupported data format %lu",
             (uint32_t)format_ptr->header.format_header.data_format);
      return FALSE;
   }

   if ((16 != format_ptr->format.bits_per_sample) && (32 != format_ptr->format.bits_per_sample))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "CAPI SOFT_VOL: Only 16/32 bit data supported. Received %lu.",
             format_ptr->format.bits_per_sample);
      return FALSE;
   }

   if (CAPI_DEINTERLEAVED_UNPACKED != format_ptr->format.data_interleaving && format_ptr->format.num_channels != 1)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI SOFT_VOL: Interleaved data not supported.");
      return FALSE;
   }

   if (!format_ptr->format.data_is_signed)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI SOFT_VOL: Unsigned data not supported.");
      return FALSE;
   }

   if (format_ptr->format.num_channels > SOFT_VOL_MAX_OUT_CHANNELS)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI SOFT_VOL: Only upto %lu channels supported. Received %lu.", CAPI_MAX_CHANNELS_V2, format_ptr->format.num_channels);
      return FALSE;
   }

   for (uint16_t i = 0; i < format_ptr->format.num_channels && i < CAPI_MAX_CHANNELS_V2; i++)
   {
      if (!is_valid_channel_type(format_ptr->channel_type[i]))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "CAPI SOFT_VOL: unsupported channel map %hu for channel no. %hu.",
                format_ptr->channel_type[i],
                i);
         return FALSE;
      }
      if(format_ptr->channel_type[i] > PCM_MAX_CHANNEL_MAP)
      {
    	 //If a higher channel map in the media format occurs even once in the execution history,
    	 //the flag will be set to true from that point onward.
   	     me_ptr->higher_channel_map_present = TRUE;
      }
   }

   return TRUE;
}

capi_err_t capi_set_default_ramp_params(capi_soft_vol_t *me_ptr)
{
   capi_err_t                  result = CAPI_EOK;
   volume_ctrl_gain_ramp_params_t default_gain_ramp_params;
   default_gain_ramp_params.period_ms     = 40;
   default_gain_ramp_params.ramping_curve = RAMP_LINEAR;
   default_gain_ramp_params.step_us       = 1000;

   capi_port_info_t capiv2_pause_port_info;
   capiv2_pause_port_info.is_valid = FALSE;
   capi_buf_t i_buf;

   i_buf.data_ptr        = (int8_t *)&default_gain_ramp_params;
   i_buf.actual_data_len = sizeof(default_gain_ramp_params);
   i_buf.max_data_len    = sizeof(default_gain_ramp_params);
   result                = me_ptr->vtbl_ptr->set_param((capi_t *)me_ptr,
                                        PARAM_ID_VOL_CTRL_GAIN_RAMP_PARAMETERS,
                                        &capiv2_pause_port_info,
                                        &i_buf);

   volume_ctrl_mute_ramp_params_t default_mute_ramp_params;
   default_mute_ramp_params.period_ms     = 40;
   default_mute_ramp_params.ramping_curve = RAMP_LINEAR;
   default_mute_ramp_params.step_us       = 1000;

   i_buf.data_ptr        = (int8_t *)&default_mute_ramp_params;
   i_buf.actual_data_len = sizeof(default_mute_ramp_params);
   i_buf.max_data_len    = sizeof(default_mute_ramp_params);
   result                = me_ptr->vtbl_ptr->set_param((capi_t *)me_ptr,
                                        PARAM_ID_VOL_CTRL_MUTE_RAMP_PARAMETERS,
                                        &capiv2_pause_port_info,
                                        &i_buf);
   return result;
}

capi_err_t capi_soft_vol_init_lib_memory(capi_soft_vol_t *me_ptr)
{
   /* TODO: Memory query return or memory allocation for required channels. */
   int8_t *ptr = (int8_t *)me_ptr;

   me_ptr->soft_vol_lib.masterGain      = UNITY_GAIN_Q13;
   me_ptr->soft_vol_lib.headroom_mB     = 0;
   me_ptr->soft_vol_lib.channelGain[0]  = 0;
   me_ptr->soft_vol_lib.softPauseEnable = FALSE;
   /* The 0th index channel is invalid, so don't allocate memory for it */
   me_ptr->soft_vol_lib.pPerChannelData[0] = NULL;
   me_ptr->soft_vol_lib.numChannels        = VOLUME_CONTROL_MAX_CHANNELS_V2;
   for (uint32_t i = 0; i < me_ptr->soft_vol_lib.numChannels; i++)
   {
      /* Initialize channel struct array to appropriate values. */
      me_ptr->soft_vol_lib.channelMapping[i] = i + 1;
   }

   CSoftVolumeControlsLib *pLib __attribute__((unused));
   posal_memory_placement_new(pLib, &me_ptr->SoftVolumeControlsLib, CSoftVolumeControlsLib);

   uint32_t     perChannelStructSize = align_to_8_byte(CSoftVolumeControlsLib::GetSizeOfPerChannelStruct());
   const uint32_t defaultGainQ28       = capi_soft_vol_calc_gain_q28(me_ptr->soft_vol_lib.masterGain, UNITY_GAIN_Q28);

   ptr += align_to_8_byte(sizeof(capi_soft_vol_t));

   for (uint32_t i = MIN_CHANNEL_TYPE; i <= MAX_CHANNEL_TYPE; i++)
   {
      me_ptr->soft_vol_lib.channelGain[i] = UNITY_GAIN_Q28;

      me_ptr->soft_vol_lib.pPerChannelData[i] = ptr;
      me_ptr->SoftVolumeControlsLib.InitializePerChannelStruct(me_ptr->soft_vol_lib.pPerChannelData[i]);
      me_ptr->SoftVolumeControlsLib.SetVolume(defaultGainQ28, me_ptr->soft_vol_lib.pPerChannelData[i]);
      ptr += perChannelStructSize;
   }
   return CAPI_EOK;
}

capi_err_t capi_soft_vol_set_input_media_format_v2(capi_soft_vol_t *me_ptr, capi_buf_t *prop_payload_ptr)
{
   capi_err_t result        = CAPI_EOK;
   uint32_t      required_size = CAPI_SOFT_VOL_MF_V2_MIN_SIZE;
   if (prop_payload_ptr->actual_data_len < required_size)
   {
      SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO,
             "Set property for INPUT MEDIA FORMAT"
             "failed due to bad size %lu",
             prop_payload_ptr->actual_data_len);
      return CAPI_EBADPARAM;
   }
   capi_media_fmt_v2_t *data_ptr = (capi_media_fmt_v2_t *)(prop_payload_ptr->data_ptr);

   if (data_ptr->format.minor_version < CAPI_MEDIA_FORMAT_MINOR_VERSION)
   {
      SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO,
             "Set property for INPUT MEDIA FORMAT"
             "failed due to unsupported version %lu",
             data_ptr->format.minor_version);
      return CAPI_EUNSUPPORTED;
   }
   if (!capi_soft_vol_is_supported_media_type_v2(me_ptr, data_ptr))
      return CAPI_EUNSUPPORTED;

   required_size += data_ptr->format.num_channels * sizeof(data_ptr->channel_type[0]);
   if (prop_payload_ptr->actual_data_len < required_size)
   {
      SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO,
             "Set property for INPUT MEDIA FORMAT"
             "failed due to bad size %lu",
             prop_payload_ptr->actual_data_len);
      return CAPI_EBADPARAM;
   }

   me_ptr->input_media_fmt.header = data_ptr->header;
   me_ptr->input_media_fmt.format = data_ptr->format;
   memscpy(me_ptr->input_media_fmt.channel_type,
           sizeof(me_ptr->input_media_fmt.channel_type),
           data_ptr->channel_type,
           data_ptr->format.num_channels * sizeof(data_ptr->channel_type[0]));
   me_ptr->input_media_fmt.format.minor_version = CAPI_MEDIA_FORMAT_MINOR_VERSION;

   me_ptr->output_media_fmt = me_ptr->input_media_fmt;
   capi_soft_vol_set_sample_rate(me_ptr, me_ptr->input_media_fmt.format.sampling_rate);
   me_ptr->SoftVolumeControlsLib.SetBytesPerSample(me_ptr->input_media_fmt.format.bits_per_sample >> 3);

   me_ptr->soft_vol_lib.numChannels = me_ptr->input_media_fmt.format.num_channels;

   const uint32_t channelMappingLen =
      me_ptr->input_media_fmt.format.num_channels * sizeof(me_ptr->soft_vol_lib.channelMapping[0]);
   memscpy(me_ptr->soft_vol_lib.channelMapping,
           sizeof(me_ptr->soft_vol_lib.channelMapping),
           me_ptr->input_media_fmt.channel_type,
           channelMappingLen);
   me_ptr->soft_vol_state = SOFT_VOL_ENABLE;

   if (me_ptr->adjust_volume_based_on_headroom)
      (void)capi_soft_vol_headroom_gain(me_ptr, me_ptr->soft_vol_lib.headroom_mB);

   if (SOFT_VOL_DISABLE != me_ptr->soft_vol_state)
   {
      result = capi_cmn_update_algo_delay_event(&me_ptr->cb_info, 0);

      uint32_t default_kpps        = SOFT_VOL_KPPS;
      uint32_t default_sample_rate = 48000;
      uint32_t kpps                = ((me_ptr->input_media_fmt.format.num_channels * default_kpps *
                        (uint64_t)me_ptr->input_media_fmt.format.sampling_rate) /
                       default_sample_rate);

      result = capi_cmn_update_kpps_event(&me_ptr->cb_info, kpps);
      result = capi_cmn_update_bandwidth_event(&me_ptr->cb_info, 0, 0);
   }
   if (SOFT_VOL_ENABLE == me_ptr->soft_vol_state)
   {
      uint32_t process_check = 0;
      for (uint32_t i = 0; i < me_ptr->soft_vol_lib.numChannels; i++)
      {
         if (!me_ptr->SoftVolumeControlsLib.isUnityGain(
                me_ptr->soft_vol_lib.pPerChannelData[me_ptr->soft_vol_lib.channelMapping[i]]))
         {
            process_check = 1;
            break;
         }
      }
      result = capi_cmn_update_process_check_event(&me_ptr->cb_info, process_check);
   }

   result = capi_cmn_output_media_fmt_event_v2(&me_ptr->cb_info, &me_ptr->output_media_fmt, FALSE, 0);
   return result;
}

capi_err_t capi_soft_vol_process_set_properties(capi_soft_vol_t *me_ptr, capi_proplist_t *proplist_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr)
   {
      SOFT_VOL_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "Set common property received null ptr");
      return CAPI_EBADPARAM;
   }

   capi_result |= capi_cmn_set_basic_properties(proplist_ptr,
                                                     (capi_heap_id_t *)&me_ptr->heap_id,
                                                     &me_ptr->cb_info,
                                                     TRUE);
   uint32_t miid = me_ptr ? me_ptr->miid : MIID_UNKNOWN;
   if (CAPI_EOK != capi_result)
   {
      SOFT_VOL_MSG(miid, DBG_ERROR_PRIO, "Set basic properties failed with result %lu", capi_result);
   }

   capi_prop_t *prop_array = proplist_ptr->prop_ptr;
   uint32_t        i;

   for (i = 0; i < proplist_ptr->props_num; i++)
   {
      capi_buf_t *payload_ptr = &(prop_array[i].payload);
	  miid = me_ptr ? me_ptr->miid : MIID_UNKNOWN;

      switch (prop_array[i].id)
      {
         case CAPI_EVENT_CALLBACK_INFO:
         case CAPI_PORT_NUM_INFO:
         case CAPI_HEAP_ID:
         case CAPI_CUSTOM_INIT_DATA:
         case CAPI_ALGORITHMIC_RESET:
         case CAPI_INTERFACE_EXTENSIONS:
         {
            break;
         }
         case CAPI_INPUT_MEDIA_FORMAT_V2:
         {
            SOFT_VOL_MSG(miid, DBG_HIGH_PRIO, "received input media fmt V2");
            capi_result |= capi_soft_vol_set_input_media_format_v2(me_ptr, payload_ptr);
            break;
         }
		 case CAPI_MODULE_INSTANCE_ID:
         {
             if (payload_ptr->actual_data_len >= sizeof(capi_module_instance_id_t))
             {
                capi_module_instance_id_t *data_ptr = (capi_module_instance_id_t *)payload_ptr->data_ptr;
                me_ptr->miid                        = data_ptr->module_instance_id;
                SOFT_VOL_MSG(miid, DBG_LOW_PRIO,
                       "This module-id 0x%08lX, instance-id 0x%08lX",
                       data_ptr->module_id,
                       me_ptr->miid);
             }
             else
             {
                SOFT_VOL_MSG(miid, DBG_ERROR_PRIO,
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
            continue;
         }
      }
   }
   return capi_result;
}

capi_err_t capi_soft_vol_process_get_properties(capi_soft_vol_t *me_ptr, capi_proplist_t *proplist_ptr)
{
   uint32_t      i;
   capi_err_t result = CAPI_EOK;

   uint32_t soft_vol_lib_size = align_to_8_byte(CSoftVolumeControlsLib::GetSizeOfPerChannelStruct());
   soft_vol_lib_size *= (MAX_CHANNEL_TYPE - MIN_CHANNEL_TYPE + 1);

   capi_basic_prop_t mod_prop;
   mod_prop.init_memory_req    = align_to_8_byte(sizeof(capi_soft_vol_t)) + align_to_8_byte(soft_vol_lib_size);
   mod_prop.stack_size         = SOFT_VOL_STACK_SIZE;
   mod_prop.num_fwk_extns      = 0;
   mod_prop.fwk_extn_ids_arr   = NULL;
   mod_prop.is_inplace         = TRUE;
   mod_prop.req_data_buffering = FALSE;
   mod_prop.max_metadata_size  = 0;

   result |= capi_cmn_get_basic_properties(proplist_ptr, &mod_prop);
   uint32_t miid = me_ptr ? me_ptr->miid : MIID_UNKNOWN;

   if (CAPI_EOK != result)
   {
      SOFT_VOL_MSG(miid, DBG_ERROR_PRIO, "Get common basic properties failed with result %lu", result);
   }

   capi_prop_t *prop_array = proplist_ptr->prop_ptr;

   for (i = 0; i < proplist_ptr->props_num; i++)
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
         case CAPI_NEEDED_FRAMEWORK_EXTENSIONS:
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
               CAPI_SET_ERROR(result, CAPI_ENEEDMORE);
               payload_ptr->actual_data_len = 0;
               SOFT_VOL_MSG(miid, DBG_ERROR_PRIO, "Insufficient get property size.");
               break;
            }
            break;
         }
         case CAPI_INTERFACE_EXTENSIONS:
         {
            capi_buf_t *                 payload_ptr   = &prop_array[i].payload;
            capi_interface_extns_list_t *intf_ext_list = (capi_interface_extns_list_t *)payload_ptr->data_ptr;
            result |=
               ((payload_ptr->max_data_len < sizeof(capi_interface_extns_list_t)) ||
                (payload_ptr->max_data_len < (sizeof(capi_interface_extns_list_t) +
                                              (intf_ext_list->num_extensions * sizeof(capi_interface_extn_desc_t)))))
                  ? CAPI_ENEEDMORE
                  : result;

            if (CAPI_FAILED(result))
            {
               payload_ptr->actual_data_len = 0;
               SOFT_VOL_MSG(miid, DBG_ERROR_PRIO, "Insufficient get property size.");
               break;
            }

            capi_interface_extn_desc_t *curr_intf_extn_desc_ptr =
               (capi_interface_extn_desc_t *)(payload_ptr->data_ptr + sizeof(capi_interface_extns_list_t));

            for (uint32_t j = 0; j < intf_ext_list->num_extensions; j++)
            {
               switch (curr_intf_extn_desc_ptr->id)
               {
                  case INTF_EXTN_IMCL:
                     curr_intf_extn_desc_ptr->is_supported = TRUE;
                     break;
                  default:
                  {
                     curr_intf_extn_desc_ptr->is_supported = FALSE;
                     break;
                  }
               }
               curr_intf_extn_desc_ptr++;
            }

            break;
         }
         case CAPI_OUTPUT_MEDIA_FORMAT_V2:
         {
            if (NULL == me_ptr)
            {
               SOFT_VOL_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "null ptr while querying output mf");
               return CAPI_EBADPARAM;
            }
            result = capi_cmn_handle_get_output_media_fmt_v2(&prop_array[i], &me_ptr->output_media_fmt);
            break;
         }
         case CAPI_PORT_DATA_THRESHOLD:
         {
            if (NULL == me_ptr)
            {
               SOFT_VOL_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "null ptr while querying threshold");
               return CAPI_EBADPARAM;
            }
            uint32_t threshold_in_bytes = 1; // default
            result                      = capi_cmn_handle_get_port_threshold(&prop_array[i], threshold_in_bytes);
            break;
         }
         default:
         {
            result |= CAPI_EUNSUPPORTED;
            continue;
         }
      }
   }
   return result;
}

void capi_set_soft_stepping_param(const capi_soft_vol_t *me_ptr,
                                     SoftSteppingParams *      pParams,
                                     uint32_t                  period,
                                     uint32_t                  step,
                                     uint32_t                  rampingCurve)
{
   if (period < MIN_PERIOD)
   {
      period = (int32_t)MIN_PERIOD;
   }
   else if (period > MAX_PERIOD)
   {
      period = (int32_t)MAX_PERIOD;
   }

   if (step < MIN_STEP)
   {
      step = (int32_t)MIN_STEP;
   }
   else if (step > MAX_STEP)
   {
      step = (int32_t)MAX_STEP;
   }

   pParams->periodMs = period;
   pParams->stepUs   = step;

   switch (rampingCurve)
   {
      case PARAM_VOL_CTRL_RAMPINGCURVE_LINEAR:
      {
         pParams->rampingCurve = RAMP_LINEAR;
         break;
      }
      case PARAM_VOL_CTRL_RAMPINGCURVE_EXP:
      {
         pParams->rampingCurve = RAMP_EXP;
         break;
      }
      case PARAM_VOL_CTRL_RAMPINGCURVE_LOG:
      {
         pParams->rampingCurve = RAMP_LOG;
         break;
      }
      case PARAM_VOL_CTRL_RAMPINGCURVE_FRAC_EXP:
      {
         pParams->rampingCurve = RAMP_FRACT_EXP;
         break;
      }
      default:
      {
         pParams->rampingCurve = RAMP_LINEAR;
         break;
      }
   } /* switch (rampingCurve) */
}

void capi_get_soft_stepping_param(const capi_soft_vol_t *me_ptr,
                                     const SoftSteppingParams *pParams,
                                     uint32_t *                pPeriod,
                                     uint32_t *                pStep,
                                     uint32_t *                pRampingCurve)
{
   *pPeriod = pParams->periodMs;
   *pStep   = pParams->stepUs;

   switch (pParams->rampingCurve)
   {
      case RAMP_LINEAR:
      {
         *pRampingCurve = PARAM_VOL_CTRL_RAMPINGCURVE_LINEAR;
         break;
      }
      case RAMP_EXP:
      {
         *pRampingCurve = PARAM_VOL_CTRL_RAMPINGCURVE_EXP;
         break;
      }
      case RAMP_LOG:
      {
         *pRampingCurve = PARAM_VOL_CTRL_RAMPINGCURVE_LOG;
         break;
      }
      case RAMP_FRACT_EXP:
      {
         *pRampingCurve = PARAM_VOL_CTRL_RAMPINGCURVE_FRAC_EXP;
         break;
      }
      default:
      {
         SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                "Unsupported ramping curve found, defaulting to PARAM_VOL_CTRL_RAMPINGCURVE_LINEAR");
         *pRampingCurve = PARAM_VOL_CTRL_RAMPINGCURVE_LINEAR;
         break;
      }
   }
}

/* Returns FALSE if same channel is set to multiple times in different configs */
bool_t check_channel_mask_softvol(uint8_t *vol_param_ptr, uint32_t param_id)
{
   uint32_t num_config           = 0;
   uint64_t check_channel_mask   = 0x0;
   uint64_t current_channel_mask = 0x0;
   bool_t   check                = TRUE;
   uint32_t offset               = memscpy(&num_config, sizeof(uint32_t), vol_param_ptr, sizeof(uint32_t));
   uint8_t *data_ptr             = vol_param_ptr;

   for (uint32_t cnfg_cntr = 0; cnfg_cntr < num_config; cnfg_cntr++)
   {
      //Increment data ptr by calculated offset to point at next payload_lsw
      data_ptr = vol_param_ptr + offset;

      // present configuration
      //offset increments by 8
      offset += memscpy(&current_channel_mask, sizeof(uint64_t), data_ptr, sizeof(uint64_t));

      switch (param_id)
      {
         default:
         {
            //offset increments by 4
            offset += sizeof(uint32_t);
         }
      }

      // need to ignore 0th bit
      if ((0 == (check_channel_mask & current_channel_mask)) || (1 == (check_channel_mask & current_channel_mask)))
      {
         check_channel_mask |= current_channel_mask;
      }
      else
      {
         check = FALSE;
         return check;
      }
   }
   return check;
}

capi_err_t capi_soft_vol_set_multichannel_gain(capi_soft_vol_t *             me_ptr,
                                                     volume_ctrl_multichannel_gain_t *vol_payload_ptr)
{
   uint64_t channel_mask = 0;
   // validating the configs
   if (!check_channel_mask_softvol((uint8_t *)vol_payload_ptr, PARAM_ID_VOL_CTRL_MULTICHANNEL_GAIN))
   {
      SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Trying to set different gain values to same channel, returning");
      return CAPI_EBADPARAM;
   }

   // loop through all the configs
   for (uint32_t cnfg_cntr = 0; cnfg_cntr <  vol_payload_ptr->num_config; cnfg_cntr++)
   {
      channel_mask = (uint64_t)vol_payload_ptr->gain_data[cnfg_cntr].channel_mask_msb << 32 |
                     vol_payload_ptr->gain_data[cnfg_cntr].channel_mask_lsb;

      if (channel_mask & 0x1)
      {
    	  SOFT_VOL_MSG(me_ptr->miid, DBG_MED_PRIO,"Warning! Set multichannel gain: reserved bit(LSB) is being set to 1");
      }
      // setting the gain data channel wise
      for (uint32_t j = 1; j < (VOLUME_CONTROL_MAX_CHANNELS + 1); j++)
      {
         channel_mask = channel_mask >> 1;
         if (channel_mask & 0x1)
         {
            me_ptr->soft_vol_lib.channelGain[j] = vol_payload_ptr->gain_data[cnfg_cntr].gain;
            uint32 gainQ28 =
               capi_soft_vol_calc_gain_q28(me_ptr->soft_vol_lib.masterGain, me_ptr->soft_vol_lib.channelGain[j]);
            me_ptr->SoftVolumeControlsLib.SetVolume(gainQ28, me_ptr->soft_vol_lib.pPerChannelData[j]);

#ifdef SOFT_VOL_DEBUG
            SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO,
                   "Set volume: channel: %hu Gain: 0x%lx",
                   j,
                   me_ptr->soft_vol_lib.channelGain[j]);
#endif
         }
      }
   }
   return CAPI_EOK;
}

capi_err_t capi_soft_vol_set_multichannel_mute(capi_soft_vol_t *             me_ptr,
                                                     volume_ctrl_multichannel_mute_t *mute_payload_ptr)
{
   uint64_t channel_mask = 0;
   // validating the configs
   if (!check_channel_mask_softvol((uint8_t *)mute_payload_ptr, PARAM_ID_VOL_CTRL_MULTICHANNEL_MUTE))
   {
      SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Trying to set different mute configurations to same channel, returning ");
      return CAPI_EBADPARAM;
   }

   // loop through all the configs
   for (uint32_t cnfg_cntr = 0; cnfg_cntr < mute_payload_ptr->num_config; cnfg_cntr++)
   {
      channel_mask = (uint64_t)mute_payload_ptr->mute_data[cnfg_cntr].channel_mask_msb << 32 |
                     mute_payload_ptr->mute_data[cnfg_cntr].channel_mask_lsb;

      if (channel_mask & 0x1)
      {
    	  SOFT_VOL_MSG(me_ptr->miid, DBG_LOW_PRIO,"Warning Set mute: reserve bit is set 1");
      }
      // setting the mute data channel wise
      for (uint32_t j = 1; j < (VOLUME_CONTROL_MAX_CHANNELS + 1); j++)
      {
         channel_mask = channel_mask >> 1;
         if (channel_mask & 0x1)
         {
            if (0 == mute_payload_ptr->mute_data[cnfg_cntr].mute)
            {
               me_ptr->SoftVolumeControlsLib.Unmute(me_ptr->soft_vol_lib.pPerChannelData[j]);
            }
            else
            {
               me_ptr->SoftVolumeControlsLib.Mute(me_ptr->soft_vol_lib.pPerChannelData[j]);
            }
#ifdef SOFT_VOL_DEBUG
            SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO,
                   "Set mute: channel: %hu Mute: %lu",
                   j,
                   mute_payload_ptr->mute_data[cnfg_cntr].mute);
#endif
         }
      }
   }
   return CAPI_EOK;
}

void capi_soft_vol_get_multichannel_mute(capi_soft_vol_t *             me_ptr,
                                            volume_ctrl_multichannel_mute_t *mute_payload_ptr)
{
   uint16_t channel_type        = 0;
   uint32_t mute                = 0;
   mute_payload_ptr->num_config = 1; // num_config is 1 (only mute data)
   mute_payload_ptr->mute_data[0].channel_mask_lsb = 0;
   mute_payload_ptr->mute_data[0].channel_mask_msb = 0;

   for (int i = 0; i < me_ptr->soft_vol_lib.numChannels; i++)
   {
      channel_type = me_ptr->soft_vol_lib.channelMapping[i];
      mute         = (me_ptr->SoftVolumeControlsLib.IsMuted(
                me_ptr->soft_vol_lib.pPerChannelData[me_ptr->soft_vol_lib.channelMapping[i]]))
                ? 1
                : 0;
      if (mute == 1)
      {
         // getting param of channels 1~31 (channel_mask_lsb)
         if (channel_type < SOFT_VOL_MASK_WIDTH)
         {
            mute_payload_ptr->mute_data[0].channel_mask_lsb =
               mute_payload_ptr->mute_data[0].channel_mask_lsb | (1 << channel_type);
         }
         // getting params of channels 32 ~ Max channel type support ( channel_mask_msb)
         else if (channel_type < SOFT_VOL_MAX_CHANNEL_TYPE + 1)
         {
            mute_payload_ptr->mute_data[0].channel_mask_msb =
               mute_payload_ptr->mute_data[0].channel_mask_msb | (1 << (channel_type - 32));
         }
	 }

   }
   if(mute_payload_ptr->mute_data[0].channel_mask_lsb != 0 ||  mute_payload_ptr->mute_data[0].channel_mask_msb != 0)
   {
	  mute_payload_ptr->mute_data[0].mute = 1;
   }

}

/* checks if the gain is already present in the previous configs added and updates the number of unique gains
 * (equivalent to number of configs)*/
bool_t check_unique_gain(uint32_t *gain_list, uint32_t gain, uint32_t *num_unique_gains_ptr, uint32_t *position_ptr)
{
   bool_t unique = TRUE;
   // loop through all the gains in the gain_list
   for (uint32_t i = 0; i < *num_unique_gains_ptr; i++)
   {
      if (gain == gain_list[i])
      {
         unique   = FALSE;
         *position_ptr = i;
         break;
      }
   }
   if (unique)
   {
      gain_list[*num_unique_gains_ptr] = gain;
      *position_ptr                    = *num_unique_gains_ptr;
      *num_unique_gains_ptr += 1 ;
   }
   return unique;
}

static void capi_soft_vol_get_target_gain_config(capi_soft_vol_t *me_ptr, param_id_imcl_gain_t *vol_payload_ptr)
{
   uint32_t position                               = 0;
   uint32_t gain                                   = 0;
   uint16_t channel_type                           = 0;
   vol_payload_ptr->num_config                     = 0;

   for (uint32_t i = 0; i < me_ptr->soft_vol_lib.numChannels; i++)
   {
      channel_type = me_ptr->soft_vol_lib.channelMapping[i];
      gain         = me_ptr->SoftVolumeControlsLib.GetTargetGain(me_ptr->soft_vol_lib.pPerChannelData[channel_type]);
      position     = vol_payload_ptr->num_config;
      for (uint32_t j = 0; j < vol_payload_ptr->num_config; j++)
      {
         if (gain == vol_payload_ptr->gain_data[j].gain)
         {
            position = j;
            break;
         }
      }
      if (position == vol_payload_ptr->num_config)
      {
         vol_payload_ptr->num_config += 1;
         vol_payload_ptr->gain_data[position].gain             = gain;
         vol_payload_ptr->gain_data[position].channel_mask_lsb = 0;
         vol_payload_ptr->gain_data[position].channel_mask_msb = 0;
      }

      // getting param of channels 1~31 (channel_mask_lsb)
      if (channel_type < SOFT_VOL_MASK_WIDTH)
      {
         vol_payload_ptr->gain_data[position].channel_mask_lsb |= (1 << channel_type);
      }
      // getting params of channels 32 ~ Max channel support ( channel_mask_msb)
      else
      {
         vol_payload_ptr->gain_data[position].channel_mask_msb |= (1 << (channel_type - SOFT_VOL_MASK_WIDTH));
      }
   }
}

uint32_t capi_soft_vol_get_multichannel_gain(capi_soft_vol_t *             me_ptr,
                                            volume_ctrl_multichannel_gain_t *vol_payload_ptr)
{
   uint32_t position                               = 0;
   uint32_t num_unique_gains                       = 0;     // num_unique_gains is equivalent to num_configs
   uint32_t gain_list[VOLUME_CONTROL_MAX_CHANNELS] = { 0 }; // list to maintain all unique gains
   uint32_t gain                                   = 0;
   uint16_t channel_type                           = 0;
   bool_t   unique                                 = TRUE;

   if (vol_payload_ptr)
   {
   vol_payload_ptr->num_config                     = 0;
   }

   for (uint32_t i = 0; i < me_ptr->soft_vol_lib.numChannels; i++)
   {
      gain   = me_ptr->soft_vol_lib.channelGain[me_ptr->soft_vol_lib.channelMapping[i]];
      unique = check_unique_gain(gain_list, gain, &num_unique_gains, &position);

      if (vol_payload_ptr)
      {
      if (unique)
      {
         vol_payload_ptr->num_config += 1;
         vol_payload_ptr->gain_data[position].gain             = gain;
         vol_payload_ptr->gain_data[position].channel_mask_lsb = 0;
         vol_payload_ptr->gain_data[position].channel_mask_msb = 0;
      }
      channel_type = me_ptr->soft_vol_lib.channelMapping[i];
      // getting param of channels 1~31 (channel_mask_lsb)
      if (channel_type < SOFT_VOL_MASK_WIDTH)
      {
         vol_payload_ptr->gain_data[position].channel_mask_lsb =
            vol_payload_ptr->gain_data[position].channel_mask_lsb | (1 << channel_type);
      }
      // getting params of channels 32 ~ Max channel support ( channel_mask_msb)
      else if (channel_type < VOLUME_CONTROL_MAX_CHANNELS + 1)
      {
         vol_payload_ptr->gain_data[position].channel_mask_msb =
            vol_payload_ptr->gain_data[position].channel_mask_msb | (1 << (channel_type - 32));
      }
      }
   }
   return num_unique_gains;
}

void capi_soft_vol_reinit_new_channels(capi_soft_vol_t *me_ptr,
                                          const uint32_t      numNewChannels,
                                          const uint16_t      newMapping[])
{
   uint32_t oldChMask = mapping_to_mask(me_ptr->soft_vol_lib.numChannels, me_ptr->soft_vol_lib.channelMapping);
   uint32_t newChMask = mapping_to_mask(numNewChannels, newMapping);

   uint32_t additionalChMask = (newChMask & (~(oldChMask)));

   while (additionalChMask != 0)
   {
      int32_t channelType = s32_get_lsb_s32(additionalChMask);
      me_ptr->SoftVolumeControlsLib.GoToTargetGainImmediately(me_ptr->soft_vol_lib.pPerChannelData[channelType]);
      additionalChMask = s32_clr_bit_s32_s32(additionalChMask, channelType);
   }
}

void capi_soft_vol_set_mute_for_all_channels(capi_soft_vol_t *me_ptr, const uint32_t mute_flag)
{
   if (0 == mute_flag)
   {
      for (uint32_t i = MIN_CHANNEL_TYPE; i <= MAX_CHANNEL_TYPE; i++)
      {
         me_ptr->SoftVolumeControlsLib.Unmute(me_ptr->soft_vol_lib.pPerChannelData[i]);
      }
   }
   else
   {
      for (uint32_t i = MIN_CHANNEL_TYPE; i <= MAX_CHANNEL_TYPE; i++)
      {
         me_ptr->SoftVolumeControlsLib.Mute(me_ptr->soft_vol_lib.pPerChannelData[i]);
      }
   }
}

static uint32_t mapping_to_mask(const uint32_t numChannels, const uint16_t mapping[])
{
   uint32_t mask = 0;

   for (uint32_t i = 0; i < numChannels; i++)
   {
      mask = s32_set_bit_s32_s32(mask, mapping[i]);
   }

   return mask;
}

static bool_t is_valid_channel_type(const uint16_t channel_type)
{
   bool_t result = TRUE;
   if (channel_type < MIN_CHANNEL_TYPE || channel_type > MAX_CHANNEL_TYPE)
   {
      result = FALSE;
   }

   return result;
}

uint32 capi_soft_vol_calc_gain_q28(const uint16_t masterGainQ13, const uint32_t channelGainQ28)
{
   /* Combine the channel gain and master gain into a final Q28 gain value */
   uint64 gainQ41 = u64_mult_u32_u32((uint32)masterGainQ13, channelGainQ28);
   uint64 gainQ28 = gainQ41 >> (41 - 28);

   /* TODO: Need to saturate it here */
   uint32 MAX_UINT32_VAL = (((uint64)1) << 32) - 1;
   uint32 gainL32Q28     = (gainQ28 > MAX_UINT32_VAL) ? MAX_UINT32_VAL : (uint32)gainQ28;

   return gainL32Q28;
}

int32 capi_soft_vol_convert_headroom_linear_gain(const int32_t reqHeadroom)
{
   int32_t gain_Q28;
   int32_t gain_dB;

   /* (1/100 in Q24 format) */
   gain_dB = (int64_t)(-reqHeadroom) * 167772;

   /* Following expression calculates round(10^(gain_dB/20)*2^28)
    * Converting the headroom in Q0 format to gain in Q28 format
    * Input to exp10_fixed-Q26, output-Q15
    * 1/20 = 13107 (in Q18) */
   gain_Q28 = exp10_fixed(s32_saturate_s64(s64_mult_fp_s32_s16_shift(gain_dB, 13107, 0))) << 13;

   /* Minimum and maximum values are taken as -96 dB(round(10^(-96/20)*2^28) == 4254)
    * and 0 dB(round(10^(0/20)*2^28) == 268435456) respectively (in Q28 format)
    * as per Google EQ standards */

   if (gain_Q28 < HEADROOM_GAIN_Q28_MIN)
   {
      gain_Q28 = HEADROOM_GAIN_Q28_MIN;
   }
   else if (gain_Q28 > HEADROOM_GAIN_Q28_MAX)
   {
      gain_Q28 = HEADROOM_GAIN_Q28_MAX;
   }

   return gain_Q28;
}

capi_err_t capi_soft_vol_headroom_gain(capi_soft_vol_t *me_ptr, int32_t headroom_mB)
{
   capi_err_t result   = AR_EOK;
   uint32_t      max_gain = 0;
   uint32_t      gainQ28[VOLUME_CONTROL_MAX_CHANNELS_V2 + 1];
   uint32_t      max_gain_with_headroom = 0;
   bool_t        headroom_flag          = 0;
   me_ptr->soft_vol_lib.topoReqHeadroom = capi_soft_vol_convert_headroom_linear_gain(headroom_mB);

   /* Comparison below is between the gain applied by the volume module
    * and the headroom applied on 0dB gain (In case the gain applied by
    * the volume module already assures the needed headroom),
    * Determine the max gain amongst all of the channels */
   for (uint8_t i = 0; i < me_ptr->soft_vol_lib.numChannels; i++)
   {
      gainQ28[i] =
         capi_soft_vol_calc_gain_q28(me_ptr->soft_vol_lib.masterGain,
                                        me_ptr->soft_vol_lib.channelGain[me_ptr->soft_vol_lib.channelMapping[i]]);

      if (gainQ28[i] > max_gain)
      {
         max_gain = gainQ28[i];
      }
   }
   uint32_t gainQ28_headroom =
      (int32_t)((((int64_t)me_ptr->soft_vol_lib.topoReqHeadroom) * HEADROOM_GAIN_Q28_MAX) >> 28);

   /* In case gain needed to provide required headroom is less than the
    * worst case/max gain of individual channels, it will be applied on
    * all of the channels in the ratio of the (max_gain - headroom)
    * to the max_gain */
   if (max_gain > gainQ28_headroom)
   {
      SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO,
             "Volume adjustment is required as headroom is less than volume. Max gain: %lu, Gain for headroom: %lu",
             max_gain,
             gainQ28_headroom);
      max_gain_with_headroom = gainQ28_headroom;
      headroom_flag          = 1;
   }

   if ((headroom_flag == 1) && (0 != headroom_mB))
   {
      for (uint8_t i = 0; i < me_ptr->soft_vol_lib.numChannels; i++)
      {
         SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO,
                "Adjusted gains: Original gain: %lu Gain for headroom: %lu Max: %lu",
                gainQ28[i],
                max_gain_with_headroom,
                max_gain);
         gainQ28[i] = (uint32_t)((gainQ28[i] * (uint64_t)max_gain_with_headroom) / (uint64_t)max_gain);
         SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO, "The new gain value: %lu", gainQ28[i]);
      }
   }
   for (uint8_t i = 0; i < me_ptr->soft_vol_lib.numChannels; i++)
   {
      me_ptr->SoftVolumeControlsLib
         .SetVolume(gainQ28[i], me_ptr->soft_vol_lib.pPerChannelData[me_ptr->soft_vol_lib.channelMapping[i]]);
   }
   return result;
}

void capi_soft_vol_set_sample_rate(capi_soft_vol_t *me_ptr, const uint32_t sample_rate)
{
   uint32_t old_sample_rate = me_ptr->SoftVolumeControlsLib.GetSampleRate();

   for (uint32_t i = MIN_CHANNEL_TYPE; i <= MAX_CHANNEL_TYPE; i++)
   {
      me_ptr->SoftVolumeControlsLib.SetSampleRate(old_sample_rate,
                                                  sample_rate,
                                                  me_ptr->soft_vol_lib.pPerChannelData[i]);
   }
}

void capi_soft_vol_send_gain_over_imcl(capi_soft_vol_t *me_ptr)
{
   capi_err_t     result = CAPI_EOK;
   capi_buf_t     buf;
   uint32_t          control_port_id = 0;
   imcl_port_state_t port_state      = CTRL_PORT_CLOSE;
   ctrl_port_data_t *port_data_ptr   = NULL;
   buf.actual_data_len               = sizeof(imc_param_header_t) + sizeof(param_id_imcl_gain_t) +
                         me_ptr->soft_vol_lib.numChannels * sizeof(imcl_gain_config_t);
   buf.data_ptr     = NULL;
   buf.max_data_len = 0;
   imcl_outgoing_data_flag_t flags;
   flags.should_send = TRUE;
   flags.is_trigger  = FALSE;

   // Get the firt control port id for the intent #INTENT_ID_GAIN_INFO
   capi_cmn_ctrl_port_list_get_next_port_data(&me_ptr->ctrl_port_info,
                                                 INTENT_ID_GAIN_INFO,
                                                 control_port_id, // initially, an invalid port id
                                                 &port_data_ptr);

   if (port_data_ptr)
   {
      control_port_id = port_data_ptr->port_info.port_id;
      port_state      = port_data_ptr->state;
   }

   while (port_data_ptr && control_port_id && (CTRL_PORT_PEER_CONNECTED == port_state))
   {
      // Get one time buf from the queue.
      result |= capi_cmn_imcl_get_one_time_buf(&me_ptr->cb_info, control_port_id, buf.actual_data_len, &buf);
      if (CAPI_FAILED(result) || (NULL == buf.data_ptr))
      {
         SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Getting one time imcl buffer failed");
         return;
      }

      imc_param_header_t *  out_cfg_ptr  = (imc_param_header_t *)buf.data_ptr;
      param_id_imcl_gain_t *gain_payload = (param_id_imcl_gain_t *)(out_cfg_ptr + 1);

      capi_soft_vol_get_target_gain_config(me_ptr, gain_payload);

      out_cfg_ptr->opcode = PARAM_ID_IMCL_TARGET_GAIN;
      out_cfg_ptr->actual_data_len =
         sizeof(param_id_imcl_gain_t) + gain_payload->num_config * sizeof(imcl_gain_config_t);

      // Send data ready to the peer module
      if (CAPI_SUCCEEDED(capi_cmn_imcl_send_to_peer(&me_ptr->cb_info, &buf, control_port_id, flags)))
      {
         SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Gain 0x%x sent to control port 0x%x", gain_payload->gain_data[0].gain, control_port_id);
      }

      // Get the next control port id for the intent #INTENT_ID_GAIN_INFO
      capi_cmn_ctrl_port_list_get_next_port_data(&me_ptr->ctrl_port_info,
                                                    INTENT_ID_GAIN_INFO,
                                                    control_port_id, // initially, an invalid port id
                                                    &port_data_ptr);

      if (port_data_ptr)
      {
         control_port_id = port_data_ptr->port_info.port_id;
         port_state      = port_data_ptr->state;
      }
   }
}

#ifdef DO_SOFT_VOL_PROFILING
void capi_soft_vol_print_kpps(capi_soft_vol_t *me_ptr)
{
   soft_vol_prof_info_t *soft_vol_kpps_data = &me_ptr->soft_vol_kpps_data;

   soft_vol_kpps_data->average_kpps =
      ((soft_vol_kpps_data->total_cycles / soft_vol_kpps_data->sample_count) * soft_vol_kpps_data->sample_rate) /
      (1000);

   SOFT_VOL_MSG(me_ptr->miid,
         DBG_HIGH_PRIO,
         "Total cycles :%llu, Total KPPS: %llu, "
         "Total Sample Count: %llu, Peak KPPS : %llu",
         soft_vol_kpps_data->total_cycles,
         soft_vol_kpps_data->average_kpps,
         soft_vol_kpps_data->sample_count,
         soft_vol_kpps_data->peak_kpps);

   soft_vol_kpps_data->total_cycles = 0;
   soft_vol_kpps_data->peak_kpps    = 0;
   soft_vol_kpps_data->sample_count = 0;
   soft_vol_kpps_data->average_kpps = 0;
}

void capi_soft_vol_profiling(capi_soft_vol_t *me_ptr, uint32_t num_samples)
{
   soft_vol_prof_info_t *soft_vol_kpps_data = &me_ptr->soft_vol_kpps_data;

   soft_vol_kpps_data->total_cycles =
      soft_vol_kpps_data->total_cycles + (soft_vol_kpps_data->end_cycles - soft_vol_kpps_data->start_cycles);

   soft_vol_kpps_data->sample_count += num_samples;

   uint64_t frame_kpps = soft_vol_kpps_data->end_cycles - soft_vol_kpps_data->start_cycles;
   frame_kpps          = frame_kpps / num_samples;
   frame_kpps          = frame_kpps * soft_vol_kpps_data->sample_rate / 1000;

   if (frame_kpps > soft_vol_kpps_data->peak_kpps)
   {
      soft_vol_kpps_data->peak_kpps = frame_kpps;
   }
}
#endif /* DO_SOFT_VOL_PROFILING */
