/* =========================================================================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 * @file capi_popless_equalizer_utils.cpp
 *
 * Common Audio Processor Interface Utility functions for Popless Equalizer.
 */

#include "capi.h"
#include "capi_cmn.h"
#include "ar_defs.h"
#include "audpp_util.h"
#include "equalizer_api.h"
#include "equalizer_calibration_api.h"
#include "capi_popless_equalizer_utils.h"
#include "popless_equalizer_api.h"
#include "capi_cmn_imcl_utils.h"

//#define PEQ_DBG 1

static capi_err_t capi_p_eq_update_headroom_event(capi_p_eq_t *me_ptr, uint32 headroom)
{
#ifdef PEQ_DBG
   P_EQ_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI P_EQ: calling update headroom event");
#endif
   capi_err_t     capi_result = CAPI_EOK;
   capi_buf_t     buf;
   uint32_t          control_port_id = 0;
   imcl_port_state_t port_state      = CTRL_PORT_CLOSE;
   ctrl_port_data_t *port_data_ptr   = NULL;
   buf.actual_data_len               = sizeof(vol_imcl_header_t) + sizeof(p_eq_vol_headroom_param_t);
   buf.data_ptr                      = NULL;
   buf.max_data_len                  = 0;
   imcl_outgoing_data_flag_t flags;
   flags.should_send = TRUE;
   flags.is_trigger  = TRUE;

   // Get the firt control port id for the intent #INTENT_ID_GAIN_INFO
   capi_cmn_ctrl_port_list_get_next_port_data(&me_ptr->ctrl_port_info,
                                                 INTENT_ID_P_EQ_VOL_HEADROOM,
                                                 control_port_id, // initially, an invalid port id
                                                 &port_data_ptr);

   if (port_data_ptr)
   {
      control_port_id = port_data_ptr->port_info.port_id;
      port_state      = port_data_ptr->state;
   }

#ifdef PEQ_DBG
   P_EQ_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI P_EQ: calling update headroom event got port id = 0x%x", control_port_id);
#endif
   if (CTRL_PORT_PEER_CONNECTED == port_state)
   {
      // Get one time buf from the queue.
      capi_result |= capi_cmn_imcl_get_one_time_buf(&me_ptr->cb_info, control_port_id, buf.actual_data_len, &buf);
      if (CAPI_FAILED(capi_result) || (NULL == buf.data_ptr))
      {
         P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Getting one time imcl buffer failed");
         return capi_result;
      }
      vol_imcl_header_t *        out_cfg_ptr      = (vol_imcl_header_t *)buf.data_ptr;
      p_eq_vol_headroom_param_t *headroom_payload = (p_eq_vol_headroom_param_t *)(out_cfg_ptr + 1);

      out_cfg_ptr->opcode                     = PARAM_ID_P_EQ_VOL_HEADROOM;
      out_cfg_ptr->actual_data_len            = sizeof(p_eq_vol_headroom_param_t);
      headroom_payload->headroom_in_millibels = headroom;
      // Send data ready to the peer module
      if (CAPI_SUCCEEDED(capi_cmn_imcl_send_to_peer(&me_ptr->cb_info, &buf, control_port_id, flags)))
      {
         P_EQ_MSG(me_ptr->miid, DBG_HIGH_PRIO,
                "Headroom 0x%x sent to control port 0x%x",
                headroom_payload->headroom_in_millibels,
                control_port_id);
      }
   }
   return capi_result;
}

capi_err_t capi_p_eq_update_process_check(capi_p_eq_t *me_ptr, uint32 process_check)
{
#ifdef PEQ_DBG
   P_EQ_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI P_EQ: Update process check event called ");
#endif
   capi_err_t capi_result    = CAPI_EOK;
   uint32_t      process_check_old = me_ptr->process_check;
   if (process_check_old != process_check)
   {
      if (1 == me_ptr->pending_disable_event)
      {
         capi_result = capi_cmn_update_process_check_event(&me_ptr->cb_info, process_check);
         me_ptr->pending_disable_event = 0;
      }
      else
      {
         me_ptr->pending_disable_event = 2;
      }
      me_ptr->process_check = process_check;
      if (0 == process_check)
      {
         capi_p_eq_update_headroom_event(me_ptr, 0);
         /* TODO: Check if update is needed */
         /* capi_p_eq_update_delay_event(me_ptr, 0); */
      }
      else if (P_EQ_WAITING_FOR_MEDIA_FORMAT != me_ptr->p_eq_state)
      {
         capi_p_eq_update_headroom(me_ptr);
      }
   }
   return capi_result;
}

void capi_p_eq_update_delay(capi_p_eq_t *me_ptr)
{
   EQ_RESULT     lib_result;
   capi_err_t capi_result  = CAPI_EOK;
   eq_num_band_t eq_num_bands    = 0;
   uint32_t      lib_actual_size = 0;

   lib_result = eq_get_param(&(me_ptr->lib_instances[0][CUR_INST]),
                             EQ_PARAM_GET_NUM_BAND,
                             (int8_t *)&eq_num_bands,
                             sizeof(eq_num_band_t),
                             &lib_actual_size);
   if ((EQ_SUCCESS != lib_result) || (0 == lib_actual_size))
   {
      P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: get delay failed");
      eq_num_bands = 0;
   }
   /* each MSIIR stage contains two delay units */
   uint32_t delay_in_us = 2 * eq_num_bands * uint64_t(1000000);
   delay_in_us /= me_ptr->lib_static_vars.sample_rate;
   capi_result = capi_cmn_update_algo_delay_event(&me_ptr->cb_info, delay_in_us);
}

capi_err_t capi_p_eq_init_media_fmt(capi_p_eq_t *me_ptr)
{
   capi_media_fmt_v2_t *media_fmt_ptr = &(me_ptr->input_media_fmt);

   media_fmt_ptr->header.format_header.data_format = CAPI_FIXED_POINT;
   media_fmt_ptr->format.bits_per_sample           = 16;
   media_fmt_ptr->format.bitstream_format          = CAPI_DATA_FORMAT_INVALID_VAL;
   media_fmt_ptr->format.data_interleaving         = CAPI_DEINTERLEAVED_UNPACKED;
   media_fmt_ptr->format.data_is_signed            = 1;
   media_fmt_ptr->format.num_channels              = CAPI_DATA_FORMAT_INVALID_VAL;
   media_fmt_ptr->format.q_factor                  = PCM_Q_FACTOR_15;
   media_fmt_ptr->format.sampling_rate             = CAPI_DATA_FORMAT_INVALID_VAL;

   for (uint32_t j = 0; (j < CAPI_MAX_CHANNELS_V2); j++)
   {
      media_fmt_ptr->format.channel_type[j] = (uint16_t)CAPI_DATA_FORMAT_INVALID_VAL;
   }
   return CAPI_EOK;
}

/**
 * Function returns the size of api payload and internal api to eq
 * lib for apis which are acceptable to p_eq module.
 * @param[in]  *me_ptr     P_EQ capi structure
 * @param[in]   param_id   Input parameter id to p_eq
 * @param[out] *param_size size of this api payload
 * @param[out] *eq_lib_api eq library api
 *
 * @return      Returns the accepted or rejected api
 */
capi_err_t get_eq_lib_api_info(uint32_t param_id, uint32_t *param_size, uint32_t *eq_lib_api)
{
   capi_err_t capi_result = CAPI_EOK;
   *eq_lib_api                  = (uint32_t)0xFFFFFFFF;
   switch (param_id)
   {
      case PARAM_ID_EQ_SINGLE_BAND_FREQ:
         *param_size = sizeof(param_id_eq_single_band_freq_t);
         break;

      case PARAM_ID_EQ_CONFIG:
         *param_size = sizeof(param_id_eq_config_t); /* TODO: Calculate this later on, for now keeping min size */
         break;

      case PARAM_ID_MODULE_ENABLE:
         *param_size = sizeof(param_id_module_enable_t);
         break;

      case PARAM_ID_EQ_NUM_BANDS:
         *param_size = sizeof(param_id_eq_num_bands_t);
         *eq_lib_api = EQ_PARAM_GET_NUM_BAND;
         break;

      case PARAM_ID_EQ_BAND_LEVELS:
         *param_size = sizeof(param_id_eq_band_levels_t);
         *eq_lib_api = EQ_PARAM_GET_BAND_LEVELS;
         break;

      case PARAM_ID_EQ_BAND_LEVEL_RANGE:
         *param_size = sizeof(param_id_eq_band_level_range_t);
         *eq_lib_api = EQ_PARAM_GET_BAND_LEVEL_RANGE;
         break;

      case PARAM_ID_EQ_BAND_FREQS:
         *param_size = sizeof(param_id_eq_band_freqs_t);
         *eq_lib_api = EQ_PARAM_GET_BAND_FREQS;
         break;

      case PARAM_ID_EQ_SINGLE_BAND_FREQ_RANGE:
         *param_size = sizeof(param_id_eq_single_band_freq_range_t);
         *eq_lib_api = EQ_PARAM_GET_BAND_FREQ_RANGE;
         break;

      case PARAM_ID_EQ_BAND_INDEX:
         *param_size = sizeof(param_id_eq_band_index_t);
         *eq_lib_api = EQ_PARAM_GET_BAND;
         break;

      case PARAM_ID_EQ_PRESET_ID:
         *param_size = sizeof(param_id_eq_preset_id_t);
         *eq_lib_api = EQ_PARAM_GET_PRESET_ID;
         break;

      case PARAM_ID_EQ_NUM_PRESETS:
         *param_size = sizeof(param_id_eq_num_presets_t);
         *eq_lib_api = EQ_PARAM_GET_NUM_PRESETS;
         break;

      case PARAM_ID_EQ_PRESET_NAME:
         *param_size = sizeof(param_id_eq_preset_name_t);
         *eq_lib_api = EQ_PARAM_GET_PRESET_NAME;
         break;

      case INTF_EXTN_PARAM_ID_IMCL_PORT_OPERATION:
         *param_size = sizeof(intf_extn_param_id_imcl_port_operation_t);
         break;

      default:
         capi_result = CAPI_EUNSUPPORTED;
         break;
   }
   return capi_result;
}

capi_err_t capi_p_eq_set_algorithmic_reset(capi_p_eq_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   EQ_RESULT     lib_result     = EQ_SUCCESS;
   for (int32_t ch = 0; ch < (int)me_ptr->num_channels; ch++)
   {
      if (NULL != me_ptr->lib_instances[ch][CUR_INST].lib_mem_ptr)
      {
         lib_result = eq_set_param(&(me_ptr->lib_instances[ch][CUR_INST]), EQ_PARAM_SET_RESET, NULL, 0);
         if (EQ_SUCCESS != lib_result)
         {
            capi_result = CAPI_EFAILED;
            break;
         }
      }
   }
   return capi_result;
}

bool_t capi_p_eq_is_supported_media_type(capi_media_fmt_v2_t *format_ptr)
{
	  
   if ((BITS_PER_SAMPLE_16 != format_ptr->format.bits_per_sample) &&
       (BITS_PER_SAMPLE_32 != format_ptr->format.bits_per_sample))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "CAPI P_EQ:: Only 16/32 bit data supported. Received %lu.",format_ptr->format.bits_per_sample);
      return FALSE;
   }

   // If sample size is 16bits then Q-Format must be 15 or If sample size is 32bits then Q-Format must be 27.
   if(((BITS_PER_SAMPLE_16 == format_ptr->format.bits_per_sample) && (PCM_Q_FACTOR_15 != format_ptr->format.q_factor)) ||
      ((BITS_PER_SAMPLE_32 == format_ptr->format.bits_per_sample) && (PCM_Q_FACTOR_27 != format_ptr->format.q_factor)))
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI P_EQ:: Unsupported data format: bits_per_sample = %lu , Q factor = %lu",format_ptr->format.bits_per_sample, format_ptr->format.q_factor);
      return FALSE;
   }

   if (CAPI_DEINTERLEAVED_UNPACKED != format_ptr->format.data_interleaving)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI P_EQ:: Interleaved data not supported.");
      return FALSE;
   }

   if ((SAMPLE_RATE_192K < format_ptr->format.sampling_rate) | (0 >= format_ptr->format.sampling_rate))
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI P_EQ:: Sampling rate not supported.");
      return FALSE;
   }

   if (!format_ptr->format.data_is_signed)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI P_EQ:: Unsigned data not supported.");
      return FALSE;
   }

   if ((0 == format_ptr->format.num_channels) | (format_ptr->format.num_channels > EQUALIZER_MAX_CHANNELS))
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI P_EQ: unsupported number of channels %lu.", format_ptr->format.num_channels);
      return FALSE;
   }
   
   return TRUE;
}

capi_err_t capi_p_eq_reinit_eq(capi_p_eq_t *me_ptr, capi_media_fmt_v2_t *data_ptr)
{
   capi_p_eq_free_eq_instance(me_ptr, NEW_INST);
   me_ptr->p_eq_state                    = P_EQ_DISABLE;
   me_ptr->transition_num_samples        = 0;
   me_ptr->transition_num_samples_preset = 0;
   me_ptr->is_new_config_pending         = 0;
   me_ptr->volume_ramp                   = 0;

   /* if media format has changed,
    * recalculate library memory size and init lib */
   if ((me_ptr->num_channels != data_ptr->format.num_channels) ||
       (me_ptr->lib_static_vars.sample_rate != data_ptr->format.sampling_rate) ||
       (me_ptr->lib_static_vars.data_width != data_ptr->format.bits_per_sample))
   {
      me_ptr->lib_static_vars.sample_rate = data_ptr->format.sampling_rate;
      me_ptr->lib_static_vars.data_width  = data_ptr->format.bits_per_sample;

      capi_err_t capi_result = equalizer_adjust_lib_size_reinit(me_ptr, data_ptr->format.num_channels);
      if (CAPI_EOK != capi_result)
      {
         P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: ReInit failed, lib size");
         return capi_result;
      }

      me_ptr->code_bw = 0;

      /* Honest BW based on channels and Sampling rate independently */
      me_ptr->data_bw = P_EQ_BW_LOW;
      if ((data_ptr->format.sampling_rate <= 48000) && (data_ptr->format.num_channels > 2))
      {
         me_ptr->data_bw = P_EQ_BW_CH_GREATER_THAN_2;
      }
      else if ((data_ptr->format.sampling_rate > 48000) && (data_ptr->format.num_channels <= 2))
      {
         me_ptr->data_bw = P_EQ_BW_FS_GREATER_THAN_48k;
      }
      else if ((data_ptr->format.sampling_rate > 48000) && (data_ptr->format.num_channels > 2))
      {
         me_ptr->data_bw = P_EQ_BW_HIGHEST;
      }
      uint32_t kpps = capi_p_eq_get_kpps(me_ptr);
      capi_result |= capi_cmn_update_kpps_event(&me_ptr->cb_info, kpps);
      capi_result |= capi_cmn_update_bandwidth_event(&me_ptr->cb_info, me_ptr->code_bw, me_ptr->data_bw);
   }

   /* TODO: Check for crossfading for reset */
   if (me_ptr->enable_flag)
      me_ptr->p_eq_state = P_EQ_VOL_CTRL;

   for (int32_t ch = 0; ch < (int)me_ptr->num_channels; ch++)
   {
      max_eq_lib_cfg_t *max_eq_cfg_ptr = &(me_ptr->max_eq_cfg);

      eq_set_param(&(me_ptr->lib_instances[ch][CUR_INST]),
                   EQ_PARAM_SET_CONFIG,
                   (int8 *)max_eq_cfg_ptr,
                   (uint32)sizeof(me_ptr->max_eq_cfg));

      eq_set_param(&(me_ptr->lib_instances[ch][CUR_INST]), EQ_PARAM_SET_RESET, NULL, 0);
   }
   return CAPI_EOK;
}

capi_err_t capi_p_eq_init_eq_first_time(capi_p_eq_t *me_ptr, capi_media_fmt_v2_t *data_ptr)
{
   me_ptr->num_channels                  = data_ptr->format.num_channels;
   me_ptr->p_eq_state                    = P_EQ_DISABLE;
   me_ptr->transition_num_samples        = 0;
   me_ptr->transition_num_samples_preset = 0;
   me_ptr->is_new_config_pending         = 0;
   me_ptr->volume_ramp                   = 0;

   me_ptr->lib_static_vars.data_width  = data_ptr->format.bits_per_sample;
   me_ptr->lib_static_vars.sample_rate = data_ptr->format.sampling_rate;

   capi_err_t capi_result = capi_p_eq_create_new_equalizers(me_ptr, CUR_INST);
   if (CAPI_EOK != capi_result)
   {
      P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: Init failed");
      return capi_result;
   }

   /* TODO: Check for crossfading for reset and club with reinit */
   if (me_ptr->enable_flag)
      me_ptr->p_eq_state = P_EQ_VOL_CTRL;

   for (int32_t ch = 0; ch < (int)me_ptr->num_channels; ch++)
   {
      max_eq_lib_cfg_t *max_eq_cfg_ptr = &(me_ptr->max_eq_cfg);

      eq_set_param(&(me_ptr->lib_instances[ch][CUR_INST]),
                   EQ_PARAM_SET_CONFIG,
                   (int8 *)max_eq_cfg_ptr,
                   (uint32)sizeof(me_ptr->max_eq_cfg));

      eq_set_param(&(me_ptr->lib_instances[ch][CUR_INST]), EQ_PARAM_SET_RESET, NULL, 0);
   }
   me_ptr->code_bw = 0;

   /* Honest BW based on channels and Sampling rate independently */
   me_ptr->data_bw = P_EQ_BW_LOW;
   if ((data_ptr->format.sampling_rate <= 48000) && (data_ptr->format.num_channels > 2))
   {
      me_ptr->data_bw = P_EQ_BW_CH_GREATER_THAN_2;
   }
   else if ((data_ptr->format.sampling_rate > 48000) && (data_ptr->format.num_channels <= 2))
   {
      me_ptr->data_bw = P_EQ_BW_FS_GREATER_THAN_48k;
   }
   else if ((data_ptr->format.sampling_rate > 48000) && (data_ptr->format.num_channels > 2))
   {
      me_ptr->data_bw = P_EQ_BW_HIGHEST;
   }

   uint32_t kpps = capi_p_eq_get_kpps(me_ptr);
   capi_result |= capi_cmn_update_kpps_event(&me_ptr->cb_info, kpps);
   capi_result |= capi_cmn_update_bandwidth_event(&me_ptr->cb_info, me_ptr->code_bw, me_ptr->data_bw);
   return capi_result;
}

static bool_t is_equal_media_format(const capi_standard_data_format_v2_t *f1,
                                    const capi_standard_data_format_v2_t *f2)
{
   bool_t is_equal = TRUE;

   is_equal = is_equal && (f1->bits_per_sample == f2->bits_per_sample);
   is_equal = is_equal && (f1->bitstream_format == f2->bitstream_format);
   is_equal = is_equal && (f1->data_interleaving == f2->data_interleaving);
   is_equal = is_equal && (f1->data_is_signed == f2->data_is_signed);
   is_equal = is_equal && (f1->num_channels == f2->num_channels);
   is_equal = is_equal && (f1->q_factor == f2->q_factor);
   is_equal = is_equal && (f1->sampling_rate == f2->sampling_rate);

   if (is_equal)
   {
      for (uint32_t i = 0; i < f1->num_channels; i++)
      {
         is_equal = is_equal && (f1->channel_type[i] == f2->channel_type[i]);
      }
   }

   return is_equal;
}

capi_err_t capi_p_eq_set_input_media_format(capi_p_eq_t *me_ptr, capi_buf_t *prop_payload_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (sizeof(capi_set_get_media_format_t) > prop_payload_ptr->actual_data_len)
   {
      P_EQ_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI P_EQ: received bad payload size");
      return CAPI_EBADPARAM;
   }
   capi_media_fmt_v2_t *data_ptr = (capi_media_fmt_v2_t *)prop_payload_ptr->data_ptr;

   if (!capi_p_eq_is_supported_media_type(data_ptr))
      return CAPI_EUNSUPPORTED;

   if (!is_equal_media_format(&me_ptr->input_media_fmt.format, &data_ptr->format))
   {
	  uint32_t req_size = sizeof(capi_set_get_media_format_t) + sizeof(capi_standard_data_format_v2_t) +
			  	  	  	  	 data_ptr->format.num_channels * sizeof(uint16_t) ;
      memscpy(&me_ptr->input_media_fmt, req_size, data_ptr, prop_payload_ptr->max_data_len);
   }
   else
   {
      P_EQ_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI P_EQ: media format unchanged");
      return CAPI_EOK;
   }



   if (P_EQ_WAITING_FOR_MEDIA_FORMAT == me_ptr->p_eq_state)
   {
      capi_result = capi_p_eq_init_eq_first_time(me_ptr, data_ptr);
   }
   else
   {
      capi_result = capi_p_eq_reinit_eq(me_ptr, data_ptr);
   }

   if (P_EQ_DISABLE != me_ptr->p_eq_state)
   {
      capi_p_eq_update_headroom(me_ptr);
      capi_p_eq_update_delay(me_ptr);
   }
   if (me_ptr->enable_flag)
   {
      capi_p_eq_update_process_check(me_ptr, me_ptr->enable_flag);
   }
   capi_cmn_output_media_fmt_event_v2(&me_ptr->cb_info, &me_ptr->input_media_fmt, FALSE, 0);
   return capi_result;
}

capi_err_t capi_p_eq_process_set_properties(capi_p_eq_t *me_ptr, capi_proplist_t *proplist_ptr)
{
   capi_err_t   capi_result  = CAPI_EOK;
   capi_err_t   capi_result2 = CAPI_EOK;
   capi_prop_t *prop_array      = proplist_ptr->prop_ptr;
   uint32_t        i               = 0;
   
   capi_result = capi_cmn_set_basic_properties(proplist_ptr, &me_ptr->heap_info, &me_ptr->cb_info, TRUE);
   if (CAPI_EOK != capi_result)
   {
      P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: Set basic properties failed with result %lu", capi_result);
      return capi_result; // TODO: continue to process
   }

   for (i = 0; i < proplist_ptr->props_num; i++)
   {
      capi_buf_t *payload_ptr = &(prop_array[i].payload);

      switch (prop_array[i].id)
      {
         case CAPI_EVENT_CALLBACK_INFO:
         case CAPI_PORT_NUM_INFO:
         case CAPI_HEAP_ID:
         case CAPI_CUSTOM_INIT_DATA:
         {
            break;
         }
         case CAPI_ALGORITHMIC_RESET:
         {
            P_EQ_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI P_EQ: reset");
            if (P_EQ_WAITING_FOR_MEDIA_FORMAT != me_ptr->p_eq_state)
            {
               capi_result |= capi_p_eq_set_algorithmic_reset(me_ptr);
            }
            break;
         }
         case CAPI_INPUT_MEDIA_FORMAT_V2:
         {
            P_EQ_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI P_EQ received input media fmt");
            capi_result |= capi_p_eq_set_input_media_format(me_ptr, payload_ptr);
            break;
         }
         case CAPI_OUTPUT_MEDIA_FORMAT_V2:
         {
            capi_result |= CAPI_EUNSUPPORTED;
            break;
         }
		 
		 case CAPI_MODULE_INSTANCE_ID:
         {
              if (payload_ptr->actual_data_len >= sizeof(capi_module_instance_id_t))
              {
                 capi_module_instance_id_t *data_ptr = (capi_module_instance_id_t *)payload_ptr->data_ptr;
                 me_ptr->miid                        = data_ptr->module_instance_id;
  #if 0
              AR_MSG(DBG_LOW_PRIO,
                         "CAPI P_EQ: This module-id 0x%08lX, instance-id 0x%08lX",
                         data_ptr->module_id,
                         me_ptr->miid);
  #endif
              }
              else
              {
                 AR_MSG(DBG_ERROR_PRIO,
                         "CAPI P_EQ: Set, Param id 0x%lx Bad param size %lu",
                         (uint32_t)prop_array[i].id,
                         payload_ptr->actual_data_len);
                 CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
              }
              break;
         }
		 
         default:
         {
            capi_result |= CAPI_EUNSUPPORTED;
         }
      } /* switch (prop_array[i].id) */
      CAPI_SET_ERROR(capi_result2, capi_result);
      if (CAPI_FAILED(capi_result2))
      {
         P_EQ_MSG(me_ptr->miid, DBG_HIGH_PRIO,
                "CAPI P_EQ: Set property for"
                " %#x failed with opcode %lu",
                prop_array[i].id,
                capi_result);
      }
      P_EQ_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI P_EQ: Set property for 0x%x done", prop_array[i].id);
   }
   return capi_result;
}

capi_err_t capi_p_eq_process_get_properties(capi_p_eq_t *me_ptr, capi_proplist_t *proplist_ptr)
{
   uint32_t             i;
   capi_err_t        capi_result  = CAPI_EOK;
   capi_err_t        capi_result2 = CAPI_EOK;
   capi_prop_t *     prop_array      = proplist_ptr->prop_ptr;
   capi_basic_prop_t mod_prop;

   mod_prop.init_memory_req    = align_to_8_byte(sizeof(capi_p_eq_t));
   mod_prop.stack_size         = P_EQ_STACK_SIZE;
   mod_prop.num_fwk_extns      = 0;
   mod_prop.fwk_extn_ids_arr   = NULL;
   mod_prop.is_inplace         = FALSE;
   mod_prop.req_data_buffering = FALSE;
   mod_prop.max_metadata_size  = 0;
   
   uint32_t miid = MIID_UNKNOWN; 
   if(me_ptr) 
   { 
		miid = me_ptr->miid;
   }
   
   capi_result = capi_cmn_get_basic_properties(proplist_ptr, &mod_prop);

   if (CAPI_EOK != capi_result)
   {
      P_EQ_MSG(miid, DBG_ERROR_PRIO, "CAPI P_EQ: Get common basic properties failed with result %lu", capi_result);
      return capi_result;
   }

   for (i = 0; i < proplist_ptr->props_num; i++)
   {
      capi_buf_t *payload_ptr          = &(prop_array[i].payload);
      const uint32_t payload_max_data_len = payload_ptr->max_data_len;
      const uint32_t param_id             = prop_array[i].id;

      switch (prop_array[i].id)
      {
         case CAPI_INIT_MEMORY_REQUIREMENT:
         case CAPI_STACK_SIZE:
         case CAPI_MAX_METADATA_SIZE:
         case CAPI_IS_INPLACE:
         case CAPI_REQUIRES_DATA_BUFFERING:
         case CAPI_NUM_NEEDED_FRAMEWORK_EXTENSIONS:
         {
            break;
         }
         case CAPI_OUTPUT_MEDIA_FORMAT_SIZE:
         {
            if (payload_max_data_len >= sizeof(capi_output_media_format_size_t))
            {
               capi_output_media_format_size_t *data_ptr =
                  (capi_output_media_format_size_t *)payload_ptr->data_ptr;
               data_ptr->size_in_bytes      = sizeof(capi_standard_data_format_v2_t);
               payload_ptr->actual_data_len = sizeof(capi_output_media_format_size_t);
            }
            else
            {
               P_EQ_MSG(miid, DBG_ERROR_PRIO,
                      "CAPI P_EQ: Get property id 0x%lx Bad param size %lu",
                      param_id,
                      payload_max_data_len);
               CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
            }
            break;
         }

         case CAPI_OUTPUT_MEDIA_FORMAT_V2:
         {
            if (payload_max_data_len >= sizeof(capi_media_fmt_v2_t))
            {
               capi_media_fmt_v2_t *data_ptr = (capi_media_fmt_v2_t *)payload_ptr->data_ptr;

               if ((NULL == me_ptr) ||
                   ((0 != prop_array[i].port_info.is_valid) || (prop_array[i].port_info.port_index)))
               {
                  P_EQ_MSG(miid, DBG_ERROR_PRIO,
                         "CAPI P_EQ: Get property id 0x%lx failed due to invalid/unexpected values",
                         param_id);
                  CAPI_SET_ERROR(capi_result, CAPI_EFAILED);
                  break;
               }

               memscpy(data_ptr,
                       sizeof(capi_media_fmt_v2_t),
                       &me_ptr->input_media_fmt,
                       sizeof(capi_media_fmt_v2_t));
               payload_ptr->actual_data_len = sizeof(capi_media_fmt_v2_t);
            }
            else
            {
               payload_ptr->actual_data_len = 0;
               P_EQ_MSG(miid, DBG_ERROR_PRIO,
                      "CAPI P_EQ: Get property id 0x%lx Bad param size %lu",
                      param_id,
                      payload_max_data_len);
               CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
            }
            break;
         }

         case CAPI_PORT_DATA_THRESHOLD:
         {
            if (payload_ptr->max_data_len >= sizeof(capi_port_data_threshold_t))
            {
               capi_port_data_threshold_t *data_ptr = (capi_port_data_threshold_t *)payload_ptr->data_ptr;
               if (!prop_array[i].port_info.is_valid)
               {
                  P_EQ_MSG(miid, DBG_ERROR_PRIO, "CAPI P_EQ Get, port id not valid");
                  capi_result |= CAPI_EBADPARAM;
                  break;
               }
               if (0 != prop_array[i].port_info.port_index)
               {
                  P_EQ_MSG(miid, DBG_ERROR_PRIO,
                         "CAPI P_EQ Get, Param id 0x%lx max in/out port is 1. asking for %lu",
                         (uint32_t)prop_array[i].id,
                         prop_array[i].port_info.port_index);
                  capi_result |= CAPI_EBADPARAM;
                  break;
               }
               data_ptr->threshold_in_bytes = 0;
               payload_ptr->actual_data_len = sizeof(capi_port_data_threshold_t);
            }
            else
            {
               P_EQ_MSG(miid, DBG_ERROR_PRIO,
                      "CAPI P_EQ: Get, Param id 0x%lx Bad param size %lu",
                      (uint32_t)prop_array[i].id,
                      payload_ptr->max_data_len);
               payload_ptr->actual_data_len =
                  0; // according to CAPI V2, actual len should be num of bytes read(set)/written(get)
               capi_result |= CAPI_ENEEDMORE;
            }
            break;
         }
         case CAPI_INTERFACE_EXTENSIONS: // added this
         {
            if (payload_ptr->max_data_len >= sizeof(capi_interface_extns_list_t))
            {
               capi_interface_extns_list_t *intf_ext_list = (capi_interface_extns_list_t *)payload_ptr->data_ptr;
               if (payload_ptr->max_data_len <
                   (sizeof(capi_interface_extns_list_t) +
                    (intf_ext_list->num_extensions * sizeof(capi_interface_extn_desc_t))))
               {
                  P_EQ_MSG(miid, DBG_ERROR_PRIO,
                         "CAPI P_EQ: CAPI_INTERFACE_EXTENSIONS invalid param size %lu",
                         payload_ptr->max_data_len);
                  CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
               }
               else
               {
                  capi_interface_extn_desc_t *curr_intf_extn_desc_ptr =
                     (capi_interface_extn_desc_t *)(payload_ptr->data_ptr + sizeof(capi_interface_extns_list_t));

                  for (uint32_t i = 0; i < intf_ext_list->num_extensions; i++)
                  {
                     switch (curr_intf_extn_desc_ptr->id)
                     {
                        case INTF_EXTN_IMCL:
                        {
                           curr_intf_extn_desc_ptr->is_supported = TRUE;
                           break;
                        }
                        default:
                        {
                           curr_intf_extn_desc_ptr->is_supported = FALSE;
                           break;
                        }
                     }
                     P_EQ_MSG(miid, DBG_HIGH_PRIO,
                            "CAPI P_EQ: CAPI_INTERFACE_EXTENSIONS intf_ext = 0x%lx, is_supported = %d",
                            curr_intf_extn_desc_ptr->id,
                            (int)curr_intf_extn_desc_ptr->is_supported);
                     curr_intf_extn_desc_ptr++;
                  }
               }
            }
            else
            {
               P_EQ_MSG(miid, DBG_ERROR_PRIO,
                      "CAPI P_EQ: CAPI_INTERFACE_EXTENSIONS Bad param size %lu",
                      payload_ptr->max_data_len);
               CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
            }
            break;
         }
         default:
         {
            P_EQ_MSG(miid, DBG_HIGH_PRIO, "CAPI P_EQ Get property for %#x. Not supported.", prop_array[i].id);
            capi_result |= CAPI_EUNSUPPORTED;
         }
      } /* switch(prop_array[i].id) */
      CAPI_SET_ERROR(capi_result2, capi_result);
      if (CAPI_FAILED(capi_result))
      {
         P_EQ_MSG(miid, DBG_HIGH_PRIO,
                "CAPI P_EQ: Get property for 0x%x failed with opcode %lu",
                prop_array[i].id,
                capi_result);
      }
      else
      {
         P_EQ_MSG(miid, DBG_LOW_PRIO, "CAPI P_EQ: Get property for 0x%x done", prop_array[i].id);
      }
   }
   return capi_result2;
}

/**
 * Function to Check crossfading of equalizer. If a second EQ
 * instance exists then EQ is crossfading. Single channel query is
 * sufficient since all the channels share same states.
 */
bool_t equalizer_is_cross_fade_active(capi_p_eq_t *me_ptr)
{
   return (NULL != me_ptr->lib_instances[0][NEW_INST].lib_mem_ptr);
}

bool_t is_new_eq_instance_crossfade_finished(capi_p_eq_t *me_ptr)
{
   EQ_RESULT            lib_result;
   eq_cross_fade_mode_t cross_fade_flag      = 0;
   uint32_t             cross_fade_flag_size = 0;

   lib_result = eq_get_param(&(me_ptr->lib_instances[0][NEW_INST]),
                             EQ_PARAM_CROSSFADE_FLAG,
                             (int8_t *)&cross_fade_flag,
                             sizeof(cross_fade_flag),
                             &cross_fade_flag_size);
   if (EQ_SUCCESS != lib_result)
   {
      /*Assuming failed get param as new crossfade finished */
      return TRUE;
   }
   return (0 == cross_fade_flag);
}

capi_err_t capi_p_eq_update_headroom(capi_p_eq_t *me_ptr)
{
   EQ_RESULT         headroom_result;
   eq_headroom_req_t cur_headroom        = 0;
   eq_headroom_req_t new_headroom        = 0;
   uint32_t          headroom_size       = 0;
   me_ptr->volume_ramp                   = 0;
   me_ptr->transition_num_samples_preset = 0;

   if (NULL != me_ptr->lib_instances[0][NEW_INST].lib_mem_ptr)
   {
      headroom_result = eq_get_param(&(me_ptr->lib_instances[0][NEW_INST]),
                                     EQ_PARAM_GET_HEADROOM_REQ,
                                     (int8_t *)&new_headroom,
                                     sizeof(new_headroom),
                                     &headroom_size);
      if (EQ_SUCCESS != headroom_result)
      {
         P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: New Instance Headroom Query failed with %d", (int)headroom_result);
         new_headroom = 0;
      }
   }
   headroom_result = eq_get_param(&(me_ptr->lib_instances[0][CUR_INST]),
                                  EQ_PARAM_GET_HEADROOM_REQ,
                                  (int8_t *)&cur_headroom,
                                  sizeof(cur_headroom),
                                  &headroom_size);
   if (EQ_SUCCESS != headroom_result)
   {
      P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: Cur Instance Headroom Query failed with %d", (int)headroom_result);
      new_headroom = 0;
   }
   me_ptr->popless_eq_headroom = cur_headroom;
   if (cur_headroom < new_headroom)
   {
      me_ptr->popless_eq_headroom = new_headroom;
      me_ptr->volume_ramp         = 1;
   }
   /* TODO: Check for comparison */
   capi_p_eq_update_headroom_event(me_ptr, me_ptr->popless_eq_headroom);
   P_EQ_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI P_EQ: cur_headroom = %lu, new_headroom = %lu", cur_headroom, new_headroom);

   return CAPI_EOK;
}

capi_err_t capi_p_eq_update_eq_state(capi_p_eq_t *me_ptr)
{
   uint32 process_check_ptr = 0;
   uint32 param_size        = 0;
   eq_get_param(&(me_ptr->lib_instances[0][CUR_INST]),
                EQ_PARAM_CHECK_STATE,
                (int8 *)&process_check_ptr,
                sizeof(uint32),
                &param_size);
   if ((0 == process_check_ptr) && (!me_ptr->enable_flag))
   {
      /* EQ has finished internal crossfading and eq disable is received,
       * hence resetting and disabling */
      for (int32_t ch = 0; ch < (int)me_ptr->num_channels; ch++)
      {
         eq_set_param(&(me_ptr->lib_instances[ch][CUR_INST]), EQ_PARAM_SET_RESET, NULL, 0);
      }

      capi_p_eq_update_process_check(me_ptr, me_ptr->enable_flag);

      me_ptr->p_eq_state = P_EQ_DISABLE;
   }
   else if (0 == process_check_ptr)
   {
      /* EQ became enable while finishing on/off crossfade */
      /* TODO: Check if direct P_EQ_OFF_ON_CROSS_FADE is possible */
      me_ptr->p_eq_state = P_EQ_VOL_CTRL;
   }
   return CAPI_EOK;
}

/**
 * Function to free memory of a EQ instance.
 * TODO: limit to number of channels.
 */
void capi_p_eq_free_eq_instance(capi_p_eq_t *me_ptr, EQ_INST inst_id)
{
   for (int32_t ch = 0; ch < (int)EQUALIZER_MAX_CHANNELS; ch++)
   {
      if (NULL != me_ptr->lib_instances[ch][inst_id].lib_mem_ptr)
      {
         posal_memory_free(me_ptr->lib_instances[ch][inst_id].lib_mem_ptr);
         me_ptr->lib_instances[ch][inst_id].lib_mem_ptr = NULL;
      }
   }
}

/**
 * Function to allocate memory of a EQ instance.
 * TODO: Allocation should be 8 byte align to lib instance.
 */
capi_err_t capi_p_eq_malloc_eq_instance(capi_p_eq_t *me_ptr, EQ_INST inst_id)
{
   capi_err_t capi_result = CAPI_EOK;
   void *        ptr;
   uint32_t      ch;

   for (ch = 0; ch < me_ptr->num_channels; ch++)
   {
      ptr = posal_memory_malloc(me_ptr->lib_size, POSAL_HEAP_DEFAULT);
      if (NULL == ptr)
      {
         P_EQ_MSG(me_ptr->miid, DBG_FATAL_PRIO, "CAPI P_EQ: creating new EQ: Out of Memory!");
         capi_result = CAPI_ENOMEMORY;
         break;
      }
      me_ptr->lib_instances[ch][inst_id].lib_mem_ptr = ptr;
   }
   if (CAPI_EOK != capi_result)
      capi_p_eq_free_eq_instance(me_ptr, inst_id);
   return capi_result;
}

/**
 * Function to Init EQ library.
 */
capi_err_t capi_p_eq_init_eq_instance(capi_p_eq_t *me_ptr, EQ_INST inst_id)
{
   EQ_RESULT lib_result = EQ_SUCCESS;
   for (uint32_t ch = 0; ch < me_ptr->num_channels; ch++)
   {
      lib_result = eq_init_memory(&(me_ptr->lib_instances[ch][inst_id]),
                                  &(me_ptr->lib_static_vars),
                                  (int8_t *)(me_ptr->lib_instances[ch][inst_id].lib_mem_ptr),
                                  me_ptr->lib_size);
      if (EQ_SUCCESS != lib_result)
      {
         P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: new EQ Init failed");
         capi_p_eq_free_eq_instance(me_ptr, inst_id);
         return CAPI_EFAILED;
      }
   }
   return CAPI_EOK;
}

/**
 * Function to query size of EQ library.
 */
capi_err_t capi_p_eq_get_eq_lib_size(capi_p_eq_t *me_ptr, uint32_t *malloc_size)
{
   EQ_RESULT                 lib_result = EQ_SUCCESS;
   eq_lib_mem_requirements_t lib_mem_req;
   lib_result = eq_get_mem_req(&lib_mem_req, &(me_ptr->lib_static_vars));
   if (EQ_SUCCESS != lib_result)
   {
      P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: error getting library memory size %d", lib_result);
      return CAPI_EFAILED;
   }
   *malloc_size = align_to_8_byte(lib_mem_req.lib_mem_size);
   return CAPI_EOK;
}

capi_err_t capi_p_eq_create_new_equalizers(capi_p_eq_t *me_ptr, EQ_INST inst_id)
{
   uint32_t malloc_size;

   if (CAPI_EOK != capi_p_eq_get_eq_lib_size(me_ptr, &malloc_size))
      return CAPI_EFAILED;

   me_ptr->lib_size = malloc_size;

   if (CAPI_EOK != capi_p_eq_malloc_eq_instance(me_ptr, inst_id))
   {
      return CAPI_ENOMEMORY;
   }
   if (CAPI_EOK != capi_p_eq_init_eq_instance(me_ptr, inst_id))
   {
      return CAPI_EFAILED;
   }
   return CAPI_EOK;
}

/**
 * This function allocates EQ memory if current memory is not
 * sufficient and then initializes eq instance.
 */
capi_err_t equalizer_adjust_lib_size_reinit(capi_p_eq_t *me_ptr, uint32_t new_num_channels)
{
   uint32_t      required_lib_size = 0;
   capi_err_t capi_result    = CAPI_EOK;

   if (CAPI_EOK != capi_p_eq_get_eq_lib_size(me_ptr, &required_lib_size))
      return CAPI_EFAILED;

   if ((required_lib_size > me_ptr->lib_size) || (me_ptr->num_channels != new_num_channels))
   {
      P_EQ_MSG(me_ptr->miid, DBG_HIGH_PRIO,
             "CAPI P_EQ: realloc library, current %lu, needed %lu",
             me_ptr->lib_size,
             required_lib_size);
      capi_p_eq_free_eq_instance(me_ptr, CUR_INST);

      me_ptr->num_channels = new_num_channels;
      me_ptr->lib_size     = required_lib_size;
      capi_result       = capi_p_eq_create_new_equalizers(me_ptr, CUR_INST);
   }
   else
   {
      capi_result = capi_p_eq_init_eq_instance(me_ptr, CUR_INST);
   }
   return capi_result;
}

capi_err_t capi_p_eq_replace_current_equalizers(capi_p_eq_t *me_ptr)
{
   if (NULL == me_ptr->lib_instances[0][NEW_INST].lib_mem_ptr)
   {
      P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: error replacing current EQ, new EQ does not exist!");
      return CAPI_EFAILED;
   }

   for (uint32 ch = 0; ch < me_ptr->num_channels; ch++)
   {
      posal_memory_free(me_ptr->lib_instances[ch][CUR_INST].lib_mem_ptr);
      me_ptr->lib_instances[ch][CUR_INST]             = me_ptr->lib_instances[ch][NEW_INST];
      me_ptr->lib_instances[ch][NEW_INST].lib_mem_ptr = NULL;
   }
   return CAPI_EOK;
}

capi_err_t capi_p_eq_set_new_eq_param(capi_p_eq_t *me_ptr, uint32 param_id, int8 *param_ptr, uint32 param_size)
{
   capi_err_t capi_result = CAPI_EOK;
   EQ_RESULT     lib_result     = EQ_SUCCESS;
   for (int32_t ch = 0; ch < (int)me_ptr->num_channels; ch++)
   {
      eq_lib_t *lib_ptr = &me_ptr->lib_instances[ch][NEW_INST];
      if (NULL != lib_ptr->lib_mem_ptr)
      {
         lib_result = eq_set_param(&(me_ptr->lib_instances[ch][NEW_INST]), param_id, param_ptr, param_size);
         if (EQ_SUCCESS != lib_result)
         {
            P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: param_id %lu failed with result %d", param_id, lib_result);
            capi_result = CAPI_EFAILED;
            break;
         }
      }
      else
      {
         P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: setting params on invalid new_inst %d", param_id, lib_result);
         break;
      }
   }
   return capi_result;
}

capi_err_t equalizer_set_new_config_for_xfade(capi_p_eq_t *me_ptr)
{
   capi_err_t          capi_result;
   eq_cross_fade_config_t eq_cross_fade_params;

   /* The following parameters are recommended by System Team */
   eq_cross_fade_params.eq_cross_fade_converge_num_samples = 32;
   eq_cross_fade_params.eq_cross_fade_total_period_msec    = 20;

   max_eq_lib_cfg_t *eq_lib_cfg_ptr = &(me_ptr->max_eq_cfg);

   /* set the new config to the new EQ instances */
   capi_result = capi_p_eq_set_new_eq_param(me_ptr,
                                                  EQ_PARAM_SET_CONFIG,
                                                  (int8 *)eq_lib_cfg_ptr,
                                                  (uint32)sizeof(me_ptr->max_eq_cfg));
   if (CAPI_EOK != capi_result)
      return capi_result;

   /* set new crossfade flag for new equalizers */
   capi_result = capi_p_eq_set_new_eq_param(me_ptr,
                                                  EQ_PARAM_CROSSFADE_CONFIG,
                                                  (int8 *)&eq_cross_fade_params,
                                                  (uint32)sizeof(eq_cross_fade_config_t));

   return capi_result;
}

capi_err_t capi_p_eq_cross_fade_init(capi_p_eq_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   capi_result = capi_p_eq_create_new_equalizers(me_ptr, NEW_INST);
   if (CAPI_EOK != capi_result)
   {
      return capi_result;
   }

   capi_result = equalizer_set_new_config_for_xfade(me_ptr);
   if (CAPI_EOK != capi_result)
   {
      return capi_result;
   }
   eq_cross_fade_mode_t cross_fade_flag = 1;
   capi_result                       = capi_p_eq_set_new_eq_param(me_ptr,
                                                  EQ_PARAM_CROSSFADE_FLAG,
                                                  (int8 *)&cross_fade_flag,
                                                  (uint32)sizeof(eq_cross_fade_mode_t));
   if (CAPI_EOK != capi_result)
   {
      return capi_result;
   }

   capi_p_eq_update_headroom(me_ptr);

   eq_mode_t p_eq_mode = EQ_ENABLE;
   capi_result =
      capi_p_eq_set_new_eq_param(me_ptr, EQ_PARAM_SET_MODE_EQ_CHANGE, (int8 *)&p_eq_mode, (uint32)sizeof(eq_mode_t));

   return capi_result;
}

/**
 * Function to check valid preset id and param_size.
 */
static bool_t is_valid_preset_id_and_payload(int8_t *param_payload_ptr, uint32_t param_size)
{
   bool_t                result            = TRUE;
   param_id_eq_config_t *inp_eq_config_ptr = (param_id_eq_config_t *)param_payload_ptr;
   uint32_t              num_bands         = inp_eq_config_ptr->num_bands;
   int32_t               preset_id         = inp_eq_config_ptr->preset_id;
   uint32_t required_size = sizeof(param_id_eq_config_t) + num_bands * sizeof(param_id_eq_per_band_config_t);

   if (param_size < required_size)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI P_EQ: set config need more memory");
      return FALSE;
   }

   if ((EQ_MAX_SETTINGS_QCOM == preset_id) || (EQ_MAX_SETTINGS_AUDIO_FX <= preset_id) || (preset_id < EQ_USER_CUSTOM))
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI P_EQ: set config, invalid preset id %lu", (uint32_t)preset_id);
      result = FALSE;
   }

   /* for non custom presets, num_bands should be 0 */
   if ((EQ_USER_CUSTOM != preset_id) && (EQ_USER_CUSTOM_AUDIO_FX != preset_id) && (0 != num_bands))
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI P_EQ: set config, num_bands must be zero if EQ presets are selected");
      result = FALSE;
   }

   if ((EQ_USER_CUSTOM == preset_id) && (num_bands > 12))
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI P_EQ: set config, invalid QCOM num_bands %lu", num_bands);
      result = FALSE;
   }

   if ((EQ_USER_CUSTOM_AUDIO_FX == preset_id) && (num_bands != EQ_AUDIO_FX_BAND))
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI P_EQ: set config, invalid OpenSL num_bands %lu", num_bands);
      result = FALSE;
   }
   return result;
}

/**
 * Function to copy input config to EQ internal config structure.
 */
void capi_p_eq_copy_config(capi_p_eq_t *me_ptr, int8_t *param_payload_ptr)
{
   param_id_eq_config_t *inp_eq_config_ptr = (param_id_eq_config_t *)param_payload_ptr;
   uint32_t              num_bands         = inp_eq_config_ptr->num_bands;

   max_eq_lib_cfg_t *max_eq_cfg_ptr = &(me_ptr->max_eq_cfg);
   /* flag for eq headroom adjustment, always set to true */
   max_eq_cfg_ptr->eq_lib_cfg.gain_adjust_flag = 1;
   max_eq_cfg_ptr->eq_lib_cfg.eq_pregain       = (int32)(inp_eq_config_ptr->eq_pregain);
   max_eq_cfg_ptr->eq_lib_cfg.eq_preset_id     = (eq_settings_t)(inp_eq_config_ptr->preset_id);
   max_eq_cfg_ptr->eq_lib_cfg.num_bands        = (eq_num_band_t)(inp_eq_config_ptr->num_bands);

   int8_t *                       ptr                     = param_payload_ptr + sizeof(param_id_eq_config_t);
   param_id_eq_per_band_config_t *inp_eq_per_band_cfg_ptr = (param_id_eq_per_band_config_t *)ptr;

   for (uint32_t band = 0; band < num_bands; band++)
   {
      max_eq_cfg_ptr->eq_per_band_spec[band].filter_type     = (eq_filter_type_t)(inp_eq_per_band_cfg_ptr->filter_type);
      max_eq_cfg_ptr->eq_per_band_spec[band].freq_millihertz = (uint32)(inp_eq_per_band_cfg_ptr->freq_millihertz);
      max_eq_cfg_ptr->eq_per_band_spec[band].gain_millibels  = (int32)(inp_eq_per_band_cfg_ptr->gain_millibels);
      max_eq_cfg_ptr->eq_per_band_spec[band].quality_factor  = (int32)(inp_eq_per_band_cfg_ptr->quality_factor);
      max_eq_cfg_ptr->eq_per_band_spec[band].band_idx        = (uint32)(inp_eq_per_band_cfg_ptr->band_idx);

      inp_eq_per_band_cfg_ptr++;
   }
   /* clear the remaining structure just in case
    * TODO: Check if needed at all */
   for (uint32_t band = num_bands; band < 12; band++)
   {
      max_eq_cfg_ptr->eq_per_band_spec[band].filter_type     = EQ_TYPE_NONE;
      max_eq_cfg_ptr->eq_per_band_spec[band].freq_millihertz = 0;
      max_eq_cfg_ptr->eq_per_band_spec[band].gain_millibels  = 0;
      max_eq_cfg_ptr->eq_per_band_spec[band].quality_factor  = 0;
      max_eq_cfg_ptr->eq_per_band_spec[band].band_idx        = 0;
   }
}

capi_err_t capi_p_eq_set_config(capi_p_eq_t *me_ptr, int8_t *param_payload_ptr, uint32_t param_size)
{
   EQ_RESULT lib_result = EQ_SUCCESS;

   if (!is_valid_preset_id_and_payload(param_payload_ptr, param_size))
      return CAPI_EFAILED;

   capi_p_eq_copy_config(me_ptr, param_payload_ptr);

   max_eq_lib_cfg_t *max_eq_cfg_ptr = &(me_ptr->max_eq_cfg);

   /* if no input data is ever processed, or the current EQ is disabled,
    * save the new config in the current library. */
   if ((1 == me_ptr->is_first_frame) || (P_EQ_DISABLE == me_ptr->p_eq_state))
   {
      for (int32_t ch = 0; ch < (int)me_ptr->num_channels; ch++)
      {
         lib_result = eq_set_param(&(me_ptr->lib_instances[ch][CUR_INST]),
                                   EQ_PARAM_SET_CONFIG,
                                   (int8 *)max_eq_cfg_ptr,
                                   (uint32)sizeof(me_ptr->max_eq_cfg));
         if (EQ_SUCCESS != lib_result)
         {
            P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: set config failed %d", lib_result);
            return CAPI_EFAILED;
         }
      }
      if (P_EQ_DISABLE != me_ptr->p_eq_state)
      {
         me_ptr->transition_num_samples = 0;
         capi_p_eq_update_headroom(me_ptr);
      }
      capi_p_eq_update_delay(me_ptr);
   }
   else if (me_ptr->volume_ramp)
   {
      P_EQ_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI P_EQ: Config set during volume ramp");
      capi_err_t capi_result = CAPI_EOK;
      capi_result               = equalizer_set_new_config_for_xfade(me_ptr);
      if (CAPI_EOK != capi_result)
      {
         return capi_result;
      }
      capi_p_eq_update_headroom(me_ptr);
   }
   else if (equalizer_is_cross_fade_active(me_ptr))
   {
      /* if crossfade is active, cache the new config in max_eq_cfg structure,
       * the new config will be honored by another cross-fading session after
       * the current cross-fading is done */
      P_EQ_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI P_EQ: New Config Cached due to ongoing xfade");
      me_ptr->is_new_config_pending = 1;
   }
   else
   {
      capi_err_t capi_result = CAPI_EOK;
      capi_result               = capi_p_eq_cross_fade_init(me_ptr);
      if (CAPI_EOK != capi_result)
      {
         P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: set new config failed %d", (int)capi_result);
         return capi_result;
      }
      P_EQ_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI P_EQ: New config set and crossfade started");
   }
   return CAPI_EOK;
}

void capi_p_eq_process_in_vol_ctrl(capi_p_eq_t *       me_ptr,
                                      capi_stream_data_t *input[],
                                      capi_stream_data_t *output[])
{
   int32    byte_sample_convert = me_ptr->lib_static_vars.data_width / 16;
   uint32_t num_samples         = input[0]->buf_ptr[0].actual_data_len >> byte_sample_convert;

  /*Backup the no of samples to update the final input and output data length */
  uint32_t temp_num_samples = num_samples;

   /* ~40ms wait for volume ramp down. */
   /* NOTE: float arithemetic crashes on dynamic loading */
   uint32_t max_transition_sampls = ((41 * me_ptr->lib_static_vars.sample_rate) >> 10);
			 
   me_ptr->prev_transition_num_samples = me_ptr->transition_num_samples;

   me_ptr->transition_num_samples += num_samples;
   if (me_ptr->transition_num_samples > max_transition_sampls)
   {

	 P_EQ_MSG(me_ptr->miid, DBG_HIGH_PRIO,
		 "P_EQ: Enable Delayed %u samples for sampling rate: %d",
		 (unsigned int)me_ptr->transition_num_samples,
		 (int)me_ptr->lib_static_vars.sample_rate);

	 if (me_ptr->prev_transition_num_samples > max_transition_sampls)
	 {
		P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: previous transition samples must be less than or equal to the max transition sampls ");
	 }
     num_samples  = max_transition_sampls - me_ptr->prev_transition_num_samples;

     for (int32_t ch = 0; ch < (int)me_ptr->num_channels; ch++)
	 {
		memscpy(output[0]->buf_ptr[ch].data_ptr,
				output[0]->buf_ptr[0].max_data_len,
				input[0]->buf_ptr[ch].data_ptr,
				num_samples << byte_sample_convert);
	 }

     input[0]->buf_ptr[0].actual_data_len  = input[0]->buf_ptr[0].actual_data_len - (num_samples << byte_sample_convert);

      /* Setting crossfading enable */
      eq_mode_t p_eq_mode = EQ_ENABLE;
      for (int32_t ch = 0; ch < (int)me_ptr->num_channels; ch++)
      {
         eq_set_param(&(me_ptr->lib_instances[ch][CUR_INST]),
                      EQ_PARAM_SET_MODE_EQ_ONOFF,
                      (int8 *)&p_eq_mode,
                      sizeof(eq_mode_t));
      }
      me_ptr->p_eq_state             = P_EQ_OFF_ON_CROSS_FADE;
      me_ptr->transition_num_samples = 0;

      /* Updating processed no of samples as of now to me_ptr->prev_transition_num_samples (instead of creating new variable we are reusing this flag) */
      /* which is using in eq process call to update the input/output pointer */
      me_ptr->prev_transition_num_samples = num_samples;
      me_ptr->vol_ctrl_to_peq_state = 1;

      capi_p_eq_process_eq(me_ptr,input ,output );

      me_ptr->vol_ctrl_to_peq_state = 0;
      me_ptr->prev_transition_num_samples = 0;

      for (int32_t ch = 0; ch < (int)me_ptr->num_channels; ch++)
      {
    	input[0]->buf_ptr[ch].actual_data_len  = (temp_num_samples << byte_sample_convert);
    	output[0]->buf_ptr[ch].actual_data_len = (temp_num_samples << byte_sample_convert);
      }
   }
   else
   {
	   for (int32_t ch = 0; ch < (int)me_ptr->num_channels; ch++)
	     {
	        memscpy(output[0]->buf_ptr[ch].data_ptr,
	                num_samples << byte_sample_convert,
	                input[0]->buf_ptr[ch].data_ptr,
	                num_samples << byte_sample_convert);
	        output[0]->buf_ptr[ch].actual_data_len = (num_samples << byte_sample_convert);
	        input[0]->buf_ptr[ch].actual_data_len  = (num_samples << byte_sample_convert);
	     }
   }
}
capi_err_t capi_p_eq_set_enable(capi_p_eq_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   me_ptr->pending_disable_event = 0;
   if (P_EQ_WAITING_FOR_MEDIA_FORMAT == me_ptr->p_eq_state)
   {
      me_ptr->pending_disable_event = 1;
      capi_p_eq_update_process_check(me_ptr, me_ptr->enable_flag);
      return CAPI_EOK;
   }

   if ((0 != me_ptr->enable_flag) && (P_EQ_DISABLE == me_ptr->p_eq_state))
   {
      me_ptr->pending_disable_event = 1;
      capi_p_eq_update_process_check(me_ptr, me_ptr->enable_flag);
      me_ptr->p_eq_state = P_EQ_VOL_CTRL;
   }

   if ((0 == me_ptr->enable_flag) && (P_EQ_DISABLE != me_ptr->p_eq_state))
   {
      if (!equalizer_is_cross_fade_active(me_ptr))
      {
         me_ptr->p_eq_state            = P_EQ_ON_OFF_CROSS_FADE;
         me_ptr->pending_disable_event = 1;
         /* initiate the library crossfade for disable */
         for (uint32_t ch = 0; ch < me_ptr->num_channels; ch++)
         {
            eq_mode_t p_eq_mode = EQ_DISABLE;
            eq_set_param(&(me_ptr->lib_instances[ch][CUR_INST]),
                         EQ_PARAM_SET_MODE_EQ_ONOFF,
                         (int8 *)&p_eq_mode,
                         sizeof(eq_mode_t));
         }
      }
   }

   P_EQ_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI P_EQ: Set enable: %lu, PEQ state = %lu", me_ptr->enable_flag, me_ptr->p_eq_state);

   return capi_result;
}
capi_err_t capi_p_eq_process_eq(capi_p_eq_t *       me_ptr,
                                      capi_stream_data_t *input[],
                                      capi_stream_data_t *output[])
{
   // AR_MSG(DBG_HIGH_PRIO, "CAPI P_EQ: process_eq function called ");
   capi_err_t result     = CAPI_EOK;
   EQ_RESULT     lib_result = EQ_SUCCESS;

   /*create and maintaining the local pointer for processing*/
   int8_t *inp_ptrs = {NULL};
   int8_t *out_ptrs = {NULL};

   if (0 == input[0]->buf_ptr->actual_data_len)
   {
      return result;
   }
   int32    byte_sample_convert = me_ptr->lib_static_vars.data_width / 16;
   uint32_t num_samples         = input[0]->buf_ptr[0].actual_data_len >> byte_sample_convert;

   for (int32_t ch = 0; ch < (int)me_ptr->num_channels; ch++)
   {
      eq_lib_t *eq_lib_ptr[TOTAL_INST];

      if (equalizer_is_cross_fade_active(me_ptr) && !me_ptr->volume_ramp)
      {
         /* crossfading in progress */
         eq_lib_ptr[CUR_INST] = &(me_ptr->lib_instances[ch][CUR_INST]);
         eq_lib_ptr[NEW_INST] = &(me_ptr->lib_instances[ch][NEW_INST]);
      }
      else
      {
         /* no crossfading or crossfading done */
         eq_lib_ptr[CUR_INST] = &(me_ptr->lib_instances[ch][CUR_INST]);
         eq_lib_ptr[NEW_INST] = NULL;
      }

      if(me_ptr->vol_ctrl_to_peq_state)
      {
    	  inp_ptrs = (input[0]->buf_ptr[ch].data_ptr + (me_ptr->prev_transition_num_samples << byte_sample_convert));
    	  out_ptrs = (output[0]->buf_ptr[ch].data_ptr + (me_ptr->prev_transition_num_samples << byte_sample_convert));
      }
      else
	  {
    	  inp_ptrs = input[0]->buf_ptr[ch].data_ptr;
    	  out_ptrs = output[0]->buf_ptr[ch].data_ptr;
	  }

     //  AR_MSG(DBG_HIGH_PRIO, "CAPI_P_EQ: limit num_samples = %lu", num_samples);
      lib_result = eq_process(eq_lib_ptr, out_ptrs, inp_ptrs, num_samples);

      if (EQ_SUCCESS != lib_result)
      {
         P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: library process failed with error %lu", lib_result);
         result = CAPI_EFAILED;
      }
      output[0]->buf_ptr[ch].actual_data_len = (num_samples << byte_sample_convert);
      input[0]->buf_ptr[ch].actual_data_len  = (num_samples << byte_sample_convert);
   }
   return result;
}

#ifdef DO_POPLESSEQUALIZER_PROFILING
void capi_p_eq_print_kpps(capi_p_eq_t *me_ptr)
{
   p_eq_prof_info_t *p_eq_kpps_data = &me_ptr->p_eq_kpps_data;

   p_eq_kpps_data->average_kpps =
      ((p_eq_kpps_data->total_cycles / p_eq_kpps_data->sample_count) * p_eq_kpps_data->sample_rate) / (1000);

   MSG_4(MSG_SSID_QDSP6,
         DBG_HIGH_PRIO,
         "CAPI P_EQ: Total cycles :%llu, Total KPPS: %llu, "
         "Total Sample Count: %llu, Peak KPPS : %llu",
         p_eq_kpps_data->total_cycles,
         p_eq_kpps_data->average_kpps,
         p_eq_kpps_data->sample_count,
         p_eq_kpps_data->peak_kpps);

   p_eq_kpps_data->total_cycles = 0;
   p_eq_kpps_data->peak_kpps    = 0;
   p_eq_kpps_data->sample_count = 0;
   p_eq_kpps_data->average_kpps = 0;
}

void capi_p_eq_profiling(capi_p_eq_t *me_ptr, uint32_t num_samples)
{
   p_eq_prof_info_t *p_eq_kpps_data = &me_ptr->p_eq_kpps_data;

   p_eq_kpps_data->total_cycles =
      p_eq_kpps_data->total_cycles + (p_eq_kpps_data->end_cycles - p_eq_kpps_data->start_cycles);

   p_eq_kpps_data->sample_count += num_samples;

   uint64_t frame_kpps = p_eq_kpps_data->end_cycles - p_eq_kpps_data->start_cycles;
   frame_kpps          = frame_kpps / num_samples;
   frame_kpps          = frame_kpps * p_eq_kpps_data->sample_rate / 1000;

   if (frame_kpps > p_eq_kpps_data->peak_kpps)
   {
      p_eq_kpps_data->peak_kpps = frame_kpps;
   }
}
#endif /* DO_POPLESSEQUALIZER_PROFILING */
