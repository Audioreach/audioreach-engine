/* =========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 * @file capi_popless_equalizer.cpp
 *
 * Common Audio Processor Interface Implementation for Popless Equalizer.
 * (library version: AUDIO_SYSTEM_EQ_1.2.2_05.05.14)
 */

#include "capi_popless_equalizer.h"
#include "equalizer_api.h"
#include "equalizer_calibration_api.h"
#include "capi_popless_equalizer_utils.h"
#include "popless_equalizer_api.h"
#include "imcl_p_eq_vol_api.h"
#include "imcl_spm_intent_api.h"

#ifdef DO_POPLESSEQUALIZER_PROFILING
#include <q6sim_timer.h>
#endif

static capi_err_t capi_p_eq_process(capi_t *            _pif,
                                          capi_stream_data_t *input[],
                                          capi_stream_data_t *output[]);

static capi_err_t capi_p_eq_end(capi_t *_pif);

static capi_err_t capi_p_eq_set_param(capi_t *                _pif,
                                            uint32_t                   param_id,
                                            const capi_port_info_t *port_info_ptr,
                                            capi_buf_t *            params_ptr);

static capi_err_t capi_p_eq_get_param(capi_t *                _pif,
                                            uint32_t                   param_id,
                                            const capi_port_info_t *port_info_ptr,
                                            capi_buf_t *            params_ptr);

static capi_err_t capi_p_eq_set_properties(capi_t *_pif, capi_proplist_t *props_ptr);

static capi_err_t capi_p_eq_get_properties(capi_t *_pif, capi_proplist_t *props_ptr);

static capi_vtbl_t capi_p_eq_vtbl = { capi_p_eq_process,        capi_p_eq_end,
                                            capi_p_eq_set_param,      capi_p_eq_get_param,
                                            capi_p_eq_set_properties, capi_p_eq_get_properties };

capi_err_t capi_p_eq_get_static_properties(capi_proplist_t *init_set_properties,
                                                 capi_proplist_t *static_properties)
{
   capi_err_t capi_result = CAPI_EFAILED;

   if (NULL != init_set_properties)
   {
      P_EQ_MSG(MIID_UNKNOWN, DBG_HIGH_PRIO, "CAPI P_EQ: get static prop ignoring init_set_properties!");
   }

   if (NULL != static_properties)
   {
      capi_result = capi_p_eq_process_get_properties((capi_p_eq_t *)NULL, static_properties);
   }
   else
   {
      P_EQ_MSG(MIID_UNKNOWN, DBG_HIGH_PRIO, "CAPI P_EQ: get static prop, Bad ptrs: %p", static_properties);
   }
   return capi_result;
}

capi_err_t capi_p_eq_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == _pif)
      return CAPI_EBADPARAM;

   capi_p_eq_t *me_ptr = (capi_p_eq_t *)_pif;

   memset(me_ptr, 0, sizeof(capi_p_eq_t));
   me_ptr->p_eq_state                       = P_EQ_WAITING_FOR_MEDIA_FORMAT;
   me_ptr->is_first_frame                   = TRUE;
   me_ptr->process_check                    = -1;
   me_ptr->num_channels                     = EQUALIZER_MAX_CHANNELS;
   me_ptr->max_eq_cfg.eq_lib_cfg.eq_pregain = P_EQ_INIT_PREGAIN;
   me_ptr->ctrl_port_received               = P_EQ_CTRL_PORT_INFO_NOT_RCVD;
   me_ptr->temp_enable_flag                 = -1;
   me_ptr->pending_disable_event			= 1;
   me_ptr->vtbl_ptr                         = &capi_p_eq_vtbl;
   capi_p_eq_init_media_fmt(me_ptr);

   /* init_set_properties contains OUT_BITS_PER_SAMPLE,
    * EVENT_CALLBACK_INFO and PORT_INFO */
   if (NULL != init_set_properties)
   {
      capi_result = capi_p_eq_process_set_properties(me_ptr, init_set_properties);
      capi_result ^= (capi_result & CAPI_EUNSUPPORTED);
   }
   capi_p_eq_update_process_check(me_ptr, me_ptr->enable_flag);
   // Initialize the control port list.
   capi_cmn_ctrl_port_list_init(&me_ptr->ctrl_port_info);

   P_EQ_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI P_EQ: Init done!");

   return capi_result;
}

static capi_err_t capi_p_eq_process(capi_t *            _pif,
                                          capi_stream_data_t *input[],
                                          capi_stream_data_t *output[])
{
   capi_err_t capi_result                     = CAPI_EOK;
   EQ_RESULT     lib_result __attribute__((unused)) = EQ_SUCCESS;

   uint32 start_new_cross_fade = 0;

   capi_p_eq_t *me_ptr = (capi_p_eq_t *)(_pif);
   // AR_MSG(DBG_HIGH_PRIO, "CAPI P_EQ: process function called with sr = %lu ",me_ptr->lib_static_vars.sample_rate);

   POSAL_ASSERT(me_ptr);
   POSAL_ASSERT(input[0]);
   POSAL_ASSERT(output[0]);
   POSAL_ASSERT(me_ptr->num_channels <= input[0]->bufs_num);
   POSAL_ASSERT(me_ptr->num_channels <= output[0]->bufs_num);

#ifdef DO_POPLESSEQUALIZER_PROFILING
   me_ptr->p_eq_kpps_data.start_cycles = q6sim_read_cycles();
#endif

   if (2 == me_ptr->pending_disable_event)
   {
      me_ptr->pending_disable_event         = 0;
      for(uint32_t inch = 0; inch < input[0]->bufs_num; inch++)
      {
          input[0]->buf_ptr[inch].actual_data_len  = 0;
      }
      for(uint32_t outch = 0; outch < output[0]->bufs_num; outch++)
      {
          output[0]->buf_ptr[outch].actual_data_len = 0;
      }
      capi_result = capi_cmn_update_process_check_event(&me_ptr->cb_info, (uint32_t)me_ptr->process_check);
      return capi_result;
   }
   int32 byte_sample_convert = me_ptr->lib_static_vars.data_width / 16;

   uint32_t num_samples;

   if (input[0]->buf_ptr[0].actual_data_len > output[0]->buf_ptr[0].max_data_len)
   {
      input[0]->buf_ptr[0].actual_data_len = output[0]->buf_ptr[0].max_data_len;
   }

   num_samples = input[0]->buf_ptr[0].actual_data_len >> byte_sample_convert;

   if (P_EQ_VOL_CTRL == me_ptr->p_eq_state)
   {
      capi_p_eq_process_in_vol_ctrl(me_ptr, input, output);
      return CAPI_EOK;
   }

   if (CAPI_EOK != capi_p_eq_process_eq(me_ptr, input, output))
      return CAPI_EFAILED;

   if (1 == me_ptr->volume_ramp)
   {
      /* max ramp duration is ~40 ms */
      /* NOTE: float arithemetic crashes on dynamic loading */
      uint32_t max_transition_sampls = ((41 * me_ptr->lib_static_vars.sample_rate) >> 10);
      me_ptr->transition_num_samples_preset += num_samples;
      if (me_ptr->transition_num_samples_preset > max_transition_sampls)
      {
         me_ptr->volume_ramp                   = 0;
         me_ptr->transition_num_samples_preset = 0;
      }
   }

   if (equalizer_is_cross_fade_active(me_ptr))
   {
      if (is_new_eq_instance_crossfade_finished(me_ptr))
      {
         start_new_cross_fade = me_ptr->is_new_config_pending;
         if (CAPI_EOK != capi_p_eq_replace_current_equalizers(me_ptr))
            return CAPI_EFAILED;
         capi_p_eq_update_delay(me_ptr);
         capi_p_eq_update_headroom(me_ptr);
      }
   }

   if (1 == start_new_cross_fade)
   {
      /* there are new configs pending during the current active xfade
       * session, now since the current xfade is done, we can honor the
       * new configs by starting another xfade session. */
      capi_result = capi_p_eq_cross_fade_init(me_ptr);
      if (CAPI_EOK != capi_result)
      {
         P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: set new EQ crossfade failed");
         return CAPI_EFAILED;
      }
      me_ptr->is_new_config_pending = 0;
   }

   me_ptr->is_first_frame = 0;

   capi_p_eq_update_eq_state(me_ptr);
   if ((!me_ptr->enable_flag) && (!equalizer_is_cross_fade_active(me_ptr)))
   {
      for (int32_t ch = 0; ch < (int)me_ptr->num_channels; ch++)
      {
         eq_mode_t p_eq_mode = EQ_DISABLE;
         eq_set_param(&(me_ptr->lib_instances[ch][CUR_INST]),
                      EQ_PARAM_SET_MODE_EQ_ONOFF,
                      (int8 *)&p_eq_mode,
                      sizeof(eq_mode_t));
      }
   }

   output[0]->flags = input[0]->flags;
   if (input[0]->flags.is_timestamp_valid)
   {
      output[0]->timestamp = input[0]->timestamp;
   }

#ifdef DO_POPLESSEQUALIZER_PROFILING
   me_ptr->p_eq_kpps_data.end_cycles = q6sim_read_cycles();
   capi_p_eq_profiling(me_ptr, num_samples);
#endif

   return CAPI_EOK;
}

static capi_err_t capi_p_eq_end(capi_t *_pif)
{
   if (NULL == _pif)
   {
      P_EQ_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "CAPI P_EQ: End received bad pointer, 0x%p", _pif);
      return CAPI_EBADPARAM;
   }

   capi_p_eq_t *me_ptr = (capi_p_eq_t *)(_pif);
   uint32_t temp_miid = me_ptr->miid;

#ifdef DO_POPLESSEQUALIZER_PROFILING
   capi_p_eq_print_kpps(me_ptr);
#endif

   capi_p_eq_free_eq_instance(me_ptr, CUR_INST);
   capi_p_eq_free_eq_instance(me_ptr, NEW_INST);
   capi_cmn_ctrl_port_list_deinit(&me_ptr->ctrl_port_info);
   me_ptr->vtbl_ptr = NULL;

   P_EQ_MSG(temp_miid, DBG_HIGH_PRIO, "CAPI P_EQ: End done");
   return CAPI_EOK;
}

static capi_err_t capi_p_eq_set_param(capi_t *                _pif,
                                            uint32_t                   param_id,
                                            const capi_port_info_t *port_info_ptr,
                                            capi_buf_t *            params_ptr)
{
   if (NULL == _pif || NULL == params_ptr)
   {
      P_EQ_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "CAPI P_EQ: Set param received bad pointer, 0x%p, 0x%p", _pif, params_ptr);
      return CAPI_EBADPARAM;
   }

   EQ_RESULT lib_result = EQ_SUCCESS;

   capi_p_eq_t *me_ptr = (capi_p_eq_t *)(_pif);

   if (NULL == params_ptr->data_ptr)
   {
      P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: Set param received bad param data pointer, 0x%p", params_ptr->data_ptr);
      return CAPI_EBADPARAM;
   }
   int8_t *param_payload_ptr = (int8_t *)(params_ptr->data_ptr);

   capi_err_t capi_result;
   uint32_t      param_size = 0;
   uint32_t      eq_lib_api = 0xFFFFFFFF;

   capi_result = get_eq_lib_api_info(param_id, &param_size, &eq_lib_api);
   if (CAPI_EOK != capi_result)
   {
      P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: Set, unsupported param ID %#x", (int)param_id);
      return capi_result;
   }
   if (param_size > params_ptr->actual_data_len)
   {
      P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO,
             "CAPI P_EQ: Set, Param id %lu need more memory, "
             "actual_len = %lu, required = %lu",
             param_id,
             params_ptr->actual_data_len,
             param_size);
      return CAPI_ENEEDMORE;
   }

   switch (param_id)
   {
      case CAPI_ALGORITHMIC_RESET:
      { // TODO: remove this ; run saplus tests on elite removing this
         P_EQ_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI P_EQ: reset");

         if (P_EQ_WAITING_FOR_MEDIA_FORMAT == me_ptr->p_eq_state)
            return CAPI_EOK;

         for (int32_t ch = 0; ch < (int)me_ptr->num_channels; ch++)
         {
            lib_result = eq_set_param(&(me_ptr->lib_instances[ch][CUR_INST]), EQ_PARAM_SET_RESET, NULL, 0);
            if (EQ_SUCCESS != lib_result)
            {
               return CAPI_EFAILED;
            }
         }
         break;
      }

      case PARAM_ID_MODULE_ENABLE:
      {
         param_id_module_enable_t *enable_ptr = (param_id_module_enable_t *)param_payload_ptr;

         if (me_ptr->ctrl_port_received)
         {
            me_ptr->enable_flag = enable_ptr->enable;
            capi_p_eq_set_enable(me_ptr);
            P_EQ_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI P_EQ: Received Set enable: %lu", me_ptr->enable_flag);
         }
         else
         {
            me_ptr->temp_enable_flag = enable_ptr->enable;
            P_EQ_MSG(me_ptr->miid, DBG_HIGH_PRIO,
                   "CAPI P_EQ: Reserved Set enable: %lu for port info, PEQ state = %lu",
                   me_ptr->enable_flag,
                   me_ptr->p_eq_state);
         }

         break;
      }

      case PARAM_ID_EQ_CONFIG:
      {
         if (P_EQ_WAITING_FOR_MEDIA_FORMAT == me_ptr->p_eq_state)
         {
            capi_p_eq_copy_config(me_ptr, param_payload_ptr);
            return CAPI_EOK;
         }

         capi_result = capi_p_eq_set_config(me_ptr, param_payload_ptr, params_ptr->actual_data_len);
         if (CAPI_EOK != capi_result)
            return capi_result;
         break;
      }

      case PARAM_ID_EQ_BAND_INDEX:
      {
         param_id_eq_band_index_t *band_index_ptr = (param_id_eq_band_index_t *)param_payload_ptr;

         me_ptr->band_idx = band_index_ptr->band_index;
         P_EQ_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI P_EQ: set band index %lu", me_ptr->band_idx);
         break;
      }

      case PARAM_ID_EQ_SINGLE_BAND_FREQ:
      {
         param_id_eq_single_band_freq_t *band_freq_ptr = (param_id_eq_single_band_freq_t *)param_payload_ptr;

         me_ptr->band_freq_millihertz = band_freq_ptr->freq_millihertz * 1000; // Storing in hertz
         P_EQ_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI P_EQ: set single band freq %lu", me_ptr->band_freq_millihertz);
         break;
      }
      case INTF_EXTN_PARAM_ID_IMCL_PORT_OPERATION:
      {
         uint32_t supported_intent[1]      = { INTENT_ID_P_EQ_VOL_HEADROOM };
         capi_result                    = capi_cmn_ctrl_port_operation_handler(&me_ptr->ctrl_port_info,
                                                                  params_ptr,
                                                                  (POSAL_HEAP_ID)me_ptr->heap_info.heap_id,
                                                                  0,
                                                                  1,
                                                                  supported_intent);
         uint32_t          control_port_id = 0;
         imcl_port_state_t port_state      = CTRL_PORT_CLOSE;
         ctrl_port_data_t *port_data_ptr   = NULL;
         // Get the first control port id for the intent #INTENT_ID_P_EQ_VOL_HEADROOM
         capi_cmn_ctrl_port_list_get_next_port_data(&me_ptr->ctrl_port_info,
                                                       INTENT_ID_P_EQ_VOL_HEADROOM,
                                                       control_port_id, // initially, an invalid port id
                                                       &port_data_ptr);
         if (port_data_ptr)
         {
            control_port_id = port_data_ptr->port_info.port_id;
            port_state      = port_data_ptr->state;
         }
         P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: Port state is  %lu, port id is 0x%x", port_state, control_port_id);
         if ((0 != control_port_id))
         {
            me_ptr->ctrl_port_received = P_EQ_CTRL_PORT_INFO_RCVD;
            if (CTRL_PORT_OPEN == port_state)
            {
               if (-1 != me_ptr->temp_enable_flag)
               {
                  capi_p_eq_set_enable(me_ptr);
                  me_ptr->temp_enable_flag = -1;
               }
            }
         }
         break;
      }
      default:
      {
         P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: Set, unsupported param ID %#lx", param_id);
         return CAPI_EBADPARAM;
      }
   }
   return CAPI_EOK;
}

static capi_err_t capi_p_eq_get_param(capi_t *                _pif,
                                            uint32_t                   param_id,
                                            const capi_port_info_t *port_info_ptr,
                                            capi_buf_t *            params_ptr)
{
   if (NULL == _pif || NULL == params_ptr)
   {
      P_EQ_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "CAPI P_EQ: Get param received bad pointer, 0x%p, 0x%p", _pif, params_ptr);
      return CAPI_EBADPARAM;
   }

   capi_p_eq_t *me_ptr = (capi_p_eq_t *)(_pif);
   if (NULL == params_ptr->data_ptr)
   {
      P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: Get param received bad param data pointer, 0x%p", params_ptr->data_ptr);
      return CAPI_EBADPARAM;
   }
   if (P_EQ_WAITING_FOR_MEDIA_FORMAT == me_ptr->p_eq_state)
   {
      P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: Get param unsupported while waiting for media format");
      return CAPI_EUNSUPPORTED;
   }

   capi_err_t capi_result;
   EQ_RESULT     lib_result = EQ_SUCCESS;
   uint32_t      param_size = 0;
   uint32_t      eq_lib_api = 0xFFFFFFFF;

   capi_result = get_eq_lib_api_info(param_id, &param_size, &eq_lib_api);
   if (CAPI_EOK != capi_result)
   {
      P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: Get, unsupported param ID %#x", (int)param_id);
      return CAPI_EBADPARAM;
   }
   else if (param_size > params_ptr->max_data_len)
   {
      P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO,
             "CAPI P_EQ: Get, Param id %lu need more memory, "
             "actual_len = %lu, required = %lu",
             param_id,
             params_ptr->actual_data_len,
             param_size);
      return CAPI_ENEEDMORE;
   }

   switch (param_id)
   {
      case PARAM_ID_MODULE_ENABLE:
      {
         param_id_module_enable_t *p_eq_enable = (param_id_module_enable_t *)(params_ptr->data_ptr);
         p_eq_enable->enable                   = me_ptr->enable_flag;
         params_ptr->actual_data_len           = sizeof(me_ptr->enable_flag);
         break;
      }
      case PARAM_ID_EQ_SINGLE_BAND_FREQ_RANGE:
      {
         eq_band_freq_range_t band_freq_range;
         uint32_t             lib_actual_size = 0;
         band_freq_range.band_idx             = me_ptr->band_idx;

         lib_result = eq_get_param(&(me_ptr->lib_instances[0][CUR_INST]),
                                   EQ_PARAM_GET_BAND_FREQ_RANGE,
                                   (int8 *)&band_freq_range,
                                   (uint32)sizeof(band_freq_range),
                                   (uint32 *)&lib_actual_size);
         if ((EQ_SUCCESS != lib_result) || (0 == lib_actual_size))
         {
            P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: Set param %lu failed with result %d", param_id, (int)lib_result);
            return CAPI_EFAILED;
         }

         param_id_eq_single_band_freq_range_t *band_freq_range_ptr =
            (param_id_eq_single_band_freq_range_t *)params_ptr->data_ptr;

         band_freq_range_ptr->min_freq_millihertz = (uint32_t)(band_freq_range.eq_band_freq_range.min_freq_millihertz);
         band_freq_range_ptr->max_freq_millihertz = (uint32_t)(band_freq_range.eq_band_freq_range.max_freq_millihertz);
         params_ptr->actual_data_len              = (uint32_t)sizeof(*band_freq_range_ptr);
         break;
      }

      case PARAM_ID_EQ_BAND_INDEX:
      {
         eq_band_index_t band_index;
         uint32_t        lib_actual_size = 0;

         band_index.freq_millihertz = me_ptr->band_freq_millihertz;
         band_index.band_idx        = 0;

         lib_result = eq_get_param(&(me_ptr->lib_instances[0][CUR_INST]),
                                   EQ_PARAM_GET_BAND,
                                   (int8 *)&band_index,
                                   (uint32)sizeof(band_index),
                                   (uint32 *)&lib_actual_size);
         if ((EQ_SUCCESS != lib_result) || (0 == lib_actual_size))
         {
            P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: Set param %lu failed with result %d", param_id, (int)lib_result);
            return CAPI_EFAILED;
         }

         param_id_eq_band_index_t *band_index_ptr = (param_id_eq_band_index_t *)params_ptr->data_ptr;

         band_index_ptr->band_index  = (uint32_t)band_index.band_idx;
         params_ptr->actual_data_len = (uint32_t)sizeof(*band_index_ptr);
         break;
      }
      case PARAM_ID_EQ_PRESET_ID:
	  {
         uint32 lib_actual_size = 0;
		 int32 *actual_custom_preset = (int32 *)params_ptr->data_ptr; //this is to make custom preset_id as -1
         /* all eq channels shares same configs */
         memset((void *)params_ptr->data_ptr, 0, sizeof(uint32)); // initializing to 0
         lib_result = eq_get_param(&(me_ptr->lib_instances[0][CUR_INST]),
                                   eq_lib_api,
                                   (int8 *)params_ptr->data_ptr,
                                   param_size,
                                   (uint32 *)&lib_actual_size);
         if ((EQ_SUCCESS != lib_result) || (0 == lib_actual_size))
         {
            P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: Set param %lu failed with result %d", param_id, (int)lib_result);
            return CAPI_EFAILED;
         }
		 if (255 == *actual_custom_preset) //only 2 bytes of data is returned from the library, meaning it would return 0x000000FF(255) in place of 0xFFFFFFFF(-1),
											//so to handle this special case of 0xFFFFFFFF(custom preset), we explicitly modify the data_ptr to -1. 
		 {
			*actual_custom_preset = -1;
		 }
         params_ptr->actual_data_len = param_size;
         break;
	  }
      case PARAM_ID_EQ_NUM_BANDS:
      case PARAM_ID_EQ_BAND_LEVELS:
      case PARAM_ID_EQ_BAND_LEVEL_RANGE:
      case PARAM_ID_EQ_BAND_FREQS:
      case PARAM_ID_EQ_NUM_PRESETS:
      case PARAM_ID_EQ_PRESET_NAME:
      {
         uint32 lib_actual_size = 0;
         /* all eq channels shares same configs */
         memset((void *)params_ptr->data_ptr, 0, sizeof(uint32)); // initializing to 0
         lib_result = eq_get_param(&(me_ptr->lib_instances[0][CUR_INST]),
                                   eq_lib_api,
                                   (int8 *)params_ptr->data_ptr,
                                   param_size,
                                   (uint32 *)&lib_actual_size);
         if ((EQ_SUCCESS != lib_result) || (0 == lib_actual_size))
         {
            P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: Set param %lu failed with result %d", param_id, (int)lib_result);
            return CAPI_EFAILED;
         }

         params_ptr->actual_data_len = param_size;
         break;
      }

      default:
      {
         P_EQ_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI P_EQ: Get, unsupported param ID %#x", (int)param_id);
         return CAPI_EBADPARAM;
      }
   }

   return CAPI_EOK;
}

static capi_err_t capi_p_eq_set_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   if (NULL == _pif)
      return CAPI_EBADPARAM;

   capi_p_eq_t *me_ptr = (capi_p_eq_t *)_pif;

   return capi_p_eq_process_set_properties(me_ptr, props_ptr);
}

static capi_err_t capi_p_eq_get_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   if (NULL == _pif)
      return CAPI_EBADPARAM;

   capi_p_eq_t *me_ptr = (capi_p_eq_t *)_pif;

   return capi_p_eq_process_get_properties(me_ptr, props_ptr);
}
