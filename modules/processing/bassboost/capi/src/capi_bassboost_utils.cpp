/* =========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 * @file capi_bassboost_utils.cpp
 *
 * C source file to implement the Audio Post Processor Interface for
 * Acoustic Bass Enhancement
 */

#include "capi_bassboost_utils.h"

static void capi_bassboost_update_event_states(capi_bassboost_t *me_ptr);
void        capi_bassboost_init_media_fmt(capi_bassboost_t *me_ptr);

static bool_t bassboost_is_supported_media_type(const capi_media_fmt_v2_t *format_ptr)
{
   if (CAPI_FIXED_POINT != format_ptr->header.format_header.data_format)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "CAPI BassBoost: unsupported data format %lu",
             (uint32_t)format_ptr->header.format_header.data_format);
      return FALSE;
   }

   if ((16 != format_ptr->format.bits_per_sample) && (32 != format_ptr->format.bits_per_sample))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "CAPI BassBoost: only supports 16 and 32 bit data. Received %lu.",
             format_ptr->format.bits_per_sample);
      return FALSE;
   }

   if (CAPI_INTERLEAVED == format_ptr->format.data_interleaving)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI BassBoost: Interleaved data not supported.");
      return FALSE;
   }

   if (!format_ptr->format.data_is_signed)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI BassBoost: Unsigned data not supported.");
      return FALSE;
   }
   
   if ((0 >= format_ptr->format.sampling_rate)
	   || (BASSBOOST_MAX_SAMPLE_RATE < format_ptr->format.sampling_rate))
   {
	  AR_MSG(DBG_ERROR_PRIO, "CAPI BassBoost: Sampling rate not supported");
      return FALSE;   
   }

   switch (format_ptr->format.num_channels)
   {
      case 0:
         AR_MSG(DBG_ERROR_PRIO, "CAPI BassBoost: Zero channels passed.");
         return FALSE;

      case 1:
         if (PCM_CHANNEL_C != format_ptr->format.channel_type[0])
         {
            AR_MSG(DBG_ERROR_PRIO, "CAPI BassBoost:: Unsupported Mono channel mapping.");
            return FALSE;
         }
         break;

      case 2:
         if ((PCM_CHANNEL_L != format_ptr->format.channel_type[0]) ||
             (PCM_CHANNEL_R != format_ptr->format.channel_type[1]))
         {
            AR_MSG(DBG_ERROR_PRIO, "CAPI BassBoost: Unsupported Stereo channel mapping.");
            return FALSE;
         }
         break;

      case 3:
      case 4:
      case 5:
      case 6:
      case 7:
      case 8:
         break;

      default:
         AR_MSG(DBG_ERROR_PRIO,
                "CAPI BassBoost: Only upto 8 channels supported. Received %lu.",
                format_ptr->format.num_channels);
         return FALSE;
   }

   return TRUE;
}

void capi_bassboost_init_media_fmt(capi_bassboost_t *me_ptr)
{
   capi_media_fmt_v2_t *media_fmt_ptr = &(me_ptr->input_media_fmt[0]);

   media_fmt_ptr->header.format_header.data_format = CAPI_FIXED_POINT;
   media_fmt_ptr->format.bits_per_sample           = 16;
   media_fmt_ptr->format.minor_version             = 16;
   media_fmt_ptr->format.bitstream_format          = CAPI_DATA_FORMAT_INVALID_VAL;
   media_fmt_ptr->format.data_interleaving         = CAPI_DEINTERLEAVED_UNPACKED;
   media_fmt_ptr->format.data_is_signed            = 1;
   media_fmt_ptr->format.num_channels              = CAPI_DATA_FORMAT_INVALID_VAL;
   media_fmt_ptr->format.q_factor                  = PCM_Q_FACTOR_15;
   media_fmt_ptr->format.sampling_rate             = CAPI_DATA_FORMAT_INVALID_VAL;

   for (uint32_t j = 0; j < CAPI_MAX_CHANNELS_V2; j++)
   {
      media_fmt_ptr->format.channel_type[j] = (uint16_t)CAPI_DATA_FORMAT_INVALID_VAL;
   }
}
/* =========================================================================
 * FUNCTION : capi_bassboost_update_event_states
 * DESCRIPTION: Function to update the event states to update using the
 *              callback function
 * =========================================================================*/
static void capi_bassboost_update_event_states(capi_bassboost_t *me_ptr)
{
   me_ptr->events_config.enable  = me_ptr->lib_config.enable;
   me_ptr->events_config.kpps    = capi_bassboost_get_kpps(me_ptr);
   me_ptr->events_config.code_bw = 0;
   /*Honest B/W votes based on sampling rate and channels independently*/
   me_ptr->events_config.data_bw = BASS_BOOST_BW_LOW;

   if ((me_ptr->lib_static_vars.sample_rate <= 48000) && (me_ptr->lib_static_vars.num_chs > 2))
   {
      me_ptr->events_config.data_bw = BASS_BOOST_BW_CH_GREATER_THAN_2;
   }
   else if ((me_ptr->lib_static_vars.sample_rate > 48000) && (me_ptr->lib_static_vars.num_chs <= 2))
   {
      me_ptr->events_config.data_bw = BASS_BOOST_BW_FS_GREATER_THAN_48K;
   }
   else if ((me_ptr->lib_static_vars.sample_rate > 48000) && (me_ptr->lib_static_vars.num_chs > 2))
   {
      me_ptr->events_config.data_bw = BASS_BOOST_BW_HIGHEST;
   }
   bassboost_delay_t delay           = 0;
   uint32_t          actual_size_ptr = 0;
   BASSBOOST_RESULT  lib_result      = BASSBOOST_SUCCESS;
   if (me_ptr->lib_instance.mem_ptr)
   {
      lib_result = bassboost_get_param(&(me_ptr->lib_instance),
                                       BASSBOOST_PARAM_DELAY,
                                       &delay,
                                       sizeof(bassboost_delay_t),
                                       &actual_size_ptr);
      if (BASSBOOST_SUCCESS != lib_result)
      {
         delay = 0;
      }
   }
   me_ptr->events_config.delay_in_us = 0;
   if (me_ptr->lib_static_vars.sample_rate)
   {
      me_ptr->events_config.delay_in_us =
         (uint32_t)(((uint64_t)delay * 1000000) / (me_ptr->lib_static_vars.sample_rate));
   }
}

/* =========================================================================
 * FUNCTION : capi_bassboost_raise_event
 * DESCRIPTION: Function to raise various events of the bass boost module
 * =========================================================================*/
capi_err_t capi_bassboost_update_raise_event(capi_bassboost_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr->cb_info.event_cb)
   {
      BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO,
             "CAPI BassBoost:  Event callback is not set. Unable to raise "
             "events!");
      CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
      return capi_result;
   }
   capi_bassboost_update_event_states(me_ptr);
   capi_result |= capi_cmn_update_process_check_event(&me_ptr->cb_info, me_ptr->events_config.enable);
   capi_result |= capi_cmn_update_kpps_event(&me_ptr->cb_info, me_ptr->events_config.kpps);
   capi_result |=
      capi_cmn_update_bandwidth_event(&me_ptr->cb_info, me_ptr->events_config.code_bw, me_ptr->events_config.data_bw);
   capi_result |= capi_cmn_update_algo_delay_event(&me_ptr->cb_info, me_ptr->events_config.delay_in_us);

   return capi_result;
}

/* =========================================================================
 * FUNCTION : capi_bassboost_check_get_cached_param
 * DESCRIPTION: Function to get the cached configuration parameters from
 *              the Bassboost config
 * =========================================================================*/
static capi_err_t capi_bassboost_check_get_cached_param(capi_bassboost_t *me_ptr,
                                                        uint32_t          param_id,
                                                        void *            param_ptr,
                                                        uint32_t          max_data_len,
                                                        uint32_t          required_size,
                                                        uint32_t *        actual_size_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if (max_data_len >= required_size)
   {
      switch (param_id)
      {
         case BASSBOOST_PARAM_MODE:
         {
            bass_boost_mode_t *mode_ptr = (bass_boost_mode_t *)(param_ptr);
            mode_ptr->bass_boost_mode   = me_ptr->lib_config.mode;
            *actual_size_ptr            = (uint32_t)sizeof(mode_ptr->bass_boost_mode);
            break;
         }

         case BASSBOOST_PARAM_STRENGTH:
         {
            bass_boost_strength_t *strength_ptr = (bass_boost_strength_t *)(param_ptr);
            strength_ptr->strength              = me_ptr->lib_config.strength;
            *actual_size_ptr                    = (uint32_t)sizeof(strength_ptr->strength);
            break;
         }
      }
   }
   else
   {
      BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI BassBoost: Get param needs more memory");
      CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
   }

   return capi_result;
}

/* =========================================================================
 * FUNCTION : capi_bassboost_check_get_param
 * DESCRIPTION: Function to get the configuration parameters from the
 *              Bassbosst library
 * =========================================================================*/
capi_err_t capi_bassboost_check_get_param(capi_bassboost_t *me_ptr,
                                          uint32_t          param_id,
                                          void *            param_ptr,
                                          uint32_t          max_data_len,
                                          uint32_t          required_size,
                                          uint32_t *        actual_size_ptr)
{
   BASSBOOST_RESULT lib_result  = BASSBOOST_SUCCESS;
   capi_err_t       capi_result = CAPI_EOK;

   if (me_ptr->lib_instance.mem_ptr == NULL)
   {
      capi_result |= capi_bassboost_check_get_cached_param(me_ptr,
                                                           param_id,
                                                           param_ptr,
                                                           max_data_len,
                                                           required_size,
                                                           actual_size_ptr);
      if (CAPI_FAILED(capi_result))
      {
         BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI BassBoost: Get parameter failed from the cached params");
         return capi_result;
      }
   }
   else
   {
      if (max_data_len >= required_size)
      {
         lib_result = bassboost_get_param(&(me_ptr->lib_instance), param_id, param_ptr, max_data_len, actual_size_ptr);
         if (BASSBOOST_SUCCESS != lib_result)
         {
            BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI BassBoost: get parameter failed");
            CAPI_SET_ERROR(capi_result, CAPI_EFAILED);
         }
      }
      else
      {
         BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI BassBoost: get parameter needs more memory");
         CAPI_SET_ERROR(capi_result, CAPI_EFAILED);
      }
   }
   return capi_result;
}

/* =========================================================================
 * FUNCTION : capi_bassboost_check_set_param
 * DESCRIPTION: Function to set the configuration parameters to the
 *              Bassboost library
 * =========================================================================*/
capi_err_t capi_bassboost_check_set_param(capi_bassboost_t *me_ptr,
                                          uint32_t          param_id,
                                          void *            param_ptr,
                                          uint32_t          param_size,
                                          uint32_t          required_size)
{
   BASSBOOST_RESULT lib_result  = BASSBOOST_SUCCESS;
   capi_err_t       capi_result = CAPI_EOK;

   if (me_ptr->lib_instance.mem_ptr == NULL)
   {
      return capi_result;
   }
   else
   {
      if (param_size >= required_size)
      {
         lib_result = bassboost_set_param(&(me_ptr->lib_instance), param_id, param_ptr, param_size);
         if (BASSBOOST_SUCCESS != lib_result)
         {
            BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI BassBoost: set parameter failed %d", lib_result);
            CAPI_SET_ERROR(capi_result, CAPI_EFAILED);
         }
      }
      else
      {
         BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI BassBoost: set parameter needs more memory");
         CAPI_SET_ERROR(capi_result, CAPI_EFAILED);
      }
   }
   return capi_result;
}

/* =========================================================================
 * FUNCTION : capi_bassboost_set_cached_lib_config
 * DESCRIPTION: Sets either a parameter value or a parameter structure
 * containing multiple parameters. In the event of a failure,
 * the appropriate error code is returned.
 * =========================================================================*/
static capi_err_t capi_bassboost_set_cached_lib_config(capi_bassboost_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if (me_ptr->lib_config.is_enable_set)
   {
      bassboost_enable_t enable_flag = me_ptr->lib_config.enable;
      capi_result |= capi_bassboost_check_set_param(me_ptr,
                                                    BASSBOOST_PARAM_ENABLE,
                                                    (void *)&(enable_flag),
                                                    (uint32_t)sizeof(enable_flag),
                                                    (uint32_t)sizeof(bassboost_enable_t));
      if (1 == me_ptr->lib_config.enable)
      {
         me_ptr->events_config.code_bw = 0;
         me_ptr->events_config.kpps    = capi_bassboost_get_kpps(me_ptr);
         /*Honest B/W votes based on sampling rate and channels independently*/
         me_ptr->events_config.data_bw = BASS_BOOST_BW_LOW;
         if ((me_ptr->lib_static_vars.sample_rate > 48000) && (me_ptr->lib_static_vars.num_chs <= 2))
         {
            me_ptr->events_config.data_bw = BASS_BOOST_BW_FS_GREATER_THAN_48K;
         }
         else if ((me_ptr->lib_static_vars.sample_rate <= 48000) && (me_ptr->lib_static_vars.num_chs > 2))
         {
            me_ptr->events_config.data_bw = BASS_BOOST_BW_CH_GREATER_THAN_2;
         }
         else if ((me_ptr->lib_static_vars.sample_rate > 48000) && (me_ptr->lib_static_vars.num_chs > 2))
         {
            me_ptr->events_config.data_bw = BASS_BOOST_BW_HIGHEST;
         }
      }

      if (CAPI_FAILED(capi_result))
      {
         BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPIv2 BassBoost: Set Cached Enabled failed %lu", capi_result);
         return capi_result;
      }
      else
      {
         capi_bassboost_update_raise_event(me_ptr);
      }
   }

   if (me_ptr->lib_config.is_mode_set)
   {
      bassboost_mode_t mode = (bassboost_mode_t)(me_ptr->lib_config.mode);
      capi_result |= capi_bassboost_check_set_param(me_ptr,
                                                    BASSBOOST_PARAM_MODE,
                                                    (void *)&(mode),
                                                    (uint32_t)sizeof(mode),
                                                    (uint32_t)sizeof(bassboost_mode_t));
      if (CAPI_FAILED(capi_result))
      {
         BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPIv2 BassBoost: Set cached mode failed %lu", capi_result);
         return capi_result;
      }
   }

   if (me_ptr->lib_config.is_strength_set)
   {
      bassboost_strength_t strength = (bassboost_strength_t)(me_ptr->lib_config.strength);
      capi_result |= capi_bassboost_check_set_param(me_ptr,
                                                    BASSBOOST_PARAM_STRENGTH,
                                                    (void *)&(strength),
                                                    (uint32_t)sizeof(strength),
                                                    (uint32_t)sizeof(bassboost_strength_t));
      if (CAPI_FAILED(capi_result))
      {
         BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPIv2 BassBoost: Set Cached strength failed %lu", capi_result);
         return capi_result;
      }
   }
   return capi_result;
}

static capi_err_t capi_bassboost_check_create_lib_instance(capi_bassboost_t *me_ptr)
{
   bassboost_mem_req_t lib_mem_req;
   capi_err_t          capi_result = CAPI_EOK;
   BASSBOOST_RESULT    lib_result  = BASSBOOST_SUCCESS;
   void *              ptr         = NULL;

   int32_t framesize_10ms                = align_to_8_byte(me_ptr->input_media_fmt[0].format.sampling_rate / 100);
   me_ptr->lib_static_vars.data_width    = me_ptr->input_media_fmt[0].format.bits_per_sample;
   me_ptr->lib_static_vars.num_chs       = me_ptr->input_media_fmt[0].format.num_channels;
   me_ptr->lib_static_vars.sample_rate   = me_ptr->input_media_fmt[0].format.sampling_rate;
   me_ptr->lib_static_vars.limiter_delay = LIMITER_DEFAULT_DELAY_MS;
   me_ptr->lib_static_vars.max_block_size =
      framesize_10ms < BASS_BOOST_MAX_FRAME_SIZE ? framesize_10ms : BASS_BOOST_MAX_FRAME_SIZE;

   lib_result = bassboost_get_mem_req(&lib_mem_req, &(me_ptr->lib_static_vars));
   if (BASSBOOST_SUCCESS != lib_result)
   {
      BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI BassBoost: error getting library memory requirements %d", lib_result);
      CAPI_SET_ERROR(capi_result, CAPI_EFAILED);
      return capi_result;
   }

   lib_mem_req.mem_size = align_to_8_byte(lib_mem_req.mem_size);

   /* create the new library as needed */
   if ((0 == me_ptr->lib_mem_req.mem_size) ||
       (lib_mem_req.mem_size > me_ptr->lib_mem_req.mem_size) || // Make lib mem req =0 after 430.
       (NULL == me_ptr->lib_instance.mem_ptr))
   {
      if (NULL != me_ptr->lib_instance.mem_ptr)
      {
         posal_memory_free(me_ptr->lib_instance.mem_ptr);
         me_ptr->lib_instance.mem_ptr = NULL;
      }
      ptr = posal_memory_malloc(lib_mem_req.mem_size, POSAL_HEAP_DEFAULT);
      if (NULL == ptr)
      {
         BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI BassBoost: failed allocating memory for bassboost library");
         CAPI_SET_ERROR(capi_result, CAPI_EFAILED);
         return capi_result;
      }
      me_ptr->lib_mem_req.mem_size = lib_mem_req.mem_size;
   }
   else
   {
      ptr = me_ptr->lib_instance.mem_ptr;
   }

   lib_result =
      bassboost_init_mem(&(me_ptr->lib_instance), &(me_ptr->lib_static_vars), ptr, me_ptr->lib_mem_req.mem_size);
   if (BASSBOOST_SUCCESS != lib_result)
   {
      BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI BassBoost: library initialization failed");
      return CAPI_EFAILED;
   }

   capi_result |= capi_bassboost_set_cached_lib_config(me_ptr);

   /* Library reset */
   lib_result = bassboost_set_param(&(me_ptr->lib_instance), BASSBOOST_PARAM_RESET, NULL, 0);
   if (BASSBOOST_SUCCESS != lib_result)
   {
      BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI BassBoost: library Reset failed");
      CAPI_SET_ERROR(capi_result, CAPI_EFAILED);
      return capi_result;
   }

   BASSBOOST_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI BassBoost: library created successfully");
   return capi_result;
}

#ifdef DO_BASSBOOST_PROFILING
void capi_bassboost_print_kpps(capi_bassboost_t *me_ptr)
{
   me_ptr->kpps_profile_data.average_kpps =
      ((me_ptr->kpps_profile_data.total_cycles / me_ptr->kpps_profile_data.sample_count) *
       me_ptr->kpps_profile_data.sample_rate) /
      (1000);
   BASSBOOST_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI BassBoost: Total cycles :%llu", me_ptr->kpps_profile_data.total_cycles);
   BASSBOOST_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI BassBoost: Total Sample processed: %llu", me_ptr->kpps_profile_data.sample_count);
   BASSBOOST_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI BassBoost: Average KPPS: %llu", me_ptr->kpps_profile_data.average_kpps);
   BASSBOOST_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI BassBoost: Peak KPPS : %llu", me_ptr->kpps_profile_data.peak_kpps);
   me_ptr->kpps_profile_data.total_cycles = 0;
   me_ptr->kpps_profile_data.peak_kpps    = 0;
   me_ptr->kpps_profile_data.sample_count = 0;
   me_ptr->kpps_profile_data.average_kpps = 0;
}

void capi_bassboost_profiling(capi_bassboost_t *me_ptr, uint32_t num_samples)
{
   me_ptr->kpps_profile_data.sample_rate = me_ptr->media_fmt[0].format.sampling_rate;
   me_ptr->kpps_profile_data.total_cycles =
      me_ptr->kpps_profile_data.total_cycles +
      (me_ptr->kpps_profile_data.end_cycles - me_ptr->kpps_profile_data.start_cycles);
   me_ptr->kpps_profile_data.sample_count += num_samples;
   uint64_t frame_kpps =
      (((me_ptr->kpps_profile_data.end_cycles - me_ptr->kpps_profile_data.start_cycles) / num_samples) *
       me_ptr->kpps_profile_data.sample_rate) /
      (1000);
   if ((frame_kpps > me_ptr->kpps_profile_data.peak_kpps) &&
       (num_samples > (me_ptr->kpps_profile_data.sample_rate / 1000)))
   {
      me_ptr->kpps_profile_data.peak_kpps = frame_kpps;
   }
}
#endif /* DO_BASSBOOST_PROFILING */

capi_err_t capi_bassboost_process_set_properties(capi_bassboost_t *me_ptr, capi_proplist_t *proplist_ptr)
{
   capi_err_t   capi_result = CAPI_EOK;
   capi_prop_t *prop_array  = proplist_ptr->prop_ptr;

   if (NULL == prop_array)
   {
      BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI Bassboost: Set property received null property array.");

      return CAPI_EBADPARAM;
   }

   capi_result = capi_cmn_set_basic_properties(proplist_ptr, &me_ptr->heap, &me_ptr->cb_info, TRUE);
   if (CAPI_EOK != capi_result)
   {
      BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI Bassboost: Get basic properties failed with result %lu", capi_result);

      return capi_result;
   }

   for (uint32_t i = 0; i < proplist_ptr->props_num; i++)
   {
      const capi_buf_t *const payload_ptr = &(prop_array[i].payload);
      capi_result                         = CAPI_EOK;

      switch (prop_array[i].id)
      {
         case CAPI_PORT_NUM_INFO:
         case CAPI_HEAP_ID:
         case CAPI_EVENT_CALLBACK_INFO:
         {
            break;
         }

         case CAPI_ALGORITHMIC_RESET:
         {
            if (NULL != me_ptr->lib_instance.mem_ptr)
            {
               BASSBOOST_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI BassBoost: Reset");
               BASSBOOST_RESULT lib_result = BASSBOOST_SUCCESS;
               lib_result = bassboost_set_param(&(me_ptr->lib_instance), BASSBOOST_PARAM_RESET, NULL, 0);
               if (BASSBOOST_SUCCESS != lib_result)
               {
                  CAPI_SET_ERROR(capi_result, CAPI_EFAILED);
               }
            }
            break;
         }

         case CAPI_INPUT_MEDIA_FORMAT_V2:
         {
            if (payload_ptr->actual_data_len >= sizeof(capi_media_fmt_v2_t))
            {
               BASSBOOST_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI BassBoost: Received Input media format");

               capi_media_fmt_v2_t *data_ptr = (capi_media_fmt_v2_t *)(payload_ptr->data_ptr);
               if (!bassboost_is_supported_media_type(data_ptr))
               {
                  CAPI_SET_ERROR(capi_result, CAPI_EFAILED);
                  break;
               }
               // copy and save the input media fmt
               memscpy(me_ptr->input_media_fmt,
                       sizeof(capi_media_fmt_v2_t),
                       data_ptr,
                       sizeof(capi_media_fmt_v2_t)); // CH Mapping
               capi_result = capi_bassboost_check_create_lib_instance(me_ptr);
               if (CAPI_FAILED(capi_result))
               {
                  BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI Bassboost: Library creation failed");
                  break; // this is catastrophic error, return
               }
               // raise event for output media format
               capi_result |= capi_bassboost_update_raise_event(me_ptr);
               capi_result |=
                  capi_cmn_output_media_fmt_event_v2(&me_ptr->cb_info,
                                                     &me_ptr->input_media_fmt[CAPI_BASSBOOST_DEFAULT_PORT],
                                                     FALSE,
                                                     CAPI_BASSBOOST_DEFAULT_PORT); // raise event for output media
                                                                                   // format
            }
            else
            {
               BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                      "CAPI BassBoost: Set property id 0x%lx Bad param size %lu",
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
  #if 0
                 AR_MSG(DBG_LOW_PRIO,
                         "CAPI BassBoost: This module-id 0x%08lX, instance-id 0x%08lX",
                         data_ptr->module_id,
                         me_ptr->miid);
  #endif
              }
              else
              {
                 AR_MSG(DBG_ERROR_PRIO,
                         "CAPI BassBoost: Set, Param id 0x%lx Bad param size %lu",
                         (uint32_t)prop_array[i].id,
                         payload_ptr->actual_data_len);
                 CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
              }
              break;
         }
		 
         default:
         {
            BASSBOOST_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI BassBoost: Set property id %#x. Not supported.", prop_array[i].id);
            CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
            break;
         }
      } /* switch (prop_array[i].id) */

      if (CAPI_FAILED(capi_result))
      {
         BASSBOOST_MSG(me_ptr->miid, DBG_HIGH_PRIO,
                "CAPI Bassboost: Set property for %#x failed with opcode %lu",
                prop_array[i].id,
                capi_result);
      }
   }

   return capi_result;
}

capi_err_t capi_bassboost_process_get_properties(capi_bassboost_t *me_ptr, capi_proplist_t *proplist_ptr)
{
   uint32_t i;

   capi_err_t result   = CAPI_EOK;
   capi_err_t result_2 = CAPI_EOK;

   capi_prop_t *prop_array = proplist_ptr->prop_ptr;
   uint32_t miid = MIID_UNKNOWN; 
    if(me_ptr) 
    { 
		miid = me_ptr->miid;
    }
   if (NULL == prop_array)
   {
      BASSBOOST_MSG(miid, DBG_ERROR_PRIO, "CAPI Bassboost: Get property received null property array.");

      return CAPI_EBADPARAM;
   }

   capi_basic_prop_t mod_prop;
   mod_prop.init_memory_req    = sizeof(capi_bassboost_t);
   mod_prop.stack_size         = BASSBOOST_STACK_SIZE;
   mod_prop.num_fwk_extns      = 0;     // NA
   mod_prop.fwk_extn_ids_arr   = NULL;  // NA
   mod_prop.is_inplace         = FALSE; // not capable of in-place processing of data
   mod_prop.req_data_buffering = FALSE;
   mod_prop.max_metadata_size  = 0; // NA

   result = capi_cmn_get_basic_properties(proplist_ptr, &mod_prop);
   if (CAPI_EOK != result)
   {
      BASSBOOST_MSG(miid, DBG_ERROR_PRIO, "CAPI Bassboost: Get common basic properties failed with result %lu", result);

      return result;
   }

   for (i = 0; i < proplist_ptr->props_num; i++)
   {
      capi_buf_t *const payload_ptr = &(prop_array[i].payload);
      const uint32_t    param_id    = prop_array[i].id;

      if (NULL == payload_ptr->data_ptr)
      {
         BASSBOOST_MSG(miid, DBG_ERROR_PRIO, "CAPI Bassboost: Get property id 0x%lx, received null buffer.", param_id);
         CAPI_SET_ERROR(result, CAPI_EBADPARAM);
         continue;
      }

      result = CAPI_EOK;

      switch (prop_array[i].id)
      {
         case CAPI_INIT_MEMORY_REQUIREMENT:
         case CAPI_IS_INPLACE:
         case CAPI_REQUIRES_DATA_BUFFERING:
         case CAPI_STACK_SIZE:
         case CAPI_PORT_DATA_THRESHOLD:
         case CAPI_MAX_METADATA_SIZE:
         {
            break;
         }

         case CAPI_OUTPUT_MEDIA_FORMAT_SIZE:
         {
            if (payload_ptr->max_data_len >= sizeof(capi_output_media_format_size_t))
            {
               capi_output_media_format_size_t *data_ptr = (capi_output_media_format_size_t *)payload_ptr->data_ptr;
               data_ptr->size_in_bytes                   = sizeof(capi_standard_data_format_t);
               payload_ptr->actual_data_len              = sizeof(capi_output_media_format_size_t);
            }
            else
            {
               BASSBOOST_MSG(miid, DBG_ERROR_PRIO,
                      "CAPI Bassboost: Get Property id 0x%lx Bad param size %lu",
                      (uint32_t)prop_array[i].id,
                      payload_ptr->max_data_len);
               CAPI_SET_ERROR(result, CAPI_ENEEDMORE);
            }

            break;
         }

         case CAPI_OUTPUT_MEDIA_FORMAT_V2:
         {
            if (payload_ptr->max_data_len >= sizeof(capi_media_fmt_v2_t))
            {
               capi_media_fmt_v2_t *data_ptr = (capi_media_fmt_v2_t *)payload_ptr->data_ptr;

               if (NULL == me_ptr || NULL == me_ptr->lib_instance.mem_ptr ||
                   (prop_array[i].port_info.is_valid && prop_array[i].port_info.port_index != 0))
               {
                  BASSBOOST_MSG(miid, DBG_ERROR_PRIO,
                         "CAPI Bassboost: Get Property id 0x%lx failed due to "
                         "invalid/unexpected values",
                         (uint32_t)prop_array[i].id);
                  CAPI_SET_ERROR(result, CAPI_EFAILED);

                  break;
               }

               *data_ptr                    = me_ptr->input_media_fmt[0];
               payload_ptr->actual_data_len = sizeof(capi_media_fmt_v2_t);
            }
            else
            {
               BASSBOOST_MSG(miid, DBG_ERROR_PRIO,
                      "CAPI Bassboost: Get Property id 0x%lx Bad param size %lu",
                      (uint32_t)prop_array[i].id,
                      payload_ptr->max_data_len);
               CAPI_SET_ERROR(result, CAPI_ENEEDMORE);
            }

            break;
         }

         default:
         {
            CAPI_SET_ERROR(result, CAPI_EUNSUPPORTED);
            break;
         }
      } /* switch (prop_array[i].id) */

      CAPI_SET_ERROR(result_2, result);

      if (CAPI_FAILED(result))
      {
         BASSBOOST_MSG(miid, DBG_HIGH_PRIO, "CAPI Bassboost: Get property for %#x failed with opcode %lu", prop_array[i].id, result);
      }
   }

   return result_2;
}
