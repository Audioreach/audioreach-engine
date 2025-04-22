/* ======================================================================== */
/*
@file capi_multistageiir_utils.cpp

   Source file to implement the Audio Post Processor Interface for Multi-Stage
   IIR filters
*/

/* =========================================================================
   * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   * SPDX-License-Identifier: BSD-3-Clause-Clear
  ========================================================================= */

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "capi_multistageiir_utils.h"
#include "msiir_calibration_api.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define MSIIR_MASK_WIDTH 32

/*------------------------------------------------------------------------
 * Function declarations
 * -----------------------------------------------------------------------*/
static capi_err_t capi_msiir_set_params_to_lib(capi_multistageiir_t *me);

static capi_err_t capi_msiir_create_new_msiir_filters(capi_multistageiir_t *me, uint32_t instance_id, uint32_t ch);

static inline uint32_t align_to_8_byte(const uint32_t num)
{
   return ((num + 7) & (0xFFFFFFF8));
}

int32_t capi_msiir_s32_min_s32_s32(int32_t x, int32_t y)
{
   return ((x < y) ? x : y);
}

static bool_t capi_msiir_is_valid_channel_type(const uint8_t channel_type)
{
   if (channel_type < PCM_CHANNEL_L)
   {
      return FALSE;
   }
   if (channel_type > PCM_MAX_CHANNEL_MAP_V2)
   {
      return FALSE;
   }
   return TRUE;
}

static capi_err_t capi_msiir_check_raise_bw_event(capi_multistageiir_t *me)
{
   const uint32_t bw = 0;

   if (NULL == me->cb_info.event_cb)
   {
      MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Event callback is not set. Unable to raise events!");
      return CAPI_EUNSUPPORTED;
   }

   // Raise event if bw is changed.
   if (bw != me->bw)
   {
      capi_err_t             capi_result = CAPI_EOK;
      capi_event_bandwidth_t event;
      capi_event_info_t      event_info;

      event.code_bandwidth = 0;
      event.data_bandwidth = bw;

      event_info.port_info.is_valid      = FALSE;
      event_info.payload.actual_data_len = sizeof(event);
      event_info.payload.max_data_len    = sizeof(event);
      event_info.payload.data_ptr        = reinterpret_cast<int8_t *>(&event);
      capi_result = me->cb_info.event_cb(me->cb_info.event_context, CAPI_EVENT_BANDWIDTH, &event_info);

      if (CAPI_FAILED(capi_result))
      {
         MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR: Failed to send BW event.");
      }
      else
      {
         me->bw = bw;
      }
   }
   return CAPI_EOK;
}

uint32_t capi_msiir_get_kpps(capi_multistageiir_t *me)
{
   uint64_t kpps              = 0;
   uint64_t kpps_per_channel  = 0;
   uint64_t frame_size        = 0;
   uint32_t num_channels      = me->media_fmt[0].format.num_channels;
   uint32_t num_biquad_stages = 0;
   uint32_t bits_per_sample   = me->media_fmt[0].format.bits_per_sample;
   uint32_t sampling_rate     = me->media_fmt[0].format.sampling_rate;
   if ((sampling_rate == 0) || (!me->media_fmt_received))
   {
      return 0;
   }

   frame_size = (((uint64_t)(me->cntr_frame_size_us) * sampling_rate) / 1000000);
   if (0 == frame_size)
   {
      return 0;
   }

   for (uint32_t ch = 0; ch < num_channels; ch++)
   {
      kpps_per_channel = 0;
      // channel_type has values range from 1 to 16
      uint8_t channel_type = me->media_fmt[0].channel_type[ch];

      // convert channel_type to channel index, channel index has range 0 ~ 7
      int32_t ch_idx    = me->channel_map_to_index[channel_type];
      num_biquad_stages = me->per_chan_msiir_cfg_max[ch_idx].num_stages;
      if (me->enable_flag[ch])
      {
         if (bits_per_sample == 16)
         {
            kpps_per_channel =
               (((NUM_PACKETS_PER_SAMPLE_16BIT * frame_size + NUM_PACKETS_PER_STAGE_WITH_ZERO_SAMPLE_16BIT) *
                    num_biquad_stages +
                 NUM_PACKETS_WITH_ZERO_STAGE_16BIT) *
                sampling_rate) /
               (frame_size * 1000);
         }
         if (bits_per_sample == 32)
         {
            kpps_per_channel =
               (((NUM_PACKETS_PER_SAMPLE_32BIT * frame_size + NUM_PACKETS_PER_STAGE_WITH_ZERO_SAMPLE_32BIT) *
                    num_biquad_stages +
                 NUM_PACKETS_WITH_ZERO_STAGE_32BIT) *
                sampling_rate) /
               (frame_size * 1000);
         }
      }
      kpps += kpps_per_channel;
   }
   MSIIR_MSG(me->miid, DBG_HIGH_PRIO, "CAPI MSIIR : Updated KPPS value is %lu", (uint32_t)kpps);
   return (uint32_t)kpps;
}

capi_err_t capi_msiir_check_raise_kpps_event(capi_multistageiir_t *me_ptr, uint32_t val)
{
   capi_err_t result = CAPI_EOK;

   if (me_ptr->kpps == val)
   {
      return result;
   }
   if (CAPI_EOK == (result = capi_cmn_update_kpps_event(&me_ptr->cb_info, val)))
   {
      me_ptr->kpps = val;
   }
   return result;
}

static capi_err_t capi_msiir_check_raise_delay_event(capi_multistageiir_t *me_ptr, uint32_t delay)
{
   if (me_ptr->delay == delay)
   {
      return CAPI_EOK;
   }
   me_ptr->delay = delay;

   return capi_cmn_update_algo_delay_event(&me_ptr->cb_info, me_ptr->delay);
}

capi_err_t capi_msiir_raise_process_check_event(capi_multistageiir_t *me_ptr)
{
   uint32_t process_check = 0;

   for (uint32_t ch = 0; ch < me_ptr->media_fmt[0].format.num_channels; ch++)
   {
      // if the current channel is enabled, set TRUE and break the loop
      process_check |= me_ptr->enable_flag[ch];
   }
   process_check &= (uint32_t)me_ptr->enable; // Over-riding with the global enable flag

   return capi_cmn_update_process_check_event(&me_ptr->cb_info, process_check);
}

capi_err_t capi_msiir_update_delay_event(capi_multistageiir_t *me)
{
   uint32_t     delay      = 0;
   MSIIR_RESULT result_lib = MSIIR_SUCCESS;

   // find the max number of stages from all channels
   int32_t                          max_stages = 0;
   capi_one_chan_msiir_config_max_t one_chan_msiir_cfg_max;

   for (uint32_t ch = 0; ch < me->media_fmt[0].format.num_channels; ch++)
   {
      msiir_lib_t *obj_ptr = NULL;

      if ((me->cross_fade_flag[ch] == 0) || (me->msiir_new_lib[ch].mem_ptr == NULL))
      {
          obj_ptr = (msiir_lib_t *)(&me->msiir_lib[ch]);
      }
      else
      {
          obj_ptr = (msiir_lib_t *)(&me->msiir_new_lib[ch]);
      }

      uint32_t actual_param_size = 0;

      result_lib = msiir_get_param(obj_ptr,
                                   MSIIR_PARAM_CONFIG,
                                   (void *)&one_chan_msiir_cfg_max,
                                   sizeof(one_chan_msiir_cfg_max),
                                   &actual_param_size);

      if (actual_param_size > sizeof(one_chan_msiir_cfg_max) || (MSIIR_SUCCESS != result_lib))
      {
         MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : get delay error result %d, size %lu", result_lib, actual_param_size);
         return CAPI_EFAILED;
      }
      if (me->enable_flag[ch])
      {
         max_stages =
            (max_stages > one_chan_msiir_cfg_max.num_stages) ? max_stages : (one_chan_msiir_cfg_max.num_stages);
      }
   }
   // assume each IIR stage has MSIIR_FILTER_STATES(2) samples of delay
   delay = MSIIR_FILTER_STATES * max_stages;
   delay = (delay * 1000000) / (me->media_fmt[0].format.sampling_rate);

   return capi_msiir_check_raise_delay_event(me, delay);
}

static bool_t capi_msiir_is_supported_media_type_v2(capi_multistageiir_t *me, const capi_media_fmt_v2_t *format_ptr)
{
   if ((format_ptr->format.bits_per_sample != 16) && (format_ptr->format.bits_per_sample != 32))
   {
      MSIIR_MSG(me->miid, DBG_ERROR_PRIO,
             "CAPI MSIIR : Only 16/24 bit data supported. Received %lu.",
             format_ptr->format.bits_per_sample);
      return FALSE;
   }

   if ((format_ptr->format.data_interleaving != CAPI_DEINTERLEAVED_UNPACKED) && (format_ptr->format.num_channels != 1))
   {
      MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Interleaved data not supported.");
      return FALSE;
   }

   if (!format_ptr->format.data_is_signed)
   {
      MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Unsigned data not supported.");
      return FALSE;
   }

   if ((format_ptr->format.q_factor != PCM_Q_FACTOR_15) && (format_ptr->format.q_factor != PCM_Q_FACTOR_27))
   {
      MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPIv2 MSIIR: Q factor %lu is not supported.", format_ptr->format.q_factor);
      return FALSE;
   }

   if ((format_ptr->format.num_channels == 0) || (format_ptr->format.num_channels > IIR_TUNING_FILTER_MAX_CHANNELS_V2))
   {
      MSIIR_MSG(me->miid, DBG_ERROR_PRIO,
             "CAPI MSIIR : Init received with %lu channels. Currently only %d are supported.",
             format_ptr->format.num_channels,
             IIR_TUNING_FILTER_MAX_CHANNELS_V2);
      return FALSE;
   }

   for (uint32_t ch = 0; ch < format_ptr->format.num_channels; ch++)
   {
      if (!capi_msiir_is_valid_channel_type(format_ptr->channel_type[ch]))
      {
         MSIIR_MSG(me->miid, DBG_ERROR_PRIO,
                "CAPI MSIIR : channel number %lu has unsupported type %hu",
                ch,
                format_ptr->channel_type[ch]);
         return FALSE;
      }
      if(format_ptr->channel_type[ch] > PCM_MAX_CHANNEL_MAP)
      {
         //If a higher channel map in the media format occurs even once in the execution history,
         //the flag will be set to true from that point onward.
         me->higher_channel_map_present = TRUE;
      }
   }

   return TRUE;
}

capi_err_t capi_msiir_init_media_fmt(capi_multistageiir_t *me)
{
   capi_media_fmt_v2_t *media_fmt_ptr = &(me->media_fmt[0]);

   media_fmt_ptr->header.format_header.data_format = CAPI_FIXED_POINT;
   media_fmt_ptr->format.minor_version             = CAPI_MEDIA_FORMAT_MINOR_VERSION;
   media_fmt_ptr->format.bits_per_sample           = 16;
   media_fmt_ptr->format.bitstream_format          = CAPI_DATA_FORMAT_INVALID_VAL;
   media_fmt_ptr->format.data_interleaving         = CAPI_DEINTERLEAVED_UNPACKED;
   media_fmt_ptr->format.data_is_signed            = 1;
   media_fmt_ptr->format.num_channels              = CAPI_DATA_FORMAT_INVALID_VAL;
   media_fmt_ptr->format.q_factor                  = PCM_Q_FACTOR_15;
   media_fmt_ptr->format.sampling_rate             = CAPI_DATA_FORMAT_INVALID_VAL;

   memset(media_fmt_ptr->channel_type, (uint8_t)CAPI_DATA_FORMAT_INVALID_VAL, sizeof(media_fmt_ptr->channel_type));

   return CAPI_EOK;
}

static capi_err_t capi_msiir_get_init_memory_req(uint32_t *size_ptr, uint32_t miid)
{
   cross_fade_lib_mem_req_t one_chan_cross_fade_lib_mem_req;
   cross_fade_static_t      cross_fade_static_vars;

   if (NULL == size_ptr)
   {
      MSIIR_MSG(miid, DBG_ERROR_PRIO, "CAPI MSIIR : Received bad pointer, 0x%p", size_ptr);
      return CAPI_EBADPARAM;
   }

   uint32_t num_channels   = IIR_TUNING_FILTER_MAX_CHANNELS_V2;
   uint32_t total_mem_size = 0;

   // cross fade library size
   cross_fade_static_vars.data_width  = 2; // cross fade use 1(16bits) or 2(32bits)
   cross_fade_static_vars.sample_rate = MSIIR_DEFAULT_SAMPLE_RATE;

   one_chan_cross_fade_lib_mem_req.cross_fade_lib_mem_size   = 0;
   one_chan_cross_fade_lib_mem_req.cross_fade_lib_stack_size = 0;

   if (CROSS_FADE_SUCCESS != audio_cross_fade_get_mem_req(&one_chan_cross_fade_lib_mem_req, &cross_fade_static_vars))
   {
      MSIIR_MSG(miid, DBG_ERROR_PRIO, "CAPI MSIIR : Get Init memory required failed get cross fade size");
      return CAPI_EFAILED;
   }

   /* memory is allocated for the capi structure of MSIIR and for the crossfade library.
      Crossfade library would be initialized after the media format has been received.
      */
   total_mem_size += num_channels * align_to_8_byte(one_chan_cross_fade_lib_mem_req.cross_fade_lib_mem_size);
   total_mem_size += align_to_8_byte(sizeof(capi_multistageiir_t));

   *size_ptr = total_mem_size;
#ifdef CAPI_MSIIR_DEBUG_MSG
   MSIIR_MSG(miid, DBG_HIGH_PRIO, "CAPI MSIIR : Get Init memory required done, requires %lu bytes", *size_ptr);
#endif
   return CAPI_EOK;
}

static capi_err_t capi_msiir_init_capi(capi_multistageiir_t *me, bool_t media_format_version)
{
   if (NULL == me)
   {
      MSIIR_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "CAPI MSIIR : Init received bad pointer, 0x%p", me);
      return CAPI_EBADPARAM;
   }

   MSIIR_RESULT result_lib = MSIIR_SUCCESS;

   CROSS_FADE_RESULT result_cross_fade_lib = CROSS_FADE_SUCCESS;

   bool_t any_crossfade_in_progress = FALSE;

   // calculate the mem size for MSIIR library and re-alloc if needed
   msiir_mem_req_t msiir_mem_req_by_reinit;

   msiir_mem_req_by_reinit.mem_size = 0;

   uint32_t msiir_bits_per_sample_before = (uint32_t)(me->msiir_static_vars.data_width);
   me->msiir_static_vars.data_width      = (int32_t)me->media_fmt[0].format.bits_per_sample;

   result_lib = msiir_get_mem_req(&msiir_mem_req_by_reinit, &(me->msiir_static_vars));
   if (MSIIR_SUCCESS != result_lib)
   {
      MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Init error %d", result_lib);
      return CAPI_EFAILED;
   }

   // Check if any crossfade is in progress.
   for (uint32_t ch = 0; ch < me->num_channels_allocated; ch++)
   {
      if (NULL == me->cross_fade_lib[ch].cross_fade_lib_mem_ptr)
      {
         continue;
      }
      uint32_t cur_crossfade_mode = FALSE;
      uint32_t param_size         = 0;

      result_cross_fade_lib = audio_cross_fade_get_param(&(me->cross_fade_lib[ch]),
                                                         CROSS_FADE_PARAM_MODE,
                                                         (int8 *)&(cur_crossfade_mode),
                                                         (uint32)sizeof(cur_crossfade_mode),
                                                         (uint32 *)&param_size);

      if ((CROSS_FADE_SUCCESS != result_cross_fade_lib))
      {
         MSIIR_MSG(me->miid, DBG_ERROR_PRIO,
                "CAPI MSIIR : Failed to get cross fade param module for ch %ld, result 0x%lx",
                ch,
                result_cross_fade_lib);
         return CAPI_EFAILED;
      }

      if (cur_crossfade_mode)
      {
         any_crossfade_in_progress = TRUE;

         me->cross_fade_flag[ch] = 0; //reset the flag and set it to the cross fade lib

         result_cross_fade_lib = audio_cross_fade_set_param(&(me->cross_fade_lib[ch]),
                                                            CROSS_FADE_PARAM_MODE,
                                                            (int8 *)&(me->cross_fade_flag[ch]),
                                                            (uint32)sizeof(me->cross_fade_flag[ch]));
         //loop through all the channels and reset the crossfade flag for all the crossfade enabled channels
      }
   }

   // if the previous malloc of MSIIR library size is smaller than the current requirement, re-alloc the MSIIR library
   if ((me->per_chan_mem_req.mem_size != (align_to_8_byte(msiir_mem_req_by_reinit.mem_size))) ||
       (me->num_channels_allocated != me->media_fmt[0].format.num_channels) || any_crossfade_in_progress)
   {
      capi_err_t result_capi = CAPI_EOK;

      // clear all the previous MSIIR library instances
      for (uint32_t ch = 0; ch < IIR_TUNING_FILTER_MAX_CHANNELS_V2; ch++)
      {
         if (NULL != me->msiir_lib[ch].mem_ptr)
         {
            posal_memory_free(me->msiir_lib[ch].mem_ptr);
            me->msiir_lib[ch].mem_ptr = NULL;
         }

         // if cross fading is active, release the new filters
         if (NULL != me->msiir_new_lib[ch].mem_ptr)
         {
            posal_memory_free(me->msiir_new_lib[ch].mem_ptr);
            me->msiir_new_lib[ch].mem_ptr = NULL;
         }

         me->cross_fade_lib[ch].cross_fade_lib_mem_ptr = NULL;
      }
      // allocate MSIIR lib instances
      for (uint32_t ch = 0; ch < me->media_fmt[0].format.num_channels; ch++)
      {
         result_capi = capi_msiir_create_new_msiir_filters(me, MSIIR_CURRENT_INSTANCE, ch);
         if (CAPI_EOK != result_capi)
         {
            return CAPI_EFAILED;
         }
      }
      me->num_channels_allocated = me->media_fmt[0].format.num_channels;
   }
   else if (msiir_bits_per_sample_before != me->media_fmt[0].format.bits_per_sample)
   {
      // although we have enough memory for msiir library, the msiir static vars have
      // changed after reinit, so we still need to reinit the current msiir library
      for (uint32_t ch = 0; ch < me->media_fmt[0].format.num_channels; ch++)
      {
         void *msiir_ptr = me->msiir_lib[ch].mem_ptr;

         result_lib =
            msiir_init_mem(&(me->msiir_lib[ch]), &(me->msiir_static_vars), msiir_ptr, me->per_chan_mem_req.mem_size);

         if (MSIIR_SUCCESS != result_lib)
         {
            MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : ReInit failed initialize library");
            return CAPI_EFAILED;
         }
      }
   }

   // reinit cross fade library with new static vars
   me->cross_fade_static_vars.data_width  = (me->media_fmt[0].format.bits_per_sample >> 4);
   me->cross_fade_static_vars.sample_rate = me->media_fmt[0].format.sampling_rate;

   result_cross_fade_lib =
      audio_cross_fade_get_mem_req(&(me->per_chan_cross_fade_mem_req), &(me->cross_fade_static_vars));
   if ((CROSS_FADE_SUCCESS != result_cross_fade_lib) || (0 == me->per_chan_cross_fade_mem_req.cross_fade_lib_mem_size))
   {
      MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Init failed at cross fade");
      return CAPI_EFAILED;
   }

   int8_t *ptr = (int8_t *)me;
   ptr += align_to_8_byte(sizeof(capi_multistageiir_t));

   // allocate crossfade memory dynamically
   for (uint32_t ch = 0; ch < me->media_fmt[0].format.num_channels; ch++)
   {
      result_cross_fade_lib = audio_cross_fade_init_memory(&(me->cross_fade_lib[ch]),
                                                           &(me->cross_fade_static_vars),
                                                           (int8 *)ptr,
                                                           me->per_chan_cross_fade_mem_req.cross_fade_lib_mem_size);
      if (CROSS_FADE_SUCCESS != result_cross_fade_lib)
      {
         MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Init failed init cross fade");
         return CAPI_EFAILED;
      }

      ptr += align_to_8_byte(me->per_chan_cross_fade_mem_req.cross_fade_lib_mem_size);
   }
   me->per_chan_cross_fade_mem_req.cross_fade_lib_mem_size =
      align_to_8_byte(me->per_chan_cross_fade_mem_req.cross_fade_lib_mem_size);

   // Calculate and raise all the events here such as kpps, output media format, delay and process check
   capi_err_t capi_result = CAPI_EOK;

   capi_result = capi_msiir_check_raise_kpps_event(me, capi_msiir_get_kpps(me));

   capi_result |= capi_msiir_check_raise_bw_event(me);
   capi_result |= capi_msiir_update_delay_event(me);

   capi_result |= capi_msiir_raise_process_check_event(me);

   capi_result |= capi_cmn_output_media_fmt_event_v2(&me->cb_info, me->media_fmt, FALSE, 0);

   if (CAPI_FAILED(capi_result))
   {
      for (uint32_t ch = 0; ch < IIR_TUNING_FILTER_MAX_CHANNELS_V2; ch++)
      {
         if (NULL != me->msiir_lib[ch].mem_ptr)
         {
            posal_memory_free(me->msiir_lib[ch].mem_ptr);
            me->msiir_lib[ch].mem_ptr = NULL;
         }

         // if cross fading is active, release the new filters
         if (NULL != me->msiir_new_lib[ch].mem_ptr)
         {
            posal_memory_free(me->msiir_new_lib[ch].mem_ptr);
            me->msiir_new_lib[ch].mem_ptr = NULL;
         }

         me->cross_fade_lib[ch].cross_fade_lib_mem_ptr = NULL;
      }
      me->msiir_static_vars.max_stages                        = 0;
      me->per_chan_mem_req.mem_size                           = 0;
      me->per_chan_cross_fade_mem_req.cross_fade_lib_mem_size = 0;
      return capi_result;
   }
#ifdef CAPI_MSIIR_DEBUG_MSG
   MSIIR_MSG(me->miid, DBG_HIGH_PRIO,
          "CAPI MSIIR : Init done with outformat %lu,%lu,%lu,%lu,%lu",
          me->media_fmt[0].format.num_channels,
          me->media_fmt[0].format.bits_per_sample,
          me->media_fmt[0].format.sampling_rate,
          me->media_fmt[0].format.data_is_signed,
          (uint32_t)me->media_fmt[0].format.data_interleaving);
   MSIIR_MSG(me->miid, DBG_HIGH_PRIO, "CAPI MSIIR : init_capi end.");
#endif
   return CAPI_EOK;
}

capi_err_t capi_msiir_process_set_properties(capi_multistageiir_t *me, capi_proplist_t *props_ptr)
{
   capi_err_t   result     = CAPI_EOK;
   capi_prop_t *prop_array = props_ptr->prop_ptr;

   if (me == NULL)
   {
      MSIIR_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "Set common property received null ptr");
      return CAPI_EBADPARAM;
   }
   for (uint8_t i = 0; i < props_ptr->props_num; i++)
   {
      capi_buf_t *payload = &(prop_array[i].payload);
      uint32_t miid = me ? me->miid : MIID_UNKNOWN;
      switch (prop_array[i].id)
      {
         case CAPI_EVENT_CALLBACK_INFO:
         {
            if (payload->actual_data_len >= sizeof(capi_event_callback_info_t))
            {
               capi_event_callback_info_t *data_ptr = (capi_event_callback_info_t *)payload->data_ptr;
               if (NULL == data_ptr)
               {
                  payload->actual_data_len =
                     0; // according to CAPI V2, actual len should be num of bytes read(set)/written(get)
                  CAPI_SET_ERROR(result, CAPI_EBADPARAM);
                  continue;
               }
               me->cb_info = *data_ptr;
            }
            else
            {
               MSIIR_MSG(miid, DBG_ERROR_PRIO,
                      "CAPI MSIIR : Set, Property id 0x%lx Bad param size %lu",
                      (uint32_t)prop_array[i].id,
                      payload->actual_data_len);
               payload->actual_data_len =
                  0; // according to CAPI V2, actual len should be num of bytes read(set)/written(get)
               CAPI_SET_ERROR(result, CAPI_ENEEDMORE);
            }
            break;
         }
         case CAPI_PORT_NUM_INFO:
         {
            if (payload->actual_data_len >= sizeof(capi_port_num_info_t))
            {
               capi_port_num_info_t *data_ptr = (capi_port_num_info_t *)payload->data_ptr;

               /* As number of ports supported is only 1, we check if the set property
                * has been done with any other value
                */
               if ((data_ptr->num_input_ports != CAPI_MSIIR_MAX_IN_PORTS) ||
                   (data_ptr->num_output_ports != CAPI_MSIIR_MAX_OUT_PORTS))
               {
                  MSIIR_MSG(miid, DBG_ERROR_PRIO,
                         "CAPI MSIIR : Set, Property id 0x%lx num of in and out ports cannot be other than 1",
                         (uint32_t)prop_array[i].id);
                  CAPI_SET_ERROR(result, CAPI_EBADPARAM);
               }
            }
            else
            {
               MSIIR_MSG(miid, DBG_ERROR_PRIO,
                      "CAPI MSIIR : Set, Property id 0x%lx Bad param size %lu",
                      (uint32_t)prop_array[i].id,
                      payload->actual_data_len);
               payload->actual_data_len =
                  0; // according to CAPI V2, actual len should be num of bytes read(set)/written(get)
               CAPI_SET_ERROR(result, CAPI_ENEEDMORE);
            }
            break;
         }

         case CAPI_HEAP_ID:
         {
            if (payload->actual_data_len >= sizeof(capi_heap_id_t))
            {
               capi_heap_id_t *data_ptr = (capi_heap_id_t *)payload->data_ptr;
               me->heap_id              = data_ptr->heap_id;
            }
            else
            {
               MSIIR_MSG(miid, DBG_ERROR_PRIO,
                      "CAPI MSIIR : Set, Property id 0x%lx Bad param size %lu",
                      (uint32_t)prop_array[i].id,
                      payload->actual_data_len);
               payload->actual_data_len =
                  0; // according to CAPI V2, actual len should be num of bytes read(set)/written(get)
               CAPI_SET_ERROR(result, CAPI_ENEEDMORE);
            }
            break;
         }

         case CAPI_ALGORITHMIC_RESET:
         {
            MSIIR_MSG(miid, DBG_HIGH_PRIO, "CAPI MSIIR : Algorithmic Reset");
            if (me->media_fmt_received)
            {
               for (uint32_t ch = 0; ch < me->media_fmt[0].format.num_channels; ch++)
               {
                  msiir_set_param(&(me->msiir_lib[ch]),
                                  MSIIR_PARAM_RESET,
                                  NULL,
                                  0); // todo : Should we check for NULL ?
               }
            }
            break;
         }

         case CAPI_INPUT_MEDIA_FORMAT_V2:
         {
#ifdef CAPI_MSIIR_DEBUG_MSG
            MSIIR_MSG(miid, DBG_HIGH_PRIO, "CAPI MSIIR : Input Media Format Received.");
#endif
            if (NULL == payload->data_ptr)
            {
               MSIIR_MSG(miid, DBG_ERROR_PRIO,
                      "CAPI FIR: Set property id 0x%lx, received null buffer.",
                      (uint32_t)prop_array[i].id);
               CAPI_SET_ERROR(result, CAPI_EBADPARAM);
               break;
            }
            uint32_t required_size = sizeof(capi_set_get_media_format_t) + sizeof(capi_standard_data_format_v2_t);
            if (payload->actual_data_len >= required_size)
            {
               capi_media_fmt_v2_t *data_ptr = (capi_media_fmt_v2_t *)payload->data_ptr;

               if (data_ptr->format.minor_version >= CAPI_MEDIA_FORMAT_MINOR_VERSION)
               {
                  if ((prop_array[i].port_info.is_valid &&
                       (!prop_array[i].port_info.is_input_port || prop_array[i].port_info.port_index != 0)) ||
                      (data_ptr->header.format_header.data_format != CAPI_FIXED_POINT))
                  {
                     MSIIR_MSG(miid, DBG_ERROR_PRIO,
                            "CAPI MSIIR : Set, failed to set Property id 0x%lx due to invalid/unexpected values",
                            (uint32_t)prop_array[i].id);
                     payload->actual_data_len =
                        0; // according to CAPI V2, actual len should be num of bytes read(set)/written(get)
                     CAPI_SET_ERROR(result, CAPI_EBADPARAM);
                     continue;
                  }

                  if (FALSE == capi_msiir_is_supported_media_type_v2(me, data_ptr))
                  {
                     CAPI_SET_ERROR(result, CAPI_EUNSUPPORTED);
                     continue;
                  }
                  required_size += data_ptr->format.num_channels * sizeof(capi_channel_type_t);
                  if (payload->actual_data_len < required_size)
                  {
                     MSIIR_MSG(miid, DBG_ERROR_PRIO,
                            "CAPI MSIIR : Set, Property id 0x%lx Bad param size %lu",
                            (uint32_t)prop_array[i].id,
                            payload->actual_data_len);
                     payload->actual_data_len =
                        0; // according to CAPI V2, actual len should be num of bytes read(set)/written(get)
                     CAPI_SET_ERROR(result, CAPI_ENEEDMORE);
                     break;
                  }
                  if (me->media_fmt[0].format.num_channels != data_ptr->format.num_channels)
                  {
                     if (NULL != me->per_chan_msiir_cfg_max)
                     {
                        posal_memory_free(me->per_chan_msiir_cfg_max);
                        me->per_chan_msiir_cfg_max = NULL;
                     }
                     me->per_chan_msiir_cfg_max = (capi_one_chan_msiir_config_max_t *)
                        posal_memory_malloc(data_ptr->format.num_channels * sizeof(capi_one_chan_msiir_config_max_t),
                                            (POSAL_HEAP_ID)me->heap_id);
                     if (NULL == me->per_chan_msiir_cfg_max)
                     {
                        MSIIR_MSG(miid, DBG_FATAL_PRIO,
                               "CAPI MSIIR : Out of Memory for per channel msiir config. mem_size requested : %lu",
                               data_ptr->format.num_channels * sizeof(capi_one_chan_msiir_config_max_t));
                        return CAPI_ENOMEMORY;
                     }
                  }
                  me->media_fmt_received                = FALSE;
                  me->media_fmt[0].header.format_header = data_ptr->header.format_header;
                  me->media_fmt[0].format               = data_ptr->format;
                  me->media_fmt[0].format.minor_version = CAPI_MEDIA_FORMAT_MINOR_VERSION;
                  for (uint32_t i = 0; i < me->media_fmt[0].format.num_channels; i++)
                  {
                     me->media_fmt[0].channel_type[i] = data_ptr->channel_type[i];
                  }

                  CAPI_SET_ERROR(result, capi_msiir_init_capi(me, 1));

                  if (CAPI_SUCCEEDED(result))
                  {
                     me->media_fmt_received = TRUE;
                     memset(&(me->channel_map_to_index), -1, sizeof(me->channel_map_to_index));
                     for (uint32_t ch = 0; ch < me->media_fmt[0].format.num_channels; ch++)
                     {
                        me->channel_map_to_index[me->media_fmt[0].channel_type[ch]] = ch;
                     }

                     if (CAPI_FAILED(capi_msiir_set_params_to_lib(me)))
                     {
                        MSIIR_MSG(miid, DBG_ERROR_PRIO, "CAPI MSIIR : Setting cached params to the library failed!!!");
                     }
                     result = capi_msiir_check_raise_kpps_event(me, capi_msiir_get_kpps(me));
                     result |= capi_msiir_check_raise_bw_event(me);
                     result |= capi_msiir_update_delay_event(me);
                     result |= capi_msiir_raise_process_check_event(me);
                  }
                  else
                  {
                     MSIIR_MSG(miid, DBG_ERROR_PRIO, "CAPI MSIIR : Failed to create MS-IIR library");
                  }
               }
               else
               {
                  MSIIR_MSG(miid, DBG_ERROR_PRIO, "CAPI MSIIR : Invalid Minor version %lu", data_ptr->format.minor_version);
                  CAPI_SET_ERROR(result, CAPI_EBADPARAM);
               }
            }
            else
            {
               MSIIR_MSG(miid, DBG_ERROR_PRIO,
                      "CAPI MSIIR : Set, Property id 0x%lx Bad param size %lu",
                      (uint32_t)prop_array[i].id,
                      payload->actual_data_len);
               payload->actual_data_len =
                  0; // according to CAPI V2, actual len should be num of bytes read(set)/written(get)
               CAPI_SET_ERROR(result, CAPI_ENEEDMORE);
            }
            break;
         }

         case CAPI_CUSTOM_INIT_DATA:
         {
            break;
         }
         case CAPI_MODULE_INSTANCE_ID:
         {
            if (payload->actual_data_len >= sizeof(capi_module_instance_id_t))
            {
               capi_module_instance_id_t *data_ptr = (capi_module_instance_id_t *)payload->data_ptr;
               me->miid                            = data_ptr->module_instance_id;
               MSIIR_MSG(miid,
                         DBG_LOW_PRIO,
                         "This module-id 0x%08lX, instance-id 0x%08lX",
                         data_ptr->module_id,
                         me->miid);
            }
            else
            {
               MSIIR_MSG(miid,
                         DBG_ERROR_PRIO,
                         "Set property id 0x%lx, Bad param size %lu",
                         prop_array[i].id,
                         payload->max_data_len);
               result |= CAPI_ENEEDMORE;
            }
            break;
         } // CAPI_MODULE_INSTANCE_ID
         default:
         {
            payload->actual_data_len =
               0; // according to CAPI V2, actual len should be num of bytes read(set)/written(get)
            CAPI_SET_ERROR(result, CAPI_EUNSUPPORTED);
            break;
         }
      }
#ifdef CAPI_MSIIR_DEBUG_MSG
      if (CAPI_FAILED(result))
      {
         MSIIR_MSG(miid, DBG_ERROR_PRIO, "CAPI MSIIR : Set property for 0x%x failed with opcode %lu", prop_array[i].id, result);
      }
#endif
   }
   // create a temp result and later accumulate them in a main result
   return result;
}

capi_err_t capi_msiir_process_get_properties(capi_multistageiir_t *me, capi_proplist_t *props_ptr)
{
   capi_err_t result = CAPI_EOK;

   capi_prop_t *prop_ptr = props_ptr->prop_ptr;
   uint32_t     i        = 0;

   uint32_t fwk_extn_ids_arr[] = { FWK_EXTN_CONTAINER_FRAME_DURATION };
   uint32_t miid = me ? me->miid : MIID_UNKNOWN;

   capi_basic_prop_t mod_prop;
   capi_msiir_get_init_memory_req(&mod_prop.init_memory_req, miid);
   mod_prop.stack_size         = CAPI_MSIIR_STACK_SIZE;
   mod_prop.num_fwk_extns      = sizeof(fwk_extn_ids_arr) / sizeof(fwk_extn_ids_arr[0]);
   mod_prop.fwk_extn_ids_arr   = fwk_extn_ids_arr;
   mod_prop.is_inplace         = FALSE; // not capable of in-place processing of data
   mod_prop.req_data_buffering = FALSE;
   mod_prop.max_metadata_size  = 0; // NA
   result = capi_cmn_get_basic_properties(props_ptr, &mod_prop);
   if (CAPI_EOK != result)
   {
      MSIIR_MSG(miid, DBG_ERROR_PRIO, "MSIIR: Get common basic properties failed with result %lu", result);
      return result;
   }

   for (i = 0; i < props_ptr->props_num; i++)
   {
      capi_buf_t *payload = &prop_ptr[i].payload;
      // Ignore prop_ptr[i].port_info.is_valid
      uint32_t miid = me ? me->miid : MIID_UNKNOWN;

      switch (prop_ptr[i].id)
      {
         case CAPI_INIT_MEMORY_REQUIREMENT:
         case CAPI_IS_INPLACE:
         case CAPI_REQUIRES_DATA_BUFFERING:
         case CAPI_STACK_SIZE:
         case CAPI_NUM_NEEDED_FRAMEWORK_EXTENSIONS:
         case CAPI_INTERFACE_EXTENSIONS:
         {
            break;
         }
         case CAPI_OUTPUT_MEDIA_FORMAT_V2:
         {
            if (NULL == me || (prop_ptr[i].port_info.is_valid &&
                               (prop_ptr[i].port_info.is_input_port || prop_ptr[i].port_info.port_index != 0)))
            {
               MSIIR_MSG(miid, DBG_ERROR_PRIO,
                      "CAPI MSIIR : Get, failed to get Property id 0x%lx due to invalid/unexpected values",
                      (uint32_t)prop_ptr[i].id);
               payload->actual_data_len =
                  0; // according to CAPI V2, actual len should be num of bytes read(set)/written(get)
               CAPI_SET_ERROR(result, CAPI_EFAILED);
               continue;
            }
            uint32_t required_size = sizeof(me->media_fmt[0].header) + sizeof(me->media_fmt[0].format);

            if (payload->max_data_len >= required_size)
            {
               capi_media_fmt_v2_t *data_ptr = (capi_media_fmt_v2_t *)payload->data_ptr;
               capi_channel_type_t *channel_type;
               uint32_t channel_type_size = sizeof(capi_channel_type_t) * me->media_fmt[0].format.num_channels;
               channel_type_size          = CAPI_MSIIR_ALIGN_4_BYTE(channel_type_size);
               required_size += channel_type_size;

               if (payload->max_data_len < required_size)
               {
                  MSIIR_MSG(miid, DBG_ERROR_PRIO,
                         "CAPI MSIIR: Get property id 0x%lx, Bad param size %lu",
                         (uint32_t)prop_ptr[i].id,
                         payload->max_data_len);
                  CAPI_SET_ERROR(result, CAPI_ENEEDMORE);
                  break;
               }

               data_ptr->header = me->media_fmt[0].header;
               data_ptr->format = me->media_fmt[0].format;
               channel_type =
                  (capi_channel_type_t *)(((int8_t *)data_ptr) + sizeof(data_ptr->header) + sizeof(data_ptr->format));
               memscpy(channel_type,
                       sizeof(capi_channel_type_t) * data_ptr->format.num_channels,
                       me->media_fmt[0].channel_type,
                       sizeof(me->media_fmt[0].channel_type));
               payload->actual_data_len = required_size;
            }
            else
            {
               MSIIR_MSG(miid, DBG_ERROR_PRIO,
                      "CAPI MSIIR : Get, Property id 0x%lx Bad param size %lu",
                      (uint32_t)prop_ptr[i].id,
                      payload->max_data_len);
               payload->actual_data_len =
                  0; // according to CAPI V2, actual len should be num of bytes read(set)/written(get)
               CAPI_SET_ERROR(result, CAPI_ENEEDMORE);
            }
            break;
         }
         break;

         default:
         {
            payload->actual_data_len = 0;
            CAPI_SET_ERROR(result, CAPI_EUNSUPPORTED);
         }
      }
   }
   return result;
}

capi_err_t capi_msiir_cache_params(capi_multistageiir_t *me, capi_buf_t *params_ptr, uint32_t param_id)
{
   capi_err_t            result               = CAPI_EOK;
   uint32_t              malloc_size          = params_ptr->actual_data_len;
   capi_cached_params_t *cache_param_data_ptr = NULL;
   uint32_t              max_param_size       = 0;

   switch (param_id)
   {
      case PARAM_ID_MSIIR_TUNING_FILTER_ENABLE:
         max_param_size =
            sizeof(param_id_msiir_enable_t) + IIR_TUNING_FILTER_MAX_CHANNELS_V2 * sizeof(param_id_msiir_ch_enable_t);
         cache_param_data_ptr            = &me->enable_params;
         me->enable_params.param_id_type = MULTISTAGE_IIR_MCHAN_PARAM;
         me->cfg_version = VERSION_V1; // indicates V1 version is set now and can only allow V1 version
                                        // configs for upcoming set params
         break;

      case PARAM_ID_MSIIR_TUNING_FILTER_PREGAIN:
         max_param_size =
            sizeof(param_id_msiir_pregain_t) + IIR_TUNING_FILTER_MAX_CHANNELS_V2 * sizeof(param_id_msiir_ch_pregain_t);
         cache_param_data_ptr             = &me->pregain_params;
         me->pregain_params.param_id_type = MULTISTAGE_IIR_MCHAN_PARAM;
         me->cfg_version = VERSION_V1; // indicates V1 version is set now and can only allow V1 version
                                        // configs for upcoming set params
         break;

      case PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS:
         max_param_size = sizeof(param_id_msiir_config_t) +
                          sizeof(param_id_msiir_ch_filter_config_t) * IIR_TUNING_FILTER_MAX_CHANNELS_V2 +
                          sizeof(int32_t) * MSIIR_MAX_STAGES * MSIIR_COEFF_LENGTH * IIR_TUNING_FILTER_MAX_CHANNELS_V2 +
                          sizeof(int16_t) * MSIIR_MAX_STAGES * IIR_TUNING_FILTER_MAX_CHANNELS_V2;
         cache_param_data_ptr            = &me->config_params;
         me->config_params.param_id_type = MULTISTAGE_IIR_MCHAN_PARAM;
         me->cfg_version = VERSION_V1; // indicates V1 version is set now and can only allow V1 version
                                        // configs for upcoming set params
         break;

      case PARAM_ID_MSIIR_TUNING_FILTER_ENABLE_V2:
         max_param_size =
            sizeof(param_id_msiir_enable_v2_t) + IIR_TUNING_FILTER_MAX_CHANNELS_V2 * (
            sizeof(param_id_msiir_ch_enable_v2_t) + CAPI_CMN_MAX_CHANNEL_MAP_GROUPS * sizeof(uint32_t));
         cache_param_data_ptr            = &me->enable_params;
         me->enable_params.param_id_type = MULTISTAGE_IIR_MCHAN_PARAM;
         me->cfg_version = VERSION_V2; // indicates V2 version is set now and can only allow V2 version
                                        // configs for upcoming set params
         break;

      case PARAM_ID_MSIIR_TUNING_FILTER_PREGAIN_V2:
         max_param_size =
            sizeof(param_id_msiir_pregain_v2_t) + IIR_TUNING_FILTER_MAX_CHANNELS_V2 * (
             sizeof(param_id_msiir_ch_pregain_v2_t) + CAPI_CMN_MAX_CHANNEL_MAP_GROUPS * sizeof(uint32_t));
         cache_param_data_ptr             = &me->pregain_params;
         me->pregain_params.param_id_type = MULTISTAGE_IIR_MCHAN_PARAM;
         me->cfg_version = VERSION_V2; // indicates V2 version is set now and can only allow V2 version
                                        // configs for upcoming set params
         break;

      case PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS_V2:
         max_param_size = sizeof(param_id_msiir_config_v2_t) +
                          sizeof(param_id_msiir_ch_filter_config_v2_t) * IIR_TUNING_FILTER_MAX_CHANNELS_V2 * (
                          sizeof(uint32_t) * CAPI_CMN_MAX_CHANNEL_MAP_GROUPS +
                          sizeof(int32_t) * MSIIR_MAX_STAGES * MSIIR_COEFF_LENGTH +
                          sizeof(int16_t) * MSIIR_MAX_STAGES);

         cache_param_data_ptr            = &me->config_params;
         me->config_params.param_id_type = MULTISTAGE_IIR_MCHAN_PARAM;
         me->cfg_version = VERSION_V2; // indicates V2 version is set now and can only allow V2 version
                                        // configs for upcoming set params
         break;
      default:
         return CAPI_EUNSUPPORTED;
         break;
   }

   malloc_size = (max_param_size > params_ptr->actual_data_len) ? params_ptr->actual_data_len : max_param_size;

   if ((cache_param_data_ptr->params_ptr.max_data_len != malloc_size) ||
       (cache_param_data_ptr->params_ptr.data_ptr == NULL))
   {
      if (cache_param_data_ptr->params_ptr.data_ptr != NULL)
      {
         posal_memory_free(cache_param_data_ptr->params_ptr.data_ptr);
         cache_param_data_ptr->params_ptr.data_ptr        = NULL;
         cache_param_data_ptr->params_ptr.max_data_len    = 0;
         cache_param_data_ptr->params_ptr.actual_data_len = 0;
      }

      cache_param_data_ptr->params_ptr.data_ptr =
         (int8_t *)posal_memory_malloc(malloc_size, (POSAL_HEAP_ID)me->heap_id);
      if (cache_param_data_ptr->params_ptr.data_ptr == NULL)
      {
         MSIIR_MSG(me->miid, DBG_ERROR_PRIO,
                "CAPI MSIIR : "
                "Memory allocation failed for size %lu",
                malloc_size);
         cache_param_data_ptr->params_ptr.max_data_len    = 0;
         cache_param_data_ptr->params_ptr.actual_data_len = 0;
         return CAPI_ENOMEMORY;
      }

      cache_param_data_ptr->params_ptr.max_data_len    = malloc_size;
      cache_param_data_ptr->params_ptr.actual_data_len = 0;
   }

   cache_param_data_ptr->params_ptr.actual_data_len = memscpy(cache_param_data_ptr->params_ptr.data_ptr,
                                                              cache_param_data_ptr->params_ptr.max_data_len,
                                                              params_ptr->data_ptr,
                                                              params_ptr->actual_data_len);

   return result;
}

capi_err_t capi_msiir_set_params_to_lib(capi_multistageiir_t *me)
{
   capi_err_t temp_result = CAPI_EOK, result = CAPI_EOK;
#ifdef CAPI_MSIIR_DEBUG_MSG
   MSIIR_MSG(me->miid, DBG_HIGH_PRIO, "CAPI MSIIR : Setting the cached params if any");
#endif

   switch (me->enable_params.param_id_type)
   {
      case MULTISTAGE_IIR_MCHAN_PARAM:
         if (me->enable_params.params_ptr.data_ptr != NULL)
         {
            if ((VERSION_V1 == me->cfg_version) && (FALSE == me->higher_channel_map_present))
            {
               temp_result = capi_msiir_set_enable_disable_per_channel(me, &me->enable_params.params_ptr);
            }
            else if (VERSION_V2 == me->cfg_version)
            {
               temp_result = capi_msiir_set_enable_disable_per_channel_v2(me, &me->enable_params.params_ptr,
                                                                     PARAM_ID_MSIIR_TUNING_FILTER_ENABLE_V2);
            }
            else
            {
               MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Unsupported config version set");
               result = CAPI_EUNSUPPORTED;
            }
         }
         break;
   }
   result |= temp_result;

   switch (me->pregain_params.param_id_type)
   {
      case MULTISTAGE_IIR_MCHAN_PARAM:
         if (me->pregain_params.params_ptr.data_ptr != NULL)
         {
            if ((VERSION_V1 == me->cfg_version) && (FALSE == me->higher_channel_map_present))
            {
               temp_result = capi_msiir_set_pregain_per_channel(me, &me->pregain_params.params_ptr);
            }
            else if (VERSION_V2 == me->cfg_version)
            {
               temp_result = capi_msiir_set_pregain_per_channel_v2(me, &me->pregain_params.params_ptr,
                                                             PARAM_ID_MSIIR_TUNING_FILTER_PREGAIN_V2);
            }
            else
            {
               MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Unsupported config version set");
               result = CAPI_EUNSUPPORTED;
            }
         }
         break;
   }
   result |= temp_result;
   switch (me->config_params.param_id_type)
   {
      case MULTISTAGE_IIR_MCHAN_PARAM:
         if (me->config_params.params_ptr.data_ptr != NULL)
         {
            if ((VERSION_V1 == me->cfg_version) && (FALSE == me->higher_channel_map_present))
            {
               temp_result = capi_msiir_set_config_per_channel(me, &me->config_params.params_ptr);
            }
            else if (VERSION_V2 == me->cfg_version)
            {
               temp_result = capi_msiir_set_config_per_channel_v2(me, &me->config_params.params_ptr,
                                                      PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS_V2);
            }
            else
            {
               MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Unsupported config version set");
               result = CAPI_EUNSUPPORTED;
            }
         }
         break;
   }
   result |= temp_result;
   return result;
}

// to check if the same channel is set at two different configs
bool_t check_channel_mask_msiir(uint8_t *iir_param_ptr, uint32_t param_id)
{
   uint32_t num_config           = 0;
   uint64_t check_channel_mask   = 0x0;
   uint64_t current_channel_mask = 0x0;
   bool_t   check                = TRUE;
   uint32_t offset               = 0;
   uint32_t buf_var              = 0;
   offset                        = memscpy(&num_config, sizeof(uint32_t), iir_param_ptr, sizeof(uint32_t));
   uint8_t *data_ptr             = iir_param_ptr;
   uint8_t *data_ptr_read        = iir_param_ptr;

   for (uint32_t cnfg_cntr = 0; cnfg_cntr < num_config; cnfg_cntr++)
   {
      data_ptr += offset;
      offset        = 0;
      data_ptr_read = data_ptr;
      // present configuration
      offset += memscpy(&current_channel_mask, sizeof(uint64_t), data_ptr, sizeof(uint64_t));
      data_ptr_read += offset;

      switch (param_id)
      {

         case PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS:
         {
            data_ptr_read += sizeof(uint16_t); // for reserved field
            offset += sizeof(uint16_t);
            offset += memscpy(&buf_var,
                              sizeof(uint16_t),
                              data_ptr_read,
                              sizeof(uint16_t)); // buf_var here is used to store number of biquad stages
            offset += buf_var * (MSIIR_COEFF_LENGTH * sizeof(int32_t) + sizeof(int16_t));
            if (buf_var & 0x1)
            {
               data_ptr += sizeof(int16_t); // 16-bits padding
            }
            break;
         }
         default:
         {
            // default case is for both enable and pregain params
            offset += sizeof(uint32_t);
         }
      }

      current_channel_mask = current_channel_mask >> 1; // ignoring the reserve bit
      if (!(check_channel_mask & current_channel_mask))
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

capi_err_t capi_msiir_set_enable_disable_per_channel(capi_multistageiir_t *me, const capi_buf_t *params_ptr)
{

   param_id_msiir_enable_t *iir_enable_ptr = (param_id_msiir_enable_t *)(params_ptr->data_ptr);
   uint32_t                 num_configs    = iir_enable_ptr->num_config;
   uint32_t                 required_payload_size =
      (uint32_t)(sizeof(iir_enable_ptr->num_config) + (num_configs * sizeof(param_id_msiir_ch_enable_t)));

   if (params_ptr->actual_data_len < required_payload_size)
   {
      MSIIR_MSG(me->miid, DBG_ERROR_PRIO,
             "CAPI MSIIR : Set Enable, Bad param size %lu, required %lu",
             params_ptr->actual_data_len,
             required_payload_size);
      return AR_ENEEDMORE;
   }
   // validating the payload received
   if (!check_channel_mask_msiir((uint8_t *)iir_enable_ptr, PARAM_ID_MSIIR_TUNING_FILTER_ENABLE))
   {
      MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Received bad enable param ");
      return CAPI_EBADPARAM;
   }

   uint64_t channel_mask = 0;
   // loop through all the configs
   for (uint32_t cnfg_cntr = 0; cnfg_cntr < num_configs; cnfg_cntr++)
   {
      channel_mask = (uint64_t)iir_enable_ptr->enable_flag_settings[cnfg_cntr].channel_mask_msb << 32 |
                     iir_enable_ptr->enable_flag_settings[cnfg_cntr].channel_mask_lsb;
      if (channel_mask & 0x1)
      {
         MSIIR_MSG(me->miid, DBG_LOW_PRIO, "capi_msiir: Warning Set enable: reserve bit is set 1");
      }
      // setting the mute data channel wise (channels can be in the range 1 ~ MSIIR_MAX_CHANNEL)
      for (uint32_t j = 1; j < (MSIIR_MAX_CHANNEL + 1); j++)
      {
         channel_mask = channel_mask >> 1;
         if (channel_mask & 0x1)
         {
            // convert channel_type to channel index, channel index has range 0 ~ 7
            int32_t ch_idx = me->channel_map_to_index[j];
            if ((ch_idx < 0) || (ch_idx >= (int32_t)me->media_fmt[0].format.num_channels))
            {
               continue;
            }
            me->enable_flag[ch_idx] = iir_enable_ptr->enable_flag_settings[cnfg_cntr].enable_flag;
            MSIIR_MSG(me->miid, DBG_HIGH_PRIO, "CAPI MSIIR : Set enable: channel: %hu enable: %lu", j, me->enable_flag[ch_idx]);
         }
      }
   }
   capi_msiir_raise_process_check_event(me);
   capi_msiir_update_delay_event(me);
   return CAPI_EOK;
}

capi_err_t capi_msiir_set_pregain_per_channel(capi_multistageiir_t *me, const capi_buf_t *params_ptr)
{
   MSIIR_RESULT result_lib = MSIIR_SUCCESS;

   param_id_msiir_pregain_t *iir_pregain_ptr = (param_id_msiir_pregain_t *)(params_ptr->data_ptr);

   uint32_t num_configs = iir_pregain_ptr->num_config;

   uint32_t required_payload_size =
      (uint32_t)(sizeof(iir_pregain_ptr->num_config) + (num_configs * sizeof(param_id_msiir_ch_pregain_t)));

   if (params_ptr->actual_data_len < required_payload_size)
   {
      MSIIR_MSG(me->miid, DBG_ERROR_PRIO,
             "CAPI MSIIR : Set pregain, Bad param size %lu, required %lu",
             params_ptr->actual_data_len,
             required_payload_size);
      return CAPI_ENEEDMORE;
   }

   // validating the payload received
   if (!check_channel_mask_msiir((uint8_t *)iir_pregain_ptr, PARAM_ID_MSIIR_TUNING_FILTER_PREGAIN))
   {
      MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Received bad pregain param ");
      return CAPI_EBADPARAM;
   }

   uint64_t channel_mask = 0;
   // loop through all the configs
   for (uint32_t cnfg_cntr = 0; cnfg_cntr < num_configs; cnfg_cntr++)
   {
      channel_mask = (uint64_t)iir_pregain_ptr->pre_gain_settings[cnfg_cntr].channel_mask_msb << 32 |
                     iir_pregain_ptr->pre_gain_settings[cnfg_cntr].channel_mask_lsb;

      if (channel_mask & 0x1)
      {
         MSIIR_MSG(me->miid, DBG_LOW_PRIO, "capi_msiir: Warning Set pregain: reserve bit is set 1");
      }
      // setting the gain data channel wise
      for (uint32_t j = 1; j < MSIIR_MAX_CHANNEL + 1; j++)
      {
         channel_mask = channel_mask >> 1;
         if (channel_mask & 0x1)
         {
            // convert channel_type to channel index, channel index has range 0 ~ 7
            int32_t ch_idx = me->channel_map_to_index[j];
            if ((ch_idx < 0) || (ch_idx >= (int32_t)me->media_fmt[0].format.num_channels))
            {
               continue;
            }
            msiir_pregain_t pregain = (msiir_pregain_t)(iir_pregain_ptr->pre_gain_settings[cnfg_cntr].pregain);
            if ((me->is_first_frame) || (me->per_chan_msiir_pregain[ch_idx] == pregain))
            {
               result_lib =
                  msiir_set_param(&(me->msiir_lib[ch_idx]), MSIIR_PARAM_PREGAIN, (void *)&pregain, sizeof(pregain));
               if (MSIIR_SUCCESS != result_lib)
               {
                  MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Set pregain error %d", result_lib);
                  return CAPI_EFAILED;
               }
               MSIIR_MSG(me->miid, DBG_HIGH_PRIO, "CAPI MSIIR : Set pregain: channel: %hu pregain: 0x%ld", j, pregain);
            }
            else
            {
               me->start_cross_fade = TRUE;
            }
            // save the new configuration
            me->per_chan_msiir_pregain[ch_idx] = pregain;
         }
      }
   }
   return CAPI_EOK;
}

capi_err_t capi_msiir_set_config_per_channel(capi_multistageiir_t *me, const capi_buf_t *params_ptr)
{
   param_id_msiir_config_t *iir_cfg_ptr  = (param_id_msiir_config_t *)(params_ptr->data_ptr);
   uint32_t                 num_configs  = iir_cfg_ptr->num_config;
   uint64_t                 channel_mask = 0;
   // validating the payload received
   if (!check_channel_mask_msiir((uint8_t *)iir_cfg_ptr, PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS))
   {
      MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Received bad config param ");
      return CAPI_EBADPARAM;
   }
   uint8_t *data_ptr   = (uint8_t *)(((uint8_t *)(params_ptr->data_ptr)) + sizeof(param_id_msiir_config_t));
   uint8_t *data_ptr_1 = (uint8_t *)(((uint8_t *)(params_ptr->data_ptr)) + sizeof(param_id_msiir_config_t));
   uint32_t offset     = 0;

   for (uint32_t cnfg_cntr = 0; cnfg_cntr < num_configs; cnfg_cntr++)
   {
      data_ptr_1 += offset;
      param_id_msiir_ch_filter_config_t *this_chan_cfg_inp_ptr = (param_id_msiir_ch_filter_config_t *)data_ptr_1;
      channel_mask = (uint64_t)this_chan_cfg_inp_ptr->channel_mask_msb << 32 | this_chan_cfg_inp_ptr->channel_mask_lsb;

      if (channel_mask & 0x1)
      {
         MSIIR_MSG(me->miid, DBG_LOW_PRIO, "capi_msiir: Warning Set config: reserve bit is set 1");
      }
      for (uint32_t j = 1; j < MSIIR_MAX_CHANNEL + 1; j++)
      {
         data_ptr     = data_ptr_1;
         channel_mask = channel_mask >> 1;
         if (channel_mask & 0x1)
         {
            uint8_t channel_type      = j;
            int32_t ch_idx            = me->channel_map_to_index[channel_type];
            int32_t num_biquad_stages = (int32_t)(this_chan_cfg_inp_ptr->num_biquad_stages);

            MSIIR_MSG(me->miid, DBG_HIGH_PRIO,
                   "CAPI MSIIR : Set config for channel_type %d, stages %lu ch_idx %lu",
                   channel_type,
                   num_biquad_stages,
                   ch_idx);

            if ((ch_idx < 0) || (ch_idx >= (int32_t)me->media_fmt[0].format.num_channels))
            {
               offset = sizeof(param_id_msiir_ch_filter_config_t);
               offset += num_biquad_stages * MSIIR_COEFF_LENGTH * sizeof(int32_t);
               offset += num_biquad_stages * sizeof(int16_t);
               offset += (num_biquad_stages & 0x1) ? sizeof(int16_t): 0;
               continue;
            }
            offset = 0;
            if ((!me->is_first_frame) && (num_biquad_stages != me->per_chan_msiir_cfg_max[ch_idx].num_stages))
            {
               // do cross fading if num stages changed in the middle of data processing
               me->start_cross_fade = TRUE;
            }

            me->per_chan_msiir_cfg_max[ch_idx].num_stages = num_biquad_stages;

            // move pointer to filter coeffs of current channel
            data_ptr += sizeof(param_id_msiir_ch_filter_config_t);
            offset += sizeof(param_id_msiir_ch_filter_config_t);

            int32_t *coeff_ptr = (int32_t *)data_ptr;

            for (int32_t stage = 0; stage < num_biquad_stages; stage++)
            {
               for (int32_t idx = 0; idx < MSIIR_COEFF_LENGTH; idx++)
               {
                  int32_t iir_coeff = *coeff_ptr++;

                  if ((!me->is_first_frame) &&
                      (iir_coeff != me->per_chan_msiir_cfg_max[ch_idx].coeffs_struct[stage].iir_coeffs[idx]))
                  {
                     me->start_cross_fade = TRUE;
                  }

                  me->per_chan_msiir_cfg_max[ch_idx].coeffs_struct[stage].iir_coeffs[idx] = iir_coeff;
               }
            }

            size_t coeff_size = num_biquad_stages * MSIIR_COEFF_LENGTH * sizeof(int32_t);

            // move pointer to numerator shift factors of current channel
            data_ptr += coeff_size;
            offset += coeff_size;

            int16_t *shift_factor_ptr = (int16_t *)data_ptr;

            for (int32_t stage = 0; stage < num_biquad_stages; stage++)
            {
               int32_t shift_factor = (int32_t)(*shift_factor_ptr++);

               if ((!me->is_first_frame) &&
                   (shift_factor != me->per_chan_msiir_cfg_max[ch_idx].coeffs_struct[stage].shift_factor))
               {
                  me->start_cross_fade = TRUE;
               }

               me->per_chan_msiir_cfg_max[ch_idx].coeffs_struct[stage].shift_factor = shift_factor;
            }

            size_t numerator_shift_fac_size = num_biquad_stages * sizeof(int16_t);

            data_ptr += numerator_shift_fac_size;
            offset += numerator_shift_fac_size;

            // move pointer to skip the padding bit if necessary
            bool_t is_odd_stages = (bool_t)(num_biquad_stages & 0x1);
            if (is_odd_stages)
            {
               data_ptr += sizeof(int16_t); // 16-bits padding
               offset += sizeof(int16_t);
            }

            if (!me->start_cross_fade)
            {
               uint32_t param_size = sizeof(me->per_chan_msiir_cfg_max[ch_idx].num_stages) +
                                     num_biquad_stages * MSIIR_COEFF_LENGTH * sizeof(int32_t) +
                                     num_biquad_stages * sizeof(int32_t);

               msiir_set_param(&(me->msiir_lib[ch_idx]),
                               MSIIR_PARAM_CONFIG,
                               (void *)&(me->per_chan_msiir_cfg_max[ch_idx]),
                               param_size);

               // when we reach here, data_ptr should points to the start of next channel's
               // config params (param_id_channel_type_msiir_config_pair_t followed by filter coeff and
               // num_shift_factor
               MSIIR_MSG(me->miid, DBG_HIGH_PRIO,
                      "CAPI MSIIR : Set config for channel_type %d, stages %lu",
                      channel_type,
                      num_biquad_stages);
               capi_msiir_update_delay_event(me);
            }
         }
      }
   }

   return CAPI_EOK;
}

capi_err_t capi_msiir_get_enable_disable_per_channel(capi_multistageiir_t *me, capi_buf_t *params_ptr)
{
   if (me->enable_params.params_ptr.data_ptr)
   {
      if (params_ptr->max_data_len < me->enable_params.params_ptr.actual_data_len)
      {
         params_ptr->actual_data_len = me->enable_params.params_ptr.actual_data_len;
         MSIIR_MSG(me->miid, DBG_ERROR_PRIO,
                "CAPI MSIIR : Get Enable/Disable, Bad payload size %lu: actual = %lu",
                params_ptr->max_data_len,
                me->enable_params.params_ptr.actual_data_len);

         return CAPI_ENEEDMORE;
      }
      params_ptr->actual_data_len = memscpy(params_ptr->data_ptr,
                                            params_ptr->max_data_len,
                                            me->enable_params.params_ptr.data_ptr,
                                            me->enable_params.params_ptr.actual_data_len);
   }
   else if (me->media_fmt_received)
   {
      uint32_t num_channels                       = me->media_fmt[0].format.num_channels;
      uint32_t num_config                         = 0;
      uint32_t enable_channel_mask_lsb            = 0;
      uint32_t enable_channel_mask_msb            = 0;
      uint32_t disable_channel_mask_lsb           = 0;
      uint32_t disable_channel_mask_msb           = 0;

      for (uint32_t ch = 0; ch < num_channels; ch++)
      {
         uint8_t ctype = me->media_fmt[0].channel_type[ch];
         if (me->enable_flag[ch] == 1)
         {
            if (ctype < MSIIR_MASK_WIDTH) // getting enable params for channels 1 ~ 31 (in channel_mask_lsb)
            {
                enable_channel_mask_lsb = enable_channel_mask_lsb | (1 << ctype);
            }
            else // getting enable params for 32 ~ 63 (in channel_mask_msb)
            {
                enable_channel_mask_msb = enable_channel_mask_msb | (1 << (ctype - 32));
            }
         }
         else
         {
             if (ctype < MSIIR_MASK_WIDTH) // getting enable params for channels 1 ~ 31 (in channel_mask_lsb)
             {
                 disable_channel_mask_lsb = disable_channel_mask_lsb | (1 << ctype);
             }
             else // getting enable params for 32 ~ 63 (in channel_mask_msb)
             {
                 disable_channel_mask_msb = disable_channel_mask_msb | (1 << (ctype - 32));
             }
         }
      }

      if (disable_channel_mask_lsb | disable_channel_mask_msb)
      {
          num_config++;
      }
      if (enable_channel_mask_lsb | enable_channel_mask_msb)
      {
          num_config++;
      }

      param_id_msiir_enable_t *iir_enable_pkt_ptr = (param_id_msiir_enable_t *)(params_ptr->data_ptr);
      uint32_t required_payload_size              = (uint32_t)(sizeof(uint32_t)  + num_config * sizeof(param_id_msiir_ch_enable_t));

      if (params_ptr->max_data_len < required_payload_size)
      {
         MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Get Enable/Disable, Bad payload size %lu", params_ptr->max_data_len);
         params_ptr->actual_data_len = 0;
         return CAPI_ENEEDMORE;
      }

      iir_enable_pkt_ptr->num_config                               = 0;
      iir_enable_pkt_ptr->enable_flag_settings[0].channel_mask_lsb = 0;
      iir_enable_pkt_ptr->enable_flag_settings[0].channel_mask_msb = 0;

      if (disable_channel_mask_lsb | disable_channel_mask_msb)
      {
          iir_enable_pkt_ptr->enable_flag_settings[iir_enable_pkt_ptr->num_config].channel_mask_lsb = disable_channel_mask_lsb;
          iir_enable_pkt_ptr->enable_flag_settings[iir_enable_pkt_ptr->num_config].channel_mask_msb = disable_channel_mask_msb;
          iir_enable_pkt_ptr->enable_flag_settings[iir_enable_pkt_ptr->num_config].enable_flag = 0;
          iir_enable_pkt_ptr->num_config ++;

          MSIIR_MSG(me->miid, DBG_HIGH_PRIO, "CAPI MSIIR : Get Enable/Disable, Disabled channel mask lsb: %lu, msb: %lu",
                  disable_channel_mask_lsb,
                  disable_channel_mask_msb);
      }
      if (enable_channel_mask_lsb | enable_channel_mask_msb)
      {
          iir_enable_pkt_ptr->enable_flag_settings[iir_enable_pkt_ptr->num_config].channel_mask_lsb = enable_channel_mask_lsb;
          iir_enable_pkt_ptr->enable_flag_settings[iir_enable_pkt_ptr->num_config].channel_mask_msb = enable_channel_mask_msb;
          iir_enable_pkt_ptr->enable_flag_settings[iir_enable_pkt_ptr->num_config].enable_flag = 1;
          iir_enable_pkt_ptr->num_config ++;

          MSIIR_MSG(me->miid, DBG_HIGH_PRIO, "CAPI MSIIR : Get Enable/Disable, Enabled channel mask lsb: %lu, msb: %lu",
                  enable_channel_mask_lsb,
                  enable_channel_mask_msb);
      }
      params_ptr->actual_data_len = required_payload_size;
   }
   else
   {
      params_ptr->actual_data_len = 0;
      return CAPI_EBADPARAM;
   }
   return CAPI_EOK;
}

capi_err_t capi_msiir_get_pregain_per_channel(capi_multistageiir_t *me, capi_buf_t *params_ptr)
{
   if (me->pregain_params.params_ptr.data_ptr)
   {
      if (params_ptr->max_data_len < me->pregain_params.params_ptr.actual_data_len)
      {
         params_ptr->actual_data_len = me->pregain_params.params_ptr.actual_data_len;
         MSIIR_MSG(me->miid, DBG_ERROR_PRIO,
                "CAPI MSIIR : Get pregain, Bad payload size %lu: actual = %lu",
                params_ptr->max_data_len,
                me->pregain_params.params_ptr.actual_data_len);

         return CAPI_ENEEDMORE;
      }
      params_ptr->actual_data_len = memscpy(params_ptr->data_ptr,
                                            params_ptr->max_data_len,
                                            me->pregain_params.params_ptr.data_ptr,
                                            me->pregain_params.params_ptr.actual_data_len);
   }
   else if (me->media_fmt_received)
   {
      param_id_msiir_pregain_t *iir_pregain_pkt_ptr = (param_id_msiir_pregain_t *)(params_ptr->data_ptr);

      uint32_t num_channels = me->media_fmt[0].format.num_channels;
      uint32_t required_payload_size =
         (uint32_t)(sizeof(iir_pregain_pkt_ptr->num_config) + (num_channels * sizeof(param_id_msiir_ch_pregain_t)));
      if (params_ptr->max_data_len < required_payload_size)
      {
         MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Get pregain, Bad payload size %lu", params_ptr->max_data_len);
         return CAPI_ENEEDMORE;
      }

      iir_pregain_pkt_ptr->num_config = num_channels;
      // getting one channel params per config

      for (uint32_t ch = 0; ch < num_channels; ch++)
      {
         uint8_t channel_type_by_reinit = me->media_fmt[0].channel_type[ch];
         if (channel_type_by_reinit < MSIIR_MASK_WIDTH) // for checking channels from 1~31 in channel_mask_lsb
         {
            iir_pregain_pkt_ptr->pre_gain_settings[ch].channel_mask_lsb =
               iir_pregain_pkt_ptr->pre_gain_settings[ch].channel_mask_lsb | (1 << channel_type_by_reinit);
         }
         else // for checking channels from 32~63 in channel_mask_msb
         {
            iir_pregain_pkt_ptr->pre_gain_settings[ch].channel_mask_msb =
               iir_pregain_pkt_ptr->pre_gain_settings[ch].channel_mask_msb | (1 << (channel_type_by_reinit - 32));
         }
         // unity gain in Q27
         msiir_pregain_t pregain_lib = (msiir_pregain_t)((int32_t)(1 << MSIIR_Q_PREGAIN));

         uint32_t     actual_param_size = 0;
         MSIIR_RESULT result_lib        = MSIIR_SUCCESS;

         if ((me->cross_fade_flag[ch] == 0) || (me->msiir_new_lib[ch].mem_ptr == NULL))
         {
            result_lib = msiir_get_param(&(me->msiir_lib[ch]),
                                         MSIIR_PARAM_PREGAIN,
                                         (void *)&pregain_lib,
                                         sizeof(pregain_lib),
                                         &actual_param_size);

            if (actual_param_size > sizeof(pregain_lib) || (MSIIR_SUCCESS != result_lib))
            {
               MSIIR_MSG(me->miid, DBG_ERROR_PRIO,
                      "CAPI MSIIR : get pregain error result %d, size %lu",
                      result_lib,
                      actual_param_size);
               return CAPI_EFAILED;
            }
         }
         else
         {
            result_lib = msiir_get_param(&(me->msiir_new_lib[ch]),
                                         MSIIR_PARAM_PREGAIN,
                                         (void *)&pregain_lib,
                                         sizeof(pregain_lib),
                                         &actual_param_size);
            if (actual_param_size > sizeof(pregain_lib) || (MSIIR_SUCCESS != result_lib))
            {
               MSIIR_MSG(me->miid, DBG_ERROR_PRIO,
                      "CAPI MSIIR : get pregain error result %d, size %lu",
                      result_lib,
                      actual_param_size);
               return CAPI_EFAILED;
            }
         }

         int32_t pregain = (int32_t)pregain_lib;

         iir_pregain_pkt_ptr->pre_gain_settings[ch].pregain = pregain;
      }

      params_ptr->actual_data_len = required_payload_size;
   }
   else
   {
      params_ptr->actual_data_len = 0;
      return CAPI_EBADPARAM;
   }

   return CAPI_EOK;
}

capi_err_t capi_msiir_get_config_per_channel(capi_multistageiir_t *me, capi_buf_t *params_ptr)
{
   if (me->config_params.params_ptr.data_ptr)
   {
      if (params_ptr->max_data_len < me->config_params.params_ptr.actual_data_len)
      {
         params_ptr->actual_data_len = me->config_params.params_ptr.actual_data_len;
         MSIIR_MSG(me->miid, DBG_ERROR_PRIO,
                "CAPI MSIIR : Get IIR configuration, Bad payload size %lu, Actual = %lu",
                params_ptr->max_data_len,
                params_ptr->actual_data_len);

         return CAPI_ENEEDMORE;
      }

      params_ptr->actual_data_len = memscpy(params_ptr->data_ptr,
                                            params_ptr->max_data_len,
                                            me->config_params.params_ptr.data_ptr,
                                            me->config_params.params_ptr.actual_data_len);
   }
   else if (me->media_fmt_received)
   {

      param_id_msiir_config_t *iir_config_pkt_ptr = (param_id_msiir_config_t *)(params_ptr->data_ptr);
      uint8_t *data_ptr = (uint8_t *)(((uint8_t *)(params_ptr->data_ptr)) + sizeof(param_id_msiir_config_t));

      uint32_t num_channels          = me->media_fmt[0].format.num_channels;
      iir_config_pkt_ptr->num_config = num_channels;
      // getting one channel's tuning config params per each config

      uint32_t actual_size = sizeof(param_id_msiir_config_t);

      for (uint32_t ch = 0; ch < num_channels; ch++)
      {
         param_id_msiir_ch_filter_config_t *this_chan_cfg_ptr = (param_id_msiir_ch_filter_config_t *)data_ptr;

         uint8_t channel_type_by_reinit      = me->media_fmt[0].channel_type[ch];
         this_chan_cfg_ptr->channel_mask_lsb = 0;
         this_chan_cfg_ptr->channel_mask_msb = 0;
         if (channel_type_by_reinit < 32)
         {
            this_chan_cfg_ptr->channel_mask_lsb = (1 << channel_type_by_reinit);
         }
         else if (channel_type_by_reinit < 64)
         {
            this_chan_cfg_ptr->channel_mask_msb = (1 << (channel_type_by_reinit - 32));
         }

         this_chan_cfg_ptr->reserved = 0;

         capi_one_chan_msiir_config_max_t *one_chan_msiir_config_max_ptr = &(me->default_per_chan_msiir_cfg_max);
         // if the filter is disabled, the library uses default 0 stages for copy through
         int32_t num_biquad_stages = 0;

         uint32_t     actual_param_size = 0;
         MSIIR_RESULT result_lib        = MSIIR_SUCCESS;

         one_chan_msiir_config_max_ptr = &(me->per_chan_msiir_cfg_max[ch]);
         if ((me->cross_fade_flag[ch] == 0) || (me->msiir_new_lib[ch].mem_ptr == NULL))
         {
#ifdef CAPI_MSIIR_DEBUG_MSG
            MSIIR_MSG(me->miid, DBG_LOW_PRIO, "CAPI MSIIR : Config get param from CUR_INST ");
#endif
            result_lib = msiir_get_param(&(me->msiir_lib[ch]),
                                         MSIIR_PARAM_CONFIG,
                                         (void *)one_chan_msiir_config_max_ptr,
                                         sizeof(me->per_chan_msiir_cfg_max[ch]),
                                         &actual_param_size);
            if (actual_param_size > sizeof(me->per_chan_msiir_cfg_max[ch]) || (MSIIR_SUCCESS != result_lib))
            {
               MSIIR_MSG(me->miid, DBG_ERROR_PRIO,
                      "CAPI MSIIR : get delay error result %d, size %lu",
                      result_lib,
                      actual_param_size);
               return CAPI_EFAILED;
            }
         }
         else
         {
#ifdef CAPI_MSIIR_DEBUG_MSG
            MSIIR_MSG(me->miid, DBG_LOW_PRIO, "CAPI MSIIR : Config get param from NEW_INST because crossfading ");
#endif
            result_lib = msiir_get_param(&(me->msiir_new_lib[ch]),
                                         MSIIR_PARAM_CONFIG,
                                         (void *)one_chan_msiir_config_max_ptr,
                                         sizeof(me->per_chan_msiir_cfg_max[ch]),
                                         &actual_param_size);
            if (actual_param_size > sizeof(me->per_chan_msiir_cfg_max[ch]) || (MSIIR_SUCCESS != result_lib))
            {
               MSIIR_MSG(me->miid, DBG_ERROR_PRIO,
                      "CAPI MSIIR : get delay error result %d, size %lu",
                      result_lib,
                      actual_param_size);
               return CAPI_EFAILED;
            }
         }

         num_biquad_stages = one_chan_msiir_config_max_ptr->num_stages;

         if (num_biquad_stages > (int32_t)MSIIR_MAX_STAGES)
         {
            MSIIR_MSG(me->miid, DBG_ERROR_PRIO,
                   "CAPI MSIIR : Get config params, bad internal num_biquad_stages %d",
                   (int)num_biquad_stages);
            return CAPI_EFAILED;
         }

         this_chan_cfg_ptr->num_biquad_stages = num_biquad_stages;

         // move pointer to filter coeffs
         data_ptr += sizeof(param_id_msiir_ch_filter_config_t);
         actual_size += sizeof(param_id_msiir_ch_filter_config_t);

         int32_t *coeff_ptr = (int32_t *)data_ptr;
         for (int32_t stage = 0; stage < num_biquad_stages; stage++)
         {
            for (int32_t idx = 0; idx < MSIIR_COEFF_LENGTH; idx++)
            {
               *coeff_ptr++ = one_chan_msiir_config_max_ptr->coeffs_struct[stage].iir_coeffs[idx];
            }
         }

         size_t coeff_size = num_biquad_stages * MSIIR_COEFF_LENGTH * sizeof(int32_t);

         // move pointer to numerator shift factors
         data_ptr += coeff_size;
         // add size of filter coeffs
         actual_size += coeff_size;

         int16_t *shift_factor_ptr = (int16_t *)data_ptr;

         for (int32_t stage = 0; stage < num_biquad_stages; stage++)
         {
            *shift_factor_ptr++ = (int16_t)(one_chan_msiir_config_max_ptr->coeffs_struct[stage].shift_factor);
         }

         size_t num_shift_fac_size = num_biquad_stages * sizeof(int16_t);

         // move pointer to config params of next channel
         data_ptr += num_shift_fac_size;
         // add size of numerator shift factors
         actual_size += num_shift_fac_size;

         // add padding if odd stages
         bool_t is_odd_stages = (bool_t)(num_biquad_stages & 0x1);
         if (is_odd_stages)
         {
            data_ptr += sizeof(int16_t);
            actual_size += sizeof(int16_t); // 16-bits padding
         }

      } // for (ch)

      params_ptr->actual_data_len = actual_size;

      if (params_ptr->max_data_len < actual_size)
      {
         MSIIR_MSG(me->miid, DBG_ERROR_PRIO,
                "CAPI MSIIR : Get config params, Bad payload size %d, required %d",
                (int)params_ptr->max_data_len,
                (int)actual_size);
         return CAPI_ENEEDMORE;
      }
   }
   else
   {
      params_ptr->actual_data_len = 0;
      return CAPI_EBADPARAM;
   }
   return CAPI_EOK;
}

// This function should be called after msiir_init_mem, so that the library's
// default configuration parameters are retrieved and saved by APPI
capi_err_t capi_msiir_get_default_pregain_config(capi_multistageiir_t *me, uint32_t ch)
{
   MSIIR_RESULT result_lib = MSIIR_SUCCESS;

   msiir_pregain_t pregain_lib = (msiir_pregain_t)((int32_t)(1 << MSIIR_Q_PREGAIN));

   uint32_t actual_param_size = 0;

   // get the pregain
   result_lib = msiir_get_param(&(me->msiir_lib[ch]),
                                MSIIR_PARAM_PREGAIN,
                                (void *)&pregain_lib,
                                sizeof(pregain_lib),
                                &actual_param_size);
   if (actual_param_size > sizeof(pregain_lib) || (MSIIR_SUCCESS != result_lib))
   {
      MSIIR_MSG(me->miid, DBG_ERROR_PRIO,
             "CAPI MSIIR : get default pregain error result %d, size %lu",
             result_lib,
             actual_param_size);
      return CAPI_EFAILED;
   }
   me->per_chan_msiir_pregain[ch] = pregain_lib;

   // get the config (num_stages, coeffs and shift factors)
   capi_one_chan_msiir_config_max_t *one_chan_msiir_config_max_ptr = &(me->per_chan_msiir_cfg_max[ch]);

   result_lib = msiir_get_param(&(me->msiir_lib[ch]),
                                MSIIR_PARAM_CONFIG,
                                (void *)one_chan_msiir_config_max_ptr,
                                sizeof(me->per_chan_msiir_cfg_max[ch]),
                                &actual_param_size);
   if (actual_param_size > sizeof(me->per_chan_msiir_cfg_max[ch]) || (MSIIR_SUCCESS != result_lib))
   {
      MSIIR_MSG(me->miid, DBG_ERROR_PRIO,
             "CAPI MSIIR : get default config error result %d, size %lu",
             result_lib,
             actual_param_size);
      return CAPI_EFAILED;
   }

   return CAPI_EOK;
}

capi_err_t capi_msiir_start_cross_fade(capi_multistageiir_t *me)
{
   CROSS_FADE_RESULT result_cross_fade_lib = CROSS_FADE_SUCCESS;
   MSIIR_RESULT      result_msiir_lib      = MSIIR_SUCCESS;

   for (uint32_t ch = 0; ch < me->media_fmt[0].format.num_channels; ch++)
   {
      uint32_t param_size = 0;

      result_cross_fade_lib = audio_cross_fade_get_param(&(me->cross_fade_lib[ch]),
                                                         CROSS_FADE_PARAM_MODE,
                                                         (int8 *)&(me->cross_fade_flag[ch]),
                                                         (uint32)sizeof(me->cross_fade_flag[ch]),
                                                         (uint32 *)&param_size);
      if ((CROSS_FADE_SUCCESS != result_cross_fade_lib) || (0 == param_size))
      {
         MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : setup cross fade failed");
         return CAPI_EFAILED;
      }

      if (1 == me->cross_fade_flag[ch])
      {
#ifdef CAPI_MSIIR_DEBUG_MSG
         MSIIR_MSG(me->miid, DBG_HIGH_PRIO, "CAPI MSIIR : cross fade is active for current channel, new params are cached!");
#endif
         continue;
      }

      // create new MSIIR library if it does not exist yet
      if (NULL == me->msiir_new_lib[ch].mem_ptr)
      {
         capi_err_t capi_result = CAPI_EOK;

         capi_result = capi_msiir_create_new_msiir_filters(me, MSIIR_NEW_INSTANCE, ch);
         if (CAPI_EOK != capi_result)
         {
            return capi_result;
         }
      }

      // set the new params to the new MSIIR filters
      result_msiir_lib = msiir_set_param(&(me->msiir_new_lib[ch]),
                                         MSIIR_PARAM_PREGAIN,
                                         (void *)&(me->per_chan_msiir_pregain[ch]),
                                         (uint32)sizeof(me->per_chan_msiir_pregain[ch]));
      if (MSIIR_SUCCESS != result_msiir_lib)
      {
         MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : start cross fade failed setting pregain");
         return CAPI_EFAILED;
      }

      int32_t num_stages = me->per_chan_msiir_cfg_max[ch].num_stages;

      param_size = sizeof(me->per_chan_msiir_cfg_max[0].num_stages);
      param_size += num_stages * sizeof(me->per_chan_msiir_cfg_max[0].coeffs_struct[0]);

      result_msiir_lib = msiir_set_param(&(me->msiir_new_lib[ch]),
                                         MSIIR_PARAM_CONFIG,
                                         (void *)&(me->per_chan_msiir_cfg_max[ch]),
                                         param_size);
      if (MSIIR_SUCCESS != result_msiir_lib)
      {
         MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : start cross fade failed setting Config");
         return CAPI_EFAILED;
      }

      // start/enable cross fade for current channel
      me->cross_fade_flag[ch] = 1;

      result_cross_fade_lib = audio_cross_fade_set_param(&(me->cross_fade_lib[ch]),
                                                         CROSS_FADE_PARAM_MODE,
                                                         (int8 *)&(me->cross_fade_flag[ch]),
                                                         (uint32)sizeof(me->cross_fade_flag[ch]));
      if (CROSS_FADE_SUCCESS != result_cross_fade_lib)
      {
         MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : start cross fade failed");
         return CAPI_EFAILED;
      }

      capi_msiir_update_delay_event(me);
      // resets start_cross_fade flag after set params completed
      me->start_cross_fade = FALSE;

   } // for (ch)

   return CAPI_EOK;
}

static capi_err_t capi_msiir_create_new_msiir_filters(capi_multistageiir_t *me, uint32_t instance_id, uint32_t ch)
{
   MSIIR_RESULT result_lib = MSIIR_SUCCESS;

   me->per_chan_mem_req.mem_size = 0;

   result_lib = msiir_get_mem_req(&(me->per_chan_mem_req), &(me->msiir_static_vars));
   if ((MSIIR_SUCCESS != result_lib) || (0 == me->per_chan_mem_req.mem_size))
   {
      MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Getsize error");
      return CAPI_EFAILED;
   }

   uint32_t malloc_size = align_to_8_byte(me->per_chan_mem_req.mem_size);

   void *ptr = posal_memory_malloc(malloc_size, (POSAL_HEAP_ID)me->heap_id);
   if (NULL == ptr)
   {
      MSIIR_MSG(me->miid, DBG_FATAL_PRIO, "CAPI MSIIR : creating new MSIIR: Out of Memory. mem_size requested : %lu", malloc_size);
      return CAPI_ENOMEMORY;
   }

   if (MSIIR_NEW_INSTANCE == instance_id)
   {
      if (NULL != me->msiir_new_lib[ch].mem_ptr)
      {
#ifdef CAPI_MSIIR_DEBUG_MSG
         MSIIR_MSG(me->miid, DBG_HIGH_PRIO,
                "CAPI MSIIR : releasing existing library ptr before creating new 0x%p",
                me->msiir_new_lib[ch].mem_ptr);
#endif
         posal_memory_free(me->msiir_new_lib[ch].mem_ptr);
         me->msiir_new_lib[ch].mem_ptr = NULL;
      }

      result_lib = msiir_init_mem(&(me->msiir_new_lib[ch]), &(me->msiir_static_vars), ptr, malloc_size);
      if (MSIIR_SUCCESS != result_lib)
      {
         MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : failed initialize new MSIIR library");
         posal_memory_free(ptr);
         me->msiir_new_lib[ch].mem_ptr = NULL;
         return CAPI_EFAILED;
      }
   }
   else if (MSIIR_CURRENT_INSTANCE == instance_id)
   {
      if (NULL != me->msiir_lib[ch].mem_ptr)
      {
#ifdef CAPI_MSIIR_DEBUG_MSG
         MSIIR_MSG(me->miid, DBG_HIGH_PRIO,
                "CAPI MSIIR : releasing existing library ptr before creating new 0x%p",
                me->msiir_lib[ch].mem_ptr);
#endif
         posal_memory_free(me->msiir_lib[ch].mem_ptr);
         me->msiir_lib[ch].mem_ptr = NULL;
      }

      result_lib = msiir_init_mem(&(me->msiir_lib[ch]), &(me->msiir_static_vars), ptr, malloc_size);
      if (MSIIR_SUCCESS != result_lib)
      {
         MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : failed initialize new MSIIR library");
         posal_memory_free(ptr);
         me->msiir_lib[ch].mem_ptr = NULL;
         return CAPI_EFAILED;
      }

      // no cross fading if only current instances are created
      me->msiir_new_lib[ch].mem_ptr = NULL;

      // read and save the library's default settings for pregain and config (coeffs and shift factors)
      if (CAPI_EOK != capi_msiir_get_default_pregain_config(me, ch))
      {
         return CAPI_EFAILED;
      }
   }
   else
   {
      MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : invalid instance id when creating MSIIR library");
      posal_memory_free(ptr);
      return CAPI_EFAILED;
   }

   me->per_chan_mem_req.mem_size = malloc_size;
#ifdef CAPI_MSIIR_DEBUG_MSG
   MSIIR_MSG(me->miid, DBG_HIGH_PRIO, "CAPI MSIIR : created new MSIIR library of size %lu", malloc_size);
#endif
   return CAPI_EOK;
}

#ifdef __cplusplus
}
#endif /* __cplusplus */
