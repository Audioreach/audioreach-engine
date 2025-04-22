/* ======================================================================== */
/**
   @file capi_chmixer_utils.cpp

   C source file to implement the Audio Post Processor Interface utilities for
   Channel Mixer.
*/

/* =========================================================================
  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
   ========================================================================= */

/*------------------------------------------------------------------------
 * Include files and Macro definitions
 * -----------------------------------------------------------------------*/
#include "capi_chmixer_utils.h"
#include "posal.h"
#ifdef CAPI_CHMIXER_DATA_LOG
#include <stdio.h>
#endif

/*------------------------------------------------------------------------
 * Function definitions
 * -----------------------------------------------------------------------*/

/*===========================================================================
    FUNCTION : capi_init_mem_req
    DESCRIPTION: Function to get init memory requirement for capi v2 module.
===========================================================================*/
uint32_t capi_get_init_mem_req()
{
   return (align_to_8_byte(sizeof(capi_chmixer_t)));
}

/*===========================================================================
    FUNCTION : copy_media_fmt_v2
    DESCRIPTION: Function to copy v2 version of media format to another v2 structure.
===========================================================================*/
static void copy_media_fmt_v2(capi_media_fmt_v2_t *media_fmt_v2_dst, const capi_media_fmt_v2_t *const media_fmt_v2_src)
{
   media_fmt_v2_dst->header               = media_fmt_v2_src->header;
   media_fmt_v2_dst->format               = media_fmt_v2_src->format;
   media_fmt_v2_dst->format.minor_version = CAPI_MEDIA_FORMAT_MINOR_VERSION;

   memscpy(media_fmt_v2_dst->channel_type,
           sizeof(media_fmt_v2_dst->channel_type),
           media_fmt_v2_src->channel_type,
           media_fmt_v2_src->format.num_channels * sizeof(media_fmt_v2_src->channel_type[0]));

   return;
}

/*===========================================================================
    FUNCTION : capi_chmixer_init_events
    DESCRIPTION: Function to init all the events with invalid values.
===========================================================================*/
void capi_chmixer_init_events(capi_chmixer_t *const me_ptr)
{
   me_ptr->event_config.chmixer_enable = TRUE;

   // Setting events to maximum(invalid) value.
   me_ptr->event_config.chmixer_KPPS        = 0x7FFFFFFF;
   me_ptr->event_config.chmixer_bandwidth   = 0x7FFFFFFF;
   me_ptr->event_config.chmixer_delay_in_us = 0x7FFFFFFF;
}

/*===========================================================================
    FUNCTION : capi_chmixer_check_ch_type
    DESCRIPTION: Function to check valid channel type.
===========================================================================*/
capi_err_t capi_chmixer_check_ch_type(const uint16_t *channel_type, const uint32_t array_size)
{
   for (uint8_t i = 0; (i < array_size) && (i < CAPI_CHMIXER_MAX_CHANNELS); i++)
   {
      if ((channel_type[i] < (uint16_t)PCM_CHANNEL_L) || (channel_type[i] > (uint16_t)PCM_MAX_CHANNEL_MAP_V2))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "CAPI CHMIXER: Unsupported channel type channel idx %d, channel type %d",
                (int)i,
                (int)channel_type[i]);
         return CAPI_EBADPARAM;
      }
   }

   return CAPI_EOK;
}

/*===========================================================================
    FUNCTION : capi_chmixer_check_output_media_change_v2
    DESCRIPTION: Function to check if output media format is changed using media_fmt_v2.
===========================================================================*/
static bool_t capi_chmixer_check_output_media_change(const capi_media_fmt_v2_t *const output_media_fmt1,
                                                     const capi_media_fmt_v2_t *const output_media_fmt2)
{
   if (output_media_fmt1->format.num_channels != output_media_fmt2->format.num_channels)
   {
      return TRUE;
   }

   for (uint8_t i = 0; (i < output_media_fmt1->format.num_channels) && (i < CAPI_CHMIXER_MAX_CHANNELS); i++)
   {
      if (output_media_fmt1->channel_type[i] != output_media_fmt2->channel_type[i])
      {
         return TRUE;
      }
   }

   return FALSE;
}

/*===========================================================================
    FUNCTION : capi_chmixer_is_supported_input_media_fmt
    DESCRIPTION: Function to check if input media format is supported by module
===========================================================================*/
static capi_err_t capi_chmixer_is_supported_input_media_fmt(const capi_media_fmt_v2_t *const input_media_fmt)
{
   if (CAPI_FIXED_POINT != input_media_fmt->header.format_header.data_format)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI CHMIXER: Unsupported data format");
      return CAPI_EUNSUPPORTED;
   }

   if ((16 != input_media_fmt->format.bits_per_sample) && (32 != input_media_fmt->format.bits_per_sample))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "CAPI CHMIXER: Only supports 16 and 32 bit data. Received %lu.",
             input_media_fmt->format.bits_per_sample);
      return CAPI_EUNSUPPORTED;
   }

   if (CAPI_DEINTERLEAVED_UNPACKED != input_media_fmt->format.data_interleaving)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI CHMIXER: Only deinterleaved unpacked data supported.");
      return CAPI_EUNSUPPORTED;
   }

   if (!input_media_fmt->format.data_is_signed)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI CHMIXER: Unsigned data not supported.");
      return CAPI_EUNSUPPORTED;
   }

   if ((0 == input_media_fmt->format.num_channels) ||
       (CAPI_CHMIXER_MAX_CHANNELS < input_media_fmt->format.num_channels))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "CAPI CHMIXER: Only upto %lu channels supported. Received %lu.",
             CAPI_CHMIXER_MAX_CHANNELS,
             input_media_fmt->format.num_channels);
      return CAPI_EUNSUPPORTED;
   }

   if (0 == input_media_fmt->format.sampling_rate)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "CAPI CHMIXER: Unsupported sampling rate. Received %lu.",
             input_media_fmt->format.sampling_rate);
      return CAPI_EUNSUPPORTED;
   }

   if (CAPI_EOK != capi_chmixer_check_ch_type(input_media_fmt->channel_type, input_media_fmt->format.num_channels))
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI CHMIXER: Unsupported channel type");
      return CAPI_EUNSUPPORTED;
   }
   return CAPI_EOK;
}

/*===========================================================================
    FUNCTION : capi_chmixer_is_supported_output_media_fmt
    DESCRIPTION: Function to check if output media format is supported by module using media_fmt_v2
===========================================================================*/
static capi_err_t capi_chmixer_is_supported_output_media_fmt(const capi_media_fmt_v2_t *const output_media_fmt)
{
   if ((0 == output_media_fmt->format.num_channels) ||
       (CAPI_CHMIXER_MAX_CHANNELS < output_media_fmt->format.num_channels))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "CAPI CHMIXER: Only upto %lu channels supported. Received %lu.",
             CAPI_CHMIXER_MAX_CHANNELS,
             output_media_fmt->format.num_channels);
      return CAPI_EUNSUPPORTED;
   }

   if (CAPI_EOK != capi_chmixer_check_ch_type(output_media_fmt->channel_type, output_media_fmt->format.num_channels))
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI CHMIXER: Unsupported channel type");
      return CAPI_EUNSUPPORTED;
   }
   return CAPI_EOK;
}

static capi_err_t capi_chmixer_is_supported_media_type(const capi_media_fmt_v2_t *const input_media_fmt,
                                                       const capi_media_fmt_v2_t *const output_media_fmt)
{
   capi_err_t out_result = capi_chmixer_is_supported_output_media_fmt(output_media_fmt);
   capi_err_t inp_result = capi_chmixer_is_supported_input_media_fmt(input_media_fmt);
   return (inp_result | out_result);
}

capi_err_t capi_chmixer_set_output_media_fmt(capi_chmixer_t *const            me_ptr,
                                             const capi_media_fmt_v2_t *const out_media_fmt)
{
   capi_err_t capi_result = CAPI_EOK;

   if (capi_chmixer_check_output_media_change(out_media_fmt, &me_ptr->output_media_fmt[CAPI_CHMIXER_DEFAULT_PORT]))
   {
      copy_media_fmt_v2(&me_ptr->output_media_fmt[CAPI_CHMIXER_DEFAULT_PORT], out_media_fmt);

      capi_result |= capi_chmixer_is_supported_output_media_fmt(&me_ptr->output_media_fmt[CAPI_CHMIXER_DEFAULT_PORT]);
      if (CAPI_SUCCEEDED(capi_result))
      {
#ifdef CAPI_CHMIXER_DEBUG_MSG
         CHMIXER_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Valid output media format.");
#endif
         if (me_ptr->input_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].format.num_channels != CAPI_DATA_FORMAT_INVALID_VAL)
         {
            capi_result |= capi_chmixer_check_init_lib_instance(me_ptr);
            if (CAPI_SUCCEEDED(capi_result))
            {
               capi_chmixer_raise_events(me_ptr, TRUE);
            }
            else
            {
               return capi_result;
            }
         }
      }
      else
      {
         me_ptr->config.lib_enable = FALSE;
         capi_chmixer_raise_events(me_ptr, FALSE);
      }
   }
   return capi_result;
}

/*===========================================================================
    FUNCTION : capi_chmixer_raise_events
    DESCRIPTION: Function to raise all the events using the callback function
===========================================================================*/
capi_err_t capi_chmixer_raise_events(capi_chmixer_t *const me_ptr, bool_t media_fmt_update)
{
   capi_err_t result = CAPI_EOK;
   if (NULL == me_ptr->cb_info.event_cb)
   {
#ifdef CAPI_CHMIXER_DEBUG_MSG
      CHMIXER_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Event callback is not set. Unable to raise events!");
#endif
      return result;
   }

   if (media_fmt_update)
   {
      // raise events which are only media format dependent
      const uint32_t sample_rate = me_ptr->input_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].format.sampling_rate;
      if (sample_rate == CAPI_DATA_FORMAT_INVALID_VAL)
      {
         return CAPI_EFAILED;
      }
      const uint32_t bw          = 0;
      const uint32_t delay_in_us = 0;

      uint32_t out_ch           = me_ptr->output_media_fmt[0].format.num_channels;
      uint32_t num_active_coeff = ChMixerGetNumActiveCoefficients((ChMixerStateStruct *)me_ptr->lib_ptr);

      // Using constants -8, 4, 5 through profiling, + 225 for capi kpps
      // Opening up the brackets for better div/mult:
      // out ch * (-8 + num_samples(4 + (5* in_ch* num_active_coeff/(in_ch * out_ch)))) + c
      // => out_ch * (-8 + (num_samples * 4) + (num_samples * 5* num_coeff/out_ch)) + c
      // sr/1000 translates to num samples
      uint32_t kpps =
         (out_ch * (-8 + ((sample_rate * 4) + (sample_rate * 5 * num_active_coeff / out_ch)) / 1000)) + 225;

      CHMIXER_MSG(me_ptr->miid,
                  DBG_HIGH_PRIO,
                  "Raising output media fmt with kpps %d num_active_coeff %d",
                  kpps,
                  num_active_coeff);

      // Raise event if kpps is changed.
      if (kpps != me_ptr->event_config.chmixer_KPPS)
      {
         me_ptr->event_config.chmixer_KPPS = kpps;
         result                            = capi_cmn_update_kpps_event(&me_ptr->cb_info, kpps);
      }

      // Raise event if bw is changed.
      if (bw != me_ptr->event_config.chmixer_bandwidth)
      {
         me_ptr->event_config.chmixer_bandwidth = bw;
         result                                 = capi_cmn_update_bandwidth_event(&me_ptr->cb_info, 0, 0);
      }

      // Raise event if algo delay is changed.
      if (delay_in_us != me_ptr->event_config.chmixer_delay_in_us)
      {
         me_ptr->event_config.chmixer_delay_in_us = delay_in_us;
         result                                   = capi_cmn_update_algo_delay_event(&me_ptr->cb_info, 0);
      }

#ifdef CAPI_CHMIXER_DEBUG_MSG
      CHMIXER_MSG(me_ptr->miid,
                  DBG_HIGH_PRIO,
                  "Raising output media fmt with num ch %d",
                  me_ptr->output_media_fmt[0].format.num_channels);
#endif

      result = capi_cmn_output_media_fmt_event_v2(&me_ptr->cb_info,
                                                  &me_ptr->output_media_fmt[CAPI_CHMIXER_DEFAULT_PORT],
                                                  FALSE,
                                                  0);
   }

   const bool_t process_check = (me_ptr->config.lib_enable) && (me_ptr->client_enable != 0);
   if (me_ptr->event_config.chmixer_enable != process_check)
   {
      me_ptr->event_config.chmixer_enable = process_check;
      result                              = capi_cmn_update_process_check_event(&me_ptr->cb_info, process_check);
   }

   return result;
}

/*===========================================================================
    FUNCTION : capi_chmixer_reinit
    DESCRIPTION: Function to initialize chmixer libraries.
===========================================================================*/
static capi_err_t capi_chmixer_reinit(capi_chmixer_t *const me_ptr)
{
   capi_media_fmt_v2_t *inp_media_fmt = &(me_ptr->input_media_fmt[CAPI_CHMIXER_DEFAULT_PORT]);
   capi_media_fmt_v2_t *out_media_fmt = &(me_ptr->output_media_fmt[CAPI_CHMIXER_DEFAULT_PORT]);

   ChMixerChType in_ch_type[CH_MIXER_MAX_NUM_CH];
   for (uint32_t i = 0; i < inp_media_fmt->format.num_channels; i++)
   {
      in_ch_type[i] = (ChMixerChType)inp_media_fmt->channel_type[i];
   }

   ChMixerChType out_ch_type[CH_MIXER_MAX_NUM_CH];
   for (uint32_t i = 0; i < out_media_fmt->format.num_channels; i++)
   {
      out_ch_type[i] = (ChMixerChType)out_media_fmt->channel_type[i];
   }

   // Find the coef set which matches the current media format
   capi_chmixer_coef_set *coef_set_ptr = me_ptr->config.coef_sets_ptr;
   int16_t *              coef_ptr     = NULL;

   for (int8_t i = me_ptr->config.num_coef_sets - 1; i >= 0; i--)
   {
      if ((me_ptr->config.num_coef_sets - 1) != i) // except the first iteration
      {
         coef_set_ptr++;
      }

      if (coef_set_ptr->num_in_ch != inp_media_fmt->format.num_channels)
      {
         continue;
      }
      if (coef_set_ptr->num_out_ch != out_media_fmt->format.num_channels)
      {
         continue;
      }

      uint32_t j = 0;
      for (j = 0; j < inp_media_fmt->format.num_channels; j++)
      {
         if (coef_set_ptr->in_ch_map[j] != inp_media_fmt->channel_type[j])
         {
            break;
         }
      }

      if (j < inp_media_fmt->format.num_channels)
      {
         continue;
      }

      for (j = 0; j < out_media_fmt->format.num_channels; j++)
      {
         if (coef_set_ptr->out_ch_map[j] != out_media_fmt->channel_type[j])
         {
            break;
         }
      }

      if (j == out_media_fmt->format.num_channels)
      {
         coef_ptr = coef_set_ptr->coef_ptr;
         break;
      }
   }

   // Initialize channel mixer lib.
   ChMixerResultType result = ChMixerSetParam(me_ptr->lib_ptr,
                                              me_ptr->lib_instance_size,
											  (uint32) inp_media_fmt->format.num_channels,
                                              in_ch_type,
                                              (uint32)out_media_fmt->format.num_channels,
                                              out_ch_type,
                                              (uint32)inp_media_fmt->format.bits_per_sample,
                                              coef_ptr);
   if (CH_MIXER_SUCCESS != result)
   {
      CHMIXER_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Reinit failed.");
      return CAPI_EFAILED;
   }

   me_ptr->config.lib_enable = TRUE; // chmixer is successfully initialized.

   // if input and output media format is exactly same and
   // there is no custom coefficients then raise process check as FALSE.
   bool_t is_chmixer_req = FALSE; // disabled by default

   if (inp_media_fmt->format.num_channels == out_media_fmt->format.num_channels)
   {
      for (uint16_t i = 0; i < inp_media_fmt->format.num_channels; i++)
      {
         if (inp_media_fmt->channel_type[i] != out_media_fmt->channel_type[i])
         {
            is_chmixer_req = TRUE; // If num channels are same but chmaps are different -> enable
            break;
         }
      }

      // inp chmap == out chmap and coef ptr is not null, check if we need to enable
      if ((FALSE == is_chmixer_req) && (NULL != coef_ptr))
      {
         bool_t is_identity_matrix = TRUE;
         for (uint32_t row = 0; row < out_media_fmt->format.num_channels; row++)
         {
            for (uint32_t col = 0; col < inp_media_fmt->format.num_channels; col++)
            {
               if ((row == col) && (*(coef_ptr + (row * inp_media_fmt->format.num_channels + col)) != 0x4000))
               {
                  // If elements of main diagonal is not equal to 1
                  is_identity_matrix = FALSE;
                  break;
               }
               else if ((row != col) && *(coef_ptr + (row * inp_media_fmt->format.num_channels + col)) != 0)
               {
                  // If other elements than main diagonal is not equal to 0
                  is_identity_matrix = FALSE;
                  break;
               }
            }
            if (!is_identity_matrix)
            {
               is_chmixer_req = TRUE;
               break; // break out of second for loop
            }
         }
      }
   }
   else
   {
      is_chmixer_req = TRUE; // If num channels are diff -> enable
   }

   me_ptr->config.lib_enable = is_chmixer_req;

   return CAPI_EOK;
}

/*===========================================================================
    FUNCTION : capi_chmixer_check_create_lib_instance
    DESCRIPTION: Function to check the media fmt and create the chmixer instance.
===========================================================================*/
capi_err_t capi_chmixer_check_init_lib_instance(capi_chmixer_t *const me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   me_ptr->config.lib_enable = FALSE;

   capi_media_fmt_v2_t *inp_media_fmt = &(me_ptr->input_media_fmt[CAPI_CHMIXER_DEFAULT_PORT]);
   capi_media_fmt_v2_t *out_media_fmt = &(me_ptr->output_media_fmt[CAPI_CHMIXER_DEFAULT_PORT]);

   // Sanity check for valid media format
   capi_result |= capi_chmixer_is_supported_media_type(inp_media_fmt, out_media_fmt);
   if (CAPI_FAILED(capi_result))
   {
      // Raise process check; FALSE
      capi_chmixer_raise_events(me_ptr, FALSE);
      return capi_result;
   }
   else
   {
      out_media_fmt->header                   = inp_media_fmt->header;
      out_media_fmt->format.minor_version     = CAPI_MEDIA_FORMAT_MINOR_VERSION;
      out_media_fmt->format.bitstream_format  = inp_media_fmt->format.bitstream_format;
      out_media_fmt->format.bits_per_sample   = inp_media_fmt->format.bits_per_sample;
      out_media_fmt->format.q_factor          = inp_media_fmt->format.q_factor;
      out_media_fmt->format.data_is_signed    = inp_media_fmt->format.data_is_signed;
      out_media_fmt->format.data_interleaving = inp_media_fmt->format.data_interleaving;
      out_media_fmt->format.sampling_rate     = inp_media_fmt->format.sampling_rate;
   }

   uint32_t lib_size = 0;
   ChMixerGetInstanceSize(&lib_size, inp_media_fmt->format.num_channels, out_media_fmt->format.num_channels);
   if ((me_ptr->lib_ptr) && (me_ptr->lib_instance_size != lib_size))
   {
	  posal_memory_aligned_free(me_ptr->lib_ptr);
      me_ptr->lib_instance_size = 0;
      me_ptr->lib_ptr           = NULL;
   }

   if (!me_ptr->lib_ptr)
   {
      me_ptr->lib_ptr =
         posal_memory_aligned_malloc(lib_size, MEM_ALIGN_EIGHT_BYTE, (POSAL_HEAP_ID)me_ptr->heap_mem.heap_id);
      if (NULL == me_ptr->lib_ptr)
      {
         CHMIXER_MSG(me_ptr->miid,
                     DBG_ERROR_PRIO,
                     "failed to allocate the memory to create channel mixer library %lu",
                     capi_result);
         me_ptr->lib_instance_size = 0;
         return CAPI_ENOMEMORY;
      }
      me_ptr->lib_instance_size = lib_size;
   }

   capi_result |= capi_chmixer_reinit(me_ptr);
   if (CAPI_FAILED(capi_result))
   {
      // Raise process check; FALSE
      capi_chmixer_raise_events(me_ptr, FALSE);
      return capi_result;
   }

#ifdef CAPI_CHMIXER_DEBUG_MSG
   CHMIXER_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Input Media Fmt:");
   CHMIXER_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Number of Channels %lu", inp_media_fmt->format.num_channels);
   for (uint8_t i = 0; i < inp_media_fmt->format.num_channels; i++)
   {
      CHMIXER_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Channel_Type[%hhu]: %hu", i, inp_media_fmt->format.channel_type[i]);
   }

   CHMIXER_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Output Media Fmt:");
   CHMIXER_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Number of Channels %lu", out_media_fmt->format.num_channels);
   for (uint8_t i = 0; i < out_media_fmt->format.num_channels; i++)
   {
      CHMIXER_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Channel_Type[%hhu]: %hu", i, out_media_fmt->format.channel_type[i]);
   }
#endif

   return capi_result;
}

/*===========================================================================
    FUNCTION : capi_chmixer_process_set_properties
    DESCRIPTION: Function to set properties
===========================================================================*/
capi_err_t capi_chmixer_process_set_properties(capi_chmixer_t *me_ptr, capi_proplist_t *proplist_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr)
   {
      CHMIXER_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "Set common property received null ptr");
      return CAPI_EBADPARAM;
   }

   capi_result |= capi_cmn_set_basic_properties(proplist_ptr, &me_ptr->heap_mem, &me_ptr->cb_info, TRUE);
   if (CAPI_EOK != capi_result)
   {
      CHMIXER_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Set basic properties failed with result %lu", capi_result);
   }

   capi_prop_t *prop_array = proplist_ptr->prop_ptr;
   for (uint8_t i = 0; i < proplist_ptr->props_num; i++)
   {
      const capi_buf_t *const payload_ptr             = &(prop_array[i].payload);
      const uint32_t          payload_actual_data_len = payload_ptr->actual_data_len;
      const uint32_t          param_id                = prop_array[i].id;

      capi_result = CAPI_EOK;
      switch (param_id)
      {
         case CAPI_HEAP_ID:
         case CAPI_EVENT_CALLBACK_INFO:
         case CAPI_ALGORITHMIC_RESET:
         case CAPI_CUSTOM_INIT_DATA:
         case CAPI_PORT_NUM_INFO:
         case CAPI_INTERFACE_EXTENSIONS:
         {
            break;
         }
         case CAPI_INPUT_MEDIA_FORMAT_V2:
         {
            if (NULL == payload_ptr->data_ptr)
            {
               CHMIXER_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Set property id 0x%lx, received null buffer.", param_id);
               CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
               break;
            }
            uint32_t required_size = CAPI_CHMIXER_V2_MIN_SIZE;
            if (payload_actual_data_len >= required_size)
            {
#ifdef CAPI_CHMIXER_DEBUG_MSG
               CHMIXER_MSG(me_ptr->miid, DBG_HIGH_PRIO, "received input media fmt");
#endif

               // sanity check for valid and input port
               if ((!prop_array[i].port_info.is_valid) || (!prop_array[i].port_info.is_input_port))
               {
                  CHMIXER_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Set property id 0x%lx, invalid port info.", param_id);
                  CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
                  break;
               }

               // sanity check for valid input port index
               if (CAPI_CHMIXER_DEFAULT_PORT != prop_array[i].port_info.port_index)
               {
                  CHMIXER_MSG(me_ptr->miid,
                              DBG_ERROR_PRIO,
                              "Set property id 0x%lx, invalid input port index %lu",
                              param_id,
                              prop_array[i].port_info.port_index);
                  CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
                  break;
               }

               const capi_media_fmt_v2_t *const in_data_ptr = (capi_media_fmt_v2_t *)(payload_ptr->data_ptr);
               if (in_data_ptr->format.minor_version >= CAPI_MEDIA_FORMAT_MINOR_VERSION)
               {
#ifdef CAPI_CHMIXER_KPPS_PROFILING
                  if (in_data_ptr->format.sampling_rate !=
                      me_ptr->input_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].format.sampling_rate)
                  {
                     capi_chmixer_kpps_print(me_ptr);
                     capi_chmixer_kpps_profiler_reset(me_ptr);
                  }
#endif
                  required_size += in_data_ptr->format.num_channels * sizeof(in_data_ptr->channel_type[0]);
                  if (payload_actual_data_len < required_size)
                  {
                     CHMIXER_MSG(me_ptr->miid,
                                 DBG_ERROR_PRIO,
                                 "Set property id 0x%lx, Bad param size %lu",
                                 param_id,
                                 payload_actual_data_len);
                     CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
                     break;
                  }
                  copy_media_fmt_v2(&me_ptr->input_media_fmt[CAPI_CHMIXER_DEFAULT_PORT], in_data_ptr);

                  capi_result |=
                     capi_chmixer_is_supported_input_media_fmt(&me_ptr->input_media_fmt[CAPI_CHMIXER_DEFAULT_PORT]);
                  if (CAPI_SUCCEEDED(capi_result))
                  {
#ifdef CAPI_CHMIXER_DEBUG_MSG
                     CHMIXER_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Valid input media format.");
#endif
                     // Set flag
                     me_ptr->inp_media_fmt_received = TRUE;

                     if (me_ptr->use_default_channel_info[CAPI_CHMIXER_DEFAULT_PORT])
                     {
                        me_ptr->output_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].format =
                           me_ptr->input_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].format;
                        memscpy(me_ptr->output_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].channel_type,
                                sizeof(me_ptr->output_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].channel_type),
                                me_ptr->input_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].channel_type,
                                me_ptr->input_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].format.num_channels *
                                   sizeof(me_ptr->input_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].channel_type[0]));
                     }

                     if (FALSE == me_ptr->is_native_mode) // overwrite if its a valid value
                     {
#ifdef CAPI_CHMIXER_DEBUG_MSG
                        CHMIXER_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Received valid output mf");
#endif
                        me_ptr->output_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].format.num_channels =
                           me_ptr->configured_num_channels;
                        memscpy(me_ptr->output_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].channel_type,
                                sizeof(me_ptr->output_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].channel_type),
                                me_ptr->configured_ch_map,
                                (sizeof(uint16_t) * me_ptr->configured_num_channels));
                     }

                     if (me_ptr->output_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].format.num_channels !=
                         CAPI_DATA_FORMAT_INVALID_VAL)
                     {
                        capi_result |= capi_chmixer_check_init_lib_instance(me_ptr);
                        if (CAPI_SUCCEEDED(capi_result))
                        {
                           capi_chmixer_raise_events(me_ptr, TRUE);
                        }
                        else
                        {
                           return capi_result;
                        }
                     }
                  }
                  else
                  {
                     me_ptr->config.lib_enable = FALSE;
                     capi_chmixer_raise_events(me_ptr, FALSE);
                  }
               }
               else
               {
                  CHMIXER_MSG(me_ptr->miid,
                              DBG_ERROR_PRIO,
                              "Set property id 0x%lx, Bad version %lu",
                              param_id,
                              in_data_ptr->format.minor_version);
                  CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
                  break;
               }
            }
            else
            {
               CHMIXER_MSG(me_ptr->miid,
                           DBG_ERROR_PRIO,
                           "Set property id 0x%lx, Bad param size %lu",
                           param_id,
                           payload_actual_data_len);
               CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
               break;
            }
            break;
         }
         case CAPI_OUTPUT_MEDIA_FORMAT_V2:
         {
            if (NULL == payload_ptr->data_ptr)
            {
               CHMIXER_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Set property id 0x%lx, received null buffer.", param_id);
               CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
               break;
            }

            uint32_t required_size = CAPI_CHMIXER_V2_MIN_SIZE;
            if (payload_actual_data_len >= required_size)
            {
#ifdef CAPI_CHMIXER_DEBUG_MSG
               CHMIXER_MSG(me_ptr->miid, DBG_HIGH_PRIO, "received output media fmt");
#endif

               // sanity check for valid and input port
               if ((!prop_array[i].port_info.is_valid) || (prop_array[i].port_info.is_input_port))
               {
                  CHMIXER_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Set property id 0x%lx, invalid port info.", param_id);
                  CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
                  break;
               }

               // sanity check for valid input port index
               if (CAPI_CHMIXER_DEFAULT_PORT != prop_array[i].port_info.port_index)
               {
                  CHMIXER_MSG(me_ptr->miid,
                              DBG_ERROR_PRIO,
                              "Set property id 0x%lx, invalid output port index %lu",
                              param_id,
                              prop_array[i].port_info.port_index);
                  CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
                  break;
               }

               const capi_media_fmt_v2_t *const out_data_ptr = (capi_media_fmt_v2_t *)(payload_ptr->data_ptr);
               if (out_data_ptr->format.minor_version >= CAPI_MEDIA_FORMAT_MINOR_VERSION)
               {
                  required_size += out_data_ptr->format.num_channels * sizeof(out_data_ptr->channel_type[0]);
                  if (payload_actual_data_len < required_size)
                  {
                     CHMIXER_MSG(me_ptr->miid,
                                 DBG_ERROR_PRIO,
                                 "Set property id 0x%lx, Bad param size %lu",
                                 param_id,
                                 payload_actual_data_len);
                     CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
                     break;
                  }
                  me_ptr->use_default_channel_info[CAPI_CHMIXER_DEFAULT_PORT] = FALSE;

                  capi_result = capi_chmixer_set_output_media_fmt(me_ptr, out_data_ptr);
               }
               else
               {
                  CHMIXER_MSG(me_ptr->miid,
                              DBG_ERROR_PRIO,
                              "Set property id 0x%lx, Bad version %lu",
                              param_id,
                              out_data_ptr->format.minor_version);
                  CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
                  break;
               }
            }
            else
            {
               CHMIXER_MSG(me_ptr->miid,
                           DBG_ERROR_PRIO,
                           "Set property id 0x%lx, Bad param size %lu",
                           param_id,
                           payload_actual_data_len);
               CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
               break;
            }
            break;
         }
         case CAPI_MODULE_INSTANCE_ID:
         {
            if (payload_ptr->actual_data_len >= sizeof(capi_module_instance_id_t))
            {
               capi_module_instance_id_t *data_ptr = (capi_module_instance_id_t *)payload_ptr->data_ptr;
               me_ptr->miid                        = data_ptr->module_instance_id;
               CHMIXER_MSG(me_ptr->miid,
                           DBG_LOW_PRIO,
                           "This module-id 0x%08lX, instance-id 0x%08lX",
                           data_ptr->module_id,
                           me_ptr->miid);
            }
            else
            {
               CHMIXER_MSG(me_ptr->miid,
                           DBG_ERROR_PRIO,
                           "Set property id 0x%lx, Bad param size %lu",
                           param_id,
                           payload_ptr->max_data_len);
               capi_result |= CAPI_ENEEDMORE;
            }
            break;
         } // CAPI_MODULE_INSTANCE_ID
         default:
         {
            CHMIXER_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Set property id 0x%lx, Not supported.", param_id);
            continue;
         }
      }
   }
   return capi_result;
}

/*===========================================================================
    FUNCTION : capi_chmixer_process_get_properties
    DESCRIPTION: chmixer get properties
===========================================================================*/
capi_err_t capi_chmixer_process_get_properties(capi_chmixer_t *me_ptr, capi_proplist_t *proplist_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   capi_basic_prop_t mod_prop;
   mod_prop.init_memory_req    = capi_get_init_mem_req();
   mod_prop.stack_size         = CAPI_CHMIXER_STACK_SIZE;
   mod_prop.num_fwk_extns      = 0;
   mod_prop.fwk_extn_ids_arr   = NULL;
   mod_prop.is_inplace         = FALSE;
   mod_prop.req_data_buffering = FALSE;
   mod_prop.max_metadata_size  = 0; // NA

   uint32_t miid = me_ptr ? me_ptr->miid : MIID_UNKNOWN;
   capi_result |= capi_cmn_get_basic_properties(proplist_ptr, &mod_prop);
   if (CAPI_EOK != capi_result)
   {
      CHMIXER_MSG(miid, DBG_ERROR_PRIO, "Get common basic properties failed with result %lu", capi_result);
   }

   capi_prop_t *const prop_array = proplist_ptr->prop_ptr;

   for (uint32_t i = 0; i < proplist_ptr->props_num; i++)
   {
      capi_buf_t *const payload_ptr = &(prop_array[i].payload);
      miid                          = me_ptr ? me_ptr->miid : MIID_UNKNOWN;

      if (NULL == payload_ptr->data_ptr)
      {
         CHMIXER_MSG(miid, DBG_ERROR_PRIO, "Get property id 0x%lx, received null buffer.", prop_array[i].id);
         continue;
      }

      switch (prop_array[i].id)
      {
         case CAPI_INIT_MEMORY_REQUIREMENT:
         case CAPI_STACK_SIZE:
         case CAPI_IS_INPLACE:
         case CAPI_REQUIRES_DATA_BUFFERING:
         case CAPI_NUM_NEEDED_FRAMEWORK_EXTENSIONS:
         case CAPI_OUTPUT_MEDIA_FORMAT_SIZE:
         case CAPI_MAX_METADATA_SIZE:
         case CAPI_INTERFACE_EXTENSIONS:
         {
            break;
         }
         case CAPI_PORT_DATA_THRESHOLD:
         {
            if (NULL == me_ptr)
            {
               CHMIXER_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "null ptr while querying threshold");
               return CAPI_EBADPARAM;
            }

            uint32_t threshold_in_bytes = 1; // default
            capi_result                 = capi_cmn_handle_get_port_threshold(&prop_array[i], threshold_in_bytes);
            break;
         }
         case CAPI_OUTPUT_MEDIA_FORMAT_V2:
         {
            if (NULL == me_ptr)
            {
               CHMIXER_MSG(MIID_UNKNOWN,
                           DBG_ERROR_PRIO,
                           "Get property id 0x%lx, module is not allocated",
                           prop_array[i].id);
               CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
               break;
            }

            // sanity check for valid output port index
            if (CAPI_CHMIXER_DEFAULT_PORT != prop_array[i].port_info.port_index)
            {
               CHMIXER_MSG(miid, DBG_ERROR_PRIO, "Get property id 0x%lx, invalid outptu port index.", prop_array[i].id);
               CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
               break;
            }

            capi_result = capi_cmn_handle_get_output_media_fmt_v2(&prop_array[i],
                                                                  &me_ptr->output_media_fmt[CAPI_CHMIXER_DEFAULT_PORT]);
            break;
         }
         default:
         {
            CHMIXER_MSG(miid, DBG_ERROR_PRIO, "Get property id 0x%lx, Not Supported.", prop_array[i].id);
            continue;
         }
      }
   }
   return capi_result;
}

#ifdef CAPI_CHMIXER_KPPS_PROFILING
/*===========================================================================
    FUNCTION : capi_chmixer_kpps_profiler_reset
    DESCRIPTION: reset profiler structure (should be called if media fmt changed)
===========================================================================*/
void capi_chmixer_kpps_profiler_reset(capi_chmixer_t *me_ptr)
{
   memset(&me_ptr->profiler, 0, sizeof(me_ptr->profiler));
}

/*===========================================================================
    FUNCTION : capi_chmixer_kpps_profiler
    DESCRIPTION: generates profiling data
===========================================================================*/
void capi_chmixer_kpps_profiler(capi_chmixer_t *me_ptr)
{
   uint64   frame_cycles;
   uint32_t sample_rate = me_ptr->input_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].format.sampling_rate;

   frame_cycles = me_ptr->profiler.end_cycles - me_ptr->profiler.start_cycles;
   me_ptr->profiler.total_cycles += frame_cycles;

   me_ptr->profiler.total_sample_count += me_ptr->profiler.frame_sample_count;

   me_ptr->profiler.frame_kpps = (frame_cycles * sample_rate) / (me_ptr->profiler.frame_sample_count * 1000);
   me_ptr->profiler.average_kpps =
      (me_ptr->profiler.total_cycles * sample_rate) / (me_ptr->profiler.total_sample_count * 1000);

   if (me_ptr->profiler.frame_kpps > me_ptr->profiler.peak_kpps)
   {
      me_ptr->profiler.peak_kpps = me_ptr->profiler.frame_kpps;
   }
}

/*===========================================================================
    FUNCTION : capi_chmixer_kpps_print
    DESCRIPTION: print profiling data
===========================================================================*/
void capi_chmixer_kpps_print(capi_chmixer_t *me_ptr)
{
   if (0 != me_ptr->profiler.total_sample_count)
   {
      CHMIXER_MSG(me_ptr->miid,
                  DBG_MED_PRIO,
                  "\nProfiling result:-\n"
                  "1.\t Number of Input Channels %lu\n"
                  "2.\t Number of Output Channels %lu\n"
                  "3.\t Sampling Rate %lu\n"
                  "4.\t Average KPPS %llu\n"
                  "5.\t Peak KPPS %llu\n"
                  "====================================================================================",
                  me_ptr->input_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].format.num_channels,
                  me_ptr->output_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].format.num_channels,
                  me_ptr->input_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].format.sampling_rate,
                  me_ptr->profiler.average_kpps,
                  me_ptr->profiler.peak_kpps);
   }
}
#endif

#ifdef CAPI_CHMIXER_DATA_LOG
/*===========================================================================
    FUNCTION : capi_chmixer_data_logger
    DESCRIPTION: log input and output data to and from chmixer capi v2
===========================================================================*/
void capi_chmixer_data_logger(capi_chmixer_t *me_ptr, const capi_stream_data_t *input, const capi_stream_data_t *output)
{
   static posal_atomic_word_t instance_id = { 0 };

   char_t in_file_name[100];
   char_t out_file_name[100];

   void * in_ptrs, *out_ptrs;
   uint16 i = 0;

   bool_t is_file_opened = TRUE;

   FILE *fp_in  = NULL;
   FILE *fp_out = NULL;

   if (0 == me_ptr->logger.instance_id)
   {
      me_ptr->logger.instance_id = (uint32_t)posal_atomic_increment(&instance_id);
   }

   const uint32_t sample_rate     = me_ptr->input_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].format.sampling_rate;
   const uint16_t num_in_channel  = me_ptr->input_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].format.num_channels;
   const uint16_t num_out_channel = me_ptr->output_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].format.num_channels;

   if ((me_ptr->logger.samp_rate != sample_rate) || (me_ptr->logger.num_in_channel != num_in_channel) ||
       (me_ptr->logger.num_out_channel != num_out_channel))
   {
      me_ptr->logger.samp_rate       = sample_rate;
      me_ptr->logger.num_in_channel  = num_in_channel;
      me_ptr->logger.num_out_channel = num_out_channel;
      me_ptr->logger.split_timestamp = me_ptr->logger.curr_timestamp;
      is_file_opened                 = FALSE;
   }

   for (i = 0; i < num_in_channel; i++)
   {
      in_ptrs = input->buf_ptr[i].data_ptr;

      snprintf(in_file_name,
               100,
               "chmixer_id%lu_in_ts%llu_sr%lu_ch%d.pcm",
               me_ptr->logger.instance_id,
               me_ptr->logger.split_timestamp,
               me_ptr->logger.samp_rate,
               i);

      if (!is_file_opened)
      {
         fp_in = fopen(in_file_name, "wb");
      }
      else
      {
         fp_in = fopen(in_file_name, "a+b");
      }

      if (NULL != fp_in)
      {
         fwrite(in_ptrs, 1, input->buf_ptr[0].actual_data_len, fp_in);
         fclose(fp_in);
         fp_in = NULL;
      }
   }

   for (i = 0; i < num_out_channel; i++)
   {
      out_ptrs = output->buf_ptr[i].data_ptr;

      snprintf(out_file_name,
               100,
               "chmixer_id%lu_out_ts%llu_sr%lu_ch%d.pcm",
               me_ptr->logger.instance_id,
               me_ptr->logger.split_timestamp,
               me_ptr->logger.samp_rate,
               i);

      if (!is_file_opened)
      {
         fp_out = fopen(out_file_name, "wb");
      }
      else
      {
         fp_out = fopen(out_file_name, "a+b");
      }

      if (NULL != fp_out)
      {
         fwrite(out_ptrs, 1, output->buf_ptr[0].actual_data_len, fp_out);
         fclose(fp_out);
         fp_out = NULL;
      }
   }

   uint8_t bit_width           = me_ptr->input_media_fmt[CAPI_CHMIXER_DEFAULT_PORT].format.bits_per_sample;
   uint8_t byte_sample_convert = (bit_width == 16) ? 1 : 2;
   me_ptr->logger.curr_timestamp += ((uint64)(input->buf_ptr[0].actual_data_len >> byte_sample_convert) * 1000000) /
                                    ((uint64)(me_ptr->logger.samp_rate));
}
#endif
