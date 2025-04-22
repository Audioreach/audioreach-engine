/* ======================================================================== */
/*
@file capi_multistageiir.cpp

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
/*------------------------------------------------------------------------
 * Static declarations
 * -----------------------------------------------------------------------*/
static capi_err_t capi_multistageiir_process(capi_t *            _pif,
                                                   capi_stream_data_t *input[],
                                                   capi_stream_data_t *output[]);

static capi_err_t capi_multistageiir_end(capi_t *_pif);

static capi_err_t capi_multistageiir_set_param(capi_t *                _pif,
                                                     uint32_t                   param_id,
                                                     const capi_port_info_t *port_info_ptr,
                                                     capi_buf_t *            params_ptr);

static capi_err_t capi_multistageiir_get_param(capi_t *                _pif,
                                                     uint32_t                   param_id,
                                                     const capi_port_info_t *port_info_ptr,
                                                     capi_buf_t *            params_ptr);

static capi_err_t capi_multistageiir_set_properties(capi_t *_pif, capi_proplist_t *props_ptr);

static capi_err_t capi_multistageiir_get_properties(capi_t *_pif, capi_proplist_t *props_ptr);

static const capi_vtbl_t capi_msiir_vtbl = { capi_multistageiir_process,
                                                   capi_multistageiir_end,
                                                   capi_multistageiir_set_param,
                                                   capi_multistageiir_get_param,
                                                   capi_multistageiir_set_properties,
                                                   capi_multistageiir_get_properties };

/*------------------------------------------------------------------------
  Function name: capi_multistageiir_process
  Processes an input buffer and generates an output buffer.
 * -----------------------------------------------------------------------*/
static capi_err_t capi_multistageiir_process(capi_t *            _pif,
                                                   capi_stream_data_t *input[],
                                                   capi_stream_data_t *output[])
{
   capi_err_t            result = CAPI_EOK;
   capi_multistageiir_t *me     = (capi_multistageiir_t *)(_pif);

   POSAL_ASSERT(me);
   POSAL_ASSERT(input[0]);
   POSAL_ASSERT(output[0]);
   POSAL_ASSERT(me->media_fmt[0].format.num_channels <= input[0]->bufs_num);
   POSAL_ASSERT(me->media_fmt[0].format.num_channels <= output[0]->bufs_num);

   void *inp_ptr[IIR_TUNING_FILTER_MAX_CHANNELS_V2] = { NULL };
   void *out_ptr[IIR_TUNING_FILTER_MAX_CHANNELS_V2] = { NULL };

   // calculate number of samples for first channel
   uint32_t num_samples  = 0;
   uint32_t shift_factor = 1;

   if (16 == me->msiir_static_vars.data_width)
   {
      shift_factor = 1; // samples are 16 bit, so divide by 2
   }
   else if (32 == me->msiir_static_vars.data_width)
   {
      shift_factor = 2; // samples are 32 bit, so divide by 4
   }

   num_samples = capi_msiir_s32_min_s32_s32(input[0]->buf_ptr[0].actual_data_len >> shift_factor,
                                 output[0]->buf_ptr[0].max_data_len >> shift_factor);

   for (uint32_t ch = 0; ch < me->media_fmt[0].format.num_channels; ch++)
   {
      inp_ptr[ch] = (void *)(input[0]->buf_ptr[ch].data_ptr);
      out_ptr[ch] = (void *)(output[0]->buf_ptr[ch].data_ptr);
   }
   if (me->start_cross_fade)
   {
      result = capi_msiir_start_cross_fade(me);
      if (CAPI_EOK != result)
      {
         MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : cross fade set param failed");
      }
   }
   // the library will loop through all the channels, and if the IIR are not enabled, the
   // library will copy_input_to_output
   for (uint32_t ch = 0; ch < me->media_fmt[0].format.num_channels; ch++)
   {
      MSIIR_RESULT      result_lib            = MSIIR_SUCCESS;
      CROSS_FADE_RESULT result_cross_fade_lib = CROSS_FADE_SUCCESS;

      if (me->enable_flag[ch])
      {
         result_lib = msiir_process_v2(&(me->msiir_lib[ch]), out_ptr[ch], inp_ptr[ch], num_samples);

         if (MSIIR_SUCCESS != result_lib)
         {
            MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : library process failed %d", result_lib);
            return CAPI_EFAILED;
         }

         // if the new msiir filters exist, check for cross fading processing
         if (NULL != me->msiir_new_lib[ch].mem_ptr)
         {
            uint32_t param_size = 0;

            result_cross_fade_lib = audio_cross_fade_get_param(&(me->cross_fade_lib[ch]),
                                                               CROSS_FADE_PARAM_MODE,
                                                               (int8 *)&(me->cross_fade_flag[ch]),
                                                               (uint32)sizeof(me->cross_fade_flag[ch]),
                                                               (uint32 *)&param_size);
            if ((CROSS_FADE_SUCCESS != result_cross_fade_lib) || (0 == param_size))
            {
               MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : cross fade processing failed");
               return CAPI_EFAILED;
            }

            if (1 == me->cross_fade_flag[ch])
            {
               int8 *cross_fade_in_ptrs[2];

               cross_fade_in_ptrs[0] = (int8 *)out_ptr[ch];
               cross_fade_in_ptrs[1] = (int8 *)inp_ptr[ch];

               // do the new msiir processing in-place on the input buffers
               result_lib = msiir_process_v2(&(me->msiir_new_lib[ch]), inp_ptr[ch], inp_ptr[ch], num_samples);

               if (MSIIR_SUCCESS != result_lib)
               {
                  MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : library process failed %d", result_lib);
                  return CAPI_EFAILED;
               }

               result_cross_fade_lib = audio_cross_fade_process(&(me->cross_fade_lib[ch]),
                                                                (int8 *)out_ptr[ch],
                                                                cross_fade_in_ptrs,
                                                                (uint32)num_samples);
               if (CROSS_FADE_SUCCESS != result_cross_fade_lib)
               {
                  MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : cross fade processing failed");
                  return CAPI_EFAILED;
               }

               // get the cross fade flag again to see if it is done or not
               result_cross_fade_lib = audio_cross_fade_get_param(&(me->cross_fade_lib[ch]),
                                                                  CROSS_FADE_PARAM_MODE,
                                                                  (int8 *)&(me->cross_fade_flag[ch]),
                                                                  (uint32)sizeof(me->cross_fade_flag[ch]),
                                                                  (uint32 *)&param_size);
               if (CROSS_FADE_SUCCESS != result_cross_fade_lib)
               {
                  MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : cross fade processing failed");
                  return CAPI_EFAILED;
               }

               if (0 == me->cross_fade_flag[ch])
               {
                  // cross fading is done, release the old msiir filters
                  posal_memory_free(me->msiir_lib[ch].mem_ptr);

                  me->msiir_lib[ch].mem_ptr     = me->msiir_new_lib[ch].mem_ptr;
                  me->msiir_new_lib[ch].mem_ptr = NULL;
               }

            } // if (1==me->cross_fade_flag[ch])
            else
            {
               // should not reach here, just to be safe
               MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : cross fade already done but msiir filters are not released!");
               return CAPI_EFAILED;
            }

         } // if (NULL != me->msiir_new_lib)

      } // if (me->enable_flag)
      else
      {
         // copy through if the filter is disabled
         // se actual data length in place of num samples
         memscpy(out_ptr[ch], (num_samples << shift_factor), inp_ptr[ch], (num_samples << shift_factor));
      }

      output[0]->buf_ptr[ch].actual_data_len = (num_samples << shift_factor);
      input[0]->buf_ptr[ch].actual_data_len  = (num_samples << shift_factor);
   }
   me->is_first_frame = FALSE;

   output[0]->flags = input[0]->flags;
   if (input[0]->flags.is_timestamp_valid)
   {
      output[0]->timestamp = input[0]->timestamp - me->delay;
   }

   return CAPI_EOK;
}

/*------------------------------------------------------------------------
  Function name: capi_multistageiir_end
  Returns the module to the uninitialized state and frees any memory
  that was allocated by it.
  @param[in,out] _pif  Pointer to the module object.
  @return        Indication of success or failure.
 * ----------------------------------------------------------------------*/
static capi_err_t capi_multistageiir_end(capi_t *_pif)
{
   if (NULL == _pif)
   {
      MSIIR_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "CAPI MSIIR : End received bad pointer, 0x%p", _pif);
      return CAPI_EBADPARAM;
   }

   capi_multistageiir_t *me = (capi_multistageiir_t *)(_pif);

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

   if (NULL != me->per_chan_msiir_cfg_max)
   {
      posal_memory_free(me->per_chan_msiir_cfg_max);
      me->per_chan_msiir_cfg_max = NULL;
   }

   if (NULL != me->pregain_params.params_ptr.data_ptr)
   {
      posal_memory_free(me->pregain_params.params_ptr.data_ptr);
      me->pregain_params.params_ptr.data_ptr = NULL;
   }

   if (NULL != me->enable_params.params_ptr.data_ptr)
   {
      posal_memory_free(me->enable_params.params_ptr.data_ptr);
      me->enable_params.params_ptr.data_ptr = NULL;
   }

   if (NULL != me->config_params.params_ptr.data_ptr)
   {
      posal_memory_free(me->config_params.params_ptr.data_ptr);
      me->config_params.params_ptr.data_ptr = NULL;
   }

   me->vtbl                                                = NULL;
   me->msiir_static_vars.max_stages                        = 0;
   me->per_chan_mem_req.mem_size                           = 0;
   me->per_chan_cross_fade_mem_req.cross_fade_lib_mem_size = 0;

   MSIIR_MSG(me->miid, DBG_HIGH_PRIO, "CAPI MSIIR : End done");
   return CAPI_EOK;
}

/*------------------------------------------------------------------------
  Function name: capi_multistageiir_set_param
  Sets either a parameter value or a parameter structure containing multiple
  parameters. In the event of a failure, the appropriate error code is
  returned.
 * -----------------------------------------------------------------------*/
static capi_err_t capi_multistageiir_set_param(capi_t *                _pif,
                                                     uint32_t                   param_id,
                                                     const capi_port_info_t *port_info_ptr,
                                                     capi_buf_t *            params_ptr)
{
   if (NULL == _pif || NULL == params_ptr)
   {
      MSIIR_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "CAPI MSIIR : Set param received bad pointer, 0x%p, 0x%p", _pif, params_ptr);
      return CAPI_EBADPARAM;
   }

   capi_err_t result              = CAPI_EOK;
   bool_t        cache_param_pending = FALSE;

   capi_multistageiir_t *me = (capi_multistageiir_t *)(_pif);

   switch (param_id)
   {
      case PARAM_ID_MODULE_ENABLE:
      case FWK_EXTN_PARAM_ID_CONTAINER_FRAME_DURATION:
         break;
      case PARAM_ID_MSIIR_TUNING_FILTER_ENABLE:
      case PARAM_ID_MSIIR_TUNING_FILTER_PREGAIN:
      case PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS:
      {
         if((VERSION_V2 == me->cfg_version) || (TRUE == me->higher_channel_map_present))
         {
            MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : SetParam 0x%lx failed as V2 config is already configured for the module. "
                   "Cannot set both V1 and V2 configs simultaneously for the module OR higher than 63 channel map present in IMF (0/1): %lu",
                   param_id,
                   me->higher_channel_map_present);
            return CAPI_EBADPARAM;
         }
         break;
      }
      break;
      case PARAM_ID_MSIIR_TUNING_FILTER_ENABLE_V2:
      case PARAM_ID_MSIIR_TUNING_FILTER_PREGAIN_V2:
      case PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS_V2:
      {
         if(VERSION_V1 == me->cfg_version)
         {
            MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : SetParam 0x%lx failed as V1 config is already configured for the module. "
                   "Cannot set both V1 and V2 configs simultaneously for the module",param_id);
            return CAPI_EBADPARAM;
         }
         break;
      }
      default:
      {
         MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : SetParam, unsupported param ID 0x%x", (int)param_id);
         return CAPI_EUNSUPPORTED;
         break;
      }
   }
   switch (param_id)
   {
      case PARAM_ID_MODULE_ENABLE:
      {
         uint16_t paramSize = (uint16_t)params_ptr->actual_data_len;
         if (paramSize >= sizeof(param_id_module_enable_t))
         {
            param_id_module_enable_t *enable_ptr = (param_id_module_enable_t *)(params_ptr->data_ptr);
            me->enable                           = enable_ptr->enable;
            MSIIR_MSG(me->miid, DBG_HIGH_PRIO, "CAPI MSIIR : Set enable: %lu", enable_ptr->enable);
            if (me->media_fmt_received)
            {
                result = capi_msiir_raise_process_check_event(me);
            }
         }
         else
         {
            MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Enable/Disable, Bad param size %hu", paramSize);
            result = CAPI_ENEEDMORE;
         }
         break;
      }

      case PARAM_ID_MSIIR_TUNING_FILTER_ENABLE:
      {
         cache_param_pending = TRUE;
         if (me->media_fmt_received)
         {
            result = capi_msiir_set_enable_disable_per_channel(me, params_ptr);
            if (CAPI_FAILED(result))
            {
               cache_param_pending = FALSE;
            }
         }
         break;
      }
      case PARAM_ID_MSIIR_TUNING_FILTER_ENABLE_V2:
      {
         cache_param_pending = TRUE;
         if (me->media_fmt_received)
         {
            result = capi_msiir_set_enable_disable_per_channel_v2(me, params_ptr, param_id);
            if (CAPI_FAILED(result))
            {
               cache_param_pending = FALSE;
            }
         }
         break;
      }
      case PARAM_ID_MSIIR_TUNING_FILTER_PREGAIN:
      {
         cache_param_pending = TRUE;
         if (me->media_fmt_received)
         {
            result = capi_msiir_set_pregain_per_channel(me, params_ptr);
            if (CAPI_FAILED(result))
            {
               cache_param_pending = FALSE;
            }
         }
         break;
      }
      case PARAM_ID_MSIIR_TUNING_FILTER_PREGAIN_V2:
      {
         cache_param_pending = TRUE;
         if (me->media_fmt_received)
         {
            result = capi_msiir_set_pregain_per_channel_v2(me, params_ptr, param_id);
            if (CAPI_FAILED(result))
            {
               cache_param_pending = FALSE;
            }
         }
         break;
      }

      case PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS:
      {
         cache_param_pending = TRUE;
         if (me->media_fmt_received)
         {
            result = capi_msiir_set_config_per_channel(me, params_ptr);
            if (CAPI_FAILED(result))
            {
               cache_param_pending = FALSE;
            }
         }
         break;
      }
      case PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS_V2:
      {
         cache_param_pending = TRUE;
         if (me->media_fmt_received)
         {
            result = capi_msiir_set_config_per_channel_v2(me, params_ptr, param_id);
            if (CAPI_FAILED(result))
            {
               cache_param_pending = FALSE;
            }
         }
         break;
      }
      case FWK_EXTN_PARAM_ID_CONTAINER_FRAME_DURATION:
      {
         if (params_ptr->actual_data_len < sizeof(fwk_extn_param_id_container_frame_duration_t))
         {
            MSIIR_MSG(me->miid, DBG_ERROR_PRIO,
                   "CAPI MSIIR : Invalid payload size for param_id=0x%lx actual_data_len=%lu  ",
                   param_id,
                   params_ptr->actual_data_len);
            result |= CAPI_ENEEDMORE;
            break;
         }

         fwk_extn_param_id_container_frame_duration_t *cfg_ptr =
            (fwk_extn_param_id_container_frame_duration_t *)params_ptr->data_ptr;

         if (0 == cfg_ptr->duration_us)
         {
            MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR :Frame duration sent as 0ms. Ignoring.");
            return result;
         }

         if (me->cntr_frame_size_us != cfg_ptr->duration_us)
         {
            me->cntr_frame_size_us = cfg_ptr->duration_us;
            MSIIR_MSG(me->miid, DBG_HIGH_PRIO, "CAPI MSIIR : Container frame size duration = %lu is set.", me->cntr_frame_size_us);
            if (CAPI_EOK == result)
            {
               result = capi_msiir_check_raise_kpps_event(me, capi_msiir_get_kpps(me));
            }
         }

         return result;
      }

      default:
      {
         MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : SetParam, unsupported param ID 0x%x", (int)param_id);
         result = CAPI_EUNSUPPORTED;
         break;
      }
   }

   if ((CAPI_EOK == result) && (me->start_cross_fade))
   {
      result = capi_msiir_start_cross_fade(me);
   }
   if (cache_param_pending)
   {
      if (CAPI_FAILED(capi_msiir_cache_params(me, params_ptr, param_id)))
      {
         MSIIR_MSG(me->miid, DBG_HIGH_PRIO, "CAPI MSIIR : SetParam, Failed to cache param ID 0x%x", (int)param_id);
      }
   }
   if (CAPI_EOK == result)
   {
      result = capi_msiir_check_raise_kpps_event(me, capi_msiir_get_kpps(me));
   }
   return result;
}

/*------------------------------------------------------------------------
  Function name: capi_multistageiir_get_param
  Gets either a parameter value or a parameter structure containing multiple
  parameters. In the event of a failure, the appropriate error code is
  returned.
 * -----------------------------------------------------------------------*/
static capi_err_t capi_multistageiir_get_param(capi_t *                _pif,
                                                     uint32_t                   param_id,
                                                     const capi_port_info_t *port_info_ptr,
                                                     capi_buf_t *            params_ptr)
{
   if (NULL == _pif || NULL == params_ptr)
   {
      MSIIR_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "CAPI MSIIR : Get param received bad pointer, 0x%p, 0x%p", _pif, params_ptr);
      return CAPI_EBADPARAM;
   }

   capi_err_t            result = CAPI_EOK;
   capi_multistageiir_t *me     = (capi_multistageiir_t *)(_pif);

   switch (param_id)
   {
      case PARAM_ID_MODULE_ENABLE:
         break;
      case PARAM_ID_MSIIR_TUNING_FILTER_ENABLE:
      case PARAM_ID_MSIIR_TUNING_FILTER_PREGAIN:
      case PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS:
      {
         if((VERSION_V2 == me->cfg_version) || (TRUE == me->higher_channel_map_present))
         {
            MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : GetParam 0x%x failed as V2 config is already configured for the module. "
                   "Cannot perform both V1 and V2 operations simultaneously for the module OR higher than 63 channel map present in IMF (0/1): %lu",
                   (int)param_id,
                   me->higher_channel_map_present);
            return CAPI_EBADPARAM;
         }
         break;
      }
      break;
      case PARAM_ID_MSIIR_TUNING_FILTER_ENABLE_V2:
      case PARAM_ID_MSIIR_TUNING_FILTER_PREGAIN_V2:
      case PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS_V2:
      {
         if(VERSION_V1 == me->cfg_version)
         {
            MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : GetParam 0x%x failed as V1 config is already configured for the module. "
                   "Cannot perform both V1 and V2 operations simultaneously for the module",(int)param_id);
            return CAPI_EBADPARAM;
         }
         break;
      }
      default:
      {
         MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : GetParam, unsupported param ID 0x%x", (int)param_id);
         return CAPI_EUNSUPPORTED;
         break;
      }
   }
   switch (param_id)
   {
      case PARAM_ID_MODULE_ENABLE:
      {
         const uint32_t payloadSize = (uint32_t)(params_ptr->max_data_len);
         if (payloadSize >= sizeof(param_id_module_enable_t))
         {
            param_id_module_enable_t *pMSiirEnable = (param_id_module_enable_t *)(params_ptr->data_ptr);
            pMSiirEnable->enable                   = me->enable;
            params_ptr->actual_data_len            = (uint32_t)sizeof(param_id_module_enable_t);
         }
         else
         {
            MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Get Enable Param, Bad payload size %lu", payloadSize);
            result = CAPI_ENEEDMORE;
         }
         break;
      }

      case PARAM_ID_MSIIR_TUNING_FILTER_ENABLE:
      {
         result = capi_msiir_get_enable_disable_per_channel(me, params_ptr);
         break;
      }

      case PARAM_ID_MSIIR_TUNING_FILTER_PREGAIN:
      {
         result = capi_msiir_get_pregain_per_channel(me, params_ptr);
         break;
      }

      case PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS:
      {
         result = capi_msiir_get_config_per_channel(me, params_ptr);
         break;
      }

      case PARAM_ID_MSIIR_TUNING_FILTER_ENABLE_V2:
      {
         result = capi_msiir_get_enable_disable_per_channel_v2(me, params_ptr);
         break;
      }

      case PARAM_ID_MSIIR_TUNING_FILTER_PREGAIN_V2:
      {
         result = capi_msiir_get_pregain_per_channel_v2(me, params_ptr);
         break;
      }

      case PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS_V2:
      {
         result = capi_msiir_get_config_per_channel_v2(me, params_ptr);
         break;
      }

      default:
      {
         MSIIR_MSG(me->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Get unsupported param ID 0x%x", (int)param_id);
         result = CAPI_EBADPARAM;
         break;
      }
   }
   return result;
}

/*-------------------------------------------------------------------------
  Function name: capi_multistageiir_set_properties
  Sets a list of property values.
  @param[in,out] _pif        Pointer to the module object.
  @param[in]     props_ptr   Contains the property values to be set.
  @return        Indication of success or failure.
 *-------------------------------------------------------------------------*/
static capi_err_t capi_multistageiir_set_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   if (NULL == _pif)
      return CAPI_EBADPARAM;

   capi_multistageiir_t *me_ptr = (capi_multistageiir_t *)_pif;

   return capi_msiir_process_set_properties(me_ptr, props_ptr);
}

/*--------------------------------------------------------------------------
  Function name: capi_multistageiir_get_properties
  Gets a list of property values.
  @param[in,out] _pif       Pointer to the module object.
  @param[out]    props_ptr  Contains the empty structures that must be
  filled with the appropriate property values
  based on the property ids provided.
  @return        Indication of success or failure.
 *-------------------------------------------------------------------------*/
static capi_err_t capi_multistageiir_get_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   if (NULL == _pif)
      return CAPI_EBADPARAM;

   capi_multistageiir_t *me_ptr = (capi_multistageiir_t *)_pif;

   return capi_msiir_process_get_properties(me_ptr, props_ptr);
}

/*--------------------------------------------------------------------------
  Function name: capi_multistageiir_init
  Instantiate MSIIR module
  @param[in,out] _pif                Pointer to the module object.
  @param[in]     init_set_properties Properties set by the service to
  be used while init().
  @return       Indication of success or failure.
 *-------------------------------------------------------------------------*/
capi_err_t capi_multistageiir_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   capi_err_t result = CAPI_EOK;

   if (NULL == _pif)
      return CAPI_EBADPARAM;

   int8_t *                 ptr    = (int8_t *)_pif;
   capi_multistageiir_t *me_ptr = (capi_multistageiir_t *)ptr;

   memset(me_ptr, 0, sizeof(capi_multistageiir_t));

   capi_msiir_init_media_fmt(me_ptr);

   // setup the channel index arrays
   // change the size to sizeof(channel_map_to_index)
   memset(&(me_ptr->channel_map_to_index), -1, sizeof(me_ptr->channel_map_to_index));

   me_ptr->msiir_static_vars.data_width = me_ptr->media_fmt[0].format.bits_per_sample;
   me_ptr->msiir_static_vars.max_stages = MSIIR_MAX_STAGES;

   me_ptr->per_chan_cross_fade_mem_req.cross_fade_lib_mem_size = 0;

   me_ptr->is_first_frame     = TRUE;
   me_ptr->media_fmt_received = FALSE;
   me_ptr->start_cross_fade   = FALSE;
   me_ptr->enable             = TRUE;

   me_ptr->vtbl                   = &capi_msiir_vtbl;
   me_ptr->kpps                   = 0;
   me_ptr->delay                  = 0;
   me_ptr->bw                     = 0x7FFFFFFF;
   me_ptr->num_channels_allocated = 0;
   me_ptr->cntr_frame_size_us     = DEFAULT_FRAME_SIZE_IN_US;

   me_ptr->cfg_version = DEFAULT;
   /* init_set_properties contains OUT_BITS_PER_SAMPLE,
    * EVENT_CALLBACK_INFO and PORT_INFO */
   if (NULL != init_set_properties)
   {
      result = capi_msiir_process_set_properties(me_ptr, init_set_properties);
      result ^= (result & CAPI_EUNSUPPORTED);
   }
   MSIIR_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI MSIIR : Init Success!");

   return result;
}
/*--------------------------------------------------------------------------
  Function name: capi_multistageiir_get_static_properties
  Queries the static properties of MSIIR
  @param[in]  init_set_properties The same properties that will be sent
  in the call to the init() function.
  @param[out] static_properties   Pointer to the structure that the
  module must fill with the appropriate
  values based on the property id.
  @return     Indication of success or failure.
 *-------------------------------------------------------------------------*/
capi_err_t capi_multistageiir_get_static_properties(capi_proplist_t *init_set_properties,
                                                          capi_proplist_t *static_properties)
{
   capi_err_t result = CAPI_EFAILED;

   if (NULL != init_set_properties)
   {
      MSIIR_MSG(MIID_UNKNOWN, DBG_HIGH_PRIO, "CAPI MSIIR: get static prop ignoring init_set_properties!");
   }

   if (NULL != static_properties)
   {
      result = capi_msiir_process_get_properties((capi_multistageiir_t *)NULL, static_properties);
   }
   else
   {
      MSIIR_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "CAPI MSIIR: get static prop, Bad ptrs: %p", static_properties);
   }
   return result;
}
