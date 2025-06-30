/* =========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 * @file capi_bassboost.cpp
 *
 * C source file to implement the Audio Post Processor Interface for
 * Acoustic Bass Enhancement
 * (library version:
 * AUDIO_SYSTEM_BASSBOOST_UNIT_TEST_RELEASE_1.2_3.1.0_08.05.2016)
 */

#define BASSBOOST_MAX_STRENGTH 1000
#define BASSBOOST_MIN_STRENGTH 0
#include "capi_bassboost_utils.h"

static capi_err_t capi_bassboost_process(capi_t *            _pif,
                                               capi_stream_data_t *input[],
                                               capi_stream_data_t *output[]);

static capi_err_t capi_bassboost_end(capi_t *_pif);

static capi_err_t capi_bassboost_set_param(capi_t *                _pif,
                                                 uint32_t                   param_id,
                                                 const capi_port_info_t *port_info_ptr,
                                                 capi_buf_t *            params_ptr);

static capi_err_t capi_bassboost_get_param(capi_t *                _pif,
                                                 uint32_t                   param_id,
                                                 const capi_port_info_t *port_info_ptr,
                                                 capi_buf_t *            params_ptr);

static capi_err_t capi_bassboost_set_properties(capi_t *_pif, capi_proplist_t *props_ptr);

static capi_err_t capi_bassboost_get_properties(capi_t *_pif, capi_proplist_t *props_ptr);

static capi_vtbl_t vtbl = { capi_bassboost_process,        capi_bassboost_end,
                               capi_bassboost_set_param,      capi_bassboost_get_param,
                               capi_bassboost_set_properties, capi_bassboost_get_properties };

/* -------------------------------------------------------------------------
 * Function name: capi_bassboost_get_static_properties
 * Capi_v2 BassBoost function to get the static properties
 * -------------------------------------------------------------------------*/
capi_err_t capi_bassboost_get_static_properties(capi_proplist_t *init_set_properties,
                                                      capi_proplist_t *static_properties)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL != static_properties)
   {
      capi_result = capi_bassboost_process_get_properties((capi_bassboost_t *)NULL, static_properties);
      if (CAPI_FAILED(capi_result))
      {
         BASSBOOST_MSG(MIID_UNKNOWN, DBG_HIGH_PRIO, "CAPI BassBoost: get static properties failed!");
         return capi_result;
      }
   }
   else
   {
      BASSBOOST_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "CAPI BassBoost: Get static properties received bad pointer, 0x%p", static_properties);
   }

   return capi_result;
}

/*------------------------------------------------------------------------
 Function name: capi_bassboost_init
 Initialize the CAPIv2 BassBosst library. This function can allocate memory.
 * -----------------------------------------------------------------------*/

capi_err_t capi_bassboost_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == _pif || NULL == init_set_properties)
   {
      BASSBOOST_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "CAPI BassBoost: Init received bad pointer, 0x%p, 0x%p", _pif, init_set_properties);

      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_bassboost_t *me_ptr = (capi_bassboost_t *)_pif;

   memset(me_ptr, 0, sizeof(capi_bassboost_t));

   me_ptr->lib_instance.mem_ptr = NULL;
   me_ptr->vtbl.vtbl_ptr        = &vtbl;
   capi_bassboost_init_media_fmt(me_ptr);
   me_ptr->events_config.kpps        = 0;
   me_ptr->events_config.code_bw     = 0;
   me_ptr->events_config.data_bw     = BASS_BOOST_BY_PASS_BW;
   me_ptr->events_config.delay_in_us = 0;
   me_ptr->events_config.enable      = TRUE;

   if (NULL != init_set_properties)
   {
      capi_result |= capi_bassboost_process_set_properties(me_ptr, init_set_properties);
      capi_result ^= (capi_result & CAPI_EUNSUPPORTED);
      if (CAPI_FAILED(capi_result))
      {
         BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI BassBoost:  Init Set Property Failed");
         return capi_result;
      }
   }
   // capi_bassboost_raise_event(me_ptr);
   BASSBOOST_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI BassBoost: Initialization completed !!");
   return capi_result;
}

/* -------------------------------------------------------------------------
 * Function name: capi_bassboost_process
 * BassBoost Data Process function to process an input buffer
 * and generates an output buffer.
 * -------------------------------------------------------------------------*/
static capi_err_t capi_bassboost_process(capi_t *            _pif,
                                               capi_stream_data_t *input[],
                                               capi_stream_data_t *output[])
{
   BASSBOOST_RESULT lib_result     = BASSBOOST_SUCCESS;
   capi_err_t    capi_result = CAPI_EOK;

   capi_bassboost_t *me_ptr = (capi_bassboost_t *)_pif;

   POSAL_ASSERT(me_ptr);
   POSAL_ASSERT(input[0]);
   POSAL_ASSERT(output[0]);
   POSAL_ASSERT(me_ptr->lib_static_vars.num_chs <= (int32)(input[0]->bufs_num));
   POSAL_ASSERT(me_ptr->lib_static_vars.num_chs <= (int32)(output[0]->bufs_num));

   void *inp_ptrs[BASS_BOOST_MAX_CHANNELS];
   void *out_ptrs[BASS_BOOST_MAX_CHANNELS];

   int32 byte_sample_convert = (16 == me_ptr->lib_static_vars.data_width) ? 1 : 2;
   // one port only, one input and one output
   uint32_t port = 0;

   if (me_ptr->is_disabled == TRUE)
   {
	  me_ptr->is_disabled  = FALSE;
	  for(uint32_t inch = 0; inch < input[0]->bufs_num; inch++)
	  {
		  input[0]->buf_ptr[inch].actual_data_len  = 0;
	  }
	  for(uint32_t outch = 0; outch < output[0]->bufs_num; outch++)
	  {
		  output[0]->buf_ptr[outch].actual_data_len = 0;
	  }
	  capi_bassboost_update_raise_event(me_ptr);
	  return CAPI_EOK;
   }

#ifdef DO_BASSBOOST_PROFILING
   me_ptr->kpps_profile_data.start_cycles = q6sim_read_cycles();
#endif

   int32_t input_num_samples      = input[port]->buf_ptr[0].actual_data_len >> (byte_sample_convert);
   int32_t output_num_samples     = output[port]->buf_ptr[0].max_data_len >> (byte_sample_convert);
   int32_t num_samples_to_process = s32_min_s32_s32(input_num_samples, output_num_samples);
   int32_t samples_to_process     = num_samples_to_process;
   int32_t samples_processed      = 0;
   while (samples_to_process > 0)
   {
      int32_t samples = s32_min_s32_s32(samples_to_process, me_ptr->lib_static_vars.max_block_size);

      for (int32_t ch = 0; ch < me_ptr->lib_static_vars.num_chs; ch++)
      {
         inp_ptrs[ch] = (void *)(input[port]->buf_ptr[ch].data_ptr + (samples_processed << byte_sample_convert));
         out_ptrs[ch] = (void *)(output[port]->buf_ptr[ch].data_ptr + (samples_processed << byte_sample_convert));
      }

      lib_result = bassboost_process(&(me_ptr->lib_instance), out_ptrs, inp_ptrs, samples);
      if (BASSBOOST_SUCCESS != lib_result)
      {
         BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI BassBoost: library process failed");
         CAPI_SET_ERROR(capi_result, CAPI_EFAILED);
         return capi_result;
      }
      samples_processed += samples;
      samples_to_process -= samples;
   }

   for (int32_t ch = 0; ch < me_ptr->lib_static_vars.num_chs; ch++)
   {
      output[port]->buf_ptr[ch].actual_data_len = (num_samples_to_process << byte_sample_convert);
      input[port]->buf_ptr[ch].actual_data_len  = (num_samples_to_process << byte_sample_convert);
   }
   /* check if x-fading is done to update the KPPS when BB is disabled. */
   if (0 == me_ptr->lib_config.enable)
   {
      bassboost_crossfade_t x_fade_flag     = 0;
      uint32_t              lib_actual_size = 0;

      lib_result = bassboost_get_param(&(me_ptr->lib_instance),
                                       BASSBOOST_PARAM_CROSSFADE_FLAG,
                                       (void *)&x_fade_flag,
                                       (uint32)sizeof(x_fade_flag),
                                       (uint32 *)&lib_actual_size);

      if ((BASSBOOST_SUCCESS != lib_result) || (0 == lib_actual_size))
      {
         BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI BassBoost: Get of xfade_flag failed");
         CAPI_SET_ERROR(capi_result, CAPI_EFAILED);
         return capi_result;
      }
      if (0 == x_fade_flag)
      {
         me_ptr->events_config.code_bw = 0;
         if ((me_ptr->lib_static_vars.sample_rate > 48000) || (me_ptr->lib_static_vars.num_chs > 2))
         {
            me_ptr->events_config.data_bw = BASS_BOOST_BY_PASS_BW;
         }
         else
         {
            me_ptr->events_config.data_bw = BASS_BOOST_BY_PASS_BW_HIGH;
         }
         me_ptr->is_disabled = TRUE;

         //        capi_bassboost_update_raise_event(me_ptr);
      }
   }

   output[0]->flags = input[0]->flags;
   if (input[0]->flags.is_timestamp_valid)
   {
      output[0]->timestamp = input[0]->timestamp - me_ptr->events_config.delay_in_us;
   }

#ifdef DO_BASSBOOST_PROFILING
   me_ptr->kpps_profile_data.end_cycles = q6sim_read_cycles();
   capi_bassboost_profiling(me_ptr, num_samples_to_process);
#endif
   return capi_result;
}

/* -------------------------------------------------------------------------
 * Function name: capi_bassboost_end
 * Bass-boost close functions, returns the library to the uninitialized
 * state and frees all the memory that was allocated. This function also
 * frees the virtual function table.
 * -------------------------------------------------------------------------*/
static capi_err_t capi_bassboost_end(capi_t *_pif)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif)
   {
      BASSBOOST_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "CAPI BassBoost: End received bad pointer, 0x%p", _pif);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_bassboost_t *me_ptr = (capi_bassboost_t *)_pif;
   uint32_t temp_miid = me_ptr->miid;
#ifdef DO_BASSBOOST_PROFILING
   capi_bassboost_print_kpps(me_ptr);
#endif
   if (NULL != me_ptr->lib_instance.mem_ptr)
   {
      posal_memory_free(me_ptr->lib_instance.mem_ptr);
      me_ptr->lib_instance.mem_ptr = NULL;
   }

   me_ptr->lib_mem_req.mem_size = 0;
   me_ptr->vtbl.vtbl_ptr        = NULL;

   BASSBOOST_MSG(temp_miid, DBG_HIGH_PRIO, "CAPI BassBoost: End done");
   return capi_result;
}

/* -------------------------------------------------------------------------
 * Function name: capi_bassboost_set_param
 * Sets either a parameter value or a parameter structure containing
 * multiple parameters. In the event of a failure, the appropriate error
 * code is returned.
 * -------------------------------------------------------------------------*/
static capi_err_t capi_bassboost_set_param(capi_t *                _pif,
                                                 uint32_t                   param_id,
                                                 const capi_port_info_t *port_info_ptr,
                                                 capi_buf_t *            params_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif || NULL == params_ptr)
   {
      BASSBOOST_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "CAPI BassBoost: set param received bad pointer, 0x%p, 0x%p", _pif, params_ptr);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }
   capi_bassboost_t *me_ptr            = (capi_bassboost_t *)(_pif);
   int8_t *             param_payload_ptr = (int8_t *)(params_ptr->data_ptr);
   uint32_t             param_size        = params_ptr->actual_data_len;

   switch (param_id)
   {
      case PARAM_ID_MODULE_ENABLE:
      {
         if (param_size >= sizeof(param_id_module_enable_t))
         {
            param_id_module_enable_t *enable_ptr = (param_id_module_enable_t *)param_payload_ptr;
            me_ptr->lib_config.enable            = (0 == enable_ptr->enable) ? 0 : 1;
            me_ptr->lib_config.is_enable_set     = TRUE;

            capi_result = capi_bassboost_check_set_param(me_ptr,
                                                               BASSBOOST_PARAM_ENABLE,
                                                               (void *)&(me_ptr->lib_config.enable),
                                                               (uint32_t)sizeof(me_ptr->lib_config.enable),
                                                               (uint32_t)sizeof(bassboost_enable_t));
            if (CAPI_SUCCEEDED(capi_result))
            {
               BASSBOOST_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI BassBoost: Set enable: %ld", me_ptr->lib_config.enable);
               if (me_ptr->lib_config.enable && me_ptr->lib_instance.mem_ptr != NULL)
               {
                  capi_bassboost_update_raise_event(me_ptr);
               }
            }
         }
      }
      break;

      case PARAM_ID_BASS_BOOST_MODE:
      {
         bass_boost_mode_t *mode_ptr    = (bass_boost_mode_t *)param_payload_ptr;
         me_ptr->lib_config.mode        = mode_ptr->bass_boost_mode;
         me_ptr->lib_config.is_mode_set = TRUE;
         bassboost_mode_t mode          = (bassboost_mode_t)(mode_ptr->bass_boost_mode);
         capi_result                 = capi_bassboost_check_set_param(me_ptr,
                                                            BASSBOOST_PARAM_MODE,
                                                            (void *)&(mode),
                                                            (uint32_t)sizeof(mode),
                                                            (uint32_t)sizeof(bassboost_mode_t));
         if (CAPI_SUCCEEDED(capi_result))
         {
            BASSBOOST_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI BassBoost: set mode %lu", me_ptr->lib_config.mode);
         }
      }
      break;

      case PARAM_ID_BASS_BOOST_STRENGTH:
      {

         bass_boost_strength_t *strength_ptr = (bass_boost_strength_t *)param_payload_ptr;
         me_ptr->lib_config.strength         = strength_ptr->strength;
         me_ptr->lib_config.is_strength_set  = TRUE;
         bassboost_strength_t strength       = (bassboost_strength_t)(strength_ptr->strength);

         if (strength >= BASSBOOST_MIN_STRENGTH && strength <= BASSBOOST_MAX_STRENGTH)
         {
            capi_result = capi_bassboost_check_set_param(me_ptr,
                                                               BASSBOOST_PARAM_STRENGTH,
                                                               (void *)&(strength),
                                                               (uint32_t)sizeof(strength),
                                                               (uint32_t)sizeof(bassboost_strength_t));
            if (CAPI_SUCCEEDED(capi_result))
            {
               BASSBOOST_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI BassBoost:  set strength %ld", me_ptr->lib_config.strength);
            }
         }
         else
         {

            return CAPI_EBADPARAM;
         }
      }
      break;

      default:
      {
         BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI BassBoost: Set unsupported parameter ID %#x", (int)param_id);
         CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      }
   }
   return capi_result;
}

/* -------------------------------------------------------------------------
 * Function name: capi_bassboost_get_param
 * Gets either a parameter value or a parameter structure containing
 * multiple parameters. In the event of a failure, the appropriate error
 * code is returned.
 * -------------------------------------------------------------------------*/
static capi_err_t capi_bassboost_get_param(capi_t *                _pif,
                                                 uint32_t                   param_id,
                                                 const capi_port_info_t *port_info_ptr,
                                                 capi_buf_t *            params_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif || NULL == params_ptr)
   {
      BASSBOOST_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "CAPI BassBoost: Get param received bad pointer, 0x%p, 0x%p", _pif, params_ptr);
      return CAPI_EBADPARAM;
   }

   capi_bassboost_t *me_ptr = (capi_bassboost_t *)_pif;
   switch (param_id)
   {
      case PARAM_ID_MODULE_ENABLE:
      {
         if (params_ptr->max_data_len >= sizeof(param_id_module_enable_t))
         {
            param_id_module_enable_t *enable_ptr = (param_id_module_enable_t *)(params_ptr->data_ptr);

            bassboost_enable_t enable_flag     = 0;
            uint32_t           lib_actual_size = 0;

            capi_result = capi_bassboost_check_get_param(me_ptr,
                                                               BASSBOOST_PARAM_ENABLE,
                                                               (void *)&enable_flag,
                                                               (uint32_t)sizeof(enable_flag),
                                                               (uint32_t)sizeof(bassboost_enable_t),
                                                               &lib_actual_size);
            if ((CAPI_FAILED(capi_result)) || (0 == lib_actual_size))
            {
               params_ptr->actual_data_len = lib_actual_size;
               return capi_result;
            }
            enable_ptr->enable          = (uint32_t)enable_flag;
            params_ptr->actual_data_len = (uint32_t)sizeof(enable_ptr->enable);
         }
         else
         {
            BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI BassBoost: get enable need more memory");
            CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
         }
      }
      break;

      case PARAM_ID_BASS_BOOST_MODE:
      {
         if (params_ptr->max_data_len >= sizeof(bass_boost_mode_t))
         {
            bass_boost_mode_t *mode_ptr = (bass_boost_mode_t *)(params_ptr->data_ptr);

            /* currently the library only supports PHYSICAL_BOOST */
            bassboost_mode_t mode            = PHYSICAL_BOOST;
            uint32_t         lib_actual_size = 0;

            capi_result = capi_bassboost_check_get_param(me_ptr,
                                                               BASSBOOST_PARAM_MODE,
                                                               (void *)&mode,
                                                               (uint32_t)sizeof(mode),
                                                               (uint32_t)sizeof(bassboost_mode_t),
                                                               &lib_actual_size);
            if ((CAPI_FAILED(capi_result)) || (0 == lib_actual_size))
            {
               params_ptr->actual_data_len = lib_actual_size;
               return capi_result;
            }
            mode_ptr->bass_boost_mode   = (uint32_t)mode;
            params_ptr->actual_data_len = (uint32_t)sizeof(mode_ptr->bass_boost_mode);
         }
         else
         {
            BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI BassBoost: get mode need more memory");
            CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
         }
      }
      break;

      case PARAM_ID_BASS_BOOST_STRENGTH:
      {
         if (params_ptr->max_data_len >= sizeof(bass_boost_strength_t))
         {

            bass_boost_strength_t *strength_ptr = (bass_boost_strength_t *)(params_ptr->data_ptr);

            bassboost_strength_t strength        = 1000; // default strength
            uint32_t             lib_actual_size = 0;

            capi_result = capi_bassboost_check_get_param(me_ptr,
                                                               BASSBOOST_PARAM_STRENGTH,
                                                               (void *)&strength,
                                                               (uint32_t)sizeof(strength),
                                                               (uint32_t)sizeof(bassboost_strength_t),
                                                               &lib_actual_size);
            if ((CAPI_FAILED(capi_result)) || (0 == lib_actual_size))
            {
               params_ptr->actual_data_len = lib_actual_size;
               return capi_result;
            }

            strength_ptr->strength      = (uint32_t)strength;
            params_ptr->actual_data_len = (uint32_t)sizeof(strength_ptr->strength);
         }
         else
         {
            BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI BassBoost: get strength needs more memory");
            CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
         }
      }
      break;

      default:
      {
         BASSBOOST_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI BassBoost: Get unsupported param ID %#x", (int)param_id);
         CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      }
   }
   return capi_result;
}

/* -------------------------------------------------------------------------
 * Function name: capi_bassboost_set_properties
 * Function to set the properties of Bassboost module
 * -------------------------------------------------------------------------*/
static capi_err_t capi_bassboost_set_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif)
   {
      BASSBOOST_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "CAPI BassBoost: Set properties received bad pointer, 0x%p", _pif);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_bassboost_t *me_ptr = (capi_bassboost_t *)_pif;

   capi_result |= capi_bassboost_process_set_properties(me_ptr, props_ptr);

   return capi_result;
}

/* -------------------------------------------------------------------------
 * Function name: capi_bassboost_get_properties
 * Function to get the properties of Bass-boost module
 * -------------------------------------------------------------------------*/
static capi_err_t capi_bassboost_get_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif)
   {
      BASSBOOST_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "CAPI BassBoost: Get properties received bad pointer, 0x%p", _pif);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_bassboost_t *me_ptr = (capi_bassboost_t *)_pif;

   capi_result |= capi_bassboost_process_get_properties(me_ptr, props_ptr);

   return capi_result;
}
