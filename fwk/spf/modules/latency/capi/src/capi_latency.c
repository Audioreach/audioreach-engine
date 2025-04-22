/* =========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 * @file capi_latency.cpp
 *
 * C source file to implement the Common Audio Post Processor Interface
 * for Tx/Rx Tuning latency block
 */

#include "capi_latency_utils.h"

static capi_err_t capi_latency_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[]);

static capi_err_t capi_latency_end(capi_t *_pif);

static capi_err_t capi_latency_set_param(capi_t                 *_pif,
                                         uint32_t                param_id,
                                         const capi_port_info_t *port_info_ptr,
                                         capi_buf_t             *params_ptr);

static capi_err_t capi_latency_get_param(capi_t                 *_pif,
                                         uint32_t                param_id,
                                         const capi_port_info_t *port_info_ptr,
                                         capi_buf_t             *params_ptr);

static capi_err_t capi_latency_set_properties(capi_t *_pif, capi_proplist_t *props_ptr);

static capi_err_t capi_latency_get_properties(capi_t *_pif, capi_proplist_t *props_ptr);

static capi_vtbl_t vtbl = { capi_latency_process,        capi_latency_end,
                            capi_latency_set_param,      capi_latency_get_param,
                            capi_latency_set_properties, capi_latency_get_properties };

/* -------------------------------------------------------------------------
 * Function name: capi_latency_get_static_properties
 * Capi_v2 latency function to get the static properties
 * -------------------------------------------------------------------------*/
capi_err_t capi_latency_get_static_properties(capi_proplist_t *init_set_properties, capi_proplist_t *static_properties)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL != static_properties)
   {
      capi_result = capi_latency_process_get_properties((capi_latency_t *)NULL, static_properties);
      if (CAPI_FAILED(capi_result))
      {
         AR_MSG(DBG_ERROR_PRIO, "CAPI latency: get static properties failed!");
         return capi_result;
      }
   }
   else
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI latency: Get static properties received bad pointer, 0x%p", static_properties);
   }

   return capi_result;
}

/*------------------------------------------------------------------------
  Function name: capi_latency_init
  Initialize the CAPI latency Module. This function can allocate memory.
 * -----------------------------------------------------------------------*/

capi_err_t capi_latency_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == _pif || NULL == init_set_properties)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI latency: Init received bad pointer, 0x%p, 0x%p", _pif, init_set_properties);

      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_latency_t *me_ptr = (capi_latency_t *)_pif;

   memset(me_ptr, 0, sizeof(capi_latency_t));

   me_ptr->vtbl.vtbl_ptr                                 = &vtbl;
   me_ptr->cache_delay.num_config                        = 0;
   me_ptr->is_media_fmt_received                         = FALSE;
   me_ptr->cache_delay.cache_delay_per_config            = NULL;
   me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr  = NULL;
   me_ptr->cache_delay_v2.cache_delay_per_config_v2_size = 0;
   me_ptr->state                                         = FIRST_FRAME;
   capi_cmn_init_media_fmt_v2(&me_ptr->media_fmt);
   capi_latency_init_events(me_ptr);
   capi_latency_init_config(me_ptr);

   if (NULL != init_set_properties)
   {
      capi_result |= capi_latency_process_set_properties(me_ptr, init_set_properties);
      capi_result ^= (capi_result & CAPI_EUNSUPPORTED);
      if (CAPI_FAILED(capi_result))
      {
         AR_MSG(DBG_ERROR_PRIO, "CAPI latency:  Initialization Set Property Failed");
         return capi_result;
      }
   }

   capi_latency_raise_event(me_ptr, FALSE);

   AR_MSG(DBG_HIGH_PRIO, "CAPI latency: Initialization completed !!");
   return capi_result;
}

/* -------------------------------------------------------------------------
 * Function name: capi_latency_process
 * latency module Data Process function to process an input buffer
 * and generates an output buffer.
 * -------------------------------------------------------------------------*/
static capi_err_t capi_latency_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[])
{
   capi_err_t      capi_result = CAPI_EOK;
   capi_latency_t *me_ptr      = (capi_latency_t *)(_pif);

   POSAL_ASSERT(me_ptr);
   POSAL_ASSERT(input[0]);
   POSAL_ASSERT(output[0]);
   POSAL_ASSERT(me_ptr->media_fmt.format.num_channels <= input[0]->bufs_num);
   POSAL_ASSERT(me_ptr->media_fmt.format.num_channels <= output[0]->bufs_num);

   uint32_t in_buf_bytes       = input[0]->buf_ptr[0].actual_data_len;
   uint32_t out_buf_bytes      = output[0]->buf_ptr[0].max_data_len;
   uint32_t bytes_to_produce   = (in_buf_bytes > out_buf_bytes) ? out_buf_bytes : in_buf_bytes;
   uint32_t samples_to_produce = 0;
   if (16 == me_ptr->media_fmt.format.bits_per_sample)
   {
      samples_to_produce = bytes_to_produce / 2;
   }
   else if (32 == me_ptr->media_fmt.format.bits_per_sample)
   {
      samples_to_produce = bytes_to_produce / 4;
   }
   else
   {
      POSAL_ASSERT(0);
   }

   if (!me_ptr->is_rt_jitter_correction_enabled || !me_ptr->ts_payload.ts_ptr)
   {
      // state tracking not needed for normal latency module.
      me_ptr->state = STEADY_STATE;
   }
   else if (input[0]->flags.end_of_frame || input[0]->flags.erasure)
   {
      me_ptr->state = EOF_STATE;
   }
   else if (EOF_STATE == me_ptr->state)
   {
      // algo reset after discontinuity, state will also move to the FIRST_FRAME
      capi_latency_algo_reset(me_ptr);
   }

   if ((EOF_STATE != me_ptr->state && STEADY_STATE != me_ptr->state) && me_ptr->is_rt_jitter_correction_enabled &&
       me_ptr->ts_payload.ts_ptr && input[0]->flags.is_timestamp_valid && me_ptr->ts_payload.ts_ptr->is_valid)
   {
      // real time input data already incurred this delay so this can be compensated from the delayline by setting it as
      // a negative delay.
      int64_t rt_rt_jitter_delay = me_ptr->ts_payload.ts_ptr->timestamp - input[0]->timestamp;

      AR_MSG(DBG_HIGH_PRIO, "CAPI latency: incoming RT TS %ld us", (uint32_t)input[0]->timestamp);
      AR_MSG(DBG_HIGH_PRIO, "CAPI latency: outgoing RT TS %ld us", (uint32_t)me_ptr->ts_payload.ts_ptr->timestamp);

      AR_MSG(DBG_HIGH_PRIO,
             "CAPI latency: rt rt jitter delay %ld us, state %lu",
             (int32_t)rt_rt_jitter_delay,
             me_ptr->state);

      if (rt_rt_jitter_delay > 0)
      {
         uint32_t negative_delay_samples =
            capi_cmn_us_to_samples(rt_rt_jitter_delay, me_ptr->media_fmt.format.sampling_rate);

         uint32_t delay_variation = negative_delay_samples > me_ptr->negative_delay_samples
                                       ? negative_delay_samples - me_ptr->negative_delay_samples
                                       : me_ptr->negative_delay_samples - negative_delay_samples;

         //After signal miss handling or US/DS STOP-START, interrupt timing will change and also
         //RT Jitter Delay will change therefore realignment is needed.
         //RT Jitter delay in subsequent frames may also change from the FIRST-FRAME because of stuck ICB.
         //During realignment if delay varies in the subsequent frames then state again moves to the FIRST-FRAME.
         //Only when delay remains constant for the 10 subsequent frames, we go into the STEADY_STATE
         if (delay_variation > 1)
         {
            capi_latency_algo_reset(me_ptr);
            me_ptr->negative_delay_samples = negative_delay_samples;

            AR_MSG(DBG_HIGH_PRIO,
                   "CAPI latency: rt rt jitter delay set in %lu samples",
                   me_ptr->negative_delay_samples);

            capi_latency_raise_delay_event(me_ptr);
         }
      }
      // will continue tracking RT-Jitter for couple of frames.
      // usually there will be multiple discontinuities back to back during upstream signal miss.
      me_ptr->state++;
   }

   for (uint32_t i = 0; i < me_ptr->media_fmt.format.num_channels; i++)
   {
      uint32_t delay_samples = me_ptr->lib_config.mchan_config[i].delay_in_samples > me_ptr->negative_delay_samples
                                  ? me_ptr->lib_config.mchan_config[i].delay_in_samples - me_ptr->negative_delay_samples
                                  : 0;

      capi_delay_delayline_read(output[0]->buf_ptr[i].data_ptr,
                                input[0]->buf_ptr[i].data_ptr,
                                &me_ptr->lib_config.mchan_config[i].delay_line,
                                delay_samples,
                                samples_to_produce);

      capi_delay_delayline_update(&me_ptr->lib_config.mchan_config[i].delay_line,
                                  input[0]->buf_ptr[i].data_ptr,
                                  samples_to_produce);
   }

   for (uint32_t i = 0; i < me_ptr->media_fmt.format.num_channels; i++)
   {
      if (NULL == me_ptr->lib_config.mchan_config)
      {
         memscpy(output[0]->buf_ptr[i].data_ptr,
                 output[0]->buf_ptr[i].max_data_len,
                 input[0]->buf_ptr[i].data_ptr,
                 input[0]->buf_ptr[i].actual_data_len);
      }
      input[0]->buf_ptr[i].actual_data_len = output[0]->buf_ptr[i].actual_data_len = bytes_to_produce;
   }

   // DBG:AR_MSG( DBG_HIGH_PRIO,"CAPI_v2 latency: post process in_size %d, out_size %d",in_size,
   // out_size);

   output[0]->flags = input[0]->flags;
   if (me_ptr->is_rt_jitter_correction_enabled)
   { // no need for output timestamp
      output[0]->flags.is_timestamp_valid = FALSE;
   }
   else if (input[0]->flags.is_timestamp_valid)
   {
      output[0]->timestamp = input[0]->timestamp - me_ptr->events_config.delay_in_us;
   }
   return capi_result;
}

/*------------------------------------------------------------------------
 * Function name: capi_latency_end
 * latency End function, returns the library to the uninitialized
 * state and frees all the memory that was allocated. This function also
 * frees the virtual function table.
 * -----------------------------------------------------------------------*/
static capi_err_t capi_latency_end(capi_t *_pif)
{

   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI latency: End received bad pointer, 0x%p", _pif);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_latency_t *me_ptr = (capi_latency_t *)_pif;

   if (NULL != me_ptr->cache_delay.cache_delay_per_config)
   {
      posal_memory_free(me_ptr->cache_delay.cache_delay_per_config);
      me_ptr->cache_delay.cache_delay_per_config = NULL;
      me_ptr->cache_delay.num_config             = 0;
   }
   if (NULL != me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr)
   {
      posal_memory_free(me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr);
      me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr  = NULL;
      me_ptr->cache_delay_v2.cache_delay_per_config_v2_size = 0;
   }
   capi_delay_destroy_buffer(me_ptr);

   me_ptr->vtbl.vtbl_ptr = NULL;

   AR_MSG(DBG_HIGH_PRIO, "CAPI latency: End done");
   return capi_result;
}

/* -------------------------------------------------------------------------
 * Function name: capi_latency_set_param
 * Sets either a parameter value or a parameter structure containing
 * multiple parameters. In the event of a failure, the appropriate error
 * code is returned.
 * -------------------------------------------------------------------------*/
static capi_err_t capi_latency_set_param(capi_t                 *_pif,
                                         uint32_t                param_id,
                                         const capi_port_info_t *port_info_ptr,
                                         capi_buf_t             *params_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI latency: set param received bad pointer, 0x%p, 0x%p", _pif, params_ptr);
      return CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
   }
   capi_latency_t *me_ptr = (capi_latency_t *)(_pif);
   switch (param_id)
   {
      case PARAM_ID_MODULE_ENABLE:
      case INTF_EXTN_PARAM_ID_STM_TS:
      case PARAM_ID_LATENCY_MODE:
         break;
      case PARAM_ID_LATENCY_CFG:
      {
         if ((VERSION_V2 == me_ptr->cfg_version) || (TRUE == me_ptr->higher_channel_map_present))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "CAPI latency : SetParam 0x%x failed as V2 config is already configured for the module. "
                   "Cannot perform both V1 and V2 operations simultaneously for the module OR higher than 63 channel "
                   "map present in IMF (0/1): %lu",
                   (int)param_id,
                   me_ptr->higher_channel_map_present);
            return CAPI_EBADPARAM;
         }
         break;
      }
      break;
      case PARAM_ID_LATENCY_CFG_V2:
      {
         if (VERSION_V1 == me_ptr->cfg_version)
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "CAPI latency : SetParam 0x%x failed as V1 config is already configured for the module. "
                   "Cannot perform both V1 and V2 operations simultaneously for the module",
                   (int)param_id);
            return CAPI_EBADPARAM;
         }
         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO, "CAPI latency : SetParam, unsupported param ID 0x%x", (int)param_id);
         return CAPI_EUNSUPPORTED;
         break;
      }
   }

   switch (param_id)
   {
      case PARAM_ID_MODULE_ENABLE:
      {
         uint16_t param_size = (uint16_t)params_ptr->actual_data_len;
         if (param_size >= sizeof(param_id_module_enable_t))
         {
            param_id_module_enable_t *enable_ptr = (param_id_module_enable_t *)(params_ptr->data_ptr);
            me_ptr->lib_config.enable            = enable_ptr->enable;
            AR_MSG(DBG_HIGH_PRIO, "CAPI PCM Delay Set Enable Param, %lu", me_ptr->lib_config.enable);
         }
         else
         {
            AR_MSG(DBG_ERROR_PRIO, "CAPI PCM Delay Set Enable Param, Bad param size %hu", param_size);
            CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
         }
         break;
      }
      case PARAM_ID_LATENCY_CFG:
      {
         if (params_ptr->actual_data_len < sizeof(param_id_latency_cfg_t))
         {
            AR_MSG(DBG_ERROR_PRIO, "CAPI Latency Delay CFG, Bad param size %lu", params_ptr->actual_data_len);
            return CAPI_ENEEDMORE;
         }
         param_id_latency_cfg_t *delay_param_ptr = (param_id_latency_cfg_t *)params_ptr->data_ptr;
         me_ptr->cfg_mode                        = delay_param_ptr->cfg_mode;

         AR_MSG(DBG_HIGH_PRIO, "CAPI Latency Delay CFG mode = %lu", me_ptr->cfg_mode);

         if (LATENCY_MODE_GLOBAL == me_ptr->cfg_mode)
         {
            param_id_latency_cfg_t *delay_param_ptr = (param_id_latency_cfg_t *)params_ptr->data_ptr;

            if (delay_param_ptr->global_delay_us > CAPI_LATENCY_MAX_DELAY_US)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "CAPI Latency Global Delay %lu too high, max is %lu",
                      delay_param_ptr->global_delay_us,
                      CAPI_LATENCY_MAX_DELAY_US);
               return CAPI_EBADPARAM;
            }

            if (NULL != me_ptr->cache_delay.cache_delay_per_config)
            {
               posal_memory_free(me_ptr->cache_delay.cache_delay_per_config);
               me_ptr->cache_delay.cache_delay_per_config = NULL;
               me_ptr->cache_delay.num_config             = 0;
            }

            AR_MSG(DBG_HIGH_PRIO, "Capi latency: Configured Global Delay = %lu us", delay_param_ptr->global_delay_us);

            // cache the payload
            me_ptr->cache_delay.num_config = 1;
            me_ptr->cache_delay.cache_delay_per_config =
               (capi_latency_cache_delay_per_config_t *)posal_memory_malloc(sizeof(
                                                                               capi_latency_cache_delay_per_config_t),
                                                                            (POSAL_HEAP_ID)me_ptr->heap_mem.heap_id);
            if (NULL == me_ptr->cache_delay.cache_delay_per_config)
            {
               AR_MSG(DBG_FATAL_PRIO,
                      "CAPI Latency: No memory to store cache_delay_per_config.Requires %lu bytes",
                      sizeof(capi_latency_cache_delay_per_config_t));
               return CAPI_ENOMEMORY;
            }

            me_ptr->cache_delay.cache_delay_per_config[0].channel_mask_lsb = 0xFFFFFFFE;
            me_ptr->cache_delay.cache_delay_per_config[0].channel_mask_msb = 0xFFFFFFFF;
            me_ptr->cache_delay.cache_delay_per_config[0].delay_in_us      = delay_param_ptr->global_delay_us;
         }
         else if (LATENCY_MODE_PER_CH == me_ptr->cfg_mode)
         {
            const uint32_t num_config = delay_param_ptr->num_config;
            if (num_config < 1 || num_config > 63)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "CAPI Latency : Per ch mode: Received incorrect num_config parameter - %lu",
                      num_config);
               return CAPI_EBADPARAM;
            }
            uint8_t *temp_cfg_ptr = (uint8_t *)params_ptr->data_ptr;
            temp_cfg_ptr += sizeof(param_id_latency_cfg_t);

            uint32_t req_payload_size =
               sizeof(param_id_latency_cfg_t) + (sizeof(delay_param_per_ch_cfg_t) * num_config);
            if (params_ptr->actual_data_len < req_payload_size)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "CAPI Latency: per ch Delay SetParam 0x%lx, invalid param size %lu ,required_size %lu",
                      param_id,
                      params_ptr->actual_data_len,
                      req_payload_size);
               return CAPI_EBADPARAM;
            }

            delay_param_per_ch_cfg_t *delay_cfg_ptr = (delay_param_per_ch_cfg_t *)temp_cfg_ptr;

            capi_result |= capi_latency_check_channel_map_delay_cfg(delay_cfg_ptr, num_config);
            if (CAPI_FAILED(capi_result))
            {
               AR_MSG(DBG_HIGH_PRIO, "CAPI Latency : Received incorrect channel map for parameter id 0x%lx", param_id);
               return capi_result;
            }

            for (uint32_t count = 0; count < num_config; count++)
            {
               if (delay_cfg_ptr[count].delay_us > CAPI_LATENCY_MAX_DELAY_US)
               {
                  AR_MSG(DBG_ERROR_PRIO,
                         "CAPI Latency Set Delay %lu too high, max is %lu",
                         delay_cfg_ptr[count].delay_us,
                         CAPI_LATENCY_MAX_DELAY_US);
                  return CAPI_EBADPARAM;
               }
            }

            if (NULL != me_ptr->cache_delay.cache_delay_per_config)
            {
               posal_memory_free(me_ptr->cache_delay.cache_delay_per_config);
               me_ptr->cache_delay.cache_delay_per_config = NULL;
               me_ptr->cache_delay.num_config             = 0;
            }

            // cache the payload
            me_ptr->cache_delay.num_config             = num_config;
            me_ptr->cache_delay.cache_delay_per_config = (capi_latency_cache_delay_per_config_t *)
               posal_memory_malloc(num_config * sizeof(capi_latency_cache_delay_per_config_t),
                                   (POSAL_HEAP_ID)me_ptr->heap_mem.heap_id);
            if (NULL == me_ptr->cache_delay.cache_delay_per_config)
            {
               AR_MSG(DBG_FATAL_PRIO,
                      "CAPI Latency: No memory to store cache_delay_per_config.Requires %lu bytes",
                      num_config * sizeof(capi_latency_cache_delay_per_config_t));
               return CAPI_ENOMEMORY;
            }

            AR_MSG(DBG_HIGH_PRIO, "Capi latency: Received per ch Delay, num_config = %lu", num_config);

            for (uint32_t count = 0; count < num_config; count++)
            {
               me_ptr->cache_delay.cache_delay_per_config[count].channel_mask_lsb =
                  delay_cfg_ptr[count].channel_mask_lsb;
               me_ptr->cache_delay.cache_delay_per_config[count].channel_mask_msb =
                  delay_cfg_ptr[count].channel_mask_msb;
               me_ptr->cache_delay.cache_delay_per_config[count].delay_in_us = delay_cfg_ptr[count].delay_us;
            }
         }
         me_ptr->cfg_version = VERSION_V1;
         if (me_ptr->is_media_fmt_received == FALSE)
         {
            return capi_result;
         }

         capi_delay_delayline_t *old_delay_lines = NULL;
         void                   *old_mem_ptr     = NULL;

         uint32_t *old_delay_in_us =
            (uint32_t *)posal_memory_malloc(me_ptr->media_fmt.format.num_channels * sizeof(uint32_t),
                                            (POSAL_HEAP_ID)me_ptr->heap_mem.heap_id);
         if (NULL == old_delay_in_us)
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "CAPI Latency: No memory to store old delay.Requires %lu bytes",
                   me_ptr->media_fmt.format.num_channels * sizeof(uint32_t));
            CAPI_SET_ERROR(capi_result, CAPI_ENOMEMORY);
            goto __bailout;
         }

         for (uint32_t i = 0; i < me_ptr->media_fmt.format.num_channels; i++)
         {
            old_delay_in_us[i]                             = me_ptr->lib_config.mchan_config[i].delay_in_us;
            me_ptr->lib_config.mchan_config[i].delay_in_us = 0;
         }

         capi_delay_set_delay(me_ptr);

         old_delay_lines = (capi_delay_delayline_t *)posal_memory_malloc(me_ptr->media_fmt.format.num_channels *
                                                                            sizeof(capi_delay_delayline_t),
                                                                         (POSAL_HEAP_ID)me_ptr->heap_mem.heap_id);
         if (NULL == old_delay_lines)
         {
            AR_MSG(DBG_ERROR_PRIO, "CAPI Latency failed to allocate memory for old_delay_lines");
            CAPI_SET_ERROR(capi_result, CAPI_ENOMEMORY);
            goto __bailout;
         }
         memset(old_delay_lines, 0, sizeof(capi_delay_delayline_t));

         for (uint32_t i = 0; i < me_ptr->media_fmt.format.num_channels; i++)
         {
            memscpy(&old_delay_lines[i],
                    sizeof(capi_delay_delayline_t),
                    &me_ptr->lib_config.mchan_config[i].delay_line,
                    sizeof(capi_delay_delayline_t));
         }
         old_mem_ptr = me_ptr->lib_config.mem_ptr;

         capi_result |= capi_delay_create_buffer(me_ptr, old_delay_in_us);
         if (CAPI_SUCCEEDED(capi_result))
         {
            for (uint32_t i = 0; i < me_ptr->media_fmt.format.num_channels; i++)
            {
               if (old_delay_in_us[i] != 0)
               {
                  capi_delay_delayline_copy(&me_ptr->lib_config.mchan_config[i].delay_line, &old_delay_lines[i]);
               }
            }
         }
      __bailout:
         if (NULL != old_delay_lines)
         {
            posal_memory_free(old_delay_lines);
            old_delay_lines = NULL;
         }

         if (NULL != old_mem_ptr)
         {
            posal_memory_free(old_mem_ptr);
            old_mem_ptr = NULL;
         }

         if (NULL != old_delay_in_us)
         {
            posal_memory_free(old_delay_in_us);
            old_delay_in_us = NULL;
         }
         break;
      }
      case PARAM_ID_LATENCY_CFG_V2:
      {
         capi_result = capi_latency_set_config_v2(me_ptr, param_id, params_ptr->actual_data_len, params_ptr->data_ptr);
         if (CAPI_FAILED(capi_result))
         {
            AR_MSG(DBG_ERROR_PRIO, "Latency config V2 SetParam 0x%lx failed.", (int)param_id);
            break;
         }
         break;
      }
      case PARAM_ID_LATENCY_MODE:
      {
         if (params_ptr->actual_data_len < sizeof(param_id_latency_mode_t))
         {

            AR_MSG(DBG_ERROR_PRIO, "CAPI Latency mode, Bad param size %lu", params_ptr->actual_data_len);
            return CAPI_ENEEDMORE;
         }
         param_id_latency_mode_t *payload_ptr = (param_id_latency_mode_t *)params_ptr->data_ptr;
         bool_t                   is_rt_jitter_correction_enabled =
            (payload_ptr->mode == LATENCY_RT_JITTER_CORRECTION_MODE) ? TRUE : FALSE;

         if (is_rt_jitter_correction_enabled != me_ptr->is_rt_jitter_correction_enabled)
         {
            me_ptr->is_rt_jitter_correction_enabled = is_rt_jitter_correction_enabled;

            AR_MSG(DBG_HIGH_PRIO, "CAPI latency: is rt jitter correction mode enabled? %d", is_rt_jitter_correction_enabled);
            // Algo reset to handle change in mode
            capi_latency_algo_reset(me_ptr);
         }

         break;
      }
      case INTF_EXTN_PARAM_ID_STM_TS:
      {
         if (params_ptr->actual_data_len < sizeof(intf_extn_param_id_stm_ts_t))
         {

            AR_MSG(DBG_ERROR_PRIO, "CAPI Latency STM TS, Bad param size %lu", params_ptr->actual_data_len);
            return CAPI_ENEEDMORE;
         }
         intf_extn_param_id_stm_ts_t *payload_ptr = (intf_extn_param_id_stm_ts_t *)params_ptr->data_ptr;
         me_ptr->ts_payload                       = *payload_ptr;

         // Algo reset to handle signal miss in the container.
         capi_latency_algo_reset(me_ptr);
         break;
      }

      default:
      {
         AR_MSG(DBG_ERROR_PRIO, "CAPI latency Set, unsupported param ID 0x%x", (int)param_id);
         CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
         break;
      }
   }

   // raise event for output media format
   capi_result |= capi_latency_raise_event(me_ptr, FALSE);

   return capi_result;
}

/* -------------------------------------------------------------------------
 * Function name: capi_latency_get_param
 * Gets either a parameter value or a parameter structure containing
 * multiple parameters. In the event of a failure, the appropriate error
 * code is returned.
 * -------------------------------------------------------------------------*/
static capi_err_t capi_latency_get_param(capi_t                 *_pif,
                                         uint32_t                param_id,
                                         const capi_port_info_t *port_info_ptr,
                                         capi_buf_t             *params_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI latency: Get param received bad pointer, 0x%p, 0x%p", _pif, params_ptr);
      return CAPI_EBADPARAM;
   }

   capi_latency_t *me_ptr = (capi_latency_t *)_pif;

   switch (param_id)
   {
      case PARAM_ID_MODULE_ENABLE:
      case PARAM_ID_LATENCY_MODE:
         break;
      case PARAM_ID_LATENCY_CFG:
      {
         if ((VERSION_V2 == me_ptr->cfg_version) || (TRUE == me_ptr->higher_channel_map_present))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "CAPI latency : GetParam 0x%x failed as V2 config is already configured for the module. "
                   "Cannot perform both V1 and V2 operations simultaneously for the module OR higher than 63 channel "
                   "map present in IMF (0/1): %lu",
                   (int)param_id,
                   me_ptr->higher_channel_map_present);
            return CAPI_EUNSUPPORTED;
         }
         break;
      }
      break;
      case PARAM_ID_LATENCY_CFG_V2:
      {
         if (VERSION_V1 == me_ptr->cfg_version)
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "CAPI latency : GetParam 0x%x failed as V1 config is already configured for the module. "
                   "Cannot perform both V1 and V2 operations simultaneously for the module",
                   (int)param_id);
            return CAPI_EUNSUPPORTED;
         }
         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO, "CAPI latency : GetParam, unsupported param ID 0x%x", (int)param_id);
         return CAPI_EUNSUPPORTED;
      }
   }

   switch (param_id)
   {
      case PARAM_ID_MODULE_ENABLE:
      {
         if (params_ptr->max_data_len >= sizeof(param_id_module_enable_t))
         {
            param_id_module_enable_t *pPcmDelayEnable = (param_id_module_enable_t *)(params_ptr->data_ptr);
            pPcmDelayEnable->enable                   = me_ptr->lib_config.enable;
            params_ptr->actual_data_len               = sizeof(param_id_module_enable_t);
         }
         else
         {
            AR_MSG(DBG_ERROR_PRIO, "CAPI PCM Delay Get Enable Param, Bad payload size %lu", params_ptr->max_data_len);
            CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
         }
         break;
      }
      case PARAM_ID_LATENCY_MODE:
      {
         if (params_ptr->max_data_len >= sizeof(param_id_latency_mode_t))
         {
            param_id_latency_mode_t *rt_jitter_correction_mode_ptr = (param_id_latency_mode_t *)(params_ptr->data_ptr);
            rt_jitter_correction_mode_ptr->mode =
               me_ptr->is_rt_jitter_correction_enabled ? LATENCY_RT_JITTER_CORRECTION_MODE : LATENCY_DEFAULT_MODE;
            params_ptr->actual_data_len = sizeof(param_id_latency_mode_t);
         }
         else
         {
            AR_MSG(DBG_ERROR_PRIO, "CAPI PCM Delay Get mode Param, Bad payload size %lu", params_ptr->max_data_len);
            CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
         }
         break;
      }
      case PARAM_ID_LATENCY_CFG:
      {
         if (params_ptr->max_data_len >= sizeof(param_id_latency_cfg_t))
         {
            param_id_latency_cfg_t *delay_param_ptr = (param_id_latency_cfg_t *)(params_ptr->data_ptr);
            delay_param_ptr->cfg_mode               = me_ptr->cfg_mode;

            if (LATENCY_MODE_GLOBAL == me_ptr->cfg_mode)
            {
               uint32_t max_delay_in_us = me_ptr->cache_delay.cache_delay_per_config[0].delay_in_us;
               for (uint32_t i = 1; i < me_ptr->cache_delay.num_config; i++)
               {
                  if (max_delay_in_us < me_ptr->cache_delay.cache_delay_per_config[i].delay_in_us)
                  {
                     max_delay_in_us = me_ptr->cache_delay.cache_delay_per_config[i].delay_in_us;
                  }
               }
               delay_param_ptr->num_config      = 0;
               delay_param_ptr->global_delay_us = max_delay_in_us;
               params_ptr->actual_data_len      = sizeof(param_id_latency_cfg_t);
               CAPI_SET_ERROR(capi_result, CAPI_EOK);
            }
            else if (LATENCY_MODE_PER_CH == me_ptr->cfg_mode)
            {
               delay_param_ptr->num_config      = me_ptr->cache_delay.num_config;
               delay_param_ptr->global_delay_us = 0; // don't care
               uint8_t *temp_cfg_ptr            = (uint8_t *)params_ptr->data_ptr;
               temp_cfg_ptr += sizeof(param_id_latency_cfg_t);

               uint32_t payload_size =
                  sizeof(param_id_latency_cfg_t) + (sizeof(delay_param_per_ch_cfg_t) * me_ptr->cache_delay.num_config);

               if (params_ptr->max_data_len >= payload_size)
               {
                  delay_param_per_ch_cfg_t *delay_cfg_ptr = (delay_param_per_ch_cfg_t *)temp_cfg_ptr;

                  for (uint32_t count = 0; count < me_ptr->cache_delay.num_config; count++)
                  {
                     delay_cfg_ptr[count].channel_mask_lsb =
                        me_ptr->cache_delay.cache_delay_per_config[count].channel_mask_lsb;
                     delay_cfg_ptr[count].channel_mask_msb =
                        me_ptr->cache_delay.cache_delay_per_config[count].channel_mask_msb;
                     delay_cfg_ptr[count].delay_us = me_ptr->cache_delay.cache_delay_per_config[count].delay_in_us;
                  }

                  params_ptr->actual_data_len = payload_size;
                  CAPI_SET_ERROR(capi_result, CAPI_EOK);
               }
               else
               {
                  AR_MSG(DBG_ERROR_PRIO, "CAPI Latency Get Delay, Bad param size %lu", params_ptr->max_data_len);
                  CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
               }
            }
         }
         else
         {
            AR_MSG(DBG_ERROR_PRIO, "CAPI Latency Get Delay, Bad param size %lu", params_ptr->max_data_len);
            CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
         }
         break;
      }

      case PARAM_ID_LATENCY_CFG_V2:
      {
         capi_result = capi_latency_get_config_v2(me_ptr, params_ptr);
         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO, "CAPI latency Get, unsupported param ID 0x%x", (int)param_id);
         CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
         break;
      }
   }
   return capi_result;
}

static capi_err_t capi_latency_set_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{

   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif || NULL == props_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI latency: Set properties received bad pointer, 0x%p", _pif);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_latency_t *me_ptr = (capi_latency_t *)_pif;

   capi_result |= capi_latency_process_set_properties(me_ptr, props_ptr);

   return capi_result;
}

/* -------------------------------------------------------------------------
 * Function name: capi_latency_get_properties
 * Function to get the properties of latency module
 * -------------------------------------------------------------------------*/
static capi_err_t capi_latency_get_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif || NULL == props_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI latency: Get properties received bad pointer, 0x%p", _pif);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   capi_latency_t *me_ptr = (capi_latency_t *)_pif;

   capi_result |= capi_latency_process_get_properties(me_ptr, props_ptr);

   return capi_result;
}
