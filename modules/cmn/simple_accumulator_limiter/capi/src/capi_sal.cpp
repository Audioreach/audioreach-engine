/* ======================================================================== */
/**
   @file capi_sal.cpp

   Source file to implement the CAPI Interface for
   Simple Accumulator-Limiter (SAL) Module.
*/

/* =========================================================================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
   ========================================================================== */

/*==========================================================================
Include files
========================================================================== */
#include "capi_sal.h"
#include "capi_sal_utils.h"
#include "sal_api.h"
#include "spf_list_utils.h"
#include "audio_basic_op.h"

static void extract_h16(int8_t *input_ch_buf, int8_t *output_ch_buf, uint32_t num_samp_per_ch);
static void extract_l16(int8_t *input_ch_buf, int8_t *output_ch_buf, uint32_t num_samp_per_ch);

/*------------------------------------------------------------------------
   Function name: capi_sal_get_static_properties
   DESCRIPTION: Function to get the static properties of sal module
-----------------------------------------------------------------------*/
capi_err_t capi_sal_get_static_properties(capi_proplist_t *init_set_properties, capi_proplist_t *static_properties)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL != static_properties)
   {
      capi_result |= capi_sal_get_properties((capi_t *)NULL, static_properties);
   }
   return capi_result;
}

/*------------------------------------------------------------------------
  Function name: capi_sal_init
  DESCRIPTION: Initialize the CAPIv2 sal module and library.
  -----------------------------------------------------------------------*/
capi_err_t capi_sal_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   capi_err_t  capi_result = CAPI_EOK;
   if ((NULL == _pif) || (NULL == init_set_properties))
   {
      AR_MSG(DBG_ERROR_PRIO, "Init received NULL pointer");
      return CAPI_EBADPARAM;
   }

   capi_sal_t *me_ptr = (capi_sal_t *)_pif;
   memset(me_ptr, 0, sizeof(capi_sal_t));
   me_ptr->vtbl.vtbl_ptr = capi_sal_get_vtbl(); // assigning the vtbl with all function pointers

   me_ptr->bps_cfg_mode                         = SAL_PARAM_NATIVE;
   me_ptr->limiter_enabled                      = (uint32_t)TRUE; // limiter is enabled by default
   me_ptr->module_flags.op_mf_requires_limiting = TRUE;           // limiter is required by default
   me_ptr->cfg_lim_block_size_ms                = CAPI_SAL_LIM_BLOCK_SIZE_1_MS;
   // init limiter cfg
   capi_sal_init_limiter_cfg(me_ptr);

   capi_cmn_init_media_fmt_v2(&me_ptr->last_raised_out_mf);

   capi_result = capi_sal_set_properties((capi_t *)me_ptr, init_set_properties);

   capi_result |= capi_cmn_raise_deinterleaved_unpacked_v2_supported_event(&me_ptr->cb_info);

   return capi_result;
}

/*------------------------------------------------------------------------
  Function name: capi_sal_end
  DESCRIPTION: Returns the library to the uninitialized state and frees the
  memory that was allocated by module. This function also frees the virtual
  function table.
  -----------------------------------------------------------------------*/
capi_err_t capi_sal_end(capi_t *_pif)
{
   capi_err_t  capi_result = CAPI_EOK;
   capi_sal_t *me_ptr      = (capi_sal_t *)_pif;

   if (NULL == me_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "End received bad pointer, 0x%p", _pif);
      return CAPI_EBADPARAM;
   }

   if (NULL != me_ptr->limiter_memory.mem_ptr)
   {
      posal_memory_free(me_ptr->limiter_memory.mem_ptr);
      me_ptr->limiter_memory.mem_ptr = NULL;
   }

   if (NULL != me_ptr->in_port_arr)
   {
      posal_memory_free((void *)me_ptr->in_port_arr);
      me_ptr->in_port_arr = NULL;
   }

   if (NULL != me_ptr->started_in_port_index_arr)
   {
      posal_memory_free(me_ptr->started_in_port_index_arr);
      me_ptr->started_in_port_index_arr = NULL;
   }

   capi_result |= capi_sal_destroy_scratch_ptr_buf(me_ptr);
   me_ptr->vtbl.vtbl_ptr = NULL;

   SAL_MSG(me_ptr->iid, DBG_HIGH_PRIO, "capi_sal_end: completed");

   return capi_result;
}
/*------------------------------------------------------------------------
  Function name: capi_sal_set_param
  DESCRIPTION: Function to set parameter value\structure.
  -----------------------------------------------------------------------*/
capi_err_t capi_sal_set_param(capi_t *                _pif,
                              uint32_t                param_id,
                              const capi_port_info_t *port_info_ptr,
                              capi_buf_t *            params_ptr)
{
   capi_err_t  capi_result = CAPI_EOK;
   capi_sal_t *me_ptr      = (capi_sal_t *)(_pif);
   if (NULL == _pif || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "set param received bad pointer, 0x%p, 0x%p", _pif, params_ptr);
      return CAPI_EBADPARAM;
   }
   switch (param_id)
   {
      case PARAM_ID_SAL_LIMITER_ENABLE:
      {
         if (params_ptr->actual_data_len < sizeof(param_id_sal_limiter_enable_t))
         {
            SAL_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "Param id 0x%lx Bad param size %lu",
                    (uint32_t)param_id,
                    params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }
         param_id_sal_limiter_enable_t *payload_ptr = (param_id_sal_limiter_enable_t *)params_ptr->data_ptr;
         me_ptr->limiter_enabled                    = payload_ptr->enable_lim;
         capi_sal_update_raise_delay_event(me_ptr);
         SAL_MSG(me_ptr->iid, DBG_MED_PRIO, "SetParam SAL Limiter enable %x", me_ptr->limiter_enabled);
         break;
      } // PARAM_ID_SAL_LIMITER_ENABLE
      case PARAM_ID_SAL_OUTPUT_CFG:
      {
         if (params_ptr->actual_data_len < sizeof(param_id_sal_output_cfg_t))
         {
            SAL_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "Param id 0x%lx Bad param size %lu",
                    (uint32_t)param_id,
                    params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }
         param_id_sal_output_cfg_t *payload_ptr = (param_id_sal_output_cfg_t *)params_ptr->data_ptr;
         /* Validate supported bit-widths */
         if (PARAM_VAL_NATIVE == payload_ptr->bits_per_sample)
         {
            SAL_MSG(me_ptr->iid, DBG_HIGH_PRIO, "Sal configured in Native Mode");
            // (valid mode to native switch)
            if ((me_ptr->operating_mf_ptr) &&
                (me_ptr->out_port_cache_cfg.q_factor != me_ptr->operating_mf_ptr->format.q_factor))
            {
               bool_t MF_RAISED_UNUSED                    = FALSE;
               me_ptr->out_port_cache_cfg.q_factor        = me_ptr->operating_mf_ptr->format.q_factor;
               me_ptr->out_port_cache_cfg.word_size_bytes = me_ptr->operating_mf_ptr->format.bits_per_sample >> 3;

               capi_result |=
                  capi_sal_alloc_scratch_lim_mem_and_raise_events(me_ptr,
                                                                  me_ptr->out_port_cache_cfg.word_size_bytes << 3,
                                                                  me_ptr->out_port_cache_cfg.q_factor);
               capi_sal_raise_out_mf(me_ptr, me_ptr->operating_mf_ptr, &MF_RAISED_UNUSED);
            }
            me_ptr->bps_cfg_mode = SAL_PARAM_NATIVE;
            // do nothing else
            break;
         }
         else if (PARAM_VAL_UNSET == payload_ptr->bits_per_sample)
         {
            // do nothing, carry on with the previous config
            SAL_MSG(me_ptr->iid, DBG_HIGH_PRIO, "Sal received unset cfg, current mode = %d", me_ptr->bps_cfg_mode);
            break;
         }
         else if (PARAM_VAL_INVALID == payload_ptr->bits_per_sample)
         {
            SAL_MSG(me_ptr->iid,
                    DBG_HIGH_PRIO,
                    "Sal received invlaid cfg, faliling, current mode = %d",
                    me_ptr->bps_cfg_mode);
            capi_result |= CAPI_EFAILED;
            break;
         }
         // possibly valid params

         if ((BIT_WIDTH_16 != payload_ptr->bits_per_sample) && (BIT_WIDTH_24 != payload_ptr->bits_per_sample) &&
             (BIT_WIDTH_32 != payload_ptr->bits_per_sample))
         {
            SAL_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "Unsupported bit-width config %lu, only 16, 24 and 32 are supported",
                    payload_ptr->bits_per_sample);
            capi_result |= CAPI_EBADPARAM;
            break;
         }

         // update output mf parameters
         uint32_t prev_qf_val = me_ptr->out_port_cache_cfg.q_factor;

         me_ptr->out_port_cache_cfg.q_factor = capi_sal_bps_to_qfactor(payload_ptr->bits_per_sample);
         me_ptr->bps_cfg_mode                = SAL_PARAM_VALID;

         SAL_MSG(me_ptr->iid,
                 DBG_HIGH_PRIO,
                 "received o/p Configuration: bit width = %lu, So QF %lu",
                 payload_ptr->bits_per_sample,
                 me_ptr->out_port_cache_cfg.q_factor);

         // 2 bytes or 4 byte output word  capi_sal_qf_to_bps
         me_ptr->out_port_cache_cfg.word_size_bytes = payload_ptr->bits_per_sample >> 3;
         me_ptr->out_port_cache_cfg.word_size_bytes =
            (BYTES_PER_SAMPLE_THREE == me_ptr->out_port_cache_cfg.word_size_bytes)
               ? BYTES_PER_SAMPLE_FOUR
               : me_ptr->out_port_cache_cfg.word_size_bytes;

         if ((me_ptr->operating_mf_ptr) && (me_ptr->out_port_cache_cfg.q_factor != prev_qf_val))
         {
            bool_t   MF_RAISED_UNUSED = FALSE;
            uint32_t lim_qf = SAL_MIN(me_ptr->out_port_cache_cfg.q_factor, me_ptr->operating_mf_ptr->format.q_factor);
            uint32_t lim_data_width = capi_sal_qf_to_bps(lim_qf);
            capi_result |= capi_sal_alloc_scratch_lim_mem_and_raise_events(me_ptr, lim_data_width, lim_qf);
            capi_sal_raise_out_mf(me_ptr, me_ptr->operating_mf_ptr, &MF_RAISED_UNUSED);
         }
         break;
      } // PARAM_ID_SAL_OUTPUT_QFACTOR

      case PARAM_ID_SAL_LIMITER_STATIC_CFG:
      {
         if (params_ptr->actual_data_len < sizeof(param_id_sal_limiter_static_cfg_t))
         {
            SAL_MSG(me_ptr->iid, DBG_ERROR_PRIO, "Limiter Static Cfg, Bad param size %lu", params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }
         param_id_sal_limiter_static_cfg_t *cfg_ptr = (param_id_sal_limiter_static_cfg_t *)params_ptr->data_ptr;
         me_ptr->cfg_lim_block_size_ms              = cfg_ptr->max_block_size_ms;
         me_ptr->limiter_static_vars.delay          = cfg_ptr->delay_in_sec_q15;

         SAL_MSG(me_ptr->iid,
                 DBG_HIGH_PRIO,
                 "Received new lim static configurations, block size (ms) = %lu, delay(Q15 sec) = %lu",
                 me_ptr->cfg_lim_block_size_ms,
                 me_ptr->limiter_static_vars.delay);

         if ((me_ptr->operating_mf_ptr) && (0 == capi_sal_get_num_active_in_ports(me_ptr)))
         {
            uint32_t lim_qf = SAL_MIN(me_ptr->out_port_cache_cfg.q_factor, me_ptr->operating_mf_ptr->format.q_factor);
            uint32_t lim_data_width = capi_sal_qf_to_bps(lim_qf);
            capi_result |= capi_sal_alloc_scratch_lim_mem_and_raise_events(me_ptr, lim_data_width, lim_qf);
         }
         else
         {
            SAL_MSG(me_ptr->iid,
                    DBG_HIGH_PRIO,
                    "Caching the static lim params since OMF is not set yet or some port(s) is(are) in flow state");
            // will be applied the next time lim needs to be reallocated.
         }
         break;
      }

      case PARAM_ID_LIMITER_CFG:
      {
         if (params_ptr->actual_data_len < sizeof(param_id_limiter_cfg_t))
         {
            SAL_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "Limiter Tuning param, Bad param size %lu",
                    params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }
         param_id_limiter_cfg_t *limiter_config_params_ptr = (param_id_limiter_cfg_t *)params_ptr->data_ptr;
         me_ptr->limiter_params.threshold                  = limiter_config_params_ptr->threshold;
         me_ptr->limiter_params.makeup_gain                = limiter_config_params_ptr->makeup_gain;
         me_ptr->limiter_params.gc                         = limiter_config_params_ptr->gc;
         me_ptr->limiter_params.max_wait                   = limiter_config_params_ptr->max_wait;
         me_ptr->limiter_params.gain_attack                = limiter_config_params_ptr->gain_attack;
         me_ptr->limiter_params.gain_release               = limiter_config_params_ptr->gain_release;
         me_ptr->limiter_params.attack_coef                = limiter_config_params_ptr->attack_coef;
         me_ptr->limiter_params.release_coef               = limiter_config_params_ptr->release_coef;
         me_ptr->limiter_params.hard_threshold             = limiter_config_params_ptr->hard_threshold;

         me_ptr->module_flags.is_lim_set_cfg_rcvd = TRUE;

         if (me_ptr->operating_mf_ptr)
         {
            limiter_tuning_v2_t limiter_cfg;
            (void)
               memscpy(&limiter_cfg, sizeof(limiter_tuning_v2_t), &me_ptr->limiter_params, sizeof(limiter_tuning_v2_t));

            if ((me_ptr->module_flags.is_lim_set_cfg_rcvd) && (16 == me_ptr->operating_mf_ptr->format.bits_per_sample))
            {
               limiter_cfg.threshold      = limiter_cfg.threshold >> 12;
               limiter_cfg.hard_threshold = limiter_cfg.hard_threshold >> 12;
            }
            for (uint32_t i = 0; i < me_ptr->operating_mf_ptr->format.num_channels; i++)
            {
               me_ptr->limiter_params.ch_idx = i;
               limiter_cfg.ch_idx            = i;
               if (LIMITER_SUCCESS != limiter_set_param(&me_ptr->lib_mem,
                                                        LIMITER_PARAM_TUNING_V2,
                                                        &limiter_cfg,
                                                        sizeof(limiter_tuning_v2_t)))
               {
                  capi_result |= CAPI_EFAILED;
                  SAL_MSG(me_ptr->iid, DBG_ERROR_PRIO, "set lim tuning param failed with capi_result %d", capi_result);
                  break;
               }
            }
         }
         else
         {
            SAL_MSG(me_ptr->iid,
                    DBG_MED_PRIO,
                    "Warning: Cannot do per channel lim tuning before receiving IMF, Caching");
         }

#ifdef SAL_DBG_LOW
         SAL_MSG(me_ptr->iid, DBG_HIGH_PRIO, "Successfully Tuned the Limiter Params V2");
#endif // SAL_DBG_LOW
         break;
      } // PARAM_ID_LIMITER_CFG
      case INTF_EXTN_PARAM_ID_METADATA_HANDLER:
      {
         if (params_ptr->actual_data_len < sizeof(intf_extn_param_id_metadata_handler_t))
         {
            SAL_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "capi_spr: Param id 0x%lx Bad param size %lu",
                    (uint32_t)param_id,
                    params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }
         intf_extn_param_id_metadata_handler_t *payload_ptr =
            (intf_extn_param_id_metadata_handler_t *)params_ptr->data_ptr;
         me_ptr->metadata_handler = *payload_ptr;
         break;
      }
      case INTF_EXTN_PARAM_ID_DATA_PORT_OPERATION:
      {
         if (NULL == me_ptr->in_port_arr)
         {
            SAL_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "Port structures memory should have been allocated for max ports during OPEN. Error");
            capi_result |= CAPI_EFAILED;
            break;
         }
         if (NULL == params_ptr->data_ptr)
         {
            SAL_MSG(me_ptr->iid, DBG_ERROR_PRIO, "Set property id 0x%lx, received null buffer", param_id);
            capi_result |= CAPI_EBADPARAM;
            break;
         }
         if (params_ptr->actual_data_len < sizeof(intf_extn_data_port_operation_t))
         {
            SAL_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "Invalid payload size for port operation %d",
                    params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }
         intf_extn_data_port_operation_t *data_ptr = (intf_extn_data_port_operation_t *)(params_ptr->data_ptr);
         if (params_ptr->actual_data_len <
             sizeof(intf_extn_data_port_operation_t) + (data_ptr->num_ports * sizeof(intf_extn_data_port_id_idx_map_t)))
         {
            SAL_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "Invalid payload size for port operation %d",
                    params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }

         // we do bookkeeping only for the input ports for SAL since there are multiple ports
         // on the input side. In general, we do bookkeeping for whichever side (can be both) that has multi-port
         // capability.

         // for input ports
         if (data_ptr->is_input_port && (data_ptr->num_ports > me_ptr->num_in_ports))
         {
            SAL_MSG(me_ptr->iid, DBG_ERROR_PRIO, "parameter, num_ports %d", data_ptr->num_ports);
            capi_result |= CAPI_EBADPARAM;
            break;
         }

         for (uint32_t i = 0; i < data_ptr->num_ports; i++)
         {
            uint32_t data_port_index = data_ptr->id_idx[i].port_index;

            if (data_ptr->is_input_port && (data_port_index >= me_ptr->num_in_ports))
            {
               SAL_MSG(me_ptr->iid,
                       DBG_ERROR_PRIO,
                       "Bad parameter in id-idx map on port %lu, port_index = %lu, num ports = %d",
                       i,
                       data_port_index,
                       me_ptr->num_in_ports);
               capi_result |= CAPI_EBADPARAM;
               break; // breaks out of for
            }

            switch (data_ptr->opcode)
            {
               case INTF_EXTN_DATA_PORT_OPEN:
               {
                  if (!data_ptr->is_input_port) // cache state fo output ports
                  {
                     me_ptr->output_state = DATA_PORT_OPEN;
                     capi_result |= CAPI_EOK;
                     break;
                  }

                  // don't need specific payload
                  me_ptr->in_port_arr[data_port_index].state = DATA_PORT_OPEN;
                  // SAL_MSG(me_ptr->iid, DBG_MED_PRIO, "opening port_index %lu", data_port_index);
                  break;
               }
               case INTF_EXTN_DATA_PORT_START:
               {
                  if (!data_ptr->is_input_port) // cache state for output ports
                  {
                     me_ptr->output_state = DATA_PORT_STARTED;
                     capi_result |= CAPI_EOK;
                     break;
                  }

                  if (DATA_PORT_CLOSED != me_ptr->in_port_arr[data_port_index].state)
                  {
                     capi_sal_handle_data_flow_start(me_ptr, data_port_index);
                  }
                  else
                  {
                     SAL_MSG(me_ptr->iid, DBG_LOW_PRIO, "Cannot Start a closed port idx %lu", data_port_index);
                  }

                  break;
               }
               case INTF_EXTN_DATA_PORT_STOP:
               {

                  if (!data_ptr->is_input_port) // cache state for output ports
                  {
                     me_ptr->output_state = DATA_PORT_STOPPED;
                     capi_result |= CAPI_EOK;
                     break;
                  }

                  if (DATA_PORT_CLOSED != me_ptr->in_port_arr[data_port_index].state)
                  {
                     capi_result |= capi_sal_handle_data_flow_stop(me_ptr, data_port_index, FALSE /*data produced*/);
                  }
                  else
                  {
                     SAL_MSG(me_ptr->iid, DBG_LOW_PRIO, "Cannot STOP a closed port idx %lu", data_port_index);
                  }
                  break;
               }
               case INTF_EXTN_DATA_PORT_CLOSE:
               {
                  if (!data_ptr->is_input_port) // cache state for output ports
                  {
                     me_ptr->output_state = DATA_PORT_CLOSED;
                     capi_result |= CAPI_EOK;
                     break;
                  }

                  bool_t all_inputs_closed            = TRUE;
                  bool_t all_inputs_at_gap            = TRUE;
                  me_ptr->module_flags.insert_int_eos = FALSE;
#if 0
                  SAL_MSG(me_ptr->iid,
                          DBG_MED_PRIO,
                          "Got port close on port_index, %lu - First stopping and then closing internally",
                          data_port_index);
#endif
                  capi_sal_handle_data_flow_stop(me_ptr, data_port_index, FALSE /*data_produced*/);

                  // SAL_MSG(me_ptr->iid, DBG_MED_PRIO, "stop done. Now closing port_index %lu", data_port_index);

                  // don't need specific payload
                  me_ptr->in_port_arr[data_port_index].state = DATA_PORT_CLOSED;
                  // so that it could potentially be open to receiving completely new MF if all are closed in
                  // runtime
                  // if all ports are closed, do algo reset again
                  for (uint32_t j = 0; j < me_ptr->num_in_ports; j++)
                  {
                     if (DATA_PORT_CLOSED != me_ptr->in_port_arr[j].state)
                     {
                        all_inputs_closed = FALSE;
                     }
                     if (!me_ptr->in_port_arr[j].port_flags.at_gap)
                     {
                        all_inputs_at_gap                     = FALSE;
                        me_ptr->module_flags.all_ports_at_gap = FALSE;
                     }
                  }

                  // insert EOS when all inputs are closed and some inputs were not at gap.
                  me_ptr->module_flags.insert_int_eos = all_inputs_closed && !all_inputs_at_gap;

                  me_ptr->module_flags.any_valid_mf_rcvd = !all_inputs_closed;

                  if (all_inputs_closed && me_ptr->operating_mf_ptr)
                  {
                     capi_result |= capi_sal_algo_reset(me_ptr);
                  }

                  capi_cmn_init_media_fmt_v2(&me_ptr->in_port_arr[data_port_index].mf); // reset the mf
                  me_ptr->in_port_arr[data_port_index].port_flags.mf_rcvd = FALSE;      // erasing the mf rcvd flag
                  break;
               }
               default:
               {
                  // SAL_MSG(me_ptr->iid, DBG_ERROR_PRIO, "Port operation - Unsupported opcode: %lu", data_ptr->opcode);
                  CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
                  break;
               }
            }
         }

         // check and raise dynamic inplace
         capi_sal_check_and_update_lim_bypass_mode(me_ptr);
         break;
      } // CAPI_PORT_OPERATION
      default:
      {
         SAL_MSG(me_ptr->iid, DBG_ERROR_PRIO, "Set Param, unsupported param ID 0x%x", (int)param_id);
         capi_result |= CAPI_EUNSUPPORTED;
         break;
      }
   }
   return capi_result;
}

capi_err_t capi_sal_raise_mimo_process_state_event(capi_sal_t *me_ptr, bool_t is_disabled)
{
   capi_err_t                  result      = CAPI_EOK;
   capi_event_callback_info_t *cb_info_ptr = &me_ptr->cb_info;

   intf_extn_event_id_mimo_module_process_state_t event_payload;
   event_payload.is_disabled = is_disabled;

   /* Create event */
   capi_event_data_to_dsp_service_t to_send;
   to_send.param_id                = INTF_EXTN_EVENT_ID_MIMO_MODULE_PROCESS_STATE;
   to_send.payload.actual_data_len = sizeof(intf_extn_event_id_mimo_module_process_state_t);
   to_send.payload.max_data_len    = sizeof(intf_extn_event_id_mimo_module_process_state_t);
   to_send.payload.data_ptr        = (int8_t *)&event_payload;

   /* Create event info */
   capi_event_info_t event_info;
   event_info.port_info.is_input_port = FALSE;
   event_info.port_info.is_valid      = FALSE;
   event_info.payload.actual_data_len = sizeof(to_send);
   event_info.payload.max_data_len    = sizeof(to_send);
   event_info.payload.data_ptr        = (int8_t *)&to_send;

   result = cb_info_ptr->event_cb(cb_info_ptr->event_context, CAPI_EVENT_DATA_TO_DSP_SERVICE, &event_info);

   return result;
}

capi_err_t capi_sal_check_and_raise_process_state_events(capi_sal_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   bool_t   can_be_inplace   = FALSE;
   bool_t   is_disable       = FALSE;
   uint32_t num_active_ports = 0, num_opened_ports = 0;
   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      if ((DATA_PORT_STARTED == me_ptr->in_port_arr[i].state) && !me_ptr->in_port_arr[i].port_flags.at_gap)
      {
         num_active_ports++;
      }

      if (DATA_PORT_CLOSED != me_ptr->in_port_arr[i].state)
      {
         num_opened_ports++;
      }
   }

   if ((num_active_ports == 1) && (me_ptr->output_state == DATA_PORT_STARTED))
   {
      // module processing can be disabled for siso operation. fwk enables the module as soon as it
      // gets data on another port.
      is_disable = TRUE;
   }

   if ((is_disable) && (num_opened_ports == 1))
   {
      // inplace can be done if there is just one input port opened.
      can_be_inplace = TRUE;
   }

   if (capi_sal_check_limiting_required(me_ptr))
   {
      // if limiting needed then it can't be inplace or disabled.
      can_be_inplace = FALSE;
      is_disable     = FALSE;
   }

   if (can_be_inplace != me_ptr->module_flags.is_inplace)
   {
      (void)capi_cmn_raise_dynamic_inplace_event(&me_ptr->cb_info, can_be_inplace);
      me_ptr->module_flags.is_inplace = can_be_inplace;
   }

   (void)capi_sal_raise_mimo_process_state_event(me_ptr, is_disable);

   return capi_result;
}

/*------------------------------------------------------------------------
  Function name: capi_sal_get_param
  DESCRIPTION: Function to get parameter value\structure.
  -----------------------------------------------------------------------*/
capi_err_t capi_sal_get_param(capi_t *                _pif,
                              uint32_t                param_id,
                              const capi_port_info_t *port_info_ptr,
                              capi_buf_t *            params_ptr)
{
   capi_err_t  capi_result = CAPI_EOK;
   capi_sal_t *me_ptr      = (capi_sal_t *)(_pif);

   if (NULL == _pif || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "set param received bad pointer, 0x%p, 0x%p", _pif, params_ptr);
      return CAPI_EBADPARAM;
   }

   switch (param_id)
   {
      case PARAM_ID_SAL_LIMITER_ENABLE:
      {
         if (params_ptr->max_data_len < sizeof(param_id_sal_limiter_enable_t))
         {
            SAL_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "Param id 0x%lx Bad param size %lu",
                    (uint32_t)param_id,
                    params_ptr->max_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }
         param_id_sal_limiter_enable_t *payload_ptr = (param_id_sal_limiter_enable_t *)params_ptr->data_ptr;
         payload_ptr->enable_lim                    = me_ptr->limiter_enabled;
         params_ptr->actual_data_len                = sizeof(param_id_sal_limiter_enable_t);
         break;
      }
      case PARAM_ID_SAL_OUTPUT_CFG:
      {
         if (params_ptr->max_data_len < sizeof(param_id_sal_output_cfg_t))
         {
            SAL_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "Param id 0x%lx Bad param size %lu",
                    (uint32_t)param_id,
                    params_ptr->max_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }
         param_id_sal_output_cfg_t *payload_ptr = (param_id_sal_output_cfg_t *)params_ptr->data_ptr;
         payload_ptr->bits_per_sample           = capi_sal_qf_to_bps(me_ptr->out_port_cache_cfg.q_factor);
         params_ptr->actual_data_len            = sizeof(param_id_sal_output_cfg_t);
         break;
      }
      case PARAM_ID_LIMITER_CFG:
      {
         if (params_ptr->max_data_len < sizeof(param_id_limiter_cfg_t))
         {
            SAL_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "Param id 0x%lx Bad param size %lu",
                    (uint32_t)param_id,
                    params_ptr->max_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }
         param_id_limiter_cfg_t *payload_ptr = (param_id_limiter_cfg_t *)params_ptr->data_ptr;
         payload_ptr->threshold              = me_ptr->limiter_params.threshold;
         payload_ptr->makeup_gain            = me_ptr->limiter_params.makeup_gain;
         payload_ptr->gc                     = me_ptr->limiter_params.gc;
         payload_ptr->max_wait               = me_ptr->limiter_params.max_wait;
         payload_ptr->gain_attack            = me_ptr->limiter_params.gain_attack;
         payload_ptr->gain_release           = me_ptr->limiter_params.gain_release;
         payload_ptr->attack_coef            = me_ptr->limiter_params.attack_coef;
         payload_ptr->release_coef           = me_ptr->limiter_params.release_coef;
         payload_ptr->hard_threshold         = me_ptr->limiter_params.hard_threshold;
         params_ptr->actual_data_len         = sizeof(param_id_limiter_cfg_t);
         break;
      }
      default:
      {
         SAL_MSG(me_ptr->iid, DBG_ERROR_PRIO, "Get Param, unsupported param ID 0x%x", (int)param_id);
         capi_result |= CAPI_EUNSUPPORTED;
         break;
      }
   }
   return capi_result;
}
/*------------------------------------------------------------------------
  Function name: capi_sal_set_properties
  DESCRIPTION: Function to set properties to the Simple Accumulator-Limiter module
  -----------------------------------------------------------------------*/
capi_err_t capi_sal_set_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_err_t  capi_result = CAPI_EOK;
   capi_sal_t *me_ptr      = (capi_sal_t *)_pif;
   if ((NULL == props_ptr) || (NULL == me_ptr))
   {
      AR_MSG(DBG_ERROR_PRIO, "Get property received null arguments");
      return CAPI_EBADPARAM;
   }

   capi_prop_t *prop_ptr = props_ptr->prop_ptr;
   // iterate over the properties
   for (uint32_t i = 0; i < props_ptr->props_num; i++)
   {
      capi_buf_t *payload_ptr = &prop_ptr[i].payload;

      switch (prop_ptr[i].id)
      {
         case CAPI_HEAP_ID:
         {
            if (NULL == payload_ptr->data_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO, "Set property id 0x%lx, received null buffer", prop_ptr[i].id);
               capi_result |= CAPI_EBADPARAM;
               break;
            }
            if (payload_ptr->max_data_len < sizeof(capi_heap_id_t))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "Set property id 0x%lx, Bad param size %lu",
                      prop_ptr[i].id,
                      payload_ptr->max_data_len);
               capi_result |= CAPI_ENEEDMORE;
               break;
            }
            capi_heap_id_t *data_ptr = (capi_heap_id_t *)payload_ptr->data_ptr;
            me_ptr->heap_mem.heap_id = data_ptr->heap_id;
            break;
         } // CAPI_HEAP_ID
         case CAPI_EVENT_CALLBACK_INFO:
         {
            if (payload_ptr->max_data_len < sizeof(capi_event_callback_info_t))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "Set property id 0x%lx, Bad param size %lu",
                      prop_ptr[i].id,
                      payload_ptr->max_data_len);
               capi_result |= CAPI_ENEEDMORE;
               break;
            }
            capi_event_callback_info_t *data_ptr = (capi_event_callback_info_t *)payload_ptr->data_ptr;
            me_ptr->cb_info.event_cb             = data_ptr->event_cb;
            me_ptr->cb_info.event_context        = data_ptr->event_context;
            payload_ptr->actual_data_len         = sizeof(capi_event_callback_info_t);
            break;
         }                        // CAPI_EVENT_CALLBACK_INFO
         case CAPI_PORT_NUM_INFO: // max number of ports
         {
            if (NULL == payload_ptr->data_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO, "Set property id 0x%lx, received null buffer", prop_ptr[i].id);
               capi_result |= CAPI_EBADPARAM;
               break;
            }

            if (payload_ptr->actual_data_len < sizeof(capi_port_num_info_t))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "Set property id 0x%lx, Bad param size %lu",
                      prop_ptr[i].id,
                      payload_ptr->actual_data_len);
               capi_result |= CAPI_ENEEDMORE;
               break;
            }
            capi_port_num_info_t *data_ptr = (capi_port_num_info_t *)payload_ptr->data_ptr;
            if (SAL_MAX_OUTPUT_PORTS < data_ptr->num_output_ports)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "Set property num port info - out of range, provided num input ports = %lu, "
                      "num "
                      "output ports = %lu",
                      data_ptr->num_input_ports,
                      data_ptr->num_output_ports);
               capi_result |= CAPI_EBADPARAM;
               break;
            }
            // max input ports
            me_ptr->num_in_ports = data_ptr->num_input_ports;
#ifdef SAL_DBG_LOW
            AR_MSG(DBG_HIGH_PRIO,
                   "Port num info set prop: num input ports: %lu, num output ports = 1",
                   me_ptr->num_in_ports);
#endif // SAL_DBG_LOW
            /*Allocate buffers used in processing*/
            // allocating memory for the state (active/inactive) bool array for max input ports
            // error check JUST IN CASE this property comes twice (not expected) TBD Error out
            if (NULL != me_ptr->in_port_arr)
            {
               posal_memory_free(me_ptr->in_port_arr);
            }
            me_ptr->in_port_arr =
               (sal_in_port_array_t *)posal_memory_malloc(me_ptr->num_in_ports * sizeof(sal_in_port_array_t),
                                                          (POSAL_HEAP_ID)me_ptr->heap_mem.heap_id);

            if (NULL == me_ptr->in_port_arr)
            {
               capi_result |= CAPI_ENOMEMORY;
               break;
            }

            if (NULL != me_ptr->started_in_port_index_arr)
            {
               posal_memory_free(me_ptr->started_in_port_index_arr);
            }
            me_ptr->started_in_port_index_arr = (int32_t *)posal_memory_malloc(me_ptr->num_in_ports * sizeof(int32_t),
                                                                               (POSAL_HEAP_ID)me_ptr->heap_mem.heap_id);
            if (NULL == me_ptr->started_in_port_index_arr)
            {
               AR_MSG(DBG_ERROR_PRIO, "memory allocation failure");
               capi_result |= CAPI_ENOMEMORY;
               break;
            }
            memset(me_ptr->started_in_port_index_arr, -1, me_ptr->num_in_ports * sizeof(int32_t));

            // false (inactive) intialize
            memset(me_ptr->in_port_arr, 0, me_ptr->num_in_ports * sizeof(sal_in_port_array_t));
            for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
            {
               capi_cmn_init_media_fmt_v2(&me_ptr->in_port_arr[i].mf);
               me_ptr->in_port_arr[i].port_flags.data_drop = TRUE; // initialize to drop everything
               me_ptr->in_port_arr[i].port_flags.at_gap    = TRUE;
               me_ptr->num_ports_at_gap++;
            }
            me_ptr->module_flags.all_ports_at_gap = TRUE;
            break;
         } // CAPI_PORT_NUM_INFO
         case CAPI_MODULE_INSTANCE_ID:
         {
            if (payload_ptr->actual_data_len >= sizeof(capi_module_instance_id_t))
            {
               capi_module_instance_id_t *data_ptr = (capi_module_instance_id_t *)payload_ptr->data_ptr;
               me_ptr->iid                         = data_ptr->module_instance_id;
               AR_MSG(DBG_LOW_PRIO,
                      "CAPI SAL: This module-id 0x%08lX, instance-id 0x%08lX",
                      data_ptr->module_id,
                      me_ptr->iid);
            }
            break;
         }
         /* --------------------------------------------------post-init set-props ------------------------------------*/
         case CAPI_ALGORITHMIC_RESET:
         {
            // SAL_MSG(me_ptr->iid, DBG_HIGH_PRIO, "Algo Reset called");
            if (NULL == me_ptr->in_port_arr)
            {
               SAL_MSG(me_ptr->iid,
                       DBG_ERROR_PRIO,
                       "Port structures memory should have been allocated for max ports during OPEN. Error");
               capi_result |= CAPI_EFAILED;
               break;
            }
            if (!prop_ptr[i].port_info.is_valid)
            {
               SAL_MSG(me_ptr->iid, DBG_ERROR_PRIO, "Algo Reset port info is invalid");
               capi_result |= CAPI_EBADPARAM;
               break;
            }
            if (!prop_ptr[i].port_info.is_input_port)
            {
               // SAL_MSG(me_ptr->iid, DBG_LOW_PRIO, "Algo Reset received for output port, doing nothing, breaking");
               break;
            }
            uint32_t port_index = prop_ptr[i].port_info.port_index; // index of the input port

            // for SAL, a port's self stop is indicated via algo-reset (input ports). Therefore, we should treat it like
            // a port stop,
            // and reset the states like at-gap, etc while retaining the media format.
            if (DATA_PORT_STARTED == me_ptr->in_port_arr[port_index].state)
            {
               SAL_MSG(me_ptr->iid,
                       DBG_MED_PRIO,
                       "Algo Reset/ Self Stop on input port index %lu. Stopping and setting to at_gap",
                       port_index);

               capi_sal_handle_data_flow_stop(me_ptr, port_index, FALSE /*data_produced*/);
               me_ptr->in_port_arr[port_index].port_flags.at_gap = TRUE;
            }

            me_ptr->module_flags.all_ports_at_gap = TRUE;
            me_ptr->num_ports_at_gap              = 0;
            for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
            {
               if (FALSE == me_ptr->in_port_arr[i].port_flags.at_gap)
               {
                  me_ptr->module_flags.all_ports_at_gap = FALSE;
               }
               else
               {
                  me_ptr->num_ports_at_gap++;
               }
            }

            uint32_t must_reset_lim                                 = TRUE;
            me_ptr->in_port_arr[port_index].port_flags.is_algo_proc = FALSE;
            if (me_ptr->in_port_arr[port_index].md_list_ptr)
            {
               module_cmn_md_list_t *node_ptr = me_ptr->in_port_arr[port_index].md_list_ptr;
               module_cmn_md_list_t *next_ptr = NULL;
               while (node_ptr)
               {
                  next_ptr = node_ptr->next_ptr;
                  me_ptr->metadata_handler.metadata_destroy(me_ptr->metadata_handler.context_ptr,
                                                            node_ptr,
                                                            TRUE /* is dropped*/,
                                                            &me_ptr->in_port_arr[port_index].md_list_ptr);
                  node_ptr = next_ptr;
               }
               me_ptr->in_port_arr[port_index].md_list_ptr = NULL;
            }
            me_ptr->in_port_arr[port_index].pending_zeros_at_eos = 0;

            // if all ports' algo proc are false, we need to reset the limiter
            for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
            {
               if (me_ptr->in_port_arr[i].port_flags.is_algo_proc)
               {
                  must_reset_lim = FALSE;
               }
            }
            if (must_reset_lim && me_ptr->operating_mf_ptr)
            {
               capi_result |= capi_sal_algo_reset(me_ptr);
               SAL_MSG(me_ptr->iid, DBG_HIGH_PRIO, "Algo Reset Done!");
            }

            break;
         } // CAPI_ALGORITHMIC_RESET
         case CAPI_INPUT_MEDIA_FORMAT_V2:
         {
            if (NULL == me_ptr->in_port_arr)
            {
               SAL_MSG(me_ptr->iid,
                       DBG_ERROR_PRIO,
                       "Port structures memory should have been allocated for max ports during OPEN. Error");
               capi_result |= CAPI_EFAILED;
               break;
            }
            if (NULL == payload_ptr->data_ptr)
            {
               SAL_MSG(me_ptr->iid, DBG_ERROR_PRIO, "Set property id 0x%lx, received null buffer", prop_ptr[i].id);
               capi_result |= CAPI_EBADPARAM;
               break;
            }
            /* Validate the MF payload */
            if (payload_ptr->actual_data_len <
                sizeof(capi_set_get_media_format_t) + sizeof(capi_standard_data_format_v2_t))
            {
               SAL_MSG(me_ptr->iid, DBG_ERROR_PRIO, "Invalid media format size %d", payload_ptr->actual_data_len);
               capi_result |= CAPI_ENEEDMORE;
               break;
            }
            if (!prop_ptr[i].port_info.is_valid)
            {
               SAL_MSG(me_ptr->iid, DBG_ERROR_PRIO, "Media format port info is invalid");
               capi_result |= CAPI_EBADPARAM;
               break;
            }

            uint32_t             port_index    = prop_ptr[i].port_info.port_index; // index of the input port
            capi_media_fmt_v2_t *media_fmt_ptr = (capi_media_fmt_v2_t *)(payload_ptr->data_ptr);
            uint32_t size_to_copy = sizeof(capi_set_get_media_format_t) + sizeof(capi_standard_data_format_v2_t) +
                                    (media_fmt_ptr->format.num_channels * sizeof(capi_channel_type_t));

            if (payload_ptr->actual_data_len < size_to_copy)
            {
               SAL_MSG(me_ptr->iid, DBG_ERROR_PRIO, "Invalid media format size %d", payload_ptr->actual_data_len);
               capi_result |= CAPI_ENEEDMORE;
               break;
            }
            /* Validate data format, interleaving and num channels */
            if ((CAPI_DEINTERLEAVED_UNPACKED_V2 != media_fmt_ptr->format.data_interleaving) ||
                (CAPI_FIXED_POINT != media_fmt_ptr->header.format_header.data_format) ||
                (CAPI_MAX_CHANNELS_V2 < media_fmt_ptr->format.num_channels))
            {
               capi_result |= CAPI_EBADPARAM;
               break;
            }
            /* Validate supported QFormats */
            if ((PCM_Q_FACTOR_31 != media_fmt_ptr->format.q_factor) &&
                (PCM_Q_FACTOR_27 != media_fmt_ptr->format.q_factor) &&
                (PCM_Q_FACTOR_15 != media_fmt_ptr->format.q_factor))
            {
               SAL_MSG(me_ptr->iid,
                       DBG_ERROR_PRIO,
                       "Unsupported Q format Q%lu, only Q31, Q27 and Q15 are supported",
                       media_fmt_ptr->format.q_factor);
               capi_result |= CAPI_EBADPARAM;
               break;
            }
            /* Validate supported Bps */
            if ((BIT_WIDTH_16 != media_fmt_ptr->format.bits_per_sample) &&
                (BIT_WIDTH_32 != media_fmt_ptr->format.bits_per_sample))
            {
               SAL_MSG(me_ptr->iid,
                       DBG_ERROR_PRIO,
                       "Unsupported BPS %lu, only 16, and 32 are supported",
                       media_fmt_ptr->format.bits_per_sample);
               capi_result |= CAPI_EBADPARAM;
               break;
            }

            SAL_MSG(me_ptr->iid,
                    DBG_HIGH_PRIO,
                    "SAL: Received media fmt for port %d with num channels = %d",
                    port_index,
                    media_fmt_ptr->format.num_channels);

            if (FALSE == me_ptr->module_flags.any_valid_mf_rcvd)
            {
               bool_t MF_RAISED_UNUSED = FALSE;
               capi_sal_raise_out_mf(me_ptr, media_fmt_ptr, &MF_RAISED_UNUSED);
               me_ptr->module_flags.any_valid_mf_rcvd = TRUE;
            }

            if (me_ptr->in_port_arr[port_index].port_flags.mf_rcvd)
            {
               // we already have mf for this port. Another mf is coming in. Before accepting, check if:
               // 1. Port is ref - if so, it means that it has OMF
               // 1a. Check if imf is different from omf - if not - all good same mf is coming again.
               // if so, new mf on ref port. accept it as the OMF, check if other ports' mfs are the same as new and
               // mark data_drops.
               // 2 if port is not ref, check if there's an OMF and validate mf against it. Mark the data_drops.
               if (me_ptr->in_port_arr[port_index].port_flags.is_ref_port)
               {
                  if (!me_ptr->operating_mf_ptr)
                  {
                     SAL_MSG(me_ptr->iid,
                             DBG_ERROR_PRIO,
                             "Unexpected error: ref port's mf should be the OMF. but ptr is null");
                     return CAPI_EFAILED;
                  }
                  capi_sal_evaluate_ref_port_imf(me_ptr,
                                                 size_to_copy,
                                                 media_fmt_ptr,
                                                 payload_ptr->actual_data_len,
                                                 port_index);
               }
               else
               {
                  // not ref port
                  capi_sal_evaluate_non_ref_port_imf(me_ptr,
                                                     size_to_copy,
                                                     media_fmt_ptr,
                                                     payload_ptr->actual_data_len,
                                                     port_index);
               }
            }
            else
            {
               // if mf was not received on this port, this is obviously was not the ref port
               capi_sal_evaluate_non_ref_port_imf(me_ptr,
                                                  size_to_copy,
                                                  media_fmt_ptr,
                                                  payload_ptr->actual_data_len,
                                                  port_index);
            }

            // figure out if lim needs to be in bypass or not (active siso)
            capi_sal_check_and_update_lim_bypass_mode(me_ptr);

            // set out port bps and qf from here if we're in native mode and we have an operating mf_ptr
            if ((SAL_PARAM_NATIVE == me_ptr->bps_cfg_mode) && (me_ptr->operating_mf_ptr))
            {
               me_ptr->out_port_cache_cfg.q_factor = me_ptr->operating_mf_ptr->format.q_factor;
               me_ptr->out_port_cache_cfg.word_size_bytes =
                  me_ptr->operating_mf_ptr->format.bits_per_sample >> 3; // will be 2 or 4
            }
            capi_sal_print_operating_mf(me_ptr);
            break;
         } // CAPI_INPUT_MEDIA_FORMAT_V2
         default:
         {
            // SAL_MSG(me_ptr->iid, DBG_ERROR_PRIO, "Unknown Prop[%d]", prop_ptr[i].id);
            capi_result |= CAPI_EUNSUPPORTED;
            break;
         }
      } // switch
   }    // for loop
   return capi_result;
}
/*------------------------------------------------------------------------
  Function name: capi_sal_get_properties
  DESCRIPTION: Function to get the properties from the Simple Accumulator-Limiter module
  -----------------------------------------------------------------------*/
capi_err_t capi_sal_get_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_err_t  capi_result = CAPI_EOK;
   capi_sal_t *me_ptr      = (capi_sal_t *)_pif;
   uint32_t    i;

   if (NULL == props_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Get property received null property array");
      return CAPI_EBADPARAM;
   }
   capi_prop_t *prop_ptr = props_ptr->prop_ptr;

   capi_basic_prop_t mod_prop;
   mod_prop.init_memory_req    = sizeof(capi_sal_t);
   mod_prop.stack_size         = CAPI_SAL_STACK_SIZE;
   mod_prop.num_fwk_extns      = 0;     // NA
   mod_prop.fwk_extn_ids_arr   = NULL;  // NA
   mod_prop.is_inplace         = FALSE; // not capable of in-place processing of data
   mod_prop.req_data_buffering = FALSE;
   mod_prop.max_metadata_size  = 0; // NA

   capi_result = capi_cmn_get_basic_properties(props_ptr, &mod_prop);
   if (CAPI_EOK != capi_result)
   {
      AR_MSG(DBG_ERROR_PRIO, "Get common basic properties failed with result %lu", capi_result);
      return capi_result;
   }

   // iterating over the properties
   for (i = 0; i < props_ptr->props_num; i++)
   {
      capi_buf_t *payload_ptr = &prop_ptr[i].payload;

      switch (prop_ptr[i].id)
      {
         case CAPI_INIT_MEMORY_REQUIREMENT:
         case CAPI_STACK_SIZE:
         case CAPI_IS_INPLACE:
         case CAPI_REQUIRES_DATA_BUFFERING:
         case CAPI_NUM_NEEDED_FRAMEWORK_EXTENSIONS:
         case CAPI_NEEDED_FRAMEWORK_EXTENSIONS:
         case CAPI_PORT_DATA_THRESHOLD: // ignore this.
         case CAPI_OUTPUT_MEDIA_FORMAT_SIZE:
         {
            break;
         }
         /*
         //default is one which is the same as SAL_PORT_THRESHOLD. So, commented for now.
         case CAPI_PORT_DATA_THRESHOLD:
         {
            if (payload_ptr->max_data_len >= sizeof(capi_port_data_threshold_t))
            {
               capi_port_data_threshold_t *data_ptr = (capi_port_data_threshold_t *)payload_ptr->data_ptr;
               if (!prop_ptr[i].port_info.is_valid)
               {
                  AR_MSG(DBG_ERROR_PRIO, "Get Property: port index not valid");
                  payload_ptr->actual_data_len = 0;
                  return CAPI_EBADPARAM;
               }

               if (!prop_ptr[i].port_info.is_input_port)
               {
                  if (0 != prop_ptr[i].port_info.port_index)
                  {
                     AR_MSG(DBG_ERROR_PRIO,
                            "Get Property: prop_id(0x%lx): Max 1 output port supported. Asking "
                            "for port_index(%lu)",
                            (uint32_t)prop_ptr[i].id,
                            prop_ptr[i].port_info.port_index);
                     payload_ptr->actual_data_len = 0;
                     return CAPI_EBADPARAM;
                  }
               }
               data_ptr->threshold_in_bytes = CAPI_SAL_PORT_THRESHOLD; // one byte
               payload_ptr->actual_data_len = sizeof(capi_port_data_threshold_t);
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "Get Property: prop_id(0x%lx): received bad param size %lu",
                      (uint32_t)prop_ptr[i].id,
                      payload_ptr->max_data_len);
               payload_ptr->actual_data_len = 0;
               return CAPI_ENEEDMORE;
            }
            break;
         } // CAPI_PORT_DATA_THRESHOLD*/
         case CAPI_OUTPUT_MEDIA_FORMAT_V2:
         {
            if (NULL == me_ptr || NULL == me_ptr->operating_mf_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO, "pif is NULL for get OUTPUT MF, or OMF is not decided");
               capi_result |= CAPI_EBADPARAM;
               break;
            }
            uint32_t ret_size = sizeof(capi_media_fmt_v1_t) +
                                (me_ptr->operating_mf_ptr->format.num_channels * sizeof(capi_channel_type_t));
            /* Validate the MF payload */
            if (payload_ptr->max_data_len < ret_size)
            {
               AR_MSG(DBG_ERROR_PRIO, "Invalid media format size %d", payload_ptr->actual_data_len);

               capi_result |= CAPI_ENEEDMORE;
               break;
            }
            capi_media_fmt_v2_t *media_fmt_ptr = (capi_media_fmt_v2_t *)(payload_ptr->data_ptr);
            memscpy(media_fmt_ptr, ret_size, me_ptr->operating_mf_ptr, sizeof(capi_media_fmt_v2_t));
            media_fmt_ptr->format.bits_per_sample = me_ptr->out_port_cache_cfg.word_size_bytes * 8;
            media_fmt_ptr->format.q_factor        = me_ptr->out_port_cache_cfg.q_factor;
            payload_ptr->actual_data_len          = ret_size;
            break;
         } // CAPI_OUTPUT_MEDIA_FORMAT_V2
         case CAPI_INTERFACE_EXTENSIONS:
         {
            if (payload_ptr->max_data_len >= sizeof(capi_interface_extns_list_t))
            {
               capi_interface_extns_list_t *intf_ext_list = (capi_interface_extns_list_t *)payload_ptr->data_ptr;
               if (payload_ptr->max_data_len < (sizeof(capi_interface_extns_list_t) +
                                                (intf_ext_list->num_extensions * sizeof(capi_interface_extn_desc_t))))
               {
                  AR_MSG(DBG_ERROR_PRIO, "CAPI_INTERFACE_EXTENSIONS invalid param size %lu", payload_ptr->max_data_len);
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
                        case INTF_EXTN_METADATA:
                        {
                           curr_intf_extn_desc_ptr->is_supported = TRUE;
                           break;
                        }
                        case INTF_EXTN_DATA_PORT_OPERATION:
                        {
                           curr_intf_extn_desc_ptr->is_supported = TRUE;
                           break;
                        }
                        case INTF_EXTN_MIMO_MODULE_PROCESS_STATE:
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
                     AR_MSG(DBG_HIGH_PRIO,
                            "CAPI_INTERFACE_EXTENSIONS intf_ext = 0x%lx, is_supported = %d",
                            curr_intf_extn_desc_ptr->id,
                            (int)curr_intf_extn_desc_ptr->is_supported);
                     curr_intf_extn_desc_ptr++;
                  }
               }
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO, "CAPI_INTERFACE_EXTENSIONS Bad param size %lu", payload_ptr->max_data_len);
               CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
            }
            break;
         }
         default:
         {
            // AR_MSG(DBG_ERROR_PRIO, "Unknown Prop[0x%lX]", prop_ptr[i].id);
            capi_result |= CAPI_EUNSUPPORTED;
            break;
         }
      } // switch
   }    // for
   return capi_result;
}

capi_err_t capi_sal_dtmf(capi_sal_t *        me_ptr,
                         capi_stream_data_t *input[],
                         capi_stream_data_t *output[],
                         bool_t              any_port_has_md_n_flags,
                         uint32_t            max_num_samples_per_ch,
                         uint32_t            input_word_size_bytes)
{
   for (uint32_t j = 0; j < me_ptr->operating_mf_ptr->format.num_channels; j++)
   {
      memset(output[0]->buf_ptr[j].data_ptr, 0, output[0]->buf_ptr[0].max_data_len);
      memscpy(output[0]->buf_ptr[j].data_ptr,
              output[0]->buf_ptr[0].max_data_len,
              input[me_ptr->unmixed_output_port_index]->buf_ptr[j].data_ptr,
              input[me_ptr->unmixed_output_port_index]->buf_ptr[0].actual_data_len);
   }

#ifdef SAL_DBG_LOW
   uint32_t unmixed_bytes_copied = 0;

   for (uint32_t j = 0; j < me_ptr->operating_mf_ptr->format.num_channels; j++)
   {
      memset(output[0]->buf_ptr[j].data_ptr, 0, output[0]->buf_ptr[0].max_data_len);
      unmixed_bytes_copied = memscpy(output[0]->buf_ptr[j].data_ptr,
                                     output[0]->buf_ptr[0].max_data_len,
                                     input[me_ptr->unmixed_output_port_index]->buf_ptr[j].data_ptr,
                                     input[me_ptr->unmixed_output_port_index]->buf_ptr[0].actual_data_len);
   }
   SAL_MSG(me_ptr->iid,
           DBG_HIGH_PRIO,
           "output[0]->buf_ptr[0].actual_data_len = %d unmixed_bytes_copied = %d port idx %d",
           output[0]->buf_ptr[0].actual_data_len,
           unmixed_bytes_copied,
           me_ptr->unmixed_output_port_index);
#endif

   capi_sal_process_metadata_handler(any_port_has_md_n_flags,
                                     me_ptr,
                                     input,
                                     output,
                                     max_num_samples_per_ch,
                                     input_word_size_bytes);

   // nothing else needed to process return
   return CAPI_EOK;
}

bool_t capi_sal_inqf_greater_than_outqf(capi_sal_t *        me_ptr,
                                        uint32_t            input_qf,
                                        uint32_t            output_qf,
                                        capi_stream_data_t *output[],
                                        uint32_t            max_num_samples_per_ch,
                                        capi_err_t *        err_code_ptr)
{
   bool_t early_return = FALSE;
   if (QF_BPS_32 == input_qf)
   {
      if (QF_BPS_16 == output_qf)
      {
         // just extract higher 16 bits
         for (uint32_t i = 0; i < me_ptr->operating_mf_ptr->format.num_channels; i++)
         {
            extract_h16(me_ptr->acc_out_scratch_arr[i].data_ptr,
                        output[0]->buf_ptr[i].data_ptr,
                        max_num_samples_per_ch);
         }
      }
      else // output bps = 24 copy the whole thing
      {
         for (uint32_t i = 0; i < me_ptr->operating_mf_ptr->format.num_channels; i++)
         {
            // need to do the down conversion - no limiting
            downconvert_ws_32(me_ptr->acc_out_scratch_arr[i].data_ptr,
                              QF_BPS_32 - QF_BPS_24 /* 4 */,
                              max_num_samples_per_ch);

            memscpy(output[0]->buf_ptr[i].data_ptr,
                    output[0]->buf_ptr[0].actual_data_len,
                    me_ptr->acc_out_scratch_arr[i].data_ptr,
                    output[0]->buf_ptr[0].actual_data_len);
         }
      }
      // we won't use limiter if input is 32 bps
   }
   else //(QF_BPS_24 == input_qf) can't be 16 because of the condition
   {
      for (uint32_t i = 0; i < me_ptr->operating_mf_ptr->format.num_channels; i++)
      {
         // need to do the down conversion
         downconvert_ws_32(me_ptr->acc_out_scratch_arr[i].data_ptr,
                           QF_BPS_24 - QF_BPS_16 /* 12 */,
                           max_num_samples_per_ch);
      }
      // output bps is 16
      // if lim is disabled, need to extract
      if (FALSE == me_ptr->limiter_enabled)
      {
         for (uint32_t i = 0; i < me_ptr->operating_mf_ptr->format.num_channels; i++)
         {
            // limiter is reqd but not enabled
            extract_l16(me_ptr->acc_out_scratch_arr[i].data_ptr,
                        output[0]->buf_ptr[i].data_ptr,
                        max_num_samples_per_ch);
         }
      }
      else // lim enabled
      {
         *err_code_ptr = capi_sal_lim_loop_process(me_ptr, max_num_samples_per_ch, output);
         if (AR_EOK != *err_code_ptr)
         {
            early_return = TRUE;
            return early_return;
         }
      }
   }
   // Note: can't have 16 input here
   return early_return;
}
static void extract_h16(int8_t *input_ch_buf, int8_t *output_ch_buf, uint32_t num_samp_per_ch)
{
#ifdef __qdsp6__
   uint32_t num_iterations = num_samp_per_ch >> 1;
   int64_t *in_buf_ptr_64  = (int64_t *)input_ch_buf;
   int32_t *out_buf_ptr_32 = (int32_t *)output_ch_buf;

   for (uint32_t i = 0; i < num_iterations; i++)
   {
      int64_t temp        = *(in_buf_ptr_64++);
      *(out_buf_ptr_32++) = Q6_R_vasrw_PI(temp, 16);
   }

   if (num_samp_per_ch & 1)
   {
      int16_t *temp_in_buf = (int16_t *)(in_buf_ptr_64 + num_iterations); // Getting remaining sample
      int32_t  temp        = (int32_t)(*in_buf_ptr_64);
      *(temp_in_buf)       = s16_extract_s32_h(temp); // Q6_R_asrh_R (w32 >> 16)
   }
#else
   int32_t *in_buf_ptr_32  = (int32_t *)(input_ch_buf);
   int16_t *out_buf_ptr_16 = (int16_t *)(output_ch_buf);
   uint32_t count          = 0;
   while (count < num_samp_per_ch)
   {
      *(out_buf_ptr_16++) = s16_extract_s32_h(*in_buf_ptr_32); // Q6_R_asrh_R (w32 >> 16)
      in_buf_ptr_32++;
      count++;
   }
#endif
}

static void extract_l16(int8_t *input_ch_buf, int8_t *output_ch_buf, uint32_t num_samp_per_ch)
{
   int32_t *in_buf_ptr_32  = (int32_t *)(input_ch_buf);
   int16_t *out_buf_ptr_16 = (int16_t *)(output_ch_buf);
   uint32_t count          = 0;
   while (count < num_samp_per_ch)
   {
      *(out_buf_ptr_16++) = s16_extract_s32_l(*in_buf_ptr_32); //  Q6_R_sxth_R
      in_buf_ptr_32++;
      count++;
   }
}
