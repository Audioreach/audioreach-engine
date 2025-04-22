/**
 * \file gen_topo_dm_ext.c
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_topo.h"
#include "gen_topo_capi.h"

/************ Global variables and macros *******************/
#define GEN_TOPO_DM_EXT_MAX_SUPPORTED_NUM_PORTS 2

// Returns topo Dm mode id mapped to the given fwk extns dm mode id
static gen_topo_dm_mode_t gen_topo_get_mapped_dm_mode(fwk_extn_dm_mode_t fwk_extn_dm_mode)
{
   switch (fwk_extn_dm_mode)
   {
      case FWK_EXTN_DM_FIXED_INPUT_MODE:
      {
         return GEN_TOPO_DM_FIXED_INPUT_MODE;
      }
      case FWK_EXTN_DM_FIXED_OUTPUT_MODE:
      {
         return GEN_TOPO_DM_FIXED_OUTPUT_MODE;
      }
      default:
      {
         return GEN_TOPO_DM_INVALID_MODE;
      }
   }
}


// Check if the given output port is in the nblc path of a DM module's variable samples stream.
// and computes the additional bytes per ch required to accomodate variable buffer size requriement of the DM module
uint32_t gen_topo_compute_if_output_needs_addtional_bytes_for_dm(gen_topo_t *            topo_ptr,
                                                                 gen_topo_output_port_t *out_port_ptr)
{
   // check if additional bytes are required for output to accomodate variable samples for DM usecase
   uint32_t additional_bytes_req_for_dm_per_ch = 0;
   if (gen_topo_is_out_port_in_dm_variable_nblc(topo_ptr, out_port_ptr))
   {
      additional_bytes_req_for_dm_per_ch =
         topo_us_to_bytes_per_ch(GEN_TOPO_DM_BUFFER_MAX_EXTRA_LEN_US, out_port_ptr->common.media_fmt_ptr);
#ifdef VERBOSE_DEBUGGING
      TOPO_MSG(topo_ptr->gu.log_id,
               DBG_HIGH_PRIO,
               "Module 0x%lX: out port id 0x%lx, additional_bytes_per_ch=%lu to handle DM modules variable "
               "input/output",
               out_port_ptr->gu.cmn.module_ptr->module_instance_id,
               out_port_ptr->gu.cmn.id,
               additional_bytes_req_for_dm_per_ch);
#endif
   }

   return additional_bytes_req_for_dm_per_ch;
}

// sets expected ip/op samples per channel for DM modules depending on the dm mode.
// important note: this function must be called only after setting the dm mode.
ar_result_t gen_topo_updated_expected_samples_for_dm_modules(gen_topo_t *topo_ptr, gen_topo_module_t *module_ptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING

   // if module disabled dm mode return
   if(!gen_topo_is_dm_enabled(module_ptr))
   {
      return AR_EOK;
   }

   if (module_ptr->gu.num_input_ports > GEN_TOPO_DM_EXT_MAX_SUPPORTED_NUM_PORTS ||
       module_ptr->gu.num_output_ports > GEN_TOPO_DM_EXT_MAX_SUPPORTED_NUM_PORTS)
   {
      TOPO_MSG(topo_ptr->gu.log_id,
               DBG_ERROR_PRIO,
               "Module 0x%lX: num input ports %lu num output port %lu is more than supported ports %lu ",
               module_ptr->gu.module_instance_id,
               module_ptr->gu.num_input_ports,
               module_ptr->gu.num_output_ports,
               GEN_TOPO_DM_EXT_MAX_SUPPORTED_NUM_PORTS);
      return result;
   }

   // payload structure
   struct
   {
      fwk_extn_dm_param_id_req_samples_t header;
      fwk_extn_dm_port_samples_t         port_req_samples[GEN_TOPO_DM_EXT_MAX_SUPPORTED_NUM_PORTS - 1];
   } cfg;
   memset(&cfg, 0, sizeof(cfg));

   if (GEN_TOPO_DM_FIXED_OUTPUT_MODE == module_ptr->flags.dm_mode)
   {
      // get sample requirement for each port
      cfg.header.is_input  = FALSE;
      cfg.header.num_ports = 0;
      for (gu_output_port_list_t *op_port_list_ptr = module_ptr->gu.output_port_list_ptr; NULL != op_port_list_ptr;
           LIST_ADVANCE(op_port_list_ptr))
      {
         gen_topo_output_port_t *out_port_ptr     = (gen_topo_output_port_t *)op_port_list_ptr->op_port_ptr;
         uint32_t                sample_available = 0, samples_required = 0;
         gen_topo_input_port_t * conn_in_port_ptr = (gen_topo_input_port_t *)out_port_ptr->gu.conn_in_port_ptr;

         if (conn_in_port_ptr && conn_in_port_ptr->nblc_end_ptr &&
             SPF_IS_PCM_DATA_FORMAT(conn_in_port_ptr->nblc_end_ptr->common.media_fmt_ptr->data_format) &&
             conn_in_port_ptr->nblc_end_ptr->common.bufs_ptr &&
             conn_in_port_ptr->nblc_end_ptr->common.bufs_ptr[0].actual_data_len &&
             !conn_in_port_ptr->nblc_end_ptr->common.sdata.flags.end_of_frame &&
             !conn_in_port_ptr->nblc_end_ptr->common.flags.media_fmt_event)
         {
            uint32_t total_bytes_available = gen_topo_get_total_actual_len(&conn_in_port_ptr->nblc_end_ptr->common);

            total_bytes_available =
               topo_rescale_byte_count_with_media_fmt(total_bytes_available,
                                                      out_port_ptr->common.media_fmt_ptr,
                                                      conn_in_port_ptr->nblc_end_ptr->common.media_fmt_ptr);

            sample_available = topo_bytes_to_samples_per_ch(total_bytes_available, out_port_ptr->common.media_fmt_ptr);
         }

         samples_required = topo_bytes_to_samples_per_ch(gen_topo_get_curr_port_threshold(&out_port_ptr->common),
                                                         out_port_ptr->common.media_fmt_ptr);
         samples_required -= MIN(sample_available, samples_required);

         cfg.header.req_samples[cfg.header.num_ports].port_index          = out_port_ptr->gu.cmn.index;
         cfg.header.req_samples[cfg.header.num_ports].samples_per_channel = samples_required;

         cfg.header.num_ports++;
      }
   }
   else if (GEN_TOPO_DM_FIXED_INPUT_MODE == module_ptr->flags.dm_mode)
   {
      // get sample requirement for each port
      cfg.header.is_input  = TRUE;
      cfg.header.num_ports = 0;
      for (gu_input_port_list_t *ip_port_list_ptr = module_ptr->gu.input_port_list_ptr; NULL != ip_port_list_ptr;
           LIST_ADVANCE(ip_port_list_ptr))
      {
         gen_topo_input_port_t *in_port_ptr = (gen_topo_input_port_t *)ip_port_list_ptr->ip_port_ptr;

         cfg.header.req_samples[cfg.header.num_ports].port_index = in_port_ptr->gu.cmn.index;
         cfg.header.req_samples[cfg.header.num_ports].samples_per_channel =
            topo_bytes_to_samples_per_ch(gen_topo_get_curr_port_threshold(&in_port_ptr->common),
                                         in_port_ptr->common.media_fmt_ptr);

         cfg.header.num_ports++;
      }
   }

   // set sample requirement if its non zero
   if (cfg.header.req_samples[0].samples_per_channel)
   {
      TRY(result,
          gen_topo_capi_set_param(topo_ptr->gu.log_id,
                                  module_ptr->capi_ptr,
                                  FWK_EXTN_DM_PARAM_ID_SET_SAMPLES,
                                  (int8_t *)&cfg,
                                  sizeof(cfg)));
#ifdef VERBOSE_DEBUGGING
      TOPO_MSG(topo_ptr->gu.log_id,
               DBG_LOW_PRIO,
               "Setting expected samples_per_channel %lu for dm module iid 0x%lx, dm_mode %d (1=f/i,2=f/o), result %ld",
               cfg.header.req_samples[0].samples_per_channel,
               module_ptr->gu.module_instance_id,
               module_ptr->flags.dm_mode,
               result);
#endif
   }

   CATCH(result, TOPO_MSG_PREFIX, topo_ptr->gu.log_id)
   {
   }

   return result;
}

static ar_result_t gen_topo_update_module_dm_mode(gen_topo_t *                       topo_ptr,
                                                  gen_topo_module_t *                module_ptr)
{
   ar_result_t result = AR_EOK;

   if (!module_ptr->flags.need_dm_extn || module_ptr->flags.disabled) // don't check for dm-disabled flag.
   {
      return AR_EOK;
   }

   // determine dm mode if its a dm extension module.
   fwk_extn_dm_mode_t fwk_extn_dm_mode;
   result = gen_topo_determine_dm_mode_of_module(topo_ptr, module_ptr, &fwk_extn_dm_mode);
   if (AR_FAILED(result))
   {
      TOPO_MSG(topo_ptr->gu.log_id,
               DBG_ERROR_PRIO,
               "Failed determining DM mode of the DM module iid 0x%lx, not setting dm mode",
               module_ptr->gu.module_instance_id);
      return AR_EFAILED;
   }

   // update dm mode to the module if there is a change.
   gen_topo_dm_mode_t topo_dm_mode = gen_topo_get_mapped_dm_mode(fwk_extn_dm_mode);
   if (topo_dm_mode != module_ptr->flags.dm_mode)
   {
      fwk_extn_dm_param_id_change_mode_t mode_cfg;
      mode_cfg.dm_mode = fwk_extn_dm_mode;

      result = gen_topo_capi_set_param(topo_ptr->gu.log_id,
                                       module_ptr->capi_ptr,
                                       FWK_EXTN_DM_PARAM_ID_CHANGE_MODE,
                                       (int8_t *)&mode_cfg,
                                       sizeof(mode_cfg));

      if (AR_DID_FAIL(result) || module_ptr->flags.is_dm_disabled)
      {
         // consider mode as invalid if module returns a failure or dm is disabled
         module_ptr->flags.dm_mode = GEN_TOPO_DM_INVALID_MODE;
         result                    = AR_EOK;
      }
      else
      {
         module_ptr->flags.dm_mode = topo_dm_mode;
      }

      TOPO_MSG(topo_ptr->gu.log_id,
               DBG_MED_PRIO,
               "Setting module iid 0x%lx to dm mode %d (0=invalid,1=f/i,2=f/o), result %ld",
               module_ptr->gu.module_instance_id,
               module_ptr->flags.dm_mode,
               result);

      if (GEN_TOPO_DM_FIXED_OUTPUT_MODE == module_ptr->flags.dm_mode)
      {
         for (gu_input_port_list_t *in_port_list_ptr = module_ptr->gu.input_port_list_ptr; NULL != in_port_list_ptr;
              LIST_ADVANCE(in_port_list_ptr))
         {
            gen_topo_input_port_t *in_port_ptr = (gen_topo_input_port_t *)in_port_list_ptr->ip_port_ptr;
            if (NULL != in_port_ptr->nblc_start_ptr && NULL == in_port_ptr->nblc_start_ptr->gu.ext_in_port_ptr)
            {
               TOPO_MSG(topo_ptr->gu.log_id,
                        DBG_ERROR_PRIO,
                        "Warning: Fixed out DM module 0x%lx ip port id %lu is not in ext input nblc path..",
                        module_ptr->gu.module_instance_id,
                        in_port_ptr->gu.cmn.id);
            }
         }
      }
   }

   return result;
}
// Updates,
// 1. DM mode of the module depending up on the SG direction property.
// 2. No of expected samples depending upon the f/i or f/o mode.
ar_result_t gen_topo_update_dm_modes(gen_topo_t *topo_ptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING

   // Sorted list doesn't contain elementary modules, but elementary modules can't be dm modules.
   for (gu_module_list_t *module_list_ptr = (gu_module_list_t *)topo_ptr->gu.sorted_module_list_ptr; (NULL != module_list_ptr);
        LIST_ADVANCE(module_list_ptr))
   {
      gen_topo_module_t *module_ptr = (gen_topo_module_t *)module_list_ptr->module_ptr;

      if (!module_ptr->flags.need_dm_extn || !gen_topo_is_dm_enabled(module_ptr))
      {
         continue;
      }

      result |= gen_topo_update_module_dm_mode(topo_ptr, module_ptr);
   }


   //dm mode must be enabled if a dm-extn module if it is in variable data path.
   for (gu_module_list_t *module_list_ptr = (gu_module_list_t *)topo_ptr->gu.sorted_module_list_ptr; (NULL != module_list_ptr);
        LIST_ADVANCE(module_list_ptr))
   {
      gen_topo_module_t *module_ptr = (gen_topo_module_t *)module_list_ptr->module_ptr;

      // continue with modules which support DM extn but disabled the dm handling.
      if (!module_ptr->flags.need_dm_extn || gen_topo_is_dm_enabled(module_ptr))
      {
         continue;
      }

      bool_t is_in_variable_path = FALSE;
      for (gu_output_port_list_t *out_port_list_ptr = module_ptr->gu.output_port_list_ptr; NULL != out_port_list_ptr;
           LIST_ADVANCE(out_port_list_ptr))
      {
         gen_topo_output_port_t *out_port_ptr = (gen_topo_output_port_t *)out_port_list_ptr->op_port_ptr;

         is_in_variable_path |= gen_topo_is_out_port_in_dm_variable_nblc(topo_ptr, out_port_ptr);
      }

      for (gu_input_port_list_t *in_port_list_ptr = module_ptr->gu.input_port_list_ptr; NULL != in_port_list_ptr;
           LIST_ADVANCE(in_port_list_ptr))
      {
         gen_topo_input_port_t *in_port_ptr = (gen_topo_input_port_t *)in_port_list_ptr->ip_port_ptr;

         is_in_variable_path |= gen_topo_is_in_port_in_dm_variable_nblc(topo_ptr, in_port_ptr);
      }

      if (is_in_variable_path)
      {
         result |= gen_topo_update_module_dm_mode(topo_ptr, module_ptr);
      }
   }

   // update varaible input/output info for the external ports based on the latest dm modes.
   VERIFY(result, topo_ptr->topo_to_cntr_vtable_ptr->update_icb_info);
   topo_ptr->topo_to_cntr_vtable_ptr->update_icb_info(topo_ptr);

   CATCH(result, TOPO_MSG_PREFIX, topo_ptr->gu.log_id)
   {
   }

   return result;
}

ar_result_t gen_topo_handle_dm_disable_event(gen_topo_t *                      topo_ptr,
                                             gen_topo_module_t *               module_ptr,
                                             capi_event_data_to_dsp_service_t *event_data_ptr)
{
   fwk_extn_dm_event_id_disable_dm_t *cfg_ptr = (fwk_extn_dm_event_id_disable_dm_t *)event_data_ptr->payload.data_ptr;

   TOPO_MSG(topo_ptr->gu.log_id,
            DBG_HIGH_PRIO,
            "DM: Received disable=%lu CB from miid 0x%lX",
            cfg_ptr->disabled,
            module_ptr->gu.module_instance_id);

   module_ptr->flags.is_dm_disabled = cfg_ptr->disabled;

   if (module_ptr->flags.is_dm_disabled)
   {
      module_ptr->flags.dm_mode = GEN_TOPO_DM_INVALID_MODE;
   }

   // update container level dm enabled flag.
   gen_topo_update_dm_enabled_flag(topo_ptr);

   return AR_EOK;
}

#define RESULT_MASK_IS_THRESHOLD_MODULE_PRESENT 0x0001
#define RESULT_MASK_IS_STM_MODULE_PRESENT 0x0002

// Recursively searches downstream of the current modules output port for threshold and STM module.
static ar_result_t gen_topo_search_for_threshold_module_downstream(gen_topo_t *            topo_ptr,
                                                                   gen_topo_output_port_t *out_port_ptr,
                                                                   uint32_t *              result_mask)
{
   ar_result_t result = AR_EOK;
   if (NULL == out_port_ptr->gu.conn_in_port_ptr)
   {
      return result;
   }

   gen_topo_module_t *    next_module_ptr  = (gen_topo_module_t *)out_port_ptr->gu.conn_in_port_ptr->cmn.module_ptr;
   gen_topo_input_port_t *next_in_port_ptr = (gen_topo_input_port_t *)out_port_ptr->gu.conn_in_port_ptr;

   if (next_module_ptr->flags.need_stm_extn)
   {
      *result_mask = *result_mask | RESULT_MASK_IS_STM_MODULE_PRESENT;
#ifdef DM_EXT_DEBUG
      TOPO_MSG(topo_ptr->gu.log_id,
               DBG_LOW_PRIO,
               "Found a STM module miid 0x%lX in the downstream",
               next_module_ptr->gu.module_instance_id);
#endif
   }
   else if (next_in_port_ptr->common.flags.port_has_threshold || next_module_ptr->flags.need_sync_extn)
   {
      *result_mask = *result_mask | RESULT_MASK_IS_THRESHOLD_MODULE_PRESENT;

#ifdef DM_EXT_DEBUG
      TOPO_MSG(topo_ptr->gu.log_id,
               DBG_LOW_PRIO,
               "Found a threshold module miid 0x%lX in the downstream",
               next_module_ptr->gu.module_instance_id);
#endif
   }

   // search recursively from next modules output ports.
   for (gu_output_port_list_t *out_port_list_ptr = next_module_ptr->gu.output_port_list_ptr; NULL != out_port_list_ptr;
        LIST_ADVANCE(out_port_list_ptr))
   {
      gen_topo_output_port_t *next_out_port_ptr = (gen_topo_output_port_t *)out_port_list_ptr->op_port_ptr;

      result |= gen_topo_search_for_threshold_module_downstream(topo_ptr, next_out_port_ptr, result_mask);
   }

   return result;
}

// Recursively searches downstream of the current modules output port for threshold and STM module.
static ar_result_t gen_topo_search_for_threshold_module_upstream(gen_topo_t *           topo_ptr,
                                                                 gen_topo_input_port_t *in_port_ptr,
                                                                 uint32_t *             result_mask)
{
   ar_result_t result = AR_EOK;
   if (NULL == in_port_ptr->gu.conn_out_port_ptr)
   {
      return result;
   }

   gen_topo_module_t *     prev_module_ptr   = (gen_topo_module_t *)in_port_ptr->gu.conn_out_port_ptr->cmn.module_ptr;
   gen_topo_output_port_t *prev_out_port_ptr = (gen_topo_output_port_t *)in_port_ptr->gu.conn_out_port_ptr;

   if (prev_module_ptr->flags.need_stm_extn)
   {
      *result_mask = *result_mask | RESULT_MASK_IS_STM_MODULE_PRESENT;

#ifdef DM_EXT_DEBUG
      TOPO_MSG(topo_ptr->gu.log_id,
               DBG_LOW_PRIO,
               "Found a STM module miid 0x%lX in the upstream",
               prev_module_ptr->gu.module_instance_id);
#endif
   }
   else if (prev_out_port_ptr->common.flags.port_has_threshold || prev_module_ptr->flags.need_sync_extn)
   {
      *result_mask = *result_mask | RESULT_MASK_IS_THRESHOLD_MODULE_PRESENT;

#ifdef DM_EXT_DEBUG
      TOPO_MSG(topo_ptr->gu.log_id,
               DBG_LOW_PRIO,
               "Found a threshold module miid 0x%lX in the upstream",
               prev_module_ptr->gu.module_instance_id);
#endif
   }

   // search recursively from prev modules input ports.
   for (gu_input_port_list_t *in_port_list_ptr = prev_module_ptr->gu.input_port_list_ptr; NULL != in_port_list_ptr;
        LIST_ADVANCE(in_port_list_ptr))
   {
      gen_topo_input_port_t *prev_in_port_ptr = (gen_topo_input_port_t *)in_port_list_ptr->ip_port_ptr;

      result |= gen_topo_search_for_threshold_module_upstream(topo_ptr, prev_in_port_ptr, result_mask);
   }

   return result;
}

// Recursively searches downstream of the current modules output port for threshold and STM module.
ar_result_t gen_topo_determine_dm_mode_of_module(gen_topo_t *        topo_ptr,
                                                 gen_topo_module_t * module_ptr,
                                                 fwk_extn_dm_mode_t *dm_mode)
{
   ar_result_t result = AR_EOK;

   // search recursively from next modules ouptut ports.
   uint32_t downstream_search_result = 0;
   for (gu_output_port_list_t *out_port_list_ptr = module_ptr->gu.output_port_list_ptr; NULL != out_port_list_ptr;
        LIST_ADVANCE(out_port_list_ptr))
   {
      gen_topo_output_port_t *next_out_port_ptr = (gen_topo_output_port_t *)out_port_list_ptr->op_port_ptr;

      result |= gen_topo_search_for_threshold_module_downstream(topo_ptr, next_out_port_ptr, &downstream_search_result);
   }

   // search recursively from next modules ouptut ports.
   uint32_t upstream_search_result = 0;
   for (gu_input_port_list_t *in_port_list_ptr = module_ptr->gu.input_port_list_ptr; NULL != in_port_list_ptr;
        LIST_ADVANCE(in_port_list_ptr))
   {
      gen_topo_input_port_t *prev_in_port_ptr = (gen_topo_input_port_t *)in_port_list_ptr->ip_port_ptr;

      result |= gen_topo_search_for_threshold_module_upstream(topo_ptr, prev_in_port_ptr, &upstream_search_result);
   }

#ifdef DM_EXT_DEBUG
   TOPO_MSG(topo_ptr->gu.log_id,
            DBG_LOW_PRIO,
            "Search result masks of module miid 0x%x downstream=0x%lx, upstream=0x%lx",
            module_ptr->gu.module_instance_id,
            downstream_search_result,
            upstream_search_result);
#endif

   // Assign dm mode based on following,
   // 1. If STM module is present upstream then f/i
   // 2. else if STM module is present downstream then f/o
   // 3. else if threshold module is present upstream then f/i
   // 4. else if threshold module is present downstream then f/o
   // 5. Else choose fixed-in mode as it is simpler for container and module to manage.
   if (downstream_search_result & RESULT_MASK_IS_STM_MODULE_PRESENT)
   {
      *dm_mode = FWK_EXTN_DM_FIXED_OUTPUT_MODE;
   }
   else if (upstream_search_result & RESULT_MASK_IS_STM_MODULE_PRESENT)
   {
      *dm_mode = FWK_EXTN_DM_FIXED_INPUT_MODE;
   }
   else if (upstream_search_result & RESULT_MASK_IS_THRESHOLD_MODULE_PRESENT)
   {
      *dm_mode = FWK_EXTN_DM_FIXED_INPUT_MODE;
   }
   else if (downstream_search_result & RESULT_MASK_IS_THRESHOLD_MODULE_PRESENT)
   {
      *dm_mode = FWK_EXTN_DM_FIXED_OUTPUT_MODE;
   }
   else
   {
      /* by default, set fixed-in.
       * container can not support multiple fixed-out module but it can handled fixed-in.
       */
      *dm_mode = FWK_EXTN_DM_FIXED_INPUT_MODE;
   }

#ifdef DM_EXT_DEBUG
   TOPO_MSG(topo_ptr->gu.log_id,
            DBG_LOW_PRIO,
            "Assigning dm mode %lu to module miid 0x%x",
            *dm_mode,
            module_ptr->gu.module_instance_id);
#endif

   return result;
}

// function to inform fixed-out dm module to allow partial input due to threshold disables state.
ar_result_t gen_topo_send_dm_consume_partial_input(gen_topo_t *       topo_ptr,
                                                   gen_topo_module_t *module_ptr,
                                                   bool_t             should_consume_partial_input)
{
   ar_result_t err_code = AR_EOK;
   // if module is not DM module then return
   if (!module_ptr->flags.need_dm_extn)
   {
      return AR_EOK;
   }

   fwk_extn_dm_param_id_consume_partial_input_t param = { .should_consume_partial_input =
                                                             should_consume_partial_input };

   TOPO_MSG(topo_ptr->gu.log_id,
            DBG_HIGH_PRIO,
            "Sending consume_partial_input %ld to dm fwk extn module miid 0x%lx.",
            should_consume_partial_input,
            module_ptr->gu.module_instance_id);

   err_code = gen_topo_capi_set_param(topo_ptr->gu.log_id,
                                      module_ptr->capi_ptr,
                                      FWK_EXTN_DM_PARAM_ID_CONSUME_PARTIAL_INPUT,
                                      (int8_t *)&param,
                                      sizeof(param));

   if ((err_code != AR_EOK) && (err_code != AR_EUNSUPPORTED))
   {
      TOPO_MSG(topo_ptr->gu.log_id,
               DBG_ERROR_PRIO,
               "Module 0x%lX: setting consume_partial_input to dm fwk extn modules failed err code %d",
               module_ptr->gu.module_instance_id,
               err_code);
      return err_code;
   }

   return err_code;
}
