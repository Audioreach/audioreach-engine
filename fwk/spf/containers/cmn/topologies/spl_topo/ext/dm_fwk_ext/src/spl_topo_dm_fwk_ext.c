/**
 * \file spl_topo_tdm.c
 *
 * \brief
 *
 *     Topo 2 functions for managing dm modes for fractional resampling cases
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "spl_topo_dm_fwk_ext.h"
#include "spl_topo_i.h"
//#define TOPO_DM_DEBUG

#if (SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4) || defined(TOPO_DM_DEBUG)
#define DM_MSG_VERBOSE TOPO_MSG
#else
#define DM_MSG_VERBOSE(...) {}
#endif

/* =======================================================================
 Static Function Definitions
 ========================================================================== */

/* =======================================================================
 Function Definitions
 ========================================================================== */
/* Checks if a particular module has enabled/disabled dm mode */
bool_t spl_topo_fwk_ext_is_dm_enabled(spl_topo_module_t *module_ptr)
{
   return (module_ptr->t_base.flags.need_dm_extn) && !(module_ptr->t_base.flags.is_dm_disabled);
}

/* Checks if a particular module is in fixed out dm mode */
bool_t spl_topo_fwk_ext_is_module_dm_fixed_out(spl_topo_module_t *module_ptr)
{
   return spl_topo_fwk_ext_is_dm_enabled(module_ptr) && (GEN_TOPO_DM_FIXED_OUTPUT_MODE == module_ptr->t_base.flags.dm_mode);
}

/* Checks if a particular module is in fixed in dm mode */
bool_t spl_topo_fwk_ext_is_module_dm_fixed_in(spl_topo_module_t *module_ptr)
{
   return spl_topo_fwk_ext_is_dm_enabled(module_ptr) && (GEN_TOPO_DM_FIXED_INPUT_MODE == module_ptr->t_base.flags.dm_mode);
}

/* Frees the rquired smaples calculated if dm modules are present */
void spl_topo_fwk_ext_free_dm_req_samples(spl_topo_t *topo_ptr)
{
   MFREE_NULLIFY(topo_ptr->fwk_extn_info.dm_info.dm_req_samples_ptr);
}

//function to send the dm mode to the module.
static ar_result_t spl_topo_fwk_ext_update_module_dm_mode(spl_topo_t *       topo_ptr,
                                                          spl_topo_module_t *module_ptr,
                                                          fwk_extn_dm_mode_t dm_mode)
{
   ar_result_t result = AR_EOK;
   if (!module_ptr->t_base.flags.need_dm_extn) // don't check for dm-disabled flag.
   {
      return AR_EOK;
   }

   fwk_extn_dm_param_id_change_mode_t dm_mode_param = {.dm_mode = dm_mode};

   if ((FWK_EXTN_DM_FIXED_INPUT_MODE != dm_mode_param.dm_mode) && (FWK_EXTN_DM_FIXED_OUTPUT_MODE != dm_mode_param.dm_mode))
   {
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_ERROR_PRIO,
               "module inst id 0x%lx. trying to set invalid dm mode. %d",
               module_ptr->t_base.gu.module_instance_id,
               dm_mode_param.dm_mode);
      return AR_EFAILED;
   }

   // reset the dm mode for each module.
   module_ptr->t_base.flags.dm_mode = GEN_TOPO_DM_INVALID_MODE;

   // temporarily setting this flag to false to avoid recursive call to this function
   module_ptr->t_base.flags.need_dm_extn = FALSE;

   result = gen_topo_capi_set_param(topo_ptr->t_base.gu.log_id,
                                    module_ptr->t_base.capi_ptr,
                                    FWK_EXTN_DM_PARAM_ID_CHANGE_MODE,
                                    (int8_t *)&dm_mode_param,
                                    sizeof(dm_mode_param));

   module_ptr->t_base.flags.need_dm_extn = TRUE;

   if (AR_DID_FAIL(result) || module_ptr->t_base.flags.is_dm_disabled)
   {
      module_ptr->t_base.flags.dm_mode = GEN_TOPO_DM_INVALID_MODE;
      result                           = AR_EOK;
   }
   else // if module accepts DM mode, cache dm_info
   {
      if (FWK_EXTN_DM_FIXED_INPUT_MODE == dm_mode_param.dm_mode)
      {
         module_ptr->t_base.flags.dm_mode = GEN_TOPO_DM_FIXED_INPUT_MODE;
         topo_ptr->fwk_extn_info.dm_info.present_dm_modules |= SPL_TOPO_FIXED_INPUT_DM_MODULE;
      }
      else
      {
         module_ptr->t_base.flags.dm_mode = GEN_TOPO_DM_FIXED_OUTPUT_MODE;
         topo_ptr->fwk_extn_info.dm_info.present_dm_modules |= SPL_TOPO_FIXED_OUTPUT_DM_MODULE;
      }
   }

   TOPO_MSG(topo_ptr->t_base.gu.log_id,
            DBG_MED_PRIO,
            "Setting module inst id 0x%lx to dm mode %d (0=invalid,1=f/i,2=f/o)",
            module_ptr->t_base.gu.module_instance_id,
            module_ptr->t_base.flags.dm_mode);

   return result;
}

/*
 Handling for event dm report samples raised by the module
 */
ar_result_t spl_topo_fwk_ext_handle_dm_report_samples_event(spl_topo_t *                      topo_ptr,
                                                            spl_topo_module_t *               module_ptr,
                                                            capi_event_data_to_dsp_service_t *event_data_ptr,
                                                            bool_t                            is_max)
{
   ar_result_t                         result      = AR_EOK;
   bool_t                              IS_MAX_TRUE = TRUE;
   fwk_extn_dm_param_id_req_samples_t *req_samples_ptr =
      (fwk_extn_dm_param_id_req_samples_t *)event_data_ptr->payload.data_ptr;

   DM_MSG_VERBOSE(topo_ptr->t_base.gu.log_id,
                  DBG_HIGH_PRIO,
                  "Received report CB from miid 0x%lX, is_max %lu",
                  module_ptr->t_base.gu.module_instance_id,
                  is_max);

   if (spl_topo_fwk_ext_is_module_dm_fixed_out(module_ptr) && (TRUE == req_samples_ptr->is_input))
   {
      // update samples on input ports
      for (uint32_t rep_index = 0; rep_index < req_samples_ptr->num_ports; rep_index++)
      {
         spl_topo_input_port_t *inport_ptr =
            (spl_topo_input_port_t *)gu_find_input_port_by_index(&module_ptr->t_base.gu,
                                                                 req_samples_ptr->req_samples[rep_index].port_index);
         if (inport_ptr)
         {
            spl_topo_req_samples_t *dm_info_ptr =
               (is_max) ? &inport_ptr->req_samples_info_max : &inport_ptr->req_samples_info;

            // if sample requirement is unchanged, there is no need to traverse further
            // This optimization causes wrongly waiting on ext input buffer for some cases - so commenting
            // if (dm_info_ptr->required_samples_in != req_samples_ptr->req_samples[rep_index].samples_per_channel)
            {

               DM_MSG_VERBOSE(topo_ptr->t_base.gu.log_id,
                              DBG_HIGH_PRIO,
                              "Required_samples_in set to %lu on index %lu",
                              req_samples_ptr->req_samples[rep_index].samples_per_channel,
                              req_samples_ptr->req_samples[rep_index].port_index);

               dm_info_ptr->samples_in = req_samples_ptr->req_samples[rep_index].samples_per_channel;
               dm_info_ptr->is_updated = TRUE;
               if (is_max)
               {
                  // Initiate traversal here if not already ongoing
                  if (!topo_ptr->during_max_traversal)
                  {
                     spl_topo_get_required_input_samples(topo_ptr, IS_MAX_TRUE);
                  }
               }
               else
               {
                  // nothing to do here, will be handled from get_required_samples
               }
            }
         }
         else
         {
            TOPO_MSG(topo_ptr->t_base.gu.log_id,
                     DBG_HIGH_PRIO,
                     "Warning: Module 0x%lX: Samples reported for invalid index %lu",
                     module_ptr->t_base.gu.module_instance_id,
                     req_samples_ptr->req_samples[rep_index].port_index);
         }
      }
   }
   else if ((GEN_TOPO_DM_FIXED_INPUT_MODE == module_ptr->t_base.flags.dm_mode) && (FALSE == req_samples_ptr->is_input))
   {
      // handle fixed input
   }
   else
   {
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_ERROR_PRIO,
               "Module 0x%lX: Incorrect reported samples is_input %lu, mode %lu",
               module_ptr->t_base.gu.module_instance_id,
               req_samples_ptr->is_input,
               module_ptr->t_base.flags.dm_mode);
      return AR_EFAILED;
   }
   return result;
}

/*
 * Checks if module is dm and sends required samples info to module, depending on what has been calculated in fwk
 */
ar_result_t spl_topo_fwk_ext_set_dm_samples_per_module(spl_topo_t *                        topo_ptr,
                                                       spl_topo_module_t *                 module_ptr,
                                                       fwk_extn_dm_param_id_req_samples_t *dm_req_samples_ptr,
                                                       bool_t                              is_max)
{
   ar_result_t result = AR_EOK;

   if (dm_req_samples_ptr && dm_req_samples_ptr->num_ports)
   {
      // Issue set param
      uint32_t param_id = (is_max) ? FWK_EXTN_DM_PARAM_ID_SET_MAX_SAMPLES : FWK_EXTN_DM_PARAM_ID_SET_SAMPLES;
      result            = gen_topo_capi_set_param(topo_ptr->t_base.gu.log_id,
                                       module_ptr->t_base.capi_ptr,
                                       param_id,
                                       (int8_t *)dm_req_samples_ptr,
                                       sizeof(fwk_extn_dm_param_id_req_samples_t) +
                                          (dm_req_samples_ptr->num_ports - 1) * sizeof(fwk_extn_dm_port_samples_t));
      if (AR_ENOTREADY == result)
      {
         TOPO_MSG(topo_ptr->t_base.gu.log_id, DBG_HIGH_PRIO, "Input media format not received yet, returning");
         // In this case we aren't using DM so we should clear dm samples.
         topo_ptr->t_base.topo_to_cntr_vtable_ptr->clear_topo_req_samples(&(topo_ptr->t_base), FALSE /*IS_MAX_FALSE*/);
         return AR_EOK;
      }
   }
   return result;
}

/*
 Mark a flag when module rasies an event to disable dm
 */
ar_result_t spl_topo_fwk_ext_handle_dm_disable_event(spl_topo_t *                      topo_ptr,
                                                     spl_topo_module_t *               module_ptr,
                                                     capi_event_data_to_dsp_service_t *event_data_ptr)
{
   ar_result_t result = AR_EOK;

   fwk_extn_dm_event_id_disable_dm_t *cfg_ptr = (fwk_extn_dm_event_id_disable_dm_t *)event_data_ptr->payload.data_ptr;

   TOPO_MSG(topo_ptr->t_base.gu.log_id,
            DBG_HIGH_PRIO,
            "Received disable=%lu CB from miid 0x%lX",
            cfg_ptr->disabled,
            module_ptr->t_base.gu.module_instance_id);

   /* no need to update nblc as all the dm module requires data buffering and will be considered for nblc during init*/
   if (module_ptr->t_base.flags.is_dm_disabled != cfg_ptr->disabled)
   {
      module_ptr->t_base.flags.is_dm_disabled = cfg_ptr->disabled;

      // protection check to avoid recursive call.
      if (module_ptr->t_base.flags.need_dm_extn)
      {
         /* Need to update dm modes when any module disables dm mode */
         result |= spl_topo_fwk_ext_update_dm_modes(topo_ptr);
      }
   }

   return result;
}

//Check if a module requires fixed input and output to process
static bool_t does_module_requires_fixed_in_out(spl_topo_module_t *module_ptr)
{
   /* Non-SISO modules are considered fixed in-out because all port should have same data.
    * Trigger policy modules are also considered fixed in-out (although they may not) because we can't estimate samples
    * from output to input ports (usually there is some internal buffering in TGP modules). So to simplify the required
    * samples query for fixed-out DM module, we need to start the query from the input port of TGP module.
    */
   /* Unlike nblc end check, this doesn't include requires-data-buffering module (another dm module)
    */
   if ((!module_ptr->t_base.gu.flags.is_siso) || (module_ptr->threshold_data.is_threshold_module) ||
       (module_ptr->t_base.flags.need_sync_extn) || module_ptr->t_base.flags.need_trigger_policy_extn)
   {
      return TRUE;
   }
   return FALSE;
}

// Recursively searches upstream of the current module's input port to determine dm mode.
static ar_result_t spl_topo_is_ip_port_in_dm_variable_path(spl_topo_t *topo_ptr, spl_topo_input_port_t *ip_port_ptr)
{
   // If there is no internal connection from this input port/input port's nblc start then there is no module in the
   // upstream which generates variable output
   if ((NULL == ip_port_ptr->t_base.gu.conn_out_port_ptr) || (NULL == ip_port_ptr->t_base.nblc_start_ptr) ||
       (NULL == ip_port_ptr->t_base.nblc_start_ptr->gu.conn_out_port_ptr))
   {
      return FALSE;
   }

   spl_topo_module_t *prev_module_ptr =
      (spl_topo_module_t *)ip_port_ptr->t_base.nblc_start_ptr->gu.conn_out_port_ptr->cmn.module_ptr;

   if (spl_topo_fwk_ext_is_dm_enabled(prev_module_ptr))
   {
      // if upstream has a fixed output module then it is not a variable data path.
      // if upstream has a fixed input module then there can be variable data generated on its output.
      return (GEN_TOPO_DM_FIXED_OUTPUT_MODE == prev_module_ptr->t_base.flags.dm_mode) ? FALSE : TRUE;
   }
   else if (does_module_requires_fixed_in_out(prev_module_ptr))
   {
      // if upstream has a threshold/threshold-like module then this can't be a variable data path.
      return FALSE;
   }
   else
   {
      // if there is any other module breaking the input nblc then it can cause internal buffering and result in
      // variable input behavior
      return TRUE;
   }
}

// Recursively searches downstream of the current module's output port to determine if it is a variable data path.
static bool_t spl_topo_is_out_port_in_dm_variable_path(spl_topo_t *topo_ptr, spl_topo_output_port_t *out_port_ptr)
{
   // If there is no internal connection from this output port/output port's nblc end then there is no module in the
   // downstream which requires variable input.
   if ((NULL == out_port_ptr->t_base.gu.conn_in_port_ptr) || (NULL == out_port_ptr->t_base.nblc_end_ptr) ||
       (NULL == out_port_ptr->t_base.nblc_end_ptr->gu.conn_in_port_ptr))
   {
      return FALSE;
   }

   spl_topo_module_t *next_module_ptr =
      (spl_topo_module_t *)out_port_ptr->t_base.nblc_end_ptr->gu.conn_in_port_ptr->cmn.module_ptr;

   if (spl_topo_fwk_ext_is_dm_enabled(next_module_ptr))
   {
      // if downstream has a fixed input module then it is not a variable data path.
      // if downstream has a fixed output module then there can be variable data requirement on its input.
      return (GEN_TOPO_DM_FIXED_INPUT_MODE == next_module_ptr->t_base.flags.dm_mode) ? FALSE : TRUE;
   }
   else if (does_module_requires_fixed_in_out(next_module_ptr))
   {
      // if downstream has a threshold/threshold-like module then this can't be a variable data path.
      return FALSE;
   }
   else
   {
      // if there is any other module breaking the output nblc then it can cause internal buffering and result in
      // variable output behavior
      return TRUE;
   }
}

// Recursively searches downstream of the current module's output port to find threshold/threshold-like module
static ar_result_t spl_topo_search_downstream_for_threshold_module(spl_topo_t *            topo_ptr,
                                                                   spl_topo_output_port_t *out_port_ptr,
                                                                   bool_t *                b_found_ptr)
{
   ar_result_t result = AR_EOK;
   // If there is no internal connection from this output port/output port's nblc end then there is no module in the
   // downstream which has threshold/threshold-like
   if ((NULL == out_port_ptr->t_base.gu.conn_in_port_ptr) || (NULL == out_port_ptr->t_base.nblc_end_ptr) ||
       (NULL == out_port_ptr->t_base.nblc_end_ptr->gu.conn_in_port_ptr))
   {
      return result;
   }

   spl_topo_module_t *next_module_ptr =
      (spl_topo_module_t *)out_port_ptr->t_base.nblc_end_ptr->gu.conn_in_port_ptr->cmn.module_ptr;

   // check if module requires to work either on threshold or container frame length.
   if (does_module_requires_fixed_in_out(next_module_ptr))
   {
      *b_found_ptr |= TRUE;

      DM_MSG_VERBOSE(topo_ptr->t_base.gu.log_id,
                     DBG_LOW_PRIO,
                     "Found a fixed input module miid 0x%lX in the downstream of miid 0x%x",
                     next_module_ptr->t_base.gu.module_instance_id,
                     out_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id);

      return result;
   }

   // search recursively from next modules output ports.
   for (gu_output_port_list_t *out_port_list_ptr = next_module_ptr->t_base.gu.output_port_list_ptr;
        NULL != out_port_list_ptr;
        LIST_ADVANCE(out_port_list_ptr))
   {
      spl_topo_output_port_t *next_out_port_ptr = (spl_topo_output_port_t *)out_port_list_ptr->op_port_ptr;

      result |= spl_topo_search_downstream_for_threshold_module(topo_ptr, next_out_port_ptr, b_found_ptr);
   }

   return result;
}

// Recursively searches upstream of the current module's input port to find threshold/threshold-like module
static ar_result_t spl_topo_search_upstream_for_threshold_module(spl_topo_t *           topo_ptr,
                                                                    spl_topo_input_port_t *ip_port_ptr,
                                                                    bool_t *               b_found_ptr)
{
   ar_result_t result = AR_EOK;
   // If there is no internal connection from this input port/input port's nblc start then there is no module in the
   // upstream which has threshold/threshold-like
   if ((NULL == ip_port_ptr->t_base.gu.conn_out_port_ptr) || (NULL == ip_port_ptr->t_base.nblc_start_ptr) ||
       (NULL == ip_port_ptr->t_base.nblc_start_ptr->gu.conn_out_port_ptr))
   {
      return result;
   }

   spl_topo_module_t *prev_module_ptr =
      (spl_topo_module_t *)ip_port_ptr->t_base.nblc_start_ptr->gu.conn_out_port_ptr->cmn.module_ptr;

   //check if module requires to work either on threshold or container frame length.
   if (does_module_requires_fixed_in_out(prev_module_ptr))
   {
      *b_found_ptr |= TRUE;

      DM_MSG_VERBOSE(topo_ptr->t_base.gu.log_id,
                     DBG_LOW_PRIO,
                     "Found a fixed output module miid 0x%lX in the upstream of miid 0x%x",
                     prev_module_ptr->t_base.gu.module_instance_id,
                     ip_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id);

      return result;
   }

   // search recursively from next modules output ports.
   for (gu_input_port_list_t *ip_port_list_ptr = prev_module_ptr->t_base.gu.input_port_list_ptr;
        NULL != ip_port_list_ptr;
        LIST_ADVANCE(ip_port_list_ptr))
   {
      spl_topo_input_port_t *prev_ip_port_ptr = (spl_topo_input_port_t *)ip_port_list_ptr->ip_port_ptr;

      result |= spl_topo_search_upstream_for_threshold_module(topo_ptr, prev_ip_port_ptr, b_found_ptr);
   }

   return result;
}

// Recursively searches downstream of the current modules output port to determine dm mode
ar_result_t spl_topo_determine_dm_mode_of_module(spl_topo_t *        topo_ptr,
                                                 spl_topo_module_t * module_ptr,
                                                 fwk_extn_dm_mode_t *dm_mode)
{
   ar_result_t result          = AR_EOK;
   bool_t      is_output_fixed = FALSE;
   bool_t      is_input_fixed  = FALSE;

   // search recursively from next modules
   for (gu_output_port_list_t *out_port_list_ptr = module_ptr->t_base.gu.output_port_list_ptr;
        NULL != out_port_list_ptr;
        LIST_ADVANCE(out_port_list_ptr))
   {
      spl_topo_output_port_t *next_out_port_ptr = (spl_topo_output_port_t *)out_port_list_ptr->op_port_ptr;

      //if there is a module in downstream which is threshold/mimo/tgp module then output from this DM module should be fixed.
      result |= spl_topo_search_downstream_for_threshold_module(topo_ptr, next_out_port_ptr, &is_output_fixed);
   }

   // search recursively from previous modules
   for (gu_input_port_list_t *in_port_list_ptr = module_ptr->t_base.gu.input_port_list_ptr; NULL != in_port_list_ptr;
        LIST_ADVANCE(in_port_list_ptr))
   {
      spl_topo_input_port_t *prev_in_port_ptr = (spl_topo_input_port_t *)in_port_list_ptr->ip_port_ptr;

      //if there is a module in upstream which is threshold/mimo/tgp module then input to this module will be fixed.
      result |= spl_topo_search_upstream_for_threshold_module(topo_ptr, prev_in_port_ptr, &is_input_fixed);
   }

   DM_MSG_VERBOSE(topo_ptr->t_base.gu.log_id,
                  DBG_LOW_PRIO,
                  "Search result module miid 0x%x is_output_fixed=%d, is_input_fixed=%d",
                  module_ptr->t_base.gu.module_instance_id,
                  is_output_fixed,
                  is_input_fixed);

   // assign default dm mode.
   if ((APM_SUB_GRAPH_SID_VOICE_CALL == module_ptr->t_base.gu.sg_ptr->sid) &&
       (APM_SUB_GRAPH_DIRECTION_RX == module_ptr->t_base.gu.sg_ptr->direction))
   {
      /* In voice rx path we don't want any data to buffer in PP because this will lead to underruns at EPC.
       * Therefore we are fixing all dm mode to fixed_in for voice rx use case.
       */
      *dm_mode = FWK_EXTN_DM_FIXED_INPUT_MODE;
   }
   else
   {
      if (APM_SUB_GRAPH_DIRECTION_TX == module_ptr->t_base.gu.sg_ptr->direction)
      {
         *dm_mode = FWK_EXTN_DM_FIXED_INPUT_MODE;
      }
      else
      {
         *dm_mode = FWK_EXTN_DM_FIXED_OUTPUT_MODE;
      }
   }

   if (is_output_fixed && is_input_fixed) //print a warning message and keep the fixed-input
   {
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_ERROR_PRIO,
               "Warning! DM module miid 0x%x is_in fixed input and fixed output path",
               module_ptr->t_base.gu.module_instance_id);

      *dm_mode = FWK_EXTN_DM_FIXED_INPUT_MODE;
   }
   else if (is_input_fixed)
   {
      *dm_mode = FWK_EXTN_DM_FIXED_INPUT_MODE;
   }
   else if (is_output_fixed)
   {
      *dm_mode = FWK_EXTN_DM_FIXED_OUTPUT_MODE;
   }

   DM_MSG_VERBOSE(topo_ptr->t_base.gu.log_id,
                  DBG_LOW_PRIO,
                  "Assigning dm mode %lu to module miid 0x%x",
                  *dm_mode,
                  module_ptr->t_base.gu.module_instance_id);

   return result;
}

static ar_result_t spl_topo_update_module_dm_mode(spl_topo_t *topo_ptr, spl_topo_module_t *module_ptr)
{
   ar_result_t        result           = AR_EOK;
   fwk_extn_dm_mode_t fwk_extn_dm_mode = FWK_EXTN_DM_INVALID_MODE;
   if (!module_ptr->t_base.flags.need_dm_extn) // don't check for dm-disabled flag.
   {
      return AR_EOK;
   }

   // determine dm mode.
   result = spl_topo_determine_dm_mode_of_module(topo_ptr, module_ptr, &fwk_extn_dm_mode);
   if (AR_FAILED(result) || (FWK_EXTN_DM_INVALID_MODE == fwk_extn_dm_mode))
   {
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_ERROR_PRIO,
               "Failed determining DM mode of the DM module iid 0x%lx, not setting dm mode",
               module_ptr->t_base.gu.module_instance_id);
      return AR_EFAILED;
   }

   return spl_topo_fwk_ext_update_module_dm_mode(topo_ptr, module_ptr, fwk_extn_dm_mode);
}

// Recursively searches downstream of the DM module's output port to find required-sample-query-start module
static ar_result_t spl_topo_find_req_sample_query_start_modules_from_output_port(spl_topo_t *            topo_ptr,
                                                                                 spl_topo_output_port_t *out_port_ptr)
{
   ar_result_t        result                            = AR_EOK;
   spl_topo_module_t *req_sample_query_start_module_ptr = NULL;
   spl_topo_module_t *next_module_ptr = NULL;

   if (NULL == out_port_ptr->t_base.gu.conn_in_port_ptr)
   {
      // [ [M1]->[DM]-> ]
      // if output port is external output then assign the self module as query-start-module.
      req_sample_query_start_module_ptr = (spl_topo_module_t *)out_port_ptr->t_base.gu.cmn.module_ptr;
   }
   else if (NULL == out_port_ptr->t_base.nblc_end_ptr)
   {
      // output nblc-end shouldn't be null. it should be same as external output port if there is no nblc-end module.
      return result;
   }
   else if (NULL == out_port_ptr->t_base.nblc_end_ptr->gu.conn_in_port_ptr)
   {
      // [ [M1]->[DM]->[M2]-> ]
      // if output port's nblc end is an external output port then assign the external port's module as
      // query-start-module
      req_sample_query_start_module_ptr = (spl_topo_module_t *)out_port_ptr->t_base.nblc_end_ptr->gu.cmn.module_ptr;
   }
   else
   {
      next_module_ptr = (spl_topo_module_t *)out_port_ptr->t_base.nblc_end_ptr->gu.conn_in_port_ptr->cmn.module_ptr;

      if (spl_topo_fwk_ext_is_module_dm_fixed_out(next_module_ptr))
      {
         // if nblc-end module itself is a fixed-out DM module then return, search will continue from this module by
         // caller
         return result;
      }
      else if (spl_topo_fwk_ext_is_module_dm_fixed_in(next_module_ptr))
      {
         // if next nblc-end module is a fixed-in DM module then this module will be query-start-module.
         req_sample_query_start_module_ptr = next_module_ptr;
      }
      else if (does_module_requires_fixed_in_out(next_module_ptr))
      {
         // if next nblc-end module requires to work either on threshold or container frame length
         // then assign this as query-start-module
         // [ [M1]->[DM]->[M2]->[Th]->[M3] ]
         req_sample_query_start_module_ptr = next_module_ptr;
      }
      else
      {
	//siso dm disabled module
      }
   }

   if (req_sample_query_start_module_ptr)
   {
      spf_list_node_t *tmp_node_ptr = NULL;
      if (AR_FAILED(spf_list_find_list_node((spf_list_node_t *)topo_ptr->req_samp_query_start_list_ptr,
                                            req_sample_query_start_module_ptr,
                                            &tmp_node_ptr)))
      {
         spf_list_insert_tail((spf_list_node_t **)&topo_ptr->req_samp_query_start_list_ptr,
                              req_sample_query_start_module_ptr,
                              topo_ptr->t_base.heap_id,
                              TRUE);
         TOPO_MSG(topo_ptr->t_base.gu.log_id,
                  DBG_MED_PRIO,
                  "Adding module 0x%x as req sample query start module.",
                  req_sample_query_start_module_ptr->t_base.gu.module_instance_id);
      }
      return result;
   }

   if (next_module_ptr)
   {
      // search recursively from next modules output ports.
      for (gu_output_port_list_t *out_port_list_ptr = next_module_ptr->t_base.gu.output_port_list_ptr;
           NULL != out_port_list_ptr;
           LIST_ADVANCE(out_port_list_ptr))
      {
         spl_topo_output_port_t *next_out_port_ptr = (spl_topo_output_port_t *)out_port_list_ptr->op_port_ptr;

         result |= spl_topo_find_req_sample_query_start_modules_from_output_port(topo_ptr, next_out_port_ptr);
      }
   }

   return result;
}

static ar_result_t spl_topo_check_add_sync_module_to_req_sample_query_modules_list(spl_topo_t        *topo_ptr,
                                                                                   spl_topo_module_t *module_ptr)
{
   ar_result_t result = AR_EOK;

   for (gu_input_port_list_t *in_port_list_ptr = module_ptr->t_base.gu.input_port_list_ptr; NULL != in_port_list_ptr;
        LIST_ADVANCE(in_port_list_ptr))
   {
      spl_topo_input_port_t *ip_port_ptr = (spl_topo_input_port_t *)in_port_list_ptr->ip_port_ptr;

      if (spl_topo_is_ip_port_in_dm_variable_path(topo_ptr, ip_port_ptr))
      {
         /* sync module input port is not in the nblc path of external input so there could be chance of internal
          * buffering. To avoid this add sync module into req_samp_query_start_list_ptr.
          * */
         spf_list_insert_tail((spf_list_node_t **)&topo_ptr->req_samp_query_start_list_ptr,
                              module_ptr,
                              topo_ptr->t_base.heap_id,
                              TRUE);
         TOPO_MSG(topo_ptr->t_base.gu.log_id,
                  DBG_MED_PRIO,
                  "Adding module 0x%x as req sample query start module.",
                  module_ptr->t_base.gu.module_instance_id);
         break;
      }
   }

   return result;
}

/* This function finds the DM module and determines whether they should be configured in fixed-output mode or fixed
 * input mode.
 * When DM module is configured in fixed-input then module must consume all the input but can generate
 * variable output for each process frame.
 * When DM module is configured in fixed-output then module must generate the expected amount of output samples (set by
 * the fwk) and in order to do that module must report the required input samples to the fwk. Fwk then makes sure that
 * the module gets the required amount of input samples to generate the expected amount of output samples.
 *
 * Whether a module will be configured in fixed-out or fixed-in depends on the upstream and downstream module.
 * If there is a threshold/trigger-policy/mimo module at the output then module must operate in fixed-out mode where
 * expected output samples will be same as the threshold/container-frame-len.
 * If there is a threshold/trigger-policy/mimo module at the input then module must operate in fixed-in mode and it must
 * consume all the data coming from the upstream module.
 *
 * There are module which support DM extension but they may enable or disable DM mode based on the configuration (like
 * MFC, enabled during fractional resampling). If there is a DM module which has disabled the DM-mode but if it is
 * placed in a variable data path then fwk inform this to the module and give another chance to decide if DM should be
 * enabled.
 * Variable data path means, either there is a fixed input module in the upstream (generating variable output) or there
 * is a fixed-output module in downstream (requiring variable input).
 *
 * DM module should be placed in the nblc path of the external ports. There is no point in placing DM module between two
 * threshold modules because it neither can work in fixed-input nor in fixed output mode.
 *
 */
ar_result_t spl_topo_fwk_ext_update_dm_modes(spl_topo_t *topo_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result    = AR_EOK;
   uint32_t    max_ports = 1;

   VERIFY(result, topo_ptr->t_base.topo_to_cntr_vtable_ptr->clear_topo_req_samples);

   // if sorted list isn't setup, nothing to do yet, so return
   if (!topo_ptr->t_base.gu.sorted_module_list_ptr)
   {
      return AR_EOK;
   }

   // Clear dm state, here we will reinitialize.
   topo_ptr->fwk_extn_info.dm_info.present_dm_modules = SPL_TOPO_NO_DM_MODULE;
   topo_ptr->t_base.topo_to_cntr_vtable_ptr->clear_topo_req_samples(&(topo_ptr->t_base), FALSE);

   // Sorted list doesn't contain elementary modules, but elementary modules can't be dm modules.
   for (gu_module_list_t *module_list_ptr = (gu_module_list_t *)topo_ptr->t_base.gu.sorted_module_list_ptr;
        (NULL != module_list_ptr);
        LIST_ADVANCE(module_list_ptr))
   {
      spl_topo_module_t *module_ptr = (spl_topo_module_t *)module_list_ptr->module_ptr;

      max_ports = MAX(max_ports, module_ptr->t_base.gu.num_input_ports);
      max_ports = MAX(max_ports, module_ptr->t_base.gu.num_output_ports);

      //skip the modules where DM is not enabled.
      if (!spl_topo_fwk_ext_is_dm_enabled(module_ptr))
      {
         continue;
      }

      //determine the DM-mode (fixed-in or fixed-out) and set it to module.
      (void)spl_topo_update_module_dm_mode(topo_ptr, module_ptr);
   }

   // dm mode must be enabled if a dm-extn module is in variable data path.
   /* example. MFC doing a conversion from 16KHz to 32KHz will disable the dm mode, but if it is in variable data path
   * then it needs to enable the dm mode again because it should be able to accept variable frame on input.
   */
   for (gu_module_list_t *module_list_ptr = (gu_module_list_t *)topo_ptr->t_base.gu.sorted_module_list_ptr;
        (NULL != module_list_ptr);
        LIST_ADVANCE(module_list_ptr))
   {
      spl_topo_module_t *module_ptr = (spl_topo_module_t *)module_list_ptr->module_ptr;

      //if module doesn't support DM or DM is already enabled then skip to the next module.
      if (!module_ptr->t_base.flags.need_dm_extn || spl_topo_fwk_ext_is_dm_enabled(module_ptr))
      {
         continue;
      }

      //Try to update the DM mode for the module which have disabled DM requirement.
      bool_t is_in_variable_path = FALSE;
      for (gu_output_port_list_t *out_port_list_ptr = module_ptr->t_base.gu.output_port_list_ptr;
           NULL != out_port_list_ptr;
           LIST_ADVANCE(out_port_list_ptr))
      {
         spl_topo_output_port_t *out_port_ptr = (spl_topo_output_port_t *)out_port_list_ptr->op_port_ptr;

         is_in_variable_path |= spl_topo_is_out_port_in_dm_variable_path(topo_ptr, out_port_ptr);
      }

      for (gu_input_port_list_t *in_port_list_ptr = module_ptr->t_base.gu.input_port_list_ptr; NULL != in_port_list_ptr;
           LIST_ADVANCE(in_port_list_ptr))
      {
         spl_topo_input_port_t *in_port_ptr = (spl_topo_input_port_t *)in_port_list_ptr->ip_port_ptr;

         is_in_variable_path |= spl_topo_is_ip_port_in_dm_variable_path(topo_ptr, in_port_ptr);
      }

      //if DM-disabled module is in variable data path then give a chance to update the DM mode
      if (is_in_variable_path)
      {
         result |= spl_topo_update_module_dm_mode(topo_ptr, module_ptr);
      }
   }

   // reallocate dm_info memory if necessary
   if (max_ports > topo_ptr->fwk_extn_info.dm_info.num_ports_dm)
   {
      // set num_ports to zero here. in case malloc fails, values should be consistent
      topo_ptr->fwk_extn_info.dm_info.num_ports_dm = 0;
      MFREE_REALLOC_MEMSET(topo_ptr->fwk_extn_info.dm_info.dm_req_samples_ptr,
                           fwk_extn_dm_param_id_req_samples_t,
                           sizeof(fwk_extn_dm_param_id_req_samples_t) +
                              (max_ports - 1) * sizeof(fwk_extn_dm_port_samples_t),
                           topo_ptr->t_base.heap_id,
                           result);
      topo_ptr->fwk_extn_info.dm_info.num_ports_dm = max_ports;
   }

   // need to find out the modules in the downstream of fixed-output dm module. These modules will be used to start the
   // backward query of required samples at input.

   // delete the list of modules where required sample query starts.
   spf_list_delete_list((spf_list_node_t **)&topo_ptr->req_samp_query_start_list_ptr, TRUE);

   // delete the list of modules where required sample query starts.
   // there can be multiple modules in ths list if there are parallel paths each with a fixed output DM module.
   for (gu_module_list_t *module_list_ptr = (gu_module_list_t *)topo_ptr->t_base.gu.sorted_module_list_ptr;
        (NULL != module_list_ptr);
        LIST_ADVANCE(module_list_ptr))
   {
      spl_topo_module_t *module_ptr = (spl_topo_module_t *)module_list_ptr->module_ptr;

      if (spl_topo_fwk_ext_is_module_dm_fixed_out(module_ptr))
      {
         for (gu_output_port_list_t *out_port_list_ptr = module_ptr->t_base.gu.output_port_list_ptr;
              NULL != out_port_list_ptr;
              LIST_ADVANCE(out_port_list_ptr))
         {
            spl_topo_output_port_t *out_port_ptr = (spl_topo_output_port_t *)out_port_list_ptr->op_port_ptr;

            spl_topo_find_req_sample_query_start_modules_from_output_port(topo_ptr, out_port_ptr);
         }
      }
      else if (module_ptr->t_base.flags.need_sync_extn)
      {
         // if sync module is not already part of the required-sample-query-start-list then check and add.
         spf_list_node_t *tmp_node_ptr = NULL;
         if (AR_FAILED(spf_list_find_list_node((spf_list_node_t *)topo_ptr->req_samp_query_start_list_ptr,
                                               module_ptr,
                                               &tmp_node_ptr)))
         {
            spl_topo_check_add_sync_module_to_req_sample_query_modules_list(topo_ptr, module_ptr);
         }
      }
   }

   topo_ptr->t_base.topo_to_cntr_vtable_ptr->update_icb_info(&(topo_ptr->t_base));

   // try to determine max samples. if any changes happen, callbacks happen in this context.
   TRY(result, spl_topo_get_required_input_samples(topo_ptr, TRUE));

   CATCH(result, TOPO_MSG_PREFIX, topo_ptr->t_base.gu.log_id)
   {
   }
   return result;
}
