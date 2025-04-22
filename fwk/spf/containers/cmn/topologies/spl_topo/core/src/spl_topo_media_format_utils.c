/**
 * \file spl_topo_media_format_utils.c
 *
 * \brief
 *
 *     Topo 2 utility functions for managing media format and media format propagation in the spl_topo graph.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spl_topo_i.h"

/* =======================================================================
Public Function Definitions
========================================================================== */

/**
 * Propagates media fromat through the topology, starting at the beginning
 * of the sortest list of modules.
 */
ar_result_t spl_topo_propagate_media_fmt(void *cxt_ptr, bool_t is_data_path)
{
   spl_topo_t *      topo_ptr              = (spl_topo_t *)cxt_ptr;
   gu_module_list_t *start_module_list_ptr = (gu_module_list_t *)topo_ptr->t_base.gu.sorted_module_list_ptr;
   return spl_topo_propagate_media_fmt_from_module(cxt_ptr, is_data_path, start_module_list_ptr);
}

/**
 * Check if propagation of media format is possible for given module and input port.
 * Propagation is possible only if
 * - Port states are appropriate
 *   - coming from data path, we must be in STARTED state
 *   - comming from command path, we must be in PREPARED state
 *     (in STOP, wait until PREPARED)
 * - The input port's connected port has a media format event.
 * - There must be no unconsumed data in the input port. That must be consumed
 *   before propagating the media format.
 */
bool_t spl_topo_is_med_fmt_prop_possible(spl_topo_t *           topo_ptr,
                                         spl_topo_module_t *    module_ptr,
                                         spl_topo_input_port_t *in_port_ptr,
                                         bool_t                 is_data_path)
{
   spl_topo_output_port_t *prev_out_ptr    = NULL;
   spl_topo_input_port_t * ext_in_port_ptr = NULL;

   spl_topo_get_connected_int_or_ext_op_port(topo_ptr, in_port_ptr, &prev_out_ptr, &ext_in_port_ptr);

#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
   TOPO_MSG(topo_ptr->t_base.gu.log_id,
            DBG_LOW_PRIO,
            "MIID, Input port index: (0x%lX, %lu) prev_out_ptr 0x%lx, "
            "prev_out_ptr->t_base.common.flags.media_fmt_event %lu, "
            "ext_in_port_ptr 0x%lx, ext_in_port_ptr->t_base.common.flags.media_fmt_event %lu, in_port_ptr = 0x%lx, "
            "in_port_ptr->t_base.common.flags.media_fmt_event %lu ",
            in_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id,
            in_port_ptr->t_base.gu.cmn.index,
            prev_out_ptr,
            prev_out_ptr ? prev_out_ptr->t_base.common.flags.media_fmt_event : 0,
            ext_in_port_ptr,
            ext_in_port_ptr ? ext_in_port_ptr->t_base.common.flags.media_fmt_event : 0,
            in_port_ptr,
            in_port_ptr->t_base.common.flags.media_fmt_event);
#endif // SPL_TOPO_DEBUG

   bool_t propagation_possible = (prev_out_ptr && prev_out_ptr->t_base.common.flags.media_fmt_event) ||
                                 (ext_in_port_ptr && ext_in_port_ptr->t_base.common.flags.media_fmt_event);

   // Force propagation if the input port hasn't received a media format yet and the previous output port
   // has a valid media format.
   // - (!propagation_possible): No need to force if already true.
   // - (prev_out_ptr): Only needed for internal input port case. External inputs will always receive a new media
   //   format on the data path before data during start.
   if ((!propagation_possible) && (!in_port_ptr->t_base.flags.media_fmt_received) && prev_out_ptr)
   {
      if (prev_out_ptr->t_base.common.flags.is_mf_valid)
      {
#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_1
         TOPO_MSG(topo_ptr->t_base.gu.log_id,
                  DBG_MED_PRIO,
                  "Forcing to propagate media format to module id = 0x%lx port id 0x%lx which has not received "
                  "media format yet.",
                  module_ptr->t_base.gu.module_instance_id,
                  in_port_ptr->t_base.gu.cmn.id);
#endif

         propagation_possible = TRUE;

         GEN_TOPO_SET_ONE_CAPI_EVENT_FLAG(&topo_ptr->t_base, media_fmt_event);
         prev_out_ptr->t_base.common.flags.media_fmt_event = TRUE;
      }
   }

   if (propagation_possible) // this check prevents unnecessary warning prints.
   {
      if (is_data_path)
      {
         if (TOPO_PORT_STATE_STARTED != in_port_ptr->t_base.common.state &&
             TOPO_PORT_STATE_PREPARED != in_port_ptr->t_base.common.state)
         {
            // Data path media format must be not propagated to parts of the graph which are stopped. Its ok
            // to propagate data path to prepared ports, since we want to propagate media format before start
            // so that we don't invoke topo_process with wrong-sized external output buffers.
            propagation_possible = FALSE;

#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_1
            TOPO_MSG(topo_ptr->t_base.gu.log_id,
                     DBG_MED_PRIO,
                     "Warning: Data path media format will not be propagated to stopped ports (0x%lX, %lu).",
                     in_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id,
                     in_port_ptr->t_base.gu.cmn.index);
#endif
         }
      }
      else
      {
         // Control path media format can be propagated during run. For the case of control path media format
         // coming on one port of a multi-in port module whose other port is already running: Whether we set
         // input media format during prepare or process, behavior won't differ.

         topo_port_state_t in_port_sg_state;
         in_port_sg_state =
            topo_sg_state_to_port_state(gen_topo_get_sg_state(in_port_ptr->t_base.gu.cmn.module_ptr->sg_ptr));

         if (TOPO_PORT_STATE_STARTED != in_port_sg_state && TOPO_PORT_STATE_PREPARED != in_port_sg_state)
         {
            propagation_possible = FALSE;

#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_1
            TOPO_MSG(topo_ptr->t_base.gu.log_id,
                     DBG_HIGH_PRIO,
                     "Warning: Ctrl path media format will not be propagated to stopped ports (0x%lX, %lu), "
                     "port_state: %lu",
                     in_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id,
                     in_port_ptr->t_base.gu.cmn.index,
                     in_port_ptr->t_base.common.state);
#endif
         }
      }
   }
#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
   else
   {
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_MED_PRIO,
               "Propagating not possible for module id = 0x%lx port id 0x%lx due to no media fmt flags.",
               module_ptr->t_base.gu.module_instance_id,
               in_port_ptr->t_base.gu.cmn.id);
   }
#endif

   // Check for unconsumed data at this module's input (data at any port will halt propagation).
   if (propagation_possible && spl_topo_ip_port_contains_unconsumed_data(topo_ptr, in_port_ptr))
   {
      propagation_possible = FALSE;

#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_1
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_LOW_PRIO,
               "Warning: Media format propagation to (0x%lX, %lu) not possible as there's pending data %lu",
               in_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id,
               in_port_ptr->t_base.gu.cmn.index,
               spl_topo_get_in_port_actual_data_len(topo_ptr, in_port_ptr));
#endif
   }

   return propagation_possible;
}

/**
 * Propagates media format from the input port's connected output port through the
 * input port.
 * - If media format flag is set on connected output port, copies media format
 *   to the input media format and sets the input port media format flag.
 * - If the input port media format flag is set, then tries to set input media format
 *   on the module for that port.
 *   - If setting input media format fails, the module becomes bypassed.
 *   - If setting input media format succeeds, handles fwk extensions at media format.
 * - After trying to set media format, the input port media format flag is cleared.
 * - Most capis will raise output media format events when setting input media format,
 *   and fwk handling of the output media format events will set the media format event
 *   flags on output ports. This is how media format gets passed downstream.
 */
ar_result_t spl_topo_prop_med_fmt_from_prev_out_to_curr_in(spl_topo_t *           topo_ptr,
                                                           spl_topo_module_t *    module_ptr,
                                                           spl_topo_input_port_t *in_port_ptr,
                                                           bool_t                 is_data_path)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t             result          = AR_EOK;
   spl_topo_output_port_t *prev_out_ptr    = NULL;
   spl_topo_input_port_t * ext_in_port_ptr = NULL;

   // Look to the connected port to check if we need to copy its media fmt.
   spl_topo_get_connected_int_or_ext_op_port(topo_ptr, in_port_ptr, &prev_out_ptr, &ext_in_port_ptr);

   if(!prev_out_ptr && !ext_in_port_ptr)
   {
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_ERROR_PRIO,
               "Warning: Failed to find either internal port or external port, ignoring");
      return result;
   }
   // Check whether the connected port is internal or external. Note that exactly one of the
   // pointers will be non-NULL.
   gen_topo_common_port_t *prev_port_ptr =
      prev_out_ptr ? &prev_out_ptr->t_base.common : &ext_in_port_ptr->t_base.common;

   gu_module_t *prev_module_ptr =
      prev_out_ptr ? prev_out_ptr->t_base.gu.cmn.module_ptr : ext_in_port_ptr->t_base.gu.cmn.module_ptr;
   uint32_t prev_port_id = prev_out_ptr ? prev_out_ptr->t_base.gu.cmn.id : ext_in_port_ptr->t_base.gu.cmn.id;

#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
   TOPO_MSG(topo_ptr->t_base.gu.log_id,
            DBG_LOW_PRIO,
            "MIID, Input port index (0x%lX, %lu) prev_out_ptr 0x%lx, "
            "prev_out_ptr->t_base.common.flags.media_fmt_event %lu, "
            "ext_in_port_ptr 0x%lx, ext_in_port_ptr->t_base.common.flags.media_fmt_event %lu ",
            in_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id,
            in_port_ptr->t_base.gu.cmn.index,
            prev_out_ptr,
            prev_out_ptr ? prev_out_ptr->t_base.common.flags.media_fmt_event : 0,
            ext_in_port_ptr,
            ext_in_port_ptr ? ext_in_port_ptr->t_base.common.flags.media_fmt_event : 0);
#endif // SPL_TOPO_DEBUG

   // Copy media format from previous output to current input, and move the media fmt event
   // flag to the current input port.
   if (prev_port_ptr && prev_port_ptr->flags.media_fmt_event)
   {
      if (SPF_IS_PCM_DATA_FORMAT(prev_port_ptr->media_fmt_ptr->data_format) &&
          TU_IS_ANY_DEINTERLEAVED_UNPACKED(prev_port_ptr->media_fmt_ptr->pcm.interleaving))
      {
         // Irrespective of what prev ports deinterleaving version is set, if the next supports V2,
         // propagate as V2, else downgrade to V1
         topo_media_fmt_t temp_mf = *(prev_port_ptr->media_fmt_ptr);
         temp_mf.pcm.interleaving = module_ptr->t_base.flags.supports_deintlvd_unpacked_v2
                                       ? TOPO_DEINTERLEAVED_UNPACKED_V2
                                       : TOPO_DEINTERLEAVED_UNPACKED;

         tu_set_media_fmt(&topo_ptr->t_base.mf_utils,
                          &in_port_ptr->t_base.common.media_fmt_ptr,
                          &temp_mf,
                          topo_ptr->t_base.heap_id);
      }
      else
      {
         tu_set_media_fmt_from_port(&topo_ptr->t_base.mf_utils,
                                    &in_port_ptr->t_base.common.media_fmt_ptr,
                                    prev_port_ptr->media_fmt_ptr);
      }
      in_port_ptr->t_base.flags.media_fmt_received     = TRUE;
      prev_port_ptr->flags.media_fmt_event             = FALSE;
      in_port_ptr->t_base.common.flags.media_fmt_event = TRUE;
      in_port_ptr->t_base.common.flags.is_mf_valid     = prev_port_ptr->flags.is_mf_valid;

      gen_topo_reset_pcm_unpacked_mask(&in_port_ptr->t_base.common);
   }

   // Set media format to module if new mf arrived at this input port. For ext in ports, this flag is set at container
   // level. Otherwise we set it when copying media format from prev port to current port.
   if (in_port_ptr->t_base.common.flags.media_fmt_event)
   {
      // Check if the media format is valid.
      if (!in_port_ptr->t_base.common.flags.is_mf_valid)
      {
         TOPO_MSG(topo_ptr->t_base.gu.log_id,
                  DBG_MED_PRIO,
                  "Warning: Can't propagate media format to input port (0x%lX, %lu), media format is not yet valid. "
                  "Absorbing media format event.",
                  in_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id,
                  in_port_ptr->t_base.gu.cmn.index);
         in_port_ptr->t_base.common.flags.media_fmt_event = FALSE;

         return result;
      }

#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_LOW_PRIO,
               "Setting input media format on input port(0x%lX, %lu), data format %lu fmt id 0x%lx",
               in_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id,
               in_port_ptr->t_base.gu.cmn.index,
               in_port_ptr->t_base.common.media_fmt_ptr->data_format,
               in_port_ptr->t_base.common.media_fmt_ptr->fmt_id);
      TOPO_PRINT_PCM_MEDIA_FMT((topo_ptr->t_base.gu.log_id), (in_port_ptr->t_base.common.media_fmt_ptr), "module input");

#endif // SPL_TOPO_DEBUG

      bool_t      IS_INPUT_MF   = TRUE;
      bool_t      prev_disabled = spl_topo_is_module_disabled(module_ptr);
      ar_result_t mf_result     = gen_topo_capi_set_media_fmt(&topo_ptr->t_base,
                                                          &module_ptr->t_base,
                                                          in_port_ptr->t_base.common.media_fmt_ptr,
                                                          IS_INPUT_MF,
                                                          in_port_ptr->t_base.gu.cmn.index);
      bool_t mf_succeeded             = (AR_EOK == mf_result);
      module_ptr->in_media_fmt_failed = !mf_succeeded;

      // If setting media format changed the disable state (disable to enable, or enable
      // to disable), then we need to update bypass ptr (create or destroy).
      bool_t cur_disabled = spl_topo_is_module_disabled(module_ptr);

      if (cur_disabled != prev_disabled)
      {
         TRY(result, spl_topo_check_update_bypass_module(topo_ptr, module_ptr, cur_disabled));
      }

      // Clear the media format flag from the input. Clearing because we already sent the media format therefore it is
      // not pending on that port anymore.
      in_port_ptr->t_base.common.flags.media_fmt_event = FALSE;
   }

   // since media format is propagated from output to input therefore update the sdata.
   // we are sure that there is no stale data remaining between these two ports.
   // for input ports, we don't need input bufs
   if (prev_port_ptr->flags.is_mf_valid && (NULL != prev_out_ptr))
   {
      gen_topo_initialize_bufs_sdata(&topo_ptr->t_base,
                                     prev_port_ptr,
                                     prev_module_ptr->module_instance_id,
                                     prev_port_id);
   }

#if 0
   if (in_port_ptr->t_base.common.flags.is_mf_valid)
   {
      gen_topo_initialize_bufs_sdata(&topo_ptr->t_base,
                                     &in_port_ptr->t_base.common,
                                     in_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id,
                                     in_port_ptr->t_base.gu.cmn.id);
   }
#endif
   CATCH(result, TOPO_MSG_PREFIX, topo_ptr->t_base.gu.log_id)
   {
   }

   return result;
}

/**
 * Propagates media fromat through the topology, starting with the module pointed
 * to by start_module_list_ptr.
 *
 * For each module, each input port, checks if media format should be propagated through
 * that port, and if so, propagates media format through that port. After looping, if media
 * format reaches any external output ports, the media format is then copied to the framework
 * layer and cleared from the topo layer.
 */
ar_result_t spl_topo_propagate_media_fmt_from_module(void *            cxt_ptr,
                                                     bool_t            is_data_path,
                                                     gu_module_list_t *start_module_list_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result            = AR_EOK;
   spl_topo_t *topo_ptr          = (spl_topo_t *)cxt_ptr;
   bool_t      sent_to_fwk_layer = FALSE; // Unused.

   VERIFY(result, topo_ptr->t_base.topo_to_cntr_vtable_ptr->check_apply_ext_out_ports_pending_media_fmt);

#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_3
   TOPO_MSG(topo_ptr->t_base.gu.log_id,
            DBG_MED_PRIO,
            "Propagating media format through topo..., is_data_path = %lu",
            is_data_path);
#endif
   topo_ptr->t_base.topo_to_cntr_vtable_ptr->check_apply_ext_out_ports_pending_media_fmt(&(topo_ptr->t_base),
                                                                                         is_data_path);

   for (gu_module_list_t *module_list_ptr = start_module_list_ptr; (NULL != module_list_ptr);
        LIST_ADVANCE(module_list_ptr))
   {
      spl_topo_module_t *module_ptr        = (spl_topo_module_t *)module_list_ptr->module_ptr;
      bool_t             UNUSED_MF_PROPPED = FALSE;
      TRY(result, spl_topo_propagate_media_fmt_single_module(topo_ptr, is_data_path, module_ptr, &UNUSED_MF_PROPPED));
   }

   // If a media fmt reached an external output port, copy the media format to the fwk layer.
   spl_topo_check_send_media_fmt_to_fwk_layer(topo_ptr, &sent_to_fwk_layer);

   TOPO_MSG(topo_ptr->t_base.gu.log_id,
            DBG_MED_PRIO,
            "Total number of media format blocks %lu",
            topo_ptr->t_base.mf_utils.num_nodes);

   CATCH(result, TOPO_MSG_PREFIX, topo_ptr->t_base.gu.log_id)
   {
   }

   return result;
}

/**
 * For each of the module's input ports, checks if media format propagation is possible and if so, propagates media
 * format.
 * Also queries output media format for all of a module's output ports which haven't received valid media format yet.
 * For bypassed modules, copies input media format to output if the media formats are different.
 */
ar_result_t spl_topo_propagate_media_fmt_single_module(spl_topo_t *       topo_ptr,
                                                       bool_t             is_data_path,
                                                       spl_topo_module_t *module_ptr,
                                                       bool_t *           mf_propped_ptr)
{
   ar_result_t result           = AR_EOK;
   bool_t      is_mf_propagated = FALSE;

   for (gu_input_port_list_t *in_port_list_ptr = module_ptr->t_base.gu.input_port_list_ptr; (NULL != in_port_list_ptr);
        LIST_ADVANCE(in_port_list_ptr))
   {
      spl_topo_input_port_t *in_port_ptr = (spl_topo_input_port_t *)in_port_list_ptr->ip_port_ptr;

      /**
       *  Do not break going through the sorted module list if propagation is not possible.
       *  Modules may be connected nonlinearly.
       */
      if (spl_topo_is_med_fmt_prop_possible(topo_ptr, module_ptr, in_port_ptr, is_data_path))
      {
         TOPO_MSG(topo_ptr->t_base.gu.log_id,
                  DBG_MED_PRIO,
                  "Propagating media format through module id = %lx.",
                  module_ptr->t_base.gu.module_instance_id);

         // Copy media fmt from prev out port to this input port (link/connection)
         spl_topo_prop_med_fmt_from_prev_out_to_curr_in(topo_ptr, module_ptr, in_port_ptr, is_data_path);
         is_mf_propagated = TRUE;
      }
   }

   // If the output media format of a port is invalid, try to query the module for media format. This is needed
   // in the case a port was closed and reopened, the port structure's media format will be erased however the module
   // already has a valid media format. After getting the media format, the output port's media_fmt_event flag will
   // be set which will help propagate media format downstream.
   //
   // Query output media format in datapath:
   // Query only if input media format has been propagated in data path, else there is no  reason for output media
   // format
   // to be updated. If port is opened newly, mf will be queried at Prepare in ctrl path; so no need to keep querying in
   // process context. Same applies to Bypassed modules as well, output mf will not change unless input has changed
   // in data path.
   if ((is_data_path && is_mf_propagated) || !is_data_path)
   {
      for (gu_output_port_list_t *out_port_list_ptr = module_ptr->t_base.gu.output_port_list_ptr;
           NULL != out_port_list_ptr;
           LIST_ADVANCE(out_port_list_ptr))
      {
         spl_topo_output_port_t *out_port_ptr = (spl_topo_output_port_t *)out_port_list_ptr->op_port_ptr;

         // don't need to get media format for inactive ports.
         if (out_port_ptr->flags.port_inactive)
         {
            continue;
         }

         if (SPF_UNKNOWN_DATA_FORMAT == out_port_ptr->t_base.common.media_fmt_ptr->data_format)
         {
            if (module_ptr->t_base.capi_ptr)
            {
#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
               TOPO_MSG(topo_ptr->t_base.gu.log_id,
                        DBG_MED_PRIO,
                        "Unknown data format found on output port idx = %ld, miid = 0x%lx, querying output media fmt",
                        out_port_ptr->t_base.gu.cmn.index,
                        out_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id);
#endif

               (void)gen_topo_capi_get_out_media_fmt(&(topo_ptr->t_base), &(module_ptr->t_base), &out_port_ptr->t_base);
               is_mf_propagated |= out_port_ptr->t_base.common.flags.is_mf_valid;

               spl_topo_update_check_valid_mf_event_flag(topo_ptr,
                                                         &out_port_ptr->t_base.gu.cmn,
                                                         out_port_ptr->t_base.common.flags.is_mf_valid);
            }
         }
      }

      // Propagate media format through disabled modules. Normal propagation path will result in output media format
      // getting stored in module bypass_ptr, but we still need to push input media format through to output port
      // manually.
      //
      // Also, if out port got destroyed and recreated while disabled, output needs to be re-initialized with input
      // media format.
      if (module_ptr->t_base.bypass_ptr) // SISO
      {
         // If the output/input port doesn't exists then media format cannot be propagated.
         if ((module_ptr->t_base.gu.num_input_ports == 0) || (module_ptr->t_base.gu.num_output_ports == 0))
         {
            TOPO_MSG(topo_ptr->t_base.gu.log_id,
                     DBG_HIGH_PRIO,
                     "Warning: Module 0x%lX doesnt is bypassed and dangling. Media format cannot be propagated.",
                     module_ptr->t_base.gu.module_instance_id);
            return result;
         }
         gen_topo_input_port_t *in_port_ptr =
            (gen_topo_input_port_t *)module_ptr->t_base.gu.input_port_list_ptr->ip_port_ptr;
         gen_topo_output_port_t *out_port_ptr =
            (gen_topo_output_port_t *)module_ptr->t_base.gu.output_port_list_ptr->op_port_ptr;

         if (in_port_ptr->common.flags.is_mf_valid &&
             tu_has_media_format_changed(out_port_ptr->common.media_fmt_ptr, in_port_ptr->common.media_fmt_ptr))
         {
            tu_set_media_fmt_from_port(&topo_ptr->t_base.mf_utils,
                                       &out_port_ptr->common.media_fmt_ptr,
                                       in_port_ptr->common.media_fmt_ptr);

            GEN_TOPO_SET_ONE_CAPI_EVENT_FLAG(&topo_ptr->t_base, media_fmt_event);
            out_port_ptr->common.flags.media_fmt_event        = TRUE;
            out_port_ptr->common.flags.is_mf_valid            = TRUE;

            gen_topo_reset_pcm_unpacked_mask(&out_port_ptr->common);

            spl_topo_update_check_valid_mf_event_flag(topo_ptr,
                                                      &out_port_ptr->gu.cmn,
                                                      out_port_ptr->common.flags.is_mf_valid);

            TOPO_PRINT_MEDIA_FMT(topo_ptr->t_base.gu.log_id,
                                 (&(module_ptr->t_base)),
                                 out_port_ptr,
                                 out_port_ptr->common.media_fmt_ptr,
                                 "bypass module output");

            is_mf_propagated = TRUE;

            gen_topo_set_med_fmt_to_attached_module(&topo_ptr->t_base, out_port_ptr);
         }
      }
   }

   // module could have raised new MF so max num channels can potentially change, check & recereate scratch memory
   GEN_TOPO_SET_ONE_CAPI_EVENT_FLAG(&topo_ptr->t_base, realloc_scratch_mem);

   // Update max buffer length based on the new mf
   if (is_mf_propagated)
   {
      result = spl_topo_update_max_buf_len_for_single_module(topo_ptr, module_ptr);
   }

   // update return flag.
   if (mf_propped_ptr)
   {
      *mf_propped_ptr = is_mf_propagated;
   }

   return result;
}

/**
 * Called from spl_topo_process() and spl_topo_propagate_media_format, to check if a media format event reached
 * an external output port. If so, we need to copy that media fmt event to the fwk layer. Returns
 * sent_to_fwk_layer if we did end up copying a media format event flag to the fwk layer.
 */
ar_result_t spl_topo_check_send_media_fmt_to_fwk_layer(spl_topo_t *topo_ptr, bool_t *sent_to_fwk_layer_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result = AR_EOK;

   VERIFY(result, topo_ptr->t_base.topo_to_cntr_vtable_ptr->set_pending_out_media_fmt);
   VERIFY(result, sent_to_fwk_layer_ptr);

   // We need to check each external output port. If that port has a pending
   // media format, then copy to fwk layer.
   for (gu_ext_out_port_list_t *ext_out_port_list_ptr = topo_ptr->t_base.gu.ext_out_port_list_ptr;
        (NULL != ext_out_port_list_ptr);
        LIST_ADVANCE(ext_out_port_list_ptr))
   {
      gu_ext_out_port_t *     ext_out_port_ptr = ext_out_port_list_ptr->ext_out_port_ptr;
      spl_topo_output_port_t *out_port_ptr     = (spl_topo_output_port_t *)ext_out_port_ptr->int_out_port_ptr;

      if (out_port_ptr && spl_topo_op_port_has_pending_media_fmt(topo_ptr, out_port_ptr))
      {
         // #if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
         TOPO_MSG(topo_ptr->t_base.gu.log_id,
                  DBG_MED_PRIO,
                  "Copying pending media fmt to fwk layer for ext out port on Module id = %lx.",
                  out_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id);
         // #endif

         topo_ptr->t_base.topo_to_cntr_vtable_ptr->set_pending_out_media_fmt(&topo_ptr->t_base,
                                                                             &out_port_ptr->t_base.common,
                                                                             ext_out_port_ptr);

         out_port_ptr->t_base.common.flags.media_fmt_event = FALSE;
         *sent_to_fwk_layer_ptr                            = TRUE;
      }
   }

   CATCH(result, TOPO_MSG_PREFIX, topo_ptr->t_base.gu.log_id)
   {
   }

   return result;
}
