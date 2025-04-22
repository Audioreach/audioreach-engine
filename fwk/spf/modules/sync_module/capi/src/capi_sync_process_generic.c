/**
 * \file capi_sync_process_generic.c
 * \brief
 *       Implementation of generic sync data processing functions
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_sync_i.h"
#include "module_cmn_api.h"

/* =========================================================================
 * Static function declarations
 * =========================================================================*/
static void generic_sync_print_process_ctx_info(capi_sync_t *me_ptr);
static void generic_sync_reset_proc_ctx(capi_sync_t *me_ptr);
static capi_err_t generic_sync_consume_input(capi_sync_t *me_ptr, capi_stream_data_t *input[]);
static capi_err_t generic_sync_render_output(capi_sync_t *       me_ptr,
                                             capi_stream_data_t *output[],
                                             capi_stream_data_t *input[]);
static capi_err_t generic_sync_push_eos_at_close(capi_sync_t *       me_ptr,
                                                 capi_stream_data_t *input[],
                                                 capi_stream_data_t *output[]);
static capi_err_t capi_sync_pass_through_metadata(capi_sync_t *       me_ptr,
                                                  capi_stream_data_t *input[],
                                                  capi_stream_data_t *output[]);
static bool_t generic_sync_is_output_process_thresh_met(capi_sync_t *me_ptr,
                                                        uint32_t     check_mask,
                                                        bool_t       has_only_ready_ports);
static bool_t capi_sync_can_output_be_filled(capi_sync_t *         me_ptr,
                                             capi_sync_out_port_t *out_port_ptr,
                                             capi_stream_data_t *  output[]);
static capi_err_t generic_sync_setup_input_process(capi_sync_t *me_ptr, capi_stream_data_t *input[]);
static capi_err_t generic_sync_setup_output_process(capi_sync_t *       me_ptr,
                                                    capi_stream_data_t *input[],
                                                    capi_stream_data_t *output[]);
static capi_err_t generic_sync_check_stopped_port_metadata(capi_sync_t *       me_ptr,
                                                           capi_stream_data_t *input[],
                                                           capi_stream_data_t *output[]);

/* =========================================================================
 * Function definitions
 * =========================================================================*/
/**
 * Entry point for the sync module process in equal prio mode
 */
capi_err_t generic_sync_mode_process(capi_sync_t *me_ptr, capi_stream_data_t *input[], capi_stream_data_t *output[])
{
   capi_err_t capi_result    = CAPI_EOK;
   bool_t     any_data_found = FALSE;
   bool_t     data_gap_found = FALSE;

#ifdef CAPI_SYNC_DEBUG
   AR_MSG(DBG_MED_PRIO, "capi sync process(): ");
#endif

   generic_sync_reset_proc_ctx(me_ptr);

   // Check if any ports are started with this process call
   capi_result |= capi_sync_validate_io_bufs(me_ptr, input, output, &any_data_found);

   if (CAPI_FAILED(capi_result))
   {
      return capi_result;
   }

   // If not data is found on any ports, then simply handle the data gap flag
   // in the input(s) if any and return
   if (!any_data_found)
   {
      AR_MSG(DBG_MED_PRIO, "capi sync process(): no data found on any input port. check handle metadata/dfg ");
      capi_result |= capi_sync_pass_through_metadata(me_ptr, input, output);
      capi_result |= capi_sync_handle_dfg(me_ptr, input, output, &data_gap_found);
      return capi_result;
   }

   // If only one port is active, wait for threshold to fill before output
   // During transition state, as long as all the sync'ed ports is filled
   // send out output
   // In sync'ed state, all inputs are expected to have threshold worth of data
   uint32_t mandatory_port_mask = generic_sync_calc_synced_input_mask(me_ptr);
   uint32_t optional_port_mask  = generic_sync_calc_waiting_input_mask(me_ptr);

   // if all inputs are stopped, pass through metadata and handle DFG
   if (!mandatory_port_mask && !optional_port_mask)
   {
#ifdef CAPI_SYNC_DEBUG
      AR_MSG(DBG_MED_PRIO, "capi sync process(): Warning No active inputs..");
#endif
      // RR: mark input as unconsumed? should not come here technically
      // or bypass??
      capi_result |= capi_sync_pass_through_metadata(me_ptr, input, output);
      capi_result |= capi_sync_handle_dfg(me_ptr, input, output, &data_gap_found);
      return capi_result;
   }

   me_ptr->proc_ctx.process_counter++;

   // Calculate the stopped ports that have metadata
   (void)generic_sync_calc_stopped_input_mask(me_ptr, input);
   capi_result |= generic_sync_check_stopped_port_metadata(me_ptr, input, output);

   // always try to consume input into internal buffers
   // Output determines transition from sync'ing & sync'ed
   capi_result |= generic_sync_setup_input_process(me_ptr, input);

   // generic_sync_print_process_ctx_info(me_ptr);

   capi_result |= generic_sync_consume_input(me_ptr, input);

   // generic_sync_print_process_ctx_info(me_ptr);

   capi_result |= generic_sync_setup_output_process(me_ptr, input, output);

   // generic_sync_print_process_ctx_info(me_ptr);

   capi_result |= generic_sync_render_output(me_ptr, output, input);

   capi_result |= generic_sync_push_eos_at_close(me_ptr, input, output);

   generic_sync_print_process_ctx_info(me_ptr);

   if (me_ptr->proc_ctx.ports_processed_mask.output != me_ptr->proc_ctx.ports_to_process_mask.output)
   {
      AR_MSG(DBG_MED_PRIO,
             "capi sync process(): WARNING! Mismatched output masks processed = %x, to_be_proc %x",
             me_ptr->proc_ctx.ports_processed_mask.output,
             me_ptr->proc_ctx.ports_to_process_mask.output);
   }

   capi_result |= capi_sync_handle_dfg(me_ptr, input, output, &data_gap_found);

   bool_t should_disable_thresh = capi_sync_should_disable_thresh(me_ptr);

#ifdef CAPI_SYNC_DEBUG
   AR_MSG(DBG_MED_PRIO,
          "capi sync process(): Current Thresh state = %d, me_ptr thresh %d",
          should_disable_thresh,
          me_ptr->threshold_is_disabled);
#endif

   // threshold_disable should be updated only if there is any output generated from this process context.
   if ((me_ptr->threshold_is_disabled != should_disable_thresh) && me_ptr->proc_ctx.ports_processed_mask.output)
   {
      AR_MSG(DBG_MED_PRIO,
             "capi sync process(): Current Thresh state = %d, me_ptr thresh %d. Raising event",
             should_disable_thresh,
             me_ptr->threshold_is_disabled);

      if (FALSE == should_disable_thresh)
      {
         me_ptr->synced_state = STATE_SYNCED;
      }

      capi_result |= capi_sync_raise_event_toggle_threshold(me_ptr, !should_disable_thresh);
   }

   // try to disable the sync module if threshold is enabled and there is just one input port.
   // since this disable is through a dedicated interface extension therefore it can be done after processing module.
   if ((!me_ptr->threshold_is_disabled) && (1 == me_ptr->num_opened_in_ports))
   {
      capi_sync_in_port_t *in_port_info_ptr = NULL;
      bool_t               is_disable       = FALSE;

      for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
      {
         if (me_ptr->in_port_info_ptr[i].cmn.state != CAPI_PORT_STATE_CLOSED)
         {
            in_port_info_ptr = &me_ptr->in_port_info_ptr[i];
            break;
         }
      }

      // disable the module if there is not data or metadata.
      if (in_port_info_ptr && (!in_port_info_ptr->is_threshold_disabled) &&
          (0 == in_port_info_ptr->int_bufs_ptr[0].actual_data_len) &&
          (!in_port_info_ptr->int_stream.flags.end_of_frame))
      {
         is_disable = TRUE;
      }

      capi_sync_raise_enable_disable_event(me_ptr, !is_disable);
   }
   //If threshold is disabled then return ENEEDMORE so that fwk continues buffering
   capi_result |= (me_ptr->threshold_is_disabled)? CAPI_ENEEDMORE: CAPI_EOK;
   return capi_result;
}

/**
 *  Utility function to consume data from the capi input buffer to the module's
 *  internal buffers.
 */
static capi_err_t generic_sync_consume_input(capi_sync_t *me_ptr, capi_stream_data_t *input[])
{
   capi_err_t capi_overall_result = CAPI_EOK;
   uint32_t   proc_in_mask        = me_ptr->proc_ctx.ports_to_process_mask.input;
   uint32_t   capi_data_avail_mask = me_ptr->proc_ctx.capi_data_avail_mask.input;

#ifdef CAPI_SYNC_DEBUG
   AR_MSG(DBG_MED_PRIO, "capi sync process(): Consume input ");
#endif

   // Based on the mask calculated in setup_input_process, loop & consume data
   while (proc_in_mask)
   {
      uint32_t in_index = s32_get_lsb_s32(proc_in_mask);
      capi_sync_clear_bit(&proc_in_mask, in_index);
      capi_sync_clear_bit(&capi_data_avail_mask, in_index);

      capi_sync_in_port_t *in_port_ptr = capi_sync_get_in_port_from_index(me_ptr, in_index);

      if (!in_port_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO, "capi sync consume_input() error, encountered null in_port_ptr");
         return CAPI_EFAILED;
      }

      capi_err_t capi_result = capi_sync_buffer_new_data(me_ptr, input, in_port_ptr);

      // At this point, if dfg is still in the input sdata, it means it wasn't consumed by this process call.
      // So we shouldn't do dfg handling.
      if (capi_sync_sdata_has_dfg(me_ptr, (capi_stream_data_v2_t *)input[in_index]))
      {
         AR_MSG(DBG_MED_PRIO,
                "capi sync process(): DFG marker found on input port idx %d after buffer_new_data. Not handling.",
                in_port_ptr->cmn.self_index);
         in_port_ptr->pending_data_flow_gap = FALSE;
      }

      // Track successful process call from the input port context
      if (CAPI_SUCCEEDED(capi_result))
      {
         capi_sync_set_bit(&me_ptr->proc_ctx.ports_processed_mask.input, in_index);
      }
      capi_overall_result |= capi_result;
   }

   // mark the data unconsumed for the ports which are not processed.
   while (capi_data_avail_mask)
   {
      uint32_t in_index = s32_get_lsb_s32(capi_data_avail_mask);

      capi_sync_clear_bit(&capi_data_avail_mask, in_index);

      capi_sync_in_port_t *in_port_ptr = capi_sync_get_in_port_from_index(me_ptr, in_index);
      if (!in_port_ptr)
      {
         continue;
      }
      capi_overall_result |= capi_sync_mark_input_unconsumed(me_ptr, input, in_port_ptr);
   }

   // RR TODO: If ports_processed != ports_to_process, mark corresponding inputs as not consumed?
   // Can this happen if the ports are synced?
   if (me_ptr->proc_ctx.ports_to_process_mask.input != me_ptr->proc_ctx.ports_processed_mask.input)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi sync process(): Consume input could not process all "
             "inputs successfully! Expected to process %x processed %x ",
             me_ptr->proc_ctx.ports_to_process_mask.input,
             me_ptr->proc_ctx.ports_processed_mask.input);
   }

   return capi_overall_result;
}

/**
 *  Utility function to render output to the capiv2 buffer from the module's
 *  internal buffer. EOS is handled here as well
 */
static capi_err_t generic_sync_render_output(capi_sync_t *       me_ptr,
                                             capi_stream_data_t *output[],
                                             capi_stream_data_t *input[])
{
   capi_err_t capi_overall_result = CAPI_EOK;

   uint32_t proc_out_mask = me_ptr->proc_ctx.ports_to_process_mask.output;

#ifdef CAPI_SYNC_DEBUG
   AR_MSG(DBG_MED_PRIO, "capi sync process(): Render output ");
#endif

   while (proc_out_mask)
   {
      uint32_t out_index = s32_get_lsb_s32(proc_out_mask);
      capi_sync_clear_bit(&proc_out_mask, out_index);

      capi_sync_out_port_t *out_port_ptr = capi_sync_get_out_port_from_index(me_ptr, out_index);
      if (!out_port_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO, "capi sync render_output() error, encountered null out_port_ptr");
         return CAPI_EFAILED;
      }
      uint32_t in_index = out_port_ptr->cmn.conn_index;

      capi_sync_in_port_t *in_port_ptr = capi_sync_get_in_port_from_index(me_ptr, in_index);
      if (!in_port_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO, "capi sync render_output() error, encountered null in_port_ptr");
         return CAPI_EFAILED;
      }

      capi_err_t capi_result = capi_sync_send_buffered_data(me_ptr, output, me_ptr->synced_state, in_port_ptr, input);

      if (CAPI_SUCCEEDED(capi_result))
      {
         capi_sync_set_bit(&me_ptr->proc_ctx.ports_processed_mask.output, out_index);
      }
      capi_overall_result |= capi_result;
   }

   return capi_overall_result;
}

static capi_err_t generic_sync_push_eos_at_close(capi_sync_t *       me_ptr,
                                                 capi_stream_data_t *input[],
                                                 capi_stream_data_t *output[])
{
   capi_err_t result = CAPI_EOK;

   for (uint32_t i = 0; i < me_ptr->num_out_ports; i++)
   {
      capi_sync_out_port_t *out_port_ptr = &(me_ptr->out_port_info_ptr[i]);

      if ((CAPI_PORT_STATE_CLOSED != out_port_ptr->cmn.state) && out_port_ptr->needs_eos_at_close)
      {
         capi_stream_data_v2_t *output_v2_ptr = (capi_stream_data_v2_t *)output[out_port_ptr->cmn.self_index];
         // If the output port sdata wasn't provided, then try again next time.
         if (!output_v2_ptr)
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "capi sync generic_sync_push_eos_at_close() error, can't push eos on op idx %ld id 0x%lx - output "
                   "sdata not provided.",
                   out_port_ptr->cmn.self_index,
                   out_port_ptr->cmn.self_port_id);
            return result;
         }

         // If data was generated this process call due to a new input port being opened corresponding to previously
         // closed output port, we can avoid sending the EOS since a new stream started.
         if (output_v2_ptr->buf_ptr && (0 == output_v2_ptr->buf_ptr[0].actual_data_len))
         {
            // Push EOS to the output port.
            module_cmn_md_t *new_md_ptr = NULL;
            capi_err_t       res        = me_ptr->metadata_handler.metadata_create(me_ptr->metadata_handler.context_ptr,
                                                                      &output_v2_ptr->metadata_list_ptr,
                                                                      sizeof(module_cmn_md_eos_t),
                                                                      me_ptr->heap_info,
                                                                      FALSE /*is_out_band*/,
                                                                      &new_md_ptr);
            if (CAPI_SUCCEEDED(res))
            {
               new_md_ptr->metadata_id           = MODULE_CMN_MD_ID_EOS;
               new_md_ptr->offset                = 0;
               module_cmn_md_eos_t *eos_md_ptr   = (module_cmn_md_eos_t *)&new_md_ptr->metadata_buf;
               eos_md_ptr->flags.is_flushing_eos = TRUE;
               eos_md_ptr->flags.is_internal_eos = TRUE;
               eos_md_ptr->cntr_ref_ptr          = NULL;

               AR_MSG(DBG_HIGH_PRIO,
                      "Created and inserted internal, flushing eos at output idx "
                      "%ld id 0x%lx for close.",
                      out_port_ptr->cmn.self_index,
                      out_port_ptr->cmn.self_port_id);

               output_v2_ptr->flags.marker_eos |= TRUE;
               output_v2_ptr->flags.end_of_frame |= TRUE;
            }
         }

         // Clear flags.
         out_port_ptr->needs_eos_at_close = FALSE;
      }
   }

   return result;
}

/**
 *  Utility function to handle the EOS markers in the capiv2 input stream
 */
static capi_err_t capi_sync_pass_through_metadata(capi_sync_t *       me_ptr,
                                                  capi_stream_data_t *input[],
                                                  capi_stream_data_t *output[])
{
   capi_err_t result = CAPI_EOK;

   for (uint32_t idx = 0; idx < me_ptr->num_in_ports; idx++)
   {
      capi_sync_in_port_t *in_port_ptr = capi_sync_get_in_port_from_index(me_ptr, idx);

      if (!in_port_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO, "capi sync pass_through_metadata() error, encountered null in_port_ptr");
         return CAPI_EFAILED;
      }

      if ((SYNC_PORT_INDEX_INVALID == in_port_ptr->cmn.self_index) ||
          (SYNC_PORT_INDEX_INVALID == in_port_ptr->cmn.conn_index))
      {
         continue;
      }
      if (input && input[in_port_ptr->cmn.self_index] && output && output[in_port_ptr->cmn.conn_index])
      {
         result |= capi_sync_pass_through_metadata_single_port(me_ptr, in_port_ptr, output, input);
      }
   }

   return result;
}

/**
 *  Utility function to reset the process context variables
 */
static void generic_sync_reset_proc_ctx(capi_sync_t *me_ptr)
{
   uint32_t curr_counter_val = me_ptr->proc_ctx.process_counter;

   memset(&me_ptr->proc_ctx, 0, sizeof(me_ptr->proc_ctx));
   me_ptr->proc_ctx.process_counter = curr_counter_val;

   // TODO(claguna): Move pending_data_flow_gap to the proc ctx?
   for (uint32_t idx = 0; idx < me_ptr->num_in_ports; idx++)
   {
      capi_sync_in_port_t *in_port_ptr = capi_sync_get_in_port_from_index(me_ptr, idx);

      if (in_port_ptr)
      {
         in_port_ptr->pending_data_flow_gap = FALSE;
         in_port_ptr->pending_eof           = FALSE;
      }
   }
}

/**
 *  Utility function to print the process context variables.
 *  Useful for debugging
 */
static void generic_sync_print_process_ctx_info(capi_sync_t *me_ptr)
{
#ifdef CAPI_SYNC_DEBUG
   capi_sync_proc_info_t *proc_ctx_ptr = &(me_ptr->proc_ctx);

   AR_MSG(DBG_LOW_PRIO, "capi sync process info counter = %d", proc_ctx_ptr->process_counter);
   AR_MSG(DBG_LOW_PRIO,
          "capi sync process info CAPI in mask %x out mask %x",
          proc_ctx_ptr->capi_data_avail_mask.input,
          proc_ctx_ptr->capi_data_avail_mask.output);
   AR_MSG(DBG_LOW_PRIO,
          "capi sync process info ports to process in mask %x ports to process out mask %x",
          proc_ctx_ptr->ports_to_process_mask.input,
          proc_ctx_ptr->ports_to_process_mask.output);
   AR_MSG(DBG_LOW_PRIO,
          "capi sync process info num synced inputs %d synced mask %x num waiting %d waiting mask %x num stopped %d "
          "stopped mask %x",
          proc_ctx_ptr->num_synced,
          proc_ctx_ptr->synced_ports_mask,
          proc_ctx_ptr->num_ready_to_sync,
          proc_ctx_ptr->ready_to_sync_mask,
          proc_ctx_ptr->num_stopped,
          proc_ctx_ptr->stopped_ports_mask);
   AR_MSG(DBG_LOW_PRIO,
          "capi sync process info ports processed in mask %x ports processed out mask %x",
          proc_ctx_ptr->ports_processed_mask.input,
          proc_ctx_ptr->ports_processed_mask.output);
#endif
}

/**
 *  Utility function to check if the output ports to be processed have met the
 *  threshold criteria (based on the internal buffering)
 *
 *  Handles all combinations of ready & sync'ed input ports
 */
static bool_t generic_sync_is_output_process_thresh_met(capi_sync_t *me_ptr,
                                                        uint32_t     check_mask,
                                                        bool_t       has_only_ready_ports)
{
   bool_t               can_proceed;
   capi_stream_data_t **DUMMY_PTR          = NULL;
   bool_t               CHECK_INTERNAL_BUF = FALSE;

   can_proceed = (has_only_ready_ports) ? FALSE : TRUE;

   while (check_mask)
   {
      uint32_t index = s32_get_lsb_s32(check_mask);
      capi_sync_clear_bit(&check_mask, index);

      capi_sync_in_port_t *in_port_ptr = capi_sync_get_in_port_from_index(me_ptr, index);
      if (!in_port_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO, "capi sync is_output_process_thresh_met() error, encountered null in_port_ptr");
         return CAPI_EFAILED;
      }

      if (has_only_ready_ports)
      {
         // If any port meets threshold, proceed
         if (capi_sync_port_meets_threshold(me_ptr, in_port_ptr, DUMMY_PTR, CHECK_INTERNAL_BUF))
         {
#ifdef CAPI_SYNC_DEBUG
            AR_MSG(DBG_MED_PRIO, "capi sync output process criteria met for %d", in_port_ptr->cmn.self_index);
#endif
            can_proceed = TRUE;
            break;
         }

         if (in_port_ptr->pending_eof || in_port_ptr->int_stream.flags.end_of_frame)
         {
#ifdef CAPI_SYNC_DEBUG
            AR_MSG(DBG_MED_PRIO,
                   "capi sync output process criteria due to eof on %d before syncing",
                   in_port_ptr->cmn.self_index);
#endif
            can_proceed                  = TRUE;
            break;
         }
      }
      else
      {

         if (in_port_ptr->is_eos_rcvd)
         {
#ifdef CAPI_SYNC_DEBUG
            AR_MSG(DBG_MED_PRIO, "capi sync output process criteria skipped for %d", in_port_ptr->cmn.self_index);
#endif
            continue; // Skip ports that have received EOS
         }

         // If a port doesnt meet threshold & doesnt have pending EOF, do not proceed
         if (!capi_sync_port_meets_threshold(me_ptr, in_port_ptr, DUMMY_PTR, CHECK_INTERNAL_BUF) &&
             !(in_port_ptr->pending_eof))
         {
#ifdef CAPI_SYNC_DEBUG
            AR_MSG(DBG_MED_PRIO,
                   "capi sync output process criteria NOT met for %d, pending_eos %ld, dfg %ld eof %d",
                   in_port_ptr->cmn.self_index,
                   in_port_ptr->pending_eos,
                   in_port_ptr->pending_data_flow_gap,
                   in_port_ptr->pending_eof);
#endif
            can_proceed = FALSE;
            break;
         }
      }
   }

   return can_proceed;
}

/**
 *  Utility function to identify the input ports that have to be processed in the current
 *  module process call
 */
static capi_err_t generic_sync_setup_input_process(capi_sync_t *me_ptr, capi_stream_data_t *input[])
{
   capi_err_t capi_result = CAPI_EOK;

   capi_sync_proc_info_t *proc_ctx_ptr = &(me_ptr->proc_ctx);

#ifdef CAPI_SYNC_DEBUG
   AR_MSG(DBG_MED_PRIO, "capi sync process(): Setup input process");
#endif

   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      capi_sync_in_port_t *in_port_ptr = &(me_ptr->in_port_info_ptr[i]);
      uint32_t             in_index    = in_port_ptr->cmn.self_index;

      if (SYNC_PORT_INDEX_INVALID == in_index || (SYNC_PORT_INDEX_INVALID == in_port_ptr->cmn.conn_index))
      {
         continue;
      }

      // If the capiv2 input buffer has data, check if the sync module has
      // space to consume data or not.
      if (capi_sync_input_has_data(input, in_index))
      {
         capi_sync_set_bit(&proc_ctx_ptr->capi_data_avail_mask.input, in_index);

         if (capi_sync_input_buffer_has_space(in_port_ptr))
         {
            capi_sync_set_bit(&proc_ctx_ptr->ports_to_process_mask.input, in_index);
         }
         else
         {
            capi_sync_clear_bit(&proc_ctx_ptr->ports_to_process_mask.input, in_index);
         }
      }
      else
      {
         // if input has no data but has metadata and is in START state, force process
         if (in_port_ptr->pending_eof && CAPI_PORT_STATE_STARTED == in_port_ptr->cmn.state)
         {
            capi_sync_set_bit(&proc_ctx_ptr->ports_to_process_mask.input, in_index);
         }
         else
         {
            capi_sync_clear_bit(&proc_ctx_ptr->capi_data_avail_mask.input, in_index);
            capi_sync_clear_bit(&proc_ctx_ptr->ports_to_process_mask.input, in_index);
         }
      }
   } // for input ports loop

   return capi_result;
}

/**
 *  Utility function to identify the output ports that have to be processed in the current
 *  module process call
 */
static capi_err_t generic_sync_setup_output_process(capi_sync_t *       me_ptr,
                                                    capi_stream_data_t *input[],
                                                    capi_stream_data_t *output[])
{
   capi_err_t             capi_result  = CAPI_EOK;
   capi_sync_proc_info_t *proc_ctx_ptr = &(me_ptr->proc_ctx);

#ifdef CAPI_SYNC_DEBUG
   AR_MSG(DBG_MED_PRIO, "capi sync process(): Setup output ");
#endif

   // Handle the following scenarios :-
   //    1. If a combination of sync'ed & ready to sync ports exist,
   //          Process all of them as long as threshold is met in the sync'ed ports.
   //
   //    2. If only sync'ed ports exist
   //          Process all of them as long as threshold is met in the sync'ed ports.
   //
   //    3. If only ready to sync ports exist
   //          Wait until one port has threshold amount of data before processing all
   //
   //    4. If EOS/data flow gap is pending on any input port, force process that port
   //           as long as the threshold criteria is met for the other ports
   //
   //    5. If EOF is set on any port before the module is sync'ed, that is also considered
   //           a force process case. Data buffered so far in the internal buffer is flushed out
   // The case with both being empty is handled in the beginning of the process call itself.

   // If only ready to sync ports exist
   if (proc_ctx_ptr->num_ready_to_sync && !proc_ctx_ptr->num_synced)
   {
      uint32_t proc_in_mask         = proc_ctx_ptr->ready_to_sync_mask;
      bool_t   HAS_ONLY_READY_PORTS = TRUE;

      if (generic_sync_is_output_process_thresh_met(me_ptr, proc_in_mask, HAS_ONLY_READY_PORTS))
      {
         bool_t INPUT_TO_OUTPUT = TRUE;
         proc_ctx_ptr->ports_to_process_mask.output =
            generic_sync_calc_conn_mask(me_ptr, proc_ctx_ptr->ready_to_sync_mask, INPUT_TO_OUTPUT);
      }
      else
      {
         // Don't render any output
         proc_ctx_ptr->ports_to_process_mask.output = 0;
      }
   }
   else // Combination of sync'ed and ready to sync ports
   {
      // Validate that all the sync'ed ports have threshold worth data
      uint32_t proc_in_mask             = proc_ctx_ptr->synced_ports_mask;
      bool_t   MIXED_READY_SYNCED_PORTS = FALSE;

      if (generic_sync_is_output_process_thresh_met(me_ptr, proc_in_mask, MIXED_READY_SYNCED_PORTS))
      {
         bool_t INPUT_TO_OUTPUT = TRUE;
         proc_ctx_ptr->ports_to_process_mask.output =
            generic_sync_calc_conn_mask(me_ptr, proc_ctx_ptr->ready_to_sync_mask, INPUT_TO_OUTPUT);
         proc_ctx_ptr->ports_to_process_mask.output |=
            generic_sync_calc_conn_mask(me_ptr, proc_ctx_ptr->synced_ports_mask, INPUT_TO_OUTPUT);
      }
      else
      {
         // Don't render any output
         proc_ctx_ptr->ports_to_process_mask.output = 0;
      }
   }

   // Make sure that there is space in the capi output buffer to accommodate the output ports to be processed
   uint32_t proc_out_mask        = proc_ctx_ptr->ports_to_process_mask.output;
   uint32_t backup_proc_out_mask = proc_ctx_ptr->ports_to_process_mask.output;

   bool_t should_reset_eof = FALSE;
   while (proc_out_mask)
   {
      uint32_t out_index = s32_get_lsb_s32(proc_out_mask);
      capi_sync_clear_bit(&proc_out_mask, out_index);

      capi_sync_out_port_t *out_port_ptr = capi_sync_get_out_port_from_index(me_ptr, out_index);
      if (!out_port_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "capi sync setup_output_process() error, encountered null out_port_ptr for %d",
                out_index);
         return CAPI_EFAILED;
      }

      if (!capi_sync_can_output_be_filled(me_ptr, out_port_ptr, output))
      {
         should_reset_eof                           = TRUE;
         proc_ctx_ptr->ports_to_process_mask.output = 0;
         capi_result = CAPI_EFAILED; // This is not expected. Return error to debug from this point onwards.

#ifdef CAPI_SYNC_DEBUG
         AR_MSG(DBG_ERROR_PRIO, "capi sync process(): Cannot fill capiv2 output for %d", out_index);
#endif
      }
   }

   if (should_reset_eof)
   {
      while (backup_proc_out_mask)
      {
         uint32_t out_index = s32_get_lsb_s32(backup_proc_out_mask);
         capi_sync_clear_bit(&backup_proc_out_mask, out_index);

         capi_sync_out_port_t *out_port_ptr = capi_sync_get_out_port_from_index(me_ptr, out_index);
         if (!out_port_ptr)
         {
            continue; // This cannot happen since failure would be returned in the previous loop
         }

         uint32_t             in_index    = out_port_ptr->cmn.conn_index;
         capi_sync_in_port_t *in_port_ptr = capi_sync_get_in_port_from_index(me_ptr, in_index);

         if (!in_port_ptr)
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "capi sync setup_output_process() error, encountered null in_port_ptr for %d",
                   in_index);
            return CAPI_EFAILED;
         }

         if (in_port_ptr->int_stream.flags.end_of_frame)
         {
            if (input && input[in_index])
            {
               AR_MSG(DBG_MED_PRIO, "rohinir debug input index %d, adding eof back to capiv2 stream", in_index);
               input[in_index]->flags.end_of_frame = TRUE;
            }
         }
      }
   }

   return capi_result;
}

/**
 *  Utility function to identify if the capiv2 output port buffer can be filled or not
 */
static bool_t capi_sync_can_output_be_filled(capi_sync_t *         me_ptr,
                                             capi_sync_out_port_t *out_port_ptr,
                                             capi_stream_data_t *  output[])
{
   capi_sync_in_port_t *in_port_ptr = capi_sync_get_in_port_from_index(me_ptr, out_port_ptr->cmn.conn_index);
   if (!in_port_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi sync can_output_be_filled() error, encountered null in_port_ptr");
      return FALSE;
   }

   // If EOF is pending, force process the internal buffer data. Return right away.
   if (in_port_ptr->pending_eof)
   {
      return TRUE;
   }

   // If MF is not yet received, return FALSE.
   if (!in_port_ptr->int_bufs_ptr)
   {
      return FALSE;
   }

   uint32_t out_index = out_port_ptr->cmn.self_index;

   if (!capi_sync_output_has_space(output, out_index))
   {
      return FALSE;
   }

   uint32_t out_buf_size = output[out_index]->buf_ptr[0].max_data_len - output[out_index]->buf_ptr[0].actual_data_len;

   // For both sync'ed and waiting to sync inputs, output size must be at least threshold bytes
   if (out_buf_size < in_port_ptr->threshold_bytes_per_ch)
   {
#ifdef CAPI_SYNC_DEBUG
      AR_MSG(DBG_LOW_PRIO,
             "capi sync can_output_be_filled(), input_index %d out_buf_size %d threshold required %d",
             in_port_ptr->cmn.self_index,
             out_buf_size,
             in_port_ptr->threshold_bytes_per_ch);
#endif
      return FALSE;
   }

   capi_stream_data_t **DUMMY_PTR          = NULL;
   bool_t               CHECK_INTERNAL_BUF = FALSE;
   uint32_t             curr_input_mask    = 0;

   capi_sync_set_bit(&curr_input_mask, in_port_ptr->cmn.self_index);

   // If input is ready to sync, return right away. Output size is checked already
   if (curr_input_mask & me_ptr->proc_ctx.ready_to_sync_mask)
   {
#ifdef CAPI_SYNC_DEBUG
      AR_MSG(DBG_LOW_PRIO,
             "capi sync can_output_be_filled(), waiting_to_sync_input %d out_buf_size %d threshold required %d",
             in_port_ptr->cmn.self_index,
             out_buf_size,
             in_port_ptr->threshold_bytes_per_ch);
#endif
      return TRUE;
   }

   // If input is sync'ed, check if internal buffer has buffered threshold worth data
   if (capi_sync_port_meets_threshold(me_ptr, in_port_ptr, DUMMY_PTR, CHECK_INTERNAL_BUF))
   {
      return TRUE;
   }
#ifdef CAPI_SYNC_DEBUG
   else
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi sync can_output_be_filled() error, synced input does_int_buf_meet_thresh %d out_buf_size %d "
             "threshold required %d",
             capi_sync_port_meets_threshold(me_ptr, in_port_ptr, DUMMY_PTR, CHECK_INTERNAL_BUF),
             out_buf_size,
             in_port_ptr->threshold_bytes_per_ch);
   }
#endif

   return FALSE;
}

/**
 *  Utility function to process the stopped ports that have pending metadata
 */
static capi_err_t generic_sync_check_stopped_port_metadata(capi_sync_t *       me_ptr,
                                                           capi_stream_data_t *input[],
                                                           capi_stream_data_t *output[])
{
   capi_sync_proc_info_t *proc_ctx_ptr = &(me_ptr->proc_ctx);

   capi_err_t result = CAPI_EOK;

   // no ports are stopped, so continue
   if (!proc_ctx_ptr->stopped_ports_mask)
   {
      return result;
   }

   uint32_t stopped_mask = proc_ctx_ptr->stopped_ports_mask;

   while (stopped_mask)
   {
      uint32_t in_index = s32_get_lsb_s32(stopped_mask);
      capi_sync_clear_bit(&stopped_mask, in_index);

      capi_sync_in_port_t *in_port_ptr = capi_sync_get_in_port_from_index(me_ptr, in_index);
      if (!in_port_ptr)
      {
         continue;
      }
      result |= capi_sync_pass_through_metadata_single_port(me_ptr, in_port_ptr, output, input);
   }

   return result;
}
