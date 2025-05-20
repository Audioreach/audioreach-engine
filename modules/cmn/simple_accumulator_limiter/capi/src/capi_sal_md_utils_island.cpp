/* ======================================================================== */
/**
   @file capi_sal_md_utils_island.cpp

   Source file to implement metadata related utility functions called by the
   CAPI Interface for Simple Accumulator-Limiter (SAL) Module.
*/

/* =========================================================================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
   ========================================================================== */

/*==========================================================================
Include files
========================================================================== */
#include "capi_sal_utils.h"
#include "audio_basic_op.h"

void capi_sal_push_zeros_for_flushing_eos(capi_sal_t *           me_ptr,
                                          capi_stream_data_v2_t *in_stream_ptr,
                                          sal_in_port_array_t *  in_port_ptr,
                                          uint32_t *             num_samples_per_ch_ptr,
                                          uint32_t *             max_num_samples_per_ch_ptr,
                                          uint32_t               out_max_samples_per_ch,
                                          uint32_t               num_active_inputs_to_acc_without_eos)
{
   uint32_t input_word_size_bytes = me_ptr->operating_mf_ptr->format.bits_per_sample >> 3;

   if (0 == in_port_ptr->pending_zeros_at_eos)
   {
      in_port_ptr->pending_zeros_at_eos = capi_cmn_us_to_bytes_per_ch(me_ptr->algo_delay_us,
                                                                      me_ptr->operating_mf_ptr->format.sampling_rate,
                                                                      me_ptr->operating_mf_ptr->format.bits_per_sample);
   }

   uint32_t zeros_to_push = 0; // bytes per ch

   // if no input buffer is given while zero pushing is going on, zero pushing can still be done depending on output
   // size
   if ((NULL == in_stream_ptr->buf_ptr[0].data_ptr) || (0 == in_stream_ptr->buf_ptr[0].max_data_len))
   {
      // We have to min again with the amount of zeros to push. Then recalculate zero samples from the min.
      zeros_to_push = SAL_MIN(out_max_samples_per_ch * input_word_size_bytes, in_port_ptr->pending_zeros_at_eos);

      //output scratch buffer should have already been memset for zero pushing in the begining of the process.
      //can not memset here because some non-eos input may have been mixed in the output scratch buffer by this time.
   }
   else // valid input
   {
      /** Limit zeros to push by the empty space available in the buffer, this prevents overflow on the input buffer*/
      zeros_to_push = SAL_MIN((in_stream_ptr->buf_ptr[0].max_data_len - in_stream_ptr->buf_ptr[0].actual_data_len),
                              in_port_ptr->pending_zeros_at_eos);

      /*If there is one or more active inputs without eos, we should not push
       *more than maximum of all input actual lengths.
       *max_num_samples_per_ch_ptr = max(actual_data_len of all inputs),
       *num_samples_per_ch_ptr = actual_data_len of current input In dm enabled
       *cases, even though max space available in inp/out buf is 1ms more we
       *cannot push more than max of inp actual len(which would be threshold
       *amount) */

      /*Current *max_num_samples_per_ch_ptr takes the inputs with eos also into account (reduce code changes).
       *Including these inputs with eos, is a don't care because:
       *-> Inputs without eos need to have threshold amount of data for module to be triggered
       *-> Inputs with eos cannot have more than threshold amount of data */
      if (num_active_inputs_to_acc_without_eos > 0)
      {
         zeros_to_push =
            SAL_MIN(((*max_num_samples_per_ch_ptr - *num_samples_per_ch_ptr) * input_word_size_bytes), zeros_to_push);
      }

      // make sure that the input_actual_data_len + zeros_to_push is not overflowing the output buffer.
      if ((out_max_samples_per_ch * input_word_size_bytes) > in_stream_ptr->buf_ptr[0].actual_data_len)
      {
         zeros_to_push =
            SAL_MIN((out_max_samples_per_ch * input_word_size_bytes) - in_stream_ptr->buf_ptr[0].actual_data_len,
                    zeros_to_push);
      }
      else
      {
         zeros_to_push = 0;
      }
   }

   uint32_t zero_samples = (zeros_to_push / input_word_size_bytes); // samples per ch

   // total input samples including zeros and any data
   (*num_samples_per_ch_ptr) += zero_samples;

   // when calling this func, max_num_samples_per_ch_ptr carries max num of input samples across all in ports.
   // any zeros we push on top of this need to be accounted here.
   *max_num_samples_per_ch_ptr = SAL_MAX(*max_num_samples_per_ch_ptr, *num_samples_per_ch_ptr);
   *max_num_samples_per_ch_ptr = SAL_MIN(*max_num_samples_per_ch_ptr, out_max_samples_per_ch);

   // no need to memset zeros when buf is given as scratch buf is already memset to zero
   if (NULL != in_stream_ptr->buf_ptr[0].data_ptr)
   {
      for (uint32_t j = 0; j < me_ptr->operating_mf_ptr->format.num_channels; j++)
      {
         memset(in_stream_ptr->buf_ptr[j].data_ptr + in_stream_ptr->buf_ptr[0].actual_data_len, 0, zeros_to_push);
      }
   }

   in_port_ptr->pending_zeros_at_eos -= zeros_to_push;

   // note: actual_len on input stream must not be updated.
   SAL_MSG_ISLAND(me_ptr->iid,
                  DBG_MED_PRIO,
                  "Zero pushing for flushing EOS. pushed %lu bytes, pending %lu bytes per ch, "
                  "num_active_inputs_to_acc_without_eos %lu",
                  zeros_to_push,
                  in_port_ptr->pending_zeros_at_eos,
                  num_active_inputs_to_acc_without_eos);
}

// caller has to check if this function is required to be called against the inser_eos flag
void capi_sal_insert_eos_for_us_gap(capi_sal_t *me_ptr, capi_stream_data_t *output[])
{

   capi_stream_data_v2_t *out_stream_ptr = (capi_stream_data_v2_t *)output[0];
   // no need to propagate this thru limiter delay as limiter is reset when this flag is set
   module_cmn_md_t *new_md_ptr = NULL;
   capi_err_t       res        = me_ptr->metadata_handler.metadata_create(me_ptr->metadata_handler.context_ptr,
                                                             &out_stream_ptr->metadata_list_ptr,
                                                             sizeof(module_cmn_md_eos_t),
                                                             me_ptr->heap_mem,
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
      new_md_ptr->tracking_ptr          = NULL;

      SAL_MSG_ISLAND(me_ptr->iid, DBG_HIGH_PRIO, "Created and inserted internal, flushing eos at output");
   }
   // clear anyway
   me_ptr->module_flags.insert_int_eos = FALSE;
}

capi_err_t capi_sal_destroy_md_list(capi_sal_t *me_ptr, module_cmn_md_list_t **md_list_pptr)
{
   module_cmn_md_list_t *next_ptr = NULL;
   for (module_cmn_md_list_t *node_ptr = *md_list_pptr; node_ptr;)
   {
      bool_t IS_DROPPED_TRUE = TRUE;
      next_ptr               = node_ptr->next_ptr;
      if (me_ptr->metadata_handler.metadata_destroy)
      {
         me_ptr->metadata_handler.metadata_destroy(me_ptr->metadata_handler.context_ptr,
                                                   node_ptr,
                                                   IS_DROPPED_TRUE,
                                                   md_list_pptr);
      }
      else
      {
         SAL_MSG_ISLAND(me_ptr->iid, DBG_ERROR_PRIO, "metadata handler not provided, can't drop metadata.");
         return CAPI_EFAILED;
      }
      node_ptr = next_ptr;
   }

   return CAPI_EOK;
}

void capi_sal_destroy_all_md(capi_sal_t *me_ptr, capi_stream_data_t *input[])
{
   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      if ((NULL == input[i]))
      {
         continue;
      }

      capi_stream_data_v2_t *in_stream_ptr = (capi_stream_data_v2_t *)input[i];
      if (in_stream_ptr->metadata_list_ptr)
      {
         capi_sal_destroy_md_list(me_ptr, &in_stream_ptr->metadata_list_ptr);
         in_stream_ptr->flags.marker_eos   = FALSE;
         in_stream_ptr->flags.end_of_frame = FALSE;
      }
   }
}

/*------------------------------------------------------------------------
  Function name: capi_sal_handle_metadata
  DESCRIPTION: Processes an input buffer and generates an output buffer.
  -----------------------------------------------------------------------*/
capi_err_t capi_sal_handle_metadata_b4_process(capi_sal_t *        me_ptr,
                                               capi_stream_data_t *input[],
                                               capi_stream_data_t *output[])
{
   capi_err_t result = CAPI_EOK;

   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      if (input[i] == NULL || (DATA_PORT_STARTED != me_ptr->in_port_arr[i].state))
      {
         continue;
      }
#ifdef SAL_SAFE_MODE
      if (CAPI_STREAM_V2 != input[i]->flags.stream_data_version)
      {
         SAL_MSG(me_ptr->iid, DBG_ERROR_PRIO, "stream version must be 1");
         result |= CAPI_EFAILED;
         continue;
      }
#endif
      capi_stream_data_v2_t *in_stream_ptr = (capi_stream_data_v2_t *)input[i];

      // No metadata in stream
      if ((!in_stream_ptr) || (!in_stream_ptr->metadata_list_ptr))
      {
         continue;
      }

      module_cmn_md_list_t *          node_ptr   = in_stream_ptr->metadata_list_ptr;
      module_cmn_md_list_t *          next_ptr   = NULL;
      module_md_sal_unmixed_output_t *sal_md_ptr = NULL;

      while (node_ptr)
      {
         next_ptr = node_ptr->next_ptr;

         module_cmn_md_t *md_ptr = (module_cmn_md_t *)node_ptr->obj_ptr;

#ifdef SAL_DBG_LOW
         SAL_MSG(me_ptr->iid, DBG_LOW_PRIO, "metadata 0x%lx port_index: %d", md_ptr->metadata_id, i);
         SAL_MSG(me_ptr->iid, DBG_LOW_PRIO, "metadata offset: %d", md_ptr->offset);
#endif
         switch (md_ptr->metadata_id)
         {
            case MODULE_MD_ID_SAL_UNMIXED_OUTPUT:
            {
               bool_t out_of_band = md_ptr->metadata_flag.is_out_of_band;
               if (out_of_band)
               {
                  sal_md_ptr = (module_md_sal_unmixed_output_t *)md_ptr->metadata_ptr;
               }
               else
               {
                  sal_md_ptr = (module_md_sal_unmixed_output_t *)&(md_ptr->metadata_buf);
               }

               me_ptr->module_flags.unmixed_output         = (bool_t)sal_md_ptr->unmixed_output;
               me_ptr->unmixed_output_port_index           = i;
               me_ptr->module_flags.dtmf_tone_started      = TRUE;
               me_ptr->module_flags.process_other_input_md = sal_md_ptr->process_other_input_md;

               SAL_MSG_ISLAND(me_ptr->iid,
                              DBG_HIGH_PRIO,
                              "Received metadata - unmixed_output: %d of port_index: %d, dtmf_tone_started %d "
                              "process_other_input_md %d",
                              me_ptr->module_flags.unmixed_output,
                              me_ptr->unmixed_output_port_index,
                              me_ptr->module_flags.dtmf_tone_started,
                              me_ptr->module_flags.process_other_input_md);
               // SAL handles this metadata and changes behavior - does not propagate this metadata
               me_ptr->metadata_handler.metadata_destroy(me_ptr->metadata_handler.context_ptr,
                                                         node_ptr,
                                                         TRUE /* don't care */,
                                                         &in_stream_ptr->metadata_list_ptr);
            }
            break;
            case MODULE_CMN_MD_ID_EOS:
            {
               if (me_ptr->module_flags.dtmf_tone_started && (i == me_ptr->unmixed_output_port_index))
               {
                  SAL_MSG_ISLAND(me_ptr->iid, DBG_HIGH_PRIO, "Received EOS unmixed_output_port_index");
                  me_ptr->module_flags.unmixed_output         = FALSE;
                  me_ptr->module_flags.dtmf_tone_started      = FALSE;
                  me_ptr->module_flags.process_other_input_md = FALSE;
                  SAL_MSG_ISLAND(me_ptr->iid,
                                 DBG_HIGH_PRIO,
                                 "Postponing EOS handling to cover for only DTMF case"); // need to propagate EOS if
                                                                                         // DTMF is the only port.
               }
            }
            break;
         }

         node_ptr = next_ptr;
      }
   }

   return result;
}

/*------------------------------------------------------------------------
  Function name: capi_sal_check_process_metadata
  DESCRIPTION: Function that processes relevant metadata in the capi input stream


  FTRT input handling:
  1. if only one input : flushing EOS is propagated as flushing.
  2. if many inputs : flushing EOS is changed as non-flushing
  3. if many inputs are connected only one is active. inactive = stopped or DFG propagated or EOS propagated
 * -----------------------------------------------------------------------*/
void capi_sal_check_process_metadata(capi_sal_t *           me_ptr,
                                     capi_stream_data_v2_t *in_stream_ptr,
                                     capi_stream_data_v2_t *out_stream_ptr,
                                     uint32_t               num_bytes_per_ch,
                                     sal_in_port_array_t *  in_port_ptr,
                                     uint32_t               data_port_index,
                                     uint32_t *             num_ports_flush_eos_ptr)
{
   module_cmn_md_list_t **internal_list_pptr = &in_port_ptr->md_list_ptr;
   // SAL has algo delay of limiter
   uint32_t algo_delay_us = me_ptr->module_flags.is_inplace ? 0 : me_ptr->algo_delay_us;

   bool_t prev_out_marker_eos = out_stream_ptr->flags.marker_eos;
   bool_t new_out_marker_eos  = FALSE;
   bool_t data_produced       = FALSE;

   // these flags can be dropped after accumulator.
   in_stream_ptr->flags.end_of_frame = FALSE;
   in_stream_ptr->flags.erasure      = FALSE;

   /*
    * we need to check if the stream version is v2, and if the eos flag is set
    * if so, we need to check if the flag in the eos metadata is set to flushing
    * if so, we need to set it to non-flushing. see capi_sal_process_metadata_b4_output
    * Proper Sequence should ensure that OMF ptr is valid if there's metadata in either lists.
    * That's because only when EOS goes to the output port - do we actually stop the input
    * port and put it at_gap
    */
#ifdef SAL_SAFE_MODE
   if (CAPI_STREAM_V2 != in_stream_ptr->flags.stream_data_version) // stream version v2
   {
      // error
   }
#endif
   if (!(in_stream_ptr->metadata_list_ptr || *internal_list_pptr))
   {
      return;
   }

   intf_extn_md_propagation_t input_md_info;
   memset(&input_md_info, 0, sizeof(input_md_info));
   input_md_info.df                          = me_ptr->operating_mf_ptr->header.format_header.data_format;
   input_md_info.len_per_ch_in_bytes         = num_bytes_per_ch;
   input_md_info.initial_len_per_ch_in_bytes = num_bytes_per_ch;
   input_md_info.bits_per_sample             = me_ptr->operating_mf_ptr->format.bits_per_sample;
   input_md_info.sample_rate                 = me_ptr->operating_mf_ptr->format.sampling_rate;

   intf_extn_md_propagation_t output_md_info;
   memscpy(&output_md_info, sizeof(output_md_info), &input_md_info, sizeof(input_md_info));
   output_md_info.initial_len_per_ch_in_bytes = 0;
   // The only way to know if this current propagation caused a new flushing eos to go to the output is to
   // clear the output side marker eos and check if it becomes TRUE. We can do this since the output's marker_eos
   // shouldn't affect behavior of the metadata_propagate() function. We cache the output marker_eos before clearing
   // and add it back after calling metadata_propagate.
out_stream_ptr->flags.marker_eos = FALSE;
   bool_t in_dfg_before             = capi_sal_sdata_has_dfg(me_ptr, in_stream_ptr);
   // multiple EoS may be present when split paths merge at acc
   // this function keeps the output EOS same as input EOS (in terms of flushing/non-flushing)
   me_ptr->metadata_handler.metadata_propagate(me_ptr->metadata_handler.context_ptr,
                                               in_stream_ptr,
                                               out_stream_ptr,
                                               internal_list_pptr,
                                               algo_delay_us,
                                               &input_md_info,
                                               &output_md_info);

   new_out_marker_eos = out_stream_ptr->flags.marker_eos;
   out_stream_ptr->flags.marker_eos |= prev_out_marker_eos;

   bool_t in_dfg_after = capi_sal_sdata_has_dfg(me_ptr, in_stream_ptr);
   // DFG was consumed if it was present before propagation and not present (in this case it would be moved
   // to output since SAL has no buffering delay) after.
   bool_t in_dfg_was_consumed = in_dfg_before && (!in_dfg_after);
   // if EOS moves out from input (or internal list) OR DFG was consumed, then mark this port as at_gap.
   if (new_out_marker_eos || in_dfg_was_consumed)
   {
      in_port_ptr->port_flags.at_gap = TRUE;
      me_ptr->num_ports_at_gap++;
      if (me_ptr->num_ports_at_gap == me_ptr->num_in_ports)
      {
         me_ptr->module_flags.all_ports_at_gap = TRUE;
      }
   }

   if (new_out_marker_eos)
   {
      *num_ports_flush_eos_ptr = *num_ports_flush_eos_ptr + 1;
   }

   // Corner case: Consider a scenario where SAL has inputs where media format is different from OMF.
   // When the ports with OMF are at gap (due to EOS), SAL picks one of the mismatched input ports as reference.
   // If EOS is propagated in the same process call (without any data), then SAL should raise output media
   // format only in the next process call. Propagating EOS (MD) and raising MF in the same process call can
   // lead to undefined behavior (including memory corruption). Hence, set this variable to TRUE if
   // SAL has generated output data/metadata
   if (((NULL != out_stream_ptr->buf_ptr) && (out_stream_ptr->buf_ptr[0].actual_data_len)) ||
       (out_stream_ptr->metadata_list_ptr))
   {
      data_produced = TRUE;
   }

   if (me_ptr->in_port_arr[data_port_index].port_flags.at_gap)
   {
      SAL_MSG_ISLAND(me_ptr->iid, DBG_HIGH_PRIO, "Port idx %lu went to at gap - calling port stop", data_port_index);
      // exiting island here as capi_sal_handle_data_flow_stop is moved to non-island
#ifdef USES_AUDIO_IN_ISLAND
      posal_island_trigger_island_exit();
#else
      posal_island_trigger_island_exit_inline();
#endif // USES_AUDIO_IN_ISLAND
      capi_sal_handle_data_flow_stop(me_ptr, data_port_index, data_produced);
   }

   return;
}

/**
 * Checks if the sdata_ptr has any DFG metadata in it by looping through the metadata list.
 * TODO(claguna): Does this belong in a common place? If so, md_id to find could be an argument.
 */
bool_t capi_sal_sdata_has_dfg(capi_sal_t *me_ptr, capi_stream_data_v2_t *sdata_ptr)
{
   bool_t has_dfg = FALSE;

   if (!sdata_ptr)
   {
      SAL_MSG_ISLAND(me_ptr->iid, DBG_ERROR_PRIO, "capi sal sdata_ptr was NULL, returning FALSE.");
      return FALSE;
   }

   module_cmn_md_list_t *list_ptr = sdata_ptr->metadata_list_ptr;
   while (list_ptr)
   {
      module_cmn_md_t *md_ptr = list_ptr->obj_ptr;
      if (MODULE_CMN_MD_ID_DFG == md_ptr->metadata_id)
      {
         has_dfg = TRUE;
         break;
      }

      list_ptr = list_ptr->next_ptr;
   }

   return has_dfg;
}