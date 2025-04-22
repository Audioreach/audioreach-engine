/**
 *   \file capi_trm_island.c
 *   \brief
 *        This file contains CAPI implementation of Timed Renderer Module
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "trm_api.h"
#include "capi_trm.h"
#include "capi_trm_utils.h"
#include "posal_memory.h"
#include "posal_timer.h"
#include "spf_list_utils.h"

/*==============================================================================
   Local Defines
==============================================================================*/

static capi_vtbl_t vtbl = { capi_trm_process,        capi_trm_end,           capi_trm_set_param, capi_trm_get_param,
                            capi_trm_set_properties, capi_trm_get_properties };

static bool_t capi_trm_needs_render_decision(capi_trm_t *me_ptr, bool_t is_signal_trigger);
static capi_trm_render_decision_t capi_trm_make_render_decision(capi_trm_t *me_ptr, bool_t is_resync);
static capi_err_t capi_trm_underrun(capi_trm_t *me_ptr, capi_stream_data_t *output[], bool_t clear_ts);

capi_vtbl_t *capi_trm_get_vtbl()
{
   return &vtbl;
}

/**
 * Check if we should try and make a render decision.
 * 1. We can only make a render decision if signal triggered, buffer triggers don't come at precise times.
 * 2. Once the decision is render, continue to render forever (no need to make another decision).
 */
static bool_t capi_trm_needs_render_decision(capi_trm_t *me_ptr, bool_t is_signal_trigger)
{
   return (CAPI_TRM_DECISION_RENDER != me_ptr->render_decision) && is_signal_trigger;
}

static capi_trm_render_decision_t capi_trm_make_render_decision(capi_trm_t *me_ptr, bool_t is_resync)
{
   capi_trm_render_decision_t render_decision;

   // Get the wall clock
   uint64_t wall_clock_us = me_ptr->wall_clock_at_trigger;

   AR_MSG_ISLAND(DBG_HIGH_PRIO,
                 "capi timed renderer: Making render decision: Current wall clock: [%lu, %lu] us, cur ttr [%lu, %lu] "
                 "us",
                 (uint32_t)(wall_clock_us >> 32),
                 (uint32_t)wall_clock_us,
                 (uint32_t)(me_ptr->curr_ttr >> 32),
                 (uint32_t)me_ptr->curr_ttr);

   int64_t buffer_time_diff_us = 0;

   buffer_time_diff_us = me_ptr->curr_ttr - wall_clock_us;

   // TTR is in the past.
   if ( (buffer_time_diff_us < 0) && (llabs(buffer_time_diff_us) > CAPI_TRM_JITTER_TOLERANCE) )
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO,
                    "capi timed renderer: Warning: wall clock is past ttr - dropping input. RT input cases are "
                    "not expected to recover.");

      render_decision = CAPI_TRM_DECISION_DROP;

      // Throw away TTR when decision is drop, then we will look for next TTR and make next render decision.
      me_ptr->first_ttr_received = FALSE;

      capi_trm_flush_held_buffer(me_ptr, NULL, TRUE /*force_free_md*/);
   }
   else
   {
	  // two frames added one for regular prebuffer and one for one time prebuffer.
	  uint32_t data_needed_before_ttr_data_us = 2 * me_ptr->frame_size_us;
	  
	  // if ttr is more than the wall-clock then correct the jitter by pre-padding some extra data.
	  if (me_ptr->curr_ttr > me_ptr->wall_clock_at_trigger)
	  {
		  data_needed_before_ttr_data_us += (uint32_t)(me_ptr->curr_ttr - me_ptr->wall_clock_at_trigger);
	  }
      // TTR is in the present or future.
      if (me_ptr->held_input_buf.actual_data_len_all_ch && is_resync) // VFR Resync case.
      {
         // Since there is already some data from previous VFR therefore keep on rendering.
         render_decision = CAPI_TRM_DECISION_RENDER;

        
         uint32_t data_needed_before_resynced_data_bytes_per_ch =
            capi_cmn_us_to_bytes_per_ch(data_needed_before_ttr_data_us,
                                        me_ptr->media_format.format.sampling_rate,
                                        me_ptr->media_format.format.bits_per_sample);

         uint32_t data_present_in_held_buffer_bytes_per_ch =
            me_ptr->held_input_buf.actual_data_len_all_ch / me_ptr->media_format.format.num_channels;

         uint32_t zeros_to_pad_bytes_per_ch = 0;
         if (data_needed_before_resynced_data_bytes_per_ch > data_present_in_held_buffer_bytes_per_ch)
         {
            zeros_to_pad_bytes_per_ch =
               data_needed_before_resynced_data_bytes_per_ch - data_present_in_held_buffer_bytes_per_ch;

            capi_trm_buffer_zeros(me_ptr, zeros_to_pad_bytes_per_ch);
         }
         else
         {
            // instead of dropping from the held buffer, we will drop from the resynced buffers.
            me_ptr->input_bytes_to_drop_per_ch =
               data_present_in_held_buffer_bytes_per_ch - data_needed_before_resynced_data_bytes_per_ch;
         }

         AR_MSG_ISLAND(DBG_HIGH_PRIO,
                       "capi timed renderer: Resync handling, zeros_to_pad %d bytes per ch, input to drop %d bytes per "
                       "ch ",
                       zeros_to_pad_bytes_per_ch,
                       me_ptr->input_bytes_to_drop_per_ch);
      }
      else
      {
         // Render if wall clock is within 1 frame size of ttr.
         uint32_t rendering_threshold_us = me_ptr->frame_size_us;
         if (llabs(buffer_time_diff_us) <= rendering_threshold_us)
         {
            render_decision = CAPI_TRM_DECISION_RENDER;

            // Now that we know how many zeros to pad for precise rendering, calculate total zeros to pad by
            // adding frame duration. Initialize remaining_zeros_to_pad so we will begin padding zeros at the
            // beginning of rendering.
            // extra two frames added one for regular prebuffer and one for one time prebuffer.
            me_ptr->total_zeros_to_pad_us     = data_needed_before_ttr_data_us;
            me_ptr->remaining_zeros_to_pad_us = me_ptr->total_zeros_to_pad_us;
         }
         else
         {
            render_decision = CAPI_TRM_DECISION_HOLD;
         }
      }
   }
   return render_decision;
}

static void capi_trm_reset_err_counts(capi_trm_t *me_ptr)
{
   me_ptr->bad_args_err_count              = 0;
   me_ptr->no_held_buf_err_count           = 0;
   me_ptr->output_not_empty_err_count      = 0;
   me_ptr->not_enough_op_err_count         = 0;
   me_ptr->unexp_render_decision_err_count = 0;
   me_ptr->unexp_underrun_err_count        = 0;
}

static void capi_trm_check_print_errors(capi_trm_t *me_ptr)
{

   uint64_t cur_time = posal_timer_get_time();
   if ((0 == me_ptr->prev_err_time) || ((cur_time - me_ptr->prev_err_time) > 10000))
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO,
                    "capi timed renderer: bad args %ld, held buf not allocated %ld, op not empty %ld, not enough op "
                    "%ld, unexpected render decision %ld, unexpected underrun %ld",
                    me_ptr->bad_args_err_count,
                    me_ptr->no_held_buf_err_count,
                    me_ptr->output_not_empty_err_count,
                    me_ptr->not_enough_op_err_count,
                    me_ptr->unexp_render_decision_err_count,
                    me_ptr->unexp_underrun_err_count);
      capi_trm_reset_err_counts(me_ptr);
      me_ptr->prev_err_time = cur_time;
   }
}

/*------------------------------------------------------------------------
  Function name: capi_trm_process
  Processes an input buffer and generates an output buffer.
 * -----------------------------------------------------------------------*/
capi_err_t capi_trm_process(capi_t *capi_ptr, capi_stream_data_t *input[], capi_stream_data_t *output[])
{
   capi_err_t result = CAPI_EOK;

   bool_t      is_in_provided  = capi_trm_in_has_data(input);
   bool_t      is_out_provided = capi_trm_out_has_space(output);
   capi_trm_t *me_ptr          = (capi_trm_t *)((capi_ptr));

   if ((NULL == capi_ptr))
   {
#ifdef TRM_DEBUG
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "capi timed renderer: null capi ptr");
#endif
      return CAPI_EFAILED;
   }

   if ((!is_in_provided && !is_out_provided))
   {
#ifdef TRM_DEBUG
      AR_MSG_ISLAND(DBG_ERROR_PRIO,
                    "capi timed renderer: bad input, capi_ptr 0x%lx, in provided %ld, out provided %ld",
                    capi_ptr,
                    is_in_provided,
                    is_out_provided);
#else
      me_ptr->bad_args_err_count++;
      capi_trm_check_print_errors(me_ptr);
#endif

      return CAPI_EFAILED;
   }

   if (NULL == me_ptr->held_input_buf.frame_ptr)
   {
#ifdef TRM_DEBUG
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "capi timed renderer: held buffer is not yet allocated!");
#else
      me_ptr->no_held_buf_err_count++;
      capi_trm_check_print_errors(me_ptr);
#endif
      
      // here Input data buf is considered consumed, so need to drop metadata as well.
      capi_trm_drop_all_metadata(me_ptr, input);
      return CAPI_EFAILED;
   }

   if (0 == me_ptr->frame_size_us)
   {
      // Err count not needed as held buf wouldn't be created - shouldn't reach this code.
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "capi timed renderer: error: frame size not yet configured during process.");

      // here Input data buf is considered consumed, so need to drop metadata as well.
      capi_trm_drop_all_metadata(me_ptr, input);
      return CAPI_EFAILED;
   }

   if (!me_ptr->is_input_mf_received)
   {
      // Err count not needed as held buf wouldn't be created - shouldn't reach this code.
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "capi timed renderer: Input Media format not set yet");
      return CAPI_EFAILED;
   }

   // Sanity checks on output side - If output is provided, we expect empty output with at least enough space to fill
   // one frame.
   if (is_out_provided)
   {
      if (0 != output[0]->buf_ptr[0].actual_data_len)
      {
#ifdef TRM_DEBUG
         AR_MSG_ISLAND(DBG_ERROR_PRIO,
                       "capi timed renderer: error: output not empty: %ld actual data len 1 ch",
                       output[0]->buf_ptr[0].actual_data_len);
#else
         me_ptr->output_not_empty_err_count++;
         capi_trm_check_print_errors(me_ptr);
#endif
         // here Input data buf is considered consumed, so need to drop metadata as well.
         capi_trm_drop_all_metadata(me_ptr, input);
         return CAPI_EFAILED;
      }
      uint32_t frame_size_bytes_per_ch = capi_cmn_us_to_bytes_per_ch(me_ptr->frame_size_us,
                                                                     me_ptr->media_format.format.sampling_rate,
                                                                     me_ptr->media_format.format.bits_per_sample);
      if (output[0]->buf_ptr[0].max_data_len < frame_size_bytes_per_ch)
      {
#ifdef TRM_DEBUG
         AR_MSG_ISLAND(DBG_ERROR_PRIO,
                       "capi timed renderer: error: out space provided %ld is less than frame size %ld (both bytes per "
                       "ch)",
                       output[0]->buf_ptr[0].max_data_len,
                       frame_size_bytes_per_ch);
#else
         me_ptr->not_enough_op_err_count++;
         capi_trm_check_print_errors(me_ptr);
#endif

         // here Input data buf is considered consumed, so need to drop metadata as well.
         capi_trm_drop_all_metadata(me_ptr, input);

         return CAPI_EFAILED;
      }
   }

   /**
    * If a flushing EOS comes in, then it's usually due to upstream stop.
    * Previously EOS was buffered in the TRM. TRM cleared marker_eos on the input while is_eos_buffered in internal buffered.
    * This meant the container never called the module (when input is at gap, process is not called - even for signal triggers).
    * Further when new data came in EOS was dropped.
    * During hand-overs or operating mode changes in voice calls, voice path restarts. Rendering should not have glitches.
    * The pop-suppresser placed after the TRM ramps down when EOS is received. Absence of EOS can lead to glitch.
    * In addition, if there are modules with algo delay such as SAL, MFC, they also need to flush out their history.
    * Not doing so can result in initial glitch after new data comes in.
    * The glitch is visible in 1586 log only if there's an ultrasound mixer right before data-logging.
    *
    * Ideally we would want to send EOS through internal buffering.
    * However, if new data comes before EOS goes out, EOS would be dropped (flushing EOS + new data).
    * This means pop-sup won't get a chance to ramp-down and glitch would be seen.
    * This could also be fixed if TRM can pass flushing EOS and wait for one more cycle to send new data.
    * This leads to implementation complexity.
    */
   if (input[0]->flags.marker_eos)
   {
      AR_MSG_ISLAND(DBG_HIGH_PRIO,
                    "capi timed renderer: EOS processing: Dropping all buffered data & moving EOS to out");

      module_cmn_md_list_t *md_list_ptr = NULL;
      capi_trm_flush_held_buffer(me_ptr, &md_list_ptr, FALSE /*force_free_md*/); // keeps any stream associated MD
      capi_trm_clear_ttr(me_ptr);

      input[0]->flags.marker_eos  = FALSE;
      output[0]->flags.marker_eos = TRUE;

      //set output as zero. pop sup will output ramp-down.
      output[0]->buf_ptr[0].actual_data_len = 0;

      spf_list_merge_lists((spf_list_node_t **)&md_list_ptr,
                           (spf_list_node_t **)&((capi_stream_data_v2_t *)input[0])->metadata_list_ptr);

      // move all the metadata to end of output buffer.
      if (md_list_ptr)
      {
         module_cmn_md_list_t *node_ptr = md_list_ptr;
         while (node_ptr)
         {
            module_cmn_md_t *md_ptr = node_ptr->obj_ptr;
            md_ptr->offset          = 0;

            AR_MSG_ISLAND(DBG_HIGH_PRIO,
                          "capi timed renderer:metadata_id 0x%lX, offset %lu",
                          md_ptr->metadata_id,
                          md_ptr->offset);

            if (MD_ID_TTR == md_ptr->metadata_id)
            {
               capi_err_t capi_res = me_ptr->metadata_handler.metadata_destroy(me_ptr->metadata_handler.context_ptr,
                                                                               node_ptr,
                                                                               TRUE, // dropping metadata
                                                                               &md_list_ptr);
               AR_MSG_ISLAND(DBG_ERROR_PRIO, "capi timed renderer: TTR metadata destroyed with err code %ld", capi_res);

               // when a node is destoyed init again to the head of the list.
               node_ptr = md_list_ptr;
               continue;
            }

            node_ptr = node_ptr->next_ptr;
         }
         spf_list_merge_lists((spf_list_node_t **)&(((capi_stream_data_v2_t *)output[0])->metadata_list_ptr),
                              (spf_list_node_t **)&md_list_ptr);
      }

      return CAPI_EOK;
   }

   // Since we raised output as blocked, we know we are signal triggered (not buffer triggered) if output was provided.
   bool_t                   is_signal_trigger = is_out_provided;
   capi_trm_propped_flags_t propped_flags;
   propped_flags.marker_eos = FALSE;

#ifdef TRM_DEBUG

   if (is_signal_trigger)
   {
      uint32_t in_actual_len  = 0;
      uint32_t in_max_len     = 0;
      uint32_t in_flags       = 0;
      uint32_t out_actual_len = 0;
      uint32_t out_max_len    = 0;
      uint32_t out_flags      = 0;

      if (is_in_provided)
      {
         in_actual_len = input[0]->buf_ptr[0].actual_data_len;
         in_max_len    = input[0]->buf_ptr[0].max_data_len;
      }

      if (input[0])
      {
         in_flags = input[0]->flags.word;
      }

      if (is_out_provided)
      {
         out_actual_len = output[0]->buf_ptr[0].actual_data_len;
         out_max_len    = output[0]->buf_ptr[0].max_data_len;
      }
      if (output[0])
      {
         out_flags = output[0]->flags.word;
      }

      AR_MSG_ISLAND(DBG_HIGH_PRIO,
                    "capi timed renderer: ---------- top of process - is signal triggered %ld, input actual len: %d, "
                    "max "
                    "len: "
                    "%d, flags 0x%lx, output actual len %ld, max len %ld, flags 0x%lx",
                    is_signal_trigger,
                    in_actual_len,
                    in_max_len,
                    in_flags,
                    out_actual_len,
                    out_max_len,
                    out_flags);
   }
#endif

   if (is_signal_trigger)
   {
      me_ptr->wall_clock_at_trigger = posal_timer_get_time();
   }

   bool_t is_resync = FALSE;
   // Get the new ttr
   capi_trm_handle_metadata_b4_process(me_ptr, input, output, &is_resync);

   // If we don't have cached TTR, it could be one of following:
   // 1. First data didn't arrive yet. We can underrun.
   // 2. First data arrived without TTR. Drop that data and underrun.
   // 3. We evalulated TTR as drop and threw away TTR. Drop data and underrun until next TTR arrives.
   if (me_ptr->first_ttr_received == FALSE)
   {
      bool_t CLEAR_TS_TRUE = TRUE;
#ifdef TRM_DEBUG
      AR_MSG_ISLAND(DBG_HIGH_PRIO, "capi timed renderer: don't have ttr. underrun.");
#endif

      // drop metadata
      capi_trm_drop_all_metadata(me_ptr, input);

      result |= capi_trm_underrun(me_ptr, output, CLEAR_TS_TRUE);
      return result;
   }

   // Make a render decision if necessary.
   if (capi_trm_needs_render_decision(me_ptr, is_signal_trigger) || is_resync)
   {
      /* We need to hold the resynced data even if this is not signal trigger,
       * Therefore we need to evaluate the render decision immediately.
       */
      if (!is_signal_trigger)
      {
         /* To handle the resync we need to know the exact time of the next signal trigger when we render the first data
          * after resync. If this is the signal trigger then "wall_clock_at_trigger" is the current time. If this is the
          * data trigger then "wall_clock_at_trigger" is the time of last signal trigger and next signal trigger will
          * approximately happen after frame_size_us time.
          */
         me_ptr->wall_clock_at_trigger += me_ptr->frame_size_us;
      }
      me_ptr->render_decision = capi_trm_make_render_decision(me_ptr, is_resync);

      if (CAPI_TRM_DECISION_RENDER == me_ptr->render_decision)
      {
         capi_trm_update_tgp_after_sync(me_ptr);
      }
   }

   // This will happen if input arrives at EP before a signal trigger (expected since signal trigger won't trigger
   // module process before first true data). In this case we should hold the input data and wait till next signal
   // trigger to make a decision.
   if (CAPI_TRM_DECISION_PENDING == me_ptr->render_decision)
   {
      AR_MSG_ISLAND(DBG_HIGH_PRIO, "capi timed renderer: didn't do render decision yet, holding data.");
      me_ptr->render_decision = CAPI_TRM_DECISION_HOLD;
   }

   switch (me_ptr->render_decision)
   {
      case CAPI_TRM_DECISION_HOLD:
      {
         bool_t CLEAR_TS_TRUE = TRUE;
         AR_MSG_ISLAND(DBG_HIGH_PRIO, "capi timed renderer: holding input at TRM");

         // Hold case: Buffer input (to render later), underrun zeros in the meantime.
         result |= capi_trm_buffer_input_data(me_ptr, input);
         if (CAPI_FAILED(result))
         {
			/*This is needed to ensure that there is no stall in the processing despite the failure to consume input.*/ 
            AR_MSG_ISLAND(DBG_ERROR_PRIO, "capi timed renderer: buffering input failed Underunning");
            result = CAPI_EOK;
            capi_trm_drop_all_metadata(me_ptr, input);
         }

         result |= capi_trm_underrun(me_ptr, output, CLEAR_TS_TRUE);
         break;
      }

      case CAPI_TRM_DECISION_RENDER:
      {
#ifdef TRM_DEBUG
         AR_MSG_ISLAND(DBG_HIGH_PRIO, "capi timed renderer: rendering input");
#endif
         me_ptr->process_count++;

         result |= capi_trm_buffer_input_data(me_ptr, input);
         if (CAPI_FAILED(result))
         {
			/*This is needed to ensure that there is no stall in the processing despite the failure to consume input.*/
            AR_MSG_ISLAND(DBG_ERROR_PRIO, "capi timed renderer: buffering input failed. Rendering From internal buffer");
            result = CAPI_EOK;
            capi_trm_drop_all_metadata(me_ptr, input);
         }

         result |= capi_trm_render_data_from_held_input_buffer(me_ptr, output, &propped_flags);
         break;
      }
      case CAPI_TRM_DECISION_DROP:
      {
         // Drop case.
         bool_t CLEAR_TS_TRUE = TRUE;

#ifdef TRM_DEBUG
         AR_MSG_ISLAND(DBG_HIGH_PRIO, "capi timed renderer: dropping input");
#endif

         // Input data implicitly dropped by keeping actual data length the same. Drop metadata.
         capi_trm_drop_all_metadata(me_ptr, input);

         result |= capi_trm_underrun(me_ptr, output, CLEAR_TS_TRUE);
         break;
      }
      default:
      {
#ifdef TRM_DEBUG
         AR_MSG_ISLAND(DBG_HIGH_PRIO, "capi timed renderer: Unexpected render decision %ld", me_ptr->render_decision);
#else
         me_ptr->unexp_render_decision_err_count++;
         capi_trm_check_print_errors(me_ptr);
#endif

         result |= CAPI_EUNSUPPORTED;
      }
      break;
   }


#ifdef TRM_DEBUG
   uint32_t in_actual_len  = 0;
   uint32_t in_max_len     = 0;
   uint32_t in_flags       = 0;
   uint32_t out_actual_len = 0;
   uint32_t out_max_len    = 0;
   uint32_t out_flags      = 0;

   if (is_in_provided)
   {
      in_actual_len = input[0]->buf_ptr[0].actual_data_len;
      in_max_len    = input[0]->buf_ptr[0].max_data_len;
   }
   if (input[0])
   {
      in_flags = input[0]->flags.word;
   }

   if (is_out_provided)
   {
      out_actual_len = output[0]->buf_ptr[0].actual_data_len;
      out_max_len    = output[0]->buf_ptr[0].max_data_len;
   }
   if (output[0])
   {
      out_flags = output[0]->flags.word;
   }

   AR_MSG_ISLAND(DBG_HIGH_PRIO,
                 "capi timed renderer: |||||||||| bottom of process - is signal triggered %ld, input actual len: %d, "
                 "max "
                 "len: "
                 "%d, flags 0x%lx, output actual len %ld, max len %ld, flags 0x%lx",
                 is_signal_trigger,
                 in_actual_len,
                 in_max_len,
                 in_flags,
                 out_actual_len,
                 out_max_len,
                 out_flags);
#endif

   if (is_out_provided && (0 == output[0]->buf_ptr[0].actual_data_len))
   {
      // For actual underrun cases, let fwk decide timestamp behavior.
      bool_t CLEAR_TS_FALSE = FALSE;

#ifdef TRM_DEBUG
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "capi timed renderer: unexpected underrun scenario. Pushing erasure zeros.");
#else
      me_ptr->unexp_underrun_err_count++;
      capi_trm_check_print_errors(me_ptr);
#endif

      result |= capi_trm_underrun(me_ptr, output, CLEAR_TS_FALSE);
   }


   return result;
}

/*
 * Outputs 1 frame of zeros to the output, and set erasure to TRUE. Doesn't handle the case where
 * output has partial data.
 */
static capi_err_t capi_trm_underrun(capi_trm_t *me_ptr, capi_stream_data_t *output[], bool_t clear_ts)
{
   capi_err_t result = CAPI_EOK;

   if (!capi_trm_out_has_space(output))
   {
#ifdef TRM_DEBUG
      AR_MSG_ISLAND(DBG_HIGH_PRIO, "capi timed renderer: not producing zeros, output not present");
#endif
      return result;
   }

   uint32_t num_channels      = me_ptr->media_format.format.num_channels;
   uint32_t frame_size_per_ch = capi_cmn_us_to_bytes_per_ch(me_ptr->frame_size_us,
                                                            me_ptr->media_format.format.sampling_rate,
                                                            me_ptr->media_format.format.bits_per_sample);

   for (uint32_t ch = 0; ch < num_channels; ch++)
   {
      memset(output[0]->buf_ptr[ch].data_ptr, 0, frame_size_per_ch);
      output[0]->buf_ptr[ch].actual_data_len = frame_size_per_ch;
   }

   output[0]->flags.erasure = TRUE;

   if (clear_ts)
   {
      output[0]->flags.is_timestamp_valid = FALSE;
      output[0]->timestamp                = 0;
   }
   return result;
}
