/* ======================================================================== */
/**
   @file capi_sal_utils.cpp

   Source file to implement utility functions called by the CAPI Interface for
   Simple Accumulator-Limiter (SAL) Module.
*/

/* =========================================================================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
   ========================================================================== */

/*==========================================================================
Include files
========================================================================== */
#include "capi_sal_utils.h"
/*==========================================================================
  Function Definitions
========================================================================== */
/*==========================================================================
MACROS
========================================================================== */
#define CAPI_LIMITER_ALIGN_TO_8(x) ((((uint32_t)(x) + 7) >> 3) << 3)

#define SAL_KPPS_MONO_8K 200
#define SAL_KPPS_MONO_16K 270
#define SAL_KPPS_MONO_32K 410
#define SAL_KPPS_MONO_48K 550
/*==========================================================================
  Function Definitions
========================================================================== */
void capi_sal_update_raise_delay_event(capi_sal_t *me_ptr)
{
   // TODO: to call this function from wherever operating_mf, limiter_enabled, etc are changed.
   uint64_t delay = 0;
   if (capi_sal_check_limiting_required(me_ptr))
   {
      delay = ((uint64_t)me_ptr->limiter_static_vars.delay * 1000000LL) >> 15;
   }

   if (delay != me_ptr->algo_delay_us)
   {
      me_ptr->algo_delay_us = delay;
      capi_cmn_update_algo_delay_event(&me_ptr->cb_info, delay);
   }
}

/*------------------------------------------------------------------------
  Function name: capi_sal_init_limiter_cfg
  DESCRIPTION: Function to init limiter config
 * -----------------------------------------------------------------------*/
void capi_sal_init_limiter_cfg(capi_sal_t *me_ptr)
{
   me_ptr->limiter_params.ch_idx         = 0;
   me_ptr->limiter_params.gc             = SAL_LIM_GC_Q15;
   me_ptr->limiter_params.makeup_gain    = SAL_LIM_MAKEUP_GAIN_Q8;
   me_ptr->limiter_params.max_wait       = SAL_LIM_DELAY_1MS_Q15;
   me_ptr->limiter_params.threshold      = SAL_LIM_32BIT_THRESHOLD_Q27;
   me_ptr->limiter_params.gain_attack    = SAL_LIM_GAIN_ATTACK_CONSTANT_Q31;
   me_ptr->limiter_params.gain_release   = SAL_LIM_GAIN_RELEASE_CONSTANT_Q31;
   me_ptr->limiter_params.attack_coef    = SAL_LIM_GAIN_ATTACK_COEFFICIENT_Q15;
   me_ptr->limiter_params.release_coef   = SAL_LIM_GAIN_RELEASE_COEFFICIENT_Q15;
   me_ptr->limiter_params.hard_threshold = SAL_LIM_32BIT_HARD_THRESHOLD_Q27;
   me_ptr->limiter_static_vars.delay     = SAL_LIM_DELAY_1MS_Q15;
}

/*------------------------------------------------------------------------
  Function name: capi_sal_destroy_scratch_ptr_buf
  DESCRIPTION: Function that destroys the scratch memory used by
               the BW convertor and accumulator
 * -----------------------------------------------------------------------*/
capi_err_t capi_sal_destroy_scratch_ptr_buf(capi_sal_t *me_ptr)
{
   if (NULL != me_ptr->acc_out_scratch_arr)
   {
      posal_memory_aligned_free((void *)me_ptr->acc_out_scratch_arr);
   }
   return CAPI_EOK;
}

capi_err_t capi_sal_algo_reset(capi_sal_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr->lib_mem.mem_ptr)
   {
      SAL_MSG(me_ptr->iid, DBG_HIGH_PRIO, "lib memory NULL in algo_reset");
      return capi_result;
   }
   for (uint32_t i = 0; i < me_ptr->operating_mf_ptr->format.num_channels; i++)
   {
      me_ptr->limiter_params.ch_idx = i;
      if (LIMITER_SUCCESS != limiter_set_param(&me_ptr->lib_mem, LIMITER_PARAM_RESET, NULL, 0))
      {
         capi_result |= CAPI_EFAILED;
         SAL_MSG(me_ptr->iid, DBG_ERROR_PRIO, "set lim tuning param failed with capi_result %d", capi_result);
         break;
      }
   }
   return capi_result;
}

/*------------------------------------------------------------------------
  Function name: capi_sal_bps_to_qfactor
  DESCRIPTION: Function that Returns the corresponding Q-Format to
               the provided bits per sample value
 * -----------------------------------------------------------------------*/
uint32_t capi_sal_bps_to_qfactor(uint32_t bps)
{
   switch (bps)
   {
      case BIT_WIDTH_16:
      {
         return QF_BPS_16;
      }
      case BIT_WIDTH_24:
      {
         return QF_BPS_24;
      }
      case BIT_WIDTH_32:
      {
         return QF_BPS_32;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO, "capi_sal_bps_to_qfactor: Unsupported bit width %lu", bps);
         return 0;
      }
   }
}

/*------------------------------------------------------------------------
  Function name: capi_sal_qf_to_bps
  DESCRIPTION: Function that returns the corresponding bit-width to the provided q_factor
 * -----------------------------------------------------------------------*/
uint32_t capi_sal_qf_to_bps(uint32_t qf)
{
   switch (qf)
   {
      case PCM_Q_FACTOR_15:
      {
         return BIT_WIDTH_16;
      }
      case PCM_Q_FACTOR_27:
      {
         return BIT_WIDTH_24;
      }
      case PCM_Q_FACTOR_31:
      {
         return BIT_WIDTH_32;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO, "capi_sal_qf_to_bps: Unsupported q_factor %lu", qf);
         return 0;
      }
   }
}

/*------------------------------------------------------------------------
  Function name: capi_sal_raise_out_mf
  DESCRIPTION: Function that raises the output MF to the fwk
 * -----------------------------------------------------------------------*/
capi_err_t capi_sal_raise_out_mf(capi_sal_t *me_ptr, capi_media_fmt_v2_t *mf_ptr, bool_t *mf_raised_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   bool_t     mf_changed  = TRUE;

   if (NULL == mf_ptr)
   {
      SAL_MSG(me_ptr->iid, DBG_ERROR_PRIO, "capi_sal_qf_to_bps: media format ptr is NULL");
      return CAPI_EFAILED;
   }

   mf_changed = !capi_cmn_media_fmt_equal(&(me_ptr->last_raised_out_mf), mf_ptr);

   if (mf_changed)
   {
      memscpy(&me_ptr->last_raised_out_mf, sizeof(me_ptr->last_raised_out_mf), mf_ptr, sizeof(capi_media_fmt_v2_t));
      // override with valid output bps and qf if mode is valid
      if (SAL_PARAM_VALID == me_ptr->bps_cfg_mode)
      {
         me_ptr->last_raised_out_mf.format.bits_per_sample = me_ptr->out_port_cache_cfg.word_size_bytes
                                                             << 3; // 16 or 32
         me_ptr->last_raised_out_mf.format.q_factor = me_ptr->out_port_cache_cfg.q_factor;
      }
      capi_result = capi_cmn_output_media_fmt_event_v2(&me_ptr->cb_info, &me_ptr->last_raised_out_mf, FALSE, 0);
      capi_sal_update_raise_delay_event(me_ptr);
   }

   *mf_raised_ptr = mf_changed;

   // reset the mf raise flag, this is set when data is produced and mf needs to be sent on the next cycle
   // Once it is raised dont have to raise again
   me_ptr->module_flags.raise_mf_on_next_process = FALSE;
   return capi_result;
}
void capi_sal_process_metadata_after_process(capi_sal_t *        me_ptr,
                                             capi_stream_data_t *input[],
                                             capi_stream_data_t *output[],
                                             uint32_t            max_num_samples_per_ch,
                                             uint32_t            input_word_size_bytes,
                                             uint32_t            num_active_inputs)
{
   uint32_t               num_ports_flush_eos = 0;
   capi_stream_data_v2_t *out_stream_ptr      = (capi_stream_data_v2_t *)output[0];
   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      if ((DATA_PORT_CLOSED == me_ptr->in_port_arr[i].state) || (NULL == input[i]) || (NULL == input[i]->buf_ptr))
      {
         // means that the port is inactive, move on
         continue;
      }

      capi_stream_data_v2_t *in_stream_ptr = (capi_stream_data_v2_t *)input[i];

      // Destroy MD if any of the following are TRUE
      //   1. If input is marked for data_drop
      //   2. If there is no valid operating media fmt ptr (when looping over the inputs, this can become NULL)
      //   3. If the input is STOPPED, then no point in propagating MD.
      // Scenario :-
      //  Consider a voice call graph with Tx DTMF Generator path added (not enabled)
      //  using Splitter and SAL such that, first input of SAL is voice call & second input of
      //  SAL is DTMF Gen Path. When the voice call is stopped, after data process the metadata is handled here.
      //  EOS on the first input port is processed and marked at gap. There is no valid OMF at this point.
      //  There is a pending EOS on the second input port (splitter and SISO DTMF). Processing this
      //  directly without any check for OMF validity leads to a crash. The issue is exposed due to the
      //  order of connections to the SAL.
      if ((me_ptr->in_port_arr[i].port_flags.data_drop) || (NULL == me_ptr->operating_mf_ptr) ||
          (DATA_PORT_STOPPED == me_ptr->in_port_arr[i].state))
      {
         capi_sal_destroy_md_list(me_ptr, &(in_stream_ptr->metadata_list_ptr));
         in_stream_ptr->flags.marker_eos   = FALSE;
         in_stream_ptr->flags.end_of_frame = FALSE;
      }
      else
      {
         // copies EOS to output stream based on algo delay. Use max input samples per ch as delay is in the common leg
         // of
         // the SAL and common leg takes max_num_samples_per_ch samples.
         // see capi_sal_process_metadata_b4_output
         capi_sal_check_process_metadata(me_ptr,
                                         in_stream_ptr,
                                         out_stream_ptr,
                                         (max_num_samples_per_ch * input_word_size_bytes),
                                         &me_ptr->in_port_arr[i],
                                         i,
                                         &num_ports_flush_eos);
      }
   }

   // check for unmixed output & process other input md flags before destroying MD at the output of SAL
   //(note: in case DTMF gen sends EOS then it will not be destroyed here because unmixed_output=False in
   // capi_sal_handle_metadata_b4_process) missing case: if DTMF gen starts sending some other metadata which needs to
   // be propagated, then we will destroy it here.
   if (me_ptr->module_flags.unmixed_output && !me_ptr->module_flags.process_other_input_md)
   {
#ifdef SAL_DBG_LOW
      SAL_MSG(me_ptr->iid,
              DBG_LOW_PRIO,
              "Drop all MD at output. unmixed_output %d process_other_input_md %d",
              me_ptr->module_flags.unmixed_output,
              me_ptr->module_flags.process_other_input_md);
#endif

      module_cmn_md_list_t *node_ptr = out_stream_ptr->metadata_list_ptr;
      module_cmn_md_list_t *next_ptr = NULL;
      while (node_ptr)
      {
         next_ptr = node_ptr->next_ptr;

         me_ptr->metadata_handler.metadata_destroy(me_ptr->metadata_handler.context_ptr,
                                                   node_ptr,
                                                   TRUE /* is dropped*/,
                                                   &out_stream_ptr->metadata_list_ptr);

         node_ptr = next_ptr;
      }
      out_stream_ptr->flags.marker_eos = FALSE; // since EOS is also destroyed.
      return;
   }

#ifdef SAL_DBG_LOW
   SAL_MSG(me_ptr->iid,
           DBG_LOW_PRIO,
           "MD_DBG unmixed_output %d process_other_input_md %d",
           me_ptr->module_flags.unmixed_output,
           me_ptr->module_flags.process_other_input_md);
#endif

   // If more than one input active, then EOS is marked as non-flushing (except for last one - one with max offset
   // beyond which there's no data)
   //  me_ptr->metadata_handler.metadata_modify_at_data_flow_start is not used because of last_eos_md_ptr logic which
   //  reverses non-flushing to flushing.
   // Container takes care of destroying internal non-flushing EOSes when it copies metadata b/w ports.
   if (num_active_inputs > 1)
   {
      // go through each MD and mark as non-flushing
      module_cmn_md_list_t *node_ptr        = out_stream_ptr->metadata_list_ptr;
      module_cmn_md_list_t *next_ptr        = NULL;
      module_cmn_md_t *     last_eos_md_ptr = NULL;
      while (node_ptr)
      {
         next_ptr = node_ptr->next_ptr;

         module_cmn_md_t *    md_ptr            = (module_cmn_md_t *)node_ptr->obj_ptr;
         module_cmn_md_eos_t *eos_md_ptr        = NULL;
         bool_t               flush_eos_present = FALSE;

         if (MODULE_CMN_MD_ID_EOS == md_ptr->metadata_id)
         {
            bool_t out_of_band = md_ptr->metadata_flag.is_out_of_band;
            if (out_of_band)
            {
               eos_md_ptr = (module_cmn_md_eos_t *)md_ptr->metadata_ptr;
            }
            else
            {
               eos_md_ptr = (module_cmn_md_eos_t *)&(md_ptr->metadata_buf);
            }

            flush_eos_present = eos_md_ptr->flags.is_flushing_eos;

            SAL_MSG_ISLAND(me_ptr->iid,
                           DBG_LOW_PRIO,
                           "EOS metadata found 0x%p, flush_eos_present %u. Marking as non-flushing",
                           eos_md_ptr,
                           flush_eos_present);

            if (flush_eos_present)
            {
               eos_md_ptr->flags.is_flushing_eos = MODULE_CMN_MD_EOS_NON_FLUSHING;
            }

            if (NULL == last_eos_md_ptr)
            {
               last_eos_md_ptr = md_ptr;
            }
            else
            {
               if (last_eos_md_ptr->offset < md_ptr->offset)
               {
                  last_eos_md_ptr = md_ptr;
               }
            }
         }
         else if (MODULE_CMN_MD_ID_DFG == md_ptr->metadata_id)
         {
            // if num active outputs > 1, absorb DFG
            me_ptr->metadata_handler.metadata_destroy(me_ptr->metadata_handler.context_ptr,
                                                      node_ptr,
                                                      TRUE /* is dropped*/,
                                                      &out_stream_ptr->metadata_list_ptr);
            if (node_ptr == out_stream_ptr->metadata_list_ptr)
            {
               out_stream_ptr->metadata_list_ptr = next_ptr;
            }
         }
         else
         {
            SAL_MSG_ISLAND(me_ptr->iid, DBG_LOW_PRIO, "metadata 0x%lx ", md_ptr->metadata_id);
         }

         node_ptr = next_ptr;
      }

#ifdef SAL_DBG_LOW
      SAL_MSG(me_ptr->iid,
              DBG_LOW_PRIO,
              "num_ports_flush_eos %lu num_active_inputs %lu ",
              num_ports_flush_eos,
              num_active_inputs);
#endif

      // if all ports have flushing EOS, then keep the the last EOS as flushing (others were converted to non-flushing
      // above)
      if ((num_ports_flush_eos == num_active_inputs) && last_eos_md_ptr)
      {
         bool_t               out_of_band = last_eos_md_ptr->metadata_flag.is_out_of_band;
         module_cmn_md_eos_t *eos_md_ptr  = NULL;
         if (out_of_band)
         {
            eos_md_ptr = (module_cmn_md_eos_t *)last_eos_md_ptr->metadata_ptr;
         }
         else
         {
            eos_md_ptr = (module_cmn_md_eos_t *)&(last_eos_md_ptr->metadata_buf);
         }

         // last EOS after
         eos_md_ptr->flags.is_flushing_eos = TRUE;
         out_stream_ptr->flags.marker_eos  = TRUE;
         SAL_MSG_ISLAND(me_ptr->iid,
                        DBG_LOW_PRIO,
                        "Marking EOS as flushing as all active ports %lu have flushing EOS",
                        num_active_inputs);
      }
      else
      {
         if (out_stream_ptr->flags.marker_eos)
         {
            SAL_MSG_ISLAND(me_ptr->iid,
                           DBG_LOW_PRIO,
                           "Marking flushing EOS as non-flushing as num_active_inputs %lu",
                           num_active_inputs);
            out_stream_ptr->flags.marker_eos = FALSE;
         }
      }
   }
   else
   {
      // if only one input, keeps flushing eos as flushing.
      if (out_stream_ptr->flags.marker_eos)
      {
         SAL_MSG_ISLAND(me_ptr->iid,
                        DBG_LOW_PRIO,
                        "Keeping flushing EOS as flushing as num_active_inputs = %lu",
                        num_active_inputs);

         out_stream_ptr->flags.end_of_frame = TRUE;
      }

      if (capi_sal_sdata_has_dfg(me_ptr, out_stream_ptr))
      {
         SAL_MSG_ISLAND(me_ptr->iid,
                        DBG_LOW_PRIO,
                        "Keeping DFG as flushing as num_active_inputs = %lu",
                        num_active_inputs);
         out_stream_ptr->flags.end_of_frame = TRUE;
      }
   }
}

capi_err_t capi_sal_accept_omf_alloc_mem_and_raise_events(capi_sal_t *me_ptr,
                                                          uint32_t    port_index,
                                                          bool_t      data_produced,
                                                          bool_t *    mf_raised_ptr)
{
   capi_err_t capi_result   = CAPI_EOK;
   me_ptr->operating_mf_ptr = &me_ptr->in_port_arr[port_index].mf;

   //................................................1ms...............................................bps,num_ch)
   me_ptr->ref_acc_out_buf_len = capi_cmn_us_to_bytes(1000, me_ptr->operating_mf_ptr->format.sampling_rate, 32, 1);

   SAL_MSG_ISLAND(me_ptr->iid,
                  DBG_MED_PRIO,
                  "port_index %lu is the ref port, Scratch buf len = %lu",
                  port_index,
                  me_ptr->ref_acc_out_buf_len);

   uint32_t lim_qf         = me_ptr->operating_mf_ptr->format.q_factor;
   uint32_t lim_data_width = me_ptr->operating_mf_ptr->format.bits_per_sample;

   if (QF_BPS_32 == me_ptr->operating_mf_ptr->format.q_factor)
   {
      me_ptr->module_flags.op_mf_requires_limiting = FALSE;
   }
   else
   {
      me_ptr->module_flags.op_mf_requires_limiting = TRUE;
   }

   // if valid
   if (SAL_PARAM_VALID == me_ptr->bps_cfg_mode)
   {
      lim_qf         = SAL_MIN(me_ptr->out_port_cache_cfg.q_factor, lim_qf);
      lim_data_width = capi_sal_qf_to_bps(lim_qf);
   }

   capi_result |= capi_sal_alloc_scratch_lim_mem_and_raise_events(me_ptr, lim_data_width, lim_qf);

   if (!data_produced)
   {
      capi_sal_raise_out_mf(me_ptr, me_ptr->operating_mf_ptr, mf_raised_ptr);
   }
   else
   {
      SAL_MSG_ISLAND(me_ptr->iid, DBG_MED_PRIO, "Data produced, raise output media format on next process call");
      me_ptr->module_flags.raise_mf_on_next_process = TRUE;
   }
   return capi_result;
}
/*------------------------------------------------------------------------
  Function name: capi_sal_limiter_allocate_lib_memory
  DESCRIPTION: Function that allocates and partitions limiter library memory
 * -----------------------------------------------------------------------*/
capi_err_t capi_sal_limiter_allocate_lib_memory(capi_sal_t *me_ptr)
{
   limiter_mem_req_t limiter_mem_req;
   memset(&limiter_mem_req, 0, sizeof(limiter_mem_req));
   capi_err_t result = CAPI_EOK;
   int8 *     p_temp;
   uint32_t   num_channels = me_ptr->operating_mf_ptr->format.num_channels;
   // Free old memory if alloc'ed
   if (me_ptr->limiter_memory.mem_ptr)
   {
      posal_memory_free(me_ptr->limiter_memory.mem_ptr);
      me_ptr->limiter_memory.mem_ptr = NULL;
      me_ptr->lib_mem.mem_ptr        = NULL;
      me_ptr->limiter_memory.mem_req = 0;
   }

   if (LIMITER_SUCCESS != limiter_get_mem_req_v2(&limiter_mem_req, &me_ptr->limiter_static_vars))
   {
      return CAPI_EFAILED;
   }

   // if lim cfg is pending, it means that we have a set param, so we shouldn't alter the limiter params
   if (FALSE == me_ptr->module_flags.is_lim_set_cfg_rcvd)
   {
      // else, we need to set thresholds as per bitwidth
      if (16 == me_ptr->operating_mf_ptr->format.bits_per_sample)
      {
         me_ptr->limiter_params.threshold      = SAL_LIM_16BIT_THRESHOLD_Q15;
         me_ptr->limiter_params.hard_threshold = SAL_LIM_16BIT_HARD_THRESHOLD_Q15;
      }
      else
      {
         me_ptr->limiter_params.threshold      = SAL_LIM_32BIT_THRESHOLD_Q27;
         me_ptr->limiter_params.hard_threshold = SAL_LIM_32BIT_HARD_THRESHOLD_Q27;
      }
   }

   me_ptr->limiter_memory.mem_req =
      CAPI_LIMITER_ALIGN_TO_8(limiter_mem_req.mem_size) +
      ((CAPI_LIMITER_ALIGN_TO_8(sizeof(int32_t *)) * num_channels) * 2); // one for each lim in and lim out arr

   // allocate new memory
   me_ptr->limiter_memory.mem_ptr =
      (limiter_lib_t *)posal_memory_malloc(me_ptr->limiter_memory.mem_req, (POSAL_HEAP_ID)me_ptr->heap_mem.heap_id);

   if (NULL == me_ptr->limiter_memory.mem_ptr)
   {
      SAL_MSG_ISLAND(me_ptr->iid, DBG_ERROR_PRIO, "limiter: Reallocating lib memory, Lib memory allocation failed.");
      return CAPI_ENOMEMORY;
   }
   else
   {
      memset(me_ptr->limiter_memory.mem_ptr, 0, me_ptr->limiter_memory.mem_req);
      p_temp = (int8_t *)me_ptr->limiter_memory.mem_ptr;

      // init limiter
      if (LIMITER_SUCCESS !=
          limiter_init_mem_v2(&me_ptr->lib_mem, &me_ptr->limiter_static_vars, p_temp, limiter_mem_req.mem_size))
      {
         return CAPI_EFAILED;
      }
      p_temp += CAPI_LIMITER_ALIGN_TO_8(limiter_mem_req.mem_size);
      me_ptr->lim_in_ptr = (int32_t **)p_temp;
      p_temp += CAPI_LIMITER_ALIGN_TO_8(sizeof(int32_t *) * num_channels);
      me_ptr->lim_out_ptr = (int32_t **)p_temp;
      // p_temp += CAPI_LIMITER_ALIGN_TO_8(sizeof(int32_t *) * num_channels);

      limiter_tuning_v2_t limiter_cfg;
      (void)memscpy(&limiter_cfg, sizeof(limiter_tuning_v2_t), &me_ptr->limiter_params, sizeof(limiter_tuning_v2_t));

      if ((me_ptr->module_flags.is_lim_set_cfg_rcvd) && (16 == me_ptr->operating_mf_ptr->format.bits_per_sample))
      {
         limiter_cfg.threshold      = limiter_cfg.threshold >> 12;
         limiter_cfg.hard_threshold = limiter_cfg.hard_threshold >> 12;
      }

      for (uint32_t i = 0; i < me_ptr->operating_mf_ptr->format.num_channels; i++)
      {
         me_ptr->limiter_params.ch_idx = i;
         limiter_cfg.ch_idx            = i;
         if (LIMITER_SUCCESS !=
             limiter_set_param(&me_ptr->lib_mem, LIMITER_PARAM_TUNING_V2, &limiter_cfg, sizeof(limiter_tuning_v2_t)))
         {
            result |= CAPI_EFAILED;
            SAL_MSG_ISLAND(me_ptr->iid, DBG_ERROR_PRIO, "set lim tuning param failed with capi_result %d", result);
         }
      }
   }

   return result;
}
/*------------------------------------------------------------------------
  Function name: capi_sal_bw_conv_allocate_scratch_ptr_buf
  DESCRIPTION: Function that allocates the scratch memory required for
               the BW convertor and accumulator
 * -----------------------------------------------------------------------*/
capi_err_t capi_sal_bw_conv_allocate_scratch_ptr_buf(capi_sal_t *me_ptr, uint32_t data_len)
{
   // we need to reallocate with the latest configuration
   if (NULL != me_ptr->acc_out_scratch_arr)
   {
      posal_island_trigger_island_exit();
      posal_memory_aligned_free((void *)me_ptr->acc_out_scratch_arr);
   }
#ifdef SAL_DBG_LOW
   SAL_MSG(me_ptr->iid,
           DBG_HIGH_PRIO,
           "capi_sal_bw_conv_allocate_scratch_ptr_buf: Allocating scratch arr with data len = %lu",
           data_len);
#endif // SAL_DBG_LOW
   uint32_t num_input_scratch_buffer_needed = 0;
#if __qdsp6__
   num_input_scratch_buffer_needed = 1;
#endif
   // Now to allocate the outbut scratch arr
   // first time/reallocate
   uint32_t alloc_size =
      CAPI_ALIGN_8_BYTE(me_ptr->operating_mf_ptr->format.num_channels * sizeof(capi_buf_t)) +
      ((me_ptr->operating_mf_ptr->format.num_channels + num_input_scratch_buffer_needed) * CAPI_ALIGN_8_BYTE(data_len));

   me_ptr->acc_out_scratch_arr =
      (capi_buf_t *)posal_memory_aligned_malloc(alloc_size, 8, (POSAL_HEAP_ID)me_ptr->heap_mem.heap_id);
   // the pointer shouldn't be null. or the malloc failed
   if (NULL == me_ptr->acc_out_scratch_arr)
   {
      SAL_MSG_ISLAND(me_ptr->iid, DBG_ERROR_PRIO, "Malloc failed");
      return CAPI_ENOMEMORY;
   }
   // now we need to create the association within this buf
   capi_buf_t *curr_ptr = me_ptr->acc_out_scratch_arr;
   int8_t *    curr_data_ptr =
      (int8_t *)curr_ptr + CAPI_ALIGN_8_BYTE(sizeof(capi_buf_t) * me_ptr->operating_mf_ptr->format.num_channels);
   uint32_t count = 0;
   while (count < me_ptr->operating_mf_ptr->format.num_channels)
   {
      curr_ptr->max_data_len = CAPI_ALIGN_8_BYTE(data_len);
      curr_ptr->data_ptr     = curr_data_ptr;
      curr_ptr++;
      curr_data_ptr += CAPI_ALIGN_8_BYTE(data_len);
      count++;
   }
#ifdef __qdsp6__
   if (num_input_scratch_buffer_needed)
   {
      me_ptr->acc_in_scratch_buf.data_ptr     = curr_data_ptr;
      me_ptr->acc_in_scratch_buf.max_data_len = data_len;
   }
   else
   {
      me_ptr->acc_in_scratch_buf.data_ptr     = NULL;
      me_ptr->acc_in_scratch_buf.max_data_len = 0;
   }
#endif
   // association complete.
   return CAPI_EOK;
}
/*------------------------------------------------------------------------
  Function name: capi_sal_update_kpps
  DESCRIPTION: Function that updates the kpps needed by SAL
 * -----------------------------------------------------------------------*/
capi_err_t capi_sal_update_and_raise_kpps_bw_event(capi_sal_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   uint32_t   kpps        = 0;
   uint32_t   bw          = 0;

   switch (me_ptr->operating_mf_ptr->format.sampling_rate)
   {
      case SAMPLE_RATE_8K:
      {
         kpps = SAL_KPPS_MONO_8K;
         break;
      }
      case SAMPLE_RATE_16K:
      {
         kpps = SAL_KPPS_MONO_16K;
         break;
      }
      case SAMPLE_RATE_32K:
      {
         kpps = SAL_KPPS_MONO_32K;
         break;
      }
      case SAMPLE_RATE_48K:
      {
         kpps = SAL_KPPS_MONO_48K;
         break;
      }
      default:
      {
         kpps = SAL_KPPS_MONO_48K;
         break;
      }
   }
   kpps *= me_ptr->operating_mf_ptr->format.num_channels;
   uint64_t data_rate = me_ptr->operating_mf_ptr->format.sampling_rate *
                        me_ptr->operating_mf_ptr->format.num_channels *
                        (me_ptr->operating_mf_ptr->format.bits_per_sample >> 3);
   bw += (data_rate * 4); // input to internal and internal to output
                          // dervied the equation from sync module
   
   me_ptr->events_config.sal_bw = bw;
   capi_result                  = capi_cmn_update_bandwidth_event(&me_ptr->cb_info, 0, me_ptr->events_config.sal_bw);
   
   if (kpps != me_ptr->events_config.sal_kpps)
   {
      me_ptr->events_config.sal_kpps = kpps;
      capi_result                    |= capi_cmn_update_kpps_event(&me_ptr->cb_info, me_ptr->events_config.sal_kpps);
   }

   return capi_result;
}

capi_err_t capi_sal_alloc_scratch_lim_mem_and_raise_events(capi_sal_t *me_ptr,
                                                           uint32_t    lim_data_width,
                                                           uint32_t    q_factor)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr)
   {
      return CAPI_EBADPARAM;
   }
   if (CAPI_EOK != (capi_result = capi_sal_bw_conv_allocate_scratch_ptr_buf(me_ptr, me_ptr->ref_acc_out_buf_len)))
   {
      return capi_result;
   }

   me_ptr->limiter_static_vars.data_width  = (BIT_WIDTH_24 == lim_data_width) ? BIT_WIDTH_32 : lim_data_width;
   me_ptr->limiter_static_vars.q_factor    = q_factor;
   me_ptr->limiter_static_vars.num_chs     = me_ptr->operating_mf_ptr->format.num_channels;
   me_ptr->limiter_static_vars.sample_rate = me_ptr->operating_mf_ptr->format.sampling_rate;
   me_ptr->limiter_static_vars.max_block_size =
      capi_cmn_us_to_samples(me_ptr->cfg_lim_block_size_ms * 1000, me_ptr->operating_mf_ptr->format.sampling_rate);

#ifdef SAL_DBG_LOW
   SAL_MSG(me_ptr->iid,
           DBG_HIGH_PRIO,
           "capi_sal_alloc_scratch_lim_mem_and_raise_events: Allocating lim with max_block_size = %lu, time = %lu",
           me_ptr->limiter_static_vars.max_block_size,
           me_ptr->cfg_lim_block_size_ms);
#endif // SAL_DBG_LOW

   if (CAPI_EOK != (capi_result = capi_sal_limiter_allocate_lib_memory(me_ptr)))
   {
      return capi_result;
   }

   if (CAPI_EOK != (capi_result = capi_sal_update_and_raise_kpps_bw_event(me_ptr)))
   {
      return capi_result;
   }
   return capi_result;
}
