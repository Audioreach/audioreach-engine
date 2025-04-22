/**
 * clang-format off
 * \file spl_cntr_voice_delivery_fwk_ext.c
 * \brief
 *  This file contains utility functions for FWK_EXTN_VOICE_DELIVERY
 *
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */
// clang-format on

/* =======================================================================
Includes
========================================================================== */
#include "spl_cntr_voice_delivery_fwk_ext.h"

/* =======================================================================
Static function declarations
========================================================================== */
static ar_result_t spl_cntr_fwk_extn_voice_delivery_proc_start_trigger(cu_base_t *base_ptr, uint32_t channel_bit_index);
static ar_result_t spl_cntr_fwk_extn_voice_delivery_handle_proc_tick_notif(spl_cntr_t *me_ptr);

/* =======================================================================
Function definitions
========================================================================== */
ar_result_t spl_cntr_fwk_extn_voice_delivery_handle_topo_proc_notif(spl_cntr_t *me_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result = AR_EOK;

   /* To find the module ptr to do set param*/
   for (gu_sg_list_t *sg_list_ptr = me_ptr->cu.gu_ptr->sg_list_ptr; (NULL != sg_list_ptr); LIST_ADVANCE(sg_list_ptr))
   {
      for (gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != module_list_ptr);
           LIST_ADVANCE(module_list_ptr))
      {
         spl_topo_module_t *curr_module_ptr = (spl_topo_module_t *)module_list_ptr->module_ptr;

         // Only send topo_proc notifications while smart_sync disabled the threshold. Once sync is achieved,
         // these notifications are no longer required.
         if (curr_module_ptr->flags.need_voice_delivery_extn &&
             curr_module_ptr->t_base.flags.is_threshold_disabled)
         {
            TRY(result,
                gen_topo_capi_set_param(me_ptr->topo.t_base.gu.log_id,
                                        curr_module_ptr->t_base.capi_ptr,
                                        FWK_EXTN_VOICE_DELIVERY_PARAM_ID_TOPO_PROCESS_NOTIF,
                                        NULL, // payload
                                        0 /*payload_size*/));

            break;
         }
      }
   }

   CATCH(result, SPL_CNTR_MSG_PREFIX, me_ptr->topo.t_base.gu.log_id)
   {
   }
   return result;
}

static ar_result_t spl_cntr_fwk_extn_voice_delivery_handle_proc_tick_notif(spl_cntr_t *me_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result = AR_EOK;

   /* To find the module ptr to do set param*/
   for (gu_sg_list_t *sg_list_ptr = me_ptr->cu.gu_ptr->sg_list_ptr; (NULL != sg_list_ptr); LIST_ADVANCE(sg_list_ptr))
   {
      for (gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != module_list_ptr);
           LIST_ADVANCE(module_list_ptr))
      {
         spl_topo_module_t *curr_module_ptr = (spl_topo_module_t *)module_list_ptr->module_ptr;

         // Only send proc tick notifications while smart_sync disabled the threshold. Once sync is achieved,
         // these notifications are no longer required.
         if (curr_module_ptr->flags.need_voice_delivery_extn &&
             curr_module_ptr->t_base.flags.is_threshold_disabled)
         {
            TRY(result,
                gen_topo_capi_set_param(me_ptr->topo.t_base.gu.log_id,
                                        curr_module_ptr->t_base.capi_ptr,
                                        FWK_EXTN_VOICE_DELIVERY_PARAM_ID_FIRST_PROC_TICK_NOTIF,
                                        NULL, // payload
                                        0 /*payload_size*/));

            break;
         }
      }
   }

   CATCH(result, SPL_CNTR_MSG_PREFIX, me_ptr->topo.t_base.gu.log_id)
   {
   }
   return result;
}

static ar_result_t spl_cntr_fwk_extn_voice_delivery_resync_trigger(cu_base_t *base_ptr, uint32_t channel_bit_index)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result = AR_EOK;
   spl_cntr_t *me_ptr = (spl_cntr_t *)base_ptr;

   /* To find the module ptr to do set param*/
   for (gu_sg_list_t *sg_list_ptr = me_ptr->cu.gu_ptr->sg_list_ptr; (NULL != sg_list_ptr); LIST_ADVANCE(sg_list_ptr))
   {
      for (gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != module_list_ptr);
           LIST_ADVANCE(module_list_ptr))
      {
         spl_topo_module_t *curr_module_ptr = (spl_topo_module_t *)module_list_ptr->module_ptr;

         if (curr_module_ptr->flags.need_voice_delivery_extn)
         {
            TRY(result,
                gen_topo_capi_set_param(me_ptr->topo.t_base.gu.log_id,
                                        curr_module_ptr->t_base.capi_ptr,
                                        FWK_EXTN_VOICE_DELIVERY_PARAM_ID_RESYNC_NOTIF,
                                        NULL, // payload
                                        0 /*payload_size*/));

            break;
         }
      }
   }

   uint32_t all_mask = ~me_ptr->cu.available_bit_mask;
   uint32_t num_iter = 1;
/*
 * CR 3280663, 3259611
 * Time at which resync notification is received and the next proc tick  are very close. And smart-sync is not able to
 * pull all the data from queue to its circular buffer, so it is inserting extra zeros while making first rendering
 * decision. Later when data is pulled from the queue, it creates an internal buffering in VpTx.
 *
 * Since VCPM starts the resync handling only when Tx path is idle and in case where there is not
 * much processing margin, we can expect VCPM to start resync_handling just before the next VpTx proc tick. So, this
 * scenario is not so rare.
 *
 * This issue is only specific to 40ms VFR period and 10m VpTx frame len. This is because VpTx gets at least two chances
 * to buffer the data, one immediately after resync notification and another at proc tick itself. It can buffer more in
 * between if it gets a chance to schedule. But we can guarantee trigger at least two times. So, VFR-40 -threshold-20
 * and VFR-20-threshold-10 is not a problem as two triggers are sufficient to buffer all the data from the queue.
 * Now going with this logic, If we can ensure VpTx trigger at least three times before proc tick then we can guarantee
 * that all the data will be buffered when VpTx triggers at proc-tick. This can be ensured by keeping a loop (with 3
 * iteration) in resync handling function (isolated to only resync handling case).
 * */

#if 0
   num_ter = cu_frames_per_period(&me_ptr->cu) - 1;
#else
   if (10000 == me_ptr->cu.cntr_frame_len.frame_len_us && 40000 == me_ptr->cu.period_us)
   {
      num_iter = 3;
   }
#endif

   for (uint32_t i = 0; i < num_iter; i++)
   {
      TRY(result, spl_cntr_check_and_process_audio(me_ptr, all_mask));
   }

   CATCH(result, SPL_CNTR_MSG_PREFIX, me_ptr->topo.t_base.gu.log_id)
   {
   }

   posal_signal_clear(me_ptr->fwk_extn_info.resync_signal_ptr);

   return AR_EOK;
}
/**
 * Callback function associated with vptx proc start signal. Gets
 * called every time the voice timer for vptx proc start tick expires.
 */
static ar_result_t spl_cntr_fwk_extn_voice_delivery_proc_start_trigger(cu_base_t *base_ptr, uint32_t channel_bit_index)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result = AR_EOK;
   spl_cntr_t *me_ptr = (spl_cntr_t *)base_ptr;

   // Caller of this function already checks if there is a voice del extn
   if (!base_ptr->flags.is_cntr_started)
   {
      SPL_CNTR_MSG(me_ptr->topo.t_base.gu.log_id,
                   DBG_HIGH_PRIO,
                   "voc proc start trigger received while no sgs are started, ignoring.");
      posal_signal_clear(me_ptr->fwk_extn_info.proc_tick_signal_ptr);
      return result;
   }

   SPL_CNTR_MSG(me_ptr->topo.t_base.gu.log_id, DBG_HIGH_PRIO, "PROC_DELAY_DEBUG: voc proc start trigger received");

   TRY(result, spl_cntr_fwk_extn_voice_delivery_handle_proc_tick_notif(me_ptr));

   // Do not update trigger policy if it is in output buffer trigger mode.
   if (VOICE_PROC_TRIGGER_MODE == me_ptr->trigger_policy)
   {
      me_ptr->trigger_policy = DEFAULT_TRIGGER_MODE;
   }

   uint32_t all_mask = ~me_ptr->cu.available_bit_mask;
   TRY(result, spl_cntr_check_and_process_audio(me_ptr, all_mask));

   CATCH(result, SPL_CNTR_MSG_PREFIX, me_ptr->topo.t_base.gu.log_id)
   {
   }

   // Even for errors, make sure to clear the proc tick signal.
   posal_signal_clear(me_ptr->fwk_extn_info.proc_tick_signal_ptr);

   return result;
}

/* Function to check if nay module in the container supports FWK_EXTN_VOICE_DELIVERY*/
bool_t spl_cntr_fwk_extn_voice_delivery_found(spl_cntr_t *me_ptr, spl_topo_module_t **found_module_pptr)
{
   for (gu_sg_list_t *sg_list_ptr = me_ptr->cu.gu_ptr->sg_list_ptr; (NULL != sg_list_ptr); LIST_ADVANCE(sg_list_ptr))
   {
      for (gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != module_list_ptr);
           LIST_ADVANCE(module_list_ptr))
      {
         spl_topo_module_t *curr_module_ptr = (spl_topo_module_t *)module_list_ptr->module_ptr;
         if (curr_module_ptr->flags.need_voice_delivery_extn)
         {
            if(found_module_pptr)
            {
               *found_module_pptr = curr_module_ptr;
            }
            return TRUE;
         }
      }
   }

   if(found_module_pptr)
   {
      *found_module_pptr = NULL;
   }
   return FALSE;
}

/* Drop data messages which are < 1ms in VPTX.
 * - Upstream messages are expected to be >= 1ms.
 * - IIR resampler within MFC requires operation at fixed frame size. For threshold-disabled scenario, we use
 *   1ms frame size.
 * - We want to avoid scenarios in VPTX where multiple data msgs get buffered in a single trigger to avoid corner-case
 *   timing scenarios.
 */
bool_t spl_cntr_fwk_extn_voice_delivery_need_drop_data_msg(spl_cntr_t *me_ptr, spl_cntr_ext_in_port_t *ext_in_port_ptr)
{
   if (!me_ptr->is_voice_delivery_cntr)
   {
      return FALSE;
   }

   uint64_t               DROP_THRESHOLD_US    = 800;
   uint64_t *             FRAC_TIME_PTR_UNUSED = NULL;
   spf_msg_header_t *     header_ptr           = (spf_msg_header_t *)(ext_in_port_ptr->cu.input_data_q_msg.payload_ptr);
   spf_msg_data_buffer_t *data_msg_buf_ptr     = (spf_msg_data_buffer_t *)&header_ptr->payload_start;

   uint64_t inp_len_us =
      topo_bytes_to_us(data_msg_buf_ptr->actual_size, &ext_in_port_ptr->cu.media_fmt, FRAC_TIME_PTR_UNUSED);

   return inp_len_us < DROP_THRESHOLD_US;
}

/**
 * Helper function to send vptx proc start signal info to smart sync module for voice timer registration
 */
ar_result_t spl_cntr_fwk_extn_voice_delivery_send_proc_start_signal_info(spl_cntr_t *       me_ptr,
                                                                         spl_topo_module_t *module_ptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING

   // Do nothing if voice delivery not found.
   if (FALSE == module_ptr->flags.need_voice_delivery_extn)
   {
      return AR_EOK;
   }

   // Enable the stm control
   SPL_CNTR_MSG(me_ptr->topo.t_base.gu.log_id,
                DBG_HIGH_PRIO,
                "sending vptx proc start signal info to voice del module");

   me_ptr->fwk_extn_info.proc_tick_signal_ptr = NULL;
   me_ptr->fwk_extn_info.resync_signal_ptr    = NULL;

   TRY(result,
       cu_init_signal(&me_ptr->cu,
                      SPL_CNTR_VOICE_TIMER_TICK_BIT_MASK,
                      spl_cntr_fwk_extn_voice_delivery_proc_start_trigger,
                      &(me_ptr->fwk_extn_info.proc_tick_signal_ptr)));

   TRY(result,
       cu_init_signal(&me_ptr->cu,
                      SPL_CNTR_VOICE_RESYNC_BIT_MASK,
                      spl_cntr_fwk_extn_voice_delivery_resync_trigger,
                      &(me_ptr->fwk_extn_info.resync_signal_ptr)));

   /* Property structure for vptx proc start trigger */
   typedef struct
   {
      capi_custom_property_t               cust_prop;
      capi_prop_voice_proc_start_trigger_t trigger;
   } vptx_proc_start_trigger_t;

   vptx_proc_start_trigger_t vptx_proc_start_trigger;

   /* Populate the stm trigger */
   vptx_proc_start_trigger.cust_prop.secondary_prop_id   = FWK_EXTN_PROPERTY_ID_VOICE_PROC_START_TRIGGER;
   vptx_proc_start_trigger.trigger.proc_start_signal_ptr = (void *)(me_ptr->fwk_extn_info.proc_tick_signal_ptr);
   vptx_proc_start_trigger.trigger.resync_signal_ptr     = (void *)(me_ptr->fwk_extn_info.resync_signal_ptr);

   capi_prop_t set_props[] = {
      { CAPI_CUSTOM_PROPERTY,
        { (int8_t *)(&vptx_proc_start_trigger), sizeof(vptx_proc_start_trigger), sizeof(vptx_proc_start_trigger) },
        { FALSE, FALSE, 0 } }
   };

   capi_proplist_t set_proplist = { SIZE_OF_ARRAY(set_props), set_props };

   if (CAPI_EOK !=
       (result = module_ptr->t_base.capi_ptr->vtbl_ptr->set_properties(module_ptr->t_base.capi_ptr, &set_proplist)))
   {
      SPL_CNTR_MSG(me_ptr->topo.t_base.gu.log_id,
                   DBG_ERROR_PRIO,
                   "Failed to apply vptx proc start trigger, result: %d",
                   result);

      THROW(result, AR_EFAILED);
   }

   CATCH(result, SPL_CNTR_MSG_PREFIX, me_ptr->topo.t_base.gu.log_id)
   {
      cu_deinit_signal(&me_ptr->cu, &me_ptr->fwk_extn_info.proc_tick_signal_ptr);
      cu_deinit_signal(&me_ptr->cu, &me_ptr->fwk_extn_info.resync_signal_ptr);
   }

   return result;
}

/**
 * Called during graph_close for voice delivery related cleanup.
 */
ar_result_t spl_cntr_fwk_extn_voice_delivery_close(spl_cntr_t *me_ptr)
{
   ar_result_t result = AR_EOK;

   if (me_ptr->fwk_extn_info.proc_tick_signal_ptr)
   {
      SPL_CNTR_MSG(me_ptr->topo.t_base.gu.log_id,
                   DBG_HIGH_PRIO,
                   "vptx proc start trigger signal ptr: 0x%lx",
                   me_ptr->fwk_extn_info.proc_tick_signal_ptr);

      cu_stop_listen_to_mask(&me_ptr->cu, SPL_CNTR_VOICE_TIMER_TICK_BIT_MASK);
      posal_signal_destroy(&me_ptr->fwk_extn_info.proc_tick_signal_ptr);
   }

   if (me_ptr->fwk_extn_info.resync_signal_ptr)
   {
      SPL_CNTR_MSG(me_ptr->topo.t_base.gu.log_id,
                   DBG_HIGH_PRIO,
                   "vptx resync signal ptr: 0x%lx",
                   me_ptr->fwk_extn_info.resync_signal_ptr);

      cu_stop_listen_to_mask(&me_ptr->cu, SPL_CNTR_VOICE_RESYNC_BIT_MASK);
      posal_signal_destroy(&me_ptr->fwk_extn_info.resync_signal_ptr);
   }

   return result;
}

ar_result_t spl_cntr_fwk_extn_voice_delivery_handle_timestamp(spl_cntr_t *            me_ptr,
                                                              spl_cntr_ext_in_port_t *ext_in_port_ptr,
                                                              spf_msg_data_buffer_t * data_buf_ptr)
{
   INIT_EXCEPTION_HANDLING

   ar_result_t result             = AR_EOK;
   uint64_t *  FRAC_TIME_PTR_NULL = NULL;

   if (ext_in_port_ptr->vptx_ts_valid)
   {
      uint64_t ts_diff = (data_buf_ptr->timestamp < ext_in_port_ptr->vptx_next_expected_ts)
                            ? 0
                            : (data_buf_ptr->timestamp - ext_in_port_ptr->vptx_next_expected_ts);

      if (ts_diff >= 1000)
      {
         spl_topo_module_t *voice_delivery_module_ptr = NULL;
         spl_cntr_fwk_extn_voice_delivery_found(me_ptr, &voice_delivery_module_ptr);
         if((voice_delivery_module_ptr) && (voice_delivery_module_ptr->t_base.flags.is_threshold_disabled))
         {
            SPL_CNTR_MSG(me_ptr->topo.t_base.gu.log_id,
                         DBG_ERROR_PRIO,
                         "input port miid 0x%lx idx %ld vptx found timestamp discontinuity of %ld us during syncing state, "
                         "sending set_param to voice delivery module miid 0x%lx, incomming ts (0x%lx, 0x%lx), next "
                         "exptected ts (0x%lx, 0x%lx)",
                         ext_in_port_ptr->gu.int_in_port_ptr->cmn.module_ptr->module_instance_id,
                         ext_in_port_ptr->gu.int_in_port_ptr->cmn.id,
                         ts_diff,
                         voice_delivery_module_ptr->t_base.gu.module_instance_id,
                         (int32_t) (data_buf_ptr->timestamp >> 32),
                         data_buf_ptr->timestamp,
                         (int32_t) (ext_in_port_ptr->vptx_next_expected_ts >> 32),
                         ext_in_port_ptr->vptx_next_expected_ts);

            TRY(result,
                gen_topo_capi_set_param(me_ptr->topo.t_base.gu.log_id,
                                        voice_delivery_module_ptr->t_base.capi_ptr,
                                        FWK_EXTN_VOICE_DELIVERY_PARAM_ID_DATA_DROP_DURING_SYNC,
                                        NULL, // payload
                                        0 /*payload_size*/));
         }
         else
         {
            // Round to closest 1ms, so we always push in multiples of 1ms.
            ext_in_port_ptr->vptx_ts_zeros_to_push_us = TOPO_CEIL(ts_diff, 1000) * 1000;

            SPL_CNTR_MSG(me_ptr->topo.t_base.gu.log_id,
                         DBG_ERROR_PRIO,
                         "input port miid 0x%lx idx %ld vptx found timestamp discontinuity of %ld us: appending %ld us "
                         "zeros to "
                         "maintain continuity",
                         ext_in_port_ptr->gu.int_in_port_ptr->cmn.module_ptr->module_instance_id,
                         ext_in_port_ptr->gu.int_in_port_ptr->cmn.id,
                         ts_diff,
                         ext_in_port_ptr->vptx_ts_zeros_to_push_us);
         }
      }
   }

   // Next expected timestamp will come after data in the held message.
   ext_in_port_ptr->vptx_ts_valid = TRUE;
   ext_in_port_ptr->vptx_next_expected_ts =
      data_buf_ptr->timestamp +
      topo_bytes_to_us(data_buf_ptr->actual_size, &(ext_in_port_ptr->cu.media_fmt), FRAC_TIME_PTR_NULL);


   CATCH(result, SPL_CNTR_MSG_PREFIX, me_ptr->topo.t_base.gu.log_id)
   {
   }
   return result;
}

ar_result_t spl_cntr_fwk_extn_voice_delivery_push_ts_zeros(spl_cntr_t *            me_ptr,
                                                           spl_cntr_ext_in_port_t *ext_in_port_ptr,
                                                           uint32_t *              data_needed_bytes_per_ch_ptr)
{
   ar_result_t result             = AR_EOK;
   uint64_t *  FRAC_TIME_PTR_NULL = NULL;

   uint32_t zeros_remaining_bpc =
      topo_us_to_bytes_per_ch(ext_in_port_ptr->vptx_ts_zeros_to_push_us, &(ext_in_port_ptr->cu.media_fmt));
   uint32_t zeros_to_push_bpc = MIN(*data_needed_bytes_per_ch_ptr, zeros_remaining_bpc);

   uint32_t actual_data_len = ext_in_port_ptr->topo_buf.buf_ptr[0].actual_data_len;
   for (uint32_t ch = 0; ch < ext_in_port_ptr->cu.media_fmt.pcm.num_channels; ch++)
   {
      /* appending new input data to the buffer */
      memset(((int8_t *)(ext_in_port_ptr->topo_buf.buf_ptr[ch].data_ptr) + actual_data_len), 0, zeros_to_push_bpc);

#ifdef SAFE_MODE_SDATA_BUF_LENGTHS
      ext_in_port_ptr->topo_buf.buf_ptr[i].actual_data_len += zeros_to_push_bpc;
   }
#else
   }
   // optimization: update only first buffer lengths outside the for loop
   ext_in_port_ptr->topo_buf.buf_ptr[0].actual_data_len += zeros_to_push_bpc;
#endif


   if (zeros_to_push_bpc == zeros_remaining_bpc)
   {
      // in this case avoid calling topo_bytes_per_ch_to_us. This function was causing rounding error.
      /* case:
       * 	zeros_to_push_bpc = 384 (2000us)
       * 	sampling rate = 96000
       * 	num_channel = 1
       * 	bps = 16
       *
       * Even after pushing 2ms zeros, function "topo_bytes_per_ch_to_us" is returning 1999us. Which is causing
       * ext_in_port_ptr->vptx_ts_zeros_to_push_us to not reset to zero.
       *
       * 00:10:47.900 spl_cntr_voice_delivery_fwk_ext.c 433 E  ADSP: SC  :1001E000: input port miid 0x4199 idx 4 vptx found timestamp discontinuity of 1001 us: appending 2000 us zeros to maintain continuity
       * 00:10:47.900 spl_cntr_voice_delivery_fwk_ext.c 489 H  ADSP: SC  :1001E000: input port miid 0x4199 idx 4 vptx pushed 384 zeros (bpc) to maintain ts continuity, 1 us remaining, new data needed 1346
       * 00:10:47.900 spl_cntr_voice_delivery_fwk_ext.c 489 H  ADSP: SC  :1001E000: input port miid 0x4199 idx 4 vptx pushed 2 zeros (bpc) to maintain ts continuity, 0 us remaining, new data needed 1152
       * */
      ext_in_port_ptr->vptx_ts_zeros_to_push_us = 0;
   }
   else
   {
      uint32_t us_zeros_pushed =
         topo_bytes_per_ch_to_us(zeros_to_push_bpc, &(ext_in_port_ptr->cu.media_fmt), FRAC_TIME_PTR_NULL);
      ext_in_port_ptr->vptx_ts_zeros_to_push_us = (us_zeros_pushed > ext_in_port_ptr->vptx_ts_zeros_to_push_us)
                                                     ? 0
                                                     : (ext_in_port_ptr->vptx_ts_zeros_to_push_us - us_zeros_pushed);
   }

   // No need to worry about negative numbers here, as min was used above in same units.
   *data_needed_bytes_per_ch_ptr -= zeros_to_push_bpc;

   SPL_CNTR_MSG(me_ptr->topo.t_base.gu.log_id,
                DBG_HIGH_PRIO,
                "input port miid 0x%lx idx %ld vptx pushed %ld zeros (bpc) to maintain ts continuity, %ld us "
                "remaining, new data needed %ld",
                ext_in_port_ptr->gu.int_in_port_ptr->cmn.module_ptr->module_instance_id,
                ext_in_port_ptr->gu.int_in_port_ptr->cmn.id,
                zeros_to_push_bpc,
                ext_in_port_ptr->vptx_ts_zeros_to_push_us,
                *data_needed_bytes_per_ch_ptr);

   return result;
}
