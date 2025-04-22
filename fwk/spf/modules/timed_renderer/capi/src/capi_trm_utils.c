/**
 *   \file capi_trm_utils.c
 *   \brief
 *        This file contains utilities implementation of Timed Renderer Module
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_trm_utils.h"
#include "module_cmn_api.h"

static bool_t capi_trm_can_alloc_held_input_buffer(capi_trm_t *me_ptr);

static bool_t capi_trm_can_alloc_held_input_buffer(capi_trm_t *me_ptr)
{
   if ((!me_ptr->is_input_mf_received) || (0 == me_ptr->frame_size_us))
   {
      AR_MSG(DBG_MED_PRIO,
             "capi timed renderer: Not ready to alloc internal buffer, is_input_mf_received %ld nominal frame size = "
             "%ldus",
             me_ptr->is_input_mf_received,
             me_ptr->frame_size_us);
      return FALSE;
   }
   return TRUE;
}

capi_err_t capi_trm_check_alloc_held_input_buffer(capi_trm_t *me_ptr)
{
   capi_err_t result       = CAPI_EOK;
   uint32_t   mem_size     = 0;
   uint32_t   num_channels = me_ptr->media_format.format.num_channels;

   // Don't alloc if configuration didn't come yet.
   if (!capi_trm_can_alloc_held_input_buffer(me_ptr))
   {
      AR_MSG(DBG_MED_PRIO, "capi timed renderer: Couldn't allocate memory for local buffer.");
      return result;
   }

   uint32_t buf_size_per_channel_per_frame = capi_cmn_us_to_bytes_per_ch(me_ptr->frame_size_us,
                                                                         me_ptr->media_format.format.sampling_rate,
                                                                         me_ptr->media_format.format.bits_per_sample);

   // If it already exists, free the buffer.
   if (me_ptr->held_input_buf.frame_ptr)
   {
      capi_trm_flush_held_buffer(me_ptr, NULL, TRUE /*force_free_md*/);
      capi_trm_dealloc_held_input_buffer(me_ptr);
   }

   // Allocate and zero memory.
   mem_size = (sizeof(capi_trm_circ_buf_frame_t) * NUM_FRAMES_IN_LOCAL_BUFFER) +
              (sizeof(capi_buf_t) * num_channels * NUM_FRAMES_IN_LOCAL_BUFFER) +
              (buf_size_per_channel_per_frame * num_channels * NUM_FRAMES_IN_LOCAL_BUFFER);

   me_ptr->held_input_buf.frame_ptr =
      (capi_trm_circ_buf_frame_t *)posal_memory_malloc(mem_size, (POSAL_HEAP_ID)me_ptr->heap_mem.heap_id);
   if (NULL == me_ptr->held_input_buf.frame_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi timed renderer: Couldn't allocate memory for local buffer.");
      return CAPI_ENOMEMORY;
   }

   memset(me_ptr->held_input_buf.frame_ptr, 0, mem_size);

   capi_buf_t *buf_ptr = (capi_buf_t *)((int8_t *)(me_ptr->held_input_buf.frame_ptr) +
                                        (sizeof(capi_trm_circ_buf_frame_t) * NUM_FRAMES_IN_LOCAL_BUFFER));
   int8_t *data_ptr = (int8_t *)(me_ptr->held_input_buf.frame_ptr) +
                      (sizeof(capi_trm_circ_buf_frame_t) * NUM_FRAMES_IN_LOCAL_BUFFER) +
                      (sizeof(capi_buf_t) * num_channels * NUM_FRAMES_IN_LOCAL_BUFFER);
   for (uint32_t i = 0; i < NUM_FRAMES_IN_LOCAL_BUFFER; i++)
   {
      me_ptr->held_input_buf.frame_ptr[i].sdata.buf_ptr = buf_ptr;
      for (uint32_t j = 0; j < num_channels; j++)
      {
         me_ptr->held_input_buf.frame_ptr[i].sdata.buf_ptr[j].data_ptr = data_ptr;
         data_ptr += buf_size_per_channel_per_frame;
      }
      buf_ptr = (capi_buf_t *)((int8_t *)buf_ptr + (sizeof(capi_buf_t) * num_channels));
   }

   me_ptr->held_input_buf.frame_len_per_ch = buf_size_per_channel_per_frame;

   AR_MSG(DBG_HIGH_PRIO,
          "capi timed renderer: allocated held input buffer for %d frames, frame size %d us ",
          NUM_FRAMES_IN_LOCAL_BUFFER,
          me_ptr->frame_size_us);

   return result;
}

void capi_trm_dealloc_held_input_buffer(capi_trm_t *me_ptr)
{
   posal_memory_free(me_ptr->held_input_buf.frame_ptr);
   me_ptr->held_input_buf.frame_ptr = NULL;
}

/*
 * TRM needs data trigger in STM container in order to consume input FTRT.
 */
capi_err_t capi_trm_raise_event_data_trigger_in_st_cntr(capi_trm_t *me_ptr)
{
   capi_err_t                                  result = CAPI_EOK;
   capi_buf_t                                  payload;
   fwk_extn_event_id_data_trigger_in_st_cntr_t event;

   event.is_enable             = TRUE;
   event.needs_input_triggers  = TRUE;
   event.needs_output_triggers = FALSE;

   payload.data_ptr        = (int8_t *)&event;
   payload.actual_data_len = payload.max_data_len = sizeof(event);

   result =
      capi_cmn_raise_data_to_dsp_svc_event(&me_ptr->event_cb_info, FWK_EXTN_EVENT_ID_DATA_TRIGGER_IN_ST_CNTR, &payload);

   if (CAPI_FAILED(result))
   {
      AR_MSG(DBG_ERROR_PRIO, "capi timed renderer: Failed to raise event to enable data_trigger.");
      return result;
   }
   else
   {
      AR_MSG(DBG_HIGH_PRIO, "capi timed renderer: raised event to enable data_trigger.");
   }

   return result;
}

/*
 * Raise trigger policy to the framework. Trigger policy is input present, output blocked. Trigger policy will only
 * apply for data triggers: When triggered by data, TRM should only buffer data. When triggered by a signal, TRM
 * should evaluate the render decision and output data accordingly.
 *
 * Blocking the output allows us to determine whether process was called due to data trigger (no output provided) or
 * signal trigger (data provided).
 */
capi_err_t capi_trm_update_tgp_before_sync(capi_trm_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   uint32_t   ONE_IP_PORT = 1;
   uint32_t   ONE_OP_PORT = 1;
   uint32_t   ONE_GROUP   = 1;

   fwk_extn_port_trigger_affinity_t  inp_affinity[ONE_IP_PORT];
   fwk_extn_port_nontrigger_policy_t inp_non_tgp[ONE_IP_PORT];
   fwk_extn_port_trigger_affinity_t  out_affinity[ONE_OP_PORT];
   fwk_extn_port_nontrigger_policy_t out_non_tgp[ONE_OP_PORT];

   // Avoid sending redundant tgp events.
   if (CAPI_TRM_TGP_BEFORE_SYNC == me_ptr->tgp_state)
   {
      return capi_result;
   }

   AR_MSG_ISLAND(DBG_HIGH_PRIO, "capi timed renderer: updating before tgp!");

   if (NULL == me_ptr->tgp.tg_policy_cb.change_data_trigger_policy_cb_fn)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi timed renderer:  callback is not set. Unable to raise trigger policy!");
      return capi_result;
   }

   me_ptr->tgp_state = CAPI_TRM_TGP_BEFORE_SYNC;

   fwk_extn_port_trigger_group_t triggerable_group = {.in_port_grp_affinity_ptr  = inp_affinity,
                                                      .out_port_grp_affinity_ptr = out_affinity };
   fwk_extn_port_nontrigger_group_t nontriggerable_group = {.in_port_grp_policy_ptr  = inp_non_tgp,
                                                            .out_port_grp_policy_ptr = out_non_tgp };

   // Input is present.
   inp_affinity[0] = FWK_EXTN_PORT_TRIGGER_AFFINITY_PRESENT;
   inp_non_tgp[0]  = FWK_EXTN_PORT_NON_TRIGGER_INVALID;

   // Output is blocked.
   out_affinity[0] = FWK_EXTN_PORT_TRIGGER_AFFINITY_NONE;
   out_non_tgp[0]  = FWK_EXTN_PORT_NON_TRIGGER_BLOCKED;

   capi_result = me_ptr->tgp.tg_policy_cb.change_data_trigger_policy_cb_fn(me_ptr->tgp.tg_policy_cb.context_ptr,
                                                                           &nontriggerable_group,
                                                                           FWK_EXTN_PORT_TRIGGER_POLICY_MANDATORY,
                                                                           ONE_GROUP,
                                                                           &triggerable_group);

#ifdef TRM_DEBUG
   if (CAPI_SUCCEEDED(capi_result))
   {
      AR_MSG(DBG_HIGH_PRIO, "capi timed renderer: trigger policy updated.");
   }
#endif

   return capi_result;
}

capi_err_t capi_trm_update_tgp_after_sync(capi_trm_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   // Avoid sending redundant tgp events.
   if (CAPI_TRM_TGP_AFTER_SYNC == me_ptr->tgp_state)
   {
      return capi_result;
   }

   AR_MSG_ISLAND(DBG_HIGH_PRIO, "capi timed renderer: updating after tgp!");

   me_ptr->tgp_state = CAPI_TRM_TGP_AFTER_SYNC;

   if (NULL == me_ptr->tgp.tg_policy_cb.change_data_trigger_policy_cb_fn)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi timed renderer:  callback is not set. Unable to raise trigger policy!");
      return capi_result;
   }

   capi_result = me_ptr->tgp.tg_policy_cb.change_data_trigger_policy_cb_fn(me_ptr->tgp.tg_policy_cb.context_ptr,
                                                                           NULL,
                                                                           FWK_EXTN_PORT_TRIGGER_POLICY_MANDATORY,
                                                                           0,
                                                                           NULL);

#ifdef TRM_DEBUG
   if (CAPI_SUCCEEDED(capi_result))
   {
      AR_MSG(DBG_HIGH_PRIO, "capi timed renderer: trigger policy updated.");
   }
#endif

   return capi_result;
}

// Raises rt property event
void capi_trm_raise_rt_port_prop_event(capi_trm_t *me_ptr, bool_t is_input)
{
   capi_buf_t                               payload;
   intf_extn_param_id_is_rt_port_property_t event;
   event.is_input   = is_input;
   event.is_rt      = (is_input) ? FALSE : TRUE; // downstream NRT and upstream RT
   event.port_index = 0;

   payload.data_ptr        = (int8_t *)&event;
   payload.actual_data_len = payload.max_data_len = sizeof(event);

   if (CAPI_FAILED(capi_cmn_raise_data_to_dsp_svc_event(&me_ptr->event_cb_info,
                                                        INTF_EXTN_EVENT_ID_IS_RT_PORT_PROPERTY,
                                                        &payload)))
   {
      AR_MSG(DBG_ERROR_PRIO, "Failed to raise is realtime port property event");
      return;
   }
#ifdef TRM_DEBUG
   else
   {
      AR_MSG(DBG_HIGH_PRIO, "Raised is realtime port property event");
   }
#endif
   
   return;
}

void capi_trm_metadata_destroy_handler(capi_stream_data_v2_t *in_stream_ptr, capi_trm_t *me_ptr)
{
   module_cmn_md_list_t *node_ptr = in_stream_ptr->metadata_list_ptr;
   module_cmn_md_list_t *next_ptr = NULL;
   while (node_ptr)
   {
      next_ptr = node_ptr->next_ptr;

      me_ptr->metadata_handler.metadata_destroy(me_ptr->metadata_handler.context_ptr,
                                                node_ptr,
                                                TRUE /* is dropped*/,
                                                &in_stream_ptr->metadata_list_ptr);

      node_ptr = next_ptr;
   }
}

void capi_trm_metadata_b4_process_nlpi(capi_trm_t *me_ptr, capi_stream_data_v2_t *in_stream_ptr, bool_t **is_resync_ptr)
{
	 module_cmn_md_list_t *node_ptr   = in_stream_ptr->metadata_list_ptr;
	   module_cmn_md_list_t *next_ptr   = NULL;
	   md_ttr_t *            ttr_md_ptr = NULL;
   while (node_ptr)
   {
      next_ptr = node_ptr->next_ptr;

      module_cmn_md_t *md_ptr = (module_cmn_md_t *)node_ptr->obj_ptr;

      switch (md_ptr->metadata_id)
      {
         case MD_ID_TTR:
         {
            bool_t out_of_band = md_ptr->metadata_flag.is_out_of_band;
            if (out_of_band)
            {
               ttr_md_ptr = (md_ttr_t *)md_ptr->metadata_ptr;
            }
            else
            {
               ttr_md_ptr = (md_ttr_t *)&(md_ptr->metadata_buf);
            }

            if (ttr_md_ptr->resync)
            {
               AR_MSG_ISLAND(DBG_HIGH_PRIO, "capi timed renderer: TTR Resync detected.");
               if (CAPI_TRM_DECISION_RENDER != me_ptr->render_decision)
               {
                  capi_trm_flush_held_buffer(me_ptr, NULL, TRUE /*force_free_md*/);
               }
               capi_trm_clear_ttr(me_ptr);
               **is_resync_ptr = TRUE;
            }

            // Only accept the first TTR. If rendering decision is drop, we will set first_ttr_received to FALSE to
            // look for the next TTR.
            if (!(me_ptr->first_ttr_received))
            {
               // Cache TTR only from the first packet of the VFR cycle. This will ensure that we have sufficient data
               // in the TRM till the next processing cycle.
               if (TTR_PACKET_TOKEN_P1 == ttr_md_ptr->packet_token)
               {
                  me_ptr->curr_ttr           = ttr_md_ptr->ttr;
                  me_ptr->first_ttr_received = TRUE;
                  me_ptr->render_decision    = CAPI_TRM_DECISION_PENDING;

                  AR_MSG_ISLAND(DBG_HIGH_PRIO,
                                "capi timed renderer: got new ttr: %d - md_offset: %d render decision %d",
                                me_ptr->curr_ttr,
                                md_ptr->offset,
                                me_ptr->render_decision);
               }
#ifdef TRM_DEBUG
               // Wait for next TTR. Data will be dropped till then.
               else
               {
                  AR_MSG_ISLAND(DBG_HIGH_PRIO,
                                "capi timed renderer: ttr with packet token %d received. wait for next ttr",
                                ttr_md_ptr->packet_token);
               }
#endif
            }
#ifdef TRM_DEBUG
            else
            {
               AR_MSG_ISLAND(DBG_HIGH_PRIO,
                             "capi timed renderer: ignoring ttr, already have ttr of %ld. ignored ttr %d - md_offset: "
                             "%d",
                             (uint32_t)me_ptr->curr_ttr,
                             (uint32_t)ttr_md_ptr->ttr,
                             md_ptr->offset);
            }
#endif

            me_ptr->metadata_handler.metadata_destroy(me_ptr->metadata_handler.context_ptr,
                                                      node_ptr,
                                                      TRUE /* is dropped*/,
                                                      &in_stream_ptr->metadata_list_ptr);
         }
         break;
      }

      node_ptr = next_ptr;
   }
}
