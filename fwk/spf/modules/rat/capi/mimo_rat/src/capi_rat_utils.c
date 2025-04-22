/*==============================================================================
 @file capi_rat_utils.c
 This file contains utility functions for Rate Adapted Timer Endpoint module

 ================================================================================
 Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 SPDX-License-Identifier: BSD-3-Clause-Clear
 ==============================================================================*/
// clang-format off
// clang-format on
/*=====================================================================
 Includes
 ======================================================================*/
#include "capi_rat_i.h"
/*=====================================================================
 Functions
 ======================================================================*/
static capi_err_t rat_ctrl_port_op(rat_out_ctrl_port_info_t * ctrl_port_info_ptr,
                                   uint32_t                   heap_id,
                                   uint32_t                   ctrl_port_id,
                                   rat_out_ctrl_port_base_t **ctrl_port_pptr,
                                   rat_ctrl_port_op_cl        ctrl_port_op);

static capi_err_t capi_rat_close_ctrl_port(capi_rat_t *              me_ptr,
                                           rat_out_ctrl_port_info_t *out_ctrl_port_info_ptr,
                                           uint32_t                  ctrl_port_id);

static capi_err_t capi_rat_is_fractional_sample_rate(uint32_t sample_rate, bool_t *is_frac_ptr);

/*============================ Media format Functions======================== */
// Check if both configured and inp mf belong to same family - fractional or integer
bool_t capi_rat_is_sample_rate_accepted(uint32_t operating_mf_sr, uint32_t out_mf_sr)
{
   capi_err_t result               = CAPI_EOK;
   bool_t     is_operating_mf_frac = FALSE, is_out_mf_frac = FALSE;

   result = capi_rat_is_fractional_sample_rate(operating_mf_sr, &is_operating_mf_frac);
   result |= capi_rat_is_fractional_sample_rate(out_mf_sr, &is_out_mf_frac);

   if (CAPI_EOK != result)
   {
      return FALSE;
   }
   else if (is_operating_mf_frac == is_out_mf_frac)
   {
      return TRUE;
   }
   return FALSE;
}

// This is checking for supported sample rate as well as frac vs integer
static capi_err_t capi_rat_is_fractional_sample_rate(uint32_t sample_rate, bool_t *is_frac_ptr)
{
   capi_err_t result = CAPI_EOK;

   if (0 == (sample_rate % 8000))
   {
      *is_frac_ptr = FALSE;
   }
   else if (0 == (sample_rate % 11025))
   {
      *is_frac_ptr = TRUE;
   }
   else
   {
      result = CAPI_EUNSUPPORTED;
   }
   return result;
}

/*============================ Event Functions======================== */

capi_err_t capi_rat_raise_thresh_event(capi_rat_t *         me_ptr,
                                       capi_media_fmt_v2_t *mf_ptr,
                                       uint32_t             inp_port_index,
                                       uint32_t             out_port_index)
{
   capi_err_t capi_result = CAPI_EOK;

   uint32_t bytes_per_sample =
      (mf_ptr->format.bits_per_sample > BIT_WIDTH_16) ? BYTES_PER_SAMPLE_FOUR : BYTES_PER_SAMPLE_TWO;

   // Port threshold event
   // mandatory : First get the samples and then scale up by channels , bytes_per_sample.
   uint32_t threshold = me_ptr->frame_size_in_samples;

   if (FALSE == me_ptr->configured_media_fmt_received)
   {
      threshold = (uint32_t)(((uint64_t)((mf_ptr->format.sampling_rate / NUM_MS_PER_SEC) * me_ptr->frame_dur_us)) /
                             NUM_MS_PER_SEC);
   }
   else
   {
      threshold = (threshold * mf_ptr->format.sampling_rate) / me_ptr->configured_media_fmt.format.sampling_rate;
   }

   threshold = threshold * mf_ptr->format.num_channels * bytes_per_sample;

   if ((RAT_PORT_INDEX_INVALID != inp_port_index) && (me_ptr->in_port_info_ptr))
   {
      uint32_t inp_port_id = me_ptr->in_port_info_ptr[inp_port_index].cmn.self_port_id;

      RAT_MSG(me_ptr->iid,
              DBG_HIGH_PRIO,
              "CAPI_RAT: Raising threshold event with threshold = %d on inp port id 0x%x ",
              threshold,
              inp_port_id);

      capi_result |= capi_cmn_update_port_data_threshold_event(&me_ptr->cb_info, threshold, TRUE, inp_port_index);
   }

   if ((RAT_PORT_INDEX_INVALID != out_port_index) && (me_ptr->out_port_info_ptr))
   {
      uint32_t out_port_id = me_ptr->out_port_info_ptr[out_port_index].cmn.self_port_id;

      RAT_MSG(me_ptr->iid,
              DBG_HIGH_PRIO,
              "CAPI_RAT: Raising threshold event with threshold = %d on out port id 0x%x ",
              threshold,
              out_port_id);
      capi_result |= capi_cmn_update_port_data_threshold_event(&me_ptr->cb_info, threshold, FALSE, out_port_index);
   }

   return capi_result;
}

capi_err_t capi_rat_raise_kpps_bw_events(capi_rat_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if (!me_ptr->configured_media_fmt_received)
   {
      RAT_MSG(me_ptr->iid,
              DBG_ERROR_PRIO,
              "CAPI_RAT: Operating media format ptr is not yet received cannot raise kpps bw event");
      return CAPI_EBADPARAM;
   }

   // KPPS event
   uint32_t kpps =
      (3 * (me_ptr->configured_media_fmt.format.sampling_rate) * (me_ptr->configured_media_fmt.format.num_channels)) /
      640;
   if (me_ptr->raised_events_config.kpps != kpps)
   {
      capi_result                       = capi_cmn_update_kpps_event(&me_ptr->cb_info, kpps);
      me_ptr->raised_events_config.kpps = kpps;
   }

   // Bandwidth events
   uint32_t code_bandwidth = 0;
   uint32_t data_bandwidth = (me_ptr->configured_media_fmt.format.sampling_rate / NUM_MS_PER_SEC) *
                             me_ptr->configured_media_fmt.format.bits_per_sample *
                             me_ptr->configured_media_fmt.format.num_channels;

   if ((me_ptr->raised_events_config.code_bw != code_bandwidth) ||
       (me_ptr->raised_events_config.data_bw != data_bandwidth))
   {
      capi_result = capi_cmn_update_bandwidth_event(&me_ptr->cb_info, code_bandwidth, data_bandwidth);
      if (CAPI_EOK != capi_result)
      {
         RAT_MSG(me_ptr->iid, DBG_ERROR_PRIO, "CAPI_RAT: Failed to send bandwidth update event with %lu", capi_result);
      }
      me_ptr->raised_events_config.code_bw = code_bandwidth;
      me_ptr->raised_events_config.data_bw = data_bandwidth;
   }

   return capi_result;
}

/*============================ Timer Functions======================== */

capi_err_t capi_rat_timer_enable(capi_rat_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   // Start
   if (me_ptr->is_timer_enable)
   {
      if (NULL == me_ptr->signal_ptr)
      {
         RAT_MSG(me_ptr->iid, DBG_ERROR_PRIO, "CAPI_RAT: Cannot start timer, signal ptr is NULL");
         return CAPI_EFAILED;
      }

      if (0 == me_ptr->frame_dur_us)
      {
         RAT_MSG(me_ptr->iid, DBG_ERROR_PRIO, "CAPI_RAT: Cannot start one shot timer, frame duration not received ");
         return CAPI_EFAILED;
      }

      /* Do a check on the ports which have been opened
       Needs to be a valid usecase:
       1. RT Proxy, only will have qt_remote drift, cant have others open
       2. A2dp source usecase will have only qt_hwep
       3. hwep_bt should be open only if qt_hwep is open
       */
      bool_t qt_hwep_port_op = (CTRL_PORT_PEER_CONNECTED == me_ptr->inp_ctrl_port_arr[QT_HWEP_PORT_IDX].state) ? 1 : 0;
      bool_t qt_remote_port_op =
         (CTRL_PORT_PEER_CONNECTED == me_ptr->inp_ctrl_port_arr[QT_REMOTE_PORT_IDX].state) ? 1 : 0;
      bool_t hwep_bt_port_op = (CTRL_PORT_PEER_CONNECTED == me_ptr->inp_ctrl_port_arr[HWEP_BT_PORT_IDX].state) ? 1 : 0;

      if (((qt_remote_port_op) && (hwep_bt_port_op || qt_hwep_port_op)) || ((!qt_hwep_port_op) && hwep_bt_port_op))
      {
         RAT_MSG(me_ptr->iid,
                 DBG_ERROR_PRIO,
                 "CAPI_RAT: Not a valid use-case, opened control ports qt_hwep: %d  qt_remote: %d, hwep_bt %d",
                 qt_hwep_port_op,
                 qt_remote_port_op,
                 hwep_bt_port_op);
         return CAPI_EFAILED;
      }

      // Check if mandatory mf is received, if it is received we can start timer
      if (FALSE == me_ptr->configured_media_fmt_received)
      {
         RAT_MSG(me_ptr->iid,
                 DBG_ERROR_PRIO,
                 "CAPI_RAT: Cannot start, did not get mandatory media fmt param on static port yet");
         return CAPI_EFAILED;
      }

      if (!me_ptr->is_timer_created)
      {

         int32_t rc = posal_timer_create(&me_ptr->timer,
                                         POSAL_TIMER_ONESHOT_ABSOLUTE,
                                         POSAL_TIMER_USER,
                                         (posal_signal_t)me_ptr->signal_ptr,
                                         (POSAL_HEAP_ID)me_ptr->heap_mem.heap_id);
         if (rc)
         {
            RAT_MSG(me_ptr->iid, DBG_ERROR_PRIO, "CAPI_RAT: Timer creation failed");
            return CAPI_EFAILED;
         }

         me_ptr->is_timer_created = TRUE;
      }

      // Reset the underrun info before START
      for (uint32_t out_port_index = 0; out_port_index < me_ptr->num_out_ports; out_port_index++)
      {
         if (RAT_PORT_INDEX_INVALID == me_ptr->out_port_info_ptr[out_port_index].cmn.self_index)
         {
            continue;
         }
         CAPI_CMN_UNDERRUN_INFO_RESET(me_ptr->out_port_info_ptr[out_port_index].underrun_info);
      }

      // Initially start timer for one frame dur length
      me_ptr->rat_out_drift_info.rat_acc_drift.time_stamp_us =
         (int64_t)me_ptr->frame_dur_us + (int64_t)posal_timer_get_time();

      me_ptr->absolute_start_time_us = me_ptr->rat_out_drift_info.rat_acc_drift.time_stamp_us;
      int32_t rc =
         posal_timer_oneshot_start_absolute(me_ptr->timer, me_ptr->rat_out_drift_info.rat_acc_drift.time_stamp_us);
      if (rc)
      {
         RAT_MSG(me_ptr->iid, DBG_ERROR_PRIO, "CAPI_RAT: one shot timer start failed result: %lu", rc);
         return CAPI_EFAILED;
      }

      RAT_MSG(me_ptr->iid,
              DBG_HIGH_PRIO,
              "CAPI_RAT: Started timer of duration %lu us which is absolute start time",
              me_ptr->rat_out_drift_info.rat_acc_drift.time_stamp_us);
   }
   else // stop, during sg stop we need to reset configured mf as well
   {
      if (me_ptr->is_timer_created)
      {
         posal_timer_destroy(&me_ptr->timer);
         me_ptr->is_timer_created = FALSE;

         // Reset static mf for the module
         if (CAPI_MIMO_RAT == me_ptr->type)
         {
            RAT_MSG(me_ptr->iid, DBG_HIGH_PRIO, "CAPI_RAT: Reset operating media format during subgraph stop");

            me_ptr->configured_media_fmt_received = FALSE;
            capi_cmn_init_media_fmt_v2(&(me_ptr->configured_media_fmt));
         }
      }

      /** Reset drift tracking struct under mutex lock */
      capi_rat_reset_out_drift_info(&me_ptr->rat_out_drift_info);
   }
   return capi_result;
}

/*============================ IMCL port open/close functions ======================== */

rat_inp_ctrl_port_info_t *capi_rat_get_inp_ctrl_port_info_ptr(capi_rat_t *me_ptr, uint32_t ctrl_port_id)
{
   switch (ctrl_port_id)
   {
      case QT_REFCLK_TIMING_INPUT:
      {
         return &me_ptr->inp_ctrl_port_arr[QT_HWEP_PORT_IDX];
      }
      case REFCLK_REMOTE_TIMING_INPUT:
      {
         return &me_ptr->inp_ctrl_port_arr[HWEP_BT_PORT_IDX];
      }
      case QT_REMOTE_TIMING_INPUT:
      {
         return &me_ptr->inp_ctrl_port_arr[QT_REMOTE_PORT_IDX];
      }
      default:
      {
         return NULL;
      }
   }
}

static capi_err_t rat_ctrl_port_op(rat_out_ctrl_port_info_t * ctrl_port_info_ptr,
                                   uint32_t                   heap_id,
                                   uint32_t                   ctrl_port_id,
                                   rat_out_ctrl_port_base_t **ctrl_port_pptr,
                                   rat_ctrl_port_op_cl        ctrl_port_op)
{
   capi_err_t                result = CAPI_EOK;
   rat_out_ctrl_port_base_t *port_base_ptr;
   spf_list_node_t *         curr_node_ptr;

   /** Set the return pointer to NULL */
   *ctrl_port_pptr = NULL;

   /** Get the pointer to the start of control port list */
   curr_node_ptr = ctrl_port_info_ptr->port_list_ptr;

   /** Iterate over the list of available control ports */
   while (curr_node_ptr)
   {
      port_base_ptr = (rat_out_ctrl_port_base_t *)curr_node_ptr->obj_ptr;

      /** If the control port with this ID already exists return */
      if (port_base_ptr->port_id == ctrl_port_id)
      {
         if (RAT_CTRL_PORT_OPEN == ctrl_port_op)
         {
            /** Populate output pointer and return */
            *ctrl_port_pptr = port_base_ptr;
         }
         else /** RAT_CTRL_PORT_CLOSE */
         {
            /** Remove this node from the list and free the obj memory */
            spf_list_delete_node_and_free_obj(&curr_node_ptr, &ctrl_port_info_ptr->port_list_ptr, TRUE);

            /** Decrement the active port counter */
            // if (ctrl_port_info_ptr->num_out_active_ports)
            {
               ctrl_port_info_ptr->num_out_active_ports--;
            }
         }
         return CAPI_EOK;
      }
      /** Advance to next node in the list */
      curr_node_ptr = curr_node_ptr->next_ptr;
   }
   /** Execution falls through if the control port with given ID
    *  is not present. Now check if the requested operation is
    *  port OPEN or CLOSE. */

   /** If CLOSE, then return failure as port must be present */
   if (RAT_CTRL_PORT_CLOSE == ctrl_port_op)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI_RAT: Failed to get control port for port_id[0x%lX]", ctrl_port_id);
      return CAPI_EBADPARAM;
   }

   /** Execution falls through if the control port with given ID
    *  is not present AND the requested operation is
    *  port OPEN. Allocate one now. */

   /** Allocate Node */
   if (NULL == (port_base_ptr = (rat_out_ctrl_port_base_t *)posal_memory_malloc(sizeof(rat_out_ctrl_port_base_t),
                                                                                (POSAL_HEAP_ID)heap_id)))
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI_RAT: Failed to allocate control port for port_id[0x%lX]", ctrl_port_id);
      return CAPI_ENOMEMORY;
   }

   // memset to ensure that the port state does not contain stale values from previous allocations.
   memset(port_base_ptr, 0, sizeof(rat_out_ctrl_port_base_t));

   /** Add the allocated node to the list */
   if (AR_EOK !=
       (result = spf_list_insert_tail(&ctrl_port_info_ptr->port_list_ptr, port_base_ptr, (POSAL_HEAP_ID)heap_id, TRUE)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "CAPI_RAT: Failed to insert allocated control port for port_id[0x%lX] into list",
             ctrl_port_id);
      return CAPI_EFAILED;
   }

   /** Increment the active port counter */
   ctrl_port_info_ptr->num_out_active_ports++;

   /** Populate output pointer and return */
   *ctrl_port_pptr = port_base_ptr;

   return result;
}

/* Closes both input and output control ports */
static capi_err_t capi_rat_close_ctrl_port(capi_rat_t *              me_ptr,
                                           rat_out_ctrl_port_info_t *out_ctrl_port_info_ptr,
                                           uint32_t                  ctrl_port_id)
{
   capi_err_t capi_result = CAPI_EOK;

   // Check if it is an input port
   rat_inp_ctrl_port_info_t *rat_ctrl_port_ptr = capi_rat_get_inp_ctrl_port_info_ptr(me_ptr, ctrl_port_id);

   if (NULL != rat_ctrl_port_ptr) // input port
   {
      memset(rat_ctrl_port_ptr, 0, sizeof(rat_inp_ctrl_port_info_t));
   }
   else
   {
      rat_out_ctrl_port_base_t *port_base_ptr = NULL;
      /** Get the control port corresponding to this ID. If the
       *  control port is present, it also freed up */
      if (CAPI_EOK != (capi_result |= rat_ctrl_port_op(out_ctrl_port_info_ptr,
                                                       me_ptr->heap_mem.heap_id, /** Heap ID, don't care */
                                                       ctrl_port_id,
                                                       &port_base_ptr,
                                                       RAT_CTRL_PORT_CLOSE)))
      {
         RAT_MSG(me_ptr->iid,
                 DBG_ERROR_PRIO,
                 "CAPI_RAT: rat_ctrl_port_op(): Failed to close ctrl port[0x%lX]",
                 ctrl_port_id);
         /** Continue the for loop to close any other valid ports */
      }
   }

   return capi_result;
}

rat_out_ctrl_port_base_t *capi_rat_get_out_ctrl_port_info_ptr(rat_out_ctrl_port_info_t *ctrl_port_info_ptr,
                                                              uint32_t                  ctrl_port_id)
{
   spf_list_node_t *         curr_node_ptr;
   rat_out_ctrl_port_base_t *port_base_ptr;
   /** Get the pointer to the start of control port list */
   curr_node_ptr = ctrl_port_info_ptr->port_list_ptr;

   while (curr_node_ptr)
   {
      port_base_ptr = (rat_out_ctrl_port_base_t *)curr_node_ptr->obj_ptr;

      /** If the control port with this ID already exists return */
      if ((NULL != port_base_ptr) && (port_base_ptr->port_id == ctrl_port_id))
      {
         return port_base_ptr;
      }
      /** Advance to next node in the list */
      curr_node_ptr = curr_node_ptr->next_ptr;
   }

   // Will fall through if it hasn't found it
   AR_MSG(DBG_ERROR_PRIO, "CAPI_RAT: Failed to get control port for port_id[0x%lX]", ctrl_port_id);
   return NULL;
}

capi_err_t capi_rat_imcl_port_operation(capi_rat_t *            me_ptr,
                                        const capi_port_info_t *port_info_ptr,
                                        capi_buf_t *            params_ptr)
{
   capi_err_t                  capi_result            = CAPI_EOK;
   uint32_t                    num_alloc_ports        = 0;
   intf_extn_imcl_port_open_t *port_open_ptr          = NULL;
   rat_out_ctrl_port_info_t *  out_ctrl_port_info_ptr = &me_ptr->out_ctrl_port_info;

   if (params_ptr->actual_data_len < sizeof(intf_extn_param_id_imcl_port_operation_t))
   {
      RAT_MSG(me_ptr->iid,
              DBG_ERROR_PRIO,
              "CAPI_RAT: Invalid payload size[%lu] for ctrl port operation",
              params_ptr->actual_data_len);
      return CAPI_ENEEDMORE;
   }

   intf_extn_param_id_imcl_port_operation_t *port_op_ptr =
      (intf_extn_param_id_imcl_port_operation_t *)(params_ptr->data_ptr);

   if (!port_op_ptr->op_payload.data_ptr)
   {
      RAT_MSG(me_ptr->iid,
              DBG_ERROR_PRIO,
              "CAPI_RAT: NULL payload for ctrl port open/close/peer connected/disconnected opcode [0x%lX]",
              port_op_ptr->opcode);
      return CAPI_EBADPARAM;
   }

   switch (port_op_ptr->opcode)
   {
      case INTF_EXTN_IMCL_PORT_OPEN:
      {
         port_open_ptr = (intf_extn_imcl_port_open_t *)port_op_ptr->op_payload.data_ptr;

         /** Get the number of control ports being opened */
         uint32_t num_ports = port_open_ptr->num_ports;

         /** Size Validation*/
         uint32_t valid_size =
            sizeof(intf_extn_imcl_port_open_t) + (num_ports * sizeof(intf_extn_imcl_id_intent_map_t));

         if (port_op_ptr->op_payload.actual_data_len < valid_size)
         {
            RAT_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "CAPI_RAT: Invalid payload size[%lu] for ctrl port OPEN",
                    params_ptr->actual_data_len);
            return CAPI_ENEEDMORE;
         }

         /** Iterate over the list of all the control ports being opened */
         for (uint32_t iter = 0; iter < num_ports; iter++)
         {
            /** Validate the number of intents per control port   */
            if (port_open_ptr->intent_map[iter].num_intents > RAT_MAX_INTENTS_PER_CTRL_PORT)
            {
               RAT_MSG(me_ptr->iid,
                       DBG_ERROR_PRIO,
                       "CAPI_RAT: CTRL Port OPEN, Expected num intent[%lu], Configured num intent[%lu]",
                       RAT_MAX_INTENTS_PER_CTRL_PORT,
                       port_open_ptr->intent_map[iter].num_intents);
               capi_result = CAPI_EFAILED;
               goto __rat_bail_out_open;
            }

            /** Size Validation*/
            valid_size += (port_open_ptr->intent_map[iter].num_intents * sizeof(uint32_t));

            /** Validate the actual payload length */
            if (port_op_ptr->op_payload.actual_data_len < valid_size)
            {
               RAT_MSG(me_ptr->iid,
                       DBG_ERROR_PRIO,
                       "CAPI_RAT: Invalid payload size[%lu] for ctrl port OPEN",
                       params_ptr->actual_data_len);

               capi_result = CAPI_ENEEDMORE;
               goto __rat_bail_out_open;
            }

            uint32_t ctrl_port_id = port_open_ptr->intent_map[iter].port_id;

            // Check if input port id
            rat_inp_ctrl_port_info_t *rat_ctrl_port_ptr = capi_rat_get_inp_ctrl_port_info_ptr(me_ptr, ctrl_port_id);

            if (NULL != rat_ctrl_port_ptr) // input port
            {
               if (CTRL_PORT_OPEN == rat_ctrl_port_ptr->state)
               {
                  RAT_MSG(me_ptr->iid, DBG_ERROR_PRIO, "CAPI_RAT: OPEN: Ctrl port[0x%lX] already opened", ctrl_port_id);
                  capi_result = CAPI_EFAILED;
                  goto __rat_bail_out_open;
               }

               /** Intent ID check */
               for (uint32_t idx = 0; idx < port_open_ptr->intent_map[iter].num_intents; idx++)
               {
                  // No INTENT_ID_MODULE_INSTANCE_INFO here because it is a dynamic port
                  if (INTENT_ID_TIMER_DRIFT_INFO == port_open_ptr->intent_map[iter].intent_arr[idx])
                  {
                     rat_ctrl_port_ptr->intent_id_list[idx] = INTENT_ID_TIMER_DRIFT_INFO;
                  }
                  else
                  {
                     capi_result |= CAPI_EBADPARAM;
                     continue;
                  }
               }

               rat_ctrl_port_ptr->port_id     = ctrl_port_id;
               rat_ctrl_port_ptr->state       = CTRL_PORT_OPEN;
               rat_ctrl_port_ptr->num_intents = port_open_ptr->intent_map[iter].num_intents;

               RAT_MSG(me_ptr->iid,
                       DBG_HIGH_PRIO,
                       "CAPI_RAT: num intents %d for ctrl port idx 0x%lx",
                       rat_ctrl_port_ptr->num_intents,
                       ctrl_port_id);
            }
            else if (port_open_ptr->intent_map[iter].intent_arr[0] == INTENT_ID_MODULE_INSTANCE_INFO)
            {
               /*
                If the intent id is INTENT_ID_MODULE_INSTANCE_INFO no action is required on
                the part of the RAT module as this information is simply given to the mailbox
                */
               continue;
               RAT_MSG(me_ptr->iid,
                       DBG_HIGH_PRIO,
                       "CAPI_RAT: IMCL OPEN: Ctrl port[0x%lX] open with INTENT_ID_MODULE_INSTANCE_INFO",
                       ctrl_port_id);
            }
            else // must be output port
            {
               /** Get the control port corresponding to this ID. For open,
                *  the control port will be allocated */
               rat_out_ctrl_port_base_t *port_base_ptr = NULL;
               if (CAPI_EOK != (capi_result = rat_ctrl_port_op(out_ctrl_port_info_ptr,
                                                               me_ptr->heap_mem.heap_id,
                                                               ctrl_port_id,
                                                               &port_base_ptr,
                                                               RAT_CTRL_PORT_OPEN)))
               {
                  RAT_MSG(me_ptr->iid, DBG_ERROR_PRIO, "CAPI_RAT: OPEN: Failed to get ctrl port[0x%lX]", ctrl_port_id);
                  goto __rat_bail_out_open;
               }
               else
               {
                  RAT_MSG(me_ptr->iid, DBG_HIGH_PRIO, "CAPI_RAT: opened ctrl port[0x%lX]", ctrl_port_id);
               }

               /** Check if the port is already opened */
               if (CTRL_PORT_OPEN == port_base_ptr->state)
               {
                  RAT_MSG(me_ptr->iid, DBG_ERROR_PRIO, "CAPI_RAT: OPEN: Ctrl port[0x%lX] already opened", ctrl_port_id);
                  /** Update the error code */
                  capi_result |= CAPI_EBADPARAM;
                  goto __rat_bail_out_open;
               }

               /** Store info */
               port_base_ptr->port_id = ctrl_port_id;
               port_base_ptr->intent  = INTENT_ID_TIMER_DRIFT_INFO;
               port_base_ptr->state   = CTRL_PORT_OPEN;
            }

            /** Increment the local counter for successfully allocated control ports */
            num_alloc_ports++;

            RAT_MSG(me_ptr->iid, DBG_HIGH_PRIO, "CAPI_RAT: IMCL OPEN: Ctrl port[0x%lX] open", ctrl_port_id);

         } /** End of for loop (num ports) */
         break;

      } /** End of case INTF_EXTN_IMCL_PORT_OPEN */
      case INTF_EXTN_IMCL_PORT_PEER_CONNECTED:
      {
         intf_extn_imcl_port_start_t *port_start_ptr = (intf_extn_imcl_port_start_t *)port_op_ptr->op_payload.data_ptr;
         /*Size Validation*/
         uint32_t num_ports = port_start_ptr->num_ports;

         uint32_t valid_size = sizeof(intf_extn_imcl_port_start_t) + (num_ports * sizeof(uint32_t));
         if (port_op_ptr->op_payload.actual_data_len < valid_size)
         {
            RAT_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "CAPI_RAT: Invalid payload size for ctrl port peer connect %d",

                    params_ptr->actual_data_len);
            return CAPI_ENEEDMORE;
         }

         // for each port id in the list that follows
         for (uint32_t iter = 0; iter < num_ports; iter++)
         {
            uint32_t ctrl_port_id = port_start_ptr->port_id_arr[iter];

            // Check if input port id
            rat_inp_ctrl_port_info_t *rat_ctrl_port_ptr = capi_rat_get_inp_ctrl_port_info_ptr(me_ptr, ctrl_port_id);

            if (NULL != rat_ctrl_port_ptr)
            {
               rat_ctrl_port_ptr->state = CTRL_PORT_PEER_CONNECTED;
            }
            else
            {
               rat_out_ctrl_port_base_t *port_base_ptr =
                  capi_rat_get_out_ctrl_port_info_ptr(&me_ptr->out_ctrl_port_info, ctrl_port_id);

               if (port_base_ptr)
               {
                  port_base_ptr->state = CTRL_PORT_PEER_CONNECTED;
                  if (INTENT_ID_TIMER_DRIFT_INFO == port_base_ptr->intent &&
                      CAPI_EOK != (capi_result |= capi_rat_send_out_drift_info_hdl(&me_ptr->cb_info,
                                                                                   &me_ptr->rat_out_drift_info,
                                                                                   ctrl_port_id)))
                  {
                     RAT_MSG(me_ptr->iid,
                             DBG_ERROR_PRIO,
                             "CAPI_RAT: IMCL PEER CONNECT: Failed to send handle to control port %d after connect",
                             ctrl_port_id);
                  }
               }
               else
               {
                  RAT_MSG(me_ptr->iid,
                          DBG_ERROR_PRIO,
                          "CAPI_RAT: IMCL PEER CONNECT: Output control port %d cannot be found",
                          ctrl_port_id);
               }
            }
         }
         break;
      }
      case INTF_EXTN_IMCL_PORT_PEER_DISCONNECTED:
      {
         intf_extn_imcl_port_stop_t *port_start_ptr = (intf_extn_imcl_port_stop_t *)port_op_ptr->op_payload.data_ptr;
         /*Size Validation*/
         uint32_t num_ports = port_start_ptr->num_ports;

         uint32_t valid_size = sizeof(intf_extn_imcl_port_stop_t) + (num_ports * sizeof(uint32_t));
         if (port_op_ptr->op_payload.actual_data_len < valid_size)
         {
            RAT_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "CAPI_RAT: Invalid payload size for ctrl port peer disconnect %d",
                    params_ptr->actual_data_len);
            return CAPI_ENEEDMORE;
         }

         // for each port id in the list that follows
         for (uint32_t iter = 0; iter < num_ports; iter++)
         {
            uint32_t ctrl_port_id = port_start_ptr->port_id_arr[iter];

            // Check if input port id
            rat_inp_ctrl_port_info_t *rat_ctrl_port_ptr = capi_rat_get_inp_ctrl_port_info_ptr(me_ptr, ctrl_port_id);

            if (NULL != rat_ctrl_port_ptr)
            {
               rat_ctrl_port_ptr->state = CTRL_PORT_PEER_DISCONNECTED;
            }
            else
            {
               rat_out_ctrl_port_base_t *port_base_ptr =
                  capi_rat_get_out_ctrl_port_info_ptr(&me_ptr->out_ctrl_port_info, ctrl_port_id);

               if (port_base_ptr)
               {
                  port_base_ptr->state = CTRL_PORT_PEER_DISCONNECTED;
               }
               else
               {
                  RAT_MSG(me_ptr->iid,
                          DBG_ERROR_PRIO,
                          "CAPI_RAT: IMCL PEER DISCONNECT: Output control port %d cannot be found",
                          ctrl_port_id);
               }
            }
         }
         break;
      }
      case INTF_EXTN_IMCL_PORT_CLOSE:
      {
         intf_extn_imcl_port_close_t *port_close_ptr = (intf_extn_imcl_port_close_t *)port_op_ptr->op_payload.data_ptr;

         /** Size Validation*/
         uint32_t num_ports = port_close_ptr->num_ports;
         if (!num_ports)
         {
            RAT_MSG(me_ptr->iid, DBG_HIGH_PRIO, "CAPI_RAT: Warning! Num ctrl ports to be closed is zero");
            return CAPI_EOK;
         }

         uint32_t valid_size = sizeof(intf_extn_imcl_port_close_t) + (num_ports * sizeof(uint32_t));

         if (port_op_ptr->op_payload.actual_data_len < valid_size)
         {
            RAT_MSG(me_ptr->iid,
                    DBG_ERROR_PRIO,
                    "CAPI_RAT: Invalid payload size for ctrl port CLOSE %d",
                    params_ptr->actual_data_len);
            return CAPI_ENEEDMORE;
         }

         /** Iterate over the list of port ID's  */
         for (uint32_t iter = 0; iter < num_ports; iter++)
         {
            /** Get the control port ID to be closed */
            uint32_t ctrl_port_id = port_close_ptr->port_id_arr[iter];
            capi_result |= capi_rat_close_ctrl_port(me_ptr, out_ctrl_port_info_ptr, ctrl_port_id);

         } /** End of for() */
         break;
      } /** End of case INTF_EXTN_IMCL_PORT_CLOSE */
      default:
      {
         RAT_MSG(me_ptr->iid,
                 DBG_ERROR_PRIO,
                 "CAPI_RAT: Set, unsupported IMC port operation opcode ID[0x%lX]",
                 port_op_ptr->opcode);

         capi_result = CAPI_EUNSUPPORTED;
         break;
      }
   } /** End of switch (port_op)) */

   return capi_result;

__rat_bail_out_open:

   /** Free up any allocated ports as part of the current command */
   if ((INTF_EXTN_IMCL_PORT_OPEN == port_op_ptr->opcode) && port_open_ptr && num_alloc_ports)
   {
      /** Iterate over the list of port ID's  */
      for (uint32_t iter = 0; iter < port_open_ptr->num_ports; iter++)
      {
         /** Get the control port ID to be closed */
         uint32_t ctrl_port_id = port_open_ptr->intent_map[iter].port_id;
         capi_rat_close_ctrl_port(me_ptr, out_ctrl_port_info_ptr, ctrl_port_id);
      }
   }

   /** Return the original failure result */
   return capi_result;
}

/*============================ Outgoing IMCL port handling functions ======================== */

/* Create mutex and setting drift function */
capi_err_t capi_rat_init_out_drift_info(rat_drift_info_t *drift_info_ptr, imcl_tdi_get_acc_drift_fn_t get_drift_fn_ptr)
{
   capi_err_t result = CAPI_EOK;

   if (!drift_info_ptr)
   {
      return CAPI_EBADPARAM;
   }

   /** Clear the drift info structure  */
   memset(drift_info_ptr, 0, sizeof(rat_drift_info_t));

   /* Create mutex for the drift info shared with rate matching modules */
   posal_mutex_create(&drift_info_ptr->drift_info_mutex, POSAL_HEAP_DEFAULT);

   /**Set the function pointer for querying the drift */
   drift_info_ptr->drift_info_hdl.get_drift_fn_ptr = get_drift_fn_ptr;

   return result;
}

/* Destroy mutex */
capi_err_t capi_rat_deinit_out_drift_info(rat_drift_info_t *drift_info_ptr)
{
   capi_err_t result = CAPI_EOK;

   if (!drift_info_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI_RAT: Drift info pointer is NULL");
      return CAPI_EBADPARAM;
   }

   /** Destroy mutex */
   posal_mutex_destroy(&drift_info_ptr->drift_info_mutex);

   return result;
}

capi_err_t capi_rat_deinit_out_control_ports(rat_out_ctrl_port_info_t *control_port_info_ptr)
{
   capi_err_t result                           = CAPI_EOK;
   control_port_info_ptr->num_out_active_ports = 0;
   if (AR_EOK != spf_list_delete_list_and_free_objs(&control_port_info_ptr->port_list_ptr, TRUE))
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI RAT: FAILED to free port_list_ptr");
      result = CAPI_EFAILED;
   }
   return result;
}

/* Copy from the rat modules (drift_info_hdl_ptr) to the drift pointer (acc_drift_out_ptr) */
ar_result_t rat_read_acc_out_drift(imcl_tdi_hdl_t *drift_info_hdl_ptr, imcl_tdi_acc_drift_t *acc_drift_out_ptr)
{
   ar_result_t result = AR_EOK;

   if (!drift_info_hdl_ptr || !acc_drift_out_ptr)
   {
      return AR_EFAILED;
   }

   rat_drift_info_t *shared_drift_ptr = (rat_drift_info_t *)drift_info_hdl_ptr;

   posal_mutex_lock(shared_drift_ptr->drift_info_mutex);

   /** Copy the accumulated drift info */
   memscpy(acc_drift_out_ptr,
           sizeof(imcl_tdi_acc_drift_t),
           &shared_drift_ptr->rat_acc_drift,
           sizeof(imcl_tdi_acc_drift_t));

   posal_mutex_unlock(shared_drift_ptr->drift_info_mutex);

   return result;
}

capi_err_t capi_rat_send_out_drift_info_hdl(capi_event_callback_info_t *event_cb_info_ptr,
                                            rat_drift_info_t *          rat_drift_info_ptr,
                                            uint32_t                    ctrl_port_id)
{

   capi_err_t                      capi_result = CAPI_EOK;
   capi_buf_t                      one_time_buf;
   imcl_tdi_set_cfg_header_t *     imcl_hdr_ptr;
   param_id_imcl_timer_drift_info *drift_info_ptr;

   /** Calculate the required payload size */
   uint32_t req_buf_size = sizeof(imcl_tdi_set_cfg_header_t) + sizeof(param_id_imcl_timer_drift_info);

   /** Get the one time buffer from the framework */
   if (CAPI_EOK !=
       (capi_result = capi_cmn_imcl_get_one_time_buf(event_cb_info_ptr, ctrl_port_id, req_buf_size, &one_time_buf)))
   {
      return capi_result;
   }

   /** Populate the received buffer  */
   imcl_hdr_ptr = (imcl_tdi_set_cfg_header_t *)one_time_buf.data_ptr;

   imcl_hdr_ptr->param_id   = IMCL_PARAM_ID_TIMER_DRIFT_INFO;
   imcl_hdr_ptr->param_size = sizeof(imcl_tdi_set_cfg_header_t);

   drift_info_ptr = (param_id_imcl_timer_drift_info *)(one_time_buf.data_ptr + sizeof(imcl_tdi_set_cfg_header_t));

   /** Update the drift info handle to be shared */
   drift_info_ptr->handle_ptr = (imcl_tdi_hdl_t *)&rat_drift_info_ptr->drift_info_hdl;

   imcl_outgoing_data_flag_t flags;
   flags.should_send = TRUE;
   flags.is_trigger  = TRUE;

   /** Send to peer */
   capi_result = capi_cmn_imcl_send_to_peer(event_cb_info_ptr, &one_time_buf, ctrl_port_id, flags);

   return capi_result;
}

capi_err_t capi_rat_reset_out_drift_info(rat_drift_info_t *drift_info_ptr)
{
   if (!drift_info_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI_RAT: Drift info pointer is NULL");
      return CAPI_EFAILED;
   }

   posal_mutex_lock(drift_info_ptr->drift_info_mutex);

   /** Clear the drift info to be shared with rate matching modules */
   memset(&drift_info_ptr->rat_acc_drift, 0, sizeof(imcl_tdi_acc_drift_t));

   posal_mutex_unlock(drift_info_ptr->drift_info_mutex);

   return AR_EOK;
}

capi_err_t capi_rat_validate_timer_duration_cfg(capi_rat_t *                          me_ptr,
                                                param_id_rat_timer_duration_config_t *rat_timer_duration_cfg_ptr)
{
   capi_err_t capi_result = CAPI_EBADPARAM;

   RAT_MSG(me_ptr->iid,
           DBG_HIGH_PRIO,
           "CAPI_RAT: timer configuration, timer config mode %d  (0 - default, 2 - in_time 3 -in samples) "
           " timer duration resolution control %d ( 0 - cfg sr, 1 - min sr)",
           rat_timer_duration_cfg_ptr->timer_config_mode,
           rat_timer_duration_cfg_ptr->timer_duration_resolution_control);

   if (!((RAT_TIMER_DURATION_RESOLUTION_CONTROL_CONFIGURED_SAMPLE_RATE_MODE ==
          rat_timer_duration_cfg_ptr->timer_duration_resolution_control) ||
         (RAT_TIMER_DURATION_RESOLUTION_CONTROL_MINIMUM_SAMPLE_RATE_MODE ==
          rat_timer_duration_cfg_ptr->timer_duration_resolution_control)))
   {
      RAT_MSG(me_ptr->iid,
              DBG_ERROR_PRIO,
              "CAPI_RAT: Unsupported timer duration resolution control %lu received",
              rat_timer_duration_cfg_ptr->timer_duration_resolution_control);
      return capi_result;
   }

   if (RAT_TIMER_DURATION_DEFAULT_MODE == rat_timer_duration_cfg_ptr->timer_config_mode)
   {
      return CAPI_EOK;
      // return OK for default and reserved mode
   }
   else if (RAT_TIMER_DURATION_CONFIGURED_IN_TIME == rat_timer_duration_cfg_ptr->timer_config_mode)
   {
      if ((rat_timer_duration_cfg_ptr->duration_in_time_us >= RAT_TIMER_DURATION_TIME_IN_US_MIN_VALUE) &&
          (rat_timer_duration_cfg_ptr->duration_in_time_us <= RAT_TIMER_DURATION_TIME_IN_US_MAX_VALUE))
      {
         RAT_MSG(me_ptr->iid,
                 DBG_HIGH_PRIO,
                 "CAPI_RAT: timer configuration, duration in time %d received",
                 rat_timer_duration_cfg_ptr->duration_in_time_us);
         return CAPI_EOK;
      }
      RAT_MSG(me_ptr->iid,
              DBG_ERROR_PRIO,
              "CAPI_RAT: Unsupported timer configuration, duration in time %d received",
              rat_timer_duration_cfg_ptr->duration_in_time_us);
   }
   else if (RAT_TIMER_DURATION_CONFIGURED_IN_SAMPLES == rat_timer_duration_cfg_ptr->timer_config_mode)
   {
      // 8K * 1ms to 384K * 40ms
      if ((rat_timer_duration_cfg_ptr->duration_in_samples >= RAT_TIMER_DURATION_IN_SAMPLES_MIN_VALUE) &&
          (rat_timer_duration_cfg_ptr->duration_in_samples <= RAT_TIMER_DURATION_IN_SAMPLES_MAX_VALUE))
      {
         uint32_t frame_dur_us = 0;
         if (me_ptr->configured_media_fmt_received)
         {
            frame_dur_us = (uint32_t)((int64_t)rat_timer_duration_cfg_ptr->duration_in_samples * NUM_MS_PER_SEC) /
                           RAT_MS_FRAME_SIZE(me_ptr->configured_media_fmt.format.sampling_rate);

            if (frame_dur_us < RAT_TIMER_DURATION_TIME_IN_US_MIN_VALUE)
            {
               RAT_MSG(me_ptr->iid,
                       DBG_ERROR_PRIO,
                       "CAPI_RAT: Unsupported timer configuration, duration in samples %d received,"
                       " sample rate %lu calculated duration in time(us) %lu",
                       rat_timer_duration_cfg_ptr->duration_in_samples,
                       me_ptr->configured_media_fmt.format.sampling_rate,
                       frame_dur_us);
               return capi_result;
            }
         }
         RAT_MSG(me_ptr->iid,
                 DBG_HIGH_PRIO,
                 "CAPI_RAT: timer configuration, duration in samples %d received, configured_media_format_set %lu, "
                 "frame_dur_us %lu",
                 rat_timer_duration_cfg_ptr->duration_in_samples,
                 (uint32_t)(me_ptr->configured_media_fmt_received),
                 frame_dur_us);

         return CAPI_EOK;
      }
      RAT_MSG(me_ptr->iid,
              DBG_ERROR_PRIO,
              "CAPI_RAT: Unsupported timer configuration, duration in samples %d received",
              rat_timer_duration_cfg_ptr->duration_in_samples);
   }
   return capi_result;
}

capi_err_t capi_rat_update_resolution_control_sample_rate(capi_rat_t *me_ptr)
{
   bool_t   capi_result                    = CAPI_EOK;
   uint32_t configured_mf_sr               = 0;
   uint32_t resolution_control_sample_rate = 0;

   if (me_ptr->configured_media_fmt_received)
   {
      bool_t is_operating_mf_frac = FALSE;
      configured_mf_sr            = me_ptr->configured_media_fmt.format.sampling_rate;
      if (RAT_TIMER_DURATION_RESOLUTION_CONTROL_MINIMUM_SAMPLE_RATE_MODE ==
          me_ptr->rat_timer_duration_config.timer_duration_resolution_control)
      {
         capi_result = capi_rat_is_fractional_sample_rate(configured_mf_sr, &is_operating_mf_frac);
         if (CAPI_EOK == capi_result)
         {
            if (is_operating_mf_frac)
            {
               resolution_control_sample_rate = RAT_MIN_SUPPORTED_FRACTIONAL_SAMPLING_RATE;
            }
            else
            {
               resolution_control_sample_rate = RAT_MIN_SUPPORTED_INTEGER_SAMPLING_RATE;
            }
         }
      }
      else if (RAT_TIMER_DURATION_RESOLUTION_CONTROL_CONFIGURED_SAMPLE_RATE_MODE ==
               me_ptr->rat_timer_duration_config.timer_duration_resolution_control)
      {
         resolution_control_sample_rate = me_ptr->configured_media_fmt.format.sampling_rate;
      }
      else
      {
         RAT_MSG(me_ptr->iid,
                 DBG_HIGH_PRIO,
                 "CAPI_RAT: update_resolution_control_sample_rate failed, "
                 "invalid timer_duration_resolution_control mode %lu",
                 me_ptr->rat_timer_duration_config.timer_duration_resolution_control);
         capi_result = CAPI_EBADPARAM;
         return capi_result;
      }
   }
   else
   {
      RAT_MSG(me_ptr->iid,
              DBG_HIGH_PRIO,
              "CAPI_RAT: update_resolution_control_sample_rate, failed to update, RAT MF is not set yet ");
      capi_result = CAPI_EBADPARAM;
      return capi_result;
   }

   RAT_MSG(me_ptr->iid,
           DBG_HIGH_PRIO,
           "CAPI_RAT: updated_resolution_control_sample_rate successfully new sr %lu  previous sr %lu",
           resolution_control_sample_rate,
           me_ptr->resolution_control_sample_rate);
   me_ptr->resolution_control_sample_rate = resolution_control_sample_rate;

   return capi_result;
}

capi_err_t capi_rat_update_frame_duration_with_resolution_control(capi_rat_t *me_ptr)
{
   capi_err_t capi_result           = CAPI_EOK;
   uint32_t   configured_mf_sr      = 0;
   uint32_t   resolution_control_sr = 0;

   capi_result = capi_rat_update_resolution_control_sample_rate(me_ptr);
   if (capi_result)
   {
      RAT_MSG(me_ptr->iid, DBG_HIGH_PRIO, "CAPI_RAT: update_frame_duration_with_resolution_control, failed");
      return capi_result;
   }

   configured_mf_sr      = me_ptr->configured_media_fmt.format.sampling_rate;
   resolution_control_sr = me_ptr->resolution_control_sample_rate;

   switch (me_ptr->rat_timer_duration_config.timer_config_mode)
   {
      case RAT_TIMER_DURATION_DEFAULT_MODE:
      {
         if ((me_ptr->sg_frame_dur_us) && (me_ptr->sg_frame_dur_us != me_ptr->frame_dur_us))
         {
            me_ptr->frame_dur_us = me_ptr->sg_frame_dur_us;
         }
      }
      break;
      case RAT_TIMER_DURATION_CONFIGURED_IN_TIME:
      {
         me_ptr->frame_dur_us = me_ptr->rat_timer_duration_config.duration_in_time_us;
      }
      break;
      case RAT_TIMER_DURATION_CONFIGURED_IN_SAMPLES:
      {
         uint64_t frame_size_in_samples = (uint64_t)me_ptr->rat_timer_duration_config.duration_in_samples;
         if (resolution_control_sr != configured_mf_sr)
         {
            frame_size_in_samples = (frame_size_in_samples * resolution_control_sr) / configured_mf_sr;
            frame_size_in_samples = (frame_size_in_samples * configured_mf_sr) / resolution_control_sr;
         }

         me_ptr->frame_dur_us =
            (uint32_t)(((int64_t)frame_size_in_samples * NUM_MS_PER_SEC) / RAT_MS_FRAME_SIZE(configured_mf_sr));
         me_ptr->frame_size_in_samples = (uint32_t)frame_size_in_samples;
      }
      break;
      default:
      {
         if ((me_ptr->sg_frame_dur_us) && (me_ptr->sg_frame_dur_us != me_ptr->frame_dur_us))
         {
            me_ptr->frame_dur_us = me_ptr->sg_frame_dur_us;
         }
      }
   }

   // if media format is configured and timer configuration mode is not in samples
   if ((RAT_TIMER_DURATION_CONFIGURED_IN_SAMPLES != me_ptr->rat_timer_duration_config.timer_config_mode))
   {
      uint64_t unit_frame_size_in_samples = ((uint64_t)RAT_MS_FRAME_SIZE(configured_mf_sr));
      uint64_t frame_size_in_samples      = (unit_frame_size_in_samples * me_ptr->frame_dur_us) / NUM_MS_PER_SEC;

      if (resolution_control_sr != configured_mf_sr)
      {
         frame_size_in_samples = (frame_size_in_samples * resolution_control_sr) / configured_mf_sr;
         frame_size_in_samples = (frame_size_in_samples * configured_mf_sr) / resolution_control_sr;
      }
      me_ptr->frame_size_in_samples = (uint32_t)frame_size_in_samples;
   }

   if (configured_mf_sr)
   {
      me_ptr->frame_dur_us =
         (uint32_t)(((int64_t)me_ptr->frame_size_in_samples * NUM_MS_PER_SEC) / RAT_MS_FRAME_SIZE(configured_mf_sr));
   }

   return capi_result;
}

capi_err_t capi_rat_basic_update_frame_duration(capi_rat_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   switch (me_ptr->rat_timer_duration_config.timer_config_mode)
   {
      case RAT_TIMER_DURATION_DEFAULT_MODE:
      {
         if ((me_ptr->sg_frame_dur_us) && (me_ptr->sg_frame_dur_us != me_ptr->frame_dur_us))
         {
            me_ptr->frame_dur_us = me_ptr->sg_frame_dur_us;
         }
      }
      break;
      case RAT_TIMER_DURATION_CONFIGURED_IN_TIME:
      {
         me_ptr->frame_dur_us = me_ptr->rat_timer_duration_config.duration_in_time_us;
      }
      break;
      case RAT_TIMER_DURATION_CONFIGURED_IN_SAMPLES:
      {
         me_ptr->frame_size_in_samples = me_ptr->rat_timer_duration_config.duration_in_samples;
      }
      break;
      default:
      {
         if ((me_ptr->sg_frame_dur_us) && (me_ptr->sg_frame_dur_us != me_ptr->frame_dur_us))
         {
            me_ptr->frame_dur_us = me_ptr->sg_frame_dur_us;
         }
      }
   }

   return capi_result;
}

capi_err_t capi_rat_update_frame_duration(capi_rat_t *me_ptr)
{
   capi_err_t capi_result                = CAPI_EOK;
   uint32_t   prev_frame_dur_us          = 0;
   uint32_t   prev_frame_size_in_samples = 0;

   if (NULL == me_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI_RAT: update frame duration received invalid instance pointer, 0x%p", me_ptr);
      CAPI_SET_ERROR(capi_result, CAPI_EBADPARAM);
      return capi_result;
   }

   prev_frame_dur_us          = me_ptr->frame_dur_us;
   prev_frame_size_in_samples = me_ptr->frame_size_in_samples;

   if (me_ptr->configured_media_fmt_received)
   {
      capi_result = capi_rat_update_frame_duration_with_resolution_control(me_ptr);
   }
   else
   {
      capi_result = capi_rat_basic_update_frame_duration(me_ptr);
   }

   if (RAT_TIMER_DURATION_CONFIGURED_IN_SAMPLES == me_ptr->rat_timer_duration_config.timer_config_mode)
   {
      RAT_MSG(me_ptr->iid,
              DBG_HIGH_PRIO,
              "CAPI_RAT: update frame duration is us prev: %lu new : %lu ,"
              "frame_size_in_samples prev : %lu, new : %lu configured : %lu",
              prev_frame_dur_us,
              me_ptr->frame_dur_us,
              prev_frame_size_in_samples,
              me_ptr->frame_size_in_samples,
              me_ptr->rat_timer_duration_config.duration_in_samples);
   }
   else
   {
      RAT_MSG(me_ptr->iid,
              DBG_HIGH_PRIO,
              "CAPI_RAT: update frame duration is us prev: %lu new : %lu configured: %lu ,"
              "frame_size_in_samples prev : %lu new : %lu",
              prev_frame_dur_us,
              me_ptr->frame_dur_us,
              me_ptr->rat_timer_duration_config.duration_in_time_us,
              prev_frame_size_in_samples,
              me_ptr->frame_size_in_samples);
   }
   return capi_result;
}