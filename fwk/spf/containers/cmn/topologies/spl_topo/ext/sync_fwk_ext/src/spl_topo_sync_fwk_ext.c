/**
 * \file spl_topo_tdm.c
 *
 * \brief
 *
 *     Topo 2 functions for managing sync fwk extn that handles active/inactive module ports
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "spl_topo_sync_fwk_ext.h"
#include "spl_topo_i.h"

//Check if port can be moved to inactive list
static bool_t is_int_out_port_can_be_inactive(spl_topo_t *topo_ptr, spl_topo_output_port_t *out_port_ptr)
{
   spl_topo_input_port_t *conn_in_port_ptr = (spl_topo_input_port_t *)out_port_ptr->t_base.gu.conn_in_port_ptr;
   spl_topo_module_t *    conn_module_ptr  = (spl_topo_module_t *)conn_in_port_ptr->t_base.gu.cmn.module_ptr;

   // can't remove port if data is flowing.
   // Sync module may have inactivated the port but we need to first propagated the EOS before.
   if (TOPO_DATA_FLOW_STATE_AT_GAP != conn_in_port_ptr->t_base.common.data_flow_state)
   {
      return FALSE;
   }

   // can't remove the port if there is any metadata stuck
   if ((conn_in_port_ptr->t_base.common.sdata.metadata_list_ptr) || (conn_module_ptr->t_base.int_md_list_ptr) ||
       conn_in_port_ptr->t_base.common.sdata.flags.marker_eos || (out_port_ptr->md_list_ptr) ||
       out_port_ptr->t_base.common.sdata.flags.end_of_frame)
   {
      return FALSE;
   }

   // can't remove port if it has any data stuck
   if (spl_topo_op_port_contains_unconsumed_data(topo_ptr, out_port_ptr))
   {
      return FALSE;
   }

   return TRUE;
}

//move the ports to the active state
static ar_result_t spl_topo_sync_handle_pending_active_ports(spl_topo_t *topo_ptr, spl_topo_module_t *module_ptr)
{
   ar_result_t result             = AR_EOK;
   bool_t      is_anthing_changed = FALSE;

   for (gu_output_port_list_t *out_list_ptr = module_ptr->t_base.gu.output_port_list_ptr; (NULL != out_list_ptr);
        LIST_ADVANCE(out_list_ptr))
   {
      spl_topo_output_port_t *out_port_ptr = (spl_topo_output_port_t *)out_list_ptr->op_port_ptr;
      if (!out_port_ptr->flags.pending_active)
      {
         continue;
      }

      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_HIGH_PRIO,
               "TOPO_SYNC_EXTN: Activating output port idx = %lu from module_ptr =0x%lx",
               out_port_ptr->t_base.gu.cmn.index,
               out_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id);

      out_port_ptr->flags.pending_active = FALSE;
      out_port_ptr->flags.port_inactive  = FALSE; // active
      is_anthing_changed                 = TRUE;

      // the connected input port should also move to its active list
      spl_topo_input_port_t *conn_in_port_ptr = (spl_topo_input_port_t *)out_port_ptr->t_base.gu.conn_in_port_ptr;

      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_HIGH_PRIO,
               "TOPO_SYNC_EXTN: Also activating connected input port idx = %lu from module_ptr =0x%lx",
               conn_in_port_ptr->t_base.gu.cmn.index,
               conn_in_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id);

      conn_in_port_ptr->flags.port_inactive = FALSE; // active
   }

   if (is_anthing_changed)
   {
      // inactive ports are activated, need to update the simp_topo_L2 flags.
      spl_topo_simp_topo_set_all_event_flag(topo_ptr);
   }

   return result;
}

//move the ports to inactive state
static ar_result_t spl_topo_sync_handle_pending_inactive_ports(spl_topo_t *       topo_ptr,
                                                               spl_topo_module_t *module_ptr,
                                                               bool_t *           is_event_handled_ptr)
{
   ar_result_t result             = AR_EOK;
   bool_t      is_anthing_changed = FALSE;

   *is_event_handled_ptr = TRUE;

   for (gu_output_port_list_t *out_list_ptr = module_ptr->t_base.gu.output_port_list_ptr; (NULL != out_list_ptr);
        LIST_ADVANCE(out_list_ptr))
   {
      spl_topo_output_port_t *out_port_ptr = (spl_topo_output_port_t *)out_list_ptr->op_port_ptr;
      if (!out_port_ptr->flags.pending_inactive)
      {
         continue;
      }

      if (!is_int_out_port_can_be_inactive(topo_ptr, out_port_ptr))
      {
         // event is not handled
         *is_event_handled_ptr = FALSE;
         continue;
      }

      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_HIGH_PRIO,
               "TOPO_SYNC_EXTN: Inactivating output port idx = %lu from module_ptr =0x%lx",
               out_port_ptr->t_base.gu.cmn.index,
               out_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id);

      out_port_ptr->flags.pending_inactive = FALSE;
      out_port_ptr->flags.port_inactive    = TRUE; // inactive
      is_anthing_changed                   = TRUE;

      // the connected input port should also move to its active list
      spl_topo_input_port_t *conn_in_port_ptr = (spl_topo_input_port_t *)out_port_ptr->t_base.gu.conn_in_port_ptr;

      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_HIGH_PRIO,
               "TOPO_SYNC_EXTN: Also inactivating connected input port idx = %lu from module_ptr =0x%lx",
               conn_in_port_ptr->t_base.gu.cmn.index,
               conn_in_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id);

      conn_in_port_ptr->flags.port_inactive = TRUE; // inactive
   }

   if (is_anthing_changed)
   {
      // active ports are inactivated, need to update the simp_topo_L2 flags.
      spl_topo_simp_topo_set_all_event_flag(topo_ptr);
   }
   return result;
}

ar_result_t spl_topo_sync_handle_inactive_port_events(spl_topo_t *topo_ptr, bool_t *is_event_handled_ptr)
{
   ar_result_t result = AR_EOK;

   *is_event_handled_ptr = TRUE;

   for (gu_sg_list_t *sg_list_ptr = topo_ptr->t_base.gu.sg_list_ptr; sg_list_ptr; LIST_ADVANCE(sg_list_ptr))
   {
      for (gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; module_list_ptr;
           LIST_ADVANCE(module_list_ptr))
      {
         spl_topo_module_t *module_ptr = (spl_topo_module_t *)module_list_ptr->module_ptr;
         if (!module_ptr->t_base.flags.need_sync_extn)
         {
            continue; // handling just for sync modules
         }
         // we should go over the active port list, check for pending inactive ports, move them
         // then go over inactive port list, check for pending active ports, move them.
         spl_topo_sync_handle_pending_inactive_ports(topo_ptr, module_ptr, is_event_handled_ptr);
         spl_topo_sync_handle_pending_active_ports(topo_ptr, module_ptr);
      }
   }

   return result;
}

ar_result_t spl_topo_handle_data_port_activity_sync_event_cb(spl_topo_t *       topo_ptr,
                                                             spl_topo_module_t *module_ptr,
                                                             capi_buf_t *       payload_ptr)
{
   ar_result_t result = AR_EOK;

   fwk_extn_sync_event_id_data_port_activity_state_t *cfg_ptr =
      (fwk_extn_sync_event_id_data_port_activity_state_t *)payload_ptr->data_ptr;

   TOPO_MSG(topo_ptr->t_base.gu.log_id,
            DBG_HIGH_PRIO,
            "TOPO_SYNC_EXTN: Received CB: out port_idx =%lu CB from miid 0x%lX, is_inactive = %lu",
            cfg_ptr->out_port_index,
            module_ptr->t_base.gu.module_instance_id,
            cfg_ptr->is_inactive);

   // get the output port pointer from the index.
   spl_topo_output_port_t *out_port_ptr =
      (spl_topo_output_port_t *)gu_find_output_port_by_index(&(module_ptr->t_base.gu), cfg_ptr->out_port_index);

   if (NULL == out_port_ptr)
   {
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_HIGH_PRIO,
               "Warning: TOPO_SYNC_EXTN: event raised on invalid output port index");
      return AR_EFAILED;
   }

   // clear the event flag, will update it accordingly.
   out_port_ptr->flags.pending_inactive = FALSE;
   out_port_ptr->flags.pending_active   = FALSE;

   // if there is no connected input port then ignore.
   if (NULL == out_port_ptr->t_base.gu.conn_in_port_ptr)
   {
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_HIGH_PRIO,
               "Warning: TOPO_SYNC_EXTN: Sync Module 0x%lx's output port idx doesn't have a connected input. "
               "Ignoring the event and returning",
               module_ptr->t_base.gu.module_instance_id);
      return AR_EOK;
   }

   if (cfg_ptr->is_inactive)
   {
      // set the event flag, if port is not already inactive.
      out_port_ptr->flags.pending_inactive = (out_port_ptr->flags.port_inactive) ? FALSE : TRUE;
   }
   else
   {
      // set the event flag, if port is not already active.
      out_port_ptr->flags.pending_active = (out_port_ptr->flags.port_inactive) ? TRUE : FALSE;
   }

   topo_ptr->fwk_extn_info.sync_extn_pending_inactive_handling |=
      (out_port_ptr->flags.pending_inactive || out_port_ptr->flags.pending_active);

   return result;
}
