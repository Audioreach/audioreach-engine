/**
 * \file gen_cntr_sync_fwk_ext.c
 *
 * \brief
 *     Implementation of sync fwk extn in gen container
 *
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear */

/* =======================================================================
Includes
========================================================================== */
#include "gen_cntr_utils.h"
#include "gen_cntr_i.h"

static void gen_cntr_fwk_ext_sync_update_tgp(gen_cntr_t *me_ptr, gen_topo_module_t *module_ptr);
/* =======================================================================
Function definitions
========================================================================== */
/*
Function to handle threshold disable/enable event raised by the modules
*/
ar_result_t gen_cntr_fwk_extn_sync_handle_toggle_threshold_buffering_event(
   gen_cntr_t *                      me_ptr,
   gen_topo_module_t *               module_ptr,
   capi_buf_t *                      payload_ptr,
   capi_event_data_to_dsp_service_t *dsp_event_ptr)
{
   ar_result_t result = AR_EOK;
   if (dsp_event_ptr->payload.actual_data_len < sizeof(fwk_extn_sync_event_id_enable_threshold_buffering_t))
   {
      GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                   DBG_ERROR_PRIO,
                   "Error in callback function. The actual size %lu is less than the required size "
                   "%lu for id %lu.",
                   payload_ptr->actual_data_len,
                   sizeof(fwk_extn_sync_event_id_enable_threshold_buffering_t),
                   (uint32_t)(dsp_event_ptr->param_id));
      return AR_EBADPARAM;
   }

   fwk_extn_sync_event_id_enable_threshold_buffering_t *data_ptr =
      (fwk_extn_sync_event_id_enable_threshold_buffering_t *)(dsp_event_ptr->payload.data_ptr);

   GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                DBG_MED_PRIO,
                "Handling toggle threshold buffering event, enable: %ld",
                data_ptr->enable_threshold_buffering);

   bool_t is_threshold_disabled = data_ptr->enable_threshold_buffering ? 0 : 1;

   if (is_threshold_disabled != module_ptr->flags.is_threshold_disabled)
   {
      module_ptr->flags.is_threshold_disabled = is_threshold_disabled;

      /* when threshold is disabled then buffering happens only if need_more_input is true.*/
      if (module_ptr->flags.is_threshold_disabled)
      {
         for (gu_input_port_list_t *list_ptr = module_ptr->gu.input_port_list_ptr; list_ptr != NULL;
              LIST_ADVANCE(list_ptr))
         {
            gen_topo_input_port_t *in_port_ptr = (gen_topo_input_port_t *)list_ptr->ip_port_ptr;
            if (TOPO_DATA_FLOW_STATE_FLOWING == in_port_ptr->common.data_flow_state)
            {
               in_port_ptr->flags.need_more_input = TRUE;
            }
         }
      }
   }

   /*update trigger policy without checking if threshold_enabled state is changing or not.
    * if new ports are opened or media format is propagated during threshold disabled state then module will also raise
    * threshold-disable event again. Now here we need to make sure that new ports are also considered in the trigger
    * policy.
    */
   gen_cntr_fwk_ext_sync_update_tgp(me_ptr, module_ptr);

   //if there is a dm module at the input of SYNC then need to notify them, so that they can start consuming partial data.
   //need to mark external input port if threshold is disabled.
   gen_topo_fwk_extn_sync_propagate_threshold_state(&me_ptr->topo);

   return result;
}

/* updating trigger policy on behalf of SYNC module.
 * Threshold is disabled:
 * 	At least one input port mandatory and all output port manadatory.
 * 	This is to make sure that all the input data goes and sits inside SYNC module until threshold is met.
 * Threshold is enabled:
 * 	Default trigger policy.
 */
static void gen_cntr_fwk_ext_sync_update_tgp(gen_cntr_t *me_ptr, gen_topo_module_t *module_ptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   fwk_extn_param_id_trigger_policy_cb_fn_t handler = { 0 };
   gen_topo_populate_trigger_policy_extn_vtable(module_ptr, &handler);

   if (handler.version != 1)
   {
      return;
   }

   //for threshold enabled case, switch to default trigger policy
   if (!module_ptr->flags.is_threshold_disabled)
   {
      // disable trigger policy
      handler.change_data_trigger_policy_cb_fn(handler.context_ptr,
                                               NULL,
                                               FWK_EXTN_PORT_TRIGGER_POLICY_MANDATORY,
                                               0,
                                               NULL);
      return;
   }

   /* number of trigger groups = number of input ports
    * "OR" across trigger groups. Meaning trigger is satisfied if any group is satisfied.
    * "AND" withing each trigger group. Meaning all ports should satisfy within a group to satisfy group trigger.
    * In Each trigger group there is one unique input port and all output ports.
    * i0 -> o0
      i1 -> o1
      i2 -> o2

      trigger groups are same as number of input ports

      (i0o0o1o2) | (i1o0o1o2)| (i2o0o1o2)
    */
   uint32_t                          num_trigger_groups = module_ptr->gu.num_input_ports;
   uint32_t                          ports_per_group = module_ptr->gu.max_input_ports + module_ptr->gu.max_output_ports;
   fwk_extn_port_trigger_group_t *   tg_array        = NULL;
   fwk_extn_port_trigger_affinity_t *port_af_ptr     = NULL;

   uint32_t size = (num_trigger_groups * sizeof(fwk_extn_port_trigger_group_t)) +
                   (num_trigger_groups * sizeof(fwk_extn_port_trigger_affinity_t) * ports_per_group);

   MALLOC_MEMSET(tg_array, fwk_extn_port_trigger_group_t, size, me_ptr->topo.heap_id, result);

   port_af_ptr = (fwk_extn_port_trigger_affinity_t *)(tg_array + num_trigger_groups);

   for (uint32_t tg_index = 0; tg_index < num_trigger_groups; tg_index++)
   {
      tg_array[tg_index].in_port_grp_affinity_ptr = port_af_ptr;
      port_af_ptr += module_ptr->gu.max_input_ports;

      tg_array[tg_index].out_port_grp_affinity_ptr = port_af_ptr;
      port_af_ptr += module_ptr->gu.max_output_ports;

      for (uint32_t ip_index = 0; ip_index < module_ptr->gu.max_input_ports; ip_index++)
      {
         tg_array[tg_index].in_port_grp_affinity_ptr[ip_index] = FWK_EXTN_PORT_TRIGGER_AFFINITY_NONE;
      }

      for (uint32_t out_index = 0; out_index < module_ptr->gu.max_output_ports; out_index++)
      {
         tg_array[tg_index].in_port_grp_affinity_ptr[out_index] = FWK_EXTN_PORT_TRIGGER_AFFINITY_NONE;
      }
   }

   //assign one unique input port to each trigger group.
   gu_input_port_list_t *mandatory_input_node_ptr = module_ptr->gu.input_port_list_ptr;
   for (uint32_t tg_index = 0; tg_index < num_trigger_groups && mandatory_input_node_ptr; tg_index++)
   {
      // assign input port affinity for this group.
      tg_array[tg_index].in_port_grp_affinity_ptr[mandatory_input_node_ptr->ip_port_ptr->cmn.index] =
         FWK_EXTN_PORT_TRIGGER_AFFINITY_PRESENT;

      LIST_ADVANCE(mandatory_input_node_ptr);
   }

   //assign all output ports to all trigger groups
   for (gu_output_port_list_t *list_ptr = module_ptr->gu.output_port_list_ptr; list_ptr != NULL; LIST_ADVANCE(list_ptr))
   {
      gen_topo_output_port_t *out_port_ptr = (gen_topo_output_port_t *)list_ptr->op_port_ptr;

      if (TOPO_PORT_STATE_STARTED == out_port_ptr->common.state && out_port_ptr->common.flags.is_mf_valid)
      {
         // assign output port affinity for each group.
         for (uint32_t tg_index = 0; tg_index < num_trigger_groups; tg_index++)
         {
            tg_array[tg_index].out_port_grp_affinity_ptr[out_port_ptr->gu.cmn.index] =
               FWK_EXTN_PORT_TRIGGER_AFFINITY_PRESENT;
         }
      }
   }

   result = handler.change_data_trigger_policy_cb_fn(handler.context_ptr,
                                                     NULL,
                                                     FWK_EXTN_PORT_TRIGGER_POLICY_MANDATORY,
                                                     num_trigger_groups,
                                                     tg_array);

   result = capi_err_to_ar_result(result);

   CATCH(result, TOPO_MSG_PREFIX, me_ptr->topo.gu.log_id)
   {
   }

   MFREE_NULLIFY(tg_array);
   return;
}

/**
 * return type:
 * TRUE; Either SYNC doesn't care about this input port or it wants more data on this input port.
 * FALSE; SYNC doesn't want more data on this input port.
 *
 * return TRUE: if
 * 1. input port is not connected to any SYNC module.
 * 2. input port is connected to SYNC module through SISO chain and SYNC module requires data on connected input port
 * return FALSE: iff
 * 1. input port is connected to SYNC module through SISO chain and SYNC module already has sufficient data at connected
 * input.
 */
bool_t gen_cntr_fwk_ext_sync_requires_data(gen_cntr_t *me_ptr, gen_topo_input_port_t *in_port_ptr)
{
   if (NULL == in_port_ptr)
   {
      // input port is not linked to SYNC module
      return TRUE;
   }

   // input port is connected to SYNC module
   gen_topo_module_t *module_ptr = (gen_topo_module_t *)in_port_ptr->gu.cmn.module_ptr;
   if (module_ptr->flags.need_sync_extn)
   {
      return (gen_topo_input_port_is_trigger_present(&me_ptr->topo, in_port_ptr, NULL)) ? FALSE : TRUE;
   }
   else
   {
      if ((1 == module_ptr->gu.num_input_ports) && (1 == module_ptr->gu.num_output_ports))
      {
         gen_topo_input_port_t *next_in_port_ptr =
            (gen_topo_input_port_t *)module_ptr->gu.output_port_list_ptr->op_port_ptr->conn_in_port_ptr;
         return gen_cntr_fwk_ext_sync_requires_data(me_ptr, next_in_port_ptr->nblc_end_ptr);
      }
      else
      {
         // input port is not connected to SYNC module through SISO chain.
         return TRUE;
      }
   }
}

// if SYNC module's input port already has sufficient data then mark the connected external input port as trigger
// satisfied, this is to make sure that this port doesn't block the trigger from other ports connected to SYNC module.
// This is helpful when external input port can not peek into the SYNC module's input port due to other non-nblc module in between (like MFC)
void gen_cntr_fwk_ext_sync_update_ext_in_tgp(gen_cntr_t *me_ptr)
{
   if (me_ptr->topo.num_data_tpm || !me_ptr->topo.flags.is_sync_module_present)
   {
      return;
   }

   for (gu_ext_in_port_list_t *ext_in_port_list_ptr = me_ptr->topo.gu.ext_in_port_list_ptr;
        (NULL != ext_in_port_list_ptr);
        LIST_ADVANCE(ext_in_port_list_ptr))
   {
      gen_cntr_ext_in_port_t *ext_in_port_ptr = (gen_cntr_ext_in_port_t *)ext_in_port_list_ptr->ext_in_port_ptr;
      gen_topo_input_port_t * in_port_ptr =
         (gen_topo_input_port_t *)ext_in_port_list_ptr->ext_in_port_ptr->int_in_port_ptr;

      if (TOPO_PORT_STATE_STARTED == in_port_ptr->common.state &&
          TOPO_DATA_FLOW_STATE_FLOWING == in_port_ptr->common.data_flow_state)
      {
         // If sync module doesn't require data from this input port then mark external port as trigger-satisfied
         if (!ext_in_port_ptr->flags.ready_to_go && !gen_cntr_fwk_ext_sync_requires_data(me_ptr, in_port_ptr))
         {
            ext_in_port_ptr->flags.ready_to_go = TRUE;
#ifdef VERBOSE_DEBUGGING
            GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_LOW_PRIO,
                         "SYNC module marking input port id 0x%x module iid 0x%x as trigger satisfied",
                         in_port_ptr->gu.cmn.id,
                         in_port_ptr->gu.cmn.module_ptr->module_instance_id);
#endif
         }
      }
   }
}
