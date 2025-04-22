/**
 * \file cu_offload_util.c
 * \brief
 *     This file contains container utility functions for external port handling (input and output).
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "cu_i.h"
#include "rd_sh_mem_ep_api.h"
#include "wr_sh_mem_ep_api.h"
#include "offload_sp_api.h"
#include "offload_path_delay_api.h"

/* =======================================================================
Public Function Definitions
========================================================================== */
/* Function to raise a module event to the clients*/
static ar_result_t cu_raise_module_events_to_clients(cu_base_t *      me_ptr,
                                                     spf_list_node_t *event_list_ptr,
                                                     uint32_t         event_id,
                                                     uint32_t         src_miid,
                                                     int8_t *         payload_ptr,
                                                     uint32_t         payload_size)
{
   ar_result_t result = AR_EOK;

   spf_list_node_t *client_list_ptr = NULL;
   if (AR_EOK != (result = cu_find_client_info(me_ptr->gu_ptr->log_id, event_id, event_list_ptr, &client_list_ptr)))
   {
      CU_MSG(me_ptr->gu_ptr->log_id,
             DBG_ERROR_PRIO,
             "Failed to get client list for event id: 0x%lx, result: %d",
             event_id,
             result);
      return result;
   }

   if (NULL == client_list_ptr)
   {
      return result;
   }

   gpr_packet_t *event_packet_ptr = NULL;
   void *        event_payload_ptr;

   for (cu_client_info_t *client_info_ptr = (cu_client_info_t *)(client_list_ptr->obj_ptr); (NULL != client_list_ptr);
        LIST_ADVANCE(client_list_ptr))
   {
      event_packet_ptr = NULL;
      gpr_cmd_alloc_ext_t args;
      args.src_domain_id = client_info_ptr->dest_domain_id;
      args.dst_domain_id = client_info_ptr->src_domain_id;
      args.src_port      = src_miid;
      args.dst_port      = client_info_ptr->src_port;
      args.token         = client_info_ptr->token;
      args.opcode        = event_id;
      args.payload_size  = payload_size;
      args.client_data   = 0;
      args.ret_packet    = &event_packet_ptr;
      result             = __gpr_cmd_alloc_ext(&args);
      if (NULL == event_packet_ptr)
      {
         CU_MSG(me_ptr->gu_ptr->log_id,
                DBG_ERROR_PRIO,
                "CNTR 0x%lX: event 0x%lX NOT sent to client with error code %lu",
                me_ptr->gu_ptr->container_instance_id,
                event_id,
                result);
         return AR_EFAILED;
      }

      event_payload_ptr = GPR_PKT_GET_PAYLOAD(void, event_packet_ptr);

      memscpy(event_payload_ptr, payload_size, payload_ptr, payload_size);

      result = __gpr_cmd_async_send(event_packet_ptr);

      if (AR_EOK != result)
      {
         CU_MSG(me_ptr->gu_ptr->log_id,
                DBG_ERROR_PRIO,
                "MIID 0x%lX: Unable to send event 0x%lX to client with result %lu "
                "destn port: 0x%lX, destn domain ID: 0x%lX, src domain ID: 0x%lX ",
                src_miid,
                event_id,
                result,
                client_info_ptr->src_port,
                client_info_ptr->src_domain_id,
                client_info_ptr->dest_domain_id);
          __gpr_cmd_free(event_packet_ptr);
      }
      else
      {
         CU_MSG(me_ptr->gu_ptr->log_id,
                DBG_HIGH_PRIO,
                "MIID 0x%lX: event 0x%lX sent to client with result %lu "
                "destn port: 0x%lX, destn domain ID: 0x%lX, src domain ID: 0x%lX ",
                src_miid,
                event_id,
                result,
                client_info_ptr->src_port,
                client_info_ptr->src_domain_id,
                client_info_ptr->dest_domain_id);
      }
   } // for loop over all clients

   return result;
}

// utility function to send ext output port property update to downstream peer port.
ar_result_t cu_send_upstream_state_to_parent_container(cu_base_t *        me_ptr,
                                                       gu_ext_out_port_t *gu_ext_out_port_ptr,
                                                       topo_port_state_t  ds_state)
{
   ar_result_t                 result             = AR_EOK;
   uint32_t                    event_payload_size = 0;
   upstream_state_cfg_event_t *event_payload_ptr  = NULL;

   if (!gu_ext_out_port_ptr || !me_ptr)
   {
      return AR_EFAILED;
   }

   // why send started state to parent container:
   //    This will help the OLC send the read buffers from the OLC to Satellite graph

   gu_module_t *module_ptr    = (gu_module_t *)gu_ext_out_port_ptr->int_out_port_ptr->cmn.module_ptr;
   cu_module_t *cu_module_ptr = (cu_module_t *)((uint8_t *)module_ptr + me_ptr->module_cu_offset);

   CU_MSG(me_ptr->gu_ptr->log_id,
          DBG_HIGH_PRIO,
          "process peer port property proapagation event id (0x%lx) from miid (0x%lx) to OLC ",
          OFFLOAD_EVENT_ID_UPSTREAM_STATE,
          module_ptr->module_instance_id);

   // determine the payload size and allocate the memory
   event_payload_size = sizeof(upstream_state_cfg_event_t);
   event_payload_ptr  = (upstream_state_cfg_event_t *)posal_memory_malloc(event_payload_size, me_ptr->heap_id);
   if (NULL == event_payload_ptr)
   {
      CU_MSG(me_ptr->gu_ptr->log_id,
             DBG_ERROR_PRIO,
             "Failed to allocate memory of size %lu to propagate upstream state from satellite to OLC",
             event_payload_size);
      return AR_ENOMEMORY;
   }

   event_payload_ptr->ep_miid                                   = module_ptr->module_instance_id;
   event_payload_ptr->num_properties                            = 1;
   event_payload_ptr->peer_port_property_payload.property_type  = PORT_PROPERTY_TOPO_STATE;
   event_payload_ptr->peer_port_property_payload.property_value = ds_state;

   result = cu_raise_module_events_to_clients(me_ptr,
                                              cu_module_ptr->event_list_ptr,
                                              OFFLOAD_EVENT_ID_UPSTREAM_STATE,
                                              module_ptr->module_instance_id,
                                              (int8_t *)event_payload_ptr,
                                              event_payload_size);

   if (AR_DID_FAIL(result))
   {
      CU_MSG(me_ptr->gu_ptr->log_id,
             DBG_ERROR_PRIO,
             " Failed to push the upstream (0x%lX, 0x%lx) state message to downstream (OLC), result %d. ",
             module_ptr->module_instance_id,
             gu_ext_out_port_ptr->int_out_port_ptr->cmn.id,
             result);
      return result;
   }
   else
   {
      CU_MSG(me_ptr->gu_ptr->log_id,
             DBG_MED_PRIO,
             " sent upstream (0x%lX, 0x%lx) state to downstream (OLC)",
             module_ptr->module_instance_id,
             gu_ext_out_port_ptr->int_out_port_ptr->cmn.id);
   }

   posal_memory_free(event_payload_ptr);

   return result;
}

// utility function to send ext output port property update to downstream peer port over GPR.
// This function is used in offload context, where the WR/RD_SHMEM_EP modules have to communicate
// the event from the satellite graph to the OLC in the master graph
ar_result_t cu_propagate_to_parent_container_ext_port(cu_base_t *      me_ptr,
                                                      spf_list_node_t *event_list_ptr,
                                                      uint32_t         event_id,
                                                      uint32_t         src_miid,
                                                      int8_t *         prop_ptr)
{
   ar_result_t result = AR_EOK;

   if (!me_ptr || !prop_ptr)
   {
      return AR_EFAILED;
   }

   CU_MSG(me_ptr->gu_ptr->log_id,
          DBG_HIGH_PRIO,
          "process peer port property proapagation event id (0x%lx) from miid (0x%lx) to OLC ",
          event_id,
          src_miid);

   spf_msg_peer_port_property_update_t *prop_payload_ptr = (spf_msg_peer_port_property_update_t *)prop_ptr;

   uint32_t prop_payload_size = 0;
   prop_payload_size += (prop_payload_ptr->num_properties) * sizeof(spf_msg_peer_port_property_info_t);

   uint32_t peer_port_property_pkt_size = sizeof(state_cfg_event_t) + prop_payload_size; // opcode + prop_payload
   uint8_t *peer_port_property_payload_ptr =
      (uint8_t *)posal_memory_malloc(peer_port_property_pkt_size, me_ptr->heap_id);
   if (NULL == peer_port_property_payload_ptr)
   {
      CU_MSG(me_ptr->gu_ptr->log_id,
             DBG_ERROR_PRIO,
             "Failed to allocate memory of size %lu to process the peer_port_property propagation",
             peer_port_property_pkt_size);
      return AR_ENOMEMORY;
   }
   state_cfg_event_t *sce_ptr = (state_cfg_event_t *)(peer_port_property_payload_ptr);
   sce_ptr->ep_miid           = src_miid;
   sce_ptr->num_properties    = prop_payload_ptr->num_properties;

   memscpy((sce_ptr + 1), prop_payload_size, &prop_payload_ptr->payload, prop_payload_size);

   result = cu_raise_module_events_to_clients(me_ptr,
                                              event_list_ptr,
                                              event_id,
                                              src_miid,
                                              (int8_t *)peer_port_property_payload_ptr,
                                              peer_port_property_pkt_size);

   if (AR_DID_FAIL(result))
   {
      CU_MSG(me_ptr->gu_ptr->log_id,
             DBG_ERROR_PRIO,
             "Failed to send the port property update to parent container, result %d",
             result);
   }
   else
   {
      CU_MSG(me_ptr->gu_ptr->log_id,
             DBG_HIGH_PRIO,
             "sucessfully sent the port property update to parent container, result %d",
             result);
   }

   if (NULL != peer_port_property_payload_ptr)
   {
      posal_memory_free(peer_port_property_payload_ptr);
   }

   return result;
}

static ar_result_t cu_propagate_us_prop_to_parent_cntr(cu_base_t *                          base_ptr,
                                                       gu_ext_out_port_t *                  gu_ext_out_port_ptr,
                                                       spf_msg_peer_port_property_update_t *prop_ptr)
{
   gu_module_t *module_ptr    = (gu_module_t *)gu_ext_out_port_ptr->int_out_port_ptr->cmn.module_ptr;
   cu_module_t *cu_module_ptr = (cu_module_t *)((uint8_t *)module_ptr + base_ptr->module_cu_offset);

   /* state propagation from RD_SHM_EP module in satellite graph to the corresponding data port in the OLC*/
   return cu_propagate_to_parent_container_ext_port(base_ptr,
                                                    cu_module_ptr->event_list_ptr,
                                                    OFFLOAD_EVENT_ID_UPSTREAM_PEER_PORT_PROPERTY,
                                                    gu_ext_out_port_ptr->int_out_port_ptr->cmn.module_ptr
                                                       ->module_instance_id,
                                                    (int8_t *)prop_ptr);
}

static ar_result_t cu_propagate_ds_prop_to_parent_cntr(cu_base_t *                          base_ptr,
                                                       gu_ext_in_port_t *                   gu_ext_in_port_ptr,
                                                       spf_msg_peer_port_property_update_t *prop_ptr)
{
   gu_module_t *module_ptr    = (gu_module_t *)gu_ext_in_port_ptr->int_in_port_ptr->cmn.module_ptr;
   cu_module_t *cu_module_ptr = (cu_module_t *)((uint8_t *)module_ptr + base_ptr->module_cu_offset);

   /* state propagation from WR_SHM_EP module in satellite graph to the corresponding data port in the OLC*/
   return cu_propagate_to_parent_container_ext_port(base_ptr,
                                                    cu_module_ptr->event_list_ptr,
                                                    OFFLOAD_EVENT_ID_DOWNSTREAM_PEER_PORT_PROPERTY,
                                                    gu_ext_in_port_ptr->int_in_port_ptr->cmn.module_ptr
                                                       ->module_instance_id,
                                                    (int8_t *)prop_ptr);
}

ar_result_t cu_offload_process_peer_port_property_propagation(cu_base_t *base_ptr, bool_t need_to_update_states)
{
   ar_result_t result = AR_EOK;

   // after RT/FTRT re-assign the states because states are not propagated for FTRT
   if (need_to_update_states)
   {
      result |= cu_update_all_sg_port_states(base_ptr, FALSE);

      // state prop and RT/FTRT is complete now. inform upstream or downstream such that propagation across container
      // happens
      result |= cu_inform_downstream_about_upstream_property(base_ptr);
      result |= cu_inform_upstream_about_downstream_property(base_ptr);
   }

   return result;
}

ar_result_t cu_create_offload_info(cu_base_t *me_ptr, apm_prop_data_t *cntr_prop_ptr)
{
   ar_result_t result = AR_EOK;
   uint32_t    log_id = 0;
   INIT_EXCEPTION_HANDLING

   cu_offload_info_t *offload_info_ptr = NULL;
   VERIFY(result, cntr_prop_ptr->prop_size >= sizeof(apm_cont_prop_id_parent_container_t));
   apm_cont_prop_id_parent_container_t *client_cfg_ptr = NULL;

   if (me_ptr->gu_ptr)
   {
      log_id = me_ptr->gu_ptr->log_id;
   }

   client_cfg_ptr = (apm_cont_prop_id_parent_container_t *)(cntr_prop_ptr + 1);

   if ((0 == client_cfg_ptr->parent_container_id) || (APM_PROP_ID_DONT_CARE == client_cfg_ptr->parent_container_id))
   {
      CU_MSG(log_id,
             DBG_HIGH_PRIO,
             "Parsing parent container property, invalid configuration [0x%lx], "
             "can be ignored on master process domain",
             client_cfg_ptr->parent_container_id);
      // APM should catch this error and no need to error from here for satellite containers
   }
   else
   {
      if (NULL == me_ptr->offload_info_ptr)
      {
         MALLOC_MEMSET(offload_info_ptr, cu_offload_info_t, sizeof(cu_offload_info_t), me_ptr->heap_id, result);

         if (NULL == offload_info_ptr)
         {
            CU_MSG(log_id,
                   DBG_ERROR_PRIO,
                   "Failed to allocate memory of size %lu to store the offload configurtation info",
                   sizeof(cu_offload_info_t));
            THROW(result, AR_ENOMEMORY);
         }
         me_ptr->offload_info_ptr = offload_info_ptr;
      }

      me_ptr->offload_info_ptr->is_satellite_container = TRUE;
      me_ptr->offload_info_ptr->client_id              = client_cfg_ptr->parent_container_id;

      CU_MSG(log_id,
             DBG_HIGH_PRIO,
             "is satellite container %lu, OLC container ID 0x%lx",
             me_ptr->offload_info_ptr->is_satellite_container,
             me_ptr->offload_info_ptr->client_id);
   }
   CATCH(result, CU_MSG_PREFIX, log_id)
   {
   }

   return result;
}

void cu_destroy_offload_info(cu_base_t *me_ptr)
{
   MFREE_NULLIFY(me_ptr->offload_info_ptr);
}

ar_result_t cu_offload_ds_propagation_init(cu_base_t *base_ptr, cu_ext_out_port_t *cu_ext_out_port_ptr)
{
   // port properties are not propagated for GPR clients. For offload, once event is registered, prop is
   // enabled.
   cu_ext_out_port_ptr->prop_info.prop_enabled               = FALSE;
   cu_ext_out_port_ptr->prop_info.prop_ds_prop_to_us_fn      = NULL;
   cu_ext_out_port_ptr->prop_info.prop_us_prop_to_ds_fn      = cu_propagate_us_prop_to_parent_cntr;
   cu_ext_out_port_ptr->prop_info.prop_us_state_ack_to_ds_fn = cu_send_upstream_state_to_parent_container;

   return AR_EOK;
}

ar_result_t cu_offload_us_propagation_init(cu_base_t *base_ptr, cu_ext_in_port_t *cu_ext_in_port_ptr)
{
   // port properties are not propagated for GPR clients. For offload, once event is registered, prop is
   // enabled.
   cu_ext_in_port_ptr->prop_info.prop_enabled               = FALSE;
   cu_ext_in_port_ptr->prop_info.prop_ds_prop_to_us_fn      = cu_propagate_ds_prop_to_parent_cntr;
   cu_ext_in_port_ptr->prop_info.prop_us_prop_to_ds_fn      = NULL;
   cu_ext_in_port_ptr->prop_info.prop_us_state_ack_to_ds_fn = cu_send_upstream_state_to_parent_container;

   return AR_EOK;
}

ar_result_t cu_raise_event_get_path_delay(cu_base_t *base_ptr,
                                          uint32_t   prev_delay_in_us,
                                          uint32_t   curr_delay_in_us,
                                          uint32_t   path_id)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   get_container_delay_event_t *delay_event_payload_ptr = NULL;
   uint32_t                     client_id               = 0;

   if (!base_ptr->offload_info_ptr)
   {
      return AR_EOK;
   }

   if (prev_delay_in_us != curr_delay_in_us)
   {
      if (base_ptr->offload_info_ptr)
      {
         client_id = base_ptr->offload_info_ptr->client_id;
      }

      CU_MSG(base_ptr->gu_ptr->log_id,
             DBG_HIGH_PRIO,
             "CMD: Raising path delay event to client 0x%lX for path id %lu, prev delay %lu new_delay %lu",
             client_id,
             path_id,
             prev_delay_in_us,
             curr_delay_in_us);

      delay_event_payload_ptr =
         (get_container_delay_event_t *)posal_memory_malloc(sizeof(get_container_delay_event_t), base_ptr->heap_id);
      if (NULL == delay_event_payload_ptr)
      {
         CU_MSG(base_ptr->gu_ptr->log_id,
                DBG_ERROR_PRIO,
                "Failed to allocate memory of size %lu to process get path delay event",
                sizeof(get_container_delay_event_t));
         return AR_ENOMEMORY;
      }

      delay_event_payload_ptr->new_delay_in_us  = (int32_t)curr_delay_in_us;
      delay_event_payload_ptr->prev_delay_in_us = (int32_t)prev_delay_in_us;
      delay_event_payload_ptr->path_id          = path_id;

      TRY(result,
          cu_raise_container_events_to_clients(base_ptr,
                                               OFFLOAD_EVENT_ID_GET_CONTAINER_DELAY,
                                               (int8_t *)delay_event_payload_ptr,
                                               sizeof(get_container_delay_event_t)));

      CU_MSG(base_ptr->gu_ptr->log_id,
             DBG_MED_PRIO,
             "CMD: Done Raising path delay event to client 0x%lX for path id %lu current channel mask=0x%x. result=%lu",
             client_id,
             path_id,
             base_ptr->curr_chan_mask,
             result);
   }

   CATCH(result, CU_MSG_PREFIX, base_ptr->gu_ptr->log_id)
   {
   }

   if (NULL != delay_event_payload_ptr)
   {
      posal_memory_free(delay_event_payload_ptr);
   }

   return result;
}

ar_result_t cu_offload_handle_gpr_cmd(cu_base_t *me_ptr, bool_t *switch_case_found_ptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   gpr_packet_t *packet_ptr = (gpr_packet_t *)me_ptr->cmd_msg.payload_ptr;
   *switch_case_found_ptr   = TRUE;
   // this handles only cmn handling b/w gen_cntr/spl_cntr.
   switch (packet_ptr->opcode)
   {
      case IMCL_INTER_PROC_TRIGGER_MSG_GPR:
      {
         result = cu_handle_inter_proc_triggered_imcl(me_ptr, packet_ptr);
         if (AR_EOK != result)
         {
            CU_MSG(me_ptr->gu_ptr->log_id,
                   DBG_ERROR_PRIO,
                   "Failed to handle IMCL inter domain message with result %lu",
                   result);
         }
         __gpr_cmd_free(packet_ptr);
         break;
      }
      case IMCL_INTER_PROC_POLLING_MSG_GPR:
      {
         intf_extn_param_id_imcl_incoming_data_t *payload_hdr_ptr =
            GPR_PKT_GET_PAYLOAD(intf_extn_param_id_imcl_incoming_data_t, packet_ptr);

         gu_ext_ctrl_port_t *gu_ext_ctrl_port_ptr = gu_get_ext_ctrl_port_for_inter_proc_imcl(me_ptr->gu_ptr,
                                                                                             packet_ptr->dst_port,
                                                                                             packet_ptr->src_domain_id,
                                                                                             payload_hdr_ptr->port_id);

         if (NULL == gu_ext_ctrl_port_ptr)
         {
            CU_MSG(me_ptr->gu_ptr->log_id,
                   DBG_ERROR_PRIO,
                   "MDF: IMCL Couldn't find ctrl port with ID %lu",
                   payload_hdr_ptr->port_id);
            result |= AR_EFAILED;
            break;
         }
         TRY(result,
             (ar_result_t)posal_queue_push_back(gu_ext_ctrl_port_ptr->this_handle.q_ptr,
                                                (posal_queue_element_t *)&me_ptr->cmd_msg));
         __gpr_cmd_free(packet_ptr);
         break;
      }
      case IMCL_INTER_PROC_PEER_STATE_UPDATE:
      {
         imcl_inter_proc_peer_state_update_t *payload_ptr =
            GPR_PKT_GET_PAYLOAD(imcl_inter_proc_peer_state_update_t, packet_ptr);

         gu_ext_ctrl_port_t *gu_ext_ctrl_port_ptr = gu_get_ext_ctrl_port_for_inter_proc_imcl(me_ptr->gu_ptr,
                                                                                             packet_ptr->dst_port,
                                                                                             packet_ptr->src_domain_id,
                                                                                             payload_ptr->ctrl_port_id);

         if (NULL == gu_ext_ctrl_port_ptr)
         {
            CU_MSG(me_ptr->gu_ptr->log_id,
                   DBG_ERROR_PRIO,
                   "MDF: IMCL Couldn't find ctrl port with ID %lu",
                   payload_ptr->ctrl_port_id);
            result |= AR_EFAILED;
            break;
         }

         TRY(result,
             me_ptr->cntr_vtbl_ptr->operate_on_ext_ctrl_port(me_ptr,
                                                             payload_ptr->sg_ops,
                                                             &gu_ext_ctrl_port_ptr,
                                                             FALSE /*is_self_sg*/));

         // Only change the connected state of a port not in the current subgraph.
         topo_port_state_t state = tu_sg_op_to_port_state(payload_ptr->sg_ops);
         if (TOPO_PORT_STATE_INVALID != state)
         {
            cu_ext_ctrl_port_t *ext_ctrl_port_ptr =
               (cu_ext_ctrl_port_t *)((uint8_t *)gu_ext_ctrl_port_ptr + me_ptr->ext_ctrl_port_cu_offset);
            ext_ctrl_port_ptr->connected_port_state = state;
         }

         cu_update_all_sg_port_states(me_ptr, FALSE);

         __gpr_cmd_free(packet_ptr);
         break;
      }
      default:
      {
         *switch_case_found_ptr = FALSE;
         return AR_EUNSUPPORTED;
      }
   }

   CATCH(result, CU_MSG_PREFIX, me_ptr->gu_ptr->log_id)
   {
   }
   return result;
}
