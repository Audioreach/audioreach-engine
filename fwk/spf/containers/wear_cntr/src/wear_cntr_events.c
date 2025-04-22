/**
 * \file wear_cntr_events.c
 * \brief
 *     This file implements the event registration utility functions
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "wear_cntr_events.h"
#include "wear_cntr_i.h"
#include "apm.h"

static ar_result_t wear_cntr_create_event_node(uint32_t                log_id,
                                              uint32_t                event_id,
                                              wcntr_client_info_t *client_info_ptr,
                                              spf_list_node_t **      root_node,
                                              POSAL_HEAP_ID           heap_id);

static ar_result_t wear_cntr_create_client_node(uint32_t                log_id,
                                               wcntr_client_info_t *client_info_ptr,
                                               spf_list_node_t **      root_node,
                                               POSAL_HEAP_ID           heap_id);

static spf_list_node_t *wcntr_find_client_node(wcntr_client_info_t *client_info_ptr, spf_list_node_t *root_node);

static spf_list_node_t *wcntr_find_event_node(uint32_t event_id, spf_list_node_t *root_node);
static ar_result_t wcntr_handle_events_reg_dereg(wcntr_base_t *       me_ptr,
                                                    wcntr_topo_reg_event_t *reg_event_ptr,
                                                    bool_t            is_register);

/*========================================================================
 * Function name : wear_cntr_create_client_node
 *  This function creates a new client node and adds it to the client list.
 *========================================================================*/
static ar_result_t wear_cntr_create_client_node(uint32_t                log_id,
                                               wcntr_client_info_t *client_info_ptr,
                                               spf_list_node_t **      root_node,
                                               POSAL_HEAP_ID           heap_id)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   wcntr_client_info_t *client_node_obj_ptr = NULL;
   MALLOC_MEMSET(client_node_obj_ptr, wcntr_client_info_t, sizeof(wcntr_client_info_t), heap_id, result);

   if (AR_EOK != (result = spf_list_insert_tail(root_node, client_node_obj_ptr, heap_id, TRUE)))
   {
      WCNTR_TOPO_MSG(log_id, DBG_ERROR_PRIO, "Failed to add client to the list, result", result);
   }

   memscpy(client_node_obj_ptr, sizeof(wcntr_client_info_t), client_info_ptr, sizeof(wcntr_client_info_t));

   CATCH(result, WCNTR_MSG_PREFIX, log_id)
   {
      MFREE_NULLIFY(client_node_obj_ptr);
   }

   return AR_EOK;
}

/*=======================================================================
 * Function name : wear_cntr_create_event_node
 *  This function creates a new event node and adds it to the event list.
 *=======================================================================*/
static ar_result_t wear_cntr_create_event_node(uint32_t                log_id,
                                              uint32_t                event_id,
                                              wcntr_client_info_t *client_info_ptr,
                                              spf_list_node_t **      root_node,
                                              POSAL_HEAP_ID           heap_id)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   wcntr_event_info_t *event_info_ptr = NULL;
   MALLOC_MEMSET(event_info_ptr, wcntr_event_info_t, sizeof(wcntr_event_info_t), heap_id, result);

   if (AR_EOK != (result = spf_list_insert_tail(root_node, event_info_ptr, heap_id, TRUE)))
   {
      WCNTR_TOPO_MSG(log_id, DBG_ERROR_PRIO, "Failed to add client to the list, result", result);
   }

   if (AR_EOK !=
       (result = wear_cntr_create_client_node(log_id, client_info_ptr, &event_info_ptr->root_client_node, heap_id)))
   {
      WCNTR_TOPO_MSG(log_id, DBG_ERROR_PRIO, "create client node failed with result: %d", result);
   }
   event_info_ptr->event_id = event_id;

   CATCH(result, WCNTR_MSG_PREFIX, log_id)
   {
      MFREE_NULLIFY(event_info_ptr);
   }
   return AR_EOK;
}

/*=======================================================================
 * Function name : wcntr_find_client_node
 *  This function returns client node for the given client info.
 *=======================================================================*/
static spf_list_node_t *wcntr_find_client_node(wcntr_client_info_t *client_info_ptr, spf_list_node_t *root_node)
{
   spf_list_node_t *currNode = root_node;
   if (currNode == NULL)
   {
      return currNode;
   }

   wcntr_client_info_t *cur_client_info_ptr = (wcntr_client_info_t *)(currNode->obj_ptr);

   while ((cur_client_info_ptr->dest_domain_id != client_info_ptr->dest_domain_id) &&
          (cur_client_info_ptr->src_domain_id != client_info_ptr->src_domain_id) &&
          (cur_client_info_ptr->src_port != client_info_ptr->src_port))
   {
      currNode = currNode->next_ptr;

      if (currNode == NULL)
      {
         break;
      }

      cur_client_info_ptr = (wcntr_client_info_t *)(currNode->obj_ptr);
   }

   return currNode;
}

/*=======================================================================
 * Function name : wcntr_find_event_node
 *  This function returns event node for the given event ID.
 *=======================================================================*/
static spf_list_node_t *wcntr_find_event_node(uint32_t event_id, spf_list_node_t *root_node)
{
   spf_list_node_t *currNode = root_node;
   if (NULL == currNode)
   {
      return currNode;
   }

   wcntr_event_info_t *event_info = (wcntr_event_info_t *)(currNode->obj_ptr);

   while ((event_id != event_info->event_id))
   {
      currNode = currNode->next_ptr;

      if (NULL == currNode)
      {
         break;
      }

      event_info = (wcntr_event_info_t *)(currNode->obj_ptr);
   }
   return currNode;
}

/*=====================================================================================================
 * Function name : wcntr_event_add_client
 * This function adds the client to the corresponding client list of the event, if exists. Otherwise
 * creates a new client list for the event and adds the client to the client list
 *======================================================================================================*/
ar_result_t wcntr_event_add_client(uint32_t                log_id,
                                      uint32_t                event_id,
                                      wcntr_client_info_t *client_info,
                                      spf_list_node_t **      root_node,
                                      POSAL_HEAP_ID           heap_id)
{
   ar_result_t result = AR_EOK;

   spf_list_node_t *find_event_node = wcntr_find_event_node(event_id, *root_node);

   if ((NULL == find_event_node))
   {
      WCNTR_TOPO_MSG(log_id, DBG_HIGH_PRIO, "Event node not present for event id: 0x%lx, creating one", event_id);

      if (AR_EOK != (result = wear_cntr_create_event_node(log_id, event_id, client_info, root_node, heap_id)))
      {
         WCNTR_TOPO_MSG(log_id, DBG_ERROR_PRIO, "create event node failed with result: %d", result);
      }

      return AR_EOK;
   }

   wcntr_event_info_t *event_info = (wcntr_event_info_t *)(find_event_node->obj_ptr);

   spf_list_node_t **root_client_node = &(event_info->root_client_node);
   spf_list_node_t * find_client_node = wcntr_find_client_node(client_info, *root_client_node);

   if ((NULL != find_client_node))
   {
      WCNTR_TOPO_MSG(log_id, DBG_ERROR_PRIO, "client already present !");
      return AR_EOK;
   }

   if (AR_EOK != (result = wear_cntr_create_client_node(log_id, client_info, root_client_node, heap_id)))
   {
      WCNTR_TOPO_MSG(log_id, DBG_ERROR_PRIO, "create client node failed with result: %d", result);
   }
   return AR_EOK;
}

/*=====================================================================================================
 * Function name : wcntr_event_delete_client
 *  This function deletes the client from the client list of the event
 *======================================================================================================*/
ar_result_t wcntr_event_delete_client(uint32_t                log_id,
                                         uint32_t                event_id,
                                         wcntr_client_info_t *client_info,
                                         spf_list_node_t **      root_node)
{
   ar_result_t result = AR_EOK;

   spf_list_node_t *find_event_node = wcntr_find_event_node(event_id, *root_node);
   if ((NULL == find_event_node))
   {
      WCNTR_TOPO_MSG(log_id, DBG_ERROR_PRIO, "Event node not present");
      return AR_EFAILED;
   }

   wcntr_event_info_t *event_info = (wcntr_event_info_t *)(find_event_node->obj_ptr);

   spf_list_node_t *find_client_node = wcntr_find_client_node(client_info, event_info->root_client_node);
   if ((NULL == find_client_node))
   {
      WCNTR_TOPO_MSG(log_id, DBG_ERROR_PRIO, "Client node not present");
      return AR_EFAILED;
   }

   wcntr_client_info_t *client_info_ptr = (wcntr_client_info_t *)find_client_node->obj_ptr;

   // AKR:
   if (client_info_ptr->event_cfg.data_ptr)
   {
      posal_memory_free(client_info_ptr->event_cfg.data_ptr);
      client_info_ptr->event_cfg.data_ptr = NULL;
   }

   if (AR_EOK != (result = spf_list_delete_node_and_free_obj(&find_client_node, &(event_info->root_client_node), TRUE)))
   {
      WCNTR_TOPO_MSG(log_id, DBG_ERROR_PRIO, "Event node deletion failed, result: %d", result);
   }

   return result;
}

/*============================================================================================
 * Function name : wcntr_delete_all_event_nodes
 *  This function deletes all the event nodes and client nodes created earlier
 *============================================================================================*/
void wcntr_delete_all_event_nodes(spf_list_node_t **root_node)
{
   for (spf_list_node_t *event_node_ptr = *root_node; (NULL != event_node_ptr); LIST_ADVANCE(event_node_ptr))
   {
      wcntr_event_info_t *event_info_ptr  = (wcntr_event_info_t *)event_node_ptr->obj_ptr;
      spf_list_node_t *      client_list_ptr = (spf_list_node_t *)(event_info_ptr->root_client_node);
      spf_list_delete_list_and_free_objs(&client_list_ptr, TRUE /* pool_used */);
   }
   spf_list_delete_list_and_free_objs(root_node, TRUE);
}

/*============================================================================================
 * Function name : wcntr_find_client_info
 *  This function returns client list for the given event ID
 *============================================================================================*/
ar_result_t wcntr_find_client_info(uint32_t          log_id,
                                      uint32_t          event_id,
                                      spf_list_node_t * root_node,
                                      spf_list_node_t **client_list_ptr)
{
   *client_list_ptr                 = NULL;
   spf_list_node_t *find_event_node = wcntr_find_event_node(event_id, root_node);
   if ((NULL == find_event_node))
   {
      // Avoiding the error msg for this event to avoid spamming of logs.
      //  if (event_id != CNTR_EVENT_ID_CONTAINER_FRAME_DELIVERY_DONE)
      {
         WCNTR_TOPO_MSG(log_id, DBG_ERROR_PRIO, "Event node not present");
      }
      return AR_EFAILED;
   }

   wcntr_event_info_t *event_info = (wcntr_event_info_t *)(find_event_node->obj_ptr);

   *client_list_ptr = event_info->root_client_node;

   return AR_EOK;
}

ar_result_t wcntr_register_module_events(wcntr_base_t *me_ptr, gpr_packet_t *packet_ptr)
{
   ar_result_t result     = AR_EOK;
   uint32_t    sent_count = 0;
   // INIT_EXCEPTION_HANDLING

   apm_cmd_header_t *in_apm_cmd_header = GPR_PKT_GET_PAYLOAD(apm_cmd_header_t, packet_ptr);
   uint8_t *         payload           = NULL;
   uint32_t          alignment_size    = 0;
   uint32_t          payload_size      = in_apm_cmd_header->payload_size;
   gpr_packet_t *    temp_gpr_pkt_ptr  = NULL;

   // VERIFY(result, me_ptr->cntr_vtbl_ptr && me_ptr->cntr_vtbl_ptr->register_events);

   result = spf_svc_get_cmd_payload_addr(me_ptr->gu_ptr->log_id,
                                         packet_ptr,
                                         &temp_gpr_pkt_ptr,
                                         (uint8_t **)&payload,
                                         &alignment_size,
                                         NULL,
                                         apm_get_mem_map_client());

   if (NULL != payload)
   {
      while (payload_size > 0)
      {
         apm_module_register_events_t *current_payload = (apm_module_register_events_t *)payload;
         if (sizeof(apm_module_register_events_t) > payload_size)
         {
            break;
         }

         uint32_t one_event_size = sizeof(apm_module_register_events_t) + current_payload->event_config_payload_size;

         if (payload_size >= one_event_size)
         {
            wcntr_topo_reg_event_t reg_event_payload;
            memset(&reg_event_payload, 0 , sizeof(reg_event_payload));

            reg_event_payload.src_port                  = packet_ptr->src_port;
            reg_event_payload.src_domain_id             = packet_ptr->src_domain_id;
            reg_event_payload.dest_domain_id            = packet_ptr->dst_domain_id;
            reg_event_payload.event_id                  = current_payload->event_id;
            reg_event_payload.token                     = packet_ptr->token;
            reg_event_payload.event_cfg.actual_data_len = current_payload->event_config_payload_size;
            reg_event_payload.event_cfg.data_ptr        = (int8_t *)(current_payload + 1);

            if (current_payload->module_instance_id == me_ptr->cntr_instance_id)
            {
               // events to the container
               return wcntr_handle_events_reg_dereg(me_ptr, &reg_event_payload, current_payload->is_register);
            }
            else
            {
               wcntr_gu_module_t *module_ptr = wcntr_gu_find_module(me_ptr->gu_ptr, current_payload->module_instance_id);
               if (NULL == module_ptr)
               {
                  WCNTR_MSG(me_ptr->gu_ptr->log_id,
                         DBG_ERROR_PRIO,
                         "register events: Module 0x%lx not found",
                         current_payload->module_instance_id);
               }
               else
               {
                  wcu_module_t *cu_module_ptr = (wcu_module_t *)((uint8_t *)module_ptr + me_ptr->module_cu_offset);
                  bool_t       capi_supports_v1_event = FALSE;
                  result |= wcntr_register_events_utils(me_ptr,
                                                           module_ptr,
                                                           &reg_event_payload,
                                                           current_payload->is_register,
                                                           &capi_supports_v1_event);

                  if (capi_supports_v1_event)
                  {
                     wcntr_client_info_t client_info;
                     client_info.src_port       = reg_event_payload.src_port;
                     client_info.src_domain_id  = reg_event_payload.src_domain_id;
                     client_info.dest_domain_id = reg_event_payload.dest_domain_id;

                     if (current_payload->is_register)
                     {
                        result = wcntr_event_add_client(me_ptr->gu_ptr->log_id,
                                                           reg_event_payload.event_id,
                                                           &client_info,
                                                           &cu_module_ptr->event_list_ptr,
                                                           me_ptr->heap_id);
                        if (AR_EOK != result)
                        {
                           WCNTR_MSG(me_ptr->gu_ptr->log_id,
                                  DBG_ERROR_PRIO,
                                  "Add client to module event list failed, result: %d",
                                  result);
                        }
                     }
                     else
                     {
                        result = wcntr_event_delete_client(me_ptr->gu_ptr->log_id,
                                                              reg_event_payload.event_id,
                                                              &client_info,
                                                              &cu_module_ptr->event_list_ptr);
                        if (AR_EOK != result)
                        {
                           WCNTR_TOPO_MSG(me_ptr->gu_ptr->log_id,
                                    DBG_ERROR_PRIO,
                                    "Delete client from module event list failed, result: %d",
                                    result);
                        }
                     }
                  }
               }
            }
         }
         payload += WCNTR_ALIGN_8_BYTES(one_event_size);
         payload_size -= one_event_size;
         sent_count++;
         WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_HIGH_PRIO, "Register events, payload_size: %d", payload_size);
      }
   }

   if (!sent_count)
   {
      WCNTR_MSG(me_ptr->gu_ptr->log_id,
             DBG_ERROR_PRIO,
             "Register events, 0 events registered, payload too small, size = %lu",
             payload_size);
   }

   return result;
}

/**
 * Common handling of register/deregister events addressed to the container instance ID.
 */
ar_result_t wcntr_handle_events_reg_dereg(wcntr_base_t *me_ptr, wcntr_topo_reg_event_t *reg_event_ptr, bool_t is_register)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING

   int8_t *event_cfg_ptr = NULL;

   WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_MED_PRIO, "Container received event reg/deregistration");

   wcntr_client_info_t client_info;
   client_info.src_port                  = reg_event_ptr->src_port;
   client_info.src_domain_id             = reg_event_ptr->src_domain_id;
   client_info.dest_domain_id            = reg_event_ptr->dest_domain_id;
   client_info.token                     = reg_event_ptr->token;
   client_info.event_cfg.actual_data_len = reg_event_ptr->event_cfg.actual_data_len;
   client_info.event_cfg.data_ptr        = NULL;

   if (0 != client_info.event_cfg.actual_data_len)
   {
      event_cfg_ptr = (int8_t *)posal_memory_malloc(client_info.event_cfg.actual_data_len, me_ptr->heap_id);

      VERIFY(result, NULL != event_cfg_ptr);

      client_info.event_cfg.data_ptr = event_cfg_ptr;
      memscpy(client_info.event_cfg.data_ptr,
              client_info.event_cfg.actual_data_len,
              (reg_event_ptr->event_cfg.data_ptr),
              client_info.event_cfg.actual_data_len);
   }

   if (is_register)
   {
      result = wcntr_event_add_client(me_ptr->gu_ptr->log_id,
                                         reg_event_ptr->event_id,
                                         &client_info,
                                         &me_ptr->event_list_ptr,
                                         me_ptr->heap_id);
      if (AR_EOK != result)
      {
         WCNTR_MSG(me_ptr->gu_ptr->log_id,
                DBG_ERROR_PRIO,
                "Add client to container event list failed, result: %d",
                result);
      }
   }
   else
   {
      result = wcntr_event_delete_client(me_ptr->gu_ptr->log_id,
                                            reg_event_ptr->event_id,
                                            &client_info,
                                            &me_ptr->event_list_ptr);
      if (AR_EOK != result)
      {
         WCNTR_MSG(me_ptr->gu_ptr->log_id,
                DBG_ERROR_PRIO,
                "Delete client from container event list failed, result: %d",
                result);
      }
   }

   CATCH(result, WCNTR_MSG_PREFIX, me_ptr->gu_ptr->log_id)
   {
   }

   return result;
}
