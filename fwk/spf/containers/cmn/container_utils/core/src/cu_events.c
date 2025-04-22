/**
 * \file cu_events.c
 * \brief
 *     This file implements the event registration utility functions
 *  
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "cu_events.h"
#include "cntr_cntr_if.h"
#include "cu_i.h"

static ar_result_t cu_create_event_node(uint32_t          log_id,
                                        uint32_t          event_id,
                                        cu_client_info_t *client_info_ptr,
                                        spf_list_node_t **root_node,
                                        POSAL_HEAP_ID     heap_id);

static ar_result_t cu_create_client_node(uint32_t          log_id,
                                         cu_client_info_t *client_info_ptr,
                                         spf_list_node_t **root_node,
                                         POSAL_HEAP_ID     heap_id);

static spf_list_node_t *cu_find_client_node(cu_client_info_t *client_info_ptr, spf_list_node_t *root_node);

static spf_list_node_t *cu_find_event_node(uint32_t event_id, spf_list_node_t *root_node);

/*========================================================================
 * Function name : cu_create_client_node
 *  This function creates a new client node and adds it to the client list.
 *========================================================================*/
static ar_result_t cu_create_client_node(uint32_t          log_id,
                                         cu_client_info_t *client_info_ptr,
                                         spf_list_node_t **root_node,
                                         POSAL_HEAP_ID     heap_id)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   cu_client_info_t *client_node_obj_ptr = NULL;
   MALLOC_MEMSET(client_node_obj_ptr, cu_client_info_t, sizeof(cu_client_info_t), heap_id, result);

   if (AR_EOK != (result = spf_list_insert_tail(root_node, client_node_obj_ptr, heap_id, TRUE)))
   {
      TOPO_MSG(log_id, DBG_ERROR_PRIO, "Failed to add client to the list, result", result);
   }

   memscpy(client_node_obj_ptr, sizeof(cu_client_info_t), client_info_ptr, sizeof(cu_client_info_t));

   CATCH(result, CU_MSG_PREFIX, log_id)
   {
      MFREE_NULLIFY(client_node_obj_ptr);
   }

   return AR_EOK;
}

/*=======================================================================
 * Function name : cu_create_event_node
 *  This function creates a new event node and adds it to the event list.
 *=======================================================================*/
static ar_result_t cu_create_event_node(uint32_t          log_id,
                                        uint32_t          event_id,
                                        cu_client_info_t *client_info_ptr,
                                        spf_list_node_t **root_node,
                                        POSAL_HEAP_ID     heap_id)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   cu_event_info_t *event_info_ptr = NULL;
   MALLOC_MEMSET(event_info_ptr, cu_event_info_t, sizeof(cu_event_info_t), heap_id, result);

   if (AR_EOK != (result = spf_list_insert_tail(root_node, event_info_ptr, heap_id, TRUE)))
   {
      TOPO_MSG(log_id, DBG_ERROR_PRIO, "Failed to add client to the list, result", result);
   }

   if (AR_EOK != (result = cu_create_client_node(log_id, client_info_ptr, &event_info_ptr->root_client_node, heap_id)))
   {
      TOPO_MSG(log_id, DBG_ERROR_PRIO, "create client node failed with result: %d", result);
   }
   event_info_ptr->event_id = event_id;

   CATCH(result, CU_MSG_PREFIX, log_id)
   {
      MFREE_NULLIFY(event_info_ptr);
   }
   return AR_EOK;
}

/*=======================================================================
 * Function name : cu_find_client_node
 *  This function returns client node for the given client info.
 *=======================================================================*/
static spf_list_node_t *cu_find_client_node(cu_client_info_t *client_info_ptr, spf_list_node_t *root_node)
{
   spf_list_node_t *currNode = root_node;
   if (currNode == NULL)
   {
      return currNode;
   }

   cu_client_info_t *cur_client_info_ptr = (cu_client_info_t *)(currNode->obj_ptr);

   while ((cur_client_info_ptr->dest_domain_id != client_info_ptr->dest_domain_id) &&
          (cur_client_info_ptr->src_domain_id != client_info_ptr->src_domain_id) &&
          (cur_client_info_ptr->src_port != client_info_ptr->src_port))
   {
      currNode = currNode->next_ptr;

      if (currNode == NULL)
      {
         break;
      }

      cur_client_info_ptr = (cu_client_info_t *)(currNode->obj_ptr);
   }
   return currNode;
}

/*=======================================================================
 * Function name : cu_find_event_node
 *  This function returns event node for the given event ID.
 *=======================================================================*/
static spf_list_node_t *cu_find_event_node(uint32_t event_id, spf_list_node_t *root_node)
{
   spf_list_node_t *currNode = root_node;
   if (NULL == currNode)
   {
      return currNode;
   }

   cu_event_info_t *event_info = (cu_event_info_t *)(currNode->obj_ptr);

   while ((event_id != event_info->event_id))
   {
      currNode = currNode->next_ptr;

      if (NULL == currNode)
      {
         break;
      }

      event_info = (cu_event_info_t *)(currNode->obj_ptr);
   }

   return currNode;
}

/*=====================================================================================================
 * Function name : cu_event_add_client
 * This function adds the client to the corresponding client list of the event, if exists. Otherwise
 * creates a new client list for the event and adds the client to the client list
 *======================================================================================================*/
ar_result_t cu_event_add_client(uint32_t          log_id,
                                uint32_t          event_id,
                                cu_client_info_t *client_info,
                                spf_list_node_t **root_node,
                                POSAL_HEAP_ID     heap_id)
{
   ar_result_t result = AR_EOK;

   spf_list_node_t *find_event_node = cu_find_event_node(event_id, *root_node);

   if ((NULL == find_event_node))
   {
      TOPO_MSG(log_id, DBG_HIGH_PRIO, "Event node not present for event id: 0x%lx, creating one", event_id);

      if (AR_EOK != (result = cu_create_event_node(log_id, event_id, client_info, root_node, heap_id)))
      {
         TOPO_MSG(log_id, DBG_ERROR_PRIO, "create event node failed with result: %d", result);
      }

      return AR_EOK;
   }

   cu_event_info_t *event_info = (cu_event_info_t *)(find_event_node->obj_ptr);

   spf_list_node_t **root_client_node = &(event_info->root_client_node);
   spf_list_node_t * find_client_node = cu_find_client_node(client_info, *root_client_node);

   if ((NULL != find_client_node))
   {
      TOPO_MSG(log_id, DBG_ERROR_PRIO, "client already present !");
      return AR_EOK;
   }

   if (AR_EOK != (result = cu_create_client_node(log_id, client_info, root_client_node, heap_id)))
   {
      TOPO_MSG(log_id, DBG_ERROR_PRIO, "create client node failed with result: %d", result);
   }

   return AR_EOK;
}

/*=====================================================================================================
 * Function name : cu_event_delete_client
 *  This function deletes the client from the client list of the event
 *======================================================================================================*/
ar_result_t cu_event_delete_client(uint32_t          log_id,
                                   uint32_t          event_id,
                                   cu_client_info_t *client_info,
                                   spf_list_node_t **root_node)
{
   ar_result_t result = AR_EOK;

   spf_list_node_t *find_event_node = cu_find_event_node(event_id, *root_node);
   if ((NULL == find_event_node))
   {
      TOPO_MSG(log_id, DBG_ERROR_PRIO, "Event node not present");
      return AR_EFAILED;
   }

   cu_event_info_t *event_info = (cu_event_info_t *)(find_event_node->obj_ptr);

   spf_list_node_t *find_client_node = cu_find_client_node(client_info, event_info->root_client_node);
   if ((NULL == find_client_node))
   {
      TOPO_MSG(log_id, DBG_ERROR_PRIO, "Client node not present");
      return AR_EFAILED;
   }

   cu_client_info_t *client_info_ptr = (cu_client_info_t *)find_client_node->obj_ptr;

   // AKR:
   if (client_info_ptr->event_cfg.data_ptr)
   {
      posal_memory_free(client_info_ptr->event_cfg.data_ptr);
      client_info_ptr->event_cfg.data_ptr = NULL;
   }

   if (AR_EOK != (result = spf_list_delete_node_and_free_obj(&find_client_node, &(event_info->root_client_node), TRUE)))
   {
      TOPO_MSG(log_id, DBG_ERROR_PRIO, "Event node deletion failed, result: %d", result);
   }

   return result;
}

/*============================================================================================
 * Function name : cu_delete_all_event_nodes
 *  This function deletes all the event nodes and client nodes created earlier
 *============================================================================================*/
void cu_delete_all_event_nodes(spf_list_node_t **root_node)
{
   for (spf_list_node_t *event_node_ptr = *root_node; (NULL != event_node_ptr); LIST_ADVANCE(event_node_ptr))
   {
      cu_event_info_t *event_info_ptr  = (cu_event_info_t *)event_node_ptr->obj_ptr;
      spf_list_node_t *client_list_ptr = (spf_list_node_t *)(event_info_ptr->root_client_node);
      spf_list_delete_list_and_free_objs(&client_list_ptr, TRUE /* pool_used */);
   }
   spf_list_delete_list_and_free_objs(root_node, TRUE);
}

/*============================================================================================
 * Function name : cu_find_client_info
 *  This function returns client list for the given event ID
 *============================================================================================*/
ar_result_t cu_find_client_info(uint32_t          log_id,
                                uint32_t          event_id,
                                spf_list_node_t * root_node,
                                spf_list_node_t **client_list_ptr)
{
   *client_list_ptr                 = NULL;
   spf_list_node_t *find_event_node = cu_find_event_node(event_id, root_node);
   if ((NULL == find_event_node))
   {
      // Avoiding the error msg for this event to avoid spamming of logs.
      if (event_id != CNTR_EVENT_ID_CONTAINER_FRAME_DELIVERY_DONE)
      {
         TOPO_MSG(log_id, DBG_ERROR_PRIO, "Event node not present");
      }
      return AR_EFAILED;
   }

   cu_event_info_t *event_info = (cu_event_info_t *)(find_event_node->obj_ptr);

   *client_list_ptr = event_info->root_client_node;

   return AR_EOK;
}
