/**
 * \file gpr_session.c
 * \brief
 *  	This file contains GPR session implementation
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/******************************************************************************
 * Includes                                                                    *
 *****************************************************************************/
#include "gpr_session.h"
#include "gpr_heap_i.h"

/**
  @brief Allocates memory and returns a pointer to a session
  (contains callback function, callback argument) for a given
  src_port

  @param[in] src_port      Address/port of src module trying to register
  @param[out] ret_entry   Double pointer to the session created

  @detdesc
  Allocates a new node in the linked list for each src port. Each node
  represents a session which points to a callback function and corresponding
  callback argument which will be populated by the caller of this function.
  This is how a module registers to the GPR service.

  @return
  #AR_EOK when successful.
*/
GPR_INTERNAL uint32_t gpr_init_session(uint32_t                src_port,
                                       gpr_module_entry_t    **ret_entry,
                                       gpr_module_node_list_t *list_handle)
{
   if (NULL == list_handle)
   {
      return AR_EFAILED;
   }

   gpr_heap_index_t heap_index = list_handle->heap_index;
   ar_heap_info     heap_info;
   gpr_populate_ar_heap_info(heap_index, AR_HEAP_ALIGN_8_BYTES, &heap_info);

   gpr_module_node_t *new_node = (gpr_module_node_t *)ar_heap_malloc(sizeof(gpr_module_node_t), &heap_info);
   if (NULL == new_node)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Error: Failed to allocate gpr module node of size %d port 0x%lx heap index %lu",
             sizeof(gpr_module_node_t),
             src_port,
             heap_index);
      return AR_ENOMEMORY;
   }

   gpr_module_node_t **chain      = &list_handle->cb_list[0];
   uint32_t            free_index = 0xFFFF;

   for (uint32_t i = 0; i < list_handle->max_cb_list_size; i++)
   {
      if (chain[i] == NULL)
      {
         free_index = i;
      }
      else // Error handling mechanism, may be removed
      {
         gpr_module_node_t *temp = chain[i];
         if (temp->my_module_port == src_port)
         {
            AR_MSG(DBG_ERROR_PRIO, "Error: Trying to register with source port %lu again, failing", src_port);
            ar_heap_free(new_node, &heap_info);
            return AR_EFAILED;
         }
      }
   }

   if (free_index == 0xFFFF) // Error handling mechanism, may be removed
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Error: Finding any free space, increase session_array_size cur_size: %lu",
             list_handle->max_cb_list_size);
      return AR_EFAILED;
   }

   // Fill up the new node
   chain[free_index]        = new_node;
   new_node->my_module_port = src_port;
   *ret_entry               = &new_node->session;

   chain[free_index]->next = NULL; // as it is array based (next node is not required)

   return AR_EOK;
}

/**
  @brief De-allocates session memory for a given src_port

  @param[in] src_port       Address/port of src module trying to de-register

  @detdesc
  De-allocates a node in the linked list for a given src_port.
  Make sure not to delet other nodes in the linked list.
  This is how a local module registers to the GPR service.

  @return
  #AR_EOK when successful.
*/
GPR_INTERNAL uint32_t gpr_deinit_session(uint32_t src_port, gpr_module_node_list_t *list_handle)
{
   if (NULL == list_handle)
   {
      return AR_EFAILED;
   }

   gpr_heap_index_t heap_index = list_handle->heap_index;
   ar_heap_info     heap_info;
   gpr_populate_ar_heap_info(heap_index, AR_HEAP_ALIGN_8_BYTES, &heap_info);

   gpr_module_node_t **cb_list = &list_handle->cb_list[0];

   for (uint32_t i = 0; i < list_handle->max_cb_list_size; i++)
   {
      if ((cb_list[i] != NULL) && (cb_list[i]->my_module_port == src_port))
      {
         ar_heap_free(cb_list[i], &heap_info);
         cb_list[i] = NULL;
         return AR_EOK;
      }
   }

   AR_MSG(DBG_HIGH_PRIO, "No such module port (%lu) found to deinit", src_port);
   return AR_ENOTEXIST; // NOT exists
}
