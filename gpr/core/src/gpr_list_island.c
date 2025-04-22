/**
 * \file gpr_list.c
 * \brief
 *  	This file contains GPR list island utils
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/******************************************************************************
 * Includes                                                                   *
 *****************************************************************************/
#include "gpr_list.h"
#include "gpr_list_i.h"

/******************************************************************************
 * Functions                                                                   *
 *****************************************************************************/

/* Verify that the header of the de-linking node is linked to a linked list.
 * This helps catch memory corruption issues, and programming errors where
 * the same node was inserted twice (i.e. init_node(node1),
 * insert(list1, node1), insert(list2, node1), error).
 */
// Enhancement: Crash for double free scenarios on SIM
uint32_t gpr_list_raw_assert_node_is_linked(gpr_list_node_t *node)
{
   if ((node->prev->next != node) || (node->next->prev != node))
   {
      AR_MSG(DBG_ERROR_PRIO, "Detected a corruption in the node trying to be inserted into the list");
      return AR_EFAILED;
   }
   return AR_EOK;
}

void gpr_list_raw_delete_node(gpr_list_node_t *node)
{
   gpr_list_raw_assert_node_is_linked(node);
   /* Verify that the header of the unlinking node was not already corrupted.
    * This helps catch buffer overflow attacks targeting the linked list  */

   node->prev->next = node->next;
   node->next->prev = node->prev;
   node->prev       = node;
   node->next       = node;
}

GPR_EXTERNAL uint32_t gpr_list_remove_head(gpr_list_t *list, gpr_list_node_t **ret_node)
{
   gpr_list_node_t *node;

   if (list == NULL)
   {
      return AR_EBADPARAM;
   }

   list->lock_fn();

   if (gpr_list_raw_is_empty(list))
   {
      list->unlock_fn();

      return AR_EFAILED;
   }

   node = list->dummy.next;
   gpr_list_raw_delete_node(node);
   list->size -= 1;

   list->unlock_fn();

   if (ret_node != NULL)
   {
      *ret_node = node;
   }

   return AR_EOK;
}

GPR_EXTERNAL bool_t gpr_list_raw_is_empty(gpr_list_t *list)
{
   return ((list == NULL) || (list->dummy.next == &list->dummy));
}

GPR_EXTERNAL uint32_t gpr_list_add_tail(gpr_list_t *list, gpr_list_node_t *node)
{
   uint32_t rc = AR_EOK;

   if ((list == NULL) || (node == NULL))
   {
      return AR_EBADPARAM;
   }

   list->lock_fn();

   rc = gpr_list_raw_insert_node_before(list, &list->dummy, node);

   if (AR_EOK != rc)
   {
      list->unlock_fn();
      return rc;
   }

   list->size += 1;
   list->unlock_fn();

   return AR_EOK;
}

uint32_t gpr_list_raw_insert_node_before(gpr_list_t *list, gpr_list_node_t *ref_node, gpr_list_node_t *new_node)
{
   uint32_t rc = AR_EOK;

   rc = gpr_list_raw_assert_node_is_linked(ref_node);
   if (rc != AR_EOK)
   {
      return rc;
   }

   rc = gpr_list_raw_assert_secure_node_is_unlinked(list, new_node);
   if (rc != AR_EOK)
   {
      return rc;
   }

   new_node->list       = list;
   ref_node->prev->next = new_node;
   new_node->prev       = ref_node->prev;
   new_node->next       = ref_node;
   ref_node->prev       = new_node;

   return AR_EOK;
}

/* Verify that the header of the linking node points to itself. This helps
 * catch programming errors where the same node is inserted more than once on
 * a linked list.
 */
// Enhancement: Crash for double free scenarios on SIM
uint32_t gpr_list_raw_assert_secure_node_is_unlinked(gpr_list_t *list, gpr_list_node_t *node)
{
   if ((list->en_secure) && ((node->next != node) || (node->prev != node)))
   {
      AR_MSG(DBG_ERROR_PRIO, "Detected same node was already inserted or is corrupted");
      return AR_EFAILED;
   }
   return AR_EOK;
}