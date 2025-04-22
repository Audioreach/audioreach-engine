/**
 * \file gpr_list.c
 * \brief
 *  	This file contains GPR list APIs
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

static inline uint32_t gpr_list_raw_is_not_empty(gpr_list_t *list)
{
   return (list->dummy.next != &list->dummy);
}

static uint32_t gpr_list_init_internal(gpr_list_t              *list,
                                       gpr_list_lock_enter_fn_t lock_fn,
                                       gpr_list_lock_leave_fn_t unlock_fn,
                                       bool_t                   en_secure)
{
   if ((list == NULL) || (NULL == lock_fn) || (NULL == unlock_fn))
   {
      return AR_EBADPARAM;
   }

   list->dummy.prev = &list->dummy;
   list->dummy.next = &list->dummy;
   list->size       = 0;
   list->lock_fn    = lock_fn;
   list->unlock_fn  = unlock_fn;
   list->en_secure  = en_secure;

   return AR_EOK;
}

GPR_EXTERNAL uint32_t gpr_list_get_head(gpr_list_t *list, gpr_list_node_t **ret_node)
{
   if (list == NULL || ret_node == NULL)
   {
      return AR_EBADPARAM;
   }

   list->lock_fn();

   if (gpr_list_raw_is_empty(list))
   {
      list->unlock_fn();

      return AR_EFAILED;
   }

   *ret_node = list->dummy.next;
   list->unlock_fn();
   return AR_EOK;
}

GPR_EXTERNAL uint32_t gpr_list_init(gpr_list_t              *list,
                                    gpr_list_lock_enter_fn_t lock_fn,
                                    gpr_list_lock_leave_fn_t unlock_fn)
{
   return gpr_list_init_internal(list, lock_fn, unlock_fn, TRUE);
}

GPR_EXTERNAL uint32_t gpr_list_init_v2(gpr_list_t              *list,
                                       gpr_list_lock_enter_fn_t lock_fn,
                                       gpr_list_lock_leave_fn_t unlock_fn)
{
   return gpr_list_init_internal(list, lock_fn, unlock_fn, FALSE);
}

GPR_EXTERNAL uint32_t gpr_list_init_node(gpr_list_node_t *node)
{
   if (node == NULL)
   {
      return AR_EBADPARAM;
   }

   node->list = NULL;
   node->prev = node;
   node->next = node;

   return AR_EOK;
}

GPR_EXTERNAL uint32_t gpr_list_delete(gpr_list_t *list, gpr_list_node_t *node)
{
   if ((list == NULL) || (node == NULL))
   {
      return AR_EBADPARAM;
   }

   list->lock_fn();

   gpr_list_raw_delete_node(node);
   list->size -= 1;

   list->unlock_fn();

   return AR_EOK;
}

GPR_EXTERNAL uint32_t gpr_list_clear(gpr_list_t *list)
{
   gpr_list_node_t *node;

   if (list == NULL)
   {
      return AR_EBADPARAM;
   }

   list->lock_fn();

   while (gpr_list_raw_is_not_empty(list))
   {
      node = list->dummy.next;
      gpr_list_raw_delete_node(node);
   }

   list->size = 0;

   list->unlock_fn();

   return AR_EOK;
}
