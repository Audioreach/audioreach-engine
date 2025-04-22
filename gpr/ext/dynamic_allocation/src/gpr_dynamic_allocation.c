/**
 * \file gpr_dynamic_allocation.c
 * \brief
 *  	This file contains implementation for dynamic packet allocation.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/******************************************************************************
 * Includes                                                                    *
 *****************************************************************************/
#include "gpr_dynamic_allocation.h"
#include "gpr_list.h"
#include "ar_osal_mutex.h"
#include "ar_osal_error.h"
#include "ar_osal_heap.h"
#include "ar_osal_mem_op.h"

/*****************************************************************************
 * Structure definitions                                                     *
 ****************************************************************************/
typedef struct gpr_dynamic_packet
{
   gpr_list_node_t node;
   gpr_packet_t * packet;
} gpr_dynamic_packet_t;

gpr_list_t      dynamic_packet_list;
ar_osal_mutex_t gpr_dyn_packet_list_lock;
static bool_t   gpr_dynamic_packet_init_done = FALSE;

/*****************************************************************************
 * Helper functions                                                          *
 ****************************************************************************/
static void gpr_dyn_packet_lock_fn(void)
{
   (void)ar_osal_mutex_lock(gpr_dyn_packet_list_lock);
}

static void gpr_dyn_packet_unlock_fn(void)
{
   (void)ar_osal_mutex_unlock(gpr_dyn_packet_list_lock);
}

static void gpr_free_dynamic_packet_raw(gpr_dynamic_packet_t *dynamic_packet)
{
   ar_heap_info heap_info;

   ar_mem_set((void *)&heap_info, 0, sizeof(ar_heap_info));
   heap_info.tag         = AR_HEAP_TAG_DEFAULT;
   heap_info.align_bytes = AR_HEAP_ALIGN_8_BYTES;

   ar_heap_free((void *)(dynamic_packet->packet), &heap_info);
   dynamic_packet->packet = NULL;
   ar_heap_free((void *)dynamic_packet, &heap_info);
   dynamic_packet = NULL;
   return;
}

static void gpr_free_dynamic_packet_list(void)
{
   gpr_list_node_t * node = NULL;

   while (!gpr_list_raw_is_empty(&dynamic_packet_list))
   {
      gpr_list_remove_head(&dynamic_packet_list, &node);
      gpr_free_dynamic_packet_raw((gpr_dynamic_packet_t *)((void *)node));
   }
}

static gpr_dynamic_packet_t *gpr_get_dynamic_packet(gpr_packet_t *packet)
{
   gpr_list_node_t *     node = NULL;
   gpr_list_node_t *     head = NULL;
   gpr_dynamic_packet_t *dynamic_packet = NULL;

   if ((gpr_list_raw_is_empty(&dynamic_packet_list)))
   {
      return dynamic_packet;
   }
   else
   {
      gpr_list_get_head(&dynamic_packet_list, &head);
      node = head;
      do
      {
         dynamic_packet = (gpr_dynamic_packet_t *)((void *)node);
         if (dynamic_packet->packet == packet)
         {
            return dynamic_packet;
         }

         node = node->next;
      } while (node != head);
   }

   dynamic_packet = NULL;
   return dynamic_packet;
}

GPR_INTERNAL uint32_t gpr_allocate_dynamic_packet(gpr_packet_t **packet, uint32_t size)
{
   gpr_dynamic_packet_t *dynamic_packet = NULL;
   ar_heap_info          heap_info;

   if (packet == NULL) {
      AR_MSG(DBG_ERROR_PRIO, "alloc error, NULL packet found");
      return AR_EBADPARAM;
   }

   ar_mem_set((void *)&heap_info, 0, sizeof(ar_heap_info));
   heap_info.tag         = AR_HEAP_TAG_DEFAULT;
   heap_info.align_bytes = AR_HEAP_ALIGN_8_BYTES;

   dynamic_packet = (gpr_dynamic_packet_t *)ar_heap_malloc(sizeof(gpr_dynamic_packet_t), &heap_info);

   if (!dynamic_packet)
   {
      AR_MSG(DBG_ERROR_PRIO, "failed to alloc memory %lu", size);
      return AR_ENOMEMORY;
   }

   *packet = (gpr_packet_t *)ar_heap_malloc(size, &heap_info);
   if (*packet == NULL)
   {
      ar_heap_free((void *)dynamic_packet, &heap_info);
      AR_MSG(DBG_ERROR_PRIO, "failed to alloc memory %lu", size);
      return AR_ENOMEMORY;
   }

   dynamic_packet->packet = *packet;
   gpr_list_init_node((gpr_list_node_t *)dynamic_packet);
   gpr_list_add_tail(&dynamic_packet_list, (gpr_list_node_t *)dynamic_packet);
   return AR_EOK;
}

GPR_INTERNAL uint32_t gpr_check_and_free_dynamic_packet(gpr_packet_t *packet)
{
   gpr_dynamic_packet_t *dynamic_packet = gpr_get_dynamic_packet(packet);

   if (dynamic_packet == NULL)
   {
      AR_MSG(DBG_ERROR_PRIO, "failed to free memory");
      return AR_EBADPARAM;
   }

   gpr_list_delete(&dynamic_packet_list, (gpr_list_node_t *)((void *)dynamic_packet));
   gpr_free_dynamic_packet_raw(dynamic_packet);
   return AR_EOK;
}

GPR_INTERNAL uint32_t gpr_dynamic_packet_init(void)
{
   uint32_t result = AR_EOK;

   if (!gpr_dynamic_packet_init_done)
   {
      ar_osal_mutex_create(&gpr_dyn_packet_list_lock);
      result = gpr_list_init(&dynamic_packet_list, gpr_dyn_packet_lock_fn, gpr_dyn_packet_unlock_fn);
      if (AR_EOK == result)
      {
         gpr_dynamic_packet_init_done = TRUE;
      }
   }

   return result;
}

GPR_INTERNAL uint32_t gpr_dynamic_packet_deinit(void)
{
   if (gpr_dynamic_packet_init_done)
   {
      gpr_free_dynamic_packet_list();
      ar_osal_mutex_destroy(gpr_dyn_packet_list_lock);
      gpr_dynamic_packet_init_done = FALSE;
   }

   return AR_EOK;
}
