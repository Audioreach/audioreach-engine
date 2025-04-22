/**
 * \file gpr_memq.c
 * \brief
 *  	This file contains GPR memq implementation
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/******************************************************************************
 * Includes                                                                   *
 *****************************************************************************/
#include "gpr_msg_if.h"
#include "gpr_memq.h"
#include "ar_osal_mem_op.h"
#include "ar_osal_heap.h"
#include "gpr_heap_i.h"
/******************************************************************************
 * Functions                                                                   *
 *****************************************************************************/

/* Memory Queue Routines */

/* The caller provides the contiguous virtual memory for partitioning into
** fixed memory allocation units. The unit_size is the amount of usable
** application memory for each allocation unit plus overhead (GPR_MEMQ_UNIT_OVERHEAD)*/

GPR_EXTERNAL int gpr_memq_init(gpr_memq_block_t        *block,
                               char_t                  *heap_base,
                               uint32_t                 heap_size,
                               uint32_t                 unit_size,
                               uint32_t                 metadata_size,
                               gpr_memq_lock_enter_fn_t lock_fn,
                               gpr_memq_lock_leave_fn_t unlock_fn,
                               gpr_heap_index_t         heap_index)
{
   uint32_t rc = AR_EOK;
   if ((NULL == block) || (NULL == heap_base) || (NULL == lock_fn) || (NULL == unlock_fn))
   {
      return AR_EBADPARAM;
   }

   if (unit_size < GPR_MEMQ_BYTES_PER_METADATA_ITEM_V * metadata_size)
   {
      return AR_EBADPARAM;
   }

   /* Partition the heap into fixed units in the free queue */
   (void)gpr_list_init(&block->free_q, lock_fn, unlock_fn);
   block->base_addr     = heap_base;
   block->unit_size     = unit_size;
   block->metadata_size = metadata_size;

   while (heap_size >= unit_size)
   {
      (void)gpr_list_init_node((gpr_list_node_t *)heap_base);

      rc = gpr_list_add_tail(&block->free_q, ((gpr_list_node_t *)heap_base));
      if (AR_EOK != rc)
      {
         AR_MSG(DBG_ERROR_PRIO, "GPR memq: Adding node to free queue failed, init failed rc %d", rc);
         return rc;
      }

      heap_base += unit_size;
      heap_size -= unit_size;
   }

   block->total_units = block->free_q.size;

   /*populate heap info*/
   ar_heap_info heap_info;
   ar_mem_set((void *)&heap_info, 0, sizeof(ar_heap_info));
   gpr_populate_ar_heap_info(heap_index, AR_HEAP_ALIGN_8_BYTES, &heap_info);

   /* Allocate memory to be used for out-of-memory detection */
   block->unique_metadata_ids =
      ((uint32_t *)ar_heap_malloc(block->total_units * GPR_MEMQ_BYTES_PER_METADATA_ITEM_V, &heap_info));

   if (block->unique_metadata_ids == NULL)
   {
      AR_MSG(DBG_ERROR_PRIO, "Cannot allocate memory for oom analysis");
      return AR_EFAILED;
   }

   ar_mem_set(block->unique_metadata_ids, 0, (block->total_units * GPR_MEMQ_BYTES_PER_METADATA_ITEM_V));

   block->unique_metadata_counts =
      ((uint32_t *)ar_heap_malloc(block->total_units * GPR_MEMQ_BYTES_PER_METADATA_ITEM_V, &heap_info));
   if (block->unique_metadata_counts == NULL)
   {
      AR_MSG(DBG_ERROR_PRIO, "Cannot allocate memory for oom analysis");
      return AR_EFAILED;
   }

   ar_mem_set(block->unique_metadata_counts, 0, (block->total_units * GPR_MEMQ_BYTES_PER_METADATA_ITEM_V));

   return AR_EOK;
}

#ifndef DISABLE_DEINIT

GPR_EXTERNAL void gpr_memq_deinit(gpr_memq_block_t *block, gpr_heap_index_t heap_index)
{
   /*populate heap info*/
   ar_heap_info heap_info;
   ar_mem_set((void *)&heap_info, 0, sizeof(ar_heap_info));
   gpr_populate_ar_heap_info(heap_index, AR_HEAP_ALIGN_8_BYTES, &heap_info);

   if (block == NULL)
   {
      AR_MSG(DBG_ERROR_PRIO, "block is NULL");
      return;
   }

   if (block->free_q.size != block->total_units)
   {
      AR_MSG(DBG_ERROR_PRIO, "memory leak detected");
   }

   gpr_list_clear(&block->free_q);

   if (NULL != block->unique_metadata_ids)
   {
      ar_heap_free(block->unique_metadata_ids, &heap_info);
      block->unique_metadata_ids = NULL;
   }

   if (NULL != block->unique_metadata_counts)
   {
      ar_heap_free(block->unique_metadata_counts, &heap_info);
      block->unique_metadata_counts = NULL;
   }
}

#endif