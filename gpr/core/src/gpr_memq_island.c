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

/******************************************************************************
 * Functions                                                                   *
 *****************************************************************************/

uint32_t gpr_memq_is_metadata_unique(gpr_memq_block_t *block, uint32_t total_unique_mds, int32_t metadata)
{
   uint32_t md_index;

   for (md_index = 0; md_index < total_unique_mds; md_index++)
   {
      if (block->unique_metadata_ids[md_index] == (uint32_t)metadata)
      {
         block->unique_metadata_counts[md_index]++;
         return 0;
      }
   }
   return 1;
}

/* Helper Functions */
uint32_t gpr_memq_size_of_metadata_and_overhead(gpr_memq_block_t *block)
{
   return ((GPR_MEMQ_BYTES_PER_METADATA_ITEM_V * block->metadata_size) + GPR_MEMQ_UNIT_OVERHEAD_V);
}

GPR_EXTERNAL uint32_t gpr_memq_node_set_metadata(gpr_memq_block_t *block, void *mem_ptr, uint32_t index, int32_t value)
{
   int32_t *metadata;
   if (index >= block->metadata_size)
   {
      return AR_EBADPARAM;
   }
   metadata =
      ((int32_t *)(((char_t *)mem_ptr) - ((block->metadata_size - index) * GPR_MEMQ_BYTES_PER_METADATA_ITEM_V)));

   *metadata = value;
   return AR_EOK;
}

GPR_EXTERNAL uint32_t gpr_memq_node_get_metadata(gpr_memq_block_t *block,
                                                 void             *mem_ptr,
                                                 uint32_t          index,
                                                 int32_t          *metadata)
{
   if (index >= block->metadata_size)
   {
      return AR_EBADPARAM;
   }
   *metadata =
      *((int32_t *)(((char_t *)mem_ptr) - ((block->metadata_size - index) * GPR_MEMQ_BYTES_PER_METADATA_ITEM_V)));

   return AR_EOK;
}

GPR_EXTERNAL void *gpr_memq_alloc(gpr_memq_block_t *block)
{
   gpr_list_node_t *node;
   char_t          *mem_ptr;
   uint32_t         md_interator;
   uint32_t         md_index;
   int32_t          metadata         = 0;
   uint32_t         total_unique_mds = 0;

   if (gpr_list_remove_head(&block->free_q, &node) == AR_EOK)
   {
      return (((char_t *)node) + gpr_memq_size_of_metadata_and_overhead(block));
   }

   AR_MSG(DBG_ERROR_PRIO, "Out of memory failure");

   ar_mem_set(block->unique_metadata_ids, 0xFF, block->total_units * GPR_MEMQ_BYTES_PER_METADATA_ITEM_V);
   ar_mem_set(block->unique_metadata_counts, 0, block->total_units * GPR_MEMQ_BYTES_PER_METADATA_ITEM_V);
   mem_ptr = block->base_addr + gpr_memq_size_of_metadata_and_overhead(block);

   for (md_interator = 0; md_interator < block->total_units; md_interator++)
   {
      gpr_memq_node_get_metadata(block, mem_ptr, 0, &metadata);
      mem_ptr = mem_ptr + block->unit_size;
      if (gpr_memq_is_metadata_unique(block, total_unique_mds, metadata))
      {
         block->unique_metadata_ids[total_unique_mds]    = metadata;
         block->unique_metadata_counts[total_unique_mds] = 1;
         total_unique_mds++;
      }
   }

   for (md_index = 0; md_index < total_unique_mds; md_index++)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Out of GPR packets: Module with Instance ID: %lx holding number of gpr packets: %lu",
             block->unique_metadata_ids[md_index],
             block->unique_metadata_counts[md_index]);
   }
   return NULL;
}

GPR_EXTERNAL void gpr_memq_free(gpr_memq_block_t *block, void *data_ptr)
{
   uint32_t         rc = AR_EOK;
   gpr_list_node_t *node;

   if ((block == NULL) || (data_ptr == NULL))
   {
      AR_MSG(DBG_ERROR_PRIO, "GPR memq: block is NULL or data ptr is NULL");
      return;
   }
   node = ((gpr_list_node_t *)(((char_t *)data_ptr) - gpr_memq_size_of_metadata_and_overhead(block)));
   rc   = gpr_list_add_tail(&block->free_q, node);
   if (AR_EOK != rc)
   {
      AR_MSG(DBG_ERROR_PRIO, "GPR memq: Cannot free packet, packet is corrupted or already freed rc %d", rc);
   }
}
