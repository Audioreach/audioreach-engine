#ifndef __GPR_MEMQ_H__
#define __GPR_MEMQ_H__
/**
 * \file gpr_memq.h
 * \brief
 *  	This file contains GPR mem queue APIs
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gpr_list.h"
#include "gpr_api.h"

typedef void (*gpr_memq_lock_enter_fn_t)(void);
typedef void (*gpr_memq_lock_leave_fn_t)(void);

/* Memory Queue Definitions */

typedef struct gpr_memq_block_t
{
   gpr_list_t free_q;
   char_t    *base_addr;
   uint32_t   total_units;
   uint32_t   unit_size;
   uint32_t   metadata_size;
   uint32_t  *unique_metadata_ids;
   uint32_t  *unique_metadata_counts;
} gpr_memq_block_t;

typedef struct gpr_memq_entry_t
{
   gpr_list_node_t link; /* Must be first */
} gpr_memq_entry_t;

#define GPR_MEMQ_BYTES_PER_METADATA_ITEM_V (sizeof(int32_t))
#define GPR_MEMQ_UNIT_OVERHEAD_V (sizeof(gpr_memq_entry_t))

/* Memory Queue Prototypes */
GPR_EXTERNAL int gpr_memq_init(gpr_memq_block_t        *block,
                               char_t                  *heap_base,
                               uint32_t                 heap_size,
                               uint32_t                 unit_size,
                               uint32_t                 metadata_size,
                               gpr_memq_lock_enter_fn_t lock_fn,
                               gpr_memq_lock_leave_fn_t unlock_fn,
                               gpr_heap_index_t         heap_index);

GPR_EXTERNAL void gpr_memq_deinit(gpr_memq_block_t *block, gpr_heap_index_t heap_index);

GPR_EXTERNAL void *gpr_memq_alloc(gpr_memq_block_t *block);

GPR_EXTERNAL void gpr_memq_free(gpr_memq_block_t *block, void *mem_ptr);

GPR_EXTERNAL uint32_t gpr_memq_node_set_metadata(gpr_memq_block_t *block, void *mem_ptr, uint32_t index, int32_t value);

GPR_EXTERNAL uint32_t gpr_memq_node_get_metadata(gpr_memq_block_t *block,
                                                 void             *mem_ptr,
                                                 uint32_t          index,
                                                 int32_t          *metadata);

#endif /* __GPR_MEMQ_H__ */
