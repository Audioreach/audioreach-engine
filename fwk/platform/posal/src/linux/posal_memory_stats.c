/**
 * \file posal_memory_globalstate.c
 * \brief
 *  	This file contains a utility for memory allocation.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* ----------------------------------------------------------------------------
 * Include Files
 * ------------------------------------------------------------------------- */
#include "posal.h"
#include "posal_globalstate.h"
#include "posal_mem_prof.h"
#include "posal_memory_i.h"
#include "posal_internal.h"

/* ----------------------------------------------------------------------------
 * Global Declarations/Definitions
 * ------------------------------------------------------------------------- */
extern posal_heap_table_t posal_heap_table[POSAL_HEAP_MGR_MAX_NUM_HEAPS];

/* -------------------------------------------------------------------------
 * Function Definitions
 * ------------------------------------------------------------------------- */

void posal_memory_stats_init(void)
{
   return;
}

void posal_memory_stats_deinit(void)
{
   return;
}

void posal_memory_stats_update(void *ptr, uint32_t is_malloc, uint32_t bytes, POSAL_HEAP_ID origheapId)
{
#if defined(DEBUG_POSAL_MEMORY) || defined(HEAP_PROFILING)
   uint32_t un_bytes = posal_mem_prof_get_mem_size(ptr, origheapId);
   un_bytes          = un_bytes ? un_bytes : bytes; // if zero is returned by posal_mem_prof_get_mem_size

   int64_t threadid = posal_thread_get_curr_tid();
   if (IS_MALLOC == is_malloc)
   {
      AR_MSG(DBG_HIGH_PRIO,
             "POSAL_MALLOC_FREE_TRACER: Thread 0x%p mallocs %lu bytes, gets ptr 0x%p, mem_heap_id 0x%lX",
             threadid,
             un_bytes,
             ptr,
             origheapId); // higher bis in origheapId are to be ignored except for per-module profiling.
   }
   else
   {
      AR_MSG(DBG_HIGH_PRIO,
             "POSAL_MALLOC_FREE_TRACER: Thread 0x%p frees %lu bytes, from ptr 0x%p",
             threadid,
             un_bytes,
             ptr);
   }
#endif
   return;
}