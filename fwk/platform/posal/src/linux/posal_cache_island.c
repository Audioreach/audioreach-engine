/**
 * \file posal_cache_island.c
 * \brief
 *  This file contains a utility for cache operations.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */


/* ----------------------------------------------------------------------------
 * Include Files
 * ------------------------------------------------------------------------- */
#include "posal_cache.h"

/* ----------------------------------------------------------------------------
 * Global Declarations/Definitions
 * ------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------
 * Function Definitions
 * ------------------------------------------------------------------------- */
ar_result_t posal_cache_invalidate(uint32_t virt_addr, uint32_t mem_size)
{
   return AR_EOK;
}

ar_result_t posal_cache_invalidate_v2(void *virt_addr_ptr, uint32_t mem_size)
{
   uint64_t *virt_addr = (uint64_t *)virt_addr_ptr;

   return AR_EOK;
}

/**
  Flushes the cache (data) of the specified memory region.

  @param[in] virt_addr  Starting virtual address.
  @param[in] mem_size   Size of the region to be flushed.

  @return
  ar_result_t error code (refer to @xhyperref{Q4,[Q4]}).

  @dependencies
  The client object must have been registered and the corresponding memory
  mapped before calling this function.
 */
ar_result_t posal_cache_flush(uint32_t virt_addr, uint32_t mem_size)
{
   return AR_EOK;
}

ar_result_t posal_cache_flush_v2(posal_mem_addr_t virt_addr_ptr, uint32_t mem_size)
{
   uint64_t *virt_addr = (uint64_t *)virt_addr_ptr;

   return AR_EOK;
}
