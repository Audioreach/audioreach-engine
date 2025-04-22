/**
 * \file posal_cache.h
 * \brief
 *  	 This file contains utilities for cache operations
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_CACHE_H
#define POSAL_CACHE_H

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "posal_types.h"
#include "ar_error_codes.h"

typedef void *posal_mem_addr_t;

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/** @addtogroup posal_cache
@{ */

/**
  Flushes the cache (data) of the specified memory region.
  @param[in] virt_addr  Starting virtual address.
  @param[in] mem_size   Size of the region to flush.

  @return
  0 -- Success
  @par
  Nonzero -- Failure
*/
ar_result_t posal_cache_flush( uint32_t virt_addr, uint32_t mem_size);

/**
  Flushes the cache (data) of the specified memory region.
  @param[in] virt_addr  Starting virtual address.
  @param[in] mem_size   Size of the region to flush.

  @return
  0 -- Success
  @par
  Nonzero -- Failure
*/
ar_result_t posal_cache_flush_v2(posal_mem_addr_t virt_addr, uint32_t mem_size);

/**
  Invalidates the cache (data) of the specified memory region.
  @param[in]  virt_addr  Starting virtual address.
  @param[in]  mem_size   Size of the region to invalidate.

  @return
  0 -- Success
  @par
  Nonzero -- Failure
*/
ar_result_t posal_cache_invalidate(uint32_t virt_addr, uint32_t mem_size);

/**
  Invalidates the cache (data) of the specified memory region.
  @param[in]  virt_addr  Starting virtual address.
  @param[in]  mem_size   Size of the region to invalidate.

  @return
  0 -- Success
  @par
  Nonzero -- Failure
*/
ar_result_t posal_cache_invalidate_v2(posal_mem_addr_t virt_addr, uint32_t mem_size);

/**
  Flushes and invalidates the cache (data) of the specified memory region.
  @param[in] virt_addr  Starting virtual address.
  @param[in] mem_size   Size of the region to flush.

  @return
  0 -- Success
  @par
  Nonzero -- Failure
*/
ar_result_t posal_cache_flush_invalidate( uint32_t virt_addr, uint32_t mem_size);

/**
  Flushes and invalidates the cache (data) of the specified memory region.
  @param[in] virt_addr  Starting virtual address.
  @param[in] mem_size   Size of the region to flush.

  @return
  0 -- Success
  @par
  Nonzero -- Failure
*/
ar_result_t posal_cache_flush_invalidate_v2(posal_mem_addr_t virt_addr, uint32_t mem_size);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef POSAL_CACHE_H

