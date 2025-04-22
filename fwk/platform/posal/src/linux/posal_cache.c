/**
 * \file posal_cache.c
 * \brief
 *  	This file contains a utility for cache operations.
 *
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
ar_result_t posal_cache_flush_invalidate_v2(posal_mem_addr_t virt_addr_ptr, uint32_t mem_size)
{
   uint64_t * virt_addr = (uint64_t *)virt_addr_ptr;

   return AR_EOK;
}
