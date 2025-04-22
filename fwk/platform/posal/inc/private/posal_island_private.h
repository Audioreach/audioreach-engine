/**
 * \file posal_island_private.h
 * \brief
 *  	 This file contains private (not shipped) declarations
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_ISLAND_PRIVATE_H
#define POSAL_ISLAND_PRIVATE_H

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#include "posal_island.h"

typedef enum
{
   POSAL_ISLAND_HEAP_PRIVATE_TCM = POSAL_ISLAND_HEAP_NUM_SUPPORTED + 1,
   POSAL_ISLAND_HEAP_MAX_NUM_SUPPORTED = POSAL_ISLAND_HEAP_PRIVATE_TCM
} posal_private_island_heap_t;

/* -----------------------------------------------------------------------
** Private function and macro declarations and definitions
** ----------------------------------------------------------------------- */

#define POSAL_ISLAND_HEAP_PRIVATE_TCM_KEY ((posal_island_heap_t)0x57)

POSAL_HEAP_ID posal_private_get_island_heap_id_v2(uint32_t island_heap_type);

posal_mem_t posal_private_get_mem_type_from_heap_type(uint32_t island_heap_type);

static inline bool_t posal_is_private_tcm_heap_used(uint32_t heap_id)
{
   POSAL_HEAP_ID private_tcm_heap_id = posal_get_island_heap_id_v2(POSAL_ISLAND_HEAP_PRIVATE_TCM_KEY);
   return (GET_ACTUAL_HEAP_ID(heap_id) == private_tcm_heap_id);
}

#ifdef __cplusplus
}
#endif //__cplusplus
#endif // #ifndef POSAL_ISLAND_H