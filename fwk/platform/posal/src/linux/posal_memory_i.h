/**
 *  \file posal_memory_i.h
 * \brief
 *      Internal definitions
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_MEMORY_I_H
#define POSAL_MEMORY_I_H
/* ----------------------------------------------------------------------------
 * Include Files
 * ------------------------------------------------------------------------- */
#include "ar_error_codes.h"
#include "posal_types.h"
#include "posal_memory.h"


/* ----------------------------------------------------------------------------
 * Global Declarations/Definitions
 * ------------------------------------------------------------------------- */
/* posal heap manager heap table structure */
typedef struct posal_heap_table_t
{
   bool_t         used_flag;          /* Heap table entry status (used=1, unused=0). */
   bool_t         dynamic_heap;       /* If heap is created dynamically */
   bool_t         is_phys_addr_range; /* If heap range is physical address flag, else it is virtual */
   uint64_t       start_addr;         /* Start address of the heap. */
   uint64_t       end_addr;           /* End address of the heap. */
} posal_heap_table_t;

/* -------------------------------------------------------------------------
 * Function Declarations
 * ------------------------------------------------------------------------- */
bool_t posal_check_if_addr_within_heap_idx_range(uint32_t heap_table_idx, void *target_addr);

#endif // POSAL_BUFMGR_I_H