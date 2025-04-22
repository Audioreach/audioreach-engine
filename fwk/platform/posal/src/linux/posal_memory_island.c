/**
 * \file posal_memory.c
 * \brief
 *  	This file contains a utility for memory allocation.
 *
 * \copyright
 *    Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *    SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* -------------------------------------------------------------------------
 * Include Files
 * ------------------------------------------------------------------------- */
#include "posal.h"
#include "posal_memory_i.h"

/* -------------------------------------------------------------------------
 * Global Declarations
 * ------------------------------------------------------------------------- */
/* Heap manager table*/
posal_heap_table_t posal_heap_table[POSAL_HEAP_MGR_MAX_NUM_HEAPS] = {};


/* -------------------------------------------------------------------------
 * Function Definitions
 * ------------------------------------------------------------------------- */
bool_t posal_check_addr_from_tcm_island_heap_mgr(void *ptr)
{
   return FALSE;
}