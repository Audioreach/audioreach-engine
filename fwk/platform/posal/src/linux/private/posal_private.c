/**
 * \file posal_private.c
 * \brief
 *  	 This file contains private (not shipped) function defintions
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "posal.h"
#include "posal_island_private.h"
#include "posal_internal.h"

/* =======================================================================
**                          Function Definitions
** ======================================================================= */
POSAL_HEAP_ID posal_private_get_island_heap_id_v2(posal_island_heap_t heap_type)
{
   return POSAL_HEAP_DEFAULT;
}

void *tcm_dyn_heap_dyn_load_modules_alloc_memory_ummap2(void * addr,
                                                        size_t mem_size,
                                                        int    prot,
                                                        int    flags,
                                                        int    fd,
                                                        long   offset,
                                                        int    ctx,
                                                        int    desc_type)
{
   return NULL;
}

void posal_private_init()
{
}

void posal_private_deinit()
{
}

