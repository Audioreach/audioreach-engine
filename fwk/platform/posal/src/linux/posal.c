/**
 * \file posal.c
 * \brief
 *     This file contains the global state structure for the posal environment.
 *     This state includes system-wide information such as number of active
 *     threads, malloc counters, etc.
 *
 * \copyright
 *    Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *    SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "posal.h"
#include "posal_internal.h"
#include "posal_globalstate.h"
#include "posal_mem_prof.h"
#include "posal_power_mgr.h"

/*--------------------------------------------------------------*/
/* Macro definitions                                            */
/* -------------------------------------------------------------*/

/* -----------------------------------------------------------------------
** Constant / Define Declarations
** ----------------------------------------------------------------------- */


/* =======================================================================
**                          Function Definitions
** ======================================================================= */

//***************************************************************************
// Global state structure for resource monitoring & debugging
//***************************************************************************

posal_globalstate_t posal_globalstate;


void posal_init(void)
{

   if (posal_globalstate.is_global_init_done)
   {
      AR_MSG(DBG_HIGH_PRIO, "POSAL is already initialized, returning.");
      return;
   }

   AR_MSG(DBG_HIGH_PRIO, "Initializing POSAL");

   posal_memory_stats_init();

   // set all fields to 0.
   posal_globalstate.nSimulatedMallocFailCount = -1;
   posal_atomic_word_create(&posal_globalstate.nMsgQs, POSAL_HEAP_DEFAULT);
   posal_atomic_word_create(&posal_globalstate.nMemRegions, POSAL_HEAP_DEFAULT);

   posal_atomic_set(posal_globalstate.nMsgQs, 0);
   posal_atomic_set(posal_globalstate.nMemRegions, 0);

   // Initialize global memory map client list.
   posal_memorymap_global_init();

   /* Create posal global mutex  */
   posal_mutex_create(&posal_globalstate.mutex, POSAL_HEAP_DEFAULT);

   if (AR_EOK != posal_mem_prof_init(POSAL_HEAP_DEFAULT))
   {
      AR_MSG(DBG_ERROR_PRIO, "FAILED to init posal mem prof");
   }

   /* Initialise posal power manager */
   posal_power_mgr_init();

   // if Q6_TCM is not available then malloc ensures we fallback to other island heaps.
   posal_queue_pool_setup(MODIFY_HEAP_ID_FOR_FWK_ALLOC_FOR_MEM_TRACKING(spf_mem_island_heap_id),
                          LPI_QUEUE_BUF_POOL_NUM_ARRAYS,
                          LPI_QUEUE_BUF_POOL_NODES_PER_ARR);
   posal_queue_pool_setup(POSAL_HEAP_DEFAULT, REGULAR_QUEUE_BUF_POOL_NUM_ARRAYS, REGULAR_QUEUE_BUF_POOL_NODES_PER_ARR);

   posal_globalstate.is_global_init_done = TRUE;

}

void posal_deinit(void)
{

}
