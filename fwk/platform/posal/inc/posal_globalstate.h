/**
 * \file posal_globalstate.h
 * \brief 
 *  	 This file contains the global state structure for the posal environment.
 *  	 This state includes system-wide information such as the number
 *  	 of active threads and malloc counters
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_GLOBALSTATE_H
#define POSAL_GLOBALSTATE_H

/*=======================================================================
INCLUDE FILES FOR MODULE
========================================================================= */
#include "posal_atomic.h"
#include "posal_thread.h"
#include "posal_mutex.h"
#include "posal_queue.h"
#include "posal_memorymap.h"
#include "posal_heapmgr.h"
#include "posal_memory.h"
#include "posal_tgt_util.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/** @addtogroup posal_globalstate
@{ */

/* Forward declaration. */
typedef struct posal_globalstate_t posal_globalstate_t;

/* Forward declaration. */
typedef struct posal_memorymap_client_t posal_memorymap_client_t;

//#define DEBUG_POSAL_MEMORY_USING_TABLE 1

#ifdef DEBUG_POSAL_MEMORY_USING_TABLE

#define POSAL_MEM_TRACE_TABLE_SIZE 2048

typedef struct posal_mem_trace_t
{
   void              *ptr;          /** pointer for which malloc was done but not free*/
   uint64_t          timestamp_us;  /** timestamp at which above ptr was allocated*/
   int32_t           thread_id;     /** thread which allocated the above ptr */
   uint32_t          size;          /** size of allocation */
} posal_mem_trace_t;
#endif //DEBUG_POSAL_MEMORY_USING_TABLE

/** Memory usage statistics obtained during a test case run.
*/
typedef struct {
   uint32_t             num_mallocs;
   /**< Total number of memory allocations up to the current point in the
        test. */

   uint32_t             num_frees;
   /**< Total number of times memory is freed to the current point in the
        test. */

   uint32_t             curr_heap;
   /**< Current heap usage at the current point in the test. */

   uint32_t             peak_heap;
   /**< Peak heap usage up to the current point in the test.*/

#ifdef DEBUG_POSAL_MEMORY_USING_TABLE
   posal_mem_trace_t    ptr_trace[POSAL_MEM_TRACE_TABLE_SIZE];
   /**< stores malloc'ed pointers until freed*/
#endif

} posal_mem_stats_t;




//***************************************************************************
// Global State
//***************************************************************************

/** Global structure used to track resources, such as threads and queues. This
  structure is intended for use in such tasks as debugging and checking for
  leaks.
*/
struct posal_globalstate_t
{
	posal_mem_stats_t 		 		avs_stats[POSAL_HEAP_MGR_MAX_NUM_HEAPS + 1];
   /**< Heap statistics for Audio-Voice Subsystem (AVS) threads.

        This number comprises one default heap plus the
        #POSAL_HEAP_MGR_MAX_NUM_HEAPS non-default heap. */

   posal_mem_stats_t non_avs_stats;
   /**< Heap statistics for non-AVS threads. */

   volatile int32_t        nSimulatedMallocFailCount;
   /**< If the failure count > 0, counts memory allocations down to zero, and
        then simulates out-of-memory. This count is used for testing. */

   posal_atomic_word_t      nMsgQs;
   /**< Counter of queues to help generate unique names. */

   posal_atomic_word_t      nMemRegions;
   /**< Counter of the number of memory regions in a system. */

   posal_mutex_t            mutex;
   /**< Mutex for thread safety of this structure. */

   posal_memorymap_client_t *mem_map_client_list[POSAL_MEMORY_MAP_MAX_CLIENTS];
   /**< Linked list of memory map clients in the system. */

   uint32_t                 num_registered_memmap_clients;

   volatile uint32_t        bEnableQLogging;
   /**< Logs the commands going into queues and coming out of queues. */

   volatile uint32_t uSvcUpStatus;
   /**< Specifies whether the aDSP static services are up and ready. */

   bool_t is_global_init_done;
   /**< Flag to set if global init is done. */
};


/** Maintains a linked list of clients registered with posal_memorymap.
 */
struct posal_memorymap_client_t
{

    posal_memorymap_node_t   *pMemMapListNode;
    /**< List of memory map nodes for this client.*/

    posal_mutex_t             mClientMutex;
    /**< Mutex to access the list. */

    uint32_t 				  client_id;
    /**< Client ID.*/
};

/** Instance of the global state structure. */
extern posal_globalstate_t posal_globalstate;

/** @} */ /* end_addtogroup posal_globalstate */

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef POSAL_GLOBALSTATE_H
