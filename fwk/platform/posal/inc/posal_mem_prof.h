#ifndef POSAL_MEM_PROF_H
#define POSAL_MEM_PROF_H
/**
 * \file posal_mem_prof.h
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "ar_error_codes.h"
#include "posal_types.h"
#include "spf_hashtable.h"
#include "posal_memory.h"
#include "posal_mutex.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/* -----------------------------------------------------------------------
** Macro definitions
** ----------------------------------------------------------------------- */

/* -----------------------------------------------------------------------
** Structure definitions
** ----------------------------------------------------------------------- */
/** Structure to mark memory for tracking profiling (appended at the end of the momory)*/
typedef struct posal_mem_prof_marker_t
{
   POSAL_HEAP_ID heap_id;
   /**< Heap ID of the memory allocated */

   uint32_t magic_number;
   /**< Magic number to verify mem tracking */
}posal_mem_prof_marker_t;

/**< Hash node structure to hold heap-id to memory count mapping */
typedef struct posal_mem_prof_node_t
{
   // Need to be first element
   spf_hash_node_t hash_node;
   /**< Hash node */

   POSAL_HEAP_ID heap_id;
   /**< Key - heap id is used as a key to hashnode */

   uint32_t mem_count;
   /**< Value - Count of memory allocations in Bytes */

} posal_mem_prof_node_t;

/**< Enum to indicate whether memory profiling started or not */
typedef enum posal_mem_prof_state_t { POSAL_MEM_PROF_STOPPED = 0, POSAL_MEM_PROF_STARTED = 1 } posal_mem_prof_state_t;

/**< Posal memory profiling main structure */
typedef struct posal_mem_prof_t
{
   spf_hashtable_t mem_ht;
   /**< Hash table to hold heap id to memory count mapping */

   POSAL_HEAP_ID heap_id;
   /**< Heap id to be used by posal memory profiler */

   posal_mutex_t prof_mutex;
   /**< Mutex to be used by posal memory profiler */

   posal_mem_prof_state_t mem_prof_status;
   /**< Flag to indicate whether profiling started or not */
} posal_mem_prof_t;

/* -----------------------------------------------------------------------
** Function declaration
** ----------------------------------------------------------------------- */
/**
  Initializes posal memory profiler, creates mutex.

  @param[in] heapId   ID of the heap from which to allocate memory.

  @return
  Result.

  @dependencies
  None
*/
ar_result_t posal_mem_prof_init(POSAL_HEAP_ID heap_id);

/**
  Starts posal memory profiling, creates hashtable needed to store the heapid to
  mem count mapping.

  @param[in] None

  @return
  Result.

  @dependencies
  None
*/
ar_result_t posal_mem_prof_start();

/**
  Stops posal memory profiling, destroys hashtable needed to store the heapid to
  memory count mapping.

  @param[in] None

  @return
  Result.

  @dependencies
  None
*/
ar_result_t posal_mem_prof_stop();

/**
  Deinits posal memory profiling, destroys profiling mutex.

  @param[in] None

  @return
  Result.

  @dependencies
  None
*/
void posal_mem_prof_deinit();

/**
  Extracts heap id from original heap id, updates bytes required.

  @param[in] orig_heap_id   Heap id sent by the client.
  @param[in] heap_id_ptr    Pointer to heap id which stores extracted from orig_heap_id.
  @param[in] heap_id_ptr    Pointer to bytes to be allocated, if profiling has started, this will be + sizeof(uint64_t)

  @return
  None.

  @dependencies
  None
*/
void posal_mem_prof_pre_process_malloc(POSAL_HEAP_ID orig_heap_id, POSAL_HEAP_ID *heap_id_ptr, uint32_t *bytes_ptr);

/**
  Updates heap id and magic number at the tail of the allocation, updates statistics if profiling is enabled.

  @param[in] ptr              Pointer to the newly allocated memory.
  @param[in] orig_heap_id     Heap id sent by the client.
  @param[in] is_mem_tracked   Boolean to indicate of memory was tracked while allocation.

  @return
  None.

  @dependencies
  None
*/
void posal_mem_prof_post_process_malloc(void *ptr, POSAL_HEAP_ID orig_heap_id, bool_t is_mem_tracked);

/**
  Extracts heapid and mem size from the ptr, updates statistics.

  @param[in] ptr       Pointer to the newly allocated memory.

  @return
  None.

  @dependencies
  None
*/
void posal_mem_prof_process_free(void *ptr);

/**
  Updates the mem usage query asked by a client if the statistics exists.

  @param[in] heap_id         Heap id of the query.
  @param[in] mem_usage_ptr   Pointer to which query update needs to be done.

  @return
  None.

  @dependencies
  None
*/
void posal_mem_prof_query(POSAL_HEAP_ID heap_id, uint32_t *mem_usage_ptr);

/**
  Use to get the memory size from a pointer.

  @param[in] ptr        Pointer of the allocated memory.
  @param[in] heap_id    Heap id of the query.


  @return
  Block size of the memory

  @dependencies
  None
*/
uint32_t posal_mem_prof_get_mem_size(void *ptr, POSAL_HEAP_ID heap_id);
#ifdef __cplusplus
}
#endif //__cplusplus

#endif // POSAL_MEM_PROF_H
