/**
 * \file posal_heapmgr.h
 * \brief
 *  	 This file contains utilities for memory allocation and release. This
 *  	 file provides memory allocation functions and macros for both C and C++.
 *
 * \copyright
 *    Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *    SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_HEAPMGR_H
#define POSAL_HEAPMGR_H

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "posal_types.h"
#include "posal_memory.h"

#ifdef __cplusplus
#include <new>
extern "C" {
#endif //__cplusplus

/** @addtogroup posal_memory
@{ */

//***************************************************************************
// Heap features
//***************************************************************************

typedef uint32_t posal_heap_tcm_handle_t;

/**
  Initializes the heap manager for a specified heap.

  @datatypes
  #POSAL_HEAP_ID

  @param[in] heap_id_ptr     Pointer to the heap ID.
  @param[in] heap_start_ptr  Pointer to the start address of the heap.
  @param[in] heap_size       Size of the heap.
  @param[in] is_init_heap_needed    Set to TRUE for heap management doing within SPF.
                                    Set to FALSE if heap management is doing in non SPF code(like coretech)
                                    for eg,TCM heap mgr where heap managemnet done by sysdrivers.

  @return
  Status of the heap manager creation.

  @dependencies
  Before calling this function, the object must be created and initialized.
*/
ar_result_t posal_memory_heapmgr_create(POSAL_HEAP_ID *heap_id_ptr,
                                        void *         heap_start_ptr,
                                        uint32_t       heap_size,
                                        bool_t         is_init_heap_needed);

/**
  Initializes the heap manager for a specified heap.

  @datatypes
  #POSAL_HEAP_ID

  @param[in] heap_id_ptr     Pointer to the heap ID.
  @param[in] heap_start_ptr  Pointer to the start address of the heap.
  @param[in] heap_size       Size of the heap.
  @param[in] is_init_heap_needed    Set to TRUE for heap management doing within SPF.
                                    Set to FALSE if heap management is doing in non SPF code(like coretech)
                                    for eg,TCM heap mgr where heap managemnet done by sysdrivers.
  @param[in] heap_type       type of Heap being created (default/ddr, lpm, island)
  @param[out] tcm_handle_ptr ptr to tcm heap handle
  @param[in] tcm_name        name of tcm pool heap
  @param[in] tcm_name_len    length of tcm_name

  @return
  Status of the heap manager creation.

  @dependencies
  Before calling this function, the object must be created and initialized.
*/
ar_result_t posal_memory_heapmgr_create_v2(POSAL_HEAP_ID *          heap_id_ptr,
                                           void *                   heap_start_ptr,
                                           uint32_t                 heap_size,
                                           bool_t                   is_init_heap_needed,
                                           uint32_t                 heap_type,
                                           posal_heap_tcm_handle_t *tcm_handle_ptr,
                                           char *                   tcm_name,
                                           uint32_t                 tcm_name_len);

/**
  De-initializes the heap manager of a specified heap ID.

  @datatypes
  #POSAL_HEAP_ID

  @param[in] heap_id  ID of the heap.

  @return
  Status of the heap manager deletion.

  @dependencies
  Before calling this function, the object must be created and initialized.
  @newpage
*/
ar_result_t posal_memory_heapmgr_destroy(POSAL_HEAP_ID heap_id);

/** @} */ /* end_addtogroup posal_memory */

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef POSAL_HEAPMGR_H