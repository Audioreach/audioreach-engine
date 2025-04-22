#ifndef _SPF_REF_COUNTER_H_
#define _SPF_REF_COUNTER_H_

/**
 * \file spf_ref_counter.h
 * \brief
 *     This file defines the api for a reference counting server.
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/** @weakgroup weakf_spf_ref_counter_overview
   The ref counting server is an api for reference counted memory allocation.
   This is useful for memory that needs to be shared across multiple contexts.
   One example of this is eos metadata, which can be duplicated by splitters,
   after which a single reference would be owned by multiple containers.

   Usage is as follows:
   1. Request allocation of a specified amount of memory using
      spf_ref_counter_create_ref(). A new memory address will be returned.

   2. Any client with access to an address created by this service can add a
      reference to that address using spf_ref_counter_add_ref().

   3. Any client with access to an address created by this service can remove a
      reference to that address using spf_ref_counter_remove_ref(). When the
      ref count reaches zero, the memory is deallocated.

      A client may wish to take actions when the ref count reaches zero. The
      client may pass in a callback function as an argument to
      spf_ref_counter_remove_ref(). This callback function will only be called
      when the ref count reaches zero. Note that the callback function will be
      called before the memory is deallocated. Note also that
      spf_ref_counter_remove_ref() is atomic, thus the callback function will be
      called while a mutex is acquired.
*/

/*-------------------------------------------------------------------------
Include Files
-------------------------------------------------------------------------*/

/* System */
#include "posal.h"

/* Audio */
#include "spf_utils.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/** @ingroup spf_ref_counter_datatypes
  Defines a function pointer signature for a callback function which is called
  during spf_ref_counter_remove_ref when the ref count reaches zero. The
  callback is called before the data_ptr is deallocated and is called while a
  mutex is acquired.

  @param[in] cb_context_ptr     Opaque pointer to the callback context.

  @dependencies
  None.
 */
typedef void (*spf_ref_counter_destroy_cb_func)(void *cb_context_ptr, uint32_t ref_count);

/*---------------------------------------------------------------------------
Function Declarations and Documentation
----------------------------------------------------------------------------*/

/** @ingroup spf_ref_counter_create_ref
  Allocates memory and creates a ref count entry for the new memory location.
  Returns the memory to the user. The user MUST NOT free the memory manually.
  Instead, the user must call spf_ref_counter_remove_ref() to free the memory.

  @datatypes

  @param[in]     data_size_bytes Size of memory to allocate.
  @param[in]     heap_id         Heap id to use for allocation.
  @param[out]    data_ptr_ptr    Double pointer to newly created memory.

  @return
  Indication of success or failure.

  @dependencies
  None.
 */
ar_result_t spf_ref_counter_create_ref(uint32_t data_size_bytes, POSAL_HEAP_ID heap_id, void **data_ptr_ptr);

/** @ingroup spf_ref_counter_add_ref
  Adds a reference to the passed in memory address.

  @datatypes

  @param[in]     data_ptr    Increases ref count of this address.

  @return
  Indication of success or failure.

  @dependencies
  None.
 */
ar_result_t spf_ref_counter_add_ref(void *data_ptr);

/** @ingroup spf_ref_counter_add_ref
  Removes a reference to the passed in memory address. If the ref count
  reaches zero, then calls the passed in callback function and deallocates
  the memory and its corresponding ref count entry.

  @datatypes
  spf_ref_counter_destroy_cb_func

  @param[in]     data_ptr         Decreases ref count of this address.
  @param[in]     destroy_cb_func  Calls this callback function when the ref
                                  count reaches zero.
  @param[in]     cb_context_ptr   Context passed into the callback function if
                                  the callback function is called.

  @return
  Indication of success or failure.

  @dependencies
  None.
 */
ar_result_t spf_ref_counter_remove_ref(void *                          data_ptr,
                                       spf_ref_counter_destroy_cb_func destroy_cb_func,
                                       void *                          cb_context_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef _SPF_REF_COUNTER_H_
