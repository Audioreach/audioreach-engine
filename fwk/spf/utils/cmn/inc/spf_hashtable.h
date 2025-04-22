#ifndef HASHTABLE_H
#define HASHTABLE_H
/**
 * \file spf_hashtable.h
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_error_codes.h"
#include "posal_types.h"
#include "posal_memory.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/* -----------------------------------------------------------------------
** Structure definitions
** ----------------------------------------------------------------------- */

/** spf Hash node */
typedef struct spf_hash_node_t
{
   struct spf_hash_node_t *next_ptr;
   /** Pointer to the next node */

   uint32_t key_size;
   /** Size of the key */

   const void *key_ptr;
   /** Pointer to the key */
} spf_hash_node_t;

/** Function ptr to the using which hash node should be freed */
typedef void node_free_f(void *free_context_ptr, spf_hash_node_t *node_ptr);

/** spf hashtable structure */
typedef struct spf_hashtable_t
{
   spf_hash_node_t **table_ptr;
   /**< Array of the hash table ptrs. */

   POSAL_HEAP_ID heap_id;
   /**< Heap ID hashtable uses for memory allocations. */

   uint32_t table_size;
   /**< Size of the hashtable. */

   uint32_t mask_size;
   /**< Mask size. */

   uint32_t resize_factor;
   /**< Factor by which hashtable should grow. */

   node_free_f *free_fptr;
   /**< Function ptr to free the hash node. */

   void *free_context_ptr;
   /**< Context ptr which needs to be sent during freeing hash node. */

   uint32_t num_nodes;
   /**< Number of total nodes in hash, including dups */

   uint32_t num_items;
   /** Number of items hashed */
} spf_hashtable_t;

/* -----------------------------------------------------------------------
** Function declaration
** ----------------------------------------------------------------------- */
/**
  Initializes hashtable with given specifications.

  @param[in] ht_ptr         Pointer to the hashtable.
  @param[in] heapId         HeapID using which hashtable should  allocate.
  @param[in] table_size     Size of the hashtable.
  @param[in] resize_factor  Factor by which hashtable should grow.
  @param[in] free_fptr      Free function pointer for freeing hash nodes.
  @param[in] context_ptr    Context pointer to be returned during free.

  @return
  Result.

  @dependencies
  None
*/
ar_result_t spf_hashtable_init(spf_hashtable_t *ht_ptr,
                               POSAL_HEAP_ID    heap_id,
                               uint32_t         table_size,
                               uint32_t         resize_factor,
                               node_free_f *    free_fptr,
                               void *           context_ptr);

/**
  Inserts a hash node from hashtable. Hash node should be allocated beforehand

  @param[in] ht_ptr    Pointer to the hashtable.
  @param[in] node_ptr  Pointer to the hash node.

  @return
  Result.

  @dependencies
  None
*/
ar_result_t spf_hashtable_insert(spf_hashtable_t *ht_ptr, spf_hash_node_t *node_ptr);

/**
  Finds the hashtable node.

  @param[in] ht_ptr      Pointer to the hashtable.
  @param[in] key_ptr     Pointer to the key.
  @param[in] key_size    Size of key.

  @return
  Pointer to the hash node.

  @dependencies
  None
*/
spf_hash_node_t *spf_hashtable_find(spf_hashtable_t *ht_ptr, const void *key_ptr, uint32_t key_size);

/**
  Removes and frees all the hash nodes.

  @param[in] ht_ptr     Pointer to the hashtable.

  @return
  Result.

  @dependencies
  None
*/
ar_result_t spf_hashtable_remove_all(spf_hashtable_t *ht_ptr);

/**
  Removes a hash node from hashtable and frees the node.

  @param[in] ht_ptr     Pointer to the hashtable.
  @param[in] key_ptr    Pointer to the key.
  @param[in] key_size   Size of key.
  @param[in] node_ptr   Pointer to the hash node.

  @return
  Result.

  @dependencies
  None
*/
ar_result_t spf_hashtable_remove(spf_hashtable_t *ht_ptr,
                                 const void *     key_ptr,
                                 uint32_t         key_size,
                                 spf_hash_node_t *node_ptr);

/**
  De-inits hashtable

  @param[in] ht_ptr    Pointer to the hashtable.

  @return
  None.

  @dependencies
  None
*/
void spf_hashtable_deinit(spf_hashtable_t *ht_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // HASHTABLE_H
