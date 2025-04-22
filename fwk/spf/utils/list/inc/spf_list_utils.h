#ifndef _SPF_LIST_UTILS_H__
#define _SPF_LIST_UTILS_H__

/**
 * \file spf_list_utils.h
 * \brief
 *     This file contains private declerations for Graphite utility functions
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "posal.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

//#define DEBUG_LIST_UTIL

/* Utility macros */
#define LIST_ADVANCE(x) (x = (x)->next_ptr)
#define LIST_RETREAT(x) (x = (x)->prev_ptr)

/*------------------------------------------------------------------------------
*  Type definitions
*----------------------------------------------------------------------------*/

/* Structure definition for graph node */
typedef struct spf_list_node_t spf_list_node_t;
struct spf_list_node_t
{
   void *obj_ptr;
   /**< pointer to the instance this node refers to.
        This is used to implement generic type independent
        list utilities*/

   spf_list_node_t *next_ptr;
   /**< pointer to the next node*/

   spf_list_node_t *prev_ptr;
   /**< pointer to the next node*/
};

/*------------------------------------------------------------------------------
 *  Function Prototypes
 *----------------------------------------------------------------------------*/
/*
  Initialize the buffer pool with one list of spf_list_node_ts.

  param[in] heap_id: heap id for memory allocation

  return: none
*/
void spf_list_buf_pool_init(POSAL_HEAP_ID heap_id, uint32_t num_arrays, uint16_t nodes_per_arr);

/*
  Destroys the buffer pool (all allocated lists).

  param[in]: none

  return: none
*/
void spf_list_buf_pool_deinit(POSAL_HEAP_ID heap_id);

/*
  Destroys the buffer pool (all allocated lists from list idx to max).

  param[in]: list_idx

  return: none
*/
void spf_list_buf_pool_reset(POSAL_HEAP_ID heap_id);

/*
  Returns the number of created (malloced) lists

  param[in]: none

  return: number of created (malloced) lists
*/
// uint32_t spf_list_get_num_created_lists(void);

/*
  Profile the memeory usage of the spf list pool

  param[out]: bytes_used_ptr
  param[out]: bytes_allocated_ptr

  return: AR_EOK on success, otherwise AR_EFAILED
*/
ar_result_t spf_list_pool_profile_memory(uint32_t* bytes_used_ptr, uint32_t* bytes_allocated_ptr);


/****************************************************** insertion
 * ******************************************************/
/*
  Insert a node into the head of the list. O(1).

  param[in] head_ptr: pointer to the head of module list
  param[in] obj_ptr: pointer to the instance that needs to be inserted
  param[in] heap_id: heap id for memory allocation
  param[in] use_pool: if use gk list pool for node memory then set to TRUE. If set to FALSE, malloc is used.

  return: none
*/
ar_result_t spf_list_insert_head(spf_list_node_t **head_ptr, void *obj_ptr, POSAL_HEAP_ID heap_id, bool_t use_pool);

/*
  Insert a node into the tail of the list. O(N).

  param[in] head_ptr: pointer to the head of module list
  param[in] obj_ptr: pointer to the instance that needs to be inserted
  param[in] heap_id: heap id for memory allocation
  param[in] use_pool: if use gk list pool for node memory then set to TRUE. If set to FALSE, malloc is used.

  return: none
*/
ar_result_t spf_list_insert_tail(spf_list_node_t **head_ptr, void *obj_ptr, POSAL_HEAP_ID heap_id, bool_t use_pool);

/*
  Cretes a node and inserts it before a given node (updates head if needed)

  param[in/out] head_pptr: pointer to the head of module list
  param[in] obj_ptr: pointer to the instance that needs to be inserted
  param[in] node_ptr: pointer to the node before which the new node should be inserted

  return: none
*/
ar_result_t spf_list_create_and_insert_before_node(spf_list_node_t **head_ptr,
                                                   void *            obj_ptr,
                                                   spf_list_node_t * node_ptr,
                                                   POSAL_HEAP_ID     heap_id,
                                                   bool_t            use_pool);
/*
  Insert the node before a given node (updates head if needed)

  param[in/out] head_pptr  : pointer to the head of module list
  param[in] new_node_ptr   : pointer to the instance that needs to be inserted
  param[in] node_ptr       : pointer to the node before which the new node should be inserted

  return: none
*/
ar_result_t spf_list_insert_before_node(spf_list_node_t **head_ptr,
                                        spf_list_node_t * new_node_ptr,
                                        spf_list_node_t * node_ptr);

/*
  Attaches a node into the tail of the list. O(N).

  param[in] head_ptr: pointer to the head of module list
  param[in] node_ptr: pointer to the list to be attached

  return: none
*/
ar_result_t spf_list_attach_at_tail(spf_list_node_t **head_ptr, spf_list_node_t *node_ptr);

/*
  Add obj to the list. If the obj is already present, skip adding to
  the list.

  param[in] head_ptr: pointer to the head of module list
  param[in] obj_ptr: pointer to the instance that needs to be deleted
  param[in] heap_id: heap ID for allocating the list node
  param[out] node_added: TRUE if node is added to the list, else FALSE
  param[in] use_pool: if use gk list pool for node memory then set to TRUE. If set to FALSE, malloc is used.

  return: MM Error code
*/
ar_result_t spf_list_search_and_add_obj(spf_list_node_t **head_ptr,
                                        void *            obj_ptr,
                                        bool_t *          node_added,
                                        POSAL_HEAP_ID     heap_id,
                                        bool_t            use_pool);

/*
  Adds the node to the top of the list

  param[in] head_ptr: pointer to the head of module list
  param[in] node_ptr: pointer to the node that needs to be added

  return: none
*/
ar_result_t spf_list_push_node(spf_list_node_t **head_ptr, spf_list_node_t *node_ptr);

/****************************************************** Query ******************************************************/
/*
  Check if the node for the particular instance is present in the list

  param[in] head_ptr: pointer to the head of module list
  param[in] obj_ptr: pointer to the instance that needs to be searched/deleted

  return: TRUE if particular instance present, else false
*/
bool_t spf_list_contains_node(spf_list_node_t **head_ptr, void *obj_ptr);

/*
   Do not use this function. Use spf_list_merge_lists.
*/
ar_result_t spf_list_merge_lists_internal(spf_list_node_t **dest_list_ptr, spf_list_node_t **src_list_ptr);

/*
  parse the given list and return the node with the given MID/IID values

  param[in] dest_list_ptr: head of the list to be merged into
  param[in] src_list_ptr: head of the list that needs to be merged into destination list

  return: AR_EOK on success, error code on failure
*/
static inline ar_result_t spf_list_merge_lists(spf_list_node_t **dest_list_ptr, spf_list_node_t **src_list_ptr)
{
   if ((NULL == dest_list_ptr) || (NULL == src_list_ptr))
   {
      return AR_EFAILED;
   }
   return spf_list_merge_lists_internal(dest_list_ptr, src_list_ptr);
}

/*
  Find the list node associated with the passed in object. Finding the object in the list
  is based on pointer comparison. Iterates through the list until pointers are equal.

  param[in] head_ptr            : pointer to the head of the list
  param[in] obj_ptr             : pointer to the object to be found in the list
  param[out] obj_list_node_pptr : returns the list node associated with the passed in object.
                                  NULL if not found.

  return: none
*/
ar_result_t spf_list_find_list_node(spf_list_node_t *head_ptr, void *obj_ptr, spf_list_node_t **obj_list_node_pptr);

ar_result_t spf_list_get_tail_node(spf_list_node_t *head_ptr, spf_list_node_t **tail_pptr);

//function to count the elements in a spf list.
uint32_t spf_list_count_elements(spf_list_node_t *head_ptr);

/****************************************************** deletion ******************************************************/
/*
  Delete the node for the particular instance from the list

  param[in] head_ptr: pointer to the head of module list
  param[in] obj_ptr: pointer to the instance that needs to be searched/deleted
  param[in] pool_used     : TRUE if gk list pool was used for allocating memory for list nodes.
                             FALSE if malloc was used.
  return: TRUE if particular instance present, else false
*/
bool_t spf_list_find_delete_node(spf_list_node_t **head_ptr, void *obj_ptr, bool_t pool_used);

/*
  pops the head of the list. Returns the instance pointer for the first node
  and deletes the head node.

  param[in] head_ptr: pointer to the head of module list
  param[in] pool_used     : TRUE if gk list pool was used for allocating memory for list nodes.
                             FALSE if malloc was used.
  return: instance pointed of the node at the head
*/
void *spf_list_pop_head(spf_list_node_t **head_ptr, bool_t pool_used);

/*
  delete the complete list, specified by the head pointer, instance memories are not
  freed up

  param[in] head_ptr: head of the list to be deleted
  param[in] pool_used     : TRUE if gk list pool was used for allocating memory for list nodes.
                             FALSE if malloc was used.
  return: AR_EOK on success, error code on failure
*/
ar_result_t spf_list_delete_list(spf_list_node_t **head_ptr, bool_t pool_used);

/*
  delete the complete list and free up all the objects

  param[in] head_ptr: head of the list to be deleted
  param[in] pool_used     : TRUE if gk list pool was used for allocating memory for list nodes.
                             FALSE if malloc was used.
  return: AR_EOK on success, error code on failure
*/
ar_result_t spf_list_delete_list_and_free_objs(spf_list_node_t **head_pptr, bool_t pool_used);

/*
 * Deletes the node pointed to by node_ptr, and updates prev_node_ptr with a pointer to the next node
 * If the last node is deleted, updates head_node_pptr to NULL
 *
 *   param[in] pool_used     : TRUE if gk list pool was used for allocating memory for list nodes.
                             FALSE if malloc was used.
 */
ar_result_t spf_list_delete_node_and_free_obj(spf_list_node_t **node_pptr,
                                              spf_list_node_t **head_node_pptr,
                                              bool_t            pool_used);

/**
 *   Deletes the current node, and updates it to point to the next node
 *
 *   param[in] pool_used     : TRUE if gk list pool was used for allocating memory for list nodes.
                             FALSE if malloc was used.
 */
ar_result_t spf_list_delete_node(spf_list_node_t **node_pptr, bool_t pool_used);

/*
  Removes node from the list, updates neighboring nodes properly (head ptr needs to be taken care outside)

  param[in] node_ptr      : pointer to the node that needs to be removed from the list
  param[in] pool_used     : TRUE if gk list pool was used for allocating memory for list nodes.
                             FALSE if malloc was used.

  return: none
*/
ar_result_t spf_list_remove_node_from_list(spf_list_node_t *node_ptr, bool_t pool_used);

/*
  Removes node from the list, updates neighboring nodes properly (head ptr needs to be taken care outside).
  Does not free node memory.

  param[in] node_ptr      : pointer to the node that needs to be removed from the list


  return: none
*/
ar_result_t spf_list_remove_node_from_list_only(spf_list_node_t *node_ptr);

/*
  node_ptr gets removed from src_pptr list and added to list_pptr.
*/
void spf_list_move_node_to_another_list(spf_list_node_t **list_pptr,
                                        spf_list_node_t * node_ptr,
                                        spf_list_node_t **src_head_pptr);

bool_t spf_list_node_is_addr_from_heap(void *ptr, POSAL_HEAP_ID heap_id);

ar_result_t spf_list_realloc_replace_node(spf_list_node_t **head_pptr,
                                          spf_list_node_t **node_pptr,
                                          void *            new_obj_ptr,
                                          bool_t            pool_used,
                                          bool_t            use_pool,
                                          POSAL_HEAP_ID     heap_id);

ar_result_t spf_list_delete_node_update_head(spf_list_node_t **node_pptr,
                                             spf_list_node_t **head_node_pptr,
                                             bool_t            pool_used);


#ifdef __cplusplus
}
#endif //__cplusplus
#endif /* _SPF_LIST_UTILS_H__ */
