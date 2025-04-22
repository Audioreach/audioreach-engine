/**
@file irm_list_utils.cpp

@brief Main file for Integrated Resource Monitor (IRM).

================================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/
// clang-format off
// clang-format on

#include "irm_i.h"

/*----------------------------------------------------------------------------------------------------------------------
 Finds a node in the list with same ID as the given ID.
 Returns NULL if it can't find any node with given ID
----------------------------------------------------------------------------------------------------------------------*/
spf_list_node_t *irm_find_node_from_id(spf_list_node_t *head_ptr, uint32_t id)
{
   spf_list_node_t *node_ptr = head_ptr;
   for (; NULL != node_ptr; node_ptr = node_ptr->next_ptr)
   {
      irm_node_obj_t *irm_node_obj_ptr = (irm_node_obj_t *)node_ptr->obj_ptr;
      if ((NULL != irm_node_obj_ptr) && (id == irm_node_obj_ptr->id))
      {
         break;
      }
   }
   return node_ptr;
}

/*----------------------------------------------------------------------------------------------------------------------
 Returns all the obj_ptrs to the buf pool
 Checks for Null before returning
----------------------------------------------------------------------------------------------------------------------*/
void irm_return_list_objs_to_pool(spf_list_node_t *head_ptr)
{
   for (spf_list_node_t *node_ptr = head_ptr; NULL != node_ptr; node_ptr = node_ptr->next_ptr)
   {
      if (NULL != node_ptr->obj_ptr)
      {
         posal_bufpool_return_node((void *)node_ptr->obj_ptr);
         node_ptr->obj_ptr = NULL;
      }
   }
}

/*----------------------------------------------------------------------------------------------------------------------
 1. Returns all the obj_pts to the buf pool.
 2. Deletes the entire list
----------------------------------------------------------------------------------------------------------------------*/
void irm_delete_list(spf_list_node_t **node_pptr)
{
   irm_return_list_objs_to_pool(*node_pptr);
   spf_list_delete_list(node_pptr, TRUE /*POOL_USED*/);
}

/*----------------------------------------------------------------------------------------------------------------------
 1. Deletes the given node from the list
 2. If the given node ptr is same as head ptr, updated the head ptr
----------------------------------------------------------------------------------------------------------------------*/
void irm_delete_node(spf_list_node_t **head_pptr, spf_list_node_t **node_pptr)
{
   if (NULL != (*node_pptr)->obj_ptr)
   {
      posal_bufpool_return_node((void *)((*node_pptr)->obj_ptr));
   }
   (*node_pptr)->obj_ptr = NULL;
   if (*head_pptr == *node_pptr)
   {
      spf_list_delete_node(head_pptr, TRUE /*pool used*/);
      *node_pptr = *head_pptr;
   }
   else
   {
      spf_list_delete_node(node_pptr, TRUE /*pool used*/);
   }
}

/*----------------------------------------------------------------------------------------------------------------------
 1. Gets the node obj from the buf pool
 2. memsets the buffer to 0s
----------------------------------------------------------------------------------------------------------------------*/
void irm_get_node_obj(irm_t *irm_ptr, irm_node_obj_t **node_obj_pptr)
{
   (*node_obj_pptr) = (irm_node_obj_t *)posal_bufpool_get_node(irm_ptr->core.irm_bufpool_handle);

   if (NULL != *node_obj_pptr)
   {
      memset(*node_obj_pptr, 0, sizeof(irm_node_obj_t));
   }
}

/*----------------------------------------------------------------------------------------------------------------------
 1. Checks if the a node with given ID is already present
 2. If present, it returns that node
 3. If it is not present,
    a. It gets nod obj from pool
    b. Finds the location to which the node needs to inserted. This is based on ascending order of the IDs
    c. Inserts the node to the list to appropriate location
----------------------------------------------------------------------------------------------------------------------*/
irm_node_obj_t *irm_check_insert_node(irm_t *irm_ptr, spf_list_node_t **head_pptr, uint32_t id)
{
   spf_list_node_t *node_ptr = *head_pptr;
   irm_node_obj_t *obj_ptr  = NULL;

   for (; NULL != node_ptr; node_ptr = node_ptr->next_ptr)
   {
      irm_node_obj_t *irm_node_obj_ptr = (irm_node_obj_t *)node_ptr->obj_ptr;
      if (NULL != irm_node_obj_ptr)
      {
         if ((irm_node_obj_ptr->id > id))
         {
            break;
         }
         else if (id == irm_node_obj_ptr->id)
         {
            return node_ptr->obj_ptr;
         }
      }
   }

   irm_get_node_obj(irm_ptr, &obj_ptr);
   if (NULL != obj_ptr)
   {
      obj_ptr->id            = id;
      obj_ptr->head_node_ptr = NULL;

      // if the head node is NULL or if all the IDs in the list are smaller than given ID
      if (NULL == node_ptr)
      {
         spf_list_insert_tail(head_pptr, obj_ptr, irm_ptr->heap_id, TRUE);
      }
      else
      {
         spf_list_create_and_insert_before_node(head_pptr, obj_ptr, node_ptr, irm_ptr->heap_id, TRUE);
      }
   }
   return obj_ptr;
}
/*--------------------------------------------------------------------------------------------------------------------*/
