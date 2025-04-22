/**
 * \file spf_hashtable.c
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spf_hashtable.h"
#include "posal_memory.h"

// 32 bit magic FNV prime and init values
#define FNV_32_PRIME 0x01000193
#define FNV_32_INIT 0x811C9DC5 // 2166136261

// assign 22 bits for the table index and 10 for the linkpos
#define TABLE_POS(x) (x >> 10)
#define LINK_POS(y) (y & 0x3FF)

#define HASH_ENUM_ST(x, y) (x << 10 | (y & 0x3FF))

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
static __inline uint32_t fnv_hash(const void *void_ptr, uint32_t len, uint32_t h)
{
   const uint8_t *byte_ptr = (uint8_t *)void_ptr;

   while (len)
   {
      h = (uint32_t)((uint64_t)h * FNV_32_PRIME);
      h ^= (uint32_t)*byte_ptr++;
      len--;
   }
   return h;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
static __inline int32_t hashtable_equalkey(const void *key_ptr1,
                                           uint32_t    key_size1,
                                           const void *key_ptr2,
                                           uint32_t    key_size2)
{
   if (key_size1 != key_size2)
   {
      return 0;
   }
   return (0 == memcmp(key_ptr1, key_ptr2, key_size1));
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
static spf_hash_node_t **hashtable_findpos(spf_hashtable_t *ht_ptr, const void *key_ptr, uint32_t key_size)
{
   spf_hash_node_t **node_pptr;
   uint32_t          nHash;
   uint32_t          nPos;

   nHash = fnv_hash(key_ptr, key_size, FNV_32_INIT);
   nPos  = nHash & ht_ptr->mask_size; // Only take enough bits as the table
                                      // size from the hash

   for (node_pptr = &ht_ptr->table_ptr[nPos];
        (*node_pptr) && !hashtable_equalkey((*node_pptr)->key_ptr, (*node_pptr)->key_size, key_ptr, key_size);
        node_pptr = &(*node_pptr)->next_ptr)
   {
   };

   return node_pptr;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
static ar_result_t hashtable_internalinsert(spf_hashtable_t *ht_ptr, spf_hash_node_t *node_ptr)
{
   spf_hash_node_t **node_pptr = hashtable_findpos(ht_ptr, node_ptr->key_ptr, node_ptr->key_size);
   if ((spf_hash_node_t *)0 != *node_pptr)
   {
      spf_hash_node_t *dup_ptr;
      // identical nodes not allowed, will result in non-terminating linked list
      for (dup_ptr = *node_pptr; 0 != dup_ptr; dup_ptr = dup_ptr->next_ptr)
      {
         if (node_ptr == dup_ptr)
         {
            return AR_EALREADY;
         }
      }

      node_ptr->next_ptr = (*node_pptr);
      *node_pptr         = node_ptr;
   }
   else
   {
      // add this new node at the beginning
      *node_pptr         = node_ptr;
      node_ptr->next_ptr = 0;

      ht_ptr->num_items++; // increment number of hashed keys
   }

   ht_ptr->num_nodes++; // increment number of total nodes

   return AR_EOK;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
static ar_result_t hashtable_grow(spf_hashtable_t *ht_ptr)
{
   uint32_t          new_table_size = ht_ptr->table_size * ht_ptr->resize_factor;
   uint32_t          old_table_size;
   spf_hash_node_t **new_table_pptr = NULL;
   spf_hash_node_t **old_table_pptr = NULL;
   ar_result_t       err            = AR_EOK;

   new_table_pptr =
      (spf_hash_node_t **)posal_memory_malloc(new_table_size * sizeof(spf_hash_node_t *), ht_ptr->heap_id);
   if (NULL == new_table_pptr)
   {
      err = AR_ENOMEMORY;
      return err;
   }

   memset(new_table_pptr, 0, new_table_size * sizeof(spf_hash_node_t *));
   old_table_pptr    = ht_ptr->table_ptr;
   ht_ptr->table_ptr = new_table_pptr;

   old_table_size     = ht_ptr->table_size;
   ht_ptr->table_size = new_table_size;
   ht_ptr->mask_size  = ht_ptr->table_size - 1;

   for (uint32_t i = 0; i < old_table_size; i++)
   {
      spf_hash_node_t *node_ptr = old_table_pptr[i]; // get list head

      // Reverse the linked list so that nodes with the same key are added in the order in which they were added to the
      // old hash table.
      spf_hash_node_t *current_node_ptr = node_ptr;
      spf_hash_node_t *prev_node_ptr    = NULL;
      while (current_node_ptr)
      {
         spf_hash_node_t *next_node = current_node_ptr->next_ptr;
         current_node_ptr->next_ptr = prev_node_ptr;
         prev_node_ptr              = current_node_ptr;
         current_node_ptr           = next_node;
      }
      node_ptr = prev_node_ptr;

      while (node_ptr) // re-insert all items in old head
      {
         spf_hash_node_t *node_to_insert_ptr = node_ptr;
         node_ptr                            = node_ptr->next_ptr; // insert whacks next_ptr
         (void)hashtable_internalinsert(ht_ptr, node_to_insert_ptr);
      }
   }

   posal_memory_free(old_table_pptr);

   return AR_EOK;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
ar_result_t spf_hashtable_remove_all(spf_hashtable_t *ht_ptr)
{
   for (uint32_t i = 0; i < ht_ptr->table_size; i++)
   {
      spf_hash_node_t *node = ht_ptr->table_ptr[i];

      while (node)
      {
         spf_hash_node_t *free_node = node;

         node = node->next_ptr;

         if (ht_ptr->free_fptr)
         {
            ht_ptr->free_fptr(ht_ptr->free_context_ptr, free_node);
         }
      }
      ht_ptr->table_ptr[i] = 0;
   }

   ht_ptr->num_items = 0;
   ht_ptr->num_nodes = 0;

   return AR_EOK;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
ar_result_t spf_hashtable_remove(spf_hashtable_t *ht_ptr,
                                 const void *     key_ptr,
                                 uint32_t         key_size,
                                 spf_hash_node_t *node_ptr)
{
   spf_hash_node_t **node_pptr;

   node_pptr = hashtable_findpos(ht_ptr, key_ptr, key_size);
   if (node_pptr && *node_pptr)
   {
      spf_hash_node_t *free_node = *node_pptr;
      spf_hash_node_t *next_ptr  = (*node_pptr)->next_ptr;

      // decrement number total nodes
      ht_ptr->num_nodes--;

      if (0 == node_ptr || node_ptr == free_node)
      {
         *node_pptr = next_ptr;
         // decrement num items in the table
         ht_ptr->num_items--;
      }
      else
      {
         // remove a specific dup
         spf_hash_node_t *free_nodeDup = free_node->next_ptr;

         while (free_nodeDup)
         {
            if (node_ptr == free_nodeDup)
            {
               free_node->next_ptr = free_nodeDup->next_ptr;
               free_node           = free_nodeDup;
               break;
            }
            free_node    = free_nodeDup;
            free_nodeDup = free_nodeDup->next_ptr;
         }
      }

      if (ht_ptr->free_fptr)
      {
         ht_ptr->free_fptr(ht_ptr->free_context_ptr, free_node);
      }

      return AR_EOK;
   }

   return AR_EFAILED;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
ar_result_t spf_hashtable_insert(spf_hashtable_t *ht_ptr, spf_hash_node_t *node_ptr)
{
   ar_result_t err    = AR_EOK;
   node_ptr->next_ptr = 0;
   err                = hashtable_internalinsert(ht_ptr, node_ptr);
   if (AR_EOK == err)
   {
      if (ht_ptr->num_items > ht_ptr->table_size * ht_ptr->resize_factor)
      {
         (void)hashtable_grow(ht_ptr);
      }
   }

   return err;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
spf_hash_node_t *spf_hashtable_find(spf_hashtable_t *ht_ptr, const void *key_ptr, uint32_t key_size)
{
   spf_hash_node_t **node_pptr = hashtable_findpos(ht_ptr, key_ptr, key_size);
   return *node_pptr;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
ar_result_t spf_hashtable_init(spf_hashtable_t *ht_ptr,
                               POSAL_HEAP_ID    heap_id,
                               uint32_t         table_size,
                               uint32_t         resize_factor,
                               node_free_f *    free_fptr,
                               void *           context_ptr)
{
   ar_result_t err      = AR_EOK;
   uint32_t    log_size = 0;

   memset(ht_ptr, 0, sizeof(spf_hashtable_t));

   ht_ptr->num_nodes        = 0;
   ht_ptr->num_items        = 0;
   ht_ptr->heap_id          = heap_id;
   ht_ptr->free_fptr        = free_fptr;
   ht_ptr->free_context_ptr = context_ptr;

   // make it a power of 2...
   // if its not a power of 2, shorten it to the closest previous 2th power
   while (0 != (table_size >>= 1))
   {
      log_size++;
   }

   table_size = (uint32_t)1 << log_size;
   if (ht_ptr->num_items || 0 == table_size)
   {
      return AR_EUNSUPPORTED;
   }

   if (0 >= resize_factor)
   {
      return AR_EUNSUPPORTED;
   }

   ht_ptr->resize_factor = resize_factor;

   if (table_size > ht_ptr->table_size)
   {
      ht_ptr->table_ptr =
         (spf_hash_node_t **)posal_memory_malloc(table_size * sizeof(spf_hash_node_t *), ht_ptr->heap_id);
      if (NULL != ht_ptr->table_ptr)
      {
         memset(ht_ptr->table_ptr, 0, table_size * sizeof(spf_hash_node_t *));
         ht_ptr->table_size = table_size;
         ht_ptr->mask_size  = table_size - 1;
      }
      else
      {
         err = AR_ENOMEMORY;
      }
   }

   return err;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
void spf_hashtable_deinit(spf_hashtable_t *ht_ptr)
{
   (void)spf_hashtable_remove_all(ht_ptr);
   if (ht_ptr->table_ptr)
   {
      posal_memory_free(ht_ptr->table_ptr);
      ht_ptr->table_ptr = NULL;
   }
}
