/**
 * \file posal_bufpool_i.h
 * \brief
 *    Header file for buffer pool functionality for small allocations.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_BUFPOOL_I_H
#define POSAL_BUFPOOL_I_H

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
/*--------------------------------------------------------------*/
/* Macro definitions                                            */
/* -------------------------------------------------------------*/
#if defined(__qdsp6__) && defined(SIM)
#define BUFPOOL_ASSERT()                                                                                               \
   {                                                                                                                   \
      *((volatile uint32_t *)0) = NULL;                                                                                \
   }
#else
#define BUFPOOL_ASSERT() (void)sizeof(0)
#endif

// Max number of pools
#define POSAL_BUFPOOL_MAX_POOLS (8)

// Number of nodes per array
#define POSAL_BUFPOOL_MAX_NODES (32) // since we're using uint32_t as list bitmasks DONT CHANGE

// Magic number for handle verification
#define POSAL_BUFPOOL_HANDLE_MAGIC (0x5a5a0000uL)

// Magic number for pool node, limited to 8 bits
#define POSAL_BUFPOOL_NODE_MAGIC (0x5a)

/*--------------------------------------------------------------*/
/* Type definitions                                             */
/* -------------------------------------------------------------*/
/* List Node structure declaration*/
typedef struct posal_bufpool_nodes_t
{
   uint32_t list_bitmask; /*< Indicates allocated and returned node indices not allocated =  0-> ready to allocate
                             not returned  =  1-> ready to return */
   void *mem_start_addr;  /*< Starting address of the range of nodes*/
} posal_bufpool_nodes_t;

/* List Node Pool Information structure declaration*/
typedef struct posal_bufpool_pool_t
{
   posal_bufpool_nodes_t *nodes_ptr;          /* Array of nodes*/
   uint16_t               num_of_node_arrays; /* number of arrays in the pool */
   uint16_t      		  nodes_per_arr; /*maximum number of nodes that can be stored in an array*/
   uint16_t               node_size;          /* Size of each individual node */
   uint16_t               allocated_nodes; /* Signed numbers to ensure proper arithmetic in case of bugs/corruption */
   uint16_t               used_nodes;
   uint16_t				  align_padding_size; /*alignment of the pointer to be returned to the client. (sizeof posal_bufpool_node_header)*/
   POSAL_HEAP_ID 		  heap_id;       /* heap id used for allocating for this pool */
   posal_mutex_t 		  pool_mutex;    /*< Mutex to prevent simultaneous access*/
} posal_bufpool_pool_t;

/* Header added to each node. Uses a union to ensure proper alignment if pointer types are
 * 64 bits */
typedef struct posal_bufpool_node_header_t
{
   union
   {
      // Anonymous struct is necessary, since bitfields don't behave properly in unions
      struct
      {
         uint32_t pool_index : 5;      // 5 bits for 32 pools
         uint32_t node_arr_index : 14; // 14 bits for 16384 arrays
         uint32_t node_index : 5;      // 5 bits for 32 nodes per array, no room for expansion here
         uint32_t magic : 8;           // magic number for debug
      };
      void *unused_ptr;
   };
} posal_bufpool_node_header_t;

/*--------------------------------------------------------------*/
/* Function Declarations/definitions                            */
/* -------------------------------------------------------------*/
ar_result_t posal_bufpool_allocate_new_nodes_arr(posal_bufpool_pool_t *pool_ptr, uint32_t index, uint32_t pool_index);
ar_result_t posal_bufpool_allocate_new_nodes_arr_util_(posal_bufpool_pool_t *pool_ptr, uint32_t index, uint32_t pool_index); // internal util

void posal_bufpool_free_nodes_arr(posal_bufpool_pool_t *pool_ptr, uint32_t index);

ar_result_t validate_handle(uint32_t handle, uint32_t *index);

//void toggle_bit_in_list_bitmask_at_idx(posal_bufpool_pool_t *pool_ptr, uint32_t index, uint32_t pos);

//uint32_t get_first_zero_idx_in_list_bitmask(posal_bufpool_pool_t *pool_ptr, uint32_t index);
#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef POSAL_BUFPOOL_I_H
