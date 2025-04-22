/**
 * \file posal_bufpool.h
 * \brief 
 *   	Header file for buffer pool functionality for small allocations.
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_BUFPOOL_H
#define POSAL_BUFPOOL_H

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/*--------------------------------------------------------------*/
/* Macro definitions                                            */
/* -------------------------------------------------------------*/
#define POSAL_BUFPOOL_INVALID_HANDLE (0)

enum posal_bufpool_align_t
{
   FOUR_BYTE_ALIGN,
   EIGHT_BYTE_ALIGN
};

typedef enum  posal_bufpool_align_t posal_bufpool_align_t;

/* =======================================================================
**                          Function Definitions
** ======================================================================= */
uint32_t posal_bufpool_pool_create(uint16_t node_size, POSAL_HEAP_ID heap_id, uint32_t num_arrays, posal_bufpool_align_t alignment,uint16_t nodes_per_arr);

void* posal_bufpool_get_node(uint32_t pool_handle);

void posal_bufpool_return_node(void* node_ptr);

void posal_bufpool_pool_destroy(uint32_t pool_handle);

void posal_bufpool_pool_reset_to_base(uint32_t pool_handle);

void posal_bufpool_pool_free_unused_lists(uint32_t pool_handle);

bool_t posal_bufpool_is_address_in_bufpool(void *ptr, uint32_t pool_handle);

uint32_t posal_bufpool_profile_all_mem_usage();

ar_result_t posal_bufpool_profile_mem_usage(uint32_t pool_handle, uint32_t* bytes_used_ptr, uint32_t* bytes_allocated_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef POSAL_BUFPOOL_H
