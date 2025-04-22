#ifndef _SPF_LPI_POOL_UTILS_H__
#define _SPF_LPI_POOL_UTILS_H__

// clang-format off
/**
 * \file spf_lpi_pool_utils.h
 * \brief
 *     This file contains declarations for the SPF Metadata Pool Utilities
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


/*==============================================================================
   MACROS
==============================================================================*/
#define MAX_NUM_SPF_LPI_POOLS 3 // can be tuned
/*------------------------------------------------------------------------------
*  Type definitions
*----------------------------------------------------------------------------*/
typedef struct pool_info_t
{
   uint32_t    pool_handle;
   uint32_t    node_size;
} pool_info_t;

typedef struct spf_lpi_pools_t
{
   uint32_t    num_pools;
   pool_info_t pool_arr[MAX_NUM_SPF_LPI_POOLS];
   uint16_t sorted_node_size_arr [MAX_NUM_SPF_LPI_POOLS];
} spf_lpi_pools_t;

// clang-format on
/*==============================================================================
   Function Declarations
==============================================================================*/
/*
  Initialize the MD Bufpool

  param[in] node_size:  Size of the nodes in the associated bufpool

  param[in] num_arrays: Number of arrays in the bufpool

  param[in] heap_id: heap id for memory allocation

  return: AR error code
*/
ar_result_t spf_lpi_pool_init(uint32_t node_size, uint32_t num_arrays, uint16_t nodes_per_array, POSAL_HEAP_ID heap_id);

/*
  Get a node of the requested size from the appropriate MD bufpool

  param[in] req_size:  Size of the nodes requested

  return: pointer to the node
*/
void *spf_lpi_pool_get_node(uint32_t req_size);

/*
  Return a node to the appropriate MD bufpool

  param[in] node_ptr: Pointer to the node that must be reurned to the bufpool

  return: none
*/
void spf_lpi_pool_return_node(void *node_ptr);

/*
  Destroy all the MD Bufpools

  return: AR error code
*/
ar_result_t spf_lpi_pool_deinit();

/*
  Address Check

  param[in] : address

  return: boolean indicaiting if it is from the bufpool
*/
bool_t spf_lpi_pool_is_addr_from_md_pool(void *ptr);

#ifdef __cplusplus
}
#endif //__cplusplus
#endif /* _SPF_LPI_POOL_UTILS_H__ */
