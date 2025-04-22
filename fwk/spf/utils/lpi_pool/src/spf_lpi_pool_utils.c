/**
 * \file spf_lpi_pool_utils.c
 * \brief
 *     This file contains implementation of Metadata pool utilities for SPF
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*==============================================================================
   Include Files
==============================================================================*/
#include "spf_lpi_pool_utils.h"
/*==============================================================================
   Globals
==============================================================================*/
extern spf_lpi_pools_t lpi_pools_info;
/*==============================================================================
   Function Definitions
==============================================================================*/
// sorts node size array from small to big
void spf_lpi_pool_nodes_size_ascending_order(spf_lpi_pools_t *pool_ptr)
{
   uint16_t index, value_to_compare;
   int16_t  index_to_compare = 1;
   for (index = 1; index < MAX_NUM_SPF_LPI_POOLS; index++)
   {
      value_to_compare = pool_ptr->sorted_node_size_arr[index];
      index_to_compare = index - 1;
      while ((index_to_compare >= 0) && (pool_ptr->sorted_node_size_arr[index_to_compare] > value_to_compare))
      {
         pool_ptr->sorted_node_size_arr[index_to_compare + 1] = pool_ptr->sorted_node_size_arr[index_to_compare];
         index_to_compare                                     = index_to_compare - 1;
      }
      pool_ptr->sorted_node_size_arr[index_to_compare + 1] = value_to_compare;
   }
}

ar_result_t spf_lpi_pool_init(uint32_t node_size, uint32_t num_arrays, uint16_t nodes_per_array, POSAL_HEAP_ID heap_id)
{
   if (MAX_NUM_SPF_LPI_POOLS == lpi_pools_info.num_pools)
   {
      AR_MSG(DBG_ERROR_PRIO, "Max allowed number of MD pools %lu are already allocated.", MAX_NUM_SPF_LPI_POOLS);
      return AR_EFAILED;
   }

   uint32_t handle = posal_bufpool_pool_create(node_size, heap_id, num_arrays, EIGHT_BYTE_ALIGN, nodes_per_array);
   if (POSAL_BUFPOOL_INVALID_HANDLE == handle)
   {
      AR_MSG(DBG_FATAL_PRIO, "Received invalid handle from posal bufpool!");
      return AR_EFAILED;
   }
   uint32_t pool_id                             = lpi_pools_info.num_pools; // arr_index
   lpi_pools_info.pool_arr[pool_id].pool_handle = handle;
   lpi_pools_info.pool_arr[pool_id].node_size   = node_size;
   lpi_pools_info.sorted_node_size_arr[pool_id] = node_size;
   lpi_pools_info.num_pools++; // increment the num of pools
   spf_lpi_pools_t *pool_ptr = &lpi_pools_info;
   spf_lpi_pool_nodes_size_ascending_order(pool_ptr);

   AR_MSG(DBG_HIGH_PRIO, "Created MD Pool - heap_id = %lu, Handle 0x%lx node_size = %lu", heap_id, handle, node_size);
   return AR_EOK;
}

ar_result_t spf_lpi_pool_deinit() // AKR: don't think we need a per pool deinit (?)
{
   for (uint32_t i = 0; i < lpi_pools_info.num_pools; i++)
   {
      posal_bufpool_pool_destroy(lpi_pools_info.pool_arr[i].pool_handle);
   }
   AR_MSG(DBG_HIGH_PRIO, "Destroyed %lu MD Pools", lpi_pools_info.num_pools);
   memset(&lpi_pools_info, 0, sizeof(lpi_pools_info));
   return AR_EOK;
}
