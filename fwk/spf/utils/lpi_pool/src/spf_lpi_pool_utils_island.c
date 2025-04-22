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
//#define MD_POOL_DBG

spf_lpi_pools_t lpi_pools_info = { 0 };

static ar_result_t spf_lpi_pool_get_best_fit_pool_handle(uint32_t *pool_handles_arr, uint32_t req_size)
{
   uint8_t best_idx = 0;
   for (uint32_t i = 0; i < MAX_NUM_SPF_LPI_POOLS; i++)
   {
      if (req_size <= lpi_pools_info.sorted_node_size_arr[i])
      {
         for (uint32_t j = 0; j < MAX_NUM_SPF_LPI_POOLS; j++)
         {
            if (lpi_pools_info.sorted_node_size_arr[i] == lpi_pools_info.pool_arr[j].node_size)
            {
               pool_handles_arr[best_idx++] = lpi_pools_info.pool_arr[j].pool_handle;
               break;
            }
         }
      }
   }
   return (best_idx == 0) ? AR_EFAILED : AR_EOK;
}

void *spf_lpi_pool_get_node(uint32_t req_size)
{
   void *   node_to_return                          = NULL;
   uint32_t pool_handles_arr[MAX_NUM_SPF_LPI_POOLS] = { 0 };

   if (AR_EOK != spf_lpi_pool_get_best_fit_pool_handle(pool_handles_arr, req_size))
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "Cannot find a fit for the req size %lu in the MD pools. Failing", req_size);
      return NULL;
   }

   for (uint32_t i = 0; i < lpi_pools_info.num_pools; i++)
   {
      if (pool_handles_arr[i])
      {
         node_to_return = posal_bufpool_get_node(pool_handles_arr[i]);
         if (node_to_return)
         {
#ifdef MD_POOL_DBG
            AR_MSG_ISLAND(DBG_HIGH_PRIO,
                          "Got Node ptr 0x%lx of req_size = %lu from pool with handle 0x%lx",
                          node_to_return,
                          req_size,
                          pool_handles_arr[i]);
#endif
            break;
         }
         // if null it can try the next best fit
      }
   }
   return node_to_return;
}

void spf_lpi_pool_return_node(void *node_ptr)
{
   if (NULL == node_ptr)
   {
      return;
   }
   posal_bufpool_return_node(node_ptr);
#ifdef MD_POOL_DBG
   AR_MSG_ISLAND(DBG_HIGH_PRIO, "Returned node ptr 0x%lx to MD Pool", node_ptr);
#endif
}

bool_t spf_lpi_pool_is_addr_from_md_pool(void *ptr)
{
   for (uint32_t i = 0; i < lpi_pools_info.num_pools; i++)
   {
      if (posal_bufpool_is_address_in_bufpool(ptr, lpi_pools_info.pool_arr[i].pool_handle))
      {
         return TRUE;
      }
   }
   return FALSE;
}
