/**
 * \file topo_buf_mgr.c
 *  
 * \brief
 *  
 *     Topology buffer manager
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_topo.h"

ar_result_t topo_buf_manager_init(gen_topo_t *topo_ptr)
{
   topo_ptr->buf_mgr.prev_destroy_unused_call_ts_us = posal_timer_get_time();
   return AR_EOK;
}

static ar_result_t topo_buf_manager_free_list_nodes(topo_buf_manager_t *buf_mgr_ptr, spf_list_node_t **head_ptr)
{
   if (NULL == head_ptr)
   {
      return AR_EFAILED;
   }

   spf_list_node_t *curr_ptr = *head_ptr;
   while (NULL != curr_ptr)
   {
      spf_list_node_t *next_ptr = curr_ptr->next_ptr;

      int8_t *ptr = (int8_t *)curr_ptr;
      posal_memory_free(ptr);

      buf_mgr_ptr->total_num_bufs_allocated--;

      curr_ptr = next_ptr;
   }

   *head_ptr = NULL;
   return AR_EOK;
}

void topo_buf_manager_deinit(gen_topo_t *topo_ptr)
{
   /* deletes nodes and objects, as memory for list node and object is allocated
    * as one chunk */

   topo_buf_manager_free_list_nodes(&topo_ptr->buf_mgr, (&topo_ptr->buf_mgr.head_node_ptr));

   if (topo_ptr->buf_mgr.total_num_bufs_allocated)
   {
      TBF_MSG(topo_ptr->gu.log_id,
              DBG_ERROR_PRIO,
              "topo_buf_manager_destroy: Not all buffers returned, number of unreturned buffers: %d",
              topo_ptr->buf_mgr.total_num_bufs_allocated);
   }

   memset(&topo_ptr->buf_mgr, 0, sizeof(topo_buf_manager_t));
}
