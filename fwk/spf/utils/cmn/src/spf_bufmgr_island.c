/**
 * \file spf_bufmgr_island.c
 * \brief
 *     This file contains the implementation for spf bufmgr wrappers.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "spf_utils.h"

/** Metadata for freed buffers.

When freed, all buffers managed by the buffer manager require extra
metadata for the following:
- To help return the buffers to their proper place in the buffer manager
- To detect double-free scenarios
 */
static const uint32_t CORRUPTION_DETECT_MAGIC = 0x836ADF71;

// Global variables.
posal_bufmgr_t *spf_bufmgr_ptr;

/* Dummy global variable whose address will be used to
   indicate that buffer node is allocated from heap. */
uint32_t g_heap_alloc_indicator;

/* =======================================================================
 **                          Function Definitions
 ** ======================================================================= */
bool_t spf_is_bufmgr_node(void *buf_ptr)
{
   // this function is called in steady state while returning a buffer
   // Today we don't use bufmgr nodes at steady state.
   // Therefore, in island mode, we return false
   if (posal_island_get_island_status())
   {
      #ifdef DEBUG_SPF_BUFMGR
         AR_MSG_ISLAND(DBG_LOW_PRIO, "In Island mode: returning spf_is_bufmgr_node as FALSE");
      #endif
      return FALSE;
   }
   else
   {
      posal_bufmgr_t *bufmgr_ptr    = spf_bufmgr_ptr;
      void *start_addr              = (void *)bufmgr_ptr->pStartAddr;
      void *end_addr                = (void *)((uint8_t *)start_addr + bufmgr_ptr->size);
      void *addr                    = (void *)buf_ptr;

      spf_bufmgr_metadata_t *pMetadata       = (spf_bufmgr_metadata_t *) ((uint8_t *)(buf_ptr) - POSAL_BUFMGR_METADATA_SIZE);
      posal_queue_t *return_q_ptr   = (posal_queue_t *)pMetadata->word0;

      /* A buffer is a bufmgr node if the buffer address falls within the
       * bufmgr range or if the return queue address == &g_heap_alloc_indicator */
      if ((((uint64_t)addr >= (uint64_t)start_addr) && ((uint64_t)addr < (uint64_t)end_addr)) || ((uint64_t *)return_q_ptr == (uint64 *)&g_heap_alloc_indicator))
      {
         return TRUE;
      }
      return FALSE;
   }
}

ar_result_t spf_bufmgr_return_buf(void *pBuf)
{
   /* Check for metadata corruption */
   spf_bufmgr_metadata_t *pMetadata = (spf_bufmgr_metadata_t *) ((uint8_t *)(pBuf) - POSAL_BUFMGR_METADATA_SIZE);
   if ((uint32_t)pMetadata->word0 != ((uint32_t)pMetadata->word1 ^ CORRUPTION_DETECT_MAGIC))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "posal Bufmgr: trying to free buffer at %p, but this pointer is corrupted! Buffer is leaked!",
             pBuf);

      POSAL_ASSERT(0);
      return AR_EFAILED;
   }

   /* check if the buffer has been allocated from heap */
   if ((uint64_t)&g_heap_alloc_indicator == (uint64_t)pMetadata->word0)
   {
#ifdef DEBUG_POSAL_BUFMGR
      AR_MSG(DBG_HIGH_PRIO, "BufMgr HEAP ReturnBuffer: Buff=0x%x", pBuf);
#endif
      posal_memory_free(pMetadata);
      return AR_EOK;
   }

   /* check thread ID. if it is already zero, it means double free is happening on this buffer */
   if (0 == (uint64_t)pMetadata->word2)
   {
      AR_MSG(DBG_ERROR_PRIO, "posal Bufmgr: Double free detected. Buffer at %p is already freed!", pBuf);
      POSAL_ASSERT(0);
      return AR_EFAILED;
   }
   else
   {
      /* set thread ID to zero */
      pMetadata->word2 = 0;

      /* form bufmgr node and push it back to its home queue. */
      posal_bufmgr_node_t bufNode;
      bufNode.return_q_ptr = (posal_queue_t *)(pMetadata->word0);
      bufNode.buf_ptr      = pBuf;
#ifdef DEBUG_POSAL_BUFMGR
      AR_MSG(DBG_HIGH_PRIO, "BufMgr ReturnBuffer: Buff=0x%x", pBuf);
#endif
      ar_result_t result = posal_queue_push_back(bufNode.return_q_ptr, (posal_queue_element_t *)&bufNode);
      if (AR_DID_FAIL(result))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "posal Bufmgr: trying to free buffer at %p, but the queue returned error! Buffer is leaked!",
                pBuf);
         return AR_EFAILED;
      }
      return AR_EOK;
   }
}

ar_result_t spf_bufmgr_poll_for_buffer(uint32_t             nDesiredSize,
                                       posal_bufmgr_node_t *pNode,
                                       uint32_t *           pnActualSize,
                                       POSAL_HEAP_ID        heap_id)
{
   posal_bufmgr_t *pBufMgr = spf_bufmgr_ptr;
   uint32_t        unChannelMask;
   uint32_t        unChannelStatus;
   ar_result_t     result;
    spf_bufmgr_metadata_t  *metadata_ptr = NULL;

   /* mask off all the bufs that are too small */
   unChannelMask = pBufMgr->unAnyBufsMask & ((1 << s32_cl0_s32(nDesiredSize - 1)) - 1);

   /* enter critical section */
   posal_mutex_lock(pBufMgr->mutex);

   /* Take node off back of stack. Use back instead of front in attempt to
    * keep using the same buffers. Better for cache performance. */
   if (!(unChannelStatus = posal_channel_poll(pBufMgr->channel_ptr, unChannelMask)) ||
       AR_DID_FAIL(result = posal_queue_pop_back(pBufMgr->aBufferBin[s32_cl0_s32(unChannelStatus)].pQ,
                                                 (posal_queue_element_t *)pNode)))
   {
      AR_MSG(DBG_HIGH_PRIO, "Buffer Manager failed to find a free buffer. Trying to allocate from heap");
      posal_mutex_unlock(pBufMgr->mutex);

      uint8_t *buf = (uint8_t *)posal_memory_malloc(nDesiredSize + POSAL_BUFMGR_METADATA_SIZE, heap_id);
      if (NULL == buf)
      {
         AR_MSG(DBG_ERROR_PRIO, "Buffer Manager failed to allocate even from heap!");
         return AR_ENEEDMORE;
      }

      /* ReturnQ is set to addr of g_heap_alloc_indicator to indicate that this buffer came from heap */
      metadata_ptr = (spf_bufmgr_metadata_t *)buf;
      metadata_ptr->word0 = (void *)&g_heap_alloc_indicator;
      metadata_ptr->word1 = (void *) ((uint64_t)(metadata_ptr->word0) ^ CORRUPTION_DETECT_MAGIC);

      /* thread ID is not useful for heap allocations but retaining for consistency sake; */
      metadata_ptr->word2 = (void *)((uint64_t)posal_thread_get_curr_tid());
      metadata_ptr->word3 = 0;

      pNode->return_q_ptr = (posal_queue_t *)(metadata_ptr->word0);
      pNode->buf_ptr      = &buf[0]+sizeof(spf_bufmgr_metadata_t);

#ifdef DEBUG_POSAL_BUFMGR
      AR_MSG(DBG_HIGH_PRIO, "BufMgr HEAP GetBuffer: Buff=0x%x", pNode->buf_ptr);
#endif

      *pnActualSize = nDesiredSize;
      return AR_EOK;
   }

   *pnActualSize = 1 << s32_cl0_s32(unChannelStatus);

   /* set the thread ID of the calling function */
   metadata_ptr = (spf_bufmgr_metadata_t *)((uint8_t *)(pNode->buf_ptr) - sizeof(spf_bufmgr_metadata_t));
   metadata_ptr->word2 = (void *)((uint64_t)posal_thread_get_curr_tid());

#ifdef DEBUG_POSAL_BUFMGR
   AR_MSG(DBG_HIGH_PRIO, "BufMgr GetBuffer: Buff=0x%x", pNode->buf_ptr);
#endif

   /* leave critical section */
   posal_mutex_unlock(pBufMgr->mutex);
   return AR_EOK;
}
