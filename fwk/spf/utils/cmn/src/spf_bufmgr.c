/**
 * \file spf_bufmgr.c
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

#include "ar_msg.h"

extern uint32_t g_heap_alloc_indicator;

/* Macro for maximum bins */
#define POSAL_BUFMGR_MAX_BUFFER_BINS 32
static const uint32_t MSB_32                  = 0x80000000L;
static const uint32_t CORRUPTION_DETECT_MAGIC = 0x836ADF71;
/*--------------------------------------------------------------*/
/* Macro definitions                                            */
/* -------------------------------------------------------------*/

/* -----------------------------------------------------------------------
 ** Constant / Define Declarations
 ** ----------------------------------------------------------------------- */

/* -----------------------------------------------------------------------
 ** Static variables
 ** ----------------------------------------------------------------------- */
static posal_atomic_word_t nInstanceCount;

// Global variables.
extern posal_bufmgr_t *spf_bufmgr_ptr;

/* =======================================================================
**                          Function Definitions
** ======================================================================= */
ar_result_t spf_bufmgr_global_init(POSAL_HEAP_ID heap_id)
{
   ar_result_t result = AR_EOK;

   AR_MSG(DBG_LOW_PRIO, "Enter spf_bufmgr_global_init");

   /*
    * This is a configuraton parameter for spf global memory pool.
    * This is the distribution of different size buffers.
    * It can be optimized based on the spf system load.
    * Current configurations:
    * 16 uint8_t:  32 buffers
    * 32 uint8_t:  16 buffers
    * 64 uint8_t:  8 buffers
    * 128 uint8_t: 8 buffers
    * 256 uint8_t: 2 buffers
    * 512 uint8_t: 4 buffers
    */
   const uint32_t buf_bins[] = { 0, 0, 0, 0, 32, 16, 8, 8, 2, 4, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

   // Initialize global variables to zero.
   spf_bufmgr_ptr = NULL;

   if (AR_DID_FAIL(result = posal_atomic_word_create(&nInstanceCount, heap_id)))
   {
      return result;
   }

   // Allocate memory.
   result = spf_bufmgr_create(buf_bins, &spf_bufmgr_ptr, heap_id);
   if (AR_DID_FAIL(result))
   {
      AR_MSG(DBG_HIGH_PRIO, "Failed to create buffer manager!!");
      return AR_EFAILED;
   }

   AR_MSG(DBG_LOW_PRIO, "Leave spf_bufmgr_global_init");

   return result;
}

void spf_bufmgr_global_deinit(void)
{
#ifndef DISABLE_DEINIT
   AR_MSG(DBG_LOW_PRIO, "Enter spf_bufmgr_global_destroy");

   // Clean up all resources.
   spf_bufmgr_destroy(spf_bufmgr_ptr);

   posal_atomic_word_destroy(nInstanceCount);

   // Unset all global variables.
   spf_bufmgr_ptr = NULL;

   AR_MSG(DBG_LOW_PRIO, "Leave spf_bufmgr_global_destroy");
#endif //#ifndef DISABLE_DEINIT
}

ar_result_t spf_bufmgr_create(const uint32_t *nBufsInBin, posal_bufmgr_t **ppBufMgr, POSAL_HEAP_ID heap_id)
{
   ar_result_t result;
   uint32_t    binIdx                = 0;
   char *      pStartAddr            = NULL;
   uint32_t    bufmgr_meta_data_size = POSAL_BUFMGR_METADATA_SIZE;
   uint32_t    mem_blob_size_bytes   = 0;
   spf_bufmgr_metadata_t  *metadata_ptr = NULL;

   if (NULL == ppBufMgr)
   {
      return AR_EBADPARAM;
   }

   for (binIdx = 0; binIdx < 32; binIdx++)
   {
      mem_blob_size_bytes += (nBufsInBin[binIdx]) * (bufmgr_meta_data_size + (1 << binIdx));
   }

   /* Initialized posal global buffer manager*/
   pStartAddr = (char *)posal_memory_aligned_malloc(mem_blob_size_bytes, 8, heap_id);
   if (NULL == pStartAddr)
   {
      AR_MSG(DBG_FATAL_PRIO, "Failed to allocate memory for buffers.");
      return AR_ENOMEMORY;
   }

   /* Allocate memory for this instance */
   if (!(*ppBufMgr = (posal_bufmgr_t *)posal_memory_malloc(sizeof(posal_bufmgr_t), heap_id)))
   {
      AR_MSG(DBG_FATAL_PRIO, "Out of memory trying to posal_bufmgr_create!");
      return AR_ENOMEMORY;
   }

   /* Memset to zero */
   POSAL_ZEROAT(*ppBufMgr);

   /* save start address of buffer */
   posal_bufmgr_t *pBufMgr = *ppBufMgr;
   pBufMgr->pStartAddr     = pStartAddr;
   pBufMgr->size           = mem_blob_size_bytes;

   /* Inititialize the channel & mutex */
   if (AR_DID_FAIL(result = posal_channel_create(&pBufMgr->channel_ptr, heap_id)))
   {
      AR_MSG(DBG_FATAL_PRIO, "posal_bufmgr_create failed to create channel");
      spf_bufmgr_destroy(pBufMgr);
      *ppBufMgr = NULL;
      return result;
   }

   posal_mutex_create(&pBufMgr->mutex, heap_id);

   /* Build up the array of buffer bins */
   uint8_t *pBuffer = (uint8_t *)pStartAddr;
   for (binIdx = 3; binIdx < POSAL_BUFMGR_MAX_BUFFER_BINS; binIdx++)
   {
      /* Continue if no buffers are requested for each bin */
      if (0 == nBufsInBin[binIdx])
      {
         continue;
      }

      /* Add this bin to the mask */
      pBufMgr->unAnyBufsMask |= (MSB_32 >> binIdx);

      /* save number of buffers */
      posal_bufbin_t *pBin = &pBufMgr->aBufferBin[binIdx];
      pBin->nBufs          = nBufsInBin[binIdx];

      /* Create the queue name */
      char name[POSAL_DEFAULT_NAME_LEN];
      int  count = posal_atomic_increment(nInstanceCount) & 0x000000FFL;
      snprintf(name, POSAL_DEFAULT_NAME_LEN, "BFRMGR%xBIN%lu", count, binIdx);

      /* Round up queue nodes to nearest power of 2 */
      int nQueueNodes = 1 << (32 - s32_cl0_s32(nBufsInBin[binIdx] - 1));

      /* Create Q and add it to channel. */
      posal_queue_init_attr_t q_attr;
      posal_queue_attr_init(&q_attr);
      posal_queue_attr_set_heap_id(&q_attr, heap_id);
      posal_queue_attr_set_max_nodes(&q_attr, nQueueNodes);
      posal_queue_attr_set_prealloc_nodes(&q_attr, 0);
      posal_queue_attr_set_name(&q_attr, name);
      if (AR_DID_FAIL(result = posal_queue_create_v1(&pBin->pQ, &q_attr)) ||
          AR_DID_FAIL(result = posal_channel_addq(pBufMgr->channel_ptr, pBin->pQ, (MSB_32 >> binIdx))))
      {
         AR_MSG(DBG_FATAL_PRIO, "posal_bufmgr_create failed to create queues for buffer nodes!");
         spf_bufmgr_destroy(pBufMgr);
         *ppBufMgr = NULL;
         return result;
      }

      /* fill the queue with pointers */
      posal_bufmgr_node_t bufNode;
      bufNode.return_q_ptr = pBin->pQ;

      uint32_t buf_size_in_words = (1 << binIdx); //pBuffer is changed to uint8_t. Hence the divide with uin32_t is removed.
      for (int buf_id = 0; buf_id < pBin->nBufs; buf_id++)
      {
         /*
          *    fill 4 metadata words for finding buffer's home queue and for debug info.
          *    Word 0 - the return queue handle;
          *              &g_heap_alloc_indicator implies that the buf is allocated from heap
          *    Word 1 - (for corruption detection) the return queue handle XOR a magic number.
          *    Word 2 - thread ID of the allocating function. 0 implies unallocated buffer.
          *    Word 3 - for 8-byte alignment.
          */
         metadata_ptr = (spf_bufmgr_metadata_t *) pBuffer;
         metadata_ptr->word0 = (void *)(pBin->pQ);
         metadata_ptr->word1 = (void *)((uint64_t)(pBin->pQ) ^ CORRUPTION_DETECT_MAGIC);
         metadata_ptr->word2 = 0;
         metadata_ptr->word3 = 0;

         pBuffer += sizeof(spf_bufmgr_metadata_t);
         bufNode.buf_ptr = (char *)pBuffer;

         posal_queue_push_back(pBin->pQ, (posal_queue_element_t *)&bufNode);
         pBuffer += buf_size_in_words;
      }
   }
   return AR_EOK;
}

/* If this function hangs, it means there is a buffer leak. */
void spf_bufmgr_destroy(posal_bufmgr_t *pBufMgr)
{
   ar_result_t         result __attribute__((unused));
   posal_bufmgr_node_t bufNode;

   if (!pBufMgr)
      return;

   /* lock out all clients and drain all the buffers */
   posal_mutex_lock(pBufMgr->mutex);

   for (int binIdx = 0; binIdx < POSAL_BUFMGR_MAX_BUFFER_BINS; binIdx++)
   {
      if (pBufMgr->unAnyBufsMask & (MSB_32 >> binIdx))
      {
         for (int buf_id = 0; buf_id < pBufMgr->aBufferBin[binIdx].nBufs; buf_id++)
         {
            (void)posal_channel_wait(pBufMgr->channel_ptr, MSB_32 >> binIdx);
            result = posal_queue_pop_front(pBufMgr->aBufferBin[binIdx].pQ, (posal_queue_element_t *)&bufNode);
            POSAL_ASSERT(AR_SUCCEEDED(result));
         }
         posal_queue_destroy(pBufMgr->aBufferBin[binIdx].pQ);
      }
   }

   /* destroy channel */
   posal_channel_destroy(&pBufMgr->channel_ptr);

   /* Free buffer memory blob */
   posal_memory_aligned_free(pBufMgr->pStartAddr);

   /* destroy mutex */
   posal_mutex_unlock(pBufMgr->mutex);
   posal_mutex_destroy(&pBufMgr->mutex);

   /* free buffer manager handle. */
   posal_memory_free(pBufMgr);
}
