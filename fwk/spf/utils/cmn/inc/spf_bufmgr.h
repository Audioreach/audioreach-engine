#ifndef _SPF_BUFMGR_H_
#define _SPF_BUFMGR_H_

/**
 * \file spf_bufmgr.h
 * \brief
 *    This file is a wrapper for bufmgr usage in spf. spf framework code
 *  will share a single bufmgr which is accessed using these wrapper functions.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*===========================================================================
NOTE: The @brief description above does not appear in the PDF.
      The description that displays in the PDF is located in the
      Elite_mainpage.dox file. Contact Tech Pubs for assistance.
===========================================================================*/

/*-------------------------------------------------------------------------
Include Files
-------------------------------------------------------------------------*/
#include "spf_utils.h"
#include "posal.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/*-------------------------------------------------------------------------
Type Declarations
-------------------------------------------------------------------------*/
/** Structure holding all spf_bufmgr static variables.
 */

/* This struct is the bin for each power-of-2 buffer size. */
typedef struct posal_bufbin_t
{
   int            nBufs;
   posal_queue_t *pQ;
} posal_bufbin_t;

/* This is the state instance of the buffer manager. */
typedef struct posal_bufmgr_t
{
   char *          pStartAddr;
   uint32_t        size;
   posal_mutex_t   mutex;
   posal_channel_t channel_ptr;
   uint32_t        unAnyBufsMask;
   posal_bufbin_t  aBufferBin[32];
} posal_bufmgr_t;

/** Node that represents a buffer. When clients request a buffer from the
  buffer manager, this node is provided.
 */
typedef struct posal_bufmgr_node_t
{
   void *buf_ptr;
   /**< Pointer to the buffer. */

   posal_queue_t *return_q_ptr;
   /**< Queue to which this buffer must be returned. @newpage */
} posal_bufmgr_node_t;

/* This struct is used to store metadata in both 32-bit and 64-bit processors */
typedef struct spf_bufmgr_metadata_t
{
   void *word0;
   void *word1;
   void *word2;
   void *word3;
} spf_bufmgr_metadata_t;

#define POSAL_BUFMGR_METADATA_SIZE ((uint32_t)sizeof(spf_bufmgr_metadata_t))

/*-------------------------------------------------------------------------
Function Declarations and Documentation
--------------------------------------------------------------------------*/

/**
   Creates the bufmgr with a predetermined memory configuration.

   @return
   Indication of succss or failure.
   */
ar_result_t spf_bufmgr_global_init(POSAL_HEAP_ID heap_id);

/**
   Destroys the bufmgr.

   @dependencies
   spf_bufmgr_global_init() must be called before calling this function.
   @newpage
   */
void spf_bufmgr_global_deinit(void);

/**
   Creates the buffer manager. This function also performs a sanity check on
   the sizes of the requested set of buffers.

   @param[in]  bufsize_bins  Pointer to an array of 32 integers. \n
                             bufsize_bins[n] is the number of buffers of size
                             2^n bytes to be managed by the buffer manager. The
                             first three elements must be 0 because 8 is the
                             minimum supported buffer size.
   @param[in]  heap_id       Heap ID required for mallocs
   @param[out] buf_mgr_pptr  Double pointer to the buffer manager created if
                             this function returns AR_EOK.

   @return
   AR_EBADPARAM -- Invalid bufsize_bins (e.g., a request for buffers smaller
                   than 8 bytes).
   @par
   AR_EOK -- bufsize_bins is valid. The buffer manager is created and its
               pointer is put in the buf_mgr_pptr parameter.

   @dependencies
   None. @newpage
 */
ar_result_t spf_bufmgr_create(const uint32_t *bufsize_bins, posal_bufmgr_t **buf_mgr_pptr, POSAL_HEAP_ID heap_id);
/**
   Takes the address of a managed buffer, looks up the metadata, and pushes the
   address to the home queue.

   @param[in] buf_ptr  Pointer to the buffer manager instance.

   @return
   0 -- Success
   @par
   Nonzero -- Failure

   @dependencies
   Before calling this function, the buffer manager must be created, and it must
   have polled for a buffer.
 */
ar_result_t spf_bufmgr_return_buf(void *buf_ptr);

/**
   Waits for all managed buffers to be returned to their queues, and
   then destroys all resources.

   @datatypes
   posal_bufmgr_t

   @param[in] buf_mgr_ptr  Pointer to the buffer manager instance.

   @return
   None.

   @dependencies
   Before calling this function, the buffer manager must be created. @newpage
 */
void spf_bufmgr_destroy(posal_bufmgr_t *buf_mgr_ptr);

/**
   Requests a buffer from the manager. If a buffer of adequate size is
   available, a node is returned with pointers to the buffer and the return
   queue of the buffer.

   @datatypes
   posal_bufmgr_node_t

   @param[in]  desired_size  Number of bytes needed in the requested buffer.
   @param[out] node_ptr      Returned node that points to the buffer and
                             return queue.
   @param[out] actual_size   Pointer to the actual size of the buffer that is
                             becoming available.

   @return
   AR_ENEEDMORE -- No buffers of adequate size are available.
   @par
   AR_EOK -- A buffer was found and is becoming available.

   @dependencies
   spf_bufmgr_global_init() must be called before calling this function.
   @newpage
  */
ar_result_t spf_bufmgr_poll_for_buffer(uint32_t             desired_size,
                                       posal_bufmgr_node_t *node_ptr,
                                       uint32_t *           actual_size,
                                       POSAL_HEAP_ID        heap_id);

/**
   Determines if the spf buffer manager allocated this buffer.

   @datatypes
   @param[in] buf_ptr       Pointer to the buffer.

   @return
   TRUE  -- The spf buffer manager allocated this buffer.
   @par
   FALSE -- The spf buffer manager did not allocate this buffer.

   @dependencies
   spf_bufmgr_global_init() must be called before calling this function.
   @newpage
  */
bool_t spf_is_bufmgr_node(void *buf_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef _SPF_BUFMGR_H_
