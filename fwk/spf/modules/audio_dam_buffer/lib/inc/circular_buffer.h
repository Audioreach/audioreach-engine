#ifndef __CIRC_BUF_UTILS_H
#define __CIRC_BUF_UTILS_H

/**
 *   \file circ_buffer.h
 *   \brief
 *        This file contains utility functions for handling fragmented circular  buffer operation.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_defs.h"
#include "posal.h"
#include "spf_list_utils.h"

#define FRAG_CIRC_BUF_MAX_CLIENTS 8
//#define DEBUG_CIRC_BUF_UTILS

/*==============================================================================
   Function declarations
==============================================================================*/

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

typedef struct circ_buf_t circ_buf_t;

typedef struct circ_buf_client_t
{
   bool_t      is_read_client;       // Flag to indicate if the client is read/write
   circ_buf_t *circ_buf_ptr;         // Handle to the circular buffer of the client.
   uint32_t    req_base_buffer_size; // base buffer size as requested at the client registration.

   spf_list_node_t *rw_chunk_node_ptr; // Pointer to the current read/write chunk node.
   uint32_t         rw_chunk_offset;   // Current offset from base of the chunk memory.
   uint32_t         unread_bytes;      // number of unread bytes

   uint32_t      req_buffer_resize; // This is the resize(in bytes) request by the clients.
   POSAL_HEAP_ID chunk_heap_id;     // heap ID used for allocating chunks in the buffer.
} circ_buf_client_t;

typedef struct chunk_buffer_t
{
   uint32_t id;         /** Index of chunk memory */
   uint32_t size;       /** Size of this chunk memory */
   int8_t * buffer_ptr; /** Pointer to the start of this chunk memory*/
} chunk_buffer_t;

typedef struct circ_buf_t
{
   uint32_t         id;
   uint32_t         circ_buf_size;        // Total circular buffer size in bytes (base_buffer_size + max_req_alloc_size)
   POSAL_HEAP_ID    heap_id;              // Default Heap id for memory allocations.
   uint32_t         preferred_chunk_size; // preferred size in bytes for buffer chunks
   uint32_t         num_chunks;           // Size of the buffer_ptr array of mem addresses
   spf_list_node_t *head_chunk_ptr;       // Array of non-contiguous memory chunks
   uint32_t         prebuffering_delay;   // pre buffering delay in bytes

   circ_buf_client_t *wr_client_ptr;      // Holds the pointer to the writer client. There is only one writer client.
   uint32_t           write_byte_counter; // This counter is incremented for every byte written into circular buffer

   uint32_t         num_read_clients;
   spf_list_node_t *rd_client_list_ptr; // List of all registered clients with the buffer.

   uint32_t         max_req_buf_resize;  // Maximum of the requested client resizes.
   POSAL_HEAP_ID    chunk_heap_id;       // Max Client heap ID used for allocating chunks in the buffer.

   uint32_t is_valid_timestamp; // Flag to indicate if the timestamp is valid.
   int64_t timestamp;           // Timestamp of the latest sample in the buffer.
} circ_buf_t;

/** Enumerations for return values of circular buffer operations */
typedef enum {
   CIRCBUF_SUCCESS = 0, /** Operation was successful */
   CIRCBUF_FAIL,        /** General failure */
   CIRCBUF_OVERRUN,     /** Buffer overflowed */
   CIRCBUF_UNDERRUN     /** Buffer underflowed */
} circbuf_result_t;

/*
 * Initializes the Circular buffer
 * circ_buf_struct_ptr[in/out]  : pointer to a circular buffer structure
 * circ_buf_size[in]            : Total circular buffer size in bytes
 * preferred_chunk_size[in]     : Preferred size of each memory chunk in bytes
 * heap_id[in]                  : heap_id for malloc
 * functionality : Allocates non-contiguous buffer and initializes the circular
 * buffer parameters. The memory chunks are multiple memory allocations, each
 * smaller than the circular buffer size. The sum of the memory chunk sizes
 * equals the circular buffer size.
 * return : circbuf_result
 */
circbuf_result_t circ_buf_alloc(circ_buf_t *  circ_buf_struct_ptr,
                                uint32_t      circ_buf_size,
                                uint32_t      preferred_chunk_size,
                                uint32_t      circ_buf_id,
                                POSAL_HEAP_ID heap_id);

/*
 * Initializes the Circular buffer client.
 * circ_buf_struct_ptr[in/out]  : pointer to a circular buffer structure
 * is_read_client[in]           : Is it the reader or writer client
 * heap_id[in]                  : heap_id for allocating chunk buffer
 * req_base_buffer_size[in]     : Requested base buffer size for allocation
 * ret_handle[out]              : Client handle
 */
circbuf_result_t circ_buf_register_client(circ_buf_t *       circ_buf_struct_ptr,
                                          bool_t             is_read_client,
                                          POSAL_HEAP_ID      chunk_heap_id,
                                          uint32_t           req_base_buffer_size,
                                          circ_buf_client_t *ret_handle);
/*
 * Read samples from the Circular buffer
 * circ_buf_struct_ptr[in/out]  : pointer to a circular buffer structure
 * out_ptr[out]                 : Pointer to where data has to be read from
 *                                circular buffer
 * bytes_to_read[in]            : number of bytes to be read
 * functionality : copies the samples_to_read number of samples to out_ptr
 * return : circbuf_result
 */
 circbuf_result_t circ_buf_read(circ_buf_client_t *rd_client_ptr,
                                int8_t *           out_ptr,
                                uint32_t           bytes_to_read,
                                uint32_t *         actual_read_len_ptr);
/*
 * Write samples to the Circular buffer
 * wr_client_ptr [in/out]       : pointer to a circular buffer structure
 * inp_ptr[in]                  : Pointer from where data has to be written
 *                                to circular buffer
 * bytes_to_write[in]           : number of bytes to write
 * functionality : Checks the available unread space in the circular and
 *                 copies the samples_to_write amount of samples to inp_ptr
 * return : circbuf_result
 */
circbuf_result_t circ_buf_write(circ_buf_client_t *wr_client_ptr,
                                int8_t *           inp_ptr,
                                uint32_t           bytes_to_write,
                                uint32_t           is_valid_timestamp,
                                int64_t            timestamp);

/*
 * Adjusts the read pointer
 * circ_buf_struct_ptr[in/out]  : pointer to a circular buffer structure
 * read_offset[in]              : offset needed between new read_index and
 *                                current write_index
 * functionality : Moves the read_index to a new position to start reading from
 * write_index - read_offset
 * return : circbuf_result
 */
circbuf_result_t circ_buf_read_adjust(circ_buf_client_t *rd_handle,
                                      uint32_t           read_offset,
                                      uint32_t *         un_read_bytes,
                                      bool_t             force_adjust);

/*
 * Read client request to resize the buffer.
 * circ_buf_struct_ptr[in/out]  : pointer to a circular buffer structure
 * req_buffer_resize[in]        : Buffer allocation re-size request in bytes.
 * functionality : Moves the read_index to a new position to start reading from
 * write_index - read_offset
 * return : circbuf_result
 */
circbuf_result_t circ_buf_read_client_resize(circ_buf_client_t *rd_client_ptr, uint32_t req_buffer_resize, POSAL_HEAP_ID heap_id);
/*
 * Adjusts the write pointer
 * circ_buf_struct_ptr[in/out]  : pointer to a circular buffer structure
 * write_value[in]              : value to be writen
 * num_bytes[in]                : Number of bytes to be written
 * functionality : Sets the first num_bytes of the block of memory from current
 * circular bufferr write pointer to specified write_value
 * return : circbuf_result
 */
circbuf_result_t circ_buf_memset(circ_buf_t *frag_circ_buf_ptr, uint8_t write_value, uint32_t num_bytes);
/*
 * Initializes the Circular buffer
 * client_ptr[in/out]  : De register the client from the circular buffer.
 * functionality : De-registers the read/write client ti
 * return : circbuf_result
 */
circbuf_result_t circ_buf_deregister_client(circ_buf_client_t *client_ptr);

/*
 * Deinits and deallocate all the memory in the circular buffer.
 * circ_buf_struct_ptr[in/out]  : pointer to a circular buffer structure
 * functionality : Deallocates the buffer and Deinits the circular buffer
 * return : circbuf_result
 */
circbuf_result_t circ_buf_free(circ_buf_t *circ_buf_ptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*__CIRC_BUF_UTILS_H*/
