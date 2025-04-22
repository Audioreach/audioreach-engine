#ifndef __SPF_CIRC_BUF_UTILS_H
#define __SPF_CIRC_BUF_UTILS_H

/**
 * \file spf_circular_buffer.h
 * \brief
 *    This file contains utility functions for handling fragmented circular
 *    buffer operation with metadata handling.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_defs.h"
#include "posal.h"
#include "spf_list_utils.h"
#include "capi_cmn.h"

#define FRAG_CIRC_BUF_MAX_CLIENTS 8

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/*==============================================================================
   Structure declarations
==============================================================================*/

typedef struct spf_circ_buf_t            spf_circ_buf_t;
typedef struct spf_circ_buf_raw_t        spf_circ_buf_raw_t;
typedef struct spf_circ_buf_client_t     spf_circ_buf_client_t;
typedef struct spf_circ_buf_raw_client_t spf_circ_buf_raw_client_t;
typedef struct spf_circ_buf_chunk_t      spf_circ_buf_chunk_t;
typedef struct spf_circ_buf_frame_t      spf_circ_buf_frame_t;
typedef struct spf_circ_buf_position_t   spf_circ_buf_position_t;

#include "spf_begin_pragma.h"
typedef struct spf_circ_buf_frame_t
{
   capi_stream_data_v2_t sdata;
   uint32_t              reader_ref_count;
   uint32_t              num_encoded_frames_in_cur_buf; // number of enc raw frames in cur buffer frame
   uint32_t              actual_data_len;               // filled bytes in the frame, max size is container frame size.
   uint8_t               data[0];
} spf_circ_buf_frame_t
#include "spf_end_pragma.h"
   ;

/* This structure can hold the current read/write position in the circular buffer.
 * To point a particualr byte in the fragmented circular buffer we need,
 *     1. Chunk pointer
 *     2. Frame in the chunk.
 *     3. byte offset in the frame.
 * */
typedef struct spf_circ_buf_position_t
{
   spf_list_node_t *     chunk_node_ptr; // Pointer to the current chunk node.
   uint32_t              frame_position; // Current r/w frame position in the chunk
   uint32_t              frame_offset;   // Current r/w byte offset in the current frame.
   capi_stream_data_v2_t readers_sdata;  // applicable only for reader client
} spf_circ_buf_position_t;

typedef struct spf_circ_buf_chunk_flags_t
{
   union
   {
      struct
      {
         uint32_t recreate : 1; // is set when an new mf is set, buffer will be recreated
         uint32_t destroy : 1;  // is set when the buffer needs to be destroyed.
      };
      uint32_t word;
   };
} spf_circ_buf_chunk_flags_t;

typedef struct spf_circ_buf_mf_info_t
{
   uint32_t bits_per_sample;
   uint32_t q_factor;
   uint32_t sampling_rate;
   uint32_t data_is_signed;
   uint32_t num_channels;
   uint16_t channel_type[CAPI_MAX_CHANNELS_V2];
   uint32_t ref_count;

} spf_circ_buf_mf_info_t;

#include "spf_begin_pragma.h"
typedef struct spf_circ_buf_chunk_t
{
   uint8_t                    id;         /** Index of chunk memory */
   uint16_t                   size;       /** Size of this chunk memory */
   uint16_t                   num_frames; /** Index of chunk memory */
   uint16_t                   frame_size;
   spf_circ_buf_chunk_flags_t flags; /* if the chunk needs to be recreated based on mf*/
   spf_circ_buf_mf_info_t *   mf;
   uint8_t                    num_channels;
   int8_t *                   buf_ptr[0]; /** Pointer to the start of this chunk memory*/
} spf_circ_buf_chunk_t
#include "spf_end_pragma.h"
   ;

typedef struct spf_circ_buf_client_t
{
   uint32_t        id;
   bool_t          is_read_client;              // Flag to indicate if the client is read/write
   spf_circ_buf_t *circ_buf_ptr;                // Handle to the circular buffer of the client.
   uint32_t        init_base_buffer_size_in_us; // base buffer size as requested at the client registration.

   spf_circ_buf_position_t rw_pos;

   uint32_t unread_bytes; // number of unread bytes  //TODO: timestamp offset can be used.

   uint32_t                dynamic_resize_req_in_us; // This is the resize(in bytes) request by the clients.
   uint32_t                resize_token;             // Resize token
   spf_circ_buf_mf_info_t *operating_mf;             // media format of the input data
   uint32_t                container_frame_size_in_us;
} spf_circ_buf_client_t;

typedef struct spf_circ_buf_raw_client_t
{
   uint32_t id;

   /* Flag to indicate if the client is read/write */
   bool_t is_read_client;

   /* Handle to the circular buffer of the client. */
   spf_circ_buf_raw_t *circ_buf_raw_ptr;

   /* Base buffer size requested by current client during registration */
   uint32_t init_base_buffer_size;

   /* Current position in circular buffer */
   spf_circ_buf_position_t rw_pos;

   /* MTU size or max raw data len */
   uint32_t raw_frame_len;

   /* Currently applicable to write client only.
    * Number of encoded frames in current input buffer
    * that is sent to the write client. */
   uint8_t num_frames_in_cur_buf;

   /* Currently applicable to read client only.
    * Number of encoded frames in circular buffer */
   uint32_t unread_num_frames;

   /* Currently applicable to read client only.
    * Number of encoded bytes max data len in current
    * input buffer that is actual data len / MTU */
   uint32_t unread_bytes_max;

   /* Currently applicable to read client only.
    * Number of encoded bytes actual data len in
    * the circular buffer*/
   uint32_t unread_bytes;

} spf_circ_buf_raw_client_t;

#define SPF_CIRC_BUF_EVENT_ID_OUTPUT_MEDIA_FORMAT 0xE001

typedef capi_err_t (*spf_event_cb_f)(void *          context_ptr,
                                     spf_circ_buf_t *circ_buf_ptr,
                                     uint32_t        event_id,
                                     void *          event_info_ptr);

typedef struct spf_circ_buf_event_cb_info_t spf_circ_buf_event_cb_info_t;

/** Payload of the #circular buffer EVENT_CALLBACK_INFO property.
 */
struct spf_circ_buf_event_cb_info_t
{
   spf_event_cb_f event_cb; /**< Callback function used to raise an
                                     event. */
   void *event_context;     /**< Opaque pointer value used as the context
                                 for this callback function. */
};

typedef struct spf_circ_buf_t
{
   uint32_t id;
   uint32_t circ_buf_size_in_us; // This is the requested/expected ciruclar buffer size by the client.
                                 // circular buffer size in us (base_buffer_size + max_req_alloc_size)
                                 // **This field must be updated only in the context of client resize request.
                                 // *Note:
                                 // The actual buffer size in bytes (circ_buf_size_bytes) may be more than the requested
                                 // size in us, during media format transition. circ_buf_size_bytes depends upon the
                                 // chunks present currently in the buffer, which includes old mf chunks that are marked
                                 // for destroy during mf transition.

   uint32_t circ_buf_size_bytes; // This is the actual buffer size in bytes based on the no.of chunks currently present
                                 // in the buffer. Note that during media format tranition it will be more than the
                                 // circ_buf_size_in_us, because of presence of chunks of old media format. But in
                                 // steady state, bytes should be <=> circ_buf_size_in_us (as requested by the client.)

   POSAL_HEAP_ID heap_id; // Heap id for memory allocations.

   uint32_t         preferred_chunk_size; // preferred size in bytes for buffer chunks
   uint32_t         num_chunks;           // Size of the buffer_ptr array of mem addresses
   spf_list_node_t *head_chunk_ptr;       // Array of non-contiguous memory chunks

   spf_circ_buf_client_t *wr_client_ptr; // Holds the pointer to the writer client.
                                         // There is only one writer client.

   uint32_t write_byte_counter; // This counter is incremented for every byte written into circular buffer

   uint32_t         num_read_clients;
   spf_list_node_t *rd_client_list_ptr; // List of all registered clients with the buffer.

   uint32_t is_valid_timestamp; // Flag to indicate if the timestamp is valid.
   uint16_t unique_id_counter;

   spf_circ_buf_event_cb_info_t           cb_info;
   intf_extn_param_id_metadata_handler_t *metadata_handler; // pointer to clients[capi module] metadata handler.
   spf_list_node_t *                      mf_list_ptr;

   /** scratch buffer array used for read/write loops */
   capi_buf_t scratch_buf_arr[CAPI_MAX_CHANNELS_V2];

} spf_circ_buf_t;

typedef struct spf_circ_buf_raw_t
{
   /* ID of the raw ciruclar buffer */
   uint32_t id;

   /* This is the actual buffer size in bytes including the frame headers
    * based on the # chunks and actual chunk size. */
   uint32_t circ_buf_size_bytes;

   /* This is the buffer size based on number of frames and number of chunks
    * without the headers and other metadata */
   uint32_t circ_buf_size;

   /* Heap id for memory allocations. */
   POSAL_HEAP_ID heap_id;

   /* Prefered chunk size - this is only a recommendation in case of
    * raw compressed since we do not value partial data and do not
    * know the frame size and use MTU */
   uint32_t preferred_chunk_size;

   /* Number of chunks present currently */
   uint32_t num_chunks;

   /* Header of the list of chunk pointers */
   spf_list_node_t *head_chunk_ptr;

   /* Writer client*/
   spf_circ_buf_raw_client_t *wr_raw_client_ptr;

   /* Total actual data len written*/
   uint32_t write_byte_counter;

   /* Total data written with gaps (MTU frames)*/
   uint32_t write_byte_counter_max;

   /* The maximum raw data bytes that is required
    * to be stored in the raw circular buffer based on
    * the bitrate.*/
   uint32_t circ_buf_raw_size_max;

   /* Maximum number of raw encoded frames required to
    * store based on frame duration / size if mentioned
    * with sampling rate of decoder */
   uint32_t num_frames_max;

   /* The hard limit on circular raw buffer based on LPI
    * limitations etc*/
   uint32_t max_raw_circ_buf_bytes_limit; // This is the absolute max irrespective of how much actual data len is stored

   /* Number of readers - currently used = 1 */
   uint32_t num_read_clients;

   /* Reader client list pointer */
   spf_list_node_t *rd_client_list_ptr;

   /* Unique id of every new chunk created. */
   uint16_t unique_id_counter;

   /* Metadata handle for raw circular buffer */
   intf_extn_param_id_metadata_handler_t *metadata_handler;

   /** scratch buffer array used for read/write loops */
   capi_buf_t scratch_buf_arr[CAPI_MAX_CHANNELS_V2];

} spf_circ_buf_raw_t;

typedef struct spf_circ_buf_alloc_inp_args_t
{
   uint32_t                               preferred_chunk_size;
   intf_extn_param_id_metadata_handler_t *metadata_handler;
   uint32_t                               buf_id;
   POSAL_HEAP_ID                          heap_id;
   spf_circ_buf_event_cb_info_t           cb_info;

} spf_circ_buf_alloc_inp_args_t;

typedef struct spf_circ_buf_raw_alloc_inp_args_t
{
   /* spf_circ_buf_raw_t::preferred_chunk_size */
   uint32_t preferred_chunk_size;

   /* spf_circ_buf_raw_t::metadata_handler */
   intf_extn_param_id_metadata_handler_t *metadata_handler;

   /* spf_circ_buf_raw_t::id */
   uint32_t buf_id;

   /* spf_circ_buf_raw_t::num_frames_max */
   uint32_t max_num_frames;

   /* spf_circ_buf_raw_t::circ_buf_raw_size_max */
   uint32_t max_raw_circ_buf_size_bytes;

   /* spf_circ_buf_raw_t::max_raw_circ_buf_bytes_limit */
   uint32_t max_raw_circ_buf_bytes_limit;

   /* spf_circ_buf_raw_t::heap_id */
   POSAL_HEAP_ID heap_id;
} spf_circ_buf_raw_alloc_inp_args_t;

/** Enumerations for return values of circular buffer operations */

typedef uint32_t spf_circ_buf_result_t;

/** Operation was successful */
#define SPF_CIRCBUF_SUCCESS 0x0

/** General failure */
#define SPF_CIRCBUF_FAIL 0x1

/** Buffer overflowed */
#define SPF_CIRCBUF_OVERRUN 0x2

/** Buffer underflowed */
#define SPF_CIRCBUF_UNDERRUN 0x3

/*==============================================================================
   Function declarations
==============================================================================*/

/*
 * Initializes the Circular buffer
 * circ_buf_ptr[in/out]         : pointer to a circular buffer structure memory client is expected to allocate this
 * memory.
 * spf_circ_buf_alloc_inp_args_t[in]     : Structure containing input arguments.
 * return : spf_circ_buf_result_t
 */
spf_circ_buf_result_t spf_circ_buf_init(spf_circ_buf_t **circ_buf_ptr, spf_circ_buf_alloc_inp_args_t *inp_args);

/*
 * Initializes the Read/Write client handle for the given circular buffer.
 *
 * spf_circ_buf_t[in/out]      : pointer to a circular buffer to which client is registering.
 * is_read_client[in]           : Total circular buffer size in bytes
 * req_base_buffer_size_us[in]  : Base buffer resize request by the client. Usually reader clients request based on
 * downstream delay. ret_handle[in/out]           : Initializes the client ptr. Callers must allocate memory and pass
 * this pointer. functionality :
 *  1. Registers Read/write client to the circular buffer handle provided.
 *  2. Resized based on max base buffer resize requests from all the clients.
 * return : spf_circ_buf_result_t
 */
spf_circ_buf_result_t spf_circ_buf_register_reader_client(spf_circ_buf_t *        circ_buf_struct_ptr,
                                                          uint32_t                req_base_buffer_size_us,
                                                          spf_circ_buf_client_t **ret_handle);

/*
 * Initializes the Read/Write client handle for the given circular buffer.
 *
 * spf_circ_buf_t[in/out]      : pointer to a circular buffer to which client is registering.
 * is_read_client[in]           : Total circular buffer size in bytes
 * req_base_buffer_size_us[in]  : Base buffer resize request by the client. Usually reader clients request based on
 * downstream delay. ret_handle[in/out]           : Initializes the client ptr. Callers must allocate memory and pass
 * this pointer. functionality :
 *  1. Registers Read/write client to the circular buffer handle provided.
 *  2. Resized based on max base buffer resize requests from all the clients.
 * return : spf_circ_buf_result_t
 */
spf_circ_buf_result_t spf_circ_buf_register_writer_client(spf_circ_buf_t *        circ_buf_struct_ptr,
                                                          uint32_t                req_base_buffer_size_us,
                                                          spf_circ_buf_client_t **ret_handle);

/*
 * Writes a frame [container_frame_size] from the circular buffer.
 * Propagates metadata if a valid "sdata_ptr" is passed.
 *
 * spf_circ_buf_client_t[in/out]   : Writer clients handle
 * sdata_ptr[in/out]              : Input capi stream data pointer.
 * allow_overflow                 : TRUE means that if the buffer size is exceeded, old data is overwritten.
 *                                  FALSE means that any input data that doesn't fit in the circular buffer
 *                                  will be marked as unconsumed.
 *
 * Functionality :
 * 1. Copies data and metadata frame from Input capi to a new frame in circular buffer.
 * 2. Maximum data that will be written is limited by MIN( container_frame_size , buf_ptr->actual_data_len )
 * 3. The buf_ptr->actual_data_len is updated based on the data consumed by the circular buffer.
 * 4. If a valid sdata_ptr is provided, it will buffered along with the circualr buffer frame.
 *
 * return : spf_circ_buf_result_t
 */
spf_circ_buf_result_t spf_circ_buf_write_one_frame(spf_circ_buf_client_t *wr_handle,
                                                   capi_stream_data_t *   sdata_ptr,
                                                   bool_t                 allow_overflow);

/*
 * Reads a frame [container_frame_size] from the circular buffer.
 * Propagates metadata if a valid "sdata_ptr" is passed.
 *
 * spf_circ_buf_client_t[in/out]   : Reader clients handle
 * buf_ptr[in/out]                : Output capi buffer pointer. Must have valid max_data_len.
 * sdata_ptr[in/out]                  : Output capi stream data pointer.
 *
 * Functionality :
 *  1.Copies data frame from circular buffer to capi output buffer. W
 *  2. Will return SPF_CIRC_BUF_UNDERRUN if the buffer is underrun.
 *  3. Maximum data that will be read is limited by MIN( container_frame_size , buf_ptr->max_data_len )
 *  4. Sometimes data may be read less than the container_frame_size due to flags/metadata thats being propagated.
 *
 * return : spf_circ_buf_result_t
 */
spf_circ_buf_result_t spf_circ_buf_read_one_frame(spf_circ_buf_client_t *rd_handle, capi_stream_data_t *sdata_ptr);

/*
 * Writes input data into circular buffer but doesn't buffer metadata.
 * Can be used in scenarios where metadata is handled by the client layer itself.
 *
 * spf_circ_buf_client_t[in/out]   : Writer clients handle
 * buf_ptr[in/out]                : Input capi buffer pointer. Must have valid max_data_len and actual data lens.
 * allow_overflow                 : TRUE means that if the buffer size is exceeded, old data is overwritten.
 *                                  FALSE means that any input data that doesn't fit in the circular buffer
 *                                  will be marked as unconsumed.
 * Functionality :
 * Writes data into the circular buffer, will return SPF_CIRC_BUF_OVERRUN if the buffer is overflown.
 *
 * Amount of data written depends upon "buf_ptr->actual_data_len"
 *
 * return : spf_circ_buf_result_t
 */
spf_circ_buf_result_t spf_circ_buf_write_data(spf_circ_buf_client_t *wr_handle,
                                              capi_stream_data_t *   in_sdata_ptr,
                                              bool_t                 allow_overflow);
/*
 * Reads data from circular buffer but doesn't propagate metadata.
 * Can be used in scenarios where metadata is handled by the client layer itself.
 *
 * spf_circ_buf_client_t[in/out]   : Readers clients handle
 * buf_ptr[in/out]                : Output capi buffer pointer. Must have valid max_data_len.
 *
 * Functionality :
 * Reads data from the circular buffer, will return SPF_CIRC_BUF_UNDERRUN if the buffer is underrun.
 *
 * Amount of data reads depends upon "buf_ptr->max_data_len"
 *
 * return : spf_circ_buf_result_t
 */
spf_circ_buf_result_t spf_circ_buf_read_data(spf_circ_buf_client_t *rd_client_ptr, capi_stream_data_t *out_sdata_ptr);

/*
 * Adjusts the read pointer
 * rd_handle[in/out]            : pointer to the reader handle
 * read_offset[in]              : Read offset as requested by the client. But actual adjusted offset depends on the
 * un read bytes.
 * actual_adj_offset_ptr[in_out]  : Updates actual number of bytes adjusted in the ptr memory.
 *
 * functionality :
 *  Moves the Reader clients position by MIN(read_offset, un_read_bytes) and returns the
 *  actual adjusted offset to the client.
 *
 * return : circbuf_result
 */
spf_circ_buf_result_t spf_circ_buf_read_adjust(spf_circ_buf_client_t *rd_handle,
                                               uint32_t               read_offset,
                                               uint32_t *             actual_adj_offset_ptr);

/*
 * Read client request to resize the buffer.
 * rd_client_ptr[in/out]     : pointer to a reader clients handle.
 * req_buffer_resize[in]         : Requested buffer resize.
 *
 * functionality :
 * Aggregates all reader clients resize requests and then expands/shrinks the circular buffer based on the current max
 * request.
 *
 * return : circbuf_result
 */
spf_circ_buf_result_t spf_circ_buf_read_client_resize(spf_circ_buf_client_t *rd_client_ptr,
                                                      uint32_t               req_buffer_resize_in_us);

/*
 * Memsets "num_bytes" by "memset_value".
 *
 * wr_handle[in/out]   : Writer clients handle
 * allow_overflow      : TRUE means that if the buffer size is exceeded, old data is overwritten.
 *                       FALSE means that any input data that doesn't fit in the circular buffer
 *                       will be marked as unconsumed.
 * num_bytes[in]       : NUmber of bytes to memeset from the current writer clients position.
 * memset_value[in]    : Value that needs to used for memset.
 *
 * Functionality :
 * Memset num_bytes with a value.
 *
 * return : spf_circ_buf_result_t
 */
spf_circ_buf_result_t spf_circ_buf_memset(spf_circ_buf_client_t *wr_client_ptr,
                                          bool_t                 allow_overflow,
                                          uint32_t               num_bytes,
                                          uint32_t               memset_value);

/*
 * Deregisters the Read/Write client from the earlier registered circular buffer.
 *
 * client_ptr[in/out]  : De register the client from the circular buffer.
 *
 * functionality :
 * De-registers the read/write clients.
 *
 * Resizes the circular buffer by aggregating resize requests of the left over clients.
 * May result in Shrinking of the circular buffer.
 *
 * return : circbuf_result
 */
spf_circ_buf_result_t spf_circ_buf_deregister_client(spf_circ_buf_client_t **client_hdl_pptr);

/*
 * Destroys the ciruclar buffer.
 *
 * circ_buf_ptr[in/out]  : pointer to a circular buffer structure
 *
 * functionality :
 * 1.  Frees any metadata that was buffered in the chunks.
 * 2.  Destroys all the circular buffer chunks.
 * 2.  s all the read/write clients for the circular buffer. Avoiding further read/writes from the client.
 *
 * return : circbuf_result
 */
spf_circ_buf_result_t spf_circ_buf_deinit(spf_circ_buf_t **circ_buf_pptr);

/*
 * Sets input media format, data written following this set param is expected to have this media format.
 * return : spf_circ_buf_result_t
 */
spf_circ_buf_result_t spf_circ_buf_set_media_format(spf_circ_buf_client_t *wr_client_ptr,
                                                    capi_media_fmt_v2_t *  mf,
                                                    uint32_t               container_frame_size_in_us);

/*
 * Get output media format, data read following this get param is expected to be of this media format.
 */
spf_circ_buf_result_t spf_circ_buf_get_media_format(spf_circ_buf_client_t *rd_client_ptr, capi_media_fmt_v2_t *mf);

/**
 * Checks if the circular buffer is empty (no unread bytes and no metadata).
 */
spf_circ_buf_result_t spf_circ_buf_driver_is_buffer_empty(spf_circ_buf_client_t *rd_hdl_ptr, bool_t *is_empty_ptr);

/*
 * Get amount of un read bytes left in the circular buffer per channel.
 */
spf_circ_buf_result_t spf_circ_buf_get_unread_bytes(spf_circ_buf_client_t *rd_handle, uint32_t *unread_bytes);

/* Checks if the stream buffer has any empty space by comparing unread_bytes to the circular buffer size.
 */
spf_circ_buf_result_t spf_circ_buf_driver_is_buffer_full(spf_circ_buf_client_t *rd_hdl_ptr, bool_t *is_full_ptr);

/* Init the raw circular buffer with input argurementes */
spf_circ_buf_result_t spf_circ_buf_raw_init(spf_circ_buf_raw_t **              circ_buf_raw_pptr,
                                            spf_circ_buf_raw_alloc_inp_args_t *inp_args);

/* Write data into the raw circular buffer */
spf_circ_buf_result_t spf_circ_buf_raw_write_data(spf_circ_buf_raw_client_t *wr_handle,
                                                  capi_stream_data_t *       in_sdata_ptr,
                                                  bool_t                     allow_overflow);

/* Register raw reader client and reset */
spf_circ_buf_result_t spf_circ_buf_raw_register_reader_client(spf_circ_buf_raw_t *        circ_buf_raw_ptr,
                                                              uint32_t                    req_base_buffer_size_bytes,
                                                              spf_circ_buf_raw_client_t **client_hdl_pptr);

/* Register raw writer client and reset */
spf_circ_buf_result_t spf_circ_buf_raw_register_writer_client(spf_circ_buf_raw_t *        circ_buf_struct_ptr,
                                                              uint32_t                    req_base_buffer_bytes,
                                                              spf_circ_buf_raw_client_t **ret_handle);

/* Write one MTU frame into raw circ buffer at a time */
spf_circ_buf_result_t spf_circ_buf_raw_write_one_frame(spf_circ_buf_raw_client_t *wr_handle,
                                                       capi_stream_data_t *       sdata_ptr,
                                                       bool_t                     allow_overflow);

/* Read one MTU frame from raw circ buffer at a time */
spf_circ_buf_result_t spf_circ_buf_raw_read_one_frame(spf_circ_buf_raw_client_t *rd_client_ptr,
                                                      capi_stream_data_t *       sdata_ptr);
spf_circ_buf_result_t spf_circ_buf_raw_write_data(spf_circ_buf_raw_client_t *wr_handle,
                                                  capi_stream_data_t *       in_sdata_ptr,
                                                  bool_t                     allow_overflow);

/*Memset Raw Circ Buffer*/
spf_circ_buf_result_t spf_circ_buf_raw_memset(spf_circ_buf_raw_client_t *wr_client_ptr,
                                              bool_t                     allow_overflow,
                                              uint32_t                   num_bytes,
                                              uint32_t                   memset_value);

/* Deinit and destroy raw circular buffer, clients and other handles */
spf_circ_buf_result_t spf_circ_buf_raw_deinit(spf_circ_buf_raw_t **circ_buf_raw_pptr);

/* Verify if raw format and set circular buffer */
spf_circ_buf_result_t spf_circ_buf_raw_set_media_format(spf_circ_buf_raw_client_t *wr_client_ptr,
                                                        capi_cmn_raw_media_fmt_t * mf);

/* Resize and create the raw circular buffer as per requirement */
spf_circ_buf_result_t spf_circ_buf_raw_resize(spf_circ_buf_raw_client_t *wr_raw_handle, uint32_t raw_frame_len);

/* Check if the raw circular buffer is empty */
spf_circ_buf_result_t spf_circ_buf_raw_driver_is_buffer_empty(spf_circ_buf_raw_client_t *rd_hdl_ptr,
                                                              bool_t *                   is_empty_ptr);

/* Get number of unread bytes (actual data) and unread bytes (max data) */
spf_circ_buf_result_t spf_circ_buf_raw_get_unread_bytes(spf_circ_buf_raw_client_t *rd_handle,
                                                        uint32_t *                 unread_bytes,
                                                        uint32_t *                 unread_bytes_max);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*__CIRC_BUF_UTILS_H*/
