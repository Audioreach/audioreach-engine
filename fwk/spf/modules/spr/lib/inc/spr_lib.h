#ifndef _SPR_LIB_H
#define _SPR_LIB_H

/**
 *   \file spr_lib.h
 *   \brief
 *        This file contains utility functions for handling circular buffering
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_defs.h"
#include "posal.h"
#include "capi_types.h"
#include "spf_list_utils.h"
#include "circular_buffer.h"

#define CAPI_AUDIO_DAM_MAX_READ_CLIENTS 8
#define CAPI_AUDIO_DAM_MAX_INPUT_CHANNELS 32

/*==============================================================================
   Function declarations
==============================================================================*/

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

typedef struct spr_driver_t spr_driver_t;

struct spr_driver_t
{
   uint32_t driver_instance_id;
   /*Driver instance ID */

   uint32_t num_writers;
   /* Numbers of writers created */

   spf_list_node_t *stream_writer_list;
   /* List of all the stream writers. */

   uint32_t num_readers;
   /* Numbers of readers created */

   spf_list_node_t *stream_reader_list;
   /* List of all the stream writers. */

   POSAL_HEAP_ID heap_id;
   /* Heap ID for memory allocations, passed at init.*/

   uint32_t preferred_chunk_size;
   /* Preferred chunk size. */
};

typedef struct spr_stream_reader_t
{
   uint16_t id;
   /* Identifier for the stream reader.*/

   uint16_t num_channels;
   /* Number of channels in the output reader stream*/

   uint32_t req_base_bufffer_size;
   /* This the additional base size requested by the reader client at the time of reader create.*/

   uint32_t req_buf_resize;
   /* This is the latest resize request by the reader client. [Usually a detection engine]*/

   uint32_t rel_sync_offset_in_bytes;
   /* Relative sync offset in bytes, used to synchronize the processed and raw channels data */

   circ_buf_client_t *rd_client_arr_ptr;
   /* Array of client handles for the channel's circular buffers. Arrays size is num_channels.*/

} spr_stream_reader_t;

typedef struct spr_stream_writer_t
{
   uint16_t id;
   /* Identifier for the stream writer.*/

   uint16_t num_channels;
   /* Number of channels in the input writer stream*/

   uint32_t base_buffer_size;
   /*Base buffer size as requested by the writer client at the time of buffer creation.*/

   uint32_t pre_buffering_delay_in_bytes; // TODO: remove this field.
   /*Additional stream Processing delay between source module and buffer module.*/

   circ_buf_t *circ_buf_arr_ptr;
   /* Array of handles to the channel circular buffers. Arrays size is num_channels.*/

   circ_buf_client_t *wr_client_arr_ptr;
   /* Array of circular buffer clients handles . Arrays size is num_channels.*/

} spr_stream_writer_t;

/*
 * Initializes the SPR driver.
 *
 * heap_id[in]                : Heap ID.
 * drv_ptr[in/out]            : Pointer to the driver pointer. Handle is created and returned.
 * driver_instance_id[in]     : Driver instance ID
 *
 * functionality : Initializes the handles for the driver. Driver handles is
 * needed for subsequent stream writer/reader creation.
 * return : ar_result_t
 */
ar_result_t spr_driver_init(POSAL_HEAP_ID heap_id, spr_driver_t **drv_ptr, uint32_t driver_instance_id);

/*
 * De initializes the SPR driver. If there are any stream/writers present
 * they will be destroyed.
 *
 * spr_driver_t[in/out]   : Destroys handles and update the pointer to NULL.
 * driver_instance_id[in]       : Driver instance to be destroyed [Currently dont care.].
 *
 * return : ar_result_t
 */
ar_result_t spr_driver_deinit(spr_driver_t **drv_ptr_pptr, uint32_t driver_instance_id);

/*
 * Creates circular buffers for each channel in the input port stream.
 *
 * circ_buf_struct_ptr[in/out]  : pointer to a circular buffer structure
 * circ_buf_size[in]            : Total circular buffer size in bytes
 *
 * functionality : Allocates non-contiguous buffer and initializes the circular
 * buffer parameters for each channel of the input stream. The memory chunks are
 * multiple memory allocations, each smaller than the circular buffer size.
 * The sum of the memory chunk sizes equals the circular buffer size.
 *
 * return : circbuf_result
 */
ar_result_t spr_stream_writer_create(spr_driver_t *        drv_ptr,
                                     uint32_t              base_buffer_size,
                                     uint32_t              pre_buffering_delay_in_ms,
                                     uint32_t              num_channels,
                                     uint32_t *            ch_id_arr,
                                     spr_stream_writer_t **writer_handle_pptr);

/*
 * Creates circular buffers for each channel in the input port stream.
 * when the media format is received.
 *
 * circ_buf_struct_ptr[in/out]  : pointer to a circular buffer structure
 * circ_buf_size[in]            : Total circular buffer size in bytes
 *
 * functionality : Allocates non-contiguous buffer and initializes the circular
 * buffer parameters for each channels. The memory chunks are multiple memory allocations, each
 * smaller than the circular buffer size. The sum of the memory chunk sizes
 * equals the circular buffer size.
 * return : circbuf_result
 */
ar_result_t spr_stream_writer_destroy(spr_driver_t *drv_ptr, spr_stream_writer_t **writer_handle);

/*
 * Creates circular buffers for each channel in the input port stream.
 * when the media format is received.
 *
 * circ_buf_struct_ptr[in/out]  : pointer to a circular buffer structure
 * circ_buf_size[in]            : Total circular buffer size in bytes
 *
 * functionality : Allocates non-contiguous buffer and initializes the circular
 * buffer parameters for each channels. The memory chunks are multiple memory allocations, each
 * smaller than the circular buffer size. The sum of the memory chunk sizes
 * equals the circular buffer size.
 * return : circbuf_result
 */
ar_result_t spr_stream_reader_create(spr_driver_t *        drv_ptr,
                                     uint32_t              downstream_setup_duration_in_bytes,
                                     uint32_t              num_channels,
                                     uint32_t *            ch_id_arr,
                                     spr_stream_reader_t **reader_handle_pptr);

/*
 * Creates circular buffers for each channel in the input port stream.
 * when the media format is received.
 *
 * circ_buf_struct_ptr[in/out]  : pointer to a circular buffer structure
 * circ_buf_size[in]            : Total circular buffer size in bytes
 *
 * functionality : Allocates non-contiguous buffer and initializes the circular
 * buffer parameters for each channels. The memory chunks are multiple memory allocations, each
 * smaller than the circular buffer size. The sum of the memory chunk sizes
 * equals the circular buffer size.
 * return : circbuf_result
 */
ar_result_t spr_stream_reader_destroy(spr_driver_t *drv_ptr, spr_stream_reader_t **reader_handle);

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
ar_result_t spr_stream_write(spr_stream_writer_t *writer_handle,
                             capi_buf_t *         input_buf_arr,
                             uint32_t             is_valid_timestamp,
                             int64_t              timestamp);

/*
 * Read samples from the Circular buffer
 * circ_buf_struct_ptr[in/out]  : pointer to a circular buffer structure
 * out_ptr[out]                 : Pointer to where data has to be read from
 *                                circular buffer
 * bytes_to_read[in]            : number of bytes to be read
 * functionality : copies the samples_to_read number of samples to out_ptr
 * return : circbuf_result
 */
ar_result_t spr_stream_read(spr_stream_reader_t *reader_handle, capi_buf_t *output_buf_arr);

/*
 * Adjusts the read pointer
 * circ_buf_struct_ptr[in/out]  : pointer to a circular buffer structure
 * read_offset[in]              : offset needed between new read_index and
 *                                current write_index
 * functionality : Moves the read_index to a new position to start reading from
 * write_index - read_offset
 * return : circbuf_result
 */
ar_result_t spr_stream_read_adjust(spr_stream_reader_t *reader_handle,
                                   uint32_t             read_offset,
                                   uint32_t             bytes_per_one_ms,
                                   uint32_t *           actual_read_offset);

/*
 * Adjusts the read pointer posistion of all the channels
 * capi_spr_buffer_t[in/out]  : pointer to a circular buffer structure
 * spr_out_port_t[in]            : output port handle.
 * functionality : Moves the read_index to a new position to start reading from
 * write_index - read_offset
 * return : circbuf_result
 */

ar_result_t spr_stream_reader_req_resize(spr_stream_reader_t *reader_handle,
                                         uint32_t             requested_alloc_size,
                                         uint32_t             is_register);

/*
 * Creates circular buffers for each channel in the input port stream.
 * when the media format is received.
 *
 * circ_buf_struct_ptr[in/out]  : pointer to a circular buffer structure
 * circ_buf_size[in]            : Total circular buffer size in bytes
 *
 * functionality : Allocates non-contiguous buffer and initializes the circular
 * buffer parameters for each channels. The memory chunks are multiple memory allocations, each
 * smaller than the circular buffer size. The sum of the memory chunk sizes
 * equals the circular buffer size.
 * return : ar_result_t
 */
ar_result_t spr_driver_set_chunk_size(spr_driver_t *drv_ptr, uint32_t preferred_chunk_size);

/*
 * Queries the circular buffer for number of unread bytes given the stream reader
 *
 * unread_bytes_per_ch_ptr[in/out] : pointer to the value of unread bytes
 * reader_handle[in]               : pointer to the stream reader handle
 *
 * return : ar_result_t
 */
ar_result_t spr_stream_reader_get_unread_bytes(spr_stream_reader_t *reader_handle, uint32_t *unread_bytes_per_ch_ptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*_SPR_LIB_H*/
