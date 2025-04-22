/*==========================================================================
 Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/*
 * @file capi_ffns_utils.h
 *
 * capiv2 interface for ffns structures
 */

#ifndef CAPI_BUFFER_UTILS_H
#define CAPI_BUFFER_UTILS_H

// include files
#include "capi_types.h"
#include "shared_lib_api.h"
#include "capi_types.h"
#include "capi_cmn.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "posal.h"

#ifdef __cplusplus
}
#endif

#ifndef MIN
#define MIN(x,y)        (((x)<(y)) ? (x) : (y))
#endif

typedef struct capi_library_internal_buf_flags_t
{
   uint32_t is_media_fmt_received : 1;
   uint32_t frame_size_recieved   : 1;
   uint32_t frame_size_in_time    : 1;  // FALSE -> frame size is specified in samples.
} capi_library_internal_buf_flags_t;

/* Internal buffers for translating capi frame size to lib frame size. */
typedef struct capi_library_internal_buf_t
{
   int8_t **data_ptrs;

   // Ensure all channels are same length. Storing per_ch saves divisions during process.
   uint32_t actual_data_len_per_ch;
   uint32_t max_data_len_per_ch;

   // For output buffer, the amount of data already copied from lib to capi. Stored here to avoid
   // moving partial data to the top of the buffer.
   uint32_t data_offset_bytes_per_ch;

   // Media format of the internal buffer. Anytime there is a change, it should be updated
   // using capi_library_internal_buffer_set_media_fmt() instead of updating inline.
   capi_media_fmt_v2_t media_fmt;

   uint32_t frame_size_us;             // The frame size the library operates on. This also
                                       // determines the internal buffer sizes.
   uint32_t frame_size_samples_per_ch; // Storing this to avoid division during process().

   POSAL_HEAP_ID heap_id;

   capi_library_internal_buf_flags_t flags;
} capi_library_internal_buf_t;

#ifdef __cplusplus
extern "C" {
#endif

capi_err_t capi_library_internal_buffer_create(capi_library_internal_buf_t *int_buf_ptr, POSAL_HEAP_ID heap_id);
void capi_library_internal_buffer_destroy(capi_library_internal_buf_t *int_buf_ptr);

bool_t capi_library_internal_buffer_exists(capi_library_internal_buf_t *int_buf_ptr);

void capi_library_internal_buffer_set_media_fmt(capi_library_internal_buf_t *int_buf_ptr,
                                             capi_media_fmt_v2_t *     data_format_ptr);
void capi_library_internal_buffer_set_frame_size_us(capi_library_internal_buf_t *int_buf_ptr, uint32_t frame_size_us);
void capi_library_internal_buffer_set_frame_size_samples(capi_library_internal_buf_t *int_buf_ptr,
                                                      uint32_t                  frame_size_samples_per_ch);


void capi_library_internal_buffer_flush(capi_library_internal_buf_t *int_buf_ptr);
uint32_t capi_library_internal_buffer_zero_fill(capi_library_internal_buf_t *int_buf_ptr, uint32_t zero_bytes_per_ch);

uint32_t capi_library_internal_buffer_read(capi_library_internal_buf_t *int_buf_ptr, capi_stream_data_v2_t *dst_sdata_ptr);
uint32_t capi_library_internal_buffer_write(capi_library_internal_buf_t *int_buf_ptr,
                                         capi_stream_data_v2_t *   src_sdata_ptr,
                                         uint32_t                  src_offset_bytes_per_ch);


#ifdef __cplusplus
}
#endif

#endif /* CAPI_BUFFER_UTILS_H */