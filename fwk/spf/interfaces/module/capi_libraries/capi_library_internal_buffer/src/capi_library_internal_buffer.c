/*==========================================================================
 Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 SPDX-License-Identifier: BSD-3-Clause-Clear
* =========================================================================*/

/*
* @file capi_ffns.cpp
*
* capiv2 implementation
*/

/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/
#include "capi_library_internal_buffer.h"

/**
 * Returns TRUE if input and output internal buffers are allocated.
 */
bool_t capi_library_internal_buffer_exists(capi_library_internal_buf_t *int_buf_ptr)
{
   return (NULL != (int_buf_ptr->data_ptrs));
}

/**
 * Allocates memory for the circular buffer based on media format and frame size.
 */
capi_err_t capi_library_internal_buffer_create(capi_library_internal_buf_t *int_buf_ptr, POSAL_HEAP_ID heap_id)
{
   capi_err_t result           = CAPI_EOK;
   uint32_t   num_ch           = int_buf_ptr->media_fmt.format.num_channels;
   uint32_t   bytes_per_sample = int_buf_ptr->media_fmt.format.bits_per_sample >> 3;
   uint32_t   bytes_per_ch     = int_buf_ptr->frame_size_samples_per_ch * bytes_per_sample;
   uint32_t   total_size       = bytes_per_ch * num_ch;
   int_buf_ptr->heap_id        = heap_id;

   if (0 == total_size)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Not ready to create internal buffer num ch %ld samples per ch %ld.",
             num_ch,
             int_buf_ptr->frame_size_samples_per_ch);
      return CAPI_EFAILED;
   }

   if (capi_library_internal_buffer_exists(int_buf_ptr))
   {
      capi_library_internal_buffer_flush(int_buf_ptr);
      // Ensure old memory is freed if it exists.
      capi_library_internal_buffer_destroy(int_buf_ptr);
   }

   // Internal buffer allocation.
   uint32_t data_ptrs_size = (sizeof(int8_t *) * num_ch);
   int_buf_ptr->data_ptrs  = (int8_t **)posal_memory_malloc(data_ptrs_size, int_buf_ptr->heap_id);
   if (NULL == int_buf_ptr->data_ptrs)
   {
      AR_MSG(DBG_ERROR_PRIO, "Couldn't allocate memory for internal data pointers size %ld.", data_ptrs_size);
      return CAPI_ENOMEMORY;
   }
   // Malloc all channel pointers in one shot at 0th channel. For rest of channels, set pointers accordingly.
   int_buf_ptr->data_ptrs[0] = (int8_t *)posal_memory_malloc(total_size, int_buf_ptr->heap_id);
   if (NULL == int_buf_ptr->data_ptrs[0])
   {
      AR_MSG(DBG_ERROR_PRIO, "Couldn't allocate memory for internal buffer size %ld.", total_size);
      return CAPI_ENOMEMORY;
   }

   int8_t *data_ptr = int_buf_ptr->data_ptrs[0];
   for (uint32_t ch_idx = 1; ch_idx < num_ch; ch_idx++)
   {
      data_ptr += bytes_per_ch;
      int_buf_ptr->data_ptrs[ch_idx] = data_ptr;
   }

   int_buf_ptr->actual_data_len_per_ch   = 0;
   int_buf_ptr->max_data_len_per_ch      = bytes_per_ch;
   int_buf_ptr->data_offset_bytes_per_ch = 0;

   AR_MSG(DBG_HIGH_PRIO, "created internal buffers of size %d bytes, #ch %ld", total_size, num_ch);
   return result;
}

/**
 * Frees memory for the circular buffer if it exists.
 */
void capi_library_internal_buffer_destroy(capi_library_internal_buf_t *int_buf_ptr)
{
   if (int_buf_ptr->data_ptrs)
   {
      if (int_buf_ptr->data_ptrs[0])
      {
         posal_memory_free(int_buf_ptr->data_ptrs[0]);

         for (uint32_t ch_idx = 0; ch_idx < int_buf_ptr->media_fmt.format.num_channels; ch_idx++)
         {
            int_buf_ptr->data_ptrs[ch_idx] = NULL;
         }
      }
      posal_memory_free(int_buf_ptr->data_ptrs);
      int_buf_ptr->data_ptrs = NULL;

      AR_MSG(DBG_HIGH_PRIO, "capi buf util: destroyed internal buffer");
   }
}

/**
 * Writes data from the source stream data (capi input) to the buffer. Src_offset_bytes_per_ch is the
 * read offset from the sdata (in case some data was previously read from the source).
 * Returns the number of bytes copied per channel.
 */
uint32_t capi_library_internal_buffer_write(capi_library_internal_buf_t *int_buf_ptr,
                                            capi_stream_data_v2_t *      src_sdata_ptr,
                                            uint32_t                     src_offset_bytes_per_ch)
{

   uint32_t num_ch              = int_buf_ptr->media_fmt.format.num_channels;
   uint32_t bytes_per_ch_copied = 0;
   uint32_t dst_size            = int_buf_ptr->max_data_len_per_ch - int_buf_ptr->actual_data_len_per_ch;
   for (uint32_t ch = 0; ch < num_ch; ch++)
   {
      // Copy data from CAPI input to lib input.
      int8_t * dst_ptr  = int_buf_ptr->data_ptrs[ch] + int_buf_ptr->actual_data_len_per_ch;
      int8_t * src_ptr  = src_sdata_ptr->buf_ptr[ch].data_ptr + src_offset_bytes_per_ch;
      uint32_t src_size = src_sdata_ptr->buf_ptr[ch].actual_data_len - src_offset_bytes_per_ch;

      bytes_per_ch_copied = memscpy(dst_ptr, dst_size, src_ptr, src_size);
   }

   // Mark internal input buffer bytes filled.
   int_buf_ptr->actual_data_len_per_ch += bytes_per_ch_copied;

   return bytes_per_ch_copied;
}

/**
 * Reads data from the buffer into the destination stream data (capi output). Data is appended in the destination
 * stream buffer - any existing data i nthe destination will NOT be overwritten.
 * Returns the amount of data read from the buffer.
 */
uint32_t capi_library_internal_buffer_read(capi_library_internal_buf_t *int_buf_ptr,
                                           capi_stream_data_v2_t *      dst_sdata_ptr)
{
   uint32_t num_ch           = int_buf_ptr->media_fmt.format.num_channels;
   uint32_t bytes_copied_out = 0;
   uint32_t src_size         = int_buf_ptr->actual_data_len_per_ch - int_buf_ptr->data_offset_bytes_per_ch;

   // Assuming all channels have the same actual data len.
   uint32_t dst_size = dst_sdata_ptr->buf_ptr[0].max_data_len - dst_sdata_ptr->buf_ptr[0].actual_data_len;

   for (uint32_t ch = 0; ch < num_ch; ch++)
   {
      // Copy data from lib output to CAPI output.
      int8_t *dst_ptr  = dst_sdata_ptr->buf_ptr[ch].data_ptr + dst_sdata_ptr->buf_ptr[ch].actual_data_len;
      int8_t *src_ptr  = int_buf_ptr->data_ptrs[ch] + int_buf_ptr->data_offset_bytes_per_ch;
      bytes_copied_out = memscpy(dst_ptr, dst_size, src_ptr, src_size);

      // Mark CAPI output bytes produced.
      dst_sdata_ptr->buf_ptr[ch].actual_data_len += bytes_copied_out;
   }

   // Mark bytes copied.
   int_buf_ptr->data_offset_bytes_per_ch += bytes_copied_out;

   // Once the entire lib output buffer is copied, mark it as empty.
   if (int_buf_ptr->actual_data_len_per_ch == int_buf_ptr->data_offset_bytes_per_ch)
   {
      int_buf_ptr->data_offset_bytes_per_ch = 0;
      int_buf_ptr->actual_data_len_per_ch   = 0;
   }

   return bytes_copied_out;
}

static void capi_library_internal_buffer_check_update_frame_size_units(capi_library_internal_buf_t *int_buf_ptr)
{
   if ((!int_buf_ptr->flags.is_media_fmt_received) || (!int_buf_ptr->flags.frame_size_recieved))
   {
      return;
   }

   // If frame size was specified in time, we need to calculate samples.
   if (int_buf_ptr->flags.frame_size_in_time)
   {
      uint64_t temp                          = int_buf_ptr->frame_size_us * int_buf_ptr->media_fmt.format.sampling_rate;
      int_buf_ptr->frame_size_samples_per_ch = (uint32_t)(temp / ((uint64_t)1000000));
   }
   // If frame size was specified in samples, we need to calculate time.
   else
   {
      uint64_t temp              = (int_buf_ptr->frame_size_samples_per_ch * 1000 * 1000);
      int_buf_ptr->frame_size_us = (uint32_t)(temp / ((uint64_t)int_buf_ptr->media_fmt.format.sampling_rate));
   }
}

/**
 * Set the frame size in microseconds.
 */
void capi_library_internal_buffer_set_frame_size_us(capi_library_internal_buf_t *int_buf_ptr, uint32_t frame_size_us)
{
   if (frame_size_us != int_buf_ptr->frame_size_us)
   {
      int_buf_ptr->frame_size_us             = frame_size_us;
      int_buf_ptr->flags.frame_size_recieved = TRUE;
      int_buf_ptr->flags.frame_size_in_time  = TRUE;

      // Calculate frame size in the unit that was not configured.
      capi_library_internal_buffer_check_update_frame_size_units(int_buf_ptr);

      // Realloc the buffers if the frame size changed.
      if (capi_library_internal_buffer_exists(int_buf_ptr))
      {
         AR_MSG(DBG_HIGH_PRIO,
                "Capi buf util: frame size changed (from %ld us to %ld us), reallocating buffer.",
                frame_size_us,
                int_buf_ptr->frame_size_us);

         capi_library_internal_buffer_create(int_buf_ptr, int_buf_ptr->heap_id);
      }
   }
}

/**
 * Set the frame size in samples.
 */
void capi_library_internal_buffer_set_frame_size_samples(capi_library_internal_buf_t *int_buf_ptr,
                                                         uint32_t                     frame_size_samples_per_ch)
{
   if (frame_size_samples_per_ch != int_buf_ptr->frame_size_samples_per_ch)
   {
      int_buf_ptr->frame_size_samples_per_ch = frame_size_samples_per_ch;
      int_buf_ptr->flags.frame_size_recieved = TRUE;
      int_buf_ptr->flags.frame_size_in_time  = FALSE;

      // Calculate frame size in the unit that was not configured.
      capi_library_internal_buffer_check_update_frame_size_units(int_buf_ptr);

      // Realloc the buffers if the frame size changed.
      if (capi_library_internal_buffer_exists(int_buf_ptr))
      {
         AR_MSG(DBG_HIGH_PRIO,
                "Capi buf util: frame size changed (from %ld samples per ch to %ld samples per ch), reallocating "
                "buffer.",
                frame_size_samples_per_ch,
                int_buf_ptr->frame_size_samples_per_ch);

         capi_library_internal_buffer_create(int_buf_ptr, int_buf_ptr->heap_id);
      }
   }
}

/**
 * Sets the buffer's media format.
 */
void capi_library_internal_buffer_set_media_fmt(capi_library_internal_buf_t *int_buf_ptr,
                                                capi_media_fmt_v2_t *        data_format_ptr)
{
   if (!capi_cmn_media_fmt_equal(&int_buf_ptr->media_fmt, data_format_ptr))
   {
      int_buf_ptr->media_fmt                   = *data_format_ptr; // Copy.
      int_buf_ptr->flags.is_media_fmt_received = TRUE;

      // Calculate frame size in the unit that was not configured.
      capi_library_internal_buffer_check_update_frame_size_units(int_buf_ptr);

      // Realloc the buffers if the frame size changed.
      if (capi_library_internal_buffer_exists(int_buf_ptr))
      {
         AR_MSG(DBG_HIGH_PRIO, "Capi buf util: media format changed, reallocating buffer.");

         capi_library_internal_buffer_create(int_buf_ptr, int_buf_ptr->heap_id);
      }
   }
}

// Remove all data from the buffer.
void capi_library_internal_buffer_flush(capi_library_internal_buf_t *int_buf_ptr)
{
   int_buf_ptr->actual_data_len_per_ch   = 0;
   int_buf_ptr->data_offset_bytes_per_ch = 0;
   AR_MSG(DBG_HIGH_PRIO, "Capi buf util: buf was flushed");
}

uint32_t capi_library_internal_buffer_zero_fill(capi_library_internal_buf_t *int_buf_ptr, uint32_t zero_bytes_per_ch)
{
   if ((!capi_library_internal_buffer_exists(int_buf_ptr)) || int_buf_ptr->actual_data_len_per_ch)
   {
      AR_MSG(DBG_HIGH_PRIO,
             "Capi buf util: Not zero filling - does buf exist %ld, actual data len %ld",
             capi_library_internal_buffer_exists(int_buf_ptr),
             int_buf_ptr->actual_data_len_per_ch);

      return 0;
   }

   uint32_t num_ch              = int_buf_ptr->media_fmt.format.num_channels;
   uint32_t memset_bytes_per_ch = MIN(int_buf_ptr->max_data_len_per_ch, zero_bytes_per_ch);
   for (uint32_t ch = 0; ch < num_ch; ch++)
   {
      // Fill with zeros.
      memset(int_buf_ptr->data_ptrs[ch], 0, memset_bytes_per_ch);
   }

   int_buf_ptr->actual_data_len_per_ch = memset_bytes_per_ch;

   AR_MSG(DBG_HIGH_PRIO,
          "Capi buf util: buf was filled with %ld bytes per ch of zeros",
          int_buf_ptr->actual_data_len_per_ch);

   return memset_bytes_per_ch;
}