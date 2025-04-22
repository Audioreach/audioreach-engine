/**
 *   \file congestion_buf_driver.c
 *   \brief
 *        This file contains implementation of Congestion Buffer driver
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "congestion_buf_driver.h"
#include "spf_circular_buffer.h"
#include "ar_msg.h"
#include "spf_list_utils.h"
#include "capi_congestion_buf_i.h"

#ifdef DEBUG_CIRC_BUF_UTILS
#define DEBUG_CONGESTION_BUF_DRIVER
#endif

/*==============================================================================
   Local Declarations
==============================================================================*/

/*==============================================================================
   Local Function Implementation
==============================================================================*/

ar_result_t congestion_buf_driver_intialize_circular_buffers(capi_congestion_buf_t *me_ptr)
{
   ar_result_t result = AR_EOK;
   if (!me_ptr)
   {
      return AR_EFAILED;
   }

   spf_circ_buf_raw_alloc_inp_args_t inp_args;

   memset(&inp_args, 0, sizeof(spf_circ_buf_raw_alloc_inp_args_t));

   /* Suggested chunk size - this is also the suggested starting size of the buffer. */
   inp_args.preferred_chunk_size = CONGESTION_BUFFER_PREFERRED_CHUNK_SIZE;

   /* If frame size is available we have the max num frames that we require to buffer */
   if (me_ptr->cfg_ptr.frame_size_mode)
   {
      uint32_t frame_size_in_us = 0;
      if (me_ptr->cfg_ptr.frame_size_mode == CONGESTION_BUF_FRAME_SIZE_IN_US)
      {
         frame_size_in_us = me_ptr->cfg_ptr.frame_size_value;
      }
      else if (me_ptr->cfg_ptr.frame_size_mode == CONGESTION_BUF_FRAME_SIZE_SAMPLES)
      {
         frame_size_in_us = ((me_ptr->cfg_ptr.frame_size_value * 1000000) / me_ptr->cfg_ptr.sampling_rate);
      }
      else
      {
         AR_MSG(DBG_ERROR_PRIO,
                "CONGESTION_BUF_DRIVER: only CONGESTION_BUF_FRAME_SIZE_IN_US or CONGESTION_BUF_FRAME_SIZE_SAMPLES");
      }

      if (frame_size_in_us)
      {
         inp_args.max_num_frames = (me_ptr->cfg_ptr.congestion_buffer_duration_ms * 1000) / frame_size_in_us;
#ifdef DEBUG_CONGESTION_BUF_DRIVER
         AR_MSG(DBG_HIGH_PRIO,
                "CONGESTION_BUF_DRIVER: Initializing driver with Max num frames %u",
                inp_args.max_num_frames);
#endif
      }
   }

   /* Max size of data that we require to buffer based on the bit rate and congestion size received */
   inp_args.max_raw_circ_buf_size_bytes  = me_ptr->congestion_size_bytes_max;
   inp_args.max_raw_circ_buf_bytes_limit = CONGESTION_BUF_RAW_BUF_MAX_MEMORY;

   /* Heap ID and metadata handler for the raw circular buffer */
   inp_args.heap_id          = me_ptr->heap_id;
   inp_args.metadata_handler = &me_ptr->metadata_handler;

   /* Initializes the raw circular buffer related params */
   if (AR_DID_FAIL(result = spf_circ_buf_raw_init(&me_ptr->driver_hdl.stream_buf, &inp_args)))
   {
#ifdef DEBUG_CONGESTION_BUF_DRIVER
      AR_MSG(DBG_ERROR_PRIO, "CONGESTION_BUF_DRIVER: buffer create failed");
#endif
      return AR_EFAILED;
   }

   /* Registers the writer handle. Congestion buffer starts with one chunk and expands as required */
   if (AR_DID_FAIL(result =
                      spf_circ_buf_raw_register_writer_client(me_ptr->driver_hdl.stream_buf,
                                                              CONGESTION_BUFFER_PREFERRED_CHUNK_SIZE, /* buf size*/
                                                              &me_ptr->driver_hdl.writer_handle)))
   {
#ifdef DEBUG_CONGESTION_BUF_DRIVER
      AR_MSG(DBG_ERROR_PRIO, "CONGESTION_BUF_DRIVER: writer registration failed.");
#endif
      return AR_EFAILED;
   }

   /* Register the reader handle. Congestion buffer has only one reader. */
   if (AR_DID_FAIL(result = spf_circ_buf_raw_register_reader_client(me_ptr->driver_hdl.stream_buf,
                                                                    0, /* not requesting any resize */
                                                                    &me_ptr->driver_hdl.reader_handle)))
   {
#ifdef DEBUG_CONGESTION_BUF_DRIVER
      AR_MSG(DBG_ERROR_PRIO, "CONGESTION_BUF_DRIVER: Reader registration failed.");
#endif
      return AR_EFAILED;
   }

   return result;
}

/*==============================================================================
   Public Function Implementation
==============================================================================*/
/* Initializes Congestion Raw Circular Buffer, Reader and Writer */
ar_result_t congestion_buf_driver_init(capi_congestion_buf_t *me_ptr)
{
   ar_result_t result = AR_EOK;

   if (NULL == me_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "CONGESTION_BUF_DRIVER: Init failed. bad input params ");
      return AR_EBADPARAM;
   }

   AR_MSG(DBG_ERROR_PRIO, "CONGESTION_BUF_DRIVER: Initializing the driver");

   /* Initialize Congestion Buffer Params, Read and Write handles. */
   if (AR_EOK != (result = congestion_buf_driver_intialize_circular_buffers(me_ptr)))
   {
      return result;
   }

   return result;
}

/* Destroys Congestion Raw Circular Buffer, Reader and Writer */
ar_result_t congestion_buf_driver_deinit(capi_congestion_buf_t *me_ptr)
{
   ar_result_t result = AR_EOK;

   if (NULL == me_ptr || NULL == me_ptr->driver_hdl.stream_buf)
   {
#ifdef DEBUG_CONGESTION_BUF_DRIVER
      AR_MSG(DBG_ERROR_PRIO, "congestion_buf_driver_deinit: Failed. bad input params.");
#endif
      return AR_EBADPARAM;
   }

   // Destory the driver.
   // Destroys circular buffers and reader/writer registers
   if (AR_DID_FAIL(result = spf_circ_buf_raw_deinit(&me_ptr->driver_hdl.stream_buf)))
   {
      return AR_EFAILED;
   }

   return result;
}

/* Write input data into the congestion buffer*/
ar_result_t congestion_buf_stream_write(capi_congestion_buf_t *me_ptr, capi_stream_data_t *in_stream)
{
   ar_result_t result              = AR_EOK;
   bool_t      ALLOW_OVERFLOW_TRUE = TRUE;

   congestion_buf_driver_t *drv_ptr = &me_ptr->driver_hdl;

   if (NULL == in_stream->buf_ptr[0].data_ptr)
   {
#ifdef DEBUG_CONGESTION_BUF_DRIVER
      AR_MSG(DBG_MED_PRIO, "CONGESTION_BUF_DRIVER: Input buffer not preset, nothing to write.");
#endif
      return AR_EOK;
   }

   if (!me_ptr->driver_hdl.stream_buf)
   {
      AR_MSG(DBG_ERROR_PRIO, "CONGESTION_BUF_DRIVER: Write failed. Driver not intialized yet.");
      return AR_EFAILED;
   }

   /* If amount of data is more than MTU amount of size then
    * mark all data as consumed and return error */
   if (in_stream->buf_ptr->actual_data_len > me_ptr->cfg_ptr.mtu_size)
   {
      AR_MSG(DBG_HIGH_PRIO,
             "congestion_buf: Encoded data more than negotiated MTU %u - dropping %u",
             me_ptr->cfg_ptr.mtu_size,
             in_stream->buf_ptr->actual_data_len);

      /* TODO: Pending - Multiple buffers */
      in_stream->buf_ptr->actual_data_len = 0;

      return AR_EFAILED;
   }

   /* We do not write empty frames into congestion buffer. */
   if (0 == in_stream->buf_ptr->actual_data_len)
   {
      return AR_EOK;
   }

   /* Write one container frame into the circular buffer. */
   if (AR_DID_FAIL(result = spf_circ_buf_raw_write_one_frame(drv_ptr->writer_handle, in_stream, ALLOW_OVERFLOW_TRUE)))
   {
      AR_MSG(DBG_ERROR_PRIO, "CONGESTION_BUF_DRIVER: Failed writing the frame to circular buffer");
      return AR_EFAILED;
   }

#ifdef DEBUG_CONGESTION_BUF_DRIVER
   uint32_t unread_bytes = 0, unread_bytes_max = 0;
   spf_circ_buf_raw_get_unread_bytes(drv_ptr->reader_handle, &unread_bytes, &unread_bytes_max);
   AR_MSG(DBG_HIGH_PRIO,
          "CONGESTION_BUF_DRIVER: Stream writer done. actual_data_len = %d, circ buf filled size=%lu",
          in_stream->buf_ptr[0].actual_data_len,
          unread_bytes);

   AR_MSG(DBG_HIGH_PRIO,
          "CONGESTION_BUF_DRIVER: Write done. total_data_written_in_ms = %lu, total_data_read_in_ms = %lu diff = %d",
          ++drv_ptr->total_data_written_in_ms,
          drv_ptr->total_data_read_in_ms,
          drv_ptr->total_data_written_in_ms - drv_ptr->total_data_read_in_ms);
#endif

   return result;
}

/* Read one frame out of the congestion buffer */
ar_result_t congestion_buf_stream_read(capi_congestion_buf_t *me_ptr, capi_stream_data_t *out_stream)
{

   ar_result_t result = AR_EOK;

   if (NULL == out_stream->buf_ptr[0].data_ptr)
   {
#ifdef DEBUG_CONGESTION_BUF_DRIVER
      AR_MSG(DBG_MED_PRIO, "CONGESTION_BUF_DRIVER: Output buffer not preset, nothing to read.");
#endif
      return AR_EOK;
   }

   if (NULL == me_ptr->driver_hdl.stream_buf && !me_ptr->raw_data_len)
   {
      AR_MSG(DBG_ERROR_PRIO, "CONGESTION_BUF_DRIVER: Read failed. Driver not intialized yet.");
      return AR_EFAILED;
   }

   congestion_buf_driver_t *drv_ptr = &me_ptr->driver_hdl;

   /* If reader is empty then no need to read data */
   bool_t is_empty = FALSE;
   result          = spf_circ_buf_raw_driver_is_buffer_empty(drv_ptr->reader_handle, &is_empty);
   if (is_empty)
   {
      return AR_EOK;
   }

   /* Read one container frame from the circular buffer. */
   result = spf_circ_buf_raw_read_one_frame(drv_ptr->reader_handle, out_stream);
   if (SPF_CIRCBUF_UNDERRUN == result)
   {
      return AR_ENEEDMORE;
   }
   else if (SPF_CIRCBUF_FAIL == result)
   {
      AR_MSG(DBG_HIGH_PRIO, "CONGESTION_BUF_DRIVER: Failed reading data from the buffer.");
      return AR_EFAILED;
   }

#ifdef DEBUG_CONGESTION_BUF_DRIVER
   uint32_t unread_bytes = 0, unread_bytes_max = 0;
   spf_circ_buf_raw_get_unread_bytes(drv_ptr->reader_handle, &unread_bytes, &unread_bytes_max);
   AR_MSG(DBG_HIGH_PRIO,
          "CONGESTION_BUF_DRIVER: Stream reader done. actual_data_len = %d, circ buf filled size=%lu",
          out_stream->buf_ptr[0].actual_data_len,
          unread_bytes);

   AR_MSG(DBG_HIGH_PRIO,
          "CONGESTION_BUF_DRIVER: Read done. total_data_written_bytes = %lu, total_data_read_bytes = %lu diff = %d",
          drv_ptr->total_data_written_bytes,
          ++drv_ptr->total_data_read_bytes,
          drv_ptr->total_data_written_bytes - drv_ptr->total_data_read_bytes);
#endif

   return result;
}
