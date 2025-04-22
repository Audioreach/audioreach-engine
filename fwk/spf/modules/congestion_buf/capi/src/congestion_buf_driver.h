#ifndef _CONGESTION_BUF_DRIVER_LIB_H
#define _CONGESTION_BUF_DRIVER_LIB_H

/**
 *   \file congestion_buf_driver.h
 *   \brief
 *        This file contains utility functions for handling Congestion Buffer buffering utils.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_defs.h"
#include "posal.h"
#include "capi_types.h"
#include "spf_list_utils.h"
#include "spf_circular_buffer.h"

#define CAPI_CONGESTION_BUF_MAX_READ_CLIENTS 1
#define CAPI_CONGESTION_BUF_MAX_INPUT_CHANNELS 32

// DEBUG macro
//#define DEBUG_CONGESTION_BUF_DRIVER

/*==============================================================================
   Function declarations
==============================================================================*/

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

typedef struct capi_congestion_buf_t capi_congestion_buf_t;
/* Capi pointer structure */

typedef struct congestion_buf_driver_t congestion_buf_driver_t;
/* Congestion buffer driver pointer */

struct congestion_buf_driver_t
{
   /* Debug variable updated under DEBUG_CONGESTION_BUF_DRIVER*/
   uint32_t total_data_written_bytes;

   /* Debug variable updated under DEBUG_CONGESTION_BUF_DRIVER */
   uint32_t total_data_read_bytes;

   /* Pointer to the circular buffer containing raw data */
   spf_circ_buf_raw_t *stream_buf;

   /* Write handle for the current buffer */
   spf_circ_buf_raw_client_t *writer_handle;

   /* Read handle for the current buffer */
   spf_circ_buf_raw_client_t *reader_handle;
};

ar_result_t congestion_buf_driver_init(capi_congestion_buf_t *me_ptr);

ar_result_t congestion_buf_calibrate_driver(capi_congestion_buf_t *me_ptr);

ar_result_t congestion_buf_driver_deinit(capi_congestion_buf_t *me_ptr);

ar_result_t congestion_buf_stream_write(capi_congestion_buf_t *drv_ptr, capi_stream_data_t *in_stream);

ar_result_t congestion_buf_stream_read(capi_congestion_buf_t *drv_ptr, capi_stream_data_t *out_stream);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*_CONGESTION_BUF_DRIVER_LIB_H*/
