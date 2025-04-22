#ifndef _RT_PROXY_DRIVER_LIB_H
#define _RT_PROXY_DRIVER_LIB_H

/**
 *   \file rt_proxy_driver.h
 *   \brief
 *        This file contains utility functions for handling rt proxy buffering utils.
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

#define CAPI_RT_PROXY_MAX_READ_CLIENTS 1

// DEBUG macro
//#define DEBUG_RT_PROXY_DRIVER

/*==============================================================================
   Function declarations
==============================================================================*/

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

typedef struct capi_rt_proxy_t    capi_rt_proxy_t;
typedef struct rt_proxy_driver_t  rt_proxy_driver_t;
typedef struct rt_proxy_ch_info_t rt_proxy_ch_info_t;

struct rt_proxy_driver_t
{
   uint32_t circ_buf_size_in_us;
   // Buffer size is calculated based on the client and HW ep frame sizes.

   uint32_t lower_drift_threshold;
   // If buffer size goes below this value, drift is corrected based on module dir.

   uint32_t upper_drift_threshold;
   // If buffer size goes above this value, drift is corrected based on module dir.

   uint32_t high_watermark_level;
   // If buffer size goes above this value an event is sent to HLOS indicating buffer writes are faster.

   uint32_t low_watermark_level;
   // If buffer size goes above this value an event is sent to HLOS indicating buffer is draining faster.

   int32_t timer_adjust_constant_value;
   // Drift correction rate.

   uint32_t total_data_written_in_ms;
   uint32_t total_data_read_in_ms;
   // Debug varaibles. Used to track total amnt of data written/read froms start of buffer
   // useful for finding out effective drift between read-write over a period of time

   spf_circ_buf_t *stream_buf;

   spf_circ_buf_client_t *writer_handle;

   spf_circ_buf_client_t *reader_handle;
};

ar_result_t rt_proxy_driver_init(capi_rt_proxy_t *me_ptr);

ar_result_t rt_proxy_driver_deinit(capi_rt_proxy_t *me_ptr);

ar_result_t rt_proxy_stream_write(capi_rt_proxy_t *drv_ptr, capi_stream_data_t *in_stream);

ar_result_t rt_proxy_calibrate_driver(capi_rt_proxy_t *me_ptr);

ar_result_t rt_proxy_stream_read(capi_rt_proxy_t *drv_ptr, capi_stream_data_t *out_stream);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*_RT_PROXY_DRIVER_LIB_H*/
