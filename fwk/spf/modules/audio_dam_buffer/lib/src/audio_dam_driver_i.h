#ifndef __AUDIO_DAM_DRIVER_H_I__
#define __AUDIO_DAM_DRIVER_H_I__

/**
 *   \file audio_dam_driver_i.h
 *   \brief
 *        This file contains Audio Dam driver internal API.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "audio_dam_driver.h"
#include "spf_interleaver.h"

/* Utility to convert PCM data duration to bytes */
uint32_t audio_dam_compute_buffer_size_in_bytes(audio_dam_driver_t *drv_ptr, uint32_t buffer_size_in_us);

/** Utility to convert PCM data duration in us to bytes,
 * includes_frame_header - includes frame header size in the bytes passed, applicable only for raw compressed formats.
 */
uint32_t audio_dam_compute_buffer_size_in_us(audio_dam_driver_t *drv_ptr,
                                             uint32_t            buffer_size_in_bytes,
                                             bool_t              includes_frame_header);

/** Virtual Buffer utility functions declaration*/
ar_result_t audio_dam_stream_read_adjust_virt_wr_mode(audio_dam_stream_reader_t *reader_handle,
                                                      uint32_t                   requested_read_offset_in_us,
                                                      uint32_t                  *actual_read_offset_in_us_ptr);

ar_result_t audio_dam_stream_read_from_virtual_buf(audio_dam_stream_reader_t *reader_handle,          // in
                                                   uint32_t                   num_chs_to_read,        // in
                                                   capi_buf_t                *output_buf_arr,         // in/out
                                                   bool_t                    *output_buf_ts_is_valid, // out
                                                   int64_t                   *output_buf_ts,          // out
                                                   uint32_t                  *output_buf_len_in_us);

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*__AUDIO_DAM_DRIVER_H_I__*/