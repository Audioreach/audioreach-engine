/*========================================================================

file spf_interleaver.h
This file contains declarations of interleaver and de-interleaver functions

   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
======================================================================*/

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#ifndef PC_INTERLEAVER_H
#define PC_INTERLEAVER_H
#include "ar_error_codes.h"
#include "capi.h"
#include "capi_cmn.h"
#include "ar_defs.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

ar_result_t spf_deintlv_to_intlv_v2(capi_buf_t *input_buf_ptr,
                                    capi_buf_t *output_buf_ptr,
                                    uint32_t    num_channels,
                                    uint32_t    bytes_per_samp,
                                    uint32_t    num_samp_per_ch);

static inline ar_result_t spf_deintlv_to_intlv(capi_buf_t *input_buf_ptr,
                                               capi_buf_t *output_buf_ptr,
                                               uint32_t    num_channels,
                                               uint32_t    word_size)
{
   // since the incoming buffer is deinterleaved unpacked, actual data len will contain data length for a channel
   uint32_t num_samp_per_ch = input_buf_ptr[0].actual_data_len / (word_size >> 3);
   uint32_t bytes_per_samp  = word_size >> 3;
   return spf_deintlv_to_intlv_v2(input_buf_ptr, output_buf_ptr, num_channels, bytes_per_samp, num_samp_per_ch);
}

/** updates only first ch's actual data len, in other words can be used for CAPI_DEINTERLEAVED_UNPACKED_V2 conversion.
 */
ar_result_t spf_intlv_to_deintlv_unpacked_v2(capi_buf_t *input_buf_ptr,
                                             capi_buf_t *output_buf_ptr,
                                             uint32_t    num_channels,
                                             uint32_t    bytes_per_samp,
                                             uint32_t    num_samp_per_ch);

/** Supports generating output with num channels less than that of interleaved data format. For example,
 * If input is 3ch, output is 2ch, genrates only the 2ch data and ignores the 3rd channels data.
 *
 * In most of the scenarios V3 is not required, since num_src_channels == num_dst_channels Hence V2 can be used.
 */
ar_result_t spf_intlv_to_deintlv_v3(capi_buf_t *input_buf_ptr,
                                    capi_buf_t *output_buf_ptr,
                                    uint32_t    num_src_channels,
                                    uint32_t    num_dst_channels,
                                    uint32_t    bytes_per_samp,
                                    uint32_t    num_samp_per_ch);

static inline ar_result_t spf_intlv_to_deintlv_v2(capi_buf_t *input_buf_ptr,
                                                  capi_buf_t *output_buf_ptr,
                                                  uint32_t    num_channels,
                                                  uint32_t    bytes_per_samp,
                                                  uint32_t    num_samp_per_ch)
{
   return spf_intlv_to_deintlv_v3(input_buf_ptr,
                                  output_buf_ptr,
                                  num_channels, // src
                                  num_channels, // & dst are same
                                  bytes_per_samp,
                                  num_samp_per_ch);
}

static inline ar_result_t spf_intlv_to_deintlv(capi_buf_t *input_buf_ptr,
                                               capi_buf_t *output_buf_ptr,
                                               uint32_t    num_channels,
                                               uint32_t    word_size)
{
   uint32_t num_samp_per_ch = input_buf_ptr->actual_data_len / (num_channels * word_size >> 3);
   uint32_t bytes_per_samp  = word_size >> 3;

   return spf_intlv_to_deintlv_v3(input_buf_ptr,
                                  output_buf_ptr,
                                  num_channels,
                                  num_channels,
                                  bytes_per_samp,
                                  num_samp_per_ch);
}

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif // PC_INTERLEAVER_H
