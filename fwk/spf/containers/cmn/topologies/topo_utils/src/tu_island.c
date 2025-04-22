/**
 * \file tu_island.c
 *
 * \brief
 *
 *     Implementation of topology utility functions.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "topo_utils.h"

/* =======================================================================
Public Function Definitions
========================================================================== */
/**
 * includes previous fractional time, and calculates new fractional time in fract_time_pns_ptr
 * Unit of *fract_time_pns_ptr is "P nano seconds (pns)"
 *
 * is_bytes_per_channel = means bytes passed is per channel, so we need to calculate ts considering
 *                        media format as 1ch.
 *
 * pns is fractional timestamp represented in terms of 10 bits.
 * to covert to nano seconds ns = (fract_timestamp_pns * 1000) >> 10.
 *
 * You must use this utility tu_convert_pns_to_ns() to convert to nano seconds
 * before doing any arthimetic op with it.
 *
 * Must be called only for PCM or packetized
 */
uint64_t _topo_bytes_to_us_optimized_util(uint32_t          bytes,
                                          bool_t            bytes_passed_is_per_channel,
                                          topo_media_fmt_t *med_fmt_ptr,
                                          uint64_t *        fract_time_pns_ptr)
{
   uint64_t time_us         = 0;
   uint32_t num_channels    = bytes_passed_is_per_channel ? 1 : med_fmt_ptr->pcm.num_channels;
   time_us               = capi_cmn_bytes_to_us_optimized(bytes,
                                            num_channels,
                                            med_fmt_ptr->pcm.sample_rate,
                                            med_fmt_ptr->pcm.bits_per_sample,
                                            fract_time_pns_ptr);
   return time_us;
}
