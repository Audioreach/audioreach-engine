/* =========================================================================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 * @file capi_pcm_mf_cnv_utils_island.cpp
 *
 * C source file to implement the Audio Post Processor Interface for
 * PCM_CNV Module
 */

#include "capi_pcm_mf_cnv_utils.h"

/*_____________________________________________________________________________________________________________________
   DESCRIPTION:
   Prints media fmt
  ____________________________________________________________________________________________________________________*/
void capi_pcm_mf_cnv_print_media_fmt(capi_pcm_mf_cnv_t *me_ptr, pc_media_fmt_t *fmt_ptr, uint32_t port)
{
   CNV_MSG(me_ptr->miid,
           DBG_HIGH_PRIO,
           "bit_width : %u , word_size : %u, q_factor : %u, interleaving : %u, endianness : %u, "
           "alignment : %u, num_channels : %u sampling rate %d",
           fmt_ptr->bit_width,
           fmt_ptr->word_size,
           fmt_ptr->q_factor,
           fmt_ptr->interleaving,
           fmt_ptr->endianness,
           fmt_ptr->alignment,
           fmt_ptr->num_channels,
           fmt_ptr->sampling_rate);
}


