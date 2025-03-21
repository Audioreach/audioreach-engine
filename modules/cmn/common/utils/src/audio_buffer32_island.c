/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*============================================================================
  @file audio_buffer32_island.c

  Buffer related 32 bit manipulations
  ============================================================================*/

#include "audio_dsp32.h"
#include "audio_basic_op_ext.h"
#include <stringl.h>

// copy 32 bit buffers
void buffer32_copy_v2(int32 *dest, int32 *src, int32 samples)
{
#ifndef __qdsp6__
   int32 i;
   for (i = 0; i < samples; ++i) {
      *dest++ = *src++;
   }
#else
   memscpy(dest, samples<<2, src, samples<<2);
#endif
}

// empty a 32 bit buffer
void buffer32_empty_v2(int32 *dest, int32 samples)
{
#ifndef __qdsp6__
   int32 i;
   for (i = 0; i < samples; ++i) {
      *dest++ = 0;
   }
#else
   memset(dest, 0, samples<<2);
#endif
}