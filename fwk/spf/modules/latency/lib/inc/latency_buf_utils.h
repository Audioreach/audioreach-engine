/*============================================================================
  @file latency_buf_utils.h
  Delayline declarations, Delayline related 16/32 bit processing functions

  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
============================================================================*/

#include "ar_defs.h"
#if __qdsp6__
   #include <hexagon_protos.h>
#endif 
/*----------------------------------------------------------------------------
   Typdefs
----------------------------------------------------------------------------*/
/* circular delay line (32 bit */
typedef struct delayline32 {
    int32_t         idx;                   // index to "now"
    int32_t         buf_size;              // delay buf size
    int32_t         *buf;                  // buf pointer
} delayline_32_t; 

typedef struct _delayline_16_t
{
    int32_t       delay_index;             /* delayline index                   */
    int32_t       delay_length;            /* delayline length                  */
    int16_t      *delay_buf;               /* pointer to delayline buffer       */
} delayline_16_t; 

/*----------------------------------------------------------------------------
   Function Prototypes
----------------------------------------------------------------------------*/
#if !defined(__qdsp6__)
/** Wrap the var1 into the modulo range from 0 to var2*/
int32_t latency_s32_modwrap_s32_u32(int32_t var1, uint32_t var2);

/**
  Finds the minimum of two signed 32-bit input numbers
  and returns the signed 32-bit result.
*/
int32_t latency_s32_min_s32_s32(int32_t var1, int32_t var2); 

#else //__qdsp6__

   /** Wrap the var1 into the modulo range from 0 to var2
   */
#define latency_s32_modwrap_s32_u32(var1, var2) Q6_R_modwrap_RR(var1, var2)
/**
  Finds the minimum of two signed 32-bit input numbers
  and returns the signed 32-bit result.
*/
#define latency_s32_min_s32_s32(var1, var2)  Q6_R_min_RR(var1,var2)

#endif //__qdsp6__

void latency_delayline32_read(int32_t *dest, int32_t *src, delayline_32_t *delayline, int32_t delay, int32_t samples);
void latency_delayline32_update(delayline_32_t *delayline, int32_t *src, int32_t samples);
void latency_buffer_delay_fill(int16_t *destBuf, int16_t *srcBuf, delayline_16_t *delayline, int32_t delay, int32_t samples);
void latency_delayline_update(delayline_16_t *delayline, int16_t *srcBuf, int32_t samples);
void latency_delayline_copy(delayline_16_t *dest, delayline_16_t *source);
void latency_delayline32_copy(delayline_32_t *dest, delayline_32_t *source);
void latency_delayline_set(delayline_16_t *delayLine, int32_t delayLen);
void latency_delayline32_set(delayline_32_t *delayLine, int32_t delayLen);
void latency_delayline_reset(delayline_16_t *delayline);
void latency_delayline32_reset(delayline_32_t *delayline);