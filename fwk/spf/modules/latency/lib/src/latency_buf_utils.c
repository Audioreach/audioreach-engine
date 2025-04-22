/*============================================================================
  @file latency_buf_utils.c
  Delayline related 16/32 bit processing functions

  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
============================================================================*/

#include "latency_buf_utils.h"
#include <stringl.h>
/*----------------------------------------------------------------------------
   Function Prototypes
----------------------------------------------------------------------------*/
void latency_buffer32_copy_v2(int32_t *dest, int32_t *src, int32_t samples);
////////////////////////////////////////////////////32bit////////////////////////////////////////
/*----------------------------------------------------------------------------
   Function Definitions
----------------------------------------------------------------------------*/
/* refill delayline with new samples */
void latency_delayline32_update(delayline_32_t *delayline, int32_t *src, int32_t samples)
{
   int32_t  delay_buf_size = delayline->buf_size;
   int32_t  delay_idx      = delayline->idx;
   int32_t *delay_buf      = delayline->buf;
   int32_t *src_ptr;
   int32_t  update_length, n;
   int32_t  samples_till_delay_buf_end = delay_buf_size - delay_idx;

   // determine number of samples to update, and where new samples begin
   if (samples > delay_buf_size)
   {
      update_length = delay_buf_size;
      src_ptr       = src + samples - delay_buf_size;
   }
   else
   {
      update_length = samples;
      src_ptr       = src;
   }

   // update samples from current idx to the end of delayline
   n = latency_s32_min_s32_s32(samples_till_delay_buf_end, update_length);
   latency_buffer32_copy_v2(delay_buf + delay_idx, src_ptr, n);
   delay_idx += n;
   src_ptr += n;

   if (n == samples_till_delay_buf_end)
   {
      update_length -= n;
      // update from the beginning of delayline
      latency_buffer32_copy_v2(delay_buf, src_ptr, update_length);
      delay_idx = update_length;
   }

   // save delayline idx
   delayline->idx = delay_idx;
}

/*===========================================================================*/
/* FUNCTION : latency_delayline32_set                                                */
/*                                                                           */
/* DESCRIPTION: Set delayline delay length. Reset delay index and            */
/*              delay buffer                                                 */
/*                                                                           */
/* INPUTS: delayline-> delayline struct                                      */
/* OUTPUTS: delayline-> delayline struct                                     */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
void latency_delayline32_set(delayline_32_t *delayline, /* delayline struct                  */
                             int32_t         delay_len  /* delay length                      */
                             )
{
   delayline->buf_size = delay_len;
   latency_delayline32_reset(delayline);

} /*------------------ end of function latency_delayline32_set ------------------------*/

/*===========================================================================*/
/* FUNCTION : delayline32_copy                                               */
/*                                                                           */
/* DESCRIPTION: Copies the contents of one delay line to another. The        */
/*              contents of the destination are lost. This is for 32 bit     */
/*              data.                                                        */
/*                                                                           */
/* INPUTS: delayline src-> delayline_32_t                                     */
/* OUTPUTS: delayline dest-> delayline_32_t                                   */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
void latency_delayline32_copy(delayline_32_t *dest,  /* The destination delay line        */
                              delayline_32_t *source /* The source delay line             */
                              )
{
   int32_t  samples_to_copy = 0;
   int32_t *src_end_ptr     = source->buf + source->buf_size;
   int32_t *src_ptr         = source->buf + source->idx;
   int32_t *dst_ptr         = dest->buf;

   if (dest->buf_size < source->buf_size)
   {
      // Some part of the source must be discarded.
      int32_t num_samples_to_discard = source->buf_size - dest->buf_size;
      src_ptr += num_samples_to_discard;
      if (src_ptr > src_end_ptr)
      {
         src_ptr -= source->buf_size;
      }
      samples_to_copy = dest->buf_size;
   }
   else
   {
      samples_to_copy = source->buf_size;
   }

   int32_t src_end_dist = src_end_ptr - src_ptr;
   if (src_end_dist < samples_to_copy)
   {
      latency_buffer32_copy_v2(dst_ptr, src_ptr, src_end_dist);
      samples_to_copy -= src_end_dist;
      src_ptr = source->buf;
      dst_ptr += src_end_dist;
   }

   latency_buffer32_copy_v2(dst_ptr, src_ptr, samples_to_copy);
   dst_ptr += samples_to_copy;

   dest->idx = dst_ptr - dest->buf;
   if (dest->idx >= dest->buf_size)
   {
      dest->idx -= dest->buf_size;
   }
} /*------------------ end of function delayline_copy ------------------------*/

/* read from delayline with specific delay amount, some cases need input fresh samples */
void latency_delayline32_read(int32_t *dest, int32_t *src, delayline_32_t *delayline, int32_t delay, int32_t samples)
{
   int32_t *delay_buf      = delayline->buf;
   int32_t  delay_buf_size = delayline->buf_size;
   int32_t  delay_idx      = delayline->idx;
   int32_t  samples_from_delay, samples_till_delay_buf_end, read_idx, n;

   // determine where to read from, assuming delay is less than delay buf size
   read_idx = latency_s32_modwrap_s32_u32(delay_idx - delay, delay_buf_size);

   // determine samples from delayline, and remaining to read from input
   samples_from_delay = latency_s32_min_s32_s32(delay, samples);
   samples -= samples_from_delay;

   // read samples from delayline (read_idx till end)
   samples_till_delay_buf_end = delay_buf_size - read_idx;
   n                          = latency_s32_min_s32_s32(samples_till_delay_buf_end, samples_from_delay);
   latency_buffer32_copy_v2(dest, delay_buf + read_idx, n);
   read_idx += n;
   dest += n;

   // read samples from delayline (wrapped around)
   if (n == samples_till_delay_buf_end)
   {
      samples_from_delay -= n;
      // update from the beginning of delayline
      latency_buffer32_copy_v2(dest, delay_buf, samples_from_delay);
      dest += samples_from_delay;
   }

   // read samples from input buffer
   latency_buffer32_copy_v2(dest, src, samples);
}

/* reset delayline */
void latency_delayline32_reset(delayline_32_t *delayline)
{
   memset(delayline->buf, 0, (delayline->buf_size << 2));
   delayline->idx = 0;
}

//////////////////////////////////////16bit Word/////////////////////////////////////////////

/*===========================================================================*/
/* FUNCTION : delayline_copy                                                 */
/*                                                                           */
/* DESCRIPTION: Copies the contents of one delay line to another. The        */
/*              contents of the destination are lost.                        */
/*                                                                           */
/* INPUTS: delayline-> delayline struct                                      */
/* OUTPUTS: delayline-> delayline struct                                     */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
void latency_delayline_copy(delayline_16_t *dest,  /* The destination delay line        */
                            delayline_16_t *source /* The source delay line             */
                            )
{
   int32_t  samples_to_copy = 0;
   int16_t *src_end_ptr     = source->delay_buf + source->delay_length;
   int16_t *src_ptr         = source->delay_buf + source->delay_index;
   int16_t *dst_ptr         = dest->delay_buf;

   if (dest->delay_length < source->delay_length)
   {
      // Some part of the source must be discarded.
      int32_t num_samples_to_discard = source->delay_length - dest->delay_length;
      src_ptr += num_samples_to_discard;
      if (src_ptr > src_end_ptr)
      {
         src_ptr -= source->delay_length;
      }
      samples_to_copy = dest->delay_length;
   }
   else
   {
      samples_to_copy = source->delay_length;
   }

   int32_t src_end_dist = src_end_ptr - src_ptr;
   if (src_end_dist < samples_to_copy)
   {
      while (src_ptr < src_end_ptr)
      {
         *dst_ptr++ = *src_ptr++;
      }
      samples_to_copy -= src_end_dist;
      src_ptr = source->delay_buf;
   }

   while (samples_to_copy > 0)
   {
      *dst_ptr++ = *src_ptr++;
      samples_to_copy--;
   }

   dest->delay_index = dst_ptr - dest->delay_buf;
   if (dest->delay_index >= dest->delay_length)
   {
      dest->delay_index -= dest->delay_length;
   }
} /*------------------ end of function delayline_copy ------------------------*/

void latency_buffer_fill(int16_t *dest_buf, /* output buffer                     */
                         int16_t *src_buf,  /* input buffer                      */
                         uint32_t samples   /* number of samples to process      */
                         )
{
   uint32_t i;

   for (i = 0; i < samples; i++)
   {
      *dest_buf++ = *src_buf++;
   }
}
/*===========================================================================*/
/* FUNCTION : buffer_delay_fill                                              */
/*                                                                           */
/* DESCRIPTION: Store in output buffer with delayed samples                  */
/* The samples could be from the delayline or the                            */
/*              input buffer.                                                */
/*                                                                           */
/* INPUTS: dest_buf-> output buffer                                           */
/*         src_buf-> input buffer                                             */
/*         delayline-> delayline struct                                      */
/*         delay: amount of delay in sample                                  */
/*         samples: total number of samples to be processed                  */
/* OUTPUTS: dest_buf-> output buffer                                          */
/*                                                                           */
/*===========================================================================*/
void latency_buffer_delay_fill(int16_t *       dest_buf,  /* output buffer                     */
                               int16_t *       src_buf,   /* input buffer  */
                               delayline_16_t *delayline, /* delayline struct */
                               int32_t         delay,     /* amount of delay in sample         */
                               int32_t         samples    /* number of samples to process      */
                               )
{
   int16_t *dest_ptr = dest_buf;
   int16_t *src_ptr  = src_buf;
   int16_t *delay_ptr;
   int32_t  delay_length = delayline->delay_length;
   int32_t  delay_index  = delayline->delay_index;
   int32_t  from_delay, i;

   /*-----------------------------------------------------------------------*/

   /*--------- set delay pointer so it points to "delay" samples ago -------*/
   delay_index -= delay;
   if (delay_index < 0)
   { /* delay should be less than delay_length; if not, we are in trouble  */
      delay_index += delay_length;
   }
   delay_ptr = delayline->delay_buf + delay_index;

   /*----------------------- process from delayline ------------------------*/
   if (delay > 0 && samples > 0)
   {
      from_delay = delay;
      if (from_delay > samples)
      { /* if delay amount is more than output samples, then all of them */
         /* should from delayline                                         */
         from_delay = samples;
      }

      for (i = 0; i < from_delay; i++)
      {
         /*-- output = delayed samples --*/
         *dest_ptr++ = *delay_ptr++;

         /* manage circular buffer */
         if (delay_ptr == delayline->delay_buf + delay_length)
         {
            delay_ptr = delayline->delay_buf;
         }
      } /* end of for (i = 0; i < from_delay; i++) */

      /* so far processed "from_delay" samples */
      samples -= from_delay;
   } /* end of if (delay > 0 && samples > 0) */

   /*----------------------- process from input buffer ---------------------*/
   if (samples > 0)
   {
      latency_buffer_fill(dest_ptr, src_ptr, samples);
   } /* end of if (samples > 0) */
} /*------------------ end of function buffer_delay_fill -------------------*/

/*===========================================================================*/
/* FUNCTION : delayline_update                                               */
/*                                                                           */
/* DESCRIPTION: Update the buffer in a delayline struct with new input       */
/*              samples from input buffer.                                   */
/*                                                                           */
/* INPUTS: delayline-> delayline struct                                      */
/*         srcBuf-> input buffer                                             */
/*         samples: number of samples of the input buffer                    */
/* OUTPUTS: delayline-> its buffer updated                                   */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/

/*
LATENCY_DELAYLINE_UPDATE_ASM_COMPILATION flag enable/disable assembly
compilation for latency_delayline_update() function . With SCons build
enable this flag to compile asm and not enable to compile c code for
CMake build.
*/
#if  defined(__qdsp6__) && defined(LATENCY_DELAYLINE_UPDATE_ASM_COMPILATION)
#else
void latency_delayline_update(delayline_16_t *delayline, /* delayline struct                  */
                              int16_t *       src_buf,   /* input buffer (new samples)        */
                              int32_t         samples    /* input buffer sample size          */
                              )
{
   int32_t  i, update_len;
   int16_t *src_ptr;
   int16_t *delay_ptr = delayline->delay_buf + delayline->delay_index;

   /*-----------------------------------------------------------------------*/

   /*------------------------ determine lengths ----------------------------*/
   /*--- if  input buffer is longer than delayline ---*/
   if (samples > delayline->delay_length)
   {
      /* whole delayline gets updated with part of input buffer samples */
      update_len = delayline->delay_length;
      /* the last "delay_length" samples from input buffer get used */
      src_ptr = src_buf + samples - delayline->delay_length;
   }
   /*--- if input buffer is shorter than delayline ---*/
   else
   { /* update "samples" number of samples */
      update_len = samples;
      /* using the whole input buffer samples */
      src_ptr = src_buf;
   }

   /*-------------------------- update samples -----------------------------*/
   for (i = 0; i < update_len; i++)
   {
      /* copy from input buffer */
      *delay_ptr++ = *src_ptr++;
      /* manage circular buffer */
      if (delay_ptr == delayline->delay_buf + delayline->delay_length)
      {
         delay_ptr = delayline->delay_buf;
      }
   } /* end of for (i = 0; i < update_len; i++) */

   /* stored the index value of delayline back into the struct */
   delayline->delay_index = (int32_t)(delay_ptr - delayline->delay_buf);
} /*-------------------- end of function delayline_update ------------------*/
#endif //__qdsp6__

/*===========================================================================*/
/* FUNCTION : latency_delayline_set                                                  */
/*                                                                           */
/* DESCRIPTION: Set delayline delay length. Reset delay index and            */
/*              delay buffer                                                 */
/*                                                                           */
/* INPUTS: delayline-> delayline struct                                      */
/* OUTPUTS: delayline-> delayline struct                                     */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
void latency_delayline_set(delayline_16_t *delayline, /* delayline struct                  */
                           int32_t         delay_len  /* delay length                      */
                           )
{
   delayline->delay_length = delay_len;
   latency_delayline_reset(delayline);
} /*------------------ end of function latency_delayline_set ------------------------*/

/*===========================================================================*/
/* FUNCTION : delayline_reset                                                */
/*                                                                           */
/* DESCRIPTION: Clean delayline buffer and reset delay index                 */
/*                                                                           */
/* INPUTS: delayline-> delayline struct                                      */
/* OUTPUTS: delayline-> delayline struct                                     */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
void latency_delayline_reset(delayline_16_t *delayline /* delayline struct                  */
                             )
{
   int16_t *buf_ptr = delayline->delay_buf;
   memset(buf_ptr, 0, (delayline->delay_length << 1)); //2bytes per sample
   delayline->delay_index = 0;
} /*------------------ end of function delayline_reset ----------------------*/
/////////////////////////////utils//////////////////////////////////////////

// copy 32 bit buffers
void latency_buffer32_copy_v2(int32_t *dest, int32_t *src, int32_t samples)
{
#ifndef __qdsp6__
   int32_t i;
   for (i = 0; i < samples; ++i)
   {
      *dest++ = *src++;
   }
#else
   memscpy(dest, samples << 2, src, samples << 2);
#endif
}

#if !defined(__qdsp6__)
int32_t latency_s32_modwrap_s32_u32(int32_t var1, uint32_t var2)
{
   if (var1 < 0)
   {
      return var1 + var2;
   }

   else if ((uint32_t)var1 >= var2)
   {
      return var1 - var2;
   }
   return var1;
}

int32_t latency_s32_min_s32_s32(int32_t var1, int32_t var2)
{
   int32_t out;

   out = (var1 < var2) ? var1 : var2;

   return (out);
}
#endif //__qdsp6__