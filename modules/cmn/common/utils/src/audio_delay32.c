/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*============================================================================
  @file audio_delay32.c
  Delayline related 32 bit processing functions
============================================================================*/
#include "audio_dsp32.h"
#include "audio_basic_op_ext.h"

/* reset delayline */
void delayline32_reset(delayline32_t *delayline)
{
   buffer32_empty_v2(delayline->buf, delayline->buf_size);
   delayline->idx = 0;
}

/* refill delayline with new samples */
void delayline32_update(delayline32_t *delayline, int32 *src, int32 samples)
{
   int32 delay_buf_size    = delayline->buf_size;
   int32 delay_idx         = delayline->idx;
   int32 *delay_buf        = delayline->buf;
   int32 *src_ptr;
   int32 update_length, n;
   int32 samples_till_delay_buf_end = delay_buf_size - delay_idx;

   // determine number of samples to update, and where new samples begin
   if (samples > delay_buf_size) {
      update_length = delay_buf_size;
      src_ptr = src + samples - delay_buf_size;
   } else {
      update_length = samples;
      src_ptr = src;
   }
   
   // update samples from current idx to the end of delayline
   n = s32_min_s32_s32(samples_till_delay_buf_end, update_length);
   buffer32_copy_v2(delay_buf+delay_idx, src_ptr, n);
   delay_idx += n;
   src_ptr += n;
   
   if (n == samples_till_delay_buf_end) {
      update_length -= n;
      // update from the beginning of delayline
      buffer32_copy_v2(delay_buf, src_ptr, update_length);
      delay_idx = update_length;
   }

   // save delayline idx
   delayline->idx = delay_idx;
}

/* exchange  buffer values between two 32 bit buffers */
static void exchange_buffer_values(int32 *buf1, int32 *buf2, int32 samples)
{
   int32 tmp32;
   int32 i;

   for (i = 0; i < samples; ++i) {
      tmp32 = *buf1;
      *buf1 = *buf2;
      *buf2 = tmp32;
      buf1++;
      buf2++;
   }
}

/* inplace delay imp for 32 bit delayline and data */
void delayline32_inplace_delay(int32 *inplace_buf, delayline32_t *delayline, int32 samples)
{
   int32 *delay_buf     = delayline->buf;
   int32 delay_buf_size = delayline->buf_size;
   int32 delay_idx      = delayline->idx;
   int32 samples_to_process, n;
   int32 samples_till_delay_buf_end = delay_buf_size - delay_idx;

   while (samples > 0) {
      samples_to_process = s32_min_s32_s32(samples, delay_buf_size);
      samples -= samples_to_process;
      n = s32_min_s32_s32(samples_till_delay_buf_end, samples_to_process);

      // exchange  buffer and delayed samples from current location in delay buf
      exchange_buffer_values(inplace_buf, delay_buf+delay_idx, n);
      inplace_buf += n;
      delay_idx += n;

      if (n == samples_till_delay_buf_end) {
         
         samples_to_process -= n;
         // exchange  buffer and delayed samples from first location in delay buf
         exchange_buffer_values(inplace_buf, delay_buf, samples_to_process);
         inplace_buf += samples_to_process;
         delay_idx = samples_to_process;
      }
   }

   // save delayline idx
   delayline->idx = delay_idx;
}

/* read from delayline with specific delay amount, some cases need input fresh samples */
void delayline32_read(int32 *dest, int32 *src, delayline32_t *delayline, int32 delay, int32 samples)
{
   int32 *delay_buf     = delayline->buf;
   int32 delay_buf_size = delayline->buf_size;
   int32 delay_idx      = delayline->idx;
   int32 samples_from_delay, samples_till_delay_buf_end, read_idx, n;

   // determine where to read from, assuming delay is less than delay buf size
   read_idx = s32_modwrap_s32_u32(delay_idx - delay, delay_buf_size);

   // determine samples from delayline, and remaining to read from input
   samples_from_delay = s32_min_s32_s32(delay, samples);
   samples -= samples_from_delay;

   // read samples from delayline (read_idx till end)
   samples_till_delay_buf_end = delay_buf_size - read_idx;
   n = s32_min_s32_s32(samples_till_delay_buf_end, samples_from_delay);
   buffer32_copy_v2(dest, delay_buf+read_idx, n);
   read_idx += n;
   dest += n;
   
   // read samples from delayline (wrapped around)
   if (n == samples_till_delay_buf_end) {
      samples_from_delay -= n;
      // update from the beginning of delayline
      buffer32_copy_v2(dest, delay_buf, samples_from_delay);
      dest += samples_from_delay;
   }

   // read samples from input buffer
   buffer32_copy_v2(dest, src, samples);
}

/* extract contents from delayline and mix into output buffer with panner */
void delayline32_panner_mix_out(int32 *dest, int32 *src, delayline32_t *delayline, pannerStruct *panner, int32 delay, int32 samples)
{
   // this function implies that samples < delayline length
   int32 *delay_buf     = delayline->buf;
   int32 delay_buf_size = delayline->buf_size;
   int32 delay_idx      = delayline->idx;
   int32 samples_till_delay_buf_end, samples_from_delay, read_idx, n;

   // determine where to read from, assuming delay is less than delay buf size
   read_idx = s32_modwrap_s32_u32(delay_idx - delay, delay_buf_size);

   // determine samples from delayline, and remaining to read from input
   samples_from_delay = s32_min_s32_s32(delay, samples);
   samples -= samples_from_delay;

   // process samples from read idx till end of delay buf
   samples_till_delay_buf_end = delay_buf_size - read_idx;
   n = s32_min_s32_s32(samples_till_delay_buf_end, samples_from_delay);
   buffer32_mix_panner(dest, delay_buf+read_idx, panner, n);
   read_idx += n;
   dest += n;
   
   // read samples from delayline (wrapped around)
   if (n == samples_till_delay_buf_end) {
      samples_from_delay -= n;
      // process samples from beginning of delay buf
      buffer32_mix_panner(dest, delay_buf, panner, samples_from_delay);
      dest += samples_from_delay;
   }

   // process samples from input buffer
   buffer32_mix_panner(dest, src, panner, samples);
}

/* mix content into delayline with smooth panner */
void delayline32_panner_mix_in(delayline32_t *delayline, int32 *src, pannerStruct *panner, int32 delay, int32 samples)
{
   // this function requires that samples < delayline length !!!
   int32 *delay_buf     = delayline->buf;
   int32 delay_buf_size = delayline->buf_size;
   int32 delay_idx      = delayline->idx;
   int32 samples_till_delay_buf_end, mix_idx, n;

   // determine where to read from, assuming delay is less than delay buf size
   mix_idx = s32_modwrap_s32_u32(delay_idx - delay, delay_buf_size);

   // process samples from read idx till end of delay buf
   samples_till_delay_buf_end = delay_buf_size - mix_idx;
   n = s32_min_s32_s32(samples_till_delay_buf_end, samples);
   buffer32_mix_panner(delay_buf+mix_idx, src, panner, n);
   mix_idx += n;
   src += n;
   
   // read samples from delayline (wrapped around)
   if (n == samples_till_delay_buf_end) {
      samples -= n;
      // process samples from beginning of delay buf
      buffer32_mix_panner(delay_buf, src, panner, samples);
   }
}

/*===========================================================================*/
/* FUNCTION : delayline_set32                                                */
/*                                                                           */
/* DESCRIPTION: Set delayline delay length. Reset delay index and            */
/*              delay buffer                                                 */
/*                                                                           */
/* INPUTS: delayline-> delayline struct                                      */
/* OUTPUTS: delayline-> delayline struct                                     */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
void delayline32_set
(
    delayline32_t *delayLine,         /* delayline struct                  */
    int32 delayLen                      /* delay length                      */
)
{
    delayLine->buf_size = delayLen;
    delayline32_reset(delayLine);

} /*------------------ end of function delayline_set ------------------------*/


/*===========================================================================*/
/* FUNCTION : buffer_delay_mix32                                             */
/*                                                                           */
/* DESCRIPTION: Mix into a running output buffer with delayed samples,       */
/*              applying L16Q15 gain at the same time. The samples could be  */
/*              from the delayline or the input buffer.                      */
/*                                                                           */
/* INPUTS: dest-> output buffer                                              */
/*         src-> input buffer                                                */
/*         gainL16Q15: gain to be applied to the samples                     */
/*         delayLine-> delayline struct                                      */
/*         delay: amount of delay in sample                                  */
/*         samples: total number of samples to be processed                  */
/* OUTPUTS: dest-> output buffer                                             */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*      To match QSound code, discuss gain value and divde into three cases  */
/*      compared to function buffer_delay_fill                               */
/*===========================================================================*/
void buffer_delay_mix32
(
    int32           *destBuf,           /* output buffer                     */
    int32           *srcBuf,            /* input buffer                      */
    int16            gainL16Q15,        /* gain to the samples               */
    delayline32_t   *delayline,         /* delayline struct                  */
    int32            delay,             /* amount of delay in sample         */
    int32            samples            /* number of samples to process      */
)
{
    int32 *destPtr = destBuf;
    int32 *srcPtr = srcBuf;
    int32 *delayPtr;
    int32 delayLength = delayline->buf_size;
    int32 delayIndex = delayline->idx;
    int32 fromDelay, i;
    int32 tmpL32; //int16 tmpL16;

    /*-----------------------------------------------------------------------*/
    
    /*--------- set delay pointer so it points to "delay" samples ago -------*/
    delayIndex -= delay;
    if (delayIndex < 0)
    {   /* delay should be less than delayLength; if not, we are in trouble  */
        delayIndex += delayLength;
    }
    delayPtr = delayline->buf + delayIndex;
    
    /*----------------------- process from delayline ------------------------*/
    if (delay > 0 && samples > 0) 
    {
        fromDelay = delay;
        if (fromDelay > samples)
        {   /* if delay amount is more than output samples, then all of them */
            /* should from delayline                                         */
            fromDelay = samples;
        }

        /*----- case 1 : if gain is unity, directly sum -----*/
        if (gainL16Q15 == Q15_ONE)
        {   
            for (i = 0; i < fromDelay; i++)
            {
                /*-- output = output + delay --*/
                *destPtr = s32_add_s32_s32_sat(*destPtr, *delayPtr++); 
                destPtr++;
                /* manage circular buffer */
                if (delayPtr == delayline->buf + delayLength)
                {
                    delayPtr = delayline->buf;
                }
            }  /* end of for (i = 0; i < fromDelay; i++) */
        }

        /*----- case 2 : if gain is minus unity, directly subtract -----*/
        else if (gainL16Q15 == Q15_MINUSONE)
        {   
            for (i = 0; i < fromDelay; i++)
            {
                /*-- output = output - delay --*/
                *destPtr = s32_sub_s32_s32_sat(*destPtr, *delayPtr++);
                destPtr++;
                /* manage circular buffer */
                if (delayPtr == delayline->buf + delayLength)
                {
                    delayPtr = delayline->buf;
                }
            }  /* end of for (i = 0; i < fromDelay; i++) */
        }

        /*----- case 3: normal case, with L16Q15 gain -----*/
        else
        {   
            for (i = 0; i < fromDelay; i++)
            {
                /*-- delay * gain --*/
                //tmpL16 = s16_extract_s40_h(s40_mult_s16_s16_shift(*delayPtr++, gainL16Q15, 1));
                tmpL32 = s32_extract_s64_l(s64_mult_s32_s16_shift(*delayPtr++, gainL16Q15, -15));

                /*-- output = output + delay * gain --*/
                *destPtr = s32_add_s32_s32_sat(*destPtr, tmpL32); 
                destPtr++;
                /* manage circular buffer */
                if (delayPtr == delayline->buf + delayLength)
                {
                    delayPtr = delayline->buf;
                }
            }  /* end of for (i = 0; i < fromDelay; i++) */
        }
        /* we have so far processed fromDelay samples */
        samples -= fromDelay;
    }

    /*----------------------- process from input buffer ---------------------*/
    if (samples > 0)
    {
    	buffer32_mix32(destPtr, srcPtr, gainL16Q15, 15, samples);
    }  /* end of if (samples > 0) */
}  /*--------------------- end of function buffer_delay_mix -----------------*/

/*===========================================================================*/
/* FUNCTION : delayline32_copy                                               */
/*                                                                           */
/* DESCRIPTION: Copies the contents of one delay line to another. The        */
/*              contents of the destination are lost. This is for 32 bit     */
/*              data.                                                        */
/*                                                                           */
/* INPUTS: delayline src-> delayline32_t                                     */
/* OUTPUTS: delayline dest-> delayline32_t                                   */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
void delayline32_copy
(
		delayline32_t *dest,          /* The destination delay line        */
		delayline32_t *source         /* The source delay line             */
)
{
	int32 samplesToCopy = 0;
	int32 *pSrcEnd = source->buf + source->buf_size;
	int32 *pSrc = source->buf + source->idx;
	int32 *pDst = dest->buf;

	if (dest->buf_size < source->buf_size)
	{
		// Some part of the source must be discarded.
		int32 numSamplesToDiscard = source->buf_size - dest->buf_size;
		pSrc += numSamplesToDiscard;
		if (pSrc > pSrcEnd)
		{
			pSrc -= source->buf_size;
		}
		samplesToCopy = dest->buf_size;
	}
	else
	{
		samplesToCopy = source->buf_size;
	}

	int32 srcDistanceToEnd = pSrcEnd - pSrc;
	if (srcDistanceToEnd < samplesToCopy)
	{
		buffer32_copy_v2(pDst, pSrc, srcDistanceToEnd);
		samplesToCopy -= srcDistanceToEnd;
		pSrc = source->buf;
		pDst += srcDistanceToEnd;
	}

	buffer32_copy_v2(pDst, pSrc, samplesToCopy);
	pDst += samplesToCopy;

	dest->idx = pDst - dest->buf;
	if (dest->idx >= dest->buf_size)
	{
		dest->idx -= dest->buf_size;
	}
} /*------------------ end of function delayline_copy ------------------------*/

/*===========================================================================*/
/* FUNCTION : buffer_delay_fill32                                            */
/*                                                                           */
/* DESCRIPTION: Store in output buffer with delayed samples, while applying  */
/*              L16Q15 gain. The samples could be from the delayline or the  */
/*              input buffer.                                                */
/*                                                                           */
/* INPUTS: dest-> output buffer                                              */
/*         src-> input buffer                                                */
/*         gainL16Q15: gain to be applied to the samples                     */
/*         delayLine-> delayline struct                                      */
/*         delay: amount of delay in sample                                  */
/*         samples: total number of samples to be processed                  */
/* OUTPUTS: dest-> output buffer                                             */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
void buffer_delay_fill32
(
   int32           *dest,              /* output buffer                     */
   int32           *src,               /* input buffer                      */
   int16            gainL16Q15,        /* gain to the samples               */
   delayline32_t   *delayline,         /* delayline struct                  */
   int32            delay,             /* amount of delay in sample         */
   int32            samples            /* number of samples to process      */
)
{
   int32 delay_buf_size = delayline->buf_size;
   int32 delay_idx = delayline->idx;
   int32 *delay_buf = delayline->buf;
   int32 from_delay=0, i;

   // set delay pointer so it points to "delay" samples ago
   delay_idx = s32_modwrap_s32_u32(delay_idx-delay, delay_buf_size);

   /*----------------------- process from delayline ------------------------*/
   if (delay > 0 && samples > 0)  {

      // if frame length less than delayline, all samples from delayline
      from_delay = delay;
      if (from_delay > samples) {
         from_delay = samples;
      }

      // case 1: unity gain, direct copy
      if (gainL16Q15 == Q15_ONE) {
         for (i = 0; i < from_delay; ++i) {
            dest[i] = delay_buf[delay_idx];
            delay_idx = s32_modwrap_s32_u32(delay_idx+1, delay_buf_size);
         }
      }

      // case 2: negative unity gain, copy negative
      else if (gainL16Q15 == Q15_MINUSONE) {
         for (i = 0; i < from_delay; ++i) {
            dest[i] = s32_neg_s32_sat(delay_buf[delay_idx]);
            delay_idx = s32_modwrap_s32_u32(delay_idx+1, delay_buf_size);
         }
      }

      // case 3: apply L16Q15 gain
      else {
         for (i = 0; i < from_delay; i++) {
            dest[i] = s32_mult_s32_s16_rnd_sat(delay_buf[delay_idx], gainL16Q15);
            delay_idx = s32_modwrap_s32_u32(delay_idx+1, delay_buf_size);
         }
      }

      samples -= from_delay;
   }

   /*----------------------- process from input buffer ---------------------*/
   if (samples > 0) {
      buffer32_fill16(dest+from_delay, src, gainL16Q15, samples);
   }
}
