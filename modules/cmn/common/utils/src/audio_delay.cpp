/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*===========================================================================*]
[* FILE NAME: audio_delay.c                                                  *]
[* DESCRIPTION:                                                              *]
[*    Functions regarding delayline                                          *]
[* FUNCTION LIST :                                                           *]
[*    delayline_reset: Clean delayline buffer and reset index                *]
[*    buffer_delay_fill: Copy delayed samples while applying L16Q15 gain     *]
[*    buffer_delay_mix: Apply L16Q15 gain and mix with delay                 *]
[*    delayline_update: Update delayline buffer with new samples             *]
[*    variable_delay_reset: Reset variable delayline                         *]
[*    variable_delay_setdelay: Setup function for variable delayline         *]
[*    variable_delay_process: Apply variable delay with rate conversion      *]
[*===========================================================================*/
#include "audio_dsp.h"
#include "audio_divide_qx.h"

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
void delayline_reset
(
    delaylineStruct *delayline          /* delayline struct                  */
)
{
    int32 i;
    int16 *bufPtr = delayline->delayBuf;
    for (i = 0; i < delayline->delayLength; i++)
    {
        *bufPtr++ = 0;
    }
    delayline->delayIndex = 0;
} /*------------------ end of function delayline_reset ----------------------*/

/*===========================================================================*/
/* FUNCTION : exchange_buffer_values                                         */
/*                                                                           */
/* DESCRIPTION: Exchanges the contents of two buffers.                       */
/*                                                                           */
/* INPUTS: pBuf1 -> Pointer to the first buffer                              */
/*         pBuf2 -> Pointer to the second buffer                             */
/*         numSamples -> number of samples in both the buffers               */
/* OUTPUTS: pBuf1 and pBuf2, with contents exchanged                         */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
static void exchange_buffer_values(int16 *pBuf1, int16 *pBuf2, const int32 numSamples)
{
   int16 tmpBuf;
   int32 i;

   for (i = 0; i < numSamples; i++)
   {
      tmpBuf = *pBuf1;
      *pBuf1 = *pBuf2;
      *pBuf2 = tmpBuf;

      pBuf1++;
      pBuf2++;
   }
}

/*===========================================================================*/
/* FUNCTION : inplace_delay                                                  */
/*                                                                           */
/* DESCRIPTION: Inplace delay certain number of samples and update delayline.*/
/*                                                                           */
/* INPUTS: inplaceBuf-> inplace buffer                                       */
/*         delayline-> delayline struct                                      */
/*         samples: total number of samples to be processed                  */
/* OUTPUTS: inplaceBuf-> inplace buffer                                      */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
void inplace_delay
(
    int16           *inplaceBuf,        /* inplace buffer                    */
    delaylineStruct *delayline,         /* delayline struct                  */
    int32            samples            /* number of samples to process      */
)
{
   int16   *delayBuf = delayline->delayBuf;
   int32    delayLength = delayline->delayLength;
   int16   *delayPtr = delayBuf + delayline->delayIndex;
   int16   *bufPtr = inplaceBuf;
   int32   samplesTillDelayBufEnd, samplesToProcess, cnt;

   samplesTillDelayBufEnd = delayLength - delayline->delayIndex;
   while (samples > 0)
   {
      samplesToProcess = s32_min_s32_s32(samples, delayLength);
      cnt = s32_min_s32_s32(samplesTillDelayBufEnd, samplesToProcess);

      //exchange  buffer and delayed samples from current location in delayBuf
      exchange_buffer_values(bufPtr, delayPtr, cnt);
      bufPtr += cnt;
      delayPtr += cnt;

      if (cnt == samplesTillDelayBufEnd)
      {
         //exchange  buffer and delayed samples from first location location in delayBuf
         delayPtr = delayBuf;
         cnt = samplesToProcess - cnt;

         exchange_buffer_values(bufPtr, delayPtr, cnt);
         bufPtr += cnt;
         delayPtr += cnt;
      }

      samples -= samplesToProcess;
   }
   /* update delayline index */
   delayline->delayIndex = (int16)(delayPtr - delayBuf);
} /*--------------------- end of function inplace_delay ---------------------*/

/*===========================================================================*/
/* FUNCTION : delayline_set                                                  */
/*                                                                           */
/* DESCRIPTION: Set delayline delay length. Reset delay index and            */
/*              delay buffer                                                 */
/*                                                                           */
/* INPUTS: delayline-> delayline struct                                      */
/* OUTPUTS: delayline-> delayline struct                                     */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
boolean delayline_set
(
    delaylineStruct *delayLine,         /* delayline struct                  */
    int32 delayLen                      /* delay length                      */
)
{
    delayLine->delayLength = delayLen;
    delayline_reset(delayLine);
    return true;
} /*------------------ end of function delayline_set ------------------------*/

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
void delayline_copy
(
		delaylineStruct *dest,          /* The destination delay line        */
		delaylineStruct *source         /* The source delay line             */
)
{
	int32 samplesToCopy = 0;
	int16 *pSrcEnd = source->delayBuf + source->delayLength;
	int16 *pSrc = source->delayBuf + source->delayIndex;
	int16 *pDst = dest->delayBuf;

	if (dest->delayLength < source->delayLength)
	{
		// Some part of the source must be discarded.
		int32 numSamplesToDiscard = source->delayLength - dest->delayLength;
		pSrc += numSamplesToDiscard;
		if (pSrc > pSrcEnd)
		{
			pSrc -= source->delayLength;
		}
		samplesToCopy = dest->delayLength;
	}
	else
	{
		samplesToCopy = source->delayLength;
	}

	int32 srcDistanceToEnd = pSrcEnd - pSrc;
	if (srcDistanceToEnd < samplesToCopy)
	{
		while (pSrc < pSrcEnd)
		{
			*pDst++ = *pSrc++;
		}
		samplesToCopy -= srcDistanceToEnd;
		pSrc = source->delayBuf;
	}

	while (samplesToCopy > 0)
	{
		*pDst++ = *pSrc++;
		samplesToCopy--;
	}

	dest->delayIndex = pDst - dest->delayBuf;
	if (dest->delayIndex >= dest->delayLength)
	{
		dest->delayIndex -= dest->delayLength;
	}
} /*------------------ end of function delayline_copy ------------------------*/

/*===========================================================================*/
/* FUNCTION : buffer_delay_fill                                              */
/*                                                                           */
/* DESCRIPTION: Store in output buffer with delayed samples, while applying  */
/*              L16Q15 gain. The samples could be from the delayline or the  */
/*              input buffer.                                                */
/*                                                                           */
/* INPUTS: destBuf-> output buffer                                           */
/*         srcBuf-> input buffer                                             */
/*         gainL16Q15: gain to be applied to the samples                     */
/*         delayline-> delayline struct                                      */
/*         delay: amount of delay in sample                                  */
/*         samples: total number of samples to be processed                  */
/* OUTPUTS: destBuf-> output buffer                                          */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*      To match QSound code, there is no discussion of the gain values,     */
/*      compared to function buffer_delay_mix                                */
/*===========================================================================*/
void buffer_delay_fill
(
    int16           *destBuf,           /* output buffer                     */
    int16           *srcBuf,            /* input buffer                      */
    int16            gainL16Q15,        /* gain to the samples               */
    delaylineStruct *delayline,         /* delayline struct                  */
    int32            delay,             /* amount of delay in sample         */
    int32            samples            /* number of samples to process      */
)
{
    int16 *destPtr = destBuf;
    int16 *srcPtr = srcBuf;
    int16 *delayPtr;
    int32 delayLength = delayline->delayLength;
    int32 delayIndex = delayline->delayIndex;
    int32 fromDelay, i;
    
    /*-----------------------------------------------------------------------*/
    
    /*--------- set delay pointer so it points to "delay" samples ago -------*/
    delayIndex -= delay;
    if (delayIndex < 0)
    {   /* delay should be less than delayLength; if not, we are in trouble  */
        delayIndex += delayLength;
    }
    delayPtr = delayline->delayBuf + delayIndex;
    
    /*----------------------- process from delayline ------------------------*/
    if (delay > 0 && samples > 0) 
    {
        fromDelay = delay;
        if (fromDelay > samples)
        {   /* if delay amount is more than output samples, then all of them */
            /* should from delayline                                         */
            fromDelay = samples;
        }
        if (gainL16Q15 == Q15_ONE)
		{
			for (i = 0; i < fromDelay; i++)
			{
				/*-- output = delayed samples * gain --*/
				*destPtr++ = *delayPtr++;
				
				/* manage circular buffer */
                if (delayPtr == delayline->delayBuf + delayLength)
				{
					delayPtr = delayline->delayBuf;
                }
			}  /* end of for (i = 0; i < fromDelay; i++) */
		}
        else if(gainL16Q15 == Q15_MINUSONE)
		{
			for (i = 0; i < fromDelay; i++)
			{
				/*-- output = delayed samples * gain --*/
                *destPtr++ = -*delayPtr++;
            
               /* manage circular buffer */
				if (delayPtr == delayline->delayBuf + delayLength)
				{
                delayPtr = delayline->delayBuf;
				}
			}  /* end of for (i = 0; i < fromDelay; i++) */
		}
		else
		{
			for (i = 0; i < fromDelay; i++)
			{
				/*-- output = delayed samples * gain --*/
                *destPtr++ = s16_extract_s64_h_rnd(s40_mult_s16_s16_shift(*delayPtr++, gainL16Q15, 1));
            
                /* manage circular buffer */
				if (delayPtr == delayline->delayBuf + delayLength)
				{
                delayPtr = delayline->delayBuf;
				}
			}  /* end of for (i = 0; i < fromDelay; i++) */
		}/* end of if (gainL16Q15 = Q15_ONE)*/
        /* so far processed "fromDelay" samples */
        samples -= fromDelay; 
    }  /* end of if (delay > 0 && samples > 0) */


    /*----------------------- process from input buffer ---------------------*/
    if (samples > 0)
    {   
        buffer_fill(destPtr, srcPtr, gainL16Q15, samples);
    }  /* end of if (samples > 0) */
}  /*------------------ end of function buffer_delay_fill -------------------*/


/*===========================================================================*/
/* FUNCTION : buffer_delay_mix                                               */
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
void buffer_delay_mix
(
    int16           *destBuf,           /* output buffer                     */
    int16           *srcBuf,            /* input buffer                      */
    int16            gainL16Q15,        /* gain to the samples               */
    delaylineStruct *delayline,         /* delayline struct                  */
    int32            delay,             /* amount of delay in sample         */
    int32            samples            /* number of samples to process      */
)
{
    int16 *destPtr = destBuf;
    int16 *srcPtr = srcBuf;
    int16 *delayPtr;
    int32 delayLength = delayline->delayLength;
    int32 delayIndex = delayline->delayIndex;
    int32 fromDelay, i;
    int16 tmpL16;

    /*-----------------------------------------------------------------------*/
    
    /*--------- set delay pointer so it points to "delay" samples ago -------*/
    delayIndex -= delay;
    if (delayIndex < 0)
    {   /* delay should be less than delayLength; if not, we are in trouble  */
        delayIndex += delayLength;
    }
    delayPtr = delayline->delayBuf + delayIndex;
    
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
                *destPtr = s16_add_s16_s16_sat(*destPtr, *delayPtr++); 
                destPtr++;
                /* manage circular buffer */
                if (delayPtr == delayline->delayBuf + delayLength)
                {
                    delayPtr = delayline->delayBuf;
                }
            }  /* end of for (i = 0; i < fromDelay; i++) */
        }

        /*----- case 2 : if gain is minus unity, directly subtract -----*/
        else if (gainL16Q15 == Q15_MINUSONE)
        {   
            for (i = 0; i < fromDelay; i++)
            {
                /*-- output = output - delay --*/
                *destPtr = s16_sub_s16_s16_sat(*destPtr, *delayPtr++);
                destPtr++;
                /* manage circular buffer */
                if (delayPtr == delayline->delayBuf + delayLength)
                {
                    delayPtr = delayline->delayBuf;
                }
            }  /* end of for (i = 0; i < fromDelay; i++) */
        }

        /*----- case 3: normal case, with L16Q15 gain -----*/
        else
        {   
            for (i = 0; i < fromDelay; i++)
            {
                /*-- delay * gain --*/
                tmpL16 = s16_extract_s64_h_rnd(s40_mult_s16_s16_shift(*delayPtr++, gainL16Q15, 1));
                /*-- output = output + delay * gain --*/
                *destPtr = s16_add_s16_s16_sat(*destPtr, tmpL16); 
                destPtr++;
                /* manage circular buffer */
                if (delayPtr == delayline->delayBuf + delayLength)
                {
                    delayPtr = delayline->delayBuf;
                }
            }  /* end of for (i = 0; i < fromDelay; i++) */
        }
        /* we have so far processed fromDelay samples */
        samples -= fromDelay;
    }

    /*----------------------- process from input buffer ---------------------*/
    if (samples > 0)
    {
        buffer_mix(destPtr, srcPtr, gainL16Q15, samples);
    }  /* end of if (samples > 0) */
}  /*--------------------- end of function buffer_delay_mix -----------------*/


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

#if  defined(__qdsp6__)
#else
void delayline_update
(
    delaylineStruct *delayline,         /* delayline struct                  */
    int16           *srcBuf,            /* input buffer (new samples)        */
    int32            samples            /* input buffer sample size          */
)
{
    int32 i, updateLength;
    int16 *srcPtr; 
    int16 *delayPtr = delayline->delayBuf + delayline->delayIndex;

    /*-----------------------------------------------------------------------*/
   
    /*------------------------ determine lengths ----------------------------*/
    /*--- if  input buffer is longer than delayline ---*/
    if (samples > delayline->delayLength)
    {
        /* whole delayline gets updated with part of input buffer samples */
        updateLength = s16_extract_s32_l(delayline->delayLength);
        /* the last "delayLength" samples from input buffer get used */
        srcPtr = srcBuf + samples - delayline->delayLength;
    }
    /*--- if input buffer is shorter than delayline ---*/
    else
    {   /* update "samples" number of samples */
        updateLength = samples;
        /* using the whole input buffer samples */
        srcPtr = srcBuf;
    }

    /*-------------------------- update samples -----------------------------*/
    for (i = 0; i < updateLength; i++)
    {
        /* copy from input buffer */
        *delayPtr++ = *srcPtr++;
        /* manage circular buffer */
        if (delayPtr == delayline->delayBuf + delayline->delayLength)
        {
            delayPtr = delayline->delayBuf;
        }
    }  /* end of for (i = 0; i < updateLength; i++) */

    /* stored the index value of delayline back into the struct */
    delayline->delayIndex = (int32)(delayPtr - delayline->delayBuf);
}  /*-------------------- end of function delayline_update ------------------*/

#endif
/*===========================================================================*/
/* FUNCTION : variable_delay_reset                                           */
/*                                                                           */
/* DESCRIPTION: Reset variable delayline struct                              */
/*                                                                           */
/* INPUTS: variableDelay-> variable delay struct                             */
/* OUTPUTS: variableDelay-> variable delay struct                            */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
void variable_delay_reset
(
    variableDelayStruct *variableDelay  /* variable delay struct             */
)
{
    /* reset buffer rate converter */
    buf_rate_converter_reset(&variableDelay->converter);
    /* reset delayline */
    delayline_reset(&variableDelay->delayline);
    /* set buffer rate converter rate to be normal playback rate */
    buf_rate_converter_setrate(&variableDelay->converter,NORMAL_PLAYBACK_RATE);
} /*------------------ end of function variable_delay_reset -----------------*/

/*===========================================================================*/
/* FUNCTION : variable_delay_setdelay                                        */
/*                                                                           */
/* DESCRIPTION: According to new delay and the time (sample) transition to   */
/*              achieve the new delay, setup params in variable delay struct.*/
/*                                                                           */
/* INPUTS: variableDelay-> variable delay struct                             */
/*         newDelay: desired new delay in sample                             */
/*         steps: ramp samples to achive the new delay                       */
/* OUTPUTS: variableDelay-> variable delay struct                            */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
void variable_delay_setdelay
(
    variableDelayStruct *variableDelay, /* variable delay struct             */
    int16                newDelay,      /* new delay in sample               */
    int32                steps          /* sample dur. to change to new delay*/
)
{
    int32       delayChange;
    int32       rateChangeL32Q16;
    int32      targetRateL32Q16;

    /*-----------------------------------------------------------------------*/

    if (steps == 0)
    {   
        /* immediately set the current delay to the new value */
        variableDelay->currentDelay = newDelay;
    }
    else /* if (steps != 0) */
    {
        delayChange = newDelay - variableDelay->currentDelay;
        
        if (delayChange >= steps)
        {
            /* Delay change is greater than step samples, which means big    */
            /* delay is required to happen really fast. This is equivolent   */
            /* to really slow playback. Since we don't want to stop or play  */
            /* backwards, we limit it to a very slow playback rate instead.  */

            buf_rate_converter_setrate(&variableDelay->converter, 1);
        }
        else /* if (delayChange < steps) */
        { 
            /* After "steps" have played, want the delay to have changed to  */
			/* the new value. This implies that the number of output samples */
            /* should be (steps) and the number of input samples             */
            /* (steps-delayChange), (an increase in delay requires a decrease*/
            /* in inputs updated to the delay buffer), hence                 */
			/*                                                               */
			/*    playback rate = (steps-delayChange)/steps                  */
			/*                  = 1-delayChange/steps                        */
			/*                                                               */
			/* expressed in 16.16 fixed point form.  To avoid overflow or    */
			/* underflow in the delay buffer we ought to round the division  */
			/* towards zero.                                                 */

            /* calculate playback rate change, Q16 */
#if BIT_EXACT_WITH_QSOUND
            rateChangeL32Q16 = Q16_divide_truncated(delayChange, steps);
#else
            rateChangeL32Q16 = divide_qx(delayChange, steps, 16);
#endif

            /* calculate target playback rate = 1 - ratechange, Q16          */
            /* if delay change is positive, rate should decrease             */
            /* if delay change is negative, rate should increase, so use s16_sub_s16_s16 */
			targetRateL32Q16 = s32_sub_s32_s32(Q16_ONE, rateChangeL32Q16);
			
            /* set playback rate to the target rate */
            buf_rate_converter_setrate(
                                 &variableDelay->converter, targetRateL32Q16);
        } /* end of else if (delayChange < steps) */
    } /* end of else if (steps != 0) */
} /*-------------- end of function variable_delay_setdelay ------------------*/


/*===========================================================================*/
/* FUNCTION : variable_delay_process                                         */
/*                                                                           */
/* DESCRIPTION: According to new delay and the time (sample) transition to   */
/*              achieve the new delay, setup params in variable delay struct.*/
/*                                                                           */
/* INPUTS: variableDelay-> variable delay struct                             */
/*         destBuf-> output buffer                                           */
/*         srcBuf-> input buffer                                             */
/*         dlyTmpBuf-> scratch buffer 1                                      */
/*         dlyMixBuf-> scratch buffer 2                                      */
/*         feedbackL16Q15: feedback gain                                     */
/*         samples: number of samples to process                             */
/* OUTPUTS: destBuf-> output buffer                                          */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*      The two scratch buffers are provided outside this function. dlyTmpBuf*/
/* is used to hold a block of samples from delayline, and dlyMixBuf is used  */
/* during delayline update to mix input samples with feedback samples. The   */
/* BLOCKSIZE of postprocessing is not visible to audio_dsp functions, so we  */
/* pass the scratch buffers from outside.                                    */
/*===========================================================================*/
uint32 variable_delay_process
(
    variableDelayStruct *vDelay,        /* variable delay struct             */
    int16               *destBuf,       /* output buffer                     */
    int16               *srcBuf,        /* input buffer                      */
    int16               *dlyTmpBuf,     /* scratch buffer 1                  */
    int16               *dlyMixBuf,     /* scratch buffer 2                  */
    int16                feedbackL16Q15,/* feedback gain                     */
    int32                samples        /* samples to process                */
)
{
    int32       blockInputs, blockOutputs, outputsWritten, i;
    int16       *dlyTmpBufPtr, *delayPtr;
    int32       delayIndex;
    int32       delayLength = vDelay->delayline.delayLength;
    int32       currentDelay = vDelay->currentDelay;

    /*-----------------------------------------------------------------------*/

    outputsWritten = 0;
    while ((blockOutputs = samples - outputsWritten) > 0)
    {
        if (currentDelay < 0)
        {   /* make sure current delay >= 0 */
             return PPERR_DELAY_NEGATIVE;
        }
        /* predict inputs from block outputs */
        blockInputs = rateConvertState_predict_inputs(
                            &vDelay->converter.srcState, blockOutputs);
        /* if it's greater than the current delay, use current delay */
        if (currentDelay != 0)
        {
            blockInputs = s32_min_s32_s32(blockInputs, currentDelay);
        }

        /*------------------- read from delay and convert -------------------*/
		  /* Read from delay and apply sample rate conversion which implements */
        /* smoothly varying delay. Amount of audio actually read and written */
        /* (blockInputs and blockOutputs) is updated.                        */

		if (currentDelay == 0)
        {
            /* if no delay, convert directly from input */
            buf_rate_converter_convert(destBuf+outputsWritten, &blockOutputs, 
                srcBuf+outputsWritten, &blockInputs, &vDelay->converter);
        }
        else /* if (currentDelay != 0) */
        {
            /* set delayline index to delayed position */
            delayIndex = vDelay->delayline.delayIndex - currentDelay;
            
            /* if less than 0, it crossed the circular buffer boundary */
            if (delayIndex < 0)
            {   
                delayIndex += delayLength;
            }

            /* set delay pointer to the delay buf starting address */
            delayPtr = vDelay->delayline.delayBuf + delayIndex;
            
            /* start copying data */
            dlyTmpBufPtr = dlyTmpBuf;       
            for (i = 0; i < blockInputs; i++)
            {
                /* copy data from delayline to the temp buffer */
                *dlyTmpBufPtr++ = *delayPtr++;
                /* manage circular buffer boundary */
                if (delayPtr == vDelay->delayline.delayBuf + delayLength)
                {   
                    delayPtr = vDelay->delayline.delayBuf;
                }
            } /* end of for (i = 0; i < blockInputs; i++) */
            /* do the smooth rate conversion */
            buf_rate_converter_convert(
                destBuf+outputsWritten, &blockOutputs, dlyTmpBuf, 
                &blockInputs, &vDelay->converter);
        } /* end of else if (currentDelay != 0) */

        /*------------------------- update delayline ------------------------*/
        if (feedbackL16Q15 != 0 && currentDelay > 0)
        {   /* if there is feedback, first copy input buf into temp buf */
            buffer_copy(dlyMixBuf, srcBuf + outputsWritten, blockOutputs);
            /* mix feedback part */
            buffer_mix(dlyMixBuf, destBuf + outputsWritten, 
                                                 feedbackL16Q15, blockOutputs);
            /* update the delayline with this mix */
            delayline_update(&vDelay->delayline,dlyMixBuf,blockOutputs);
        }
        else /* if no feedback */
        {
            /* update the delayline with input samples */
            delayline_update(&vDelay->delayline, 
                                   srcBuf + outputsWritten, blockOutputs);
        } /* end of else */

        /* update current delay and block sample counters */
        currentDelay += blockOutputs - blockInputs;
        /* ensure current delay is non-negative */
        if (currentDelay < 0)
        {
            currentDelay = 0;
        }
        outputsWritten += blockOutputs;
    } /* end of while (blockOutputs = samples - outputsWritten > 0) */
    
    /* store the current delay back */
    vDelay->currentDelay = currentDelay;
    
    /*---------------------------- some sanity check ------------------------*/
    if (currentDelay < 0 || currentDelay > delayLength)
    {
        return PPERR_DELAY_INVALID;
    }
	return PPSUCCESS;
} /*--------------- end of function variable_delay_process ------------------*/
