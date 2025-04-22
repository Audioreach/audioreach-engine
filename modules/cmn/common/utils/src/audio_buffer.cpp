/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*===========================================================================*]
[* FILE NAME: audio_buffer.c                                                 *]
[* DESCRIPTION:                                                              *]
[*    Functions regarding audio buffer                                       *]
[* FUNCTION LIST :                                                           *]
[*    buffer_empty: Set all samples in a buffer to zero                      *]
[*    buffer_copy: Copy one buffer to another                                *]
[*    buffer_fill: Apply L16Q15 gain to a buffer and store result in another *]
[*    buffer_mix: Apply L16Q15 gain to a buffer and mix result into another  *]
[*===========================================================================*/
#include "audio_dsp.h"
#if defined CAPI_STANDALONE
#include "capi_util.h"
#elif defined APPI_EXAMPLE_STANDALONE
#include "appi_util.h"
#else
#include <stringl.h>
#endif
#if ((defined __hexagon__) || (defined __qdsp6__))
#include <string.h>
#endif
/*===========================================================================*/
/* FUNCTION : buffer_empty                                                   */
/*                                                                           */
/* DESCRIPTION: Set all samples of a buffer to zero.                         */
/*                                                                           */
/* INPUTS: buf-> buffer to be processed                                      */
/*         samples: size of buffer                                           */
/* OUTPUTS: buf-> buffer to be processed                                     */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
void buffer_empty
(
    int16           *buf,               /* buffer to be processed            */
    uint32           samples            /* number of samples in this buffer  */
)
{
#ifndef __qdsp6__
    uint32 i;

    for (i = 0; i < samples; i++)
    {
        *buf++ = 0;
    }
#else
	memset(buf, 0, samples<<1);
#endif	
} /*---------------------- end of function buffer_empty ---------------------*/


/*===========================================================================*/
/* FUNCTION : buffer_copy                                                    */
/*                                                                           */
/* DESCRIPTION: Copy from one buffer to another                              */
/*                                                                           */
/* INPUTS: destBuf-> output buffer                                           */
/*         srcBuf-> input buffer                                             */
/*         samples: size of buffer                                           */
/* OUTPUTS: destBuf-> output buffer                                          */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
void buffer_copy
(
    int16           *destBuf,           /* output buffer                     */
    int16           *srcBuf,            /* input buffer                      */
    uint32           samples            /* number of samples to process      */
)
{
#ifndef __qdsp6__
    uint32 i;
    for (i = 0; i < samples; i++)
    {
        *destBuf++ = *srcBuf++;
    }
#else
	memscpy(destBuf, samples<<1, srcBuf, samples<<1);
#endif	
} /*---------------------- end of function buffer_copy ----------------------*/

/*===========================================================================*/
/* FUNCTION : buffer_fill                                                    */
/*                                                                           */
/* DESCRIPTION: Apply L16Q15 gain to input and store the result to the       */
/*              output buffer.                                               */
/*                                                                           */
/* INPUTS: destBuf-> output buffer                                           */
/*         srcBuf-> input buffer                                             */
/*         gainL16Q15: gain to be applied to the samples                     */
/*         samples: total number of samples to be processed                  */
/* OUTPUTS: destBuf-> output buffer                                          */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*      To match QSound code, there is no discussion of the gain values,     */
/*      compared to function buffer_mix                                      */
/*===========================================================================*/

void buffer_fill
(
    int16           *destBuf,           /* output buffer                     */
    int16           *srcBuf,            /* input buffer                      */
    int16            gainL16Q15,        /* gain to the samples               */
    uint32           samples            /* number of samples to process      */
)
{
#ifndef __qdsp6__
    uint32 i;
    
    if (gainL16Q15 == Q15_ONE)
    {
       for (i = 0; i < samples; i++)
       {
          *destBuf++ = *srcBuf++;
       }
    }
    else if(gainL16Q15 == Q15_MINUSONE)
    {
       for (i = 0; i < samples; i++)
       {
          *destBuf++ = -*srcBuf++;
       }
    }
    else
    {
       for (i = 0; i < samples; i++)
       {
           /* output = input * gain --*/
           *destBuf++ = s16_extract_s64_h_rnd(s40_mult_s16_s16_shift(*srcBuf++, gainL16Q15 ,1));
       }
    }
#else
	uint32 i, count, address;

	/************************************************************************/
	/* Following code process samples till destination buffer is 8 byte     */
	/* Aligned. Destination buffer is made to take aligned address after    */
	/* first three samples. This is avoid out of boundary memory access     */
	/* that can happen on source buffer in assembly code.                   */
	/************************************************************************/
	address = (uint32)(&destBuf[0]);
	//number of samples to make destination aligned.
    count = 4 - ((address&6)>>1);
    if (count < 3)
    {
	   // destination is made to take next aligned address.
	   count = count + 4;  
    }	
	count = s32_min_s32_s32(samples, count);
	
	if (count)
	{			
		if (gainL16Q15 == Q15_ONE)
		{
			for (i=count; i; i--)
			{
				*destBuf++ = *srcBuf++;
			}
		}
		else if(gainL16Q15 == Q15_MINUSONE)
		{
			for (i=count; i; i--)
			{
				*destBuf++ = -*srcBuf++;
			}
		}
		else
		{
			for (i=count; i; i--)
			{
				/* output = input * gain --*/
             *destBuf++ = s16_extract_s64_h_rnd(Q6_P_mpy_RlRl_s1(*srcBuf++, gainL16Q15));
			}
		}
		samples = samples-count;
	}

	/***********************************************************************/
	/* To avoid out of boundary memory access on source buffer last 4      */
        /* samples are processed in c. In adition to these 4 samples, if sample*/
	/* count is not multiple of 4, remaining samples will also be processed*/
	/* in c code.                                                          */
	/***********************************************************************/
	/*assembly code will execute if number of samples >=12. Assembly code  */
	/*processes samples in multiples of 4 and if any samples are left they */
	/*will be processed in c code. >= 12 is because of loop unrolling in   */
	/* assembly to better pack pipe line. >=(12+4) condition is to account */
	/* last 4 samples that will be processed in c code                     */
	/***********************************************************************/
	if (samples>=(12+4))
	{
		// last 4 samples will be processed in c to avoid out of boudary 
		// memory access on source pointers
	        samples = samples - 4;
		
		//assembly
		buffer_fill_asm(destBuf, srcBuf, gainL16Q15, samples);

		/*update address pointers as they will not be updated in function  */
		/* call                                                            */
		srcBuf  = srcBuf + (samples&(~3));
		destBuf = destBuf + (samples&(~3));
		
		// samples to be processed in c
		samples = 4 + (samples&3);
	}

	/*process remaining samples                                            */		
	if (samples)
	{	
		if (gainL16Q15 == Q15_ONE)
		{
			for (i = samples; i; i--)
			{
				*destBuf++ = *srcBuf++;
			}
		}
		else if(gainL16Q15 == Q15_MINUSONE)
		{
			for (i = samples; i; i--)
			{
				*destBuf++ = -*srcBuf++;
			}
		}
		else
		{
			for (i = samples; i; i--)
			{
				/* output = input * gain --*/
				*destBuf++ = s16_extract_s64_h_rnd(Q6_P_mpy_RlRl_s1(*srcBuf++, gainL16Q15));
			}
		}
	}
#endif
}  /*--------------------- end of function buffer_fill ----------------------*/

/*===========================================================================*/
/* FUNCTION : buffer_mix                                                     */
/*                                                                           */
/* DESCRIPTION: Apply L16Q15 gain to input and mix (sum) it into a running   */
/*              output buffer.                                               */
/*                                                                           */
/* INPUTS: destBuf-> output buffer                                           */
/*         srcBuf-> input buffer                                             */
/*         gainL16Q15: gain to be applied to the samples                     */
/*         samples: total number of samples to be processed                  */
/* OUTPUTS: destBuf-> output buffer                                          */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*      To match QSound code, discuss gain value and divde into three cases  */
/*      compared to function buffer_fill                                     */
/*===========================================================================*/

void buffer_mix
(
    int16           *destBuf,           /* output buffer                     */
    int16           *srcBuf,            /* input buffer                      */
    int16            gainL16Q15,        /* gain to the samples               */
    uint32           samples            /* number of samples to process      */
)
{
#ifndef __qdsp6__
    uint32 i;
    int16 tmpL16;

    /*-----------------------------------------------------------------------*/
    /*------------ case 1 : if gain is unity, directly sum ------------------*/
    if (gainL16Q15 == Q15_ONE)
    {   
        for (i = 0; i < samples; i++)
        {
            /*-- output = output + input --*/
            *destBuf = s16_add_s16_s16_sat(*destBuf, *srcBuf++); 
            destBuf++;
        }
    }  /* end of if (gainL16Q15 == Q15_ONE) */

    /*------------ case 2 : if gain is minus unity, directly subtract -------*/
    else if (gainL16Q15 == Q15_MINUSONE)
    {
        for (i = 0; i < samples; i++)
        {
            /*-- output = output - input --*/
            *destBuf = s16_sub_s16_s16_sat(*destBuf, *srcBuf++); 
            destBuf++;
        }
    }  /* end of else if (gainL16Q15 == Q15_MINUSONE) */

    /*------------ case 3: normal case, with L16Q15 gain --------------------*/
    else
    {
        for (i = 0; i < samples; i++)
        {
            /*-- input * gain --*/
            tmpL16 = s16_extract_s64_h_rnd(s40_mult_s16_s16_shift(*srcBuf++, gainL16Q15 ,1));
            /*-- output = output + input * gain --*/
            *destBuf = s16_add_s16_s16_sat(*destBuf, tmpL16); 
            destBuf++;
        }
    }  /* end of else */
#else
	uint32 i, count, address;
	int16 tmpL16;

	/************************************************************************/
	/* Following code process samples till destination buffer is 8 byte     */
	/* Aligned. Destination buffer is made to take aligned address after    */
	/* first three samples. This is avoid out of boundary memory access     */
	/* that can happen on source buffer in assembly code.                   */
	/************************************************************************/
	address = (uint32)(&destBuf[0]);
	//number of samples to make destination aligned.
    count = 4 - ((address&6)>>1);
    if (count < 3)
    {
	   // destination is made to take next aligned address.
	   count = count + 4;  
    }	
	count = s32_min_s32_s32(samples, count);

	if(count)
	{
		if (gainL16Q15 == Q15_ONE)
		{   
			for (i=count; i; i--)
			{
				/*-- output = output + input --*/
				*destBuf = s16_add_s16_s16_sat(*destBuf, *srcBuf++); 
				destBuf++;
			}
		}  /* end of if (gainL16Q15 == Q15_ONE) */

		/*------------ case 2 : if gain is minus unity, directly subtract -------*/
		else if (gainL16Q15 == Q15_MINUSONE)
		{
			for (i=count; i; i--)
			{
				/*-- output = output - input --*/
				*destBuf = s16_sub_s16_s16_sat(*destBuf, *srcBuf++); 
				destBuf++;
			}
		}  /* end of else if (gainL16Q15 == Q15_MINUSONE) */

		/*------------ case 3: normal case, with L16Q15 gain --------------------*/
		else
		{
			for (i=count; i; i--)
			{
				/*-- input * gain --*/
				tmpL16 = s16_extract_s64_h_rnd(Q6_P_mpy_RlRl_s1(*srcBuf++, gainL16Q15));
				/*-- output = output + input * gain --*/
				*destBuf = s16_add_s16_s16_sat(*destBuf, tmpL16); 
				destBuf++;
			}
		}  /* end of else */
		samples = samples-count;
	}

	/***********************************************************************/
	/* To avoid out of boundary memory access on source buffer last 4      */
        /* samples are processed in c. In adition to these 4 samples, if sample*/
	/* count is not multiple of 4, remaining samples will also be processed*/
	/* in c code.                                                          */
	/***********************************************************************/
	/*assembly code will execute if number of samples >=8. Assembly code   */
	/*processes samples in multiples of 4 and if any samples are left they */
	/*will be processed in c code. >= 8 is because of loop unrolling in    */
	/* assembly to better pack pipe line. >=(8+4) condition is to account  */
	/* last 4 samples that will be processed in c code                     */
	/***********************************************************************/
	if (samples>=(8+4))
	{
	    // last 4 samples will be processed in c to avoid out of boudary 
		// memory access on source pointers
	        samples = samples - 4;
		
		//assembly
		buffer_mix_asm(destBuf, srcBuf, gainL16Q15, samples);

		/*update address pointers as they will not be updated in function  */
		/* call                                                            */
		srcBuf  = srcBuf + (samples&(~3));
		destBuf = destBuf + (samples&(~3));
		
		// samples to be processed in c
		samples = 4 + (samples&3);
	}

	/*process remaining samples                                            */		
	if (samples)
	{	
		if (gainL16Q15 == Q15_ONE)
		{   
			for (i=samples; i; i--)
			{
				/*-- output = output + input --*/
				*destBuf = s16_add_s16_s16_sat(*destBuf, *srcBuf++); 
				destBuf++;
			}
		}  /* end of if (gainL16Q15 == Q15_ONE) */

		/*------------ case 2 : if gain is minus unity, directly subtract -------*/
		else if (gainL16Q15 == Q15_MINUSONE)
		{
			for (i=samples; i; i--)
			{
				/*-- output = output - input --*/
				*destBuf = s16_sub_s16_s16_sat(*destBuf, *srcBuf++); 
				destBuf++;
			}
		}  /* end of else if (gainL16Q15 == Q15_MINUSONE) */

		/*------------ case 3: normal case, with L16Q15 gain --------------------*/
		else
		{
			for (i=samples; i; i--)
			{
				/*-- input * gain --*/
				tmpL16 =  s16_extract_s64_h_rnd(Q6_P_mpy_RlRl_s1(*srcBuf++, gainL16Q15));
				/*-- output = output + input * gain --*/
				*destBuf = s16_add_s16_s16_sat(*destBuf, tmpL16); 
				destBuf++;
			}
		}  /* end of else */
	}
#endif
}  /*---------------------- end of function buffer_mix ----------------------*/


/*===========================================================================*/
/* FUNCTION : buffer_fill_mix                                                */
/*                                                                           */
/* DESCRIPTION: Apply L16Q15 gain to input and mix (sum) it into a running   */
/*              output buffer.                                               */
/*                                                                           */
/* INPUTS: dest-> output buffer                                              */
/*         src1-> input buffer 1                                             */
/*         src1-> input buffer 2                                             */
/*         gainL16Q15: gain to be applied to the samples                     */
/*         samples: total number of samples to be processed                  */
/* OUTPUTS: destBuf-> output buffer                                          */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*    The function is implemented by merging the buffer_fill,buffer_mix      */
/*    functions                                                              */
/*===========================================================================*/
void buffer_fill_mix
(
    int16           *src1,           /* Input1 buffer                     */
    int16           *src2,           /* input2 buffer                     */
    int16           *dst,            /* Output buffer                     */
    int16            gainL16Q15,     /* gain to the samples               */
    int16            samples         /* number of samples to process      */
)
{
#ifndef __qdsp6__
	int16 i, tmpL16;
	/*-----------------------------------------------------------------------*/
	/*------------ case 1 : if gain is unity, directly sum ------------------*/
	if (gainL16Q15 == Q15_ONE)
	{
		for (i = 0; i < samples; i++)
		{
			/*-- output = src1 + src2 --*/
			*dst = s16_add_s16_s16_sat(*src1++, *src2++);
			dst++;
		}
	} 
	/*------------ case 2 : if gain is minus unity, directly subtract -------*/
	else if (gainL16Q15 == Q15_MINUSONE)
	{
		for (i = 0; i < samples; i++)
		{
			/*-- output = src1 - src2 --*/
			*dst = s16_sub_s16_s16_sat(*src1++, *src2++);
			dst++;
		}
	}
	/*------------ case 3: normal case, with L16Q15 gain --------------------*/
	else
	{
		for (i = 0; i < samples; i++)
		{
			/*-- input * gain --*/
			tmpL16 = s16_extract_s40_h(s40_mult_s16_s16_shift(*src2++, gainL16Q15 ,1));
			/*-- output = output + input * gain --*/
			*dst = s16_add_s16_s16_sat(*src1++, tmpL16);
			dst++;
		}
	}
#else

	int16 i, tmpL16;
	uint32 count, address;

	/************************************************************************/
	/* Following code process samples till destination buffer is 8 byte     */
	/* Aligned. Destination buffer is made to take aligned address after    */
	/* first three samples. This is avoid out of boundary memory access     */
	/* that can happen on source buffer (on left side) in assembly code.    */
	/************************************************************************/
	address = (uint32)(&dst[0]);
	//number of samples to make destination aligned.
    count = 4 - ((address&6)>>1);
    if (count < 3)
    {
	   // destination is made to take next aligned address
	   count = count + 4;  
    }	
	count = s32_min_s32_s32(samples, count);

	if(count)
	{
		if (gainL16Q15 == Q15_ONE)
		{
			for (i=count; i; i--)
			{
				/*-- output = src1 + src2 --*/
				*dst = s16_add_s16_s16_sat(*src1++, *src2++);
				dst++;
			}
		} 
		/*------------ case 2 : if gain is minus unity, directly subtract -------*/
		else if (gainL16Q15 == Q15_MINUSONE)
		{
			for (i=count; i; i--)
			{
				/*-- output = src1 - src2 --*/
				*dst = s16_sub_s16_s16_sat(*src1++, *src2++);
				dst++;
			}
		}
		/*------------ case 3: normal case, with L16Q15 gain --------------------*/
		else
		{
			for (i=count; i; i--)
			{
				/*-- input * gain --*/
				tmpL16 = s16_extract_s40_h(Q6_P_mpy_RlRl_s1(*src2++, gainL16Q15));
				/*-- output = output + input * gain --*/
				*dst = s16_add_s16_s16_sat(*src1++, tmpL16);
				dst++;
			}
		}
		samples = samples-count;
	}
    
	/***********************************************************************/
	/* To avoid out of boundary memory access on source buffer last 4      */
        /* samples are processed in c. In adition to these 4 samples, if sample*/
	/* count is not multiple of 4, remaining samples will also be processed*/
	/* in c code.                                                          */
	/***********************************************************************/
	/*assembly code will execute if number of samples >=12. Assembly code  */
	/*processes samples in multiples of 4 and if any samples are left they */
	/*will be processed in c code. >= 12 is because of loop unrolling in   */
	/* assembly to better pack pipe line. >=(12+4) condition is to account */
	/* last 4 samples that will be processed in c code                     */
	/***********************************************************************/
	if (samples>=(12+4))
	{
	    // last 4 samples will be processed in c to avoid out of boudary 
		// memory access on source pointers
		samples = samples - 4;
		
		//assembly
		buffer_fill_mix_asm(src1, src2, dst, gainL16Q15, samples);

		//update address pointers as they will not be updated in function call
		src1 = src1 + (samples&(~3));
		src2 = src2 + (samples&(~3));
		dst = dst + (samples&(~3));
		
		// samples to be processed in c
		samples = 4 + (samples&3);
	}

	//process remaining samples
	if (samples)
	{	
		if (gainL16Q15 == Q15_ONE)
		{
			for (i=samples; i; i--)
			{
				/*-- output = src1 + src2 --*/
				*dst = s16_add_s16_s16_sat(*src1++, *src2++);
				dst++;
			}
		} 
		/*------------ case 2 : if gain is minus unity, directly subtract -------*/
		else if (gainL16Q15 == Q15_MINUSONE)
		{
			for (i=samples; i; i--)
			{
				/*-- output = src1 - src2 --*/
				*dst = s16_sub_s16_s16_sat(*src1++, *src2++);
				dst++;
			}
		}
		/*------------ case 3: normal case, with L16Q15 gain --------------------*/
		else
		{
			for (i=samples; i; i--)
			{
				/*-- input * gain --*/
				tmpL16 = s16_extract_s40_h(Q6_P_mpy_RlRl_s1(*src2++, gainL16Q15));
				/*-- output = output + input * gain --*/
				*dst = s16_add_s16_s16_sat(*src1++, tmpL16);
				dst++;
			}
		}
	}	
#endif
}  /*---------------------- end of function buff_fill_mix--------------------*/

// smoothly crossfade between src1 and src2 with Q15 smooth panner, unity panner = all src2
void buffer_crossmix_panner(int16 *dest, int16 *src1, int16 *src2, pannerStruct *panner, int32 samples)
{
   int16 target_q15  = panner->targetGainL16Q15;
   int32 pan_samples = panner->sampleCounter;
   int32 delta_q31   = panner->deltaL32Q31;
   int16 gain_q15, tmp16;
   int32 gain_q31, ramp_samples, i;

   // current implementation totally ignores delay of panner
   // src1 gets applied gain, and src2 is applied 1-gain
   gain_q15 = panner_get_current(*panner);
   gain_q31 = s32_shl_s32_sat((int32)gain_q15, 16);

   ramp_samples = s32_min_s32_s32(pan_samples, samples);

   // process samples with dynamic gains
   for (i = 0; i < ramp_samples; ++i) {
      // get q15 gain
      gain_q15 = s16_extract_s32_h(gain_q31);
      // apply gain to src 2 and save
      *dest = s16_extract_s64_h_rnd(s40_mult_s16_s16_shift(*src2++, gain_q15, 1));
      // get 1-gain and apply to src 1 and mix
      gain_q15 = s16_sub_s16_s16_sat(Q15_ONE, gain_q15);
      tmp16 = s16_extract_s64_h_rnd(s40_mult_s16_s16_shift(*src1++, gain_q15, 1));
      *dest = s16_add_s16_s16_sat(*dest, tmp16);
      dest++;
      // add delta to q31 gain
      gain_q31 = s32_add_s32_s32_sat(gain_q31, delta_q31);
   }
   pan_samples -= ramp_samples;

   // process samples with no gain change
   samples -= ramp_samples;
   if (samples > 0) {
      if (Q15_ONE == target_q15) {
         buffer_copy(dest, src2, samples);
      }
      else if (0 == target_q15) {
         buffer_copy(dest, src1, samples);
      }
      else {
         gain_q15 = s16_sub_s16_s16_sat(Q15_ONE, target_q15);
         buffer_fill(dest, src2, target_q15, samples);
         buffer_mix(dest, src1, gain_q15, samples);
      }
   }
   panner->sampleCounter = pan_samples;
}


