/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*===========================================================================*]
[* FILE NAME: audio_panner.c                                                 *]
[* DESCRIPTION:                                                              *]
[*    Functions regarding panner: linear smooth value change                 *]
[* FUNCTION LIST :                                                           *]
[*    panner_get_current: Get current gain of a panner                       *]
[*    panner_setup: setup panner parameters                                  *]
[*    get_cur_cir_scale: Get current angle of an angle panner         *]
[*    cir_scale_setup: setup angle panner parameters                      *]
[*    buffer_scale_mix: Apply panner to a buffer and mix into another  *]
[*===========================================================================*/
#include "audio_dsp.h"
#include "audio_divide_qx.h"

#if ((defined __hexagon__) || (defined __qdsp6__))
#ifdef __cplusplus
extern "C"
#else
extern
#endif /* __cplusplus */
void buffer_scale_mix_asm(int16 *srcPtr, int16 *destPtr, int32 currentGainL32Q31, 
                                int32 deltaL32Q31, int16 rampSamples);
#endif
/*================== diagram for a linear panning process ===================*/
/* typedef struct {                                                          */
/*   int16       targetGainL16Q15;       - target gain to pan toward, L16Q15 */
/*   int32       deltaL32Q31;            - gain increment per sample, L32Q31 */
/*   int32       sampleCounter;          - total sample steps of the panner  */
/*   int32       delaySamples;           - delay of samples to start panning */
/* } pannerStruct;                                                           */
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                      current/----------------  <- targetGainL16Q15        */
/*                       time / .                                            */
/*      call       start  .  /  .                                            */
/*      setup     panning . /   .                                            */
/*       .            .   v/    .                                            */
/*       .            .   / . . . . . . . . . .   <- current gain            */
/*       .            .  /.     .                                            */
/*       v            v / .     .                                            */
/*       --------------/. . . . . . . . . . . .   <- initial gain            */
/*        delaySamples ramp samples                                          */
/*       |<---------->|<------->|                                            */
/*                        |<--->|                                            */
/*                      sampleCounter (set to ramp sample during setup)      */
/*       ------------------------------------------> time                    */
/*  (as the panner works, sampleCounter will decrease until it becomes zero, */
/*   and at that moment, gain of the panner should reach targetGainL16Q15)   */
/*  (angle panner has similar structure, can use same graph as reference)    */
/*===========================================================================*/


/*===========================================================================*/
/* FUNCTION : panner_get_current                                             */
/*                                                                           */
/* DESCRIPTION: Return current gain of a panner                              */
/*                                                                           */
/* INPUTS: panner: panner struct                                             */
/* OUTPUTS: return current gain in L16Q15                                    */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
int16 panner_get_current 
(
    pannerStruct panner                 /* panner struct                     */
)
{
    int16 tmpL16Q15;

    if (panner.sampleCounter == 0)
    {   /* if counter zeroed, target gain already achieved */
        return panner.targetGainL16Q15;
    }
    else
    {
        /* calculate diff of current gain from target gain, Q15 */
        tmpL16Q15 = s16_saturate_s32(
                        Q16_mult(panner.deltaL32Q31, panner.sampleCounter)); 
        /* get current gain */
        return s16_sub_s16_s16_sat(panner.targetGainL16Q15, tmpL16Q15);
    }
} /*----------------- end of function panner_get_current --------------------*/


/*===========================================================================*/
/* FUNCTION : panner_setup                                                   */
/*                                                                           */
/* DESCRIPTION: According to new target gain and ramp sample number, setup   */
/*              a panner's target gain, delta and sample counter.            */
/*                                                                           */
/* INPUTS: panner-> panner struct                                            */
/*         newGainL16Q15: new target panner gain                             */
/*         rampSamples: number of samples in the ramp                        */
/*         newDelay: delay after setup to execute the panner, in sample      */
/* OUTPUTS: panner-> panner struct, with values set                          */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*      By inputting zero ramSamples and zero newDelay, this function can    */
/*      set a static gain to the panner immediately.                         */
/*===========================================================================*/
void panner_setup
(   
    pannerStruct    *panner,            /* panner struct                     */
    int16            newGainL16Q15,     /* new target panner gain            */
    int32            rampSamples,       /* number of samples in the ramp     */
    int32            newDelay           /* delay of ramping process          */
)
{
    int16 changeL16Q15, currentGainL16Q15;

    /* substitute panner struct values with shorter names */ 
    int32 sampleCounter    = panner->sampleCounter;
    int32 deltaL32Q31      = panner->deltaL32Q31;
    int32 delaySamples     = panner->delaySamples;
    
    /*----------------------------- if no ramp ------------------------------*/
    if (rampSamples <= 0)
    {
        /* set counter, delta, and delay to zero */
        sampleCounter = 0;
        deltaL32Q31 = 0;
        delaySamples = 0;   // if no ramp, set gain immediately without delay 
    } /* end of if (rampSamples <= 0) */
    
    /*--------------- if there is ramp, then need some work here ------------*/
    else /* (rampSamples > 0) */
    {
        /*------ set new delay samples ------*/
        delaySamples = newDelay;

        /*------ get current gain ------*/        
        currentGainL16Q15 = panner_get_current(*panner);

        /*------ determine change -------*/
        changeL16Q15 = s16_sub_s16_s16_sat(newGainL16Q15, currentGainL16Q15);
        /* if no change, then reset counter and delta */
        if (changeL16Q15 == 0)
        {
            sampleCounter = 0;
            deltaL32Q31 = 0;
        }
        /* else, set counter and calculate delta */
        else /* (changeL16Q15 != 0) */
        {
            sampleCounter = rampSamples;
            /* Q15/Q0, s16_add_s16_s16 Q16: delta will be in Q31 */
            deltaL32Q31 = divide_qx(changeL16Q15, sampleCounter, 16);
        }
    } /* end of else (rampSamples > 0) */

    /*------------- store new values back into panner struct ----------------*/
    panner->targetGainL16Q15 = newGainL16Q15;       // update target gain
    panner->sampleCounter    = sampleCounter;
    panner->deltaL32Q31      = deltaL32Q31;
    panner->delaySamples     = delaySamples;
} /*------------------- end of function panner_setup ------------------------*/

/*===========================================================================*/
/* FUNCTION : buffer_fill_with_panner                                        */
/*                                                                           */
/* DESCRIPTION: Apply L16Q15 panner to input and store it into output buffer.*/
/*                                                                           */
/* INPUTS: destBuf-> output buffer                                           */
/*         srcBuf-> input buffer                                             */
/*         panner-> panner to be applied to the input buffer                 */
/*         samples: total number of samples to be processed                  */
/* OUTPUTS: destBuf-> output buffer                                          */
/*          panner-> panner internal variables gets updated also             */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*      Assuming samples <= 32767                                            */
/*===========================================================================*/
void buffer_fill_with_panner
(
    int16           *destBuf,           /* output buffer                     */
    int16           *srcBuf,            /* input buffer                      */
    pannerStruct    *panner,            /* panner applied to the samples     */
    int32            samples            /* number of samples to process      */
)
{
    int16 *srcPtr  = srcBuf;
    int16 *destPtr = destBuf;
    int16 targetGainL16Q15 = panner->targetGainL16Q15;
    int32 panSamples       = panner->sampleCounter;
    int32 deltaL32Q31      = panner->deltaL32Q31;
    int32 pannerDelay      = panner->delaySamples;
    int16 currentGainL16Q15;
    int32 currentGainL32Q31;

    int32 i, rampSamples;
 
    /*-----------------------------------------------------------------------*/

    /*--------------------- get current panner gain value -------------------*/
    currentGainL16Q15 = panner_get_current(*panner);

    /*------ if the panner is just a static gain, switch to buffer_fill -----*/
    if (panSamples == 0 || deltaL32Q31 == 0 || pannerDelay >= samples)
    {
        buffer_fill(destBuf, srcBuf, currentGainL16Q15, samples);
        if (pannerDelay >= samples)
        {
            /* if delay of ramping is greater than the current frame */
            panner->delaySamples = s32_sub_s32_s32_sat(pannerDelay, samples);
        }
        return;
    } /* end of if panner is static gain */

    /*--------- otherwise, panner is doing ramped mix somewhere  ------------*/
    /*--- 1. if there still leftover delays in panner, mix that part ---*/
    if (pannerDelay > 0)
    {
        buffer_fill(destBuf,srcBuf,currentGainL16Q15,s16_extract_s32_l(pannerDelay));
    }
    samples = s32_sub_s32_s32_sat(samples, pannerDelay);

    /*--- 2. now start processing ramped samples ---*/
    /* advance input / output pointers by what's processed so far */
    srcPtr  += pannerDelay;
    destPtr += pannerDelay;

    /* convert current gain to Q31 version */
    currentGainL32Q31 = s32_shl_s32_sat((int32)currentGainL16Q15, 16);
    
    /* if panner ramp is shorter than remaining sample number in the frame,  */
    /* we only process this part with panner, and directly mix the rest      */
    rampSamples = s16_extract_s32_l(s32_min_s32_s32(samples, panSamples));
    
    /* process the ramped samples with panner */
    for (i = 0; i < rampSamples; i ++)
    {
        /* covert back the current gain in L16Q15 */
        currentGainL16Q15 = s16_extract_s32_h(currentGainL32Q31);
#if ((defined __hexagon__) || (defined __qdsp6__))
      
        *destPtr = s16_extract_s40_h(Q6_P_mpy_RlRl_s1(*srcPtr, currentGainL16Q15));
        destPtr++;
        srcPtr++;

#else
        /* apply the Q15 gain to input, and store output */
        *destPtr = s16_extract_s64_h_rnd(s40_mult_s16_s16_shift(*srcPtr, currentGainL16Q15,1));
        destPtr++;
        srcPtr++;
#endif
        /* update the current gain, adding delta to it, Q31 */
        currentGainL32Q31 = s32_add_s32_s32_sat(currentGainL32Q31, deltaL32Q31);
    } /* end of for (i = 0; i < rampSamples; i ++) */
    samples = s32_sub_s32_s32_sat(samples, rampSamples);
    
    /*--- 3. if still have samples left, mix them with static gain ---*/
    if (samples > 0)
    {
        buffer_fill(destPtr, srcPtr, targetGainL16Q15, samples);
    }

    /*------------------------ advance the panner ---------------------------*/
    if (panSamples > 0)
    {
        panSamples = s32_sub_s32_s32_sat(panSamples, rampSamples);
        if (panSamples == 0)
            {  deltaL32Q31 = 0;   }
    }
    panner->delaySamples  = 0;
    panner->sampleCounter = panSamples;
    panner->deltaL32Q31 = deltaL32Q31;
}  /*---------------- end of function buffer_fill_with_panner ---------------*/


/*===========================================================================*/
/* FUNCTION : buffer_scale_mix                                         */
/*                                                                           */
/* DESCRIPTION: Apply L16Q15 panner to input and mix (sum) it into a running */
/*              output buffer.                                               */
/*                                                                           */
/* INPUTS: destBuf-> output buffer                                           */
/*         srcBuf-> input buffer                                             */
/*         panner-> panner to be applied to the input buffer                 */
/*         samples: total number of samples to be processed                  */
/* OUTPUTS: destBuf-> output buffer                                          */
/*          panner-> panner internal variables gets updated also             */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*      Assuming that samples <= 32767                                       */
/*===========================================================================*/
void buffer_mix_with_panner
(
    int16           *destBuf,           /* output buffer                     */
    int16           *srcBuf,            /* input buffer                      */
    pannerStruct    *panner,            /* panner applied to the samples     */
    int32            samples            /* number of samples to process      */
)
{
    int16 *srcPtr  = srcBuf;
    int16 *destPtr = destBuf;
    int16 targetGainL16Q15 = panner->targetGainL16Q15;
    int32 panSamples       = panner->sampleCounter;
    int32 deltaL32Q31      = panner->deltaL32Q31;
    int32 pannerDelay      = panner->delaySamples;
    int16 currentGainL16Q15;
    int32 currentGainL32Q31;

    int32 i, rampSamples;
	
#ifndef __qdsp6__
	int16 tmpL16;
#endif
 
    /*-----------------------------------------------------------------------*/

    /*--------------------- get current panner gain value -------------------*/
    currentGainL16Q15 = panner_get_current(*panner);

    /*------ if the panner is just a static gain, switch to buffer_mix ------*/
    if (panSamples == 0 || deltaL32Q31 == 0 || pannerDelay >= samples)
    {
        buffer_mix(destBuf, srcBuf, currentGainL16Q15, samples);
        if (pannerDelay >= samples)
        {
            /* if delay of ramping is greater than the current frame */
            panner->delaySamples = s32_sub_s32_s32_sat(pannerDelay, samples);
        }
        return;
    } /* end of if (sampleCounter == 0) */

    /*---------------- panner is doing ramped mix somewhere  ----------------*/
    /*--- 1. if there still leftover delays in panner, mix that part ---*/
    if (pannerDelay > 0)
    {
        buffer_mix(destBuf, srcBuf, currentGainL16Q15, s16_extract_s32_l(pannerDelay));
    }
    samples = s32_sub_s32_s32_sat(samples, pannerDelay);

    /*--- 2. now start processing ramped samples ---*/
    /* advance input / output pointers by what's processed so far */
    srcPtr  += pannerDelay;
    destPtr += pannerDelay;

    /* convert current gain to Q31 version */
    currentGainL32Q31 = s32_shl_s32_sat((int32)currentGainL16Q15, 16);
    
    /* if panner ramp is shorter than remaining sample number in the frame,  */
    /* we only process this part with panner, and directly mix the rest      */
    rampSamples = s16_extract_s32_l(s32_min_s32_s32(samples, panSamples));
    
    /* process the ramped samples with panner */
    for (i = 0; i < rampSamples; i ++)
    {
        /* covert back the current gain in L16Q15 */
        currentGainL16Q15 = s16_extract_s32_h(currentGainL32Q31);
#if ((defined __hexagon__) || (defined __qdsp6__))

         *destPtr = s16_add_s16_s16_sat(*destPtr, s16_extract_s40_h(Q6_P_mpy_RlRl_s1(*srcPtr, currentGainL16Q15)));
         srcPtr++;
         destPtr++;

#else
		/* apply the Q15 gain to input buffer sample */
        tmpL16 = s16_extract_s40_h(s40_mult_s16_s16_shift(*srcPtr, currentGainL16Q15, 1));
        srcPtr++;
        /* mix the altered sample into running output buffer */
        *destPtr = s16_add_s16_s16_sat(*destPtr, tmpL16);
        destPtr++;

#endif
        /* update the current gain, adding delta to it, Q31 */
        currentGainL32Q31 = s32_add_s32_s32_sat(currentGainL32Q31, deltaL32Q31);
    } /* end of for (i = 0; i < rampSamples; i ++) */
    samples = s32_sub_s32_s32_sat(samples, rampSamples);
    
    /*--- 3. if still have samples left, mix them with static gain ---*/
    if (samples > 0)
    {
        buffer_mix(destPtr, srcPtr, targetGainL16Q15, samples);
    }

    /*------------------------ advance the panner ---------------------------*/
    if (panSamples > 0)
    {
        panSamples = s32_sub_s32_s32_sat(panSamples, rampSamples);
        if (panSamples == 0)
            {  deltaL32Q31 = 0;   }
    }
    panner->delaySamples = 0;
    panner->sampleCounter = panSamples;
    panner->deltaL32Q31 = deltaL32Q31;
}  /*---------------- end of function buffer_mix_with_panner ----------------*/

/*===========================================================================*/
/* FUNCTION : get_cur_cir_scale                                       */
/*                                                                           */
/* DESCRIPTION: Return current angle of an angle panner                      */
/*                                                                           */
/* INPUTS: panner: angle panner struct                                       */
/* OUTPUTS: return current angle in L32Q16                                   */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*      Angle values are unit 0~32 Q16, corresponding to 0~180 degrees.      */
/*===========================================================================*/
int32 get_cur_cir_scale 
(
    anglePannerStruct panner            /* angle panner struct               */
)
{
    int32       productL32, deltaL32Q24;
    int16       aHighL16, bHighL16;
    uint16      aLowUL16, bLowUL16;
    boolean     negative;

    if (panner.sampleCounter == 0 || panner.deltaL32Q24 == 0)
    {   /* if counter zeroed, target angle already achieved */
        return panner.targetAngleL32Q16;
    }
    else
    {
        /* calculate diff of current angle from target angle, Q16 */
#if 1
		deltaL32Q24 = panner.deltaL32Q24;
		negative=FALSE;
#else
        if (panner.deltaL32Q24 < 0)
        {
            deltaL32Q24 = -panner.deltaL32Q24;
            negative = TRUE;
        }
        else
        {
            deltaL32Q24 = panner.deltaL32Q24;
            negative = FALSE;
        }
#endif
        /* multiply delta and sample counter, result down-shift 8 */
        aLowUL16 = s16_extract_s32_l(deltaL32Q24);
        aHighL16 = s16_extract_s32_h(deltaL32Q24);
        bLowUL16 = s16_extract_s32_l(panner.sampleCounter);
        bHighL16 = s16_extract_s32_h(panner.sampleCounter);

        productL32 = s32_saturate_s40(s40_shl_s40(s40_add_s40_s40(u32_mult_u16_u16(aLowUL16, bLowUL16), 0), -8));
        productL32 = s32_add_s32_s32(productL32, s32_shl_s32_sat(s32_mult_s16_u16(aHighL16, bLowUL16), 8));
        productL32 = s32_add_s32_s32(productL32, s32_shl_s32_sat(s32_mult_s16_u16(bHighL16, aLowUL16), 8));
        productL32 = s32_add_s32_s32(productL32, s32_shl_s32_sat(s32_mult_s16_s16(aHighL16, bHighL16), 24));

        productL32 = negative ? -productL32 : productL32;

        /* get current gain */
        return s32_sub_s32_s32_sat(panner.targetAngleL32Q16, productL32);
    }
} /*------------- end of function get_cur_cir_scale ------------------*/


/*===========================================================================*/
/* FUNCTION : cir_scale_setup                                             */
/*                                                                           */
/* DESCRIPTION: According to new target angle and ramp sample number, setup  */
/*              an angle panner's target angle, delta and sample counter.    */
/*                                                                           */
/* INPUTS: panner-> angle panner struct                                      */
/*         newAngleL32Q16: new target angle, L32Q16                          */
/*         rampSamples: number of samples in the ramp                        */
/* OUTPUTS: panner-> angle panner struct, with values set                    */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
void cir_scale_setup
(   
    anglePannerStruct *panner,          /* angle panner struct               */
    int32            newAngleL32Q16,    /* new target angle                  */
    int32            rampSamples        /* number of samples in the ramp     */
)
{
    int32 changeL32Q16, currentAngleL32Q16;

    /* substitute angle panner struct values with shorter names */ 
    int32 sampleCounter    = panner->sampleCounter;
    int32 deltaL32Q24      = panner->deltaL32Q24;
    
    /*----------------------------- if no ramp ------------------------------*/
    if (rampSamples <= 0)
    {
        /* set counter and delta to zero */
        sampleCounter = 0;
        deltaL32Q24 = 0;
    } /* end of if (rampSamples <= 0) */
    
    /*--------------- if there is ramp, then need some work here ------------*/
    else /* (rampSamples > 0) */
    {
        /*------ get current gain ------*/        
        currentAngleL32Q16 = get_cur_cir_scale(*panner);

        /*------ determine change -------*/
        changeL32Q16 = s32_sub_s32_s32_sat(newAngleL32Q16, currentAngleL32Q16);
        /* if no change, then reset counter and delta */
        if (changeL32Q16 == 0)
        {
            sampleCounter = 0;
            deltaL32Q24 = 0;
        }
        /* else, set counter and calculate delta */
        else /* (changeL16Q15 != 0) */
        {
            sampleCounter = rampSamples;
            /* Q16/Q0, add Q8: delta will be in Q24 */
			
            //deltaL32Q24 = divide_qx(changeL32Q16, sampleCounter, 8);
    

		    deltaL32Q24 = s32_saturate_s40(audio_divide_dp(changeL32Q16, sampleCounter, -21));


	//		temp = divide_qx(changeL32Q16, sampleCounter, 8);
	//		if (temp != deltaL32Q24)
	//			deltaL32Q24 = temp;
        }
    } /* end of else (rampSamples > 0) */

    /*--------- store new values back into angle panner struct --------------*/
    panner->targetAngleL32Q16 = newAngleL32Q16;       // update target angle
    panner->sampleCounter     = sampleCounter;
    panner->deltaL32Q24       = deltaL32Q24;
} /*---------------- end of function cir_scale_setup ---------------------*/

/*===========================================================================*/
/* FUNCTION : dscale_setup                                        */
/*                                                                           */
/* DESCRIPTION: According to new target gain and ramp sample number, setup   */
/*              a panner's target gain, delta and sample counter.            */
/*                                                                           */
/* INPUTS: panner-> panner struct                                            */
/*         newGainL16Q15: new target panner gain                             */
/*         rampSamples: number of samples in the ramp                        */
/*         newDelay: delay after setup to execute the panner, in sample      */
/* OUTPUTS: panner-> panner struct, with values set                          */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*      By inputting zero ramSamples and zero newDelay, this function can    */
/*      set a static gain to the panner immediately.                         */
/*===========================================================================*/
void dscale_setup
(
    pannerStruct *panner,  /* panner struct                  */
    int16 newGainL16Q15,   /* new target panner gain         */
    int32 rampSamples,     /* number of samples in the ramp  */
    int32 newDelay         /* delay of ramping process       */
)
{
    int16 changeL16Q15, currentGainL16Q15;

    /* substitute panner struct values with shorter names */ 
    int32 sampleCounter    = panner->sampleCounter;
    int32 deltaL32Q31      = panner->deltaL32Q31;
    int32 delaySamples     = panner->delaySamples;
    
    /*----------------------------- if no ramp ------------------------------*/
    if (rampSamples <= 0)
    {
          
	  delaySamples = newDelay;
        if (newDelay == 0)
        {
            /* set counter, delta, and delay to zero */
            sampleCounter = 0;
            deltaL32Q31 = 0;
        }
        else /* the panner has a delay, but become static after the delay */
        {
            currentGainL16Q15 = panner_get_current(*panner);
            sampleCounter = 1;
            deltaL32Q31 = s32_shl_s32_sat(s16_sub_s16_s16_sat(newGainL16Q15, currentGainL16Q15), 16);
        }
    } /* end of if (rampSamples <= 0) */

    /*--------------- if there is ramp, then need some work here ------------*/
    else /* (rampSamples > 0) */
    {
        /*------ set new delay samples ------*/
        delaySamples = newDelay;

        /*------ get current gain ------*/        
        currentGainL16Q15 = panner_get_current(*panner);

        /*------ determine change -------*/
        changeL16Q15 = s16_sub_s16_s16_sat(newGainL16Q15, currentGainL16Q15);
        /* if no change, then reset counter and delta */
        if (changeL16Q15 == 0)
        {
            sampleCounter = 0;
            deltaL32Q31 = 0;
        }
        /* else, set counter and calculate delta */
        else /* (changeL16Q15 != 0) */
        {
            sampleCounter = rampSamples;
            /* Q15/Q0, s16_add_s16_s16 Q16: delta will be in Q31 */
            deltaL32Q31 = divide_qx(changeL16Q15, sampleCounter, 16);
        }
    } /* end of else (rampSamples > 0) */

    /*------------- store new values back into panner struct ----------------*/
    panner->targetGainL16Q15 = newGainL16Q15;       // update target gain
    panner->sampleCounter    = sampleCounter;
    panner->deltaL32Q31      = deltaL32Q31;
    panner->delaySamples     = delaySamples;
} /*------------------- end of function dscale_setup -------------*/