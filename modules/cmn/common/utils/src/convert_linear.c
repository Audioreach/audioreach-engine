/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*===========================================================================*]
[* FILE NAME: convert_linear.c                                               *]
[* DESCRIPTION:                                                              *]
[*    Core functions for linear rate conversion.                             *]
[* FUNCTION LIST :                                                           *]
[*    convert_linear: Linear rate conversion.                                *]
[*    convert_rate_stereo: Linear rate conversion for a stereo signal        *]
[*===========================================================================*/
#include "audpp_converter.h"
#include "audio_basic_op_ext.h"


#if ((defined __hexagon__) || (defined __qdsp6__))

#else
/*===========================================================================*/
/* FUNCTION : convert_linear_interp_c                                          */
/*                                                                           */
/* DESCRIPTION: Linear interpolation of input samples from the circular      */
/*              input buffer to linear output buffer                         */
/*                                                                           */
/* INPUTS: *srcBuf -> source buffer                                          */
/*         **destPtr1 -> dest buffer ptr                                     */
/*         *indexUL32Q161 -> Index                                           */
/*         rateUL32Q16: Rate                                                 */
/*         ui32size: Size                                                    */
/*         n: Loop count                                                     */
/*         n1: Loop count                                                    */
/*                                                                           */
/* OUTPUTS: destBuf-> output buffer                                          */
/*          function returns how many samples it is actually outputing       */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*     This function involves mult of (sample number Q0 * playback rate Q16).*/
/* To prevent the result from overflowing an int32 register, we need:        */
/*       (sample number L16Q0) * (playback rate L32Q16) <= 0x7FFF FFFF.      */
/* This condition is not hard to satisfy with reasonable playback rate and a */
/* reasonable size of sample number. So, we directly multiply them without   */
/* checking saturation.                                                      */
/*    convert_circular_loop is  optimized in c level by unrolling the loop   */
/*    count twice to make use of efficient vectorization from the compiler   */
/*===========================================================================*/
int convert_linear_interp_loop_c
(
    int16               *srcBuf,                /* source buffer             */
    int16               **destPtr1,             /* dest buffer               */
    int32               *indexUL32Q161,         /* Index                     */
    int32               rateUL32Q16,            /* Rate                      */
    int32               ui32size,               /* Size                      */
    int32               n,                      /* Loop count                */
    int32               n1                      /* Loop count                */
 )
{

    int32    indexUL32Q16=*indexUL32Q161;
    int16*   destPtr = *destPtr1;
    int16    i, i_int, sample0, sample1 ;
    uint16   i_frac;
    int32    tmpL32, i32temp;

    i32temp=(int16) (UMAX_16 & (indexUL32Q16>> 16));
    i=0;

    /* Condition inside the loop is removed with a while loop */
    /* The code has been optimized by unrolling the loop twice
    for making the conditional check once for 2 samples*/

    while((i < n)&&((n1)>i32temp))
    {
        /* get integer part of the index */
        i_int=(int16) (UMAX_16 & (indexUL32Q16 >> 16));
        /* get fractional part of the index */
        i_frac=  (int16) (UMAX_16 & indexUL32Q16);

        /*Circular buffering*/
        if((i_int>=ui32size))
        {
            i_int=i_int-CIRC_MAX_SIZE;
        }
        /* get two sample values closest to the index */
        sample0 = srcBuf[i_int++];

        /*Circular buffering*/
        if((i_int>=ui32size))
        {
            i_int=i_int-CIRC_MAX_SIZE;
        }
        sample1 = srcBuf[i_int];

        /* get output sample via linear interpolation  */
        tmpL32=sample1-sample0;
        tmpL32 = (int16)(((int32)tmpL32*i_frac)>>16);
        *destPtr++=(UMAX_16 &tmpL32)+sample0;
        /* update index and playback rate */

        indexUL32Q16  =indexUL32Q16+rateUL32Q16;

        /*second time */
        i_int=(int16) (UMAX_16 & (indexUL32Q16 >> 16));
        /* get fractional part of the index */
        i_frac=  (int16) (UMAX_16 & indexUL32Q16);

        /*Circular buffering*/
        if((i_int>=ui32size))
        {
            i_int=i_int-CIRC_MAX_SIZE;
        }
        /* get two sample values closest to the index */
        sample0 = srcBuf[i_int++];

        /*Circular buffering*/
        if((i_int>=ui32size))
        {
            i_int=i_int-CIRC_MAX_SIZE;
        }
        sample1 = srcBuf[i_int];

        /* get output sample via linear interpolation  */
        tmpL32=sample1-sample0;
        tmpL32 = (int16)(((int32)tmpL32*i_frac)>>16);
        *destPtr++=(UMAX_16 &tmpL32)+sample0;
        /* update index and playback rate */

        indexUL32Q16  =indexUL32Q16+rateUL32Q16;
        i32temp=(int16) (UMAX_16 & (indexUL32Q16 >> 16));
        i++;
    } /* end of for (i = 0; i < n; i++) */

    *indexUL32Q161=indexUL32Q16;
    *destPtr1=destPtr;
    return 2*(i);
}
#endif /* #if !defined(__qdsp6__) */

/*===========================================================================*/
/* FUNCTION : convert_linear                                                 */
/*                                                                           */
/* DESCRIPTION: Convert samples from certain inputs in a linear manner.      */
/*                                                                           */
/* INPUTS: destBuf-> output buffer                                           */
/*         outputs: number of desired output samples                         */
/*         srcBuf-> input buffer                                             */
/*         inputs: number of input samples                                   */
/*         state-> rate convert state struct                                 */
/* OUTPUTS: destBuf-> output buffer                                          */
/*          function returns how many samples it is actually outputing       */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*     This function involves mult of (sample number Q0 * playback rate Q16).*/
/* To prevent the result from overflowing an int32 register, we need:        */
/*       (sample number L16Q0) * (playback rate L32Q16) <= 0x7FFF FFFF.      */
/* This condition is not hard to satisfy with reasonable playback rate and a */
/* reasonable size of sample number. So, we directly multiply them without   */
/* checking saturation.                                                      */
/*      In the current implementation of chorus, accelL32Q16 == 0.           */
/*===========================================================================*/
int32 convert_linear
(
    int16               *destBuf,           /* output buffer                 */
    int32                outputs,           /* number of output samples      */
    int16               *srcBuf,            /* input buffer                  */
    int32                inputs,            /* number of input samples       */
    rateConvertState    *state              /* buffer converter state        */
)
{
    int32   indexL32Q16 = state->indexL32Q16;
    int32   rateL32Q16  = state->rateL32Q16;
    int32   accelL32Q16  = state->accelL32Q16;
    int32   lastInput = inputs - 1;
    int16*  destPtr = destBuf;

    int32   n1, n, testIndex, i;
    int16   i_int;
    int16   sample0, sample1;
    uint16  i_frac;
    int32   newRateL32Q16;
    int32   tmpL32;

    /*----------------------- if playback rate is changing ------------------*/
    if (accelL32Q16 != 0) 
    {
        if (outputs > 2 && inputs > 3) 
        {
            /* calculate the last index needed to get the outputs */
            n1 = outputs - 1;
                        
            /* newRateL32Q16 = rateL32Q16 + (n1-1) * accelL32Q16; */
            tmpL32 = s32_extract_s40_l(s64_mult_s32_s32(accelL32Q16, n1-1));
            newRateL32Q16 = s32_add_s32_s32(rateL32Q16, tmpL32);
        
            /* testIndex = s16_extract_s32_h((indexL32Q16 + (n1 * (rateL32Q16 + 
                newRateL32Q16) / 2) - newRateL32Q16));     */
            tmpL32 = s32_shr_s32_sat(s32_add_s32_s32(rateL32Q16, newRateL32Q16), 1);
            tmpL32 = s32_extract_s40_l(s64_mult_s32_s32(tmpL32, n1));
            tmpL32 = s32_add_s32_s32(indexL32Q16, tmpL32);
            tmpL32 = s32_sub_s32_s32(tmpL32, newRateL32Q16);
            testIndex = s16_extract_s32_h(tmpL32);
            
            if (testIndex < lastInput) 
            {
                /* Have enough inputs to produce output samples, so can      */
                /* omit the check on the input buffer                        */
                for (i = 0; i < n1; i++) 
                {
                    /* get integer part of index */
                    i_int  = s16_extract_s32_h(indexL32Q16);
                    
                    /* get fractional part of index */
                    i_frac = s16_extract_s32_l(indexL32Q16);
                    
                    /* get value of two samples closest to the index */
                    sample0 = srcBuf[i_int];
                    sample1 = srcBuf[i_int + 1];
                    
                    /* get output sample via linear interpolation  */
                    tmpL32 = s32_sub_s32_s32(sample1, sample0);   
                                                 // sampel value difference
                    tmpL32 = (int32)s40_mult_s32_u16_shift(tmpL32, i_frac, 0); 
                                                 // diff * frac
                    *destPtr++ = s16_add_s16_s16(s16_extract_s32_l(tmpL32), sample0);         
                                                 // out = sample0 + diff*frac
                    
                    /* update index and playback rate */
                    indexL32Q16 = s32_add_s32_s32(indexL32Q16, rateL32Q16);
                    rateL32Q16  = s32_add_s32_s32(rateL32Q16, accelL32Q16);
                } /* end of for (i = 0; i < n1; i++) */
            } /* end of if (testIndex < lastInput) */
        } /* end of if (outputs > 2 && inputs > 3) */
        
        /* process the rest of the outputs, if the inputs are sufficient */
        n = outputs - (int32)(destPtr - destBuf); // outputs remained
        for (i = 0; i < n; i++) 
        {
            i_int = s16_extract_s32_h(indexL32Q16); // index as int
            
            /* check on input, if used up, then break */
            if (i_int >= lastInput) 
            {
                break;
            }

            /* get fractional part of the index */
            i_frac = s16_extract_s32_l(indexL32Q16);
            
            /* get two sample values closest to the index */
            sample0 = srcBuf[i_int];
            sample1 = srcBuf[i_int + 1];
            
            /* get output sample via linear interpolation  */
            tmpL32 = s32_sub_s32_s32(sample1, sample0);   // sampel value difference
            tmpL32 = (int32)s40_mult_s32_u16_shift(tmpL32, i_frac, 0); 
                                                // diff * frac
            *destPtr++ = s16_add_s16_s16(s16_extract_s32_l(tmpL32), sample0);         
                                                // out = sample0 + diff*frac
            /* update index and playback rate */
            indexL32Q16 = s32_add_s32_s32(indexL32Q16, rateL32Q16);
            rateL32Q16  = s32_add_s32_s32(rateL32Q16, accelL32Q16);
        } /* end of for (i = 0; i < n; i++) */
    } /* end of if (accelL32Q16 != 0) */
        
    /*----------------------- if playback rate is constant ------------------*/
    else /* if (accelL32Q16 == 0) */
    {
        if (outputs > 1 && inputs > 3) 
        {
            /* calculate the last index needed to get the outputs */
            n1 = outputs - 1;
            
            /* testIndex = s16_extract_s32_h(indexL32Q16 + n1 * rateL32Q16); */
            tmpL32 = s32_extract_s40_l(s64_mult_s32_s32(rateL32Q16, n1));
            testIndex = s16_extract_s32_h(s32_add_s32_s32(indexL32Q16, tmpL32));

            if (testIndex < lastInput) 
            {   
                /* Have enough inputs to produce output samples, so can      */
                /* omit the check on the input buffer                        */
                for (i = 0; i < n1; i++) 
                {
                    /* get integer part of index */
                    i_int  = s16_extract_s32_h(indexL32Q16);
                    
                    /* get fractional part of index */
                    i_frac = s16_extract_s32_l(indexL32Q16);
                    
                    /* get value of two samples closest to the index */
                    sample0 = srcBuf[i_int];
                    sample1 = srcBuf[i_int + 1];
                    
                    /* get output sample via linear interpolation  */
                    tmpL32 = s32_sub_s32_s32(sample1, sample0);   
                                                 // sampel value difference
                    tmpL32 = (int32)s40_mult_s32_u16_shift(tmpL32, i_frac, 0); 
                                                 // diff * frac
                    *destPtr++ = s16_add_s16_s16(s16_extract_s32_l(tmpL32), sample0);         
                                                 // out = sample0 + diff*frac
                    /* update index */
                    indexL32Q16 = s32_add_s32_s32(indexL32Q16, rateL32Q16);
                } /* end of for (i = 0; i < n1; i++) */
            } /* end of if (testIndex < lastInput) */
        } /* end of if (outputs > 1 && inputs > 3) */

        /* process the rest of the outputs, if the inputs are sufficient */
        n = outputs - (int32)(destPtr - destBuf);   // outputs remained
        for (i = 0; i < n; i++) 
        {
            i_int = s16_extract_s32_h(indexL32Q16); // int part of index
            
            /* check on input, if used up, then break */
            if (i_int >= lastInput) 
            {
                break;
            }

            /* get fractional part of the index */
            i_frac = s16_extract_s32_l(indexL32Q16);
            
            /* get two sample values closest to the index */
            sample0 = srcBuf[i_int];
            sample1 = srcBuf[i_int + 1];
            
            /* get output sample via linear interpolation  */
            tmpL32 = s32_sub_s32_s32(sample1, sample0);   // sample value difference
            tmpL32 = (int32)s40_mult_s32_u16_shift(tmpL32, i_frac, 0); 
                                                // diff * frac
            *destPtr++ = s16_add_s16_s16(s16_extract_s32_l(tmpL32), sample0);         
                                                // out = sample0 + diff*frac
            /* update index */
            indexL32Q16 = s32_add_s32_s32(indexL32Q16, rateL32Q16);
        } /* end of for (i = 0; i < n; i++) */
    } /* end of else if (accelL32Q16 == 0) */

    /*-------------------- update index and rate of the state ---------------*/
    state->indexL32Q16 = indexL32Q16;
    state->rateL32Q16 = rateL32Q16;

    /*------------------ return the number of processed outputs -------------*/
    return (int32)(destPtr - destBuf);
} /*------------------- end of function convert_linear ----------------------*/


/*===========================================================================*/
/* FUNCTION : convert_rate_stereo                                            */
/*                                                                           */
/* DESCRIPTION: Convert stereo samples from mono input with linear interp.   */
/*                                                                           */
/* INPUTS: destBufL-> output buffer L                                        */
/*         destBufR-> output buffer R                                        */
/*         outputs: number of desired output samples                         */
/*         srcBuf-> input buffer                                             */
/*         inputs: number of input samples                                   */
/*         state-> rate convert state struct                                 */
/* OUTPUTS: destBufL-> output buffer L                                       */
/*          destBufR-> output buffer R                                       */
/*          function returns how many samples it is actually outputing       */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*     This function involves mult of (sample number Q0 * playback rate Q16).*/
/* To prevent the result from overflowing an int32 register, we need:        */
/*       (sample number L16Q0) * (playback rate L32Q16) <= 0x7FFF FFFF.      */
/* This condition is not hard to satisfy with reasonable playback rate and a */
/* reasonable size of sample number. So, we directly multiply them without   */
/* checking saturation.                                                      */
/*===========================================================================*/
int32 convert_rate_stereo
(
    int16               *destBufL,          /* output buffer L               */
    int16               *destBufR,          /* output buffer R               */
    int32                outputs,           /* number of output samples      */
    int16               *srcBuf,            /* input buffer                  */
    int32                inputs,            /* number of input samples       */
    rateConvertState    *state              /* buffer converter state        */
)
{
    int32  indexL32Q16 = state->indexL32Q16;
    int32  rateL32Q16  = state->rateL32Q16;
    int32   accelL32Q16  = state->accelL32Q16;
    
    int32   lastInput = inputs - 1;
    int32   written;
    int16   i_int, sample0, sample1;
    uint16  i_frac;
    int32   tmpL32;

    for (written = 0; written < outputs; written++) 
    {
        i_int = s16_extract_s32_h(indexL32Q16); // int part of index
        
        /* check on input, if used up, then break */
        if (i_int >= lastInput) 
        {
            break;
        }

        i_int = s16_shl_s16_sat(i_int, 1);      // i_int * 2
    
        /* get fractional part of the index */
        i_frac = s16_extract_s32_l(indexL32Q16);
        
        /* get two sample values closest to the index */
        sample0 = srcBuf[i_int];
        sample1 = srcBuf[i_int + 2];
        
        /* get output sample via linear interpolation  */
        tmpL32 = s32_sub_s32_s32(sample1, sample0);            // sample value difference
        tmpL32 = (int32)s40_mult_s32_u16_shift(tmpL32, i_frac, 0);   // diff*frac
        *destBufL++ = s16_add_s16_s16(s16_extract_s32_l(tmpL32), sample0); // out=sample0+diff*frac

        /* get two sample values closest to the index */
        sample0 = srcBuf[i_int + 1];
        sample1 = srcBuf[i_int + 3];
        
        /* get output sample via linear interpolation  */
        tmpL32 = s32_sub_s32_s32(sample1, sample0);            // sample value difference
        tmpL32 = (int32)s40_mult_s32_u16_shift(tmpL32, i_frac, 0);   // diff*frac
        *destBufR++ = s16_add_s16_s16(s16_extract_s32_l(tmpL32), sample0); // out=sample0+diff*frac

        /* update index and rate */
        indexL32Q16 = s32_add_s32_s32(indexL32Q16, rateL32Q16);
        rateL32Q16 = s32_add_s32_s32(rateL32Q16, accelL32Q16);
    } /* end of for (i = 0; i < n; i++) */

    /*-------------------- update index and rate of the state ---------------*/
    state->indexL32Q16 = indexL32Q16;
    state->rateL32Q16 = rateL32Q16;

    /*------------------ return the number of processed outputs -------------*/
    return written;
} /*----------------- end of function convert_rate_stereo -------------------*/
