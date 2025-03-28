/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*===========================================================================*]
[* FILE NAME: util.c                                                         *]
[* DESCRIPTION:                                                              *]
[*    Various utility functions for post processor.                          *]
[* FUNCTION LIST :                                                           *]
[*    changed: compare two integer values and report if different            *]
[*    find_freq: select closest frequency to match the specified             *]
[*    find_exact_freq: match exact frequency, return -1 if no match          *]
[*    ms_to_sample: convert millisecond to sample                            *]
[*    time_to_sample: convert 100 nanosecond to sample                       *]
[*===========================================================================*/
#include "audpp_util.h"
#include "audio_divide_qx.h"

/*===========================================================================*/
/* FUNCTION : find_freq                                                      */
/*                                                                           */
/* DESCRIPTION: According to a specified frequency, find the best match      */
/*              from a table of supported frequencies.                       */
/*                                                                           */
/* INPUTS: freqHz: input reference frequency                                 */
/*         designFreqArray-> array of available frequencies                  */
/*         arraySize: size of the above array                                */
/* OUTPUTS: function returns the array index of the closest match            */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
int16 find_freq
(
    int32           freqHz,                 /* input reference frequency     */
    int32  const   *designFreqArray,        /* available design freq array   */
    int16           arraySize               /* size of the design freq array */
)
{
    int32 minDiff = 0, diff;
    int16 i, bestMatchedIndex = 0;		//kwcheck : keeping -1 might lead to -ve index value, so changing it to 0
    /*-----------------------------------------------------------------------*/

    for (i = 0; i < arraySize; i++) 
    {
        /* compare the sampling rate with each of the avaiable ones in array */
        diff = u32_abs_s32_sat(s32_sub_s32_s32(designFreqArray[i], freqHz));
        if (i == 0 || minDiff > diff) 
        {   /* log and keep the best match */
            bestMatchedIndex = i;
            minDiff = diff;
        } /* end of if (i == 0 || minDiff > diff) */
    } /* end of for (i = 0; i < arraySize; i++) */
    return bestMatchedIndex;
} /*------------------------- end of function find_freq ---------------------*/


/*===========================================================================*/
/* FUNCTION : find_exact_freq                                                */
/*                                                                           */
/* DESCRIPTION: According to a specified frequency, find the exact match     */
/*              from a table of supported frequencies. Returns -1 if         */
/*              there is no match.                                           */
/*                                                                           */
/* INPUTS: freqHz: input reference frequency                                 */
/*         designFreqArray-> array of available frequencies                  */
/*         arraySize: size of the above array                                */
/* OUTPUTS: function returns the matched index or -1 if no match             */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
int16 find_exact_freq
(
    int32           freqHz,                 /* input reference frequency     */
    int32  const   *designFreqArray,        /* available design freq array   */
    int16           arraySize               /* size of the design freq array */
)
{
    int16 i;

    for (i = 0; i < arraySize; i++) 
    {
        if (designFreqArray[i] == freqHz) 
        {
            return i;                   // there exists exact match
        }
    } /* end of for (i = 0; i < arraySize; i++) */
    return -1;                          // there is no match
} /*------------------ end of function find_exact_freq ----------------------*/


/*===========================================================================*/
/* FUNCTION : ms_to_sample                                                   */
/*                                                                           */
/* DESCRIPTION: Convert millisecond value to sample value.                   */
/*                                                                           */
/* INPUTS: ms: value in millisecond                                          */
/*         sampleRate: sampling rate                                         */
/* OUTPUTS: function returns the corresponding sample value                  */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*      1. the version bit-exact with QSound uses one division               */
/*      2. second version avoids division                                    */
/*===========================================================================*/
int32 ms_to_sample
(
    int16           ms,                     /* value in millisecond          */
    uint32          sampleRate              /* samping rate                  */
)
{
    /* the following code tries to avoid division */
    int32 tmpL32;
    
    /*-- ms * sampleRate --*/
    //tmpL32 = s32_mult_s16_u16(ms, sampleRate);
	tmpL32 = (int32)s64_mult_s32_s16(sampleRate,ms); // uint16 -> int32 frequency change
    /*-- rounding --*/
    // tmpL32 = s32_add_s32_s32(tmpL32, 500);
    /*-- ms * sampleRate / 1000 , where 1099511628 is 0.001 in Q40 --*/
    tmpL32 = s32_saturate_s40(s40_mult_s32_s32_shift(tmpL32, 1099511628, 0));
            // s40_mult_s32_s32_shift contains >>32, the result is Q8
    return s32_shl_s32_sat(tmpL32, -8);   // further >>8 to get Q0

}  /*----------------------- end of function ms_to_sample -------------------*/


/*===========================================================================*/
/* FUNCTION : time_to_sample                                                 */
/*                                                                           */
/* DESCRIPTION: Convert 100 nanoseconds (10^(-7) sec) value to sample value. */
/*                                                                           */
/* INPUTS: time_100ns: value in 100ns (10^(-7) sec)                          */
/*         sampleRate: sampling rate                                         */
/* OUTPUTS: function returns the corresponding sample value                  */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*      time is in 100 ns units, 1 second==10000000 units                    */
/*===========================================================================*/
int16 time_to_sample
(
    int16           time_100ns,             /* value in 100 ns               */
    uint32          sampleRate              /* sampling rate                 */
)
{   
    int32 samplesPerUnitL32Q23;

    if (time_100ns == 0) 
    {
        return 0;
    }
    
    /* calculate samples per unit */
    samplesPerUnitL32Q23 = divide_qx(sampleRate, 10000000, 23);      // Q23

    /* return sample values, in int16 */
    return s16_saturate_s32(Q23_mult(time_100ns, samplesPerUnitL32Q23));
}  /*--------------------- end of function time_to_sample -------------------*/

