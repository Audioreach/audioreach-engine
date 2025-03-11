/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*===========================================================================*]
[* FILE NAME: filterDesign.c                                                 *]
[* DESCRIPTION:                                                              *]
[*    Basic filter design functions, adapted from QSound code.               *]
[* FUNCTION LIST :                                                           *]
[*    Q23_cosine_norm_freq: calculate cosine value in Q23                   *]
[*    find_root: solve one equation for 1st order filter design              *]
[*    designFirstOrderLowpassCoeffs: 1st order low-pass filter design        *]
[*    designFirstOrderHighpassCoeffs: 1st order high-pass filter design      *]
[*    cascadeFirstOrderFilters: cascade two 1p1z to get one biquad           *]
[*    designBiquadLowpassCoeffs: design biquad by cascading two 1st order    *]
[*    designBiquadBandpassCoeffs: design biquad by cascading two 1st order   *]
[*    designBiquadHighpassCoeffs: design biquad by cascading two 1st order   *]
[*===========================================================================*/
#include "filter_design.h"
#include "ar_defs.h"
#include "audio_divide_qx.h"

/*===========================================================================*/
/* FUNCTION : Q23_cosine_norm_freq                                          */
/*                                                                           */
/* DESCRIPTION: Compute consine value for input frequency (Hz).              */
/*                                                                           */
/* INPUTS: freqHz: input frequency                                           */
/*         sampleRate: sampling rate                                         */
/* OUTPUTS: function returns cosine value for freqHz                         */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*      Results can be from table look-up or computation, depending on the   */
/*      inputs values.                                                       */
/*===========================================================================*/
int32 Q23_cosine_norm_freq
(
    int32       freqHz,                     /* input frequency in Hz         */
    int32       sampleRate                  /* sampling rate                 */
)
{
    int16 i, j;
    int32 tmpL32;
    
    static const int32 designFrequencies[] =       /* Hz */
    {
        75,
        80,
        100,
        120,
        300,
        3000,
        5000,
        9000
    };
    static const int32 designRates[] =             /* Hz */
    {
        8000,
        11025,
        12000,
        16000,
        22050,
        24000,
        32000,
        44100,
        48000,
        96000,
        192000
    };
    static const int32 cosinesL32Q23[11][8] =        /* L32Q23 */
        {    //75        80       100      120      300      3000      5000      9000
        {  8374059, 8372055, 8362749, 8351379, 8156830, -5931642, -5931642,  5931642 }, // 8000
        {  8380947, 8379892, 8374990, 8368999, 8266304, -1161548, -8033337,  3395568 }, // 11025
        {  8382141, 8381250, 8377112, 8372055, 8285330,        0, -7264748,        0 }, // 12000
        {  8384970, 8384469, 8382141, 8379296, 8330462,  3210181, -3210181, -7750063 }, // 16000
        {  8386693, 8386429, 8385202, 8383704, 8357976,  5505678,  1220702, -7030395 }, // 22050
        {  8386991, 8386768, 8385733, 8384469, 8362749,  5931642,  2171132, -5931642 }, // 24000
        {  8387699, 8387573, 8386991, 8386280, 8374059,  6974873,  4660461, -1636536 }, // 32000
        {  8388129, 8388063, 8387757, 8387382, 8380947,  7633927,  6348574,  2386789 }, // 44100
      {  8388129, 8388063, 8387757, 8387382, 8380947,  7633927,  6348574,  2386789 }, // 48000
      {  8388507, 8388493, 8388428, 8388349, 8386991,  8227423,  7943426,  6974873 }, // 96000
      {  8388583, 8388579, 8388563, 8388543, 8388204,  8348215,  8276564,  8027397 }, // 192000
   };

    /* find if there's match to table look-up values */
    j = find_exact_freq(freqHz, designFrequencies, 8);
    i = find_exact_freq(sampleRate, designRates, 11); 

    /* if there's match in the look-up table, use the values */
    if (i >= 0 && j >= 0) 
    {
        return cosinesL32Q23[i][j];
    }
    /* if not, compute from scratch */
    else 
    {
        /*--- norm_freq: freq / sampleRate ---*/
        tmpL32 = divide_qx(freqHz, sampleRate, 23);
        /*--- freq in radians:  2pi * norm_freq ---*/
        tmpL32 = Q23_mult(Q23_TWOPI, tmpL32);
        /*--- cosine value output ---*/
        return Q23_cosine(tmpL32);
    }
} /*----------------- end of function consine_norm_freq ---------------------*/


/*===========================================================================*/
/* FUNCTION : find_root                                                      */
/*                                                                           */
/* DESCRIPTION: Find the root to the quadratic equation                      */
/*              x**2 - 2*u*x + 1 = 0, where u = numer/denom                  */
/*                                                                           */
/* INPUTS: numerL32Q23: numerator                                            */
/*         denomL32Q23: denominator                                          */
/* OUTPUTS: function returns the root                                        */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*      Normally the roots will be u+sqrt(u**2-1) and u-sqrt(u**2-1)         */
/*      but in some circumstances u**2 can overflow. This function will      */
/*      handle this case.                                                    */
/*===========================================================================*/
static int32 find_root
(
    int32       numerL32Q23,                    
    int32       denomL32Q23                     /* u = number/denom          */
)
{
    int32 rootL32Q23, uL32Q23, vL32Q23, detL32Q23;
    
    if (u32_abs_s32_sat(numerL32Q23) >= u32_abs_s32_sat(denomL32Q23)<<4) 
    {   
        /* will cause overflow when calculating u**2 */
        
        /* v = 1/u (reciprocal, will be tiny) */
        vL32Q23 = divide_qx(denomL32Q23, numerL32Q23, 23); 
        if (vL32Q23 < Q23_ONE>>10 && vL32Q23 > -Q23_ONE>>10) 
        {
            /* use approximation where u-sqrt(u**2-1) ~= 1/(2u) = v/2 */
            rootL32Q23 = vL32Q23 >> 1;
        }
        else 
        {   /* u-sqrt(u**2-1) = [1-sqrt(1-v**2)]/v */
            detL32Q23 = s32_sub_s32_s32(Q23_ONE, Q23_square(vL32Q23));
            if(!(detL32Q23 >= 0))
            {
                return PPERR_FILTER;
            }
            rootL32Q23 = divide_qx(
                s32_sub_s32_s32(Q23_ONE, Q23_sqrt(detL32Q23)), vL32Q23, 23);
        }
    } /* end of if (u32_abs_s32_sat(numerL32Q23) >= u32_abs_s32_sat(denomL32Q23)<<4) */
    else /* case with no saturation problem */
    {
        /* u = number/denom */
        uL32Q23 = divide_qx(numerL32Q23, denomL32Q23, 23);
        /* det = u**2-1 */
        detL32Q23 = s32_sub_s32_s32(Q23_square(uL32Q23), Q23_ONE);
        if(!(detL32Q23 >= 0))
        {
            return PPERR_FILTER;
        }
        if (uL32Q23 > 0) 
        {
            /* u-sqrt(u**2-1) */
            rootL32Q23 = s32_sub_s32_s32(uL32Q23, Q23_sqrt(detL32Q23));
        }
        else 
        {
            /* u+sqrt(u**2-1) */
            rootL32Q23 = s32_add_s32_s32(uL32Q23, Q23_sqrt(detL32Q23));
        }
    } /* end of else (case with no saturation problem) */
    return rootL32Q23;
} /*----------------- end of function find_root -----------------------------*/


/*===========================================================================*/
/* FUNCTION : designFirstOrderLowpassCoeffs                                  */
/*                                                                           */
/* DESCRIPTION: Design a first order low-pass filter, coeffs in L32Q23.      */
/*                                                                           */
/* INPUTS: coeffsL32Q23-> coeffs in L32Q23                                   */
/*         mB: attenuation (millibel) at cutoff frequency                    */
/*         freqHz: cutoff frequency                                          */
/*         sampleRate: sampling rate                                         */
/*         withZero: with a zero or not at samplerate/2                      */
/* OUTPUTS: coeffsL32Q23-> coeffs in L32Q23                                  */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*---------------------------------------------------------------------------*/
/* The first order recurrence equation is                                    */
/* y(n) = c0*x(n)+c1*x(n-1)+c2*y(n-1), and the transfer function is          */
/*                                                                           */
/*    c0+c1*z**-1                                                            */
/*    -----------                                                            */
/*     1-c2*z**-1                                                            */
/*                                                                           */
/* A low-pass filter requires that gain is 1 at 0 Hz (z==1).  For a fast     */
/* implementation (withZero==FALSE) we set c1 to zero.  These imply that     */
/* c0 = 1-c2, and the transfer function is                                   */
/*                                                                           */
/*       1-c2                                                                */
/*    ----------                                                             */
/*    1-c2*z**-1                                                             */
/*                                                                           */
/* For better rejection at high frequencies we use a zero (withZero==TRUE)   */
/* at sampleRate/2 (z==-1) so c1=c0 and c0=(1-c2)/2 and the transfer         */
/* function is                                                               */
/*                                                                           */
/*    1-c2   1+   z**-1                                                      */
/*    ---- * ----------                                                      */
/*     2     1-c2*z**-1                                                      */
/*                                                                           */
/* Evaluating z=exp(j*w) (where w=2*pi*freq/sampleRate), we find that in     */
/* both cases we need to solve the quadratic equation  c2**2-2*u*c2+1 = 0    */
/* where                                                                     */
/*                                                                           */
/*        1-gain**2*cos(w)                                                   */
/*    u = ----------------             for the no zero case and              */
/*        1-gain**2                                                          */
/*                                                                           */
/*        (1+cos(w))-2*gain**2*cos(w)                                        */
/*    u = ---------------------------  for the case where we do want a zero. */
/*        (1+cos(w))-2*gain**2                                               */
/*                                                                           */
/*   There are two roots:                                                    */
/*                                                                           */
/*    c2 = u+sqrt(u**2-1)  and  c2 = u-sqrt(u**2-1)                          */
/*                                                                           */
/* The find_root function does this calculation, picking the appropriate     */
/* root, and takes care of potential overflow conditions.                    */
/*===========================================================================*/
void designFirstOrderLowpassCoeffs
(
    int32       coeffsL32Q23[3],            /* coeffs in L32Q23              */
    int16       mB,                         /* attenuation at cutoff freq    */
    int32       freqHz,                     /* cutoff frequency              */
    int32       sampleRate,                 /* sampling rate                 */
    boolean     withZero                    /* with a zero or not at fs/2    */
)
{
    int32 g2L32Q23, coswL32Q23, cosw1L32Q23;
    
    if (mB < 0 && freqHz <= sampleRate>>1) 
    {
        g2L32Q23 = Q23_initMB(mB*2);        // mB*2 for square

        coswL32Q23 = Q23_cosine_norm_freq(freqHz, sampleRate);
        if (withZero == TRUE)
        {
            cosw1L32Q23 = Q23_ONE + coswL32Q23;
            coeffsL32Q23[2] = find_root(
                cosw1L32Q23 - 2 * Q23_mult(g2L32Q23, coswL32Q23), 
                cosw1L32Q23 - 2 * g2L32Q23);
            coeffsL32Q23[0] = (Q23_ONE - coeffsL32Q23[2])/2;
            coeffsL32Q23[1] = coeffsL32Q23[0];
        } /* end of (withZero == TRUE) */
        else /* if (withZero == FALSE) */
        {
            coeffsL32Q23[2] = find_root(
                Q23_ONE - Q23_mult(g2L32Q23, coswL32Q23),
                Q23_ONE - g2L32Q23);
            coeffsL32Q23[0] = Q23_ONE - coeffsL32Q23[2];
            coeffsL32Q23[1] = 0;
        } /* end of else (withZero == FALSE) */
    } /* end of if (mB < 0 && freqHz <= sampleRate>>1) */
    else 
    {
        coeffsL32Q23[0] = Q23_ONE;          // use all-pass
        coeffsL32Q23[2] = 0;
        coeffsL32Q23[1] = 0;
    } /* end of else */
} /*----------- end of function designFirstOrderLowpassCoeffs ---------------*/


/*===========================================================================*/
/* FUNCTION : designFirstOrderHighpassCoeffs                                 */
/*                                                                           */
/* DESCRIPTION: Design a first order high-pass filter, coeffs in L32Q23.     */
/*                                                                           */
/* INPUTS: coeffsL32Q23-> coeffs in L32Q23                                   */
/*         mB: attenuation (millibel) at cutoff frequency                    */
/*         freqHz: cutoff frequency                                          */
/*         sampleRate: sampling rate                                         */
/*         withZero: with a zero or not at 0 Hz                              */
/* OUTPUTS: coeffsL32Q23-> coeffs in L32Q23                                  */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*---------------------------------------------------------------------------*/
/* The first order recurrence equation is                                    */
/* y(n) = c0*x(n)+c1*x(n-1)+c2*y(n-1), and the transfer function is          */
/*                                                                           */
/*    c0+c1*z**-1                                                            */
/*    -----------                                                            */
/*     1-c2*z**-1                                                            */
/*                                                                           */
/* A high-pass filter requires that gain is 1 at sampleRate/2 (z==-1).  For  */
/* a fast implementation (withZero==FALSE) we set c1 to zero.  These imply   */
/* that c0 = 1+c2, and the transfer function is                              */
/*                                                                           */
/*       1+c2                                                                */
/*    ----------                                                             */
/*    1-c2*z**-1                                                             */
/*                                                                           */
/* For better rejection at low frequencies we use a zero (withZero==TRUE)    */
/* at 0 Hz (z==1) so c1=-c0 and c0=(1+c2)/2 and the transfer function is     */
/*                                                                           */
/*    1+c2   1-   z**-1                                                      */
/*    ---- * ----------                                                      */
/*     2     1-c2*z**-1                                                      */
/*                                                                           */
/* In both cases we need to solve the quadratic equation                     */
/*                                                                           */
/*    c2**2-2*u*c2+1 = 0                                                     */
/*                                                                           */
/* where                                                                     */
/*                                                                           */
/*          1+gain**2*cos(w)                                                 */
/*    u = - ----------------             for the no zero case and            */
/*          1-gain**2                                                        */
/*                                                                           */
/*          (1-cos(w))+2*gain**2*cos(w)                                      */
/*    u = - ---------------------------  where we do want a zero.            */
/*          (1-cos(w))-2*gain**2                                             */
/*                                                                           */
/*   There are two roots:                                                    */
/*                                                                           */
/*    c2 = u+sqrt(u**2-1)  and  c2 = u-sqrt(u**2-1)                          */
/*                                                                           */
/* The find_root function does this calculation, picking the appropriate     */
/* root, and takes care of potential overflow conditions.                    */
/*===========================================================================*/
void designFirstOrderHighpassCoeffs
(
    int32       coeffsL32Q23[3],            /* coeffs in L32Q23              */
    int16       mB,                         /* attenuation at cutoff freq    */
    int32       freqHz,                     /* cutoff frequency              */
    int32       sampleRate,                 /* sampling rate                 */
    boolean     withZero                    /* with a zero or not at fs/2    */
)
{
    int32 g2L32Q23, coswL32Q23, cosw1L32Q23;

    if (mB >= 0) 
    {
        coeffsL32Q23[0] = Q23_ONE;          // all-pass
        coeffsL32Q23[1] = 0;
        coeffsL32Q23[2] = 0;
    } /* end of if (mB >= 0) */
    else if (freqHz < sampleRate>>1) 
    {
        g2L32Q23   = Q23_initMB(mB*2);     // *2 for square
        coswL32Q23 = Q23_cosine_norm_freq(freqHz, sampleRate);
        
        if (withZero == TRUE)
        {
            cosw1L32Q23 = Q23_ONE - coswL32Q23;
            coeffsL32Q23[2] = find_root(
                -cosw1L32Q23 - 2*Q23_mult(g2L32Q23, coswL32Q23), 
                 cosw1L32Q23 - 2*g2L32Q23);
            coeffsL32Q23[0] = (Q23_ONE + coeffsL32Q23[2])/2;
            coeffsL32Q23[1] = -coeffsL32Q23[0];
        } /* end of if (withZero == TRUE) */
        else /* if (withZero == FALSE) */
        {
            coeffsL32Q23[2] = find_root(
                -Q23_ONE - Q23_mult(g2L32Q23, coswL32Q23),
                 Q23_ONE - g2L32Q23);
            coeffsL32Q23[0] = Q23_ONE + coeffsL32Q23[2];
            coeffsL32Q23[1] = 0;
        } /* end of else (withZero == FALSE) */
    } /* end of else if (freqHz < sampleRate>>1) */
    else 
    {
        coeffsL32Q23[0] = 0;                      // no-pass
        coeffsL32Q23[1] = 0;
        coeffsL32Q23[2] = 0;
    } /* end of else */
}

/* QSound version */
//----------------------------------------------------------------------------
// DesignFirstOrderCoeffs_Zero()
// - General purpose first order filter design, with zero.
//
// Arguments:   coeffs[3] - designed filter coefficients
//              dc_mb - gain at DC (0 Hz) (mb)
//              nyquist_mb - gain at sampleRate/2 (Nyquist rate, or pi radians) (mb)
//              mb - gain at transition frequency freqHz (mb)
//              freqHz - transition frequency (samples/second)
//              sampleRate - sample rate of filter (samples/second)
// Returns:     none
//----------------------------------------------------------------------------
void DesignFirstOrderCoeffs_Zero(int32 coeffs[3],
    int32 dc_mb, int32 nyquist_mb, int32 mb,
    int32 freqHz, int32 sampleRate
    )
{
    // assert(mb<s32_max_s32_s32(dc_mb, nyquist_mb));
    mb = s32_min_s32_s32(mb, s32_max_s32_s32(dc_mb, nyquist_mb));

    //
    // The first order recurrence equation is y(n) = c0*x(n)+c1*x(n-1)+c2*y(n-1) and the
    // transfer function H is
    //
    //    c0+c1*z**-1
    //    -----------
    //     1-c2*z**-1
    //
    // At frequencies 0 and pi we have z = 1 and -1, respectively.  Let gains
    // at these frequencies be g0 and gp, we get
    //
    //         c0+c1         c0-c1
    //    g0 = ----- ,  gp = -----
    //          1-c2          1+c2
    //
    // These can be rerranged so that
    //
    //    c0 = (g0+gp)/2 - (g0-gp)/2*c2
    //    c1 = (g0-gp)/2 - (g0+gp)/2*c2
    //
    // Evaluating z = exp(j*w) (where w = 2*pi*freq/sampleRate), we find that
    // the square of the gain g is
    //
    //           c0**2 + c1**2 + 2*c0*c1*cos(w)
    //    g**2 = ------------------------------
    //           1 + c2**2 - 2*c2*cos(w)
    //
    // Cranking through the math, we find that we can get c2 by solving the quadradic
    //
    //    c2**2 - 2*u*c2 + 1 = 0
    //
    // where u is the ratio
    //
    //        (g0**2-gp**2)+(g0**2+gp**2)*cos(w)-2*g**2*cos(w)
    //    u = ------------------------------------------------
    //        (g0**2+gp**2)+(g0**2-gp**2)*cos(w)-2*g**2
    //
    // There are two roots to this kind of quadratic
    //
    //    c2 = u+sqrt(u**2-1)  and  c2 = u-sqrt(u**2-1)
    //
    // The FindRoot function does this calculation, picking the appropriate root, and takes
    // care of potential overflow conditions.
    //
    int32 g0 = Q23_initMB(dc_mb);               // gain at 0
    int32 gp = Q23_initMB(nyquist_mb);          // gain at pi
    int32 g  = Q23_initMB(mb);                  // gain at w (2*pi*freqHz/sampleRate)

    int32 g02 = Q23_square(g0);                 // squares
    int32 gp2 = Q23_square(gp);
    int32 g2  = Q23_square(g);

    int32 g2_sum  = g02+gp2;                    // sum and difference
    int32 g2_diff = g02-gp2;

    int32 cosw = Q23_cosine_norm_freq(freqHz, sampleRate);

    // solve quadratic for c2, rational form (i.e. numerator and denominator) used to avoid overflows
    coeffs[2] = find_root(g2_diff+Q23_mult(g2_sum-2*g2, cosw), g2_sum+Q23_mult(g2_diff, cosw)-2*g2);

    // get other coefficients
    int32 g_sum  = (g0+gp)/2;
    int32 g_diff = (g0-gp)/2;
    coeffs[0] = g_sum  - Q23_mult(g_diff, coeffs[2]);
    coeffs[1] = g_diff - Q23_mult(g_sum , coeffs[2]);
}

//----------------------------------------------------------------------------
// DesignFirstOrderCoeffs_NoZero()
// - General purpose first order filter design, without zero.
//
// Arguments:  coeffs - designed filter coefficients
//          mb1 - gain at f1 (mb)
//          f1 - first design frequency (samples/second)
//          mb2 - gain at f2 (mb)
//          f2 - second design frequency (samples/second)
//          sampleRate - sample rate of filter (samples/second)
// Returns:    none
//----------------------------------------------------------------------------
void DesignFirstOrderCoeffs_NoZero(int32 coeffs[3],
   int32 mb1, int32 f1,
   int32 mb2, int32 f2,
   int32 sampleRate
   )
{
   // assert(f1!=f2);

   int32 g1 = Q23_initMB(mb1);               // gain at f1
   int32 g2 = Q23_initMB(mb2);               // gain at f2
   if (f1==f2 || g1==g2) {
      coeffs[0] = g1;
      coeffs[1] = 0;
      coeffs[2] = 0;
   }
   else {
      //
      // The first order recurrence equation is y(n) = c0*x(n)+c1*x(n-1)+c2*y(n-1) and the
      // transfer function is
      //
      //    c0+c1*z**-1
      //    -----------
      //     1-c2*z**-1
      //
      // For a faster implementation we set c1 to zero, meaning that the filter has no
      // zero in its transfer function, which becomes
      //
      //        c0
      //    ----------
      //    1-c2*z**-1
      //
      // The magnitude of this is
      //
      //                 c0**2
      //    H**2 = -------------------
      //           1+c2**2-2*c2*cos(w)
      //
      // where w = 2*pi*freq/sampleRate).  We can solve for c2 if given two gains,
      // g1 and g2, at two frequencies f1 and f2:
      //
      //   g1**2*[1-2*cos(w1)*c2+c2**2] = c0**2
      //   g2**2*[1-2*cos(w2)*c2+c2**2] = c0**2
      //
      // This results in needing to solve the quadratic equation  c2**2-2*u*c2+1 = 0
      // where
      //
      //        g1**2*cos(w1)-g2**2*cos(w2)
      //    u = ---------------------------
      //               g1**2-g2**2
      //
      // There are two roots:
      //
      //    c2 = u+sqrt(u**2-1)  and  c2 = u-sqrt(u**2-1)
      //
      // The FindRoot function does this calculation, picking the appropriate root, and takes
      // care of potential overflow conditions.
      //
      // The c0 coefficient is derived by picking one of the above equations, although a
      // speedup is available if it is recognized that sqrt[1-2*cos(w1)*c2+c2**2] is
      // 1-c2 or 1+c2 when cos(w) is +1 or -1, respectively.
      //
      int32 g1_sqr = Q23_square(g1);
      int32 g2_sqr = Q23_square(g2);

      int32 cosw1 = Q23_cosine_norm_freq(f1, sampleRate);
      int32 cosw2 = Q23_cosine_norm_freq(f2, sampleRate);

      // solve quadratic for c2, rational form (i.e. numerator and denominator) used to avoid overflows
      coeffs[2] = find_root(Q23_mult(g1_sqr, cosw1)-Q23_mult(g2_sqr, cosw2), g1_sqr-g2_sqr);

      int32 g, cosw;
      if (g1!=0) {
         g = g1, cosw = cosw1;
      }
      else {
         g = g2, cosw = cosw2;
      }
      int32 c0_norm;
      if (cosw==Q23_ONE) {
         c0_norm = Q23_ONE-coeffs[2];
      }
      else if (cosw==Q23_MINUSONE) {
         c0_norm = Q23_ONE+coeffs[2];
      }
      else {
         c0_norm = Q23_sqrt( Q23_ONE - Q23_mult(2*coeffs[2], cosw) + Q23_square(coeffs[2]) );
      }
      coeffs[0] = Q23_mult(g, c0_norm);
      coeffs[1] = 0;
   }
}

/*===========================================================================*/
/* FUNCTION : cascadeFirstOrderFilters                                       */
/*                                                                           */
/* DESCRIPTION: Combine coefficients of two first-order IIR filters to make  */
/*              the coefficients for the equivalent cascaded biquad.         */
/*                                                                           */
/* INPUTS: coeffsL32Q23-> biquad coeffs, L32Q23                              */
/*         uL32Q23-> first-order filter coeffs                               */
/*         vL32Q23-> first-order filter coeffs                               */
/* OUTPUTS: coeffsL32Q23-> biquad coeffs, L32Q23                             */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*---------------------------------------------------------------------------*/
/* The coefficients for the first-order IIR represent the calculations       */
/*                                                                           */
/*    y1(n) = u[0]*x(n) + u[1]*x(n-1) + u[2]*y1(n-1)                         */
/*    y(n) = v[0]*y1(n) + v[1]*y1(n-1) + u[2]*y(n-1)                         */
/*                                                                           */
/* The cascaded transfer function is                                         */
/*                                                                           */
/*   u[0]+u[1]*z**-1   v[0]+v[1]*z**-1                                       */
/*   --------------- * --------------- =                                     */
/*   1   -u[2]*z**-1   1   -v[2]*z**-1                                       */
/*                                                                           */
/*              u[0]*v[0] + (u[0]*v[1]+u[1]*v[0])*z**-1 + u[1]*v[1]*z**-2    */
/*              ---------------------------------------------------------    */
/*              1         - (u[2]+v[2]          )*z**-1 + u[2]*v[2]*z**-2    */
/*                                                                           */
/* The equivalent biquad recurrence equation is                              */
/*                                                                           */
/*   y(n) = u[0]*v[0]*x(n) - u[2]*v[2]*y(n-2) + (u[2]+v[2])*y(n-1) +         */
/*          u[1]*v[1]*x(n-2) + (u[0]*v[1]+u[1]*v[0])*x(n-1)                  */
/*                                                                           */
/* so the biquad coefficients are                                            */
/*      u[0]*v[0], -u[2]*v[2], u[2]+v[2], u[1]*v[1], u[0]*v[1]+u[1]*v[0].    */
/*===========================================================================*/
void cascadeFirstOrderFilters
(
    int32 coeffsL32Q23[5],                  /* biquad coeffs, L32Q23         */
    int32 uL32Q23[3],                       /* first-order filter coeffs     */
    int32 vL32Q23[3]                        /* first-order filter coeffs     */
)
{
    coeffsL32Q23[0] = Q23_mult(uL32Q23[0], vL32Q23[0]);
    coeffsL32Q23[1] = -Q23_mult(uL32Q23[2], vL32Q23[2]);
    coeffsL32Q23[2] = uL32Q23[2] + vL32Q23[2];
    coeffsL32Q23[3] = Q23_mult(uL32Q23[1], vL32Q23[1]);
    coeffsL32Q23[4] = Q23_mult(uL32Q23[0], vL32Q23[1]) + 
                      Q23_mult(uL32Q23[1], vL32Q23[0]);
} /*--------------- end of function cascadeFirstOrderFilters ----------------*/


/*===========================================================================*/
/* FUNCTION : designBiquadLowpassCoeffs                                      */
/*                                                                           */
/* DESCRIPTION: Design low-pass biquad based on two 1st order low-pass       */
/*              filters cascaded together.                                   */
/*                                                                           */
/* INPUTS: coeffsL32Q23-> coeffs in L32Q23                                   */
/*         mB: attenuation (millibel) at cutoff frequency                    */
/*         freqHz: cutoff frequency                                          */
/*         sampleRate: sampling rate                                         */
/*         withZero: with a zero or not at samplerate/2                      */
/* OUTPUTS: coeffsL32Q23-> coeffs in L32Q23                                  */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
void designBiquadLowpassCoeffs
(
    int32       coeffsL32Q23[5],            /* coeffs in L32Q23              */
    int16       mB,                         /* attenuation at cutoff freq    */
    int32       freqHz,                     /* cutoff frequency              */
    int32       sampleRate,                 /* sampling rate                 */
    boolean     withZero                    /* with a zero or not at fs/2    */
)
{
    int32 lowL32Q23[3];
    designFirstOrderLowpassCoeffs(lowL32Q23,mB>>1,freqHz,sampleRate,withZero);
    cascadeFirstOrderFilters(coeffsL32Q23, lowL32Q23, lowL32Q23);
} /*------------- end of function designBiquadLowpassCoeffs -----------------*/


/*===========================================================================*/
/* FUNCTION : designBiquadBandpassCoeffs                                     */
/*                                                                           */
/* DESCRIPTION: Design band-pass biquad based on one 1st order low-pass      */
/*              and one 1st order high-pass filter cascaded together.        */
/*                                                                           */
/* INPUTS: coeffsL32Q23-> coeffs in L32Q23                                   */
/*         mB: attenuation (millibel) at cutoff frequency                    */
/*         freqHz: cutoff frequency                                          */
/*         sampleRate: sampling rate                                         */
/*         withZero: with a zero or not                                      */
/* OUTPUTS: coeffsL32Q23-> coeffs in L32Q23                                  */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
void designBiquadBandpassCoeffs
(
    int32       coeffsL32Q23[5],            /* coeffs in L32Q23              */
    int16       mB,                         /* attenuation at cutoff freq    */
    int32       freqHz,                     /* cutoff frequency              */
    int32       sampleRate,                 /* sampling rate                 */
    boolean     withZero                    /* with a zero or not            */
)
{
    int32 lowL32Q23[3];
    int32 hiL32Q23[3];
    designFirstOrderHighpassCoeffs(hiL32Q23,mB>>1,freqHz,sampleRate,withZero);
    designFirstOrderLowpassCoeffs(lowL32Q23,mB>>1,freqHz,sampleRate,withZero);
    cascadeFirstOrderFilters(coeffsL32Q23, lowL32Q23, hiL32Q23);
} /*-------------- end of function designBiquadBandpassCoeffs ---------------*/


/*===========================================================================*/
/* FUNCTION : designBiquadHighpassCoeffs                                     */
/*                                                                           */
/* DESCRIPTION: Design high-pass biquad based on two 1st order high-pass     */
/*              filters cascaded together.                                   */
/*                                                                           */
/* INPUTS: coeffsL32Q23-> coeffs in L32Q23                                   */
/*         mB: attenuation (millibel) at cutoff frequency                    */
/*         freqHz: cutoff frequency                                          */
/*         sampleRate: sampling rate                                         */
/*         withZero: with a zero or not at 0 Hz                              */
/* OUTPUTS: coeffsL32Q23-> coeffs in L32Q23                                  */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
void designBiquadHighpassCoeffs
(
    int32       coeffsL32Q23[5],            /* coeffs in L32Q23              */
    int16       mB,                         /* attenuation at cutoff freq    */
    int32       freqHz,                     /* cutoff frequency              */
    int32       sampleRate                  /* sampling rate                 */
)
{
    int32 hiL32Q23[3];
    //designFirstOrderHighpassCoeffs(hiL32Q23,mB>>1,freqHz,sampleRate,withZero);

    // use QSound version to do bit-exact match
    DesignFirstOrderCoeffs_Zero(hiL32Q23, -12000, 0, mB/2, freqHz, sampleRate);
    cascadeFirstOrderFilters(coeffsL32Q23, hiL32Q23, hiL32Q23);
} /*------------ end of function designBiquadHighpassCoeffs -----------------*/
