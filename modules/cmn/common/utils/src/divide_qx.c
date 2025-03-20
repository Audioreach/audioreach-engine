/*-----------------------------------------------------------------------
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
-----------------------------------------------------------------------*/

/**
@file divide_qx.c

@brief This contains divide functions

*/

/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/
#include "audio_divide_qx.h"
#include "audio_basic_op.h"
#include "audio_basic_op_ext.h"


/***************************************************************************
*
*   FUNCTION NAME: s32_div_s32_s32_sat
*
*   PURPOSE:
*
*     Divide var1 by var2 and provide output in the requested Q format.
*     The two inputs must be in the same Qformat.
*     The two inputs are normalized so as to represent them with full
*     32-bit precision in the range [0,1). The normalized inputs are
*     truncated to upper 16-bits and 16-bit division is performed.
*     The quotient is re-normalized back to the requested Q format.
*
*   INPUTS:
*
*     var1
*               32 bit signed integer (int16_t) whose value
*               falls in the range 0x8000 0000 <= var1 <= 0x7fff ffff.
*     var2
*               32 bit signed integer (int16_t) whose value
*               falls in the range 0x8000 0000 <= var1 <= 0x7fff ffff.
*               var2 != 0.
*     iqL16:
*               Integer part of Qfactor for the output.
*
*   OUTPUTS:
*
*     none
*
*   RETURN VALUE:
*
*     outL32
*                     32 bit signed integer (int32_t) with
*                     Q format specified in the argument.
*
*
*   KEYWORDS: div
*
*************************************************************************/

int32_t s32_div_s32_s32_sat(int32_t var1, int32_t var2, int16_t iqL16)
{
    int32_t tmp32, outL32=0;
    int16_t qexp, tmp16;

#ifdef WMOPS_FX
    counter_fx.s32_div_s32_s32_normalized++;
#endif

    if(var1 ==0)
    {
       return outL32;
    }

    if(var2 !=0)
    {
      tmp32 = s32_div_s32_s32_normalized(var1, var2, &qexp);
      outL32 = s32_shl_s32_sat(tmp32, 16);

   	  qexp = qexp - iqL16;
   	  outL32 = s32_shl_s32_sat( outL32, qexp);                       /* denormalize the result */
    }
    else
    {
   	   if(iqL16 > 0)
   	   {
   		  tmp16 = 0x8000 >> iqL16;
   	   }
   	   else
   	   {
   	      tmp16 = MAX_16;
   	   }

	   outL32 = s32_shl_s32_sat((int32_t)tmp16, 16);
     }

	 return outL32;

}
#if !defined(__qdsp6__)

/* ------------------------------------------------------------- */
/*  CONSTANTS                                                    */
/* ------------------------------------------------------------- */
#define DSP_INV_LUT_BITS   4   /* number of bits to index the inverse lookup table (actually 2^(val-1) segments) */
#define DSP_INV_LUT_SIZE  (1+(1<<(DSP_INV_LUT_BITS-1)))  /* number of entries in the inverse LUT */

/* ------------------------------------------------------------- */
/*  LOOK UP TABLES                                               */
/*                                                               */
/* this LUT has been generated using the matlab function         */
/* "find_optimal_lut_mse_cantoni.m" in                           */
/*  \\vivekv\Public\Jaguar\cdma1x\matlab. See the matlab file    */
/* for details of the mathematical analysis.                     */
/* ------------------------------------------------------------- */
int16 invTable[DSP_INV_LUT_SIZE]= {32690, 29066, 26171, 23798, 21820, 20145, 18709, 17463, 16373};

/*==========================================================================*/
/* FUNCTION: divide_int32_qx                                                */
/*                                                                          */
/* DESCRIPTION: numerator/denominator (both in the same Q) and outputs      */
/*              quotient in Q0.                                             */
/*                                                                          */
/* INPUTS: numeratorL32: numerator in Qin.                                  */
/*         denominatorL32: denominator in Qin.                              */
/* OUTPUTS: quotient: quotient of operation in Q0.                          */
/*                                                                          */
/* IMPLEMENTATION NOTES:                                                    */
/*                                                                          */
/*        if(num >= 2^31*den)                                               */
/*           num = num - den;                                               */
/*           quotient = quotient<<1+1;                                      */
/*        else                                                              */
/*           quotient = quotient<<1;                                        */
/*      num = num<<1;                                                       */
/*                                                                          */
/*==========================================================================*/
int32_t divide_int32_qx(int32_t numeratorL32,
                              int32_t denominatorL32,
                              int16_t outputQ)
{
    int32_t quotientL32=0;
    int16_t nShift, dShift;
    int16_t i, count;
    int16_t negFlag=0;
    int40 shiftNumerL40, tempL40;

    if(numeratorL32 < 0)
    {
        negFlag = 1;
        numeratorL32 = -numeratorL32;
    }
    if(denominatorL32 < 0)
    {
        negFlag = !negFlag;
        denominatorL32 = -denominatorL32;
    }
    if(numeratorL32 == 0)
    {
        return(0);

    }
    if(denominatorL32 == 0)
    {
        return(0xFFFFFFFF);

    }


    /* normalize numerator and denominator */
    nShift=s16_norm_s32(numeratorL32);
    shiftNumerL40 = s32_shl_s32_sat(numeratorL32, nShift);
    dShift = s16_norm_s32(denominatorL32);
    denominatorL32 = s32_shl_s32_sat(denominatorL32, dShift);

    count = s16_add_s16_s16(dShift,1);
    count = s16_sub_s16_s16(count, nShift);
    count = s16_add_s16_s16(count, outputQ);
    for(i = 0; i<count; i++)
    {
        tempL40 = s40_sub_s40_s40(shiftNumerL40, denominatorL32);
        shiftNumerL40 = s40_shl_s40(shiftNumerL40,1);
        quotientL32 = s32_shl_s32_sat(quotientL32,1 );
        if (tempL40 >=0)
        {
            shiftNumerL40 = s40_shl_s40(tempL40,1);
            quotientL32 = s32_add_s32_s32_sat(quotientL32,1);
        }
    }/* end of repetitions*/
    if(negFlag)
        quotientL32 = -quotientL32;
    return quotientL32;
} /* end of divide_int32() function. */
#endif //#if !defined(__qdsp6__)
/* adding Q16 and Q23 divide function for bit-exact matching */
static int32_t Q16_divideUU(int32_t numer, int32_t denom)
{
   uint32_t x, y, t;
   int i;
   x = numer>>16;
   y = numer<<16;
   for (i = 1; i<=32; i++) {
      t = (int32_t)x>>31;		// all 1's if x[31]==1
      x = (x<<1) | (y>>31);			// shift x||y left by one bit
      y <<= 1;						// ...
      if ((x|t)>=(uint32_t)denom) {
         x -= denom;
         y += 1;
      }
   }
   // x is remainder, y is quotient
   return x>=(uint32_t)denom/2 ? y+1 : y;
}

int32_t Q16_divide(int32_t numer, int32_t denom)
{
   bool_t negative;
   int32_t quotient;

   if (numer==0) return 0;

   negative = FALSE;
   if (numer<0) numer = -numer, negative = TRUE;
   if (denom<0) denom = -denom, negative = !negative;

   quotient = numer==denom ? 65536 : Q16_divideUU((uint32_t)numer, (uint32_t)denom);
   return negative ? -quotient : quotient;
}

/* end of added divide code */

/*===================================================================*/
/* FUNCTION      :  audio_divide_dp ().                                    */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  double precision division                        */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*         _ (int40)      Lnum:   numerator                         */
/*         _ (int40)      Ldenom: denominator                       */
/*         _ (int16_t)      Qadj: Q factor adjustor                   */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*         _ None                                                    */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*         _ None                                                    */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                */
/*         _(int32_t)       Lquotient in Q(29+Qadj)                   */
/*                         0 if Ldenom=0                             */
/*===================================================================*/
int40 audio_divide_dp(int40 Lnum, int40 Ldenom, int16_t Qadj)
{
  int40 Lacc,Lacc1;
  int32_t Ltemp3;
  int16_t th, dh, nh, t1h, i, n1;
  uint16_t tl, dl, nl, t1l;

  int16_t densign = 0;
  if (Ldenom == 0) return(0);
  n1 = s16_norm_s40(Lnum);  // normalize numerator
  Lnum <<= n1;
  if (Ldenom < 0)densign =-1;
  if (Ldenom < 0)Ldenom =-Ldenom;
  Qadj=n1 - Qadj;
  n1=s16_norm_s40(Ldenom);
  Ldenom <<= n1;
  Qadj = n1-Qadj;
  Ltemp3 = 0x55555555;

  for (i=0;i<5;i++) {
    tl=s16_extract_s32_l(Ltemp3);
    th=s16_extract_s32_h(Ltemp3);
	dl=s16_extract_s32_l(s32_saturate_s40(Ldenom));
    dh=s16_extract_s32_h(s32_saturate_s40(Ldenom));

	Lacc=s40_add_s40_s40(s40_mult_s16_u16_shift(th,dl,0),s40_mult_s16_u16_shift(dh,tl,0));
	Lacc=s40_add_s40_s40(Lacc,0x8000);

	Lacc >>=16;
    Lacc1=s40_add_s40_s40(s40_mult_s16_s16_shift(th,dh,0),s64_mult_s32_s32(0xffff8000,0x8000));
	Lacc=s40_add_s40_s40(Lacc,Lacc1);
	Lacc=s32_saturate_s40(Lacc);

	t1l=s16_extract_s32_l(s32_saturate_s40(Lacc));
    t1h=s16_extract_s32_h(s32_saturate_s40(Lacc));

	Lacc= -0x8000;
	Lacc=s40_sub_s40_s40(Lacc, s40_mult_s16_u16_shift(th,t1l,3));
	Lacc=s40_sub_s40_s40(Lacc, s40_mult_s16_u16_shift(t1h,tl,3));

	Lacc >>=16;
	Lacc=s40_sub_s40_s40(Lacc, s40_mult_s16_s16_shift(th,t1h,3));
    Ltemp3=s32_saturate_s40(Lacc);
    }
    if(densign < 0) Lacc = s40_sub_s40_s40(0,Lacc);

    Ltemp3=s32_saturate_s40(Lacc);
  	nl=s16_extract_s32_l(s32_saturate_s40(Lnum));
    nh=s16_extract_s32_h(s32_saturate_s40(Lnum));
	tl=s16_extract_s32_l(Ltemp3);
    th=s16_extract_s32_h(Ltemp3);
  	Lacc=s40_add_s40_s40(s40_mult_s16_u16_shift(th,nl,0),s40_mult_s16_u16_shift(nh,tl,0));
	Lacc=s40_add_s40_s40(Lacc,0x8000);

	Lacc >>=16;
	Lacc1=s40_mult_s16_s16_shift(th,nh,0);
	Lacc=s40_add_s40_s40(Lacc,Lacc1);

	Lacc=s40_shl_s40(Lacc, Qadj);
	return(s32_saturate_s40(Lacc));
}

/*===================================================================*/
/* FUNCTION      :  audio_divide_sp()                                  */
/*-------------------------------------------------------------------*/
/* PURPOSE       :  single precision division with Q6 friendly       */
/*                  implementation (low MIPS)                        */
/*                  Low MIPS replacement for div_int32().            */
/*-------------------------------------------------------------------*/
/* INPUT ARGUMENTS  :                                                */
/*         _ (int32)      Lnum:   numerator                          */
/*         _ (int32)      Ldenom: denominator                        */
/*         _ (int16)      Qadj: Q factor adjustor                    */
/*-------------------------------------------------------------------*/
/* OUTPUT ARGUMENTS :                                                */
/*         _ None                                                    */
/*-------------------------------------------------------------------*/
/* INPUT/OUTPUT ARGUMENTS :                                          */
/*         _ None                                                    */
/*-------------------------------------------------------------------*/
/* RETURN ARGUMENTS :                                                */
/*         _(int32)       Lquotient in Q(31-Qadj)                    */
/*                        Max value if Ldenom=0                      */
/*===================================================================*/
#if !defined(__qdsp6__)
int32 audio_divide_sp(int32 Lnum, int32 Ldenom, int16 Qadj)
{
    int40 Lacc=0,Lacc1;
    int32 Ltemp3;
    int16 i, n1, densign = 0;

    // Return max value if the denominator is zero
    if (Ldenom == 0)
    {
        if(Lnum >= 0)
        {
            return(0x7fffffffL);        // Positive maximum
        }
        else
        {
            return(0x80000000L);        // Negative maximum
        }
    }

    // Adjust the input Qadj factor so as to make it integer part of output Qfactor
    Qadj = 1 - Qadj;

    // Normalize the numerator to improve precision
    n1 = s16_norm_s32(Lnum);               // normalize numerator
    Lnum <<= n1;

    // Modify the adjustment factor based on numerator exponent
    Qadj=n1 - Qadj;

    // For negative denominators, take care of the sign
    if (Ldenom < 0)
    {
        densign =-1;
        Ldenom =-Ldenom;
    }

    // Normalize the denominator to improve precision
    n1=s16_norm_s32(Ldenom);
    Ldenom <<= n1;

    // Modify the adjustment factor based on denominator exponent
    Qadj = n1-Qadj;

    // Set the initial guess for division iteration
    Ltemp3 = 0x55555555L;

    // Division implementation using iterative procedure
    // Implement 1/D ~ x*(2-D*x) with an initial guess x
    // and iterate the inverse 5 times to get more accurate result
    // x: Q1.30, D: Q0.31.
    for (i=0;i<5;i++)
    {
        // Implement D*x and store the product as Q2.29
        // No rounding done in the product computation
        Lacc1 = s64_sub_s64_s64(0x4000000000000000LL, s64_mult_s32_s32(Ltemp3, Ldenom));
        Lacc = s64_shl_s64(Lacc1, -32);

        //    Lacc = (int64)Q6_P_asr_PI(Lacc1, 32);
        Lacc = s64_mult_s32_s32((int32)Lacc, Ltemp3);

        //    Ltemp3=(int32)(Q6_P_asr_PI(Lacc,29));
        Ltemp3 = (int32) s64_shl_s64(Lacc, -29);
        Lacc = (int64)Ltemp3;
    }

    // Reverse the sign if the denominator is negative
    if(densign < 0)
    {
        Lacc = s64_sub_s64_s64(0,Lacc);
        Ltemp3=s32_saturate_s64(Lacc);
    }

    // Multiply the inverse with the numerator to get the result in Q1.30
    Lacc = s64_mult_s32_s32(Ltemp3, Lnum);

    // Adjust the shift factor of the result to the desired Q-factor
    if (-33 >= Qadj)
    {
        if (0 <= Lacc)
        {
            Lacc = 0;
        }
        else
        {
            Lacc = -1;
        }
    }
    else
    {
        Lacc=s64_shl_s64(Lacc, Qadj-31);
    }

    // Return the adjusted result
    return(s32_saturate_s64(Lacc));
}

/*--------------------------------------------------------------
 * dsplib_approx_invert()
 *
 *     DSP library function "approx_invert".
 *     Approximates inversion:  out ~= 2^(31-floor(lg2(in)))/in
 *
 *     Approximation done with a linearly interpolated lookup table. With
 *     9 point entries (8 line segments) the maximum error is 0.238%. The
 *     number to be inverted must be positive for valid results. If not
 *     positive, then the lookup table is invalidly indexed.
 *
 *     If input is Qi and the output is Qo, then
 *        Qo = 32 + (*shift_factor) - Qi.
 *
 * Inputs:
 *     input          integer to be inverted
 *
 * Return:
 *     int64 (shift_factor : result)
 *     where
 *      result        Q-shifted inverse
 *                    result ~= 2^(31-floor(lg2(input))) / input
 *                             = 2^(32+(*shift_factor)) / input
 *      shift_factor  (output_Q_factor - 32) of integer inverse
 *                    shift_factor = -1-floor(lg2(input))
 *
 *      if input <=0:  return
 *        result       = 0xFFFFFFFF
 *        shift_factor = 0xFFFFFFFF
 *----------------------------------------------------------------*/
int64 dsplib_approx_invert(int32 input)
{
  int32 norm_divisor;
  int32 n1,interp;
  int32 index;
  int32 r;
  int32 result, shift_factor;

  /* check for negative input (invalid) */
  if (input <= 0) {
    return (-1);
  }

  /* bit-align-normalize input */
  r = (int32)s16_norm_s32(input);
  norm_divisor = s32_shl_s32_sat(input, (int16)r);

  /* determine inverse LUT index and interpolation factor */
  n1     = norm_divisor >> (15-DSP_INV_LUT_BITS);
  interp = n1 % (1<<16);
  index  = (n1 >> 16) - (DSP_INV_LUT_SIZE-1);

  /* inverse linear interpolation between LUT entries */
  result = (invTable[index]<<16) + interp * (invTable[index+1] - invTable[index]);
  shift_factor = r-31;

  /* return values  */
  return ( ((int64)shift_factor<<32)|(uint32)result );

}

/*--------------------------------------------------------------
 * dsplib_approx_divide()
 *
 *     DSP library function "approx_divide".
 *     Approximates inversion:  out ~= numer*2^(31-floor(lg2(in)))/denom
 *
 *     Approximation done with a linearly interpolated lookup table. With
 *     9 point entries (8 line segments) the maximum error is 0.238%. The
 *     number to be inverted must be positive for valid results. If not
 *     positive, then the lookup table is invalidly indexed.
 *
 *
 * Inputs:
 *     numer          integer numerator
 *     denom          integer denominator
 *
 * Return:
 *     int64 (shift_factor : result)
 *     where
 *      result        - quotient
 *                    result ~= numer*2^(31-floor(lg2(input))) / denom
 *                             = numer*2^(32+(*shift_factor)) / denom
 *      shift_factor  - Qfactor of (*result)
 *
 *
 * Notes : (*result << *shift_factor) will be floating point result in Q0
 *
 *----------------------------------------------------------------*/
int64 dsplib_approx_divide(int32 numer,int32 denom)
{
  int32 norm_num,r,s_d,s_n;
  int64 norm_denom;
  int32 result, shift_factor;

  if (denom <= 0) {
    return (-1);
  }

  norm_denom = dsplib_approx_invert(denom);
  r = (int32)norm_denom;
  s_d = (int32)(norm_denom>>32);
  s_d=s_d+1;

  s_n = (int32)s16_norm_s32(numer);
  norm_num = s32_shl_s32_sat(numer, (int16)s_n);

  result = s32_mult_s32_s16_rnd_sat(norm_num, s16_round_s32_sat(r));
  shift_factor = s_d - s_n;

  return ( ((int64)shift_factor<<32)|(uint32)result );
}
#endif

