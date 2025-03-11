/**=========================================================================
  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause
============================================================================ */

/* FILE NAME: basic_math.c                                                   *
 * DESCRIPTION:                                                              *
 *   Contains basic math operations such as square-root, logarithm,          *
 *   exponential etc.                                                        *
 *                                                                           *
 * FUNCTION LIST:                                                            *
 * int32_t div_compute ()                                                    *
 * int32_t sqrt_compute ()                                                   *
 * int32_t audio_log_compute ()                                              *
 * int32_t exp_compute ()                                                    *
 * int32_t div_int32_t ()                                                    *
 *===========================================================================*/

/*===========================================================================*
 *           INCLUDE HEADER FILES                                            *
 *===========================================================================*/
#include <string.h>
#include <stdio.h>
#include "basic_math.h"
#include "audio_fft_basic_ops.h"

#ifndef __qdsp6__

#include "audio_divide_qx.h"
#define div_s s16_div_s16_s16_sat

/*============================================================================*/
/*                                                                            */
/* FUNCTION:        sqrt_compute                                              */
/* DESCRIPTION:                                                               */
/*       Implements a cubic spline approximation for computing square-root of */
/*       of a 32-bit fractional number                                        */
/* PRECONDITIONS:                                                             */
/*       pwrInL32: Input 32-bit power value                                   */
/*       iqInL16: Integer part of Q-factor for the input power                */
/*       iqOutL16: Integer part of Q-factor for the square-root output        */
/*       Input and output Q-factors can be specified any value from 0 to 31   */
/*       Output will be rounded and saturated according to the iqOutL16 value */
/* POSTCONDITIONS:                                                            */
/*       sqrtL32:  Returns the 32-bit square-root output                      */
/*                                                                            */
/* Square root too expensive, so the square root is approximated              */
/* with third order polynomial computed by Song Wang. The input power is      */
/* normalized to be in the range 1 <= cal_in < 4 and the approximate          */
/*  square-root is computed and then it is normalized back appropriately.     */
/*                                                                            */
/*    sqrtL32 = ((S0*pwrInL32 + S1)*pwrInL32 + S2)*pwrInL32 + S3.             */
/*    S0 = 0.0089 Q15, S1 = -0.1029 Q29, S2 = 0.6605 Q29, S3 = 0.4334 Q29     */
/*    sqrt error ratio is less than -50 dB for all positive real numbers      */
/*============================================================================*/
int32_t sqrt_compute ( int32_t pwrInL32,
                     int16_t iqInL16,
                 int16_t iqOutL16
                )
{
  int32_t accu32, sqrtL32, rnd32;
  int16_t pwrInL16, pwr_norm, mant_norm, pwr_offset, out_norm;

  if (pwrInL32 <= 0)
  {
    sqrtL32 = 0;
     return sqrtL32;
  }

  // Computing normalization factor for extracting the mantissa of the input power
  pwr_norm = s16_norm_s32(pwrInL32);              // Compute norm of the input power
  mant_norm = (iqInL16-1) - pwr_norm;       // Compute integer part of log2 norm of the input power
  mant_norm = (mant_norm >>1)<<1;           // Compute highest even integer less than log2 norm
  pwr_offset = (iqInL16-2) - mant_norm;     // Compute offset for shifting the Qfactor to Q2.29

  // Normalizing the power input so that it falls in the range: 1 <= pwr_in < 4
  // and then changing its Qfactor to Q2.29 to allow maximum precision represenation
  pwrInL32 = s32_shl_s32_sat(pwrInL32, pwr_offset);
  pwrInL16 = s16_round_s32_sat(pwrInL32);          // Round to 16-bits Q2.13

  // Implementing cubic approximation to compute the square-root
  accu32 = (int32_t )s32_mult_s32_s16_rnd_sat(pwrInL32, SQRT_S0);      // S0*x
  sqrtL32 = s32_add_s32_s32_sat(accu32, SQRT_S1);                         // S0*x+S1
  accu32 = (int32_t )s32_mult_s32_s16_rnd_sat(sqrtL32, pwrInL16);      // (S0*x+S1)*x
  accu32 = s32_shl_s32_sat(accu32, 2);
  sqrtL32 = s32_add_s32_s32_sat(accu32, SQRT_S2);                         // (S0*x+S1)*x+S2
  accu32 = (int32_t )s32_mult_s32_s16_rnd_sat(sqrtL32, pwrInL16);      // ((S0*x+S1)*x+S2)*x
  accu32 = s32_shl_s32_sat(accu32, 2);
  sqrtL32 = s32_add_s32_s32_sat(accu32, SQRT_S3);                         // ((S0*x+S1)*x+S2)*x+S3

  // Scaling back and rounding and adjusting the Q-factor of the square-root result
  out_norm = (iqOutL16-2) - (mant_norm >>1);
  rnd32 = s32_shl_s32_sat(((int32_t )0x0001),out_norm-1);               // Rounding value for shifting
  sqrtL32 = s32_add_s32_s32_sat(sqrtL32, rnd32);                          // Rounding the result before shifting
  sqrtL32 = s32_shl_s32_sat(sqrtL32, -out_norm);

  return sqrtL32;
}
/*============================================================================*/
/*                                                                            */
/* FUNCTION:        audio_log_compute                                               */
/* DESCRIPTION:                                                               */
/*       Implements a normalized cubic approximation for computing log10      */
/*       of a 32-bit number                                                   */
/* PRECONDITIONS:                                                             */
/*       pwrInL32: Input 32-bit power value                                   */
/*       iqInL16: Integer part of Q-factor for the input power                */
/*       iqOutL16: Integer part of Q-factor for the log10 output              */
/*       logConstL16Q14: Constant to convert from log2 to log_base            */
/*       To compute log10, let logConstL16Q14 = log10(2) in Q14               */
/*       Input and output Q-factors can be specified any value from 0 to 31   */
/*       Output will be rounded and saturated according to the iqOutL16 value */
/*       Depending on the value of logConstL16Q14 you can compute log with    */
/*       any base. logConstL16Q14 = log_base(2) in Q14                        */
/* POSTCONDITIONS:                                                            */
/*       logL32:  Returns the 32-bit log10 output                             */
/*                                                                            */
/* Logarithm is too expensive, so it is approximated as a cubic               */
/* third order polynomial. The input power is expressed in mantissa and       */
/* exponent with mantissa in the range [1,2] and log2 approximation is        */
/* applied only to mantissa and log2 of the input power is computed.          */
/*    logL32 = ((C0*mant + C1)*mant + C2)*mant + C3 + exponent.               */
/* C0 = 0.1840 Q15, C1 = -1.1688 Q2.29, C2 = 3.2187 Q2.29, C3 = -2.2340 Q2.29 */
/* mean squared error ratio is less than -50 dB for all positive real numbers */
/* Finally, log to any base is computed from log2 using the logConst value    */
/*============================================================================*/
int32_t audio_log_compute   ( int32_t pwrInL32,
                      int16_t iqInL16,
                  int16_t iqOutL16,
                      int16_t logConstL16Q14
                )
{
  int32_t accu32, logL32, rnd32, out_exp32;
  int16_t pwrInL16, pwr_norm, exp_norm, mant_norm, pwr_offset, out_norm;

  if (pwrInL32 <= 0)
  {
    logL32 = MIN_32;
     return logL32;
  }

  // Computing normalization factor for extracting the mantissa of the input power
  pwr_norm = s16_norm_s32(pwrInL32);              // Compute norm of the input power
  mant_norm = (iqInL16-1) - pwr_norm;       // Compute integer part of log2 norm of the input power
  pwr_offset =(iqInL16-2) - mant_norm;      // Compute offset for shifting the Qfactor to Q2.29

  // Normalizing the power input so that it falls in the range: 1 <= pwr_in < 2
  // and then changing its Qfactor to Q2.29 to allow maximum precision represenation
  pwrInL32 = s32_shl_s32_sat(pwrInL32, pwr_offset);
  pwrInL16 = s16_round_s32_sat(pwrInL32);           // Round to 16-bits Q2.13

  // Implementing cubic approximation to compute log2 of the mantissa
  accu32 = (int32_t )s32_mult_s32_s16_rnd_sat(pwrInL32, LOG_C0);     // C0*x
  logL32 = s32_add_s32_s32_sat(accu32, LOG_C1);                         // C0*x+C1
  accu32 = (int32_t )s32_mult_s32_s16_rnd_sat(logL32, pwrInL16);     // (C0*x+C1)*x
  accu32 = s32_shl_s32_sat(accu32, 2);
  logL32 = s32_add_s32_s32_sat(accu32, LOG_C2);                         // (C0*x+C1)*x+C2
  accu32 = (int32_t )s32_mult_s32_s16_rnd_sat(logL32, pwrInL16);     // ((C0*x+C1)*x+C2)*x
  accu32 = s32_shl_s32_sat(accu32, 2);
  logL32 = s32_add_s32_s32_sat(accu32, LOG_C3);                         // ((C0*x+C1)*x+C2)*x+C3

  // Normalizing the exponent so as to express it with highest precision
  out_exp32 = (int32_t )mant_norm;
  exp_norm = s16_norm_s32(out_exp32);
  out_exp32 = s32_shl_s32_sat(out_exp32, exp_norm);                 // Qfactor shifted Q(31-exp_norm).exp_norm

  // Converting from log2 to log10 of the input
  // After conversion, logL32 is Q3.28 and out_exp32 is Q(32-exp_norm).(exp_norm-1)
  logL32 = (int32_t )s32_mult_s32_s16_rnd_sat(logL32, logConstL16Q14);
  out_exp32 = (int32_t )s32_mult_s32_s16_rnd_sat(out_exp32, logConstL16Q14);

  // Rounding and adjusting the Q-factor of the log of the mantissa
  out_norm = iqOutL16-3;
  rnd32 = s32_shl_s32_sat(((int32_t )0x0001),out_norm-1);             // Rounding value for shifting
  logL32 = s32_add_s32_s32_sat(logL32, rnd32);                          // Rounding the result before shifting
  logL32 = s32_shl_s32_sat(logL32, -out_norm);

  // Adjusting the Q-factor of the input exp and adding it to the output
  out_exp32 = s32_shl_s32_sat(out_exp32, (32-exp_norm)-iqOutL16);
  logL32 = s32_add_s32_s32_sat(logL32, out_exp32);

  return logL32;
}
/*============================================================================*/
/*                                                                            */
/* FUNCTION:        exp_compute                                               */
/* DESCRIPTION:                                                               */
/*       Implements a normalized cubic approximation for computing exp        */
/*       of a 32-bit number                                                   */
/* PRECONDITIONS:                                                             */
/*       xInL32: Input 32-bit number                                          */
/*       iqInL16: Integer part of Q-factor for the input                      */
/*       iqOutL16: Integer part of Q-factor for the exp output                */
/*       expConstL16Q13: Constant to convert from 2^(x) to exp(x)             */
/*       To compute exp(x), let expConstL16Q13 = log2(exp) in Q13             */
/*       Input and output Q-factors can be specified any value from 0 to 31   */
/*       Output will be rounded and saturated according to the iqOutL16 value */
/*       Depending on the value of expConstL16Q13 you can compute 2^(x) or    */
/*       exp(x) or 10^(x). expConstL16Q13 = log2(base_power)                  */
/*       Since it is in Q13 format, cannot use more than 16^(x) with it.      */
/* POSTCONDITIONS:                                                            */
/*       expL32:  Returns the 32-bit exp output                               */
/*                                                                            */
/* Exponential is too expensive, so it is approximated as a cubic             */
/* third order polynomial. The input number is scaled down to the range       */
/* [0,1] and exp approximation is applied and the output is scaled back       */
/* to the original range.                                                     */
/*    expL32 = (((C0*a*x + C1)*a*x + C2)*a*x + C3)*2.^n.                      */
/*  a = log2(e) = 1.4427 Q2.13 - can be modified to compute power to any base */
/*   C0 = 0.0778 Q15, C1 = 0.2258 Q2.29, C2 = 0.6962 Q2.29, C3 = 1 Q2.29      */
/* mean squared error ratio is less than -50 dB for all real numbers          */
/*============================================================================*/
int32_t exp_compute ( int32_t xInL32,
                    int16_t iqInL16,
                int16_t iqOutL16,
                    int16_t expConstL16Q13
               )
{
  int32_t accu32, expL32, rnd32, x_shift32;
  int16_t xInL16, x_floor, x_norm, out_norm;

  // Computing normalization factor for adjusting the Q-factor of the input
  x_norm = s16_norm_s32(xInL32);

  x_norm = s16_min_s16_s16(x_norm, 11);
  // Normalize the input by shifting its iq to (iq-x_norm+2) for better precision
  xInL32 = s32_shl_s32_sat(xInL32, x_norm-2);

  // Multiply the input with log2(e) to enable computing 2^x instead of e^x
  xInL32 = (int32_t )s32_mult_s32_s16_rnd_sat(xInL32, expConstL16Q13);
  xInL32 = s32_shl_s32_sat(xInL32,2);                 // Qfactor compensation for expConstL16Q13 (Q2.29)

  // Extract the integer part of the input so as to subtract it from the input
  x_floor = s16_saturate_s32(s32_shl_s32_sat(xInL32, iqInL16-29-x_norm));

  // Shift back the integer part and subtract it from the input to get its fractional part
  x_shift32 = s32_shl_s32_sat((int32_t )x_floor, x_norm+29-iqInL16);
  xInL32 = s32_sub_s32_s32_sat(xInL32,x_shift32);

  // Re-scale the fractional part of the input so as to shift its Qfactor to Q2.29
  xInL32 = s32_shl_s32_sat(xInL32, iqInL16-x_norm);
  xInL16 = s16_round_s32_sat(xInL32);              // Round to 16-bits Q2.13

  // Implementing cubic approximation to compute 2^x of the fractional part
  accu32 = (int32_t )s32_mult_s32_s16_rnd_sat(xInL32, EXP_C0);        // C0*x
  expL32 = s32_add_s32_s32_sat(accu32, EXP_C1);                          // C0*x+C1
  accu32 = (int32_t )s32_mult_s32_s16_rnd_sat(expL32, xInL16);        // (C0*x+C1)*x
  accu32 = s32_shl_s32_sat(accu32, 2);
  expL32 = s32_add_s32_s32_sat(accu32, EXP_C2);                          // (C0*x+C1)*x+C2
  accu32 = (int32_t )s32_mult_s32_s16_rnd_sat(expL32, xInL16);        // ((C0*x+C1)*x+C2)*x
  accu32 = s32_shl_s32_sat(accu32, 2);
  expL32 = s32_add_s32_s32_sat(accu32, EXP_C3);                          // ((C0*x+C1)*x+C2)*x+C3

  // Scaling back and rounding and adjusting the Q-factor of the square-root result
  out_norm = (iqOutL16-2) - x_floor;
  rnd32 = s32_shl_s32_sat(((int32_t )0x0001),out_norm-1);               // Rounding value for shifting
  expL32 = s32_add_s32_s32_sat(expL32, rnd32);                            // Rounding the result before shifting
  expL32 = s32_shl_s32_sat(expL32, -out_norm);

  return expL32;
}

#endif /* __qdsp6__ */

/*============================================================================*/
/*                                                                            */
/* FUNCTION:        sqrt_compute_Nth                                          */
/* DESCRIPTION:                                                               */
/*       Implements an Nth order polynomial approximation for computing       */
/*       square-root of a 32-bit fractional number                            */
/* PRECONDITIONS:                                                             */
/*       pwrInL32: Input 32-bit power value                                   */
/*       iqInL16: Integer part of Q-factor for the input power                */
/*       iqOutL16: Integer part of Q-factor for the square-root output        */
/*       Input and output Q-factors can be specified any value from 0 to 31   */
/*       Output will be rounded and saturated according to the iqOutL16 value */
/* POSTCONDITIONS:                                                            */
/*       sqrtL32:  Returns the 32-bit square-root output                      */
/*                                                                            */
/* Square root too expensive, so the square root is approximated              */
/* with third order polynomial computed by Song Wang. The input power is      */
/* normalized to be in the range 1 <= cal_in < 4 and the approximate          */
/*  square-root is computed and then it is normalized back appropriately.     */
/*                                                                            */
/*    sqrtL32 = ((S0*pwrInL32 + S1)*pwrInL32 + S2)*pwrInL32 + S3.             */
/*    sqrtL32 = sqrtL32 * pwrInL32 + S4                                       */
/*    sqrtL32 = sqrtL32 * pwrInL32 + S5                                       */
/*============================================================================*/
#if SQRT_NTH_ORD == 5 // 5th
/* approx sqrt(x), 1<= x <4, by 5th order poly, < -74dB error */
   #define SQRT_NTH_S0_L16 23669
   #define SQRT_NTH_S0_Q -10
   #define SQRT_NTH_S1_Q29 -5896532
   #define SQRT_NTH_S2_Q29 38497248
   #define SQRT_NTH_S3_Q29 -143483846
   #define SQRT_NTH_S4_Q29 460002699
   #define SQRT_NTH_S5_Q29 187372645
#endif
#if SQRT_NTH_ORD == 4 // 4th
/* approx sqrt(x), 1<= x <4, by 4th order poly, < -64dB error */
   #define SQRT_NTH_S0_L16 -20234
   #define SQRT_NTH_S0_Q -8
   #define SQRT_NTH_S1_Q29 17131329
   #define SQRT_NTH_S2_Q29 -96397849
   #define SQRT_NTH_S3_Q29 411055888
   #define SQRT_NTH_S4_Q29 206376491
#endif
#if SQRT_NTH_ORD == 3 // 3rd
/* approx sqrt(x), 1<= x <4, by 3rd order poly, < -52dB error */
   #define SQRT_NTH_S0_L16 17249
   #define SQRT_NTH_S0_Q -6
   #define SQRT_NTH_S1_Q29 -52587488
   #define SQRT_NTH_S2_Q29 349139229
   #define SQRT_NTH_S3_Q29 235903316
#endif

int32 sqrt_compute_Nth ( int32 pwrInL32,
                     int16 iqInL16,
                 int16 iqOutL16
                )
{
  int32 accu32, sqrtL32, rnd32;
  int16 pwrInL16, pwr_norm, mant_norm, pwr_offset, out_norm;

  if (pwrInL32 <= 0)
  {
    sqrtL32 = 0;
     return sqrtL32;
  }

  // Computing normalization factor for extracting the mantissa of the input power
  pwr_norm = s16_norm_s32(pwrInL32);              // Compute norm of the input power
  mant_norm = (iqInL16-1) - pwr_norm;       // Compute integer part of log2 norm of the input power
  mant_norm = (mant_norm >>1)<<1;           // Compute highest even integer less than log2 norm
  pwr_offset = (iqInL16-2) - mant_norm;     // Compute offset for shifting the Qfactor to Q2.29

  // Normalizing the power input so that it falls in the range: 1 <= pwr_in < 4
  // and then changing its Qfactor to Q2.29 to allow maximum precision represenation
  pwrInL32 = s32_shl_s32_sat(pwrInL32, pwr_offset);
  pwrInL16 = s16_round_s32_sat(pwrInL32);          // Round to 16-bits Q2.13

  // Implementing cubic approximation to compute the square-root
  accu32 = (int32 )s32_mult_s32_s16_rnd_sat(pwrInL32, SQRT_NTH_S0_L16);      // S0*x
  accu32 = s32_shl_s32_sat(accu32, SQRT_NTH_S0_Q);                               // S0 Q
  sqrtL32 = s32_add_s32_s32_sat(accu32, SQRT_NTH_S1_Q29);                         // S0*x+S1
  accu32 = (int32 )s32_mult_s32_s16_rnd_sat(sqrtL32, pwrInL16);      // (S0*x+S1)*x
  accu32 = s32_shl_s32_sat(accu32, 2);
  sqrtL32 = s32_add_s32_s32_sat(accu32, SQRT_NTH_S2_Q29);                         // (S0*x+S1)*x+S2
  accu32 = (int32 )s32_mult_s32_s16_rnd_sat(sqrtL32, pwrInL16);      // ((S0*x+S1)*x+S2)*x
  accu32 = s32_shl_s32_sat(accu32, 2);
  sqrtL32 = s32_add_s32_s32_sat(accu32, SQRT_NTH_S3_Q29);                         // ((S0*x+S1)*x+S2)*x+S3
#if SQRT_NTH_ORD > 3
  accu32 = (int32 )s32_mult_s32_s16_rnd_sat(sqrtL32, pwrInL16);      // (((S0*x+S1)*x+S2)*x+S3)*x
  accu32 = s32_shl_s32_sat(accu32, 2);
  sqrtL32 = s32_add_s32_s32_sat(accu32, SQRT_NTH_S4_Q29);                         // (((S0*x+S1)*x+S2)*x+S3)*x+S4
#endif
#if SQRT_NTH_ORD > 4
  accu32 = (int32 )s32_mult_s32_s16_rnd_sat(sqrtL32, pwrInL16);      // ((((S0*x+S1)*x+S2)*x+S3)*x+S4)*x
  accu32 = s32_shl_s32_sat(accu32, 2);
  sqrtL32 = s32_add_s32_s32_sat(accu32, SQRT_NTH_S5_Q29);                         // ((((S0*x+S1)*x+S2)*x+S3)*x+S4)*x+S5
#endif
  // Scaling back and rounding and adjusting the Q-factor of the square-root result
  out_norm = (iqOutL16-2) - (mant_norm >>1);
  rnd32 = s32_shl_s32_sat(((int32 )0x0001),out_norm-1);               // Rounding value for shifting
  sqrtL32 = s32_add_s32_s32_sat(sqrtL32, rnd32);                          // Rounding the result before shifting
  sqrtL32 = s32_shl_s32_sat(sqrtL32, -out_norm);

  return sqrtL32;
}


/*============================================================================*/
/*                                                                            */
/* FUNCTION:    div_int32                                                     */
/* DESCRIPTION:                                                               */
/*     Divides two int32 numbers and returns the quotient with the            */
/*     specified Q-factor                                                     */
/* PRECONDITIONS:                                                             */
/*     numL32:  numerator for the division                                    */
/*     denL32:  denominator for the division                                  */
/*     iqL16:  integer part of qfactor for the quotient                       */
/*     numL32 and denL32 must have the same Q-factor                          */
/* POSTCONDITIONS:                                                            */
/*     quotL32: returned quotient with the specified Q-factor                 */
/*                                                                            */
/*============================================================================*/
int32 div_int32( int32 numL32,
             int32 denL32,
             int16 iqL16
                )
{
     int32 tmpdata1, quotL32=0;
    int16 qexp, tmp16;

    if(numL32 ==0)
    {
       return quotL32;
    }

     if(denL32 !=0)
     {
//       tmpdata1 = div32_normalized(numL32, denL32, &qexp);    /* do normalized division */
        tmpdata1 = div_compute(numL32, denL32, &qexp);

//      MOV_MSB( tmpdata1, NBITS, quotL32, 2*NBITS, int32 );
        quotL32 = ( tmpdata1 << NBITS );

    qexp = qexp - iqL16;
    quotL32 = s32_shl_s32_sat( quotL32, qexp);                       /* denormalize the result */
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
//        MOV_MSB( tmp16, NBITS, quotL32, 2*NBITS, int32 );
        quotL32 = (((int32 )tmp16) << NBITS );
     }
   return quotL32;
}

/*============================================================================*/
/*                                                                            */
/* FUNCTION:        div_compute                                               */
/* DESCRIPTION:                                                               */
/*          The two inputs are normalized so as to represent them with full   */
/*          32-bit precision in the range [0,1). The normalized inputs are    */
/*          truncated to upper 16-bits and divided and the normalized         */
/*          quotient is returned with 16-bit precision where the 16-bits      */
/*          are in LSB. The normalized quotient is in Q16.15 format.          */
/*          The required shift factor to adjust the quotient value            */
/*          is returned in the pointer qShiftL16Ptr.                          */
/*          To get the output in a desired 32-bit Qformat Qm.(31-m),          */
/*          left shift the output by  qShiftL16Ptr+(16-m).                    */
/*          To get the output in a desired 16-bit Qformat Qm.(15-m)with       */
/*          16-bit result in the LSB of the 32-bit output, left shift the     */
/*          output by  qShiftL16Ptr-m.                                        */
/* PRECONDITIONS:                                                             */
/*          numL32: numerator for division                                    */
/*          denL32: denominator for division                                  */
/*          qShiftL16Ptr:  pointer to output shift factor                     */
/* POSTCONDITIONS:                                                            */
/*          outL32: returned normalized quotient value                        */
/*          qShiftL16Ptr:  pointer to modified shift factor for               */
/*                           denormalizing the quotient                       */
/*============================================================================*/

int32 div_compute( int32 numL32,
               int32 denL32,
               int16 *qShiftL16Ptr
              )
{
     int32 outL32=0;
     int16 num_exp, den_exp, tmp1, tmp2, sign = 1;

     if(numL32 == 0)
    {
      qShiftL16Ptr[0] = 0;
      return outL32;
    }
     if(numL32 < 0)
     {
        numL32 = s32_neg_s32_sat(numL32);
        sign = -1;
     }
     if(denL32 < 0)
     {
        denL32 = s32_neg_s32_sat(denL32);
        sign = -sign;
     }

     if(denL32 != 0)
     {
        // Compute the exponents of the numerator and denominator
        num_exp = s16_norm_s32( numL32 );
        den_exp = s16_norm_s32( denL32 );

        // Normalize the inputs such that numerator < denominator and are in full 32-bit precision
       tmp1 = s16_extract_s32_h( s32_shl_s32_sat( numL32, num_exp-1 ));
       tmp2 = s16_extract_s32_h( s32_shl_s32_sat( denL32, den_exp ));

       // Perform division with 16-bit precision
        outL32 = (int32 )div_s( tmp1, tmp2);

        // Shift factor value for de-normalizing the quotient
       qShiftL16Ptr[0] = den_exp - num_exp +1;
     }
     else
     {
        outL32 = 0xFFFF;
        qShiftL16Ptr[0] = 15;
     }

     // Negate the quotient if either numerator or denominator is negative
     if(sign < 0)
     {
        outL32 = -outL32;
     }

   return outL32;
}
