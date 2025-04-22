/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*========================================================================

B A S I C   O P   E X T

*//** @file basic_op_ext.c
This file contains the C implementations of the basic_op_ext intrinsics.

@par EXTERNALIZED FUNCTIONS
(none)

@par INITIALIZATION AND SEQUENCING REQUIREMENTS
(none)
*//*====================================================================== */

/* =======================================================================

INCLUDE FILES FOR MODULE

========================================================================== */

#include "audio_basic_op.h"
#include "audio_basic_op_ext.h"
#include "ar_defs.h"

/***************************************************************************
*
*   FUNCTION NAME: s64_complex_mult_rnd_sat_s64_s32
*
*   PURPOSE:
*
*     Perform multiplication of two complex with high32 packing.
*
*   INPUTS:
*
*     var1
*                     complex
*     var2
*                     complex
*
*   OUTPUTS:
*
*     none
*
*   RETURN VALUE:
*
*     L_Out
*                     complex
*
*   IMPLEMENTATION:
*     Multiply two complex with high 32bit packing.
*
*   KEYWORDS: multiply, mult, mpy, complex
*
    int16_t tempImagL16, tempRealL16;
    int32_t resultL32 = 0x00;

    tempImagL16 = s16_extract_s32_h(s32_saturate_s64(
                        (s64_mult_s32_s32(s16_imag_s32(var1),s16_real_s32(var2)) << 1)
                        + (s64_mult_s32_s32(s16_imag_s32(var2),s16_real_s32(var1)) << 1)
                        + 0x8000));
    tempRealL16 = s16_extract_s32_h(s32_saturate_s64(
                        (s64_mult_s32_s32(s16_real_s32(var1),s16_real_s32(var2)) << 1)
                        - (s64_mult_s32_s32(s16_imag_s32(var1),s16_imag_s32(var2)) << 1)
                        + 0x8000));


    resultL32 |= tempImagL16 << 16;
    resultL32 |= (int32_t)tempRealL16 & 0xFFFF;
*************************************************************************/
int64 s64_complex_mult_rnd_sat_s64_s32(int64 x, int32_t y)
{
   int32_t xRe, xIm;
   int16_t yRe, yIm;
   int32_t zRe, zIm;

   xRe = s32_real_s64(x);
   xIm = s32_imag_s64(x);
   yRe = s16_real_s32(y);
   yIm = s16_imag_s32(y);

   // the complex MPY is defined in such a way in order
   // to facilitate faster ASM implementation on Q6

   //      zIm = s32_saturate_s64((
   //                      (s64_mult_s32_s16(xIm, yRe) << 1)
   //                      + (s64_mult_s32_s16(xRe, yIm) << 1)
   //                      + 0x8000) >> 16);
   //      zRe = s32_saturate_s64((
   //                      (s64_mult_s32_s16(xRe, yRe) << 1)
   //                      - (s64_mult_s32_s16(xIm, yIm) << 1)
   //                      + 0x8000) >> 16);
   zRe = s32_complex_mult_s32_s16_sat(xRe, yRe);
   zIm = s32_complex_mult_s32_s16_sat(xIm, yRe);

   zRe = s32_add_s32_s32(zRe, s32_complex_mult_s32_s16_sat(s32_neg_s32_sat(xIm), yIm));

   // to match Q6 definition of 32x16 MAC
   if ((xRe == (int32_t)0x80000000L) && (yIm == (int16_t)0x8000))
      zIm = s32_saturate_s64((int64)zIm + 0x080000000LL);
   else
      zIm = s32_add_s32_s32(zIm, s32_complex_mult_s32_s16_sat(xRe, yIm));

   return (s64_complex_s32_s32(zRe, zIm));
} /* End of s64_complex_mult_rnd_sat_s64_s32 function*/

#if !defined(L_shift_r)
/***************************************************************************
 *
 *   FUNCTION NAME: L_shift_r
 *
 *   PURPOSE:
 *
 *     Shift and round.  Perform a shift right. After shifting, use
 *     the last bit shifted out of the LSB to round the result up
 *     or down.  Optimized for lower MIPS (11/12/10).
 *
 *   INPUTS:
 *
 *     L_var1
 *                     32 bit long signed integer (int32_t) whose value
 *                     falls in the range
 *                     0x8000 0000 <= L_var1 <= 0x7fff ffff.
 *     var2
 *                     16 bit short signed integer (int16_t) whose value
 *                     falls in the range 0xffff 8000 <= var2 <= 0x0000 7fff.
 *
 *   OUTPUTS:
 *
 *     none
 *
 *   RETURN VALUE:
 *
 *     L_var1
 *                     32 bit long signed integer (int32_t) whose value
 *                     falls in the range
 *                     0x8000 0000 <= L_var1 <= 0x7fff ffff.
 *
 *
 *   IMPLEMENTATION:
 *
 *     Shift and round.  Perform a shift right. After shifting, use
 *     the last bit shifted out of the LSB to round the result up
 *     or down.  This is just like shift_r above except that the
 *     input/output is 32 bits as opposed to 16.
 *
 *     if var2 is positve perform a arithmetic left shift
 *     with saturation (see l_shl() above).
 *
 *     If var2 is zero simply return L_var1.
 *
 *     If var2 is negative perform a arithmetic right shift (L_shr)
 *     of L_var1 by (-var2)+1.  Add the LS bit of the result to
 *     L_var1 shifted right (L_shr) by -var2.
 *
 *     Note that there is no constraint on var2, so if var2 is
 *     -0xffff 8000 then -var2 is 0x0000 8000, not 0x0000 7fff.
 *     This is the reason the l_shl function is used.
 *
 *
 *   KEYWORDS:
 *
 *************************************************************************/
int32_t L_shift_r(int32_t L_var1, int16_t var2)
{
    int32_t L_Out, L_rnd;
    int16_t swSign;

    // Simple shifting operation
    L_Out = s32_shl_s32_sat(L_var1, var2);

    // Extract the sign of the shift amount
    swSign = (var2 & 0x8000) >> 15;

      // Rounding operation for right shift
    L_rnd = (s32_shl_s32_sat(L_var1, (int16_t)(var2+1))) & swSign;

    L_Out = s32_add_s32_s32_sat(L_Out, L_rnd);

#ifdef WMOPS_FX
    counter_fx.l_shl-=2;
    counter_fx.l_add_sat--;
    counter_fx.L_shift_r++;
#endif

    return (L_Out);
}
#endif //#if !defined(L_shift_r)
