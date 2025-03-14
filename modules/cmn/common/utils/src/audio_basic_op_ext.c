/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
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
 *   FUNCTION NAME: s32_cl0_s32
 *
 *   PURPOSE:
 *     counts the number of consecutive zeros starting with the most
 *     significant bits.
 *
 *
 *************************************************************************/
#if !defined(s32_cl0_s32)
int32_t s32_cl0_s32(int32_t var)
{
    int count = 0;
    int i;

    for ( i = 31 ; i >= 0 ; i-- ) {
        if ((var & ( 0x01 << i )) == 0) {
            count++;
        }
        else {
            break;
        }
    }
    return count;
}

#endif //#if !defined(s32_cl0_s32)

#if !defined(s32_cl1_s64)
int32_t s32_cl1_s64(int64 var)
{
	int32_t count = 0;
	int16_t i;

	for ( i = 63 ; i >= 0 ; i-- ) {
		if ((var & ( 0x01LL << i )) != 0) {
			count++;
		}
		else {
			break;
		}
	}
	return count;
}
#endif //#if !defined(s32_cl1_s64)

#if !defined(s32_cl0_s64)
int32_t s32_cl0_s64(int64 var)
{
	int32_t count = 0;
	int16_t i;

	for ( i = 63 ; i >= 0 ; i-- ) {
		if ((var & ( 0x01LL << i )) == 0) {
			count++;
		}
		else {
			break;
		}
	}
	return count;
}
#endif //#if !defined(s32_cl0_s64)

#if !defined(s16_imag_s32)
/***************************************************************************
 *
 *   FUNCTION NAME: s16_imag_s32
 *
 *   PURPOSE:
 *
 *     extract imagine from the complex
 *
 *   INPUTS:
 *
 *     var1
 *                     complex
 *
 *   OUTPUTS:
 *
 *     none
 *
 *   RETURN VALUE:
 *
 *     L_Out
 *                     imagine
 *
 *   IMPLEMENTATION:
 *     extract imagine s16_extract_s32_h
 *
 *   KEYWORDS: imagine, imag, complex
 *
 *************************************************************************/
int16_t s16_imag_s32(int32_t vari)
{
    return s16_extract_s32_h(vari);
} /* End of s16_imag_s32 function */
#endif //#if !defined(s16_imag_s32)

#if !defined(s32_imag_s64)
/***************************************************************************
 *
 *   FUNCTION NAME: s32_imag_s64
 *
 *   PURPOSE:
 *
 *     extract imagine from the complex
 *
 *   INPUTS:
 *
 *     var1
 *                     complex
 *
 *   OUTPUTS:
 *
 *     none
 *
 *   RETURN VALUE:
 *
 *     L_Out
 *                     imagine
 *
 *   IMPLEMENTATION:
 *     extract imagine from the complex
 *
 *   KEYWORDS: imagine, imag, complex
 *
 *************************************************************************/
int32_t s32_imag_s64(int64 vari)
{
    return (int32_t)(vari >> 32);
} /* End of s32_imag_s64 function */
#endif //#if !defined(s32_imag_s64)

#if !defined(s16_real_s32)
/***************************************************************************
 *
 *   FUNCTION NAME: s16_real_s32
 *
 *   PURPOSE:
 *
 *     extract real from the complex
 *
 *   INPUTS:
 *
 *     var1
 *                     complex
 *
 *   OUTPUTS:
 *
 *     none
 *
 *   RETURN VALUE:
 *
 *     L_Out
 *                     real
 *
 *   IMPLEMENTATION:
 *     extract real s16_extract_s32_l
 *
 *   KEYWORDS: real, complex
 *
 *************************************************************************/
int16_t s16_real_s32(int32_t varr)
{
    return s16_extract_s32_l(varr);
} /* End of s16_real_s32 function */
#endif //#if !defined(s16_real_s32)

#if !defined(s32_real_s64)
/***************************************************************************
 *
 *   FUNCTION NAME: s32_real_s64
 *
 *   PURPOSE:
 *
 *     extract real from the complex
 *
 *   INPUTS:
 *
 *     var1
 *                     complex
 *
 *   OUTPUTS:
 *
 *     none
 *
 *   RETURN VALUE:
 *
 *     L_Out
 *                     real
 *
 *   IMPLEMENTATION:
 *     extract real s16_extract_s32_l
 *
 *   KEYWORDS: real, complex
 *
 *************************************************************************/
int32_t s32_real_s64(int64 varr)
{
    return (int32_t)(varr);
} /* End of s32_real_s64 function */
#endif //#if !defined(s32_real_s64)

#if !defined(s32_complex_s16_s16)
/***************************************************************************
 *
 *   FUNCTION NAME: s32_complex_s16_s16
 *
 *   PURPOSE:
 *
 *     pack real and imagine into a complex
 *
 *   INPUTS:
 *
 *     var1
 *                     real
 *     var2
 *                     imagine
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
 *     pack real and imagine
 *
 *   KEYWORDS: pack, complex
 *
 *************************************************************************/
int32_t s32_complex_s16_s16(int16_t varr, int16_t vari)
{
    int32_t resultL32 = 0x00;
    resultL32 |= ((vari & 0x0000FFFF) << 16);
    resultL32 |= (varr & 0x0000FFFF);

    return resultL32;
} /* End of s32_complex_s16_s16 function */
#endif //#if !defined(s32_complex_s16_s16)

#if !defined(s64_complex_s32_s32)
/***************************************************************************
 *
 *   FUNCTION NAME: s64_complex_s32_s32
 *
 *   PURPOSE:
 *
 *     pack real and imagine into a complex
 *
 *   INPUTS:
 *
 *     var1
 *                     real
 *     var2
 *                     imagine
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
 *     pack real and imagine
 *
 *   KEYWORDS: pack, complex
 *
 *************************************************************************/
int64 s64_complex_s32_s32(int32_t varr, int32_t vari)
{
    long long c;
    long long xr64 = (long long) varr;
    long long xi64 = (long long) vari;

    c = xr64 & (0xFFFFFFFFLL);
    c |= (xi64 << 32);

    return( c );
} /* End of s64_complex_s32_s32 function */
#endif //#if !defined(s64_complex_s32_s32)

#if !defined(s64_complex_add_s64_s64)
/***************************************************************************
 *
 *   FUNCTION NAME: s64_complex_add_s64_s64
 *
 *   PURPOSE:
 *
 *     add complex
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
 *     add complex
 *
 *   KEYWORDS: add, complex
 *
 *************************************************************************/
int64 s64_complex_add_s64_s64(int64 var1, int64 var2)
{
    int64 var1r = (int64) s32_real_s64(var1);
    int64 var2r = (int64) s32_real_s64(var2);
    int64 var1i = (int64) s32_imag_s64(var1);
    int64 var2i = (int64) s32_imag_s64(var2);

    return s64_complex_s32_s32((int32_t)(var1r + var2r), (int32_t)(var1i + var2i));

} /* End of s64_complex_add_s64_s64 function */
#endif //#if !defined(s64_complex_add_s64_s64)

#if !defined(s64_complex_sub_s64_s64)
/***************************************************************************
 *
 *   FUNCTION NAME: s64_complex_sub_s64_s64
 *
 *   PURPOSE:
 *
 *     sub complex
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
 *     sub complex
 *
 *   KEYWORDS: sub, complex
 *
 *************************************************************************/
int64 s64_complex_sub_s64_s64(int64 var1, int64 var2)
{
    int64 var1r = (int64) s32_real_s64(var1);
    int64 var2r = (int64) s32_real_s64(var2);
    int64 var1i = (int64) s32_imag_s64(var1);
    int64 var2i = (int64) s32_imag_s64(var2);

    return s64_complex_s32_s32((int32_t)(var1r-var2r), (int32_t)(var1i - var2i));

} /* End of s64_complex_sub_s64_s64 function */
#endif //#if !defined(s64_complex_sub_s64_s64)


#if !defined(s32_complex_mult_rnd_sat_s32_s32)
/***************************************************************************
*
*   FUNCTION NAME: s32_complex_mult_rnd_sat_s32_s32
*
*   PURPOSE:
*
*     Perform multiplication of two complex with high16 packing.
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
*     Multiply two complex with high 16bit packing.
*
*   KEYWORDS: multiply, mult, mpy, complex
*
*************************************************************************/
int32_t s32_complex_mult_rnd_sat_s32_s32( int32_t var1, int32_t var2 )
{
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

    return resultL32;

} /* End of s32_complex_mult_rnd_sat_s32_s32 function*/
#endif //#if !defined(s32_complex_mult_rnd_sat_s32_s32)

#if !defined(s32_complex_conjugate_sat_s32)
/***************************************************************************
*
*   FUNCTION NAME: s32_complex_conjugate_sat_s32
*
*   PURPOSE:
*
*     Perform conjugate of complex.
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
*     conjugate complex. a + bi -> a + bi, a - bi -> a + bi
*
*   KEYWORDS: conjugate, conj, complex
*
*************************************************************************/
int32_t s32_complex_conjugate_sat_s32( int32_t var1)
{
    int32_t resultL32 = 0x00;
    int16_t tempRealL16 = s16_real_s32(var1);
    int16_t tempImagL16 = s16_imag_s32(var1);

    resultL32 |= s16_neg_s16_sat(tempImagL16) << 16;
    resultL32 |= (int32_t)tempRealL16 & 0xFFFF;

    return resultL32;

} /* End of s32_complex_conjugate_sat_s32 function*/
#endif //#if !defined(s32_complex_conjugate_sat_s32)

#if !defined(s64_complex_conjugate_sat_s64)
/***************************************************************************
*
*   FUNCTION NAME: s64_complex_conjugate_sat_s64
*
*   PURPOSE:
*
*     Perform conjugate of complex.
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
*     conjugate complex. a + bi -> a + bi, a - bi -> a + bi
*
*   KEYWORDS: conjugate, conj, complex
*
*************************************************************************/
int64 s64_complex_conjugate_sat_s64( int64 var1)
{
    return s64_complex_s32_s32(s32_real_s64(var1), s32_neg_s32_sat(s32_imag_s64(var1)));

} /* End of s64_complex_conjugate_sat_s64 function*/
#endif //#if !defined(s64_complex_conjugate_sat_s64)

#if !defined(s64_complex_average_s64_s64)
/***************************************************************************
*
*   FUNCTION NAME: s64_complex_average_s64_s64
*
*   PURPOSE:
*
*     Average two long complex numbers with convergent rounding
*
*   INPUTS:
*
*     var1
*                     long complex
*     var2
*                     long complex
*
*   OUTPUTS:
*
*     none
*
*   RETURN VALUE:
*
*     L_Out
*                     long complex
*
*   IMPLEMENTATION:
*     Average two long complex numbers with convergent rounding
*
*   KEYWORDS: average, avg, complex
*
*************************************************************************/
int64 s64_complex_average_s64_s64(int64 x, int64 y)
{
    int64 t64;
    int32_t x32[2], y32[2];
    int32_t s32[2];
    int16_t i;

    x32[0] = s32_real_s64(x);
    x32[1] = s32_imag_s64(x);
    y32[0] = s32_real_s64(y);
    y32[1] = s32_imag_s64(y);

   for (i=0; i < 2; i++)
   {
      t64 = s64_add_s32_s32(x32[i], y32[i]);
      // convergent rounding
      t64 = s64_add_s64_s64( t64, (int64)( ( ((int32_t)t64)>>1 ) & 1) );
      t64 = s64_shl_s64(t64, -1);
      s32[i] = (int32_t) t64;
   }

   return( s64_complex_s32_s32(s32[0], s32[1]) );
}
#endif //#if !defined(s64_complex_average_s64_s64)

#if !defined(s64_complex_neg_average_s64_s64_sat)
/***************************************************************************
*
*   FUNCTION NAME: s64_complex_neg_average_s64_s64_sat
*
*   PURPOSE:
*
*     Average two long complex numbers with convergent rounding
*
*   INPUTS:
*
*     var1
*                     long complex
*     var2
*                     long complex
*
*   OUTPUTS:
*
*     none
*
*   RETURN VALUE:
*
*     L_Out
*                     long complex
*
*   IMPLEMENTATION:
*     Average two long complex numbers with convergent rounding
*
*   KEYWORDS: average, avg, complex
*
*************************************************************************/
int64 s64_complex_neg_average_s64_s64_sat(int64 x, int64 y)
{
    int64 t64;
    int32_t x32[2], y32[2];
    int32_t s32[2];
    int16_t i;

    x32[0] = s32_real_s64(x);
    x32[1] = s32_imag_s64(x);
    y32[0] = s32_real_s64(y);
    y32[1] = s32_imag_s64(y);

   for (i=0; i < 2; i++)
   {
      t64 = s64_sub_s64_s64(x32[i], y32[i]);
      // convergent rou nding
      t64 = s64_add_s64_s64( t64, (int64)( ( ((int32_t)t64)>>1 ) & 1) );
      t64 = s64_shl_s64(t64, -1);
      s32[i] = s32_saturate_s64(t64);
   }

   return( s64_complex_s32_s32(s32[0], s32[1]) );
}
#endif //#if !defined(s64_complex_neg_average_s64_s64_sat)

#if !defined(s32_bitrev_s32)
int32_t s32_bitrev_s32(int32_t x, int32_t BITS)
{
    int i;
    int32_t y = 0;
    for ( i= 0; i<BITS; i++) {
      y = (y << 1)| (x & 1);
      x >>= 1;
    }
    return y;
} /*End of s32_bitrev_s32 function */
#endif //#if !defined(s32_bitrev_s32)

#if !defined(u64_mult_u32_u32)
/***************************************************************************
*
*   FUNCTION NAME: u64_mult_u32_u32
*
*   PURPOSE:
*
*     Perform a multiplication of the two 32 bit input unsigned numbers
*     Output a 64 bit unsigned number.
*
*   INPUTS:
*
*     var1
*                     32 bit unsigned integer (uint32_t) whose value
*                     falls in the range 0x0000 0000 <= var1 <= 0xffff ffff.
*     var2
*                     32 bit unsigned integer (uint32_t) whose value
*                     falls in the range 0x0000 0000 <= var1 <= 0xffff ffff.
*
*   OUTPUTS:
*
*     none
*
*   RETURN VALUE:
*
*     L_Out
*                     64 bit long unsigned integer (uint64) whose value
*                     falls in the range
*                     0x0000 0000 0000 0000 <= out <= 0xffff ffff ffff ffff.
*
*   IMPLEMENTATION:
*
*     Multiply the two 32 bit input numbers.
*
*   KEYWORDS: multiply, mult, mpy
*
*************************************************************************/
uint64 u64_mult_u32_u32(uint32_t var1,uint32_t var2)
{
  return (uint64) var1 * var2;
}
#endif //if !defined(u64_mult_u32_u32)

#if !defined(s32_modwrap_s32_u32)
int32_t s32_modwrap_s32_u32(int32_t var1, uint32_t var2) {
    if ( var1 < 0 ) {
        return var1 + var2;
    }

    else if ( (uint32_t)var1 >= var2 ) {
        return var1 - var2;
    }

    return var1;
}
#endif //#if !defined(s32_modwrap_s32_u32)
#if !defined(s64_mac_s32_s32)
int64 s64_mac_s32_s32(int64 mac64, int32_t var1, int32_t var2)
{
   int64 tmp64;

   tmp64 = (int64) var1 * var2;

   return (mac64 + tmp64);
}
#endif // #if !defined(s64_mac_s32_s32)

#if !defined(s32_mac_s32_s32_s1_sat)
int32_t s32_mac_s32_s32_s1_sat(int32_t mac32, int32_t var1, int32_t var2)
{
   int64 tmp64;

   tmp64 = (int64) var1 * var2;

   tmp64 >>= 31;    //(var1 * var2) << 1
   tmp64 += mac32;

   mac32 = s32_saturate_s64(tmp64);

   return (mac32);
}
#endif // #if !defined(s64_mac_s32_s32)

#if !defined(s32_mac_s32_s32_s1_rnd_sat)
int32_t s32_mac_s32_s32_s1_rnd_sat(int32_t mac32, int32_t var1, int32_t var2)
{
   int64 tmp64;
   tmp64 = (int64) var1 * var2;
   tmp64 += (int64) (1<<30);
   tmp64 >>= 31;    //(var1 * var2) << 1
   tmp64 += mac32;
   mac32 = s32_saturate_s64(tmp64);
   return (mac32);
}
#endif // #if !defined(s32_mac_s32_s32_s1_rnd_sat)
#if !defined(s32_accu_asr_s32_imm5)
int32_t s32_accu_asr_s32_imm5(int32_t accu32, int32_t var, uint32_t u5)
{
   if (u5 < 32)
   {
      accu32 = accu32 + (var >> u5);
      return (accu32);
   }
   else
   {
      return accu32;
   }
}
#endif

#if !defined(s32_accu_asl_s32_imm5)
int32_t s32_accu_asl_s32_imm5(int32_t accu32, int32_t var, uint32_t u5)
{
   if (u5 < 32)
   {
      accu32 = accu32 + (var << u5);
   }
   else
   {
      if (var > 0)
      {
         accu32 = s32_saturate_s64((int64) accu32 + MAX_32);
      }
      else
      {
         accu32 = s32_saturate_s64((int64) accu32 + MIN_32);
      }
   }
   return (accu32);
}
#endif

#if !defined(s32_set_bit_s32_s32)
int32_t s32_set_bit_s32_s32(int32_t var1, int32_t ind)
{
  return var1|(1<<ind);
}
#endif

#if !defined(s32_clr_bit_s32_s32)
int32_t s32_clr_bit_s32_s32(int32_t var1, int32_t ind)
{
  return var1 & ~(1<<ind);
}
#endif

#if !defined(s64_shr_s64_imm5_rnd)
int64_t s64_shr_s64_imm5_rnd(int64_t var1, int16_t shift)
{
   int32 rnd;
   int64 sum, output;

   rnd = 1 << (shift - 1);
   sum = s64_add_s64_s32(var1, rnd);
   output = s64_shl_s64(sum, -shift);

   return (output);
}
#endif

#if !defined(u32_mult_u32_u32)
uint32_t u32_mult_u32_u32(uint32_t var1, uint32_t var2)
{
   uint32 prod32;
   uint64 prod64;

   prod64 = u64_mult_u32_u32(var1, var2);
   prod32 = (uint32) prod64;

   return (prod32);
}
#endif

#if !defined(s32_mult_s32_s32_left_shift_1)
int32_t s32_mult_s32_s32_left_shift_1(int32_t var1, int32_t var2)
{
   int32 output;
   int64 prod;

   prod = s64_mult_s32_s32(var1, var2);
   prod = s64_shl_s64(prod, 1);
   output = (int32) prod;

   return (output);

}
#endif
