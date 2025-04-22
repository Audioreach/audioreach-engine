/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*============================================================================
  FILE:          audio_complex_basic_op.c

  OVERVIEW:      Implement operations for complex numbers, which can be
                 mapped to QDSP6 intrinsics.

  DEPENDENCIES:  None
============================================================================*/

/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/

#include "audio_complex_basic_op.h"

/*----------------------------------------------------------------------------
 * Preprocessor Definitions and Constants
 * -------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 * Type Declarations
 * -------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 * Global Data Definitions
 * -------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 * Static Variable Definitions
 * -------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 * Static Function Declarations and Definitions
 * -------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 * Externalized Function Definitions
 * -------------------------------------------------------------------------*/

#if !defined(cl0)
/*======================================================================

  FUNCTION      cl0

  DESCRIPTION   Count leading zero bits.

  DEPENDENCIES  None

  PARAMETERS    x: [in] Input parameter

  RETURN VALUE  Number of leading zero bits (Q6_R_cl0_R(x))

  SIDE EFFECTS  None

======================================================================*/

int32 cl0(int32 x)
{
   int32 l0 = 0;
   if (x == 0)
   {
      l0 = 32;
   }
   else if (x > 0)
   {
      l0 = s16_norm_s32 ( x ) + 1;
   }
   return( l0 );
}

#endif

/*======================================================================

  FUNCTION      complex

  DESCRIPTION   Form complex number from real and imaginary part.
                16-bit LSW: real part
                16-bit MSW: imaginary part

  DEPENDENCIES  None

  PARAMETERS    xr: [in] Real part
                xi: [in] Imaginary part

  RETURN VALUE  Complex number (Q6_R_combine_RlRl(xi,xr))

  SIDE EFFECTS  None

======================================================================*/

cint2x16 c32_complex_s16_s16(int16 xr, int16 xi)
{
   cint2x16 c;
   c = ((int32)xr) & 0xFFFF;
   c |= (((int32)xi) << 16);
   return( c );
}

#if !defined(c32_conjugate_c32)
/*======================================================================

  FUNCTION      c32_conjugate_c32

  DESCRIPTION   Complex conjugate (with saturation)

  DEPENDENCIES  None

  PARAMETERS    x: [in] Complex input

  RETURN VALUE  Complex conjugate (Q6_P_vconj_P_sat(x))

  SIDE EFFECTS  None

======================================================================*/

cint2x16 c32_conjugate_c32(cint2x16 x)
{
   int16 xr = s16_real_c32(x);
   int16 xi = s16_imag_c32(x);
   return( c32_complex_s16_s16(xr, s16_neg_s16_sat(xi)) );
}
#endif


/*======================================================================

  FUNCTION      c64_mult_c32_c32

  DESCRIPTION   Fractionally multiply two complex numbers with
                16-bit real and imaginary parts, producing a
                product with 32-bit real and imaginary parts.

  DEPENDENCIES  None

  PARAMETERS    var1: [in] First complex number
                var2: [in] Second complex number

  RETURN VALUE  sat((var1 * var2)<<1) (Q6_P_cmpy_RR_s1_sat(var1,var2))

  SIDE EFFECTS  None

======================================================================*/

cint2x32 c64_mult_c32_c32(cint2x16 var1, cint2x16 var2)
{
   int16 var1r = s16_real_c32(var1);
   int16 var1i = s16_imag_c32(var1);
   int16 var2r = s16_real_c32(var2);
   int16 var2i = s16_imag_c32(var2);

   int32 xr = s32_shl_s32_sat( s32_sub_s32_s32_sat(s32_mult_s16_s16(var1r, var2r), s32_mult_s16_s16(var1i, var2i)), 1);
   int32 xi = s32_shl_s32_sat( s32_add_s32_s32_sat (s32_mult_s16_s16(var1r, var2i), s32_mult_s16_s16(var1i, var2r)), 1);
   return( c64_complex_s32_s32(xr, xi) );
}


/*======================================================================

  FUNCTION      c32_mult_c32_c32

  DESCRIPTION   Fractionally multiply two complex numbers with
                16-bit real and imaginary parts, producing a
                rounded product with 16-bit real and imaginary parts.

  DEPENDENCIES  None

  PARAMETERS    var1: [in] First complex number
                var2: [in] Second complex number

  RETURN VALUE  round( sat((var1 * var2)<<1) )
                (Q6_R_cmpy_RR_s1_rnd_sat(var1,var2))

  SIDE EFFECTS  None

======================================================================*/

cint2x16 c32_mult_c32_c32(cint2x16 var1, cint2x16 var2)
{
   int16 var1r = s16_real_c32(var1);
   int16 var1i = s16_imag_c32(var1);
   int16 var2r = s16_real_c32(var2);
   int16 var2i = s16_imag_c32(var2);

   int16 xr = s16_round_s32_sat(s32_shl_s32_sat( s32_sub_s32_s32_sat ( s32_mult_s16_s16(var1r, var2r), s32_mult_s16_s16(var1i, var2i) ), 1) );
   int16 xi = s16_round_s32_sat(s32_shl_s32_sat( s32_add_s32_s32_sat ( s32_mult_s16_s16(var1r, var2i), s32_mult_s16_s16(var1i, var2r) ), 1) );
   return( c32_complex_s16_s16(xr, xi) );
}


/*======================================================================

  FUNCTION      c32_mult_c32_c32conj

  DESCRIPTION   Fractionally multiply one complex number with
                the conjugate of another.  Each input has
                16-bit real and imaginary parts, and the product is
                rounded with 16-bit real and imaginary parts.

  DEPENDENCIES  None

  PARAMETERS    var1: [in] First complex number
                var2: [in] Second complex number

  RETURN VALUE  round( sat((var1 * conj(var2))<<1) )
                (Q6_R_cmpy_RR_conj_s1_rnd_sat(var1,var2))

  SIDE EFFECTS  None

======================================================================*/

cint2x16 c32_mult_c32_c32conj(cint2x16 var1, cint2x16 var2)
{
   return( c32_mult_c32_c32(var1, c32_conjugate_c32(var2)) );
}


/*======================================================================

  FUNCTION      c32_add_c32_c32_sat

  DESCRIPTION   Add two complex numbers, with saturation.

  DEPENDENCIES  None

  PARAMETERS    var1: [in] First complex number
                var2: [in] Second complex number

  RETURN VALUE  sat(var1 + var2)

  SIDE EFFECTS  None

======================================================================*/

cint2x16 c32_add_c32_c32_sat(cint2x16 var1, cint2x16 var2)
{
   int16 zr, zi;
   zr = s16_add_s16_s16_sat( s16_real_c32(var1), s16_real_c32(var2) );
   zi = s16_add_s16_s16_sat( s16_imag_c32(var1), s16_imag_c32(var2) );

   return( c32_complex_s16_s16((int16)zr, (int16)zi) );
}


/*======================================================================

  FUNCTION      c32_sub_c32_c32_sat

  DESCRIPTION   Subtract two complex numbers, with saturation.

  DEPENDENCIES  None

  PARAMETERS    var1: [in] First complex number
                var2: [in] Second complex number

  RETURN VALUE  sat(var1 - var2)

  SIDE EFFECTS  None

======================================================================*/

cint2x16 c32_sub_c32_c32_sat(cint2x16 var1, cint2x16 var2)
{
   int16 zr, zi;
   zr = s16_sub_s16_s16_sat( s16_real_c32(var1), s16_real_c32(var2) );
   zi = s16_sub_s16_s16_sat( s16_imag_c32(var1), s16_imag_c32(var2) );

   return( c32_complex_s16_s16((int16)zr, (int16)zi) );
}

#if !defined(c64_complex_s32_s32)
/*======================================================================

  FUNCTION      c64_complex_s32_s32

  DESCRIPTION   Form long complex number from real and imaginary part.
                32-bit LSW: real part
                32-bit MSW: imaginary part

  DEPENDENCIES  None

  PARAMETERS    xr: [in] Real part
                xi: [in] Imaginary part

  RETURN VALUE  Complex number (Q6_P_combine_RR(xi,xr))

  SIDE EFFECTS  None

======================================================================*/

cint2x32 c64_complex_s32_s32(int32 xr, int32 xi)
{
   cint2x32 c;
   c = ((int64)xr) & 0xFFFFFFFF;
   c |= (((int64)xi) << 32);
   return( c );
}
#endif

#if !defined(c64_conjugate_c64)
/*======================================================================

  FUNCTION      c64_conjugate_c64

  DESCRIPTION   Long complex conjugate (with saturation)

  DEPENDENCIES  None

  PARAMETERS    x: [in] Complex input

  RETURN VALUE  Complex conjugate

  SIDE EFFECTS  None

======================================================================*/

cint2x32 c64_conjugate_c64(cint2x32 x)
{
   return( c64_complex_s32_s32( s32_real_c64(x), s32_neg_s32_sat(s32_imag_c64(x)) ) );
}
#endif

#if !defined(c64_add_c64_c64_sat)
/*======================================================================

  FUNCTION      c64_add_c64_c64_sat

  DESCRIPTION   Add two long complex numbers, with saturation.

  DEPENDENCIES  None

  PARAMETERS    x: [in] First complex number
                y: [in] Second complex number

  RETURN VALUE  sat(x + y) (Q6_P_vaddw_PP_sat)

  SIDE EFFECTS  None

======================================================================*/

cint2x32 c64_add_c64_c64_sat(cint2x32 x, cint2x32 y)
{
   int32 xr, xi, yr, yi, sr, si;

   xr = s32_real_c64(x);
   xi = s32_imag_c64(x);
   yr = s32_real_c64(y);
   yi = s32_imag_c64(y);

   sr = s32_add_s32_s32_sat(xr, yr);
   si = s32_add_s32_s32_sat(xi, yi);

   return(c64_complex_s32_s32(sr, si));
}
#endif

#if !defined(c64_sub_c64_c64_sat)
/*======================================================================

  FUNCTION      c64_sub_c64_c64_sat

  DESCRIPTION   Subtract two long complex numbers, with saturation.

  DEPENDENCIES  None

  PARAMETERS    x: [in] First complex number
                y: [in] Second complex number

  RETURN VALUE  sat(x - y) (Q6_P_vsubw_PP_sat)

  SIDE EFFECTS  None

======================================================================*/

cint2x32 c64_sub_c64_c64_sat(cint2x32 x, cint2x32 y)
{

  int32 xr, xi, yr, yi, sr, si;

   xr = s32_real_c64(x);
   xi = s32_imag_c64(x);
   yr = s32_real_c64(y);
   yi = s32_imag_c64(y);

   sr = s32_sub_s32_s32_sat(xr, yr);
   si = s32_sub_s32_s32_sat(xi, yi);

   return(c64_complex_s32_s32(sr, si));

}
#endif

/*======================================================================

  FUNCTION      c32_avrg_c32_c32_rnd

  DESCRIPTION   Average two complex numbers with convergent rounding.

  DEPENDENCIES  None

  PARAMETERS    x: [in] First complex number
                y: [in] Second complex number

  RETURN VALUE  convround((x + y)>>1)
                (see Q6_P_vavgh_PP_crnd)

  SIDE EFFECTS  None

======================================================================*/

cint2x16 c32_avrg_c32_c32_rnd( cint2x16 x, cint2x16 y ) {
   int32 zr, zi;
   zr = (int32)s16_real_c32(x) + (int32)s16_real_c32(y);
   zi = (int32)s16_imag_c32(x) + (int32)s16_imag_c32(y);

   // convergent rouding
   zr +=  (zr>>1) & 1;
   zi +=  (zi>>1) & 1;
   //zr += 1;
   //zi += 1;

   zr >>= 1;
   zi >>= 1;

   return( c32_complex_s16_s16((int16)zr, (int16)zi) );
}


/*======================================================================

  FUNCTION      c32_avrg_c32_c32neg_rnd

  DESCRIPTION   Average a complex number with the negation of another,
                and apply convergent rounding.

  DEPENDENCIES  None

  PARAMETERS    x: [in] First complex number
                y: [in] Second complex number

  RETURN VALUE  convround((x - y)>>1)
                (see Q6_P_vnavgh_PP_crnd_sat)

  SIDE EFFECTS  None

======================================================================*/

cint2x16 c32_avrg_c32_c32neg_rnd( cint2x16 x, cint2x16 y ) {
   int32 zr, zi;
   zr = (int32)s16_real_c32(x) - (int32)s16_real_c32(y);
   zi = (int32)s16_imag_c32(x) - (int32)s16_imag_c32(y);

   // convergent rouding
   zr +=  (zr>>1) & 1;
   zi +=  (zi>>1) & 1;
   //zr += 1;
   //zi += 1;

   zr >>= 1;
   zi >>= 1;

   return( c32_complex_s16_s16(s16_saturate_s32(zr), s16_saturate_s32(zi)) );
}

#if !defined(c64_avrg_c64_c64_rnd)
/*======================================================================

  FUNCTION      c64_avrg_c64_c64_rnd

  DESCRIPTION   Average two long complex numbers with convergent
                rounding.

  DEPENDENCIES  None

  PARAMETERS    x: [in] First complex number
                y: [in] Second complex number

  RETURN VALUE  convround((x + y)>>1) (Q6_P_vavgw_PP_crnd)

  SIDE EFFECTS  None

======================================================================*/

cint2x32 c64_avrg_c64_c64_rnd(cint2x32 x, cint2x32 y)
{
    int64 t64;
    int32 xr, xi, yr, yi, sr, si;

    xr = s32_real_c64(x);
    xi = s32_imag_c64(x);
    yr = s32_real_c64(y);
    yi = s32_imag_c64(y);

    t64 = s64_add_s32_s32(xr, yr);
    // convergent rounding
    t64 = s64_add_s64_s64( t64, (int64)( ( ((int32)t64)>>1 ) & 1) );
    t64 = s64_shl_s64(t64,-1);
    sr = (int32) t64;

    t64 = s64_add_s32_s32(xi, yi);
    // convergent rounding
    t64 = s64_add_s64_s64( t64, (int64)( ( ((int32)t64)>>1 ) & 1) );
    t64 = s64_shl_s64(t64,-1);
    si = (int32) t64;

    return(c64_complex_s32_s32(sr, si));
}
#endif

#if !defined(c64_avrg_c64_c64neg_rnd)
/*======================================================================

  FUNCTION      c64_avrg_c64_c64neg_rnd

  DESCRIPTION   Average a long complex number with the negation of
                another, and apply convergent rounding.

  DEPENDENCIES  None

  PARAMETERS    x: [in] First complex number
                y: [in] Second complex number

  RETURN VALUE  convround((x - y)>>1)
                (Q6_P_vnavgw_PP_crnd_sat)

  SIDE EFFECTS  None

======================================================================*/

cint2x32 c64_avrg_c64_c64neg_rnd(cint2x32 x, cint2x32 y)
{
    int64 t64;
    int32 xr, xi, yr, yi, sr, si;

    xr = (int64) s32_real_c64(x);
    xi = (int64) s32_imag_c64(x);
    yr = (int64) s32_real_c64(y);
    yi = (int64) s32_imag_c64(y);

    t64 = s64_sub_s64_s64(xr, yr);
    // convergent rounding
    t64 = s64_add_s64_s64( t64, (int64)( ( ((int32)t64)>>1 ) & 1) );
    t64 = s64_shl_s64(t64,-1);
    sr = s32_saturate_s64(t64);

    t64 = s64_sub_s64_s64(xi, yi);
    // convergent rounding
    t64 = s64_add_s64_s64( t64, (int64)( ( ((int32)t64)>>1 ) & 1) );
    t64 = s64_shl_s64(t64,-1);
    si = s32_saturate_s64(t64);

    return(c64_complex_s32_s32(sr, si));

}
#endif


// NEW FUNCTION ADDED TO SUPPORT Quad mic ABF Processing
/*======================================================================

  FUNCTION      c32_mult_c32_s16_shift_sat

  DESCRIPTION   Multiply a complex number with 16-bit fractional real number
                and s16_round_s32_sat the result to complex number.

  DEPENDENCIES  None

  PARAMETERS    var1: [in] Input complex number
                var2: [in] Input Real number
                iqShiftL16: [in] Integer part of Qfactor of the real number

  RETURN VALUE  (var1*var2) << (iqshiftL16+1)

  SIDE EFFECTS  None

======================================================================*/

cint2x16 c32_mult_c32_s16_shift_sat( cint2x16 var1,
                                     int16 var2,
                                     int16 iqShiftL16)
{

   int32 accu32;
   int16 xr, xi;
   int16 var1r = s16_real_c32(var1);     // Real part of the complex number
   int16 var1i = s16_imag_c32(var1);     // Imaginary part of the complex number

   // Multiply with the real part
   accu32 = s32_shl_s32_sat(s32_mult_s16_s16_shift_sat(var1r, var2), iqShiftL16);    // Multiply and shift
   xr = s16_round_s32_sat(accu32);                                        // Round down to 16 bits

   // Multiply with the imaginary part
   accu32 = s32_shl_s32_sat(s32_mult_s16_s16_shift_sat(var1i, var2), iqShiftL16);    // Multiply and shift
   xi = s16_round_s32_sat(accu32);                                        // Round down to 16 bits

   return( c32_complex_s16_s16(xr, xi) );

}

/*======================================================================

  FUNCTION      c32_mult_c32_s32_shift_sat

  DESCRIPTION   Multiply a complex number with 32-bit fractional real number
                and round the result to complex number.

  DEPENDENCIES  None

  PARAMETERS    var1: [in] Input complex number
                var2: [in] Input Real number
                iqShiftL16: [in] Integer part of Qfactor of the real number

  RETURN VALUE  (var1*var2) << (iqshiftL16+1)

  SIDE EFFECTS  None

======================================================================*/

cint2x16 c32_mult_c32_s32_shift_sat( cint2x16 var1,
                                     int32 var2,
                                     int16 iqShiftL16)
{

   int32 accu32;
   int16 xr, xi;
   int16 var1r = s16_real_c32(var1);     // Real part of the complex number
   int16 var1i = s16_imag_c32(var1);     // Imaginary part of the complex number

   // Multiply with the real part
   accu32 = (int32)s32_mult_s32_s16_rnd_sat(var2, var1r);     // Multiply a 32-bit with 16-bit and round
   accu32 = s32_shl_s32_sat(accu32, iqShiftL16);     // Left shift and adjust the Q-factor
   xr = s16_round_s32_sat(accu32);                               // Round down to 16 bits

   // Multiply with the imaginary part
   accu32 = (int32)s32_mult_s32_s16_rnd_sat(var2, var1i);     // Multiply a 32-bit with 16-bit and round
   accu32 = s32_shl_s32_sat(accu32, iqShiftL16);     // Left shift and adjust the Q-factor
   xi = s16_round_s32_sat(accu32);                               // Round down to 16 bits

   return( c32_complex_s16_s16(xr, xi) );
}

/*======================================================================

  FUNCTION      c64_mult_c32_s16_sat

  DESCRIPTION   Multiply a 16-bit complex number with 16-bit fractional real number
                and return the 32-bit complex number

  DEPENDENCIES  None

  PARAMETERS    var1: [in] Input complex number
                var2: [in] Input Real number

  RETURN VALUE  (var1*var2) << 1

  SIDE EFFECTS  None

======================================================================*/

cint2x32 c64_mult_c32_s16_sat( cint2x16 var1,
                              int16 var2 )
{

   int32 xr32, xi32;
   int16 var1r = s16_real_c32(var1);     // Real part of the complex number
   int16 var1i = s16_imag_c32(var1);     // Imaginary part of the complex number

   // Multiply with the real part
   xr32 = s32_mult_s16_s16_shift_sat(var1r, var2);       // Multiply 16x16 and get 32-bit output

   // Multiply with the imaginary part
   xi32 = s32_mult_s16_s16_shift_sat(var1i, var2);       // Multiply 16x16 and get 32-bit output

   return( c64_complex_s32_s32(xr32, xi32) );
}

#if !defined(c64_mult_c64_s16_shift_sat)
/*======================================================================

  FUNCTION      c64_mult_c64_s16_shift_sat

  DESCRIPTION   Multiply a 64-bit complex number with 16-bit fractional real number
                and round the result to 64-bit complex number.

  DEPENDENCIES  None

  PARAMETERS    var1: [in] Input 64-bit complex number
                var2: [in] Input 16-bit Real number
                iqShiftL16: [in] Integer part of Qfactor of the real number

  RETURN VALUE  (var1*var2) << (iqshiftL16+1)

  SIDE EFFECTS  None

======================================================================*/

cint2x32 c64_mult_c64_s16_shift_sat( cint2x32 var1,
                          int16 var2,
                          int16 iqShiftL16)
{
   int32 xr32, xi32;
   int32 var1r = s32_real_c64(var1);           // Real part of the complex number
   int32 var1i = s32_imag_c64(var1);           // Imaginary part of the complex number

   // Multiply with the real part
   xr32 = (int32)s32_mult_s32_s16_rnd_sat(var1r, var2);     // Multiply a 32-bit with 16-bit and round
   xr32 = s32_shl_s32_sat(xr32, iqShiftL16);                 // Left shift and adjust the Q-factor

   // Multiply with the imaginary part
   xi32 = (int32)s32_mult_s32_s16_rnd_sat(var1i, var2);     // Multiply a 32-bit with 16-bit and round
   xi32 = s32_shl_s32_sat(xi32, iqShiftL16);                 // Left shift and adjust the Q-factor

   return( c64_complex_s32_s32(xr32, xi32) );
}
#endif

/*======================================================================

  FUNCTION      c64_mult_c64_c32_rnd_sat

  DESCRIPTION   Fractionally multiply 32-bit complex number with
                16-bit complex number, producing a
                product with 32-bit complex number

  DEPENDENCIES  None

  PARAMETERS    var1: [in] First complex number (32-bit)
                var2: [in] Second complex number (16-bit)

  RETURN VALUE  sat((var1 * var2)<<1)

  SIDE EFFECTS  None

======================================================================*/

cint2x32 c64_mult_c64_c32_rnd_sat(cint2x32 var1, cint2x16 var2)
{
   int32 var1r = s32_real_c64(var1);
   int32 var1i = s32_imag_c64(var1);
   int16 var2r = s16_real_c32(var2);
   int16 var2i = s16_imag_c32(var2);

   int32 xr = s32_sub_s32_s32_sat ( (int32 )s32_mult_s32_s16_rnd_sat(var1r, var2r),
                           (int32 )s32_mult_s32_s16_rnd_sat(var1i, var2i));

   int32 xi = s32_add_s32_s32_sat ( (int32 )s32_mult_s32_s16_rnd_sat(var1r, var2i),
                           (int32 )s32_mult_s32_s16_rnd_sat(var1i, var2r));
   return( c64_complex_s32_s32(xr, xi) );
}


/*======================================================================

  FUNCTION      c64_mult_c64_c32_rnd_sat_q6

  DESCRIPTION   Fractionally multiply 32-bit complex number with
                16-bit complex number, producing a
                product with 32-bit complex number

  DEPENDENCIES  None

  PARAMETERS    var1: [in] First complex number (32-bit)
                var2: [in] Second complex number (16-bit)

  RETURN VALUE  sat((var1 * var2)<<1)

  SIDE EFFECTS  None

======================================================================*/

cint2x32 c64_mult_c64_c32_rnd_sat_q6( cint2x32 x, cint2x16 y)
{
   int32 xRe, xIm;
   int16 yRe, yIm;
   int32 zRe, zIm;

   xRe = s32_real_c64(x);  xIm = s32_imag_c64(x);
   yRe = s16_real_c32(y);  yIm = s16_imag_c32(y);

   // the complex MPY is defined in such a way in order
   // to facilitate faster ASM implementation on Q6
   zRe = s32_mult_s32_s16_rnd_sat( xRe, yRe );
   zIm = s32_mult_s32_s16_rnd_sat( xIm, yRe );

   zRe = s32_add_s32_s32_sat( zRe, s32_mult_s32_s16_rnd_sat( s32_neg_s32_sat(xIm), yIm ) );

   // to match Q6 definition of 32x16 MAC
   if ((xRe == (int32)0x80000000L) && (yIm == -32768))
       zIm = s32_saturate_s64((int64)zIm + 0x080000000LL);
   else
       zIm = s32_add_s32_s32_sat( zIm, s32_mult_s32_s16_rnd_sat( xRe, yIm ) );

   return (c64_complex_s32_s32(zRe, zIm));
}

