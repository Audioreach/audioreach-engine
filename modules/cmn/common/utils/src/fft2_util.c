/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*============================================================================

  FILE:          fft2_util.c

  OVERVIEW:      Implement helper functions for real-valued forward and

                 inverse FFT (QDSP6 reference).

  DEPENDENCIES:  None

============================================================================*/

#include "fft2_util.h"

#include "fft2_private_comdef.h"


#if !(defined(bitrev))
Word32 bitrev( Word32 x, Word32 BITS ) {

   int i;

   Word32 y = 0;

   for ( i= 0; i<BITS; i++) {

      y = (y << 1)| (x & 1);

      x >>= 1;

   }

   return y;

}
#endif
 

/*************************************/

/*  Define butterfly operations for  */

/*  FFT                              */

/*************************************/

void ButterflyRadix2Scaled ( CWord2x16 *x ) {

   CWord2x16 a,b;



   a =  cavg(x[0],x[1]);

   b = cnavg(x[0],x[1]);

   

   x[0] = a;

   x[1] = b;

   return;

}



void ButterflyRadix4Scaled ( CWord2x16 *x ) {

   CWord2x16 a,b,c,d;



   a =  cavg(x[0],x[1]);

   b = cnavg(x[0],x[1]);

   c =  cavg(x[2],x[3]);

   d = cnavg(x[2],x[3]);

   

   // j*b

   b = complex(negate(imag(b)), real(b));



   x[0] =  cavg(a,c);

   x[1] =  cavg(b,d);

   x[2] = cnavg(a,c);

   x[3] = cnavg(b,d);

   return;

}



void ifftButterflyRadix2 ( CWord2x16 *x ) {

   CWord2x16 a,b;

   a = cadd(x[0],x[1]);

   b = csub(x[0],x[1]);

   

   x[0] = a;

   x[1] = b;

   return;

}



void ifftButterflyRadix4 ( CWord2x16 *x ) {

   CWord2x16 a,b,c,d;



   a = cadd(x[0],x[1]);

   b = csub(x[0],x[1]);

   c = cadd(x[2],x[3]);

   d = csub(x[2],x[3]);

   

   // -j*b

   b = complex(imag(b), negate(real(b)));



   x[0] = cadd(a,c);

   x[1] = cadd(b,d);

   x[2] = csub(a,c);

   x[3] = csub(b,d);

   return;

}





/*************************************/

/*  32-bit FFT                       */

/*************************************/



/* 32x16 complex mpy with accumulate */

CWord2x32 L_mac_cmult32x16( CWord2x32 s, CWord2x32 x, CWord2x16 y)

{

   Word32 xRe, xIm; 

   Word16 yRe, yIm;

   Word32 zRe, zIm;

   Word32 sRe, sIm;



   xRe = L_real(x);  xIm = L_imag(x);

   yRe = real(y);    yIm = imag(y);

   sRe = L_real(s);  sIm = L_imag(s);

   

   // the complex MPY is defined in such a way in order

   // to facilitate faster ASM implementation on Q6

   zRe = L_add( sRe, L_mult32x16r( xRe, yRe ));

   zIm = L_add( sIm, L_mult32x16r( xIm, yRe ));



   zRe = L_add( zRe, L_mult32x16r( L_negate(xIm), yIm ) );



   // to match Q6 definition of 32x16 MAC

   if ((xRe == (Word32)0x80000000L) && (yIm == (Word16)0x8000))

       zIm = L_sat((Word64)zIm + 0x080000000LL);

   else 

       zIm = L_add( zIm, L_mult32x16r( xRe, yIm ) );



   return (L_complex(zRe, zIm));

}





/*

CWord2x32 L_cmult32x16( CWord2x32 x, CWord2x16 y)

{

   Word32 xRe, xIm; 

   Word16 yRe, yIm;

   Word32 zRe, zIm;



   xRe = L_real(x);  xIm = L_imag(x);

   yRe = real(y);    yIm = imag(y);

   

   // the complex MPY is defined in such a way in order

   // to facilitate faster ASM implementation on Q6

   zRe = L_mult32x16r( xRe, yRe );

   zIm = L_mult32x16r( xIm, yRe );



   zRe = L_add( zRe, L_mult32x16r( L_negate(xIm), yIm ) );



   // to match Q6 definition of 32x16 MAC

   if ((xRe == (Word32)0x80000000L) && (yIm == (Word16)0x8000))

       zIm = L_sat((Word64)zIm + 0x080000000LL);

   else 

       zIm = L_add( zIm, L_mult32x16r( xRe, yIm ) );



   return (L_complex(zRe, zIm));

}

*/



CWord2x32 L_cmult32x16( CWord2x32 x, CWord2x16 y)

{

   Word32 xRe, xIm;

   Word16 yRe, yIm;

   Word32 zRe, zIm;

   Word64 zRe64, zIm64;



   xRe = L_real(x);  xIm = L_imag(x);

   yRe = real(y);    yIm = imag(y);



   zRe64 = s64_sub_s64_s64(s64_mult_s32_s16(xRe,yRe),s64_mult_s32_s16(xIm,yIm));

   zIm64 = s64_add_s64_s64(s64_mult_s32_s16(xRe,yIm),s64_mult_s32_s16(xIm,yRe));



   //Shifting, Rounding, Saturating

   zRe = s32_saturate_s64(s64_shl_s64(s64_add_s64_s64(zRe64,0x4000),-15));

   zIm = s32_saturate_s64(s64_shl_s64(s64_add_s64_s64(zIm64,0x4000),-15));



   return (L_complex(zRe, zIm));

}







void ButterflyRadix2_32x16 ( CWord2x32 *x ) {



   CWord2x32 a, b;



   a = L_Vadd(x[0], x[1]);

   b = L_Vsub(x[0], x[1]);



   x[0] = a;

   x[1] = b;

   return;

}





void ButterflyRadix4_32x16 ( CWord2x32 *x ) {



   CWord2x32 a,b,c,d;



   a = L_Vadd(x[0],x[1]);

   b = L_Vsub(x[0],x[1]);

   c = L_Vadd(x[2],x[3]);

   d = L_Vsub(x[2],x[3]);



   // j*b

   b = L_complex(L_negate(L_imag(b)), L_real(b));



   x[0] = L_Vadd(a,c);

   x[1] = L_Vadd(b,d);

   x[2] = L_Vsub(a,c);

   x[3] = L_Vsub(b,d);

   return;

}



void ButterflyRadix2Scaled_32x16 ( CWord2x32 *x ) {



   CWord2x32 a, b;



   a =  L_Vavg(x[0], x[1]);

   b = L_Vnavg(x[0], x[1]);



   x[0] = a;

   x[1] = b;

   return;

}





void ButterflyRadix4Scaled_32x16 ( CWord2x32 *x ) {



   CWord2x32 a,b,c,d;



   a =  L_Vavg(x[0],x[1]);

   b = L_Vnavg(x[0],x[1]);

   c =  L_Vavg(x[2],x[3]);

   d = L_Vnavg(x[2],x[3]);



   // j*b

   b = L_complex(L_negate(L_imag(b)), L_real(b));



   x[0] =  L_Vavg(a,c);

   x[1] =  L_Vavg(b,d);

   x[2] = L_Vnavg(a,c);

   x[3] = L_Vnavg(b,d);

   return;

}





void ifftButterflyRadix2_32x16 ( CWord2x32 *x ) {



   CWord2x32 a, b;



   a = L_Vadd(x[0], x[1]);

   b = L_Vsub(x[0], x[1]);



   x[0] = a;

   x[1] = b;

   return;

}





void ifftButterflyRadix4_32x16 ( CWord2x32 *x ) {



   CWord2x32 a,b,c,d;



   a = L_Vadd(x[0],x[1]);

   b = L_Vsub(x[0],x[1]);

   c = L_Vadd(x[2],x[3]);

   d = L_Vsub(x[2],x[3]);



   // -j*b

   b = L_complex(L_imag(b), L_negate(L_real(b)));



   x[0] = L_Vadd(a,c);

   x[1] = L_Vadd(b,d);

   x[2] = L_Vsub(a,c);

   x[3] = L_Vsub(b,d);

   return;

}



void ifftButterflyRadix2Scaled_32x16 ( CWord2x32 *x ) {



   CWord2x32 a, b;



   a =  L_Vavg(x[0], x[1]);

   b = L_Vnavg(x[0], x[1]);



   x[0] = a;

   x[1] = b;

   return;

}





void ifftButterflyRadix4Scaled_32x16 ( CWord2x32 *x ) {



   CWord2x32 a,b,c,d;



   a =  L_Vavg(x[0],x[1]);

   b = L_Vnavg(x[0],x[1]);

   c =  L_Vavg(x[2],x[3]);

   d = L_Vnavg(x[2],x[3]);



   // -j*b

   b = L_complex(L_imag(b), L_negate(L_real(b)));





   x[0] =  L_Vavg(a,c);

   x[1] =  L_Vavg(b,d);

   x[2] = L_Vnavg(a,c);

   x[3] = L_Vnavg(b,d);

   return;

}

