/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "audio_basic_op.h"
#include "audio_complex_basic_op.h"
#include "fft_util.h"

Word32 bitrev( Word32 x, Word32 BITS ) {
   int i;
   Word32 y = 0;
   for ( i= 0; i<BITS; i++) {
      y = (y << 1)| (x & 1);
      x >>= 1;
   }
   return y;
}

int32 bitrev_s32( int32 x, int32 BITS ) {
   int i;
   int32 y = 0;
   for ( i= 0; i<BITS; i++) {
      y = (y << 1)| (x & 1);
      x >>= 1;
   }
   return y;
}


void ButterflyRadix2_c64( cint2x32 *x ) {

   cint2x32 a, b;

   //a =  c64_avrg_c64_c64_rnd(x[0], x[1]);
   //b = c64_avrg_c64_c64neg_rnd(x[0], x[1]);
   a =  c64_add_c64_c64_sat(x[0], x[1]);
   b =  c64_sub_c64_c64_sat(x[0], x[1]);

   x[0] = a;
   x[1] = b;
   return;
}


void ButterflyRadix4_c64( cint2x32 *x ) {

   cint2x32 a,b,c,d;

   /*a =  c64_avrg_c64_c64_rnd(x[0],x[1]); 
   b = c64_avrg_c64_c64neg_rnd(x[0],x[1]); 
   c =  c64_avrg_c64_c64_rnd(x[2],x[3]); 
   d = c64_avrg_c64_c64neg_rnd(x[2],x[3]); */
   a = c64_add_c64_c64_sat(x[0],x[1]);
   b = c64_sub_c64_c64_sat(x[0],x[1]);
   c = c64_add_c64_c64_sat(x[2],x[3]);
   d = c64_sub_c64_c64_sat(x[2],x[3]);

   // j*b
   b = c64_complex_s32_s32(s32_neg_s32_sat(s32_imag_c64(b)), s32_real_c64(b));

   /*x[0] =  c64_avrg_c64_c64_rnd(a,c);
   x[1] =  c64_avrg_c64_c64_rnd(b,d);
   x[2] = c64_avrg_c64_c64neg_rnd(a,c);
   x[3] = c64_avrg_c64_c64neg_rnd(b,d);*/
   x[0] = c64_add_c64_c64_sat(a,c);
   x[1] = c64_add_c64_c64_sat(b,d);
   x[2] = c64_sub_c64_c64_sat(a,c);
   x[3] = c64_sub_c64_c64_sat(b,d);
   return;
}

void ifftButterflyRadix2_c64 ( cint2x32 *x ) {
   
	cint2x32 a,b;
    a =  c64_avrg_c64_c64_rnd(x[0],x[1]);
    b = c64_avrg_c64_c64neg_rnd(x[0],x[1]);
   
    x[0] = a;
    x[1] = b;
    return;
}


void ifftButterflyRadix4_c64 ( cint2x32 *x ) {
   
	cint2x32 a,b,c,d;

    a =   c64_avrg_c64_c64_rnd(x[0],x[1]);
    b =  c64_avrg_c64_c64neg_rnd(x[0],x[1]);
    c =   c64_avrg_c64_c64_rnd(x[2],x[3]);
    d =  c64_avrg_c64_c64neg_rnd(x[2],x[3]);
   
    // -j*b
    b = c64_complex_s32_s32(s32_imag_c64(b), s32_neg_s32_sat(s32_real_c64(b)));

    x[0] =  c64_avrg_c64_c64_rnd(a,c);
    x[1] =  c64_avrg_c64_c64_rnd(b,d);
    x[2] = c64_avrg_c64_c64neg_rnd(a,c);
    x[3] = c64_avrg_c64_c64neg_rnd(b,d);
    return;
}


 
/***************************/
/*  Scaled version         */
/***************************/
void sButterflyRadix2_c32( cint2x16 *x ) {
   cint2x16 a,b;
   a =  c32_avrg_c32_c32_rnd(x[0],x[1]);
   b = c32_avrg_c32_c32neg_rnd(x[0],x[1]);
   
   x[0] = a;
   x[1] = b;
   return;
}

void sButterflyRadix4qv3_c32( cint2x16 *x ) {
   cint2x16 a,b,c,d;

   a =  c32_avrg_c32_c32_rnd(x[0],x[1]);
   b = c32_avrg_c32_c32neg_rnd(x[0],x[1]);
   c =  c32_avrg_c32_c32_rnd(x[2],x[3]);
   d = c32_avrg_c32_c32neg_rnd(x[2],x[3]);
   
   // j*b
   b = c32_complex_s16_s16(s16_neg_s16_sat(s16_imag_c32(b)), s16_real_c32(b));

   x[0] =  c32_avrg_c32_c32_rnd(a,c);
   x[1] =  c32_avrg_c32_c32_rnd(b,d);
   x[2] = c32_avrg_c32_c32neg_rnd(a,c);
   x[3] = c32_avrg_c32_c32neg_rnd(b,d);
   return;
}


void sButterflyRadix4_c32( cint2x16 *x ) {
   cint2x16 a,b,c,d;

   a =  c32_avrg_c32_c32_rnd(x[0],x[1]);
   b = c32_avrg_c32_c32neg_rnd(x[0],x[1]);
   c =  c32_avrg_c32_c32_rnd(x[2],x[3]);
   d = c32_avrg_c32_c32neg_rnd(x[2],x[3]);
   
   // j*d
   d = c32_complex_s16_s16(s16_neg_s16_sat(s16_imag_c32(d)), s16_real_c32(d));

   x[0] =  c32_avrg_c32_c32_rnd(a,c);
   x[1] = c32_avrg_c32_c32neg_rnd(b,d);
   x[2] = c32_avrg_c32_c32neg_rnd(a,c);
   x[3] =  c32_avrg_c32_c32_rnd(b,d);
   return;
}

/***************************/
/*  Non-scaled version     */
/***************************/
void ButterflyRadix2_c32( cint2x16 *x ) {
   cint2x16 a,b;
   a = c32_add_c32_c32_sat(x[0],x[1]);
   b = c32_sub_c32_c32_sat(x[0],x[1]);
   
   x[0] = a;
   x[1] = b;
   return;
}

void ButterflyRadix4qv3_c32( cint2x16 *x ) {
   cint2x16 a,b,c,d;

   a = c32_add_c32_c32_sat(x[0],x[1]);
   b = c32_sub_c32_c32_sat(x[0],x[1]);
   c = c32_add_c32_c32_sat(x[2],x[3]);
   d = c32_sub_c32_c32_sat(x[2],x[3]);

   // j*b   
   b = c32_complex_s16_s16(s16_neg_s16_sat(s16_imag_c32(b)), s16_real_c32(b));

   x[0] = c32_add_c32_c32_sat(a,c);
   x[1] = c32_add_c32_c32_sat(b,d);
   x[2] = c32_sub_c32_c32_sat(a,c);
   x[3] = c32_sub_c32_c32_sat(b,d);
   return;
}


void ButterflyRadix4_c32( cint2x16 *x ) {
   cint2x16 a,b,c,d;

   a = c32_add_c32_c32_sat(x[0],x[1]);
   b = c32_sub_c32_c32_sat(x[0],x[1]);
   c = c32_add_c32_c32_sat(x[2],x[3]);
   d = c32_sub_c32_c32_sat(x[2],x[3]);
   
   // j*d
   d = c32_complex_s16_s16(s16_neg_s16_sat(s16_imag_c32(d)), s16_real_c32(d));

   x[0] = c32_add_c32_c32_sat(a,c);
   x[1] = c32_sub_c32_c32_sat(b,d);
   x[2] = c32_sub_c32_c32_sat(a,c);
   x[3] = c32_add_c32_c32_sat(b,d);
   return;
}


/***************************/
/*     ifft                */
/***************************/
void ifftButterflyRadix2_c32( cint2x16 *x ) {
   cint2x16 a,b;
   a =  c32_avrg_c32_c32_rnd(x[0],x[1]);
   b = c32_avrg_c32_c32neg_rnd(x[0],x[1]);
   
   x[0] = a;
   x[1] = b;
   return;
}


void ifftButterflyRadix4qv3_c32( cint2x16 *x ) {
   cint2x16 a,b,c,d;

   a =  c32_avrg_c32_c32_rnd(x[0],x[1]);
   b = c32_avrg_c32_c32neg_rnd(x[0],x[1]);
   c =  c32_avrg_c32_c32_rnd(x[2],x[3]);
   d = c32_avrg_c32_c32neg_rnd(x[2],x[3]);
   
   // -j*b
   b = c32_complex_s16_s16(s16_imag_c32(b), s16_neg_s16_sat(s16_real_c32(b)));

   x[0] =  c32_avrg_c32_c32_rnd(a,c);
   x[1] =  c32_avrg_c32_c32_rnd(b,d);
   x[2] = c32_avrg_c32_c32neg_rnd(a,c);
   x[3] = c32_avrg_c32_c32neg_rnd(b,d);
   return;
}


void ifftButterflyRadix4_c32( cint2x16 *x ) {
   cint2x16 a,b,c,d;

   a =  c32_avrg_c32_c32_rnd(x[0],x[1]);
   b = c32_avrg_c32_c32neg_rnd(x[0],x[1]);
   c =  c32_avrg_c32_c32_rnd(x[2],x[3]);
   d = c32_avrg_c32_c32neg_rnd(x[2],x[3]);
   
   // -j*b
   d = c32_complex_s16_s16(s16_imag_c32(d), s16_neg_s16_sat(s16_real_c32(d)));

   x[0] =  c32_avrg_c32_c32_rnd(a,c);
   x[1] = c32_avrg_c32_c32neg_rnd(b,d);
   x[2] = c32_avrg_c32_c32neg_rnd(a,c);
   x[3] =  c32_avrg_c32_c32_rnd(b,d);
   return;
}

