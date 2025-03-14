/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "audio_fft_basic_ops.h"

Word32 bitrev( Word32 x, Word32 BITS );

int32 bitrev_s32( int32 x, int32 BITS );

void ButterflyRadix2_c64 ( cint2x32 *x );
void ButterflyRadix4_c64 ( cint2x32 *x );

void ifftButterflyRadix2_c64 ( cint2x32 *x );
void ifftButterflyRadix4_c64 ( cint2x32 *x );



void ButterflyRadix2_c32( cint2x16 *x );
void ButterflyRadix4qv3_c32( cint2x16 *x );
void ButterflyRadix4_c32( cint2x16 *x );

// scaled version 
void sButterflyRadix2_c32( cint2x16 *x );
void sButterflyRadix4_c32( cint2x16 *x );
void sButterflyRadix4qv3_c32( cint2x16 *x );

void ifftButterflyRadix2_c32( cint2x16 *x );
void ifftButterflyRadix4_c32( cint2x16 *x );
void ifftButterflyRadix4qv3_c32( cint2x16 *x );
