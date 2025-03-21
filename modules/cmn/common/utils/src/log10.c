/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "audio_log10.h"
#include "audio_basic_op.h"

/* Input log table */
static const int16_t log2tbl[] = 
{
      0x7800, 0x782d, 0x785a, 0x7884, 0x78ae,
      0x78d6, 0x78fe, 0x7924, 0x794a, 0x796e,
      0x7992, 0x79b4, 0x79d6, 0x79f8, 0x7a18,
      0x7a38, 0x7a57, 0x7a75, 0x7a93, 0x7ab1,
      0x7acd, 0x7ae9, 0x7b05, 0x7b20, 0x7b3b,
      0x7b55, 0x7b6f, 0x7b88, 0x7ba1, 0x7bb9,
      0x7bd1, 0x7be9
};


/*==============================================================================

FUNCTION      int32_t log10_fixed(int32_t input)

DESCRIPTION   Calculates 10log10(x) using table look up for log2(input)
10*log10(input)=3.0103*log2(input) 

DEPENDENCIES  none

RETURN VALUE  Signed 32 bit value 10log10(x)

SIDE EFFECTS  none

===============================================================================*/
int32_t log10_fixed(int32_t input)
{

   int16_t tableOffset = 128; /* 2^7 for offset to table */
   int16_t constantTerm = 24660;  /* 3.013 in Q13 */
   int16_t shiftNorm;
   int32_t normalizedInput, intermediateTerm, exponent;
   int32_t termA, termB;
   int16_t index;   
   

   /* Normalize Input */
   shiftNorm = s16_norm_s32(input); /* Final normalization shift factor */
   normalizedInput = s32_shl_s32_sat(input, shiftNorm); /* normalize input */

   /* Calculate index for log2(input) */
   intermediateTerm = s32_mult_s16_s16(s16_extract_s32_h(normalizedInput), tableOffset); 
   index = s16_add_s16_s16(s16_extract_s32_h(intermediateTerm), -32); /* get index to log2tbl */

   /* get the exponent to subtract for q factor adjustment */
   exponent = s32_saturate_s40(s40_mult_s16_s16_shift(tableOffset, shiftNorm, 3)); /* - expon << 10 */

   /* get 3.0103*log2(input) and 3.0103*exponent in Q23 */
   termA = s32_mult_s16_s16(log2tbl[index], constantTerm); /* 3.0103*log2(x) Q23 */
   termB = s32_mult_s16_s16(constantTerm, s16_extract_s32_l(exponent)); /* 3.0103*expon Q23 */

   /* return the difference of termA and termB */
   return s32_sub_s32_s32(termA, termB);
}
