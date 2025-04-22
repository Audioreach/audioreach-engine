/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
/*===========================================================================
 * FILE NAME: SoftVolumeControls.cpp
 * DESCRIPTION:
 *    Volume and Soft stepping
 *===========================================================================*/

#ifdef AVS_BUILD_SOS
#include "capi_cmn.h"
#endif

#include "SoftVolumeControls.h"
#include "audio_basic_op.h"
#include "audio_divide_qx.h"
#include "audio_log10.h"
#include "audio_basic_op_ext.h"
#include <math.h>
#include "audio_exp10.h"
#include "apply_gain.h"
#include "audio_clips.h"
#include "string.h"
#include "posal.h"

#if ((defined __hexagon__) || (defined __qdsp6__))
#include <hexagon_protos.h>
#include <hexagon_types.h>
#include <q6sim_timer.h>
// #include "OmmUtils.h"
#endif

// Use to override AR_MSG with AR_MSG_ISLAND. Always include this after ar_msg.h
#ifdef AR_MSG_IN_ISLAND
#include "ar_msg_island_override.h"
#endif

static uint32_t pow_1_75(uint32_t x);

#if ((defined __hexagon__) || (defined __qdsp6__))
extern "C" {
void SoftVolumeControlsExp2Fixed_asm(int32 input1L32Q26, int32 input2L32Q26, int32 *y1L32Q14, int32 *y2L32Q14);
}
#endif

static const int16 log2table[] = { 0x7800, 0x782d, 0x785a, 0x7884, 0x78ae, 0x78d6, 0x78fe, 0x7924,
                                   0x794a, 0x796e, 0x7992, 0x79b4, 0x79d6, 0x79f8, 0x7a18, 0x7a38,
                                   0x7a57, 0x7a75, 0x7a93, 0x7ab1, 0x7acd, 0x7ae9, 0x7b05, 0x7b20,
                                   0x7b3b, 0x7b55, 0x7b6f, 0x7b88, 0x7ba1, 0x7bb9, 0x7bd1, 0x7be9 };

static const int32  THRESHOLD_NOT_DETECTED = -1;
extern "C" {
const int32 fractionalPowersOf2L32Q16[] = {
   65536,  // 2^0
   77936,  // 2^0.25
   92682,  // 2^0.5
   110218, // 2^0.75
};
} // extern C
static const uint32 UNITY_L32_Q28 = 1 << 28;

#if ((defined __hexagon__) || (defined __qdsp6__))
static boolean isAlignedTo8Byte(void *ptr)
{
   uint32 val = reinterpret_cast<uint32>(ptr);
   if (0 == (val & 0x7))
   {
      return 1;
   }
   else
   {
      return 0;
   }
}

static void ApplySteadyGainLessThanOne16(void *pOutPtr, void *pInPtr, const uint32 gainQ28, uint32 samples)
{
   int16 *pInput  = (int16 *)(pInPtr);
   int16 *pOutput = (int16 *)(pOutPtr);

   while (!isAlignedTo8Byte(pOutput) && (samples > 0))
   {
      int64 input   = *pInput++;
      int64 product = input * gainQ28;
      product       = s64_shr_s64_imm5_rnd(product, 28);
      *pOutput++    = s16_saturate_s32(product);
      samples--;
   }

   union reg64bit
   {
      uint64 full;
      struct
      {
         uint32 l;
         uint32 h;
      };
   };

   reg64bit gain;
   gain.l = gain.h = gainQ28;

   uint32  count               = samples / 8;
   uint32  remaining           = samples % 8;
   uint64 *pOutput8ByteAligned = (uint64 *)(pOutput);
   int64   round               = 0x0000080000000800;

   for (uint32 i = 0; i < count; i++)
   {
      reg64bit input_1, input_2, input_3, input_4, prod_1, prod_2, prod_3, prod_4;

      input_1.l = *pInput++;
      input_1.h = *pInput++;
      input_2.l = *pInput++;
      input_2.h = *pInput++;

      input_3.l = *pInput++;
      input_3.h = *pInput++;
      input_4.l = *pInput++;
      input_4.h = *pInput++;

      prod_1.full = Q6_P_vmpywehacc_PP_sat(round, gain.full, input_1.full);
      prod_2.full = Q6_P_vmpywehacc_PP_sat(round, gain.full, input_2.full);
      prod_3.full = Q6_P_vmpywehacc_PP_sat(round, gain.full, input_3.full);
      prod_4.full = Q6_P_vmpywehacc_PP_sat(round, gain.full, input_4.full);

      reg64bit output_1, output_2;
      output_1.l = Q6_R_vasrw_PI(prod_1.full, 28 - 16);
      output_1.h = Q6_R_vasrw_PI(prod_2.full, 28 - 16);
      output_2.l = Q6_R_vasrw_PI(prod_3.full, 28 - 16);
      output_2.h = Q6_R_vasrw_PI(prod_4.full, 28 - 16);

      *pOutput8ByteAligned++ = output_1.full;
      *pOutput8ByteAligned++ = output_2.full;
   }
   pOutput += (count * 8);

   for (uint32 i = 0; i < remaining; i++)
   {
      int64 input   = *pInput++;
      int64 product = input * gainQ28;
      product       = s64_shr_s64_imm5_rnd(product, 28);
      *pOutput++    = s16_saturate_s32(product);
   }
}
#endif

static void ApplyLinearRamp16bitStepSize1(void *pOutPtr, void *pInPtr, SvpannerStruct *panner, uint32 samples)
{
   int64 currentGainL64Q59_1 = panner->coeffs.linear.currentGainL64Q59;
   int64 currentGainL64Q59_2 = panner->coeffs.linear.currentGainL64Q59;

   uint32 currentGainL32Q28_1 = panner->currentGainL32Q28;
   uint32 currentGainL32Q28_2 = panner->currentGainL32Q28;
   uint32 targetgainL32Q28    = panner->targetgainL32Q28;
   int64  deltaGainL64Q59     = panner->coeffs.linear.deltaL64Q59;

   int16 *pInput  = (int16 *)(pInPtr);
   int16 *pOutput = (int16 *)(pOutPtr);

   uint32 count = samples - 1;

   if (panner->newGainL32Q28 < (uint32)0x80000000)
   {
      // unroll the loop by 2
      while (count >= 2)
      {
         currentGainL64Q59_1 = currentGainL64Q59_2 + deltaGainL64Q59;
         currentGainL64Q59_2 = currentGainL64Q59_1 + deltaGainL64Q59;

         int32 input  = *pInput++;
         int32 input2 = *pInput++;
         /* The value of currentgain for the next iteration*/
         currentGainL32Q28_2 = currentGainL64Q59_1 >> (59 - 28);

         int64 product, product2;

         {
            product  = s64_mult_s32_s32(input, currentGainL32Q28_1);
            product2 = s64_mult_s32_s32(input2, currentGainL32Q28_2);
         }

         product  = s64_shr_s64_imm5_rnd(product, 28);
         product2 = s64_shr_s64_imm5_rnd(product2, 28);

         *pOutput++ = s16_saturate_s32(product);
         *pOutput++ = s16_saturate_s32(product2);

         currentGainL32Q28_1 = currentGainL64Q59_2 >> (59 - 28);

         count -= 2;
      } // while loop
   }
   else
   {
      // unroll the loop by 2
      while (count >= 2)
      {
         currentGainL64Q59_1 = currentGainL64Q59_2 + deltaGainL64Q59;
         currentGainL64Q59_2 = currentGainL64Q59_1 + deltaGainL64Q59;

         int32 input  = *pInput++;
         int32 input2 = *pInput++;
         /* The value of currentgain for the next iteration*/
         currentGainL32Q28_2 = currentGainL64Q59_1 >> (59 - 28);

         int64 product, product2;

         {
            product  = (int64)input * currentGainL32Q28_1;
            product2 = (int64)input2 * currentGainL32Q28_2;
         }

         product  = s64_shr_s64_imm5_rnd(product, 28);
         product2 = s64_shr_s64_imm5_rnd(product2, 28);

         *pOutput++ = s16_saturate_s32(product);
         *pOutput++ = s16_saturate_s32(product2);

         currentGainL32Q28_1 = currentGainL64Q59_2 >> (59 - 28);

         count -= 2;
      } // while loop
   }

   count++;

   // process remaining samples
   while (count--)
   {
      currentGainL64Q59_2 += deltaGainL64Q59;
      if (currentGainL64Q59_2 < 0)
      {
         currentGainL64Q59_2 = targetgainL32Q28;
      }

      int32 input = *pInput++;
      int64 product;

      if ((currentGainL32Q28_1 < (uint32)0x80000000))
      {
         product = s64_mult_s32_s32(input, currentGainL32Q28_1);
      }
      else
      {
         product = (int64)input * currentGainL32Q28_1;
      }

      product    = s64_shr_s64_imm5_rnd(product, 28);
      *pOutput++ = s16_saturate_s32(product);

      /* The value of currentgain for the next iteration*/
      currentGainL32Q28_1 = currentGainL64Q59_2 >> (59 - 28);
   } // for loop

   panner->currentGainL32Q28               = currentGainL32Q28_1;
   panner->coeffs.linear.currentGainL64Q59 = currentGainL64Q59_2;
}

static void ApplyLinearRamp32bitStepSize1(void *pOutPtr, void *pInPtr, SvpannerStruct *panner, uint32 samples)
{
   int64 currentGainL64Q59_1 = panner->coeffs.linear.currentGainL64Q59;
   int64 currentGainL64Q59_2 = panner->coeffs.linear.currentGainL64Q59;

   uint32 currentGainL32Q28_1 = panner->currentGainL32Q28;
   uint32 currentGainL32Q28_2 = panner->currentGainL32Q28;

   int64  deltaGainL64Q59  = panner->coeffs.linear.deltaL64Q59;
   uint32 targetgainL32Q28 = panner->targetgainL32Q28;
   int32 *pInput           = (int32 *)(pInPtr);
   int32 *pOutput          = (int32 *)(pOutPtr);

   uint32 count = samples - 1; // count will be incremented next

   if (panner->newGainL32Q28 < (uint32)0x80000000)
   {
      // unroll the loop by 2
      while (count >= 2)
      {
         currentGainL64Q59_1 = currentGainL64Q59_2 + deltaGainL64Q59;
         currentGainL64Q59_2 = currentGainL64Q59_1 + deltaGainL64Q59;

         int32 input  = *pInput++;
         int32 input2 = *pInput++;
         /* The value of currentgain for the next iteration*/
         currentGainL32Q28_2 = currentGainL64Q59_1 >> (59 - 28);
         int64 product;
         int64 product2;

         {
            product  = s64_mult_s32_s32(input, currentGainL32Q28_1);
            product2 = s64_mult_s32_s32(input2, currentGainL32Q28_2);
         }

         product  = s64_shr_s64_imm5_rnd(product, 28);
         product2 = s64_shr_s64_imm5_rnd(product2, 28);

         *pOutput++ = s32_saturate_s64(product);
         *pOutput++ = s32_saturate_s64(product2);

         currentGainL32Q28_1 = currentGainL64Q59_2 >> (59 - 28);

         count -= 2;
      } // for loop
   }
   else
   {
      // unroll the loop by 2
      while (count >= 2)
      {
         currentGainL64Q59_1 = currentGainL64Q59_2 + deltaGainL64Q59;
         currentGainL64Q59_2 = currentGainL64Q59_1 + deltaGainL64Q59;

         int32 input  = *pInput++;
         int32 input2 = *pInput++;
         /* The value of currentgain for the next iteration*/
         currentGainL32Q28_2 = currentGainL64Q59_1 >> (59 - 28);
         int64 product;
         int64 product2;

         {
            product  = (int64)input * currentGainL32Q28_1;
            product2 = (int64)input2 * currentGainL32Q28_2;
         }

         product  = s64_shr_s64_imm5_rnd(product, 28);
         product2 = s64_shr_s64_imm5_rnd(product2, 28);

         *pOutput++ = s32_saturate_s64(product);
         *pOutput++ = s32_saturate_s64(product2);

         currentGainL32Q28_1 = currentGainL64Q59_2 >> (59 - 28);

         count -= 2;
      } // for loop
   }

   count++;

   // process remaining samples
   while (count--)
   {
      currentGainL64Q59_2 += deltaGainL64Q59;
      if (currentGainL64Q59_2 < 0)
      {
         currentGainL64Q59_2 = targetgainL32Q28;
      }

      int32 input = *pInput++;
      int64 product;

      if ((currentGainL32Q28_1 < (uint32)0x80000000))
      {
         product = s64_mult_s32_s32(input, currentGainL32Q28_1);
      }
      else
      {

         product = (int64)input * currentGainL32Q28_1;
      }

      product    = s64_shr_s64_imm5_rnd(product, 28);
      *pOutput++ = s32_saturate_s64(product);

      /* The value of currentgain for the next iteration*/
      currentGainL32Q28_1 = currentGainL64Q59_2 >> (59 - 28);
   } // for loop

   panner->currentGainL32Q28               = currentGainL32Q28_1;
   panner->coeffs.linear.currentGainL64Q59 = currentGainL64Q59_2;
}

/*=============================================================================
FUNCTION      audpp_sftvlmctrl_logbase2_fixed

DESCRIPTION   Calculates log2(input). Works only for positive numbers.

INPUTS
input: input in integer format.

OUTPUTS

DEPENDENCIES  None

RETURN VALUE
log2(input), in Q10 format

SIDE EFFECTS
===============================================================================*/
static int32 SoftVolumeControlsLogBase2Fixed(int32 input)
{
   int16 shiftNorm;
   int32 normalizedInput, exponentL32Q10, logNormInputL32Q10, logInputL32Q10;

   /* Normalize Input */
   shiftNorm       = s16_norm_s32(input);               /* Final normalization shift factor */
   normalizedInput = s32_shl_s32_sat(input, shiftNorm); /* normalize input */

   /* Calculate log2(normalizedInput) */
   {
      int16 intermediateTerm, index;
      intermediateTerm   = normalizedInput >> (31 - 6); // This will give a number in the range (32,63)
      index              = intermediateTerm - 32;       // This will give a number in the range (0,31)
      logNormInputL32Q10 = log2table[index];
   }

   /* get the exponent to subtract for normalization adjustment */
   exponentL32Q10 = shiftNorm << 10;

   /* Adjust for normalization */
   logInputL32Q10 = logNormInputL32Q10 - exponentL32Q10;

   return logInputL32Q10;
}

static void SoftVolumeControlsLogBase2Fixed(int32 input1, int32 input2, int32 *output1, int32 *output2)
{
   int16 shiftNorm1, shiftNorm2;
   int32 normalizedInput_1, exponentL32Q10_1, logNormInputL32Q10_1;
   int32 normalizedInput_2, exponentL32Q10_2, logNormInputL32Q10_2;

   /* Normalize Input */
   shiftNorm1        = s16_norm_s32(input1);                /* Final normalization shift factor */
   shiftNorm2        = s16_norm_s32(input2);                /* Final normalization shift factor */
   normalizedInput_1 = s32_shl_s32_sat(input1, shiftNorm1); /* normalize input */
   normalizedInput_2 = s32_shl_s32_sat(input2, shiftNorm2); /* normalize input */

   /* Calculate log2(normalizedInput) */
   {
      int16 intermediateTerm_1, index_1;
      int16 intermediateTerm_2, index_2;
      intermediateTerm_1   = normalizedInput_1 >> (31 - 6); // This will give a number in the range (32,63)
      intermediateTerm_2   = normalizedInput_2 >> (31 - 6); // This will give a number in the range (32,63)
      index_1              = intermediateTerm_1 - 32;       // This will give a number in the range (0,31)
      index_2              = intermediateTerm_2 - 32;       // This will give a number in the range (0,31)
      logNormInputL32Q10_1 = log2table[index_1];
      logNormInputL32Q10_2 = log2table[index_2];
   }

   /* get the exponent to subtract for normalization adjustment */
   exponentL32Q10_1 = shiftNorm1 << 10;
   exponentL32Q10_2 = shiftNorm2 << 10;
   /* Adjust for normalization */
   *output1 = logNormInputL32Q10_1 - exponentL32Q10_1;
   *output2 = logNormInputL32Q10_2 - exponentL32Q10_2;
}

/*=============================================================================
FUNCTION      audpp_sftvlmctrl_exp2_fixed

DESCRIPTION   Calculates 2^input. Works only for positive numbers.

INPUTS
inputL32Q26: input in Q26 format. Input must be in the range (0,32)

OUTPUTS

DEPENDENCIES  None

RETURN VALUE
2^input, in Q14 format

SIDE EFFECTS
===============================================================================*/
int32 SoftVolumeControlsExp2FixedSingle(int32 inputL32Q26)
{
   // Calculate 2^x for the lower 24 bits
   //   int64 yL64Q26 = 0;

   /*
   Uses the Taylor series expansion around 0.125 to calculate
   2^x where 0 <= x < 0.25

   2^x ~= 2^0.125 + 2^0.125*ln(2)*(x-0.125) + 2^0.125*(ln(2))^2*(x-0.125)^2
   */

   const int32 A1L32Q26 = 67116628;
   const int32 B1L32Q16 = 45245;
   const int32 C1L32Q16 = 17168;
   // x = Lower 24 bits of the input.
   const uint32 mask_0_25 = ((1 << 24) - 1);
   int32        xL32Q26   = (inputL32Q26 & mask_0_25);

   //   A1 + B1x + C1x^2
   //    int64 product64 = A1L32Q26;

   int64 productL64Q42_1 = s64_mult_s32_s32(B1L32Q16, xL32Q26);

   //     int64 productL64Q42_1 = B1L32Q16 * xL64Q26;
   int64 x2L64Q26 = s64_mult_s32_s32(xL32Q26, xL32Q26) >> 26;
   //     int64 x2L64Q26 = (xL64Q26 * xL64Q26)>>26;

   int64 productL64Q42_2 = (x2L64Q26 * C1L32Q16);
   //     int64 productL64Q42_2 = s64_mult_s32_s32(x2L32Q26 , C1L32Q16);

   int64 yL64Q26 = A1L32Q26 + ((productL64Q42_1 + productL64Q42_2) >> 16);

   // Now handle the next 2 bits

   const uint32 mask_frac_Q2 = ((1 << 2) - 1);

   // x = Fractional part of the input in Q2
   int32 xL32Q2 = (inputL32Q26 >> (26 - 2)) & mask_frac_Q2;

   // y = y * x
   yL64Q26 = ((yL64Q26 * fractionalPowersOf2L32Q16[xL32Q2]) >> 16);

   // Now handle the integer part, and convert to Q14
   int32 yL32Q14 = 0;

   // y = y * 2^(integer part of the input)
   int32 shiftVal = 26 - 14 - (inputL32Q26 >> 26); // 26 - 14 is to convert to Q14
   yL32Q14        = (yL64Q26 >> shiftVal);

   return yL32Q14;
}

void SoftVolumeControlsExp2Fixed(int32 input1L32Q26, int32 input2L32Q26, int32 *y1L32Q14, int32 *y2L32Q14)
{
   // Calculate 2^x for the lower 24 bits

   /*
   Uses the Taylor series expansion around 0.125 to calculate
   2^x where 0 <= x < 0.25

   2^x ~= 2^0.125 + 2^0.125*ln(2)*(x-0.125) + 2^0.125*(ln(2))^2*(x-0.125)^2
   */

   const int32 A1L32Q26 = 67116628;
   const int32 B1L32Q16 = 45245;
   const int32 C1L32Q16 = 17168;

   // x = Lower 24 bits of the input.
   const uint32 mask_0_25 = ((1 << 24) - 1);

   int32 x1L32Q26 = (input1L32Q26 & mask_0_25);
   int32 x2L32Q26 = (input2L32Q26 & mask_0_25);

   int64 product1L64Q42_1 = s64_mult_s32_s32(B1L32Q16, x1L32Q26);
   int64 product1L64Q42_2 = s64_mult_s32_s32(B1L32Q16, x2L32Q26);

   int64 x2L64Q26_1 = s64_mult_s32_s32(x1L32Q26, x1L32Q26) >> 26;
   int64 x2L64Q26_2 = s64_mult_s32_s32(x2L32Q26, x2L32Q26) >> 26;

   int64 product2L64Q42_1 = (x2L64Q26_1 * C1L32Q16);
   int64 product2L64Q42_2 = (x2L64Q26_2 * C1L32Q16);

   int64 y1L64Q26 = A1L32Q26 + ((product1L64Q42_1 + product2L64Q42_1) >> 16);
   int64 y2L64Q26 = A1L32Q26 + ((product1L64Q42_2 + product2L64Q42_2) >> 16);

   // Now handle the next 2 bits

   const uint32 mask_frac_Q2 = ((1 << 2) - 1);

   // x = Fractional part of the input in Q2
   int32       x1L32Q2                     = (input1L32Q26 >> (26 - 2)) & mask_frac_Q2;
   int32       x2L32Q2                     = (input2L32Q26 >> (26 - 2)) & mask_frac_Q2;
   const int32 fractionalPowersOf2L32Q16[] = {
      65536,  // 2^0
      77936,  // 2^0.25
      92682,  // 2^0.5
      110218, // 2^0.75
   };

   // y = y * x
   y1L64Q26 = ((y1L64Q26 * fractionalPowersOf2L32Q16[x1L32Q2]) >> 16);
   y2L64Q26 = ((y2L64Q26 * fractionalPowersOf2L32Q16[x2L32Q2]) >> 16);

   // Now handle the integer part, and convert to Q14

   // y = y * 2^(integer part of the input)
   int32 shiftVal1 = 26 - 14 - (input1L32Q26 >> 26); // 26 - 14 is to convert to Q14
   int32 shiftVal2 = 26 - 14 - (input2L32Q26 >> 26); // 26 - 14 is to convert to Q14
   *y1L32Q14       = (y1L64Q26 >> shiftVal1);
   *y2L32Q14       = (y2L64Q26 >> shiftVal2);

   return;
}

/*=============================================================================
FUNCTION      CSoftVolumeControlsLib::apply_linear_ramp

DESCRIPTION   increase gain(Q12) in a linear curve and apply to the input(Q15) and
save to output (Q15)
OUTPUTS

DEPENDENCIES  None

RETURN VALUE  None

SIDE EFFECTS
===============================================================================*/
void CSoftVolumeControlsLib::ApplyLinearRamp(void *pOutPtr, void *pInPtr, SvpannerStruct *panner, uint32 samples)
{
   uint32 temp, deltaStep;
   int64  currentGainL64Q59 = panner->coeffs.linear.currentGainL64Q59;
   uint32 currentGainL32Q28 = panner->currentGainL32Q28;
   uint32 i;
   int64  deltaGainL64Q59   = panner->coeffs.linear.deltaL64Q59;
   uint32 pannerStep        = panner->step;
   uint32 pannerStepResidue = panner->stepResidue;
   uint32 pannerIndex       = panner->index;

   if (panner->step <= 1)
   {
      if (m_bytesPerSample == 2)
      {
         ApplyLinearRamp16bitStepSize1(pOutPtr, pInPtr, panner, samples);
      }    // if
      else // (m_bytesPerSample == 4)
      {
         ApplyLinearRamp32bitStepSize1(pOutPtr, pInPtr, panner, samples);
      }
   }
   else // (step > 1)
   {
      for (i = samples; i > 0;)
      {
         /* deltastep is the number of samples for which the current value of gain is to be applied*/
         temp        = pannerStep - pannerStepResidue;
         deltaStep   = (i < temp) ? i : temp;
         pannerIndex = pannerIndex + deltaStep;

         /* apply the gain to deltastep number of samples*/
         ApplySteadyGain(pOutPtr, pInPtr, currentGainL32Q28, deltaStep);
         IncrementPointer(&pInPtr, deltaStep);
         IncrementPointer(&pOutPtr, deltaStep);

         /* number of samples left in the current frame */
         i -= deltaStep;

         if (deltaStep < temp) /* not enough samples left in the current frame apply gain to step size of samples*/
         {
            pannerStepResidue = pannerStepResidue + deltaStep;
         }
         else /* enough samples left in the current frame apply gain to step size of samples*/
         {
            pannerStepResidue = 0;
            currentGainL64Q59 += deltaGainL64Q59;
            if (currentGainL64Q59 < 0)
            {
               currentGainL64Q59 = 0;
            }
            currentGainL32Q28 = currentGainL64Q59 >> (59 - 28);
         }
      }
      panner->currentGainL32Q28               = currentGainL32Q28;
      panner->coeffs.linear.currentGainL64Q59 = currentGainL64Q59;
      panner->step                            = pannerStep;
      panner->stepResidue                     = pannerStepResidue;
      panner->index                           = pannerIndex;
   }
}

static void ApplyLogRamp16bitStep1(void *pOutPtr, void *pInPtr, SvpannerStruct *panner, uint32 samples)
{
   int32  AL32Q26           = panner->coeffs.log.AL32Q26;
   int32  BL32Q26           = panner->coeffs.log.BL32Q26;
   int32  CL32Q16_1         = panner->coeffs.log.CL32Q16;
   int32  CL32Q16_2         = panner->coeffs.log.CL32Q16;
   int32  deltaCL32Q16      = panner->coeffs.log.deltaCL32Q16;
   uint32 step              = panner->step;
   uint32 stepResidue       = panner->stepResidue;
   uint32 currentGainL32Q28 = panner->currentGainL32Q28;

   int16 *pInput  = (int16 *)(pInPtr);
   int16 *pOutput = (int16 *)(pOutPtr);

   if (0 != stepResidue)
   {

      int32 input = *pInput++;
      int64 product;

      // mpy works for both signed or unsigned, so as a work around when gain >= 0x80000000 we are using
      // standard multiplication
      if ((currentGainL32Q28 < (uint32)0x80000000))
      {
         product = s64_mult_s32_s32(input, currentGainL32Q28);
      }
      else
      {

         product = (int64)input * currentGainL32Q28;
      }

      product    = s64_shr_s64_imm5_rnd(product, 28);
      *pOutput++ = s16_saturate_s32(product);

      stepResidue = 0;
      samples--;
   }

   if (panner->newGainL32Q28 < (uint32)0x80000000)
   {
      // loop unroll by 2
      while (samples >= 2)
      {

         // Calculate the new gain
         CL32Q16_1 = CL32Q16_2 + deltaCL32Q16;
         CL32Q16_2 = CL32Q16_1 + deltaCL32Q16;

         int32 logL32Q10_1, logL32Q10_2;
         SoftVolumeControlsLogBase2Fixed(CL32Q16_1, CL32Q16_2, &logL32Q10_1, &logL32Q10_2);

         // Since the log function returns the log assuming integer argument,
         // need to adjust for the input being Q16
         logL32Q10_1 -= (16 << 10);
         logL32Q10_2 -= (16 << 10);

         int32 logL32Q14_1 = logL32Q10_1 << (14 - 10); // The range will be within (0,5), so no overflow will happen
         int32 logL32Q14_2 = logL32Q10_2 << (14 - 10); // The range will be within (0,5), so no overflow will happen

         int64 productL64Q40_1 = s64_mult_s32_s32(AL32Q26, logL32Q14_1); // both are signed
         int64 productL64Q40_2 = s64_mult_s32_s32(AL32Q26, logL32Q14_2);

         int64 productL64Q26_1 = (productL64Q40_1 >> (40 - 26));
         int64 productL64Q26_2 = (productL64Q40_2 >> (40 - 26));

         int32 productL32Q26_1 = s32_saturate_s64(productL64Q26_1);
         int32 productL32Q26_2 = s32_saturate_s64(productL64Q26_2);

         int32 gainL32Q26_1 = BL32Q26 + productL32Q26_1;

         if (gainL32Q26_1 > 0)
         {
            currentGainL32Q28 = gainL32Q26_1 << (28 - 26);
         }
         else
         {
            currentGainL32Q28 = 0;
         }

         int32 gainL32Q26_2 = BL32Q26 + productL32Q26_2;
         int32 input1       = *pInput++;
         int32 input2       = *pInput++;
         int64 product1;
         if ((currentGainL32Q28 < (uint32)0x80000000))
         {
            product1 = s64_mult_s32_s32(input1, currentGainL32Q28);
         }
         else
         {
            product1 = (int64)input1 * currentGainL32Q28;
         }

         if (gainL32Q26_2 > 0)
         {
            currentGainL32Q28 = gainL32Q26_2 << (28 - 26);
         }
         else
         {
            currentGainL32Q28 = 0;
         }

         int64 product2;

         product2 = s64_mult_s32_s32(input2, currentGainL32Q28);

         product1 = s64_shr_s64_imm5_rnd(product1, 28);
         product2 = s64_shr_s64_imm5_rnd(product2, 28);

         *pOutput++ = s16_saturate_s32(product1);
         *pOutput++ = s16_saturate_s32(product2);

         samples -= 2;
      }
   }
   else
   {
      // loop unroll by 2
      while (samples >= 2)
      {
         // Calculate the new gain
         CL32Q16_1 = CL32Q16_2 + deltaCL32Q16;
         CL32Q16_2 = CL32Q16_1 + deltaCL32Q16;

         int32 logL32Q10_1, logL32Q10_2;
         SoftVolumeControlsLogBase2Fixed(CL32Q16_1, CL32Q16_2, &logL32Q10_1, &logL32Q10_2);

         // Since the log function returns the log assuming integer argument,
         // need to adjust for the input being Q16
         logL32Q10_1 -= (16 << 10);
         logL32Q10_2 -= (16 << 10);

         int32 logL32Q14_1 = logL32Q10_1 << (14 - 10); // The range will be within (0,5), so no overflow will happen
         int32 logL32Q14_2 = logL32Q10_2 << (14 - 10); // The range will be within (0,5), so no overflow will happen

         int64 productL64Q40_1 = s64_mult_s32_s32(AL32Q26, logL32Q14_1); // both are signed
         int64 productL64Q40_2 = s64_mult_s32_s32(AL32Q26, logL32Q14_2);

         int64 productL64Q26_1 = (productL64Q40_1 >> (40 - 26));
         int64 productL64Q26_2 = (productL64Q40_2 >> (40 - 26));

         int32 productL32Q26_1 = s32_saturate_s64(productL64Q26_1);
         int32 productL32Q26_2 = s32_saturate_s64(productL64Q26_2);

         int32 gainL32Q26_1 = BL32Q26 + productL32Q26_1;

         if (gainL32Q26_1 > 0)
         {
            currentGainL32Q28 = gainL32Q26_1 << (28 - 26);
         }
         else
         {
            currentGainL32Q28 = 0;
         }

         int32 gainL32Q26_2 = BL32Q26 + productL32Q26_2;
         int32 input1       = *pInput++;
         int32 input2       = *pInput++;
         int64 product1;

         product1 = (int64)input1 * currentGainL32Q28;

         if (gainL32Q26_2 > 0)
         {
            currentGainL32Q28 = gainL32Q26_2 << (28 - 26);
         }
         else
         {
            currentGainL32Q28 = 0;
         }

         int64 product2;

         product2 = (int64)input2 * currentGainL32Q28;

         product1   = s64_shr_s64_imm5_rnd(product1, 28);
         product2   = s64_shr_s64_imm5_rnd(product2, 28);
         *pOutput++ = s16_saturate_s32(product1);
         *pOutput++ = s16_saturate_s32(product2);

         samples -= 2;
      }
   }

   // process remaining samples
   while (samples--)
   {

      // Calculate the new gain
      CL32Q16_2 += deltaCL32Q16;

      int32 logL32Q10 = SoftVolumeControlsLogBase2Fixed(CL32Q16_2);
      // Since the log function returns the log assuming integer argument,
      // need to adjust for the input being Q16
      logL32Q10 -= (16 << 10);

      int32 logL32Q14     = logL32Q10 << (14 - 10); // The range will be within (0,5), so no overflow will happen
      int64 productL64Q40 = s64_mult_s32_s32(AL32Q26, logL32Q14);
      int64 productL64Q26 = (productL64Q40 >> (40 - 26));

      int32 productL32Q26 = s32_saturate_s64(productL64Q26);

      int32 gainL32Q26 = BL32Q26 + productL32Q26;

      if (gainL32Q26 > 0)
      {
         currentGainL32Q28 = gainL32Q26 << (28 - 26);
      }
      else
      {
         currentGainL32Q28 = 0;
      }

      int32 input = *pInput++;

      int64 product;
      if ((currentGainL32Q28 < (uint32)0x80000000))
      {
         product = s64_mult_s32_s32(input, currentGainL32Q28);
      }
      else
      {
         product = (int64)input * currentGainL32Q28;
      }

      product    = s64_shr_s64_imm5_rnd(product, 28);
      *pOutput++ = s16_saturate_s32(product);
   }

   panner->coeffs.log.AL32Q26      = AL32Q26;
   panner->coeffs.log.BL32Q26      = BL32Q26;
   panner->coeffs.log.CL32Q16      = CL32Q16_2;
   panner->coeffs.log.deltaCL32Q16 = deltaCL32Q16;
   panner->step                    = step;
   panner->stepResidue             = stepResidue;
   panner->currentGainL32Q28       = currentGainL32Q28;
}

static void ApplyLogRamp32bitStep1(void *pOutPtr, void *pInPtr, SvpannerStruct *panner, uint32 samples)
{

   int32  AL32Q26           = panner->coeffs.log.AL32Q26;
   int32  BL32Q26           = panner->coeffs.log.BL32Q26;
   int32  CL32Q16_1         = panner->coeffs.log.CL32Q16;
   int32  CL32Q16_2         = panner->coeffs.log.CL32Q16;
   int32  deltaCL32Q16      = panner->coeffs.log.deltaCL32Q16;
   uint32 step              = panner->step;
   uint32 stepResidue       = panner->stepResidue;
   uint32 currentGainL32Q28 = panner->currentGainL32Q28;

   int32 *pInput  = (int32 *)(pInPtr);
   int32 *pOutput = (int32 *)(pOutPtr);

   if (0 != stepResidue)
   {

      int32 input = *pInput++;
      int64 product;
      if ((currentGainL32Q28 < (uint32)0x80000000))
      {
         product = s64_mult_s32_s32(input, currentGainL32Q28);
      }
      else
      {
         product = (int64)input * currentGainL32Q28;
      }

      product    = s64_shr_s64_imm5_rnd(product, 28);
      *pOutput++ = s32_saturate_s64(product);

      stepResidue = 0;
      samples--;
   }

   // loop unroll by 2
   while (samples >= 2)
   {
      // Calculate the new gain
      CL32Q16_1 = CL32Q16_2 + deltaCL32Q16;
      CL32Q16_2 = CL32Q16_1 + deltaCL32Q16;

      int32 logL32Q10_1, logL32Q10_2;
      SoftVolumeControlsLogBase2Fixed(CL32Q16_1, CL32Q16_2, &logL32Q10_1, &logL32Q10_2);

      // Since the log function returns the log assuming integer argument,
      // need to adjust for the input being Q16
      logL32Q10_1 -= (16 << 10);
      logL32Q10_2 -= (16 << 10);

      int32 logL32Q14_1 = logL32Q10_1 << (14 - 10); // The range will be within (0,5), so no overflow will happen
      int32 logL32Q14_2 = logL32Q10_2 << (14 - 10); // The range will be within (0,5), so no overflow will happen

      int64 productL64Q40_1 = s64_mult_s32_s32(AL32Q26, logL32Q14_1);
      int64 productL64Q40_2 = s64_mult_s32_s32(AL32Q26, logL32Q14_2);

      int64 productL64Q26_1 = (productL64Q40_1 >> (40 - 26));
      int64 productL64Q26_2 = (productL64Q40_2 >> (40 - 26));

      int32 productL32Q26_1 = s32_saturate_s64(productL64Q26_1);
      int32 productL32Q26_2 = s32_saturate_s64(productL64Q26_2);

      int32 gainL32Q26_1 = BL32Q26 + productL32Q26_1;

      if (gainL32Q26_1 > 0)
      {
         currentGainL32Q28 = gainL32Q26_1 << (28 - 26);
      }
      else
      {
         currentGainL32Q28 = 0;
      }

      int32 gainL32Q26_2 = BL32Q26 + productL32Q26_2;
      int32 input1       = *pInput++;
      int32 input2       = *pInput++;
      int64 product1;
      if ((currentGainL32Q28 < (uint32)0x80000000))
      {
         product1 = s64_mult_s32_s32(input1, currentGainL32Q28);
      }
      else
      {
         product1 = (int64)input1 * currentGainL32Q28;
      }

      if (gainL32Q26_2 > 0)
      {
         currentGainL32Q28 = gainL32Q26_2 << (28 - 26);
      }
      else
      {
         currentGainL32Q28 = 0;
      }

      int64 product2;
      if ((currentGainL32Q28 < (uint32)0x80000000))
      {
         product2 = s64_mult_s32_s32(input2, currentGainL32Q28);
      }
      else
      {
         product2 = (int64)input2 * currentGainL32Q28;
      }

      product1   = s64_shr_s64_imm5_rnd(product1, 28);
      product2   = s64_shr_s64_imm5_rnd(product2, 28);
      *pOutput++ = s32_saturate_s64(product1);
      *pOutput++ = s32_saturate_s64(product2);

      samples -= 2;
   }

   // process remaining samples
   while (samples--)
   {

      // Calculate the new gain
      CL32Q16_2 += deltaCL32Q16;

      int32 logL32Q10 = SoftVolumeControlsLogBase2Fixed(CL32Q16_2);
      // Since the log function returns the log assuming integer argument,
      // need to adjust for the input being Q16
      logL32Q10 -= (16 << 10);

      int32 logL32Q14     = logL32Q10 << (14 - 10); // The range will be within (0,5), so no overflow will happen
      int64 productL64Q40 = s64_mult_s32_s32(AL32Q26, logL32Q14);
      int64 productL64Q26 = (productL64Q40 >> (40 - 26));

      int32 productL32Q26 = s32_saturate_s64(productL64Q26);

      int32 gainL32Q26 = BL32Q26 + productL32Q26;

      if (gainL32Q26 > 0)
      {
         currentGainL32Q28 = gainL32Q26 << (28 - 26);
      }
      else
      {
         currentGainL32Q28 = 0;
      }

      int32 input = *pInput++;
      int64 product;
      if ((currentGainL32Q28 < (uint32)0x80000000))
      {
         product = s64_mult_s32_s32(input, currentGainL32Q28);
      }
      else
      {
         product = (int64)input * currentGainL32Q28;
      }

      product    = s64_shr_s64_imm5_rnd(product, 28);
      *pOutput++ = s32_saturate_s64(product);
   }

   panner->coeffs.log.AL32Q26      = AL32Q26;
   panner->coeffs.log.BL32Q26      = BL32Q26;
   panner->coeffs.log.CL32Q16      = CL32Q16_2;
   panner->coeffs.log.deltaCL32Q16 = deltaCL32Q16;
   panner->step                    = step;
   panner->stepResidue             = stepResidue;
   panner->currentGainL32Q28       = currentGainL32Q28;
}

/*=============================================================================
FUNCTION      CSoftVolumeControlsLib::ApplyLogRamp

DESCRIPTION   increase gain(Q12) in a logarithmic curve and apply to the input(Q15) and
save to output (Q15)
OUTPUTS

DEPENDENCIES  None

RETURN VALUE  None

SIDE EFFECTS
===============================================================================*/

void CSoftVolumeControlsLib::ApplyLogRamp(void *pOutPtr, void *pInPtr, SvpannerStruct *panner, uint32 samples)
{
   uint32 i;
   int32  AL32Q26           = panner->coeffs.log.AL32Q26;
   int32  BL32Q26           = panner->coeffs.log.BL32Q26;
   int32  CL32Q16           = panner->coeffs.log.CL32Q16;
   int32  deltaCL32Q16      = panner->coeffs.log.deltaCL32Q16;
   uint32 step              = panner->step;
   uint32 stepResidue       = panner->stepResidue;
   uint32 currentGainL32Q28 = panner->currentGainL32Q28;

   if (step <= 1)
   {
      if (m_bytesPerSample == 2)
      {
         ApplyLogRamp16bitStep1(pOutPtr, pInPtr, panner, samples);

      }    // if
      else // (m_bytesPerSample == 4)
      {
         ApplyLogRamp32bitStep1(pOutPtr, pInPtr, panner, samples);
      }
   }
   else // step > 1
   {
      for (i = samples; i > 0;)
      {
         if (0 == stepResidue)
         {
            // Calculate the new gain
            CL32Q16 += deltaCL32Q16;

            int32 logL32Q10 = SoftVolumeControlsLogBase2Fixed(CL32Q16);
            // Since the log function returns the log assuming integer argument,
            // need to adjust for the input being Q16
            logL32Q10 -= (16 << 10);

            int32 logL32Q14     = logL32Q10 << (14 - 10); // The range will be within (0,5), so no overflow will happen
            int64 productL64Q40 = ((int64)(AL32Q26)) * logL32Q14;
            int64 productL64Q26 = (productL64Q40 >> (40 - 26));

            int32 productL32Q26 = s32_saturate_s64(productL64Q26);

            int32 gainL32Q26 = BL32Q26 + productL32Q26;
            if (gainL32Q26 > 0)
            {
               currentGainL32Q28 = gainL32Q26 << (28 - 26);
            }
            else
            {
               currentGainL32Q28 = 0;
            }

            stepResidue = step;
         }

         uint32 deltaStep = (i < stepResidue) ? i : stepResidue;

         ApplySteadyGain(pOutPtr, pInPtr, currentGainL32Q28, deltaStep);

         IncrementPointer(&pInPtr, deltaStep);
         IncrementPointer(&pOutPtr, deltaStep);

         i -= deltaStep;
         stepResidue -= deltaStep;
      }
      panner->coeffs.log.AL32Q26      = AL32Q26;
      panner->coeffs.log.BL32Q26      = BL32Q26;
      panner->coeffs.log.CL32Q16      = CL32Q16;
      panner->coeffs.log.deltaCL32Q16 = deltaCL32Q16;
      panner->step                    = step;
      panner->stepResidue             = stepResidue;
      panner->currentGainL32Q28       = currentGainL32Q28;
   }
}

uint32 CSoftVolumeControlsLib::GetBytesPerSample(void) const
{
   return m_bytesPerSample;
}

static void ApplyExpRamp16bitStep1(void *pOutPtr, void *pInPtr, SvpannerStruct *panner, uint32 samples)
{
   int32 AL32Q26 = panner->coeffs.exp.AL32Q26;
   int32 BL32Q26 = panner->coeffs.exp.BL32Q26;

   int32 CL32Q26_1 = panner->coeffs.exp.CL32Q26;
   int32 CL32Q26_2 = panner->coeffs.exp.CL32Q26;

   int32  deltaCL32Q26      = panner->coeffs.exp.deltaCL32Q26;
   uint32 step              = panner->step;
   uint32 stepResidue       = panner->stepResidue;
   uint32 currentGainL32Q28 = panner->currentGainL32Q28;

   int16 *pInput  = (int16 *)(pInPtr);
   int16 *pOutput = (int16 *)(pOutPtr);

   if (0 != stepResidue)
   {

      int32 input = *pInput++;

      int64 product;

      if ((currentGainL32Q28 < (uint32)0x80000000))
      {
         product = s64_mult_s32_s32(input, currentGainL32Q28);
      }
      else
      {
         product = (int64)input * currentGainL32Q28;
      }

      product    = s64_shr_s64_imm5_rnd(product, 28);
      *pOutput++ = s16_saturate_s32(product);

      stepResidue = 0;
      samples--;
   }

   if (panner->newGainL32Q28 < (uint32)0x80000000)
   {
      // loop unrolled by 2
      while (samples >= 2)
      {
         // Calculate the new gain
         CL32Q26_1 = CL32Q26_2 + deltaCL32Q26;
         CL32Q26_2 = CL32Q26_1 + deltaCL32Q26;

         int32 expL32Q14_1, expL32Q14_2;

#if ((defined __hexagon__) || (defined __qdsp6__))
         SoftVolumeControlsExp2Fixed_asm(CL32Q26_1, CL32Q26_2, &expL32Q14_1, &expL32Q14_2);
#else
         SoftVolumeControlsExp2Fixed(CL32Q26_1, CL32Q26_2, &expL32Q14_1, &expL32Q14_2);
#endif

         int64 product1L64Q40 = s64_mult_s32_s32(AL32Q26, expL32Q14_1);
         int64 product2L64Q40 = s64_mult_s32_s32(AL32Q26, expL32Q14_2);
         int64 product1L64Q26 = (product1L64Q40 >> (40 - 26));
         int64 product2L64Q26 = (product2L64Q40 >> (40 - 26));

         int32 product1L32Q26 = s32_saturate_s64(product1L64Q26);
         int32 product2L32Q26 = s32_saturate_s64(product2L64Q26);

         int32 gainL32Q26  = BL32Q26 + product1L32Q26;
         int32 gain2L32Q26 = BL32Q26 + product2L32Q26;

         if (gainL32Q26 > 0)
         {
            currentGainL32Q28 = gainL32Q26 << (28 - 26);
         }
         else
         {
            currentGainL32Q28 = 0;
         }

         int32 input  = *pInput++;
         int32 input2 = *pInput++;

         int64 product;

         product = s64_mult_s32_s32(input, currentGainL32Q28);

         if (gain2L32Q26 > 0)
         {
            currentGainL32Q28 = gain2L32Q26 << (28 - 26);
         }
         else
         {
            currentGainL32Q28 = 0;
         }
         int64 product2;

         product2 = s64_mult_s32_s32(input2, currentGainL32Q28);

         product  = s64_shr_s64_imm5_rnd(product, 28);
         product2 = s64_shr_s64_imm5_rnd(product2, 28);

         *pOutput++ = s16_saturate_s32(product);
         *pOutput++ = s16_saturate_s32(product2);
         samples -= 2;
      }
   }
   else
   {
      // loop unrolled by 2
      while (samples >= 2)
      {
         // Calculate the new gain
         CL32Q26_1 = CL32Q26_2 + deltaCL32Q26;
         CL32Q26_2 = CL32Q26_1 + deltaCL32Q26;

         int32 expL32Q14_1, expL32Q14_2;
#if ((defined __hexagon__) || (defined __qdsp6__))
         SoftVolumeControlsExp2Fixed_asm(CL32Q26_1, CL32Q26_2, &expL32Q14_1, &expL32Q14_2);
#else
         SoftVolumeControlsExp2Fixed(CL32Q26_1, CL32Q26_2, &expL32Q14_1, &expL32Q14_2);
#endif

         int64 product1L64Q40 = s64_mult_s32_s32(AL32Q26, expL32Q14_1);
         int64 product2L64Q40 = s64_mult_s32_s32(AL32Q26, expL32Q14_2);
         int64 product1L64Q26 = (product1L64Q40 >> (40 - 26));
         int64 product2L64Q26 = (product2L64Q40 >> (40 - 26));

         int32 product1L32Q26 = s32_saturate_s64(product1L64Q26);
         int32 product2L32Q26 = s32_saturate_s64(product2L64Q26);

         int32 gainL32Q26  = BL32Q26 + product1L32Q26;
         int32 gain2L32Q26 = BL32Q26 + product2L32Q26;

         if (gainL32Q26 > 0)
         {
            currentGainL32Q28 = gainL32Q26 << (28 - 26);
         }
         else
         {
            currentGainL32Q28 = 0;
         }

         int32 input  = *pInput++;
         int32 input2 = *pInput++;

         int64 product;

         product = (int64)input * currentGainL32Q28;

         if (gain2L32Q26 > 0)
         {
            currentGainL32Q28 = gain2L32Q26 << (28 - 26);
         }
         else
         {
            currentGainL32Q28 = 0;
         }
         int64 product2;

         product2 = (int64)input2 * currentGainL32Q28;

         product  = s64_shr_s64_imm5_rnd(product, 28);
         product2 = s64_shr_s64_imm5_rnd(product2, 28);

         *pOutput++ = s16_saturate_s32(product);
         *pOutput++ = s16_saturate_s32(product2);
         samples -= 2;
      }
   }

   // remaining samples
   while (samples--)
   {
      CL32Q26_2 += deltaCL32Q26;

      int32 expL32Q14     = SoftVolumeControlsExp2FixedSingle(CL32Q26_2);
      int64 productL64Q40 = s64_mult_s32_s32(AL32Q26, expL32Q14);

      int64 productL64Q26 = (productL64Q40 >> (40 - 26));

      int32 productL32Q26 = s32_saturate_s64(productL64Q26);

      int32 gainL32Q26 = BL32Q26 + productL32Q26;
      if (gainL32Q26 > 0)
      {
         currentGainL32Q28 = gainL32Q26 << (28 - 26);
      }
      else
      {
         currentGainL32Q28 = 0;
      }

      int32 input = *pInput++;
      int64 product;
      if ((currentGainL32Q28 < (uint32)0x80000000))
      {
         product = s64_mult_s32_s32(input, currentGainL32Q28);
      }
      else
      {
         product = (int64)input * currentGainL32Q28;
      }

      product    = s64_shr_s64_imm5_rnd(product, 28);
      *pOutput++ = s16_saturate_s32(product);
   }

   panner->coeffs.exp.AL32Q26      = AL32Q26;
   panner->coeffs.exp.BL32Q26      = BL32Q26;
   panner->coeffs.exp.CL32Q26      = CL32Q26_2;
   panner->coeffs.exp.deltaCL32Q26 = deltaCL32Q26;
   panner->step                    = step;
   panner->stepResidue             = stepResidue;
   panner->currentGainL32Q28       = currentGainL32Q28;
}

void ApplyExpRamp32bitStep1(void *pOutPtr, void *pInPtr, SvpannerStruct *panner, uint32 samples)
{
   int32 AL32Q26 = panner->coeffs.exp.AL32Q26;
   int32 BL32Q26 = panner->coeffs.exp.BL32Q26;

   int32  CL32Q26_1         = panner->coeffs.exp.CL32Q26;
   int32  CL32Q26_2         = panner->coeffs.exp.CL32Q26;
   int32  deltaCL32Q26      = panner->coeffs.exp.deltaCL32Q26;
   uint32 step              = panner->step;
   uint32 stepResidue       = panner->stepResidue;
   uint32 currentGainL32Q28 = panner->currentGainL32Q28;

   int32 *pInput  = (int32 *)(pInPtr);
   int32 *pOutput = (int32 *)(pOutPtr);

   if (0 != stepResidue)
   {

      int32 input = *pInput++;
      int64 product;
      if ((currentGainL32Q28 < (uint32)0x80000000))
      {
         product = s64_mult_s32_s32(input, currentGainL32Q28);
      }
      else
      {
         product = (int64)input * currentGainL32Q28;
      }

      product    = s64_shr_s64_imm5_rnd(product, 28);
      *pOutput++ = s32_saturate_s64(product);

      stepResidue = 0;
      samples--;
   }

   if ((currentGainL32Q28 < (uint32)0x80000000))
   {
      // loop unrolled by 2
      while (samples >= 2)
      {
         // Calculate the new gain
         CL32Q26_1 = CL32Q26_2 + deltaCL32Q26;
         CL32Q26_2 = CL32Q26_1 + deltaCL32Q26;

         int32 expL32Q14_1, expL32Q14_2;

#if ((defined __hexagon__) || (defined __qdsp6__))
         SoftVolumeControlsExp2Fixed_asm(CL32Q26_1, CL32Q26_2, &expL32Q14_1, &expL32Q14_2);
#else
         SoftVolumeControlsExp2Fixed(CL32Q26_1, CL32Q26_2, &expL32Q14_1, &expL32Q14_2);
#endif

         int64 product1L64Q40 = s64_mult_s32_s32(AL32Q26, expL32Q14_1);
         int64 product2L64Q40 = s64_mult_s32_s32(AL32Q26, expL32Q14_2);
         int64 product1L64Q26 = (product1L64Q40 >> (40 - 26));
         int64 product2L64Q26 = (product2L64Q40 >> (40 - 26));

         int32 product1L32Q26 = s32_saturate_s64(product1L64Q26);
         int32 product2L32Q26 = s32_saturate_s64(product2L64Q26);

         int32 gainL32Q26  = BL32Q26 + product1L32Q26;
         int32 gain2L32Q26 = BL32Q26 + product2L32Q26;

         if (gainL32Q26 > 0)
         {
            currentGainL32Q28 = gainL32Q26 << (28 - 26);
         }
         else
         {
            currentGainL32Q28 = 0;
         }

         int32 input  = *pInput++;
         int32 input2 = *pInput++;
         int64 product;

         product = s64_mult_s32_s32(input, currentGainL32Q28);

         if (gain2L32Q26 > 0)
         {
            currentGainL32Q28 = gain2L32Q26 << (28 - 26);
         }
         else
         {
            currentGainL32Q28 = 0;
         }
         int64 product2;

         product2 = s64_mult_s32_s32(input2, currentGainL32Q28);

         product  = s64_shr_s64_imm5_rnd(product, 28);
         product2 = s64_shr_s64_imm5_rnd(product2, 28);

         *pOutput++ = s32_saturate_s64(product);
         *pOutput++ = s32_saturate_s64(product2);
         samples -= 2;
      }
   }
   else
   {
      // loop unrolled by 2
      while (samples >= 2)
      {
         // Calculate the new gain
         CL32Q26_1 = CL32Q26_2 + deltaCL32Q26;
         CL32Q26_2 = CL32Q26_1 + deltaCL32Q26;

         int32 expL32Q14_1, expL32Q14_2;

#if ((defined __hexagon__) || (defined __qdsp6__))
         SoftVolumeControlsExp2Fixed_asm(CL32Q26_1, CL32Q26_2, &expL32Q14_1, &expL32Q14_2);
#else
         SoftVolumeControlsExp2Fixed(CL32Q26_1, CL32Q26_2, &expL32Q14_1, &expL32Q14_2);
#endif

         int64 product1L64Q40 = s64_mult_s32_s32(AL32Q26, expL32Q14_1);
         int64 product2L64Q40 = s64_mult_s32_s32(AL32Q26, expL32Q14_2);
         int64 product1L64Q26 = (product1L64Q40 >> (40 - 26));
         int64 product2L64Q26 = (product2L64Q40 >> (40 - 26));

         int32 product1L32Q26 = s32_saturate_s64(product1L64Q26);
         int32 product2L32Q26 = s32_saturate_s64(product2L64Q26);

         int32 gainL32Q26  = BL32Q26 + product1L32Q26;
         int32 gain2L32Q26 = BL32Q26 + product2L32Q26;

         if (gainL32Q26 > 0)
         {
            currentGainL32Q28 = gainL32Q26 << (28 - 26);
         }
         else
         {
            currentGainL32Q28 = 0;
         }

         int32 input  = *pInput++;
         int32 input2 = *pInput++;
         int64 product;

         product = (int64)input * currentGainL32Q28;

         if (gain2L32Q26 > 0)
         {
            currentGainL32Q28 = gain2L32Q26 << (28 - 26);
         }
         else
         {
            currentGainL32Q28 = 0;
         }
         int64 product2;

         product2 = (int64)input2 * currentGainL32Q28;

         product  = s64_shr_s64_imm5_rnd(product, 28);
         product2 = s64_shr_s64_imm5_rnd(product2, 28);

         *pOutput++ = s32_saturate_s64(product);
         *pOutput++ = s32_saturate_s64(product2);
         samples -= 2;
      }
   }

   // remaining samples
   while (samples--)
   {
      // Calculate the new gain
      CL32Q26_2 += deltaCL32Q26;

      int32 expL32Q14     = SoftVolumeControlsExp2FixedSingle(CL32Q26_2);
      int64 productL64Q40 = s64_mult_s32_s32(AL32Q26, expL32Q14);

      int64 productL64Q26 = (productL64Q40 >> (40 - 26));

      int32 productL32Q26 = s32_saturate_s64(productL64Q26);

      int32 gainL32Q26 = BL32Q26 + productL32Q26;
      if (gainL32Q26 > 0)
      {
         currentGainL32Q28 = gainL32Q26 << (28 - 26);
      }
      else
      {
         currentGainL32Q28 = 0;
      }

      int32 input = *pInput++;
      int64 product;
      if ((currentGainL32Q28 < (uint32)0x80000000))
      {
         product = s64_mult_s32_s32(input, currentGainL32Q28);
      }
      else
      {
         product = (int64)input * currentGainL32Q28;
      }

      product    = s64_shr_s64_imm5_rnd(product, 28);
      *pOutput++ = s32_saturate_s64(product);
   }

   panner->coeffs.exp.AL32Q26      = AL32Q26;
   panner->coeffs.exp.BL32Q26      = BL32Q26;
   panner->coeffs.exp.CL32Q26      = CL32Q26_2;
   panner->coeffs.exp.deltaCL32Q26 = deltaCL32Q26;
   panner->step                    = step;
   panner->stepResidue             = stepResidue;
   panner->currentGainL32Q28       = currentGainL32Q28;
}

/*=============================================================================
FUNCTION      CSoftVolumeControlsLib::ApplyExpRamp

DESCRIPTION   increase gain(Q12) in a exponential curve and apply to the input(Q15) and
save to output (Q15)
OUTPUTS

DEPENDENCIES  None

RETURN VALUE  None

SIDE EFFECTS
===============================================================================*/
void CSoftVolumeControlsLib::ApplyExpRamp(void *pOutPtr, void *pInPtr, SvpannerStruct *panner, uint32 samples)
{
   uint32 i;
   int32  AL32Q26           = panner->coeffs.exp.AL32Q26;
   int32  BL32Q26           = panner->coeffs.exp.BL32Q26;
   int32  CL32Q26           = panner->coeffs.exp.CL32Q26;
   int32  deltaCL32Q26      = panner->coeffs.exp.deltaCL32Q26;
   uint32 step              = panner->step;
   uint32 stepResidue       = panner->stepResidue;
   uint32 currentGainL32Q28 = panner->currentGainL32Q28;

   if (step <= 1)
   {
      if (m_bytesPerSample == 2)
      {
         ApplyExpRamp16bitStep1(pOutPtr, pInPtr, panner, samples);

      }    // if
      else // (m_bytesPerSample == 4)
      {
         ApplyExpRamp32bitStep1(pOutPtr, pInPtr, panner, samples);
      }
   }
   else
   {
      for (i = samples; i > 0;)
      {
         if (0 == stepResidue)
         {
            // Calculate the new gain
            CL32Q26 += deltaCL32Q26;

            int32 expL32Q14     = SoftVolumeControlsExp2FixedSingle(CL32Q26);
            int64 productL64Q40 = ((int64)(AL32Q26)) * expL32Q14;
            int64 productL64Q26 = (productL64Q40 >> (40 - 26));

            int32 productL32Q26 = s32_saturate_s64(productL64Q26);

            int32 gainL32Q26 = BL32Q26 + productL32Q26;
            if (gainL32Q26 > 0)
            {
               currentGainL32Q28 = gainL32Q26 << (28 - 26);
            }
            else
            {
               currentGainL32Q28 = 0;
            }

            stepResidue = step;
         }

         uint32 deltaStep = (i < stepResidue) ? i : stepResidue;

         ApplySteadyGain(pOutPtr, pInPtr, currentGainL32Q28, deltaStep);

         IncrementPointer(&pInPtr, deltaStep);
         IncrementPointer(&pOutPtr, deltaStep);

         i -= deltaStep;
         stepResidue -= deltaStep;
      }

      panner->coeffs.exp.AL32Q26      = AL32Q26;
      panner->coeffs.exp.BL32Q26      = BL32Q26;
      panner->coeffs.exp.CL32Q26      = CL32Q26;
      panner->coeffs.exp.deltaCL32Q26 = deltaCL32Q26;
      panner->step                    = step;
      panner->stepResidue             = stepResidue;
      panner->currentGainL32Q28       = currentGainL32Q28;
   }
}

/*=============================================================================
FUNCTION      CSoftVolumeControlsLib::Process

DESCRIPTION   Process function is called by enhancedconvert and applies gain to the
input samples. The gain can also ramp up/down in a linear/log/exp curve
as well.
OUTPUTS

DEPENDENCIES  None

RETURN VALUE  None

SIDE EFFECTS
===============================================================================*/
void CSoftVolumeControlsLib::Process(void *pInPtr, void *pOutPtr, const uint32 nSampleCnt, void *pChannelStruct)
{

   perChannelData *pChannelData = reinterpret_cast<perChannelData *>(pChannelStruct);

   SvpannerStruct *pPanner = &pChannelData->panner;
   uint32          samples = nSampleCnt;
   if (0 == samples)
   {
      return;
   }
   if (pPanner->sampleCounter > 0)
   {
      // Soft stepping is needed
      uint32 rampSamples;
      rampSamples = (samples < pPanner->sampleCounter) ? samples : pPanner->sampleCounter;

      switch (pPanner->rampingCurve)
      {
         case RAMP_LINEAR:
            ApplyLinearRamp(pOutPtr, pInPtr, pPanner, rampSamples);
            break;
         case RAMP_LOG:
            ApplyLogRamp(pOutPtr, pInPtr, pPanner, rampSamples);
            break;
         case RAMP_EXP:
            ApplyExpRamp(pOutPtr, pInPtr, pPanner, rampSamples);
            break;
         case RAMP_FRACT_EXP:
            ApplyFractExpRamp(pOutPtr, pInPtr, pPanner, rampSamples);
            break;
      }

      IncrementPointer(&pInPtr, rampSamples);
      IncrementPointer(&pOutPtr, rampSamples);

      samples -= rampSamples;
      pPanner->sampleCounter -= rampSamples;

      if (pPanner->sampleCounter <= 0)
      {
         pPanner->currentGainL32Q28 = pPanner->targetgainL32Q28;
      }
   }

   /*-------------- if there are still samples , apply static gain-------*/
   ApplySteadyGain(pOutPtr, pInPtr, pPanner->currentGainL32Q28, samples);

} /*------------------- end of function Process-----------------------------------*/

boolean CSoftVolumeControlsLib::ProcessV2SingleChannel(void *       pInPtr,
                                                       void *       pOutPtr,
                                                       const uint32 nSampleCnt,
                                                       void *       pChannelStruct)
{
   boolean         ramp_complete = false;
   perChannelData *pChannelData  = reinterpret_cast<perChannelData *>(pChannelStruct);
   SvpannerStruct *pPanner       = &pChannelData->panner;
   uint32          samples       = nSampleCnt;

   if (0 == samples)
   {
      return false;
   }

   if (pPanner->sampleCounter > 0)
   {
      // Soft stepping is needed
      uint32 rampSamples;
      rampSamples = (samples < pPanner->sampleCounter) ? samples : pPanner->sampleCounter;

      switch (pPanner->rampingCurve)
      {
         case RAMP_LINEAR:
            ApplyLinearRamp(pOutPtr, pInPtr, pPanner, rampSamples);
            break;
         case RAMP_LOG:
            ApplyLogRamp(pOutPtr, pInPtr, pPanner, rampSamples);
            break;
         case RAMP_EXP:
            ApplyExpRamp(pOutPtr, pInPtr, pPanner, rampSamples);
            break;
         case RAMP_FRACT_EXP:
            ApplyFractExpRamp(pOutPtr, pInPtr, pPanner, rampSamples);
            break;
      }

      IncrementPointer(&pInPtr, rampSamples);
      IncrementPointer(&pOutPtr, rampSamples);

      samples -= rampSamples;
      pPanner->sampleCounter -= rampSamples;

      if (0 == pPanner->sampleCounter)
      {
         pPanner->currentGainL32Q28 = pPanner->targetgainL32Q28;
         ramp_complete              = true;
      }
   }
   else
   {
      ramp_complete = true;
   }

   /*-------------- if there are still samples , apply static gain-------*/
   ApplySteadyGain(pOutPtr, pInPtr, pPanner->currentGainL32Q28, samples);

   return ramp_complete;
}

void CSoftVolumeControlsLib::ProcessAllChannels(PerChannelDataBlock *pChannels, uint32 nChannelCnt, uint8_t *is_paused)
{
   uint32  pannerOffset     = 0xFFFFFF;
   boolean curRampCompleted = false;
   boolean rampCompleted    = false;

   // For steady state check if in-place, if yes return, if not, copy data and return
   if (STEADY == m_pauseState)
   {
      for (uint32_t channel_idx = 0; channel_idx < nChannelCnt; channel_idx++)
      {
         PerChannelDataBlock channelData = pChannels[channel_idx];
         if (channelData.inPtr != channelData.outPtr)
         {
            memscpy(channelData.outPtr,
                    (channelData.sampleCount * m_bytesPerSample),
                    channelData.inPtr,
                    (channelData.sampleCount * m_bytesPerSample));
         }
      }
      return;
   }

   // If Pause, zero fill and return.
   if (PAUSE == m_pauseState)
   {
      for (uint32_t channel_idx = 0; channel_idx < nChannelCnt; channel_idx++)
      {
         PerChannelDataBlock channelData = pChannels[channel_idx];
         memset(channelData.outPtr, 0, (channelData.sampleCount * m_bytesPerSample));
      }
      return;
   }

   // 1.Search for the earliest time a threshold was detected. If no threshold is detected, fill with zeros and then
   // return.
   // 2. If it is detected find the sample and apply 0 till that sample and start ramp after
   if (WAITING == m_pauseState)
   {
      // find earliest sample over all channels that hits threshold value
      for (int channel_idx = 0; channel_idx < nChannelCnt; channel_idx++)
      {
         PerChannelDataBlock channelData = pChannels[channel_idx];

         int curOffset = DetectThreshold(channelData.inPtr, channelData.sampleCount);
         if (THRESHOLD_NOT_DETECTED != curOffset)
         {
            // if current offset is lesser, set as new panner offset
            if (curOffset < pannerOffset)
            {
               pannerOffset = curOffset;
            }
         }
      }

      // If threshold not reached, memset to zero and return
      if (0xFFFFFF == pannerOffset)
      {
         for (int channel_idx = 0; channel_idx < nChannelCnt; channel_idx++)
         {
            PerChannelDataBlock channelData = pChannels[channel_idx];
            memset(channelData.outPtr, 0, (channelData.sampleCount * m_bytesPerSample));
         }
         return;
      }
      m_muteBeforeResumeParams.m_muteSamplesPending += pannerOffset;
   }

   if ((MUTE_BEFORE_RAMPUP == m_pauseState) | (WAITING == m_pauseState))
   {
      uint32_t mute_samples_pending = 0;
      for (int channel_idx = 0; channel_idx < nChannelCnt; channel_idx++)
      {
         mute_samples_pending             = m_muteBeforeResumeParams.m_muteSamplesPending;
         PerChannelDataBlock *channelData = &pChannels[channel_idx];
         if (mute_samples_pending < channelData->sampleCount)
         {
            memset(channelData->outPtr, 0, (mute_samples_pending * m_bytesPerSample));
            channelData->sampleCount = channelData->sampleCount - mute_samples_pending;
            channelData->inPtr       = channelData->inPtr + (mute_samples_pending * m_bytesPerSample);
            channelData->outPtr      = channelData->outPtr + (mute_samples_pending * m_bytesPerSample);
            curRampCompleted         = ProcessV2SingleChannel(channelData->inPtr,
                                                      channelData->outPtr,
                                                      channelData->sampleCount,
                                                      (void *)&(channelData->channelStruct));
            mute_samples_pending = 0;
         }
         else
         {
            memset(channelData->outPtr, 0, channelData->sampleCount * m_bytesPerSample);
            mute_samples_pending = mute_samples_pending - channelData->sampleCount;
         }
      }
      m_muteBeforeResumeParams.m_muteSamplesPending = mute_samples_pending;
      if (m_muteBeforeResumeParams.m_muteSamplesPending)
      {
         m_pauseState = MUTE_BEFORE_RAMPUP;
      }
      else
      {
         m_pauseState = RAMPING_UP;
      }
      return;
   }

   // If ramping up or ramping down or pending dfg, process data by applying ramp
   for (uint32_t channel_idx = 0; channel_idx < nChannelCnt; channel_idx++)
   {

      PerChannelDataBlock *channelData = &pChannels[channel_idx];
      curRampCompleted                 = ProcessV2SingleChannel(channelData->inPtr,
                                                channelData->outPtr,
                                                channelData->sampleCount,
                                                (void *)&(channelData->channelStruct));

      rampCompleted = (rampCompleted || curRampCompleted);
   }

   // If ramp completed, change state and set flag to paused
   if (rampCompleted)
   {
      if (m_pauseState == RAMPING_UP)
      {
         m_pauseState = STEADY;
      }
      else if (m_pauseState == RAMPING_DOWN)
      {
         // Set flag
         *is_paused   = PAUSE_RAMP_COMPLETE;
         m_pauseState = PAUSE;
      }
   }
}

int32 CSoftVolumeControlsLib::DetectThreshold(void *pInPtr, const uint32 nSampleCnt)
{
   int32 threshold_idx = THRESHOLD_NOT_DETECTED;

   if (m_qFactor == 15)
   {
      threshold_idx = DetectThreshold16(pInPtr, nSampleCnt);
   }
   else if (m_qFactor == 27)
   {
      threshold_idx = DetectThreshold24(pInPtr, nSampleCnt);
   }
   else if (m_qFactor == 31)
   {
      threshold_idx = DetectThreshold32(pInPtr, nSampleCnt);
   }

   return threshold_idx;
}

int32 CSoftVolumeControlsLib::DetectThreshold16(void *pInPtr, const uint32 nSampleCnt)
{
   int16 *pInput        = (int16 *)pInPtr;
   int32  threshold_idx = THRESHOLD_NOT_DETECTED;

   for (uint32_t sample_idx = 0; sample_idx < nSampleCnt; sample_idx++)
   {
      int32  sampleQ15     = pInput[sample_idx];
      uint32 abs_sampleQ15 = (uint32_t)u16_abs_s16_sat((int16_t)sampleQ15);
      if (abs_sampleQ15 > m_thresholdQ15)
      {

         threshold_idx = sample_idx;
         break;
      }
   }
   return threshold_idx;
}

int32 CSoftVolumeControlsLib::DetectThreshold24(void *pInPtr, const uint32 nSampleCnt)
{
   int16 *pInput        = (int16 *)pInPtr;
   int32  threshold_idx = THRESHOLD_NOT_DETECTED;

   for (uint32_t sample_idx = 0; sample_idx < nSampleCnt; sample_idx++)
   {
      int32  sampleQ27     = pInput[sample_idx];
      uint32 abs_sampleQ27 = u32_abs_s32_sat(sampleQ27);
      if (abs_sampleQ27 > m_thresholdQ27)
      {
         threshold_idx = sample_idx;
         break;
      }
   }
   return threshold_idx;
}

int32 CSoftVolumeControlsLib::DetectThreshold32(void *pInPtr, const uint32 nSampleCnt)
{
   int32 *pInput        = (int32 *)pInPtr;
   int32  threshold_idx = THRESHOLD_NOT_DETECTED;

   for (uint32_t sample_idx = 0; sample_idx < nSampleCnt; sample_idx++)
   {
      int32  sampleQ31     = pInput[sample_idx];
      uint32 abs_sampleQ31 = u32_abs_s32_sat(sampleQ31);

      if (abs_sampleQ31 > m_thresholdQ31)
      {
         threshold_idx = sample_idx;
         break;
      }
   }
   return threshold_idx;
}

uint32 CSoftVolumeControlsLib::GetPauseState() const
{
   return (uint32_t)m_pauseState;
}

uint32 CSoftVolumeControlsLib::GetTargetGain(void *pChannelStruct)
{
   perChannelData *pChannelData = reinterpret_cast<perChannelData *>(pChannelStruct);
   return pChannelData->panner.targetgainL32Q28;
}

void CSoftVolumeControlsLib::ApplySteadyGain(void *pOutPtr, void *pInPtr, const uint32 gainQ28, uint32 samples)
{
   switch (m_bytesPerSample)
   {
      case 2:
#if ((defined __hexagon__) || (defined __qdsp6__))
         if (gainQ28 < UNITY_L32_Q28)
         {
            ApplySteadyGainLessThanOne16(pOutPtr, pInPtr, gainQ28, samples);
         }
         else
#endif
            ApplySteadyGain16(pOutPtr, pInPtr, gainQ28, samples);
         break;

      case 4:
         ApplySteadyGain32(pOutPtr, pInPtr, gainQ28, samples);
         break;
   }
}

void CSoftVolumeControlsLib::ApplyFractExpRamp(void *pOutPtr, void *pInPtr, SvpannerStruct *panner, uint32 samples)
{
   uint32 i;
   uint32 step              = panner->step;
   uint32 stepResidue       = panner->stepResidue;
   uint32 currentGainL32Q28 = panner->currentGainL32Q28;
   uint32 AL32Q28           = panner->coeffs.fract.AL32Q28;
   uint32 BL32Q28           = panner->coeffs.fract.BL32Q28;
   uint32 CL32Q31           = panner->coeffs.fract.CL32Q31;
   int32  deltaCL32Q31      = panner->coeffs.fract.deltaCL32Q31;

   for (i = samples; i > 0;)
   {
      if (0 == stepResidue)
      {
         // Calculate the new gain
         currentGainL32Q28 = AL32Q28 + u32_mult_u32_u32(BL32Q28, pow_1_75(CL32Q31) << 1);
         CL32Q31 += deltaCL32Q31;
         stepResidue = step;
      }

      uint32 deltaStep = (i < stepResidue) ? i : stepResidue;

      ApplySteadyGain(pOutPtr, pInPtr, currentGainL32Q28, deltaStep);

      IncrementPointer(&pInPtr, deltaStep);
      IncrementPointer(&pOutPtr, deltaStep);

      i -= deltaStep;
      stepResidue -= deltaStep;
      panner->index += deltaStep;
   }

   panner->step                 = step;
   panner->coeffs.fract.CL32Q31 = CL32Q31;
   panner->stepResidue          = stepResidue;
   panner->currentGainL32Q28    = currentGainL32Q28;
}

void CSoftVolumeControlsLib::IncrementPointer(void **pPtr, uint32 samples)
{
   uint32 numBytes = samples * m_bytesPerSample;
   byte * ptr      = (byte *)(*pPtr);
   ptr += numBytes;
   *pPtr = ptr;
}

void CSoftVolumeControlsLib::ApplySteadyGain16(void *pOutPtr, void *pInPtr, const uint32 gainQ28, uint32 samples)
{

   int16 *pInput  = (int16 *)(pInPtr);
   int16 *pOutput = (int16 *)(pOutPtr);

   if (gainQ28 < (uint32)0x80000000)
   {
      // unroll loop by 2
      while (samples >= 2)
      {
         int32 input1 = *pInput++;
         int32 input2 = *pInput++;
         int64 product1, product2;

         product1   = s64_mult_s32_s32(input1, gainQ28);
         product2   = s64_mult_s32_s32(input2, gainQ28);
         product1   = s64_shr_s64_imm5_rnd(product1, 28);
         product2   = s64_shr_s64_imm5_rnd(product2, 28);
         *pOutput++ = s16_saturate_s32(product1);
         *pOutput++ = s16_saturate_s32(product2);
         samples -= 2;
      }

      while (samples--)
      {
         int32 input1 = *pInput++;

         int64 product1;

         product1 = s64_mult_s32_s32(input1, gainQ28);

         product1   = s64_shr_s64_imm5_rnd(product1, 28);
         *pOutput++ = s16_saturate_s32(product1);
      }
   }
   else
   {
      // unroll loop by 2
      while (samples >= 2)
      {
         int32 input1 = *pInput++;
         int32 input2 = *pInput++;
         int64 product1, product2;

         //      int64 input = pInput[i];
         product1 = (int64)input1 * gainQ28;
         product2 = (int64)input2 * gainQ28;

         product1   = s64_shr_s64_imm5_rnd(product1, 28);
         product2   = s64_shr_s64_imm5_rnd(product2, 28);
         *pOutput++ = s16_saturate_s32(product1);
         *pOutput++ = s16_saturate_s32(product2);
         samples -= 2;
      }

      while (samples--)
      {
         int32 input1 = *pInput++;

         int64 product1;

         product1   = (int64)input1 * gainQ28;
         product1   = s64_shr_s64_imm5_rnd(product1, 28);
         *pOutput++ = s16_saturate_s32(product1);
      }
   }
}

void CSoftVolumeControlsLib::ApplySteadyGain32(void *pOutPtr, void *pInPtr, const uint32 gainQ28, uint32 samples)
{
   int32 *pInput  = (int32 *)(pInPtr);
   int32 *pOutput = (int32 *)(pOutPtr);

   if (gainQ28 < (uint32)0x80000000)
   {
      // unroll loop by 2
      while (samples >= 2)
      {
         int32 input1 = *pInput++;
         int32 input2 = *pInput++;

         int64 product1;
         int64 product2;

         product1 = s64_mult_s32_s32(input1, gainQ28);
         product2 = s64_mult_s32_s32(input2, gainQ28);

         product1   = s64_shr_s64_imm5_rnd(product1, 28);
         product2   = s64_shr_s64_imm5_rnd(product2, 28);
         *pOutput++ = s32_saturate_s64(product1);
         *pOutput++ = s32_saturate_s64(product2);
         samples -= 2;
      }

      while (samples--)
      {
         int32 input1 = *pInput++;

         int64 product1;

         product1 = s64_mult_s32_s32(input1, gainQ28);

         product1   = s64_shr_s64_imm5_rnd(product1, 28);
         *pOutput++ = s32_saturate_s64(product1);
      }
   }
   else // gainQ28 >= 0x80000000
   {
      // unroll loop by 2
      while (samples >= 2)
      {
         int32 input1 = *pInput++;
         int32 input2 = *pInput++;

         int64 product1;
         int64 product2;

         //      int64 input = pInput[i];
         product1 = (int64)input1 * gainQ28;
         product2 = (int64)input2 * gainQ28;

         product1   = s64_shr_s64_imm5_rnd(product1, 28);
         product2   = s64_shr_s64_imm5_rnd(product2, 28);
         *pOutput++ = s32_saturate_s64(product1);
         *pOutput++ = s32_saturate_s64(product2);
         samples -= 2;
      }

      while (samples--)
      {
         int32 input1 = *pInput++;

         int64 product1;

         product1 = (int64)input1 * gainQ28;

         product1   = s64_shr_s64_imm5_rnd(product1, 28);
         *pOutput++ = s32_saturate_s64(product1);
      }
   }
}

// Look up table for the function pow_1_75.
const int32_t coeffs_1_75[64][4] = {
   { -1164088560, 313392297, 1319088, 0 },        { -1164088560, 258825646, 10259994, 92682 },
   { -307879499, 204258995, 17495691, 311744 },   { -303631637, 189827143, 23653287, 633807 },
   { -193864184, 175594410, 29362999, 1048576 },  { -159201065, 166507027, 34708334, 1549503 },
   { -127113798, 159044477, 39795076, 2131865 },  { -106977359, 153086017, 44672115, 2792007 },
   { -91359926, 148071454, 49377700, 3526975 },   { -79572318, 143788957, 53938019, 4334304 },
   { -70219336, 140059005, 58373144, 5211886 },   { -62686037, 136767473, 62698558, 6157893 },
   { -56488224, 133829065, 66926628, 7170709 },   { -51312925, 131181180, 71067413, 8248895 },
   { -46932167, 128775886, 75129243, 9391155 },   { -43181370, 126575941, 79119115, 10596309 },
   { -39937509, 124551814, 83042986, 11863283 },  { -37107280, 122679744, 86905979, 13191086 },
   { -34618601, 120940340, 90712543, 14578801 },  { -32414981, 119317593, 94466573, 16025579 },
   { -30451554, 117798141, 98171507, 17530626 },  { -28692258, 116370724, 101830395, 19093199 },
   { -27107797, 115025775, 105445965, 20712600 }, { -25674141, 113755097, 109020667, 22388172 },
   { -24371402, 112551621, 112556709, 24119295 }, { -23182988, 111409212, 116056097, 25905379 },
   { -22094954, 110322509, 119520655, 27745866 }, { -21095496, 109286808, 122952051, 29640226 },
   { -20174565, 108297957, 126351813, 31587953 }, { -19323550, 107352274, 129721348, 33588563 },
   { -18535035, 106446483, 133061953, 35641594 }, { -17802600, 105577653, 136374830, 37746605 },
   { -17120661, 104743156, 139661093, 39903169 }, { -16484340, 103940625, 142921777, 42110881 },
   { -15889356, 103167922, 146157848, 44369347 }, { -15331939, 102423108, 149370208, 46678190 },
   { -14808758, 101704424, 152559701, 49037047 }, { -14316856, 101010263, 155727118, 51445566 },
   { -13853601, 100339160, 158873202, 53903408 }, { -13416645, 99689773, 161998654, 56410246 },
   { -13003885, 99060868, 165104133, 58965762 },  { -12613431, 98451311, 168190261, 61569649 },
   { -12243584, 97860056, 171257626, 64221610 },  { -11892809, 97286138, 174306785, 66921355 },
   { -11559717, 96728663, 177338267, 69668605 },  { -11243049, 96186801, 180352571, 72463086 },
   { -10941659, 95659783, 183350173, 75304536 },  { -10654507, 95146893, 186331528, 78192695 },
   { -10380640, 94647463, 189297065, 81127313 },  { -10119188, 94160870, 192247195, 84108148 },
   { -9869354, 93686533, 195182310, 87134960 },   { -9630408, 93223907, 198102786, 90207519 },
   { -9401677, 92772482, 201008980, 93325598 },   { -9182542, 92331778, 203901234, 96488977 },
   { -8972431, 91901347, 206779876, 99697440 },   { -8770823, 91480764, 209645222, 102950779 },
   { -8577206, 91069632, 212497572, 106248786 },  { -8391217, 90667575, 215337216, 109591261 },
   { -8212108, 90274237, 218164431, 112978009 },  { -8040720, 89889294, 220979487, 116408837 },
   { -7872098, 89512385, 223782638, 119883556 },  { -7723049, 89143381, 226574134, 123401983 },
   { -7529275, 88781363, 229354208, 126963938 },  { -7529275, 88428428, 232123111, 130569244 },
};

/*
 * Implements the function x^1.75.
 *
 * Input:
 * x : Input value in Q31 format. 0 <= x < 1.
 *
 * Return value:
 * x^1.75 in Q31 format.
 *
 * Implementation details:
 * The function is implemented using spline interpolation.
 * MATLAB was used to generate the spline coefficients using
 * 64 intervals from 0 to 1. These coefficients are stored in
 * Q27 format (since the largest coefficient had magnitude about
 * 8.6). The coefficients are stored in a 2 dimensional array.
 * The first index is the interval number. The top 6 bits
 * of the input data can be used to get this number. The
 * rest of the bits will be the offset z to be plugged into
 * y = a*z^3 + b*z^2 + c*z + d
 * to get the final result y.
 */
static uint32_t pow_1_75(uint32_t x)
{
   const uint32_t DATA_Q_FACTOR               = 31;
   const uint32_t COEFF_Q_FACTOR              = 27;
   const uint32_t NUM_INDEX_BITS              = 6;
   const uint32_t Q_FACTOR_FOR_MULTIPLICATION = 31;

   uint32_t index  = x >> (DATA_Q_FACTOR - NUM_INDEX_BITS);
   int32_t  offset = x & ((1 << (DATA_Q_FACTOR - NUM_INDEX_BITS)) - 1);

   int32_t z  = offset;
   int32_t z2 = s32_mult_s32_s32_left_shift_1(z, z);
   int32_t z3 = s32_mult_s32_s32_left_shift_1(z2, z);

   int32_t y = (coeffs_1_75[index][3] + s32_mult_s32_s32_left_shift_1(coeffs_1_75[index][2], z) +
                s32_mult_s32_s32_left_shift_1(coeffs_1_75[index][1], z2) + s32_mult_s32_s32_left_shift_1(coeffs_1_75[index][0], z3))
               << (Q_FACTOR_FOR_MULTIPLICATION - COEFF_Q_FACTOR);

   return ((y > 0) ? y : 0);
}
