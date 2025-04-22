/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*============================================================================
  @file audio_buffer32.c

  Buffer related 32 bit manipulations
  ============================================================================*/
#include "audio_dsp32.h"
#include "audio_basic_op_ext.h"
#include <stringl.h>


// copy 16 bit buffer into 32 bit buffer
void buffer32_copy16(int32 *dest, int16 *src, int32 samples)
{
   int32 i;
   for (i = 0; i < samples; ++i) {
      *dest++ = *src++;
   }
}

// apply 32 bit gain, then write to output 32 bit buffer
void buffer32_fill32(int32 *dest, int32 *src, int32 gain, int16 q_factor, int32 samples)
{
   int32 i;
   int16 shift = s16_sub_s16_s16(32, q_factor);
   int32 unity = 1<<q_factor;

   int32 p5 = 0;
   if (shift <= 31){
	   p5 = (int32)(1 << (31 - shift));
   } else
   {
	   p5 = 0;
   }
   if (gain == unity || gain == unity-1) {
      buffer32_copy_v2(dest, src, samples);
   } else if (gain == -unity) {
      for (i = 0; i < samples; ++i) {
         *dest++ = s32_neg_s32_sat(*src);
         src++;
      }
   } else {
      for (i = 0; i < samples; ++i) {
		 *dest++ = s32_saturate_s64(s64_shl_s64(s64_add_s64_s32(s64_mult_s32_s32(*src, gain),p5), shift - 32));
		 src++;
      }
   }
}

// apply 32 bit gain, then mix to output 32 bit buffer
void buffer32_mix32(int32 *dest, int32 *src, int32 gain, int16 q_factor, int32 samples)
{
   int32 i, tmp32;
   int16 shift = s16_sub_s16_s16(32, q_factor);
   int32 unity = 1<<q_factor, tmpShiftL32=0;
   int64 tmpL64=0;

   if (gain == unity || gain == unity-1) {
      for (i = 0; i < samples; ++i) {
         *dest = s32_add_s32_s32_sat(*dest, *src);
         dest++;
         src++;
      }
   } else if (gain == -unity) {
      for (i = 0; i < samples; ++i) {
         *dest = s32_sub_s32_s32_sat(*dest, *src);
         dest++;
         src++;
      }
   } else {
	  if(shift < 32)
			tmpShiftL32 = ((int32)1) << (31-shift);
      for (i = 0; i < samples; ++i) {
		  tmpL64 = s64_add_s64_s32(s64_mult_s32_s32(*src, gain), tmpShiftL32);
		  tmp32 = s32_saturate_s64(s64_shl_s64(tmpL64,shift-32));
          //tmp32 = s32_saturate_s64(s64_mult_s32_s32_shift_rnd(*src++, gain, shift));
         *dest = s32_add_s32_s32_sat(*dest, tmp32);
         dest++;
         src++;
      }
   }
}

// apply 16 bit Q15 gain to 32 bit input, then write to 32 bit output
void buffer32_fill16(int32 *dest, int32 *src, int16 gain, int32 samples)
{
   int32 i;

   if (Q15_ONE == gain) {
      buffer32_copy_v2(dest, src, samples);
   } else if (Q15_MINUSONE == gain) {
      for (i = 0; i < samples; ++i) {
         *dest++ = s32_neg_s32_sat(*src);
         src++;
      }
   } else {
      for (i = 0; i < samples; ++i) {
         *dest++ = s32_mult_s32_s16_rnd_sat(*src, gain);
         src++;
      }
   }
}

// apply 16 bit Q15 gain to 32 bit input, then mix to 32 bit output
void buffer32_mix16(int32 *dest, int32 *src, int16 gain, int32 samples)
{
   int32 i, gain32, temp;

   if (Q15_ONE == gain) {
      for (i = 0; i < samples; ++i) {
         temp = s32_add_s32_s32_sat(*dest, *src);
         src++;
         *dest++ = temp;
      }
   } else if (Q15_MINUSONE == gain) {
      for (i = 0; i < samples; ++i) {
         temp = s32_sub_s32_s32_sat(*dest, *src);
         src++;
         *dest++ = temp;
      }
   } else {
      gain32 = s32_shl_s32_sat((int32)gain, 16);
      for (i = 0; i < samples; ++i) {
         temp = s32_mac_s32_s32_s1_sat(*dest, *src, gain32);
         src++;
         *dest++ = temp;
      }
   }
}

// apply 32 bit gain to 16 bit input, then write to 16 bit output
void buffer16_fill32(int16 *dest, int16 *src, int32 gain, int16 q_factor, int32 samples)
{
   int32 i;
   int16 shift = s16_sub_s16_s16(32, q_factor);
   int32 p5 = 0;

   if (shift <= 31){
	   p5 = (int32)(1 << (31 - shift));
   } else
   {
	   p5 = 0;
   }
   for (i = 0; i < samples; ++i) {
      *dest = s16_saturate_s32(s32_saturate_s64(
         s64_shl_s64(s64_add_s64_s32(
		 s64_mult_s32_s32((int32)(*src), gain), p5), shift - 32)));
      src++;
      dest++;
   }
}

// apply 16 bit gain with Q factor to 16 bit input, then write to 32 bit output
void buffer32_fill16src(int32 *dest, int16 *src, int16 gain, int16 q_factor, int32 samples)
{
    int32 i;
   int16 shift = s16_sub_s16_s16(16, q_factor);
   int32 unity = 1<<q_factor, tmpShiftL32 = 0;
   int64 tmpL64=0;

   if (gain == unity || gain == unity-1) {
      for (i = 0; i < samples; ++i) {
         *dest++ = (int32)*src++;
      }
   } else if (gain == -unity) {
      for (i = 0; i < samples; ++i) {
         *dest++ = s32_neg_s32_sat(*src);
         src++;
      }
   } else {
	   if(shift < 16)
			tmpShiftL32 = ((int32)1) << (15-shift);
      for (i = 0; i < samples; ++i) {
		  tmpL64 = s64_add_s64_s32(s64_mult_s32_s32(*src, (int32)gain), tmpShiftL32);
          *dest++ = s32_saturate_s64(s64_shl_s64(tmpL64,shift-16));
          src++;
		  //*dest++ = s32_saturate_s64(s64_mult_s32_s16_shift_rnd((int32)*src++,gain,shift));
      }
   }
}

// apply 16 bit gain to 16 bit src, then mix to 32 bit out
void buffer32_mix16src(int32 *dest, int16 *src, int16 gain, int16 q_factor, int32 samples)
{
   int32 i, tmp32;
   int16 shift = s16_sub_s16_s16(16, q_factor);
   int32 unity = 1<<q_factor, tmpShiftL32 = 0;
   int64 tmpL64=0;

   if (gain == unity || gain == unity-1) {
      for (i = 0; i < samples; ++i) {
         *dest = s32_add_s32_s32_sat(*dest, (int32)*src);
         src++;
         dest++;
      }
   } else if (gain == -unity) {
      for (i = 0; i < samples; ++i) {
         *dest = s32_sub_s32_s32_sat(*dest, (int32)*src);
         src++;
         dest++;
      }
   } else {
	   if(shift < 16)
			tmpShiftL32 = ((int32)1) << (15-shift);
      for (i = 0; i < samples; ++i) {
		  tmpL64 = s64_add_s64_s32(s64_mult_s32_s32(*src, (int32)gain), tmpShiftL32);
         tmp32 = s32_saturate_s64(s64_shl_s64(tmpL64,shift-16));
         *dest = s32_add_s32_s32_sat(*dest, tmp32);
         dest++;
         src++;
      }
   }
}

// apply Q15 smooth panner to 32 bit input and write into 32 bit output
void buffer32_fill_panner(int32 *dest, int32 *src, pannerStruct *panner, int32 samples)
{
   int16 target_q15  = panner->targetGainL16Q15;
   int32 pan_samples = panner->sampleCounter;
   int32 delta_q31   = panner->deltaL32Q31;
   int16 gain_q15;
   int32 gain_q31, ramp_samples, i;

   // assuming panner has no delay of execution
   gain_q15 = panner_get_current(*panner);
   gain_q31 = s32_shl_s32_sat((int32)gain_q15, 16);

   ramp_samples = s32_min_s32_s32(pan_samples, samples);

   for (i = 0; i < ramp_samples; ++i) {
      // get q15 gain
      gain_q15 = s16_extract_s32_h(gain_q31);
      // apply gain and output
      *dest++ = s32_mult_s32_s16_rnd_sat(*src, gain_q15);
      src++;
      // add delta to q31 gain
      gain_q31 = s32_add_s32_s32_sat(gain_q31, delta_q31);
   }
   samples -= ramp_samples;
   pan_samples -= ramp_samples;

   if (samples > 0) {
      for (i = 0; i < samples; ++i) {
         *dest++ = s32_mult_s32_s16_rnd_sat(*src, target_q15);
         src++;
      }
   }

   panner->sampleCounter = pan_samples;
}

// apply Q15 smooth panner to 32 bit input and mix into 32 bit output
void buffer32_mix_panner(int32 *dest, int32 *src, pannerStruct *panner, int32 samples)
{
   int32 target_q31;
   int32 pan_samples = panner->sampleCounter;
   int32 delta_q31   = panner->deltaL32Q31;
   int16 gain_q15;
   int32 gain_q31, ramp_samples, i;

   // assuming panner has no delay of execution
   gain_q15 = panner_get_current(*panner);
   gain_q31 = s32_shl_s32_sat((int32)gain_q15, 16);
   target_q31 = s32_shl_s32_sat((int32)panner->targetGainL16Q15, 16);

   ramp_samples = s32_min_s32_s32(pan_samples, samples);

   for (i = 0; i < ramp_samples; ++i) {
	  *dest = s32_mac_s32_s32_s1_rnd_sat(*dest, *src, gain_q31);
      dest++;
      src++;
      // add delta to q31 gain
      gain_q31 = s32_add_s32_s32_sat(gain_q31, delta_q31);
   }
   samples -= ramp_samples;
   pan_samples -= ramp_samples;

   for (i = 0; i < samples; ++i) {
		 *dest = s32_mac_s32_s32_s1_rnd_sat(*dest, *src, target_q31);
         dest++;
         src++;
   }

   panner->sampleCounter = pan_samples;
}

// smoothly crossfade src1 into src2 with Q15 smooth panner, results saved in output buf
void buffer32_crossmix_panner(int32 *dest, int32 *src1, int32 *src2, pannerStruct *panner, int32 samples)
{
   int16 target_q15  = panner->targetGainL16Q15;
   int32 pan_samples = panner->sampleCounter;
   int32 delta_q31   = panner->deltaL32Q31;
   int16 gain_q15;
   int32 gain_q31, ramp_samples, i, tmp32, temp;

   // current implementation totally ignores delay of panner
   // src1 gets applied gain, and src2 is applied 1-gain
   gain_q15 = panner_get_current(*panner);
   gain_q31 = s32_shl_s32_sat((int32)gain_q15, 16);

   ramp_samples = s32_min_s32_s32(pan_samples, samples);

   // process samples with dynamic gains
   for (i = 0; i < ramp_samples; ++i) {
      // get q15 gain
      gain_q15 = s16_extract_s32_h(gain_q31);
      // apply gain to src 2 and save
      *dest = s32_mult_s32_s16_rnd_sat(*src2, gain_q15);
      src2++;
      // get 1-gain and apply to src 1 and mix
      gain_q15 = s16_sub_s16_s16_sat(Q15_ONE, gain_q15);
      tmp32 = s32_mult_s32_s16_rnd_sat(*src1, gain_q15);
      src1++;
      temp = s32_add_s32_s32_sat(*dest, tmp32);
      *dest++ = temp;
      // add delta to q31 gain
      gain_q31 = s32_add_s32_s32_sat(gain_q31, delta_q31);
   }
   pan_samples -= ramp_samples;

   // process samples with no gain change
   samples -= ramp_samples;
   if (samples > 0) {
      if (Q15_ONE == target_q15) {
         buffer32_copy_v2(dest, src2, samples);
      } 
      else if (0 == target_q15) {
         buffer32_copy_v2(dest, src1, samples);
      } 
      else {
         gain_q15 = s16_sub_s16_s16_sat(Q15_ONE, target_q15);
         buffer32_fill16(dest, src2, target_q15, samples);
         buffer32_mix16(dest, src1, gain_q15, samples);
      }
   }

   panner->sampleCounter = pan_samples;
}
