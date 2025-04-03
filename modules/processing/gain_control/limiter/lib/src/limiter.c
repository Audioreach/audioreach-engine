/*============================================================================
  @file limiter.c

  Low distortion Limiter Implementation, including zero-crossing algorithm,
  and peak-history algorithm


        Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
        SPDX-License-Identifier: BSD-3-Clause
============================================================================*/

/*----------------------------------------------------------------------------
   Include Files
----------------------------------------------------------------------------*/
#include "limiter.h"
#include "audio_divide_qx.h"
#include "audio_dsp32.h"
#include "simple_mm.h"
#include "audio_basic_op_ext.h"
#include "stringl.h"

#ifdef LIM_ASM
void process_delay_zc_asm(limiter_per_ch_t *per_ch_ptr, int32 *scratch32, int32 dly_smps_m1, int32 samples);
void process_delayless_zc_asm(limiter_per_ch_t *per_ch_ptr, int32 *scratch32, int32 samples);
void process_with_history_peak_asm(limiter_per_ch_t *per_ch_ptr,
                                   int32 *           scratch32,
                                   int32             dly_smps_m1,
                                   int32             samples,
                                   int32             q_factor);
void apply_makeup_gain_asm(limiter_private_t *obj_ptr, void **out_ptr, int32 samples, uint32_t ch);
#endif // LIM_ASM

/*----------------------------------------------------------------------------
   Local Functions
----------------------------------------------------------------------------*/
int32 ms_to_sample_lim(int32 period, int32 sampling_rate)
{
   int32 samples;

   samples = s32_saturate_s64(s64_mult_s32_s32_shift(c_1_over_1000_q31, sampling_rate, 16)); // Q15
   samples = s32_saturate_s64(s64_mult_s32_s32_shift(samples, period, 17));                  // Q0
   return samples;
}

/* reset internal memories */
static void reset(limiter_private_t *obj_ptr)
{
   int32             ch;
   limiter_per_ch_t *per_ch_ptr;
   int32             delay_samples = obj_ptr->dly_smps_m1 + 1;

   obj_ptr->bypass_smps = ms_to_sample_lim(BYPASS_TRANSITION_PERIOD_MSEC, obj_ptr->static_vars.sample_rate);
   for (ch = 0; ch < obj_ptr->static_vars.num_chs; ++ch)
   {
      per_ch_ptr = &obj_ptr->per_ch[ch];

      // init the de-compressed limiter gain buffer
      per_ch_ptr->gain_q27        = c_gain_unity; // unity gain (Q27)
      per_ch_ptr->target_gain_q27 = c_gain_unity; // unity gain (Q27)

      // init static members of the data structure
      per_ch_ptr->cur_idx         = 0; // init current operating index
      per_ch_ptr->peak_subbuf_idx = 0; // init current peak buffer index
      per_ch_ptr->prev_peak_idx   = 0; // init the previous index
      per_ch_ptr->local_max_peak  = 0; // init the local maxima peak
      per_ch_ptr->global_peak     = 0; // init the global peak
      per_ch_ptr->fade_flag       = 0; // init fade flag
      per_ch_ptr->prev_zc_idx     = 0;
      per_ch_ptr->prev_sample_l32 = 0;
      per_ch_ptr->fade_flag_prev  = 0; // init previous fade flag

      // clean buffer memories
      buffer32_empty_v2(per_ch_ptr->delay_buf, delay_samples);

      if (obj_ptr->static_vars.history_winlen > 0)
      {
         buffer32_empty_v2(per_ch_ptr->history_peak_buf, c_local_peak_bufsize);
      }
      else
      {
         buffer32_empty_v2(per_ch_ptr->zc_buf, delay_samples);
      }
   }

   buffer32_empty_v2(obj_ptr->scratch_buf, obj_ptr->static_vars.max_block_size);
}

/* convert tuning parameter into internal vars */
static void convert_tuning_params(limiter_private_t *obj_ptr, int32 ch)
{
   int32                accu32;
   int16                tmp16;
   limiter_tuning_v2_t *tuning_ptr = &obj_ptr->per_ch[ch].tuning_params;
   int32                q_factor   = obj_ptr->static_vars.q_factor;

   // convert threshold to be aligned with input q-factor
   int32 default_qfactor = (obj_ptr->static_vars.data_width == 16 ? 15 : 27);

   obj_ptr->per_ch[ch].shifted_threshold =
      s32_shl_s32_sat(tuning_ptr->threshold, (int16)q_factor - (int16)default_qfactor);
   obj_ptr->per_ch[ch].shifted_hard_threshold =
      s32_shl_s32_sat(tuning_ptr->hard_threshold, (int16)q_factor - (int16)default_qfactor);

   // convert and calculatge max wait sample - value in static mem
   if (obj_ptr->static_vars.history_winlen == 0)
   { // process using zc, without history peak buffer
      tmp16 = s16_saturate_s32(tuning_ptr->max_wait);
   }
   else
   { // process with history peak buffer
      // history subbuf length = (history window length)/4 + 1
      // peak buffer len = 4
      tmp16 = s16_saturate_s32(s32_add_s32_s32_sat(obj_ptr->static_vars.history_winlen, 0x0004) >> 2);
   }
   accu32                               = s32_mult_s32_s16_rnd_sat(obj_ptr->static_vars.sample_rate, tmp16);
   obj_ptr->per_ch[ch].max_wait_smps_m1 = s16_saturate_s32(accu32) - 1;

   // left shift the make up gain by (7+1) to Q7.16 before multiplication with the data
   obj_ptr->per_ch[ch].makeup_gain_q16 = s32_shl_s32_sat(tuning_ptr->makeup_gain, 8);
}

/* set default values for limiter */
static void set_default(limiter_private_t *obj_ptr)
{
   int16                ch;
   limiter_tuning_v2_t *tuning_ptr;
   int32                q_factor             = obj_ptr->static_vars.q_factor;
   int32                default_threshold_32 = 93945856;
   int32                default_threshold_16 = 22936;

   for (ch = 0; ch < obj_ptr->static_vars.num_chs; ++ch)
   {
      tuning_ptr = &obj_ptr->per_ch[ch].tuning_params;

      tuning_ptr->ch_idx = ch;
      if (32 == obj_ptr->static_vars.data_width)
      {
         // threshold: -3 dB (Q27, 0.7)
         tuning_ptr->threshold = s32_shl_s32_sat(default_threshold_32, (int16)q_factor - 27);
         // threshold: -3 dB (Q27, 0.7)
         tuning_ptr->hard_threshold = s32_shl_s32_sat(default_threshold_32, (int16)q_factor - 27);
      }
      else
      { // 16
         // threshold: -3 dB (Q15, 0.7)
         tuning_ptr->threshold = s32_shl_s32_sat(default_threshold_16, (int16)q_factor - 15);
         // threshold: -3 dB (Q15, 0.7)
         tuning_ptr->hard_threshold = s32_shl_s32_sat(default_threshold_16, (int16)q_factor - 15);
      }

      tuning_ptr->makeup_gain = 256;   // unity  (Q8)
      tuning_ptr->gc          = 32440; // gain recovery coeff (Q15, 0.99)
      tuning_ptr->max_wait    = 82;    // max wait (Q15 sec, 0.0025)

      tuning_ptr->gain_attack  = 188099735; // gain attack time constant (Q31)
      tuning_ptr->gain_release = 32559427;  // gain release time constant (Q31)
      tuning_ptr->attack_coef  = 32768;     // unity attack time constant coef (Q15)
      tuning_ptr->release_coef = 32768;     // unity release time constant coef (Q15)

      convert_tuning_params(obj_ptr, ch);
   }

   // default to normal processing mode and no bypass
   obj_ptr->mode   = NORMAL_PROC;
   obj_ptr->bypass = 0;

   // flush memory
   reset(obj_ptr);
}

int32 search_global_peak(int32 *history_peak_buf, int32 num_history_peaks)
{
   int32 global_peak = history_peak_buf[0];
   int32 i;

   for (i = 1; i < num_history_peaks; i++)
   {
      if (global_peak < history_peak_buf[i])
      {
         global_peak = history_peak_buf[i];
      }
   }
   return global_peak;
}

#ifndef LIM_ASM
/*----------------------------------------------------------------------------
Apply gain smoothing logic to smooth the gain, so that the gain will achieve
the target gain within pre-defined time constant.

Use a look ahead buffer (limiter delay) to detect the ¡®local peak¡¯, so that
the gain can be gradually adapted to the target value before the local peak.

Introduce a long peak window to track the ¡®global peak¡¯ during certain period,
and then decide the target gain using this global peak. In this way, the target
gain change/fluctuation will be less. This helps to achieve a smoother gain and
less distortion.
----------------------------------------------------------------------------*/
static void process_with_history_peak(limiter_per_ch_t *per_ch_ptr,
                                      int32 *           scratch32,
                                      int32             dly_smps_m1,
                                      int32             samples,
                                      int32             q_factor)
{
   int32  j, gp_change_flag = 0;
   int64  accu64;
   int32  inpL32, attn32, absL32, iq32 /*, prod32*/;
   int32  cur_idx, peak_subbuf_idx, prev_peak_idx, max_wait_smps_m1;
   int32  global_peak, local_max_peak, new_global_peak;
   int32  threshold, hard_thresh, gain_diff_q27;
   uint32 time_const, time_coef;
   int32  target_gain_q27, gain_q27;

   threshold   = per_ch_ptr->shifted_threshold;
   hard_thresh = per_ch_ptr->shifted_hard_threshold;

   max_wait_smps_m1 = per_ch_ptr->max_wait_smps_m1;

   // Index for current sample in the zero-crossing buffer/input buffer
   cur_idx         = per_ch_ptr->cur_idx;
   peak_subbuf_idx = per_ch_ptr->peak_subbuf_idx;
   prev_peak_idx   = per_ch_ptr->prev_peak_idx;
   local_max_peak  = per_ch_ptr->local_max_peak;
   global_peak     = per_ch_ptr->global_peak;
   target_gain_q27 = per_ch_ptr->target_gain_q27;
   gain_q27        = per_ch_ptr->gain_q27;

   for (j = 0; j < samples; ++j)
   {

      // Extract and store the current input data
      inpL32 = scratch32[j];

      // Compute the absolute magnitude of the input
      absL32 = (int32)u32_abs_s32_sat(inpL32);

      // Compute the local maxima in the input audio
      local_max_peak = s32_max_s32_s32(local_max_peak, absL32);

      /************************************************************************
      Compute the global maxima - start
       *************************************************************************/
      gp_change_flag = 0;
      if (global_peak < local_max_peak)
      {
         global_peak    = local_max_peak;
         gp_change_flag = 1;
      }

      peak_subbuf_idx++;

      // If the 'max_wait_smps_m1' samples' local maxima is computed, store the
      // value into the peak history buffer, and re-compute the global peak
      if (peak_subbuf_idx > max_wait_smps_m1)
      {
         per_ch_ptr->history_peak_buf[prev_peak_idx] = local_max_peak;
         prev_peak_idx                               = s32_modwrap_s32_u32(prev_peak_idx + 1, c_local_peak_bufsize);
         local_max_peak                              = 0;
         peak_subbuf_idx                             = 0;

         new_global_peak = search_global_peak(per_ch_ptr->history_peak_buf, c_local_peak_bufsize);
         if (global_peak != new_global_peak)
         {
            global_peak    = new_global_peak;
            gp_change_flag = 1;
         }
      }

      // When the user wrongly set history window length < delay, this case will happen
      // Set global peak = abs(current sample) to avoid overshoot
      if ((int32)u32_abs_s32_sat(per_ch_ptr->delay_buf[cur_idx]) > global_peak)
      {
         global_peak    = (int32)u32_abs_s32_sat(per_ch_ptr->delay_buf[cur_idx]);
         gp_change_flag = 1;
      }

      if (gp_change_flag == 1)
      {
         if (global_peak > threshold)
         {
            // Use Q6 DSP's linear approximation division routine to lower down the MIPS
            // The inverse is computed with a normalized shift factor
            accu64 = dsplib_approx_divide(threshold, global_peak);
            attn32 = (int32)accu64;         // Extract the normalized inverse
            iq32   = (int32)(accu64 >> 32); // Extract the normalization shift factor

            // Shift the result to get the quotient in desired Q27 format
            target_gain_q27 = s32_shl_s32_sat(attn32, (int16)iq32 + 27);
         }
         else
         {
            target_gain_q27 = c_gain_unity;
         }
      }

      /************************************************************************
         Implementation of Limiter gain computation
       *************************************************************************/
      if (gain_q27 != target_gain_q27)
      { // do gain smoothing
         if (gain_q27 < target_gain_q27)
         {
            time_coef  = per_ch_ptr->tuning_params.release_coef;
            time_const = per_ch_ptr->tuning_params.gain_release;
         }
         else
         {
            time_coef  = per_ch_ptr->tuning_params.attack_coef;
            time_const = per_ch_ptr->tuning_params.gain_attack;
         }

         // accu64 = (1-coef)*abs(x) + coef
         accu64 = s64_mult_s32_s32_shift(s32_sub_s32_s32_sat(c_unity_q15, (int32)time_coef),
                                         absL32,
                                         32 - (int16)q_factor);                // Q15
         accu64 = s64_add_s32_s32(s32_saturate_s64(accu64), (int32)time_coef); // Q15

         // time_const = accu64 * gain_release, Q31
         time_const = (uint32)s64_mult_s32_u32_shift(s32_saturate_s64(accu64), time_const, 17);

         // limit the time_const uppper bound to be 1
         time_const = time_const > c_unity_q31 ? c_unity_q31 : time_const;

         // gain_q27	= gain_q27*(1-time_const) + target_gain_q27*time_const
         //			= gain_q27 + (target_gain_q27 - gain_q27)*time_const
         gain_diff_q27 = s32_sub_s32_s32_sat(target_gain_q27, gain_q27);
         gain_q27 =
            s32_add_s32_s32_sat(gain_q27, s32_saturate_s64(s64_mult_s32_u32_shift(gain_diff_q27, time_const, 1)));
      }

      /************************************************************************
         Implementation of Limiter gain on the input data
       *************************************************************************/
      // Gain application - Multiply and shift and round and sat (one cycle in Q6)
      if (dly_smps_m1 >= 0)
      { // if process with limiter delay
         scratch32[j] = s32_saturate_s64(
            s64_shl_s64(s64_add_s64_s64(s64_mult_s32_s32(per_ch_ptr->delay_buf[cur_idx], gain_q27), 0x4000000), -27));
         // Store the new input sample in the input buffer
         per_ch_ptr->delay_buf[cur_idx] = inpL32;

         cur_idx = s32_modwrap_s32_u32(cur_idx + 1, dly_smps_m1 + 1);
      }
      else
      { //  if process delayless
         scratch32[j] =
            s32_saturate_s64(s64_shl_s64(s64_add_s64_s64(s64_mult_s32_s32(inpL32, gain_q27), 0x4000000), -27));
      }

      /************************************************************************
         Apply hard-limiting if output exceeds hard threshold
       *************************************************************************/
      if (scratch32[j] > hard_thresh || scratch32[j] < -hard_thresh)
      {
         scratch32[j] = scratch32[j] > 0 ? hard_thresh : -hard_thresh;
         gain_q27     = target_gain_q27;
      }

   } /* for loop */

   per_ch_ptr->cur_idx          = cur_idx;
   per_ch_ptr->prev_peak_idx    = prev_peak_idx;
   per_ch_ptr->peak_subbuf_idx  = peak_subbuf_idx;
   per_ch_ptr->local_max_peak   = local_max_peak;
   per_ch_ptr->global_peak      = global_peak;
   per_ch_ptr->max_wait_smps_m1 = max_wait_smps_m1;
   per_ch_ptr->target_gain_q27  = target_gain_q27;
   per_ch_ptr->gain_q27         = gain_q27;

   return;
}

/*----------------------------------------------------------------------------
Detects zero crossing on the input signal and updates the limiter data
structure. The limiter alogorithm is based on Pei Xiang's investigation and
modification of a limiter technique based on updating gain at the zero-xing.

Dan Mapes-Riordan and W.Marshall Leach, "The Design of a digital signal peak
limiter for audio signal processing," Journal of Audio Engineering Society,
Vol. 36, No. 7/8, 1988 July/August.

Updates the limiter gain based on local maxima peak searching and zero-crossing
locations and applies the gain on the input data.
----------------------------------------------------------------------------*/
static void process_delay_zc(limiter_per_ch_t *per_ch_ptr, int32 *scratch32, int32 dly_smps_m1, int32 samples)
{
   int32                j;
   int64                accu64, prod64;
   int32                inpL32, accu32, attn32, absL32;
   int32                current_zc_buf_value, iq32;
   int32                cur_idx, prev_zc_idx, local_max_peak, prev_sample_l32;
   int32                threshold, gc;
   int32                gain_var_q27, gain_q27;
   limiter_tuning_v2_t *tuning_ptr = &per_ch_ptr->tuning_params;

   threshold = per_ch_ptr->shifted_threshold;
   gc        = tuning_ptr->gc;

   // Index for current sample in the zero-crossing buffer/input buffer
   cur_idx         = per_ch_ptr->cur_idx;
   prev_zc_idx     = per_ch_ptr->prev_zc_idx;
   local_max_peak  = per_ch_ptr->local_max_peak;
   prev_sample_l32 = per_ch_ptr->prev_sample_l32;

   gain_var_q27 = per_ch_ptr->gain_var_q27;
   gain_q27     = per_ch_ptr->gain_q27;

   for (j = 0; j < samples; ++j)
   {

      // Extract and store the current input data
      inpL32 = scratch32[j];

      // Compute the absolute magnitude of the input
      absL32 = (int32)u32_abs_s32_sat(inpL32);

      /************************************************************************
      Compute the zero-crossing locations and local maxima in the input audio - start
       *************************************************************************/
      current_zc_buf_value = per_ch_ptr->zc_buf[cur_idx];

      // Detect zero-crossing in the data
      prod64 = s64_mult_s32_s32(inpL32, prev_sample_l32);

      // Update the previous sample
      prev_sample_l32 = inpL32;

      /***********************************************************************
      Special case: No zero-crossings during the buffer duration.
      If max delayline is reached and we have to force a zero crossing.
      This is identified when the zcIndex reaches prev_zc_idx. By design,
      prev_zc_idx is always chasing zcIndex at any zc, so when zcIndex
      catches up with prev_zc_idx, it means zcIndex has searched more than
      "delay" times and still not finding a zc. we'll force write the current
      local max to this location for it to correctly update the gain afterwards.
       ************************************************************************/
      if (cur_idx == prev_zc_idx)
      {
         // Store tracked local peak max to zc location
         per_ch_ptr->zc_buf[cur_idx] = local_max_peak;

         current_zc_buf_value = local_max_peak;

         local_max_peak = absL32; // Reset local max value
      }
      // Zero-crossing condition: sample is exactly zero or it changes sign.
      // If it's exactly zero, the loop is entered twice, but it's okay.
      // Update the peak detection if zero-crossing is detected
      else if (prod64 <= 0)
      {
         // Update zero-crossing buffer to store local maximum.
         per_ch_ptr->zc_buf[prev_zc_idx] = local_max_peak;

         // Update previous zero-crossing index with the current buffer index
         prev_zc_idx = cur_idx;

         local_max_peak = absL32; // Reset local maximum.

         // Note that so far just prev_zc_idx points to this location,
         // Prepare for the next write to the location when next zc is
         // reached, but nothing has been written at this location in zcBuffer yet
      }
      else
      {

         // Mark non zero-crossing reference sample as zero.
         per_ch_ptr->zc_buf[cur_idx] = 0;

         // Continue searching if zero-crossing is not detected
         local_max_peak = s32_max_s32_s32(local_max_peak, absL32);
      }

      /************************************************************************
         Compute the limiter gain and update it at zero-crossings - start
       ************************************************************************/

      // Update the limiter gain if zero-crossing is detected
      if (current_zc_buf_value != 0)
      {
         // Compute the new gain as (1-GRC*gainVar) - March 2013 optimizations
         gain_var_q27 = s32_mult_s32_s16_rnd_sat(gain_var_q27, (int16)gc);

         // If peak is not above the threshold, do a gain release.
         // Release slowly using gain recovery coefficient (Q15 multiplication)
         // If peak is above the threshold, attack is required, then both gain_q15 and gain_var_q15
         // will be updated again with attenuations in gain_q15
         gain_q27 = s32_sub_s32_s32_sat(c_gain_unity, gain_var_q27);

         // Buffer data x gain - Multiply 32x16 round, shift and sat in one cycle
         accu32 = s32_saturate_s64(s64_mult_s32_s32_shift(current_zc_buf_value, gain_q27, 5));

         // If the peak in the data is above the threshold, attack the gain and reduce it.
         if (accu32 > threshold)
         {
            // Use Q6 DSP's linear approximation division routine to lower down the MIPS
            // The inverse is computed with a normalized shift factor
            accu64 = dsplib_approx_divide(threshold, current_zc_buf_value);
            attn32 = (int32)accu64;         // Extract the normalized inverse
            iq32   = (int32)(accu64 >> 32); // Extract the normalization shift factor

            // Shift the result to get the quotient in desired Q15 format
            gain_q27 = s32_shl_s32_sat(attn32, (int16)iq32 + 27);

            // Attack gain immediately if the peak overshoots the threshold
            gain_var_q27 = s32_sub_s32_s32_sat(c_gain_unity, gain_q27);
         }

      } /* if(per_ch_ptr->zc_buf32[cur_idx] != 0) */

      /************************************************************************
         Implementation of Limiter gain on the input data
       *************************************************************************/
      // Gain application - Multiply and shift and round and sat (one cycle in Q6)
      scratch32[j] = s32_saturate_s64(
         s64_shl_s64(s64_add_s64_s64(s64_mult_s32_s32(per_ch_ptr->delay_buf[cur_idx], gain_q27), 0x4000000), -27));

      // Store the new input sample in the input buffer
      per_ch_ptr->delay_buf[cur_idx] = inpL32;

      cur_idx = s32_modwrap_s32_u32(cur_idx + 1, dly_smps_m1 + 1);

   } /* for loop */

   per_ch_ptr->cur_idx         = cur_idx;
   per_ch_ptr->prev_zc_idx     = prev_zc_idx;
   per_ch_ptr->local_max_peak  = local_max_peak;
   per_ch_ptr->prev_sample_l32 = prev_sample_l32;
   per_ch_ptr->gain_var_q27    = gain_var_q27;
   per_ch_ptr->gain_q27        = gain_q27;

   return;
}

/*----------------------------------------------------------------------------
In delay-less implementation, the limiter gains are updated immediately if
the instantaneous peak exceeds the threshold. The gain is also updated if the
time interval between gain updates exceeds the specified wait time.
----------------------------------------------------------------------------*/
static void process_delayless_zc(limiter_per_ch_t *per_ch_ptr, int32 *scratch32, int32 samples)
{
   int                  j;
   int64                accu64, prod64;
   int32                accu32, attn32, inpL32, absL32;
   int16                gc;
   int32                attnQ27, iq32;
   int32                tmp32, gain_var_q27, gain_q27, new_gain_var_q27;
   int32                prev_sample_l32, cur_idx, max_wait_smps_m1;
   int32                threshold;
   limiter_tuning_v2_t *tuning_ptr = &per_ch_ptr->tuning_params;

   threshold = per_ch_ptr->shifted_threshold;

   gc               = (int16)(tuning_ptr->gc);
   max_wait_smps_m1 = per_ch_ptr->max_wait_smps_m1;

   prev_sample_l32 = per_ch_ptr->prev_sample_l32;
   cur_idx         = per_ch_ptr->cur_idx;
   gain_var_q27    = per_ch_ptr->gain_var_q27;
   gain_q27        = per_ch_ptr->gain_q27;

   for (j = 0; j < samples; ++j)
   {
      // Extract and store the current input data
      inpL32 = scratch32[j];

      // Compute the absolute magnitude of the input
      absL32 = (int32)u32_abs_s32_sat(inpL32);

      // Detect zero-crossing in the data
      prod64 = s64_mult_s32_s32(inpL32, prev_sample_l32);

      // Compute maximum of gain vs. previous instantaneous gain - March 2011 Changes
      // tmp16 = max(gain, 1-GRC*gainVar)
      new_gain_var_q27 = s32_mult_s32_s16_rnd_sat(gain_var_q27, gc);
      tmp32            = s32_sub_s32_s32_sat(c_gain_unity, new_gain_var_q27);

      // absolute input x gain - Multiply 32x16 round, shift and sat in one cycle
      accu32 = s32_saturate_s64(s64_mult_s32_s32_shift(absL32, tmp32, 5));

      // If the peak in the data is above the threshold, attack the gain immediately.
      if (accu32 > threshold)
      {

         // Use Q6 DSP's linear approximation division routine to lower down the MIPS
         // The inverse is computed with a normalized shift factor
         accu64 = dsplib_approx_divide(threshold, absL32);
         attn32 = (int32)accu64;         // Extract the normalized inverse
         iq32   = (int32)(accu64 >> 32); // Extract the normalization shift factor

         // Shift the result to get the quotient in desired Q15 format
         attnQ27 = s32_shl_s32_sat(attn32, (int16)iq32 + 27);

         // Attack gain immediately if the peak overshoots the threshold
         gain_var_q27 = s32_sub_s32_s32_sat(c_gain_unity, attnQ27);

         // Re-set the wait time index counter
         cur_idx = 0;
      }
      else if ((prod64 < 0) || (absL32 == 0) || (cur_idx > max_wait_smps_m1))
      {
         // Gain release at zero-crossings or if the wait time is exceeded.
         // Release slowly using gain recovery coefficient (Q15 multiplication)
         gain_var_q27 = new_gain_var_q27;

         // Re-set the wait time index counter
         cur_idx = 0;
      }

      // Update the gain at the zero-crossings (no saturation)
      gain_q27 = s32_sub_s32_s32(c_gain_unity, gain_var_q27);

      // Store previous sample in the memory
      prev_sample_l32 = inpL32;

      // Increment the wait-time counter index in a circular buffer fashion
      cur_idx++;

      /****************************************************************************
      Implementation of Limiter gain on the input data
       *****************************************************************************/
      // Gain application - Multiply and shift and round and sat (one cycle in Q6)
      scratch32[j] = s32_saturate_s64(s64_shl_s64(s64_add_s64_s64(s64_mult_s32_s32(inpL32, gain_q27), 0x4000000), -27));
   } // for loop for j

   per_ch_ptr->prev_sample_l32 = prev_sample_l32;
   per_ch_ptr->cur_idx         = cur_idx;
   per_ch_ptr->gain_var_q27    = gain_var_q27;
   per_ch_ptr->gain_q27        = gain_q27;

   return;
}

static void apply_makeup_gain(limiter_private_t *obj_ptr, void **out_ptr, int32 samples, uint32_t ch)
{
   int64                accu64;
   int32 *              out_ptr32;
   int16 *              out_ptr16;
   limiter_per_ch_t *   per_ch_ptr = &obj_ptr->per_ch[ch];
   limiter_tuning_v2_t *tuning_ptr = &per_ch_ptr->tuning_params;

   if (32 == obj_ptr->static_vars.data_width)
   {
      out_ptr32 = (int32 *)out_ptr[ch];
      if (c_mgain_unity == tuning_ptr->makeup_gain)
      {
         buffer32_copy_v2(out_ptr32, obj_ptr->scratch_buf, samples);
      }
      else
      {
         for (uint32_t j = 0; j < samples; ++j)
         {
            // Multiply output with the Q7.16 make-up gain
            accu64 = s64_add_s64_s32(s64_mult_s32_s32(obj_ptr->scratch_buf[j], per_ch_ptr->makeup_gain_q16), 0x8000);
            // Copy the audio output from the local buffer
            out_ptr32[j] = s32_saturate_s64(s64_shl_s64(accu64, -16));
         }
      }
   }
   else
   { // 16 bit
      out_ptr16 = (int16 *)out_ptr[ch];
      if (c_mgain_unity == tuning_ptr->makeup_gain)
      {
         for (uint32_t j = 0; j < samples; ++j)
         {
            out_ptr16[j] = s16_saturate_s32(obj_ptr->scratch_buf[j]);
         }
      }
      else
      {
         for (uint32_t j = 0; j < samples; ++j)
         {
            // Multiply 32bit output with the Q7.16 make-up gain, and rounding
            accu64 = s64_add_s64_s32(s64_mult_s32_s32(obj_ptr->scratch_buf[j], per_ch_ptr->makeup_gain_q16), 0x8000);
            // Copy the audio output from the local buffer
            out_ptr16[j] = s16_extract_s64_h_sat(accu64);
         }
      }
   } // end of 16 bit makeup gain
}

#endif // LIM_ASM

/*----------------------------------------------------------------------------
Pass data with just delay, no limiter processing. During transition from
limiter processing to no-limiter processing, fade out the existing limiter gain
gradually so as to minimize sudden discontinuities.
----------------------------------------------------------------------------*/
static void pass_data(limiter_per_ch_t *per_ch_ptr,
                      int32 *           scratch32,
                      int32             dly_smps_m1,
                      int32             history_winlen,
                      int32             samples)
{
   int   j;
   int32 inp32 = 0;
   int32 cur_idx;
   int32 gain_q27;

   // handling zero delay case
   if (dly_smps_m1 < 0)
   {
      // Perform gain release to bring the gain back to unity
      if (per_ch_ptr->gain_q27 < c_gain_unity)
      {
         per_ch_ptr->gain_var_q27 = s32_sub_s32_s32_sat(c_gain_unity, per_ch_ptr->gain_q27);

         // Release the gain variable gradually to smooth out discontinuities
         per_ch_ptr->gain_var_q27 = s32_mult_s32_s16_rnd_sat(per_ch_ptr->gain_var_q27, c_fade_grc);

         // Update the gain once every frame
         per_ch_ptr->gain_q27 = s32_sub_s32_s32(c_gain_unity, per_ch_ptr->gain_var_q27);

         for (j = 0; j < samples; j++)
         {
            // Apply limiter gain on the data
            // Gain application - (one cycle in Q6)
            //*scratch32++ = s32_saturate_s64(s64_mult_s32_s32_shift(*scratch32, per_ch_ptr->gain_q27, 5));
            scratch32[j] = s32_saturate_s64(s64_mult_s32_s32_shift(scratch32[j], per_ch_ptr->gain_q27, 5));
         }
      }
      else
      {
         // nop
      }
   }
   else
   {
      // Perform gain release to bring the gain back to unity

      if (per_ch_ptr->gain_q27 < c_gain_unity) // Perform gain release to bring the gain back to unity
      {
         cur_idx = per_ch_ptr->cur_idx;
         if (samples <= per_ch_ptr->bypass_sample_counter)
         {
            gain_q27 = per_ch_ptr->gain_q27;
            for (j = 0; j < samples; ++j)
            {
               // Extract and store the current input data
               inp32 = *scratch32;

               // Apply limiter gain on the data
               // Gain application - (one cycle in Q6)
               *scratch32++ = s32_saturate_s64(s64_mult_s32_s32_shift(per_ch_ptr->delay_buf[cur_idx], gain_q27, 5));

               // Update the input buffer
               per_ch_ptr->delay_buf[cur_idx] = inp32;

               // Increment the zero-crossing index for the circular buffer
               cur_idx = s32_modwrap_s32_u32(cur_idx + 1, dly_smps_m1 + 1);

               // Update temp gain
               gain_q27 = s32_add_s32_s32(gain_q27, per_ch_ptr->bypass_delta_gain_q27);
            }
            // Update global variables
            if (gain_q27 > c_gain_unity)
            {
               gain_q27 = c_gain_unity;
            }
            per_ch_ptr->gain_q27 = gain_q27;
            per_ch_ptr->bypass_sample_counter -= samples;
         }
         else
         {
            gain_q27 = per_ch_ptr->gain_q27;
            for (j = 0; j < per_ch_ptr->bypass_sample_counter; ++j)
            {
               // Extract and store the current input data
               inp32 = *scratch32;

               // Apply limiter gain on the data
               // Gain application - (one cycle in Q6)
               *scratch32++ = s32_saturate_s64(s64_mult_s32_s32_shift(per_ch_ptr->delay_buf[cur_idx], gain_q27, 5));

               // Update the input buffer
               per_ch_ptr->delay_buf[cur_idx] = inp32;

               // Increment the zero-crossing index for the circular buffer
               cur_idx = s32_modwrap_s32_u32(cur_idx + 1, dly_smps_m1 + 1);

               // Update temp gain
               gain_q27 = s32_add_s32_s32(gain_q27, per_ch_ptr->bypass_delta_gain_q27);
            }
            // Update global variables
            if (gain_q27 != c_gain_unity)
            {
               gain_q27 = c_gain_unity;
            }
            per_ch_ptr->gain_q27 = gain_q27;

            // handling the rest of transition samples
            for (j = 0; j < (samples - per_ch_ptr->bypass_sample_counter); ++j)
            {
               // Extract and store the current input data
               inp32 = *scratch32;

               // Apply limiter gain on the data
               // Gain application - (one cycle in Q6)
               //*scratch32++ = s32_saturate_s64(s64_mult_s32_s32_shift(per_ch_ptr->delay_buf[cur_idx],
               //per_ch_ptr->gain_q27, 5));

               // copy input to delay buffer; no gain is applied as the gain is unity.
               *scratch32++ = per_ch_ptr->delay_buf[cur_idx];

               // Update the input buffer
               per_ch_ptr->delay_buf[cur_idx] = inp32;

               // Increment the zero-crossing index for the circular buffer
               cur_idx = s32_modwrap_s32_u32(cur_idx + 1, dly_smps_m1 + 1);
            }
            // set the bypass_sample_counter 0
            per_ch_ptr->bypass_sample_counter = 0;
         }

         per_ch_ptr->cur_idx         = cur_idx; // save the index after the loop
         per_ch_ptr->prev_sample_l32 = inp32;
      }
      else
      {

         cur_idx = per_ch_ptr->cur_idx;

         // Pass the delayed buffer data to the output
         for (j = 0; j < samples; ++j)
         {
            // Extract and store the current input data
            inp32 = *scratch32;

            // Place the delay buffer data into the output
            *scratch32++ = per_ch_ptr->delay_buf[cur_idx];

            // Update the input buffer
            per_ch_ptr->delay_buf[cur_idx] = inp32;

            // Increment the zero-crossing index for the circular buffer
            cur_idx = s32_modwrap_s32_u32(cur_idx + 1, dly_smps_m1 + 1);
         }

         per_ch_ptr->cur_idx         = cur_idx; // save the index after the loop
         per_ch_ptr->prev_sample_l32 = inp32;
      }
   }
   // Do one-time initialization of the zero-crossing buffers and memory variables
   if (per_ch_ptr->bypass_sample_counter > 0)
   {
      per_ch_ptr->fade_flag = 1;
   }
   else
   {
      per_ch_ptr->fade_flag = 0;
   }

   if ((0 == per_ch_ptr->fade_flag) && (1 == per_ch_ptr->fade_flag_prev))
   {
      // Reset the history peak buffer
      if (history_winlen > 0)
      {
         buffer32_empty_v2(per_ch_ptr->history_peak_buf, c_local_peak_bufsize);
         // Reset the local max peak
         per_ch_ptr->local_max_peak = (int32)0x0001;

         // Reset the global peak
         per_ch_ptr->global_peak = (int32)0x0001;

         per_ch_ptr->peak_subbuf_idx = 0;            // init current peak buffer index
         per_ch_ptr->prev_peak_idx   = 0;            // init the previous index
         per_ch_ptr->target_gain_q27 = c_gain_unity; // unity gain (Q27)
      }
      else if (dly_smps_m1 > 0)
      {
         // Reset the zero-crossing buffer
         buffer32_empty_v2(per_ch_ptr->zc_buf, dly_smps_m1 + 1);

         // Reset the previous sample
         // per_ch_ptr->prev_sample_l32 = 0;
         // per_ch_ptr->prev_sample_l32 = (int32)0x7FFF;

         per_ch_ptr->prev_zc_idx  = 0;
         per_ch_ptr->gain_var_q27 = 0;

         per_ch_ptr->local_max_peak = 0;
      }
      else
      {
         per_ch_ptr->gain_var_q27 = 0;
      }
      per_ch_ptr->gain_q27 = c_gain_unity; // unity gain (Q27)
   }
   // update the previous fade flag
   per_ch_ptr->fade_flag_prev = per_ch_ptr->fade_flag;
}

/*----------------------------------------------------------------------------
   API Functions
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
// ** Processing one block of samples
// Description: Process multi-channel input audio sample by sample and limit
                the input to specified threshold level. The input can be in
                any sampling rate: 8, 16, 22.05, 32, 44.1, 48, 96, 192 KHz.
                The input is 32-bit Q23 and  the output is 32-bit Q23.
                Implements zero-crossing update based limiter to limit the
                input audio signal. The process function separates out delayed
                and delay-less implementation.
// lib_ptr: [in] pointer to library structure
// out_ptr: [out] pointer to output sample block (multi channel double ptr)
// in_ptr: [in] pointer to input sample block (multi channel double ptr)
// samples: [in] number of samples to be processed per channel
----------------------------------------------------------------------------*/
LIMITER_RESULT limiter_process(limiter_lib_t *lib_ptr, void **out_ptr, int32 **in_ptr, uint32 api_samples)
{
   limiter_private_t *obj_ptr = (limiter_private_t *)lib_ptr->mem_ptr;
   limiter_per_ch_t * per_ch_ptr;
   int32              ch;
   int32              samples = (int32)api_samples;

   /* check if block size is within range */
   if (samples > obj_ptr->static_vars.max_block_size)
   {
      return LIMITER_FAILURE;
   }

   /* process per channel */
   for (ch = 0; ch < obj_ptr->static_vars.num_chs; ++ch)
   {
      per_ch_ptr = &obj_ptr->per_ch[ch];

      // copy input into scratch buffer
      buffer32_copy_v2(obj_ptr->scratch_buf, in_ptr[ch], samples);

      // proc according to limiter processing modes
      if (NORMAL_PROC == obj_ptr->mode)
      {
         /* when switch from non-bypass to bypass, a fade flag is set */ // move to set_param
         if (1 == obj_ptr->bypass)
         {
            /* no limiting but apply delay only & handling transition */
            pass_data(per_ch_ptr,
                      obj_ptr->scratch_buf,
                      obj_ptr->dly_smps_m1,
                      obj_ptr->static_vars.history_winlen,
                      samples);
         }
#ifdef LIM_ASM
         else if (obj_ptr->static_vars.history_winlen > 0)
         {
            process_with_history_peak_asm(per_ch_ptr,
                                          obj_ptr->scratch_buf,
                                          obj_ptr->dly_smps_m1,
                                          samples,
                                          obj_ptr->static_vars.q_factor);
         }
         else if (obj_ptr->dly_smps_m1 >= 0)
         {
            /* zc limiter with delay */
            process_delay_zc_asm(per_ch_ptr, obj_ptr->scratch_buf, obj_ptr->dly_smps_m1, samples);
         }
         else
         {
            /* delayless zc limiter implementation */
            process_delayless_zc_asm(per_ch_ptr, obj_ptr->scratch_buf, samples);
         }
      } // if ( NORMAL_PROC )

      // apply makeup gain
      apply_makeup_gain_asm(obj_ptr, out_ptr, samples, ch);
#else  // LIM_ASM
         else if (obj_ptr->static_vars.history_winlen > 0)
         {
            process_with_history_peak(per_ch_ptr,
                                      obj_ptr->scratch_buf,
                                      obj_ptr->dly_smps_m1,
                                      samples,
                                      obj_ptr->static_vars.q_factor);
         }
         else if (obj_ptr->dly_smps_m1 >= 0)
         {
            /* zc limiter with delay */
            process_delay_zc(per_ch_ptr, obj_ptr->scratch_buf, obj_ptr->dly_smps_m1, samples);
         }
         else
         {
            /* delayless zc limiter implementation */
            process_delayless_zc(per_ch_ptr, obj_ptr->scratch_buf, samples);
         }
      } // if ( NORMAL_PROC )

      // apply makeup gain
      apply_makeup_gain(obj_ptr, out_ptr, samples, ch);
#endif // LIM_ASM
   }   /* ch index loop */

   return LIMITER_SUCCESS;
}

/*----------------------------------------------------------------------------
// ** Get library memory requirements
// mem_req_ptr: [out] pointer to mem requirements structure
// limiter_static_vars_t: [in] pointer to static variable structure
----------------------------------------------------------------------------*/
LIMITER_RESULT limiter_get_mem_req(limiter_mem_req_t *mem_req_ptr, limiter_static_vars_t *static_vars_ptr)
{
   uint32 reqsize;
   int32  ch, delay_samples;
   int32  sample_rate    = static_vars_ptr->sample_rate;
   int32  max_block_size = static_vars_ptr->max_block_size;
   int32  num_chs        = static_vars_ptr->num_chs;

   reqsize = smm_malloc_size(sizeof(limiter_private_t));

   // alloc lim data struct according to num of channels
   reqsize += smm_calloc_size(num_chs, sizeof(limiter_per_ch_t));

   // per channel calc
   delay_samples = s16_saturate_s32(s32_mult_s32_s16_rnd_sat(sample_rate, s16_saturate_s32(static_vars_ptr->delay)));

   for (ch = 0; ch < num_chs; ++ch)
   {
      reqsize += smm_calloc_size(delay_samples, sizeof(int32)); // delay buf
      reqsize += smm_calloc_size(delay_samples, sizeof(int32)); // zerocrossing buf
   }

   // scratch buf
   reqsize += smm_calloc_size(max_block_size, sizeof(int32));

   // assign mem sizes
   mem_req_ptr->mem_size   = reqsize;
   mem_req_ptr->stack_size = c_limiter_max_stack_size;

   return LIMITER_SUCCESS;
}

/*----------------------------------------------------------------------------
// ** Get library memory requirements
// mem_req_ptr: [out] pointer to mem requirements structure
// limiter_static_vars_v2_t: [in] pointer to static variable structure v2
----------------------------------------------------------------------------*/
LIMITER_RESULT limiter_get_mem_req_v2(limiter_mem_req_t *mem_req_ptr, limiter_static_vars_v2_t *static_vars_v2_ptr)
{
   uint32 reqsize;
   int32  ch, delay_samples;
   int32  sample_rate    = static_vars_v2_ptr->sample_rate;
   int32  max_block_size = static_vars_v2_ptr->max_block_size;
   int32  num_chs        = static_vars_v2_ptr->num_chs;
   int32  history_winlen = static_vars_v2_ptr->history_winlen;

   reqsize = smm_malloc_size(sizeof(limiter_private_t));

   // alloc lim data struct according to num of channels
   reqsize += smm_calloc_size(num_chs, sizeof(limiter_per_ch_t));

   // per channel calc
   delay_samples = s16_saturate_s32(s32_mult_s32_s16_rnd_sat(sample_rate, s16_saturate_s32(static_vars_v2_ptr->delay)));

   for (ch = 0; ch < num_chs; ++ch)
   {
      reqsize += smm_calloc_size(delay_samples, sizeof(int32)); // delay buf
      if (history_winlen > 0)
      {
         reqsize += smm_calloc_size(c_local_peak_bufsize, sizeof(int32)); // history peak buf
      }

      reqsize += smm_calloc_size(delay_samples, sizeof(int32)); // zerocrossing buf
   }

   // scratch buf
   reqsize += smm_calloc_size(max_block_size, sizeof(int32));

   // assign mem sizes
   mem_req_ptr->mem_size   = reqsize;
   mem_req_ptr->stack_size = c_limiter_max_stack_size;

   return LIMITER_SUCCESS;
}

/*----------------------------------------------------------------------------
// ** Partition, initialize memory, and set default values
// lib_ptr: [in, out] pointer to library structure
// static_vars_ptr: [in] pointer to static variable structure
// mem_ptr: [in] pointer to allocated memory
// mem_size: [in] size of memory allocated
----------------------------------------------------------------------------*/
LIMITER_RESULT limiter_init_mem(limiter_lib_t *        lib_ptr,
                                limiter_static_vars_t *static_vars_ptr,
                                void *                 mem_ptr,
                                uint32                 mem_size)
{
   SimpleMemMgr       MemMgr;
   SimpleMemMgr *     smm = &MemMgr;
   limiter_mem_req_t  mem_req;
   limiter_private_t *obj_ptr;
   limiter_per_ch_t * per_ch_ptr;
   int32              num_chs        = static_vars_ptr->num_chs;
   int32              data_width     = static_vars_ptr->data_width;
   int32              sample_rate    = static_vars_ptr->sample_rate;
   int32              max_block_size = static_vars_ptr->max_block_size;
   int32              delay_samples, ch;

   /* double check if allocated mem is enough */
   limiter_get_mem_req(&mem_req, static_vars_ptr);
   if (mem_req.mem_size > mem_size)
   {
      return LIMITER_MEMERROR;
   }

   /* assign initial mem pointers and allocate current instance */
   lib_ptr->mem_ptr = mem_ptr;
   smm_init(smm, lib_ptr->mem_ptr);
   obj_ptr = (limiter_private_t *)smm_malloc(smm, sizeof(limiter_private_t));

   /* keep a copy of static vars */
   memscpy(&obj_ptr->static_vars, sizeof(limiter_static_vars_t), static_vars_ptr, sizeof(limiter_static_vars_t));
   obj_ptr->static_vars.q_factor       = data_width == 16 ? 15 : 27;
   obj_ptr->static_vars.history_winlen = 0;

   /* calculate delay samples */
   delay_samples = s16_saturate_s32(s32_mult_s32_s16_rnd_sat(sample_rate, s16_saturate_s32(static_vars_ptr->delay)));
   obj_ptr->dly_smps_m1 = delay_samples - 1;
   obj_ptr->bypass_smps = ms_to_sample_lim(BYPASS_TRANSITION_PERIOD_MSEC, static_vars_ptr->sample_rate);

   /* alloc per ch struct array */
   obj_ptr->per_ch = (limiter_per_ch_t *)smm_calloc(smm, num_chs, sizeof(limiter_per_ch_t));
   for (ch = 0; ch < num_chs; ++ch)
   {
      per_ch_ptr            = &obj_ptr->per_ch[ch];
      per_ch_ptr->zc_buf    = (int32 *)smm_calloc(smm, delay_samples, sizeof(int32)); // zc buf
      per_ch_ptr->delay_buf = (int32 *)smm_calloc(smm, delay_samples, sizeof(int32)); // delay buf
   }

   /* alloc scratch buffer */
   obj_ptr->scratch_buf = (int32 *)smm_calloc(smm, max_block_size, sizeof(int32));

   /* set default values */
   set_default(obj_ptr);

   return LIMITER_SUCCESS;
}

/*----------------------------------------------------------------------------
// ** Partition, initialize memory, and set default values
// lib_ptr: [in, out] pointer to library structure
// static_vars_v2_ptr: [in] pointer to static variable structure v2
// mem_ptr: [in] pointer to allocated memory
// mem_size: [in] size of memory allocated
----------------------------------------------------------------------------*/
LIMITER_RESULT limiter_init_mem_v2(limiter_lib_t *           lib_ptr,
                                   limiter_static_vars_v2_t *static_vars_v2_ptr,
                                   void *                    mem_ptr,
                                   uint32                    mem_size)
{
   SimpleMemMgr       MemMgr;
   SimpleMemMgr *     smm = &MemMgr;
   limiter_mem_req_t  mem_req;
   limiter_private_t *obj_ptr;
   limiter_per_ch_t * per_ch_ptr;
   int32              num_chs        = static_vars_v2_ptr->num_chs;
   int32              sample_rate    = static_vars_v2_ptr->sample_rate;
   int32              max_block_size = static_vars_v2_ptr->max_block_size;
   int32              history_winlen = static_vars_v2_ptr->history_winlen;
   int32              delay_samples, ch;

   /* double check if allocated mem is enough */
   limiter_get_mem_req_v2(&mem_req, static_vars_v2_ptr);
   if (mem_req.mem_size > mem_size)
   {
      return LIMITER_MEMERROR;
   }

   /* assign initial mem pointers and allocate current instance */
   lib_ptr->mem_ptr = mem_ptr;
   smm_init(smm, lib_ptr->mem_ptr);
   obj_ptr = (limiter_private_t *)smm_malloc(smm, sizeof(limiter_private_t));

   /* keep a copy of static vars */
   obj_ptr->static_vars = *static_vars_v2_ptr;

   /* calculate delay samples */
   delay_samples = s16_saturate_s32(s32_mult_s32_s16_rnd_sat(sample_rate, s16_saturate_s32(static_vars_v2_ptr->delay)));
   obj_ptr->dly_smps_m1 = delay_samples - 1;
   obj_ptr->bypass_smps = ms_to_sample_lim(BYPASS_TRANSITION_PERIOD_MSEC, static_vars_v2_ptr->sample_rate);

   /* alloc per ch struct array */
   obj_ptr->per_ch = (limiter_per_ch_t *)smm_calloc(smm, num_chs, sizeof(limiter_per_ch_t));
   for (ch = 0; ch < num_chs; ++ch)
   {
      per_ch_ptr = &obj_ptr->per_ch[ch];
      if (history_winlen > 0)
      {
         per_ch_ptr->history_peak_buf =
            (int32 *)smm_calloc(smm, c_local_peak_bufsize, sizeof(int32)); // history peak buf
      }

      per_ch_ptr->zc_buf    = (int32 *)smm_calloc(smm, delay_samples, sizeof(int32)); // zc buf
      per_ch_ptr->delay_buf = (int32 *)smm_calloc(smm, delay_samples, sizeof(int32)); // delay buf
   }

   /* alloc scratch buffer */
   obj_ptr->scratch_buf = (int32 *)smm_calloc(smm, max_block_size, sizeof(int32));

   /* set default values */
   set_default(obj_ptr);

   return LIMITER_SUCCESS;
}

/*----------------------------------------------------------------------------
// ** Set parameters to library
// lib_ptr: [in, out] pointer to lib structure
// param_id: [in] parameter id
// param_ptr: [in] pointer to the memory where the new values are stored
// param_size:[in] size of the memory pointed by param_ptr
----------------------------------------------------------------------------*/
LIMITER_RESULT limiter_set_param(limiter_lib_t *lib_ptr, uint32 param_id, void *mem_ptr, uint32 mem_size)
{
   limiter_private_t *  obj_ptr = (limiter_private_t *)lib_ptr->mem_ptr;
   limiter_tuning_t *   tuning_ptr;
   limiter_tuning_v2_t *tuning_v2_ptr;
   limiter_mode_t       new_mode;
   limiter_bypass_t     new_bypass;

   switch (param_id)
   {

      case LIMITER_PARAM_MODE:
         // set limiter modes
         //    MAKEUPGAIN_ONLY      : apply makeup gain only, and no delay
         //    NORMAL_PROC          : normal limiter processing
         if (mem_size == sizeof(limiter_mode_t))
         {
            new_mode = *((limiter_mode_t *)mem_ptr);

            // sanity check of limiter mode
            if ((MAKEUPGAIN_ONLY != new_mode) && (NORMAL_PROC != new_mode))
            {
               return LIMITER_FAILURE;
            }
            // since delay changes between modes, need to flush mem
            if (obj_ptr->mode != new_mode)
            {
               obj_ptr->mode = new_mode;
               reset(obj_ptr);
            }
         }
         else
         {
            return LIMITER_MEMERROR; // parameter size doesn't match
         }
         break;

      case LIMITER_PARAM_BYPASS:
         if (mem_size == sizeof(limiter_bypass_t))
         {
            int32 ch;
            int32 delta_gain;

            new_bypass = *((limiter_bypass_t *)mem_ptr);
            if (new_bypass != 0)
            {
               new_bypass = 1;
            }
            if (obj_ptr->bypass != new_bypass)
            {
               obj_ptr->bypass = new_bypass;
               if (obj_ptr->bypass == 1)
               {
                  // SCHI: bypass mode
                  for (ch = 0; ch < obj_ptr->static_vars.num_chs; ch++)
                  {
                     if (obj_ptr->per_ch[ch].gain_q27 != c_gain_unity)
                     {
                        delta_gain = s32_sub_s32_s32(c_gain_unity, obj_ptr->per_ch[ch].gain_q27);
                        obj_ptr->per_ch[ch].bypass_delta_gain_q27 =
                           divide_int32_qx(delta_gain, obj_ptr->bypass_smps, 0); // Q27
                        obj_ptr->per_ch[ch].bypass_sample_counter = obj_ptr->bypass_smps;
                        obj_ptr->per_ch[ch].fade_flag             = 1;
                     }
                     else
                     {
                        obj_ptr->per_ch[ch].bypass_delta_gain_q27 = 0; // Q27
                        obj_ptr->per_ch[ch].bypass_sample_counter = 0;
                        obj_ptr->per_ch[ch].fade_flag             = 0;
                     }
                  }
               }
            }
         }
         else
         {
            return LIMITER_MEMERROR; // parameter size doesn't match
         }
         break;

      case LIMITER_PARAM_TUNING:
         if (mem_size == sizeof(limiter_tuning_t))
         {
            tuning_ptr = (limiter_tuning_t *)mem_ptr;
            if (tuning_ptr->ch_idx < obj_ptr->static_vars.num_chs)
            {
               // obj_ptr->per_ch[tuning_ptr->ch_idx].tuning_params = *tuning_ptr;
               memscpy(&obj_ptr->per_ch[tuning_ptr->ch_idx].tuning_params, mem_size, tuning_ptr, mem_size);
               // convert tuning params
               convert_tuning_params(obj_ptr, tuning_ptr->ch_idx);
               // and no need for mem flush when setting these params
            }
            else
            {
               return LIMITER_FAILURE; // channel idx out of range
            }
         }
         else
         {
            return LIMITER_MEMERROR; // parameter size doesn't match
         }
         break;

      case LIMITER_PARAM_TUNING_V2:
         if (mem_size == sizeof(limiter_tuning_v2_t))
         {
            tuning_v2_ptr = (limiter_tuning_v2_t *)mem_ptr;
            if (tuning_v2_ptr->ch_idx < obj_ptr->static_vars.num_chs)
            {
               obj_ptr->per_ch[tuning_v2_ptr->ch_idx].tuning_params = *tuning_v2_ptr;
               // convert tuning params
               convert_tuning_params(obj_ptr, tuning_v2_ptr->ch_idx);
               // and no need for mem flush when setting these params
            }
            else
            {
               return LIMITER_FAILURE; // channel idx out of range
            }
         }
         else
         {
            return LIMITER_MEMERROR; // parameter size doesn't match
         }
         break;

      case LIMITER_PARAM_RESET:
         reset(obj_ptr);
         break;

      default:
         return LIMITER_FAILURE; // unidentified parma ID
   }

   return LIMITER_SUCCESS;
}

/*----------------------------------------------------------------------------
// ** Get parameters from library
// lib_ptr: [in] pointer to library structure
// param_id: [in] parameter id
// param_ptr: [out] pointer to the memory where the retrieved value is going to be stored
// param_size:[in] size of the memory pointed by param_ptr
// param_actual_size_ptr: [out] pointer to memory that will hold the actual size of the parameter
----------------------------------------------------------------------------*/
LIMITER_RESULT limiter_get_param(limiter_lib_t *lib_ptr,
                                 uint32         param_id,
                                 void *         mem_ptr,
                                 uint32         mem_size,
                                 uint32 *       param_size_ptr)
{
   limiter_private_t *  obj_ptr = (limiter_private_t *)lib_ptr->mem_ptr;
   limiter_tuning_t *   tuning_ptr;
   limiter_tuning_v2_t *tuning_v2_ptr;

   switch (param_id)
   {

      case LIMITER_PARAM_LIB_VER:
         // get library version
         if (mem_size >= sizeof(limiter_version_t))
         {
            *((limiter_version_t *)mem_ptr) = LIMITER_LIB_VER;
            *param_size_ptr                 = sizeof(limiter_version_t);
         }
         else
         {
            return LIMITER_MEMERROR;
         }
         break;

      case LIMITER_PARAM_MODE:
         if (mem_size >= sizeof(limiter_mode_t))
         {
            *((limiter_mode_t *)mem_ptr) = obj_ptr->mode;
            *param_size_ptr              = sizeof(limiter_mode_t);
         }
         else
         {
            return LIMITER_MEMERROR;
         }
         break;

      case LIMITER_PARAM_BYPASS:
         if (mem_size >= sizeof(limiter_bypass_t))
         {
            *((limiter_bypass_t *)mem_ptr) = obj_ptr->bypass;
            *param_size_ptr                = sizeof(limiter_bypass_t);
         }
         else
         {
            return LIMITER_MEMERROR;
         }
         break;

      case LIMITER_PARAM_TUNING:
         // get tuning parameters for certain obj_ptr->per_ch[tuning_ptr->ch_idx] in channel
         // caller must specify channel index inside the structure first
         if (mem_size >= sizeof(limiter_tuning_t))
         {
            tuning_ptr = (limiter_tuning_t *)mem_ptr;
            if (tuning_ptr->ch_idx < obj_ptr->static_vars.num_chs)
            {
               //*tuning_ptr = obj_ptr->per_ch[tuning_ptr->ch_idx].tuning_params;
               memscpy(tuning_ptr,
                       sizeof(limiter_tuning_t),
                       &obj_ptr->per_ch[tuning_ptr->ch_idx].tuning_params,
                       sizeof(limiter_tuning_t));
               *param_size_ptr = sizeof(limiter_tuning_t);
            }
            else
            {
               return LIMITER_FAILURE; // channel idx out of range
            }
         }
         else
         {
            return LIMITER_MEMERROR;
         }
         break;

      case LIMITER_PARAM_TUNING_V2:
         // get tuning parameters for certain obj_ptr->per_ch[tuning_ptr->ch_idx] in channel
         // caller must specify channel index inside the structure first
         if (mem_size >= sizeof(limiter_tuning_v2_t))
         {
            tuning_v2_ptr = (limiter_tuning_v2_t *)mem_ptr;
            if (tuning_v2_ptr->ch_idx < obj_ptr->static_vars.num_chs)
            {
               *tuning_v2_ptr  = obj_ptr->per_ch[tuning_v2_ptr->ch_idx].tuning_params;
               *param_size_ptr = sizeof(limiter_tuning_v2_t);
            }
            else
            {
               return LIMITER_FAILURE; // channel idx out of range
            }
         }
         else
         {
            return LIMITER_MEMERROR;
         }
         break;

   default:
      return LIMITER_FAILURE;
   }

   return LIMITER_SUCCESS;
}
