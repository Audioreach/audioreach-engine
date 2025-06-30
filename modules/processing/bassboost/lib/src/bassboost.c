/*============================================================================
  FILE:          bassboost.c

  OVERVIEW:      BassBoost Implementations

  DEPENDENCIES:  None

               Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
               SPDX-License-Identifier: BSD-3-Clause-Clear
============================================================================*/
#include "bassboost.h"
#include "simple_mm.h"
#include "filter_design.h"
#include "audio_clips.h"
#include "drc_calib_api.h"

void buffer32_copy
(
    int32           *destBuf,           /* output buffer                     */
    int32           *srcBuf,            /* input buffer                      */
    uint32           samples            /* number of samples to process      */
);

/* configure the internal safeguard limiter */
static BASSBOOST_RESULT config_limiter(bassboost_private_t *obj_ptr)
{
   int32 ch, status = 0;
   limiter_tuning_t tuning;

   limiter_mode_t mode      = NORMAL_PROC;
   limiter_bypass_t bypass  = 0;
   tuning.makeup_gain       = 256;           // unity Q8
   tuning.gc                = 32113;         // GC = 0.98
   tuning.max_wait          = 82;            // (this value doens't matter)

   if (16 == obj_ptr->static_vars.data_width) {
      tuning.threshold         = 32022;      // -0.2 dB, Q27
   } else {
      tuning.threshold         = 131162560;  // -0.2 dB, Q15
   }

   // set mode
   status += limiter_set_param(&obj_ptr->limiter_lib, LIMITER_PARAM_MODE,
      &mode, sizeof(limiter_mode_t));

   // set bypass
   status += limiter_set_param(&obj_ptr->limiter_lib, LIMITER_PARAM_BYPASS,
      &bypass, sizeof(limiter_bypass_t));

   // set per channel tuning
   for (ch = 0; ch < obj_ptr->static_vars.num_chs; ++ch) {
      tuning.ch_idx = ch;
      status += limiter_set_param(&obj_ptr->limiter_lib, LIMITER_PARAM_TUNING,
         &tuning, sizeof(limiter_tuning_t));
   }

   return (status == 0 ? BASSBOOST_SUCCESS: BASSBOOST_FAILURE);
}

/* configure the internal drc */
static BASSBOOST_RESULT config_drc(bassboost_private_t *obj_ptr)
{
   drc_feature_mode_t drc_mode;
   drc_config_t tuning;
   int16 index;
   int32 atrt, status = 0;

   // set drc mode to enable
   drc_mode                         = DRC_ENABLED;
   status += drc_set_param(&obj_ptr->drc_lib, DRC_PARAM_FEATURE_MODE,
      (void*)&drc_mode, sizeof(drc_feature_mode_t));

   // set drc config
   // totally 10 dB boost, 4dB from drc makeup, another 6dB before limiter
   tuning.channelLinked             = CHANNEL_LINKED;
   tuning.downSampleLevel           = 8;         // downsample 8
   tuning.rmsTavUL16Q16             = 310;       // ~ 10ms 44/48k

   tuning.dnExpaThresholdL16Q7      = 4520;      // -55 dBfs
   tuning.dnExpaSlopeL16Q8          = -256;      // 1:2
   tuning.dnExpaHysterisisUL16Q14   = 18855;
   tuning.dnExpaMinGainDBL32Q23     = -167772160;

   tuning.upCompThresholdL16Q7      = 5416;      // - 48 dBfs
   tuning.upCompSlopeUL16Q16        = 43691;     // 3:1
   tuning.upCompHysterisisUL16Q14   = 18855;

   tuning.dnCompThresholdL16Q7      = 9000;      // -20 dBfs thresh
   tuning.dnCompSlopeUL16Q16        = 49152;     // 4:1 comp ratio
   tuning.dnCompHysterisisUL16Q14   = 18855;

   tuning.makeupGainUL16Q12         = 6492;      // makeup: 4dB

   // get 15 ms attack/release time. we support all sample rates but only
   // 4 typicals are pre-calculated. we'll use them for nearby rates
   index = find_freq(obj_ptr->static_vars.sample_rate,
      bassboost_rates_const, BASSBOOST_NUM_RATES);
   atrt = bassboost_atrt_const[index];
   tuning.dnCompAttackUL32Q31       = atrt;
   tuning.dnCompReleaseUL32Q31      = atrt;
   tuning.upCompAttackUL32Q31       = atrt;
   tuning.upCompReleaseUL32Q31      = atrt;
   tuning.dnExpaAttackUL32Q31       = atrt;
   tuning.dnExpaReleaseUL32Q31      = atrt;

   status += drc_set_param(&obj_ptr->drc_lib, DRC_PARAM_CONFIG,
      (void*)&tuning, sizeof(drc_config_t));

   return (status == 0 ? BASSBOOST_SUCCESS: BASSBOOST_FAILURE);
}

/* convert biquad coeffs designed by QSound code to be used by msiir library */
static void convert_iir_coeffs(int32 *out_coeffs, int32 *in_coeffs)
{
   // filter coeffs designed by QSound code are Q23 with such order:
   //          b0, -a2, -a1, b2, b1,
   // MSIIR filter coeff order:
   //          b0, b1, b2, a1, a2
   // we'll do proper adjustment and for simplcity, make each coeff Q30

   // b0, Q23 to Q30
   out_coeffs[0] = s32_shl_s32_rnd_sat(in_coeffs[0], 7);
   // b1, Q23 to Q30
   out_coeffs[1] = s32_shl_s32_rnd_sat(in_coeffs[4], 7);
   // b2, Q23 to Q30
   out_coeffs[2] = s32_shl_s32_rnd_sat(in_coeffs[3], 7);
   // a1, negate, then Q23 to Q30
   out_coeffs[3] = s32_shl_s32_rnd_sat(s32_neg_s32_sat(in_coeffs[2]), 7);
   // a2, negate, then Q23 to Q30
   out_coeffs[4] = s32_shl_s32_rnd_sat(s32_neg_s32_sat(in_coeffs[1]), 7);
}

/* configure internal iir filters */
static BASSBOOST_RESULT config_iir_filters(bassboost_private_t *obj_ptr)
{
   typedef struct combined_struct {
      int32          num_stages;
      msiir_coeffs_t coeffs_struct[BASSBOOST_IIR_STAGES];
   } combined_struct;

   int32 ch, status = 0;
   int32 coeffsL32Q23[5];
   int32 sample_rate = obj_ptr->static_vars.sample_rate;
   int32 num_chs = obj_ptr->static_vars.num_chs;

   combined_struct config;
   config.num_stages = BASSBOOST_IIR_STAGES;
   config.coeffs_struct[0].shift_factor = c_iir_shift_factor;

   // design low-pass filter
   designBiquadLowpassCoeffs(coeffsL32Q23, -602, c_bass_freq_hz, sample_rate, TRUE);
   convert_iir_coeffs(config.coeffs_struct[0].iir_coeffs, coeffsL32Q23);

   for (ch = 0; ch < num_chs; ++ch) {
      status += msiir_set_param(&obj_ptr->bass_fltrs[ch], MSIIR_PARAM_CONFIG,
         &config, sizeof(combined_struct));
   }

   // design band-pass filter
   designBiquadBandpassCoeffs(coeffsL32Q23, -602, c_band_freq_hz, sample_rate, TRUE);
   convert_iir_coeffs(config.coeffs_struct[0].iir_coeffs, coeffsL32Q23);

   for (ch = 0; ch < num_chs; ++ch) {
      status += msiir_set_param(&obj_ptr->band_fltrs[ch], MSIIR_PARAM_CONFIG,
         &config, sizeof(combined_struct));
   }

   return (status == 0 ? BASSBOOST_SUCCESS: BASSBOOST_FAILURE);
}

/* clean buffers and memories */
static void reset(bassboost_private_t *obj_ptr)
{
   int32 ch, dummy;

   limiter_set_param(&obj_ptr->limiter_lib, LIMITER_PARAM_RESET, &dummy, sizeof(dummy));
   drc_set_param(&obj_ptr->drc_lib, DRC_PARAM_SET_RESET, (int8*)NULL, 0);

   for (ch = 0; ch < obj_ptr->static_vars.num_chs; ++ch) {
      // clear delay line
      delayline32_reset(&obj_ptr->delaylines[ch]);
      // reset iir filters
      msiir_set_param(&obj_ptr->bass_fltrs[ch], MSIIR_PARAM_RESET, &dummy, sizeof(dummy));
      msiir_set_param(&obj_ptr->band_fltrs[ch], MSIIR_PARAM_RESET, &dummy, sizeof(dummy));
   }
}

/* set library with default value after init */
static BASSBOOST_RESULT set_default(bassboost_private_t *obj_ptr)
{
   int32 ch, status = 0;

   // set default params
   obj_ptr->enable            = 0;
   obj_ptr->mode              = PHYSICAL_BOOST;
   obj_ptr->strength          = 1000;

   // set strength fader
   obj_ptr->ramp_smps = ms_to_sample(c_fader_smooth_ms, obj_ptr->static_vars.sample_rate);
   obj_ptr->onset_smps = ms_to_sample(c_onset_smooth_ms, obj_ptr->static_vars.sample_rate);
   panner_setup(&obj_ptr->strenghth_fader, Q15_ONE, 0, 0); // set to max

   // set onoff fader
   for (ch = 0; ch < obj_ptr->static_vars.num_chs; ++ch) {
      panner_setup(&obj_ptr->onoff_fader[ch], Q15_ONE, 0, 0); // disable (reverse logic)
   }

   // config sub modules
   status += config_limiter(obj_ptr);
   status += config_drc(obj_ptr);
   status += config_iir_filters(obj_ptr);

   // reset lib
   reset(obj_ptr);

   return (status == 0 ? BASSBOOST_SUCCESS: BASSBOOST_FAILURE);
}

/* mix bass with delayed direct sound with smooth gain */
// gain values are squared to be closer to how we perceive volume
static void smooth_bass_mix16(int32 **mix_buf, int16 **bass_buf,
   pannerStruct *x_fader, int32 num_chs, int32 samples)
{
   int16 bass_gain_q15, bassGainSqL16Q15;
   int32 i, ch, ramped_samples, tmp_sample, bass_gain_q31;
   int32 deltaL32Q31 = x_fader->deltaL32Q31;

   bass_gain_q15 = panner_get_current(*x_fader);
   bass_gain_q31 = s32_shl_s32_sat((int32)bass_gain_q15, 16);
   bassGainSqL16Q15 = s16_extract_s32_h(
      s32_mult_s16_s16_shift_sat(bass_gain_q15, bass_gain_q15));
   ramped_samples = s32_min_s32_s32(samples, x_fader->sampleCounter);

   // additional 6 dB gain applied during mix. notice that this is done
   // by 1 extract upshift in ramped samples and setting Q14 for a Q15
   // gain when mixing with not ramped samples

      for (i = 0; i < ramped_samples; ++i) {
         // obtain gain and square it
         bass_gain_q15 = s16_extract_s32_h(bass_gain_q31);
         bassGainSqL16Q15 = s16_extract_s32_h(
            s32_mult_s16_s16_shift_sat(bass_gain_q15, bass_gain_q15));

         // apply gain squre to samples and mix into 32 bit mix buffers
         for (ch = 0; ch < num_chs; ++ch) {
         tmp_sample = s16_extract_s64_h_rnd(s40_mult_s16_s16_shift(
            *(bass_buf[ch]+i), bassGainSqL16Q15, 2));
            mix_buf[ch][i] = s32_add_s32_s32(mix_buf[ch][i], tmp_sample);
         }
         // add delta increment to Q31 gain
         bass_gain_q31 = s32_add_s32_s32_sat(bass_gain_q31, deltaL32Q31);
      } // end of for loop

      // decrease sample count for ramped samples
      samples -= ramped_samples;
      x_fader->sampleCounter -= ramped_samples;

      // mix the rest
      for (ch = 0; ch < num_chs; ++ch) {
      buffer32_mix16src(mix_buf[ch]+i, bass_buf[ch]+i, bassGainSqL16Q15, 14, samples);
      }
   }

static void smooth_bass_mix32(int32 **mix_buf, int32 **bass_buf,
   pannerStruct *x_fader, int32 num_chs, int32 samples)
{
   // bass gain uses the squared value of linear gain to be one step closer
   // to how we perceive volume, but still stay simple on computation

   int16 bass_gain_q15, bassGainSqL16Q15;
   int32 i, ch, ramped_samples, tmp_sample, bass_gain_q31;
   int32 deltaL32Q31 = x_fader->deltaL32Q31;

   bass_gain_q15 = panner_get_current(*x_fader);
   bass_gain_q31 = s32_shl_s32_sat((int32)bass_gain_q15, 16);
   bassGainSqL16Q15 = s16_extract_s32_h(
      s32_mult_s16_s16_shift_sat(bass_gain_q15, bass_gain_q15));
   ramped_samples = s32_min_s32_s32(samples, x_fader->sampleCounter);

      for (i = 0; i < ramped_samples; ++i) {
         // obtain gain and squre it
         bass_gain_q15 = s16_extract_s32_h(bass_gain_q31);
         bassGainSqL16Q15 = s16_extract_s32_h(
            s32_mult_s16_s16_shift_sat(bass_gain_q15, bass_gain_q15));

         // apply gain squre to samples and mix into 32 bit mix buffers
         for (ch = 0; ch < num_chs; ++ch) {
            tmp_sample = s32_saturate_s64(s64_mult_s32_s16_shift(
            *(bass_buf[ch]+i), bassGainSqL16Q15,2));
            mix_buf[ch][i] = s32_add_s32_s32(mix_buf[ch][i], tmp_sample);
         }
         // add delta increment to Q31 gain
         bass_gain_q31 = s32_add_s32_s32_sat(bass_gain_q31, deltaL32Q31);
      } // end of for loop

      // decrease sample count for ramped samples
      samples -= ramped_samples;
      x_fader->sampleCounter -= ramped_samples;

      // mix the rest
      for (ch = 0; ch < num_chs; ++ch) {
      buffer32_mix32(mix_buf[ch]+i, bass_buf[ch]+i, bassGainSqL16Q15, 14, samples);
      }
}

// smoothly crossfade between src1 and src2 with Q15 smooth panner, unity panner = all src2
// should check these two functions in common lib when possible
// 16 bit version
static void bb_crossmix_panner(int16 *dest, int16 *src1, int16 *src2, pannerStruct *panner, int32 samples)
{
   int16 target_q15  = panner->targetGainL16Q15;
   int32 pan_samples = panner->sampleCounter;
   int32 delta_q31   = panner->deltaL32Q31;
   int16 gain_q15, tmp16;
   int32 gain_q31, ramp_samples, i;

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
      *dest = s16_extract_s40_h(s40_mult_s16_s16_shift(*src2++, gain_q15, 1));
      // get 1-gain and apply to src 1 and mix
      gain_q15 = s16_sub_s16_s16_sat(Q15_ONE, gain_q15);
      tmp16 = s16_extract_s40_h(s40_mult_s16_s16_shift(*src1++, gain_q15, 1));
      *dest = s16_add_s16_s16_sat(*dest, tmp16);
      dest++;
      // add delta to q31 gain
      gain_q31 = s32_add_s32_s32_sat(gain_q31, delta_q31);
   }
   pan_samples -= ramp_samples;

   // process samples with no gain change
   samples -= ramp_samples;
   if (samples > 0) {
      if (Q15_ONE == target_q15) {
         buffer_copy(dest, src2, samples);
      }
      else if (0 == target_q15) {
         buffer_copy(dest, src1, samples);
      }
      else {
         gain_q15 = s16_sub_s16_s16_sat(Q15_ONE, target_q15);
         buffer_fill(dest, src2, target_q15, samples);
         buffer_mix(dest, src1, gain_q15, samples);
      }
   }
   panner->sampleCounter = pan_samples;
}

// 32 bit version (different than common lib, with bug fix)
void bb32_crossmix_panner(int32 *dest, int32 *src1, int32 *src2, pannerStruct *panner, int32 samples)
{
   int16 target_q15  = panner->targetGainL16Q15;
   int32 pan_samples = panner->sampleCounter;
   int32 delta_q31   = panner->deltaL32Q31;
   int16 gain_q15;
   int32 gain_q31, ramp_samples, i, tmp32;

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
      *dest = s32_mult_s32_s16_rnd_sat(*src2++, gain_q15);
      // get 1-gain and apply to src 1 and mix
      gain_q15 = s16_sub_s16_s16_sat(Q15_ONE, gain_q15);
      tmp32 = s32_mult_s32_s16_rnd_sat(*src1++, gain_q15);
      *dest = s32_add_s32_s32_sat(*dest, tmp32);
      dest++;
      // add delta to q31 gain
      gain_q31 = s32_add_s32_s32_sat(gain_q31, delta_q31);
   }
   pan_samples -= ramp_samples;

   // process samples with no gain change
   samples -= ramp_samples;
   if (samples > 0) {
      if (Q15_ONE == target_q15) {
         buffer32_copy(dest, src2, samples);
      }
      else if (0 == target_q15) {
         buffer32_copy(dest, src1, samples);
      }
      else {
         gain_q15 = s16_sub_s16_s16_sat(Q15_ONE, target_q15);
         buffer32_fill16(dest, src2, target_q15, samples);
         buffer32_mix16(dest, src1, gain_q15, samples);
      }
   }

   panner->sampleCounter = pan_samples;
}


static BASSBOOST_RESULT bassboost_proc16(bassboost_private_t *obj_ptr, int16 **out_ptr, int16 **in_ptr, int32 samples)
{
   int32 num_chs = obj_ptr->static_vars.num_chs;
   int16 **bass_buf        = (int16 **)obj_ptr->bass_buf;
   int16 **onset_buf       = (int16 **)obj_ptr->onset_buf;
   int32 **mix_buf         = obj_ptr->mix_buf;
   pannerStruct *strength  = &obj_ptr->strenghth_fader;
   pannerStruct *onset     = obj_ptr->onset_fader;
   pannerStruct *onoff = obj_ptr->onoff_fader;
   int32 ch;

   // bypass mode (low MIPS)
   if (0 >= obj_ptr->onoff_fader->sampleCounter && 0 == obj_ptr->enable) {
      if (out_ptr != in_ptr) {
      for (ch = 0; ch < num_chs; ++ch) {
            buffer_copy(out_ptr[ch], in_ptr[ch], samples);
         }
      }
      return BASSBOOST_SUCCESS;
      }

      for (ch = 0; ch < num_chs; ++ch) {
      // fade in or copy input samples
      buffer_fill_with_panner(onset_buf[ch], in_ptr[ch], &onset[ch], samples);
      // filter input, put bass part in bass buf
      msiir_process_v2(&obj_ptr->bass_fltrs[ch], bass_buf[ch], onset_buf[ch], samples);
      // filter input, put band pass portion in out buf as scratch
      msiir_process_v2(&obj_ptr->band_fltrs[ch], out_ptr[ch], onset_buf[ch], samples);
      // copy input to mix buf
      buffer32_copy16(mix_buf[ch], onset_buf[ch], samples);
      // mix band pass signal into bass buf: bass+0.5*band
      buffer_mix(bass_buf[ch], out_ptr[ch], Q15_HALF, samples);
      // inplace delay mixing buf contents
      delayline32_inplace_delay(mix_buf[ch], &obj_ptr->delaylines[ch], samples);
      }

   // apply drc on bass
   drc_process(&obj_ptr->drc_lib, (int8 **)bass_buf, (int8 **)bass_buf, samples);
   // mix bass and direct sound
   smooth_bass_mix16(mix_buf, bass_buf, strength, num_chs, samples);
   // use bass_buf as scratch for processed samples
   limiter_process(&obj_ptr->limiter_lib, (void **)bass_buf, mix_buf, samples);
   // cross fade at the output when needed
   for (ch = 0; ch < num_chs; ++ch) {
      bb_crossmix_panner(out_ptr[ch], bass_buf[ch], in_ptr[ch], &onoff[ch], samples);
   }

   return BASSBOOST_SUCCESS;
}

static BASSBOOST_RESULT bassboost_proc32(bassboost_private_t *obj_ptr, int32 **out_ptr, int32 **in_ptr, int32 samples)
{
   int32 num_chs           = obj_ptr->static_vars.num_chs;
   int32 **bass_buf        = (int32 **)obj_ptr->bass_buf;
   int32 **onset_buf       = (int32 **)obj_ptr->onset_buf;
   int32 **mix_buf         = obj_ptr->mix_buf;
   pannerStruct *strength  = &obj_ptr->strenghth_fader;
   pannerStruct *onset     = obj_ptr->onset_fader;
   pannerStruct *onoff     = obj_ptr->onoff_fader;
   int32 ch;

   // bypass mode (low MIPS)
   if (0 >= obj_ptr->onoff_fader->sampleCounter && 0 == obj_ptr->enable) {
         if (out_ptr != in_ptr) {
            for (ch = 0; ch < num_chs; ++ch) {
            buffer32_copy(out_ptr[ch], in_ptr[ch], samples);
         }
      }
      return BASSBOOST_SUCCESS;
   }

   for (ch = 0; ch < num_chs; ++ch) {
      // fade in or copy input samples
      buffer32_fill_panner(onset_buf[ch], in_ptr[ch], &onset[ch], samples);
      // filter input, put bass part in bass buf
      msiir_process_v2(&obj_ptr->bass_fltrs[ch], bass_buf[ch], onset_buf[ch], samples);
      // filter input, put band pass portion in out buf as scratch
      msiir_process_v2(&obj_ptr->band_fltrs[ch], out_ptr[ch], onset_buf[ch], samples);
         // copy input to mix buf
      buffer32_copy(mix_buf[ch], onset_buf[ch], samples);
         // mix band pass signal into bass buf: bass+0.5*band
      buffer32_mix16(bass_buf[ch], out_ptr[ch], Q15_HALF, samples);
      // inplace delay mixing buf contents
      delayline32_inplace_delay(mix_buf[ch], &obj_ptr->delaylines[ch], samples);
      }

   // apply drc on bass
   drc_process(&obj_ptr->drc_lib, (int8 **)bass_buf, (int8 **)bass_buf, samples);
   // mix bass and direct sound
   smooth_bass_mix32(mix_buf, bass_buf, strength, num_chs, samples);
   // use bass_buf as scratch for processed samples
   limiter_process(&obj_ptr->limiter_lib, (void **)bass_buf, mix_buf, samples);
   // cross fade at the output when needed
   for (ch = 0; ch < num_chs; ++ch) {
      bb32_crossmix_panner(out_ptr[ch], bass_buf[ch], in_ptr[ch], &onoff[ch], samples);
      }

   return BASSBOOST_SUCCESS;
   }

/*----------------------------------------------------------------------------
   API Functions
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
// ** Processing one block of samples
// lib_ptr: [in] pointer to library structure
// out_ptr: [out] pointer to output sample block (multi channel double ptr)
// in_ptr: [in] pointer to input sample block (multi channel double ptr)
// samples: [in] number of samples to be processed per channel
----------------------------------------------------------------------------*/
BASSBOOST_RESULT bassboost_process(bassboost_lib_t *lib_ptr, void **out_ptr, void **in_ptr, uint32 api_samples)
{
   bassboost_private_t *obj_ptr = (bassboost_private_t *)lib_ptr->mem_ptr;
   int32 samples = (int32)api_samples;

   // sanity check block size
   if (samples > obj_ptr->static_vars.max_block_size) {
      return BASSBOOST_FAILURE;
   }

   // process for 16/32 bit respectively
   if (16 == obj_ptr->static_vars.data_width) {
      return bassboost_proc16(obj_ptr, (int16 **)out_ptr, (int16 **)in_ptr, samples);
   } else {
      return bassboost_proc32(obj_ptr, (int32 **)out_ptr, (int32 **)in_ptr, samples);
   }
}

/*----------------------------------------------------------------------------
// ** Get library memory requirements
// mem_req_ptr: [out] pointer to mem requirements structure
// static_vars_ptr: [in] pointer to static variable structure
----------------------------------------------------------------------------*/
BASSBOOST_RESULT bassboost_get_mem_req(bassboost_mem_req_t *mem_req_ptr, bassboost_static_vars_t* static_vars_ptr)
{
   uint32 reqsize;
   int32 data_width        = static_vars_ptr->data_width;
   int32 sample_rate       = static_vars_ptr->sample_rate;
   int32 max_block_size    = static_vars_ptr->max_block_size;
   int32 num_chs           = static_vars_ptr->num_chs;
   int32 limiter_delay_ms  = static_vars_ptr->limiter_delay;

   msiir_static_vars_t           msiir_static;
   msiir_mem_req_t               msiir_mem_req;
   drc_static_struct_t           drc_static;
   drc_lib_mem_requirements_t    drc_mem_req;
   limiter_static_vars_t         limiter_static;
   limiter_mem_req_t             limiter_mem_req;

   int32 direct_delay, bytes_per_sample, delay_samples, limiter_delay_q15_sec, ch;

   /* first, sanity check data width */
   if (16 == data_width) {
      bytes_per_sample = 2;
   } else if (32 == data_width) {
      bytes_per_sample = 4;
   } else {
      return BASSBOOST_FAILURE;
   }

   /* start with fixed structure */
   reqsize = smm_malloc_size(sizeof(bassboost_private_t));

   /*------------------------ calculate limiter mems ------------------------*/
   clip_32(&limiter_delay_ms, 0, c_limiter_max_delay_ms);
   limiter_delay_q15_sec = s32_mult_s16_s16((int16)limiter_delay_ms, 33);

   limiter_static.data_width     = data_width;
   limiter_static.sample_rate    = sample_rate;
   limiter_static.max_block_size = max_block_size;
   limiter_static.num_chs        = num_chs;
   limiter_static.delay          = limiter_delay_q15_sec;

   if (LIMITER_SUCCESS != limiter_get_mem_req(&limiter_mem_req, &limiter_static)) {
      return BASSBOOST_FAILURE;
   }
   reqsize += smm_malloc_size(limiter_mem_req.mem_size);

   /*-------------------------- calculate drc mems --------------------------*/
   delay_samples = ms_to_sample(c_drc_delay_ms, sample_rate);

   drc_static.data_width         = (16 == data_width)? BITS_16: BITS_32;
   drc_static.sample_rate        = sample_rate;
   drc_static.num_channel        = num_chs;
   drc_static.delay              = delay_samples;

   if ((int)MSIIR_SUCCESS != (int)drc_get_mem_req(&drc_mem_req, &drc_static)) {
      return BASSBOOST_FAILURE;
   }
   reqsize += smm_malloc_size(drc_mem_req.lib_mem_size);

   /*------------------------- calculate delay line -------------------------*/
   direct_delay = ms_to_sample(c_drc_delay_ms, sample_rate);
   reqsize += smm_calloc_size(num_chs, sizeof(delayline32_t));
   for (ch = 0; ch < num_chs; ++ch) {
      reqsize += smm_calloc_size(direct_delay, sizeof(int32));
   }

   /*--------------------- calculate onoff/onset faders ---------------------*/
   reqsize += smm_calloc_size(num_chs, sizeof(pannerStruct));
   reqsize += smm_calloc_size(num_chs, sizeof(pannerStruct));

   /*------------------------- calculate msiir mems -------------------------*/
   reqsize += smm_calloc_size(num_chs, sizeof (msiir_lib_t)); // bass
   reqsize += smm_calloc_size(num_chs, sizeof (msiir_lib_t)); // band

   msiir_static.data_width       = data_width;
   msiir_static.max_stages       = 1;
   if (MSIIR_SUCCESS != msiir_get_mem_req(&msiir_mem_req, &msiir_static)) {
      return BASSBOOST_FAILURE;
   }

   for (ch = 0; ch < num_chs; ++ch) {
      reqsize += smm_malloc_size(msiir_mem_req.mem_size);   // bass
      reqsize += smm_malloc_size(msiir_mem_req.mem_size);   // band
   }

   /*--------------------------- scratch buffers ----------------------------*/
   reqsize += smm_calloc_size(num_chs, sizeof (void *));    // bass scratch
   reqsize += smm_calloc_size(num_chs, sizeof (void *));    // onset scratch
   reqsize += smm_calloc_size(num_chs, sizeof (int32 *));   // mix buf

   for (ch = 0; ch < num_chs; ++ch) {
      reqsize += smm_calloc_size(max_block_size, bytes_per_sample); // bass
      reqsize += smm_calloc_size(max_block_size, bytes_per_sample); // onset
      reqsize += smm_calloc_size(max_block_size, sizeof(int32)); // 32bit mix
   }

   /*------------------------- store requirements ---------------------------*/
   mem_req_ptr->mem_size = reqsize;
   mem_req_ptr->stack_size = c_bassboost_max_stack_size;

   return BASSBOOST_SUCCESS;
}

/*----------------------------------------------------------------------------
// ** Partition, initialize memory, and set default values
// lib_ptr: [in, out] pointer to library structure
// static_vars_ptr: [in] pointer to static variable structure
// mem_ptr: [in] pointer to allocated memory
// mem_size: [in] size of memory allocated
----------------------------------------------------------------------------*/
BASSBOOST_RESULT bassboost_init_mem(bassboost_lib_t *lib_ptr, bassboost_static_vars_t* static_vars_ptr, void* mem_ptr, uint32 mem_size)
{
   SimpleMemMgr            MemMgr;
   SimpleMemMgr            *smm = &MemMgr;
   bassboost_mem_req_t     mem_req;
   bassboost_private_t     *obj_ptr;
   int32 num_chs           = static_vars_ptr->num_chs;
   int32 sample_rate       = static_vars_ptr->sample_rate;
   int32 data_width        = static_vars_ptr->data_width;
   int32 max_block_size    = static_vars_ptr->max_block_size;
   int32 limiter_delay_ms  = static_vars_ptr->limiter_delay;

   msiir_static_vars_t           msiir_static;
   msiir_mem_req_t               msiir_mem_req;
   drc_static_struct_t           drc_static;
   drc_lib_mem_requirements_t    drc_mem_req;
   limiter_static_vars_t         limiter_static;
   limiter_mem_req_t             limiter_mem_req;

   int32 bytes_per_sample, delay_samples, direct_delay, limiter_delay_q15_sec, ch;

   /* double check if allocated mem is enough */
   bassboost_get_mem_req(&mem_req, static_vars_ptr);
   if (mem_req.mem_size > mem_size) {
      return BASSBOOST_MEMERROR;
   }

   /* sanity check data width */
   if (16 == data_width) {
      bytes_per_sample = 2;
   } else if (32 == data_width) {
      bytes_per_sample = 4;
   } else {
      return BASSBOOST_FAILURE;
   }

   /* assign initial mem pointers and allocate current instance */
   lib_ptr->mem_ptr = mem_ptr;
   smm_init(smm, lib_ptr->mem_ptr);
   obj_ptr = (bassboost_private_t *)smm_malloc(smm, sizeof(bassboost_private_t));

   /* keep a copy of static vars */
   obj_ptr->static_vars = *static_vars_ptr;

   /*-------------------- alloc & config safeguard limiter ------------------*/
   clip_32(&limiter_delay_ms, 0, c_limiter_max_delay_ms);
   limiter_delay_q15_sec = s32_mult_s16_s16((int16)limiter_delay_ms, 33);

   limiter_static.data_width     = data_width;
   limiter_static.sample_rate    = sample_rate;
   limiter_static.max_block_size = max_block_size;
   limiter_static.num_chs        = num_chs;
   limiter_static.delay          = limiter_delay_q15_sec;

   limiter_get_mem_req(&limiter_mem_req, &limiter_static);
   mem_size = limiter_mem_req.mem_size;
   mem_ptr = smm_malloc(smm, mem_size);
   if (LIMITER_SUCCESS != limiter_init_mem(
      &obj_ptr->limiter_lib, &limiter_static, mem_ptr, mem_size)) {
         return BASSBOOST_FAILURE;
   }

   /*------------------------- alloc & config drc  --------------------------*/
   delay_samples = ms_to_sample(c_drc_delay_ms, sample_rate);

   drc_static.data_width         = (16 == data_width)? BITS_16: BITS_32;
   drc_static.sample_rate        = sample_rate;
   drc_static.num_channel        = num_chs;
   drc_static.delay              = delay_samples;

   drc_get_mem_req(&drc_mem_req, &drc_static);
   mem_size = drc_mem_req.lib_mem_size;
   mem_ptr = smm_malloc(smm, mem_size);
   if (DRC_SUCCESS != drc_init_memory(
      &obj_ptr->drc_lib, &drc_static, mem_ptr, mem_size)) {
         return BASSBOOST_FAILURE;
   }

   /*------------------------- alloc direct delay  --------------------------*/
   direct_delay = ms_to_sample(c_drc_delay_ms, sample_rate);
   obj_ptr->delaylines = (delayline32_t *)smm_calloc(smm, num_chs, sizeof(delayline32_t));
   for (ch = 0; ch < num_chs; ++ch) {
      obj_ptr->delaylines[ch].buf = (int32 *)smm_calloc( smm, direct_delay, sizeof(int32));
      obj_ptr->delaylines[ch].buf_size = direct_delay;
   }

   /*----------------------- alloc onoff/onset fader ------------------------*/
   obj_ptr->onoff_fader = (pannerStruct *)smm_calloc(smm, num_chs, sizeof(pannerStruct));
   obj_ptr->onset_fader = (pannerStruct *)smm_calloc(smm, num_chs, sizeof(pannerStruct));

   /*---------------------- alloc for msiir filter --------------------------*/
   obj_ptr->bass_fltrs = (msiir_lib_t *)smm_calloc(smm, num_chs, sizeof(msiir_lib_t));
   obj_ptr->band_fltrs = (msiir_lib_t *)smm_calloc(smm, num_chs, sizeof(msiir_lib_t));

   msiir_static.data_width       = data_width;
   msiir_static.max_stages       = BASSBOOST_IIR_STAGES;
   msiir_get_mem_req(&msiir_mem_req, &msiir_static);
   mem_size = msiir_mem_req.mem_size;

   for (ch = 0; ch < num_chs; ++ch) {
      mem_ptr = smm_malloc(smm, mem_size);
      if (MSIIR_SUCCESS != msiir_init_mem(
         &obj_ptr->bass_fltrs[ch], &msiir_static, mem_ptr, mem_size)) {
            return BASSBOOST_FAILURE;
      }

      mem_ptr = smm_malloc(smm, mem_size);
      if (MSIIR_SUCCESS != msiir_init_mem(
         &obj_ptr->band_fltrs[ch], &msiir_static, mem_ptr, mem_size)) {
            return BASSBOOST_FAILURE;
      }
   }

   /*--------------------------- alloc scratch buffers ----------------------*/
   obj_ptr->bass_buf = (void **)smm_calloc(smm, num_chs, sizeof(void *));
   obj_ptr->onset_buf = (void **)smm_calloc(smm, num_chs, sizeof(void *));
   obj_ptr->mix_buf = (int32 **)smm_calloc(smm, num_chs, sizeof(int32 *));

   for (ch = 0; ch < num_chs; ++ch) {
      obj_ptr->bass_buf[ch] = (void *)smm_calloc(smm, max_block_size, bytes_per_sample);
      obj_ptr->onset_buf[ch] = (void *)smm_calloc(smm, max_block_size, bytes_per_sample);
      obj_ptr->mix_buf[ch] = (int32 *)smm_calloc(smm, max_block_size, sizeof(int32));
   }

   /*--------------------------- set default values -------------------------*/
   return set_default(obj_ptr);
}

/*----------------------------------------------------------------------------
// ** Set parameters to library
// lib_ptr: [in, out] pointer to lib structure
// param_id: [in] parameter id
// param_ptr: [in] pointer to the memory where the new values are stored
// param_size:[in] size of the memory pointed by param_ptr
----------------------------------------------------------------------------*/
BASSBOOST_RESULT bassboost_set_param(bassboost_lib_t* lib_ptr, uint32 param_id, void* mem_ptr, uint32 mem_size)
{
   bassboost_private_t *obj_ptr = (bassboost_private_t *)lib_ptr->mem_ptr;
   bassboost_mode_t    new_mode;
   int16 strength, strength_q15;
   int32 enable, ch;

   switch (param_id) {

   case BASSBOOST_PARAM_ENABLE:
      if (mem_size == sizeof(bassboost_enable_t)) {
         enable = *((bassboost_enable_t *)mem_ptr);
         if (0 != enable) { enable = 1; }
         if (obj_ptr->enable != enable) {
            obj_ptr->enable = enable;
            // handle onset panner and buf to ramp up input
            if (1 == enable && 0 == obj_ptr->onoff_fader->sampleCounter) {
               reset(obj_ptr);
               for (ch = 0; ch < obj_ptr->static_vars.num_chs; ++ch) {
                  panner_setup(&obj_ptr->onset_fader[ch], 0, 0, 0);
                  panner_setup(&obj_ptr->onset_fader[ch], Q15_ONE, obj_ptr->onset_smps, 0);
               }
            }
            // onoff crossfader: has reverse logic, 0: on, Q15_ONE: off
            // this design is to accommodate inplace processing
            strength_q15 = enable ? 0 : Q15_ONE;
            for (ch = 0; ch < obj_ptr->static_vars.num_chs; ++ch) {
               panner_setup(&obj_ptr->onoff_fader[ch], strength_q15, obj_ptr->ramp_smps, 0);
            }
         }
      } else {
         return BASSBOOST_MEMERROR;  // parameter size doesn't match
      }
      break;

   case BASSBOOST_PARAM_MODE:
      if (mem_size == sizeof(bassboost_mode_t)) {
         new_mode = *((bassboost_mode_t *)mem_ptr);

         if (obj_ptr->mode != new_mode) {
            // current bass boost supports only physical boost, thus it's
            // hard coded here. when we add virtual boost later, the following
            // lines should be changed. - April, 2013

            //obj_ptr->mode = new_mode;
            obj_ptr->mode = PHYSICAL_BOOST;
            reset(obj_ptr);
         }
      } else {
         return BASSBOOST_MEMERROR;  // parameter size doesn't match
      }
      break;

   case BASSBOOST_PARAM_STRENGTH:
      if (mem_size == sizeof(bassboost_strength_t)) {
         strength = s16_saturate_s32(*((bassboost_strength_t *)mem_ptr));
         if (obj_ptr->strength != strength) {
            obj_ptr->strength = strength;
            // convert to Q15 value: gain * 32767 / 1000
            strength_q15 = s16_saturate_s32(Q23_mult(strength, 274869518));
            panner_setup(&obj_ptr->strenghth_fader, strength_q15, obj_ptr->ramp_smps, 0);
         }
      } else {
         return BASSBOOST_MEMERROR;  // parameter size doesn't match
      }
      break;

   case BASSBOOST_PARAM_RESET:
      reset(obj_ptr);
      break;

   default:
      return BASSBOOST_FAILURE; // unidentified parma ID
   }

   return BASSBOOST_SUCCESS;
}


/*----------------------------------------------------------------------------
// ** Get parameters from library
// lib_ptr: [in] pointer to library structure
// param_id: [in] parameter id
// param_ptr: [out] pointer to the memory where the retrieved value is going to be stored
// param_size:[in] size of the memory pointed by param_ptr
// param_actual_size_ptr: [out] pointer to memory that will hold the actual size of the parameter
----------------------------------------------------------------------------*/
BASSBOOST_RESULT bassboost_get_param(bassboost_lib_t* lib_ptr, uint32 param_id, void* mem_ptr, uint32 mem_size, uint32 *param_size_ptr)
{
   bassboost_private_t *obj_ptr = (bassboost_private_t *)lib_ptr->mem_ptr;

   switch (param_id) {

   case BASSBOOST_PARAM_LIB_VER:
      // get library version
      if (mem_size >= sizeof(bassboost_version_t)) {
         *((bassboost_version_t *)mem_ptr) = BASSBOOST_LIB_VER;
         *param_size_ptr = sizeof(bassboost_version_t);
      } else {
         return BASSBOOST_MEMERROR;
      }
      break;

   case BASSBOOST_PARAM_ENABLE:
      if (mem_size >= sizeof(bassboost_enable_t)) {
         *((bassboost_enable_t *)mem_ptr) = obj_ptr->enable;
         *param_size_ptr = sizeof(bassboost_enable_t);
      } else {
         return BASSBOOST_MEMERROR;
      }
      break;

   case BASSBOOST_PARAM_MODE:
      if (mem_size >= sizeof(bassboost_mode_t)) {
         *((bassboost_mode_t *)mem_ptr) = obj_ptr->mode;
         *param_size_ptr = sizeof(bassboost_mode_t);
      } else {
         return BASSBOOST_MEMERROR;
      }
      break;

   case BASSBOOST_PARAM_STRENGTH:
      // get tuning parameters for certain channel
      if (mem_size >= sizeof(bassboost_strength_t)) {
         *((bassboost_strength_t *)mem_ptr) = obj_ptr->strength;
         *param_size_ptr = sizeof(bassboost_strength_t);
      } else {
         return BASSBOOST_MEMERROR;
      }
      break;

   case BASSBOOST_PARAM_DELAY:
      // algorithmic delay = drc + limiter delay, in samples
      if (mem_size >= sizeof(bassboost_delay_t)) {
         if (0 >= obj_ptr->onoff_fader->sampleCounter &&
            0 == obj_ptr->enable) { // if disabled steady state, no delay
            *((bassboost_delay_t *)mem_ptr) = 0;
         } else {
            *((bassboost_delay_t *)mem_ptr) = obj_ptr->delaylines[0].buf_size +
            ms_to_sample((int16)obj_ptr->static_vars.limiter_delay,
            obj_ptr->static_vars.sample_rate);
         }
         *param_size_ptr = sizeof(bassboost_delay_t);
      } else {
         return BASSBOOST_MEMERROR;
      }
      break;

   case BASSBOOST_PARAM_CROSSFADE_FLAG:
      // if onoff fader still has samples left, then it's in transition
      if (mem_size >= sizeof(bassboost_crossfade_t)) {
         if (0 >= obj_ptr->onoff_fader->sampleCounter) {
            *((bassboost_crossfade_t *)mem_ptr) = 0;
         } else {
            *((bassboost_crossfade_t *)mem_ptr) = 1;
         }
         *param_size_ptr = sizeof(bassboost_crossfade_t);
      } else {
         return BASSBOOST_MEMERROR;
      }
      break;
   default:
      return BASSBOOST_FAILURE; // invalid parameter id
   }

   return BASSBOOST_SUCCESS;
}


