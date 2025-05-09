/* =========================================================================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
   ========================================================================== */

#include "capi_popless_equalizer_utils.h"

#define NUM_GENERAL_FREQUENCIES 12

static const uint32_t freq_array[] = { 8000,  11025, 16000, 22050, 32000,  44100,
                                       48000, 64000, 88200, 96000, 176400, 192000 };

static const uint32_t p_eq_kpps_table_enable[2][EQUALIZER_MAX_CHANNELS][NUM_GENERAL_FREQUENCIES] =
   { { { 290, 386, 545, 738, 1311, 1792, 1947, 2584, 3547, 3858, 7057, 7678 },
       { 558, 751, 1068, 1454, 2600, 3563, 3873, 5147, 7073, 7693, 14093, 15334 },
       { 826, 1116, 1591, 2171, 3889, 5333, 5799, 7709, 10598, 11529, 21117, 22968 },
       { 1095, 1481, 2115, 2887, 5178, 7104, 7724, 10271, 14124, 15365, 28164, 30647 },
       { 1363, 1846, 2638, 3603, 6466, 8874, 9650, 12834, 17649, 19201, 35199, 38303 },
       { 1631, 2211, 3161, 4319, 7755, 10645, 11576, 15396, 21175, 23037, 42234, 45959 },
       { 1899, 2575, 3685, 5036, 9044, 12415, 13502, 17959, 24700, 26873, 49270, 53616 },
       { 2168, 2940, 4208, 5752, 10333, 14186, 15427, 20521, 28226, 30709, 56305, 61272 } },
     { { 258, 343, 482, 652, 1154, 1576, 1712, 2270, 3115, 3387, 6192, 6737 },
       { 495, 665, 943, 1281, 2286, 3130, 3402, 4519, 6208, 6752, 12363, 13452 },
       { 732, 986, 1403, 1911, 3418, 4684, 5093, 6767, 9301, 10117, 18528, 20156 },
       { 969, 1308, 1864, 2541, 4550, 6238, 6783, 9016, 12394, 13482, 24704, 26881 },
       { 1206, 1630, 2325, 3171, 5682, 7793, 8473, 11265, 15487, 16848, 30875, 33596 },
       { 1443, 1951, 2785, 3801, 6814, 9347, 10163, 13513, 18580, 20213, 37045, 40311 },
       { 1680, 2273, 3246, 4431, 7946, 10901, 11854, 15762, 21673, 23578, 43216, 47026 },
       { 1917, 2594, 3706, 5061, 9078, 12455, 13544, 18010, 24766, 26943, 49386, 53741 } }

   };

static inline int32_t get_frequency_index(uint32_t freq)
{
   int16 i = 0;
   for (i = 0; i < NUM_GENERAL_FREQUENCIES; i++)
   {
      if (freq == freq_array[i])
         return i;
   }
   return -1;
}

static inline int32_t get_channel_index(uint32_t num_ch)
{
   return (num_ch - 1);
}

uint32_t capi_p_eq_get_kpps(capi_p_eq_t *me_ptr)
{

   const uint32_t numChan         = me_ptr->input_media_fmt.format.num_channels;
   const uint32_t Freq            = me_ptr->input_media_fmt.format.sampling_rate;
   uint32_t       bits_per_sample = me_ptr->input_media_fmt.format.bits_per_sample;

   int32_t  ch_index   = get_channel_index(numChan);
   int32_t  freq_index = get_frequency_index(Freq);
   int32_t  bps_index  = (BITS_PER_SAMPLE_16 == bits_per_sample) ? 0 : 1;
   uint32_t kpps       = 0;

   if ((freq_index >= 0) && (ch_index >= 0))
   {
      kpps = p_eq_kpps_table_enable[bps_index][ch_index][freq_index];
   }

   return kpps;
}
