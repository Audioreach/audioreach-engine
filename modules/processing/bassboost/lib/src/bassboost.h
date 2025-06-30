#ifndef BASSBOOST_H
#define BASSBOOST_H
/*============================================================================
  @file bassboost.h

  Private Header for Bass Boost Effect

        Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
        SPDX-License-Identifier: BSD-3-Clause-Clear
============================================================================*/
#include "bassboost_api.h"

/*----------------------------------------------------------------------------
   Include Files
----------------------------------------------------------------------------*/
#include "audio_dsp32.h"
#include "limiter_api.h"
#include "msiir_api.h"
#include "drc_api.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*----------------------------------------------------------------------------
   Constants
----------------------------------------------------------------------------*/
#define BASSBOOST_IIR_STAGES                    (1)    // only one stage iir  
#define BASSBOOST_NUM_RATES                     (6)    // sample rate count
static const int32 c_bassboost_max_stack_size = 800;   // worst stack size (552)
static const int16 c_limiter_max_delay_ms     = 20;    // 20ms supports down to 25 Hz signal
static const int16 c_drc_delay_ms             = 4;     // 4ms constant drc delay
static const int16 c_fader_smooth_ms          = 100;   // panner smooth time (ms)
static const int16 c_onset_smooth_ms          = 30;    // onset panner smooth time (ms)
static const int32 c_bass_freq_hz             = 200;   // bass cutoff frequency
static const int32 c_band_freq_hz             = 350;   // band-pass center frequency
static const int32 c_iir_shift_factor         = 2;     // iir filter shift factor

/*----------------------------------------------------------------------------
   Type Declarations
----------------------------------------------------------------------------*/
/* sampling rates having pre-calculated params for */
static const int32 bassboost_rates_const[BASSBOOST_NUM_RATES] = {
   22050, 
   32000,
   44100,
   48000,
   96000,
   192000
};

/* attack / release constant for the 4 frequencies (15ms) */
static const int32 bassboost_atrt_const[BASSBOOST_NUM_RATES] = {
   111285605,
   77314964,
   56382981,
   51857647,
   26087276,
   13083493
};

/* main data struct */
typedef struct bassboost_private_t {       // ** main private structure

   /* internal copy of static vars */
   bassboost_static_vars_t static_vars;

   /* internal copy of tuning vars */
   bassboost_enable_t   enable;           //    on/off switch [0 or 1]
   bassboost_mode_t     mode;             //    (placeholder)
   bassboost_strength_t strength;         //    effect strength [0, 1000]

   /* other internal vars */
   limiter_lib_t        limiter_lib;      //    safeguard limiter instance
   drc_lib_t            drc_lib;          //    drc instance
   msiir_lib_t          *bass_fltrs;      //    low-pass filter array
   msiir_lib_t          *band_fltrs;      //    band-pass filter array
   delayline32_t        *delaylines;      //    direct delays to match drc
   
   int32                ramp_smps;        //    smooth panner ramp samples
   int32                onset_smps;       //    smooth samples for onset fader
   pannerStruct         strenghth_fader;  //    fader for strength
   pannerStruct         *onoff_fader;     //    fader for enable/disable
   pannerStruct         *onset_fader;     //    fader for enable transition

   void                 **bass_buf;       //    buffer to hold bass content
   void                 **onset_buf;      //    buffer to hold smoothed input samples
   int32                **mix_buf;        //    buffer to hold samples before limiting

} bassboost_private_t;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* BASSBOOST_H */
