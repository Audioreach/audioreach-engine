#ifndef LIMITER_H
#define LIMITER_H
/*============================================================================
  @file limiter.h

  Private Header for Zero-Crossing Limiter, Peak-History Limiter

		Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
      SPDX-License-Identifier: BSD-3-Clause

============================================================================*/
#include "limiter_api.h"

#if ((defined __hexagon__) || (defined __qdsp6__))
#define LIM_ASM
#endif

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*----------------------------------------------------------------------------
   Constants
----------------------------------------------------------------------------*/
#define BYPASS_TRANSITION_PERIOD_MSEC          10   
static const int32  c_1_over_1000_q31        = 2147484;    // 0.001 in Q31

static const int32  c_limiter_max_stack_size = 2000;       // worst case stack mem
static const int16  c_mgain_unity            = 256;        // unity makeup gain (1, Q8)
static const int32  c_gain_unity             = 134217728;  // limiter unity gain (1, Q27)
static const int32  c_unity_q15              = 32768;      // limiter unity number (1, Q15)
static const uint32 c_unity_q31              = 0x80000000; // limiter unity number (1, Q31)
static const int16  c_fade_grc               = 24576;      // release const (0.75, Q15)
static const int16  c_local_peak_bufsize     = 4;          // history peak buffer size

/*----------------------------------------------------------------------------
 * Type Declarations
 * -------------------------------------------------------------------------*/
typedef struct limiter_per_ch_t {               // ** per channel variables
   										        
   limiter_tuning_v2_t  tuning_params;          //    internal copy: tuning params
										        
   int32                max_wait_smps_m1;       //    max wait sample count - 1
   int32                makeup_gain_q16;        //    left-shifted makeup gain in Q16
   int32                local_max_peak;         //    local maximum peak array
   int32                prev_sample_l32;        //    previous sample in the buffer
   int32                cur_idx;                //    current sample index
   int32                prev_zc_idx;            //    previous zero-crossing idx array
   int32                fade_flag;              //    fade flag for transition
   int32                gain_q27;               //    limiter gain (q27)
   int32                gain_var_q27;           //    limiter gain var (Q27)

   int32                shifted_threshold;      //    limiter threshold aligned to input q-factor
   int32                shifted_hard_threshold; //    limiter hard-threshold aligned to input q-factor

   int32                target_gain_q27;        //    target limiter gain (Q27) 
   int32                global_peak;            //    global peak
   int32                peak_subbuf_idx;        //    current index for the history window shifting buffer
   int32                prev_peak_idx;          //    previous zero-crossing idx array
										        
   int32                *delay_buf;             //    delay buffer (32 bit)
   int32                *zc_buf;                //    zero-crossing buffer (32 bit)
   int32                *history_peak_buf;      //    history peak buffer (32 bit)
   int32                fade_flag_prev;         //    fade flag for previous frame
   int32                bypass_sample_counter;  //    bypass transition samples
   int32                bypass_delta_gain_q27;
   int32                reserved_var;
} limiter_per_ch_t;

typedef struct limiter_private_t {        // ** main private structure
   
   limiter_static_vars_v2_t static_vars;  //    internal copy: static vars

   limiter_mode_t       mode;             //    processing mode
   limiter_bypass_t     bypass;           //    bypass flag
   int32                dly_smps_m1;      //    delay sample count - 1

   limiter_per_ch_t     *per_ch;          //    per channel struct
   int32                *scratch_buf;     //    scratch buffer
   int32                bypass_smps;      //    bypass transition samples
} limiter_private_t;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIMITER_H */
