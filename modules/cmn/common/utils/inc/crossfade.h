#ifndef CROSSFADE_H
#define CROSSFADE_H

/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*============================================================================
  FILE:          crossfade.h

  OVERVIEW:      This file has the configaration and data structures for cross fade algorithm.

  DEPENDENCIES:  None
============================================================================*/

/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/


#include "crossfade_api.h"




#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*----------------------------------------------------------------------------
 * Preprocessor Definitions and Constants
 * -------------------------------------------------------------------------*/

#define CROSS_FADE_LIB_VER   (0x01000200)   // lib version : 1.2.0
                                       // (major.minor.bug) (8.16.8 bits)


static const uint32_t cross_fade_max_stack_size = 200;   // worst case stack mem in bytes; 
#define ALIGN8(o)         (((o)+7)&(~7))

#define CF_ONE_Q15                  32767
#define CF_ONE_Q31                  2147483647L
#define CF_HALF_Q31             1073741824L
#define CF_ONE_BY_1000_IN_0DOT32    4294968 
#define CF_BITS32                   2
#define CF_BITS16                   1

/*----------------------------------------------------------------------------
 * Type Declarations
 * -------------------------------------------------------------------------*/

typedef enum CROSS_FADE_PARAMS_DEFAULT
{
    // Default parameters   
    AUDIO_CROSS_FADE_CONVERGE_PERIOD_SAMPLES  = 24,     
    AUDIO_CROSS_FADE_TOTAL_PERIOD_MSEC        = 20, /*in ms*/
} CROSS_FADE_PARAMS_DEFAULT;

typedef enum CROSS_FADE_STATE
{
    
    STEADY_STATE    = 0,
    CONVERGENCE_STATE = 1,
    CROSSFADE_STATE     = 2,
} CROSS_FADE_STATE;




typedef struct cross_fade_data_t
{
    int32_t state; 
    uint32_t converge_num_of_samples; // samples left to be processed in the convergence period
    uint32_t sample_count;            // samples processed
    uint32_t total_period_samples; 
    int32_t cross_fade_step_size; 
    int32_t ramp_up_scale_factor; 
    int32_t ramp_down_scale_factor; 

    
} cross_fade_data_t;



typedef struct cross_fade_lib_mem_t 
{
 
    cross_fade_static_t*  cross_fade_static_ptr;    
    uint32_t cross_fade_static_size;                    
    cross_fade_mode_t* cross_fade_mode_ptr;
    uint32_t cross_fade_mode_size;
    cross_fade_config_t* cross_fade_config_ptr;
    uint32_t cross_fade_config_size;
    cross_fade_data_t* cross_fade_data_ptr;       
    uint32_t cross_fade_data_size;                   

    
    
} cross_fade_lib_mem_t; 



/*----------------------------------------------------------------------------
   Function Declarations
----------------------------------------------------------------------------*/

// cross fade process for 16 bits
void audio_cross_fade_16(cross_fade_data_t *cross_fade_data_ptr,
                       int16_t *in1_ptr16,
                       int16_t *in2_ptr16,
                       int16_t *out_ptr16,
                       int32_t block_size);

// cross fade process for 32 bits
void audio_cross_fade_32(cross_fade_data_t *cross_fade_data_ptr,
                       int32_t *in1_ptr32,
                       int32_t *in2_ptr32,
                       int32_t *out_ptr32,
                       int32_t block_size);

// cross fade init
void audio_cross_fade_init ( cross_fade_config_t *cross_fade_config_ptr,
                        cross_fade_data_t *cross_fade_data_ptr,                     
                        int32_t sample_rate );

// cross fade config
void audio_cross_fade_cfg ( cross_fade_config_t *cross_fade_config_ptr,
                        cross_fade_data_t *cross_fade_data_ptr,                     
                        int32_t sample_rate );

// cross fade reset
void audio_cross_fade_reset (cross_fade_data_t *cross_fade_data_ptr);




#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* #ifndef CROSSFADE_H */
