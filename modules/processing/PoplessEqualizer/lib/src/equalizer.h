/* Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
/****************************************************************************
 * FILE NAME:   equalizer.h                                      
 * DESCRIPTION: EQ internal header file                                    
*****************************************************************************/

#ifndef EQUALIZER_H
#define EQUALIZER_H

/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/

#include "stringl.h"
#include "msiir_api.h"
#include "equalizer_filter_design.h"
#include "crossfade_api.h"


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


#define EQ_LIB_VER   (0x01000202)   // lib version : 1.2.2
                                       // (major.minor.bug) (8.16.8 bits)

static const uint32 eq_max_stack_size = 1050;   // worst case stack mem in bytes
                                       


#define ALIGN8(o)         (((o)+7)&(~7))



/*=============================================================================
Typedefs 
=============================================================================*/


// msiir coeffs struct
typedef struct eq_msiir_coeffs_t 
{        
   int32 iir_coeffs[MSIIR_COEFF_LENGTH]; 
   int32 shift_factor;                 
} eq_msiir_coeffs_t;                      
                        
// msiir config struct
typedef struct eq_msiir_config_t 
{            
   int32             num_stages;       
   // eq_msiir_coeffs_t  coeffs_struct[num_stages]
} eq_msiir_config_t;

// msiir setting struct
typedef struct eq_msiir_settings_t 
{        
	// must follow msiir format where num_bands is followed by band coeffs/shift factors
	eq_msiir_config_t msiir_config;                    
    eq_msiir_coeffs_t msiir_coeffs[EQ_MAX_BANDS];   

} eq_msiir_settings_t; 


// internal state structure
typedef struct eq_states_t 
{        
	uint32	headroom_req_db;

} eq_states_t; 

typedef enum eq_logic_t{
	EQ_FALSE = 0,
	EQ_TRUE
}eq_logic_t;


// EQ lib mem structure
typedef struct eq_lib_mem_t 
{
 	
	eq_static_struct_t     *eq_static_struct_ptr;
	eq_band_internal_specs_t	*eq_band_internal_specs_ptr;
	uint32					eq_band_internal_specs_size;
	eq_config_t				*eq_config_ptr;
	eq_states_t				*eq_states_ptr;
	msiir_lib_t             msiir_lib_mem;     
    cross_fade_lib_t        cross_fade_lib_mem;
	uint32*					eq_audio_fx_freq_ranges_table_ptr;
	eq_mode_t               eq_mode;
	cross_fade_mode_t       eq_on_off_crossfade_mode; // on/off cross-fade related internal state
	
} eq_lib_mem_t; 





#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif  /* EQUALIZER_H */
