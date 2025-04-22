/* =========================================================================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
   ========================================================================== */

#ifndef IIR_MBDRC_CALIBRATION_API
#define IIR_MBDRC_CALIBRATION_API

#include "ar_defs.h"
#include "iir_mbdrc_api.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*----------------------------------------------------------------------------
 * Preprocessor Definitions and Constants
 * -------------------------------------------------------------------------*/
#define MSIIR_MAX_COEFFS_PER_BAND (10) // # of coeffs needed per filter block
	                                   // = #of biquad coeffs * (3 even stages + 2 odd stages)

/*----------------------------------------------------------------------------
 * Pramameter ID Definitions
 * -------------------------------------------------------------------------*/

// param id to get lib version
#define IIR_MBDRC_PARAM_GET_LIB_VER (0)	 // read only
typedef int64_t	iir_mbdrc_lib_ver_t;	 // lib version
                                     // external_major.external_minor.major.minor.revision.custom_alphabet.custom_number.reserved) 
                                     // (8.8.8.8.8.8.8.8 bits)

// this param id is for iir_mbdrc_feature_mode_t
#define IIR_MBDRC_PARAM_FEATURE_MODE (1) // read/write; apply to all channels
typedef enum iir_mbdrc_mode_t
{
	IIR_MBDRC_BYPASSED = 0,              // IIR_MBDRC processing bypassed; no IIR_MBDRC processing
	IIR_MBDRC_ENABLED		             // IIR_MBDRC processing enabled; normal IIR_MBDRC processing
} iir_mbdrc_mode_t;
typedef iir_mbdrc_mode_t	iir_mbdrc_feature_mode_t;

// param id to get/set actual num of band
#define IIR_MBDRC_PARAM_NUM_BAND (2)	 // read/write; apply to all channels;
typedef	uint32_t	iir_mbdrc_num_band_t;

// param id to get/set mute flags
#define IIR_MBDRC_PARAM_MUTE_FLAGS (3)	 // read/write;
typedef struct iir_mbdrc_mute_flags_t
{
	uint32_t   ch_idx;                      // channel index
	int32_t	*mute_flags;                  // total mute_flag[num_band] per channel	
} iir_mbdrc_mute_flags_t;
                                          // read/write the number of band flags


// this param id is for iir_mbdrc_drc_feature_mode_struct_t
#define IIR_MBDRC_PARAM_DRC_FEATURE_MODE (4)  // read/write; apply to all channels
typedef enum iir_mbdrc_drc_mode_t
{
	IIR_MBDRC_DRC_BYPASSED = 0,               // DRC processing bypassed;
	                                      // no DRC processing and only delay is implemented
    IIR_MBDRC_DRC_ENABLED,		              // DRC processing enabled; normal DRC processing
} iir_mbdrc_drc_mode_t;

typedef struct iir_mbdrc_drc_feature_mode_struct_t
{
	uint32_t  ch_idx;                             // channel index: 0 ~ IIR_MBDRC_MAX_BANDS-1
	uint32_t	band_index;		                    // band index: 0 ~ IIR_MBDRC_MAX_CHANNELS-1 (apply same mode to entire channels)
	iir_mbdrc_drc_mode_t iir_mbdrc_drc_feature_mode;	// 1 is with DRC processing;
	                                            // 0 is no DRC processing
	                                            // (bypassed, only delay is implemented)
} iir_mbdrc_drc_feature_mode_struct_t;


// this param id is for iir_mbdrc_drc_config_struct_t
#define IIR_MBDRC_PARAM_DRC_CONFIG (5)	// read/write
typedef struct iir_mbdrc_drc_config_t
{
	// below two should not change during Reinit
	int16_t	channel_linked;		    // Q0  channel mode -- Linked(1) or Not-Linked(0); 
	int16_t	down_sample_level;	    // Q0  Down Sample Level to save MIPS

	uint16_t	rms_tav;			    // Q16 Time Constant used to compute Input RMS
	uint16_t	make_up_gain;		    // Q12 Makeup Gain Value

	int16_t	dn_expa_threshold;      // Q7  downward expansion threshold
	int16_t	dn_expa_slope;          // Q8  downward expansion slope 
	uint32_t	dn_expa_attack;         // Q31 downward expansion attack time 
	uint32_t	dn_expa_release;        // Q31 downward expansion release time 
	int32_t	dn_expa_min_gain_db;    // Q23 downward expansion minimum gain in dB
	uint16_t	dn_expa_hysterisis;     // Q14 downward expansion hysterisis time  

	int16_t	up_comp_threshold;      // Q7  upward compression threshold
	uint32_t	up_comp_attack;         // Q31 upward compression attack time 
	uint32_t	up_comp_release;        // Q31 upward compression release time  
	uint16_t	up_comp_slope;          // Q8  upward compression slope
	uint16_t	up_comp_hysterisis;     // Q14 upward compression hysterisis time  

	int16_t	dn_comp_threshold;      // Q7  downward compression threshold
	uint16_t	dn_comp_slope;          // Q8  downward compression slope
	uint32_t	dn_comp_attack;         // Q31 downward compression attack time 
	uint32_t	dn_comp_release;        // Q31 downward compression release time  
	uint16_t	dn_comp_hysterisis;     // Q14 downward compression hysterisis time  

	int16_t reserved;

} iir_mbdrc_drc_config_t;

typedef struct iir_mbdrc_drc_config_struct_t
{
	uint32_t  ch_idx;                        // channel index: 0 ~ IIR_MBDRC_MAX_BANDS-1
	uint32_t	band_index;		               // band index: 0 ~ IIR_MBDRC_MAX_CHANNELS-1
	iir_mbdrc_drc_config_t	iir_mbdrc_drc_config;
} iir_mbdrc_drc_config_struct_t;


// param id for get/set limiter mode
#define IIR_MBDRC_PARAM_LIMITER_MODE (6)	// read/write
typedef enum iir_mbdrc_limiter_mode_t
{          
	LIM_MAKEUPGAIN_ONLY = 0,
	LIM_NORMAL_PROC
} iir_mbdrc_limiter_mode_t;


// param id for get/set limiter bypass mode
// this id itself has been verified but currently IIR_MBDRC
// does not use limiter in bypass mode

// this id is here to maintain consistency under unified 
// API guideline since limiter lib has this param id

// this id can be used for new feature support in future
#define IIR_MBDRC_PARAM_LIMITER_BYPASS     (7)    // read/write
typedef int32_t iir_mbdrc_limiter_bypass_t;


// param id for limiter cfg params
#define IIR_MBDRC_PARAM_LIMITER_CONFIG (8)	// read/write
typedef struct iir_mbdrc_limiter_tuning_t
{     
	int32_t             ch_idx;           //    channel index
	int32_t             threshold;        //    threshold (16bit: Q15; 32bit:Q27)
	int32_t             makeup_gain;      //    make up gain (Q8)
	int32_t             gc;               //    recovery const (Q15)
	int32_t             max_wait;         //    max wait (Q15 value of sec)

	uint32_t            gain_attack;      //    limiter gain attack time (Q31)
	uint32_t            gain_release;     //    limiter gain release time (Q31)
	uint32_t            attack_coef;      //    limiter gain attack time speed coef (Q15)
	uint32_t            release_coef;     //    limiter gain release time speed coef (Q15)
	int32_t             hard_threshold;   //    threshold for hard limiter (16bit: Q15; 32bit:Q27)
} iir_mbdrc_limiter_tuning_t;

// param id for IIR coeffs
// write only, only set_param is needed since 
// there is no menu to do get_param in ACDB
#define IIR_MBDRC_PARAM_MSIIR_CONFIG (9)

typedef struct iir_mbdrc_iir_config_struct_t {
	uint32_t  ch_idx;                     //  channel index
	uint32_t	band_index;                 // 0 ~ 9
	uint32_t  num_stages[2];              // num_stages[0]: even filter stages; 
                                        // num_stages[1]: odd filter stages
	int32_t   iir_coeffs[MSIIR_MAX_COEFFS_PER_BAND]; // store order: 3-stage even, 2-stage odd.
} iir_mbdrc_iir_config_struct_t;

// param id for get/set crossover freqs
#define IIR_MBDRC_PARAM_CROSSOVER_FREQS (10)	// read/write
typedef struct iir_mbdrc_crossover_freqs_t
{
	uint32_t   ch_idx;                               // channel index
	uint32_t	*crossover_freqs;                      // cross-over frequencies;
} iir_mbdrc_crossover_freqs_t;
                                                   // read/write num_band-1 crossover freqs 

// param id for reset
#define IIR_MBDRC_PARAM_SET_RESET      (11) // write only
// clear below internal mem
// - Even IIR
// - Odd IIR
// - delay IIR
// - DRC
// - limiter

#define IIR_MBDRC_PARAM_PER_CHANNEL_CALIBRATION_MODE    (12) // read/write
typedef enum iir_mbdrc_per_channel_calib_mode_t
{
	IIR_MBDRC_PER_CHANNEL_CALIB_DISABLE = 0,                 // disable per-channel calibration(== old version)
	IIR_MBDRC_PER_CHANNEL_CALIB_ENABLE                       // enable per-channel calibration
}iir_mbdrc_per_channel_calib_mode_t;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // IIR_MBDRC_CALIBRATION_API
