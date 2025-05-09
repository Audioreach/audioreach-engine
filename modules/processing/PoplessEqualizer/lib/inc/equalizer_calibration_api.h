/* Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 SPDX-License-Identifier: BSD-3-Clause-Clear
 */
/*****************************************************************************
 * FILE NAME:   equalizer_calibration_api.h
 * DESCRIPTION: EQ API
 *****************************************************************************/
#ifndef EQUALIZER_CALIBRATION_API_H
#define EQUALIZER_CALIBRATION_API_H

/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/

#include "AEEStdDef.h"
#include "crossfade_calibration_api.h"


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */




/*=============================================================================
Typedefs 
=============================================================================*/

#define EQ_MAX_PRESET_NAME_LEN      32   
#define	EQ_AUDIO_FX_BAND  5


typedef enum eq_filter_type_t
{
	EQ_TYPE_NONE,
	EQ_BASS_BOOST,
	EQ_BASS_CUT,
	EQ_TREBLE_BOOST,
	EQ_TREBLE_CUT,
	EQ_BAND_BOOST,
	EQ_BAND_CUT
} eq_filter_type_t;




typedef enum eq_settings_t
{
	// qcom settings
	EQ_USER_CUSTOM = -1, // qcom user customization ID
	EQ_PRESET_BLANK,
	EQ_PRESET_CLUB,
	EQ_PRESET_DANCE,
	EQ_PRESET_FULLBASS,
	EQ_PRESET_BASSTREBLE,
	EQ_PRESET_FULLTREBLE,
	EQ_PRESET_LAPTOP,
	EQ_PRESET_LHALL,
	EQ_PRESET_LIVE,
	EQ_PRESET_PARTY,
	EQ_PRESET_POP,
	EQ_PRESET_REGGAE,
	EQ_PRESET_ROCK,
	EQ_PRESET_SKA,
	EQ_PRESET_SOFT,
	EQ_PRESET_SOFTROCK,
	EQ_PRESET_TECHNO,

	EQ_MAX_SETTINGS_QCOM,	// invalid for config


	// audio fx settings
	EQ_USER_CUSTOM_AUDIO_FX,	// audio fx user customization ID
	EQ_PRESET_NORMAL_AUDIO_FX,
	EQ_PRESET_CLASSICAL_AUDIO_FX,
	EQ_PRESET_DANCE_AUDIO_FX,
	EQ_PRESET_FLAT_AUDIO_FX,
	EQ_PRESET_FOLK_AUDIO_FX,
	EQ_PRESET_HEAVYMETAL_AUDIO_FX,
	EQ_PRESET_HIPHOP_AUDIO_FX,
	EQ_PRESET_JAZZ_AUDIO_FX,
	EQ_PRESET_POP_AUDIO_FX,
	EQ_PRESET_ROCK_AUDIO_FX,

	EQ_MAX_SETTINGS_AUDIO_FX	// invalid for config

} eq_settings_t;



// define type for num of band
typedef uint32 eq_num_band_t;



#define EQ_PARAM_GET_LIB_VER      (0)   // read only
typedef int32 eq_lib_ver_t;             // lib version(major.minor.bug); (8bits.16bits.8bits)


// this id is for eq_config_t
#define EQ_PARAM_SET_CONFIG  (1) // write only
typedef struct eq_band_specs_t
{                           
	eq_filter_type_t			filter_type; // from eq_filter_type_t; for audio fx customize, it MUST be only set to EQ_BAND_BOOST
	uint32            freq_millihertz;       // Hz*1000; in terms of Hz, for qcom it is up to fs/2 where fs can be up to 192KHz, for audio fx it must be any of (60, 230, 910, 3600, 14000) depending on which band it is
	int32             gain_millibels;		 // dB*100; range is [-15,-14,...,-1,0,1,...,14,15]*100                       
	int32             quality_factor;        // Q8; must > 0 and only used by band boost/cut; for audio fx it can be left unconfigured since lib will auto configure it
	uint32            band_idx;              // 0 ~  num_bands-1
} eq_band_specs_t;    
typedef struct eq_config_t
{
	uint32			gain_adjust_flag;			// 0 or 1
	int32			eq_pregain;					// Q27
	eq_settings_t	eq_preset_id;				// from eq_settings_t
	eq_num_band_t	num_bands;					// 1 ~ 12 for qcom customize; must be 5 for audio fx customize
	// eq_band_specs_t  eq_band_specs[num_bands]
} eq_config_t;
// gain adjust flag, pre-gain, and preset id are always required to be set for correct processing
// when eq_preset_id is user customization ID, both num_bands and corresponding band specs are also needed
// when eq_preset_id is any of the presets, num_bands and corresponding band specs are not needed


// when EQ needs to be changed, crossfade is needed and this needs to be set to 1
#define EQ_PARAM_CROSSFADE_FLAG  (2) // read/write
typedef uint32 eq_cross_fade_mode_t;	// 0 is no cross fade, 1 is cross fade


// when crossfade is needed, need to set num of converge samples and total crossfade time
#define EQ_PARAM_CROSSFADE_CONFIG  (3) // read/write
typedef struct eq_cross_fade_config_t
{
	uint32  eq_cross_fade_converge_num_samples;      //  number of samples from current instance output
	uint32  eq_cross_fade_total_period_msec;         //  ms; T_convergence + T_crossfade
} eq_cross_fade_config_t;


// get needed headroom based on EQ gain settings
// EQ library does not use this value. EQ library only computes the value and stores in the library
// This value is computed only when gain_adjust_flag==1 and when audio fx EQ is used
#define EQ_PARAM_GET_HEADROOM_REQ  (4) // read only
typedef uint32 eq_headroom_req_t;	// millibels = dB*100

// get the current num of band in EQ
#define EQ_PARAM_GET_NUM_BAND  (5) // read only
// payload is defined at top

// get the max num of band EQ can support
#define EQ_PARAM_GET_MAX_NUM_BAND  (6) // read only
typedef uint32 eq_max_num_band_t;


// this id is for eq_band_levels_t
// get the gains used by EQ bands
#define EQ_PARAM_GET_BAND_LEVELS  (7) // read only
typedef int32  eq_level_t;    // millibels = dB*100
typedef struct eq_band_levels_t
{
	uint32		num_bands;
	// eq_level_t  levels[num_bands]
} eq_band_levels_t;


// get the gain range EQ can support, which is -1500 ~ 1500 millibels
#define EQ_PARAM_GET_BAND_LEVEL_RANGE  (8) // read only
typedef struct eq_band_level_range_t
{
	int32		min_level_millibels;	// dB*100
	int32		max_level_millibels;	// dB*100
} eq_band_level_range_t;


// this id is for eq_band_freqs_t
// get the freqs used by EQ bands
#define EQ_PARAM_GET_BAND_FREQS  (9) // read only
typedef uint32  eq_freq_t;    // millihertz = Hz*1000
typedef struct eq_band_freqs_t
{
	uint32		num_bands;
	// eq_freq_t  freqs[num_bands]
} eq_band_freqs_t;



// this id is for eq_band_freq_range_t
#define EQ_PARAM_GET_BAND_FREQ_RANGE		(10) // read only
typedef struct eq_freq_range_t
{
	uint32		min_freq_millihertz;	// Hz*1000
	uint32		max_freq_millihertz;	// Hz*1000
} eq_freq_range_t;  
typedef struct eq_band_freq_range_t		// given a band index, return the corresponding freq range
{
	uint32		band_idx;	
	eq_freq_range_t		eq_band_freq_range;
} eq_band_freq_range_t;


// given a freq, return the corresponding band index
#define EQ_PARAM_GET_BAND		(11) // read only
typedef struct eq_band_index_t		
{
	uint32		freq_millihertz;	// Hz*1000
	uint32		band_idx;	
} eq_band_index_t;


// get preset ID
#define EQ_PARAM_GET_PRESET_ID  (12) // read only
typedef eq_settings_t eq_preset_id_t;

// get num of presets EQ can support
#define EQ_PARAM_GET_NUM_PRESETS  (13) // read only
typedef uint32 eq_num_preset_t;

// get preset name
#define EQ_PARAM_GET_PRESET_NAME  (14) // read only
typedef struct eq_preset_name_t
{
	char preset_name[EQ_MAX_PRESET_NAME_LEN];
} eq_preset_name_t;


// clear(flush) msiir stages internal states
// reset is NOT supported during cross fading, library will return error
#define EQ_PARAM_SET_RESET       (15)  // write only

// set EQ mode
// must be used for enable/disable of EQ effect
// must NOT be used for preset crossfading to new EQ effect
#define EQ_PARAM_SET_MODE_EQ_ONOFF            (16)  // write only
typedef enum eq_mode_t
{
   EQ_DISABLE  =    0,     /* Turn off EQ */
   EQ_ENABLE               /* Turn on EQ */
}eq_mode_t;


// get headroom requirement, can be used independent of library instance
// this ID uses eq_config_t as payload
#define EQ_PARAM_COMPUTE_HEADROOM_REQ       (17)  // read only
// 1. When this ID is used by get_param API, the memory pointer is used as both input and output. This ID does not utilize the library pointer.
// 2. Upon get_param API call, the memory pointer is used as input which points to the provided payload. 
// 3. After get_param API call completes, the API will use the returned headroom(in millibels == dB*100) to overwrite the provided payload


// set EQ mode
// payload is eq_mode_t
// must be used for preset crossfading to new EQ effect
// must NOT be used for enable/disable of EQ effect
#define EQ_PARAM_SET_MODE_EQ_CHANGE                  (18)  // write only
// check EQ state
// EQ is considered disable only when mode is disable and on/off crossfade is inactive
// so during off-crossfade period, even though EQ mode is disable, this param ID will return enable
#define EQ_PARAM_CHECK_STATE           (19)  // read only
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif  /* EQUALIZER_CALIBRATION_API_H */
