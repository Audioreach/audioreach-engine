/*======================= COPYRIGHT NOTICE =========== ======================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause
===========================================================================*/

#ifndef IIR_MBDRC_H
#define IIR_MBDRC_H

#include "iir_mbdrc_api.h"
#include "drc_api.h"
#include "limiter_api.h"
#include "audio_dsp.h"
#include "audio_apiir_df1_opt.h"


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*----------------------------------------------------------------------------
 * Preprocessor Definitions and Constants
 * -------------------------------------------------------------------------*/
#define IIR_MBDRC_LIB_VER  (0x0400010000000000)  // lib version : 4.0_1.0.0
                                          	 // external_major.external_minor.major.minor.revision.custom_alphabet.custom_number.reserved) 
	                                         // (8.8.8.8.8.8.8.8 bits)


static const uint32 eq_max_stack_size = 600;   // worst case stack mem in bytes
#define ALIGN8(o)         (((o)+7)&(~7))


#define IIR_MBDRC_TIME_CONST 0x1BF7A00 // UL32Q31, same as DN_EXPA_RELEASE_DEFAULT

/* define IIR-related constants */
// max states required per band
#define MAX_MSIIR_STATES_MEM      (32)
// max # of even allpass stage
#define EVEN_AP_STAGE 3
// max # of odd allpass stage
#define ODD_AP_STAGE 2
// use fixed Q27 for all coef
#define IIR_COEFFS_TARGET_Q_FACTOR 27 
// for allpass biquad iir, only b0 and b1 are needed
#define BIQUAD_APIIR_COEFF_LENGTH 2

#define MBDRC_NUM_BAND_DEFAULT 1
/*----------------------------------------------------------------------------
 * Type Declarations
 * -------------------------------------------------------------------------*/

// iir_mbdrc states struct
typedef struct iir_mbdrc_states_mem_t
{
	// dynamic mem allocation for mute_flags[num_channels][IIR_MBDRC_MAX_BANDS]
	int32	**mute_flags; 

	// dynamic mem allocation for iir_mbdrc_cross_over_freqs[num_channels][IIR_MBDRC_MAX_BANDS-1]
	uint32	**iir_mbdrc_cross_over_freqs;

	// dynamic mem allocation for drc_enable[num_channels][IIR_MBDRC_MAX_BANDS]
	uint16	**drc_enable;

	// dynamic mem allocation for iir_mbdrc_makeupgain[num_channels][IIR_MBDRC_MAX_BANDS]
	uint16	**iir_mbdrc_makeupgain;

	// dynamic mem allocation for iir_mbdrc_inst_makeupgain[num_channels][IIR_MBDRC_MAX_BANDS]
	uint16  **iir_mbdrc_inst_makeupgain;

	uint32	num_band;
} iir_mbdrc_states_mem_t;


// iir_mbdrc internal memory struct
typedef struct iir_mbdrc_misc_scratch_buf_t
{
	// LPF output; shared by all bands
	// dynamic mem allocation for int32 *subBandLPBuf[IIR_MBDRC_MAX_CHANNELS]
    int32	 **subBandLPBuf;

	// HPF output, which is input of next LPF; shared by all bands
	// dynamic mem allocation for int32 *subBandHPBuf[IIR_MBDRC_MAX_CHANNELS]
    int32    **subBandHPBuf;

	// IIR filter bank output, which is used for 16-bit case; shared by all bands
	// dynamic mem allocation for int16 *internalOutBuf16[IIR_MBDRC_MAX_CHANNELS]
    int16    **internalOutBuf16;

	// mixer output, which is limiter input; shared by all bands
	// dynamic mem allocation for *internalOutBuf32[IIR_MBDRC_MAX_CHANNELS]
    int32    **internalOutBuf32;

	// shared by all bands/channels
    int32     *scatch_buf; 

} iir_mbdrc_misc_scratch_buf_t;

// iir filter structure
// array is allocated for worse case (9-order IIR design)
typedef struct iir_filter_t
{
	uint32 num_stages[2]; // num_stages[0]: even filter stages;
	                      // num_stages[1]: odd filter stages

	/* according to allpass biquad filter's transfer function:         */
	/* H_ap(z) = (b0 + b1*z^-1 + z^-2)/(1 + b1*z^-1 + b0*z^-2)         */
	/* only b0 and b1 needs to be saved					               */
	/* Besides, the 3-rd stage of even allpass filter is 1st order IIR */
	/* H_even_3rd(z) = (b0 + z^-1)/(1 + b0*z^-1)                       */
	/* only b0 needs to be saved								       */
	/* iir_coeffs store order is:									   */
	/* [b0_even1 b1_even1 | b0_even2 b1_even2 | b0_even3 b1_even3 | ...*/
	/* ...b0_odd1 b1_odd1 | b0_odd2 b1_odd2] */
	int32 iir_coeffs[MSIIR_MAX_COEFFS_PER_BAND];

	/* each biquad IIR needs 4 mem, 1st order IIR needs 2 mem          */
	/* states store order is:										   */
	/* [4 mem for even ap1 | 4 mem for even ap2 | ...                  */
	/* ... 4 mem for 1st order iir | 4 mem for odd ap1 | ...           */
	/* ... 4 mem for odd ap2 | ...                                     */
	/* ... 3 * (4 mem for delay apx)]                                  */
	int32 states[MAX_MSIIR_STATES_MEM];
} iir_filter_t;


// IIR_MBDRC lib mem structure
typedef struct iir_mbdrc_lib_mem_t
{
	iir_mbdrc_static_struct_t			*iir_mbdrc_static_struct_ptr;
    iir_mbdrc_feature_mode_t			*iir_mbdrc_feature_mode_ptr;
	iir_mbdrc_per_channel_calib_mode_t  *iir_mbdrc_per_channel_calib_mode_ptr;

	// dynamic mem allocation for DRC library
	drc_lib_t  					    *drc_lib_linked_mem;     // DRC when per-channel mode is enabled: drc_lib_linked_mem[IIR_MBDRC_MAX_BANDS]
	drc_lib_t  					    **drc_lib_mem;           // DRC when per-channel mode is disabled: drc_lib_mem[num_channel][IIR_MBDRC_MAX_BANDS]
    limiter_lib_t					limiter_lib_mem;         // multi ch ready
	iir_mbdrc_states_mem_t				*iir_mbdrc_states_mem_ptr;
	iir_mbdrc_misc_scratch_buf_t		*iir_mbdrc_misc_scratch_buf_ptr;

	// dynamic mem allocation for iir_filter_t *iir_filter_struct_ptr_array[IIR_MBDRC_MAX_BANDS-1][IIR_MBDRC_MAX_CHANNELS]
	iir_filter_t                    ***iir_filter_struct_ptr_array;

} iir_mbdrc_lib_mem_t;


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // IIR_MBDRC_H
