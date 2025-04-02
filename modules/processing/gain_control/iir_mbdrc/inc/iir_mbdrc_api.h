/*======================= COPYRIGHT NOTICE =========== ======================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause
===========================================================================*/

#ifndef IIR_MBDRC_API_H
#define IIR_MBDRC_API_H

#include "iir_mbdrc_calibration_api.h"
#include "iir_mbdrc_function_defines.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/*----------------------------------------------------------------------------
 * Type Declarations
 * -------------------------------------------------------------------------*/
typedef enum IIR_MBDRC_RESULT
{
	IIR_MBDRC_SUCCESS = 0,
	IIR_MBDRC_FAILURE,
	IIR_MBDRC_MEMERROR
}IIR_MBDRC_RESULT;

//IIR_MBDRC static params structure
typedef struct iir_mbdrc_static_struct_t
{
	uint32_t   data_width;				         // 16 or 32
	uint32_t   sample_rate;				         // Hz
	uint32_t	 num_channel;				         // num of channel; up to 8
    uint32_t	*drc_delay_samples;	                 // dynamic mem allocation for drc_delay_samples * max_num_band;
                                                 // samples per channel in Q0
	uint32_t   max_num_band;				         // max num of band; 1 ~ 10
	uint32_t	 limiter_delay_secs;		         // seconds per channel in Q15, the same for all channels
	uint32_t	 max_block_size;			         // for limiter
	uint32_t   limiter_history_winlen;             // length of history window (Q15 value of sec)
} iir_mbdrc_static_struct_t;


// IIR_MBDRC lib structure
typedef struct iir_mbdrc_lib_t
{ 
	void* iir_mbdrc_lib_mem_ptr; // ptr to the total chunk of lib mem
} iir_mbdrc_lib_t;


// IIR_MBDRC lib mem requirements structure
typedef struct iir_mbdrc_lib_mem_requirements_t
{ 
	uint32_t iir_mbdrc_lib_mem_size; // size of the lib mem pointed by lib_mem_ptr
	uint32_t iir_mbdrc_lib_stack_size; // stack mem size
} iir_mbdrc_lib_mem_requirements_t;


/*----------------------------------------------------------------------------
 * Function Declarations and Documentation
 * -------------------------------------------------------------------------*/

// get lib mem req
// iir_mbdrc_lib_mem_requirements_ptr: [out] Pointer to lib mem requirements structure
// iir_mbdrc_static_struct_ptr: [in] Pointer to static structure
IIR_MBDRC_RESULT iir_mbdrc_get_mem_req(iir_mbdrc_lib_mem_requirements_t *iir_mbdrc_lib_mem_requirements_ptr, iir_mbdrc_static_struct_t *iir_mbdrc_static_struct_ptr);


// partition and init the mem with default values
// iir_mbdrc_lib_ptr: [in, out] Pointer to lib structure
// iir_mbdrc_static_struct_ptr: [in] Pointer to static structure
// mem_ptr: [in] Pointer to lib memory
// mem_size: [in] Size of the memory pointed by mem_ptr
IIR_MBDRC_RESULT iir_mbdrc_init_memory(iir_mbdrc_lib_t *iir_mbdrc_lib_ptr, iir_mbdrc_static_struct_t *iir_mbdrc_static_struct_ptr, int8_t *mem_ptr, uint32_t mem_size);


// retrieve params from lib mem
// iir_mbdrc_lib_ptr: [in] Pointer to lib structure
// param_id: [in] ID of the param
// mem_ptr: [in/out] Pointer to the memory where params are to be stored
// mem_size:[in] Size of the memory pointed by mem_ptr
// param_size_ptr: [out] Pointer to param size which indicates the size of the retrieved param(s)
IIR_MBDRC_RESULT iir_mbdrc_get_param(iir_mbdrc_lib_t *iir_mbdrc_lib_ptr, uint32_t param_id, int8_t *mem_ptr, uint32_t mem_size, uint32_t *param_size_ptr);


// set the params in the lib mem
// iir_mbdrc_lib_ptr: [in, out] Pointer to lib structure
// param_id: [in] ID of the param
// mem_ptr: [in] Pointer to the memory where the values stored are used to set up the params in the lib memory
// mem_size:[in] Size of the memory pointed by mem_ptr
IIR_MBDRC_RESULT iir_mbdrc_set_param(iir_mbdrc_lib_t *iir_mbdrc_lib_ptr, uint32_t param_id, int8_t *mem_ptr, uint32_t mem_size);


// IIR_MBDRC processing of de-interleaved multi-channel audio signal
// iir_mbdrc_lib_ptr: [in] Pointer to lib structure
// out_ptr: [out] Pointer to de-interleaved multi-channel output PCM samples
// in_ptr: [in] Pointer to de-interleaved multi-channel input PCM samples
// block_size: [in] Number of samples to be processed per channel
IIR_MBDRC_RESULT iir_mbdrc_process(iir_mbdrc_lib_t *iir_mbdrc_lib_ptr, int8_t *out_ptr[], int8_t *in_ptr[], uint32_t block_size);


#ifdef IIR_MBDRC_SPLITFILTER32_LOOP_ASM
void iir_mbdrc_SplitFilter32_loop(int32_t *tmpOddOut,
								  int32_t *tmpEvenOut,
								  uint32_t nSampleCnt);
#endif
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // IIR_MBDRC_API_H
