/* Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 SPDX-License-Identifier: BSD-3-Clause-Clear
 */
/*****************************************************************************
 * FILE NAME:   equalizer_api.h
 * DESCRIPTION: EQ API
 *****************************************************************************/
#ifndef EQUALIZER_API_H
#define EQUALIZER_API_H


/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/


#include "equalizer_calibration_api.h"





#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/*=============================================================================
Typedefs 
=============================================================================*/


typedef enum EQ_RESULT 
{
   EQ_SUCCESS = 0,
   EQ_FAILURE,
   EQ_MEMERROR
}EQ_RESULT;


typedef enum EQ_INST 
{
    CUR_INST = 0,
	NEW_INST,
	TOTAL_INST
}EQ_INST;



typedef struct eq_lib_mem_requirements_t
{ 
   
	uint32 lib_mem_size; 
	uint32 lib_stack_size;
} eq_lib_mem_requirements_t;


typedef struct eq_static_struct_t
{
   uint32           data_width; 	   //  16 or 32
   uint32           sample_rate;       //  Hz; up to 192k
   
} eq_static_struct_t;


typedef struct eq_lib_t
{ 
   
    void* lib_mem_ptr; // ptr to the total chunk of lib mem
	
} eq_lib_t;




/*===========================================================================
*     Function Declarations 
* ==========================================================================*/

// get EQ lib mem req
// eq_lib_mem_requirements_ptr: [out] Pointer to lib mem requirements structure
// eq_static_struct_ptr: [in] Pointer to static structure
EQ_RESULT eq_get_mem_req(eq_lib_mem_requirements_t *eq_lib_mem_requirements_ptr, eq_static_struct_t* eq_static_struct_ptr);


// partition and init the mem with params
// eq_lib_ptr: [in, out] Pointer to lib structure
// eq_static_struct_ptr: [in] Pointer to static structure
// mem_ptr: [in] Pointer to the lib memory
// mem_size: [in] Size of the memory pointed by mem_ptr
EQ_RESULT eq_init_memory(eq_lib_t *eq_lib_ptr, eq_static_struct_t* eq_static_struct_ptr, int8* mem_ptr, uint32 mem_size);


// set the params in the lib mem with those pointed by mem_ptr
// eq_lib_ptr: [in, out] Pointer to lib structure
// param_id: [in] ID of the param
// mem_ptr: [in] Pointer to the memory where the values stored are used to set up the params in the lib memory
// mem_size:[in] Size of the memory pointed by mem_ptr
EQ_RESULT eq_set_param(eq_lib_t* eq_lib_ptr, uint32 param_id, int8* mem_ptr, uint32 mem_size);


// retrieve params from lib mem
// eq_lib_ptr: [in] Pointer to lib structure
// param_id: [in] ID of the param
// mem_ptr: [out] Pointer to the memory where params are to be stored
// mem_size:[in] Size of the memory pointed by mem_ptr
// param_size_ptr: [out] Pointer to param size which indicates the size of the retrieved param(s)
EQ_RESULT eq_get_param(eq_lib_t* eq_lib_ptr, uint32 param_id, int8* mem_ptr, uint32 mem_size, uint32 *param_size_ptr);


// EQ processing of single channel audio signal
// eq_lib_ptr: [in] Pointer to lib structure
// out_ptr: [out] Pointer to single channel output PCM samples
// in_ptr: [in] Pointer to single channel input PCM samples
// sample_per_channel: [in] Number of samples to be processed per channel
EQ_RESULT eq_process(eq_lib_t *eq_lib_ptr[TOTAL_INST], int8 *out_ptr, int8 *in_ptr, uint32 sample_per_channel);
// 1. when there is only one EQ instance, it must be put in eq_lib_ptr[CUR_INST] and eq_lib_ptr[NEW_INST] must be set to NULL
// 2. during cross fade processing there would be two instances: eq_lib_ptr[CUR_INST] which must point to current instance and eq_lib_ptr[NEW_INST] which must point to new instance
// 3. after cross fade is finished, eq_lib_ptr[CUR_INST] must point to the new instance and eq_lib_ptr[NEW_INST] must be set to NULL
// 4. to do cross fade, cross fade total period must be set in such a way that following relationship is true otherwise library will return error: sample_per_channel <= cross fade total period(in samples)










#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif  /* EQUALIZER_API_H */
