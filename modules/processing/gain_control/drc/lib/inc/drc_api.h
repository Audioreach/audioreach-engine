#ifndef DRCAPI_H
#define DRCAPI_H
/*============================================================================
  @file CDrcApi.h

  Public api for DRC.

        Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
		SPDX-License-Identifier: BSD-3-Clause
============================================================================*/

/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/

#include "drc_calib_api.h"


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */



/*----------------------------------------------------------------------------
 * Type Declarations
 * -------------------------------------------------------------------------*/


typedef enum DRC_RESULT 
{
   DRC_SUCCESS = 0,
   DRC_FAILURE,
   DRC_MEMERROR
}DRC_RESULT;

typedef enum data_width_type
{
    BITS_16 = 0, // 16-bits sample
    BITS_32      // 32-bits sample
} data_width_type;


//DRC static params structure
typedef struct drc_static_struct_t
{
   
   data_width_type  data_width;          // 0(16-bits sample) or 1(32-bits sample)
   uint32_t   sample_rate;                 // Hz   
   uint32_t   num_channel;                 // Q0 Number of channels  
   uint32_t   delay;                       // Q0 Delay in samples per channel
   
} drc_static_struct_t;





// DRC lib structure
typedef struct drc_lib_t
{ 
   
    void* lib_mem_ptr; // ptr to the total chunk of lib mem
    
} drc_lib_t;



// DRC lib mem requirements structure
typedef struct drc_lib_mem_requirements_t
{ 
   
    uint32_t lib_mem_size; // size of the lib mem pointed by lib_mem_ptr
    uint32_t lib_stack_size; // stack mem size
} drc_lib_mem_requirements_t;


/*----------------------------------------------------------------------------
 * Function Declarations and Documentation
 * -------------------------------------------------------------------------*/

// DRC processing of de-interleaved multi-channel input audio signal sample by sample
// drc_lib_ptr: [in] Pointer to lib structure
// out_ptr: [out] Pointer to de-interleaved multi - channel output PCM samples
// in_ptr: [in] Pointer to de-interleaved multi - channel input PCM samples
// sample_per_channel: [in] Number of samples to be processed per channel
DRC_RESULT drc_process(drc_lib_t *drc_lib_ptr, int8_t **out_ptr, int8_t **in_ptr, uint32_t sample_per_channel);


// get DRC lib mem size
// drc_lib_mem_requirements_ptr: [out] Pointer to lib mem requirements structure
// static_struct_ptr: [in] Pointer to static structure
DRC_RESULT drc_get_mem_req(drc_lib_mem_requirements_t *drc_lib_mem_requirements_ptr, drc_static_struct_t* drc_static_struct_ptr);

// partition and init the mem with params
// drc_lib_ptr: [in, out] Pointer to lib structure
// static_struct_ptr: [in] Pointer to static structure
// mem_ptr: [in] Pointer to the lib memory
// mem_size: [in] Size of the memory pointed by mem_ptr
DRC_RESULT drc_init_memory(drc_lib_t *drc_lib_ptr, drc_static_struct_t *drc_static_struct_ptr, int8_t *mem_ptr, uint32_t mem_size);

// set the params in the lib mem with those pointed by mem_ptr
// drc_lib_ptr: [in, out] Pointer to lib structure
// param_id: [in] ID of the param
// mem_ptr: [in] Pointer to the memory where the values stored are used to set up the params in the lib memory
// mem_size:[in] Size of the memory pointed by mem_ptr
DRC_RESULT drc_set_param(drc_lib_t *drc_lib_ptr, uint32_t param_id, int8_t *mem_ptr, uint32_t mem_size);

// retrieve params from lib mem
// drc_lib_ptr: [in] Pointer to lib structure
// param_id: [in] ID of the param
// mem_ptr: [out] Pointer to the memory where params are to be stored
// mem_size:[in] Size of the memory pointed by mem_ptr
// param_size_ptr: [out] Pointer to param size which indicates the size of the retrieved param(s)
DRC_RESULT drc_get_param(drc_lib_t *drc_lib_ptr, uint32_t param_id, int8_t *mem_ptr, uint32_t mem_size, uint32_t *param_size_ptr);






#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* #ifndef DRCAPI_H */
