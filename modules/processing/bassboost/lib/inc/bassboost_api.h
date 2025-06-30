#ifndef BASSBOOST_API_H
#define BASSBOOST_API_H
/*============================================================================
  @file bassboost_api.h

  Public API for Bass Boost Effect

        Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
        SPDX-License-Identifier: BSD-3-Clause-Clear
============================================================================*/
#include "bassboost_calibration_api.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*----------------------------------------------------------------------------
   Library Version
----------------------------------------------------------------------------*/
#define BASSBOOST_LIB_VER (0x01000203) // lib version : rel/1.2/3.1.0
                                       // (major.minor.bug) (8.16.8 bits)

/*----------------------------------------------------------------------------
   Error code
----------------------------------------------------------------------------*/
typedef enum BASSBOOST_RESULT {
   BASSBOOST_SUCCESS = 0,
   BASSBOOST_FAILURE,
   BASSBOOST_MEMERROR
} BASSBOOST_RESULT;

/*----------------------------------------------------------------------------
   Type Declarations
----------------------------------------------------------------------------*/
typedef struct bassboost_static_vars_t {//** static params
   int32             data_width;       //    16 (Q15 PCM) or 32 (Q27 PCM)
   int32             sample_rate;      //    sampling rate
   int32             max_block_size;   //    max block size
   int32             num_chs;          //    num of channels
   int32             limiter_delay;    //    internal limiter delay (ms)
} bassboost_static_vars_t;

typedef struct bassboost_mem_req_t {   // ** memory requirements
   uint32            mem_size;         //    lib mem size (lib_mem_ptr ->)
   uint32            stack_size;       //    stack mem size
} bassboost_mem_req_t;

typedef struct bassboost_lib_t {       // ** library struct
    void*            mem_ptr;          //    mem pointer
} bassboost_lib_t;

/*-----------------------------------------------------------------------------
   Function Declarations
-----------------------------------------------------------------------------*/

// ** Process one block of samples (multi channel)
// lib_ptr: [in] pointer to library structure
// out_ptr: [out] pointer to output sample block (multi channel double ptr)
// in_ptr: [in] pointer to input sample block (multi channel double ptr)
// samples: [in] number of samples to be processed per channel
BASSBOOST_RESULT bassboost_process(bassboost_lib_t *lib_ptr, void **out_ptr, void **in_ptr, uint32 samples);

// ** Get library memory requirements
// mem_req_ptr: [out] pointer to mem requirements structure
// static_vars_ptr: [in] pointer to static variable structure
BASSBOOST_RESULT bassboost_get_mem_req(bassboost_mem_req_t *mem_req_ptr, bassboost_static_vars_t* static_vars_ptr);

// ** Partition, initialize memory, and set default values
// lib_ptr: [in, out] pointer to library structure
// static_vars_ptr: [in] pointer to static variable structure
// mem_ptr: [in] pointer to allocated memory
// mem_size: [in] size of memory allocated
BASSBOOST_RESULT bassboost_init_mem(bassboost_lib_t *lib_ptr, bassboost_static_vars_t* static_vars_ptr, void* mem_ptr, uint32 mem_size);

// ** Set parameters to library
// lib_ptr: [in, out] pointer to lib structure
// param_id: [in] parameter id
// param_ptr: [in] pointer to the memory where the new values are stored
// param_size:[in] size of the memory pointed by param_ptr
BASSBOOST_RESULT bassboost_set_param(bassboost_lib_t* lib_ptr, uint32 param_id, void* param_ptr, uint32 param_size);

// ** Get parameters from library
// lib_ptr: [in] pointer to library structure
// param_id: [in] parameter id
// param_ptr: [out] pointer to the memory where the retrieved value is going to be stored
// param_size:[in] size of the memory pointed by param_ptr
// param_actual_size_ptr: [out] pointer to memory that will hold the actual size of the parameter
BASSBOOST_RESULT bassboost_get_param(bassboost_lib_t* lib_ptr, uint32 param_id, void* param_ptr, uint32 param_size, uint32 *param_actual_size_ptr);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* BASSBOOST_API_H */
