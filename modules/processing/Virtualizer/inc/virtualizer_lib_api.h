/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef VIRTUALIZER_API_H
#define VIRTUALIZER_API_H

/*
 * This file was renamed by fw team from virtualizer_api.h to avoid
 * compilation issues on windows
 */
/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/
#include "virtualizer_calibration_api.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*----------------------------------------------------------------------------
   Library Version
----------------------------------------------------------------------------*/
#define VIRTUALIZER_LIB_VER (0x01000300)  // lib version : 1.3.0
                                          // (major.minor.bug) (8.16.8 bits)

/*----------------------------------------------------------------------------
   Error code
----------------------------------------------------------------------------*/
typedef enum VIRTUALIZER_RESULT {
   VIRTUALIZER_SUCCESS = 0,
   VIRTUALIZER_FAILURE,
   VIRTUALIZER_MEMERROR,
} VIRTUALIZER_RESULT;

/*----------------------------------------------------------------------------
   Channel Numbers:

   Virtualizer only works with input/output channels with specific orders 
   defined below:
      1: mono [C]
      2: stereo [L, R]
      6: 5.1 channel [L, R, C, LFE, Ls, Rs]
   other non-standard channel numbers or channel orders should be properly
   converted by wrapper before using virtualizer library.
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
   Type Declarations
----------------------------------------------------------------------------*/

typedef struct _virtualizer_static_vars_t {  // ** static params
   int32             data_width;             //   16 (Q15 PCM) or 32 (Q27 PCM)
   int32             sample_rate;            //   sampling rate [22, 32, 44, 48]
   int32             max_block_size;         //   max processing block size
   int32             out_chs;                //   # of output chs [1, 2]
   int32             in_chs;                 //   # of input chs [1, 2, 6]
   int32             limiter_delay_ms;       //   limiter delay in ms [0, 20]
} virtualizer_static_vars_t;

typedef struct _virtualizer_mem_req_t {      // ** memory requirements
   uint32            mem_size;               //    lib mem size (lib_mem_ptr ->)
   uint32            stack_size;             //    max stack mem size
} virtualizer_mem_req_t;

typedef struct _virtualizer_lib_t {          // ** library struct
    void*            mem_ptr;                //    mem pointer
} virtualizer_lib_t;

/*-----------------------------------------------------------------------------
   Function Declarations
-----------------------------------------------------------------------------*/

// ** Process one block of samples (multi channel)
// lib_ptr: [in] pointer to library structure
// out_ptr: [out] pointer to output sample block (multi channel double ptr)
// in_ptr: [in] pointer to input sample block (multi channel double ptr)
// samples: [in] number of samples to be processed per channel
VIRTUALIZER_RESULT virtualizer_process(virtualizer_lib_t *lib_ptr, void **out_ptr, void **in_ptr, uint32 samples);

// ** Get library memory requirements
// mem_req_ptr: [out] pointer to mem requirements structure
// static_vars_ptr: [in] pointer to static variable structure
VIRTUALIZER_RESULT virtualizer_get_mem_req(virtualizer_mem_req_t *mem_req_ptr, virtualizer_static_vars_t* static_vars_ptr);

// ** Partition, initialize memory, and set default values
// lib_ptr: [in, out] pointer to library structure
// static_vars_ptr: [in] pointer to static variable structure
// mem_ptr: [in] pointer to allocated memory
// mem_size: [in] size of memory allocated
VIRTUALIZER_RESULT virtualizer_init_mem(virtualizer_lib_t *lib_ptr, virtualizer_static_vars_t* static_vars_ptr, void* mem_ptr, uint32 mem_size);

// ** Set parameters to library
// lib_ptr: [in, out] pointer to lib structure
// param_id: [in] parameter id
// param_ptr: [in] pointer to the memory where the new values are stored
// param_size:[in] size of the memory pointed by param_ptr
VIRTUALIZER_RESULT virtualizer_set_param(virtualizer_lib_t* lib_ptr, uint32 param_id, void* param_ptr, uint32 param_size);

// ** Get parameters from library
// lib_ptr: [in] pointer to library structure
// param_id: [in] parameter id
// param_ptr: [out] pointer to the memory where the retrieved value is going to be stored
// param_size:[in] size of the memory pointed by param_ptr
// param_actual_size_ptr: [out] pointer to memory that will hold the actual size of the parameter
VIRTUALIZER_RESULT virtualizer_get_param(virtualizer_lib_t* lib_ptr, uint32 param_id, void* param_ptr, uint32 param_size, uint32 *param_actual_size_ptr);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* VIRTUALIZER_API_H */
