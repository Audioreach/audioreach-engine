#ifndef MSIIR_API_H
#define MSIIR_API_H
/*============================================================================
  @file msiir_api.h

  Public API for Multi Stage IIR Filter (single channel).

        Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
        SPDX-License-Identifier: BSD-3-Clause-Clear

============================================================================*/
#include "msiir_calibration_api.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*----------------------------------------------------------------------------
   Constants
----------------------------------------------------------------------------*/
#define MSIIR_LIB_VER      (0x01000201)   // lib version : 1.2.1
                                       // (major.minor.bug) (8.16.8 bits)

#define MSIIR_Q_PREGAIN    (27)           // pregain Q factor (27)
/*----------------------------------------------------------------------------
   Error code
----------------------------------------------------------------------------*/
typedef enum MSIIR_RESULT {
   MSIIR_SUCCESS = 0,
   MSIIR_FAILURE,
   MSIIR_MEMERROR
} MSIIR_RESULT;

/*----------------------------------------------------------------------------
   Type Declarations
----------------------------------------------------------------------------*/
typedef struct msiir_static_vars_t {   // ** static params
   int32             data_width; 	   //    16 (Q15 PCM) or 32 (Q27 PCM)
   int32             max_stages;       //    max num of stages at setup time
} msiir_static_vars_t;

typedef struct msiir_mem_req_t {       // ** memory requirements
   uint32            mem_size;         //    lib mem size (lib_mem_ptr ->)
   uint32            stack_size;       //    max stack size
} msiir_mem_req_t;

typedef struct msiir_lib_t {           // ** library struct
    void*            mem_ptr;          //    mem pointer
} msiir_lib_t;

/*----------------------------------------------------------------------------
   Function Declarations
----------------------------------------------------------------------------*/

// ** Process one block of samples (one channel)
// lib_ptr: [in] pointer to library structure
// out_ptr: [out] pointer to output sample block
// in_ptr: [in] pointer to input sample block
// samples: [in] number of samples to be processed per channel
MSIIR_RESULT msiir_process_v2(msiir_lib_t *lib_ptr, void *out_ptr, void *in_ptr, uint32 samples);

// ** Get library memory requirements
// mem_req_ptr: [out] pointer to mem requirements structure
// msiir_static_vars_t: [in] pointer to static variable structure
MSIIR_RESULT msiir_get_mem_req(msiir_mem_req_t *mem_req_ptr, msiir_static_vars_t* static_vars_ptr);

// ** Partition, initialize memory, and set default values
// lib_ptr: [in, out] pointer to library structure
// static_vars_ptr: [in] pointer to static variable structure
// mem_ptr: [in] pointer to allocated memory
// mem_size: [in] size of memory allocated
MSIIR_RESULT msiir_init_mem(msiir_lib_t *lib_ptr, msiir_static_vars_t* static_vars_ptr, void* mem_ptr, uint32 mem_size);

// ** Set parameters to library
// lib_ptr: [in, out] pointer to lib structure
// param_id: [in] parameter id
// param_ptr: [in] pointer to the memory where the new values are stored
// param_size:[in] size of the memory pointed by param_ptr
MSIIR_RESULT msiir_set_param(msiir_lib_t* lib_ptr, uint32 param_id, void* param_ptr, uint32 param_size);

// ** Get parameters from library
// lib_ptr: [in] pointer to library structure
// param_id: [in] parameter id
// param_ptr: [out] pointer to the memory where the retrieved value is going to be stored
// param_size:[in] size of the memory pointed by param_ptr
// param_actual_size_ptr: [out] pointer to memory that will hold the actual size of the parameter
MSIIR_RESULT msiir_get_param(msiir_lib_t* lib_ptr, uint32 param_id, void* param_ptr, uint32 param_size, uint32 *param_actual_size_ptr);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* MSIIR_API_H */

