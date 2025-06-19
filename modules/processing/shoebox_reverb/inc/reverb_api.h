/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef REVERB_API_H
#define REVERB_API_H

#include "reverb_calibration_api.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*----------------------------------------------------------------------------
   Library Version
----------------------------------------------------------------------------*/
#define REVERB_LIB_VER  (0x02000004)   // lib version : 2.0.4
                                       // (major.minor.bug) (8.16.8 bits)

/*----------------------------------------------------------------------------
   Error Code and Types
----------------------------------------------------------------------------*/
typedef enum REVERB_RESULT {
   REVERB_SUCCESS = 0,
   REVERB_FAILURE,
   REVERB_MEMERROR,
   REVERB_UNSUPPORTED_PARAM,
} REVERB_RESULT;

typedef enum reverb_mode_t {           // ** reverb topology mode
   INSERT_EFFECT = 0,                  //    insert: cascaded with others
   AUX_EFFECT                          //    aux: parallel with others
} reverb_mode_t;   

/*----------------------------------------------------------------------------
   Topology: Reverb operates in two modes

   1. INSERT_EFFECT: One source in and one out, input/output channels match
      so that reverb output is a mixture of original (dry) sound and reverb 
      (wet) sound. e.g. applying reverb to a music stream.

                       |--------|
      source (dry) --> | reverb | --> mixed output (dry + wet)
                       |--------|

      in this mode, reverb is applied by calling reverb_process function.

   2. AUX_EFFECT: Multiple input sources and one global reverb engine. Each 
      input source makes a copy of itself, with its own (Q15) reverb send gain
      applied, then mixed internally inside the reverb lib. The sound sources
      can have different channel numbers, and don't need to match the reverb
      output channel number. After mixing all inputs, reverb generates dense
      echos (wet sound), and the reverb (wet) output needs to be mixed
      somewhere outside the reverb library with the direct (dry) sound.
      e.g. applying one global reverb for gaming or multi-track MIDI.
               
      source1 -.----------->|------------|     |-------|
      source2 -|-.--------->| dry path   |---->| mixer |--> mixed output
       ...     | |          | processing |     |-------|
      sourceN -|-|-.------->|------------|         ^
               | | |                               |
               | | \-gainN->|------------|         |
               | \---gain2->|   reverb   |---------/
               \-----gain1->|------------| (wet sound only)

      in this mode, reverb is applied by calling reverb_process_aux_input 
      multiple times when mixing each input source, then 
      reverb_process_aux_output is called one time after all the 
      sources are done with mixing for the current block of samples.
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
   Channel Numbers:

   Reverb only works with input/output channels with specific orders defined
   below:
      1: mono [C]
      2: stereo [L, R]
      6: 5.1 channel [L, R, C, LFE, Ls, Rs]
      8: 7.1 channel [L, R, C, LFE, Ls, Rs, Lbk, Rbk]
   other non-standard channel numbers or channel orders should be properly
   converted by wrapper before using reverb library.
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
   Static Types and Lib Mems
----------------------------------------------------------------------------*/

typedef struct reverb_static_vars_t {  // ** static params
   int32             data_width;       //    16 (Q15 PCM) or 32 (Q27 PCM)
   int32             sample_rate;      //    sampling rate
   int32             max_block_size;   //    max block size
   int32             num_chs;          //    num of output channels
   reverb_mode_t     mode;             //    insert or aux effect
} reverb_static_vars_t;

typedef struct reverb_mem_req_t {      // ** memory requirements
   uint32            mem_size;         //    lib mem size (lib_mem_ptr ->)
   uint32            stack_size;       //    stack mem size
} reverb_mem_req_t;

typedef struct reverb_lib_t {          // ** library struct
    void*            mem_ptr;          //    mem pointer
} reverb_lib_t;

/*-----------------------------------------------------------------------------
   Function Declarations
----------------------------------------------------------------------------*/

// ** Mix one input source into aux mode reverb's internal buffers
// Description: The caller should supply input source channel number and 
// reverb send gain (Q15) that comes with this input source. This function 
// can be called one time for each input source and one frame
// lib_ptr: [out] pointer to library structure
// in_ptr: [in] pointer to input sample block (multi channel double ptr)
// in_chs: [in] input source channel number [1, 2, 6, 8]
// send_gain: [in] reverb send gain for this audio source (Q15)
// samples: [in] number of samples to be processed per channel
REVERB_RESULT reverb_process_aux_input(reverb_lib_t *lib_ptr, void **in_ptr, int32 in_chs, int16 send_gain, uint32 samples);

// ** Processing one block of aux mode reverb output 
// Description: before calling this function, the internal input mix buffer 
// should have accumulated samples from all the input sources who wants to 
// have themselves reverberated. This function will process from this buffer
// and produce output. This function is only called once per frame.
// lib_ptr: [in] pointer to library structure
// out_ptr: [out] pointer to output sample block (multi channel double ptr)
// samples: [in] number of samples to be processed per channel
REVERB_RESULT reverb_process_aux_output(reverb_lib_t *lib_ptr, void **out_ptr, uint32 samples);

// ** Processing one block of insert type reverb
// lib_ptr: [in] pointer to library structure
// out_ptr: [out] pointer to output sample block (multi channel double ptr)
// in_ptr: [in] pointer to input sample block (multi channel double ptr)
// samples: [in] number of samples to be processed per channel
REVERB_RESULT reverb_process(reverb_lib_t *lib_ptr, void **out_ptr, void **in_ptr, uint32 samples);


// ** Get library memory requirements
// mem_req_ptr: [out] pointer to mem requirements structure
// static_vars_ptr: [in] pointer to static variable structure
REVERB_RESULT reverb_get_mem_req(reverb_mem_req_t *mem_req_ptr, reverb_static_vars_t* static_vars_ptr);

// ** Partition, initialize memory, and set default values
// lib_ptr: [in, out] pointer to library structure
// static_vars_ptr: [in] pointer to static variable structure
// mem_ptr: [in] pointer to allocated memory
// mem_size: [in] size of memory allocated
REVERB_RESULT reverb_init_mem(reverb_lib_t *lib_ptr, reverb_static_vars_t* static_vars_ptr, void* mem_ptr, uint32 mem_size);

// ** Set parameters to library
// lib_ptr: [in, out] pointer to lib structure
// param_id: [in] parameter id
// param_ptr: [in] pointer to the memory where the new values are stored
// param_size:[in] size of the memory pointed by param_ptr
REVERB_RESULT reverb_set_param(reverb_lib_t* lib_ptr, uint32 param_id, void* param_ptr, uint32 param_size);

// ** Get parameters from library
// lib_ptr: [in] pointer to library structure
// param_id: [in] parameter id
// param_ptr: [out] pointer to the memory where the retrieved value is going to be stored
// param_size:[in] size of the memory pointed by param_ptr
// param_actual_size_ptr: [out] pointer to memory that will hold the actual size of the parameter
REVERB_RESULT reverb_get_param(reverb_lib_t* lib_ptr, uint32 param_id, void* param_ptr, uint32 param_size, uint32 *param_actual_size_ptr);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* REVERB_API_H */
