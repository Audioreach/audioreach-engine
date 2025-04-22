/*============================================================================
  @file msiir.c

  Multi Stage IIR Filter implementation.

        Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
        SPDX-License-Identifier: BSD-3-Clause-Clear
============================================================================*/

/*----------------------------------------------------------------------------
   Includes
----------------------------------------------------------------------------*/
#include "msiir.h"
#include "simple_mm.h"
#include "audio_dsp.h"
#include "audio_dsp32.h"
#include "audio_iir_tdf2.h"
#include "stringl.h"
#ifdef AVS_BUILD_SOS
#include "capi_cmn.h"
#endif
/*----------------------------------------------------------------------------
   Local Functions
----------------------------------------------------------------------------*/
/* clear memorys for state w1 and w2 for all stages */
static void reset(mult_stage_iir_t *obj_ptr)
{
   int32 i, j;
   for (i = 0; i < obj_ptr->num_stages; ++i) {
      for (j = 0; j < MSIIR_FILTER_STATES; ++j) {
         obj_ptr->sos[i].states[j] = 0;
      }
   }
}

/* set default values for the first run */
static void set_default(mult_stage_iir_t *obj_ptr)
{
   obj_ptr->pre_gain = c_unity_pregain;
   obj_ptr->num_stages = 0;
   // by default there is no coeffs, and filter is a passthru
   // when setting stages, coeffs must follow accordingly
   // by using parameter id MSIIR_PARAM_CONFIG
}

/* process for 16 bit data */
static MSIIR_RESULT process_16(mult_stage_iir_t *obj_ptr, int16 *out_ptr, int16 *in_ptr, int32 samples)
{
   iir_data_t* iir_ptr = obj_ptr->sos;
   int16 *sos_in_ptr = in_ptr;
   int32 i;

   // 1. if zero gain, directly output zero
   if (0 == obj_ptr->pre_gain) {
      buffer_empty(out_ptr, samples);
      return MSIIR_SUCCESS;
   }

   // 2. if not unity pregain, apply it and store to output
   if (c_unity_pregain != obj_ptr->pre_gain) {
      buffer16_fill32(out_ptr, in_ptr, obj_ptr->pre_gain, MSIIR_Q_PREGAIN, samples);
      sos_in_ptr = out_ptr; // later, process from the scratch mem (output)
   }

   // 3. process sos sections
   if (obj_ptr->num_stages <= 0) {
      // bypass for invalid stage count
      buffer_copy(out_ptr, sos_in_ptr, samples);
      return MSIIR_SUCCESS;
   }

   for (i = 0; i < obj_ptr->num_stages; i++) {
      // filtering subroutine
      iirTDF2_16(sos_in_ptr, out_ptr, samples,
         &iir_ptr->coeffs[0],
         &iir_ptr->coeffs[MSIIR_NUM_COEFFS],
         &iir_ptr->states[0],
         (int16)iir_ptr->shift_factor,
         MSIIR_DEN_SHIFT);

      // feed correct ptrs for the next iteration
      sos_in_ptr = out_ptr;

      // point to the next biquad
      iir_ptr++;
   }
   return MSIIR_SUCCESS;
}

/* process for 32 bit data */
static MSIIR_RESULT process_32(mult_stage_iir_t *obj_ptr, int32 *out_ptr, int32 *in_ptr, int32 samples)
{
   iir_data_t* iir_ptr = obj_ptr->sos;
   int32 *sos_in_ptr = in_ptr;
   int32 i;

   // 1. if zero gain, output zero
   if (0 == obj_ptr->pre_gain) {
      memset(out_ptr, 0, samples*sizeof(*out_ptr));
      return MSIIR_SUCCESS;
   }

   // 2. if not unity pregain apply it and store to output mem
   if (c_unity_pregain != obj_ptr->pre_gain) {
      buffer32_fill32(out_ptr, in_ptr, obj_ptr->pre_gain, MSIIR_Q_PREGAIN, samples);
      sos_in_ptr = out_ptr; // later, process from the scratch mem (output)
   }

   // 3. process sos sections
   if (obj_ptr->num_stages <= 0) {
      // bypass for invalid stage count
      memscpy(out_ptr,samples*sizeof(*out_ptr), sos_in_ptr, samples*sizeof(*out_ptr));
      return MSIIR_SUCCESS;
   }

   for (i = 0; i < obj_ptr->num_stages; i++) {
      // filtering subroutine
      iirTDF2_32(sos_in_ptr, out_ptr, samples,
         &iir_ptr->coeffs[0],
         &iir_ptr->coeffs[MSIIR_NUM_COEFFS],
         &iir_ptr->states[0],
         (int16)iir_ptr->shift_factor,
         MSIIR_DEN_SHIFT);

      // feed correct pointer for the next iteration
      sos_in_ptr = out_ptr;

      // point to the next biquad
      iir_ptr++;
   }
   return MSIIR_SUCCESS;
}


/*-----------------------------------------------------------------------------
   API Functions
-----------------------------------------------------------------------------*/

// ** Processing one block of samples with IIRTDF2 implementation
MSIIR_RESULT msiir_process_v2(msiir_lib_t *lib_ptr, void *out_ptr, void *in_ptr, uint32 samples)
{
   mult_stage_iir_t *obj_ptr = (mult_stage_iir_t *)lib_ptr->mem_ptr;

   if (16 == obj_ptr->static_vars.data_width) {
      return process_16(obj_ptr, (int16 *)out_ptr, (int16 *)in_ptr, samples);

   } else if (32 == obj_ptr->static_vars.data_width) {
      return process_32(obj_ptr, (int32 *)out_ptr, (int32 *)in_ptr, samples);

   } else {
      return MSIIR_FAILURE;   // invalid data width
   }
}

// ** Get memory requirements
MSIIR_RESULT msiir_get_mem_req(msiir_mem_req_t *mem_req_ptr, msiir_static_vars_t* static_vars_ptr)
{
   uint32 reqsize;

   /* main lib struct */
   reqsize = smm_malloc_size(sizeof(mult_stage_iir_t));

   /* iir data struct for each stage */
   reqsize += smm_calloc_size(static_vars_ptr->max_stages, sizeof(iir_data_t));

   /* store results */
   mem_req_ptr->mem_size   = reqsize;
   mem_req_ptr->stack_size = msiir_max_stack_size;
   return MSIIR_SUCCESS;
}

// ** Partition, initialize memory, and set default values
MSIIR_RESULT msiir_init_mem(msiir_lib_t *lib_ptr, msiir_static_vars_t* static_vars_ptr, void* mem_ptr, uint32 mem_size)
{
   SimpleMemMgr         MemMgr;
   SimpleMemMgr         *smm = &MemMgr;
   msiir_mem_req_t      mem_req;
   mult_stage_iir_t     *obj_ptr;

   /* double check if allocated mem is enough */
   msiir_get_mem_req(&mem_req, static_vars_ptr);
   if (mem_req.mem_size > mem_size) {
      return MSIIR_MEMERROR;
   }

   /* assign initial mem pointers to lib instance */
   lib_ptr->mem_ptr = mem_ptr;
   smm_init(smm, lib_ptr->mem_ptr);
   obj_ptr = (mult_stage_iir_t *)smm_malloc(smm, sizeof(mult_stage_iir_t));

   /* copy static vars */
   obj_ptr->static_vars = *static_vars_ptr;

   /* allocate for sos sections */
   obj_ptr->sos = (iir_data_t *)smm_calloc(
      smm, obj_ptr->static_vars.max_stages, sizeof(iir_data_t));

   /* set default values */
   set_default(obj_ptr);

   return MSIIR_SUCCESS;
}

// ** Set parameters to library
MSIIR_RESULT msiir_set_param(msiir_lib_t* lib_ptr, uint32 param_id, void* param_ptr, uint32 param_size)
{
   mult_stage_iir_t *obj_ptr = (mult_stage_iir_t *)lib_ptr->mem_ptr;
   msiir_coeffs_t *coeffs_ptr;
   msiir_config_t *cfg_ptr;
   int32 i, j, reset_flag;

   switch (param_id) {
   case MSIIR_PARAM_PREGAIN:
      if (param_size == sizeof(msiir_pregain_t)) {
         obj_ptr->pre_gain = *((msiir_pregain_t *)param_ptr);
      } else {
         return MSIIR_MEMERROR;
      }
      break;

   case MSIIR_PARAM_CONFIG:
      cfg_ptr = (msiir_config_t *)param_ptr;
      if (cfg_ptr->num_stages >= 0 && cfg_ptr->num_stages <= obj_ptr->static_vars.max_stages) {
         obj_ptr->num_stages = cfg_ptr->num_stages;
      } else {
         return MSIIR_FAILURE; // invalid num stages must be within [1, max]
      }
      if (param_size == sizeof(msiir_config_t) + obj_ptr->num_stages * sizeof(msiir_coeffs_t)) {

         coeffs_ptr = (msiir_coeffs_t *)((char*)cfg_ptr + sizeof(msiir_config_t));

         reset_flag = 0;
         for (i = 0; i < obj_ptr->num_stages; ++i) {
            for (j = 0; j < MSIIR_COEFF_LENGTH; ++j) {
               // check if denominator has changes
               if (j >= 3 && obj_ptr->sos[i].coeffs[j] != (coeffs_ptr+i)->iir_coeffs[j]) {
                  reset_flag = 1;
               }
               // copy coeffs
               obj_ptr->sos[i].coeffs[j] = (coeffs_ptr+i)->iir_coeffs[j];
            }
            obj_ptr->sos[i].shift_factor = (coeffs_ptr+i)->shift_factor;
         }
         // reset lib after config change
         if (1 == reset_flag) {
            reset(obj_ptr);
         }
      } else {
         return MSIIR_MEMERROR;
      }
      break;

   case MSIIR_PARAM_RESET:
      reset(obj_ptr);
      break;

   default:
      return MSIIR_FAILURE; // invalid id
   }

   return MSIIR_SUCCESS;
}

// ** Get parameters from library
MSIIR_RESULT msiir_get_param(msiir_lib_t* lib_ptr, uint32 param_id, void* param_ptr, uint32 param_size, uint32 *param_actual_size_ptr)
{
   mult_stage_iir_t *obj_ptr = (mult_stage_iir_t *)lib_ptr->mem_ptr;
   msiir_coeffs_t *coeffs_ptr;
   uint32 actual_size;
   int32 i, j;

   switch (param_id) {

   case MSIIR_PARAM_LIB_VER:
      if (param_size >= sizeof(msiir_version_t)) {
         *((msiir_version_t *)param_ptr) = MSIIR_LIB_VER;
         *param_actual_size_ptr = sizeof(msiir_version_t);
      } else {
         return MSIIR_MEMERROR;
      }
      break;

   case MSIIR_PARAM_PREGAIN:
      if (param_size >= sizeof(msiir_pregain_t)) {
         *((msiir_pregain_t *)param_ptr) = obj_ptr->pre_gain;
         *param_actual_size_ptr = sizeof(msiir_pregain_t);
      } else {
         return MSIIR_MEMERROR;
      }
      break;

   case MSIIR_PARAM_CONFIG:
      actual_size = sizeof(msiir_config_t) + obj_ptr->num_stages * sizeof(msiir_coeffs_t);
      if (param_size >= actual_size) {
         // copy num_stages
         ((msiir_config_t *)param_ptr)->num_stages = obj_ptr->num_stages;

         coeffs_ptr = (msiir_coeffs_t *)((char *)param_ptr + sizeof(msiir_config_t));

         for (i = 0; i < obj_ptr->num_stages; ++i) {
            for (j = 0; j < MSIIR_COEFF_LENGTH; ++j) {
               (coeffs_ptr+i)->iir_coeffs[j] = obj_ptr->sos[i].coeffs[j];
            }
            (coeffs_ptr+i)->shift_factor = obj_ptr->sos[i].shift_factor;
         }
         // save size
         *param_actual_size_ptr = actual_size;

      } else {
         return MSIIR_MEMERROR;
      }
      break;

   default:
      return MSIIR_FAILURE; // invalid id
   }

   return MSIIR_SUCCESS;
}
