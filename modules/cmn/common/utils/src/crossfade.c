/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*============================================================================
  FILE:          crossfade.c

  OVERVIEW:      Cross fading of two input signals and generate an output.

  DEPENDENCIES:  None
============================================================================*/

/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/
#if defined CAPI_STANDALONE
#include "capi_util.h"
#elif defined APPI_EXAMPLE_STANDALONE
#include "appi_util.h"
#else
#include <stringl.h>
#endif
#include "crossfade.h"
#include "audio_basic_op.h"
#include "audio_divide_qx.h"

/*----------------------------------------------------------------------------
 * Function Definitions
 * -------------------------------------------------------------------------*/



/*======================================================================

  FUNCTION      audio_cross_fade_get_mem_req

  DESCRIPTION   Determine lib mem size. Called once at audio connection set up time.

  DEPENDENCIES  Input pointers must not be NULL.

  PARAMETERS    cross_fade_lib_mem_req_ptr: [out] Pointer to lib mem requirement structure
                cross_fade_static_ptr: [in] Pointer to static structure.

  SIDE EFFECTS  None

======================================================================*/
CROSS_FADE_RESULT audio_cross_fade_get_mem_req(cross_fade_lib_mem_req_t *cross_fade_lib_mem_req_ptr, cross_fade_static_t* cross_fade_static_ptr)
{


    uint32_t cross_fade_lib_mem_size, cross_fade_static_size, cross_fade_mode_size, cross_fade_config_size, cross_fade_data_size, total_mem_size=0;



    // clear memory
    memset(cross_fade_lib_mem_req_ptr,0,sizeof(cross_fade_lib_mem_req_t));

    //------
    //cross_fade_lib_mem_t
    //------
    //cross_fade_static_t
    //------
    //cross_fade_mode_t
    //------
    //cross_fade_config_t
    //------
    //cross_fade_data_t
    //------

    cross_fade_lib_mem_size = ALIGN8(sizeof(cross_fade_lib_mem_t));
    total_mem_size += cross_fade_lib_mem_size;

    cross_fade_static_size = ALIGN8(sizeof(cross_fade_static_t));
    total_mem_size += cross_fade_static_size;

    cross_fade_mode_size = ALIGN8(sizeof(cross_fade_mode_t));
    total_mem_size += cross_fade_mode_size;

    cross_fade_config_size = ALIGN8(sizeof(cross_fade_config_t));
    total_mem_size += cross_fade_config_size;

    cross_fade_data_size = ALIGN8(sizeof(cross_fade_data_t));
    total_mem_size += cross_fade_data_size;



    // store total lib mem size
    cross_fade_lib_mem_req_ptr->cross_fade_lib_mem_size = total_mem_size;


    // maximal lib stack mem consumption
    cross_fade_lib_mem_req_ptr->cross_fade_lib_stack_size = cross_fade_max_stack_size;



    return CROSS_FADE_SUCCESS;
}


/*======================================================================

  FUNCTION      audio_cross_fade_init_memory

  DESCRIPTION   Performs partition and initialization of lib memory.
                Called once at audio connection set up time.

  DEPENDENCIES  Input pointers must not be NULL.

  PARAMETERS    cross_fade_lib_ptr: [in, out] Pointer to lib structure
                cross_fade_static_ptr: [in] Pointer to static structure
                mem_ptr: [in] Pointer to the lib memory
                mem_size: [in] Size of the memory pointed by mem_ptr

  SIDE EFFECTS  None

======================================================================*/
CROSS_FADE_RESULT audio_cross_fade_init_memory(cross_fade_lib_t *cross_fade_lib_ptr, cross_fade_static_t* cross_fade_static_ptr, int8_t* mem_ptr, uint32_t mem_size)
{
    cross_fade_lib_mem_t* cross_fade_lib_mem_ptr = NULL;
    int8_t *temp_ptr=mem_ptr;

    uint32_t cross_fade_lib_mem_size, cross_fade_static_size, cross_fade_mode_size, cross_fade_config_size, cross_fade_data_size, total_mem_size=0;


    // re-calculate lib mem size
    cross_fade_lib_mem_size = ALIGN8(sizeof(cross_fade_lib_mem_t));
    total_mem_size += cross_fade_lib_mem_size;

    cross_fade_static_size = ALIGN8(sizeof(cross_fade_static_t));
    total_mem_size += cross_fade_static_size;

    cross_fade_mode_size = ALIGN8(sizeof(cross_fade_mode_t));
    total_mem_size += cross_fade_mode_size;

    cross_fade_config_size = ALIGN8(sizeof(cross_fade_config_t));
    total_mem_size += cross_fade_config_size;

    cross_fade_data_size = ALIGN8(sizeof(cross_fade_data_t));
    total_mem_size += cross_fade_data_size;


    // error out if the mem space given is not enough
    if (mem_size < total_mem_size)
    {

        return CROSS_FADE_MEMERROR;
    }


    // before initializing lib_mem_ptr, it is FW job to make sure that pMem is 8 bytes aligned(with enough space)
    memset(mem_ptr,0,mem_size);                                   // clear the mem
    cross_fade_lib_ptr->cross_fade_lib_mem_ptr = (void*)mem_ptr;


    //------
    //cross_fade_lib_mem_t
    //------
    //cross_fade_static_t
    //------
    //cross_fade_mode_t
    //------
    //cross_fade_config_t
    //------
    //cross_fade_data_t
    //------


    // lib memory partition starts here
    cross_fade_lib_mem_ptr = (cross_fade_lib_mem_t*)cross_fade_lib_ptr->cross_fade_lib_mem_ptr;
    temp_ptr += cross_fade_lib_mem_size;

    cross_fade_lib_mem_ptr->cross_fade_static_ptr = (cross_fade_static_t*)temp_ptr;
    cross_fade_lib_mem_ptr->cross_fade_static_size = cross_fade_static_size;
    cross_fade_lib_mem_ptr->cross_fade_static_ptr->sample_rate = cross_fade_static_ptr->sample_rate;
    cross_fade_lib_mem_ptr->cross_fade_static_ptr->data_width = cross_fade_static_ptr->data_width;
    temp_ptr += cross_fade_lib_mem_ptr->cross_fade_static_size;

    cross_fade_lib_mem_ptr->cross_fade_mode_ptr = (cross_fade_mode_t*)temp_ptr;
    cross_fade_lib_mem_ptr->cross_fade_mode_size = cross_fade_mode_size;
    *cross_fade_lib_mem_ptr->cross_fade_mode_ptr = 0;
    temp_ptr += cross_fade_lib_mem_ptr->cross_fade_mode_size;

    cross_fade_lib_mem_ptr->cross_fade_config_ptr = (cross_fade_config_t*)temp_ptr;
    cross_fade_lib_mem_ptr->cross_fade_config_size = cross_fade_config_size;
    cross_fade_lib_mem_ptr->cross_fade_config_ptr->converge_num_samples = AUDIO_CROSS_FADE_CONVERGE_PERIOD_SAMPLES;
    cross_fade_lib_mem_ptr->cross_fade_config_ptr->total_period_msec = AUDIO_CROSS_FADE_TOTAL_PERIOD_MSEC;
    temp_ptr += cross_fade_lib_mem_ptr->cross_fade_config_size;

    cross_fade_lib_mem_ptr->cross_fade_data_ptr = (cross_fade_data_t*)temp_ptr;
    cross_fade_lib_mem_ptr->cross_fade_data_size = cross_fade_data_size;
    audio_cross_fade_init(cross_fade_lib_mem_ptr->cross_fade_config_ptr, cross_fade_lib_mem_ptr->cross_fade_data_ptr,  (int32_t)cross_fade_static_ptr->sample_rate);
    temp_ptr += cross_fade_lib_mem_ptr->cross_fade_data_size;


    // check to see if memory partition is correct
    if (temp_ptr != mem_ptr + total_mem_size)
    {

        return CROSS_FADE_MEMERROR;
    }


    return CROSS_FADE_SUCCESS;
}


/*======================================================================

  FUNCTION      audio_cross_fade_get_param

  DESCRIPTION   Get the params from lib mem

  DEPENDENCIES  Input pointers must not be NULL.

  PARAMETERS    cross_fade_lib_ptr: [in] Pointer to lib structure
                param_id: [in] ID of the param
                mem_ptr: [out] Pointer to the memory where params are to be stored
                mem_size:[in] Size of the memory pointed by mem_ptr
                param_size_ptr: [out] Pointer to param size which indicates the size of the retrieved param(s)

  SIDE EFFECTS  None

======================================================================*/
CROSS_FADE_RESULT audio_cross_fade_get_param(cross_fade_lib_t *cross_fade_lib_ptr, uint32_t param_id, int8_t* mem_ptr, uint32_t mem_size, uint32_t *param_size_ptr)
{

    memset(mem_ptr,0,mem_size);

    switch (param_id)
    {
        case CROSS_FADE_PARAM_LIB_VER:
        {
            // check if the memory buffer has enough space to write the parameter data
            if(mem_size >= sizeof(cross_fade_lib_ver_t))
            {
                cross_fade_lib_ver_t* cross_fade_lib_ver_ptr = (cross_fade_lib_ver_t *)mem_ptr;
                *cross_fade_lib_ver_ptr = CROSS_FADE_LIB_VER;
                *param_size_ptr = sizeof(cross_fade_lib_ver_t);

            }
            else
            {

                return CROSS_FADE_MEMERROR;
            }
            break;
        }
        case CROSS_FADE_PARAM_MODE:
        {
            // check if the memory buffer has enough space to write the parameter data
            if(mem_size >= sizeof(cross_fade_mode_t))
            {
                memscpy(mem_ptr, sizeof(cross_fade_mode_t), ((cross_fade_lib_mem_t*)(cross_fade_lib_ptr->cross_fade_lib_mem_ptr))->cross_fade_mode_ptr, sizeof(cross_fade_mode_t));
                *param_size_ptr = sizeof(cross_fade_mode_t);

            }
            else
            {

                return CROSS_FADE_MEMERROR;
            }
            break;
        }
        case CROSS_FADE_PARAM_CONFIG:
        {
            // check if the memory buffer has enough space to write the parameter data
            if(mem_size >= sizeof(cross_fade_config_t))
            {
                    memscpy(mem_ptr, sizeof(cross_fade_config_t), ((cross_fade_lib_mem_t*)(cross_fade_lib_ptr->cross_fade_lib_mem_ptr))->cross_fade_config_ptr, sizeof(cross_fade_config_t));
                    *param_size_ptr = sizeof(cross_fade_config_t);

            }
            else
            {

                return CROSS_FADE_MEMERROR;
            }
            break;
        }
        default:
        {

            return CROSS_FADE_FAILURE;
        }
    }



    return CROSS_FADE_SUCCESS;
}


/*======================================================================

  FUNCTION      audio_cross_fade_set_param

  DESCRIPTION   Set the params in the lib memory

  DEPENDENCIES  Input pointers must not be NULL.

  PARAMETERS    cross_fade_lib_ptr: [in, out] Pointer to lib structure
                param_id: [in] ID of the param
                mem_ptr: [in] Pointer to the memory where the values stored are used to set up the params in the lib memory
                mem_size:[in] Size of the memory pointed by mem_ptr

  SIDE EFFECTS  None

======================================================================*/
CROSS_FADE_RESULT audio_cross_fade_set_param(cross_fade_lib_t *cross_fade_lib_ptr, uint32_t param_id, int8_t* mem_ptr, uint32_t mem_size)
{


    switch (param_id)
    {
        case CROSS_FADE_PARAM_MODE:
        {
            // check if the memory buffer has enough space to write the parameter data
            if(mem_size == sizeof(cross_fade_mode_t))
            {
                // set the calibration params in the lib memory
                memscpy(((cross_fade_lib_mem_t*)(cross_fade_lib_ptr->cross_fade_lib_mem_ptr))->cross_fade_mode_ptr, mem_size, mem_ptr, mem_size);

            }
            else //
            {

                return CROSS_FADE_MEMERROR;
            }

            break;
        }
        case CROSS_FADE_PARAM_CONFIG:
        {
            // check if the memory buffer has enough space to write the parameter data
            if(mem_size == sizeof(cross_fade_config_t))
            {


                // set the calibration params in the lib memory
                memscpy(((cross_fade_lib_mem_t*)(cross_fade_lib_ptr->cross_fade_lib_mem_ptr))->cross_fade_config_ptr, mem_size, mem_ptr, mem_size);

                // configures some state variables
                audio_cross_fade_cfg(((cross_fade_lib_mem_t*)(cross_fade_lib_ptr->cross_fade_lib_mem_ptr))->cross_fade_config_ptr,
                                 ((cross_fade_lib_mem_t*)(cross_fade_lib_ptr->cross_fade_lib_mem_ptr))->cross_fade_data_ptr,
                                 (int32_t)((cross_fade_lib_mem_t*)(cross_fade_lib_ptr->cross_fade_lib_mem_ptr))->cross_fade_static_ptr->sample_rate);


            }
            else //
            {

                return CROSS_FADE_MEMERROR;
            }

            break;
        }
        default:
        {

            return CROSS_FADE_FAILURE;
        }
    }



    return CROSS_FADE_SUCCESS;
}


/*======================================================================

  FUNCTION      audio_cross_fade_process

  DESCRIPTION   cross fade the two inputs and outputs a single output.
                rampdown of old filter output and rampup of new filter output.

  DEPENDENCIES  Input pointers must not be NULL.
                audio_cross_fade_init_memory must be called prior to any call to audio_cross_fade_process.
                Length of output buffer must be at least as large as the
                length of the input buffer.

  PARAMETERS    cross_fade_lib_ptr: [in] Pointer to lib structure
                out_ptr: [out] Pointer to single-channel output PCM samples
                in_ptr[TOTAL_INPUT]: [in] Pointer to two single-channel input PCM samples
                block_size: [in] Number of samples to be processed

  SIDE EFFECTS  None.

======================================================================*/
CROSS_FADE_RESULT audio_cross_fade_process(cross_fade_lib_t *cross_fade_lib_ptr, int8_t *out_ptr, int8_t *in_ptr[TOTAL_INPUT], uint32_t block_size)
{


    int16_t *pIn1L16 = 0, *pIn2L16 = 0, *pOutL16 = 0;
    int32_t *pIn1L32 = 0, *pIn2L32 = 0, *pOutL32 = 0;

    cross_fade_lib_mem_t* cross_fade_lib_mem_ptr = (cross_fade_lib_mem_t*)(cross_fade_lib_ptr->cross_fade_lib_mem_ptr);
    cross_fade_static_t* pStatic = cross_fade_lib_mem_ptr->cross_fade_static_ptr;
    cross_fade_data_t* pData = cross_fade_lib_mem_ptr->cross_fade_data_ptr;
    cross_fade_config_t* pCfg = cross_fade_lib_mem_ptr->cross_fade_config_ptr;

    int32_t samples_left_in_crossfade_period;


    // ensure correctness of cross fade processing
    if(pData->total_period_samples < block_size)
    {
        return CROSS_FADE_FAILURE;
    }


    if(pStatic->data_width == CF_BITS32)
    {
        pIn1L32 = (int32_t *)in_ptr[CUR_INPUT];
        pIn2L32 = (int32_t *)in_ptr[NEW_INPUT];
        pOutL32 = (int32_t *)out_ptr;

        pIn1L16 = NULL;
        pIn2L16 = NULL;
        pOutL16 = NULL;
    }
    else if(pStatic->data_width == CF_BITS16)
    {
        pIn1L16 = (int16_t *)in_ptr[CUR_INPUT];
        pIn2L16 = (int16_t *)in_ptr[NEW_INPUT];
        pOutL16 = (int16_t *)out_ptr;

        pIn1L32 = NULL;
        pIn2L32 = NULL;
        pOutL32 = NULL;
    }
    else
    {
        return CROSS_FADE_FAILURE;
    }

    // If still in convergence mode, no need to do cross-fading, but simply output the old outputs
    if(pData->sample_count < pCfg->converge_num_samples)
    {
        /* convergence period*/
        pData->state = CONVERGENCE_STATE;

        // if convergence period is greater than frame size, copy the whole old output frame into current output frame
        if(pData->converge_num_of_samples >= block_size)
        {
            memscpy(out_ptr, (pStatic->data_width<<1)*block_size, in_ptr[CUR_INPUT], (pStatic->data_width<<1)*block_size);

            pData->converge_num_of_samples -= block_size;
        }
        else  // if convergence period is smaller than frame size 1) copy only converge_num_of_samples samples from old output.  2) For the remaining samples, do cross-fading
        {
            // copy only converge_num_of_samples samples from old output.
            memscpy(out_ptr, (pStatic->data_width<<1)*pData->converge_num_of_samples, in_ptr[CUR_INPUT], (pStatic->data_width<<1)*pData->converge_num_of_samples);

            // For the remaining samples, do cross-fading
            if(pStatic->data_width == CF_BITS32)
            {
                audio_cross_fade_32(pData,&pIn1L32[pData->converge_num_of_samples],&pIn2L32[pData->converge_num_of_samples],
                    &pOutL32[pData->converge_num_of_samples],(block_size-pData->converge_num_of_samples));
            }
            else
            {
                audio_cross_fade_16(pData,&pIn1L16[pData->converge_num_of_samples],&pIn2L16[pData->converge_num_of_samples],
                    &pOutL16[pData->converge_num_of_samples],(block_size-pData->converge_num_of_samples));
            }

            pData->converge_num_of_samples = 0;
        }
        pData->sample_count += block_size;
    }
    else  // If already in cross-fading mode, continue doing cross-fading for the number of samples needed
    {
        // if num of samples processed is smaller than total period samples and one more frame of processing will be bigger than total period samples
        if(((uint32_t)s32_add_s32_s32_sat(pData->sample_count,block_size) > pData->total_period_samples)&&(pData->sample_count < pData->total_period_samples))
        {
            // find out how many more samples need to be processed to reach total period samples
            samples_left_in_crossfade_period = s32_sub_s32_s32_sat(pData->total_period_samples, pData->sample_count);

            // do cross fading for the needed num of samples to complete the cross fade processing
            if(pStatic->data_width == CF_BITS32)
            {
                audio_cross_fade_32(pData, pIn1L32, pIn2L32, pOutL32, samples_left_in_crossfade_period);
                memscpy(pOutL32+samples_left_in_crossfade_period, (block_size-samples_left_in_crossfade_period)<<2, pIn2L32+samples_left_in_crossfade_period, (block_size-samples_left_in_crossfade_period)<<2);
            }
            else
            {
                audio_cross_fade_16(pData, pIn1L16, pIn2L16, pOutL16, samples_left_in_crossfade_period);
                memscpy(pOutL16+samples_left_in_crossfade_period, (block_size-samples_left_in_crossfade_period)<<1, pIn2L16+samples_left_in_crossfade_period, (block_size-samples_left_in_crossfade_period)<<1);
            }
            pData->sample_count += samples_left_in_crossfade_period;

        }
        else if(pData->sample_count < pData->total_period_samples) // if num of samples processed is smaller than total period samples
        {
            // do processing for one frame worth of samples; after this processing the number of totally processed samples should be still smaller than total period samples
            if(pStatic->data_width == CF_BITS32)
            {
                audio_cross_fade_32(pData, pIn1L32, pIn2L32, pOutL32, block_size);
            }
            else
            {
                audio_cross_fade_16(pData, pIn1L16, pIn2L16, pOutL16, block_size);
            }
            pData->sample_count += block_size;
        }

    }

    //cross fading finished and move the state to STEADY_STATE
    if(pData->sample_count == pData->total_period_samples)
    {

        pData->state = STEADY_STATE;
        pData->sample_count = 0;
        pData->converge_num_of_samples = pCfg->converge_num_samples;
        *cross_fade_lib_mem_ptr->cross_fade_mode_ptr = 0;
    }



    return CROSS_FADE_SUCCESS;
}



/*======================================================================

  FUNCTION      audio_cross_fade_init

  DESCRIPTION   init for the cross fading algorithm.


  PARAMETERS    pCfg:  [in] Pointer to configuration structure
                pData: [out] Pointer to data structure
                samplingRate: [in] sampling rate of the input signal

  SIDE EFFECTS  None

======================================================================*/
void audio_cross_fade_init ( cross_fade_config_t *pCfg,
                        cross_fade_data_t *pData,
                        int32_t samplingRate )
{
    // config
    audio_cross_fade_cfg(pCfg,pData,samplingRate);

    // reset
    audio_cross_fade_reset(pData);

}


void audio_cross_fade_reset (cross_fade_data_t *pData)
{

    // Initialize the members of the data structure
    pData->sample_count = 0;

    //rampup and rampdown scale factors
    pData->ramp_up_scale_factor = 0;
    pData->ramp_down_scale_factor = CF_ONE_Q31;

    //state of cross fade
    pData->state = STEADY_STATE;

}


void audio_cross_fade_cfg ( cross_fade_config_t *pCfg,
                        cross_fade_data_t *pData,
                        int32_t samplingRate )
{
    int32_t tempL32;

    // convert the total period in to number of samples
    /* (periodin_msec*fs/1000)*/
    tempL32 = s32_saturate_s64(s64_mult_s32_s16(samplingRate, (int16_t)pCfg->total_period_msec));
    pData->total_period_samples = s32_mult_s32_s32_rnd_sat(tempL32, CF_ONE_BY_1000_IN_0DOT32);

    //cross fade step size 1/(totaltime-convergeperiod) in Q31 format
    pData->cross_fade_step_size = s32_div_s32_s32_sat(CF_ONE_Q31,
                            s32_sub_s32_s32_sat(pData->total_period_samples, (int32_t)pCfg->converge_num_samples),31);  // Q31 format

    //converge number of samples
    pData->converge_num_of_samples = pCfg->converge_num_samples;
}




void audio_cross_fade_16(cross_fade_data_t *pData,
                       int16_t *pInPtr1L16,
                       int16_t *pInPtr2L16,
                       int16_t *pOutPtrL16,
                       int32_t block_size)
{
    int32_t   tempOutL32 = 0, temp1L32, temp2L32, i=0;
    int16_t   tempL16;

    if(pData->state == CONVERGENCE_STATE)
    {
        pData->ramp_down_scale_factor = CF_ONE_Q31;
        pData->ramp_up_scale_factor = 0;
        pData->state = CROSSFADE_STATE;
    }

    /* Do cross-fading by multiplying two lines with slope 1 and -1
    * respectively, and then add together*/
    for (i = 0; i < block_size; i++)
    {
        temp1L32 = (int32_t)s64_mult_s32_s16_shift(pData->ramp_down_scale_factor, pInPtr1L16[i], 0);
        temp2L32 = (int32_t)s64_mult_s32_s16_shift(pData->ramp_up_scale_factor, pInPtr2L16[i], 0);
        tempOutL32 = s32_add_s32_s32_sat(temp1L32,temp2L32);

        //Extract the first 16-bit out and saved as int16_t
        pOutPtrL16[i] = s16_saturate_s32(s32_shr_s32_sat(tempOutL32,15));

        tempL16 = s16_min_s16_s16(pInPtr1L16[i], pInPtr2L16[i]);
        if(pOutPtrL16[i] < tempL16)
        {
            pOutPtrL16[i] = tempL16;
        }

        /* Compute the coefficients each buffer element nees to be multiplied*/
        pData->ramp_up_scale_factor = s32_add_s32_s32_sat(pData->ramp_up_scale_factor, pData->cross_fade_step_size);
        pData->ramp_down_scale_factor = s32_sub_s32_s32_sat(CF_ONE_Q31, pData->ramp_up_scale_factor);
        if(pData->ramp_down_scale_factor < 0 )
            pData->ramp_down_scale_factor = 0;

    }
}



void audio_cross_fade_32(cross_fade_data_t *pData,
                       int32_t *pInPtr1L32,
                       int32_t *pInPtr2L32,
                       int32_t *pOutPtrL32,
                       int32_t block_size)
{
    int64_t   tempOutL64 = 0, temp1L64, temp2L64;
    int32_t   tempL32, i=0;

    if(pData->state == CONVERGENCE_STATE)
    {
        pData->ramp_down_scale_factor = CF_ONE_Q31;
        pData->ramp_up_scale_factor = 0;
        pData->state = CROSSFADE_STATE;
    }

    /* Do cross-fading by multiplying two lines with slope 1 and -1
    * respectively, and then add together*/
    for (i = 0; i < block_size; i++)
    {
        temp1L64 = s64_mult_s32_s32(pInPtr1L32[i], pData->ramp_down_scale_factor);
        temp2L64 = s64_mult_s32_s32(pInPtr2L32[i], pData->ramp_up_scale_factor);
        tempOutL64 = s64_add_s64_s64(temp1L64,temp2L64);

        //Extract the first 32-bit out and saved as int32_t
        pOutPtrL32[i] = (int32_t)(s64_shl_s64(tempOutL64,-31));

        tempL32 = s32_min_s32_s32(pInPtr1L32[i], pInPtr2L32[i]);
        if(pOutPtrL32[i] < tempL32)
        {
            pOutPtrL32[i] = tempL32;
        }

        /* Compute the coefficients each buffer element nees to be multiplied*/
        pData->ramp_up_scale_factor = s32_add_s32_s32_sat(pData->ramp_up_scale_factor, pData->cross_fade_step_size);
        pData->ramp_down_scale_factor = s32_sub_s32_s32_sat(MAX_32, pData->ramp_up_scale_factor);
        if(pData->ramp_down_scale_factor < 0 )
            pData->ramp_down_scale_factor = 0;

    }
}

