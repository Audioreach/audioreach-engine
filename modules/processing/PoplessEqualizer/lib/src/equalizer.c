/* Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
 */
/*===========================================================================
 * FILE NAME: equalizer.c
 * DESCRIPTION:
 *    Contains setup and process functions for equalizer.
 *===========================================================================*/
/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/

#include "audio_basic_op.h"
#include "audio_iir_tdf2.h"
#include "audio_divide_qx.h"
#include "audio_log10.h"
#include "audio_exp10.h"
#include "equalizer_preset.h"
#include "equalizer.h"
#include "equalizer_api.h"
#include "stringl.h"




#define EQ_CF_FOUR          4
#define EQ_GAIN_LEVELS      31
#define ONE_OVER_FIFTEEN    4369 // 1/15 in Q16

// audio fx EQ uses fixed band freqs
const uint32 eq_audio_fx_freqs[EQ_AUDIO_FX_BAND]={60000, 230000, 910000, 3600000, 14000000};

// to better match audio fx EQ performance using our parametric EQ model, use different quality factor at 14KHz based on gain set at 14KHz
const int32 eq_quality_factor_table[][EQ_GAIN_LEVELS]=
{
    {-1500,-1400,-1300,-1200,-1100,-1000,-900,-800,-700,-600,-500,-400,-300,-200,-100,   0, 100,200,300,400,500,600,700,800,900,1000,1100,1200,1300,1400,1500}, // gain in millibels
    { 100,   100,  100,  100,  105,  105, 110, 115, 120, 135, 155, 210, 256, 256, 256, 256, 256,256,256,210,155,135,120,115,110, 105, 105, 100, 100, 100, 100}  // Q8 quality factor
};



extern char eq_settings_names[][EQ_MAX_PRESET_NAME_LEN];
extern const uint32 eq_audio_fx_band_freq_range[EQ_AUDIO_FX_BAND][2];


EQ_RESULT eq_msiir_design(eq_lib_mem_t*  eq_lib_mem_ptr, eq_msiir_settings_t   *eq_msiir_settings_ptr);
void eq_headroom_req_compute(uint32 *headroom_needed_db, eq_band_internal_specs_t   *eq_band_internal_specs_ptr);
/*----------------------------------------------------------------------------
 * Function Definitions
 * -------------------------------------------------------------------------*/
/*======================================================================

  FUNCTION      eq_get_mem_req

  DESCRIPTION   Determine lib mem size

  DEPENDENCIES  Input pointers must not be NULL.

  PARAMETERS    eq_lib_mem_requirements_ptr: [out] Pointer to lib mem requirement structure
eq_static_struct_ptr: [in] Pointer to static structure.

SIDE EFFECTS  None

======================================================================*/
EQ_RESULT eq_get_mem_req(eq_lib_mem_requirements_t *eq_lib_mem_requirements_ptr, eq_static_struct_t* eq_static_struct_ptr)

{

    msiir_mem_req_t  msiir_mem_req;
    msiir_static_vars_t  msiir_static_vars;
    cross_fade_lib_mem_req_t  cross_fade_lib_mem_req;
    cross_fade_static_t  cross_fade_static;
    uint32 mem_req=0;


    // clear memory
    memset(eq_lib_mem_requirements_ptr,0,sizeof(eq_lib_mem_requirements_t));

    // eq_lib_mem_t
    // static struct
    // internal band specs(compatible to internal library)
    // eq_config_t and external bands specs(compatible to upper layer)
    // iir
    // cross fade
    // audio fx EQ freq ranges table


    // determine mem size
    // eq_lib_mem_t
    mem_req += ALIGN8(sizeof(eq_lib_mem_t));

    // static struct size
    mem_req += ALIGN8(sizeof(eq_static_struct_t));

    // internal band specs(compatible to internal library)
    mem_req += ALIGN8(EQ_MAX_BANDS*sizeof(eq_band_internal_specs_t));

    // eq_config_t and external bands specs
    mem_req += ALIGN8(sizeof(eq_config_t) + EQ_MAX_BANDS*sizeof(eq_band_specs_t));


    // eq states
    mem_req += ALIGN8(sizeof(eq_states_t));


    // msiir
    msiir_static_vars.data_width = (int32)eq_static_struct_ptr->data_width;
    msiir_static_vars.max_stages = EQ_MAX_BANDS;
    if (MSIIR_SUCCESS != msiir_get_mem_req(&msiir_mem_req, &msiir_static_vars))
    {
        return EQ_FAILURE;
    }
    mem_req += msiir_mem_req.mem_size;

    // cross fade
    cross_fade_static.data_width = eq_static_struct_ptr->data_width >> 4; // cross fade use 1(16bits) or 2(32bits)
    cross_fade_static.sample_rate = eq_static_struct_ptr->sample_rate;
    if(audio_cross_fade_get_mem_req(&cross_fade_lib_mem_req, &cross_fade_static) != CROSS_FADE_SUCCESS)
    {

        return EQ_FAILURE;
    }
    mem_req += cross_fade_lib_mem_req.cross_fade_lib_mem_size;

    // audio fx EQ freq ranges table
    mem_req += ALIGN8(sizeof(eq_audio_fx_band_freq_range));


    // total lib mem needed
    eq_lib_mem_requirements_ptr->lib_mem_size = mem_req;


    // maximal lib stack mem consumption
    eq_lib_mem_requirements_ptr->lib_stack_size = eq_max_stack_size;

    return EQ_SUCCESS;
}


/*======================================================================

  FUNCTION      eq_init_memory

  DESCRIPTION   Performs partition and initialization of lib memory.


  DEPENDENCIES  Input pointers must not be NULL.

  PARAMETERS    eq_lib_ptr: [in, out] Pointer to lib structure
eq_static_struct_ptr: [in] Pointer to static structure
mem_ptr: [in] Pointer to the lib memory
mem_size: [in] Size of the memory pointed by mem_ptr

SIDE EFFECTS  None

======================================================================*/
EQ_RESULT eq_init_memory(eq_lib_t *eq_lib_ptr, eq_static_struct_t* eq_static_struct_ptr, int8* mem_ptr, uint32 mem_size)
{

    msiir_mem_req_t  msiir_mem_req;
    msiir_static_vars_t  msiir_static_vars;
    cross_fade_lib_mem_req_t  cross_fade_lib_mem_req;
    cross_fade_static_t  cross_fade_static;
    int8 *temp_ptr = mem_ptr;
    eq_lib_mem_t*  eq_lib_mem_ptr;
    uint32 i, coeff_size;
    eq_msiir_settings_t   eq_msiir_settings;
    eq_band_specs_t     *eq_band_specs_ptr;
    eq_lib_mem_requirements_t   eq_lib_mem_requirements;


    // check if mem ptr is valid
    if (mem_ptr == NULL)
    {
        return EQ_MEMERROR;
    }


    // re-calculate mem size. Use get_mem_req(.) to comupte total size
    // Individual size info for msiir and crossfade cannot be retrieved from get_mem_req, re-compute them here again

    // iir
    msiir_static_vars.data_width = (int32)eq_static_struct_ptr->data_width;
    msiir_static_vars.max_stages = EQ_MAX_BANDS;
    if (MSIIR_SUCCESS != msiir_get_mem_req(&msiir_mem_req, &msiir_static_vars))
    {
        return EQ_FAILURE;
    }

    // cross fade
    cross_fade_static.data_width = eq_static_struct_ptr->data_width >> EQ_CF_FOUR; // cross fade use 1(16bits) or 2(32bits)
    cross_fade_static.sample_rate = eq_static_struct_ptr->sample_rate;
    if(audio_cross_fade_get_mem_req(&cross_fade_lib_mem_req, &cross_fade_static) != CROSS_FADE_SUCCESS)
    {

        return EQ_FAILURE;
    }

    // get total mem size
    if(eq_get_mem_req(&eq_lib_mem_requirements, eq_static_struct_ptr) != EQ_SUCCESS)
    {

        return EQ_FAILURE;
    }



    // error out if the mem space given is not enough
    if (mem_size < eq_lib_mem_requirements.lib_mem_size)
    {

        return EQ_MEMERROR;
    }




    // before initializing, it is FW job to make sure that pMem is 8 bytes aligned(with enough space)
    memset(mem_ptr,0,mem_size);                                   // clear the mem
    eq_lib_ptr->lib_mem_ptr=(void*)mem_ptr;


    // eq_lib_mem_t
    // static struct
    // internal band specs(compatible to internal library)
    // eq_config_t and bands specs(API-interfacing)
    // eq states
    // msiir
    // cross fade
    // audio fx EQ freq ranges table



    // init eq_lib_mem_t
    eq_lib_mem_ptr = (eq_lib_mem_t*)eq_lib_ptr->lib_mem_ptr;
    temp_ptr += ALIGN8(sizeof(eq_lib_mem_t));


    // init static struct
    eq_lib_mem_ptr->eq_static_struct_ptr = (eq_static_struct_t*)temp_ptr;
    eq_lib_mem_ptr->eq_static_struct_ptr->data_width = eq_static_struct_ptr->data_width;
    eq_lib_mem_ptr->eq_static_struct_ptr->sample_rate = eq_static_struct_ptr->sample_rate;
    temp_ptr += ALIGN8(sizeof(eq_static_struct_t));



    // init converted band specs(compatible to internal library)
    eq_lib_mem_ptr->eq_band_internal_specs_ptr = (eq_band_internal_specs_t*)temp_ptr;
    eq_lib_mem_ptr->eq_band_internal_specs_size = ALIGN8(EQ_MAX_BANDS*sizeof(eq_band_internal_specs_t));
    // fill in internal specs next step
    temp_ptr += eq_lib_mem_ptr->eq_band_internal_specs_size;



    // init eq_config_t and bands specs(API-interfacing)
    eq_lib_mem_ptr->eq_config_ptr = (eq_config_t*)temp_ptr;
    eq_lib_mem_ptr->eq_config_ptr->gain_adjust_flag = 0; // default gain adjuts flag is 0
    eq_lib_mem_ptr->eq_config_ptr->eq_pregain = 134217728; // default preGain is 0dB
    eq_lib_mem_ptr->eq_config_ptr->eq_preset_id = EQ_PRESET_BLANK; // default is blank effect
    // fill in internal specs
    equalizer_load_preset(eq_lib_mem_ptr->eq_band_internal_specs_ptr, &eq_lib_mem_ptr->eq_config_ptr->num_bands, eq_lib_mem_ptr->eq_config_ptr->eq_preset_id);
    eq_band_specs_ptr = (eq_band_specs_t*)(eq_lib_mem_ptr->eq_config_ptr+1);
    // fill in API-interfacing specs from internal specs and do conversion for dB -> dB*100 and freq -> freq*1000
    memscpy((void*)eq_band_specs_ptr, (size_t)eq_lib_mem_ptr->eq_band_internal_specs_size, (const void*)eq_lib_mem_ptr->eq_band_internal_specs_ptr, (size_t)eq_lib_mem_ptr->eq_band_internal_specs_size);
    for (i=0; i < eq_lib_mem_ptr->eq_config_ptr->num_bands; i++)
    {
        eq_band_specs_ptr[i].freq_millihertz = eq_band_specs_ptr[i].freq_millihertz*1000;
        eq_band_specs_ptr[i].gain_millibels = eq_band_specs_ptr[i].gain_millibels*100;
    }
    temp_ptr += ALIGN8(sizeof(eq_config_t) + EQ_MAX_BANDS*sizeof(eq_band_specs_t));


    // eq states
    eq_lib_mem_ptr->eq_states_ptr = (eq_states_t*)temp_ptr;
    eq_lib_mem_ptr->eq_states_ptr->headroom_req_db = 0; // default is 0dB headroom needed
    temp_ptr += ALIGN8(sizeof(eq_states_t));


    // init iir
    if (MSIIR_SUCCESS != msiir_init_mem(&eq_lib_mem_ptr->msiir_lib_mem, &msiir_static_vars, (void*)temp_ptr, msiir_mem_req.mem_size))
    {
        return EQ_FAILURE;
    }

    // design blank effect(12 stages all pass msiir coeffs)
    if(eq_msiir_design(eq_lib_mem_ptr, &eq_msiir_settings) != EQ_SUCCESS)
    {
        return EQ_FAILURE;
    }

    // set up msiir using 12 stages all pass coeffs as EQ default
    coeff_size = sizeof(msiir_config_t) + eq_msiir_settings.msiir_config.num_stages * sizeof(msiir_coeffs_t);
    if(msiir_set_param(&eq_lib_mem_ptr->msiir_lib_mem, MSIIR_PARAM_CONFIG, (void*)&eq_msiir_settings, coeff_size) != MSIIR_SUCCESS)
    {

        return EQ_FAILURE;
    }
    temp_ptr += msiir_mem_req.mem_size;



    // init cross fade
    if(audio_cross_fade_init_memory(&eq_lib_mem_ptr->cross_fade_lib_mem, &cross_fade_static, temp_ptr, cross_fade_lib_mem_req.cross_fade_lib_mem_size) != CROSS_FADE_SUCCESS)
    {

        return EQ_FAILURE;
    }
    temp_ptr += cross_fade_lib_mem_req.cross_fade_lib_mem_size;



    // init audio fx EQ freq ranges table
    eq_lib_mem_ptr->eq_audio_fx_freq_ranges_table_ptr = (uint32*)temp_ptr;
    memscpy((void*)eq_lib_mem_ptr->eq_audio_fx_freq_ranges_table_ptr, (size_t)sizeof(eq_audio_fx_band_freq_range), (const void*)eq_audio_fx_band_freq_range, (size_t)sizeof(eq_audio_fx_band_freq_range));
    temp_ptr += ALIGN8(sizeof(eq_audio_fx_band_freq_range));

    // init EQ feature set : this memory already resides in eq_lib_mem_t
    eq_lib_mem_ptr->eq_mode = EQ_DISABLE;
    // init EQ cross-fading interal states
    eq_lib_mem_ptr->eq_on_off_crossfade_mode = EQ_FALSE;

    // check to see if memory partition is correct
    if (mem_ptr + eq_lib_mem_requirements.lib_mem_size != temp_ptr)
    {

        return EQ_MEMERROR;
    }



    return EQ_SUCCESS;



}


/*======================================================================

  FUNCTION      eq_get_param

  DESCRIPTION   Get the params from lib mem

  DEPENDENCIES  Input pointers must not be NULL.

  PARAMETERS    eq_lib_ptr: [in] Pointer to lib structure
param_id: [in] ID of the param
mem_ptr: [out] Pointer to the memory where params are to be stored
mem_size:[in] Size of the memory pointed by mem_ptr
param_size_ptr: [out] Pointer to param size which indicates the size of the retrieved param(s)

SIDE EFFECTS  None

======================================================================*/
EQ_RESULT eq_get_param(eq_lib_t* eq_lib_ptr, uint32 param_id, int8* mem_ptr, uint32 mem_size, uint32 *param_size_ptr)
{
    eq_lib_mem_t* eq_lib_mem_ptr = (eq_lib_mem_t*)eq_lib_ptr->lib_mem_ptr;
    uint32 size;

    // check if mem ptr is valid
    if (mem_ptr == NULL)
    {
        return EQ_MEMERROR;
    }


    switch (param_id)
    {
        case EQ_PARAM_CHECK_STATE:
            if(mem_size >= sizeof(eq_mode_t))
            {
                *param_size_ptr = sizeof(uint32);
                *((uint32*)mem_ptr) = (uint32)EQ_ENABLE;
                if ((eq_lib_mem_ptr->eq_mode == EQ_DISABLE) &&
                    (eq_lib_mem_ptr->eq_on_off_crossfade_mode == EQ_FALSE))
                {
                    *((uint32*)mem_ptr) = (uint32)EQ_DISABLE;
                }
            }
            else
            {
                return EQ_MEMERROR;
            }
            break;

        case EQ_PARAM_GET_LIB_VER:
            if(mem_size >= sizeof(eq_lib_ver_t))
            {
                *((eq_lib_ver_t *)mem_ptr) = EQ_LIB_VER;
                *param_size_ptr = sizeof(eq_lib_ver_t);
            }
            else
            {
                return EQ_MEMERROR;
            }
            break;

        case EQ_PARAM_CROSSFADE_FLAG:

            if(audio_cross_fade_get_param(&eq_lib_mem_ptr->cross_fade_lib_mem, CROSS_FADE_PARAM_MODE, mem_ptr, mem_size, param_size_ptr) != CROSS_FADE_SUCCESS)
            {
                return EQ_FAILURE;
            }

            break;

        case EQ_PARAM_CROSSFADE_CONFIG:

            if(audio_cross_fade_get_param(&eq_lib_mem_ptr->cross_fade_lib_mem, CROSS_FADE_PARAM_CONFIG, mem_ptr, mem_size, param_size_ptr) != CROSS_FADE_SUCCESS)
            {
                return EQ_FAILURE;
            }
            break;

        case EQ_PARAM_GET_HEADROOM_REQ:

            if(mem_size >= sizeof(eq_headroom_req_t))
            {
                //*(eq_headroom_req_t*)mem_ptr = (eq_headroom_req_t)s64_mult_fp_u32_s16_shift(eq_lib_mem_ptr->eq_states_ptr->headroom_req_db, 100, 16);
                *(eq_headroom_req_t*)mem_ptr = eq_lib_mem_ptr->eq_states_ptr->headroom_req_db;
                *param_size_ptr = sizeof(eq_headroom_req_t);
            }
            else
            {
                return EQ_MEMERROR;
            }

            break;

        case EQ_PARAM_GET_NUM_BAND:

            if(mem_size >= sizeof(eq_num_band_t))
            {
                memscpy((void*)mem_ptr, (size_t)sizeof(eq_num_band_t), (const void*)&eq_lib_mem_ptr->eq_config_ptr->num_bands, (size_t)sizeof(eq_num_band_t));
                *param_size_ptr = sizeof(eq_num_band_t);
            }
            else
            {
                return EQ_MEMERROR;
            }
            break;

        case EQ_PARAM_GET_MAX_NUM_BAND:

            if(mem_size >= sizeof(eq_max_num_band_t))
            {

                if (eq_lib_mem_ptr->eq_config_ptr->eq_preset_id >= EQ_USER_CUSTOM && eq_lib_mem_ptr->eq_config_ptr->eq_preset_id < EQ_MAX_SETTINGS_QCOM)
                {
                    *((eq_max_num_band_t*)mem_ptr) = EQ_MAX_BANDS;
                }
                else if (eq_lib_mem_ptr->eq_config_ptr->eq_preset_id >= EQ_USER_CUSTOM_AUDIO_FX && eq_lib_mem_ptr->eq_config_ptr->eq_preset_id < EQ_MAX_SETTINGS_AUDIO_FX)
                {
                    *((eq_max_num_band_t*)mem_ptr) = EQ_AUDIO_FX_BAND;
                }
                else
                {
                    return EQ_FAILURE;
                }

                *param_size_ptr = sizeof(eq_max_num_band_t);

            }
            else
            {
                return EQ_MEMERROR;
            }
            break;

        case EQ_PARAM_GET_BAND_LEVELS:

            size = sizeof(eq_band_levels_t)+eq_lib_mem_ptr->eq_config_ptr->num_bands*sizeof(eq_level_t);

            if(mem_size >= size)
            {
                uint32 i;
                eq_band_levels_t        *eq_band_levels_ptr = (eq_band_levels_t*)mem_ptr;
                eq_level_t      *eq_level_ptr = (eq_level_t*)(eq_band_levels_ptr+1);
                eq_band_specs_t     *eq_band_specs_ptr = (eq_band_specs_t*)(eq_lib_mem_ptr->eq_config_ptr+1);

                eq_band_levels_ptr->num_bands = eq_lib_mem_ptr->eq_config_ptr->num_bands;
                for (i=0; i < eq_band_levels_ptr->num_bands; i++)
                {
                    eq_level_ptr[i] = eq_band_specs_ptr[i].gain_millibels;
                }
                *param_size_ptr = size;

            }
            else
            {
                return EQ_MEMERROR;
            }
            break;

        case EQ_PARAM_GET_BAND_LEVEL_RANGE:
            if(mem_size >= sizeof(eq_band_level_range_t))
            {
                ((eq_band_level_range_t*)mem_ptr)->max_level_millibels = EQ_MAX_GAIN;
                ((eq_band_level_range_t*)mem_ptr)->min_level_millibels = EQ_MIN_GAIN;
                *param_size_ptr = sizeof(eq_band_level_range_t);
            }
            else
            {
                return EQ_MEMERROR;
            }
            break;

        case EQ_PARAM_GET_BAND_FREQS:

            size = sizeof(eq_band_freqs_t)+eq_lib_mem_ptr->eq_config_ptr->num_bands*sizeof(eq_freq_t);

            if(mem_size >= size)
            {
                uint32 i;
                eq_band_freqs_t     *eq_band_freqs_ptr = (eq_band_freqs_t*)mem_ptr;
                eq_freq_t       *eq_freq_ptr = (eq_freq_t*)(eq_band_freqs_ptr+1);
                eq_band_specs_t     *eq_band_specs_ptr = (eq_band_specs_t*)(eq_lib_mem_ptr->eq_config_ptr+1);

                eq_band_freqs_ptr->num_bands = eq_lib_mem_ptr->eq_config_ptr->num_bands;
                for (i=0; i < eq_band_freqs_ptr->num_bands; i++)
                {
                    eq_freq_ptr[i] = eq_band_specs_ptr[i].freq_millihertz;
                }

                *param_size_ptr = size;

            }
            else
            {
                return EQ_MEMERROR;
            }
            break;

        case EQ_PARAM_GET_BAND_FREQ_RANGE:

            // check whether input band index is within range
            if (((eq_band_freq_range_t*)mem_ptr)->band_idx >= (uint32)eq_lib_mem_ptr->eq_config_ptr->num_bands)
            {
                return EQ_FAILURE;
            }

            if(mem_size >= sizeof(eq_band_freq_range_t))
            {

                // for qcom EQ, get band freq range from our own library
                if (eq_lib_mem_ptr->eq_config_ptr->eq_preset_id >= EQ_USER_CUSTOM && eq_lib_mem_ptr->eq_config_ptr->eq_preset_id < EQ_MAX_SETTINGS_QCOM)
                {

                    uint32  sample_rate = eq_lib_mem_ptr->eq_static_struct_ptr->sample_rate*1000;
                    eq_band_specs_t     *eq_band_specs_ptr = (eq_band_specs_t*)(eq_lib_mem_ptr->eq_config_ptr+1);

                    // get the min and max frequencies for a band given its band index
                    if(GetBandFreqRangeEQFD_v2 ( eq_band_specs_ptr,
                                                 sample_rate,
                                                 (uint16)eq_lib_mem_ptr->eq_config_ptr->num_bands,
                                                 (uint16)((eq_band_freq_range_t*)mem_ptr)->band_idx,
                                                 &((eq_band_freq_range_t*)mem_ptr)->eq_band_freq_range.min_freq_millihertz,
                                                 &((eq_band_freq_range_t*)mem_ptr)->eq_band_freq_range.max_freq_millihertz) != EQ_SUCCESS)

                    {

                        return EQ_FAILURE;
                    }

                } // for audio fx EQ, get the band freq range directly from the table
                else if (eq_lib_mem_ptr->eq_config_ptr->eq_preset_id >= EQ_USER_CUSTOM_AUDIO_FX && eq_lib_mem_ptr->eq_config_ptr->eq_preset_id < EQ_MAX_SETTINGS_AUDIO_FX)
                {
                    ((eq_band_freq_range_t*)mem_ptr)->eq_band_freq_range.min_freq_millihertz = eq_audio_fx_band_freq_range[((eq_band_freq_range_t*)mem_ptr)->band_idx][0];
                    ((eq_band_freq_range_t*)mem_ptr)->eq_band_freq_range.max_freq_millihertz = eq_audio_fx_band_freq_range[((eq_band_freq_range_t*)mem_ptr)->band_idx][1];
                }
                else
                {
                    return EQ_FAILURE;
                }

                *param_size_ptr = sizeof(eq_freq_range_t);

            }
            else
            {
                return EQ_MEMERROR;
            }

            break;

        case EQ_PARAM_GET_BAND:

            // check whether input freq is within range
            if (((eq_band_index_t*)mem_ptr)->freq_millihertz > (uint32)s64_mult_u32_s16_shift(eq_lib_mem_ptr->eq_static_struct_ptr->sample_rate>>1, 1000, 16))
            {
                return EQ_FAILURE;
            }

            if(mem_size >= sizeof(eq_band_index_t))
            {

                uint32  band;

                // for qcom EQ, follow existing EQ implementation to support backward compatibility
                if (eq_lib_mem_ptr->eq_config_ptr->eq_preset_id >= EQ_USER_CUSTOM && eq_lib_mem_ptr->eq_config_ptr->eq_preset_id < EQ_MAX_SETTINGS_QCOM)
                {

                    uint32  min_freq_millihertz = 0, max_freq_millihertz = 0, sample_rate = eq_lib_mem_ptr->eq_static_struct_ptr->sample_rate*1000;
                    eq_band_specs_t     *eq_band_specs_ptr = (eq_band_specs_t*)(eq_lib_mem_ptr->eq_config_ptr+1);

                    for ( band = 0; band < eq_lib_mem_ptr->eq_config_ptr->num_bands; band++ )
                    {

                        // get the min and max frequencies for a band given its band index
                        if(GetBandFreqRangeEQFD_v2 ( eq_band_specs_ptr,
                                                     sample_rate,
                                                     (uint16)eq_lib_mem_ptr->eq_config_ptr->num_bands,
                                                     (uint16)band,
                                                     &min_freq_millihertz,
                                                     &max_freq_millihertz) != EQ_SUCCESS)

                        {
                            return EQ_FAILURE;
                        }

                        // return success if the given frequency is within the frequency range for the current band
                        if ( ((eq_band_index_t*)mem_ptr)->freq_millihertz <= max_freq_millihertz )
                        {
                            if ( ((eq_band_index_t*)mem_ptr)->freq_millihertz >= min_freq_millihertz )
                            {
                                ((eq_band_index_t*)mem_ptr)->band_idx = band;

                            }
                            else
                            {
                                return EQ_FAILURE;
                            }
                            break; // break for loop
                        }

                    }
                } // for audio fx EQ, below follows audio fx implementation to match their results
                else if (eq_lib_mem_ptr->eq_config_ptr->eq_preset_id >= EQ_USER_CUSTOM_AUDIO_FX && eq_lib_mem_ptr->eq_config_ptr->eq_preset_id < EQ_MAX_SETTINGS_AUDIO_FX)
                {
                    ((eq_band_index_t*)mem_ptr)->band_idx = 0;

                    if(((eq_band_index_t*)mem_ptr)->freq_millihertz < eq_audio_fx_band_freq_range[0][0])
                    {
                        return EQ_FAILURE;
                    }

                    for(band=0; band < EQ_AUDIO_FX_BAND; band++)
                    {
                        if((((eq_band_index_t*)mem_ptr)->freq_millihertz >= eq_audio_fx_band_freq_range[band][0])&&(((eq_band_index_t*)mem_ptr)->freq_millihertz <= eq_audio_fx_band_freq_range[band][1]))
                        {
                            ((eq_band_index_t*)mem_ptr)->band_idx = band;
                            break; // break for loop after finding the band index
                        }
                    }
                }
                else
                {
                    return EQ_FAILURE;
                }

                *param_size_ptr = sizeof(uint32); // return size of band_idx

            }
            else
            {
                return EQ_MEMERROR;
            }
            break;

        case EQ_PARAM_GET_PRESET_ID:

            if(mem_size >= sizeof(eq_preset_id_t))
            {
                memscpy((void*)mem_ptr, (size_t)sizeof(eq_preset_id_t), (const void*)&eq_lib_mem_ptr->eq_config_ptr->eq_preset_id, (size_t)sizeof(eq_preset_id_t));
                *param_size_ptr = sizeof(eq_preset_id_t);
            }
            else
            {
                return EQ_MEMERROR;
            }
            break;

        case EQ_PARAM_GET_NUM_PRESETS:

            if(mem_size >= sizeof(eq_num_preset_t))
            {
                // if qcom EQ is used, return the num of presets supported
                if (eq_lib_mem_ptr->eq_config_ptr->eq_preset_id >= EQ_USER_CUSTOM && eq_lib_mem_ptr->eq_config_ptr->eq_preset_id < EQ_MAX_SETTINGS_QCOM)
                {
                    *((eq_num_preset_t*)mem_ptr) = EQ_MAX_SETTINGS_QCOM;

                }
                // if audio fx EQ is used, return the num of presets supported
                else if (eq_lib_mem_ptr->eq_config_ptr->eq_preset_id >= EQ_USER_CUSTOM_AUDIO_FX && eq_lib_mem_ptr->eq_config_ptr->eq_preset_id < EQ_MAX_SETTINGS_AUDIO_FX)
                {
                    *((eq_num_preset_t*)mem_ptr) = EQ_MAX_SETTINGS_AUDIO_FX - EQ_PRESET_NORMAL_AUDIO_FX;
                }
                else
                {
                    return EQ_FAILURE;
                }

                *param_size_ptr = sizeof(eq_num_preset_t);
            }
            else
            {
                return EQ_MEMERROR;
            }
            break;

        case EQ_PARAM_GET_PRESET_NAME:

            if(mem_size >= sizeof(eq_preset_name_t))
            {
                if ((eq_lib_mem_ptr->eq_config_ptr->eq_preset_id >= EQ_USER_CUSTOM && eq_lib_mem_ptr->eq_config_ptr->eq_preset_id < EQ_MAX_SETTINGS_QCOM) || (eq_lib_mem_ptr->eq_config_ptr->eq_preset_id >= EQ_USER_CUSTOM_AUDIO_FX && eq_lib_mem_ptr->eq_config_ptr->eq_preset_id < EQ_MAX_SETTINGS_AUDIO_FX))
                {
                    memscpy((void*)((eq_preset_name_t*)mem_ptr)->preset_name, (size_t)sizeof(eq_preset_name_t), (const void*)eq_settings_names[eq_lib_mem_ptr->eq_config_ptr->eq_preset_id + 1], (size_t)sizeof(eq_preset_name_t));
                }
                else
                {
                    return EQ_FAILURE;
                }

                *param_size_ptr = sizeof(eq_preset_name_t);
            }
            else
            {
                return EQ_MEMERROR;
            }
            break;


        case EQ_PARAM_COMPUTE_HEADROOM_REQ:

            if(mem_size >= (sizeof(eq_config_t)+((eq_config_t*)mem_ptr)->num_bands*sizeof(eq_band_specs_t)))
            {

                uint32 band = 0;
                eq_config_t* eq_config_ptr = (eq_config_t*)mem_ptr;
                eq_band_specs_t* eq_band_specs_ptr = (eq_band_specs_t*)(eq_config_ptr+1);
                eq_band_internal_specs_t    eq_band_internal_specs[EQ_AUDIO_FX_BAND];
			    memset(&eq_band_internal_specs, 0, EQ_AUDIO_FX_BAND*sizeof(eq_band_internal_specs_t));
                uint32 headroom_req_db = 0;

                // headroom compute currently only supports audio FX EQ
                if (eq_config_ptr->eq_preset_id > EQ_PRESET_ROCK_AUDIO_FX || eq_config_ptr->eq_preset_id < EQ_USER_CUSTOM_AUDIO_FX)
                {
                    return EQ_FAILURE;
                }
                // when we use audio FX EQ, the gain adjust flag must be set
                if (eq_config_ptr->gain_adjust_flag != 1)
                {
                    return EQ_FAILURE;
                }
                // when we use audio FX EQ, num of band must be EQ_AUDIO_FX_BAND
                if (eq_config_ptr->num_bands != EQ_AUDIO_FX_BAND)
                {
                    return EQ_FAILURE;
                }

                // convert millibels into dB
                for (band=0; band < eq_config_ptr->num_bands; band++)
                {
                    eq_band_internal_specs[band].iFilterGain = divide_int32_qx(eq_band_specs_ptr[band].gain_millibels, 100, 0);
                    eq_band_internal_specs[band].FiltType = eq_band_specs_ptr[band].filter_type;
                }

                // compute needed headroom in dB
                eq_headroom_req_compute(&headroom_req_db, eq_band_internal_specs);

                // convert dB back to millibels
                *((uint32*)mem_ptr) = headroom_req_db;

                *param_size_ptr = sizeof(uint32);

            }
            else
            {
                return EQ_MEMERROR;
            }
            break;



        default:
            return EQ_FAILURE; // invalid id
    }

    return EQ_SUCCESS;
}


/*======================================================================

  FUNCTION      eq_set_param

  DESCRIPTION   Set the params in the lib memory

  DEPENDENCIES  Input pointers must not be NULL.

  PARAMETERS    eq_lib_ptr: [in, out] Pointer to lib structure
param_id: [in] ID of the param
mem_ptr: [in] Pointer to the memory where the values stored are used to set up the params in the lib memory
mem_size:[in] Size of the memory pointed by mem_ptr

SIDE EFFECTS  None

======================================================================*/
EQ_RESULT eq_set_param(eq_lib_t* eq_lib_ptr, uint32 param_id, int8* mem_ptr, uint32 mem_size)
{
    eq_lib_mem_t* eq_lib_mem_ptr = (eq_lib_mem_t*)eq_lib_ptr->lib_mem_ptr;
    if(eq_lib_mem_ptr == NULL)
    {
    	return EQ_FAILURE;
    }
    eq_cross_fade_mode_t    eq_cross_fade_mode;
    eq_mode_t               new_mode;
    uint32  paramSize;

    // other than reset processing, mem ptr must be valid
    if (param_id != EQ_PARAM_SET_RESET  &&  mem_ptr == NULL)
    {
        return EQ_MEMERROR;
    }


    switch (param_id)
    {
        case EQ_PARAM_SET_MODE_EQ_ONOFF:
            if (mem_size == sizeof(eq_mode_t))
            {
                new_mode = *((eq_mode_t *)mem_ptr);

                // if mode changes, trigger cross-fading for on/off transition
                if (eq_lib_mem_ptr->eq_mode != new_mode) {
                    eq_lib_mem_ptr->eq_mode = new_mode;
                    eq_lib_mem_ptr->eq_on_off_crossfade_mode = EQ_TRUE;
                    eq_cross_fade_mode = EQ_TRUE;
                    if(audio_cross_fade_set_param(&eq_lib_mem_ptr->cross_fade_lib_mem, CROSS_FADE_PARAM_MODE, (int8*)(&eq_cross_fade_mode), sizeof(eq_cross_fade_mode_t)) != CROSS_FADE_SUCCESS)
                    {
                        return EQ_FAILURE;
                    }
                }
            }
            else
            {
                return EQ_FAILURE;  // parameter size doesn't match
            }
            break;

        case EQ_PARAM_SET_MODE_EQ_CHANGE:
            if (mem_size == sizeof(eq_mode_t))
            {
                new_mode = *((eq_mode_t *)mem_ptr);
                eq_lib_mem_ptr->eq_mode = new_mode;
            }
            else
            {
                return EQ_FAILURE;  // parameter size doesn't match
            }
            break;
        case EQ_PARAM_SET_CONFIG:

            if(mem_size >= (sizeof(uint32)+sizeof(int32)+sizeof(eq_settings_t))  &&  mem_size <= ALIGN8(sizeof(eq_config_t) + EQ_MAX_BANDS*sizeof(eq_band_specs_t)))
            {
                eq_config_t *eq_config_ptr = (eq_config_t*)mem_ptr;
                eq_msiir_settings_t   eq_msiir_settings;
                uint32 band, coeff_size;

                eq_band_specs_t *eq_band_specs_lib_ptr = (eq_band_specs_t*)(eq_lib_mem_ptr->eq_config_ptr+1);


                // check if id is valid
                if(eq_config_ptr->eq_preset_id < EQ_USER_CUSTOM || eq_config_ptr->eq_preset_id >= EQ_MAX_SETTINGS_AUDIO_FX || eq_config_ptr->eq_preset_id == EQ_MAX_SETTINGS_QCOM)
                {
                    return EQ_FAILURE;
                }


                // store basic config info
                eq_lib_mem_ptr->eq_config_ptr->eq_preset_id = eq_config_ptr->eq_preset_id;
                eq_lib_mem_ptr->eq_config_ptr->gain_adjust_flag = eq_config_ptr->gain_adjust_flag;
                eq_lib_mem_ptr->eq_config_ptr->eq_pregain = eq_config_ptr->eq_pregain;


                // if we use customize id, either audio fx or qcom
                if(eq_lib_mem_ptr->eq_config_ptr->eq_preset_id == EQ_USER_CUSTOM || eq_lib_mem_ptr->eq_config_ptr->eq_preset_id == EQ_USER_CUSTOM_AUDIO_FX)
                {

                    eq_band_specs_t *eq_band_specs_wrapper_ptr = (eq_band_specs_t*)(eq_config_ptr+1);

                    // check if num of band is out of range
                    if (eq_config_ptr->num_bands < 1  ||  eq_config_ptr->num_bands > EQ_MAX_BANDS)
                    {
                        return EQ_FAILURE;
                    }

                    // user must provide exactly 5 bands for customized audio fx EQ
                    if (eq_lib_mem_ptr->eq_config_ptr->eq_preset_id == EQ_USER_CUSTOM_AUDIO_FX  &&  eq_config_ptr->num_bands != EQ_AUDIO_FX_BAND)
                    {
                        return EQ_FAILURE;
                    }
                    eq_lib_mem_ptr->eq_config_ptr->num_bands = eq_config_ptr->num_bands;


                    // fill in API-interfacing specs
                    for (band=0; band < eq_lib_mem_ptr->eq_config_ptr->num_bands; band++)
                    {
                        eq_band_specs_lib_ptr[band].filter_type = eq_band_specs_wrapper_ptr[band].filter_type;

                        // if audio fx customize id is used, store freq info from pre-defined table
                        if (eq_lib_mem_ptr->eq_config_ptr->eq_preset_id == EQ_USER_CUSTOM_AUDIO_FX && eq_config_ptr->num_bands == EQ_AUDIO_FX_BAND)
                        {
                            // safety check if freqs are set wrong when audio fx EQ is used
                            if(eq_band_specs_wrapper_ptr[band].freq_millihertz != eq_audio_fx_freqs[band])
                            {
                                return EQ_FAILURE;
                            }
                            eq_band_specs_lib_ptr[band].freq_millihertz = eq_audio_fx_freqs[band];
                        }
                        else // if qcom customize id is used, store freq info from wrapper
                        {
                            eq_band_specs_lib_ptr[band].freq_millihertz = eq_band_specs_wrapper_ptr[band].freq_millihertz;
                        }
                        eq_band_specs_lib_ptr[band].gain_millibels = eq_band_specs_wrapper_ptr[band].gain_millibels;
                        eq_band_specs_lib_ptr[band].quality_factor = eq_band_specs_wrapper_ptr[band].quality_factor;

                        // 1. use different quality factor only in last band(14k) for audio fx EQ depending on the gain set for this band
                        // 2. rest 4 bands use 256
                        // 3. purpose is to better match audio fx performance

                        /*
                           -15 <= gain <= -12 or 12 <= gain <= 15, use q = 100
                           -11 <= gain <= -10 or 10 <= gain <= 11, use q = 105
                           -9 or 9, use 110
                           -8 or 8, use 115
                           -7 or 7, use 120
                           -6 or 6, use 135
                           -5 and 5, use 155
                           -4 and 4, use 210
                           -3 <= gain <= 3, use 256
                           */

                        // if audio fx customize id is used
                        if (eq_lib_mem_ptr->eq_config_ptr->eq_preset_id == EQ_USER_CUSTOM_AUDIO_FX)
                        {
                            // if this band is audio fx EQ last band
                            if(band == eq_lib_mem_ptr->eq_config_ptr->num_bands-1)
                            {
                                // if audio fx EQ last band is 14k
                                if(eq_band_specs_lib_ptr[band].freq_millihertz == 14000000)
                                {
                                    uint32  index;

                                    for(index=0; index < EQ_GAIN_LEVELS; index++)
                                    {
                                        if(eq_band_specs_lib_ptr[band].gain_millibels == eq_quality_factor_table[0][index])
                                        {
                                            eq_band_specs_lib_ptr[band].quality_factor = eq_quality_factor_table[1][index];
                                            break; // break for loop once param is set
                                        }
                                    }
                                }
                                else
                                {
                                    return EQ_FAILURE; // audio fx EQ last band is not 14000Hz
                                }
                            }
                            else
                            {
                                // audio fx EQ: if not the last band, use 256 for quality factor
                                eq_band_specs_lib_ptr[band].quality_factor = 256;
                            }
                        }

                        // when given X bands, there must be X bands worth of specs data
                        // if given more specs data than X bands, rest data will be ignored and not used
                        if (eq_band_specs_wrapper_ptr[band].band_idx != band)
                        {
                            return EQ_FAILURE;
                        }
                        eq_band_specs_lib_ptr[band].band_idx = eq_band_specs_wrapper_ptr[band].band_idx;
                    }

                    // fill in internal specs
                    memscpy((void*)eq_lib_mem_ptr->eq_band_internal_specs_ptr, (size_t)eq_lib_mem_ptr->eq_band_internal_specs_size, (const void*)eq_band_specs_lib_ptr, (size_t)eq_lib_mem_ptr->eq_band_internal_specs_size); // copy all EQ_MAX_BANDS bands specs from external to internal
                    for (band=0; band < eq_lib_mem_ptr->eq_config_ptr->num_bands; band++)
                    {
                        eq_lib_mem_ptr->eq_band_internal_specs_ptr[band].FreqHz = divide_int32_qx(eq_lib_mem_ptr->eq_band_internal_specs_ptr[band].FreqHz, 1000, 0);
                        eq_lib_mem_ptr->eq_band_internal_specs_ptr[band].iFilterGain = divide_int32_qx(eq_lib_mem_ptr->eq_band_internal_specs_ptr[band].iFilterGain, 100, 0);
                    }


                    // if flag == 0, update needed headroom to 0dB
                    if (eq_lib_mem_ptr->eq_config_ptr->gain_adjust_flag == 0)
                    {
                        eq_lib_mem_ptr->eq_states_ptr->headroom_req_db = 0;
                    }
                    // if flag == 1 and qcom EQ is used, update needed headroom to 0dB
                    // if flag == 1 and audio fx EQ is used, update needed headroom to actual value
                    else if (eq_lib_mem_ptr->eq_config_ptr->gain_adjust_flag == 1)
                    {
                        // if qcom customize id is used, update needed headroom to 0dB
                        if (eq_lib_mem_ptr->eq_config_ptr->eq_preset_id == EQ_USER_CUSTOM)
                        {
                            eq_lib_mem_ptr->eq_states_ptr->headroom_req_db = 0;
                        }
                        // if audio fx customize id is used, update needed headroom to actual value
                        else if (eq_lib_mem_ptr->eq_config_ptr->eq_preset_id == EQ_USER_CUSTOM_AUDIO_FX)
                        {
                            eq_headroom_req_compute(&eq_lib_mem_ptr->eq_states_ptr->headroom_req_db, eq_lib_mem_ptr->eq_band_internal_specs_ptr);
                        }
                    }
                    else // invalid flag value
                    {
                        return EQ_FAILURE;
                    }

                    // configure msiir pre-gain
                    if(msiir_set_param(&eq_lib_mem_ptr->msiir_lib_mem, MSIIR_PARAM_PREGAIN, (void*)&eq_lib_mem_ptr->eq_config_ptr->eq_pregain, sizeof(msiir_pregain_t)) != MSIIR_SUCCESS)
                    {

                        return EQ_FAILURE;
                    }

                    // design msiir coeffs
                    if(eq_msiir_design(eq_lib_mem_ptr, &eq_msiir_settings) != EQ_SUCCESS)
                    {
                        return EQ_FAILURE;
                    }

                    // set up msiir coeffs
                    coeff_size = sizeof(msiir_config_t) + eq_msiir_settings.msiir_config.num_stages * sizeof(msiir_coeffs_t);
                    if(msiir_set_param(&eq_lib_mem_ptr->msiir_lib_mem, MSIIR_PARAM_CONFIG, (void*)&eq_msiir_settings, coeff_size) != MSIIR_SUCCESS)
                    {

                        return EQ_FAILURE;
                    }

                }
                else // if we use preset id, either audio fx or qcom
                {

                    // fill in internal specs
                    // equalizer_load_preset(.) will error out when preset id is not valid
                    if(equalizer_load_preset(eq_lib_mem_ptr->eq_band_internal_specs_ptr, &eq_lib_mem_ptr->eq_config_ptr->num_bands, (eq_settings_t)eq_lib_mem_ptr->eq_config_ptr->eq_preset_id) != EQ_SUCCESS)
                    {

                        return EQ_FAILURE;
                    }
                    // fill in API-interfacing specs
                    memscpy((void*)eq_band_specs_lib_ptr, (size_t)eq_lib_mem_ptr->eq_band_internal_specs_size, (const void*)eq_lib_mem_ptr->eq_band_internal_specs_ptr, (size_t)eq_lib_mem_ptr->eq_band_internal_specs_size); // copy all EQ_MAX_BANDS bands specs from internal to external
                    for (band=0; band < eq_lib_mem_ptr->eq_config_ptr->num_bands; band++)
                    {
                        eq_band_specs_lib_ptr[band].freq_millihertz = (uint32)s64_mult_u32_s16_shift(eq_lib_mem_ptr->eq_band_internal_specs_ptr[band].FreqHz, 1000, 16);
                        eq_band_specs_lib_ptr[band].gain_millibels = (int32)s64_mult_s32_s16_shift(eq_lib_mem_ptr->eq_band_internal_specs_ptr[band].iFilterGain, 100, 16);
                    }


                    // if flag == 0, update needed headroom to 0dB
                    if (eq_lib_mem_ptr->eq_config_ptr->gain_adjust_flag == 0)
                    {
                        eq_lib_mem_ptr->eq_states_ptr->headroom_req_db = 0;
                    }
                    // if flag == 1 and qcom EQ is used, update needed headroom to 0dB
                    // if flag == 1 and audio fx EQ is used, update needed headroom to actual value
                    else if (eq_lib_mem_ptr->eq_config_ptr->gain_adjust_flag == 1)
                    {
                        // if qcom preset is used, update needed headroom to 0dB
                        if (eq_lib_mem_ptr->eq_config_ptr->eq_preset_id >= EQ_PRESET_BLANK  &&  eq_lib_mem_ptr->eq_config_ptr->eq_preset_id < EQ_MAX_SETTINGS_QCOM)
                        {
                            eq_lib_mem_ptr->eq_states_ptr->headroom_req_db = 0;
                        }
                        // if audio fx preset is used, update needed headroom to actual value
                        else if (eq_lib_mem_ptr->eq_config_ptr->eq_preset_id >= EQ_PRESET_NORMAL_AUDIO_FX  &&  eq_lib_mem_ptr->eq_config_ptr->eq_preset_id < EQ_MAX_SETTINGS_AUDIO_FX)
                        {
                            eq_headroom_req_compute(&eq_lib_mem_ptr->eq_states_ptr->headroom_req_db, eq_lib_mem_ptr->eq_band_internal_specs_ptr);
                        }
                    }
                    else // invalid flag value
                    {
                        return EQ_FAILURE;
                    }


                    // configure msiir pre-gain
                    if(msiir_set_param(&eq_lib_mem_ptr->msiir_lib_mem, MSIIR_PARAM_PREGAIN, (void*)&eq_lib_mem_ptr->eq_config_ptr->eq_pregain, sizeof(msiir_pregain_t)) != MSIIR_SUCCESS)
                    {

                        return EQ_FAILURE;
                    }


                    // design msiir coeffs
                    if(eq_msiir_design(eq_lib_mem_ptr, &eq_msiir_settings) != EQ_SUCCESS)
                    {
                        return EQ_FAILURE;
                    }

                    // set up msiir coeffs
                    coeff_size = sizeof(msiir_config_t) + eq_msiir_settings.msiir_config.num_stages * sizeof(msiir_coeffs_t);
                    if(msiir_set_param(&eq_lib_mem_ptr->msiir_lib_mem, MSIIR_PARAM_CONFIG, (void*)&eq_msiir_settings, coeff_size) != MSIIR_SUCCESS)
                    {

                        return EQ_FAILURE;
                    }

                }

            }
            else
            {
                return EQ_MEMERROR;
            }

            break;

        case EQ_PARAM_CROSSFADE_FLAG:

            if(audio_cross_fade_set_param(&eq_lib_mem_ptr->cross_fade_lib_mem, CROSS_FADE_PARAM_MODE, mem_ptr, mem_size) != CROSS_FADE_SUCCESS)
            {

                return EQ_FAILURE;
            }

            break;


        case EQ_PARAM_CROSSFADE_CONFIG:
            if(audio_cross_fade_set_param(&eq_lib_mem_ptr->cross_fade_lib_mem, CROSS_FADE_PARAM_CONFIG, mem_ptr, mem_size) != CROSS_FADE_SUCCESS)
            {

                return EQ_FAILURE;
            }

            break;

        case EQ_PARAM_SET_RESET:

            // get the cross fade flag
            if(audio_cross_fade_get_param(&eq_lib_mem_ptr->cross_fade_lib_mem, CROSS_FADE_PARAM_MODE, (int8*)&eq_cross_fade_mode, sizeof(eq_cross_fade_mode_t), &paramSize) != CROSS_FADE_SUCCESS)
            {
                return EQ_FAILURE;
            }

            // reset is allowed during normal EQ processing
            if (eq_cross_fade_mode == 0)
            {
                if (msiir_set_param(&eq_lib_mem_ptr->msiir_lib_mem, MSIIR_PARAM_RESET, (void*)mem_ptr, 0) != MSIIR_SUCCESS)
                {
                    return EQ_FAILURE;
                }
            }
            // reset is NOT allowed during cross fading
            else if (eq_cross_fade_mode == 1)
            {
                return EQ_FAILURE;
            }
            else
            {
                return EQ_FAILURE; // invalid cross fade flag
            }

            break;

        default:
            return EQ_FAILURE; // invalid id
    }




    return EQ_SUCCESS;
}




/*======================================================================

  FUNCTION      eq_process

  DESCRIPTION   EQ processing of single channel audio signal.

  DEPENDENCIES  Input pointers must not be NULL.


  PARAMETERS    eq_lib_ptr[TOTAL_INST]: [in] Pointer to lib structure
out_ptr: [out] Pointer to single channel output PCM samples
in_ptr: [in] Pointer to single channel input PCM samples
sample_per_channel: [in] Number of samples to be processed

SIDE EFFECTS  None.

======================================================================*/
EQ_RESULT eq_process(eq_lib_t *eq_lib_ptr[TOTAL_INST], int8 *out_ptr, int8 *in_ptr, uint32 sample_per_channel)
{

    uint32 byte_per_sample;

    // add sanity check
    if(eq_lib_ptr == NULL || out_ptr == NULL || in_ptr == NULL || eq_lib_ptr[CUR_INST] == NULL)
        return EQ_FAILURE;

	eq_lib_mem_t* eq_cur_lib_mem_ptr = (eq_lib_mem_t*)eq_lib_ptr[CUR_INST]->lib_mem_ptr;

	if(eq_cur_lib_mem_ptr == NULL )
        return EQ_FAILURE;
		
    if(eq_cur_lib_mem_ptr->eq_static_struct_ptr->data_width == 16)
        byte_per_sample = 2;
    else if(eq_cur_lib_mem_ptr->eq_static_struct_ptr->data_width == 32)
        byte_per_sample = 4;
    else
        return EQ_FAILURE;

    if(eq_cur_lib_mem_ptr->eq_mode == EQ_DISABLE && eq_cur_lib_mem_ptr->eq_on_off_crossfade_mode == EQ_FALSE){
        memscpy(out_ptr, byte_per_sample*sample_per_channel, in_ptr, byte_per_sample*sample_per_channel);
    }
    else{ // do EQ

        // do processing for existing EQ effects
        if (msiir_process_v2(&eq_cur_lib_mem_ptr->msiir_lib_mem, (void*)out_ptr, (void*)in_ptr, sample_per_channel) != MSIIR_SUCCESS)
        {
            return EQ_FAILURE;
        }

        if(eq_cur_lib_mem_ptr->eq_on_off_crossfade_mode == EQ_TRUE){
            int8* cross_fade_in_ptr[TOTAL_INPUT];
            uint32  paramSize;

            if (eq_cur_lib_mem_ptr->eq_mode == EQ_DISABLE){  //Turning off
                cross_fade_in_ptr[CUR_INPUT]=(int8*)out_ptr; //EQ effects
                cross_fade_in_ptr[NEW_INPUT]=(int8*)in_ptr;  //bypass input
            }
            else{                                            //Turning on
                cross_fade_in_ptr[CUR_INPUT]=(int8*)in_ptr;  //bypass input
                cross_fade_in_ptr[NEW_INPUT]=(int8*)out_ptr; //EQ effects
            }

            // sanity check
            if (eq_cur_lib_mem_ptr->cross_fade_lib_mem.cross_fade_lib_mem_ptr == NULL)
                return EQ_FAILURE;

            // do cross fade between outputs of existing and new EQ effects and store results back into existing EQ output data buffer
            if(audio_cross_fade_process(&eq_cur_lib_mem_ptr->cross_fade_lib_mem, out_ptr, cross_fade_in_ptr, sample_per_channel) != CROSS_FADE_SUCCESS)
            {
                return EQ_FAILURE;
            }

            // update on-off crossfade mode
            // when ramp up/down is done, on-off crossfade mode is set to 0.
            if(audio_cross_fade_get_param(&eq_cur_lib_mem_ptr->cross_fade_lib_mem, CROSS_FADE_PARAM_MODE, (int8*)&eq_cur_lib_mem_ptr->eq_on_off_crossfade_mode, sizeof(eq_cross_fade_mode_t), &paramSize) != CROSS_FADE_SUCCESS)
            {
                return EQ_FAILURE;
            }

        }
        // if the new EQ lib does not exist, it means no need to do cross fading
        // on/off cross-fading and generic cross-fading can't happen at the same time
        // --> frame work prevent the above event.
        else if (eq_lib_ptr[NEW_INST] != NULL)
        {

            eq_lib_mem_t* eq_new_lib_mem_ptr = (eq_lib_mem_t*)eq_lib_ptr[NEW_INST]->lib_mem_ptr;
            eq_cross_fade_mode_t    eq_cross_fade_mode;
            uint32  paramSize;

            if(eq_new_lib_mem_ptr->eq_mode != EQ_ENABLE)
            {
                return EQ_FAILURE;
            }
            // get the cross fade flag
            if(audio_cross_fade_get_param(&eq_new_lib_mem_ptr->cross_fade_lib_mem, CROSS_FADE_PARAM_MODE, (int8*)&eq_cross_fade_mode, sizeof(eq_cross_fade_mode_t), &paramSize) != CROSS_FADE_SUCCESS)
            {

                return EQ_FAILURE;
            }


            // when cross fade flag is on, 1) do processing for new EQ effects 2) do cross fade
            if (eq_cross_fade_mode == 1)
            {

                int8* cross_fade_in_ptr[TOTAL_INPUT];

                cross_fade_in_ptr[CUR_INPUT]=(int8*)out_ptr;
                cross_fade_in_ptr[NEW_INPUT]=(int8*)in_ptr;


                // do processing for new EQ effects
                if (msiir_process_v2(&eq_new_lib_mem_ptr->msiir_lib_mem, (void*)in_ptr, (void*)in_ptr, sample_per_channel) != MSIIR_SUCCESS)
                {
                    return EQ_FAILURE;
                }


                // do cross fade between outputs of existing and new EQ effects and store results back into existing EQ output data buffer
                if(audio_cross_fade_process(&eq_new_lib_mem_ptr->cross_fade_lib_mem, out_ptr, cross_fade_in_ptr, sample_per_channel) != CROSS_FADE_SUCCESS)
                {

                    return EQ_FAILURE;
                }

            }
            else
            {
                return EQ_FAILURE;
            }

        }

    }//do EQ

    return EQ_SUCCESS;
}



// design msiir coeffs and re-format to the way required by msiir lib
EQ_RESULT eq_msiir_design(eq_lib_mem_t*  eq_lib_mem_ptr, eq_msiir_settings_t   *eq_msiir_settings_ptr)
{
    EQFilterDesign_t  EQFilterDesign;
    uint32  band, i;


    // init output of ProcessEQFD(filter coeffs, shift factor) to zeros to remove false positive KW warnings
    memset(EQFilterDesign.piFilterCoeff, 0, EQ_MAX_BANDS*(MSIIR_NUM_COEFFS + MSIIR_DEN_COEFFS));
    memset(EQFilterDesign.piNumShiftFactor, 0, EQ_MAX_BANDS);

    // calculate coeffs using filter design tool
    EQFilterDesign.uiNumBands = (uint16)eq_lib_mem_ptr->eq_config_ptr->num_bands;
    EQFilterDesign.uiSampleRate = eq_lib_mem_ptr->eq_static_struct_ptr->sample_rate;
    EQFilterDesign.pEQFilterDesignData = eq_lib_mem_ptr->eq_band_internal_specs_ptr;
    // even though num_bands is given as input design param, ProcessEQFD can return smaller number based on whether band freq > fs/2
    eq_msiir_settings_ptr->msiir_config.num_stages = (int32)ProcessEQFD_v2(&EQFilterDesign);

    // check if num of band after design is out of range
    if (eq_msiir_settings_ptr->msiir_config.num_stages < 1  ||  eq_msiir_settings_ptr->msiir_config.num_stages > EQ_MAX_BANDS)
    {
        return EQ_FAILURE;
    }

    // re-structure the computed coeffs to the format required by msiir lib
    for (band=0; band < (uint32)eq_msiir_settings_ptr->msiir_config.num_stages; band++)
    {
        for (i = 0; i < MSIIR_COEFF_LENGTH; ++i)
        {
            eq_msiir_settings_ptr->msiir_coeffs[band].iir_coeffs[i] = EQFilterDesign.piFilterCoeff[i + MSIIR_COEFF_LENGTH*band];

        }
        eq_msiir_settings_ptr->msiir_coeffs[band].shift_factor = EQFilterDesign.piNumShiftFactor[band];

    }


    return EQ_SUCCESS;

}



// compute needed headroom when audio fx EQ is used. this algorithm shares the same basic concept used in audio fx EQ
// 1. include contribution for cut(negative boost) from neighboring bands
// 2. halve contribution from neighboring bands if it is greater than or equal to contribution from self band
// 3. needed headroom is capped at 15dB max
void eq_headroom_req_compute(uint32 *headroom_needed_db, eq_band_internal_specs_t   *eq_band_internal_specs_ptr)
{

    uint32 gain_contribute_from_neighbors, margin=0;
    int32 m, n, gain, weighting, band_gain_db[EQ_AUDIO_FX_BAND];
    int32 max_gain = 0, avg_gain = 0, avg_count = 0;


    for (m=0; m < EQ_AUDIO_FX_BAND; m++)
    {
        band_gain_db[m] = eq_band_internal_specs_ptr[m].iFilterGain;
        if (eq_band_internal_specs_ptr[m].FiltType ==  EQ_BAND_CUT ||
            eq_band_internal_specs_ptr[m].FiltType ==  EQ_TREBLE_CUT ||
            eq_band_internal_specs_ptr[m].FiltType ==  EQ_BASS_CUT)
            band_gain_db[m] = -band_gain_db[m];
    }


    for (m = 0; m < EQ_AUDIO_FX_BAND; m++)
    {
        if (band_gain_db[m] >= max_gain)
        {

            int32 tmp_max_gain = band_gain_db[m];
            int32 tmp_avg_gain = 0;
            int32 tmp_avg_count = 0;

            // in case  tmp_avg_gain >= avg_gain  never holds true
            max_gain = band_gain_db[m];

            for (n = 0; n < EQ_AUDIO_FX_BAND; n++)
            {
                gain = band_gain_db[n];
                // skip current band
                if (n == m)
                {
                    continue;
                }

                if (gain > tmp_max_gain)
                {
                    tmp_avg_gain = -1;
                    break;
                }

                weighting = 1;
                if (n < (m + 2) && n > (m - 2)) // if within proximity
                {
                    if (gain >= max_gain){
                        weighting = 6;
                    }
                    else
                        weighting = 4;
                }


                tmp_avg_gain += weighting * gain;
                tmp_avg_count += weighting;
            }
            if (tmp_avg_gain >= avg_gain) // final modify
            {
                max_gain = tmp_max_gain;
                avg_gain = tmp_avg_gain;
                avg_count = tmp_avg_count;
            }
        }

    }


    // max_gain is L32Q0
    margin = (uint32)s64_mult_u32_s16_shift((uint32)s64_mult_s32_s16_shift(max_gain, ONE_OVER_FIFTEEN, 16), 100, 0);
    *headroom_needed_db = (uint32)s64_mult_s32_s32(max_gain,100);
    *headroom_needed_db += margin ; //Give margin to compensate estimation error

    if (avg_count)
    {
        // if neighboring bands' total contribution is in same magnitude as the self band's, adjust it by halving it
        gain_contribute_from_neighbors = (uint32)divide_int32_qx(avg_gain, avg_count, 16);
        gain_contribute_from_neighbors = (uint32) s64_mult_u32_s16_shift(gain_contribute_from_neighbors, 100, 0);

        *headroom_needed_db += gain_contribute_from_neighbors;
        if (*headroom_needed_db >= 3099)
        {
            *headroom_needed_db += 25;
        }
    }

}


