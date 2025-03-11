/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*===========================================================================*]
[* FILE NAME: converter_state.c                                              *]
[* DESCRIPTION:                                                              *]
[*    Functions regarding rate convert state struct.                         *]
[* FUNCTION LIST :                                                           *]
[*    rateConvertState_predict_inputs: Determine no. of input samples needed.*]
[*    rateConvertState_update_index: Update index in convert state.          *]
[*===========================================================================*/
#include "audpp_converter.h"

/*===========================================================================*/
/* FUNCTION : rateConvertState_predict_inputs                                */
/*                                                                           */
/* DESCRIPTION: Predict input sample number from output sample number        */
/*                                                                           */
/* INPUTS: state-> rate convert state struct                                 */
/*         outputs: desired output samples                                   */
/* OUTPUTS: function returns predicted input sample number                   */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*     This function involves mult of (sample number Q0 * playback rate Q16).*/
/* To prevent the result from overflowing an int32 register, we need:        */
/*       (sample number L16Q0) * (playback rate L32Q16) <= 0x7FFF FFFF.     */
/* This condition is not hard to satisfy with reasonable playback rate and a */
/* reasonable size of sample number. So, we directly multiply them without   */
/* checking saturation.                                                      */
/*===========================================================================*/
int32 rateConvertState_predict_inputs
(    
    rateConvertState    *state,             /* rate convert state struct     */
    int32                outputs            /* output sample number          */
)
{
    int32 newIndexL32Q16;
    int32 newRateL32Q16;
    int32 indexL32Q16 = state->indexL32Q16;
    int32 rateL32Q16  = state->rateL32Q16;
    int32  accelL32Q16  = state->accelL32Q16;
    int32  tmpL32;
    /*-----------------------------------------------------------------------*/

    /* if no rate change, directly get new index in Q16 */
    if (accelL32Q16 == 0) 
    {
        /* newIndexL32Q16 = indexL32Q16 + (outputs - 1) * rateL32Q16; */
        tmpL32 = s32_extract_s40_l(s64_mult_s32_s32(rateL32Q16, outputs-1));
        newIndexL32Q16 = s32_add_s32_s32(indexL32Q16, tmpL32);
    }
    else /* if (accelL32Q16 != 0) */
    {
        /* new rate at the last input to generate the desired output   */
        /* newRateL32Q16 = rateL32Q16 + (outputs - 1) * accelL32Q16; */
        tmpL32 = s32_extract_s40_l(s64_mult_s32_s32(accelL32Q16, outputs-1));
        newRateL32Q16 = s32_add_s32_s32(rateL32Q16, tmpL32);
        /* calculate last input's index with changing playback rate */
        /* newIndexL32Q16 = indexL32Q16 + (outputs * (rateL32Q16 */
        /*    + newRateL32Q16) / 2) - newRateL32Q16;              */
        tmpL32 = s32_shr_s32_sat(s32_add_s32_s32(rateL32Q16, newRateL32Q16), 1);
        tmpL32 = s32_extract_s40_l(s64_mult_s32_s32(tmpL32, outputs));
        tmpL32 = s32_add_s32_s32(indexL32Q16, tmpL32);
        newIndexL32Q16 = s32_sub_s32_s32(tmpL32, newRateL32Q16);        
    } /* end of else */

    /* check if overflow occured */
    if (!(newIndexL32Q16 >= 0 && newIndexL32Q16 < Q31_ONE /* 32767, Q16 */))
    {
        return PPERR_RATECONVERTER_INVALID_INDEX;
    }

    /* return input sample number, L16Q0 */
    return s16_extract_s32_h(newIndexL32Q16);
} /*-------------- end of function rateConvertState_predict_inputs ----------*/


/*===========================================================================*/
/* FUNCTION : rateConvertState_update_index                                  */
/*                                                                           */
/* DESCRIPTION: Update index in converter state according to number of       */
/*              converted input samples.                                     */
/*                                                                           */
/* INPUTS: state-> rate convert state struct                                 */
/*         inputs-> number of converted input samples                        */
/* OUTPUTS: index in converter state updated                                 */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
void rateConvertState_update_index
(    
    rateConvertState    *state,             /* rate convert state struct     */
    int32               *inputs             /* input sample number           */
)
{
    /* set if lower */
    *inputs = s32_min_s32_s32(*inputs, s16_extract_s32_h(state->indexL32Q16));
    
    /*  index -= inputs << 16  */
    state->indexL32Q16 = s32_sub_s32_s32(state->indexL32Q16, s32_shl_s32_sat(*inputs, 16));
} /*------------- end of function rateConvertState_update_index -------------*/


// other member functions of rate convert state not ported here:
// - SimulateConversion
// - Advance
