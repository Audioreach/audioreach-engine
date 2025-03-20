/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*===========================================================================*]
[* FILE NAME: buffer_converter.c                                             *]
[* DESCRIPTION:                                                              *]
[*    Functions regarding buffer rate conversion.                            *]
[* FUNCTION LIST :                                                           *]
[*    buf_rate_converter_convert: Process function of buffer rate converter  *]
[*    buf_rate_converter_reset: Reset buffer rate converter                  *]
[*    buf_rate_converter_setrate: Set playback rate of the converter         *]
[*===========================================================================*/
#include "audpp_converter.h"

/*===========================================================================*/
/* FUNCTION : buf_rate_converter_convert                                     */
/*                                                                           */
/* DESCRIPTION: Convert certain number of output samples from certain number */
/*              of input samples. Output to output buffer and update the     */
/*              input and output sample number depends on how many get       */
/*              actually processed.                                          */
/*                                                                           */
/* INPUTS: destBuf-> output buffer                                           */
/*         outputsPtr-> number of desired output samples                     */
/*         srcBuf-> input buffer                                             */
/*         inputsPtr-> number of input samples                               */
/*         converter-> buffer rate converter struct                          */
/* OUTPUTS: destBuf-> output buffer                                          */
/*          outputsPtr-> number of desired output samples                    */
/*          inputsPtr-> number of input samples                              */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
uint32 buf_rate_converter_convert
(
    int16               *destBuf,           /* output buffer                 */
    int32               *outputsPtr,        /* -> output number of samples   */
    int16               *srcBuf,            /* input buffer                  */
    int32               *inputsPtr,         /* -> input number of samples    */
    bufRateCvtrStruct   *converter          /* buffer rate converter struct  */
)
{
    int32 n1, n2;
    int16 tmpInput[3];

    /* get input and output sample number value */
    int32 outputs = *outputsPtr;
    int32 inputs  = *inputsPtr;

    int16 *oldInputs = converter->oldInputs;

    /*------------------- convert samples from old inputs -------------------*/
    n1 = 0;
    if (converter->srcState.indexL32Q16 < Q16_TWO) 
    {
        if (inputs == 0) 
        {   /* if there are no inputs, use two old input samples */
            n1 = convert_linear(
                destBuf, outputs, oldInputs, 2, &converter->srcState);
        }
        else /* (inputs != 0) */
        {   /* use two old input samples and the first from input buffer */
            tmpInput[0] = oldInputs[0];
            tmpInput[1] = oldInputs[1];
            tmpInput[2] = srcBuf[0];
            n1 = convert_linear(
                destBuf, outputs, tmpInput, 3, &converter->srcState);
        } /* end of else if (inputs != 0) */

        /* update according to converted number of samples */
        destBuf += n1;
        outputs -= n1;
    } /* end of if (converter->srcState.indexL32Q16 < Q16_TWO) */

    /*--------------- convert samples directly from input buffer ------------*/
    n2 = 0;
    if (outputs > 0 && converter->srcState.indexL32Q16 >= Q16_TWO) 
    {
        converter->srcState.indexL32Q16 =
            s32_sub_s32_s32(converter->srcState.indexL32Q16, Q16_TWO);

        n2 = convert_linear(
            destBuf, outputs, srcBuf, inputs, &converter->srcState);

        /* update according to converted number of samples */
        destBuf += n2;
        outputs -= n2;

        /* s16_add_s16_s16 two to the index in converter state */
        converter->srcState.indexL32Q16 =
            s32_add_s32_s32(converter->srcState.indexL32Q16, Q16_TWO);
    } /* end of if (outputs>0 && converter->srcState.indexL32Q16 >= Q16_TWO) */

    /* make sure that outputs > 0 */
    if(!(outputs >= 0))
    {
        return PPERR_RATECONVERTER;
    }
    outputs = n1 + n2;

    /*------------------------- update state buffer -------------------------*/
    /* update index */
    rateConvertState_update_index(&converter->srcState, &inputs);
    
    /* update old inputs */
    if (inputs >= 2) 
    {
        oldInputs[0] = srcBuf[inputs-2];
        oldInputs[1] = srcBuf[inputs-1];
    }
    else if (inputs == 1) 
    {
        oldInputs[0] = oldInputs[1];
        oldInputs[1] = srcBuf[inputs - 1];
    } /* end of else if (inputs == 1) */

    /* store outputs and inputs values back */
    *outputsPtr = outputs;
    *inputsPtr  = inputs;
	return PPSUCCESS;
} /*---------------- end of funcion buf_rate_converter_convert --------------*/


/*===========================================================================*/
/* FUNCTION : buf_rate_converter_reset                                       */
/*                                                                           */
/* DESCRIPTION: Reset buffer rate converter                                  */
/*                                                                           */
/* INPUTS: converter-> buffer rate converter struct                          */
/* OUTPUTS: converter-> buffer rate converter struct                         */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
void buf_rate_converter_reset
(
    bufRateCvtrStruct   *converter          /* buffer rate converter struct  */
)
{
    converter->srcState.indexL32Q16 = Q16_TWO;
    converter->oldInputs[0] = 0;
    converter->oldInputs[1] = 0;
} /*------------- end of function buf_rate_converter_reset ------------------*/


/*===========================================================================*/
/* FUNCTION : buf_rate_converter_setrate                                     */
/*                                                                           */
/* DESCRIPTION: Set playback rate of buffer rate converter                   */
/*                                                                           */
/* INPUTS: converter-> buffer rate converter struct                          */
/* OUTPUTS: converter-> buffer rate converter struct                         */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
void buf_rate_converter_setrate
(
    bufRateCvtrStruct   *converter,         /* buffer rate converter struct  */
    int32               playbackRateL32Q16/* playback rate in Q16          */
)
{
    converter->srcState.rateL32Q16 = playbackRateL32Q16;
    converter->srcState.accelL32Q16 = 0;
} /*------------- end of function buf_rate_converter_setrate ----------------*/
