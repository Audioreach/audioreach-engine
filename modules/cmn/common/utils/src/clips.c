#include "audio_clips.h"
#include "ar_defs.h"

/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*===========================================================================*/
/* FUNCTION : clip_16                                                        */
/*                                                                           */
/* DESCRIPTION: Read interface value and limit its range.                    */
/*                                                                           */
/* INPUTS: input-> pointer to the interface value                            */
/*         lowerBound: valid minimum value                                   */
/*         upperBound: valid maximum value                                   */
/* OUTPUTS: input-> pointer to the interface value                           */
/*          function returns if the input value is clipped                   */
/*===========================================================================*/
bool_t clip_16
(
    int16_t          *input,                  /* ptr to input value            */
    int16_t           lowerBound,             /* valid minimum value           */
    int16_t           upperBound              /* valid maxmum value            */
)
{
    if (*input < lowerBound)
    {
        *input = lowerBound;
        return TRUE;
    }

    if (*input > upperBound)
    {
        *input = upperBound;
        return TRUE;
    }
    return FALSE;
} /*----------------------- end of function clip_16 -------------------------*/


/*===========================================================================*/
/* FUNCTION : clip_32                                                        */
/*                                                                           */
/* DESCRIPTION: Read interface value and limit its range.                    */
/*                                                                           */
/* INPUTS: input-> pointer to the interface value                            */
/*         lowerBound: valid minimum value                                   */
/*         upperBound: valid maximum value                                   */
/* OUTPUTS: input-> pointer to the interface value                           */
/*          function returns if the input value is clipped                   */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/
bool_t clip_32
(
    int32_t          *input,                  /* ptr to input value            */
    int32_t           lowerBound,             /* valid minimum value           */
    int32_t           upperBound              /* valid maxmum value            */
)
{
    if (*input < lowerBound)
    {
        *input = lowerBound;
        return TRUE;
    }

    if (*input > upperBound)
    {
        *input = upperBound;
        return TRUE;
    }
    return FALSE;
} /*----------------------- end of function clip_16 -------------------------*/

