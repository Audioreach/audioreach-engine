/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
/*===========================================================================
 * FILE NAME: apply_gain.h
 * DESCRIPTION:
 *    Volume and Balance Controls 
 *===========================================================================*/


#ifndef  _APPLYGAIN_H_
#define  _APPLYGAIN_H_

#include "audio_basic_op.h"
#include "audio_basic_op_ext.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

void audpp_volume_apply_gain_16(int16* outptr,   /* Pointer to output */
				int16* inptr,    /* Pointer to input */
				uint16 gain,     /* Gain in Q12 format */
				uint32 samples);   /* No of samples to which the gain is to be applied */

void audpp_volume_apply_gain_32_G1(int32* outptr,   /* Pointer to output */
				int32* inptr,    /* Pointer to input */
				uint16 gain,     /* Gain in Q12 format */
				uint32 samples);   /* No of samples to which the gain is to be applied */

void audpp_volume_apply_gain_32_L1(int32* outptr,   /* Pointer to output */
				int32* inptr,    /* Pointer to input */
				uint16 gain,     /* Gain in Q12 format */
				uint32 samples);   /* No of samples to which the gain is to be applied */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // _APPLYGAIN_H

