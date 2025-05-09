
/* Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef TANGENT_H
#define TANGENT_H



#include "audio_basic_op.h"
#include "posal_types.h"
#include "audpp_common.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


# define ANGLES_LENGTH 20
# define FRACTIONAL_BITS 30
# define UNIT_DEFINED_QFACTOR 1073741824

int32 tangent_v2 (int32 theta, int n );
int32 mul32x32Shift2_v2(int32 x, int32 y, int16 shift);
PPStatus dsplib_approx_divideEQFD_v2(int32 numer,int32 denom,int32 *result,int32 *shift_factor);
//PPStatus dsplib_approx_invert(int32 input,int32 *result,int32 *shift_factor);
PPStatus dsplib_taylor_invert_v2(int32 ,int32 *,int32 *,int32 );
int32 qdsp_norm_v2(int32 input,int32 *output, int32 *shift_factor);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif  /*TANGENT_H*/
