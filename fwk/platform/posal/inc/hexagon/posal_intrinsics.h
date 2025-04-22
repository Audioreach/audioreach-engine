#ifndef _POSAL_INTRINSICS_H_
#define _POSAL_INTRINSICS_H_
/**
 * \file posal_intrinsics.h
 * \brief
 *  	 This file contains basic types and pre processor macros.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* -----------------------------------------------------------------------
 ** Standard Types
 ** ----------------------------------------------------------------------- */
#if ((defined __hexagon__) || (defined __qdsp6__))
#include <hexagon_protos.h>
#endif

// 64 byte alignment for hexagon.
static const uint32_t CACHE_ALIGNMENT = 0x3F;

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#define s32_ct1_s32(var1)     Q6_R_ct1_R(var1)

/** Counts the leading ones starting from the MSB.*/
#ifndef s32_cl1_s32
#define s32_cl1_s32(var1)     (Q6_R_cl1_R(var1))
#endif

/** Counts the leading zeros starting from the MSB.*/
#define s32_cl0_s32(var1)     Q6_R_cl0_R(var1)

/** counts the trailing zeros. */
#define s32_get_lsb_s32(var1) 	  Q6_R_ct0_R(var1)

#define s32_shl_s32_sat(var1, shift)  Q6_R_asl_RR_sat(var1, shift)

#define s32_shr_s32_sat(var1, shift)  Q6_R_asr_RR_sat(var1, shift)

#define u32_popcount_u64(var1) Q6_R_popcount_P(var1)

/**
  Saturates the input signed 32-bit input number (possibly exceeding the
  16-bit dynamic range) to the signed 16-bit number and returns the result.
*/
#ifndef s16_saturate_s32
#define s16_saturate_s32(var1)  (int16) Q6_R_sath_R(var1)
#endif /* #ifndef s16_saturate_s32 */

/**
  Saturates the input signed 64-bit input number (possibly exceeding the
  32-bit dynamic range) to the signed 32-bit number and returns the result.
*/
#ifndef s32_saturate_s64
#define s32_saturate_s64(var1)  Q6_R_sat_P(var1)
#endif /* #ifndef s32_saturate_s64 */

/**
  Adds two signed 16-bit integers and returns the signed, saturated
  16-bit sum.
*/
#ifndef s16_add_s16_s16_sat
#define s16_add_s16_s16_sat(var1, var2)  (int16) Q6_R_add_RlRl_sat(var1, var2)
#endif /* #ifndef s16_add_s16_s16_sat */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* #ifndef _POSAL_INTRINSICS_H_ */