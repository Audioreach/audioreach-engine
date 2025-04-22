#ifndef _POSAL_INTRINSICS_H_
#define _POSAL_INTRINSICS_H_
/**
 * \file posal_intrinsics.h
 * \brief
 *  	 This file contains generic implementation of some intrinsics
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "posal_types.h"

/* -----------------------------------------------------------------------
 ** Standard Types
 ** ----------------------------------------------------------------------- */
// 64 bit alignment by default.
static const uint32_t CACHE_ALIGNMENT = 0x7;

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#if !defined(uses_s32_ct1_s32)
/** Counts the trailing ones starting from the LSB.*/
static inline int32_t s32_ct1_s32(int32_t x)
{
   int32_t num = 0;
   for (int i = 0; i < 32; i++)
   {
      if (!(x & 0x1))
      {
         break;
      }
      num++;
      x = x >> 1;
   }
   return num;
}
#define uses_s32_ct1_s32
#endif /* uses_s32_ct1_s32 */

#if !defined(uses_s32_cl1_s32)
/** Counts the leading ones starting from the MSB.*/
static inline int32_t s32_cl1_s32(int32_t var1)
{
    int32 cnt = 0;
    unsigned int mask = 1 << (31-cnt); //start from MSB

    while (0 != (var1 & mask ))
    {
        cnt++;
        mask = 1 << (31-cnt);
    }

#ifdef WMOPS_FX
    counter_fx.norm++;
    counter_fx.deposit--;
    counter_fx.norm--;
#endif

    return (cnt);
}
#define uses_s32_cl1_s32
#endif /* uses_s32_cl1_s32 */

#if !defined(uses_s32_cl0_s32)
/** Counts the leading zeros starting from the MSB.*/
static inline int32_t s32_cl0_s32(int32_t var)
{
    int count = 0;
    int i;

    for ( i = 31 ; i >= 0 ; i-- ) {
        if ((var & ( 0x01 << i )) == 0) {
            count++;
        }
        else {
            break;
        }
    }
    return count;
}
#define uses_s32_cl0_s32
#endif /* uses_s32_cl0_s32 */

#if !defined(uses_s32_get_lsb_s32)
/** Counts the trailing zeros starting from the LSB.*/
static inline int32_t s32_get_lsb_s32(int32_t var1)
{
  int index=0;

  while((~var1)&1){
    var1>>=1;
    index++;
  }
  return index;
}
#define uses_s32_get_lsb_s32
#endif /* uses_s32_get_lsb_s32 */

#if !defined(uses_u32_popcount_u64)
/** Counts all the ones in the 64 bit unsigned int */
static inline uint32_t u32_popcount_u64(uint64_t x)
{
	uint32_t sum = 0;
	for (uint32_t i = 0; i < sizeof(uint64_t)* 8; ++i)
	{
		sum += (x >> i) & 1;
	}
	return sum;
}
#define uses_u32_popcount_u64
#endif /* uses_u32_popcount_u64 */


/** Maximum value of a signed 32-bit integer.
*/
static const int32 P_MAX_32 = 0x7FFFFFFFL;

/** Maximum value of an unsigned 32-bit integer.
*/
static const uint32 P_UMAX_32 = 0xFFFFFFFFL;

/** Minimum value of a signed 32-bit integer.
*/
static const int32 P_MIN_32 = 0x80000000L;

/** Minimum value of a signed 32-bit integer.
*/
#define P_LONGWORD_MIN  P_MIN_32

/** Maximum value of a signed 32-bit integer.
*/
#define P_LONGWORD_MAX  P_MAX_32

/** Bitmask for the sign bit in a signed 32-bit integer.
*/
#define P_LONGWORD_SIGN  P_MIN_32       /* sign bit */

#if !defined(uses_s32_shl_s32_sat)

static inline int32 s32_shr_s32_sat(int32_t var1, int16_t shift);

/**
 * Arithmetic left shift
 */
static inline int32 s32_shl_s32_sat(int32_t var1, int16_t shift)
{
    int32 L_Mask, L_Out = 0;
    int i, iOverflow, giOverflow = 0;

    if (shift == 0 || var1 == 0)
    {
        L_Out = var1;
    }
    else if (shift < 0)
    {
        if (shift <= -31)
        {
            if (var1 > 0)
                L_Out = 0;
            else
                L_Out = P_UMAX_32;
        }
        else
        {
            L_Out = s32_shr_s32_sat(var1, (int16)(-shift));

#ifdef WMOPS_FX
            counter_fx.sh_sat--;
#endif

        }
    }
    else
    {
        if (shift >= 31)
            iOverflow = 1;
        else
        {
            if (var1 < 0)
                L_Mask = P_LONGWORD_SIGN; /* sign bit mask */
            else
                L_Mask = 0x0;
            L_Out = var1;
            for (i = 0; i < shift && !iOverflow; i++)
            {
                /* check the sign bit */
                L_Out = (L_Out & P_LONGWORD_MAX) << 1;
                if ((L_Mask ^ L_Out) & P_LONGWORD_SIGN)
                    iOverflow = 1;
            }
        }

        if (iOverflow)
        {
            /* s16_saturate_s32 */
            if (var1 > 0)
                L_Out = P_LONGWORD_MAX;
            else
                L_Out = P_LONGWORD_MIN;

            giOverflow = 1;
        }
    }

#ifdef WMOPS_FX
    counter_fx.sh_sat++;
#endif

    return (L_Out);
}
#define uses_s32_shl_s32_sat
#endif /* uses_s32_shl_s32_sat */


#if !defined(uses_s32_shr_s32_sat)
/**
 * Arithmetic right shift
 */
static inline int32 s32_shr_s32_sat(int32_t var1, int16_t shift)
{
    int32 L_Mask, L_Out;
    int giOverflow = 0;

    if (shift == 0 || var1 == 0)
    {
        L_Out = var1;
    }
    else if (shift < 0)
    {
        /* perform a left shift */
        /*----------------------*/
        if (shift <= -31)
        {
            /* s16_saturate_s32 */
            if (var1 > 0)
            {
                L_Out = P_LONGWORD_MAX;
                giOverflow = 1;
            }
            else
            {
                L_Out = P_LONGWORD_MIN;
                giOverflow = 1;
            }
        }
        else
        {
            L_Out = s32_shl_s32_sat(var1, (int16_t)(-shift));  //  OP_COUNT(-2);

#ifdef WMOPS_FX
            counter_fx.sh_sat--;
#endif

        }
    }
    else
    {

        if (shift >= 31)
        {
            if (var1 > 0)
                L_Out = 0;
            else
                L_Out = P_UMAX_32;
        }
        else
        {
            L_Mask = 0;
            if (var1 < 0)
            {
                L_Mask = ~L_Mask << (32 - shift);
            }
            var1 >>= shift;
            L_Out = L_Mask | var1;
        }
    }

#ifdef WMOPS_FX
    counter_fx.sh_sat++;
#endif

    return (L_Out);
}
#define uses_s32_shr_s32_sat
#endif /* uses_s32_shr_s32_sat */

#if !defined(uses_s16_saturate_s32)
/**
  Saturates the input signed 32-bit input number (possibly exceeding the
  16-bit dynamic range) to the signed 16-bit number and returns that
  result.
*/
int16 s16_saturate_s32(int32 var1);
#define uses_s16_saturate_s32
#endif /* uses_s16_saturate_s32 */

#if !defined(uses_s32_saturate_s64)
/**
  Saturates the input signed 64-bit input number (possibly exceeding the
  32-bit dynamic range) to the signed 32-bit number and returns that
  result.
*/
int32 s32_saturate_s64(int64 var1);
#define uses_s32_saturate_s64
#endif /* uses_s32_saturate_s64 */

#if !defined(uses_s16_add_s16_s16_sat)
/**
  Adds two signed 16-bit integers and returns the signed, saturated
  16-bit sum.
*/
int16 s16_add_s16_s16_sat(int16 var1, int16 var2);
#define uses_s16_add_s16_s16_sat
#endif /* uses_s16_add_s16_s16_sat */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* #ifndef _POSAL_INTRINSICS_H_ */

