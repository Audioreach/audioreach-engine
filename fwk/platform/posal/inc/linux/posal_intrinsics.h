#ifndef _POSAL_INTRINSICS_H_
#define _POSAL_INTRINSICS_H_
/**
 * \file posal_intrinsics.h
 * \brief
 *  	 This file contains linux implementation of some intrinsics
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
static inline int32_t s32_cl1_s32(int32_t x)
{
   int32_t num = 0;
   unsigned int mask = 1 << (31-num); //start from MSB

   while (0 != (x & mask ))
   {
      num++;
      mask = 1 << (31-num);
   }

   return num;
}
#define uses_s32_cl1_s32
#endif /* uses_s32_cl1_s32 */

#if !defined(uses_s32_cl0_s32)
/** Counts the leading zeros starting from the MSB.*/
static inline int32_t s32_cl0_s32(int32_t x)
{
   int32_t num = 0;
   while(x >= 0)
   {
      x = x << 1;
      num++;
   }
   return num;
}
#define uses_s32_cl0_s32
#endif /* uses_s32_cl0_s32 */

#if !defined(uses_s32_get_lsb_s32)
/** Counts the trailing zeros starting from the LSB.*/
static inline int32_t s32_get_lsb_s32(int32_t x)
{
   int32_t num = 0;
   while(0 == (x & 0x1))
   {
      x = x >> 1;
      num++;
   }
   return num;
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


#if !defined(uses_s32_shl_s32_sat)
/**
 * Airthmetic left shift
 */
static inline int32_t s32_shl_s32_sat(int32_t x, int16_t shift)
{
   long y = ( ((long)x) << shift);

   // after shifting sign bit changes, then saturate.
   if ((x < 0) && !(y & 0x8000000))
   {
      return INT32_MIN;
   }
   if ((x > 0) && (y & 0x8000000))
   {
      return INT32_MAX;
   }

   return y;
}
#define uses_s32_shl_s32_sat
#endif /* uses_s32_shl_s32_sat */

#if !defined(uses_s32_shr_s32_sat)
/**
 * Airthmetic right shift
 */
static inline int32_t s32_shr_s32_sat(int32_t x, int16_t shift)
{
   long y = ( ((long)x) >> shift);

   // after shifting sign bit changes, then saturate.
   if ((x < 0) && !(y & 0x8000000))
   {
      return INT32_MIN;
   }
   if ((x > 0) && (y & 0x8000000))
   {
      return INT32_MAX;
   }

   return y;
}
#define uses_s32_shr_s32_sat
#endif /* uses_s32_shr_s32_sat */

#if !defined(uses_s24_cl0_s24)
/** Counts the leading zeros starting from the MSB. 24 bit*/
static inline int32_t s24_cl0_s24(int32_t x)
{
    int count = 0;
    int i;

    for ( i = 23 ; i >= 0 ; i-- ) {
        if ((x & ( 0x01 << i )) == 0) {
            count++;
        }
        else {
            break;
        }
    }
    return count;
}
#define uses_s24_cl0_s24
#endif /* uses_s24_cl0_s24 */

#endif /* #ifndef POSAL_TYPES_H */

