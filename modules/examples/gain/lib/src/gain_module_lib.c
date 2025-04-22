/**
 * \file gain_module_lib.c
 *  
 * \brief
 *  
 *     Example Gain Module
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */


#include "gain_module_lib.h"
/*=============================================================================
FUNCTION      void example_apply_gain_16

DESCRIPTION   apply gain (Q12) to the input buffer (Q15), shift  and place the result in the
              output buffer (Q15).
OUTPUTS

DEPENDENCIES  None

RETURN VALUE  None

SIDE EFFECTS
===============================================================================*/

void example_apply_gain_16(int16_t *outptr,  /* Pointer to output */
                   int16_t *inptr,   /* Pointer to input */
                   uint16_t gain,    /* Gain in Q12 format */
                   uint32_t samples) /* No of samples to which the gain is to be applied */
{
   int32_t nGainQ15 = ((uint32_t)(gain)) << 3; /* scale the gain up by 3 bit to become Q15 for ease of truncation*/
   while (samples--)
   {
      /* Multiply 32 bit number with 16 bit number*/
      int32_t mul_result = (int32_t)(nGainQ15 * ((int32_t)(*inptr++)));

      /* Adjust q factor after multiplication */
      mul_result = (mul_result + 0x4000) >> 15;

      /* Saturate to 16 bits*/
      if (mul_result > MAX_16) // 0x7FFF
      {
         *outptr++ = MAX_16;
      }
      else if (mul_result < MIN_16) //-32768
      {
         *outptr++ = MIN_16;
      }
      else
      {
         *outptr++ = (int16_t)mul_result;
      }
   }
}

/*=============================================================================
FUNCTION      void example_apply_gain_32_G1

DESCRIPTION   apply gain (Q12) to the input buffer (Q31), shift  and place the result in the
              output buffer (Q31).
OUTPUTS

DEPENDENCIES  For Gain value greater than 1.

RETURN VALUE  None

SIDE EFFECTS
===============================================================================*/

void example_apply_gain_32_G1(int32_t *outptr,  /* Pointer to output */
                      int32_t *inptr,   /* Pointer to input */
                      uint16_t gain,    /* Gain in Q12 format */
                      uint32_t samples) /* No of samples to which the gain is to be applied */
{
   int32_t nGainQ15 = ((uint32_t)gain) << 3; /* scale the gain up by 3 bit to become Q15 for ease of truncation*/

   /*--------------------------------------- Non q6 vesion-----------------------------------------------*/
   while (samples--)
   {

      /* Multiply 32 bit number with 32 bit number*/
      int64 mul_result = (int64)(nGainQ15 * (*inptr++));

      /* round and adjust q factor*/
      mul_result = (mul_result + 0x4000) >> 15;

      /* Saturate to 32 bits*/
      if (mul_result > MAX_32) // 0x7FFFFFFFL
      {
         *outptr++ = MAX_32;
      }
      else if (mul_result < MIN_32) // 0x80000000L
      {
         *outptr++ = MIN_32;
      }
      else
      {
         *outptr++ = (int32_t)mul_result;
      }
   }
}

/*=============================================================================
FUNCTION      void example_apply_gain_32_L1

DESCRIPTION   apply gain (Q12) to the input buffer (Q31), shift  and place the result in the
              output buffer (Q31).
OUTPUTS

DEPENDENCIES  For Gain value less than 1.

RETURN VALUE  None

SIDE EFFECTS
===============================================================================*/
void example_apply_gain_32_L1(int32_t *outptr,  /* Pointer to output */
                      int32_t *inptr,   /* Pointer to input */
                      uint16_t gain,    /* Gain in Q12 format */
                      uint32_t samples) /* No of samples to which the gain is to be applied */
{

   int16_t nGainQ15 = gain << 3; /* scale the gain up by 3 bit to become Q15 for ease of truncation*/
   while (samples--)
   {
      /* Multiply 32 bit number with 32 bit number*/
      int64 mul_result = (int32_t)(nGainQ15 * (*inptr++));

      /* round and adjust q factor*/
      mul_result = (mul_result + 0x4000) >> 15;

      *outptr++  = (int32_t)mul_result;
   }
}
