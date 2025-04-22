/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
/*===========================================================================
 * FILE NAME: apply_gain.c
 * DESCRIPTION:
 *    Apply gain (Q12) to the input buffer
 *===========================================================================*/

#ifdef AVS_BUILD_SOS
#include "capi_cmn.h"
#endif
#include "apply_gain.h"
#include "ar_msg.h"
/*=============================================================================
FUNCTION      void audpp_volume_apply_gain_16

DESCRIPTION   apply gain (Q12) to the input buffer (Q15), shift  and place the result in the
              output buffer (Q15). Optimized for Q6 by loading 2 values of input at the same time.
OUTPUTS

DEPENDENCIES  None

RETURN VALUE  None

SIDE EFFECTS
===============================================================================*/

void audpp_volume_apply_gain_16(int16* outptr,   /* Pointer to output */
				int16* inptr,    /* Pointer to input */
				uint16 gain,     /* Gain in Q12 format */
				uint32 samples)   /* No of samples to which the gain is to be applied */
{
#if ((defined __hexagon__) || (defined __qdsp6__))
	int32 nGainQ15 = (Q6_R_zxth_R(gain)) << 3;   /* scale the gain up by 3 bit to become Q15 for ease of truncation*/
	int64 nCombinedGain = Q6_P_combine_RR(nGainQ15, nGainQ15); /* combine int32 into int64 for vector operation*/

	int64 nInputCombinedP1,nInputCombinedP2,nInputCombinedP3,nInputCombinedP4;
	int64 temp1,temp2,temp3,temp4;

	// if output buffer doesn't start with 8 byte (or 4 half-word) boundary, it must start with 2 byte boundary
	while ((0 != (((uint32)outptr) & 0x6)) && (samples))
	{
		// when output buffer address doesn't end with 000, such as in 0b...xy0, x|y=1
		*outptr++ = s16_saturate_s32(s32_mult_s32_s16_rnd_sat(nGainQ15, *inptr++));
		samples--;
	}

	// The rest will be computed with two vector mults in a row, ie, 2 samples a packet, 4 samples per iteration
	int64 *pALGN8OutPtr = (int64 *)outptr;
	while (samples >=8 )
	{
		// apply gain to left channel
		nInputCombinedP1 = Q6_P_combine_RR(inptr[1],inptr[0]);
		nInputCombinedP2 = Q6_P_combine_RR(inptr[3],inptr[2]);
		nInputCombinedP3 = Q6_P_combine_RR(inptr[5],inptr[4]);
		nInputCombinedP4 = Q6_P_combine_RR(inptr[7],inptr[6]);
		samples-=8;
		inptr+= 8;

		temp1 = Q6_P_vmpyweh_PP_s1_rnd_sat(nCombinedGain, nInputCombinedP1);
		temp2 = Q6_P_vmpyweh_PP_s1_rnd_sat(nCombinedGain, nInputCombinedP2);
		temp3 = Q6_P_vmpyweh_PP_s1_rnd_sat(nCombinedGain, nInputCombinedP3);
		temp4 = Q6_P_vmpyweh_PP_s1_rnd_sat(nCombinedGain, nInputCombinedP4);

		*pALGN8OutPtr++ = Q6_P_combine_RR(Q6_R_vsatwh_P(temp2),Q6_R_vsatwh_P(temp1));
		*pALGN8OutPtr++ = Q6_P_combine_RR(Q6_R_vsatwh_P(temp4),Q6_R_vsatwh_P(temp3));
	}
	outptr =  (int16 *)pALGN8OutPtr;
	// if there are less than 4 samples left,
	while (samples--)
	{
		*outptr++ = s16_saturate_s32(s32_mult_s32_s16_rnd_sat(nGainQ15, *inptr++));
	}
#else
	//AR_MSG(DBG_HIGH_PRIO, "apply gain level 2");
    /*--------------------------------------- Unoptimized non q6 vesion-----------------------------------------------*/
	int32 nGainQ15 = ((uint32)(gain)) << 3;   /* scale the gain up by 3 bit to become Q15 for ease of truncation*/

	//Replaced while(samples--) with for loop because samples is unsigned integer
	//Sanitizer throws unsigned overflow error during build on ARM
	for (int i = 0; i < samples; i++)
	{
		*outptr++ = s16_saturate_s32(s32_mult_s32_s16_rnd_sat(nGainQ15, *inptr++));
	}

#endif
}

/*=============================================================================
FUNCTION      void audpp_volume_apply_gain_32_G1

DESCRIPTION   apply gain (Q12) to the input buffer (Q31), shift  and place the result in the
              output buffer (Q31).
OUTPUTS

DEPENDENCIES  For Gain value greater than 1.

RETURN VALUE  None

SIDE EFFECTS
===============================================================================*/

void audpp_volume_apply_gain_32_G1(int32* outptr,   /* Pointer to output */
				int32* inptr,    /* Pointer to input */
				uint16 gain,     /* Gain in Q12 format */
				uint32 samples)   /* No of samples to which the gain is to be applied */
{

#if ((defined __hexagon__) || (defined __qdsp6__))
	/*--------------------------------------- q6 vesion-----------------------------------------------*/

	int32 nGainQ15 = (Q6_R_zxth_R(gain)) << 3;   /* scale the gain up by 3 bit to become Q15 for ease of truncation*/
	int64 temp1,temp2;

	temp1 = Q6_P_asrrnd_PI(Q6_P_mpy_RR(*inptr++, nGainQ15),15);

	// if output buffer doesn't start with 8 byte (or 4 half-word) boundary, it must start with 4 byte boundary
	while ((0 != (((uint32)outptr) & 0x4)) && (samples))
	{
		// when output buffer address doesn't end with 000
		*outptr++ = Q6_R_sat_P(temp1);
		samples--;
		if(samples)
		{
			temp1 = Q6_P_asrrnd_PI(Q6_P_mpy_RR(*inptr++, nGainQ15),15);
		}
	}

	if(0==samples)
	   return;

	int64 *pALGN8OutPtr = (int64 *)outptr;
	temp2  = 0;
	if(samples>1)
	{
		temp2 = Q6_P_asrrnd_PI(Q6_P_mpy_RR(*inptr++, nGainQ15),15);
	}
	else
	{
		*outptr++ = Q6_R_sat_P(temp1);
		return;
	}

    while (samples>3)
    {
    	*pALGN8OutPtr++ = Q6_P_combine_RR(Q6_R_sat_P(temp2),Q6_R_sat_P(temp1));
    	temp1 = Q6_P_asrrnd_PI(Q6_P_mpy_RR(*inptr++, nGainQ15),15);
    	temp2 = Q6_P_asrrnd_PI(Q6_P_mpy_RR(*inptr++, nGainQ15),15);
    	samples-=2;
    }
    *pALGN8OutPtr++ = Q6_P_combine_RR(Q6_R_sat_P(temp2),Q6_R_sat_P(temp1));
	samples-=2;
    outptr =  (int32 *)pALGN8OutPtr;

    if(samples)
	{
		temp1 = Q6_P_asrrnd_PI(Q6_P_mpy_RR(*inptr++, nGainQ15),15);
		*outptr++ = Q6_R_sat_P(temp1);
	}

#else
	int32 nGainQ15 = ((uint32)gain) << 3;   /* scale the gain up by 3 bit to become Q15 for ease of truncation*/

    /*--------------------------------------- Non q6 vesion-----------------------------------------------*/

	//Replaced while(samples--) with for loop because samples is unsigned integer
	//Sanitizer throws unsigned overflow error during build on ARM
	for (int i = 0; i < samples; i++)
    {
		*outptr++ = s32_saturate_s64((s64_mult_s32_s32(*inptr++, nGainQ15) + 0x4000) >> 15);
	}
#endif
}

/*=============================================================================
FUNCTION      void audpp_volume_apply_gain_32_L1

DESCRIPTION   apply gain (Q12) to the input buffer (Q31), shift  and place the result in the
              output buffer (Q31).
OUTPUTS

DEPENDENCIES  For Gain value less than 1.

RETURN VALUE  None

SIDE EFFECTS
===============================================================================*/
void audpp_volume_apply_gain_32_L1(int32* outptr,   /* Pointer to output */
		                int32* inptr,    /* Pointer to input */
		                uint16 gain,     /* Gain in Q12 format */
		                uint32 samples)   /* No of samples to which the gain is to be applied */
{

    int16 nGainQ15 = gain << 3;   /* scale the gain up by 3 bit to become Q15 for ease of truncation*/

#if ((defined __hexagon__) || (defined __qdsp6__))
	/*--------------------------------------- optimized q6 vesion-----------------------------------------------*/

	int64 nCombinedGain = Q6_P_combine_RR(nGainQ15,nGainQ15);
	int64 nInputCombinedP1,nInputCombinedP2;

	// if output buffer doesn't start with 8 byte (or 4 half-word) boundary, it must start with 4 byte boundary
	while ((0 != (((uint32)outptr) & 0x4)) && (samples))
	{
	    // when output buffer address doesn't end with 000
	    *outptr++ = s32_mult_s32_s16_rnd_sat(*inptr++, nGainQ15);
	    samples--;
	}

	// The rest will be computed with two vector mults in a row, ie, 2 samples a packet, 4 samples per iteration
	int64 *pALGN8OutPtr = (int64 *)outptr;
	while (samples >=4 )
	{
	    // apply gain to left channel
	    nInputCombinedP1 = Q6_P_combine_RR(inptr[1],inptr[0]);
	    nInputCombinedP2 = Q6_P_combine_RR(inptr[3],inptr[2]);

	    samples-=4;
	    inptr+= 4;

	    *pALGN8OutPtr++ = Q6_P_vmpyweh_PP_s1_rnd_sat(nInputCombinedP1,nCombinedGain);
	    *pALGN8OutPtr++ = Q6_P_vmpyweh_PP_s1_rnd_sat(nInputCombinedP2,nCombinedGain);
	}
	outptr =  (int32 *)pALGN8OutPtr;
	// if there are less than 4 samples left,
	while (samples--)
	{
	    *outptr++ = s32_mult_s32_s16_rnd_sat(*inptr++, nGainQ15);
	}
#else
	/*--------------------------------------- Unoptimized non q6 vesion-----------------------------------------------*/

	//Replaced while(samples--) with for loop because samples is unsigned integer
	//Sanitizer throws unsigned overflow error during build on ARM
	for (int i = 0; i < samples; i++)
	{
		*outptr++ = s32_mult_s32_s16_rnd_sat(*inptr++, nGainQ15);
	}
#endif
}


