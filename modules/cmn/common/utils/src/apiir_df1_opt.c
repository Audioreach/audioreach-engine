/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*****************************************************************************]
[* FILE NAME:   audio_apiir_df1_opt.h                TYPE: C-header file           *]

[* DESCRIPTION: Contains global variables and fn declarations for equalizer. *]
[*****************************************************************************/



/* This function calculates the response of a two-pole allpass IIR filter	*/

/* that is implemented in DF1 form											*/

/* numcoefs->b(0), only 2 coefs are stored									*/



#include "audio_apiir_df1_opt.h"

#include "audio_basic_op.h"

#include "ar_defs.h"

/* To use the following function, it should be ensured that the Q-factor of den and */

/* num coefs should be <= Q27, in order to avoid overflow                           */

/* The suggested Q-factors are:                                                     */

/* Q1.27, Q2.26, Q3.25, Q4.24...                                                    */


#ifndef IIR_DF1_32_AP_ASM
void iirDF1_32_ap(int32_t *inp,

	int32_t *out,

	int32_t samples,

	int32_t *numcoefs,

	int32_t *mem,

	int16_t qfactor)

{

	int64_t y, ff, fb;

	int32_t yScaled; // yScaled will be the scaled version of y used while calculating w1 and w2



	int64_t b0TimesX, b1TimesW1, b2TimesW2;

	int64_t a1TimesW3, a2TimesW4;

	int32_t w1Temp, w3Temp, w4Temp;

	int64_t w2Temp;



	int32_t b0, b1;

	int32_t i, p5;



	if (qfactor >= 1){

		p5 = (int32_t)(1 << (qfactor - 1));

	} else

	{

		p5 = 0;

	}



	b0 = *numcoefs;

	b1 = *(numcoefs + 1);



	   /* repeat the loop for every sample*/

       /* in allpass biquad, a1 = b1, a2 = b0, b2 = 1 */

       /* equations are ff= b0*x+b1*w1+b2*w2  */ // use yScaled

       /*          i.e. ff= b0*x+b1*w1+w2     */

       /*                         w2=w1                        */

       /*                         w1=x                         */

       /*               fb= a1*w3+a2*w4       */ // use yScaled

       /*          i.e. fb= b1*w3+b0*w4       */

       /*                         w4=w3                        */

       /*                         y = ff - fb        */

       /*                         w3=y                         */





	for (i = 0; i < samples; ++i)

	{

		/******************************* calculate ff ********************************/

		b0TimesX = s64_mult_s32_s32(b0, *inp);                                  // Q(27 + qfactor)



		w1Temp = *mem++; // after this, mem points to w2                        // Q27

		b1TimesW1 = s64_mult_s32_s32(b1, w1Temp);                               // Q(27 + qfactor)



		ff = s64_add_s64_s64(b0TimesX, b1TimesW1);								// Q(27 + qfactor)



		w2Temp = (int64_t)*mem; // after this, mem points to w2					// Q27

		b2TimesW2 = s64_shl_s64(w2Temp, qfactor);                               // Q(27 + qfactor)



		ff = s64_add_s64_s64(ff, b2TimesW2);									// Q(27 + qfactor)



		*mem-- = w1Temp; // after this, mem points to w1 						// w2 = w1, Q27

		*mem = *inp++;														    // w1 = x

		// inp incremented to next input location in the above line

		mem += 2; // after this, mem points to w3



		/******************************* calculate fb *******************************/

		w3Temp = *mem++; // after this, mem points to w4                        // Q27

		a1TimesW3 = s64_mult_s32_s32(b1, w3Temp);                               // Q(27 + qfactor)



		w4Temp = *mem; // after this, mem points to w4                          // Q27

		a2TimesW4 = s64_mult_s32_s32(b0, w4Temp);                               // Q(27 + qfactor)



		fb = s64_add_s64_s64(a1TimesW3, a2TimesW4);								// Q(27 + qfactor)



		*mem-- = w3Temp; // after this, mem points to w3						// w4 = w3



		/******************************* calculate y *******************************/

		y = s64_sub_s64_s64(ff, fb);											// Q(27 + qfactor)	

		yScaled = s32_saturate_s64(s64_shl_s64(s64_add_s64_s32(y, p5), -qfactor));	// Q27



		*mem = yScaled;															// Q27

		mem -= 2; // after this, mem points to w1



		/***************************** store the output ******************************/

		*out++ = yScaled;                                                       // Q27

		/* Performing this here in order to support in-place computation */

	}

}
#endif
