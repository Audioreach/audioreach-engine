/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef _AUDIO_IIRDF1_H_

#define _AUDIO_IIRDF1_H_



#include "ar_defs.h"



/*=============================================================================

      Function Declarations 

=============================================================================*/



#ifdef __cplusplus

extern "C" {

#endif /* __cplusplus */



/* To use the following function, it should be ensured that the Q-factor of den and */

/* num coefs should be <= Q27, in order to avoid overflow                           */

/* The suggested Q-factors are:                                                     */

/* Q1.27, Q2.26, Q3.25, Q4.24...                                                    */



/* This is the df1 allpass biquad iir function										*/

/* Only b0 and b1 is needed, using transfer function:								*/

/* H_ap(z) = (b0 + b1*z^-1 + z^-2)/(1 + b1*z^-1 + b0*z^-2)							*/

void iirDF1_32_ap(int32_t *inp,

	int32_t *out,

	int32_t samples,

	int32_t *numcoefs,

	int32_t *mem,

	int16_t qfactor);





#ifdef __cplusplus

}

#endif /* __cplusplus */



#endif  /* _AUDIO_IIRDF1_H_ */

