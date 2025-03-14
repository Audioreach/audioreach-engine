/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*****************************************************************************
 * FILE NAME:   audpp_filterDesign.h              TYPE: C-header file
 * DESCRIPTION: Header file for filter design, adapted from QSound code.
 *****************************************************************************/
#ifndef _AUDPP_FILTERDESIGN_H_
#define _AUDPP_FILTERDESIGN_H_
#include "audpp_util.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*=============================================================================
      Function Declarations 
=============================================================================*/
int32_t Q23_cosine_norm_freq
(
    int32_t       freqHz,                     /* input frequency in Hz         */
    int32_t       sampleRate                  /* sampling rate                 */
);

/* Design functions.  1st order filters need three coefficients, biquads     */
/* need five.  Some filters have zeros which improve signal rejection        */
/* (e.g. a low-pass filter with zeros will be better at filtering out high   */
/* frequencies than one that doesn't) but run a little more slowly.          */

void designFirstOrderLowpassCoeffs
(
    int32_t       coeffsL32Q23[3],            /* coeffs in L32Q23              */
    int16_t       mB,                         /* attenuation at cutoff freq    */
    int32_t       freqHz,                     /* cutoff frequency              */
    int32_t       sampleRate,                 /* sampling rate                 */
    bool_t       withZero                    /* with a zero or not at fs/2    */
);

void designFirstOrderHighpassCoeffs
(
    int32_t       coeffsL32Q23[3],            /* coeffs in L32Q23              */
    int16_t       mB,                         /* attenuation at cutoff freq    */
    int32_t       freqHz,                     /* cutoff frequency              */
    int32_t       sampleRate,                 /* sampling rate                 */
    bool_t       withZero                    /* with a zero or not at fs/2    */
);

/* QSound version */
void DesignFirstOrderCoeffs_Zero
(
    int32_t coeffs[3],                        /* coeffs in L32Q23              */
    int32_t dc_mb,                            /* dc gain in mB                 */  
    int32_t nyquist_mb,                       /* gain at fs/2 in mB            */
    int32_t mb,                               /* gain at the following freq    */  
    int32_t freqHz,                           /* freq for the above gain       */
    int32_t sampleRate                        /* sampling rate                 */
);

void designBiquadLowpassCoeffs
(
    int32_t       coeffsL32Q23[5],            /* coeffs in L32Q23              */
    int16_t       mB,                         /* attenuation at cutoff freq    */
    int32_t       freqHz,                     /* cutoff frequency              */
    int32_t       sampleRate,                 /* sampling rate                 */
    bool_t       withZero                    /* with a zero or not at fs/2    */
);

void designBiquadBandpassCoeffs
(
    int32_t       coeffsL32Q23[5],            /* coeffs in L32Q23              */
    int16_t       mB,                         /* attenuation at cutoff freq    */
    int32_t       freqHz,                     /* cutoff frequency              */
    int32_t       sampleRate,                 /* sampling rate                 */
    bool_t       withZero                    /* with a zero or not            */
);

void designBiquadHighpassCoeffs
(
    int32_t       coeffsL32Q23[5],            /* coeffs in L32Q23              */
    int16_t       mB,                         /* attenuation at cutoff freq    */
    int32_t       freqHz,                     /* cutoff frequency              */
    int32_t       sampleRate                  /* sampling rate                 */
);

void cascadeFirstOrderFilters
(
    int32_t coeffsL32Q23[5],                  /* biquad coeffs, L32Q23         */
    int32_t uL32Q23[3],                       /* first-order filter coeffs     */
    int32_t vL32Q23[3]                        /* first-order filter coeffs     */
);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _FILTERDESIGN_H_ */
