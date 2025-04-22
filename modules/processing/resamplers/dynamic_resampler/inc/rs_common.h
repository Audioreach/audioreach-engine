#include "multi_def_resampler.h"

/*=========================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
========================================================================= */

#ifndef  _RS_COMMON_H_
#define  _RS_COMMON_H_

#include "audio_basic_op.h"

/*===========================================================================*
 *           TYPE DEFINITIONS                                                *
 *===========================================================================*/


/*===========================================================================*
 *           constants                                                       *
 *===========================================================================*/
#define     Q15_SHIFT           15
#define     Q20_SHIFT           20
#define     Q23_SHIFT           23
#define     MAX_Q15             32767
#define     MIN_Q15             -32768


/*===========================================================================*
 *           EXTERNAL VARIABLES                                              *
 *===========================================================================*/
extern const int16 oneQ15, halfQ15;
extern const int32 oneQ23, halfQ23;

/*===========================================================================*
 *           STRUCTURES                                                      *
 *===========================================================================*/
typedef struct
{
    int64   totalPhStepCoef;/* Phase update step size for filter coefficient*/
    int32   fracPhMask;
    int32   fracPhStepCoef; /* Phase update step size for filter coefficient*/
    int32   inFreq;         /* input sampling rate                          */
    int32   outFreq;        /* output sampling rate                         */
    int32   maxFreq;
    int32   totalPhCap;     /* cap of total phase                           */
    int32   totalPhStep;    /* phase update step size for starting point    */
    int32   curTotalPh;     /* total phase for starting point               */
    int32   fracPhStep;
    int32   curFracPh;      /* fractional phase for starting point, no init */
    int32   invMaxFreq;     /* inverse of input frequency                   */
    int16   intPhStepCoef;  /* Phase update step size for filter coefficient*/
    int16   sampStep;       /* phase update step size for starting point    */
    int16   intPhStep;
    int16   curIntPh;       /* integer phase for starting point, no init    */
    int16   invMaxFreqShift;/* shifting factor of invInFreq                 */
    int16   totalIntPh;     /* filter over sampling ratio                   */
    int16   convIndex;      /* filter index for convolution                 */
    int16   loadIndex;      /* filter index for loading filter memory       */
    int16   async;          /* Asynchronous mode indicator                  */
    int16   intRatio;       /* Integer resampling ratio indicator           */
}   RSPhase;

typedef struct
{
    void    *pMem;          /* point to filter memory                       */
    void    *pCoef;         /* point to filter coefficient                  */
    void    *pCoef2;
    void    *pGenCoef;      /* pointer to generated filter coefficients     */
    int32   halfQfac;       /* one half for a given Q factor                */
    int32   coefLen;        /* length of filter coefficient                 */
    int32   coefLen2;       /* half length of filter coefficient if filter  */
                            /* coefficients is symmetric.                   */
    int32   coefLen3;       /* length of filter coefficient                 */
    int32   coefLen4;       /* half length of filter coefficient if filter  */
                            /* coefficients is symmetric.                   */
    int16   coefStep;       /* convolution step size of filter coefficient  */
    int16   coefStep2;
    int16   coefStart;
    int16   qFac;           /* Qfactor of the filter coefficients           */
    int16   memLen;         /* length of filter memory                      */
    int16   memStep;        /* convolution step size of filter memmory      */
}   Filters;

/*===========================================================================*
 *           FUNCTION DECLARATIONS                                           *
 *===========================================================================*/
int16 int_div_16(int16 num16, int16 den16, int16* rem16);
int32 int_div_32(int32 num32, int32 den32, int32* rem32);
int64 int_div_64(int64 num64, int64 den64, int64* rem64);
int32 frac_div_32(int32 num, int32 den, int16 Qfac);
int32 fxp_inv_32(int32 num, int16 qFac, int16 *shift);

#endif /* _RS_COMMON_H_ */
/*===========================================================================* 
 * End of rs_common.h                                                        * 
 *===========================================================================*/
