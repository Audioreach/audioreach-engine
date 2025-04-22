#include "multi_def_resampler.h"

/*=========================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
========================================================================= */

#ifndef     _RESAMPLER_32B_EXT_H_
#define     _RESAMPLER_32B_EXT_H_

#ifdef __cplusplus
extern "C" {
#endif
/*===========================================================================*
 *           INCLUDE HEADER FILES                                            *
 *===========================================================================*/
#include "audio_basic_op.h"

/*===========================================================================*
 *           Constants.                                                      *
 *===========================================================================*/

/* For windows SPF builds we use the original resampler and
 * not the qdsp or xtensa optimised versions */
#if !((defined __hexagon__) || (defined __qdsp6__)) && !defined(__XTENSA__)
#define RESAMPLER_ORIGINAL
#endif

#if defined(__XTENSA__)
#define RESAMPLER_C_OPT
#endif

#ifndef RESAMPLER_ORIGINAL
	#ifndef RESAMPLER_C_OPT
		#define RESAMPLER_ASM_OPT
	#endif
#endif

#ifdef RESAMPLER_ORIGINAL
	#undef RESAMPLER_C_OPT
#endif


/* Comprehensive analysis report covers the conversion cases from 8 to 384 kHz.
 * It is safe to claim that the maximum supported frequency and down-conversion ratio
 * are 384 kHz and 48, respectively. To claim higher frequency or conversion ratio,
 * the comprehensive analysis must be conducted before claiming
 */
#define     MAX_FREQ            1536000
#define     MAX_DOWN_RATIO      192
#define     ASYNC_RANGE         20
#define     MASK_QUALITY        0xF
#define     MASK_INTERP         0x1
#define     MASK_ASYNC          0x1
#define     MASK_CHANNEL        0xF
#define     MASK_INPUT          0x1
#define     MASK_BITS           0x1
#define     MASK_COEF           0x2
#define     MASK_DYNAMIC        0x1
#define     MASK_DELAY          0x1
#define     MASK_FILT_SEL       0x1
#define     MASK_DATA_QFACT     0x1
#define     SHIFT_QUALITY       6
#define     SHIFT_INTERP        10
#define     SHIFT_ASYNC         11
#define     SHIFT_CHANNEL       2
#define     SHIFT_INPUT         1
#define     SHIFT_BITS          0
#define     SHIFT_DYNAMIC       12
#define     SHIFT_DELAY         13
#define     SHIFT_FILTER_SELECT 14
#define     SHIFT_DATA_Q_FACTOR 15


typedef enum
{
    MODE,               /* SRC mode word                                 */
    IN_FREQ,            /* input frequency                               */
    OUT_FREQ,           /* output frequency                              */
    RS_OSR,             /* up-sampler OSR                                */
    RS_FLEN,            /* up-sampler filter length                      */
    RS_MEM_LEN,         /* up-sampler memory length                      */
    RS_QFAC,            /* up-sampler filter coefficient q-factor        */
    RS_SYMMETRIC,       /* up-sampler filter symmetry indicator          */
    MODE_COEF_BIT,      /* coefficient bit width indicator, 16 bit input */
                        /*   0 -- 16 bit filter coefficients             */
                        /*   1 -- 32 bit filter coefficients             */
    MODE_INTERP,        /* interpolation mode indicator                  */
                        /*   0 -- linear interpolation                   */
                        /*   1 -- quadratic interpolation                */
    MODE_ASYNC,         /* asynchronous indicator                        */
    NUM_CHAN,           /* number of channels                            */
	MODE_DYNAMIC,       /* dynamic resampler indicator                   */
	MODE_DYNAMIC_DELAY, /* delay mode for dyanmic resampler              */
	                    /* 0 -- low delay with visible transitional phase distortion */
						/* 1 -- high delay with smooth transition                    */
	MODE_FILT_SEL_FLAG,
                        /* 0 -- disable new filter selection */
                        /* 1 -- enable new filter selection  */
	DATA_Q27_FLAG,
                        /* 0 -- not Q27 for 32bit data */
                        /* 1 -- Q27 only when data width is 32 bits */


} PARAM;

/*===========================================================================*
 *           FUNCTION DECLARATIONS FOR BOTH UP-SAMPLER AND DOWN-SAMPLER      *
 *===========================================================================*/
void  resamp_size(int32 *par, int16 *pRsSize, int16 *pRsMemSize);
void* select_filter(int32 minFreq, int32 maxFreq, int32 async, int32* param);
int16 dynamic_resamp_output_latency(void *pRS);
int16 dynamic_resamp_input_latency(void *pRS);
int32 resamp_fixedin(void*, void**, void**, int32);
int32 resamp_fixedout(void*, void**, void**, int32);
int32 resamp_calc_fixedin(void*, int32);
int32 resamp_calc_fixedout(void*, int32);
void  resamp_overshoot_protection_q27(void**, int32, int32);

uint32 resamp_memreq_opt_coef_table(void*);
int32 resamp_gen_opt_coef_table(void*,void*,uint32);

/*===========================================================================*
 *           FUNCTION DECLARATIONS FOR UP-SAMPLER                            *
 *===========================================================================*/
int32 init_upsamp(void*, int32*, void*, void*, void**);

/*===========================================================================*
 *           FUNCTION DECLARATIONS FOR DOWN_SAMPLER                          *
 *===========================================================================*/
int32 init_dnsamp(void*, int32*, void*, void*, void**);

/*===========================================================================*
 *           FUNCTION DECLARATIONS FOR DYNAMIC_RESAMPLER                     *
 *===========================================================================*/
int32 init_dynamic_resamp(void*, int32*, void*, void*, void**);
int32 reinit_dynamic_resamp(void* pStruct, int32 inFreq);

#ifdef __cplusplus
}
#endif

#endif  /* _RESAMPLER_32B_EXT_H_ */
