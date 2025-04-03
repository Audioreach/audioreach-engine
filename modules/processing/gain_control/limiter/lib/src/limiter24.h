/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*****************************************************************************
 * FILE NAME:   limiter24.h             TYPE: C-header file                  
 * DESCRIPTION: Implements the limiter algorithm.                            
 *****************************************************************************/

#ifndef _LIMITER_H
#define _LIMITER_H

#if ((defined __hexagon__) || (defined __qdsp6__))
#define LIM_ASM
#endif
/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/

#include "posal.h"

/*----------------------------------------------------------------------------
 * Preprocessor Definitions and Constants
 * -------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 * Type Declarations
 * -------------------------------------------------------------------------*/

/**
@brief Hardcoded constants used in the Limiter processing
*/
typedef enum
{		
	LIM_MGAIN_UNITY           = 0x0100,   // Q7.8 (1) Unity gain value for make up gain
	LIM_GAIN_UNITY            = 0x7FFF,   // Q15 (1) Unity gain value for limiter gain
	LIM_FADE_GRC              = 0x6000    // Q15 (0.7) Gain release constant for transitions
} LimConstants;

/**
@brief Set 32bit buffer to zero 
*/
void buffer32_empty
(
    int32           *buf,               /* buffer to be processed            */
    uint32           samples            /* number of samples in this buffer  */
);

/**
@brief Copy one 32bit buffer to another 
*/
void buffer32_copy
(
    int32           *destBuf,           /* output buffer                     */
    int32           *srcBuf,            /* input buffer                      */
    uint32           samples            /* number of samples to process      */
);


/**
@brief Perform limiter processing with delay.             

@param pCfg: [in] Pointer to limiter configuration structure
@param pData: [in,out] Pointer to limiter data structure 
@param pIn : [in] Pointer to input data , dummy not used in func
@param pOut : [in,out] Pointer to output data , dummy not used 
 			in func
@param iSubFrameSize: [in] Frame size for current data buffer
*/

void lim_proc_delay ( LimCfgType *pCfg,
                      LimDataType *pData, int32 *in,void *out,
                      int16 iSubFrameSize );
					  
/**
@brief Perform limiter processing for the
delay-less implementation.

@param pCfg: [in] Pointer to limiter configuration structure
@param pData: [in,out] Pointer to limiter data structure 
@param pIn : [in] Pointer to input data , dummy not used in func
@param pOut : [in,out] Pointer to output data , dummy not used 
 			in func
@param iSubFrameSize: [in] Frame size for current data buffer
*/

void lim_proc_delayless ( LimCfgType *pCfg,
                         LimDataType *pData, int32 *in, void *out,
                      int16 iSubFrameSize );

/**
@brief Pass data buffer without limiter processing
but with necessary delay.

@param pCfg: [in] Pointer to limiter configuration structure
@param pData: [in,out] Pointer to limiter data structure
@param iSubFrameSize: [in] Frame size for current data buffer
*/

void lim_pass_data(LimCfgType *pCfg,
                     LimDataType *pData,
                     int16 iSubFrameSize);


#endif /* #ifndef _LIMITER_H */
