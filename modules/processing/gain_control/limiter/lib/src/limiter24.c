/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */                              
/*****************************************************************************
* FILE NAME:   limiter24.c             TYPE: C-source file                  
* DESCRIPTION: Implements the limiter algorithm.                            
*****************************************************************************/

/*----------------------------------------------------------------------------
* Include Files
* -------------------------------------------------------------------------*/

#include "limiter24_api.h"
#include "limiter24.h"
#include "audio_basic_op.h"
#include "audio_basic_op_ext.h"
#include "audio_divide_qx.h"
#include <stringl.h>

#ifdef LIM_ASM
/*Functions defined in ASM */
void lim_proc_delay_asm_16(LimCfgType *pCfg,
						   LimDataType *pData, int32 *in, void *out,
						   int16 iSubFrameSize);
void lim_proc_delay_asm_32(LimCfgType *pCfg,
						   LimDataType *pData, int32 *in, void *out,
						   int16 iSubFrameSize);

void lim_proc_delayless_asm_16(LimCfgType *pCfg,
							   LimDataType *pData, int32 *in, void *out,
							   int16 iSubFrameSize);
void lim_proc_delayless_asm_32(LimCfgType *pCfg,
							   LimDataType *pData, int32 *in, void *out,
							   int16 iSubFrameSize);
#endif

static unsigned int align_to_8_byte(const unsigned int num)
{
   return ((num + 7) & (0xFFFFFFF8));
}

static void apply_makeup_gain16(LimCfgType *pCfg, LimDataType *pData,
								void** pOutFrame,int16 iSubFrameSize)
{
	int j;
	int64 accu64;
	/****************************************************************************
	Implementation of Make up gain on the limiter output
	// out16 = (out32 * gain) << (7-gainexp);
	//       = out32 * ( gain << (7-gainexp) );
	*****************************************************************************/

	int16 *pOutFrameL16 = (int16*)(*pOutFrame);
	if (LIM_MGAIN_UNITY == pCfg->params.limMakeUpGain)
	{
		for(j=0; j<iSubFrameSize; j++)
		{
			pOutFrameL16[j] = (int16) pData->ptempBuffer[j];
		}
	}
	else
	{
	for(j=0; j<iSubFrameSize; j++)
	{
		// Multiply 32bit output with the Q7.16 make-up gain
			accu64 = s64_mult_s32_s32(pData->ptempBuffer[j],pCfg->mGainL32);
		// Copy the audio output from the local buffer
		pOutFrameL16[j] = s16_extract_s64_h_sat(accu64);
	}
	}
	//update output frame data pointers
	pOutFrameL16 += iSubFrameSize;
	*pOutFrame = pOutFrameL16;

}

static void apply_makeup_gain32(LimCfgType *pCfg, LimDataType *pData,
								void** pOutFrame,int16 iSubFrameSize)
{

	int j;
	int64 accu64;
	/****************************************************************************
	Implementation of Make up gain on the limiter output
	// out16 = (out32 * gain) << (7-gainexp);
	//       = out32 * ( gain << (7-gainexp) );
	*****************************************************************************/

	int32 *pOutFrameL32 = (int32*)(*pOutFrame);
	if (LIM_MGAIN_UNITY == pCfg->params.limMakeUpGain)
	{
		memscpy(pOutFrameL32, iSubFrameSize*sizeof(int32),pData->ptempBuffer, iSubFrameSize*sizeof(int32));
	}
	else
	{
	for(j=0; j<iSubFrameSize; j++)
	{
		// Multiply output with the Q7.16 make-up gain
			accu64 = s64_mult_s32_s32(pData->ptempBuffer[j],pCfg->mGainL32);
		// Copy the audio output from the local buffer
		pOutFrameL32[j] = s32_saturate_s64(s64_shl_s64(accu64, -16));
	}
	}

	//update output frame data pointers
	pOutFrameL32 += iSubFrameSize;
	*pOutFrame = pOutFrameL32;

}
/*======================================================================

FUNCTION      Lim_init_default

DESCRIPTION   Initialize 24bit limiter data structure with default
parameters. Sampling rate info and a pointer to static
memory in storage is passed in for configuring the
LIM static configuration structure.

Called once at audio connection set up time.

PARAMETERS    params: [in] Pointer to tuning parameter list

SIDE EFFECTS  None

======================================================================*/

int Lim_init_default(int32 *params,
					 int16 numChannels,
					 int16 bitsPerSample)
{
	int16 i;

	// Initialize tuning parameter with default values
	for (i = 0; i < numChannels; i++)
	{
		params[i*LIM_PAR_TOTAL + LIM_ENABLE]       = LIM_ENABLE_VAL;           // Q0 Limiter enable word
		params[i*LIM_PAR_TOTAL + LIM_MODE]         = LIM_MODE_VAL;             // Q0 Limiter mode word
		if (32 == bitsPerSample)
		{
			params[i*LIM_PAR_TOTAL + LIM_THRESH]       = LIM_THRESH_VAL32;           // Q8.23 Limiter threshold
		}
		else if (16 == bitsPerSample)
		{
			params[i*LIM_PAR_TOTAL + LIM_THRESH]       = LIM_THRESH_VAL16;           // Q8.23 Limiter threshold
		}
		params[i*LIM_PAR_TOTAL + LIM_MAKEUP_GAIN]  = LIM_MAKEUP_GAIN_VAL;      // Q7.8 Limiter make-up gain
		params[i*LIM_PAR_TOTAL + LIM_GC]           = LIM_GC_VAL;               // Q15 Limiter gain recovery coefficient
		params[i*LIM_PAR_TOTAL + LIM_DELAY]        = LIM_DELAY_VAL;            // Q0.15 Limiter delay in seconds
		params[i*LIM_PAR_TOTAL + LIM_MAX_WAIT]     = LIM_MAX_WAIT_VAL;         // Q0.15 Limiter waiting time in seconds
	}

	/////////////////////////////////////////////////////////
	// End module default initialization
	/////////////////////////////////////////////////////////
	return(0);
}

/*======================================================================

FUNCTION      Lim_init

DESCRIPTION   Performs initialization of data structures for the
LIM algorithm. Pointers to configuration and data
structures are passed in for configuration.

Called once at audio connection set up time.

PARAMETERS    pLimCfgStruct: [in] Pointer to configuration structure
pLimDataStruct: [in] Pointer to data structure
params: [in] Pointer to tuning parameter list
samplingRate: [in] Input sampling rate

SIDE EFFECTS  None

======================================================================*/

int Lim_init(CLimiterLib *pLimStruct,
			 int32 *params,
			 int16 chIdx,
			 int32 sampleRate,
			 int32 bitsPerSample)
{
	int32 accu32;
	LimCfgType *pCfg = &pLimStruct->LimCfgStruct;
	LimDataType *pLimDataStruct = &pLimStruct->LimDataStruct;

	pLimStruct->bitsPerSample = bitsPerSample;

	// Initialize limiter configuration structure: LimCfgType

	// Initialize Limiter tuning parameters
	pCfg->params.limEnable      = (int16)params[chIdx*LIM_PAR_TOTAL + LIM_ENABLE];      // Q0 Limiter enable flag
	pCfg->params.limMode        = (int16)params[chIdx*LIM_PAR_TOTAL + LIM_MODE];        // Q0 Limiter mode word
	pCfg->params.limThresh      = params[chIdx*LIM_PAR_TOTAL + LIM_THRESH];             // Limiter threshold:Q3.15 for 16bit; Q8.23 for 32bit
	pCfg->params.limMakeUpGain  = (int16) params[chIdx*LIM_PAR_TOTAL + LIM_MAKEUP_GAIN];// Q7.8 Limiter make-up gain
	pCfg->params.limGc          = (int16)params[chIdx*LIM_PAR_TOTAL + LIM_GC];          // Q15 Limiter gain recovery coefficient
	pCfg->params.limDelay       = (int16)params[chIdx*LIM_PAR_TOTAL + LIM_DELAY];       // Q15 Limiter delay in seconds
	pCfg->params.limMaxWait     = (int16)params[chIdx*LIM_PAR_TOTAL + LIM_MAX_WAIT];    // Q15 Limiter waiting time in seconds

	// Convert the delay and waiting time from Q15 seconds to Q0 samples
	accu32 = s32_mult_s32_s16_rnd_sat(sampleRate, pCfg->params.limDelay );
	pCfg->params.limDelay = s16_saturate_s32(accu32);

	accu32 = s32_mult_s32_s16_rnd_sat(sampleRate, pCfg->params.limMaxWait);
	pCfg->params.limMaxWait = s16_saturate_s32(accu32);

	// Calculate and store delay-minus-one sample in the static memory
	pCfg->limDelayMinusOne = pCfg->params.limDelay-1;

	// Calculate and store max wait-minus-one sample in the static memory
	pCfg->limWaitMinusOne = pCfg->params.limMaxWait-1;

	// Left shift the make up gain by (7+1) to Q7.16 before multiplication with the data
	pCfg->mGainL32 = s32_shl_s32_sat((int32 )pCfg->params.limMakeUpGain, 8);

	pCfg->threshL32Q15 = pCfg->params.limThresh;
	pCfg->limGc = pCfg->params.limGc ;

	if(bitsPerSample == 16)
	{
		/*Assign Function pointers based on delay and bitsPerSample  */
		if( pCfg->params.limDelay > 0)
		{
#ifdef LIM_ASM
			pLimStruct->lim_proc = lim_proc_delay_asm_16;
#else
			pLimStruct->lim_proc = lim_proc_delay;
#endif
		}
		else
		{
#ifdef LIM_ASM
			pLimStruct->lim_proc = lim_proc_delayless_asm_16;

#else
			pLimStruct->lim_proc = lim_proc_delayless;
#endif
		}

		pLimStruct->apply_makeup_gain = apply_makeup_gain16;
	}
	else if (bitsPerSample == 32)
	{

		if( pCfg->params.limDelay > 0)
		{
#ifdef LIM_ASM
			pLimStruct->lim_proc = lim_proc_delay_asm_32;
#else
			pLimStruct->lim_proc = lim_proc_delay;
#endif
		}
		else
		{
#ifdef LIM_ASM
			pLimStruct->lim_proc = lim_proc_delayless_asm_32;
#else
			pLimStruct->lim_proc = lim_proc_delayless;
#endif
		}

		pLimStruct->apply_makeup_gain = apply_makeup_gain32;
	}


	// for delay = 0, we dont need these buffers, make them NULL
	if(pCfg->params.limDelay == 0)
	{
		pLimDataStruct->pInpBuffer = NULL;
	  	pLimDataStruct->pZcBuffer =  NULL;
	}
	else
	{
	// Initialize limiter data structure pointers
	pLimDataStruct->pInpBuffer = pLimStruct->pDelayBuf;
	pLimDataStruct->pZcBuffer = pLimStruct->pZcBuf;
	}
	// Reset limiter data structure
	Lim_DataStruct_Reset(pLimDataStruct, pCfg);

	/////////////////////////////////////////////////////////
	// End module initialization
	/////////////////////////////////////////////////////////
	return 0;
}

/*======================================================================

  FUNCTION      Lim_reinit

  DESCRIPTION   Performs reinit of configuration structures without
                modifiying the existing data structure for the
                LIM algorithm. Pointers to configuration structures
				are passed in for configuration.

                Called in scenarios when only configuration data needs
				to be updated.

  PARAMETERS    pLimCfgStruct: [in] Pointer to configuration structure
                params: [in] Pointer to tuning parameter list
                numChannels: [in] Number of channels
                samplingRate: [in] Input sampling rate

  SIDE EFFECTS  None

======================================================================*/

int Lim_reinit(LimCfgType *pCfg,
	int32 *params,
	int32 sampleRate)
{
	int32 accu32;

	// Initialize limiter configuration structure: LimCfgType
	// Re-initialize Limiter tuning parameters
	pCfg->params.limThresh      = params[LIM_THRESH];               // Limiter threshold:Q3.15 for 16bit; Q4.27 for 32bit
	pCfg->params.limMakeUpGain  = (int16) params[LIM_MAKEUP_GAIN];  // Q7.8 Limiter make-up gain
	pCfg->params.limGc          = (int16)params[LIM_GC];            // Q15 Limiter gain recovery coefficient
	pCfg->params.limMaxWait     = (int16)params[LIM_MAX_WAIT];      // Q15 Limiter waiting time in seconds

	accu32 = s32_mult_s32_s16_rnd_sat(sampleRate, pCfg->params.limMaxWait);
	pCfg->params.limMaxWait = s16_saturate_s32(accu32);

	// Calculate and store max wait-minus-one sample in the static memory
	pCfg->limWaitMinusOne = pCfg->params.limMaxWait-1;

	// Left shift the make up gain by (7+1) to Q7.16 before multiplication with the data
	pCfg->mGainL32 = s32_shl_s32_sat((int32 )pCfg->params.limMakeUpGain, 8);

#ifdef LIM_DEBUG
	printf("Limiter Enable: %d \n", pCfg->params.limEnable);
	printf("Limiter Mode: %d \n", pCfg->params.limMode);
	printf("Limiter Threshold: %d \n", pCfg->params.limThresh);
	printf("Limiter Makeup Gain: %d \n", pCfg->params.limMakeUpGain);
	printf("Limiter Gain Coeff: %d \n", pCfg->params.limGc);
	printf("Limiter Delay: %d \n", pCfg->params.limDelay);
	printf("Limiter Wait-time: %d \n", pCfg->params.limMaxWait);
#endif


	/////////////////////////////////////////////////////////
	// End module Reinitialization
	/////////////////////////////////////////////////////////
	return (0);
}


/*======================================================================

FUNCTION      Lim_DataStruct_Reset

DESCRIPTION   Performs reset of data structures for the
LIM algorithm. Pointers to the data
structure is passed in for reset.

PARAMETERS    pLimDataStruct: [in] Pointer to data structure

SIDE EFFECTS  None

======================================================================*/

void Lim_DataStruct_Reset(LimDataType *pLimDataStruct,
						  LimCfgType *pLimCfgStruct)
{
	LimDataType *pData = pLimDataStruct;

	// Initialize the de-compressed LIM gain buffer
	pData->gainQ15 = LIM_GAIN_UNITY;                // Q15 (1) Unity gain
	pData->gainVarQ15 = 0;                          // Initialize the gain variable

	// Initialize static members of the data structure
	pData->curIndex = 0;                            // Initialize the current operating index
	pData->prev0xIndex = 0;                         // Initialize the previous index

	pData->localMaxPeakL32 = 0;                     // Initialize the local maxima peak
	pData->prevSampleL32 = 0;                       // Initialize the previous sample

	pData->fadeFlag = FADE_INIT;                            // Initialize the fade flag

	// Initialize the buffer memory to zero
	buffer32_empty(pData->pInpBuffer, pLimCfgStruct->params.limDelay);
	buffer32_empty(pData->pZcBuffer, pLimCfgStruct->params.limDelay);
}


/*======================================================================

FUNCTION      Lim_process

DESCRIPTION   Process multi-channel input audio sample by sample and limit the
input to specified threshold level. The input can be in any sampling
rate - 8, 16, 22.05, 32, 44.1, 48, 96, 192 KHz. The input is 32-bit Q23
and	the output is 32-bit Q23. Implements zero-crossing update based
limiter to limit the input audio signal. The process function separates
out delayed and delay-less implementation.

DEPENDENCIES  Input pointers must not be NULL.
lim_init must be called prior to any call to lim_process.
Length of output buffer must be at least as large the
length of the input buffer.

PARAMETERS    pInpL32Q23: [in] Pointer to 32-bit Q23 multi-channel audio
pOutL32Q23: [out] Pointer to 32-bit Q23 output audio

RETURN VALUE  None.

SIDE EFFECTS  None.

======================================================================*/

void Lim_process (CLimiterLib *pLimStruct,
				  void        *pOut,
				  int32       *pInp,
				  int16       frameSize,
				  int16       bypass)
{
	int16 iSubFrameSize = LIMITER_BLOCKSIZE;
	LimCfgType *pCfg = &pLimStruct->LimCfgStruct;
	LimDataType *pData = &pLimStruct->LimDataStruct;
	int16  iFrameSize;
	int32 *pInpFrameL32;
	void *pOutFrame = pOut;
	int16 ChannelBypass;
	int32 limMode = pCfg->params.limMode;
#ifdef LIM_ASM
   int32 bitsPerSample = pLimStruct->bitsPerSample;
#endif
   pInpFrameL32 = (int32 *)pInp;

	iFrameSize = frameSize;
	ChannelBypass = bypass;
	/****************************************************************************
	Process Limiter operation on the input audio frame data
	****************************************************************************/
	while ( iFrameSize > 0 )
	{
		// Determine the sub frame size for processing.
		// If the block length is larger than LIMITER_BLOCKSIZE,
		// take LIMITER_BLOCKSIZE at a time for processing.
		// For the remaining samples less than LIMITER_BLOCKSIZE,
		// use that sample count directly.
		if ( iFrameSize <= iSubFrameSize )
		{
			iSubFrameSize = iFrameSize;
		}
		iFrameSize -= iSubFrameSize;               // Decrement framesize

		if (1 == ChannelBypass)
		{
				// Copy the 32-bit input data into a temporary buffer
				pData->ptempBuffer = pInpFrameL32;
				if(FADE_INIT == pData->fadeFlag)
				{
					// Re-set limiter bypass mode just for this frame to do normal limiter processing
					ChannelBypass = 0;
					pData->fadeFlag = FADE_START;
				}

		}
		else // not bypass mode
		{
			// Copy the 32-bit input data into a temporary buffer
			pData->ptempBuffer = pInpFrameL32;
			pData->fadeFlag = FADE_INIT; // Reset fade-in flag
		}

		// limiter processing
		if(limMode) //limiter is enabled
		{
			if (1 == ChannelBypass)
			{
				/*****************************************************************
				Delay only processing without limiting for single input streams
				******************************************************************/
				lim_pass_data(pCfg, pData, iSubFrameSize);
			}
			else // bypass
			{
				/*****************************************************
				Limiter processing
				*****************************************************/
				(*pLimStruct->lim_proc)(pCfg, pData, pData->ptempBuffer, pOutFrame, iSubFrameSize);

#ifdef LIM_ASM
				/* increment the output ptr*/
				pOutFrame = (char*)pOutFrame + iSubFrameSize*(bitsPerSample>>3);
#endif

			}

		}/* if(pCfg[ChIndx]->params.limMode) */


		/****************************************************************************
		Implementation of Make up gain on the limiter output
		// out16 = (out32 * gain) << (7-gainexp);
		//       = out32 * ( gain << (7-gainexp) );
		*****************************************************************************/
#ifdef LIM_ASM
		if ((pCfg->params.limMode == 0) || (ChannelBypass == 1))
		{
#endif		 // make-up gain is done in asm only

			(*pLimStruct->apply_makeup_gain)(pCfg,pData,&pOutFrame,iSubFrameSize);

#ifdef LIM_ASM
		}
#endif
		// Update the input frame data pointers
		pInpFrameL32 += iSubFrameSize;
	} /* while ( frameSize > 0 ) */

}

/*======================================================================

FUNCTION      lim_proc_delay

DESCRIPTION   Limiter processing function.
Detects zero crossing on the input signal and updates the
limiter data structure. The limiter alogorithm is based on
Pei Xiang's investigation and modification of a limiter technique
based on updating gain at the zero-crossings.

Dan Mapes-Riordan, and W.Marshall Leach, "The Design of a digital
signal peak limiter for audio signal processing," Journal of
Audio Engineering Society, Vol. 36, No. 7/8, 1988 July/August.

Updates the limiter gain based on local maxima peak searching
and zero-crossing locations and applies the gain on the input
data.

DEPENDENCIES  Input pointers must not be NULL.

PARAMETERS    pCfg: [in] Pointer to limiter configuration structure
pData: [in,out] Pointer to limiter data structure
iSubFrameSize: [in] FrameSize of the data buffer

RETURN VALUE  None

SIDE EFFECTS  None.

======================================================================*/
void lim_proc_delay ( LimCfgType *pCfg,
					 LimDataType *pData, int32 *in, void *out,
					 int16 iSubFrameSize )
{
	int j, cur_idx;
	int64 accu64, prod64;
	int32 inpL32, accu32, attn32, absL32, local_max32;
	int16 attnQ15, iq16, tmp16;

	for (j=0; j<iSubFrameSize; j++)
	{
		// Index for current sample in the zero-crossing buffer/input buffer
		cur_idx = pData->curIndex;

		// Extract and store the current input data
		inpL32 = pData->ptempBuffer[j];

		// Compute the absolute magnitude of the input
		absL32 = (int32)u32_abs_s32_sat(inpL32);

		/****************************************************************************
		Special case: No zero-crossings during the buffer duration.
		If max delayline is reached and we have to force a zero crossing.
		This is identified when the zcIndex reaches prev0xIndex. By design,
		prev0xIndex is always chasing zcIndex at any zc, so when zcIndex
		catches up with prev0xIndex, it means zcIndex has searched more than
		"delay" times and still not finding a zc. we'll force write the current
		local max to this location for it to correctly update the gain afterwards.
		****************************************************************************/
		if(cur_idx == pData->prev0xIndex)
		{
			// Store tracked local peak max to this location
			pData->pZcBuffer[pData->prev0xIndex] = pData->localMaxPeakL32;

			// Reset local max value
			pData->localMaxPeakL32 =absL32;
		}

		/****************************************************************************
		Compute the limiter gain and update it at zero-crossings - start
		*****************************************************************************/

		// Update the limiter gain if zero-crossing is detected
		if(pData->pZcBuffer[cur_idx] != 0)
		{
			// Compute maximum of gain vs. previous instantaneous gain - March 2011 Changes
			// tmp16 = max(gain, 1-GRC*gainVar)
			tmp16 = s16_extract_s32_h(s32_mult_s16_s16_shift_sat(pData->gainVarQ15, (int16)pCfg->params.limGc));
			tmp16 = s16_sub_s16_s16_sat(LIM_GAIN_UNITY, tmp16);
			tmp16 = s16_max_s16_s16(tmp16, (int16)pData->gainQ15);

			// Buffer data x gain - Multiply 32x16 round, shift and sat in one cycle
			accu32 = s32_mult_s32_s16_rnd_sat(pData->pZcBuffer[cur_idx], tmp16);

			// If the peak in the data is above the threshold, attack the gain and reduce it.
			if(accu32 > pCfg->params.limThresh )
			{
				// Use Q6 DSP's linear approximation division routine to lower down the MIPS
				// The inverse is computed with a normalized shift factor
				accu64 = dsplib_approx_divide(pCfg->params.limThresh, pData->pZcBuffer[cur_idx]);
				attn32 = (int32 )accu64;                       // Extract the normalized inverse
				iq16   = (int16 )(accu64 >> 32);               // Extract the normalization shift factor

				// Shift the result to get the quotient in desired Q15 format
				attnQ15 = s16_saturate_s32(s32_shl_s32_sat(attn32, iq16+15));

				// Attack gain immediately if the peak overshoots the threshold
				pData->gainVarQ15 = s16_sub_s16_s16_sat(LIM_GAIN_UNITY, attnQ15);
			}
			else
			{
				// If peak is not above the threshold, do a gain release.
				// Release slowly using gain recovery coefficient (Q15 multiplication)
				pData->gainVarQ15 = s16_extract_s32_h(s32_mult_s16_s16_shift_sat(pData->gainVarQ15, (int16)pCfg->params.limGc));
			}

			// Update the gain at the zero-crossings
			pData->gainQ15 = s16_sub_s16_s16_sat(LIM_GAIN_UNITY, (int16)pData->gainVarQ15);

		} /* if(pData->pZcBuffer32[cur_idx] != 0) */

		/****************************************************************************
		Implementation of Limiter gain on the input data
		*****************************************************************************/
		// Gain application - Multiply and shift and round and sat (one cycle in Q6)
		if (pData->gainQ15 == (int16)LIM_GAIN_UNITY) {
			pData->ptempBuffer[j] = pData->pInpBuffer[cur_idx];
		}
		else {
		pData->ptempBuffer[j]=s32_mult_s32_s16_rnd_sat(pData->pInpBuffer[cur_idx], (int16)pData->gainQ15);
		}

		/****************************************************************************
		Compute the zero-crossing locations and local maxima in the input audio - start
		*****************************************************************************/
		// Store the local max peak in a variable
		local_max32 = pData->localMaxPeakL32;

		// Detect zero-crossing in the data
		prod64 = s64_mult_s32_s32(inpL32, pData->prevSampleL32);

		// Zero-crossing condition: sample is exactly zero or it changes sign.
		// If it's exactly zero, the loop is entered twice, but it's okay.
		// Update the peak detection if zero-crossing is detected
		if( (prod64 <= 0) || (cur_idx == pData->prev0xIndex))
		{
			// Update zero-crossing buffer to store local maximum.
			pData->pZcBuffer[pData->prev0xIndex] = local_max32;

			// Update previous zero-crossing index with the current buffer index
			pData->prev0xIndex = cur_idx;
			local_max32 = absL32;       // Reset local maximum.

			// Note that so far just prev0xIndex points to this location,
			// Prepare for the next write to the location when next zc is
			// reached, but nothing has been written at this location in zcBuffer yet
		}
		else
		{
			// Mark non zero-crossing reference sample as zero.
			pData->pZcBuffer[cur_idx] = 0;

			// Continue searching if zero-crossing is not detected
			local_max32 = s32_max_s32_s32(local_max32,absL32);
		}

		// Save the local maximum in the static memory
		pData->localMaxPeakL32 = local_max32;

		// Store the new input sample in the input buffer
		pData->pInpBuffer[pData->curIndex] = inpL32;

		// Update the previous sample
		pData->prevSampleL32 = inpL32;

		// Increment the zero-crossing index for the circular buffer
		pData->curIndex += 1;

		if(pData->curIndex > (pCfg->limDelayMinusOne))
		{
			pData->curIndex = 0;
		}
	} /* for (j=0; j<iSubFrameSize; j++) */
}

/*======================================================================

FUNCTION      lim_proc_delayless

DESCRIPTION   In delay-less implementation, the limiter
gains are updated immediately if the instantaneous peak
exceeds the threshold. The gain is also updated if the
time interval between gain updates exceeds the specified
wait time.

DEPENDENCIES  Input pointers must not be NULL.

PARAMETERS    pCfg: [in] Pointer to limiter configuration structure
pData: [in,out] Pointer to limiter data structure
iSubFrameSize: [in] Frame size for current data buffer

RETURN VALUE  None

SIDE EFFECTS  None.

======================================================================*/
void lim_proc_delayless ( LimCfgType *pCfg,
						 LimDataType *pData, int32 *in, void *out,
						 int16 iSubFrameSize )
{
	int j;
	int64 accu64, prod64;
	int32 accu32, attn32, inpL32, absL32;
	int16 attnQ15, iq16, tmp16;

	for (j=0; j<iSubFrameSize; j++)
	{
		// Extract and store the current input data
		inpL32 = pData->ptempBuffer[j];

		// Compute the absolute magnitude of the input
		absL32 = (int32)u32_abs_s32_sat(inpL32);

		// Detect zero-crossing in the data
		prod64 = s64_mult_s32_s32(inpL32, pData->prevSampleL32);

		// Compute maximum of gain vs. previous instantaneous gain - March 2011 Changes
		// tmp16 = max(gain, 1-GRC*gainVar)
		tmp16 = s16_extract_s32_h(s32_mult_s16_s16_shift_sat(pData->gainVarQ15, (int16)pCfg->params.limGc));
		tmp16 = s16_sub_s16_s16_sat(LIM_GAIN_UNITY, tmp16);
		tmp16 = s16_max_s16_s16(tmp16, (int16)pData->gainQ15);

		// absolute input x gain - Multiply 32x16 round, shift and sat in one cycle
		accu32 = s32_mult_s32_s16_rnd_sat(absL32, tmp16);

		// If the peak in the data is above the threshold, attack the gain immediately.
		if(accu32 > pCfg->params.limThresh )
		{
			// Use Q6 DSP's linear approximation division routine to lower down the MIPS
			// The inverse is computed with a normalized shift factor
			accu64 = dsplib_approx_divide(pCfg->params.limThresh, absL32);
			attn32 = (int32 )accu64;                       // Extract the normalized inverse
			iq16   = (int16 )(accu64 >> 32);               // Extract the normalization shift factor

			// Shift the result to get the quotient in desired Q15 format
			attnQ15 = s16_saturate_s32(s32_shl_s32_sat(attn32, iq16+15));

			// Attack gain immediately if the peak overshoots the threshold
			pData->gainVarQ15 = s16_sub_s16_s16_sat(LIM_GAIN_UNITY, attnQ15);

			// Re-set the wait time index counter
			pData->curIndex = 0;
		}
		else if((prod64 < 0)||(absL32 == 0) ||(pData->curIndex > pCfg->limWaitMinusOne))
		{
			// Gain release at zero-crossings or if the wait time is exceeded.
			// Release slowly using gain recovery coefficient (Q15 multiplication)
			pData->gainVarQ15 = s16_extract_s32_h(s32_mult_s16_s16_shift_sat(pData->gainVarQ15, pCfg->params.limGc));

			// Re-set the wait time index counter
			pData->curIndex = 0;
		}

		// Update the gain at the zero-crossings (no saturation)
		pData->gainQ15 = s16_sub_s16_s16(LIM_GAIN_UNITY, (int16)pData->gainVarQ15);

		// Store previous sample in the memory
		pData->prevSampleL32 = inpL32;

		// Increment the wait-time counter index in a circular buffer fashion
		pData->curIndex += 1;


		/****************************************************************************
		Implementation of Limiter gain on the input data
		*****************************************************************************/
		// Gain application - Multiply and shift and round and sat (one cycle in Q6)
		if (pData->gainQ15 == (int16)LIM_GAIN_UNITY) {
			pData->ptempBuffer[j] = inpL32;
		}
		else {
		pData->ptempBuffer[j]=s32_mult_s32_s16_rnd_sat(inpL32, pData->gainQ15);
		}
	} /* for (j=0; j<iSubFrameSize; j++) */
}


/*===========================================================================*/
/* FUNCTION : buffer32_copy                                                    */
/*                                                                           */
/* DESCRIPTION: Copy from one buffer to another                              */
/*                                                                           */
/* INPUTS: destBuf-> output buffer                                           */
/*         srcBuf-> input buffer                                             */
/*         samples: size of buffer                                           */
/* OUTPUTS: destBuf-> output buffer                                          */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/

void buffer32_copy
(
 int32           *destBuf,           /* output buffer                     */
 int32           *srcBuf,            /* input buffer                      */
 uint32           samples            /* number of samples to process      */
 )
{
	uint32 i;

	for (i = 0; i < samples; i++)
	{
		*destBuf++ = *srcBuf++;
	}
}

/*===========================================================================*/
/* FUNCTION : buffer32_empty                                                 */
/*                                                                           */
/* DESCRIPTION: Set all samples of a buffer to zero.                         */
/*                                                                           */
/* INPUTS: buf-> buffer to be processed                                      */
/*         samples: size of buffer                                           */
/* OUTPUTS: buf-> buffer to be processed                                     */
/*                                                                           */
/* IMPLEMENTATION NOTES:                                                     */
/*===========================================================================*/

void buffer32_empty
(
 int32           *buf,               /* buffer to be processed            */
 uint32           samples            /* number of samples in this buffer  */
 )
{
	uint32 i;

	for (i = 0; i < samples; i++)
	{
		*buf++ = 0;
	}
}
/*======================================================================

FUNCTION      lim_pass_data

DESCRIPTION   Pass data with just delay, no limiter processing.
During transition from limiter processing to no-limiter
processing, fade out the existing limiter gain gradually
so as to minimize sudden discontinuities.

DEPENDENCIES  Input pointers must not be NULL.

PARAMETERS    pCfg: [in] Pointer to limiter configuration structure
pData: [in,out] Pointer to limiter data structure
iSubFrameSize: [in] FrameSize of the data buffer

RETURN VALUE  None

SIDE EFFECTS  None.

======================================================================*/
void lim_pass_data ( LimCfgType *pCfg,
					  LimDataType *pData,
					  int16 iSubFrameSize )
{
	int j;
	int32 inp32;
	int32 *buf32 = pData->ptempBuffer;
	int32 gainVarQ15 = pData->gainVarQ15;
	int32 gainQ15 = pData->gainQ15;
	int32 curIndex = pData->curIndex;
	int32 limDelayMinusOne = pCfg->limDelayMinusOne;
	int32 limDelay = pCfg->params.limDelay;

	// Perform gain release to bring the gain back to unity
	if(gainVarQ15 > 0)
	{
		// Release the gain variable gradually to smooth out discontinuities
		gainVarQ15 = s16_extract_s32_h(s32_mult_s16_s16_shift_sat((int16)gainVarQ15, LIM_FADE_GRC));

		// Update the gain once every frame
		gainQ15 = s16_sub_s16_s16(LIM_GAIN_UNITY, (int16)gainVarQ15);

		for (j=0; j<iSubFrameSize; j++)
		{
			// Extract and store the current input data
			inp32 = *buf32;

			// Apply limiter gain on the data
			// Gain application - Multiply and shift and round and sat (one cycle in Q6)
			*buf32++ = s32_mult_s32_s16_rnd_sat(pData->pInpBuffer[curIndex], (int16)gainQ15);

			// Update the input buffer
			pData->pInpBuffer[curIndex] = inp32;

			// Increment the zero-crossing index for the circular buffer
			curIndex += 1;

			// Increment the current Index
			if(curIndex > (limDelayMinusOne))
			{
				curIndex = 0;
			}
		}
	}
	else
	{
		// Pass the delayed buffer data to the output
		for (j=0; j<iSubFrameSize; j++)
		{
			// Extract and store the current input data
			inp32 = *buf32;

			// Place the delay buffer data into the output
			*buf32++ = pData->pInpBuffer[curIndex];

			// Update the input buffer
			pData->pInpBuffer[curIndex] = inp32;

			// Increment the zero-crossing index for the circular buffer
			curIndex += 1;

			// Increment the current Index
			if(curIndex > (limDelayMinusOne))
			{
				curIndex = 0;
			}
		}
	}

	// Do one-time initialization of the zero-crossing buffers and memory variables
	if(pData->fadeFlag == FADE_START)
	{
		// Reset the zero-crossing buffer
		memset(pData->pZcBuffer, 0, limDelay*sizeof(int32));

		// Reset the previous sample
		pData->prevSampleL32 = (int32 )0x7FFF;

		// Reset the local max peak
		pData->localMaxPeakL32 = (int32)0x0001;

		// Reset the fade-in flag
		pData->fadeFlag = FADE_STOP;
	}
	pData->gainVarQ15 = gainVarQ15;
	pData->gainQ15 = gainQ15;
	pData->curIndex = curIndex;
	pCfg->limDelayMinusOne = limDelayMinusOne;


}

/*======================================================================

FUNCTION      Lim_get_lib_size

DESCRIPTION   Returns the limiter library size for a given delay,samplerata,numchannels
			  so that the library users can use this funtion to allocate the num of bytes
			  returned by this.

PARAMETERS    limDelay: [in] limiter delay in sec Q15 format
			  sampleRate: [in] sampleRate
			  numChannles: [in] num of channels

RETURN VALUE  limLibSize: [out] total library size for the above input params

======================================================================*/

int32 Lim_get_lib_size(int16 limDelay,int32 sampleRate,int32 numChannels)
{
	int32 limLibSize;
	int32 DelayBufSize;
	int32 accu32;

	// Convert the delay and waiting time from Q15 seconds to Q0 samples
	accu32 = s32_mult_s32_s16_rnd_sat(sampleRate,limDelay );
	DelayBufSize = s16_saturate_s32(accu32);

	limLibSize = align_to_8_byte(sizeof(CLimiterLib))
		         + align_to_8_byte(4*DelayBufSize) // delay buf Size
				 + align_to_8_byte(4*DelayBufSize); // Zc Buf size

	limLibSize = limLibSize*numChannels;

	return limLibSize;
}

/*======================================================================

FUNCTION      Lim_set_lib_memory

DESCRIPTION   Places the library structures and buffers at the given input pointer/memory location

PARAMETERS    pLimBufPtr: [in] the start address of the malloced memory
			  pLimiter: [in,out] address of the first element of the array of limiter pointers,
						the array members will be filled with the input memory pointer
			  limDelay: [in] limiter delay in sec Q15 format
			  sampleRate: [in] sampleRate
			  chIdx: [in] channel index num

RETURN VALUE  none

======================================================================*/

void Lim_set_lib_memory(int8 *pLimBufPtr,CLimiterLib **pLimiter, int16 limDelay,int32 sampleRate,
						int32 chIdx)
{
	int32 accu32;
	int32 limDelayBufSize;
	int32 totalLibSize;
	int32 delayBufOffset = align_to_8_byte(sizeof(CLimiterLib));
	int32 zcBufOffset;
	CLimiterLib *pLimStruct;

	// Convert the delay and waiting time from Q15 seconds to Q0 samples
	accu32 = s32_mult_s32_s16_rnd_sat(sampleRate, limDelay );
	limDelayBufSize = s16_saturate_s32(accu32);

	zcBufOffset = delayBufOffset +  align_to_8_byte(4*limDelayBufSize);

	totalLibSize = align_to_8_byte(4*limDelayBufSize) + align_to_8_byte(4*limDelayBufSize)
					+ align_to_8_byte(sizeof(CLimiterLib));

	*pLimiter = (CLimiterLib*)(pLimBufPtr + (totalLibSize*chIdx));

	pLimStruct = (CLimiterLib*)(*pLimiter);

	if(limDelay != 0)
	{
		//Init buffer pointers
		pLimStruct->pDelayBuf = (int32*)(pLimBufPtr + (chIdx*totalLibSize) + delayBufOffset);
		pLimStruct->pZcBuf = (int32*)(pLimBufPtr + (chIdx*totalLibSize) + zcBufOffset);
	}

	pLimStruct->LimDataStruct.pInpBuffer = pLimStruct->pDelayBuf;
	pLimStruct->LimDataStruct.pZcBuffer = pLimStruct->pZcBuf;

}
