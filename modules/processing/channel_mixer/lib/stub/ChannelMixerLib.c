/*============================================================================
  @file ChannelMixerLib.c

  Implementation of the Channel Mixer algorithm. */

/*=========================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
========================================================================= */

#include "ChannelMixerLib.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/*
@brief Return the size of state structure required to operate one instance of
Channel mixer.

@param pSize: [out] Pointer to memory location where size of structure must be
updated.
*/
void ChMixerGetInstanceSize(uint32 *pSize)
{
    *pSize = 0;
    return;
}

/*
@brief Initialize Channel mixing algorithm
Based on the input and output channels, identify the input & output channel
configurations respectively

@param pCMState : [out] Pointer to the state structure that must be initialized.
@param numInputChannels : [in] Number of input channels.
@param inputChannels : [in] Pointer to array that indicates what each input channel is
@param numOutputChannels : [in] Number of output channels.
@param outputChannels : [in] Pointer to array that indicates what each output channel is
@param dataBitWidth : [in] Bit-width of data

Return values: A value indicate success or failure ( a different value for each
               failure)
*/

ChMixerResultType ChMixerInitialize(void *pCMState, uint32 mem_size, uint32 numInputChannels,
                ChMixerChType *inputChannels, uint32 numOutputChannels,
                ChMixerChType *outputChannels, uint32 dataBitWidth)
{
	return CH_MIXER_ENOT_SUPPORTED;
}


/*
@brief set param interface for Channel mixing algorithm
Based on the parameters set by client, update the channel mixer library accordingly.

@param pCMState : [out] Pointer to the state structure that must be initialized.
@param numInputChannels : [in] Number of input channels.
@param inputChannels : [in] Pointer to array that indicates what each input channel is
@param numOutputChannels : [in] Number of output channels.
@param outputChannels : [in] Pointer to array that indicates what each output channel is
@param dataBitWidth : [in] Bit-width of data
@param pCustomCoeffSet : [in] Pointer to array of custom coefficients sent through set param

Return values: A value indicate success or failure ( a different value for each
               failure)
*/
ChMixerResultType ChMixerSetParam(void *pCMState, uint32 mem_size, uint32 numInputChannels,
                ChMixerChType *inputChannels, uint32 numOutputChannels,
                ChMixerChType *outputChannels, uint32 dataBitWidth,
                void *pCustomCoeffSet)
{
	return CH_MIXER_ENOT_SUPPORTED;
}



/*
@brief Apply the Channel mixing algorithm

@param pCMState : [in] Pointer to the state structure
@param input : [in] Multi channel input pointer
@param output : [out] Multi channel output pointer
@param numSamples : [in] Number of samples in the input
*/
void ChMixerProcess(void *pCMState, void **output, void **input, uint32 numSamples)
{
	return;
}

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */


