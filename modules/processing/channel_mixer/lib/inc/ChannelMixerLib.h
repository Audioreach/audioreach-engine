#ifndef CHANNEL_MIXER_LIB_H
#define CHANNEL_MIXER_LIB_H
/*============================================================================
  @file ChannelMixerLib.h

  Public header file for the Channel Mixer algorithm. */

/*=========================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
========================================================================= */

#include "AudioComdef.h"
#include "ar_defs.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef PROD_SPECIFIC_MAX_CH
#define CH_MIXER_MAX_NUM_CH PROD_SPECIFIC_MAX_CH   // Maximum number of channels.
#else
#define CH_MIXER_MAX_NUM_CH 32   // Maximum number of channels.
#endif

#define CH_MIXER_ALIGN_8_BYTE(x) (((x) + 7) & (0xFFFFFFF8))
#define CH_MIXER_BIT_MASK_SIZE 64
#define CH_MIXER_MAX_BITMASK_GROUPS 3
#define CH_MIXER_BIT_MASK_SIZE_MINUS_ONE (CH_MIXER_BIT_MASK_SIZE - 1)

typedef enum _ChMixerBitMaskGroup
{
   ChMixerBitMaskGroup_1to63    = 0,
   ChMixerBitMaskGroup_64to127  = 1,
   ChMixerBitMaskGroup_128to191 = 2,
} ChMixerBitMaskGroup;

// ID for each channel
typedef enum _ChMixerChType
{
    CH_MIXER_PCM_CH_NONE = 0,   //  None
    CH_MIXER_PCM_CH_L =    1,   //  Front-left (FL)
    CH_MIXER_PCM_CH_R =    2,   //  Front-right (FR)
    CH_MIXER_PCM_CH_C =    3,   //  Front-center (FC)
    CH_MIXER_PCM_CH_LS =   4,   //  Left-surround (LS)
    CH_MIXER_PCM_CH_RS =   5,   //  Right-surround (RS)
    CH_MIXER_PCM_CH_LFE_1 =6,   //  Low frequency effects (LFE-1)
    CH_MIXER_PCM_CH_CS =   7,   //  Center-surround (CS, RC)
    CH_MIXER_PCM_CH_LB =   8,   //  Left-back (LB, RL)
    CH_MIXER_PCM_CH_RB =   9,   //  Right-back (RB, RR)
    CH_MIXER_PCM_CH_TS =   10,  //  Top-surround (TS)
    CH_MIXER_PCM_CH_CVH =  11,  //  Center-vertical-height (CVH)
    CH_MIXER_PCM_CH_TFC =  CH_MIXER_PCM_CH_CVH,  // Top Front Center (TFC)
    CH_MIXER_PCM_CH_MS =   12,  //  Mono-surround (MS)
    CH_MIXER_PCM_CH_FLC =  13,  //  Front left of center (FLC)
    CH_MIXER_PCM_CH_FRC =  14,  //  Front Right of center (FRC)
    CH_MIXER_PCM_CH_RLC =  15,  //  Rear left of center (RLC)
    CH_MIXER_PCM_CH_RRC =  16,  //  Rear right of center (RRC)
    CH_MIXER_PCM_CH_CB  =  CH_MIXER_PCM_CH_CS,  //  Center-back (CB)
    CH_MIXER_PCM_CH_LFE_2 =17,  //  Low frequency effects (LFE-2)
    CH_MIXER_PCM_CH_SL  =  18,  //  Side-Left (SL)
    CH_MIXER_PCM_CH_SR  =  19,  //  Side-Right(SR)
    CH_MIXER_PCM_CH_TFL =  20,  //  Top Front Left (TFL)
    CH_MIXER_PCM_CH_LVH =  CH_MIXER_PCM_CH_TFL,  //  Left Vertical Height (LVH)
    CH_MIXER_PCM_CH_TFR =  21,  //  Top Front Right (TFR)
    CH_MIXER_PCM_CH_RVH =  CH_MIXER_PCM_CH_TFR,  //  Right Vertical Height (RVH)
    CH_MIXER_PCM_CH_TC  =  22,  //  Top Center (TC)
    CH_MIXER_PCM_CH_TBL =  23,  //  Top Back Left (TBL)
    CH_MIXER_PCM_CH_TBR =  24,  //  Top Back Right (TBR)
    CH_MIXER_PCM_CH_TSL =  25,  //  Top Side Left (TSL)
    CH_MIXER_PCM_CH_TSR =  26,  //  Top Side Right (TSR)
    CH_MIXER_PCM_CH_TBC =  27,  //  Top Back Center (TBC)
    CH_MIXER_PCM_CH_BFC =  28,  //  Bottom Front Center (BFC)
    CH_MIXER_PCM_CH_BFL =  29,  //  Bottom Front Left (BFL)
    CH_MIXER_PCM_CH_BFR =  30,  //  Bottom Front Right (BFR)
    CH_MIXER_PCM_CH_LW  =  31,  //  Left Wide (LW)
    CH_MIXER_PCM_CH_RW  =  32,  //  Right Wide (RW)
    CH_MIXER_PCM_CH_LSD =  33,  //  Left Side Direct (LSD)
    CH_MIXER_PCM_CH_RSD =  34,  //  Right Side Direct (RSD)
    CH_MIXER_PCM_CH_MAX_TYPE = 128
} ChMixerChType;

// Possible return values to indicate status of library.
typedef enum _ChMixerResultType
{
    CH_MIXER_SUCCESS = 0,
    CH_MIXER_ENOT_SUPPORTED = -1,  // Return this if the specific combination of
                                  // input and output channel conversion is not supported
	CH_MIXER_EBAD_PARAM = -2
} ChMixerResultType;

// Maintain internal state
typedef struct _ChMixerdynStateStruct
{
	// input channel map array inputChannels[num_input_channels]
    ChMixerChType *pInputChannels;
    // output channel map array outputChannels[num_output_channels]
    ChMixerChType *pOutputChannels;
    // The mixer matrix.
    // linear matrix array mixerMatrixL16Q14[num_output_channels * num_input_channels]
    int16 *pMixerMatrixL16Q14;
    // To identify which matrix coefficients are zero, define the following array
    // row array dimension is defined with one more than number of channels for keeping the end marker value
    // 2-D step matrix represented in linear array inputStepMatrix[num_output_channels][num_input_channels + 1];
    int8 *pInputStepMatrix;
 } ChMixerdynStateStruct;

// Maintain internal state
 typedef struct _ChMixerStateStruct {
   uint32 numInputCh;  // Number of input channels
   uint32 numOutputCh;  // Number of output channels
   uint32 num_active_coeff;
   // Number of coefficients refers to numbber of input channels that contribute
   // to all the output channels
   uint32 dataBitWidth;  // Data bit width of input and output.
   // If the input channels and output channels are same, then
   // it is a trivial copy (with some reordering).
   bool_t isTrivialCopy;
   // flag to indicate if custom mixer coefficients are applicable.
   // 0 indicates default coefficient set is used.
   bool_t isCustomCoeffValid;
   // ptr to custom coefficient memory
   int16 *ptrCustomCoeffset;
   // ptr to coefficients
   int16 *ptrCoeff;
   // dynamic channel mixer structure
   ChMixerdynStateStruct dynState;
 } ChMixerStateStruct;

/*
@brief Return the size of state structure required to operate one instance of
Channel mixer. The size is in bytes (at least 4-byte aligned).

@param pSize: [out] Pointer to memory location where size of structure must be
updated.
Return values: None
*/
void ChMixerGetInstanceSize(uint32 *pSize, uint32 numInputChannels, uint32 numOutputChannels);

/*
@brief Initialize Channel mixing algorithm
Based on the input and output channels, identify the input & output channel
configurations respectively

@param pCMState : [out] Pointer to the state structure that must be initialized.
@param numInputChannels : [in] Number of input channels.
@param inputChannels : [in] Pointer to array that indicates what each input channel is
@param numOutputChannels : [in] Number of output channels.
@param inputChannels : [in] Pointer to array that indicates what each output channel is
@param dataBitWidth : [in] Bit-width of data

Return values: Return CH_MIXER_SUCCESS if this conversion can be performed and
return CH_MIXER_ENOT_SUPPORTED if conversion is not supported.
*/

ChMixerResultType ChMixerInitialize(void *         pCMState,
                                    uint32         lib_mem_size,
                                    uint32         numInputChannels,
                                    ChMixerChType *inputChannels,
                                    uint32         numOutputChannels,
                                    ChMixerChType *outputChannels,
                                    uint32         dataBitWidth);

/*
@brief Apply the Channel mixing algorithm

@param pCMState : [in] Pointer to the state structure
@param output : [out] Multi channel output pointer
@param input : [in] Multi channel input pointer
@param numSamples : [in] Number of samples in the input
*/
void ChMixerProcess(void *pCMState, void **output, void **input, uint32 numSamples);

/*
@brief set param interface for Channel mixing algorithm
Based on the parameters set by client, update the channel mixer library accordingly.

@param pCMState : [in] Pointer to the state structure that must be initialized.
@param numInputChannels : [in] Number of input channels.
@param inputChannels : [in] Pointer to array that indicates what each input channel is
@param numOutputChannels : [in] Number of output channels.
@param outputChannels : [in] Pointer to array that indicates what each output channel is
@param dataBitWidth : [in] Bit-width of data
@param pCustomCoeffSet : [in] Pointer to array of custom coefficients sent through set param

Return values: A value indicate success or failure ( a different value for each
               failure)
*/
ChMixerResultType ChMixerSetParam(void *         pCMState,
                                  uint32         lib_mem_size,
                                  uint32         numInputChannels,
                                  ChMixerChType *inputChannels,
                                  uint32         numOutputChannels,
                                  ChMixerChType *outputChannels,
                                  uint32         dataBitWidth,
                                  void *         pCustomCoeffSet);

/*
@brief Query for num active coefficients
*/
uint32 ChMixerGetNumActiveCoefficients(ChMixerStateStruct *pState);


#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif  // #ifndef CHANNEL_MIXER_LIB_H

