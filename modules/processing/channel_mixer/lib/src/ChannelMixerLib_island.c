/*============================================================================
  @file ChannelMixerLib.c

  Impelementation of the Channel Mixer algorithm. */

/*=========================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
========================================================================= */

/*
@brief Apply the Channel mixing algorithm

@param pCMState : [in] Pointer to the state structure
@param input : [in] Multi channel input pointer
@param output : [out] Multi channel output pointer
@param numSamples : [in] Number of samples in the input
*/

#include "ChannelMixerLib.h"
#include "ChannelMixerRemapRules.h"
#include "audio_basic_op.h"

void ChMixerTrivialCopy(ChMixerStateStruct *pState, void **output, void **input, uint32 numSamples);

/*
@brief Copy the input to the output in the order specified in the Channel mixer
       state (inputStepMatrix)

@param pCMState : [in] Pointer to the state structure
@param input : [in] Multi channel input pointer
@param output : [out] Multi channel output pointer
@param numSamples : [in] Number of samples in the input
*/
void ChMixerTrivialCopy(ChMixerStateStruct *pState, void **output, void **input, uint32 numSamples)
{
   uint32 sampleIndex, chIndex;
   // Load the order in which the copy must be done
   int8 *inputStep = pState->dynState.pInputStepMatrix;

   // In this function, number of output channels == number of input channels.
   if (16 == pState->dataBitWidth)
   {
      int16 *input16[CH_MIXER_MAX_NUM_CH];
      int16 *output16[CH_MIXER_MAX_NUM_CH];
      for (chIndex = 0; (chIndex < pState->numOutputCh) && (chIndex < CH_MIXER_MAX_NUM_CH); chIndex++)
      {
         // Identify the input and output channel start addresses
         output16[chIndex] = ((int16 **)output)[chIndex];
         input16[chIndex]  = ((int16 **)input)[inputStep[chIndex]];
         for (sampleIndex = 0; sampleIndex < numSamples; sampleIndex++)
         {
            // Copy each channel
            *output16[chIndex] = *input16[chIndex];
            output16[chIndex]++;
            input16[chIndex]++;
         }
      }
   }
   else
   {
      int32 *input32[CH_MIXER_MAX_NUM_CH];
      int32 *output32[CH_MIXER_MAX_NUM_CH];
      for (chIndex = 0; (chIndex < pState->numOutputCh) && (chIndex < CH_MIXER_MAX_NUM_CH); chIndex++)
      {
         // Identify the input and output channel start addresses
         output32[chIndex] = ((int32 **)output)[chIndex];
         input32[chIndex]  = ((int32 **)input)[inputStep[chIndex]];
         for (sampleIndex = 0; sampleIndex < numSamples; sampleIndex++)
         {
            // Copy each channel
            *output32[chIndex] = *input32[chIndex];
            output32[chIndex]++;
            input32[chIndex]++;
         }
      }
   }
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

   uint32 sampleIndex;
   uint32 outputChIndex;
   uint32 inputChIndex;
   int16 *pMatrixCoeffL16Q14;
   // int16 *pMatrixCoeffStartL16Q14;
   int8 *              inputStep;
   int32               tempL32, prodL32;
   int64               tempL64, prodL64;
   ChMixerStateStruct *pState = (ChMixerStateStruct *)pCMState;

   if (pState->isTrivialCopy)
   {
      ChMixerTrivialCopy(pCMState, output, input, numSamples);
      return;
   }

   if (16 == pState->dataBitWidth)
   {
      // Generate output one channel at a time. This helps in reducing cache thrashing
      for (outputChIndex = 0; outputChIndex < pState->numOutputCh; outputChIndex++)
      {
         // Matrix coefficients of the row corresponding to the output channel
         pMatrixCoeffL16Q14 = pState->ptrCoeff + pState->numInputCh * outputChIndex;

         for (sampleIndex = 0; sampleIndex < numSamples; sampleIndex++)
         {
            inputStep = &pState->dynState.pInputStepMatrix[outputChIndex * (pState->numInputCh + 1)];
            // output[output channel i][sample] = matrix[output channel i ][input channel j] *
            //                                       input[input channel j][sample]
            // The above product is calculated for all input channels and the products are summed.
            // The data ordering can be exploited to write memory access based on pointers so that
            // the compiler can generate efficient code. For e.g
            // (1) All the output samples for a channel are stored one after the other. Hence in the
            //     above equation, output[output channel i][sample] is just *output[channel]++
            // (2) Similarly for calculating the output samples for a channel, only the factors in
            //     a particular row in the matrix are required. Since C implements matrices row-wise,
            //     we can access these coefficients as matrixCoeff++
            // (3) Input samples of all the channels must be accessed even for calculating output samples
            //     for one channels. So we initialize a local array with starting addresses of all input
            //     channels and increment each one of them as required.
            tempL32 = 0;
            for (inputChIndex = *inputStep; inputChIndex < pState->numInputCh; inputChIndex = *(++inputStep))
            {
               int16 *in_ch_data_ptr = (int16 *)input[inputChIndex];
               prodL32 = s32_mult_s16_s16(*(in_ch_data_ptr + sampleIndex), pMatrixCoeffL16Q14[inputChIndex]);

               // Maximum number of channels is 8, so this should avoid any overflow
               // We right shift by 4 to provide enough guard bits.
               prodL32 = s32_shr_s32(prodL32, 4);
               tempL32 = s32_add_s32_s32(tempL32, prodL32);
            }
            // Do the remaining right-shift to get it back to same QFactor as input
             int16 *out_ch_data_ptr = (int16 *)output[outputChIndex];
            *(out_ch_data_ptr + sampleIndex) = s16_saturate_s32(s32_shr_s32(tempL32, Q14_FACTOR - 4));
         }
      }
   }
   else
   {
      // Generate output one channel at a time. This helps in reducing cache thrashing
      for (outputChIndex = 0; outputChIndex < pState->numOutputCh; outputChIndex++)
      {
         // Calculate the starting address for the input channels

         // Matrix coefficients of the row corresponding to the output channel
         pMatrixCoeffL16Q14 = pState->ptrCoeff + pState->numInputCh * outputChIndex;

         for (sampleIndex = 0; sampleIndex < numSamples; sampleIndex++)
         {
            inputStep = &pState->dynState.pInputStepMatrix[outputChIndex * (pState->numInputCh + 1)];
            // output[output channel i][sample] = matrix[output channel i ][input channel j] *
            //                                       input[input channel j][sample]
            // The above product is calculated for all input channels and the products are summed.
            // The data ordering can be exploited to write memory access based on pointers so that
            // the compiler can generate efficient code. For e.g
            // (1) All the output samples for a channel are stored one after the other. Hence in the
            //     above equation, output[output channel i][sample] is just *output[channel]++
            // (2) Similarly for calculating the output samples for a channel, only the factors in
            //     a particular row in the matrix are required. Since C implements matrices row-wise,
            //     we can access these coefficients as matrixCoeff++
            // (3) Input samples of all the channels must be accessed even for calculating output samples
            //     for one channels. So we initialize a local array with starting addresses of all input
            //     channels and increment each one of them as required.
            tempL64 = 0;
            for (inputChIndex = *inputStep; inputChIndex < pState->numInputCh; inputChIndex = *(++inputStep))
            {
               int32 *in_ch_data_ptr = (int32 *)input[inputChIndex];
               prodL64 = s64_mult_s32_s16(*(in_ch_data_ptr + sampleIndex), pMatrixCoeffL16Q14[inputChIndex]);
               // Maximum number of channels is 8, so there are enough bits to avoid overflow
               tempL64 = s64_add_s64_s64(tempL64, prodL64);
            }
            // Do the remaining right-shift to get it back to same QFactor as input
            int32 *out_ch_data_ptr = (int32 *)output[outputChIndex];
            *(out_ch_data_ptr + sampleIndex) = s32_saturate_s64(s64_shl_s64(tempL64, -Q14_FACTOR));
         }
      }
   }
}
