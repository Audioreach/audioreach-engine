/*============================================================================
  @file ChannelMixerLib.c

  Impelementation of the Channel Mixer algorithm. */

/*=========================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
========================================================================= */

#include "audio_basic_op.h"
#include "audio_basic_op_ext.h"
#include "ChannelMixerLib.h"
#include "ChannelMixerRemapRules.h"
#include "audio_divide_qx.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

static void copy_in_out_chan_params(ChMixerStateStruct *pState,
                                    uint32              numInputChannels,
                                    ChMixerChType *     inputChannels,
                                    uint32              numOutputChannels,
                                    ChMixerChType *     outputChannels,
                                    uint32              dataBitWidth);

/*
@brief Normalize the Mixer matrix

@param pState : [in/out] Pointer to the state structure
*/
void ChMixerMatrixNormalize(ChMixerStateStruct *pState)
{
   uint32 outputChIndex, inputChIndex;
   int32  rowTotal[CH_MIXER_MAX_NUM_CH];
   int32  rowTotalMax;
   int32  coefficient;
   int16  shift;

   /* Calculate the total contribution by all input channels for every output channel.*/
   // Calculate for the first output channel and initialize rowTotalMax
   rowTotalMax = 0;
   for (inputChIndex = 0; inputChIndex < pState->numInputCh; inputChIndex++)
      rowTotalMax += pState->ptrCoeff[inputChIndex];

   // Calculate all other channels
   for (outputChIndex = 1; outputChIndex < pState->numOutputCh; outputChIndex++)
   {
      rowTotal[outputChIndex] = 0;

      for (inputChIndex = 0; inputChIndex < pState->numInputCh; inputChIndex++)
         rowTotal[outputChIndex] += pState->ptrCoeff[pState->numInputCh * outputChIndex + inputChIndex];
      // Find maximum
      if (rowTotalMax < rowTotal[outputChIndex])
         rowTotalMax = rowTotal[outputChIndex];
   }

   for (outputChIndex = 0; outputChIndex < pState->numOutputCh; outputChIndex++)
   {
      // Normalize the coefficients for each row
      for (inputChIndex = 0; inputChIndex < pState->numInputCh; inputChIndex++)
      {
         // If coefficient is not zero or if there is only one non-zero coefficient in a row, normalize
         coefficient = (int32)pState->ptrCoeff[pState->numInputCh * outputChIndex + inputChIndex];
         if (coefficient == rowTotalMax)
         {
            // Q14 format of unity
            // temporary change to avoid signal saturation for a case, where none of the input channels actually
            // contribute to output. (only for 8550, generic fix in next target)
            if (0 == rowTotalMax)
            {
               pState->ptrCoeff[pState->numInputCh * outputChIndex + inputChIndex] = 16384 / pState->numInputCh;
            }
            else
            {
               pState->ptrCoeff[pState->numInputCh * outputChIndex + inputChIndex] = 16384;
            }
         }
         else if ((coefficient != 0) && (rowTotalMax != 0))
         {
            coefficient = s32_div_s32_s32_normalized(coefficient, rowTotalMax, &shift);
            // Convert and store the matrix mixing coefficient in Q14 format.
            pState->ptrCoeff[pState->numInputCh * outputChIndex + inputChIndex] =
               s16_saturate_s32(s32_shl_s32_sat(coefficient, (shift - 1)));
         }
      }
   }
}

/*
@brief Calculate input/output channel BitMasks

@param pState : [in] Pointer to the state structure
@param pInputBitMask : [out] input channel bit mask calculated
@param pOutputBitMask : [out] output channel bit mask calcualted
*/
void CalculateInOutChBitMask(ChMixerStateStruct *pState, uint64 *pInputBitMaskArray, uint64 *pOutputBitMaskArray)
{
   uint32        chIndex;
   ChMixerChType remapFromCh, remapToCh;

   for (uint32 cnt = 0; cnt < CH_MIXER_MAX_BITMASK_GROUPS; cnt++)
   {
      pInputBitMaskArray[cnt]  = 0;
      pOutputBitMaskArray[cnt] = 0;
   }

   // Form bit mask of input channels. The bit mask is a 64-bit value with bits
   // corresponding to the input channel types set.
   for (chIndex = 0; chIndex < pState->numInputCh; chIndex++)
   {
      remapFromCh = pState->dynState.pInputChannels[chIndex];
      if (remapFromCh < CH_MIXER_BIT_MASK_SIZE)
      {
         pInputBitMaskArray[ChMixerBitMaskGroup_1to63] |= ((1ULL << remapFromCh));
      }
      else if ((remapFromCh < 2 * CH_MIXER_BIT_MASK_SIZE))
      {
         pInputBitMaskArray[ChMixerBitMaskGroup_64to127] |= ((1ULL << (remapFromCh  & CH_MIXER_BIT_MASK_SIZE_MINUS_ONE)));
      }
      else if ((remapFromCh < 3 * CH_MIXER_BIT_MASK_SIZE))
      {
         pInputBitMaskArray[ChMixerBitMaskGroup_128to191] |= ((1ULL << (remapFromCh & CH_MIXER_BIT_MASK_SIZE_MINUS_ONE)));
      }
   }

   // Form bit mask of output channels. The bit mask is a 64-bit value with bits
   // corresponding to the output channel types set.

   for (chIndex = 0; chIndex < pState->numOutputCh; chIndex++)
   {
      remapToCh = pState->dynState.pOutputChannels[chIndex];
      if (remapToCh < CH_MIXER_BIT_MASK_SIZE)
      {
         pOutputBitMaskArray[ChMixerBitMaskGroup_1to63] |= ((1ULL << remapToCh));
      }
      else if ((remapToCh < 2 * CH_MIXER_BIT_MASK_SIZE))
      {
         pOutputBitMaskArray[ChMixerBitMaskGroup_64to127] |= ((1ULL << (remapToCh & CH_MIXER_BIT_MASK_SIZE_MINUS_ONE)));
      }
      else if ((remapToCh < 3 * CH_MIXER_BIT_MASK_SIZE))
      {
         pOutputBitMaskArray[ChMixerBitMaskGroup_128to191] |= ((1ULL << (remapToCh & CH_MIXER_BIT_MASK_SIZE_MINUS_ONE)));
      }
   }
}

/*
@brief Apply the paired remmaping rules defined

@param pState : [in/out] Pointer to the state structure
*/
void ChMixerApplyPairedRemapRules(ChMixerStateStruct *pState)
{
   uint32 ruleIndex, chIndex;
   uint64 inputBitMask[CH_MIXER_MAX_BITMASK_GROUPS], remapFromChBitMask, remapToChBitMask;
   uint64 outputBitMask[CH_MIXER_MAX_BITMASK_GROUPS];

   CalculateInOutChBitMask(pState, &inputBitMask[0], &outputBitMask[0]);

   for (ruleIndex = 0; ruleIndex < NUM_PAIRED_REMAP_RULES; ruleIndex++)
   {
      // Form the bit mask of the 'Remap from' channels in the rule
      remapFromChBitMask = ((uint64)1 << PairedRemapRulesList[ruleIndex].fromCh1) |
                           ((uint64)1 << PairedRemapRulesList[ruleIndex].fromCh2);
      // Form the bit mask of the 'Remap to' channels in the rule
      remapToChBitMask =
         ((uint64)1 << PairedRemapRulesList[ruleIndex].toCh1) | ((uint64)1 << PairedRemapRulesList[ruleIndex].toCh2);
      // Check if the rule is applicable to this input/output channel configuration.
      if (((outputBitMask[ChMixerBitMaskGroup_1to63] & remapToChBitMask) == remapToChBitMask) &&
          ((inputBitMask[ChMixerBitMaskGroup_1to63] & remapFromChBitMask) == remapFromChBitMask) &&
          ((outputBitMask[ChMixerBitMaskGroup_1to63] & remapFromChBitMask) != remapFromChBitMask))
      {
         // Remap the channel
         for (chIndex = 0; chIndex < pState->numInputCh; chIndex++)
         {
            if (pState->dynState.pInputChannels[chIndex] == PairedRemapRulesList[ruleIndex].fromCh1)
            {
               pState->dynState.pInputChannels[chIndex] = PairedRemapRulesList[ruleIndex].toCh1;
               continue;
            }
            if (pState->dynState.pInputChannels[chIndex] == PairedRemapRulesList[ruleIndex].fromCh2)
            {
               pState->dynState.pInputChannels[chIndex] = PairedRemapRulesList[ruleIndex].toCh2;
               continue;
            }
         }
      }
   }
}
/*
@brief Apply the paired remmaping rules defined

@param pState : [in/out] Pointer to the state structure
*/
ChMixerResultType ChMixerApplySingleChRemapRules(ChMixerStateStruct *pState)
{
   int32         ruleIndex, remapToChIndex;
   uint32        inputChIndex, outputChIndex;
   ChMixerChType remapFromCh, remapToCh;
   uint64        inputBitMask[CH_MIXER_MAX_BITMASK_GROUPS], outputBitMask[CH_MIXER_MAX_BITMASK_GROUPS];
   uint64        remapToChBitMask, remapfromChBitMask, is_input_present_in_output;
   int32         index;

   CalculateInOutChBitMask(pState, &inputBitMask[0], &outputBitMask[0]);
   // Initialize the mixer matrix coefficients.
   for (inputChIndex = 0; inputChIndex < pState->numInputCh; inputChIndex++)
      for (outputChIndex = 0; outputChIndex < pState->numOutputCh; outputChIndex++)
      {
         index                   = pState->numInputCh * outputChIndex + inputChIndex;
         pState->ptrCoeff[index] = 0;
      }

   // Warning if output has custom channel type and input doesn't have any custom channel types
   for (outputChIndex = 0; outputChIndex < pState->numOutputCh; outputChIndex++)
   {
      remapToCh                   = pState->dynState.pOutputChannels[outputChIndex];
      bool_t custom_input_ch_type = FALSE;

      // output has custom channel type
      if (remapToCh > CH_MIXER_PCM_CH_RSD)
      {
         for (inputChIndex = 0; inputChIndex < pState->numInputCh; inputChIndex++)
         {
            remapFromCh = pState->dynState.pInputChannels[inputChIndex];

            if (remapFromCh > CH_MIXER_PCM_CH_RSD)
            {
               custom_input_ch_type = TRUE;
               break;
            }
         }

         if (!custom_input_ch_type)
         {
            AR_MSG(DBG_HIGH_PRIO,
                   "CHMIXER Lib: Warning: No coefficient array set, hence setting chmixer coefficient"
                   "corresponding to custom output channel type %d to zero",
                   remapToCh);
         }
      }
   }

   // For each input channel, identify the mapping that must be done.
   for (inputChIndex = 0; inputChIndex < pState->numInputCh; inputChIndex++)
   {
      remapFromCh                = pState->dynState.pInputChannels[inputChIndex];
      is_input_present_in_output = 0;
      if (pState->dynState.pInputChannels[inputChIndex] < CH_MIXER_BIT_MASK_SIZE)
      {
         remapfromChBitMask         = (1ULL << remapFromCh);
         is_input_present_in_output = (remapfromChBitMask & outputBitMask[ChMixerBitMaskGroup_1to63]);
      }
      else if (pState->dynState.pInputChannels[inputChIndex] < 2 * CH_MIXER_BIT_MASK_SIZE)
      {
         remapfromChBitMask         = (1ULL << (remapFromCh & CH_MIXER_BIT_MASK_SIZE_MINUS_ONE));
         is_input_present_in_output = ((remapfromChBitMask) & outputBitMask[ChMixerBitMaskGroup_64to127]);
      }
      else if (pState->dynState.pInputChannels[inputChIndex] < 3 * CH_MIXER_BIT_MASK_SIZE)
      {
         remapfromChBitMask         = (1ULL << (remapFromCh & CH_MIXER_BIT_MASK_SIZE_MINUS_ONE));
         is_input_present_in_output = ((remapfromChBitMask)&outputBitMask[ChMixerBitMaskGroup_128to191]);
      }
      if (is_input_present_in_output)
      {
         // If input channel is present in output, just copy it over (i.e) set matrix coefficient to unity
         for (outputChIndex = 0; outputChIndex < pState->numOutputCh; outputChIndex++)
         {
            remapToCh = pState->dynState.pOutputChannels[outputChIndex];
            if (remapFromCh == remapToCh)
            {
               // Identify the index of the channel in the output
               pState->ptrCoeff[pState->numInputCh * outputChIndex + inputChIndex] = 16384;
               break;
            }
         }
      }
      else
      {
         // If input is a custom channel type and output doesn't have the same channel type (checked above)
         if (remapFromCh > CH_MIXER_PCM_CH_RSD)
         {
            bool_t output_has_custom_ch_type = FALSE;

            for (outputChIndex = 0; outputChIndex < pState->numOutputCh; outputChIndex++)
            {
               remapToCh = pState->dynState.pOutputChannels[outputChIndex];
               if (remapToCh > CH_MIXER_PCM_CH_RSD)
               {
                  output_has_custom_ch_type = TRUE;
                  break;
               }
            }

            /* If there is even one custom channel type in output,
               implies inp and out have custom channel types which dont match and we error out */
            if (output_has_custom_ch_type)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "CHMIXER Lib: Error: No coefficient array found for input to ouput custom "
                      "channel type mapping");
               return CH_MIXER_ENOT_SUPPORTED;
            }
            else
            {
               // Warning if input has custom ch type but output doesnt
               AR_MSG(DBG_HIGH_PRIO,
                      "CHMIXER Lib: Warning: No coefficient array set, hence setting coefficient "
                      "corresponding to custom input channel type %d to zero",
                      remapFromCh);
            }
         }

         for (ruleIndex = 0; ruleIndex < SingleChRemapRulesList[remapFromCh].numRulesPerFromCh; ruleIndex++)
         {
            // Check if this rule for a particular "Remap From" channel is applicable.
            // Find the "Remap To" channel bit mask of the rule and see if the bit mask exists
            // in the outputBitMask
            remapToChBitMask = 0;
            for (remapToChIndex = 0; remapToChIndex < SingleChRemapRulesList[remapFromCh].map[ruleIndex].numToChPerRule;
                 remapToChIndex++)
            {
               remapToChBitMask |=
                  ((uint64)1 << SingleChRemapRulesList[remapFromCh].map[ruleIndex].outMap[remapToChIndex].remapToCh);
            }
            // If the "Remap To" channel bit mask is present in the output, this rule is applicable.
            if ((outputBitMask[ChMixerBitMaskGroup_1to63] & remapToChBitMask) == remapToChBitMask)
            {
               // This rule is applicable. So copy the mixing parameters
               for (remapToChIndex = 0;
                    remapToChIndex < SingleChRemapRulesList[remapFromCh].map[ruleIndex].numToChPerRule;
                    remapToChIndex++)
               {
                  for (outputChIndex = 0; outputChIndex < pState->numOutputCh; outputChIndex++)
                  {
                     if (SingleChRemapRulesList[remapFromCh].map[ruleIndex].outMap[remapToChIndex].remapToCh ==
                         pState->dynState.pOutputChannels[outputChIndex])
                     {
                        // Identify the index of the channel in the output
                        pState->ptrCoeff[pState->numInputCh * outputChIndex + inputChIndex] =
                           SingleChRemapRulesList[remapFromCh].map[ruleIndex].outMap[remapToChIndex].mixingParam;
                        break;
                     }
                  }
               }
               // No more rules can be applied for this input channel.
               break;
            }
         }
      }
   }

   return CH_MIXER_SUCCESS;
}

/*
@brief Setup the Channel mixer matrix

@param pState : [in] Pointer to the state structure
*/

ChMixerResultType ChMixerSetupCoefficients(ChMixerStateStruct *pState)
{
   uint32 outputChIndex, inputChIndex;
   int16 *pMatrixCoeffL16Q14;
   // int16 *pMatrixCoeffStartL16Q14;
   int8 * inputStep;
   uint64 inputBitMask[CH_MIXER_MAX_BITMASK_GROUPS];
   uint64 outputBitMask[CH_MIXER_MAX_BITMASK_GROUPS];
   pState->num_active_coeff = 0;

   // Recalculate the input and output Bit Mask
   CalculateInOutChBitMask(pState, &inputBitMask[0], &outputBitMask[0]);

   if (!pState->isCustomCoeffValid)
   {
      pState->ptrCoeff = pState->dynState.pMixerMatrixL16Q14;

      // Normalize the matrix coefficients so that the sum on a row is 1 (16384 in Q14). This means the total
      // contribution of all the input channels to an output channel is 1 (no gain).
      ChMixerMatrixNormalize(pState);
   }
   else
   {
      pState->ptrCoeff = pState->ptrCustomCoeffset;
   }

   if ((inputBitMask[ChMixerBitMaskGroup_1to63] != outputBitMask[ChMixerBitMaskGroup_1to63]) ||
       (inputBitMask[ChMixerBitMaskGroup_64to127] != outputBitMask[ChMixerBitMaskGroup_64to127]) ||
       (inputBitMask[ChMixerBitMaskGroup_128to191] != outputBitMask[ChMixerBitMaskGroup_128to191]) ||
       (pState->isCustomCoeffValid))
   {
      pState->isTrivialCopy = 0;
      // Some of the coefficients in a row of the mixer matrix might be zero. This
      // means the corresponding input channels do not contribute to the output channels
      // The following piece of code, identifies which input channels contribute to which
      // output channels.
      for (outputChIndex = 0; outputChIndex < pState->numOutputCh; outputChIndex++)
      {
         inputStep          = &pState->dynState.pInputStepMatrix[outputChIndex * (pState->numInputCh + 1)];
         pMatrixCoeffL16Q14 = pState->ptrCoeff + pState->numInputCh * outputChIndex;
         for (inputChIndex = 0; inputChIndex < pState->numInputCh; inputChIndex++)
         {
            if (pMatrixCoeffL16Q14[inputChIndex] != 0)
            {
               *inputStep++ = (int8)inputChIndex;
               pState->num_active_coeff++;
            }
         }
         // This is the end marker for input pointer advance.
         *inputStep = (int8)pState->numInputCh;
      }
   }
   else
   {
      pState->isTrivialCopy = 1;
      // Output is a trivial remap of the input. Identify what the remapping
      // is and store it in the first row of the pState->inputStep matrix.
      inputStep = &pState->dynState.pInputStepMatrix[0];
      for (outputChIndex = 0; (outputChIndex < pState->numOutputCh) && (outputChIndex < CH_MIXER_MAX_NUM_CH);
           outputChIndex++)
      {
         for (inputChIndex = 0; inputChIndex < pState->numInputCh; inputChIndex++)
         {
            if (pState->dynState.pOutputChannels[outputChIndex] == pState->dynState.pInputChannels[inputChIndex])
            {
               *inputStep++ = inputChIndex;
               pState->num_active_coeff++;
               break;
            }
         }
      }
   }
   return CH_MIXER_SUCCESS;
}

/*
@brief Return the num coefficients in inputstep matrix for kpps calculations.
*/
uint32 ChMixerGetNumActiveCoefficients(ChMixerStateStruct *pState)
{
   return pState->num_active_coeff;
}

/*
@brief return the memory requirements of channel mixer library
*/
uint32 ChMixerGetdynMemoryReq(uint32 numInputChannels, uint32 numOutputChannels)
{
   uint32 dynMemorySize = 0;

   // memory for inputChannels
   dynMemorySize = CH_MIXER_ALIGN_8_BYTE(sizeof(ChMixerChType) * (numInputChannels));
   // memory for outputChannels
   dynMemorySize += CH_MIXER_ALIGN_8_BYTE(sizeof(ChMixerChType) * (numOutputChannels));
   // memory for mixerMatrixL16Q14
   dynMemorySize += CH_MIXER_ALIGN_8_BYTE(sizeof(int16) * (numInputChannels * numOutputChannels));
   // memory for inputStepMatrix
   dynMemorySize += CH_MIXER_ALIGN_8_BYTE(sizeof(int8) * (numOutputChannels * (numInputChannels + 1)));

   AR_MSG(DBG_HIGH_PRIO, "CHMIXER Lib: dynamic_mem size %d", dynMemorySize);
   return dynMemorySize;
}

/*
@brief Return the size of state structure required to operate one instance of
Channel mixer.

@param pSize: [out] Pointer to memory location where size of structure must be
updated.
*/
void ChMixerGetInstanceSize(uint32 *pSize, uint32 numInputChannels, uint32 numOutputChannels)
{
   *pSize = CH_MIXER_ALIGN_8_BYTE(sizeof(ChMixerStateStruct));
   *pSize += CH_MIXER_ALIGN_8_BYTE(ChMixerGetdynMemoryReq(numInputChannels, numOutputChannels));
   AR_MSG(DBG_HIGH_PRIO, "CHMIXER Lib: total_mem size %d", *pSize);
}

/*
@brief initialize the dynamic state structure variables
*/
ChMixerResultType ChMixerInitMemory(void *mem_ptr, int32 mem_size, uint32 numInputChannels, uint32 numOutputChannels)
{
   uint32 getLibSize   = 0;
   int8 * base_mem_ptr = NULL;
   ChMixerGetInstanceSize(&getLibSize, numInputChannels, numOutputChannels);
   if (getLibSize > mem_size)
   {
      return CH_MIXER_ENOT_SUPPORTED;
   }
   base_mem_ptr = (int8 *)mem_ptr;

   ChMixerStateStruct *pState = (ChMixerStateStruct *)base_mem_ptr;

   base_mem_ptr += CH_MIXER_ALIGN_8_BYTE(sizeof(ChMixerStateStruct));
   pState->dynState.pInputChannels = (ChMixerChType *)base_mem_ptr;

   // offset by memory for inputChannels
   base_mem_ptr += CH_MIXER_ALIGN_8_BYTE(sizeof(ChMixerChType) * (numInputChannels));
   pState->dynState.pOutputChannels = (ChMixerChType *)base_mem_ptr;

   // offset by memory for outputChannels
   base_mem_ptr += CH_MIXER_ALIGN_8_BYTE(sizeof(ChMixerChType) * (numOutputChannels));
   pState->dynState.pMixerMatrixL16Q14 = (int16 *)base_mem_ptr;

   // offset by  memory for mixerMatrixL16Q14
   base_mem_ptr += CH_MIXER_ALIGN_8_BYTE(sizeof(int16) * (numInputChannels * numOutputChannels));
   pState->dynState.pInputStepMatrix = (int8 *)base_mem_ptr;

   return CH_MIXER_SUCCESS;
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

ChMixerResultType ChMixerInitialize(void *         pCMState,
                                    uint32         mem_size,
                                    uint32         numInputChannels,
                                    ChMixerChType *inputChannels,
                                    uint32         numOutputChannels,
                                    ChMixerChType *outputChannels,
                                    uint32         dataBitWidth)
{
   ChMixerStateStruct *pState = (ChMixerStateStruct *)pCMState;

   if (ChMixerInitMemory(pCMState, mem_size, numInputChannels, numOutputChannels))
   {
      return CH_MIXER_EBAD_PARAM;
   }

   // Store variables for future reference
   copy_in_out_chan_params(pState, numInputChannels, inputChannels, numOutputChannels, outputChannels, dataBitWidth);
   pState->isCustomCoeffValid = 0;

   pState->ptrCoeff = pState->dynState.pMixerMatrixL16Q14;

   // Apply paired input channel remapping rules first
   // For e.g, Remap (LS,LB) pair channels in input to (LB,RLC) in output
   ChMixerApplyPairedRemapRules(pState);

   // Apply channel remapping rules for single channel
   ChMixerApplySingleChRemapRules(pState);

   ChMixerSetupCoefficients(pState);

   return CH_MIXER_SUCCESS;
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
ChMixerResultType ChMixerSetParam(void *         pCMState,
                                  uint32         mem_size,
                                  uint32         numInputChannels,
                                  ChMixerChType *inputChannels,
                                  uint32         numOutputChannels,
                                  ChMixerChType *outputChannels,
                                  uint32         dataBitWidth,
                                  void *         pCustomCoeffSet)
{

   ChMixerResultType   result = CH_MIXER_SUCCESS;
   ChMixerStateStruct *pState = (ChMixerStateStruct *)pCMState;

   if (NULL != pCustomCoeffSet)
   {
      if (ChMixerInitMemory(pCMState, mem_size, numInputChannels, numOutputChannels))
      {
         return CH_MIXER_EBAD_PARAM;
      }
      // Store variables for future reference
      copy_in_out_chan_params(pState, numInputChannels, inputChannels, numOutputChannels, outputChannels, dataBitWidth);
      pState->isCustomCoeffValid = 1;
      pState->ptrCustomCoeffset  = (int16 *)pCustomCoeffSet;

      result = ChMixerSetupCoefficients(pState);
   }
   else
   {
      result = ChMixerInitialize(pCMState,
                                 mem_size,
                                 numInputChannels,
                                 inputChannels,
                                 numOutputChannels,
                                 outputChannels,
                                 dataBitWidth);
   }
   return result;
}

static void copy_in_out_chan_params(ChMixerStateStruct *pState,
                                    uint32              numInputChannels,
                                    ChMixerChType *     inputChannels,
                                    uint32              numOutputChannels,
                                    ChMixerChType *     outputChannels,
                                    uint32              dataBitWidth)
{
   int32 chIndex;
   pState->numInputCh    = numInputChannels;
   pState->numOutputCh   = numOutputChannels;
   pState->dataBitWidth  = dataBitWidth;
   pState->isTrivialCopy = 0;

   for (chIndex = 0; chIndex < numInputChannels; chIndex++)
   {
      pState->dynState.pInputChannels[chIndex] = inputChannels[chIndex];
   }
   for (chIndex = 0; chIndex < numOutputChannels; chIndex++)
   {
      pState->dynState.pOutputChannels[chIndex] = outputChannels[chIndex];
   }
}

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */
