/* =========================================================================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause
   ========================================================================== */

/* =========================================================================
   FILE NAME: drc_lib_asm.S
   DESCRIPTION:
   This file contains the assembly equivalent C code of drc process functions.
  =========================================================================  */

#include "drc_lib.h"
#include "CDrcLib.h"
#include <string.h>
#include "audio_basic_op_ext.h"
#include "audio_log10.h"
#include "audio_exp10.h"
#ifdef DRCLIB_OPT

const static int16 shift_factor_arr[] = { 0, 0, -1, -15, -2, -15, -15, -15, -3 };

const static uint16 mult_factor_arr[] = { 1, 1, 1, ONE_BY_3_Q15, 1, ONE_BY_5_Q15, ONE_BY_6_Q15, ONE_BY_7_Q15, 1 };

DRC_RESULT  ProcessBP16(drc_state_struct_t * pState,
                        drc_static_struct_t *pStatic,
                        uint32               nSamplePerChannel,
                        uint32               delayBuffSize,
                        int8 **              pOutPtr,
                        int8 **              pInPtr);
DRC_RESULT  ProcessBP32(drc_state_struct_t * pState,
                        drc_static_struct_t *pStatic,
                        uint32               nSamplePerChannel,
                        uint32               delayBuffSize,
                        int8 **              pOutPtr,
                        int8 **              pInPtr);
DRC_RESULT  ProcessMono16(drc_lib_t *pDrcLib,
                          uint16     negativeDrcTavUL16Q16,
                          uint32     nSamplePerChannel,
                          uint32     delayBuffSize,
                          int8 **    pOutPtr,
                          int8 **    pInPtr);
DRC_RESULT  ProcessMono32(drc_lib_t *pDrcLib,
                          uint16     negativeDrcTavUL16Q16,
                          uint32     nSamplePerChannel,
                          uint32     delayBuffSize,
                          int8 **    pOutPtr,
                          int8 **    pInPtr);
DRC_RESULT  ProcessMC16Linked(drc_lib_t *pDrcLib,
                              uint16     negativeDrcTavUL16Q16,
                              uint32     nSamplePerChannel,
                              uint32     delayBuffSize,
                              int8 **    pOutPtr,
                              int8 **    pInPtr);
DRC_RESULT  ProcessMC16Unlinked(drc_lib_t *pDrcLib,
                                uint16     negativeDrcTavUL16Q16,
                                uint32     nSamplePerChannel,
                                uint32     delayBuffSize,
                                int8 **    pOutPtr,
                                int8 **    pInPtr);
DRC_RESULT  ProcessMC32Linked(drc_lib_t *pDrcLib,
                              uint16     negativeDrcTavUL16Q16,
                              uint32     nSamplePerChannel,
                              uint32     delayBuffSize,
                              int8 **    pOutPtr,
                              int8 **    pInPtr);
DRC_RESULT  ProcessMC32Unlinked(drc_lib_t *pDrcLib,
                                uint16     negativeDrcTavUL16Q16,
                                uint32     nSamplePerChannel,
                                uint32     delayBuffSize,
                                int8 **    pOutPtr,
                                int8 **    pInPtr);
static void compute_drc_gain(drc_state_struct_t *pState, drc_config_t *drc_config_ptr, uint32 gainNum);

/*======================================================================

FUNCTION      ProcessBP16

DESCRIPTION   DRC processing for 16bit Bypass mode

PARAMETERS    pState: [in] pointer to the state structure
              pStatic: [in] pointer to the config structure
              nSamplePerChannel: [in] number of samples per channel
              delayBuffSize: [in] size of delay buffer
              pOutPtr: [out] pointer to the output data
           pInPtr: [in] pointer to the input data

RETURN VALUE  Failure or Success

SIDE EFFECTS  None.

======================================================================*/
DRC_RESULT ProcessBP16(drc_state_struct_t * pState,
                       drc_static_struct_t *pStatic,
                       uint32               nSamplePerChannel,
                       uint32               delayBuffSize,
                       int8 **              pOutPtr,
                       int8 **              pInPtr)
{
   uint32 i, channelNo;
   uint32 process_index;
   uint32 input_index;
   int16 *delayBuffer;
   int16 *pInput, *pOutput;

   for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
   {
      delayBuffer = (int16 *)pState->delayBuffer[channelNo];
      pInput      = (int16 *)pInPtr[channelNo];
      pOutput     = (int16 *)pOutPtr[channelNo];

      process_index = pState->processIndex;
      input_index   = pState->inputIndex;

      for (i = 0; i < nSamplePerChannel; i++)
      {
         delayBuffer[input_index] = pInput[i];
         input_index++;
         process_index = s32_modwrap_s32_u32(process_index, delayBuffSize);

         pOutput[i] = delayBuffer[process_index];
         process_index++;
         input_index = s32_modwrap_s32_u32(input_index, delayBuffSize);
      }
   }
   process_index        = s32_modwrap_s32_u32(process_index, delayBuffSize);
   pState->processIndex = process_index;
   pState->inputIndex   = input_index;

   return DRC_SUCCESS;
}

/*======================================================================

FUNCTION      ProcessBP32

DESCRIPTION   DRC processing for 32bit Bypass mode

PARAMETERS    pState: [in] pointer to the state structure
              pStatic: [in] pointer to the config structure
              nSamplePerChannel: [in] number of samples per channel
              delayBuffSize: [in] size of delay buffer
              pOutPtr: [out] pointer to the output data
           pInPtr: [in] pointer to the input data

RETURN VALUE  Failure or Success

SIDE EFFECTS  None.

======================================================================*/

DRC_RESULT ProcessBP32(drc_state_struct_t * pState,
                       drc_static_struct_t *pStatic,
                       uint32               nSamplePerChannel,
                       uint32               delayBuffSize,
                       int8 **              pOutPtr,
                       int8 **              pInPtr)
{
   uint32 i, channelNo;
   uint32 process_index;
   uint32 input_index;
   int32 *delayBuffer;
   int32 *pInput, *pOutput;

   for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
   {
      delayBuffer = (int32 *)pState->delayBuffer[channelNo];
      pInput      = (int32 *)pInPtr[channelNo];
      pOutput     = (int32 *)pOutPtr[channelNo];

      process_index = pState->processIndex;
      input_index   = pState->inputIndex;

      for (i = 0; i < nSamplePerChannel; i++)
      {
         delayBuffer[input_index] = pInput[i];
         input_index++;
         process_index = s32_modwrap_s32_u32(process_index, delayBuffSize);

         pOutput[i] = delayBuffer[process_index];
         process_index++;
         input_index = s32_modwrap_s32_u32(input_index, delayBuffSize);
      }
   }
   process_index        = s32_modwrap_s32_u32(process_index, delayBuffSize);
   pState->processIndex = process_index;
   pState->inputIndex   = input_index;

   return DRC_SUCCESS;
}

/*======================================================================

FUNCTION      ProcessMono16

DESCRIPTION   DRC processing for 16bit single channel case

PARAMETERS    pDrcLib: [in] pointer to the library structure
              negativeDrcTavUL16Q16: [in] precalculated negative time constant
              nSamplePerChannel: [in] number of samples per channel
              delayBuffSize: [in] size of delay buffer
              pOutPtr: [out] pointer to the output data
           pInPtr: [in] pointer to the input data

RETURN VALUE  Failure or Success

SIDE EFFECTS  None.

======================================================================*/

DRC_RESULT ProcessMono16(drc_lib_t *pDrcLib,
                         uint16     negativeDrcTavUL16Q16,
                         uint32     nSamplePerChannel,
                         uint32     delayBuffSize,
                         int8 **    pOutPtr,
                         int8 **    pInPtr)
{
   uint32 i;
   int16 *output;
   int16  processData;
   int16 *delayBuffer;
   int32  tempL32;
   int64  tempOutL64;
   int32  currDrcRmsL32;

   drc_lib_mem_t *     pDrcLibMem     = (drc_lib_mem_t *)pDrcLib->lib_mem_ptr;
   drc_state_struct_t *pState         = pDrcLibMem->drc_state_struct_ptr;
   drc_config_t *      drc_config_ptr = pDrcLibMem->drc_config_ptr;

   delayBuffer = (int16 *)pState->delayBuffer[0];
   output      = (int16 *)(pOutPtr[0]);

   for (i = 0; i < nSamplePerChannel; i++)
   {
      delayBuffer[pState->inputIndex] = ((int16 *)pInPtr[0])[i];

      //------------ Compute long term RMS of DRC for mono case: ------------

      currDrcRmsL32 = s32_mult_s16_s16(delayBuffer[pState->inputIndex], delayBuffer[pState->inputIndex]);

      // current_drcRms = previous_drcRms*(1-drcTav) + drcTav * x[n] ^ 2
      pState->rmsStateL32[0] =
         s32_extract_s64_l(s64_add_s64_s64(s64_mult_s32_u16_shift(currDrcRmsL32, drc_config_ptr->rmsTavUL16Q16, 0),
                                           s64_mult_s32_u16_shift(pState->rmsStateL32[0], negativeDrcTavUL16Q16, 0)));

      //---------------- Update gain computation only if at downSampleLevels --------
      if (0 == pState->downSampleCounter)
      {
         pState->downSampleCounter = drc_config_ptr->downSampleLevel;
         // compute the current INPUT RMS in dB (log domain)
         // log10_fixed: input L32Q0 format, output Q23 format
         if (pState->rmsStateL32[0] != 0)
         {
            pState->drcRmsDBL32Q23[0] =
               log10_fixed(pState->rmsStateL32[0]); // log10_fixed is 10*log10(.) in fixed point
         }
         else
         {
            pState->drcRmsDBL32Q23[0] = MIN_RMS_DB_L32Q23;
         }

         // ----------- Compute DRC gain ------------------------
         compute_drc_gain(pState, drc_config_ptr, 0);
      }
      pState->downSampleCounter--;

      // ----------- Apply DRC gain --------------------------
      // delayBuffer = (int16 *) (pState->delayBuffer[0]);
      processData = delayBuffer[pState->processIndex];
      tempL32     = s32_saturate_s64(s64_mult_s32_s16_shift(pState->gainUL32Q15[0],
                                                        processData,
                                                        1)); // apply gain and output has same QFactor as input

      if (drc_config_ptr->makeupGainUL16Q12 != MAKEUPGAIN_UNITY) // Implement only non-unity gain
      {
         // Multiply output with the shift normalized makeup gain
         tempOutL64 = s64_mult_s32_u16_shift(tempL32, drc_config_ptr->makeupGainUL16Q12, 4);
         tempL32    = s32_saturate_s64(tempOutL64);
      }

      // output results
      output[i] = s16_saturate_s32(tempL32);

      // Check if Delay buffer reaches the cirulary bundary
      pState->processIndex++;
      pState->inputIndex++;

      pState->processIndex = s32_modwrap_s32_u32(pState->processIndex, delayBuffSize);
      pState->inputIndex   = s32_modwrap_s32_u32(pState->inputIndex, delayBuffSize);
   }

   return DRC_SUCCESS;
}

/*======================================================================

FUNCTION      ProcessMono32

DESCRIPTION   DRC processing for 32bit single channel case

PARAMETERS    pDrcLib: [in] pointer to the library structure
              negativeDrcTavUL16Q16: [in] precalculated negative time constant
              nSamplePerChannel: [in] number of samples per channel
              delayBuffSize: [in] size of delay buffer
              pOutPtr: [out] pointer to the output data
           pInPtr: [in] pointer to the input data

RETURN VALUE  Failure or Success

SIDE EFFECTS  None.

======================================================================*/

DRC_RESULT ProcessMono32(drc_lib_t *pDrcLib,
                         uint16     negativeDrcTavUL16Q16,
                         uint32     nSamplePerChannel,
                         uint32     delayBuffSize,
                         int8 **    pOutPtr,
                         int8 **    pInPtr)
{
   uint32 i;
   int32 *output;
   int32  processData;
   int32 *delayBuffer;
   int32  tempL32;
   int64  currDrcRmsL64, tempOutL64;
   int32  currDrcRmsL32;

   drc_lib_mem_t *     pDrcLibMem     = (drc_lib_mem_t *)pDrcLib->lib_mem_ptr;
   drc_state_struct_t *pState         = pDrcLibMem->drc_state_struct_ptr;
   drc_config_t *      drc_config_ptr = pDrcLibMem->drc_config_ptr;

   delayBuffer = (int32 *)pState->delayBuffer[0];
   output      = (int32 *)(pOutPtr[0]);

   for (i = 0; i < nSamplePerChannel; i++)
   {

      delayBuffer[pState->inputIndex] = ((int32 *)pInPtr[0])[i];
      //-------- Compute long term input RMS for DRC ---------

      // x[n] ^ 2
      currDrcRmsL64 = s64_mult_s32_s32(delayBuffer[pState->inputIndex], delayBuffer[pState->inputIndex]);
      // Right shift to get to the same Q-factor as in the 16-bits data width case.
      currDrcRmsL32 = s32_saturate_s64(s64_shl_s64(currDrcRmsL64, s16_shl_s16(s16_sub_s16_s16(Q15, Q27), ONE)));

      // current_drcRms = previous_drcRms*(1-drcTav) + drcTav * x[n] ^ 2
      pState->rmsStateL32[0] =
         s32_extract_s64_l(s64_add_s64_s64(s64_mult_s32_u16_shift(currDrcRmsL32, drc_config_ptr->rmsTavUL16Q16, 0),
                                           s64_mult_s32_u16_shift(pState->rmsStateL32[0], negativeDrcTavUL16Q16, 0)));

      //---------------- Update gain computation only if at downSampleLevels --------
      if (0 == pState->downSampleCounter)
      {
         pState->downSampleCounter = drc_config_ptr->downSampleLevel;
         // compute the current INPUT RMS in dB (log domain)
         // log10_fixed: input L32Q0 format, output Q23 format
         if (pState->rmsStateL32[0] != 0)
         {
            pState->drcRmsDBL32Q23[0] =
               log10_fixed(pState->rmsStateL32[0]); // log10_fixed is 10*log10(.) in fixed point
         }
         else
         {
            pState->drcRmsDBL32Q23[0] = MIN_RMS_DB_L32Q23;
         }

         // ----------- Compute DRC gain ------------------------
         compute_drc_gain(pState, drc_config_ptr, 0);
      }
      pState->downSampleCounter--;

      // ---------- Apply DRC gain ----------------------
      processData = delayBuffer[pState->processIndex];

      tempOutL64 = s64_mult_s32_s32_shift(processData,
                                          pState->gainUL32Q15[0],
                                          17); // apply gain and output has same QFactor as input
      tempL32    = s32_saturate_s64(tempOutL64);

      // apply make up gain if needed
      if (drc_config_ptr->makeupGainUL16Q12 != MAKEUPGAIN_UNITY) // Implement only non-unity gain
      {
         // Multiply output with the shift normalized makeup gain
         tempOutL64 = s64_mult_s32_u16_shift(tempL32, drc_config_ptr->makeupGainUL16Q12, 4);
         tempL32    = s32_saturate_s64(tempOutL64);
      }

      // output results

      output[i] = tempL32;

      // Check if Delay buffer reaches the cirulary bundary
      pState->processIndex++;
      pState->inputIndex++;

      pState->processIndex = s32_modwrap_s32_u32(pState->processIndex, delayBuffSize);
      pState->inputIndex   = s32_modwrap_s32_u32(pState->inputIndex, delayBuffSize);
   }

   return DRC_SUCCESS;
}

/*======================================================================

FUNCTION      ProcessMC16Linked

DESCRIPTION   DRC processing for 16bit multichannel linked case

PARAMETERS    pDrcLib: [in] pointer to the library structure
              negativeDrcTavUL16Q16: [in] precalculated negative time constant
              nSamplePerChannel: [in] number of samples per channel
              delayBuffSize: [in] size of delay buffer
              pOutPtr: [out] pointer to the output data
           pInPtr: [in] pointer to the input data

RETURN VALUE  Failure or Success

SIDE EFFECTS  None.

======================================================================*/

DRC_RESULT ProcessMC16Linked(drc_lib_t *pDrcLib,
                             uint16     negativeDrcTavUL16Q16,
                             uint32     nSamplePerChannel,
                             uint32     delayBuffSize,
                             int8 **    pOutPtr,
                             int8 **    pInPtr)
{

   uint32 i, channelNo;
   int16 *output;
   int16  processData;
   int16 *delayBuffer;
   int32  tempL32;
   int64  tempOutL64;
   int32  currDrcRmsL32;
   int64  squareInputL64;

   drc_lib_mem_t *      pDrcLibMem     = (drc_lib_mem_t *)pDrcLib->lib_mem_ptr;
   drc_state_struct_t * pState         = pDrcLibMem->drc_state_struct_ptr;
   drc_static_struct_t *pStatic        = pDrcLibMem->drc_static_struct_ptr;
   drc_config_t *       drc_config_ptr = pDrcLibMem->drc_config_ptr;

   const int16  shift_factor = shift_factor_arr[pStatic->num_channel];
   const uint16 mult_factor  = mult_factor_arr[pStatic->num_channel];

   for (i = 0; i < nSamplePerChannel; i++)
   {
      squareInputL64 = 0;

      for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
      {
         delayBuffer                     = (int16 *)pState->delayBuffer[channelNo];
         delayBuffer[pState->inputIndex] = ((int16 *)pInPtr[channelNo])[i];
         //-------- Compute long term input RMS for DRC ---------
         squareInputL64 = s64_add_s64_s64(squareInputL64,
                                          s64_mult_s16_s16_shift(delayBuffer[pState->inputIndex],
                                                                 delayBuffer[pState->inputIndex],
                                                                 0));
      }

      // calculate squareInputL64 = squareInputL64/pDrcLib->drc_static_struct.num_channel
      squareInputL64 = s64_shl_s64(squareInputL64, shift_factor);
      if (mult_factor != 1)
      {
         squareInputL64 = s64_mult_s32_s16(s32_saturate_s64(squareInputL64), mult_factor);
      }

      currDrcRmsL32 = s32_saturate_s64(squareInputL64);

      // current_drcRms = previous_drcRms*(1-drcTav) + drcTav * x[n] ^ 2
      pState->rmsStateL32[0] =
         s32_extract_s64_l(s64_add_s64_s64(s64_mult_s32_u16_shift(currDrcRmsL32, drc_config_ptr->rmsTavUL16Q16, 0),
                                           s64_mult_s32_u16_shift(pState->rmsStateL32[0], negativeDrcTavUL16Q16, 0)));

      //---------------- Update gain computation only if at downSampleLevels --------
      if (0 == pState->downSampleCounter)
      {
         pState->downSampleCounter = drc_config_ptr->downSampleLevel;
         // compute the current INPUT RMS in dB (log domain)
         // log10_fixed: input L32Q0 format, output Q23 format
         if (pState->rmsStateL32[0] != 0)
         {
            pState->drcRmsDBL32Q23[0] =
               log10_fixed(pState->rmsStateL32[0]); // log10_fixed is 10*log10(.) in fixed point
         }
         else
         {
            pState->drcRmsDBL32Q23[0] = MIN_RMS_DB_L32Q23;
         }

         // ----------- Compute DRC gain ------------------------
         compute_drc_gain(pState, drc_config_ptr, 0);
      }
      pState->downSampleCounter--;

      // ---------- Apply DRC gain ----------------------
      for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
      {
         delayBuffer = (int16 *)(pState->delayBuffer[channelNo]);
         output      = (int16 *)(pOutPtr[channelNo]);
         processData = delayBuffer[pState->processIndex];

         tempL32 = s32_saturate_s64(s64_mult_s32_s16_shift(pState->gainUL32Q15[0],
                                                           processData,
                                                           1)); // apply gain and output has same QFactor as input

         // apply make up gain if needed
         if (drc_config_ptr->makeupGainUL16Q12 != MAKEUPGAIN_UNITY) // Implement only non-unity gain
         {
            // Multiply output with the shift normalized makeup gain
            tempOutL64 = s64_mult_s32_u16_shift(tempL32, drc_config_ptr->makeupGainUL16Q12, 4);
            tempL32    = s32_saturate_s64(tempOutL64);
         }
         output[i] = s16_saturate_s32(tempL32);
      }

      // Check if Delay buffer reaches the cirulary bundary
      pState->processIndex++;
      pState->inputIndex++;

      pState->processIndex = s32_modwrap_s32_u32(pState->processIndex, delayBuffSize);
      pState->inputIndex   = s32_modwrap_s32_u32(pState->inputIndex, delayBuffSize);
   }

   for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
   {
      pState->currState[channelNo]   = pState->currState[0];   // use same currState for all channels
      pState->gainUL32Q15[channelNo] = pState->gainUL32Q15[0]; // use same gain for all channels
   }

   return DRC_SUCCESS;
}

/*======================================================================

FUNCTION      ProcessMC16Unlinked

DESCRIPTION   DRC processing for 16bit multichannel unlinked case

PARAMETERS    pDrcLib: [in] pointer to the library structure
              negativeDrcTavUL16Q16: [in] precalculated negative time constant
              nSamplePerChannel: [in] number of samples per channel
              delayBuffSize: [in] size of delay buffer
              pOutPtr: [out] pointer to the output data
           pInPtr: [in] pointer to the input data

RETURN VALUE  Failure or Success

SIDE EFFECTS  None.

======================================================================*/

DRC_RESULT ProcessMC16Unlinked(drc_lib_t *pDrcLib,
                               uint16     negativeDrcTavUL16Q16,
                               uint32     nSamplePerChannel,
                               uint32     delayBuffSize,
                               int8 **    pOutPtr,
                               int8 **    pInPtr)
{

   uint32 i, channelNo;
   int16 *output;
   int16  processData;
   int16 *delayBuffer;
   int32  tempL32;
   int64  tempOutL64;
   int32  currDrcRmsL32;

   drc_lib_mem_t *      pDrcLibMem     = (drc_lib_mem_t *)pDrcLib->lib_mem_ptr;
   drc_state_struct_t * pState         = pDrcLibMem->drc_state_struct_ptr;
   drc_static_struct_t *pStatic        = pDrcLibMem->drc_static_struct_ptr;
   drc_config_t *       drc_config_ptr = pDrcLibMem->drc_config_ptr;

   int16 downsample_counter;
   int16 process_index;
   int16 input_index;

   for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
   {
      delayBuffer = (int16 *)pState->delayBuffer[channelNo];
      output      = (int16 *)(pOutPtr[channelNo]);

      downsample_counter = pState->downSampleCounter;
      process_index      = pState->processIndex;
      input_index        = pState->inputIndex;

      for (i = 0; i < nSamplePerChannel; i++)
      {
         delayBuffer[input_index] = ((int16 *)pInPtr[channelNo])[i];

         //-------- Compute long term input RMS for DRC ---------
         currDrcRmsL32 = s32_mult_s16_s16(delayBuffer[input_index], delayBuffer[input_index]);

         pState->rmsStateL32[channelNo] = s32_extract_s64_l(
            s64_add_s64_s64(s64_mult_s32_u16_shift(currDrcRmsL32, drc_config_ptr->rmsTavUL16Q16, 0),
                            s64_mult_s32_u16_shift(pState->rmsStateL32[channelNo], negativeDrcTavUL16Q16, 0)));

         //---------------- Update gain computation only if at downSampleLevels --------
         if (0 == downsample_counter)
         {
            downsample_counter = drc_config_ptr->downSampleLevel;
            // compute the current INPUT RMS in dB (log domain)
            // log10_fixed: input L32Q0 format, output Q23 format
            if (pState->rmsStateL32[channelNo] != 0)
            {
               pState->drcRmsDBL32Q23[channelNo] =
                  log10_fixed(pState->rmsStateL32[channelNo]); // log10_fixed is 10*log10(.) in fixed point
            }
            else
            {
               pState->drcRmsDBL32Q23[channelNo] = MIN_RMS_DB_L32Q23;
            }

            // ----------- Compute DRC gain ------------------------
            compute_drc_gain(pState, drc_config_ptr, channelNo);
         }
         downsample_counter--;

         // ---------- Apply DRC gain ----------------------
         processData = delayBuffer[process_index];
         tempL32     = s32_saturate_s64(s64_mult_s32_s16_shift(pState->gainUL32Q15[channelNo],
                                                           processData,
                                                           1)); // apply gain and output has same QFactor as input

         // apply make up gain if needed
         if (drc_config_ptr->makeupGainUL16Q12 != MAKEUPGAIN_UNITY) // Implement only non-unity gain
         {
            // Multiply output with the shift normalized makeup gain
            tempOutL64 = s64_mult_s32_u16_shift(tempL32, drc_config_ptr->makeupGainUL16Q12, 4);
            tempL32    = s32_saturate_s64(tempOutL64);
         }

         // output results
         output[i] = s16_saturate_s32(tempL32);

         // Check if Delay buffer reaches the cirulary bundary
         process_index++;
         input_index++;

         process_index = s32_modwrap_s32_u32(process_index, delayBuffSize);
         input_index   = s32_modwrap_s32_u32(input_index, delayBuffSize);
      }
   }

   pState->downSampleCounter = downsample_counter;
   pState->processIndex      = process_index;
   pState->inputIndex        = input_index;

   return DRC_SUCCESS;
}

/*======================================================================

FUNCTION      ProcessMC32Linked

DESCRIPTION   DRC processing for 32bit multichannel linked case

PARAMETERS    pDrcLib: [in] pointer to the library structure
              negativeDrcTavUL16Q16: [in] precalculated negative time constant
              nSamplePerChannel: [in] number of samples per channel
              delayBuffSize: [in] size of delay buffer
              pOutPtr: [out] pointer to the output data
           pInPtr: [in] pointer to the input data

RETURN VALUE  Failure or Success

SIDE EFFECTS  None.

======================================================================*/

DRC_RESULT ProcessMC32Linked(drc_lib_t *pDrcLib,
                             uint16     negativeDrcTavUL16Q16,
                             uint32     nSamplePerChannel,
                             uint32     delayBuffSize,
                             int8 **    pOutPtr,
                             int8 **    pInPtr)
{
   uint32 i, channelNo;
   int32 *output;
   int32  processData;
   int32 *delayBuffer;
   int32  tempL32;
   int64  tempOutL64;
   int64  squareInputL64;
   int64  currDrcRmsL64;
   int32  currDrcRmsL32;

   drc_lib_mem_t *      pDrcLibMem     = (drc_lib_mem_t *)pDrcLib->lib_mem_ptr;
   drc_state_struct_t * pState         = pDrcLibMem->drc_state_struct_ptr;
   drc_static_struct_t *pStatic        = pDrcLibMem->drc_static_struct_ptr;
   drc_config_t *       drc_config_ptr = pDrcLibMem->drc_config_ptr;

   const int16  shift_factor = shift_factor_arr[pStatic->num_channel];
   const uint16 mult_factor  = mult_factor_arr[pStatic->num_channel];

   for (i = 0; i < nSamplePerChannel; i++)
   {
      squareInputL64 = 0;

      for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
      {
         delayBuffer                     = (int32 *)pState->delayBuffer[channelNo];
         delayBuffer[pState->inputIndex] = ((int32 *)pInPtr[channelNo])[i];

         //-------- Compute long term input RMS for DRC ---------
         // For stereo linked, currDrcRms = (xL(n)^2 + xR(n)^2)/2 (stereo channel example)
         currDrcRmsL64  = s64_mult_s32_s32(delayBuffer[pState->inputIndex], delayBuffer[pState->inputIndex]);
         currDrcRmsL64  = s64_shl_s64(currDrcRmsL64, s16_shl_s16(s16_sub_s16_s16(Q15, Q27), ONE));
         squareInputL64 = s64_add_s64_s64(squareInputL64, currDrcRmsL64);
      }

      // calculate squareInputL64 = squareInputL64/pDrcLib->drc_static_struct.num_channel
      squareInputL64 = s64_shl_s64(squareInputL64, shift_factor);
      if (mult_factor != 1)
      {
         squareInputL64 = s64_mult_s32_s16(s32_saturate_s64(squareInputL64), mult_factor);
      }

      currDrcRmsL32 = s32_saturate_s64(squareInputL64);

      // current_drcRms = previous_drcRms*(1-drcTav) + drcTav * x[n] ^ 2
      pState->rmsStateL32[0] =
         s32_extract_s64_l(s64_add_s64_s64(s64_mult_s32_u16_shift(currDrcRmsL32, drc_config_ptr->rmsTavUL16Q16, 0),
                                           s64_mult_s32_u16_shift(pState->rmsStateL32[0], negativeDrcTavUL16Q16, 0)));

      //---------------- Update gain computation only if at downSampleLevels --------
      if (0 == pState->downSampleCounter)
      {
         pState->downSampleCounter = drc_config_ptr->downSampleLevel;
         // compute the current INPUT RMS in dB (log domain)
         // log10_fixed: input L32Q0 format, output Q23 format
         if (pState->rmsStateL32[0] != 0)
         {
            pState->drcRmsDBL32Q23[0] =
               log10_fixed(pState->rmsStateL32[0]); // log10_fixed is 10*log10(.) in fixed point
         }
         else
         {
            pState->drcRmsDBL32Q23[0] = MIN_RMS_DB_L32Q23;
         }

         // ----------- Compute DRC gain ------------------------
         compute_drc_gain(pState, drc_config_ptr, 0);
      }
      pState->downSampleCounter--;

      // ---------- Apply DRC gain ----------------------
      for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
      {
         delayBuffer = (int32 *)(pState->delayBuffer[channelNo]);
         output      = (int32 *)(pOutPtr[channelNo]);
         processData = delayBuffer[pState->processIndex];

         tempOutL64 = s64_mult_s32_s32_shift(processData,
                                             pState->gainUL32Q15[0],
                                             17); // apply gain and output has same QFactor as input
         tempL32    = s32_saturate_s64(tempOutL64);

         // apply make up gain if needed
         if (drc_config_ptr->makeupGainUL16Q12 != MAKEUPGAIN_UNITY) // Implement only non-unity gain
         {
            // Multiply output with the shift normalized makeup gain
            tempOutL64 = s64_mult_s32_u16_shift(tempL32, drc_config_ptr->makeupGainUL16Q12, 4);
            tempL32    = s32_saturate_s64(tempOutL64);
         }

         output[i] = tempL32;
      }

      // Check if Delay buffer reaches the cirulary bundary
      pState->processIndex++;
      pState->inputIndex++;

      pState->processIndex = s32_modwrap_s32_u32(pState->processIndex, delayBuffSize);
      pState->inputIndex   = s32_modwrap_s32_u32(pState->inputIndex, delayBuffSize);
   }

   for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
   {
      pState->currState[channelNo]   = pState->currState[0];   // use same currState for all channels
      pState->gainUL32Q15[channelNo] = pState->gainUL32Q15[0]; // use same gain for all channels
   }

   return DRC_SUCCESS;
}

/*======================================================================

FUNCTION     ProcessMC32Unlinked

DESCRIPTION   DRC processing for 32bit multichannel unlinked case

PARAMETERS    pDrcLib: [in] pointer to the library structure
              negativeDrcTavUL16Q16: [in] precalculated negative time constant
              nSamplePerChannel: [in] number of samples per channel
              delayBuffSize: [in] size of delay buffer
              pOutPtr: [out] pointer to the output data
           pInPtr: [in] pointer to the input data

RETURN VALUE  Failure or Success

SIDE EFFECTS  None.

======================================================================*/

DRC_RESULT ProcessMC32Unlinked(drc_lib_t *pDrcLib,
                               uint16     negativeDrcTavUL16Q16,
                               uint32     nSamplePerChannel,
                               uint32     delayBuffSize,
                               int8 **    pOutPtr,
                               int8 **    pInPtr)
{
   uint32 i, channelNo;
   int32 *output;
   int32  processData;
   int32 *delayBuffer;
   int32  tempL32;
   int64  tempOutL64;
   int64  currDrcRmsL64;
   int32  currDrcRmsL32;

   drc_lib_mem_t *      pDrcLibMem     = (drc_lib_mem_t *)pDrcLib->lib_mem_ptr;
   drc_state_struct_t * pState         = pDrcLibMem->drc_state_struct_ptr;
   drc_static_struct_t *pStatic        = pDrcLibMem->drc_static_struct_ptr;
   drc_config_t *       drc_config_ptr = pDrcLibMem->drc_config_ptr;

   int16 downsample_counter;
   int16 process_index;
   int16 input_index;

   for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
   {
      delayBuffer = (int32 *)pState->delayBuffer[channelNo];
      output      = (int32 *)(pOutPtr[channelNo]);

      downsample_counter = pState->downSampleCounter;
      process_index      = pState->processIndex;
      input_index        = pState->inputIndex;

      for (i = 0; i < nSamplePerChannel; i++)
      {
         delayBuffer[input_index] = ((int32 *)pInPtr[channelNo])[i];

         // -------- Compute long term input RMS for DRC ---------
         // x[n] ^ 2 for each channelNo separately & is same Q-factor as in the 16-bit width case.
         currDrcRmsL64 = s64_mult_s32_s32(delayBuffer[input_index], delayBuffer[input_index]);
         currDrcRmsL64 = s64_shl_s64(currDrcRmsL64, s16_shl_s16(s16_sub_s16_s16(Q15, Q27), ONE));
         currDrcRmsL32 = s32_saturate_s64(currDrcRmsL64);

         pState->rmsStateL32[channelNo] = s32_extract_s64_l(
            s64_add_s64_s64(s64_mult_s32_u16_shift(currDrcRmsL32, drc_config_ptr->rmsTavUL16Q16, 0),
                            s64_mult_s32_u16_shift(pState->rmsStateL32[channelNo], negativeDrcTavUL16Q16, 0)));

         //---------------- Update gain computation only if at downSampleLevels --------
         if (0 == downsample_counter)
         {
            downsample_counter = drc_config_ptr->downSampleLevel;
            // compute the current INPUT RMS in dB (log domain)
            // log10_fixed: input L32Q0 format, output Q23 format
            if (pState->rmsStateL32[channelNo] != 0)
            {
               pState->drcRmsDBL32Q23[channelNo] =
                  log10_fixed(pState->rmsStateL32[channelNo]); // log10_fixed is 10*log10(.) in fixed point
            }
            else
            {
               pState->drcRmsDBL32Q23[channelNo] = MIN_RMS_DB_L32Q23;
            }

            // ----------- Compute DRC gain ------------------------
            compute_drc_gain(pState, drc_config_ptr, channelNo);
         }
         downsample_counter--;

         // ---------- Apply DRC gain ----------------------
         processData = delayBuffer[process_index];

         tempOutL64 = s64_mult_s32_s32_shift(processData,
                                             pState->gainUL32Q15[channelNo],
                                             17); // apply gain and output has same QFactor as input
         tempL32    = s32_saturate_s64(tempOutL64);

         // apply make up gain if needed
         if (drc_config_ptr->makeupGainUL16Q12 != MAKEUPGAIN_UNITY) // Implement only non-unity gain
         {
            // Multiply output with the shift normalized makeup gain
            tempOutL64 = s64_mult_s32_u16_shift(tempL32, drc_config_ptr->makeupGainUL16Q12, 4);
            tempL32    = s32_saturate_s64(tempOutL64);
         }
         // output results
         output[i] = tempL32;

         // Check if Delay buffer reaches the cirulary bundary
         process_index++;
         input_index++;

         process_index = s32_modwrap_s32_u32(process_index, delayBuffSize);
         input_index   = s32_modwrap_s32_u32(input_index, delayBuffSize);
      }
   }

   pState->downSampleCounter = downsample_counter;
   pState->processIndex      = process_index;
   pState->inputIndex        = input_index;

   return DRC_SUCCESS;
}

/*======================================================================

FUNCTION      compute_drc_gain

DESCRIPTION   Computes the DRC gain for current input samples

PARAMETERS    pState: [in/out] pointer to the state structure that saves the DRC processing states
drc_config_ptr:[in] Pointer to the calibration structure

RETURN VALUE  Computed drc gain value for current input samples

SIDE EFFECTS  None.

======================================================================*/
void compute_drc_gain(drc_state_struct_t *pState, drc_config_t *drc_config_ptr, uint32 j)
{

   // ------------ Variable Declaration ------------
   int16        drcRmsDBL16Q7, outDrcRmsDBL16Q7;
   int32        drcRmsDBL32Q23, outDrcRmsDBL32Q23;
   uint32       newTargetGainUL32Q15;
   int64        currDrcGainL64Q27, gainDiffL64Q27;
   DrcStateType currState;
   int32        tempOutL32;
   uint16       tempSlopeUL16Q16;
   int16        tempThresholdL16Q7, tempSlopeL16Q8;
   int32        newTargetGainL32Q26, newTargetGainL32Q23;
   int64        tempRmsDBL40Q23;

   //---------------- Update gain computation only if at downSampleLevels --------

   currState = pState->currState[j];

   drcRmsDBL32Q23 = pState->drcRmsDBL32Q23[j];         // current INPUT RMS in dB (log domain)
   drcRmsDBL16Q7  = s16_extract_s32_h(drcRmsDBL32Q23); // Q7
   drcRmsDBL16Q7  = s16_max_s16_s16(drcRmsDBL16Q7, MIN_RMS_DB_L16Q7);

   // compute the current output RMS in dB (log domain)
   // outDrcRmsDB = 10*log10(input*gainUL32Q15)^2
   // convert UL32 to L32 format that log10_fixed can take as input
   tempOutL32 = s32_saturate_s64(s64_shl_s64(pState->gainUL32Q15[j], -1));
   // log10_fixed: input L32Q0 format, output Q23 format
   if (tempOutL32 != 0)
   {
      outDrcRmsDBL32Q23 = log10_fixed(tempOutL32);
      outDrcRmsDBL32Q23 =
         (int32)s64_add_s32_s32((int32)s64_sub_s64_s64(s64_shl_s64(outDrcRmsDBL32Q23, 1), DB_16384_L32Q23),
                                drcRmsDBL32Q23);
   }
   else
   {
      outDrcRmsDBL32Q23 = MIN_RMS_DB_L32Q23;
   }
   outDrcRmsDBL16Q7 = s16_extract_s32_h(outDrcRmsDBL32Q23);

   // ---- Compute the Target Gain for the current sample based on input RMS ----
   // newTargetGainUL32Q15 = target_gain_comp(drc_config_ptr, drcRmsDBL16Q7);

   // figure out what part of compression curve the rms value is in
   if (drcRmsDBL16Q7 > drc_config_ptr->dnCompThresholdL16Q7)
   // in Down Dompression
   {
      tempThresholdL16Q7 = drc_config_ptr->dnCompThresholdL16Q7;
      tempSlopeUL16Q16   = drc_config_ptr->dnCompSlopeUL16Q16;

      // newTarget = (dwCompThreshold - Xrms[n]) * dwCompSlopeUL16Q16
      newTargetGainL32Q23 = s32_mult_s16_u16(s16_sub_s16_s16(tempThresholdL16Q7, drcRmsDBL16Q7),
                                             tempSlopeUL16Q16); // Q23
   }
   else if (drcRmsDBL16Q7 < drc_config_ptr->dnExpaThresholdL16Q7)
   // in Down Expansion
   {
      tempThresholdL16Q7 = drc_config_ptr->upCompThresholdL16Q7;
      tempSlopeUL16Q16   = drc_config_ptr->upCompSlopeUL16Q16;

      // newTarget = (dwExpaThresholdL16Q7 - Xrms[n])*dwExpaSlopeUL16Q16 + ...
      //             (uwCompThresholdL16Q7 - dwExpaThresholdL16Q7)*uwCompSlopeUL16Q16;
      newTargetGainL32Q23 = s32_mult_s16_u16(s16_sub_s16_s16(tempThresholdL16Q7, drc_config_ptr->dnExpaThresholdL16Q7),
                                             tempSlopeUL16Q16); // Q23

      tempThresholdL16Q7 = drc_config_ptr->dnExpaThresholdL16Q7;
      tempSlopeL16Q8     = drc_config_ptr->dnExpaSlopeL16Q8;

      tempRmsDBL40Q23 = s64_mult_s16_s16_shift(s16_sub_s16_s16(tempThresholdL16Q7, drcRmsDBL16Q7), tempSlopeL16Q8, 8);
      newTargetGainL32Q23 = s32_saturate_s64(s64_add_s64_s32(tempRmsDBL40Q23, newTargetGainL32Q23));
      // L32Q23

      // Limit the gain reduction in Downward Expander part to be dnExpaMinGainDB
      if (newTargetGainL32Q23 < drc_config_ptr->dnExpaMinGainDBL32Q23)
      {
         newTargetGainL32Q23 = drc_config_ptr->dnExpaMinGainDBL32Q23;
      }
   }
   else if (drcRmsDBL16Q7 < drc_config_ptr->upCompThresholdL16Q7)
   // in Up Dompression
   {
      tempThresholdL16Q7 = drc_config_ptr->upCompThresholdL16Q7;
      tempSlopeUL16Q16   = drc_config_ptr->upCompSlopeUL16Q16;

      // newTarget = (uwCompThreshold - Xrms[n]) * uwCompSlopeUL16Q16
      newTargetGainL32Q23 = s32_mult_s16_u16(s16_sub_s16_s16(tempThresholdL16Q7, drcRmsDBL16Q7),
                                             tempSlopeUL16Q16); // Q23
   }
   else
   {
      newTargetGainL32Q23 = 0x00000000;
   }

   // calculate new target gain = 10^(new target gain log / 20): input L32Q26, out:L32Q15
   newTargetGainL32Q26          = (int32)s64_mult_s32_s16_shift(newTargetGainL32Q23, ONE_OVER_TWENTY_UQ19, 0);
   newTargetGainUL32Q15         = exp10_fixed(newTargetGainL32Q26); // Q15
   pState->targetGainUL32Q15[j] = newTargetGainUL32Q15;

   // --- Find out the appropriate time constant based on output RMS ---
   if (outDrcRmsDBL16Q7 > pState->outDnCompThresholdL16Q7)
   {
      if (newTargetGainUL32Q15 < pState->gainUL32Q15[j])
      {
         pState->timeConstantUL32Q31[j] = drc_config_ptr->dnCompAttackUL32Q31;
         currState                      = ATTACK;
      }
      else if (newTargetGainUL32Q15 >=
               (uint32)s32_saturate_s64(
                  s64_mult_u32_s16_shift(pState->gainUL32Q15[j], drc_config_ptr->dnCompHysterisisUL16Q14, 2)))
      {
         pState->timeConstantUL32Q31[j] = drc_config_ptr->dnCompReleaseUL32Q31;
         currState                      = RELEASE;
      }
      else
      {
         if (currState == ATTACK)
         {
            pState->timeConstantUL32Q31[j] = 0;
            currState                      = NO_CHANGE;
         }
      }

      pState->dwcomp_state_change[j] = 0;
   }
   else if (outDrcRmsDBL16Q7 < pState->outDnExpaThresholdL16Q7)
   {
      if ((uint32)s32_saturate_s64(
             s64_mult_u32_s16_shift(newTargetGainUL32Q15, drc_config_ptr->dnExpaHysterisisUL16Q14, 2)) <
          pState->gainUL32Q15[j])
      {
         pState->timeConstantUL32Q31[j] = drc_config_ptr->dnExpaAttackUL32Q31;
         currState                      = ATTACK;
      }
      else if (newTargetGainUL32Q15 > pState->gainUL32Q15[j])
      {
         pState->timeConstantUL32Q31[j] = drc_config_ptr->dnExpaReleaseUL32Q31;
         currState                      = RELEASE;
      }
      else
      {
         if (currState == RELEASE)
         {
            pState->timeConstantUL32Q31[j] = 0;
            currState                      = NO_CHANGE;
         }
      }
   }
   else if (outDrcRmsDBL16Q7 < pState->outUpCompThresholdL16Q7)
   {
      if (newTargetGainUL32Q15 < pState->gainUL32Q15[j])
      {
         pState->timeConstantUL32Q31[j] = drc_config_ptr->upCompAttackUL32Q31;
         currState                      = ATTACK;
      }
      else if (newTargetGainUL32Q15 >=
               (uint32)s32_saturate_s64(
                  s64_mult_u32_s16_shift(pState->gainUL32Q15[j], drc_config_ptr->upCompHysterisisUL16Q14, 2)))
      {
         pState->timeConstantUL32Q31[j] = drc_config_ptr->upCompReleaseUL32Q31;
         currState                      = RELEASE;
      }
      else
      {
         if (currState == ATTACK)
         {
            pState->timeConstantUL32Q31[j] = 0;
            currState                      = NO_CHANGE;
         }
      }

      pState->uwcomp_state_change[j] = 0;
   }
   else
   {
      if (drcRmsDBL16Q7 > drc_config_ptr->dnCompThresholdL16Q7)
      {
         pState->timeConstantUL32Q31[j] = drc_config_ptr->dnCompAttackUL32Q31;
         currState                      = ATTACK;
      }
      else if (drcRmsDBL16Q7 < drc_config_ptr->dnExpaThresholdL16Q7)
      {
         pState->timeConstantUL32Q31[j] = drc_config_ptr->dnExpaAttackUL32Q31;
         currState                      = ATTACK;
      }
      else if (drcRmsDBL16Q7 < drc_config_ptr->upCompThresholdL16Q7)
      {
         pState->timeConstantUL32Q31[j] = drc_config_ptr->upCompReleaseUL32Q31;
         currState                      = RELEASE;
      }
      else
      {
         // pState->timeConstantUL32Q31[j] = 0;
         currState = NO_CHANGE;

         if (pState->dwcomp_state_change[j] == 0)
         {
            pState->timeConstantUL32Q31[j] = drc_config_ptr->dnCompReleaseUL32Q31;
            currState                      = RELEASE;
            pState->dwcomp_state_change[j] = 1;
         }
         else if (pState->uwcomp_state_change[j] == 0)
         {
            pState->timeConstantUL32Q31[j] = drc_config_ptr->upCompAttackUL32Q31;
            currState                      = ATTACK;
            pState->uwcomp_state_change[j] = 1;
         }
      }
   }

   // --- calculate DRC gain with determined smooth factor ---
   // drcGain	= drcGain*(1-timeConstant) + drcTargetGain*timeConstant
   //			= drcGain + (drcTargetGain - drcGain)*timeConstant
   gainDiffL64Q27 = s64_sub_s64_s64(((int64)pState->targetGainUL32Q15[j]) << 12, pState->instGainUL64Q27[j]);
   currDrcGainL64Q27 =
      s64_mult_s32_s32_shift(s32_saturate_s64(gainDiffL64Q27), pState->timeConstantUL32Q31[j], 1); // Q27
   // Since timeconstant will always be less than 0 (most significant bit always be zero),
   // therefore we can replace signed-unsigned multiplication with signed-signed multiplication.
   currDrcGainL64Q27          = s64_add_s64_s64(currDrcGainL64Q27, pState->instGainUL64Q27[j]);
   pState->gainUL32Q15[j]     = s32_saturate_s64(s64_shl_s64(currDrcGainL64Q27, -12));
   pState->instGainUL64Q27[j] = currDrcGainL64Q27;
   pState->currState[j]       = currState;
}
#endif
