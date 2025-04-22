/*============================================================================
  FILE:          CDrcLib.c

  OVERVIEW:      Implements the drciter algorithm.

  DEPENDENCIES:  None

                 Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
                 SPDX-License-Identifier: BSD-3-Clause-Clear
============================================================================*/

/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/

#include "drc_lib.h"
#include <string.h>
#include "audio_basic_op_ext.h"
#include "audio_log10.h"
#include "audio_exp10.h"

/*----------------------------------------------------------------------------
 * Private Function Declarations
 * -------------------------------------------------------------------------*/
DRC_RESULT output_rms_comp(drc_lib_mem_t *drc_lib_mem_ptr);
DRC_RESULT drc_processing_defaults(drc_config_t *drc_config_ptr);
DRC_RESULT state_memory_defaults(drc_lib_t *drc_lib_ptr);

DRC_RESULT drc_processing_mode(drc_static_struct_t * pStatic,
                               drc_feature_mode_t    mode,
                               drc_channel_linking_t link,
                               drc_state_struct_t *  state);
DRC_RESULT ProcessBP16(drc_state_struct_t * pState,
                       drc_static_struct_t *pStatic,
                       uint32               nSamplePerChannel,
                       uint32               delayBuffSize,
                       int8 **              pOutPtr,
                       int8 **              pInPtr);
DRC_RESULT ProcessBP32(drc_state_struct_t * pState,
                       drc_static_struct_t *pStatic,
                       uint32               nSamplePerChannel,
                       uint32               delayBuffSize,
                       int8 **              pOutPtr,
                       int8 **              pInPtr);
DRC_RESULT ProcessMono16(drc_lib_t *pDrcLib,
                         uint16     negativeDrcTavUL16Q16,
                         uint32     nSamplePerChannel,
                         uint32     delayBuffSize,
                         int8 **    pOutPtr,
                         int8 **    pInPtr);
DRC_RESULT ProcessMono32(drc_lib_t *pDrcLib,
                         uint16     negativeDrcTavUL16Q16,
                         uint32     nSamplePerChannel,
                         uint32     delayBuffSize,
                         int8 **    pOutPtr,
                         int8 **    pInPtr);
DRC_RESULT ProcessMC16Linked(drc_lib_t *pDrcLib,
                             uint16     negativeDrcTavUL16Q16,
                             uint32     nSamplePerChannel,
                             uint32     delayBuffSize,
                             int8 **    pOutPtr,
                             int8 **    pInPtr);
DRC_RESULT ProcessMC16Unlinked(drc_lib_t *pDrcLib,
                               uint16     negativeDrcTavUL16Q16,
                               uint32     nSamplePerChannel,
                               uint32     delayBuffSize,
                               int8 **    pOutPtr,
                               int8 **    pInPtr);
DRC_RESULT ProcessMC32Linked(drc_lib_t *pDrcLib,
                             uint16     negativeDrcTavUL16Q16,
                             uint32     nSamplePerChannel,
                             uint32     delayBuffSize,
                             int8 **    pOutPtr,
                             int8 **    pInPtr);
DRC_RESULT ProcessMC32Unlinked(drc_lib_t *pDrcLib,
                               uint16     negativeDrcTavUL16Q16,
                               uint32     nSamplePerChannel,
                               uint32     delayBuffSize,
                               int8 **    pOutPtr,
                               int8 **    pInPtr);

#ifndef QDSP6_DRCLIB_ASM
static void compute_drc_gain(drc_state_struct_t *pState, drc_config_t *drc_config_ptr, uint32 gainNum);
#endif
/*----------------------------------------------------------------------------
 * Private Table Definitions
 * -------------------------------------------------------------------------*/
#ifdef PROD_SPECIFIC_MAX_CH
int16 rms_constant_factor[MAX_NUM_CHANNEL + 1] =
   { 0,    32767, 16384, 10923, 8192, 6554, 5461, 4681, 4096, 3641, 3277, 2979, 2731, 2521, 2341, 2185, 2048,
     1928, 1820,  1725,  1638,  1560, 1489, 1425, 1365, 1311, 1260, 1214, 1170, 1130, 1092, 1057, 1024, 993,
	 964,  936,   910,   886,   862,  840,  819,  799,  780,  762,  745,  728,  712,  697,  683,  669,  655,
	 643,  630,   618,   607,   596,  585,  575,  565,  555,  546,  537,  529,  520,  512,  504,  496,  489,
	 482,  475,   468,   462,   455,  449,  443,  437,  431,  426,  420,  415,  410,  405,  400,  395,  390,
	 386,  381, 377,     372,   368,  364,  360,  356,  352,  349,  345,  341,  338,  334,  331,  328,  324,
	 321,  318,   315,   312,   309,  306,  303,  301,  298,  295,  293,  290,  287,  285,  282,  280,  278,
	 275,  273,   271,   269,   266,  264,  262,  260,  258,  256 };
#else
int16 rms_constant_factor[MAX_NUM_CHANNEL + 1] =
   { 0,    32767, 16384, 10923, 8192, 6554, 5461, 4681, 4096, 3641, 3277, 2979, 2731, 2521, 2341, 2185, 2048,
     1928, 1820,  1725,  1638,  1560, 1489, 1425, 1365, 1311, 1260, 1214, 1170, 1130, 1092, 1057, 1024 };	
#endif

/*----------------------------------------------------------------------------
 * Function Definitions
 * -------------------------------------------------------------------------*/
/*======================================================================

FUNCTION      drc_get_mem_req

DESCRIPTION   Determine lib mem size. Called once at audio connection set up time.

DEPENDENCIES  Input pointers must not be NULL.

PARAMETERS    drc_lib_mem_requirements_ptr: [out] Pointer to lib mem requirements structure
drc_static_struct_ptr: [in] Pointer to static structure

SIDE EFFECTS  None

======================================================================*/

DRC_RESULT drc_get_mem_req(drc_lib_mem_requirements_t *drc_lib_mem_requirements_ptr,
                           drc_static_struct_t *       drc_static_struct_ptr)

{
   uint32 libMemStructSize;
   uint32 staticStructSize;
   uint32 featureModeStructSize;
   uint32 processingStructSize;

   uint32 stateStructSize, stateSize;
   uint32 delayBufferSize;
   uint32 delayBufferSizePerChannel;
   uint32 size;

   // check if num_channel is valid for the c-sim
   if (drc_static_struct_ptr->num_channel > MAX_NUM_CHANNEL)
   {
      return DRC_FAILURE;
   }

   // clear memory
   memset(drc_lib_mem_requirements_ptr, 0, sizeof(drc_lib_mem_requirements_t));

   // determine mem size
   libMemStructSize      = sizeof(drc_lib_mem_t);
   libMemStructSize      = ALIGN8(libMemStructSize);
   staticStructSize      = sizeof(drc_static_struct_t);
   staticStructSize      = ALIGN8(staticStructSize);
   featureModeStructSize = sizeof(drc_feature_mode_t);
   featureModeStructSize = ALIGN8(featureModeStructSize);
   processingStructSize  = sizeof(drc_config_t);
   processingStructSize  = ALIGN8(processingStructSize);

   stateStructSize = sizeof(drc_state_struct_t);
   stateStructSize = ALIGN8(stateStructSize);

   size = sizeof(int8 *) * drc_static_struct_ptr->num_channel;
   size = ALIGN8(size);
   size += sizeof(int32) * drc_static_struct_ptr->num_channel;
   size = ALIGN8(size);
   size += sizeof(int32) * drc_static_struct_ptr->num_channel;
   size = ALIGN8(size);
   size += sizeof(uint32) * drc_static_struct_ptr->num_channel;
   size = ALIGN8(size);
   size += sizeof(uint32) * drc_static_struct_ptr->num_channel;
   size = ALIGN8(size);
   size += sizeof(DrcStateType) * drc_static_struct_ptr->num_channel;
   size = ALIGN8(size);
   size += sizeof(uint32) * drc_static_struct_ptr->num_channel;
   size = ALIGN8(size);
   size += sizeof(int32) * drc_static_struct_ptr->num_channel;
   size = ALIGN8(size);
   size += sizeof(int32) * drc_static_struct_ptr->num_channel;
   size = ALIGN8(size);
   size += sizeof(int32) * drc_static_struct_ptr->num_channel;
   size = ALIGN8(size);
   size += sizeof(uint64) * drc_static_struct_ptr->num_channel;
   stateSize = ALIGN8(size);

   delayBufferSizePerChannel = (uint32)(BITS_16 == drc_static_struct_ptr->data_width
                                           ? s64_shl_s64(s64_add_s32_u32(ONE, drc_static_struct_ptr->delay), ONE)
                                           : s64_shl_s64(s64_add_s32_u32(ONE, drc_static_struct_ptr->delay), TWO));
   delayBufferSizePerChannel = (uint32)(ALIGN8(delayBufferSizePerChannel));
   delayBufferSize           = (uint32)(drc_static_struct_ptr->num_channel * delayBufferSizePerChannel);

   // lib memory arrangement

   // -------------------  ----> drc_lib_mem_requirements_ptr->lib_mem_size
   // drc_lib_mem_t
   // -------------------
   // drc_static_struct_t
   // -------------------
   // drc_feature_mode_t
   // -------------------
   // drc_processing_t
   // -------------------
   // drc_state_struct_t
   // -------------------
   // states
   // -------------------
   // delay buffer
   // -------------------

   // total lib mem needed = drc_lib_mem_t + drc_static_struct_t + drc_feature_mode_t + drc_processing_t +
   // drc_state_struct_t + stateSize + delay buffer
   drc_lib_mem_requirements_ptr->lib_mem_size = libMemStructSize + staticStructSize + featureModeStructSize +
                                                processingStructSize + stateStructSize + stateSize + delayBufferSize;

   // maximal lib stack mem consumption
   drc_lib_mem_requirements_ptr->lib_stack_size = drc_max_stack_size;

   return DRC_SUCCESS;
}

/*======================================================================

FUNCTION      drc_init_memory

DESCRIPTION   Performs partition(allocation) and initialization of lib memory for the
drc algorithm. Called once at audio connection set up time.

DEPENDENCIES  Input pointers must not be NULL.

PARAMETERS    drc_lib_ptr: [in, out] Pointer to lib structure
drc_static_struct_ptr: [in] Pointer to static structure
pMem: [in] Pointer to the lib memory
memSize: [in] Size of the memory pointed by pMem

SIDE EFFECTS  None

======================================================================*/
DRC_RESULT drc_init_memory(drc_lib_t *          drc_lib_ptr,
                           drc_static_struct_t *drc_static_struct_ptr,
                           int8 *               pMem,
                           uint32               memSize)
{
   drc_lib_mem_t *pDrcLibMem = NULL;
   int8 *         pTemp      = pMem;
   uint32         channelNo, delayBufferSizePerChannel;

   uint32 libMemSize, libMemStructSize, staticStructSize, featureModeStructSize, processingStructSize, stateStructSize,
      stateSize, delayBufferSize;
   uint32 stateSize1, stateSize2, stateSize3, stateSize4, stateSize5, stateSize6, stateSize7, stateSize8, stateSize9,
      stateSize10, stateSize11;

   // check if num_channel is valid for the c-sim
   if (drc_static_struct_ptr->num_channel > MAX_NUM_CHANNEL)
   {
      return DRC_FAILURE;
   }

   // re-calculate lib mem size
   libMemStructSize      = ALIGN8(sizeof(drc_lib_mem_t));
   staticStructSize      = ALIGN8(sizeof(drc_static_struct_t));
   featureModeStructSize = ALIGN8(sizeof(drc_feature_mode_t));
   processingStructSize  = ALIGN8(sizeof(drc_config_t));

   stateStructSize = ALIGN8(sizeof(drc_state_struct_t));
   stateSize1      = ALIGN8(sizeof(int8 *) * drc_static_struct_ptr->num_channel);
   stateSize2      = ALIGN8(sizeof(int32) * drc_static_struct_ptr->num_channel);
   stateSize3      = ALIGN8(sizeof(int32) * drc_static_struct_ptr->num_channel);
   stateSize4      = ALIGN8(sizeof(uint32) * drc_static_struct_ptr->num_channel);
   stateSize5      = ALIGN8(sizeof(uint32) * drc_static_struct_ptr->num_channel);
   stateSize6      = ALIGN8(sizeof(DrcStateType) * drc_static_struct_ptr->num_channel);
   stateSize7      = ALIGN8(sizeof(uint32) * drc_static_struct_ptr->num_channel);
   stateSize8      = ALIGN8(sizeof(int32) * drc_static_struct_ptr->num_channel);
   stateSize9      = ALIGN8(sizeof(int32) * drc_static_struct_ptr->num_channel);
   stateSize10     = ALIGN8(sizeof(int32) * drc_static_struct_ptr->num_channel);
   stateSize11     = ALIGN8(sizeof(uint64) * drc_static_struct_ptr->num_channel);
   stateSize = stateSize1 + stateSize2 + stateSize3 + stateSize4 + stateSize5 + stateSize6 + stateSize7 + stateSize8 +
               stateSize9 + stateSize10 + stateSize11;

   delayBufferSizePerChannel =
      (uint32)(ALIGN8(BITS_16 == drc_static_struct_ptr->data_width
                         ? s64_shl_s64(s64_add_s32_u32(ONE, drc_static_struct_ptr->delay), ONE)
                         : s64_shl_s64(s64_add_s32_u32(ONE, drc_static_struct_ptr->delay), TWO)));
   delayBufferSize = (uint32)(drc_static_struct_ptr->num_channel * delayBufferSizePerChannel);

   // total lib mem needed = drc_lib_mem_t + drc_static_struct_t + drc_feature_mode_t + drc_processing_t +
   // drc_state_struct_t + stateSize + delay buffer
   libMemSize = libMemStructSize + staticStructSize + featureModeStructSize + processingStructSize + stateStructSize +
                stateSize + delayBufferSize;

   // error out if the mem space given is not enough
   if (memSize < libMemSize)
   {
      return DRC_MEMERROR;
   }

   // before initializing lib_mem_ptr, it is FW job to make sure that pMem is 8 bytes aligned(with enough space)
   memset(pMem, 0, memSize);        // clear the mem
   drc_lib_ptr->lib_mem_ptr = pMem; // init drc_lib_t;

   // lib memory arrangement
   // -------------------  ----> drc_lib_ptr->lib_mem_ptr
   // drc_lib_mem_t
   // -------------------
   // drc_static_struct_t
   // -------------------
   // drc_feature_mode_t
   // -------------------
   // drc_processing_t
   // -------------------
   // drc_state_struct_t
   // -------------------
   // states
   // -------------------
   // delay buffer
   // -------------------

   // lib memory partition starts here
   pDrcLibMem = (drc_lib_mem_t *)drc_lib_ptr->lib_mem_ptr; // allocate memory for drc_lib_mem_t
   pTemp += libMemStructSize;                              // pTemp points to where drc_static_struct_t will be located

   pDrcLibMem->drc_static_struct_ptr =
      (drc_static_struct_t *)pTemp;                       // init drc_lib_mem_t; allocate memory for drc_static_struct_t
   pDrcLibMem->drc_static_struct_size = staticStructSize; // init drc_lib_mem_t
   pTemp += pDrcLibMem->drc_static_struct_size;           // pTemp points to where drc_feature_mode_t will be located

   // init drc_static_struct_t
   pDrcLibMem->drc_static_struct_ptr->data_width  = drc_static_struct_ptr->data_width;
   pDrcLibMem->drc_static_struct_ptr->sample_rate = drc_static_struct_ptr->sample_rate;
   pDrcLibMem->drc_static_struct_ptr->num_channel = drc_static_struct_ptr->num_channel;
   pDrcLibMem->drc_static_struct_ptr->delay       = drc_static_struct_ptr->delay;

   pDrcLibMem->drc_feature_mode_ptr =
      (drc_feature_mode_t *)pTemp; // init drc_lib_mem_t; allocate memory for drc_feature_mode_t
   pDrcLibMem->drc_feature_mode_size = featureModeStructSize; // init drc_lib_mem_t
   pTemp += pDrcLibMem->drc_feature_mode_size;                // pTemp points to where drc_processing_t will be located

   // init drc_processing_t with defaults
   *pDrcLibMem->drc_feature_mode_ptr = (drc_feature_mode_t)MODE_DEFAULT;

   pDrcLibMem->drc_config_ptr  = (drc_config_t *)pTemp; // init drc_lib_mem_t; allocate memory for drc_processing_t
   pDrcLibMem->drc_config_size = processingStructSize;  // init drc_lib_mem_t
   pTemp += pDrcLibMem->drc_config_size;                // pTemp points to where drc_state_struct_t will be located

   // init drc_processing_t with defaults
   if (drc_processing_defaults(pDrcLibMem->drc_config_ptr) != DRC_SUCCESS)
   {
      return DRC_FAILURE;
   }

   pDrcLibMem->drc_state_struct_ptr =
      (drc_state_struct_t *)pTemp;                      // init drc_lib_mem_t; allocate memory for drc_state_struct_t
   pDrcLibMem->drc_state_struct_size = stateStructSize; // init drc_lib_mem_t
   pTemp += pDrcLibMem->drc_state_struct_size;          // pTemp points to where delayBuffer will be pointing to

   // init drc_state_struct_t
   pDrcLibMem->drc_state_struct_ptr->delayBuffer = (int8 **)pTemp;
   pTemp += stateSize1; // pTemp points to where rmsStateL32 will be pointing to

   pDrcLibMem->drc_state_struct_ptr->rmsStateL32 = (int32 *)pTemp;
   pTemp += stateSize2; // pTemp points to where drcRmsDBL32Q23 will be pointing to

   pDrcLibMem->drc_state_struct_ptr->drcRmsDBL32Q23 = (int32 *)pTemp;
   pTemp += stateSize3; // pTemp points to where targetGainUL32Q15 will be pointing to

   pDrcLibMem->drc_state_struct_ptr->targetGainUL32Q15 = (uint32 *)pTemp;
   pTemp += stateSize4; // pTemp points to where gainUL32Q15 will be pointing to

   pDrcLibMem->drc_state_struct_ptr->gainUL32Q15 = (uint32 *)pTemp;
   pTemp += stateSize5; // pTemp points to where currState will be pointing to

   pDrcLibMem->drc_state_struct_ptr->currState = (DrcStateType *)pTemp;
   pTemp += stateSize6; // pTemp points to where timeConstantUL32Q31 will be pointing to

   pDrcLibMem->drc_state_struct_ptr->timeConstantUL32Q31 = (uint32 *)pTemp;
   pTemp += stateSize7; // pTemp points to where delayBuffer[0] will be pointing to

   pDrcLibMem->drc_state_struct_ptr->dwcomp_state_change = (int32 *)pTemp;
   pTemp += stateSize8; // pTemp points to where dwcomp_state_change[0] will be pointing to

   pDrcLibMem->drc_state_struct_ptr->uwcomp_state_change = (int32 *)pTemp;
   pTemp += stateSize9; // pTemp points to where uwcomp_state_change[0] will be pointing to
   pDrcLibMem->drc_state_struct_ptr->dnexpa_state_change = (int32 *)pTemp;
   pTemp += stateSize10; // pTemp points to where instGainUL64Q27[0] will be pointing to

   pDrcLibMem->drc_state_struct_ptr->instGainUL64Q27 = (uint64 *)pTemp;
   pTemp += stateSize11;

   // initialize dwcomp_state_change and uwcomp_state_change to 1 for each channel
   for (channelNo = 0; channelNo < drc_static_struct_ptr->num_channel; channelNo++)
   {
      pDrcLibMem->drc_state_struct_ptr->dwcomp_state_change[channelNo] = 1;
      pDrcLibMem->drc_state_struct_ptr->uwcomp_state_change[channelNo] = 1;
      pDrcLibMem->drc_state_struct_ptr->dnexpa_state_change[channelNo] = 1;
   }

   pDrcLibMem->drc_state_struct_ptr->inputIndex        = drc_static_struct_ptr->delay;
   pDrcLibMem->drc_state_struct_ptr->processIndex      = 0; // Initialize the current index of the delay buffer
   pDrcLibMem->drc_state_struct_ptr->downSampleCounter = 0; // Reset the down sample counter

   // Note: This function needs to be called to set up new output RMS thresholds every time the input RMS thresholds
   // and/or slope change
   if (output_rms_comp(pDrcLibMem) != DRC_SUCCESS)
   {
      return DRC_FAILURE;
   }

   // init the states section
   // init delayBuffer[channelNo] in the states section; allocate memory for delay buffer
   for (channelNo = 0; channelNo < drc_static_struct_ptr->num_channel; channelNo++)
   {
      pDrcLibMem->drc_state_struct_ptr->delayBuffer[channelNo] = pTemp;
      pTemp += delayBufferSizePerChannel;
   }
   // init the rest of the states
   if (state_memory_defaults(drc_lib_ptr) != DRC_SUCCESS)
   {
      return DRC_FAILURE;
   }

   // check to see if memory partition is correct
   if (pTemp != (int8 *)pMem + libMemSize)
   {
      return DRC_MEMERROR;
   }

   // update drc processing mode
   drc_processing_mode(pDrcLibMem->drc_static_struct_ptr,
                       *pDrcLibMem->drc_feature_mode_ptr,
                       (drc_channel_linking_t)pDrcLibMem->drc_config_ptr->channelLinked,
                       pDrcLibMem->drc_state_struct_ptr);

   return DRC_SUCCESS;
}

/*======================================================================

FUNCTION      drc_processing_defaults

DESCRIPTION   Performs initialization of DRC calibration structure with default values

DEPENDENCIES  Input pointers must not be NULL.

PARAMETERS    pProcessing: [in, out] Pointer to DRC processing structure

SIDE EFFECTS  None

======================================================================*/
DRC_RESULT drc_processing_defaults(drc_config_t *drc_config_ptr)
{
   drc_config_ptr->channelLinked     = CHANNEL_LINKED_DEFAULT;
   drc_config_ptr->downSampleLevel   = DOWNSAMPLE_LEVEL_DEFAULT;
   drc_config_ptr->rmsTavUL16Q16     = RMS_TAV_DEFAULT;
   drc_config_ptr->makeupGainUL16Q12 = MAKEUP_GAIN_DEFAULT;

   drc_config_ptr->dnExpaThresholdL16Q7    = DN_EXPA_THRESHOLD_DEFAULT;
   drc_config_ptr->dnExpaSlopeL16Q8        = DN_EXPA_SLOPE_DEFAULT;
   drc_config_ptr->dnExpaHysterisisUL16Q14 = DN_EXPA_HYSTERISIS_DEFAULT;
   drc_config_ptr->dnExpaAttackUL32Q31     = DN_EXPA_ATTACK_DEFAULT;
   drc_config_ptr->dnExpaReleaseUL32Q31    = DN_EXPA_RELEASE_DEFAULT;
   drc_config_ptr->dnExpaMinGainDBL32Q23   = DN_EXPA_MIN_GAIN_DEFAULT;

   drc_config_ptr->upCompThresholdL16Q7    = UP_COMP_THRESHOLD_DEFAULT;
   drc_config_ptr->upCompSlopeUL16Q16      = UP_COMP_SLOPE_DEFAULT;
   drc_config_ptr->upCompAttackUL32Q31     = UP_COMP_ATTACK_DEFAULT;
   drc_config_ptr->upCompReleaseUL32Q31    = UP_COMP_RELEASE_DEFAULT;
   drc_config_ptr->upCompHysterisisUL16Q14 = UP_COMP_HYSTERISIS_DEFAULT;

   drc_config_ptr->dnCompThresholdL16Q7    = DN_COMP_THRESHOLD_DEFAULT;
   drc_config_ptr->dnCompSlopeUL16Q16      = DN_COMP_SLOPE_DEFAULT;
   drc_config_ptr->dnCompHysterisisUL16Q14 = DN_COMP_HYSTERISIS_DEFAULT;
   drc_config_ptr->dnCompAttackUL32Q31     = DN_COMP_ATTACK_DEFAULT;
   drc_config_ptr->dnCompReleaseUL32Q31    = DN_COMP_RELEASE_DEFAULT;

   return DRC_SUCCESS;
}

/*======================================================================

FUNCTION      state_memory_defaults

DESCRIPTION   Performs initialization of DRC state structure with default values

DEPENDENCIES  Input pointers must not be NULL.

PARAMETERS    pDrcLib: [in, out] Pointer to DRC lib structure

SIDE EFFECTS  None

======================================================================*/

DRC_RESULT state_memory_defaults(drc_lib_t *pDrcLib)

{
   drc_lib_mem_t *      pDrcLibMem = (drc_lib_mem_t *)pDrcLib->lib_mem_ptr;
   drc_static_struct_t *pStatic    = pDrcLibMem->drc_static_struct_ptr;
   drc_state_struct_t * pState     = pDrcLibMem->drc_state_struct_ptr;
   uint32               channelNo;

   for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
   {
      pState->rmsStateL32[channelNo]         = 0;
      pState->drcRmsDBL32Q23[channelNo]      = MIN_RMS_DB_L32Q23; // rmsStateL32 in log domain
      pState->targetGainUL32Q15[channelNo]   = 32768;             // target gain
      pState->gainUL32Q15[channelNo]         = 32768;             // smoothed gain
      pState->currState[channelNo]           = NO_CHANGE;
      pState->timeConstantUL32Q31[channelNo] = 0;
      pState->instGainUL64Q27[channelNo]     = 134217728; // 2^27
   }

   return DRC_SUCCESS;
}

/*======================================================================

FUNCTION      output_rms_comp

DESCRIPTION   Performs calculation of output RMS thresholds based on
input RMS thresholds and the static curve slopes

DEPENDENCIES  Input pointers must not be NULL.

PARAMETERS    drc_lib_mem_ptr: [in, out] Pointer to DRC lib mem structure

SIDE EFFECTS  None

======================================================================*/
DRC_RESULT output_rms_comp(drc_lib_mem_t *drc_lib_mem_ptr)
{
   int16 tempThresholdL16Q7;

   tempThresholdL16Q7 =
      s16_extract_s32_h(s32_mult_s16_u16(s16_sub_s16_s16(drc_lib_mem_ptr->drc_config_ptr->upCompThresholdL16Q7,
                                                         drc_lib_mem_ptr->drc_config_ptr->dnExpaThresholdL16Q7),
                                         drc_lib_mem_ptr->drc_config_ptr->upCompSlopeUL16Q16)); // Q23
   drc_lib_mem_ptr->drc_state_struct_ptr->outDnExpaThresholdL16Q7 =
      s16_add_s16_s16_sat(tempThresholdL16Q7, drc_lib_mem_ptr->drc_config_ptr->dnExpaThresholdL16Q7);
   drc_lib_mem_ptr->drc_state_struct_ptr->outDnCompThresholdL16Q7 =
      drc_lib_mem_ptr->drc_config_ptr->dnCompThresholdL16Q7;
   drc_lib_mem_ptr->drc_state_struct_ptr->outUpCompThresholdL16Q7 =
      drc_lib_mem_ptr->drc_config_ptr->upCompThresholdL16Q7;

   return DRC_SUCCESS;
}

/*======================================================================

FUNCTION      drc_get_param

DESCRIPTION   Get the default calibration params from pDRCLib and store in pMem

DEPENDENCIES  Input pointers must not be NULL.

PARAMETERS    pDrcLib: [in] Pointer to lib structure
paramID: [in] ID of the param
pMem: [out] Pointer to the memory where params are to be stored
memSize:[in] Size of the memory pointed by pMem
pParamSize: [out] Pointer to param size which indicates the size of the retrieved param(s)

SIDE EFFECTS  None

======================================================================*/
DRC_RESULT drc_get_param(drc_lib_t *pDrcLib, uint32 paramID, int8 *pMem, uint32 memSize, uint32 *pParamSize)
{
   drc_lib_mem_t *pDrcLibMem = (drc_lib_mem_t *)pDrcLib->lib_mem_ptr;
   if (NULL == pDrcLibMem)
   {
      return DRC_MEMERROR;
   }

   drc_static_struct_t *pStatic = pDrcLibMem->drc_static_struct_ptr;

   memset(pMem, 0, memSize);

   switch (paramID)
   {
      case DRC_PARAM_FEATURE_MODE:
      {
         // check if the memory buffer has enough space to write the parameter data
         if (memSize >= sizeof(drc_feature_mode_t))
         {

            drc_feature_mode_t *drc_feature_mode_ptr = (drc_feature_mode_t *)pMem;
            *drc_feature_mode_ptr                    = *pDrcLibMem->drc_feature_mode_ptr;

            *pParamSize = sizeof(drc_feature_mode_t);
         }
         else
         {
            return DRC_MEMERROR;
         }
         break;
      }
      case DRC_PARAM_CONFIG:
      {
         // check if the memory buffer has enough space to write the parameter data
         if (memSize >= sizeof(drc_config_t))
         {

            *(drc_config_t *)pMem = *pDrcLibMem->drc_config_ptr;

            *pParamSize = sizeof(drc_config_t);
         }
         else
         {
            return DRC_MEMERROR;
         }
         break;
      }
      case DRC_PARAM_GET_LIB_VER:
      {
         // check if the memory buffer has enough space to write the parameter data
         if (memSize >= sizeof(drc_lib_ver_t))
         {
            *(drc_lib_ver_t *)pMem = DRC_LIB_VER;
            *pParamSize            = sizeof(drc_lib_ver_t);
         }
         else
         {
            return DRC_MEMERROR;
         }
         break;
      }
      case DRC_PARAM_GET_DELAY:
      {
         // check if the memory buffer has enough space to write the parameter data
         if (memSize >= sizeof(drc_delay_t))
         {
            *(drc_delay_t *)pMem = (drc_delay_t)pStatic->delay;
            *pParamSize          = sizeof(drc_delay_t);
         }
         else
         {
            return DRC_MEMERROR;
         }
         break;
      }
      default:
      {

         return DRC_FAILURE;
      }
   }

   return DRC_SUCCESS;
}

/*======================================================================

FUNCTION      drc_set_param

DESCRIPTION   Set the calibration params in the lib memory using the values pointed by pMem

DEPENDENCIES  Input pointers must not be NULL.

PARAMETERS    pDrcLib: [in, out] Pointer to lib structure
paramID: [in] ID of the param
pMem: [in] Pointer to the memory where the values stored are used to set up the params in the lib memory
memSize:[in] Size of the memory pointed by pMem

SIDE EFFECTS  None

======================================================================*/
DRC_RESULT drc_set_param(drc_lib_t *pDrcLib, uint32 paramID, int8 *pMem, uint32 memSize)
{
   drc_lib_mem_t *pDrcLibMem = (drc_lib_mem_t *)pDrcLib->lib_mem_ptr;
   if (NULL == pDrcLibMem)
   {
      return DRC_MEMERROR;
   }

   drc_static_struct_t *pStatic = pDrcLibMem->drc_static_struct_ptr;
   drc_state_struct_t * pState  = pDrcLibMem->drc_state_struct_ptr;
   uint32               channelNo;

   switch (paramID)
   {
      case DRC_PARAM_FEATURE_MODE:
      {
         // copy only when mem size matches to what is allocated in the lib memory
         if (memSize == sizeof(drc_feature_mode_t))
         {
            // set the calibration params in the lib memory
            *pDrcLibMem->drc_feature_mode_ptr = *(drc_feature_mode_t *)pMem;

            // update drc processing mode
            drc_processing_mode(pStatic,
                                *pDrcLibMem->drc_feature_mode_ptr,
                                (drc_channel_linking_t)pDrcLibMem->drc_config_ptr->channelLinked,
                                pDrcLibMem->drc_state_struct_ptr);
         }
         else //
         {

            return DRC_MEMERROR;
         }

         break;
      }
      case DRC_PARAM_CONFIG:
      {
         // copy only when mem size matches to what is allocated in the lib memory
         if (memSize == sizeof(drc_config_t))
         {

            // set the calibration params in the lib memory
            *pDrcLibMem->drc_config_ptr = *(drc_config_t *)pMem;

            // update drc processing mode
            drc_processing_mode(pStatic,
                                *pDrcLibMem->drc_feature_mode_ptr,
                                (drc_channel_linking_t)pDrcLibMem->drc_config_ptr->channelLinked,
                                pDrcLibMem->drc_state_struct_ptr);

            // if at least one of the tuning params which determine output RMS thresholds gets updated above,
            // re-calculate output RMS thresholds again
            if (output_rms_comp(pDrcLibMem) != DRC_SUCCESS)
            {

               return DRC_FAILURE;
            }
         }
         else //
         {

            return DRC_MEMERROR;
         }

         break;
      }
      case DRC_PARAM_SET_RESET:
      {
         // Reset internal states(flush memory) here; wrapper no need to provide memory space for doing this
         if (pMem == NULL && memSize == 0)
         {
            uint32 channel;

            if (state_memory_defaults(pDrcLib) != DRC_SUCCESS)
            {

               return DRC_FAILURE;
            }

            pState->inputIndex        = pStatic->delay;
            pState->processIndex      = 0; // Initialize the current index of the delay buffer
            pState->downSampleCounter = 0; // Reset the down sample counter

            // initialize dwcomp_state_change and uwcomp_state_change to 1 for each channel
            for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
            {
               pState->dwcomp_state_change[channelNo] = 1;
               pState->uwcomp_state_change[channelNo] = 1;
               pState->dnexpa_state_change[channelNo] = 1;
            }

            // clean up the delay buffer
            if (BITS_16 == pStatic->data_width)
            {
               for (channel = 0; channel < pStatic->num_channel; channel++)
               {
                  memset(pState->delayBuffer[channel],
                         0,
                         (size_t)s64_shl_s64(s64_add_s32_u32(ONE, pStatic->delay), ONE));
               }
            }
            else
            {
               for (channel = 0; channel < pStatic->num_channel; channel++)
               {
                  memset(pState->delayBuffer[channel],
                         0,
                         (size_t)s64_shl_s64(s64_add_s32_u32(ONE, pStatic->delay), TWO));
               }
            }
         }
         else
         {
            return DRC_MEMERROR;
         }

         break;
      }

      default:
      {
         return DRC_FAILURE;
      }
   }

   return DRC_SUCCESS;
}

/*======================================================================

FUNCTION      drc_process

DESCRIPTION   Process de-interleaved multi-channel input audio signal
sample by sample. The input can be in any sampling rate
- 8, 16, 22.05, 32, 44.1, 48 KHz. If the input is 16-bit
Q15 and the output is also in the form of 16-bit Q15. If
the input is 32-bit Q27, the output is also in the form of 32-bit Q27.

DEPENDENCIES  Input pointers must not be NULL.

PARAMETERS    pDrcLib: [in] Pointer to lib structure
pOutPtr: [out] Pointer to de-interleaved multi - channel output PCM samples
pInPtr: [in] Pointer to de-interleaved multi - channel input PCM samples
nSamplePerChannel: [in] Number of samples to be processed per channel

SIDE EFFECTS  None.

======================================================================*/

DRC_RESULT drc_process(drc_lib_t *pDrcLib, int8 **pOutPtr, int8 **pInPtr, uint32 nSamplePerChannel)
{
   drc_lib_mem_t *      pDrcLibMem     = (drc_lib_mem_t *)pDrcLib->lib_mem_ptr;
   drc_static_struct_t *pStatic        = pDrcLibMem->drc_static_struct_ptr;
   drc_state_struct_t * pState         = pDrcLibMem->drc_state_struct_ptr;
   drc_config_t *       drc_config_ptr = pDrcLibMem->drc_config_ptr;

   //-------------------- variable declarations -----------------------------
   uint16 negativeDrcTavUL16Q16;
   uint32 delayBuffSize;

   delayBuffSize = (uint32)s64_add_s32_u32(ONE, pStatic->delay);
   // for unsigned Q16, (1-TAV) equals to -TAV
   negativeDrcTavUL16Q16 = s16_neg_s16_sat(drc_config_ptr->rmsTavUL16Q16);

   // switching between drc processing modes (Ying)
   switch (pState->drcProcessMode)
   {
      case ProcessMultiChan16Linked:
         ProcessMC16Linked(pDrcLib, negativeDrcTavUL16Q16, nSamplePerChannel, delayBuffSize, pOutPtr, pInPtr);
         break;

      case ProcessMultiChan16Unlinked:
         ProcessMC16Unlinked(pDrcLib, negativeDrcTavUL16Q16, nSamplePerChannel, delayBuffSize, pOutPtr, pInPtr);
         break;

      case ProcessMultiChan32Linked:
         ProcessMC32Linked(pDrcLib, negativeDrcTavUL16Q16, nSamplePerChannel, delayBuffSize, pOutPtr, pInPtr);
         break;

      case ProcessMultiChan32Unlinked:
         ProcessMC32Unlinked(pDrcLib, negativeDrcTavUL16Q16, nSamplePerChannel, delayBuffSize, pOutPtr, pInPtr);
         break;

      case ProcessOneChan16:
         ProcessMono16(pDrcLib, negativeDrcTavUL16Q16, nSamplePerChannel, delayBuffSize, pOutPtr, pInPtr);
         break;

      case ProcessOneChan32:
         ProcessMono32(pDrcLib, negativeDrcTavUL16Q16, nSamplePerChannel, delayBuffSize, pOutPtr, pInPtr);
         break;

      case ProcessBypass16:
         ProcessBP16(pState, pStatic, nSamplePerChannel, delayBuffSize, pOutPtr, pInPtr);
         break;

      case ProcessBypass32:
         ProcessBP32(pState, pStatic, nSamplePerChannel, delayBuffSize, pOutPtr, pInPtr);
         break;

      default:

         return DRC_FAILURE;
   }

   return DRC_SUCCESS;
}

/*======================================================================

FUNCTION      drc_processing_mode

DESCRIPTION   Checks on the static/calib parameters to determine DRC processing mode

PARAMETERS    pStatic: [in] pointer to the config structure
mode: [in] Bypass or DRC mode
link: [in] channel linked or unlink
state: [out] pointer to the state structure that saves the DRC processing states

RETURN VALUE  Failure or Success

SIDE EFFECTS  None.

======================================================================*/
DRC_RESULT drc_processing_mode(drc_static_struct_t * pStatic,
                               drc_feature_mode_t    mode,
                               drc_channel_linking_t link,
                               drc_state_struct_t *  state)
{

   // DRC process mode determination to avoid checks in the process function (Ying)
   if ((drc_feature_mode_t)DRC_BYPASSED == mode) // DRC Bypass mode;
   {
      if (BITS_16 == pStatic->data_width) // 16bit
      {
         state->drcProcessMode = ProcessBypass16;
      }
      else // 32bit
      {
         state->drcProcessMode = ProcessBypass32;
      }
   }
   else // DRC processing mode
   {
      if (1 == pStatic->num_channel) // mono
      {
         if (BITS_16 == pStatic->data_width) // 16bit
         {
            state->drcProcessMode = ProcessOneChan16;
         }
         else // 32bit
         {
            state->drcProcessMode = ProcessOneChan32;
         }
      }
      else // multichannel
      {
         if (BITS_16 == pStatic->data_width) // 16bit
         {
            if (link == CHANNEL_LINKED) // linked
            {
               state->drcProcessMode = ProcessMultiChan16Linked;
            }
            else // unlinked
            {
               state->drcProcessMode = ProcessMultiChan16Unlinked;
            }
         }
         else // 32bit
         {
            if (link == CHANNEL_LINKED) // linked
            {
               state->drcProcessMode = ProcessMultiChan32Linked;
            }
            else
            {
               state->drcProcessMode = ProcessMultiChan32Unlinked;
            }
         }
      }
   }

   return DRC_SUCCESS;
}

#ifdef DRCLIB_ORIGINAL

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

   for (i = 0; i < nSamplePerChannel; i++)
   {
      for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
      {
         ((int16 *)pState->delayBuffer[channelNo])[pState->inputIndex] = ((int16 *)pInPtr[channelNo])[i];
         ((int16 *)(pOutPtr[channelNo]))[i] = ((int16 *)pState->delayBuffer[channelNo])[pState->processIndex];
      }

      // Check if Delay buffer reaches the cirulary bundary
      pState->processIndex++;
      pState->inputIndex++;

      pState->processIndex = s32_modwrap_s32_u32(pState->processIndex, delayBuffSize);
      pState->inputIndex   = s32_modwrap_s32_u32(pState->inputIndex, delayBuffSize);
   }

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

   for (i = 0; i < nSamplePerChannel; i++)
   {
      for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
      {
         ((int32 *)pState->delayBuffer[channelNo])[pState->inputIndex] = ((int32 *)pInPtr[channelNo])[i];
         ((int32 *)(pOutPtr[channelNo]))[i] = ((int32 *)pState->delayBuffer[channelNo])[pState->processIndex];
      }

      // Check if Delay buffer reaches the cirulary bundary
      pState->processIndex++;
      pState->inputIndex++;

      pState->processIndex = s32_modwrap_s32_u32(pState->processIndex, delayBuffSize);
      pState->inputIndex   = s32_modwrap_s32_u32(pState->inputIndex, delayBuffSize);
   }

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
   uint32 gainNum = 1;
   int32  currDrcRmsL32;

   drc_lib_mem_t *     pDrcLibMem     = (drc_lib_mem_t *)pDrcLib->lib_mem_ptr;
   drc_state_struct_t *pState         = pDrcLibMem->drc_state_struct_ptr;
   drc_config_t *      drc_config_ptr = pDrcLibMem->drc_config_ptr;

   for (i = 0; i < nSamplePerChannel; i++)
   {
      ((int16 *)pState->delayBuffer[0])[pState->inputIndex] = ((int16 *)pInPtr[0])[i];

      //------------ Compute long term RMS of DRC for mono case: ------------
      delayBuffer   = (int16 *)pState->delayBuffer[0];
      currDrcRmsL32 = s32_mult_s16_s16(delayBuffer[pState->inputIndex], delayBuffer[pState->inputIndex]);

      // current_drcRms = previous_drcRms*(1-drcTav) + drcTav * x[n] ^ 2
      pState->rmsStateL32[0] =
         s32_extract_s64_l(s64_add_s64_s64(s64_mult_s32_u16_shift(currDrcRmsL32, drc_config_ptr->rmsTavUL16Q16, 0),
                                           s64_mult_s32_u16_shift(pState->rmsStateL32[0], negativeDrcTavUL16Q16, 0)));

      // compute the current INPUT RMS in dB (log domain)
      // log10_fixed: input L32Q0 format, output Q23 format
      if (pState->rmsStateL32[0] != 0)
      {
         pState->drcRmsDBL32Q23[0] = log10_fixed(pState->rmsStateL32[0]); // log10_fixed is 10*log10(.) in fixed point
      }
      else
      {
         pState->drcRmsDBL32Q23[0] = MIN_RMS_DB_L32Q23;
      }

      // ----------- Compute DRC gain ------------------------
      compute_drc_gain(pState, drc_config_ptr, gainNum);

      // ----------- Apply DRC gain --------------------------
      // delayBuffer = (int16 *) (pState->delayBuffer[0]);
      processData = delayBuffer[pState->processIndex];
      // apply gain and output has same QFactor as input
      tempL32 = s32_saturate_s64(
         s64_shl_s64(s64_add_s64_s32(s64_mult_u32_s16(pState->gainUL32Q15[0], processData), 0x4000), -15));

      if (drc_config_ptr->makeupGainUL16Q12 != MAKEUPGAIN_UNITY) // Implement only non-unity gain
      {
         // Multiply output with the shift normalized makeup gain
         tempOutL64 =
            s64_shl_s64(s64_add_s64_s32(s64_mult_s32_u16(tempL32, drc_config_ptr->makeupGainUL16Q12), 0x800), -12);
         tempL32 = s32_saturate_s64(tempOutL64);
      }

      // output results
      output    = (int16 *)(pOutPtr[0]);
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
   uint32 gainNum = 1;
   int32  currDrcRmsL32;

   drc_lib_mem_t *     pDrcLibMem     = (drc_lib_mem_t *)pDrcLib->lib_mem_ptr;
   drc_state_struct_t *pState         = pDrcLibMem->drc_state_struct_ptr;
   drc_config_t *      drc_config_ptr = pDrcLibMem->drc_config_ptr;

   for (i = 0; i < nSamplePerChannel; i++)
   {

      ((int32 *)pState->delayBuffer[0])[pState->inputIndex] = ((int32 *)pInPtr[0])[i];
      //-------- Compute long term input RMS for DRC ---------
      delayBuffer = (int32 *)pState->delayBuffer[0];
      // x[n] ^ 2
      currDrcRmsL64 = s64_mult_s32_s32(delayBuffer[pState->inputIndex], delayBuffer[pState->inputIndex]);
      // Right shift to get to the same Q-factor as in the 16-bits data width case.
      currDrcRmsL32 = s32_saturate_s64(s64_shl_s64(currDrcRmsL64, s16_shl_s16(s16_sub_s16_s16(Q15, Q27), ONE)));

      // current_drcRms = previous_drcRms*(1-drcTav) + drcTav * x[n] ^ 2
      pState->rmsStateL32[0] =
         s32_extract_s64_l(s64_add_s64_s64(s64_mult_s32_u16_shift(currDrcRmsL32, drc_config_ptr->rmsTavUL16Q16, 0),
                                           s64_mult_s32_u16_shift(pState->rmsStateL32[0], negativeDrcTavUL16Q16, 0)));

      // compute the current INPUT RMS in dB (log domain)
      // log10_fixed: input L32Q0 format, output Q23 format
      if (pState->rmsStateL32[0] != 0)
      {
         pState->drcRmsDBL32Q23[0] = log10_fixed(pState->rmsStateL32[0]); // log10_fixed is 10*log10(.) in fixed point
      }
      else
      {
         pState->drcRmsDBL32Q23[0] = MIN_RMS_DB_L32Q23;
      }

      // ---------- Compute DRC gain -------------------
      compute_drc_gain(pState, drc_config_ptr, gainNum);

      // ---------- Apply DRC gain ----------------------
      delayBuffer = (int32 *)(pState->delayBuffer[0]);
      processData = delayBuffer[pState->processIndex];

      // apply gain and output has same QFactor as input
      tempOutL64 = s64_shl_s64(s64_add_s64_s32(s64_mult_s32_u32(processData, pState->gainUL32Q15[0]), 0x4000), -15);
      tempL32    = s32_saturate_s64(tempOutL64);

      // apply make up gain if needed
      if (drc_config_ptr->makeupGainUL16Q12 != MAKEUPGAIN_UNITY) // Implement only non-unity gain
      {
         // Multiply output with the shift normalized makeup gain
         tempOutL64 =
            s64_shl_s64(s64_add_s64_s32(s64_mult_s32_u16(tempL32, drc_config_ptr->makeupGainUL16Q12), 0x800), -12);
         tempL32 = s32_saturate_s64(tempOutL64);
      }

      // output results
      output    = (int32 *)(pOutPtr[0]);
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
   int32  tempL32[MAX_NUM_CHANNEL];
   int64  tempOutL64;
   int32  currDrcRmsL32[MAX_NUM_CHANNEL];
   int64  squareInputL64;
   uint32 gainNum = 1;

   drc_lib_mem_t *      pDrcLibMem     = (drc_lib_mem_t *)pDrcLib->lib_mem_ptr;
   drc_state_struct_t * pState         = pDrcLibMem->drc_state_struct_ptr;
   drc_static_struct_t *pStatic        = pDrcLibMem->drc_static_struct_ptr;
   drc_config_t *       drc_config_ptr = pDrcLibMem->drc_config_ptr;

   for (i = 0; i < nSamplePerChannel; i++)
   {
      squareInputL64 = 0;

      for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
         ((int16 *)pState->delayBuffer[channelNo])[pState->inputIndex] = ((int16 *)pInPtr[channelNo])[i];

      //-------- Compute long term input RMS for DRC ---------
      // For stereo linked, currDrcRms = (xL(n)^2 + xR(n)^2)/2 (stereo channel example)
      for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
      {
         delayBuffer    = (int16 *)pState->delayBuffer[channelNo];
         squareInputL64 = s64_add_s64_s64(squareInputL64,
                                          s64_mult_s16_s16_shift(delayBuffer[pState->inputIndex],
                                                                 delayBuffer[pState->inputIndex],
                                                                 0));
      }

      // num_channel must be checked during get_mem() and init_mem();
      // Since there is no need to check the range of num_channel here;
      // Can safely assume num_channel is valid;

      // calculate squareInputL64 = squareInputL64/pDrcLib->drc_static_struct.num_channel
      switch (pStatic->num_channel)
      {
         case 2:
            squareInputL64 = s64_shl_s64(squareInputL64, MINUS_ONE); // squareInputL64 >>= 1;
            break;
         case 3:
            // to do: squareInputL64 = (squareInputL64 * ONE_BY_3_Q15) >> Q15. But, need to scale down squareInputL64
            // before multiply since it is int64 to simplify shifting, scale down squareInputL64 by the same amount as
            // Q15 ( (squareInputL64 >> Q15)*ONE_BY_3_Q15 << Q15 ) >> Q15 => (squareInputL64 >> Q15)*ONE_BY_3_Q15
            squareInputL64 =
               s64_mult_s32_s16(s32_saturate_s64(s64_shl_s64(squareInputL64, MINUS_FIFTEEN)), ONE_BY_3_Q15);
            break;
         case 4:
            squareInputL64 = s64_shl_s64(squareInputL64, MINUS_TWO); // squareInputL64 >>= 2;
            break;
         case 5:
            // to do: squareInputL64 = (squareInputL64 * ONE_BY_5_Q15) >> Q15. But, need to scale down squareInputL64
            // before multiply since it is int64 to simplify shifting, scale down squareInputL64 by the same amount as
            // Q15 ( (squareInputL64 >> Q15)*ONE_BY_5_Q15 << Q15 ) >> Q15 => (squareInputL64 >> Q15)*ONE_BY_5_Q15
            squareInputL64 =
               s64_mult_s32_s16(s32_saturate_s64(s64_shl_s64(squareInputL64, MINUS_FIFTEEN)), ONE_BY_5_Q15);
            break;
         case 6:
            // to do: squareInputL64 = (squareInputL64 * ONE_BY_6_Q15) >> Q15. But, need to scale down squareInputL64
            // before multiply since it is int64 to simplify shifting, scale down squareInputL64 by the same amount as
            // Q15 ( (squareInputL64 >> Q15)*ONE_BY_6_Q15 << Q15 ) >> Q15 => (squareInputL64 >> Q15)*ONE_BY_6_Q15
            squareInputL64 =
               s64_mult_s32_s16(s32_saturate_s64(s64_shl_s64(squareInputL64, MINUS_FIFTEEN)), ONE_BY_6_Q15);
            break;
         case 7:
            // to do: squareInputL64 = (squareInputL64 * ONE_BY_7_Q15) >> Q15. But, need to scale down squareInputL64
            // before multiply since it is int64 to simplify shifting, scale down squareInputL64 by the same amount as
            // Q15 ( (squareInputL64 >> Q15)*ONE_BY_7_Q15 << Q15 ) >> Q15 => (squareInputL64 >> Q15)*ONE_BY_7_Q15
            squareInputL64 =
               s64_mult_s32_s16(s32_saturate_s64(s64_shl_s64(squareInputL64, MINUS_FIFTEEN)), ONE_BY_7_Q15);
            break;
         case 8:
            squareInputL64 = s64_shl_s64(squareInputL64, MINUS_THREE); // squareInputL64 >>= 3;
            break;
         case 16:
            squareInputL64 = s64_shl_s64(squareInputL64, MINUS_FOUR); // squareInputL64 >>= 4;
            break;
         case 32:
            squareInputL64 = s64_shl_s64(squareInputL64, MINUS_FIVE); // squareInputL64 >>= 5;
            break;
         default:
            // Default case is for remaining no. of channels till 32
            squareInputL64 = s64_mult_s32_s16(s32_saturate_s64(s64_shl_s64(squareInputL64, MINUS_FIFTEEN)),
                                              rms_constant_factor[pStatic->num_channel]);
            break;
      }
      currDrcRmsL32[0] = s32_saturate_s64(squareInputL64);

      // current_drcRms = previous_drcRms*(1-drcTav) + drcTav * x[n] ^ 2
      pState->rmsStateL32[0] =
         s32_extract_s64_l(s64_add_s64_s64(s64_mult_s32_u16_shift(currDrcRmsL32[0], drc_config_ptr->rmsTavUL16Q16, 0),
                                           s64_mult_s32_u16_shift(pState->rmsStateL32[0], negativeDrcTavUL16Q16, 0)));

      // compute the current INPUT RMS in dB (log domain)
      // log10_fixed: input L32Q0 format, output Q23 format
      if (pState->rmsStateL32[0] != 0)
      {
         pState->drcRmsDBL32Q23[0] = log10_fixed(pState->rmsStateL32[0]); // log10_fixed is 10*log10(.) in fixed point
      }
      else
      {
         pState->drcRmsDBL32Q23[0] = MIN_RMS_DB_L32Q23;
      }

      // ---------- Compute DRC gain -------------------
      compute_drc_gain(pState, drc_config_ptr, gainNum);

      // ---------- Apply DRC gain ----------------------
      for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
      {
         delayBuffer = (int16 *)(pState->delayBuffer[channelNo]);
         processData = delayBuffer[pState->processIndex];

         pState->currState[channelNo]   = pState->currState[0];   // use same currState for all channels
         pState->gainUL32Q15[channelNo] = pState->gainUL32Q15[0]; // use same gain for all channels
         // apply gain and output has same QFactor as input
         tempL32[channelNo] = s32_saturate_s64(
            s64_shl_s64(s64_add_s64_s32(s64_mult_u32_s16(pState->gainUL32Q15[channelNo], processData), 0x4000), -15));
      }

      // apply make up gain if needed
      if (drc_config_ptr->makeupGainUL16Q12 != MAKEUPGAIN_UNITY) // Implement only non-unity gain
      {

         for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
         {
            // Multiply output with the shift normalized makeup gain
            tempOutL64 =
               s64_shl_s64(s64_add_s64_s32(s64_mult_s32_u16(tempL32[channelNo], drc_config_ptr->makeupGainUL16Q12),
                                           0x800),
                           -12);
            tempL32[channelNo] = s32_saturate_s64(tempOutL64);
         }
      }

      // output results
      for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
      {
         output    = (int16 *)(pOutPtr[channelNo]);
         output[i] = s16_saturate_s32(tempL32[channelNo]);
      }

      // Check if Delay buffer reaches the cirulary bundary
      pState->processIndex++;
      pState->inputIndex++;

      pState->processIndex = s32_modwrap_s32_u32(pState->processIndex, delayBuffSize);
      pState->inputIndex   = s32_modwrap_s32_u32(pState->inputIndex, delayBuffSize);
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

   uint32 i, gainNum, channelNo;
   int16 *output;
   int16  processData;
   int16 *delayBuffer;
   int32  tempL32[MAX_NUM_CHANNEL];
   int64  tempOutL64;
   int32  currDrcRmsL32[MAX_NUM_CHANNEL];

   drc_lib_mem_t *      pDrcLibMem     = (drc_lib_mem_t *)pDrcLib->lib_mem_ptr;
   drc_state_struct_t * pState         = pDrcLibMem->drc_state_struct_ptr;
   drc_static_struct_t *pStatic        = pDrcLibMem->drc_static_struct_ptr;
   drc_config_t *       drc_config_ptr = pDrcLibMem->drc_config_ptr;

   for (i = 0; i < nSamplePerChannel; i++)
   {
      for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
         ((int16 *)pState->delayBuffer[channelNo])[pState->inputIndex] = ((int16 *)pInPtr[channelNo])[i];

      //-------- Compute long term input RMS for DRC ---------
      for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
      {
         delayBuffer = (int16 *)pState->delayBuffer[channelNo];
         // x[n] ^ 2 for each channel separately
         currDrcRmsL32[channelNo] = s32_mult_s16_s16(delayBuffer[pState->inputIndex], delayBuffer[pState->inputIndex]);
      }

      gainNum = pStatic->num_channel;

      // current_drcRms = previous_drcRms*(1-drcTav) + drcTav * x[n] ^ 2
      for (channelNo = 0; channelNo < gainNum; channelNo++)
      {
         pState->rmsStateL32[channelNo] = s32_extract_s64_l(
            s64_add_s64_s64(s64_mult_s32_u16_shift(currDrcRmsL32[channelNo], drc_config_ptr->rmsTavUL16Q16, 0),
                            s64_mult_s32_u16_shift(pState->rmsStateL32[channelNo], negativeDrcTavUL16Q16, 0)));

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
      }

      // ---------- Compute DRC gain -------------------
      compute_drc_gain(pState, drc_config_ptr, gainNum);

      // ---------- Apply DRC gain ----------------------
      for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
      {
         delayBuffer = (int16 *)(pState->delayBuffer[channelNo]);
         processData = delayBuffer[pState->processIndex];

         // apply gain and output has same QFactor as input
         tempL32[channelNo] = s32_saturate_s64(
            s64_shl_s64(s64_add_s64_s32(s64_mult_u32_s16(pState->gainUL32Q15[channelNo], processData), 0x4000), -15));
      }
      // apply make up gain if needed
      if (drc_config_ptr->makeupGainUL16Q12 != MAKEUPGAIN_UNITY) // Implement only non-unity gain
      {
         for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
         {
            // Multiply output with the shift normalized makeup gain
            tempOutL64 =
               s64_shl_s64(s64_add_s64_s32(s64_mult_s32_u16(tempL32[channelNo], drc_config_ptr->makeupGainUL16Q12),
                                           0x800),
                           -12);
            tempL32[channelNo] = s32_saturate_s64(tempOutL64);
         }
      }

      // output results
      for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
      {
         output    = (int16 *)(pOutPtr[channelNo]);
         output[i] = s16_saturate_s32(tempL32[channelNo]);
      }

      // Check if Delay buffer reaches the cirulary bundary
      pState->processIndex++;
      pState->inputIndex++;

      pState->processIndex = s32_modwrap_s32_u32(pState->processIndex, delayBuffSize);
      pState->inputIndex   = s32_modwrap_s32_u32(pState->inputIndex, delayBuffSize);
   }

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
   uint32 i, gainNum, channelNo;
   int32 *output;
   int32  processData;
   int32 *delayBuffer;
   int32  tempL32[MAX_NUM_CHANNEL];
   int64  tempOutL64;
   int64  squareInputL64;
   int64  currDrcRmsL64;
   int32  currDrcRmsL32[MAX_NUM_CHANNEL];

   drc_lib_mem_t *      pDrcLibMem     = (drc_lib_mem_t *)pDrcLib->lib_mem_ptr;
   drc_state_struct_t * pState         = pDrcLibMem->drc_state_struct_ptr;
   drc_static_struct_t *pStatic        = pDrcLibMem->drc_static_struct_ptr;
   drc_config_t *       drc_config_ptr = pDrcLibMem->drc_config_ptr;

   for (i = 0; i < nSamplePerChannel; i++)
   {
      squareInputL64 = 0;

      for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
         ((int32 *)pState->delayBuffer[channelNo])[pState->inputIndex] = ((int32 *)pInPtr[channelNo])[i];

      //-------- Compute long term input RMS for DRC ---------
      // For stereo linked, currDrcRms = (xL(n)^2 + xR(n)^2)/2 (stereo channel example)
      for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
      {
         delayBuffer    = (int32 *)pState->delayBuffer[channelNo];
         currDrcRmsL64  = s64_mult_s32_s32(delayBuffer[pState->inputIndex], delayBuffer[pState->inputIndex]);
         currDrcRmsL64  = s64_shl_s64(currDrcRmsL64, s16_shl_s16(s16_sub_s16_s16(Q15, Q27), ONE));
         squareInputL64 = s64_add_s64_s64(squareInputL64, currDrcRmsL64);
      }

      // num_channel must be checked during get_mem() and init_mem();
      // Since there is no need to check the range of num_channel here;
      // Can safely assume num_channel is valid;

      // calculate squareInputL64 = squareInputL64/pDrcLib->drc_static_struct.num_channel
      switch (pStatic->num_channel)
      {
         case 2:
            squareInputL64 = s64_shl_s64(squareInputL64, MINUS_ONE); // squareInputL64 >>= 1;
            break;
         case 3:
            // to do: squareInputL64 = (squareInputL64 * ONE_BY_3_Q15) >> Q15. But, need to scale down squareInputL64
            // before multiply since it is int64 to simplify shifting, scale down squareInputL64 by the same amount as
            // Q15 ( (squareInputL64 >> Q15)*ONE_BY_3_Q15 << Q15 ) >> Q15 => (squareInputL64 >> Q15)*ONE_BY_3_Q15
            squareInputL64 =
               s64_mult_s32_s16(s32_saturate_s64(s64_shl_s64(squareInputL64, MINUS_FIFTEEN)), ONE_BY_3_Q15);
            break;
         case 4:
            squareInputL64 = s64_shl_s64(squareInputL64, MINUS_TWO); // squareInputL64 >>= 2;
            break;
         case 5:
            // to do: squareInputL64 = (squareInputL64 * ONE_BY_5_Q15) >> Q15. But, need to scale down squareInputL64
            // before multiply since it is int64 to simplify shifting, scale down squareInputL64 by the same amount as
            // Q15 ( (squareInputL64 >> Q15)*ONE_BY_5_Q15 << Q15 ) >> Q15 => (squareInputL64 >> Q15)*ONE_BY_5_Q15
            squareInputL64 =
               s64_mult_s32_s16(s32_saturate_s64(s64_shl_s64(squareInputL64, MINUS_FIFTEEN)), ONE_BY_5_Q15);
            break;
         case 6:
            // to do: squareInputL64 = (squareInputL64 * ONE_BY_6_Q15) >> Q15. But, need to scale down squareInputL64
            // before multiply since it is int64 to simplify shifting, scale down squareInputL64 by the same amount as
            // Q15 ( (squareInputL64 >> Q15)*ONE_BY_6_Q15 << Q15 ) >> Q15 => (squareInputL64 >> Q15)*ONE_BY_6_Q15
            squareInputL64 =
               s64_mult_s32_s16(s32_saturate_s64(s64_shl_s64(squareInputL64, MINUS_FIFTEEN)), ONE_BY_6_Q15);
            break;
         case 7:
            // to do: squareInputL64 = (squareInputL64 * ONE_BY_7_Q15) >> Q15. But, need to scale down squareInputL64
            // before multiply since it is int64 to simplify shifting, scale down squareInputL64 by the same amount as
            // Q15 ( (squareInputL64 >> Q15)*ONE_BY_7_Q15 << Q15 ) >> Q15 => (squareInputL64 >> Q15)*ONE_BY_7_Q15
            squareInputL64 =
               s64_mult_s32_s16(s32_saturate_s64(s64_shl_s64(squareInputL64, MINUS_FIFTEEN)), ONE_BY_7_Q15);
            break;
         case 8:
            squareInputL64 = s64_shl_s64(squareInputL64, MINUS_THREE); // squareInputL64 >>= 3;
            break;
         case 16:
            squareInputL64 = s64_shl_s64(squareInputL64, MINUS_FOUR); // squareInputL64 >>= 4;
            break;
         case 32:
            squareInputL64 = s64_shl_s64(squareInputL64, MINUS_FIVE); // squareInputL64 >>= 5;
            break;
         default:
            // Default case is for remaining no. of channels till 32
            squareInputL64 = s64_mult_s32_s16(s32_saturate_s64(s64_shl_s64(squareInputL64, MINUS_FIFTEEN)),
                                              rms_constant_factor[pStatic->num_channel]);
            break;
      }
      currDrcRmsL32[0] = s32_saturate_s64(squareInputL64);

      gainNum = 1;

      // current_drcRms = previous_drcRms*(1-drcTav) + drcTav * x[n] ^ 2
      for (channelNo = 0; channelNo < gainNum; channelNo++)
      {
         pState->rmsStateL32[channelNo] = s32_extract_s64_l(
            s64_add_s64_s64(s64_mult_s32_u16_shift(currDrcRmsL32[channelNo], drc_config_ptr->rmsTavUL16Q16, 0),
                            s64_mult_s32_u16_shift(pState->rmsStateL32[channelNo], negativeDrcTavUL16Q16, 0)));

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
      }

      // ---------- Compute DRC gain -------------------
      compute_drc_gain(pState, drc_config_ptr, gainNum);

      // ---------- Apply DRC gain ----------------------
      for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
      {
         delayBuffer = (int32 *)(pState->delayBuffer[channelNo]);
         processData = delayBuffer[pState->processIndex];

         pState->currState[channelNo]   = pState->currState[0];   // use same currState for all channels
         pState->gainUL32Q15[channelNo] = pState->gainUL32Q15[0]; // use same gain for all channels
         // apply gain and output has same QFactor as input
         tempOutL64 =
            s64_shl_s64(s64_add_s64_s32(s64_mult_s32_u32(processData, pState->gainUL32Q15[channelNo]), 0x4000), -15);
         tempL32[channelNo] = s32_saturate_s64(tempOutL64);
      }

      // apply make up gain if needed
      if (drc_config_ptr->makeupGainUL16Q12 != MAKEUPGAIN_UNITY) // Implement only non-unity gain
      {
         for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
         {
            // Multiply output with the shift normalized makeup gain
            tempOutL64 =
               s64_shl_s64(s64_add_s64_s32(s64_mult_s32_u16(tempL32[channelNo], drc_config_ptr->makeupGainUL16Q12),
                                           0x800),
                           -12);
            tempL32[channelNo] = s32_saturate_s64(tempOutL64);
         }
      }

      // output results
      for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
      {
         output    = (int32 *)(pOutPtr[channelNo]);
         output[i] = tempL32[channelNo];
      }

      // Check if Delay buffer reaches the cirulary bundary
      pState->processIndex++;
      pState->inputIndex++;

      pState->processIndex = s32_modwrap_s32_u32(pState->processIndex, delayBuffSize);
      pState->inputIndex   = s32_modwrap_s32_u32(pState->inputIndex, delayBuffSize);
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
   uint32 i, gainNum, channelNo;
   int32 *output;
   int32  processData;
   int32 *delayBuffer;
   int32  tempL32[MAX_NUM_CHANNEL];
   int64  tempOutL64;
   int64  currDrcRmsL64;
   int32  currDrcRmsL32[MAX_NUM_CHANNEL];

   drc_lib_mem_t *      pDrcLibMem     = (drc_lib_mem_t *)pDrcLib->lib_mem_ptr;
   drc_state_struct_t * pState         = pDrcLibMem->drc_state_struct_ptr;
   drc_static_struct_t *pStatic        = pDrcLibMem->drc_static_struct_ptr;
   drc_config_t *       drc_config_ptr = pDrcLibMem->drc_config_ptr;

   for (i = 0; i < nSamplePerChannel; i++)
   {
      for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
         ((int32 *)pState->delayBuffer[channelNo])[pState->inputIndex] = ((int32 *)pInPtr[channelNo])[i];

      // -------- Compute long term input RMS for DRC ---------
      for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
      {
         delayBuffer = (int32 *)pState->delayBuffer[channelNo];
         // x[n] ^ 2 for each channelNo separately & is same Q-factor as in the 16-bit width case.
         currDrcRmsL64            = s64_mult_s32_s32(delayBuffer[pState->inputIndex], delayBuffer[pState->inputIndex]);
         currDrcRmsL64            = s64_shl_s64(currDrcRmsL64, s16_shl_s16(s16_sub_s16_s16(Q15, Q27), ONE));
         currDrcRmsL32[channelNo] = s32_saturate_s64(currDrcRmsL64);
      }

      gainNum = pStatic->num_channel;

      // current_drcRms = previous_drcRms*(1-drcTav) + drcTav * x[n] ^ 2
      // for(channel = 0; channel < pStatic->num_channel; channel++)
      for (channelNo = 0; channelNo < gainNum; channelNo++)
      {
         pState->rmsStateL32[channelNo] = s32_extract_s64_l(
            s64_add_s64_s64(s64_mult_s32_u16_shift(currDrcRmsL32[channelNo], drc_config_ptr->rmsTavUL16Q16, 0),
                            s64_mult_s32_u16_shift(pState->rmsStateL32[channelNo], negativeDrcTavUL16Q16, 0)));

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
      }

      // ---------- Compute DRC gain -------------------
      compute_drc_gain(pState, drc_config_ptr, gainNum);

      // ---------- Apply DRC gain ----------------------
      for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
      {
         delayBuffer = (int32 *)(pState->delayBuffer[channelNo]);
         processData = delayBuffer[pState->processIndex];

         // apply gain and output has same QFactor as input
         tempOutL64 =
            s64_shl_s64(s64_add_s64_s32(s64_mult_s32_u32(processData, pState->gainUL32Q15[channelNo]), 0x4000), -15);

         tempL32[channelNo] = s32_saturate_s64(tempOutL64);
      }
      // apply make up gain if needed
      if (drc_config_ptr->makeupGainUL16Q12 != MAKEUPGAIN_UNITY) // Implement only non-unity gain
      {
         for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
         {
            // Multiply output with the shift normalized makeup gain
            tempOutL64 =
               s64_shl_s64(s64_add_s64_s32(s64_mult_s32_u16(tempL32[channelNo], drc_config_ptr->makeupGainUL16Q12),
                                           0x800),
                           -12);
            tempL32[channelNo] = s32_saturate_s64(tempOutL64);
         }
      }

      // output results
      for (channelNo = 0; channelNo < pStatic->num_channel; channelNo++)
      {
         output    = (int32 *)(pOutPtr[channelNo]);
         output[i] = tempL32[channelNo];
      }

      // Check if Delay buffer reaches the cirulary bundary
      pState->processIndex++;
      pState->inputIndex++;

      pState->processIndex = s32_modwrap_s32_u32(pState->processIndex, delayBuffSize);
      pState->inputIndex   = s32_modwrap_s32_u32(pState->inputIndex, delayBuffSize);
   }

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

void compute_drc_gain(drc_state_struct_t *pState, drc_config_t *drc_config_ptr, uint32 gainNum)
{

   // ------------ Variable Declaration ------------
   int16        drcRmsDBL16Q7;
   int32        drcRmsDBL32Q23;
   uint32       j, newTargetGainUL32Q15;
   int64        currDrcGainL64Q27, gainDiffL64Q27;
   DrcStateType currState;
   uint16       tempSlopeUL16Q16;
   int16        tempThresholdL16Q7, tempSlopeL16Q8;
   int32        newTargetGainL32Q26, newTargetGainL32Q23;
   int64        tempRmsDBL40Q23;

   //---------------- Update gain computation only if at downSampleLevels --------
   if (0 == pState->downSampleCounter)
   {
      pState->downSampleCounter = drc_config_ptr->downSampleLevel;

      //------------- Update gain based on DRC mode ----------------
      j = 0; // channel index
      do
      {
         currState = pState->currState[j];

         drcRmsDBL32Q23 = pState->drcRmsDBL32Q23[j];         // current INPUT RMS in dB (log domain)
         drcRmsDBL16Q7  = s16_extract_s32_h(drcRmsDBL32Q23); // Q7
         drcRmsDBL16Q7  = s16_max_s16_s16(drcRmsDBL16Q7, MIN_RMS_DB_L16Q7);

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
            newTargetGainL32Q23 =
               s32_mult_s16_u16(s16_sub_s16_s16(tempThresholdL16Q7, drc_config_ptr->dnExpaThresholdL16Q7),
                                tempSlopeUL16Q16); // Q23

            tempThresholdL16Q7 = drc_config_ptr->dnExpaThresholdL16Q7;
            tempSlopeL16Q8     = drc_config_ptr->dnExpaSlopeL16Q8;

            tempRmsDBL40Q23 =
               s64_mult_s16_s16_shift(s16_sub_s16_s16(tempThresholdL16Q7, drcRmsDBL16Q7), tempSlopeL16Q8, 8);
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

         // --- Find out the appropriate time constant based on input RMS ---
         if (drcRmsDBL16Q7 > drc_config_ptr->dnCompThresholdL16Q7)
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
            pState->dnexpa_state_change[j] = 1;
            pState->uwcomp_state_change[j] = 1;
         }
         else if (drcRmsDBL16Q7 < drc_config_ptr->dnExpaThresholdL16Q7)
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

            pState->dnexpa_state_change[j] = 0;
            pState->uwcomp_state_change[j] = 1;
            pState->dwcomp_state_change[j] = 1;
         }
         else if (drcRmsDBL16Q7 < drc_config_ptr->upCompThresholdL16Q7)
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
            pState->dnexpa_state_change[j] = 1;
            pState->dwcomp_state_change[j] = 1;
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
            else if (pState->dnexpa_state_change[j] == 0)
            {
               pState->timeConstantUL32Q31[j] = drc_config_ptr->dnExpaReleaseUL32Q31;
               currState                      = RELEASE;
               pState->dnexpa_state_change[j] = 1;
            }
         }

         // --- calculate DRC gain with determined smooth factor ---
         // drcGain	= drcGain*(1-timeConstant) + drcTargetGain*timeConstant
         //			= drcGain + (drcTargetGain - drcGain)*timeConstant
         gainDiffL64Q27 = s64_sub_s64_s64(((int64)pState->targetGainUL32Q15[j]) << 12, pState->instGainUL64Q27[j]);
         currDrcGainL64Q27 =
            s64_mult_s32_u32_shift(s32_saturate_s64(gainDiffL64Q27), pState->timeConstantUL32Q31[j], 1); // Q27
         currDrcGainL64Q27          = s64_add_s64_s64(currDrcGainL64Q27, pState->instGainUL64Q27[j]);
         pState->gainUL32Q15[j]     = s32_saturate_s64(s64_shl_s64(currDrcGainL64Q27, -12));
         pState->instGainUL64Q27[j] = currDrcGainL64Q27;
         pState->currState[j]       = currState;

         j++;

      } while (j < gainNum);
   }

   pState->downSampleCounter--;
}

#endif // QDSP6_DRCLIB_ASM
