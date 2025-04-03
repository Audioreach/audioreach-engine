#ifndef DRCLIB_H
#define DRCLIB_H
/*============================================================================
  @file CDrcLib.h

  Public header file for the Limiter algorithm.

        Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
        SPDX-License-Identifier: BSD-3-Clause
============================================================================*/

/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/


#include "drc_api.h"

#if ((defined __hexagon__) || (defined __qdsp6__))
#define QDSP6_DRCLIB_ASM 1
#elif (defined __XTENSA__)
#define DRCLIB_OPT 1
#else
#define DRCLIB_ORIGINAL 1
#endif


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define DRC_LIB_VER 0X020A020102000000  // 2.10/2.1.2 external(major).external(minor)/major.minor.revision

static const uint32_t drc_max_stack_size = 2048;
// worst case stack mem consumption in bytes;
// this number is obtained offline via stack profiling
// stack mem consumption should be no bigger than this number

/*----------------------------------------------------------------------------
* Constants Definition
* -------------------------------------------------------------------------*/
#define ONE 1
#define TWO 2
//#define THREE 3
#define MINUS_ONE -1
#define MINUS_TWO -2
#define MINUS_THREE -3
#define MINUS_FOUR -4
#define MINUS_FIVE -5
#define ONE_BY_3_Q15 10923
#define ONE_BY_5_Q15 6554
#define ONE_BY_6_Q15 5461
#define ONE_BY_7_Q15 4681
#define Q15 15  // Q factor for Q15
#define Q27 27  // Q factor for Q27
#define MINUS_FIFTEEN -Q15
#define ALIGN8(o)         (((o)+7)&(~7))



/*----------------------------------------------------------------------------
 * Type Declarations
 * -------------------------------------------------------------------------*/
// internal use only
typedef enum DrcStateType
{
    NO_CHANGE   =   0,
    ATTACK      =   1,
    RELEASE     =   2

} DrcStateType;


// internal use only
typedef enum DrcConstants
{

    ONE_OVER_TWENTY_UQ19    = 26214,
    MIN_RMS_DB_L32Q23       = 0,
    MIN_RMS_DB_L16Q7        = -728,
    DB_16384_L32Q23         = 707051520,
    MAKEUPGAIN_UNITY        = 4096,
#ifdef PROD_SPECIFIC_MAX_CH
    MAX_NUM_CHANNEL         = 128
#else
    MAX_NUM_CHANNEL         = 32
#endif

} DrcConstants;

// Ying
typedef enum DrcProcessMode
{
	ProcessMultiChan16Linked = 0,
	ProcessMultiChan16Unlinked = 1,
	ProcessMultiChan32Linked = 2,
	ProcessMultiChan32Unlinked = 3,
	ProcessOneChan16 = 4,
	ProcessOneChan32 = 5,
	ProcessBypass16 = 6,
	ProcessBypass32 = 7
} DrcProcessMode;


// Default calibration parameters; internal use only
typedef enum DrcParamsDefault
{

    MODE_DEFAULT                = 0x0, // 1 is with DRC processing; 0 is no DRC processing(bypassed, only delay is implemented)

    CHANNEL_LINKED_DEFAULT      = 0x0,
    DOWNSAMPLE_LEVEL_DEFAULT    = 0x1,

    RMS_TAV_DEFAULT             = 0x0B78,
    MAKEUP_GAIN_DEFAULT         = 0x1000,

    DN_EXPA_THRESHOLD_DEFAULT   = 0x0A28,
    DN_EXPA_SLOPE_DEFAULT       = 0xFF9A,
    DN_EXPA_ATTACK_DEFAULT      = 0x00B3BAB3,
    DN_EXPA_RELEASE_DEFAULT     = 0x01BF7A00,
    DN_EXPA_HYSTERISIS_DEFAULT  = 0x49A7,
    DN_EXPA_MIN_GAIN_DEFAULT    = 0xFD000000,

    UP_COMP_THRESHOLD_DEFAULT   = 0x0A28,
    UP_COMP_SLOPE_DEFAULT       = 0x0,
    UP_COMP_ATTACK_DEFAULT      = 0x0059FCFC,
    UP_COMP_RELEASE_DEFAULT     = 0x0059FCFC,
    UP_COMP_HYSTERISIS_DEFAULT  = 0x49A7,

    DN_COMP_THRESHOLD_DEFAULT   = 0x1BA8,
    DN_COMP_SLOPE_DEFAULT       = 0xF333,
    DN_COMP_ATTACK_DEFAULT      = 0x06D9931E,
    DN_COMP_RELEASE_DEFAULT     = 0x00120478,
    DN_COMP_HYSTERISIS_DEFAULT  = 0x49A7

} DrcParamsDefault;



//DRC state params structure
typedef struct drc_state_struct_t
{

   int8_t**   delayBuffer;                 // dynamic mem allocation for int8_t* delayBuffer[ch_num]; Delay buffer used to save the channel-delayed samples
   int32_t*   rmsStateL32;                 // dynamic mem allocation for int32_t rmsStateL32[ch_num]; Current sample index
   int32_t*   drcRmsDBL32Q23;              // dynamic mem allocation for int32_t drcRmsDBL32Q23[ch_num];
   uint32_t*  targetGainUL32Q15;           // dynamic mem allocation for uint32_t targetGainUL32Q15[ch_num];
   uint32_t*  gainUL32Q15;                 // dynamic mem allocation for uint32_t gainUL32Q15[ch_num];
   DrcStateType* currState;              // dynamic mem allocation for DrcStateType currState[ch_num];
   uint32_t*  timeConstantUL32Q31;         // dynamic mem allocation for uint32_t timeConstantUL32Q31[ch_num];
   int32_t*   dwcomp_state_change;         // dynamic mem allocation for int32_t* dwcomp_state_change[ch_num]
   int32_t*   uwcomp_state_change;         // dynamic mem allocation for int32_t* uwcomp_state_change[ch_num]
   int32_t*   dnexpa_state_change;         // dynamic mem allocation for int32_t* dnexpa_state_change[ch_num]
   uint64_t*  instGainUL64Q27;		    // dynamic mem allocation for uint64_t instGainUL64Q27[ch_num];

   uint32_t   inputIndex;                 // Current input data index in the delay buffer
   uint32_t   processIndex;              // Current process data index in the delay buffer
   uint32_t   drcProcessMode;           // drc mode for different types of process() Ying


   int16_t    downSampleCounter;

   int16_t    outDnCompThresholdL16Q7;
   int16_t    outDnExpaThresholdL16Q7;
   int16_t    outUpCompThresholdL16Q7;

} drc_state_struct_t;


// DRC lib mem structure
typedef struct drc_lib_mem_t
{
    drc_static_struct_t*  drc_static_struct_ptr;    // ptr to the static struct in mem
    int32_t drc_static_struct_size;                   // size of the allocated mem pointed by the static struct
    drc_feature_mode_t* drc_feature_mode_ptr;
    int32_t drc_feature_mode_size;
    drc_config_t* drc_config_ptr;
    int32_t drc_config_size;
    drc_state_struct_t* drc_state_struct_ptr;       // ptr to the state struct in lib mem
    int32_t drc_state_struct_size;                    // size of the allocated mem pointed by the state struct

} drc_lib_mem_t;




#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* #ifndef DRCLIB_H */
