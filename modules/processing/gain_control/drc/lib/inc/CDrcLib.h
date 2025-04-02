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

#include <stdio.h>

#include "AudioComdef.h"
#include "audpp_common.h" 

#ifndef __GNUC__
#define __attribute__(arg) /* noop */ 
#endif


static const int16  MAX_DELAY_SAMPLE        = 1201;
static const uint16 ONE_OVER_TWENTY_UQ19    = 26214;
static const uint32 DRC_BLOCKSIZE = 80;        /* native DRC frame size       */
static const int32  MIN_RMS_DB_L32Q23       = 0;
static const int16  MIN_RMS_DB_L16Q7        = -728;
static const int32  DB_16384_L32Q23         = 707051520;
static const int16  MAKEUPGAIN_UNITY        = 4096;


/*----------------------------------------------------------------------------
 * Type Declarations
 * -------------------------------------------------------------------------*/

/**
  @brief Data types used in the DRC library
 */
typedef enum
{
    DRC_DISABLED =  0,     
    DRC_ENABLED =   1      

} DrcFeaturesType;

typedef enum
{
    CHANNEL_NOT_LINKED  =   0,      
    CHANNEL_LINKED      =   1      

} DrcStereoLinkedType;

typedef enum
{
    NO_CHANGE   =   0,     
    ATTACK      =   1,      
    RELEASE     =   2

} DrcStateType;

/**
  @brief Default values of tuning parameters and hardcoded parameters used in the algorithm
 */
typedef enum
{
    // Default parameters   
    NUM_CHANNEL_DEFAULT         = 1,
    STEREO_LINKED_DEFAULT       = 1,

    MODE_DEFAULT                = 1,
    DOWNSAMPLE_LEVEL_DEFAULT    = 1,
    DELAY_DEFAULT               = 800,
    RMS_TAV_DEFAULT             = 4,
    MAKEUP_GAIN_DEFAULT         = 4650,

    DN_EXPA_THRESHOLD_DEFAULT   = 0,                 
    DN_EXPA_SLOPE_DEFAULT       = 1,
    DN_EXPA_ATTACK_DEFAULT      = 2,
    DN_EXPA_RELEASE_DEFAULT     = 3,
    DN_EXPA_HYSTERISIS_DEFAULT  = 10023,
    DN_EXPA_MIN_GAIN_DEFAULT    = 32767,

    UP_COMP_THRESHOLD_DEFAULT   = 4,                 
    UP_COMP_SLOPE_DEFAULT       = 5,
    UP_COMP_ATTACK_DEFAULT      = 6,
    UP_COMP_RELEASE_DEFAULT     = 7,
    UP_COMP_HYSTERISIS_DEFAULT  = 10023,

    DN_COMP_THRESHOLD_DEFAULT   = 8,                 
    DN_COMP_SLOPE_DEFAULT       = 9,
    DN_COMP_ATTACK_DEFAULT      = 10,
    DN_COMP_RELEASE_DEFAULT     = 11,
    DN_COMP_HYSTERISIS_DEFAULT  = 10023

} DrcParamsDefault;




/*----------------------------------------------------------------------------
 * Type Declarations for overall configuration and state data structures
 * -------------------------------------------------------------------------*/
/**
  @brief DRC configuration parameter structure containing tuning parameters 
  this structure is used with the Initialize/ReInitial functions, it is part of the
  library interface, the structure is used by external caller to this library.
 */

typedef struct _DrcConfig
{
    // DrcMiscCfg
    int16   numChannel;                 // Q0 Number of channels - Internally set value
    int16   stereoLinked;               // Q0 stereo mode -- Linked or Not-Linked

    int16   mode;                       // Q0 DRC mode code - DRC_DISABLED or DRC_ENABLED
    int16   downSampleLevel;            // Q0 Down Sample Level to save MIPS
    int16   delay;                      // Q0 Delay in samples
    uint16  rmsTavUL16Q16;              // Q16 Time Constant used to compute Input RMS
    uint16  makeupGainUL16Q12;          // Q12 Makeup Gain Value - [258, 64918] absolute

    // DrcStaticCurveCfg
    int16   dnExpaThresholdL16Q7;
    int16   dnExpaSlopeL16Q8;
    uint32  dnExpaAttackUL32Q31;
    uint32  dnExpaReleaseUL32Q31;
    uint16  dnExpaHysterisisUL16Q14;
    int32   dnExpaMinGainDBL32Q23;

    int16   upCompThresholdL16Q7;
    uint16  upCompSlopeUL16Q16;
    uint32  upCompAttackUL32Q31;
    uint32  upCompReleaseUL32Q31;
    uint16  upCompHysterisisUL16Q14;

    int16   dnCompThresholdL16Q7;
    uint16  dnCompSlopeUL16Q16;
    uint32  dnCompAttackUL32Q31;
    uint32  dnCompReleaseUL32Q31;
    uint16  dnCompHysterisisUL16Q14;

} DrcConfig;

/**
  @brief DRC configuration parameter structure containing tuning parameters 
  this structure is used with the ASM optimized code, for 
  library internal use. 
 */
typedef struct _DrcConfigInternal
{
    // Don't change the order of the fields above the line which are used in ASM
    uint16 rmsTavUL16Q16;               // Q16 Time Constant used to compute Input RMS
    uint16 negativeDrcTavUL16Q16;      // for unsigned Q16, (1-TAV) equals to -TAV
    uint16 makeupGainUL16Q12;           // Q12 Makeup Gain Value - [258, 64918] absolute
    int16 delay;                        // Q0 Delay in samples
    int32 downSampleLevel;          // Q0 Down Sample Level to save MIPS

    int32 dnCompThresholdL16Q7;
    int32 dnCompThrMultSlopeL32Q23;
    uint32 dnCompSlopeUL16Q16;

    int32 dnExpaSlopeL16Q8;
    int32 dnExpaThresholdL16Q7;
    int32 dnExpaNewTargetGainL32Q23;
    int32 dnExpaMinGainDBL32Q23;

    int32 upCompThrMultSlopeL32Q23;
    uint32 upCompSlopeUL16Q16;
    int32 upCompThresholdL16Q7;
#if ((defined __hexagon__) || (defined __qdsp6__))
    int32 stereoLinked;
#else
    DrcStereoLinkedType stereoLinked;               // Q0 stereo mode -- Linked or Not-Linked
#endif
    uint32 dnCompHysterisisUL16Q14Asl1;
    int32 outDnCompThresholdL16Q7;
    uint32 dnCompAttackUL32Q31;
    uint32 dnCompReleaseUL32Q31;

    uint32 dnExpaHysterisisUL16Q14Asl1;
    int32 outDnExpaThresholdL16Q7;
    uint32 dnExpaAttackUL32Q31;
    uint32 dnExpaReleaseUL32Q31;

    uint32 upCompHysterisisUL16Q14Asl1;
    int32 outUpCompThresholdL16Q7;
    uint32 upCompAttackUL32Q31;
    uint32 upCompReleaseUL32Q31;
    //------------------------------

    int16   numChannel;                 // Q0 Number of channels - Internally set value

    DrcFeaturesType mode;                       // Q0 DRC mode code - DRC_DISABLED or DRC_ENABLED           

    uint32 dnCompHysterisisUL16Q14;
    uint32 dnExpaHysterisisUL16Q14;
    uint32 upCompHysterisisUL16Q14;

} DrcConfigInternal;


/**
  @brief DRC processing data structure
 */
typedef struct _DrcData
{
    // Don't change the order of the fields which are used in ASM
    int32 *paGainL32Q15[2];
    int16 *pDelayBufferLeftL16;
    int16 *pDelayBufferRightL16;
    int32 rmsStateL32[2];
    int32 *pRmsStateL32;
    int32 downSampleCounter;
#if ((defined __hexagon__) || (defined __qdsp6__))
    uint32 currState[2];
#else
    DrcStateType currState[2];
#endif
    int32 gainL32Q15[2];
    uint32 timeConstantUL32Q31[2];
    int32  dwcomp_state_change[2]; 
    int32  uwcomp_state_change[2]; 
} DrcData;

/*----------------------------------------------------------------------------
 * Type Declarations for overall configuration and state data structures
 * -------------------------------------------------------------------------*/

class CDrcLib {
private:
    
    /*----------------------------------------------------------------------------
    * Constants Definition
    * -------------------------------------------------------------------------*/
    static const int16  MAX_INT16_DB_Q7         = 11559;
    static const int32  ONE_Q23                 = 8388608;
    static const uint16 HALF_UL16_Q16           = 32768;

private:
    DrcConfigInternal m_drcCfgInt __attribute__ ((aligned (8)));
    DrcData m_drcData __attribute__ ((aligned (8)));

    int32 m_aGainL32Q15[2][DRC_BLOCKSIZE] __attribute__ ((aligned (8)));
    int32 m_aRmsStateL32[2*DRC_BLOCKSIZE] __attribute__ ((aligned (8)));
    int16 m_delayBufferLeftL16[MAX_DELAY_SAMPLE+DRC_BLOCKSIZE] __attribute__ ((aligned (8)));
    int16 m_delayBufferRightL16[MAX_DELAY_SAMPLE+DRC_BLOCKSIZE] __attribute__ ((aligned (8)));

    void (*fnpProcess)(DrcConfigInternal *pDrcCfgInt, DrcData *pDrcData,
        int16 *pOutPtrL16, int16 *pOutPtrR16, 
        int16 *pInPtrL16, int16 *pInPtrR16, 
        uint32 nSampleCnt);

public:
    CDrcLib();

    /**
    @brief Process input audio data with DRC algorithm

    @param pInPtrL16: [in] Pointer to 16-bit Q15 input channel data
    @param pInPtrR16: [in] Pointer to 16-bit Q15 input channel data
    @param pOutPtrL16: [out] Pointer to 16-bit Q15 output channel data
    @param pOutPtrR16: [out] Pointer to 16-bit Q15 output channel data
    */
    void Process(int16 *pOutPtrL16,
                 int16 *pOutPtrR16,
                 int16 *pInPtrL16,
                 int16 *pInPtrR16,
                 uint32 nSampleCnt=DRC_BLOCKSIZE);

    /**
    @brief Initialize DRC algorithm

    Performs initialization of data structures for the
    DRC algorithm. Two pointers to two memory is passed for 
    configuring the DRC static configuration
    structure.

    */
    PPStatus Initialize(DrcConfig &cfg);

    PPStatus ReInitialize(DrcConfig &cfg);

    void Reset();
};


#endif /* #ifndef DRCLIB_H */
