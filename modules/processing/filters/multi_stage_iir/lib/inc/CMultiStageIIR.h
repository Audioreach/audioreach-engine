/*========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *========================================================================
 */

#ifndef _CMULTISTAGEIIR_H_
#define _CMULTISTAGEIIR_H_

#include "posal_types.h"
#include "audpp_common.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/*=============================================================================
Constants
=============================================================================*/
#define MULTISTAGE_IIR_NUM_COEFFS      3
#define MULTISTAGE_IIR_DEN_COEFFS      2 
#define MULTISTAGE_IIR_FILTER_STATES   2  /* each biquad stage has a memory of length 2 */

#define MSIIR_Q_PREGAIN            27
static const int32 MSIIR_UNITY_PREGAIN = (1 << MSIIR_Q_PREGAIN);

/*=============================================================================
Enumerators
=============================================================================*/
typedef enum
{
    MSIIR_LINKED, // same gain and filtering applied to all channels
    MSIIR_UNLINKED // different specifications per channel
} MultiChannelMode;

/*=============================================================================
Typedefs 
=============================================================================*/
typedef struct _MultiStageIIRCfgStruct
{
    boolean*                pbIIRFilterEnable;   /* Enable flag for IIR filters for all channels */

    uint16                  uiNumChannels;
   
    uint16*                 puiNumStages;         /* no. of biquad stages */

    int32*                  piPreGain;          /*/< PreGain to be applied to input data */
    
    int32**                 ppiFilterCoeff;     /*/< Biquad filter coefficients */
    // *(ppiFilterCoeff + k) is the address to filter coefficients for kth channel:
    // |   first biquad stage   |   second biquad stage  |  third ...
    // | b0 | b1 | b2 | a1 | a2 | b0 | b1 | b2 | a1 | a2 | b0 | ...

    MultiChannelMode        mMultiChannelMode;   /*/< MSIIR_LINKED or MSIIR_UNLINKED */

    int16**                 ppiNumShiftFactor;  /*/< Numerator shift factors*/
    // *(ppiNumShiftFactor + k) is the address to the numerator shift factors (one per biquad stage) for the kth channel

    uint16                  uiBitsPerSample;        /* 16-bit or 32-bit */
} MultiStageIIRCfgStruct;


/* One MSIIRDataStruct per biquad stage, contains 
        filter coefficients, numerator shift factor and memory for states w1 & w2 */
typedef struct _MSIIRDataStruct
{
    int64               iFiltMemory[MULTISTAGE_IIR_FILTER_STATES];

    int32               nIIRFilterCoeffs[MULTISTAGE_IIR_NUM_COEFFS+MULTISTAGE_IIR_DEN_COEFFS];

    int16               IIRFilterNumShiftFactor; /*/< Numerator shift factor */
} MSIIRDataStruct;


typedef struct {
    MSIIRDataStruct**     ppMultiStageIIRStruct;
    //  *(*(ppMultiStageIIRStruct + k) + j) is the j-th stage MSIIRDataStruct for the k-th channel

    boolean*            pbMultiStageIIREnable;
    //  *(pMultiStageIIREnable + k) is the enable flag for the k-th channel

    uint16              uiNumChannels;
    MultiChannelMode    mMultiChannelMode;      /*/< MSIIR_LINKED or MSIIR_UNLINKED */
    
    uint16*             puiNumIIRStages;        /* number of biquad stages for all channels*/

    int32*              piSTFPreGain;           /* preGain for all channels */

    uint16              uiBitsPerSample;            /* Size of input and output data : 16 or 32 */
} CMultiStageIIRLib;


/*
  msiir_reset() clears the states (w1 & w2) for all biquad stages in the IIR filter
*/
void msiir_reset(CMultiStageIIRLib *CMultiStageIIR_obj);


/**
  @brief  Use information in the configuration structure and provided memory to initialize the 
      fields of the CMultiStageIIR object.

    @param[in] CMultiStageIIR_obj – CMultiStageIIR object; the memory for internal pointer fields needs to be 
        passed in, see arguments 3-6
    @param[in] pIIRInpCfg - IIR Filter config Structure
    @param[in] pIIRFilterStruct – properly calloc’ed 
        MSIIRDataStruct**; this memory will store the field
        ppMultiStageIIRStruct
    @param[in] pFilterEnable - properly calloc’ed boolean*; this memory will store the field pMultiStageIIREnable
    @param[in] pPreGain - properly calloc’ed int32*; this memory will store the field piSTFPreGain
    @param[in] pNumStages - properly calloc’ed uint16*; this memory will store the field puiNumIIRStages

    @return
    * - PPSUCCESS -         The initialization was successful.
    * - error code -        There was an error which needs to propagate.
   */
PPStatus msiir_initialize(  CMultiStageIIRLib* CMultiStageIIR_obj, 
                            MultiStageIIRCfgStruct* pIIRInpCfg , 
                            MSIIRDataStruct** pIIRFilterStruct, 
                            boolean* pFilterEnable,
                            int32* pPreGain, 
                            uint16* pNumStages );


/**
    @brief  This function processes data in the input buffer and writes to the output buffer;
        calls Process_one_channel for every channel

    @param[in] CMultiStageIIR_obj – CmultiStageIIR object containing filter details
    @param[in] pOut – array of pointers, each pointing to output data for a channel
    @param[in] pInp – array of pointers, each pointing to input data for a channel
    @param[in] iNrOfSamples – number of samples to be processed

    @return
    * - PPSUCCESS - The initialization was successful.
    * - error code - There was an error which needs to propagate.
    */
PPStatus msiir_process( CMultiStageIIRLib* CMultiStageIIR_obj,
                        void** pOut,
                        void** pInp,
                        int32 iNrOfSamples);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _CMULTISTAGEIIR_H_ */


