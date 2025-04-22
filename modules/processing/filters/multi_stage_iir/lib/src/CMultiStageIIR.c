/*======================= COPYRIGHT NOTICE ==================================*]
[* Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear                                   *]
[*===========================================================================*]

/*=======================================================================
                     INCLUDE FILES FOR MODULE
========================================================================== */
#include "CMultiStageIIR.h"
#include "audio_iir_tdf2.h"
#include <stringl.h>
#include "audio_basic_op.h"
#include "ar_defs.h"
#ifdef AVS_BUILD_SOS
#include "capi_cmn.h"
#endif
/*-----------------------------------------------------------------------
** Internal function declarations
**-----------------------------------------------------------------------*/
PPStatus msiir_process_one_channel(   CMultiStageIIRLib *CMultiStageIIR_obj,
                                    void* pOut,
                                    void* pInp,
                                    int32 iNrOfSamples,
                                    uint16 k);

PPStatus apply_multistage_iir(
        MSIIRDataStruct* pMultiStageIIRStruct,
        int16 iNumIIRStages,
        void* pOut,
        void* pInp,
        int32 iNrOfSamples,
        int16 uiBitsPerSample);

PPStatus msiir_apply_stf_pregain(
        int32 iSTFPreGain,
        int16 QFactor,
        void* pOut,
        void* pInp,
        int32 iNrOfSamples,
        int16 uiBitsPerSample);


/*-----------------------------------------------------------------------
** Member Function definitions
**-----------------------------------------------------------------------*/

/*
    Reset() clears the arrays in memory that store the states (w1,w2) of the
    system for all IIR stages for all channels
*/
void msiir_reset(CMultiStageIIRLib *CMultiStageIIR_obj)
{
    MSIIRDataStruct* pIIR;
    int k;

    for (k = 0; k < CMultiStageIIR_obj->uiNumChannels; ++k)
    {
        //if (*(CMultiStageIIR_obj->ppMultiStageIIRStruct + k))
        //{
            int16 i;
            int16 j;

            pIIR = *(CMultiStageIIR_obj->ppMultiStageIIRStruct + k);
            for( i = 0; i < *(CMultiStageIIR_obj->puiNumIIRStages + k) ; i++)
            {
                for (j = 0 ; j < MULTISTAGE_IIR_FILTER_STATES ; j++)
                {
                    pIIR->iFiltMemory[j] = 0;
                }
                pIIR++;
            }
        //}
    }
}

/*
    Initialize CMultiStageIIR object using the configuration struct
    The passed-in memory locations are used to store the fields of the CMultiStageIIR object
*/
PPStatus msiir_initialize(CMultiStageIIRLib *CMultiStageIIR_obj,
                    MultiStageIIRCfgStruct* pIIRInpCfg ,
                    MSIIRDataStruct** pIIRFilterStruct,
                    boolean * pFilterEnable,
                    int32 * pPreGain,
                    uint16 * pNumStages )
{
    int16 i;
    uint16 k;
    MSIIRDataStruct* pIIR;
    int32* pFiltCoeff;
    int16* piNumShift;

    /* check pointers */
    if ((NULL == CMultiStageIIR_obj) ||
        (NULL == pIIRFilterStruct) ||
        (NULL == pIIRInpCfg->pbIIRFilterEnable) ||
        (NULL == pIIRInpCfg->puiNumStages) ||
        (NULL == pIIRInpCfg->piPreGain))
    {
        return PPFAILURE;
    }

    /* assign the non-pointer fields */
    CMultiStageIIR_obj->mMultiChannelMode = pIIRInpCfg->mMultiChannelMode;
    CMultiStageIIR_obj->uiNumChannels = pIIRInpCfg->uiNumChannels;
    CMultiStageIIR_obj->uiBitsPerSample = pIIRInpCfg->uiBitsPerSample;

    /* use the memory that was passed in for other fields */
    CMultiStageIIR_obj->pbMultiStageIIREnable = pFilterEnable;
    CMultiStageIIR_obj->piSTFPreGain = pPreGain;
    CMultiStageIIR_obj->puiNumIIRStages = pNumStages;
    CMultiStageIIR_obj->ppMultiStageIIRStruct = pIIRFilterStruct;

    /* The following two lines are only to take care of level-4 warning (C4701).
    We check that pIIRInpCfg->uiNumChannels is valid while calling Initialize, hence
    it is guaranteed that we enter the next for loop, which will necessarily initialize
    pFiltCoeff and piNumShift */
    pFiltCoeff = *(pIIRInpCfg->ppiFilterCoeff);
    piNumShift = *(pIIRInpCfg->ppiNumShiftFactor);

    for (k = 0; k < CMultiStageIIR_obj->uiNumChannels; ++k)
    {
        /* pIIR: pointer to MSIIRDataStructs for current channel */
        pIIR = *(pIIRFilterStruct + k);

        /*
            If mode is MSIIR_UNLINKED, pIIRInpCfg contains different specifications per channel
            If mode is MSIIR_LINKED, use the specifications for the first channel (channel 0)
                in pIIRInpCfg.
        */
        if (MSIIR_UNLINKED == CMultiStageIIR_obj->mMultiChannelMode)
        {
            // if bad pointers in pIIRInpCfg, disable channel
            if ((NULL == *(pIIRInpCfg->ppiFilterCoeff + k)) ||
                (NULL == *(pIIRInpCfg->ppiNumShiftFactor + k)))
            {
                *(CMultiStageIIR_obj->pbMultiStageIIREnable + k) = FALSE;
            }
            else
            {
                // channel-k specifications
                pFiltCoeff = *(pIIRInpCfg->ppiFilterCoeff + k);
                piNumShift = *(pIIRInpCfg->ppiNumShiftFactor + k);
                *(pFilterEnable + k) = *(pIIRInpCfg->pbIIRFilterEnable + k);
                *(pPreGain + k) = *(pIIRInpCfg->piPreGain + k);
                *(pNumStages + k) = *(pIIRInpCfg->puiNumStages + k);
            }
        }
        else /* LINKED */
        {
            // channel-0 specifications
            pFiltCoeff = *(pIIRInpCfg->ppiFilterCoeff);
            piNumShift = *(pIIRInpCfg->ppiNumShiftFactor);
            *(pFilterEnable + k) = *(pIIRInpCfg->pbIIRFilterEnable);
            *(pPreGain + k) = *(pIIRInpCfg->piPreGain);
            *(pNumStages + k) = *(pIIRInpCfg->puiNumStages);
        }

        /*
            assign filter coefficients and numerator shift factors
            for all biquad stages in the current channel
        */
        for (i = 0; i < *(CMultiStageIIR_obj->puiNumIIRStages + k); ++i)
        {
            int16 j;
            for (j = 0; j < MULTISTAGE_IIR_NUM_COEFFS; ++j)
            {
                pIIR->nIIRFilterCoeffs[j] = *pFiltCoeff++;
            }
            for (j = 0; j < MULTISTAGE_IIR_DEN_COEFFS; ++j)
            {
                pIIR->nIIRFilterCoeffs[MULTISTAGE_IIR_NUM_COEFFS + j] = *pFiltCoeff++;
            }

            pIIR->IIRFilterNumShiftFactor = *piNumShift++;
            pIIR++; // move to the next biquad stage
        }
    }

    msiir_reset(CMultiStageIIR_obj);
    return PPSUCCESS;
}

/*
    Calls Process_one_channel for every channel
*/
PPStatus msiir_process(CMultiStageIIRLib *CMultiStageIIR_obj, void** pOut, void** pInp, int32 iNrOfSamples)
{
    PPStatus result;
    uint16 k;

    if ((NULL == pOut) || (NULL == pInp) || (NULL == CMultiStageIIR_obj)) {
        return PPFAILURE;
    }

    if (CMultiStageIIR_obj->uiBitsPerSample == 16)
    {
        int16** pInp_cast;
        int16** pOut_cast;
        pInp_cast = (int16**)pInp;
        pOut_cast = (int16**)pOut;
        for (k = 0; k < CMultiStageIIR_obj->uiNumChannels; ++k)
        {
            result = msiir_process_one_channel(
                CMultiStageIIR_obj,
                *(pOut_cast + k),
                *(pInp_cast + k),
                iNrOfSamples,
                k);
            if (PPSUCCESS != result) {
                return result;
            }
        }
    }
    else
    {
        int32** pInp_cast;
        int32** pOut_cast;
        pInp_cast = (int32**)pInp;
        pOut_cast = (int32**)pOut;
        for (k = 0; k < CMultiStageIIR_obj->uiNumChannels; ++k)
        {
            result = msiir_process_one_channel(
                CMultiStageIIR_obj,
                *(pOut_cast + k),
                *(pInp_cast + k),
                iNrOfSamples,
                k);
            if (PPSUCCESS != result) {
                return result;
            }
        }
    }
    return PPSUCCESS;
}

PPStatus msiir_process_one_channel(CMultiStageIIRLib *CMultiStageIIR_obj, void* pOut, void* pInp, int32 iNrOfSamples, uint16 k)
{
    // k = channel number
    int32 STFPreGain_k;
    MSIIRDataStruct* pIIR_k;
    int16 numStages_k;

    numStages_k = *(CMultiStageIIR_obj->puiNumIIRStages + k);
    pIIR_k = *(CMultiStageIIR_obj->ppMultiStageIIRStruct + k);

    if ( (FALSE == *(CMultiStageIIR_obj->pbMultiStageIIREnable + k))
         || (0==numStages_k) )
    {
        if (pOut==pInp)
        {
            return PPSUCCESS; //inplace, do not copy input to output
        }
        /* filter not enabled, copy_input_to_output (bypass), input and output memory does not overlap */
        int32 bytes_per_sample = CMultiStageIIR_obj->uiBitsPerSample >> 3;
        memscpy(pOut, bytes_per_sample*iNrOfSamples, pInp, bytes_per_sample*iNrOfSamples);
        return PPSUCCESS;
    }

    /* process data */
    /*  If gain == 0, assign_zero_to_output
        If gain == 1, apply_multiband_iir
        Else apply_stf_pregain and apply_multiband_iir
    */
    STFPreGain_k = *(CMultiStageIIR_obj->piSTFPreGain + k);
    if (0 == STFPreGain_k)
    {
        int32 bytes_per_sample = CMultiStageIIR_obj->uiBitsPerSample >> 3;
        memset(pOut, 0, bytes_per_sample*iNrOfSamples);
        return PPSUCCESS;
    }

    PPStatus result = PPSUCCESS;
    if (MSIIR_UNITY_PREGAIN == STFPreGain_k)
    {
        result = apply_multistage_iir(
                pIIR_k,
                numStages_k,
                pOut,
                pInp,
                iNrOfSamples,
                CMultiStageIIR_obj->uiBitsPerSample);
    }
    else
    {
        result = msiir_apply_stf_pregain(
                STFPreGain_k,
                MSIIR_Q_PREGAIN,
                pOut,
                pInp,
                iNrOfSamples,
                CMultiStageIIR_obj->uiBitsPerSample);
        if (PPSUCCESS != result)
        {
            return result;
        }
        result = apply_multistage_iir(
                pIIR_k,
                numStages_k,
                pOut,
                pOut,
                iNrOfSamples,
                CMultiStageIIR_obj->uiBitsPerSample);
    }

    return result;
}


/*
    Call iirTDF2 for all the stages
    If-Else block for data = 16-bit OR 32-bit
*/
PPStatus apply_multistage_iir(MSIIRDataStruct* pMultiStageIIRStruct,
                             int16 iNumIIRStages,
                             void *pOut,
                             void *pInp,
                             int32 iNrOfSamples,
                             int16 uiBitsPerSample)
{
    int32 i;
    const int16 iDenShiftFac = 2;
    MSIIRDataStruct* pIIR;

    pIIR = pMultiStageIIRStruct;

    if (uiBitsPerSample == 16)
    {
        int16 * pInp_new;
        int16 * pOut_new;
        pInp_new = (int16*)pInp;
        pOut_new = (int16*)pOut;

        for (i = 0; i < iNumIIRStages; i++)
        {
            /* Filtering subroutine */
            iirTDF2_16(
                    pInp_new,
                    pOut_new,
                    iNrOfSamples,
                    &pIIR->nIIRFilterCoeffs[0],
                    &pIIR->nIIRFilterCoeffs[MULTISTAGE_IIR_NUM_COEFFS],
                    &pIIR->iFiltMemory[0],
                    pIIR->IIRFilterNumShiftFactor,
                    iDenShiftFac
                    );

            /* For in-place computation, use output buffer (pointer) to feed into input buffer for next iteration */
            pInp_new = pOut_new;

            /* Increment IIR Filter structure pointer for the next biquad stage */
            pIIR++;
        }
    } else {
        int32 * pInp_new;
        int32 * pOut_new;
        pInp_new = (int32*)pInp;
        pOut_new = (int32*)pOut;

        for (i = 0; i < iNumIIRStages; i++)
        {
            /* Filtering subroutine */
            iirTDF2_32(
                    pInp_new,
                    pOut_new,
                    iNrOfSamples,
                    &pIIR->nIIRFilterCoeffs[0],
                    &pIIR->nIIRFilterCoeffs[MULTISTAGE_IIR_NUM_COEFFS],
                    &pIIR->iFiltMemory[0],
                    pIIR->IIRFilterNumShiftFactor,
                    iDenShiftFac
                    );

            /* For in-place computation, use output buffer (pointer) to feed into  input buffer for next iteration */
            pInp_new = pOut_new;

            /* Increment IIR Filter structure pointer for the next stage */
            pIIR++;
        }
    }
    return PPSUCCESS;
}


/*
    Multiply input signal by preGain value
    If-Else block for data = 16-bit OR 32-bit
*/
PPStatus msiir_apply_stf_pregain(
    int32 iSTFPreGain,
    int16 QFactor,
    void *pOut,
    void *pInp,
    int32 iNrOfSamples,
    int16 uiBitsPerSample)
{
    int32 i;
    int16 right_shift;

    if ( (!pInp) || (!pOut) )
    {
        return PPFAILURE;
    }

    right_shift = 32 - QFactor;

    if (uiBitsPerSample == 16)
    {
        int16 * pInp_new;
        int16 * pOut_new;
        pInp_new = (int16*)pInp;
        pOut_new = (int16*)pOut;
        for (i = 0; i < iNrOfSamples ; i++)
        {
            *pOut_new = s16_saturate_s32(s32_saturate_s64(s64_mult_s32_s32_shift((int32)(*pInp_new),iSTFPreGain,right_shift)));
            pInp_new++;
            pOut_new++;
        }
    } else {
        int32 * pInp_new;
        int32 * pOut_new;
        pInp_new = (int32*)pInp;
        pOut_new = (int32*)pOut;
        for (i = 0; i < iNrOfSamples ; i++)
        {
            *pOut_new = s32_saturate_s64(s64_mult_s32_s32_shift(*pInp_new,iSTFPreGain,right_shift));
            pInp_new++;
            pOut_new++;
        }
    }

    return PPSUCCESS;
}


