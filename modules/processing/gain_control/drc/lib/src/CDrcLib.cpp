/*============================================================================
  FILE:          CDrcLib.cpp

  OVERVIEW:      Implements the drciter algorithm.

  DEPENDENCIES:  None

Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
============================================================================*/

/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <stringl.h>
#include "CDrcLib.h"
#include "audio_basic_op.h"
#include "audio_log10.h"
#include "audio_exp10.h"


#if ((defined __hexagon__) || (defined __qdsp6__))

#define s32_mult_s32_u16_shift16(var1,var2) (int32)Q6_P_vmpyweuh_PP_sat((int64)(var1),(uint64)(var2))
#define s32_lsr_u32(var1,shift) Q6_R_lsr_RR((var1),(shift))
#define s32_msu_s16_u16(var3,var1,var2) Q6_P_mpynac_RR((int64)(var3),(int32)(var1),(uint32)(var2))
#define s32_mult_s32_s16_shift15_sat(var1,var2) (int32)Q6_P_vmpyweh_PP_s1_sat((int64)(var1),(uint64)(var2))
#define s32_mult_s32_u16_shift15_sat(var1,var2) (int32)Q6_P_vmpyweuh_PP_s1_sat((int64)(var1),(uint64)(var2))

extern "C" {
int32 Exp10Fixed(int32 input);
int32 Log10Fixed(int32 input);
}

#else

#define s32_mult_s32_u16_shift16(var1,var2) (int32)s64_mult_s32_u16_shift((var1),(var2),0)
#define s32_lsr_u32(var1,shift) s32_saturate_s64(s64_shl_s64((var1),(-shift)))
#define s32_msu_s16_u16(var3,var1,var2) s32_sub_s32_s32((var3),s32_mult_s16_u16((var1),(var2)))
#define s32_mult_s32_s16_shift15_sat(var1,var2) s32_saturate_s64(s64_mult_s32_s16_shift(((var1)<<1),(var2),0))
#define s32_mult_s32_u16_shift15_sat(var1,var2) s32_saturate_s64(s64_mult_s32_u16_shift((var1),(var2),1))

#define Exp10Fixed exp10_fixed
#define Log10Fixed log10_fixed

#endif


#if ((defined __hexagon__) || (defined __qdsp6__))

extern "C" {
void ComputeInputRmsMono(DrcConfigInternal *pDrcCfg, DrcData *pDrcData, int16 *pInputL16, uint32 nSampleCnt);
}

#else

static void ComputeInputRmsMono(DrcConfigInternal *pDrcCfg, DrcData *pDrcData, int16 *pInputL16, uint32 nSampleCnt)
{
    uint32 i;
    int16 nInputL;
    int32 currDrcRmsLeftL32;

    for (i=0; i<nSampleCnt; i++) {
        nInputL = *pInputL16++;

        // Compute currDrcRms = x(n)^2
        currDrcRmsLeftL32 = s32_mult_s16_s16(nInputL, nInputL);

        // Left channel:  drcRms = drcRms*(1-drcTav) + new_drcRms*drcTav
        pDrcData->rmsStateL32[0] = s32_add_s32_s32(
            s32_mult_s32_u16_shift16(currDrcRmsLeftL32, pDrcCfg->rmsTavUL16Q16),
            s32_mult_s32_u16_shift16(pDrcData->rmsStateL32[0], pDrcCfg->negativeDrcTavUL16Q16));

        pDrcData->pRmsStateL32[i] = pDrcData->rmsStateL32[0];
    }
}

#endif


#if ((defined __hexagon__) || (defined __qdsp6__))

extern "C" {
void ComputeInputRmsStereoLinked(DrcConfigInternal *pDrcCfg, DrcData *pDrcData,
                                 int16 *pInputL16, int16 *pInputR16, uint32 nSampleCnt);
}

#else

static void ComputeInputRmsStereoLinked(DrcConfigInternal *pDrcCfg, DrcData *pDrcData,
                                        int16 *pInputL16, int16 *pInputR16, uint32 nSampleCnt)
{
    uint32 i;
    int16 nInputL, nInputR;
    int32 currDrcRmsLeftL32;
    int64 squareInputL64;

    for (i=0; i<nSampleCnt; i++) {
        nInputL = *pInputL16++;
        nInputR = *pInputR16++;

        // For stereo linked, currDrcRms = (xL(n)^2 + xR(n)^2)/2
        squareInputL64 = s64_add_s32_s32(
            s32_mult_s16_s16(nInputL, nInputL),
            s32_mult_s16_s16(nInputR, nInputR));
        squareInputL64 = s64_shl_s64(squareInputL64, -1);

        currDrcRmsLeftL32 = s32_saturate_s64(squareInputL64);

        // Left channel:  drcRms = drcRms*(1-drcTav) + new_drcRms*drcTav
        pDrcData->rmsStateL32[0] = s32_add_s32_s32(
            s32_mult_s32_u16_shift16(currDrcRmsLeftL32, pDrcCfg->rmsTavUL16Q16),
            s32_mult_s32_u16_shift16(pDrcData->rmsStateL32[0], pDrcCfg->negativeDrcTavUL16Q16));

        pDrcData->pRmsStateL32[i] = pDrcData->rmsStateL32[0];
    }
}

#endif


#if ((defined __hexagon__) || (defined __qdsp6__))

extern "C" {
void ComputeInputRmsStereoUnLinked(DrcConfigInternal *pDrcCfg, DrcData *pDrcData,
                                   int16 *pInputL16, int16 *pInputR16, uint32 nSampleCnt);
}

#else

static void ComputeInputRmsStereoUnLinked(DrcConfigInternal *pDrcCfg, DrcData *pDrcData,
                                          int16 *pInputL16, int16 *pInputR16, uint32 nSampleCnt)
{
    uint32 i;
    int16 nInputL, nInputR;
    int32 currDrcRmsLeftL32, currDrcRmsRightL32;

    for (i=0; i<nSampleCnt; i++) {
        nInputL = *pInputL16++;
        nInputR = *pInputR16++;

        // For left channel computation
        currDrcRmsLeftL32 = s32_mult_s16_s16(nInputL, nInputL);

        // For right channel computation
        currDrcRmsRightL32 = s32_mult_s16_s16(nInputR, nInputR);

        // Left channel:  drcRms = drcRms*(1-drcTav) + new_drcRms*drcTav
        pDrcData->rmsStateL32[0] = s32_add_s32_s32(
            s32_mult_s32_u16_shift16(currDrcRmsLeftL32, pDrcCfg->rmsTavUL16Q16),
            s32_mult_s32_u16_shift16(pDrcData->rmsStateL32[0], pDrcCfg->negativeDrcTavUL16Q16));

        // Right channel: drcRms = drcRms*(1-drcTav) + new_drcRms*drcTav
        pDrcData->rmsStateL32[1] = s32_add_s32_s32(
            s32_mult_s32_u16_shift16(currDrcRmsRightL32, pDrcCfg->rmsTavUL16Q16),
            s32_mult_s32_u16_shift16(pDrcData->rmsStateL32[1], pDrcCfg->negativeDrcTavUL16Q16));

        pDrcData->pRmsStateL32[2*i] = pDrcData->rmsStateL32[0];
        pDrcData->pRmsStateL32[2*i+1] = pDrcData->rmsStateL32[1];
    }
}

#endif


#if ((defined __hexagon__) || (defined __qdsp6__))

extern "C" {
uint32 ComputeTargetGain(DrcConfigInternal *pDrcCfg, int16 drcRmsDBL16Q7);
}

#else

static uint32 ComputeTargetGain(DrcConfigInternal *pDrcCfg, int16 drcRmsDBL16Q7)
{
    uint16 tempSlopeUL16Q16;
    int16 tempThresholdL16Q7, tempSlopeL16Q8;
    int32 newTargetGainL32Q26, newTargetGainL32Q23, newTargetGainUL32Q15;
    int64 tempRmsDBL40Q23;
    int32 tempThrMultSlopeL32Q23;

    /* figure out what part of compression curve the rms value is in */
    if (drcRmsDBL16Q7 > pDrcCfg->dnCompThresholdL16Q7) {
        // in Down Dompression
        tempSlopeUL16Q16 = pDrcCfg->dnCompSlopeUL16Q16;
        tempThrMultSlopeL32Q23 = pDrcCfg->dnCompThrMultSlopeL32Q23;

        // newTarget = dwCompThreshold * dwCompSlopeUL16Q16 - Xrms[n] * dwCompSlopeUL16Q16
        newTargetGainL32Q23 = s32_msu_s16_u16(tempThrMultSlopeL32Q23, drcRmsDBL16Q7, tempSlopeUL16Q16);
    }
    else if (drcRmsDBL16Q7 < pDrcCfg->dnExpaThresholdL16Q7) {
        // in Down Expansion
        // newTarget = (dwExpaThresholdL16Q7 - Xrms[n])*dwExpaSlopeUL16Q16 + ...
        //             (uwCompThresholdL16Q7 - dwExpaThresholdL16Q7)*uwCompSlopeUL16Q16; */

        tempThresholdL16Q7 = pDrcCfg->dnExpaThresholdL16Q7;
        tempSlopeL16Q8 = pDrcCfg->dnExpaSlopeL16Q8;
#if ((defined __hexagon__) || (defined __qdsp6__))
        tempRmsDBL40Q23 = s64_mult_s32_s32_shift(s32_sub_s32_s32(tempThresholdL16Q7, drcRmsDBL16Q7),
            tempSlopeL16Q8, 40);
#else
        tempRmsDBL40Q23 = s64_mult_s16_s16_shift(s32_sub_s32_s32(tempThresholdL16Q7, drcRmsDBL16Q7),
            tempSlopeL16Q8, 8);
#endif
        newTargetGainL32Q23 = s32_saturate_s64(s64_add_s64_s32(tempRmsDBL40Q23,pDrcCfg->dnExpaNewTargetGainL32Q23));
        /* L32Q23 */

        //Limit the gain reduction in Downward Expander part to be dnExpaMinGainDB
        newTargetGainL32Q23 = s32_max_s32_s32(newTargetGainL32Q23, pDrcCfg->dnExpaMinGainDBL32Q23);
    }
    else if (drcRmsDBL16Q7 < pDrcCfg->upCompThresholdL16Q7) {
        // in Up Compression
        tempSlopeUL16Q16 = pDrcCfg->upCompSlopeUL16Q16;
        tempThrMultSlopeL32Q23 = pDrcCfg->upCompThrMultSlopeL32Q23;

        // newTarget = uwCompThreshold * uwCompSlopeUL16Q16  - Xrms[n] * uwCompSlopeUL16Q16
        newTargetGainL32Q23 = s32_msu_s16_u16(tempThrMultSlopeL32Q23, drcRmsDBL16Q7, tempSlopeUL16Q16);
    }
    else {
        newTargetGainL32Q23 = 0x0000;
    }

    /* calculate new target gain = 10^(new target gain log / 20): input L32Q26, out:L32Q15 */
    newTargetGainL32Q26 = s32_mult_s32_u16_shift16(newTargetGainL32Q23, ONE_OVER_TWENTY_UQ19);
    newTargetGainUL32Q15 = Exp10Fixed(newTargetGainL32Q26); /* Q15 */

    return (newTargetGainUL32Q15);
}

#endif


#if ((defined __hexagon__) || (defined __qdsp6__))

extern "C" {
void CalculateDrcGainOneGain(DrcConfigInternal *pDrcCfg, DrcData *pDrcData, uint32 nSampleCnt);
}

#else

static void CalculateDrcGainOneGain(DrcConfigInternal *pDrcCfg, DrcData *pDrcData, uint32 nSampleCnt)
{
    uint32 i;
    int64 currDrcGainL40Q15;
    int32 newTargetGainL32Q15;
    int32 drcRmsDBL32Q23, tempOutL32, outDrcRmsDBL32Q23, gainDiffL32Q15;
    int16 drcRmsDBL16Q7, outDrcRmsDBL16Q7;

    int32 downSampleCounter = pDrcData->downSampleCounter;
    /*DrcStateType*/uint32 currState = pDrcData->currState[0];
    int32 gainL32Q15 = pDrcData->gainL32Q15[0];
    uint32 timeConstantUL32Q31 = pDrcData->timeConstantUL32Q31[0];
    int32 dwcomp_state_change = pDrcData->dwcomp_state_change[0];
    int32 uwcomp_state_change = pDrcData->uwcomp_state_change[0];

    for (i=0; i<nSampleCnt; i++) {
        if (0 == downSampleCounter) {
            downSampleCounter = pDrcCfg->downSampleLevel;

            /* compute the current INPUT RMS in dB (log domain) */
            // log10_fixed: input L32Q0 format, output Q23 format
            if (pDrcData->pRmsStateL32[i] != 0) {
                drcRmsDBL32Q23 = log10_fixed(pDrcData->pRmsStateL32[i]);
            }
            else {
                drcRmsDBL32Q23 = MIN_RMS_DB_L32Q23;
            }
            drcRmsDBL16Q7 = s16_extract_s32_h(drcRmsDBL32Q23);   /* Q7 */
            drcRmsDBL16Q7 = s16_max_s16_s16(drcRmsDBL16Q7, MIN_RMS_DB_L16Q7);

            /* compute the current output RMS in dB (log domain) */
            // outDrcRmsDB = 10*log10(input*pDrcData->gainUL32Q15)^2
            // convert UL32 to L32 format that log10_fixed can take as input
            tempOutL32 = s32_lsr_u32(gainL32Q15, 1);
            // log10_fixed: input L32Q0 format, output Q23 format
            if (tempOutL32 != 0) {
                outDrcRmsDBL32Q23 = log10_fixed(tempOutL32);
                outDrcRmsDBL32Q23 = s32_add_s32_s32(
                    (int32)s64_sub_s64_s64(s64_shl_s64(outDrcRmsDBL32Q23,1),
                    DB_16384_L32Q23), drcRmsDBL32Q23);
            }
            else {
                outDrcRmsDBL32Q23 = MIN_RMS_DB_L32Q23;
            }
            outDrcRmsDBL16Q7 = s16_extract_s32_h(outDrcRmsDBL32Q23);

            /* Compute the Target Gain for the current sample based on input RMS */
            newTargetGainL32Q15 = ComputeTargetGain(pDrcCfg, drcRmsDBL16Q7);

            /* Find out the appropriate time constant based on output RMS */
            if (outDrcRmsDBL16Q7 > pDrcCfg->outDnCompThresholdL16Q7) {
                if (newTargetGainL32Q15 < gainL32Q15) {
                    timeConstantUL32Q31 = pDrcCfg->dnCompAttackUL32Q31;
                    currState = ATTACK;
                }
                else if(newTargetGainL32Q15 >= s32_mult_s32_u16_shift15_sat(gainL32Q15,
                    pDrcCfg->dnCompHysterisisUL16Q14Asl1))
                {
                    timeConstantUL32Q31 = pDrcCfg->dnCompReleaseUL32Q31;
                    currState = RELEASE;
                }
                else {
                    if (currState == ATTACK) {
                        timeConstantUL32Q31 = 0;
                        currState = NO_CHANGE;
                    }
                }
                dwcomp_state_change = 0; /* CR480616 fix */
            }
            else if (outDrcRmsDBL16Q7 < pDrcCfg->outDnExpaThresholdL16Q7) {
                if (s32_mult_s32_u16_shift15_sat(newTargetGainL32Q15,
                    pDrcCfg->dnExpaHysterisisUL16Q14Asl1) < gainL32Q15)
                {
                    timeConstantUL32Q31 = pDrcCfg->dnExpaAttackUL32Q31;
                    currState = ATTACK;
                }
                else if(newTargetGainL32Q15 > gainL32Q15) {
                    timeConstantUL32Q31 = pDrcCfg->dnExpaReleaseUL32Q31;
                    currState = RELEASE;
                }
                else {
                    if (currState == RELEASE) {
                        timeConstantUL32Q31 = 0;
                        currState = NO_CHANGE;
                    }
                }
            }
            else if (outDrcRmsDBL16Q7 < pDrcCfg->outUpCompThresholdL16Q7) {
                if (newTargetGainL32Q15 < gainL32Q15) {
                    timeConstantUL32Q31 = pDrcCfg->upCompAttackUL32Q31;
                    currState = ATTACK;
                }
                else if(newTargetGainL32Q15 >= s32_mult_s32_u16_shift15_sat(gainL32Q15,
                    pDrcCfg->upCompHysterisisUL16Q14Asl1))
                {
                    timeConstantUL32Q31 = pDrcCfg->upCompReleaseUL32Q31;
                    currState = RELEASE;
                }
                else {
                    if (currState == ATTACK) {
                        timeConstantUL32Q31 = 0;
                        currState = NO_CHANGE;
                    }
                }
                uwcomp_state_change = 0; /* CR480616 fix */
            }
            else {
                if (drcRmsDBL16Q7 > pDrcCfg->dnCompThresholdL16Q7) {
                    timeConstantUL32Q31 = pDrcCfg->dnCompAttackUL32Q31;
                    currState = ATTACK;
                }
                else if(drcRmsDBL16Q7 < pDrcCfg->dnExpaThresholdL16Q7) {
                    timeConstantUL32Q31 = pDrcCfg->dnExpaAttackUL32Q31;
                    currState = ATTACK;
                }
                else if (drcRmsDBL16Q7 < pDrcCfg->upCompThresholdL16Q7) {
                    timeConstantUL32Q31 = pDrcCfg->upCompReleaseUL32Q31;
                    currState = RELEASE;
                }
                else {
                    /* timeConstantUL32Q31 = 0; */ /* CR270058 fix */
                    currState = NO_CHANGE;
                    /* CR480616 fix */
                    if (dwcomp_state_change == 0)
                    {
                       timeConstantUL32Q31 = pDrcCfg->dnCompReleaseUL32Q31;
                       currState = RELEASE;
                       dwcomp_state_change = 1;
                    }
                    else if (uwcomp_state_change == 0)
                    {
                       timeConstantUL32Q31 = pDrcCfg->upCompAttackUL32Q31;
                       currState = ATTACK;
                       uwcomp_state_change = 1;
                    }
                }
            }

            /* calculate DRC gain with determined smooth factor */
            // drcGain  = drcGain*(1-timeConstant) + drcTargetGain*timeConstant
            //          = drcGain + (drcTargetGain - drcGain)*timeConstant
            gainDiffL32Q15 = s32_sub_s32_s32(newTargetGainL32Q15, gainL32Q15);
            currDrcGainL40Q15 = s64_mult_s32_u32_shift(gainDiffL32Q15, timeConstantUL32Q31, 1);
            currDrcGainL40Q15 = s64_add_s64_s64(currDrcGainL40Q15, gainL32Q15);

            gainL32Q15 = s32_saturate_s64(currDrcGainL40Q15);
        }

        downSampleCounter--;

        pDrcData->paGainL32Q15[0][i] = gainL32Q15;
    }

    pDrcData->downSampleCounter = downSampleCounter;
	
	/*For other than Hexagon processors we need below typecasting change otherwise we hit with compilation error */
#ifdef QDSP6_DRCLIB_ASM
    pDrcData->currState[0]      = currState;
#else
    pDrcData->currState[0] = (DrcStateType)currState;
#endif

    pDrcData->gainL32Q15[0] = gainL32Q15;
    pDrcData->timeConstantUL32Q31[0] = timeConstantUL32Q31;
    pDrcData->dwcomp_state_change[0] = dwcomp_state_change;
    pDrcData->uwcomp_state_change[0] = uwcomp_state_change;
}

#endif


#if ((defined __hexagon__) || (defined __qdsp6__))

extern "C" {
void CalculateDrcGainTwoGain(DrcConfigInternal *pDrcCfg, DrcData *pDrcData, uint32 nSampleCnt);
}

#else

static void CalculateDrcGainTwoGain(DrcConfigInternal *pDrcCfg, DrcData *pDrcData, uint32 nSampleCnt)
{
    uint32 i;
    int64 currDrcGainL40Q15;
    int32 newTargetGainL32Q15;
    int32 drcRmsDBL32Q23, tempOutL32, outDrcRmsDBL32Q23, gainDiffL32Q15;
    int16 drcRmsDBL16Q7, outDrcRmsDBL16Q7;

    int32 downSampleCounter = pDrcData->downSampleCounter;

    /*DrcStateType*/uint32 currStateL = pDrcData->currState[0];
    int32 gainL32Q15L = pDrcData->gainL32Q15[0];
    uint32 timeConstantUL32Q31L = pDrcData->timeConstantUL32Q31[0];
    int32 dwcomp_state_changeL = pDrcData->dwcomp_state_change[0];
    int32 uwcomp_state_changeL = pDrcData->uwcomp_state_change[0];

    /*DrcStateType*/uint32 currStateR = pDrcData->currState[1];
    int32 gainL32Q15R = pDrcData->gainL32Q15[1];
    uint32 timeConstantUL32Q31R = pDrcData->timeConstantUL32Q31[1];
    int32 dwcomp_state_changeR = pDrcData->dwcomp_state_change[1];
    int32 uwcomp_state_changeR = pDrcData->uwcomp_state_change[1];

    for (i=0; i<nSampleCnt; i++) {
        if (0 == downSampleCounter) {
            downSampleCounter = pDrcCfg->downSampleLevel;

            /* compute the current INPUT RMS in dB (log domain) */
            // log10_fixed: input L32Q0 format, output Q23 format
            if (pDrcData->pRmsStateL32[2*i] != 0) {
                drcRmsDBL32Q23 = log10_fixed(pDrcData->pRmsStateL32[2*i]);
            }
            else {
                drcRmsDBL32Q23 = MIN_RMS_DB_L32Q23;
            }
            drcRmsDBL16Q7 = s16_extract_s32_h(drcRmsDBL32Q23);   /* Q7 */
            drcRmsDBL16Q7 = s16_max_s16_s16(drcRmsDBL16Q7, MIN_RMS_DB_L16Q7);

            /* compute the current output RMS in dB (log domain) */
            // outDrcRmsDB = 10*log10(input*pDrcData->gainUL32Q15)^2
            // convert UL32 to L32 format that log10_fixed can take as input
            tempOutL32 = s32_lsr_u32(gainL32Q15L, 1);
            // log10_fixed: input L32Q0 format, output Q23 format
            if (tempOutL32 != 0) {
                outDrcRmsDBL32Q23 = log10_fixed(tempOutL32);
                outDrcRmsDBL32Q23 = s32_add_s32_s32(
                    (int32)s64_sub_s64_s64(s64_shl_s64(outDrcRmsDBL32Q23,1),
                    DB_16384_L32Q23), drcRmsDBL32Q23);
            }
            else {
                outDrcRmsDBL32Q23 = MIN_RMS_DB_L32Q23;
            }
            outDrcRmsDBL16Q7 = s16_extract_s32_h(outDrcRmsDBL32Q23);

            /* Compute the Target Gain for the current sample based on input RMS */
            newTargetGainL32Q15 = ComputeTargetGain(pDrcCfg, drcRmsDBL16Q7);

            /* Find out the appropriate time constant based on output RMS */
            if (outDrcRmsDBL16Q7 > pDrcCfg->outDnCompThresholdL16Q7) {
                if (newTargetGainL32Q15 < gainL32Q15L) {
                    timeConstantUL32Q31L = pDrcCfg->dnCompAttackUL32Q31;
                    currStateL = ATTACK;
                }
                else if(newTargetGainL32Q15 >= s32_mult_s32_u16_shift15_sat(gainL32Q15L,
                    pDrcCfg->dnCompHysterisisUL16Q14Asl1))
                {
                    timeConstantUL32Q31L = pDrcCfg->dnCompReleaseUL32Q31;
                    currStateL = RELEASE;
                }
                else {
                    if (currStateL == ATTACK) {
                        timeConstantUL32Q31L = 0;
                        currStateL = NO_CHANGE;
                    }
                }
                dwcomp_state_changeL = 0; /* CR480616 fix */
            }
            else if (outDrcRmsDBL16Q7 < pDrcCfg->outDnExpaThresholdL16Q7) {
                if (s32_mult_s32_u16_shift15_sat(newTargetGainL32Q15,
                    pDrcCfg->dnExpaHysterisisUL16Q14Asl1) < gainL32Q15L)
                {
                    timeConstantUL32Q31L = pDrcCfg->dnExpaAttackUL32Q31;
                    currStateL = ATTACK;
                }
                else if(newTargetGainL32Q15 > gainL32Q15L) {
                    timeConstantUL32Q31L = pDrcCfg->dnExpaReleaseUL32Q31;
                    currStateL = RELEASE;
                }
                else {
                    if (currStateL == RELEASE) {
                        timeConstantUL32Q31L = 0;
                        currStateL = NO_CHANGE;
                    }
                }
            }
            else if (outDrcRmsDBL16Q7 < pDrcCfg->outUpCompThresholdL16Q7) {
                if (newTargetGainL32Q15 < gainL32Q15L) {
                    timeConstantUL32Q31L = pDrcCfg->upCompAttackUL32Q31;
                    currStateL = ATTACK;
                }
                else if(newTargetGainL32Q15 >= s32_mult_s32_u16_shift15_sat(gainL32Q15L,
                    pDrcCfg->upCompHysterisisUL16Q14Asl1))
                {
                    timeConstantUL32Q31L = pDrcCfg->upCompReleaseUL32Q31;
                    currStateL = RELEASE;
                }
                else {
                    if (currStateL == ATTACK) {
                        timeConstantUL32Q31L = 0;
                        currStateL = NO_CHANGE;
                    }
                }
                uwcomp_state_changeL = 0; /* CR480616 fix */
            }
            else {
                if (drcRmsDBL16Q7 > pDrcCfg->dnCompThresholdL16Q7) {
                    timeConstantUL32Q31L = pDrcCfg->dnCompAttackUL32Q31;
                    currStateL = ATTACK;
                }
                else if(drcRmsDBL16Q7 < pDrcCfg->dnExpaThresholdL16Q7) {
                    timeConstantUL32Q31L = pDrcCfg->dnExpaAttackUL32Q31;
                    currStateL = ATTACK;
                }
                else if (drcRmsDBL16Q7 < pDrcCfg->upCompThresholdL16Q7) {
                    timeConstantUL32Q31L = pDrcCfg->upCompReleaseUL32Q31;
                    currStateL = RELEASE;
                }
                else {
                    /* timeConstantUL32Q31L = 0; */ /* CR270058 fix */
                    currStateL = NO_CHANGE;
                    /* CR480616 fix */
                    if (dwcomp_state_changeL == 0)
                    {
                        timeConstantUL32Q31L = pDrcCfg->dnCompReleaseUL32Q31;
                        currStateL = RELEASE;
                        dwcomp_state_changeL = 1;
                    } else if (uwcomp_state_changeL == 0) {
                        timeConstantUL32Q31L = pDrcCfg->upCompAttackUL32Q31;
                        currStateL = ATTACK;
                        uwcomp_state_changeL = 1;
                    }
                }
            }

            /* calculate DRC gain with determined smooth factor */
            // drcGain  = drcGain*(1-timeConstant) + drcTargetGain*timeConstant
            //          = drcGain + (drcTargetGain - drcGain)*timeConstant
            gainDiffL32Q15 = s32_sub_s32_s32(newTargetGainL32Q15, gainL32Q15L);
            currDrcGainL40Q15 = s64_mult_s32_u32_shift(gainDiffL32Q15, timeConstantUL32Q31L, 1);
            currDrcGainL40Q15 = s64_add_s64_s64(currDrcGainL40Q15, gainL32Q15L);

            gainL32Q15L = s32_saturate_s64(currDrcGainL40Q15);


            /* compute the current INPUT RMS in dB (log domain) */
            // log10_fixed: input L32Q0 format, output Q23 format
            if (pDrcData->pRmsStateL32[2*i+1] != 0) {
                drcRmsDBL32Q23 = log10_fixed(pDrcData->pRmsStateL32[2*i+1]);
            }
            else {
                drcRmsDBL32Q23 = MIN_RMS_DB_L32Q23;
            }
            drcRmsDBL16Q7 = s16_extract_s32_h(drcRmsDBL32Q23);   /* Q7 */
            drcRmsDBL16Q7 = s16_max_s16_s16(drcRmsDBL16Q7, MIN_RMS_DB_L16Q7);

            /* compute the current output RMS in dB (log domain) */
            // outDrcRmsDB = 10*log10(input*pDrcData->gainUL32Q15)^2
            // convert UL32 to L32 format that log10_fixed can take as input
            tempOutL32 = s32_lsr_u32(gainL32Q15R, 1);
            // log10_fixed: input L32Q0 format, output Q23 format
            if (tempOutL32 != 0) {
                outDrcRmsDBL32Q23 = log10_fixed(tempOutL32);
                outDrcRmsDBL32Q23 = s32_add_s32_s32(
                    (int32)s64_sub_s64_s64(s64_shl_s64(outDrcRmsDBL32Q23,1),
                    DB_16384_L32Q23), drcRmsDBL32Q23);
            }
            else {
                outDrcRmsDBL32Q23 = MIN_RMS_DB_L32Q23;
            }
            outDrcRmsDBL16Q7 = s16_extract_s32_h(outDrcRmsDBL32Q23);

            /* Compute the Target Gain for the current sample based on input RMS */
            newTargetGainL32Q15 = ComputeTargetGain(pDrcCfg, drcRmsDBL16Q7);

            /* Find out the appropriate time constant based on output RMS */
            if (outDrcRmsDBL16Q7 > pDrcCfg->outDnCompThresholdL16Q7) {
                if (newTargetGainL32Q15 < gainL32Q15R) {
                    timeConstantUL32Q31R = pDrcCfg->dnCompAttackUL32Q31;
                    currStateR = ATTACK;
                }
                else if(newTargetGainL32Q15 >= s32_mult_s32_u16_shift15_sat(gainL32Q15R,
                    pDrcCfg->dnCompHysterisisUL16Q14Asl1))
                {
                    timeConstantUL32Q31R = pDrcCfg->dnCompReleaseUL32Q31;
                    currStateR = RELEASE;
                }
                else {
                    if (currStateR == ATTACK) {
                        timeConstantUL32Q31R = 0;
                        currStateR = NO_CHANGE;
                    }
                }
                dwcomp_state_changeR = 0; /* CR480616 fix */
            }
            else if (outDrcRmsDBL16Q7 < pDrcCfg->outDnExpaThresholdL16Q7) {
                if (s32_mult_s32_u16_shift15_sat(newTargetGainL32Q15,
                    pDrcCfg->dnExpaHysterisisUL16Q14Asl1) < gainL32Q15R)
                {
                    timeConstantUL32Q31R = pDrcCfg->dnExpaAttackUL32Q31;
                    currStateR = ATTACK;
                }
                else if(newTargetGainL32Q15 > gainL32Q15R) {
                    timeConstantUL32Q31R = pDrcCfg->dnExpaReleaseUL32Q31;
                    currStateR = RELEASE;
                }
                else {
                    if (currStateR == RELEASE) {
                        timeConstantUL32Q31R = 0;
                        currStateR = NO_CHANGE;
                    }
                }
            }
            else if (outDrcRmsDBL16Q7 < pDrcCfg->outUpCompThresholdL16Q7) {
                if (newTargetGainL32Q15 < gainL32Q15R) {
                    timeConstantUL32Q31R = pDrcCfg->upCompAttackUL32Q31;
                    currStateR = ATTACK;
                }
                else if(newTargetGainL32Q15 >= s32_mult_s32_u16_shift15_sat(gainL32Q15R,
                    pDrcCfg->upCompHysterisisUL16Q14Asl1))
                {
                    timeConstantUL32Q31R = pDrcCfg->upCompReleaseUL32Q31;
                    currStateR = RELEASE;
                }
                else {
                    if (currStateR == ATTACK) {
                        timeConstantUL32Q31R = 0;
                        currStateR = NO_CHANGE;
                    }
                }
                uwcomp_state_changeR = 0; /* CR480616 fix */
            }
            else {
                if (drcRmsDBL16Q7 > pDrcCfg->dnCompThresholdL16Q7) {
                    timeConstantUL32Q31R = pDrcCfg->dnCompAttackUL32Q31;
                    currStateR = ATTACK;
                }
                else if(drcRmsDBL16Q7 < pDrcCfg->dnExpaThresholdL16Q7) {
                    timeConstantUL32Q31R = pDrcCfg->dnExpaAttackUL32Q31;
                    currStateR = ATTACK;
                }
                else if (drcRmsDBL16Q7 < pDrcCfg->upCompThresholdL16Q7) {
                    timeConstantUL32Q31R = pDrcCfg->upCompReleaseUL32Q31;
                    currStateR = RELEASE;
                }
                else {
                    /* timeConstantUL32Q31R = 0; */ /* CR270058 fix */
                    currStateR = NO_CHANGE;
                    /* CR480616 fix */
                    if (dwcomp_state_changeR == 0) {
                        timeConstantUL32Q31R = pDrcCfg->dnCompReleaseUL32Q31;
                        currStateR = RELEASE;
                        dwcomp_state_changeR = 1;
                    } else if (uwcomp_state_changeR == 0) {
                        timeConstantUL32Q31R = pDrcCfg->upCompAttackUL32Q31;
                        currStateR = ATTACK;
                        uwcomp_state_changeR = 1;
                    }
                }
            }

            /* calculate DRC gain with determined smooth factor */
            // drcGain  = drcGain*(1-timeConstant) + drcTargetGain*timeConstant
            //          = drcGain + (drcTargetGain - drcGain)*timeConstant
            gainDiffL32Q15 = s32_sub_s32_s32(newTargetGainL32Q15, gainL32Q15R);
            currDrcGainL40Q15 = s64_mult_s32_u32_shift(gainDiffL32Q15, timeConstantUL32Q31R, 1);
            currDrcGainL40Q15 = s64_add_s64_s64(currDrcGainL40Q15, gainL32Q15R);

            gainL32Q15R = s32_saturate_s64(currDrcGainL40Q15);
        }

        downSampleCounter--;

        pDrcData->paGainL32Q15[0][i] = gainL32Q15L;
        pDrcData->paGainL32Q15[1][i] = gainL32Q15R;
    }

    pDrcData->downSampleCounter = downSampleCounter;

#ifdef QDSP6_DRCLIB_ASM
    pDrcData->currState[0]      = currStateL;
#else
    pDrcData->currState[0] = (DrcStateType)currStateL;
#endif

    pDrcData->gainL32Q15[0]          = gainL32Q15L;
    pDrcData->timeConstantUL32Q31[0] = timeConstantUL32Q31L;
    pDrcData->dwcomp_state_change[0] = dwcomp_state_changeL;
    pDrcData->uwcomp_state_change[0] = uwcomp_state_changeL;
#ifdef QDSP6_DRCLIB_ASM
    pDrcData->currState[1]           = currStateR;
#else
    pDrcData->currState[1] = (DrcStateType)currStateR;
#endif

    pDrcData->gainL32Q15[1]          = gainL32Q15R;
    pDrcData->timeConstantUL32Q31[1] = timeConstantUL32Q31R;
    pDrcData->dwcomp_state_change[1] = dwcomp_state_changeR;
    pDrcData->uwcomp_state_change[1] = uwcomp_state_changeR;
}

#endif


#if ((defined __hexagon__) || (defined __qdsp6__))

extern "C" {
void ApplyGainMono(DrcConfigInternal *pDrcCfg, DrcData *pDrcData, uint32 nSampleCnt);
}

#else

static void ApplyGainMono(DrcConfigInternal *pDrcCfg, DrcData *pDrcData, uint32 nSampleCnt)
{
    uint32 i;
    int16 processData;
    int32 tempOutL32;
    int64 tempOutL64;
    int16 *pOutputL16 = pDrcData->pDelayBufferLeftL16;

    if (MAKEUPGAIN_UNITY != pDrcCfg->makeupGainUL16Q12) {
        for (i=0; i<nSampleCnt; i++) {
            processData = pDrcData->pDelayBufferLeftL16[i];
            tempOutL32 = s32_mult_s32_s16_shift15_sat(pDrcData->paGainL32Q15[0][i], processData);

            tempOutL64 = s64_mult_s32_u16_shift(tempOutL32, pDrcCfg->makeupGainUL16Q12, 4);
            tempOutL32 = s32_saturate_s64(tempOutL64);

            *pOutputL16++ = s16_saturate_s32(tempOutL32);
        }
    }
    else {
        for (i=0; i<nSampleCnt; i++) {
            processData = pDrcData->pDelayBufferLeftL16[i];
            tempOutL32 = s32_mult_s32_s16_shift15_sat(pDrcData->paGainL32Q15[0][i], processData);

            *pOutputL16++ = s16_saturate_s32(tempOutL32);
        }
    }
}

#endif


#if ((defined __hexagon__) || (defined __qdsp6__))

extern "C" {
void ApplyGainStereoLinked(DrcConfigInternal *pDrcCfg, DrcData *pDrcData, uint32 nSampleCnt);
}

#else

static void ApplyGainStereoLinked(DrcConfigInternal *pDrcCfg, DrcData *pDrcData, uint32 nSampleCnt)
{
    uint32 i;
    int16 processData;
    int32 tempOutL32;
    int64 tempOutL64;
    int16 *pOutputL16 = pDrcData->pDelayBufferLeftL16;
    int16 *pOutputR16 = pDrcData->pDelayBufferRightL16;

    if (MAKEUPGAIN_UNITY != pDrcCfg->makeupGainUL16Q12) {
        for (i=0; i<nSampleCnt; i++) {
            processData = pDrcData->pDelayBufferLeftL16[i];
            tempOutL32 = s32_mult_s32_s16_shift15_sat(pDrcData->paGainL32Q15[0][i], processData);

            tempOutL64 = s64_mult_s32_u16_shift(tempOutL32, pDrcCfg->makeupGainUL16Q12, 4);
            tempOutL32 = s32_saturate_s64(tempOutL64);

            *pOutputL16++ = s16_saturate_s32(tempOutL32);

            processData = pDrcData->pDelayBufferRightL16[i];
            tempOutL32  = s32_mult_s32_s16_shift15_sat(pDrcData->paGainL32Q15[0][i], processData);

            tempOutL64 = s64_mult_s32_u16_shift(tempOutL32, pDrcCfg->makeupGainUL16Q12, 4);
            tempOutL32 = s32_saturate_s64(tempOutL64);

            *pOutputR16++ = s16_saturate_s32(tempOutL32);
        }
    }
    else {
        for (i=0; i<nSampleCnt; i++) {
            processData = pDrcData->pDelayBufferLeftL16[i];
            tempOutL32 = s32_mult_s32_s16_shift15_sat(pDrcData->paGainL32Q15[0][i], processData);

            *pOutputL16++ = s16_saturate_s32(tempOutL32);

            processData = pDrcData->pDelayBufferRightL16[i];
            tempOutL32  = s32_mult_s32_s16_shift15_sat(pDrcData->paGainL32Q15[0][i], processData);

            *pOutputR16++ = s16_saturate_s32(tempOutL32);
        }
    }
}

#endif


#if ((defined __hexagon__) || (defined __qdsp6__))

extern "C" {
void ApplyGainStereoUnLinked(DrcConfigInternal *pDrcCfg, DrcData *pDrcData, uint32 nSampleCnt);
}

#else

static void ApplyGainStereoUnLinked(DrcConfigInternal *pDrcCfg, DrcData *pDrcData, uint32 nSampleCnt)
{
    uint32 i;
    int16 processData;
    int32 tempOutL32;
    int64 tempOutL64;
    int16 *pOutputL16 = pDrcData->pDelayBufferLeftL16;
    int16 *pOutputR16 = pDrcData->pDelayBufferRightL16;

    if (MAKEUPGAIN_UNITY != pDrcCfg->makeupGainUL16Q12) {
        for (i=0; i<nSampleCnt; i++) {
            processData = pDrcData->pDelayBufferLeftL16[i];
            tempOutL32  = s32_mult_s32_s16_shift15_sat(pDrcData->paGainL32Q15[0][i], processData);

            tempOutL64 = s64_mult_s32_u16_shift(tempOutL32, pDrcCfg->makeupGainUL16Q12, 4);
            tempOutL32 = s32_saturate_s64(tempOutL64);

            *pOutputL16++ = s16_saturate_s32(tempOutL32);

            processData = pDrcData->pDelayBufferRightL16[i];
            tempOutL32  = s32_mult_s32_s16_shift15_sat(pDrcData->paGainL32Q15[1][i], processData);

            tempOutL64 = s64_mult_s32_u16_shift(tempOutL32, pDrcCfg->makeupGainUL16Q12, 4);
            tempOutL32 = s32_saturate_s64(tempOutL64);

            *pOutputR16++ = s16_saturate_s32(tempOutL32);
        }
    }
    else {
        for (i=0; i<nSampleCnt; i++) {
            processData = pDrcData->pDelayBufferLeftL16[i];
            tempOutL32  = s32_mult_s32_s16_shift15_sat(pDrcData->paGainL32Q15[0][i], processData);

            *pOutputL16++ = s16_saturate_s32(tempOutL32);

            processData = pDrcData->pDelayBufferRightL16[i];
            tempOutL32  = s32_mult_s32_s16_shift15_sat(pDrcData->paGainL32Q15[1][i], processData);

            *pOutputR16++ = s16_saturate_s32(tempOutL32);
        }
    }
}

#endif


static void ProcessNoDrc(DrcConfigInternal *pDrcCfg, DrcData *pDrcData,
                         int16 *pOutPtrL16, int16 *pOutPtrR16,
                         int16 *pInPtrL16, int16 *pInPtrR16,
                         uint32 nSampleCnt)
{
    // copy data from input to output
    if (pInPtrL16 != pOutPtrL16) {
        memscpy(pOutPtrL16, sizeof(int16)*nSampleCnt, pInPtrL16, sizeof(int16)*nSampleCnt);
    }

    if (2 == pDrcCfg->numChannel) {
        if (pInPtrR16 != pOutPtrR16) {
            memscpy(pOutPtrR16, sizeof(int16)*nSampleCnt, pInPtrR16, sizeof(int16)*nSampleCnt);
        }
    }
}


static void ProcessDrcOneChan(DrcConfigInternal *pDrcCfg, DrcData *pDrcData,
                              int16 *pOutPtrL16, int16 *pOutPtrR16,
                              int16 *pInPtrL16, int16 *pInPtrR16,
                              uint32 nSampleCnt)
{
    uint32 nSubFrameSize;

    while (nSampleCnt > 0) {
        nSubFrameSize = s16_min_s16_s16(nSampleCnt, DRC_BLOCKSIZE);
        nSampleCnt -= nSubFrameSize;

        memscpy(pDrcData->pDelayBufferLeftL16+pDrcCfg->delay, sizeof(int16)*nSubFrameSize, pInPtrL16, sizeof(int16)*nSubFrameSize);

        ComputeInputRmsMono(pDrcCfg, pDrcData, pInPtrL16, nSubFrameSize);

        CalculateDrcGainOneGain(pDrcCfg, pDrcData, nSubFrameSize);

        ApplyGainMono(pDrcCfg, pDrcData, nSubFrameSize);

        memscpy(pOutPtrL16, sizeof(int16)*nSubFrameSize, pDrcData->pDelayBufferLeftL16, sizeof(int16)*nSubFrameSize);

        memsmove(pDrcData->pDelayBufferLeftL16, sizeof(int16)*pDrcCfg->delay, pDrcData->pDelayBufferLeftL16+nSubFrameSize, sizeof(int16)*pDrcCfg->delay);

        pInPtrL16 += nSubFrameSize;
        pOutPtrL16 += nSubFrameSize;
    }
}


static void ProcessDrcTwoChanNotLinked(DrcConfigInternal *pDrcCfg, DrcData *pDrcData,
                                       int16 *pOutPtrL16, int16 *pOutPtrR16,
                                       int16 *pInPtrL16, int16 *pInPtrR16,
                                       uint32 nSampleCnt)
{
    uint32 nSubFrameSize;

    while (nSampleCnt > 0) {
        nSubFrameSize = s16_min_s16_s16(nSampleCnt, DRC_BLOCKSIZE);
        nSampleCnt -= nSubFrameSize;

        memscpy(pDrcData->pDelayBufferLeftL16+pDrcCfg->delay, sizeof(int16)*nSubFrameSize, pInPtrL16, sizeof(int16)*nSubFrameSize);
        memscpy(pDrcData->pDelayBufferRightL16+pDrcCfg->delay, sizeof(int16)*nSubFrameSize, pInPtrR16, sizeof(int16)*nSubFrameSize);

        ComputeInputRmsStereoUnLinked(pDrcCfg, pDrcData, pInPtrL16, pInPtrR16, nSubFrameSize);

        CalculateDrcGainTwoGain(pDrcCfg, pDrcData, nSubFrameSize);

        ApplyGainStereoUnLinked(pDrcCfg, pDrcData, nSubFrameSize);

        memscpy(pOutPtrL16, sizeof(int16)*nSubFrameSize, pDrcData->pDelayBufferLeftL16, sizeof(int16)*nSubFrameSize);
        memscpy(pOutPtrR16, sizeof(int16)*nSubFrameSize, pDrcData->pDelayBufferRightL16, sizeof(int16)*nSubFrameSize);

        memsmove(pDrcData->pDelayBufferLeftL16, sizeof(int16)*pDrcCfg->delay, pDrcData->pDelayBufferLeftL16+nSubFrameSize, sizeof(int16)*pDrcCfg->delay);
        memsmove(pDrcData->pDelayBufferRightL16, sizeof(int16)*pDrcCfg->delay, pDrcData->pDelayBufferRightL16+nSubFrameSize, sizeof(int16)*pDrcCfg->delay);

        pInPtrL16 += nSubFrameSize;
        pInPtrR16 += nSubFrameSize;

        pOutPtrL16 += nSubFrameSize;
        pOutPtrR16 += nSubFrameSize;
    }
}


static void ProcessDrcTwoChanLinked(DrcConfigInternal *pDrcCfg, DrcData *pDrcData,
                                    int16 *pOutPtrL16, int16 *pOutPtrR16,
                                    int16 *pInPtrL16, int16 *pInPtrR16,
                                    uint32 nSampleCnt)
{
    uint32 nSubFrameSize;

    while (nSampleCnt > 0) {
        nSubFrameSize = s16_min_s16_s16(nSampleCnt, DRC_BLOCKSIZE);
        nSampleCnt -= nSubFrameSize;

        memscpy(pDrcData->pDelayBufferLeftL16+pDrcCfg->delay, sizeof(int16)*nSubFrameSize, pInPtrL16, sizeof(int16)*nSubFrameSize);
        memscpy(pDrcData->pDelayBufferRightL16+pDrcCfg->delay, sizeof(int16)*nSubFrameSize, pInPtrR16, sizeof(int16)*nSubFrameSize);

        ComputeInputRmsStereoLinked(pDrcCfg, pDrcData, pInPtrL16, pInPtrR16, nSubFrameSize);

        CalculateDrcGainOneGain(pDrcCfg, pDrcData, nSubFrameSize);

        ApplyGainStereoLinked(pDrcCfg, pDrcData, nSubFrameSize);

        memscpy(pOutPtrL16, sizeof(int16)*nSubFrameSize, pDrcData->pDelayBufferLeftL16, sizeof(int16)*nSubFrameSize);
        memscpy(pOutPtrR16, sizeof(int16)*nSubFrameSize, pDrcData->pDelayBufferRightL16, sizeof(int16)*nSubFrameSize);

        memsmove(pDrcData->pDelayBufferLeftL16, sizeof(int16)*pDrcCfg->delay, pDrcData->pDelayBufferLeftL16+nSubFrameSize, sizeof(int16)*pDrcCfg->delay);
        memsmove(pDrcData->pDelayBufferRightL16, sizeof(int16)*pDrcCfg->delay, pDrcData->pDelayBufferRightL16+nSubFrameSize, sizeof(int16)*pDrcCfg->delay);

        pInPtrL16 += nSubFrameSize;
        pInPtrR16 += nSubFrameSize;

        pOutPtrL16 += nSubFrameSize;
        pOutPtrR16 += nSubFrameSize;
    }
}


/*----------------------------------------------------------------------------
 * Function Definitions
 * -------------------------------------------------------------------------*/
CDrcLib::CDrcLib()
{
    // Set default values for the tuning parameters
    // Keep the initial integer parameters unmodified
    m_drcCfgInt.numChannel     = NUM_CHANNEL_DEFAULT;
    m_drcCfgInt.stereoLinked       = (DrcStereoLinkedType)STEREO_LINKED_DEFAULT;

    m_drcCfgInt.mode               = (DrcFeaturesType)MODE_DEFAULT;
    m_drcCfgInt.downSampleLevel    = DOWNSAMPLE_LEVEL_DEFAULT;
    m_drcCfgInt.delay              = DELAY_DEFAULT;
    m_drcCfgInt.rmsTavUL16Q16      = RMS_TAV_DEFAULT;
    m_drcCfgInt.makeupGainUL16Q12  = MAKEUP_GAIN_DEFAULT;


    m_drcCfgInt.dnExpaThresholdL16Q7   = DN_EXPA_THRESHOLD_DEFAULT;
    m_drcCfgInt.dnExpaSlopeL16Q8       = DN_EXPA_SLOPE_DEFAULT;
    m_drcCfgInt.dnExpaAttackUL32Q31    = DN_EXPA_ATTACK_DEFAULT;
    m_drcCfgInt.dnExpaReleaseUL32Q31   = DN_EXPA_RELEASE_DEFAULT;
    m_drcCfgInt.dnExpaHysterisisUL16Q14    = DN_EXPA_HYSTERISIS_DEFAULT;
    m_drcCfgInt.dnExpaMinGainDBL32Q23  = DN_EXPA_MIN_GAIN_DEFAULT;

    m_drcCfgInt.upCompThresholdL16Q7   = UP_COMP_THRESHOLD_DEFAULT;
    m_drcCfgInt.upCompSlopeUL16Q16 = UP_COMP_SLOPE_DEFAULT;
    m_drcCfgInt.upCompAttackUL32Q31    = UP_COMP_ATTACK_DEFAULT;
    m_drcCfgInt.upCompReleaseUL32Q31   = UP_COMP_RELEASE_DEFAULT;
    m_drcCfgInt.upCompHysterisisUL16Q14    = UP_COMP_HYSTERISIS_DEFAULT;


    m_drcCfgInt.dnCompThresholdL16Q7   = DN_COMP_THRESHOLD_DEFAULT;
    m_drcCfgInt.dnCompSlopeUL16Q16 = DN_COMP_SLOPE_DEFAULT;
    m_drcCfgInt.dnCompAttackUL32Q31    = DN_COMP_ATTACK_DEFAULT;
    m_drcCfgInt.dnCompReleaseUL32Q31   = DN_COMP_RELEASE_DEFAULT;
    m_drcCfgInt.dnCompHysterisisUL16Q14    = DN_COMP_HYSTERISIS_DEFAULT;

}


/*======================================================================

  FUNCTION      Initialize

  DESCRIPTION   Performs initialization of data structures for the
                drc algorithm. Two pointers to two memory is passed
                for configuring the drc static configuration
                structure.

                Called once at audio connection set up time.

  PARAMETERS    paramMiscValues: [in] Pointer to 16-bit tuning parameter list
                paramStaticCurveValues: [in] Pointer to 32-bit tuning parameter list
                m_drcCfg:  [in,out] Pointer to configuration structure
                m_drcCfg: [out] Pointer to data structure

  SIDE EFFECTS  None

======================================================================*/
PPStatus CDrcLib::Initialize (DrcConfig &cfg)
{
   PPStatus errorCode = PPFAILURE;

   /* If delay configured is out of range, return error code */
   if (cfg.delay > MAX_DELAY_SAMPLE)
   {
      return (errorCode = PPERR_DELAY_INVALID);
   }
   else if (cfg.delay < 0)
   {
      return (errorCode = PPERR_DELAY_NEGATIVE);
   }
   else
   {
      // valid delay value
      m_drcCfgInt.delay   = cfg.delay;
   }

   errorCode = ReInitialize(cfg);
   if (PPSUCCESS != errorCode)
   {
      return errorCode;
   }

   Reset();

   /////////////////////////////////////////////////////////
   // End module initialization
   /////////////////////////////////////////////////////////
   return PPSUCCESS;
}


PPStatus CDrcLib::ReInitialize (DrcConfig &cfg)
{
    int16 tempThresholdL16Q7;

    /*-------------- Initialize members of Misc part of the data structure --------------*/
    m_drcCfgInt.numChannel     = cfg.numChannel;
#ifdef QDSP6_DRCLIB_ASM
    m_drcCfgInt.stereoLinked = cfg.stereoLinked;
#else
    m_drcCfgInt.stereoLinked = (DrcStereoLinkedType)cfg.stereoLinked;
#endif

    m_drcCfgInt.mode = (DrcFeaturesType) cfg.mode;

    m_drcCfgInt.downSampleLevel   = cfg.downSampleLevel;
    m_drcCfgInt.rmsTavUL16Q16     = cfg.rmsTavUL16Q16;
    m_drcCfgInt.makeupGainUL16Q12 = cfg.makeupGainUL16Q12;

    // for unsigned Q16, (1-TAV) equals to -TAV
    m_drcCfgInt.negativeDrcTavUL16Q16 = s16_neg_s16_sat(m_drcCfgInt.rmsTavUL16Q16);

    /*-------------- Initialize tuning parameters for Static Curve -----------------*/
    m_drcCfgInt.dnExpaThresholdL16Q7     = cfg.dnExpaThresholdL16Q7;
    m_drcCfgInt.dnExpaSlopeL16Q8         = cfg.dnExpaSlopeL16Q8;
    m_drcCfgInt.dnExpaAttackUL32Q31      = cfg.dnExpaAttackUL32Q31;
    m_drcCfgInt.dnExpaReleaseUL32Q31     = cfg.dnExpaReleaseUL32Q31;
    m_drcCfgInt.dnExpaHysterisisUL16Q14  = cfg.dnExpaHysterisisUL16Q14;
    m_drcCfgInt.dnExpaMinGainDBL32Q23    = cfg.dnExpaMinGainDBL32Q23;

    m_drcCfgInt.dnExpaHysterisisUL16Q14Asl1 = m_drcCfgInt.dnExpaHysterisisUL16Q14<<1;

    m_drcCfgInt.upCompThresholdL16Q7   = cfg.upCompThresholdL16Q7;
    m_drcCfgInt.upCompSlopeUL16Q16     = cfg.upCompSlopeUL16Q16;
    m_drcCfgInt.upCompAttackUL32Q31    = cfg.upCompAttackUL32Q31;
    m_drcCfgInt.upCompReleaseUL32Q31   = cfg.upCompReleaseUL32Q31;
    m_drcCfgInt.upCompHysterisisUL16Q14 = cfg.upCompHysterisisUL16Q14;

    m_drcCfgInt.upCompHysterisisUL16Q14Asl1 = m_drcCfgInt.upCompHysterisisUL16Q14<<1;

    m_drcCfgInt.dnCompThresholdL16Q7   = cfg.dnCompThresholdL16Q7;
    m_drcCfgInt.dnCompSlopeUL16Q16 = cfg.dnCompSlopeUL16Q16;
    m_drcCfgInt.dnCompAttackUL32Q31    = cfg.dnCompAttackUL32Q31;
    m_drcCfgInt.dnCompReleaseUL32Q31   = cfg.dnCompReleaseUL32Q31;
    m_drcCfgInt.dnCompHysterisisUL16Q14 = cfg.dnCompHysterisisUL16Q14;

    m_drcCfgInt.dnCompHysterisisUL16Q14Asl1 = m_drcCfgInt.dnCompHysterisisUL16Q14<<1;

    m_drcCfgInt.outDnCompThresholdL16Q7 = m_drcCfgInt.dnCompThresholdL16Q7;
    tempThresholdL16Q7 = s16_extract_s32_h(s32_mult_s16_u16(s16_sub_s16_s16(m_drcCfgInt.upCompThresholdL16Q7,
                                    m_drcCfgInt.dnExpaThresholdL16Q7),
                                    m_drcCfgInt.upCompSlopeUL16Q16)); /* Q23 */
    tempThresholdL16Q7 = s16_add_s16_s16_sat(tempThresholdL16Q7, m_drcCfgInt.dnExpaThresholdL16Q7);
    m_drcCfgInt.outDnExpaThresholdL16Q7 = tempThresholdL16Q7;
    m_drcCfgInt.outUpCompThresholdL16Q7 = m_drcCfgInt.upCompThresholdL16Q7;

    // (uwCompThresholdL16Q7 - dwExpaThresholdL16Q7)*uwCompSlopeUL16Q16
    m_drcCfgInt.dnExpaNewTargetGainL32Q23 = s32_mult_s16_u16(s16_sub_s16_s16(m_drcCfgInt.upCompThresholdL16Q7,
        m_drcCfgInt.dnExpaThresholdL16Q7), m_drcCfgInt.upCompSlopeUL16Q16);

    // dwCompThreshold * dwCompSlopeUL16Q16
    m_drcCfgInt.dnCompThrMultSlopeL32Q23 = s32_mult_s16_u16(m_drcCfgInt.dnCompThresholdL16Q7,
        m_drcCfgInt.dnCompSlopeUL16Q16);

    // uwCompThreshold * uwCompSlopeUL16Q16
    m_drcCfgInt.upCompThrMultSlopeL32Q23 = s32_mult_s16_u16(m_drcCfgInt.upCompThresholdL16Q7,
        m_drcCfgInt.upCompSlopeUL16Q16);

    if (DRC_ENABLED == m_drcCfgInt.mode) {
        if (1 == m_drcCfgInt.numChannel) {
            fnpProcess = ProcessDrcOneChan;
        }
        else if (2 == m_drcCfgInt.numChannel) {
            if (CHANNEL_NOT_LINKED == m_drcCfgInt.stereoLinked) {
                fnpProcess = ProcessDrcTwoChanNotLinked;
            }
            else if (CHANNEL_LINKED == m_drcCfgInt.stereoLinked) {
                fnpProcess = ProcessDrcTwoChanLinked;
            }
            else{
                fnpProcess = NULL;
                return PPFAILURE;
            }
        }
        else {
            fnpProcess = NULL;
            return PPFAILURE;
        }
    }
    else if (DRC_DISABLED == m_drcCfgInt.mode) {
        if (1==m_drcCfgInt.numChannel || 2==m_drcCfgInt.numChannel) {
            fnpProcess = ProcessNoDrc;
        }
        else {
            fnpProcess = NULL;
            return PPFAILURE;
        }
    }
    else {
        fnpProcess = NULL;
        return PPFAILURE;
    }

   /////////////////////////////////////////////////////////
   // End module initialization
   /////////////////////////////////////////////////////////
   return PPSUCCESS;
}

void CDrcLib::Reset ()
{
    /*-------------- Reset members varialbes --------------*/
    m_drcData.downSampleCounter     = 0;                        // Reset the down sample counter
    m_drcData.rmsStateL32[0]        = 0;                    // Reset the RMS estimate value for Left channel
    m_drcData.rmsStateL32[1]        = 0;                    // Reset the RMS estimate value for Right channel
    m_drcData.gainL32Q15[0]     = 32768;
    m_drcData.gainL32Q15[1]     = 32768;
    m_drcData.currState[0]          = NO_CHANGE;
    m_drcData.currState[1]          = NO_CHANGE;
    m_drcData.timeConstantUL32Q31[0] = 0;
    m_drcData.timeConstantUL32Q31[1] = 0;

    m_drcData.paGainL32Q15[0] = m_aGainL32Q15[0];
    m_drcData.paGainL32Q15[1] = m_aGainL32Q15[1];

    m_drcData.pRmsStateL32 = m_aRmsStateL32;

    m_drcData.pDelayBufferLeftL16 = m_delayBufferLeftL16;
    m_drcData.pDelayBufferRightL16 = m_delayBufferRightL16;
    m_drcData.dwcomp_state_change[0] = 1;
    m_drcData.dwcomp_state_change[1] = 1;
    m_drcData.uwcomp_state_change[0] = 1;
    m_drcData.uwcomp_state_change[1] = 1;

    memset(m_drcData.pDelayBufferLeftL16, 0, sizeof(int16)*(m_drcCfgInt.delay+DRC_BLOCKSIZE));//sizeof(m_delayBufferLeftL16));
    memset(m_drcData.pDelayBufferRightL16,0, sizeof(int16)*(m_drcCfgInt.delay+DRC_BLOCKSIZE));//sizeof(m_delayBufferRightL16));

    memset(m_drcData.pRmsStateL32, 0, sizeof(int32)*2*DRC_BLOCKSIZE);// Reset the RMS estimate value

    memset(m_drcData.paGainL32Q15[0], 0, sizeof(int32)*DRC_BLOCKSIZE);
    memset(m_drcData.paGainL32Q15[1], 0, sizeof(int32)*DRC_BLOCKSIZE);
}


/*======================================================================

  FUNCTION      Process

  DESCRIPTION   Process multi-channel input audio sample by sample and drcit the
                input to specified threshold level. The input can be in any sampling
                rate - 8, 16, 22.05, 32, 44.1, 48 KHz. The input is 16-bit Q15 and
                the output is also in the form of 16-bit Q15.

  DEPENDENCIES  Input pointers must not be NULL.
                drc_init must be called prior to any call to drc_process.

  PARAMETERS    pInPtrL16: [in] Pointer to 16-bit Q15 Left channel signal
                pInPtrR16: [in] Pointer to 16-bit Q15 Right channel signal
                pOutPtrL16: [out] Pointer to 16-bit Q15 Left channel output audio
                pOutPtrR16: [out] Pointer to 16-bit Q15 Right channel output audio

  RETURN VALUE  gainNum: [out] How many gain values need to be computed for each input sample.

  SIDE EFFECTS  None.

======================================================================*/

void CDrcLib::Process ( int16 *pOutPtrL16,
                        int16 *pOutPtrR16,
                        int16 *pInPtrL16,
                        int16 *pInPtrR16,
                        uint32 nSampleCnt)
{
    (*fnpProcess)(&m_drcCfgInt, &m_drcData, pOutPtrL16, pOutPtrR16,
        pInPtrL16, pInPtrR16, nSampleCnt);
}
