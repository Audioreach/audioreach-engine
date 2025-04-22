/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
/*===========================================================================
 * FILE NAME: SoftVolumeControls.cpp
 * DESCRIPTION:
 *    Volume and Soft stepping
 *===========================================================================*/

#ifdef AVS_BUILD_SOS
#include "capi_cmn.h"
#endif

#include "SoftVolumeControls.h"
#include "audio_basic_op.h"
#include "audio_divide_qx.h"
#include "audio_log10.h"
#include "audio_basic_op_ext.h"
#include <math.h>
#include "audio_exp10.h"
#include "apply_gain.h"
#include "audio_clips.h"
#include "string.h"
#include "posal.h"

#if ((defined __hexagon__) || (defined __qdsp6__))
#include <hexagon_protos.h>
#include <hexagon_types.h>
#include <q6sim_timer.h>
// #include "OmmUtils.h"
#endif


static const uint32 DEFAULT_THRESHOLD      = 0;
static const int32  K_expL32Q26            = (10 << 26);
static const int32  M_expL32Q31            = 2099202; // M = 1/(2^K -1)
static const int32  K_logL32Q16            = (200 << 16);
static const int32  M_logL32Q31            = 280678230; // M = 1/log2(1+K)

static const uint32 UNITY_L32_Q28 = 1 << 28;

CSoftVolumeControlsLib::CSoftVolumeControlsLib()
   : m_sampleRate(48000), m_bytesPerSample(2), m_isPaused(false), m_thresholdQ15(0)
{
   m_softVolumeParams.periodMs     = 0;
   m_softVolumeParams.rampingCurve = RAMP_LINEAR;
   m_softVolumeParams.stepUs       = 0;

   m_softMuteParams.periodMs     = 0;
   m_softMuteParams.rampingCurve = RAMP_LINEAR;
   m_softMuteParams.stepUs       = 0;

   m_softPauseParams.periodMs     = 0;
   m_softPauseParams.rampingCurve = RAMP_LINEAR;
   m_softPauseParams.stepUs       = 0;

   m_softResumeParams.periodMs     = 0;
   m_softResumeParams.rampingCurve = RAMP_LINEAR;
   m_softResumeParams.stepUs       = 0;

   SetThreshold(DEFAULT_THRESHOLD);
   m_muteBeforeResumeParams.m_muteBeforeRampSamples = 0;
   m_muteBeforeResumeParams.m_muteSamplesPending    = 0;
   m_muteBeforeResumeParams.m_resumeWithDelayInMs   = 0;
}

CSoftVolumeControlsLib::~CSoftVolumeControlsLib()
{
}

void CSoftVolumeControlsLib::SetSoftPauseParams(const SoftSteppingParams &softPauseParams)
{
   m_softPauseParams = softPauseParams;

   // Ensure that the step is not greater than the period
   uint32 periodUs = m_softPauseParams.periodMs * 1000;
   if (m_softPauseParams.stepUs > periodUs)
   {
      m_softPauseParams.stepUs = periodUs;
   }
}

void CSoftVolumeControlsLib::SetSoftResumeParams(const SoftSteppingParams &softResumeParams)
{
   m_softResumeParams = softResumeParams;

   // Ensure that the step is not greater than the period
   uint32 periodUs = m_softResumeParams.periodMs * 1000;
   if (m_softResumeParams.stepUs > periodUs)
   {
      m_softResumeParams.stepUs = periodUs;
   }
}

void CSoftVolumeControlsLib::SetSoftVolumeParams(const SoftSteppingParams &softVolumeParams)
{
   m_softVolumeParams = softVolumeParams;

   // Ensure that the step is not greater than the period
   uint32 periodUs = m_softVolumeParams.periodMs * 1000;
   if (m_softVolumeParams.stepUs > periodUs)
   {
      m_softVolumeParams.stepUs = periodUs;
   }
}

void CSoftVolumeControlsLib::SetSoftMuteParams(const SoftSteppingParams &softMuteParams)
{
   m_softMuteParams = softMuteParams;

   // Ensure that the step is not greater than the period
   uint32 periodUs = m_softMuteParams.periodMs * 1000;
   if (m_softMuteParams.stepUs > periodUs)
   {
      m_softMuteParams.stepUs = periodUs;
   }
}

void CSoftVolumeControlsLib::SetResumeWithDelayParam(const uint32 &delayResumeParam)
{
   m_muteBeforeResumeParams.m_resumeWithDelayInMs = delayResumeParam;
}

boolean CSoftVolumeControlsLib::IsMuted(const void *pChannelStruct) const
{
   const perChannelData *pChannelData = reinterpret_cast<const perChannelData *>(pChannelStruct);

   return pChannelData->isMuted;
}

void CSoftVolumeControlsLib::Unmute(void *pChannelStruct)
{
   perChannelData *pChannelData = reinterpret_cast<perChannelData *>(pChannelStruct);

   if (pChannelData->isMuted)
   {
      pChannelData->isMuted = false;

      if (!m_isPaused)
      {
         SetupPanner(&pChannelData->panner, pChannelData->chanGainQ28, m_softMuteParams);
      }
   }
}

void CSoftVolumeControlsLib::Mute(void *pChannelStruct)
{
   perChannelData *pChannelData = reinterpret_cast<perChannelData *>(pChannelStruct);

   if (!pChannelData->isMuted)
   {
      pChannelData->isMuted = true;

      SetupPanner(&pChannelData->panner, 0, m_softMuteParams);
   }
}

void CSoftVolumeControlsLib::SetThreshold(uint32 pThreshold_Q15)
{
   // todo: should there be a limit check
   if (pThreshold_Q15 == m_thresholdQ15)
   {
      return;
   }

   m_thresholdQ15 = pThreshold_Q15;       // directly getting it in Q15
   m_thresholdQ27 = pThreshold_Q15 << 12; // 27-15
   m_thresholdQ31 = pThreshold_Q15 << 16; // 31-15
}

uint32 CSoftVolumeControlsLib::GetThreshold() const
{
   return m_thresholdQ15;
}

uint32 CSoftVolumeControlsLib::GetPauseRampPeriod() const
{
   return m_softPauseParams.periodMs;
}

/*=============================================================================
FUNCTION      void soft_panner_log_setup

DESCRIPTION  Setup Panner for log ramp
OUTPUTS

DEPENDENCIES  None

RETURN VALUE  None

SIDE EFFECTS
===============================================================================*/

void CSoftVolumeControlsLib::SoftPannerLogSetup(SvpannerStruct *panner,        /* panner struct */
                                                uint32          newGainL32Q28, /* new target panner gain            */
                                                uint32          rampSamples,   /* number of samples in the ramp     */
                                                uint32          step)
{
   /* substitute panner struct values with shorter names */
   uint32 currentGainL32Q28 = panner->currentGainL32Q28;
   int32  AL32Q26           = 0;
   int32  BL32Q26           = 0;
   int32  CL32Q16           = 0;
   int32  deltaCL32Q16      = 0;
   uint32 sampleCounter     = 0;
   panner->newGainL32Q28    = newGainL32Q28;

   if (step <= 0)
   {
      step = 1;
   }

   /*----------------------------- if no ramp ------------------------------*/
   if (rampSamples <= 0)
   {
      currentGainL32Q28 = newGainL32Q28;
   } /* end of if (rampSamples <= 0) */
   /*--------------- if there is ramp, then need some work here ------------*/
   else /* (rampSamples > 0) */
   {
      if (newGainL32Q28 != currentGainL32Q28)
      {
         /*------ determine change -------*/
         int32 newGainL32Q27     = (int32)(newGainL32Q28 >> (28 - 27));
         int32 currentGainL32Q27 = (int32)(currentGainL32Q28 >> (28 - 27));

         sampleCounter           = rampSamples;
         int32 changeL32Q27      = newGainL32Q27 - currentGainL32Q27;
         int32 currentGainL32Q26 = currentGainL32Q27 >> (27 - 26);

         AL32Q26      = s32_mult_s32_s32_rnd_sat(changeL32Q27, M_logL32Q31);
         BL32Q26      = currentGainL32Q26;
         CL32Q16      = 1 << 16;
         deltaCL32Q16 = (K_logL32Q16 / sampleCounter * step);
      }
   } /* end of else (rampSamples > 0) */

   /*------------- store new values back into panner struct ----------------*/

   panner->step                    = step;
   panner->stepResidue             = step;
   panner->coeffs.log.BL32Q26      = BL32Q26;
   panner->coeffs.log.AL32Q26      = AL32Q26;
   panner->coeffs.log.CL32Q16      = CL32Q16;
   panner->coeffs.log.deltaCL32Q16 = deltaCL32Q16;
   panner->targetgainL32Q28        = newGainL32Q28; // update target gain
   panner->sampleCounter           = sampleCounter;
   panner->currentGainL32Q28       = currentGainL32Q28;
   panner->index                   = 0;
} /*------------------- end of function soft_panner_log_setup------------------*/

/*=============================================================================
FUNCTION      void soft_panner_linear_setup

DESCRIPTION  Setup Panner for linear ramp
OUTPUTS

DEPENDENCIES  None

RETURN VALUE  None

SIDE EFFECTS
===============================================================================*/
void CSoftVolumeControlsLib::SoftPannerLinearSetup(SvpannerStruct *panner,        /* panner struct */
                                                   uint32          newGainL32Q28, /* new target panner gain */
                                                   uint32          rampSamples, /* number of samples in the ramp     */
                                                   uint32          step)
{

   /* substitute panner struct values with shorter names */
   uint32 sampleCounter     = panner->sampleCounter;
   uint32 currentGainL32Q28 = panner->currentGainL32Q28;
   int64  deltaL64Q59;

   /*----------------------------- if no ramp ------------------------------*/
   if (rampSamples <= 0)
   {
      sampleCounter     = 0;
      deltaL64Q59       = 0;
      currentGainL32Q28 = newGainL32Q28; /* rampSamples =0 then apply the
                                 new value of gain immediately*/

   } /* end of if (rampSamples <= 0) */

   /*--------------- if there is ramp, then need some work here ------------*/
   else /* (rampSamples > 0) */
   {

      /*------ determine change -------*/
      /* if no change, then reset counter and delta */
      if (newGainL32Q28 == currentGainL32Q28)
      {

         sampleCounter = 0;
         deltaL64Q59   = 0;
      }
      /* else, set counter and calculate delta */
      else
      {

         sampleCounter = rampSamples;
         if (0 == step)
         {
            step = 1;
         }

         uint64 stepL64Q31              = ((uint64)(step)) << 31;
         uint32 stepOverSamplesL32Q31   = (uint32)(stepL64Q31 / rampSamples);
         uint64 scaledCurrentGainL64Q59 = u64_mult_u32_u32(currentGainL32Q28, stepOverSamplesL32Q31);
         uint64 scaledNewGainL64Q59     = u64_mult_u32_u32(newGainL32Q28, stepOverSamplesL32Q31);

         deltaL64Q59 = scaledNewGainL64Q59 -
                       scaledCurrentGainL64Q59; // Since the max gain value is 16, there will be no wraparound.
      }
   } /* end of else (rampSamples > 0) */

   /*------------- store new values back into panner struct ----------------*/
   panner->step                            = step;
   panner->stepResidue                     = 0;
   panner->coeffs.linear.deltaL64Q59       = deltaL64Q59;
   panner->targetgainL32Q28                = newGainL32Q28; // update target gain
   panner->sampleCounter                   = sampleCounter;
   panner->currentGainL32Q28               = currentGainL32Q28;
   panner->coeffs.linear.currentGainL64Q59 = ((int64)currentGainL32Q28) << (59 - 28);
   panner->index                           = 0;

} /*------------------- end of function soft_panner_linear_setup --------------*/

/*=============================================================================
FUNCTION      void soft_panner_exp_setup

DESCRIPTION  Setup Panner for an exponential ramp
OUTPUTS

DEPENDENCIES  None

RETURN VALUE  None

SIDE EFFECTS
===============================================================================*/
void CSoftVolumeControlsLib::SoftPannerExpSetup(SvpannerStruct *panner,        /* panner struct */
                                                uint32          newGainL32Q28, /* new target panner gain  in Q28    */
                                                uint32          rampSamples,   /* number of samples in the ramp     */
                                                uint32          step)
{
   /* substitute panner struct values with shorter names */
   uint32 currentGainL32Q28 = panner->currentGainL32Q28;
   int32  AL32Q26           = 0;
   int32  BL32Q26           = 0;
   int32  CL32Q26           = 0;
   int32  deltaCL32Q26      = 0;
   uint32 sampleCounter     = 0;

   if (step <= 0)
   {
      step = 1;
   }

   /*----------------------------- if no ramp ------------------------------*/
   if (rampSamples <= 0)
   {
      currentGainL32Q28 = newGainL32Q28;
   } /* end of if (rampSamples <= 0) */
   /*--------------- if there is ramp, then need some work here ------------*/
   else /* (rampSamples > 0) */
   {
      if (newGainL32Q28 != currentGainL32Q28)
      {
         /*------ determine change -------*/
         int32 newGainL32Q27     = (int32)(newGainL32Q28 >> (28 - 27));
         int32 currentGainL32Q27 = (int32)(currentGainL32Q28 >> (28 - 27));

         sampleCounter           = rampSamples;
         int32 changeL32Q27      = newGainL32Q27 - currentGainL32Q27;
         int32 currentGainL32Q26 = currentGainL32Q27 >> (27 - 26);

         AL32Q26      = s32_mult_s32_s32_rnd_sat(changeL32Q27, M_expL32Q31);
         BL32Q26      = s32_sub_s32_s32_sat(currentGainL32Q26, AL32Q26);
         deltaCL32Q26 = (K_expL32Q26 / sampleCounter * step);
      }
   } /* end of else (rampSamples > 0) */

   /*------------- store new values back into panner struct ----------------*/

   panner->step                    = step;
   panner->stepResidue             = step;
   panner->coeffs.exp.BL32Q26      = BL32Q26;
   panner->coeffs.exp.AL32Q26      = AL32Q26;
   panner->coeffs.exp.CL32Q26      = CL32Q26;
   panner->coeffs.exp.deltaCL32Q26 = deltaCL32Q26;
   panner->targetgainL32Q28        = newGainL32Q28; // update target gain
   panner->sampleCounter           = sampleCounter;
   panner->currentGainL32Q28       = currentGainL32Q28;
   panner->index                   = 0;
} /*------------------- end of function soft_panner_exp_setup --------------*/

boolean CSoftVolumeControlsLib::ShouldRespond(PauseCommand command)
{
   bool should_respond = false;
   switch (command)
   {
      case COMMAND_PAUSE:
      {
         if (m_pauseState == STEADY || m_pauseState == WAITING || m_pauseState == RAMPING_UP ||
             MUTE_BEFORE_RAMPUP == m_pauseState)
         {
            should_respond = true;
         }
         break;
      }
      case COMMAND_FORCE_PAUSE:
      {
         if (m_pauseState != PAUSE)
            should_respond = true;
         break;
      }
      case COMMAND_RESUME:
      {
         if (m_pauseState == PAUSE || m_pauseState == RAMPING_DOWN)
         {
            should_respond = true;
         }
         break;
      }
      default:
      {
         break;
      }
   }

   return should_respond;
}

// Note that ProcessV2() can also cause state transitions.
void CSoftVolumeControlsLib::StateTransition(PauseCommand command)
{
   switch (command)
   {
      case COMMAND_INIT:
      {
         m_pauseState = STEADY;
         break;
      }
      case COMMAND_PAUSE:
      {
         // Unless the module is paused/ramping down, any other state transitions to ramping down
         // TODO: Check MUTE_BEFORE_RAMPUP state usage
         if (m_pauseState == STEADY || m_pauseState == RAMPING_UP || m_pauseState == WAITING ||
             m_pauseState == MUTE_BEFORE_RAMPUP)
         {
            m_pauseState = RAMPING_DOWN;
         }
         break;
      }
      case COMMAND_FORCE_PAUSE:
      {
         m_pauseState = PAUSE;
         break;
      }
      case COMMAND_RESUME:
      {
         if (m_pauseState == PAUSE || m_pauseState == RAMPING_DOWN)
         {
            m_pauseState = WAITING;
         }
         break;
      }
      default:
      {
         break;
      }
   }
}

void CSoftVolumeControlsLib::SetVolume(const uint32_t gainQ28, void *pChannelStruct)
{
   perChannelData *pChannelData = reinterpret_cast<perChannelData *>(pChannelStruct);

   pChannelData->chanGainQ28 = gainQ28;

   if (!m_isPaused && !pChannelData->isMuted)
   {
      // Ramp to target gain
      SetupPanner(&pChannelData->panner, pChannelData->chanGainQ28, m_softVolumeParams);
   }
}

void CSoftVolumeControlsLib::ForcePause(PerChannelDataBlock *pChannels, const uint32 nChannelCnt)
{
   if (!ShouldRespond(COMMAND_FORCE_PAUSE))
   {
      return;
   }

   // Set gain to 0
   for (int channel_idx = 0; channel_idx < nChannelCnt; channel_idx++)
   {
      PerChannelDataBlock *channelData                    = &pChannels[channel_idx];
      channelData->channelStruct.panner.currentGainL32Q28 = 0;
   }

   StateTransition(COMMAND_FORCE_PAUSE);
}

void CSoftVolumeControlsLib::StartSoftPause(void *pChannelStruct)
{
   perChannelData *pChannelData = reinterpret_cast<perChannelData *>(pChannelStruct);

   m_isPaused = true;

   if (!pChannelData->isMuted)
   {
      // Ramp to zero
      SetupPanner(&pChannelData->panner, 0, m_softPauseParams);
   }
}

void CSoftVolumeControlsLib::StartSoftPauseAllChannels(PerChannelDataBlock *pChannels, uint32 nChannelCnt)
{
   if (!ShouldRespond(COMMAND_PAUSE))
   {
      return;
   }
   if (MUTE_BEFORE_RAMPUP == m_pauseState)
   {
      m_muteBeforeResumeParams.m_muteBeforeRampSamples = 0;
      m_muteBeforeResumeParams.m_muteSamplesPending    = m_muteBeforeResumeParams.m_muteBeforeRampSamples;
   }
   for (int channel_idx = 0; channel_idx < nChannelCnt; channel_idx++)
   {
      PerChannelDataBlock *channelData = &pChannels[channel_idx];
      StartSoftPause((void *)&(channelData->channelStruct));
   }

   StateTransition(COMMAND_PAUSE); // steady goes to ramping down
}

void CSoftVolumeControlsLib::StartSoftResume(void *pChannelStruct)
{
   perChannelData *pChannelData = reinterpret_cast<perChannelData *>(pChannelStruct);

   m_isPaused = false;
   // Ramp to target gain
   if (!pChannelData->isMuted)
   {
      SetupPanner(&pChannelData->panner, pChannelData->chanGainQ28, m_softResumeParams);
   }
}

void CSoftVolumeControlsLib::StartSoftResumeAllChannels(PerChannelDataBlock *pChannels, const uint32 nChannelCnt)
{
   if (!ShouldRespond(COMMAND_RESUME))
   {
      return;
   }
   if (PAUSE == m_pauseState)
   {
      m_muteBeforeResumeParams.m_muteBeforeRampSamples =
         (uint32)((uint64)m_muteBeforeResumeParams.m_resumeWithDelayInMs * (uint64)m_sampleRate / 1000);
      m_muteBeforeResumeParams.m_muteSamplesPending = m_muteBeforeResumeParams.m_muteBeforeRampSamples;
   }
   for (uint32_t channel_idx = 0; channel_idx < nChannelCnt; channel_idx++)
   {
      PerChannelDataBlock *channelData = &pChannels[channel_idx];
      StartSoftResume((void *)&(channelData->channelStruct));
   }
   StateTransition(COMMAND_RESUME);
}

void CSoftVolumeControlsLib::SetSampleRate(uint32_t oldSampleRate, uint32_t newSampleRate, void *pChannelStruct)
{
   m_sampleRate = newSampleRate;

   if (oldSampleRate != m_sampleRate)
   {
      // Need to recalculate the panner params for the new sampling rate
      perChannelData *pChannelData = reinterpret_cast<perChannelData *>(pChannelStruct);
      SvpannerStruct *pPanner      = &pChannelData->panner;

      SoftSteppingParams params;

      params.rampingCurve = pPanner->rampingCurve;
      // To avoid overflow, use 64 bit calculations here.
      params.stepUs   = (uint64)pPanner->step * 1000000 / oldSampleRate;
      params.periodMs = (uint64)pPanner->sampleCounter * 1000 / oldSampleRate;

      SetupPanner(pPanner, pPanner->targetgainL32Q28, params);
   }
}

void CSoftVolumeControlsLib::GetSoftPauseParams(SoftSteppingParams *pSoftPauseParams) const
{
   *pSoftPauseParams = m_softPauseParams;
}

void CSoftVolumeControlsLib::GetSoftResumeParams(SoftSteppingParams *pSoftResumeParams) const
{
   *pSoftResumeParams = m_softResumeParams;
}

void CSoftVolumeControlsLib::GetSoftVolumeParams(SoftSteppingParams *pSoftVolumeParams) const
{
   *pSoftVolumeParams = m_softVolumeParams;
}

void CSoftVolumeControlsLib::GetSoftMuteParams(SoftSteppingParams *pSoftMuteParams) const
{
   *pSoftMuteParams = m_softMuteParams;
}

uint32 CSoftVolumeControlsLib::GetSampleRate(void) const
{
   return m_sampleRate;
}

uint32 CSoftVolumeControlsLib::GetResumeWithDelayParam() const
{
   return m_muteBeforeResumeParams.m_resumeWithDelayInMs;
}

void CSoftVolumeControlsLib::SetBytesPerSample(const uint32_t bytesPerSample)
{
   boolean isSupported = ((2 == bytesPerSample) || (4 == bytesPerSample));

   if (isSupported)
   {
      m_bytesPerSample = bytesPerSample;
   }
}

void CSoftVolumeControlsLib::SetQFactor(const uint32 qFactor)
{
   boolean isSupported = ((15 == qFactor) || (27 == qFactor) || (31 == qFactor));

   if (isSupported)
   {
      m_qFactor = qFactor;
   }
}

void CSoftVolumeControlsLib::SetupPanner(SvpannerStruct *panner, uint32 newGainL32Q28, const SoftSteppingParams &params)
{
   // To avoid overflow, use 64 bit calculations here. This is achieved by having the sample rate be 64 bit.
   uint64 sampleRate    = m_sampleRate;
   uint32 periodSamples = params.periodMs * sampleRate / 1000;
   uint32 stepSamples   = params.stepUs * sampleRate / 1000000;
   panner->rampingCurve = params.rampingCurve;

   switch (params.rampingCurve)
   {
      case RAMP_LINEAR:
         SoftPannerLinearSetup(panner, newGainL32Q28, periodSamples, stepSamples);
         break;
      case RAMP_EXP:
         SoftPannerExpSetup(panner, newGainL32Q28, periodSamples, stepSamples);
         break;
      case RAMP_LOG:
         SoftPannerLogSetup(panner, newGainL32Q28, periodSamples, stepSamples);
         break;
      case RAMP_FRACT_EXP:
         SoftPannerFractExpSetup(panner, newGainL32Q28, periodSamples, stepSamples);
         break;
   }
}

boolean CSoftVolumeControlsLib::isUnityGain(const void *pChannelStruct) const
{
   if (m_isPaused)
   {
      return false;
   }

   const perChannelData *pChannelData = reinterpret_cast<const perChannelData *>(pChannelStruct);

   if (pChannelData->isMuted)
   {
      return false;
   }

   // Check if we are soft stepping
   const SvpannerStruct *pPanner = &pChannelData->panner;
   if (pPanner->targetgainL32Q28 != pPanner->currentGainL32Q28)
   {
      return false;
   }

   // Check if the gain is unity
   if (UNITY_L32_Q28 != pPanner->currentGainL32Q28)
   {
      return false;
   }

   return true;
}

void CSoftVolumeControlsLib::InitializePerChannelStruct(void *pChannelStruct)
{
   perChannelData *pChannelData = reinterpret_cast<perChannelData *>(pChannelStruct);

   pChannelData->chanGainQ28 = UNITY_L32_Q28;
   pChannelData->isMuted     = false;
   SetupPanner(&pChannelData->panner, pChannelData->chanGainQ28, m_softVolumeParams);
}

uint32 CSoftVolumeControlsLib::GetSizeOfPerChannelStruct(void)
{
   return sizeof(perChannelData);
}

void CSoftVolumeControlsLib::GoToTargetGainImmediately(void *pChannelStruct)
{
   perChannelData *pChannelData = reinterpret_cast<perChannelData *>(pChannelStruct);

   SoftSteppingParams params;
   params.periodMs     = 0;
   params.rampingCurve = RAMP_LINEAR;
   params.stepUs       = 0;

   SetupPanner(&pChannelData->panner, pChannelData->panner.targetgainL32Q28, params);
}

void CSoftVolumeControlsLib::SoftPannerFractExpSetup(SvpannerStruct *panner,
                                                     uint32          newGainL32Q28,
                                                     uint32          rampSamples,
                                                     uint32          step)
{
   /* substitute panner struct values with shorter names */
   uint32 currentGainL32Q28 = panner->currentGainL32Q28;
   uint32 sampleCounter     = 0;
   uint32 AL32Q28           = 0;
   uint32 BL32Q28           = 0;
   uint32 CL32Q31           = 0;
   int32  deltaCL32Q31      = 0;

   if (step <= 0)
   {
      step = 1;
   }

   /*----------------------------- if no ramp ------------------------------*/
   if (rampSamples <= 0)
   {
      currentGainL32Q28 = newGainL32Q28;
   } /* end of if (rampSamples <= 0) */
   /*--------------- if there is ramp, then need some work here ------------*/
   else /* (rampSamples > 0) */
   {
      if (currentGainL32Q28 < newGainL32Q28)
      {
         AL32Q28                      = currentGainL32Q28;
         BL32Q28                      = newGainL32Q28 - currentGainL32Q28;
         uint64 stepL64Q31            = ((uint64)(step)) << 31;
         uint32 stepOverSamplesL32Q31 = (uint32)(stepL64Q31 / rampSamples);
         deltaCL32Q31                 = stepOverSamplesL32Q31;
         CL32Q31                      = stepOverSamplesL32Q31;
      }
      else
      {
         AL32Q28                      = newGainL32Q28;
         BL32Q28                      = currentGainL32Q28 - newGainL32Q28;
         uint64 stepL64Q31            = ((uint64)(step)) << 31;
         int32  stepOverSamplesL32Q31 = (int32)(stepL64Q31 / rampSamples);
         deltaCL32Q31                 = -stepOverSamplesL32Q31;
         CL32Q31                      = (1u << 31) - stepOverSamplesL32Q31;
      }

      sampleCounter = rampSamples;
   } /* end of else (rampSamples > 0) */

   /*------------- store new values back into panner struct ----------------*/

   panner->step                      = step;
   panner->stepResidue               = step;
   panner->coeffs.fract.AL32Q28      = AL32Q28;
   panner->coeffs.fract.BL32Q28      = BL32Q28;
   panner->coeffs.fract.CL32Q31      = CL32Q31;
   panner->coeffs.fract.deltaCL32Q31 = deltaCL32Q31;
   panner->targetgainL32Q28          = newGainL32Q28; // update target gain
   panner->sampleCounter             = sampleCounter;
   panner->currentGainL32Q28         = currentGainL32Q28;
   panner->index                     = 0;
}
