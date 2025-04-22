/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
/*****************************************************************************
 * FILE NAME: SoftVolumeControls.h                                           
 * DESCRIPTION:                                                              
 *    Volume and Balance Controls                                            
******************************************************************************/

#ifndef _SOFTVOLUMECONTROLS_H_
#define _SOFTVOLUMECONTROLS_H_
#include "audio_comdef.h"

/*=============================================================================
      Constants
=============================================================================*/
static const uint32 RAMP_LINEAR    = 0;
static const uint32 RAMP_EXP       = 1;
static const uint32 RAMP_LOG       = 2;
static const uint32 RAMP_FRACT_EXP = 3;
static const uint32 PAUSE_RAMP_COMPLETE = 1;
/*===========================================================================*/
/*                                                                           */
/*                      current/----------------  <- targetgainL16Q12        */
/*                       time / .                                            */
/*                 start  .  /  .                                            */
/*                panning . /   .                                            */
/*                    .   v/..................... index                      */
/*                    .   / .                                                */
/*                    .  /  .                                                */
/*                    v /   .                                                */
/*                     /. . . . . . . . . . . .   <- currentgainL16Q12       */
/*                          .                                                */
/*                          index to the current ramp sample                 */
/*                     |<------->|                                           */
/*                       sampleCounter                                       */
/*                                                                           */
/*       ------------------------------------------> time                    */
/*  (as the panner works, sampleCounter will decrease until it becomes zero, */
/*   and at that moment, gain of the panner should reach targetGainL16Q15)   */
/*  (angle panner has similar structure, can use same graph as reference)    */
/*===========================================================================*/

struct linearCurveCoefficients
{
   /* for a linear ramp curve */
   int64 currentGainL64Q59;
   int64 deltaL64Q59; /* Delta is the difference in consecutive gains */
};

struct expCurveCoefficients
{
   int32 BL32Q26; /* Exp curve equation variables */
   int32 AL32Q26;
   int32 CL32Q26;
   int32 deltaCL32Q26;
};

struct logCurveCoefficients
{
   int32 AL32Q26; /* Additional Log curve equation variables */
   int32 BL32Q26;
   int32 CL32Q16;
   int32 deltaCL32Q16;
};

struct fractExpCurveCoefficients
{
   uint32 AL32Q28; /* Fractional exponent curve equation variables */
   uint32 BL32Q28;
   uint32 CL32Q31;
   int32  deltaCL32Q31;
};

union curveCoefficients
{
   linearCurveCoefficients   linear;
   expCurveCoefficients      exp;
   logCurveCoefficients      log;
   fractExpCurveCoefficients fract;
};

struct SvpannerStruct
{
   uint32 targetgainL32Q28;  /* Ramp to this gain value */
   uint32 currentGainL32Q28; /* Ramp from this gain value */
   uint32 sampleCounter;     /* Period of ramping */

   curveCoefficients coeffs;

   uint32 index;       /* index to the current ramp sample */
   uint32 step;        /* step is the number of samples on which the currentgainL16Q12 is
               applied before the calculating the new currentgainL16Q12 */
   uint32 stepResidue; /* Number of  samples in the previous frame to which the currentgainL16Q12
               has been applied. step-stepresidue are the number of samples to which the currentGainL16Q12
               should be applied for the current frame */
   uint32 rampingCurve;
   uint32 newGainL32Q28;
};

struct SoftSteppingParams
{
   uint32 periodMs;     // Soft stepping period in ms
   uint32 rampingCurve; // Use the RAMP_* constants defined in this file.
   uint32 stepUs;       // Soft stepping step in us.
};

struct perChannelData
{
   uint32         chanGainQ28;
   boolean        isMuted; // Indicates whether muted or not
   SvpannerStruct panner;
};

struct PerChannelDataBlock
{
   int8_t *       inPtr;
   int8_t *       outPtr;
   uint32_t       sampleCount;
   perChannelData channelStruct;
};

struct MuteBeforeRampParams
{
   uint32 m_resumeWithDelayInMs; //time in ms for which the module is muted before resume rampup
   uint32 m_muteBeforeRampSamples; //number of mute samples before ramping up
   uint32 m_muteSamplesPending; //number of mute samples pending before ramping up

};

class CSoftVolumeControlsLib
{

 private:
   enum PauseState
   {
      STEADY,
      RAMPING_DOWN,
      PAUSE,
      WAITING,
      RAMPING_UP,
	  MUTE_BEFORE_RAMPUP
   };

   enum PauseCommand
   {
      COMMAND_PAUSE,
      COMMAND_RESUME,
      COMMAND_FORCE_PAUSE,
      COMMAND_INIT
   };

   uint32             m_sampleRate; // Sampling rate of the stream
   uint32             m_bytesPerSample;
   boolean            m_isPaused; // Indicates whether paused or not
   SoftSteppingParams m_softVolumeParams;
   SoftSteppingParams m_softMuteParams;
   SoftSteppingParams m_softPauseParams;
   SoftSteppingParams m_softResumeParams;
   MuteBeforeRampParams m_muteBeforeResumeParams;
   PauseState m_pauseState;

   uint32 m_thresholdQ15; // threshold 16-bit
   uint32 m_thresholdQ27; // threshold 24-bit
   uint32 m_thresholdQ31; // threshold 32-bit
   uint32 m_qFactor;

 public:
   CSoftVolumeControlsLib();
   ~CSoftVolumeControlsLib();

   // Functions for handling the per channel structures
   static uint32 GetSizeOfPerChannelStruct(void);
   void InitializePerChannelStruct(void *pChannelStruct);
   /*
    * This function is to be use for the following case:
    * If a media type comes during ramping and some channel goes
    * away, the data from that channel will immediately stop. So
    * the state of the panner will stay in the middle of ramping.
    * When the channel re-appears, we do not want it to continue
    * ramping from the old state since the other channels will
    * have finished ramping by this point.
    *
    * Hence, this function should be called whenever a new channel
    * appears in the data, to ensure that any previous ramping
    * state is discarded and it starts immediately from its target
    * gain.
    */
   void GoToTargetGainImmediately(void *pChannelStruct);

   //returns the target gain.
   uint32 GetTargetGain(void *pChannelStruct);

   // Functions to process the data stream
   /* Lib function takes in ptr, out ptr, sample count and the channel struct for each channel
    * To be used when individual channels can be processed at a time */
   void Process(void *pInPtr, void *pOutPtr, const uint32 nSampleCnt, void *pChannelStruct);

   /* Function takes all channels' data as input. Loops over each channel inside the lib.
    * Sets flag is_paused to true when module goes to pause state after processing all channels
    * To be used when common flag is to be set over all channels just once */
   void ProcessAllChannels(PerChannelDataBlock *pChannels, const uint32 nChannelCnt, uint8_t *is_paused);

   // Threshold related functions
   void SetThreshold(uint32 pThreshold_dBfs);
   uint32 GetThreshold(void) const;
   int32 DetectThreshold(void *pInPtr, const uint32 nSampleCnt);

   // Media format related functions
   void SetSampleRate(uint32_t oldSampleRate, uint32_t newSampleRate, void *pChannelStruct);
   uint32 GetSampleRate(void) const;
   void SetBytesPerSample(const uint32_t bytesPerSample);
   void SetQFactor(const uint32 qFactor);
   uint32 GetBytesPerSample(void) const;

   // Volume related functions
   void SetVolume(const uint32_t gainQ28, void *pChannelStruct);
   void SetSoftVolumeParams(const SoftSteppingParams &softVolumeParams);
   void SetSoftMuteParams(const SoftSteppingParams &softMuteParams);
   void GetSoftVolumeParams(SoftSteppingParams *pSoftVolumeParams) const;
   void GetSoftMuteParams(SoftSteppingParams *pSoftMuteParams) const;

   // Mute related functions.
   // Note: There is no soft stepping on mute/unmute.
   void Mute(void *pChannelStruct);
   void Unmute(void *pChannelStruct);
   boolean IsMuted(const void *pChannelStruct) const;

   // Pause related parameters.
   void SetSoftPauseParams(const SoftSteppingParams &softPauseParams);
   void SetSoftResumeParams(const SoftSteppingParams &softResumeParams);
   void SetResumeWithDelayParam(const uint32 &resumeWithDelayParam);
   uint32 GetResumeWithDelayParam() const;
   void GetSoftPauseParams(SoftSteppingParams *pSoftPauseParams) const;
   void GetSoftResumeParams(SoftSteppingParams *pSoftPauseParams) const;
   void StartSoftPause(void *pChannelStruct); // Has to be called for every channel when pausing
   void StartSoftPauseAllChannels(PerChannelDataBlock *pChannels, const uint32 nChannelCnt);
   void StartSoftResume(void *pChannelStruct); // Has to be called for every channel when resuming
   void StartSoftResumeAllChannels(PerChannelDataBlock *pChannels, const uint32 nChannelCnt);
   void ForcePause(PerChannelDataBlock *pChannels, const uint32 nChannelCnt); // Forces transition to PAUSE state.
   uint32 GetPauseRampPeriod() const;
   uint32 GetPauseState() const;

   // Will return true if the library is applying a steady state unity gain to the channel.
   // Can be used for optimization - no need to call the library if this returns true.
   boolean isUnityGain(const void *pChannelStruct) const;

 private:
   // Functions for applying the ramps
   void ApplyLinearRamp(void *pOutPtr, void *pInPtr, SvpannerStruct *panner, uint32 samples);
   void ApplyLogRamp(void *pOutPtr, void *pInPtr, SvpannerStruct *panner, uint32 samples);
   void ApplyExpRamp(void *pOutPtr, void *pInPtr, SvpannerStruct *panner, uint32 samples);
   void ApplyFractExpRamp(void *pOutPtr, void *pInPtr, SvpannerStruct *panner, uint32 samples);

   // Functions for setting up the panners
   void SetupPanner(SvpannerStruct *panner, uint32 newGainL32Q28, const SoftSteppingParams &params);
   void SoftPannerLinearSetup(SvpannerStruct *panner,        /* panner struct                     */
                              uint32          newGainL32Q28, /* new target panner gain            */
                              uint32          rampSamples,   /* number of samples in the ramp     */
                              uint32          step);
   void SoftPannerExpSetup(SvpannerStruct *panner,        /* panner struct                     */
                           uint32          newGainL32Q28, /* new target panner gain  in Q28    */
                           uint32          rampSamples,   /* number of samples in the ramp     */
                           uint32          step);
   void SoftPannerLogSetup(SvpannerStruct *panner,        /* panner struct                     */
                           uint32          newGainL32Q28, /* new target panner gain            */
                           uint32          rampSamples,   /* number of samples in the ramp     */
                           uint32          step);
   void SoftPannerFractExpSetup(SvpannerStruct *panner,        /* panner struct                     */
                                uint32          newGainL32Q28, /* new target panner gain            */
                                uint32          rampSamples,   /* number of samples in the ramp     */
                                uint32          step);

   void ApplySteadyGain(void *pOutPtr, void *pInPtr, const uint32 gainQ28, uint32 samples);
   void ApplySteadyGain16(void *pOutPtr, void *pInPtr, const uint32 gainQ28, uint32 samples);
   void ApplySteadyGain32(void *pOutPtr, void *pInPtr, const uint32 gainQ28, uint32 samples);
   void IncrementPointer(void **pPtr, uint32 samples);

   // Threshold functions
   int32 DetectThreshold16(void *pInPtr, const uint32 nSampleCnt);
   int32 DetectThreshold24(void *pInPtr, const uint32 nSampleCnt);
   int32 DetectThreshold32(void *pInPtr, const uint32 nSampleCnt);

   // process function
   boolean ProcessV2SingleChannel(void *inPtr, void *pOutPtr, const uint32 nSampleCnt, void *pChannelStruct);

   // State management.
   boolean ShouldRespond(PauseCommand command);
   void StateTransition(PauseCommand command);
};

#endif // _SOFTVOLUMECONTROLS_H_
