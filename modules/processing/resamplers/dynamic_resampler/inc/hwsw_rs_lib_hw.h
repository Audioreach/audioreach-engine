/* ======================================================================== */
/**
   @file hwsw_rs_lib_hw.h
   Header file for combo hw-sw resampler lib with hw defines
*/

/*=========================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
========================================================================= */

#ifndef HWSW_RS_LIB_HW_H
#define HWSW_RS_LIB_HW_H

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/

#include "posal.h"
#if ((defined __hexagon__) || (defined __qdsp6__))
#include "hwd_devcfg.h"
#endif
#include "hw_rs_lib.h"


/*------------------------------------------------------------------------
 * Macros, Defines, Type declarations
 * -----------------------------------------------------------------------*/
static const uint32_t HW_RS_MAX_FREQ     = 384000;
static const uint32_t HW_RS_MAX_CHANNELS = 32;

// OSR for Down-sampling is 24.
static const uint32_t HW_RS_MAX_DOWNSAMPLE_RATIO = 24;

// Highest OSR for Up-sampling is 160.
static const uint32_t HW_RS_MAX_UPSAMPLE_RATIO = 160;

static const uint32_t HW_RS_BUFFER_DELAY_US        = 10000;                        // 10 MS
static const uint32_t HW_RS_BUFFER_DELAY_MS        = HW_RS_BUFFER_DELAY_US / 1000; // 10 MS
static const int32_t  HW_RS_FRAME_SIZE_US          = HW_RS_BUFFER_DELAY_US;        // 10 MS
static const uint32_t HW_RS_PROCESS_TIME_THRESHOLD = 7000;

/*------------------------------------------------------------------------
 * Structure definitions
 * -----------------------------------------------------------------------*/

// Hw Resampler Input-Output Media info Structure.
typedef struct hwsw_rs_hw_resampler_media_info
{
   int32  in_sampling_rate;
   int32  out_sampling_rate;
   uint16 bits_per_sample;
   uint16 num_channels;
} hwsw_rs_hw_resampler_media_info_t;

// Hw Resampler job signal Structure
typedef struct hwsw_rs_hw_resampler_job_signal
{
   posal_signal_t  sig_job_ptr;     // signal for the job submitted
   posal_channel_t channel_job_ptr; // channel used to embed the signal
   uint32          sig_mask;
} hwsw_rs_hw_resampler_job_signal_t;

// Hw Resampler Internal Buffer Structure
typedef struct hwsw_rs_hw_resampler_internal_buf
{
   capi_buf_t buf[HW_RS_MAX_CHANNELS];
   uint32_t      start_byte_index;    // read byte index in each buffer
   uint32_t      valid_bytes;         // valid bytes in each buffer
   uint32_t      frame_size_in_bytes; // bytes allocated per buffer
} hwsw_rs_hw_resampler_internal_buf_t;

// Hw Resampler Buffer Manager.
typedef struct hwsw_rs_hw_resampler_buffer_mgr
{
   // Buffer where data is copied from ext input buffer
   hwsw_rs_hw_resampler_internal_buf_t *data_input_buffer_ptr;

   // Input Buffer which is used to call the Hw resampler process
   hwsw_rs_hw_resampler_internal_buf_t *process_input_buffer_ptr;

   // Buffer from data is copied to ext output buffer
   hwsw_rs_hw_resampler_internal_buf_t *data_output_buffer_ptr;

   // Output Buffer which is used to call the Hw resampler process
   hwsw_rs_hw_resampler_internal_buf_t *process_output_buffer_ptr;

   // Pointer to the memory allocated for buffers.
   int8_t *mem_ptr;

   // size of allocated memory.
   uint32_t mem_size;

   // Total normalized input samples consumed by the module
   uint32_t input_samples_consumed;

   // Total normalized output samples produced by the module
   uint32_t output_samples_produced;

   // number of samples to consume precalculated before process
   uint32_t input_samples_to_consume;

   // number of samples to produce precalculated before process
   uint32_t output_samples_to_generate;
} hwsw_rs_hw_resampler_buffer_mgr_t;

// Hw Resampler Structure
typedef struct hwsw_rs_hw_resampler
{
   /*
    * This flag is used to identify if Hw resampler is supported by Chip.
    * It can also be false if client want to force-disable the Hw resampler.
    */
   bool_t hw_resampler_supported;

   /*
    * This is in an internal flag which is set only when both hw resampler can be used
    * AND client has asked to use it through the set param api
    */
   bool_t use_hw_rs;

   /*
    * This flag is used to identify if a job is completed or not.
    * If it is true then it doesn't always mean that job is pending at Hw resampler.
    * It is possible that Hw resampler has completed the job and raised the signal through
    * callback function. But caller hasnt cleared that signal yet.
    */
   bool_t is_job_pending;

   // Flag to identify if session is suspended or not.
   bool_t is_session_suspended;

   /*
    * It accumulates the minimum estimated time(at max clock) of all those jobs
    * which are closed because of the session suspend.
    * We intend to reopen these jobs once session is resumed.
    * This variable is used to correct the Time threshold.
    */
   posal_atomic_word_t *estimated_time_of_all_suspended_session;

   // Minimum estimated time(at max clock) to complete the current job only
   uint32_t minimum_estimated_time_of_this_session;

   // Signal, used to identify if a job is completed by Hw resampler.
   hwsw_rs_hw_resampler_job_signal_t job_signal;

   // Media info for which Hw resampler job is need to be opened.
   hwsw_rs_hw_resampler_media_info_t media_info;

   // Buffer manager which keeps the pointer to all the process and data buffers.
   hwsw_rs_hw_resampler_buffer_mgr_t buf_mgr;

   // Mutex, to avoid simultaneous access of Hw resampler api.
   hw_resampler_mutex_t *mutex_info;

   // Hw resampler handle.
   //rs_drv_handle_t rs_handle;

   // Hw resampler job config structure.
   //rs_drv_job_cfg_t rs_job_cfg;

   // heap id, to allocate the memory for buffers.
   uint32_t heap_id;

   // keeps record of number of Hw jobs currently opened.
   posal_atomic_word_t *phwrs_resource_state;

#if HW_RS_JOB_COMPLETION_TIME
   uint64_t job_complete_time;
#endif
} hwsw_rs_hw_resampler_t;

#endif // HWSW_RS_LIB_HW_H
