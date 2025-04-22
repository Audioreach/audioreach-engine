/* ======================================================================== */
/**
   @file hwsw_rs_lib.h
   Header file for combo hw- sw resampler lib
*/

/*=========================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
========================================================================= */

#ifndef HWSW_RS_LIB_H
#define HWSW_RS_LIB_H

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/

#ifndef CAPI_UNIT_TEST
#include "shared_lib_api.h"
#else
#include "Elite_intf_extns_change_media_fmt.h"
#include "capi.h"
#endif

#include "audpp_util.h"
#include "audio_dsp32.h"
#include "resampler_32b_ext.h"
#include "hwsw_rs_lib_hw.h"
#include "hw_rs_lib.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplu
/*------------------------------------------------------------------------
 * Macros, Defines, Type declarations
 * -----------------------------------------------------------------------*/
#define INTM_GUARD_BITS 32
// Enum to list the number of stages supported
typedef enum {
	STAGE_ZERO,
	STAGE_ONE,
	MAX_MULTI_STAGES_SUPPORTED
}stage_t;

//Enum: column indices for sampling rates in table
typedef enum{
	 MS_IP_SR = 1,
	 MS_INT_SR = 2,
	 MS_OP_SR = 3,
	 MS_UNIT_IP_SMPLS = 4,    // unit frame size input sample index
	 MS_UNIT_INTM_SMPLS = 5,
	 MS_UNIT_OP_SMPLS = 6
}tbl_fs_idx;

typedef enum{
	FIXED_IN = 0,
	FIXED_OUT
}process_mode;

#define TEN_MS 10   // duration in ms

// Enum : Supported multi-stage list
typedef enum{
	DYRS_384_44_1,
	MAX_MULTI_STAGE_SUPPORTED_LIST
}ms_lst_fs_t;

const uint32_t multi_stage_sr_info[][7] = {
	 {DYRS_384_44_1, 384000, 192000, 44100, 384, 192, 44 },
};

// To print all lib status messages
//#define HWSW_RESAMPLER_DEBUG_MSG 1

// To print each frame status
//#define HWSW_RESAMPLER_PRINT_FRAME_STATS   1
/*------------------------------------------------------------------------
 * Structure definitions
 * -----------------------------------------------------------------------*/
typedef struct hwsw_rs_sw_mode_config
{
   uint16_t dynamic_mode; // 0: non-dynamic mode, 1: dynamic mode
   uint16_t delay_type;   // 0: High delay, 1: Low delay (relevant if dynamic mode is set)
} hwsw_rs_sw_mode_config_t;

typedef struct hwsw_rs_sw_lib_config
{
   uint32_t first_frame;
   int32_t  config_params[20];
} hwsw_rs_sw_lib_config_t;

typedef struct hwsw_rs_sw_lib_mem_req
{
   uint32_t drsStructSize;
   uint32_t drsGenCoeffSize;
   uint32_t drsPerChannelDataMemSize;
   uint32_t drsMemSize;
   uint32_t drsFiltCoeffMxSize;
} hwsw_rs_sw_lib_mem_req_t;

typedef struct hwsw_rs_sw_lib_mem_ptr
{
   // Library State structure memory
   void *pStructMem;

   // Generated polyphase filter coefficients
   void *pGenCoeff;

   // Per channel filter memory
   void *pChannelData[CAPI_MAX_CHANNELS_V2];

   // Coefficient matrix
   void *pCoefMatrix;

   // Filter selected based on inp/out freq
   void *pResampCoef;
} hwsw_rs_sw_lib_mem_ptr_t;

typedef struct hwsw_rs_sw_memory
{
   hwsw_rs_sw_lib_mem_req_t drs_mem_req;
   hwsw_rs_sw_lib_mem_req_t prev_drs_mem_req;
   hwsw_rs_sw_lib_mem_ptr_t drs_mem_ptr;
} hwsw_rs_sw_memory_t;

typedef struct hwsw_rs_media_fmt_t
{
   uint32_t inp_sample_rate;
   uint16_t bits_per_sample;
   uint16_t num_channels;
   uint32_t output_sample_rate;
   uint32_t q_factor;
} hwsw_rs_media_fmt_t;

typedef struct hwsw_rs_events_config
{
   uint32_t KPPS;
   uint32_t delay_in_us;
   uint32_t bandwidth;
} hwsw_rs_events_config_t;

typedef struct hwsw_rs_resampler_config
{
   uint32_t use_hw_rs;
   uint16_t dynamic_mode;
   uint16_t delay_type;
} hwsw_rs_resampler_config_t;

typedef enum hwsw_rs_using_resampler_t { SW_RESAMPLER = 0, HW_RESAMPLER, NO_RESAMPLER } hwsw_rs_using_resampler_t;

// Information required for multi-stage
typedef struct ms_info_t{
	uint32_t intm_max_smpls_ms;
	uint32_t num_stages;
	int32_t tbl_idx;  // index in the Multi stage sampling rate conversion table
	capi_stream_data_t intm_buf_ptr; // intermediate stage stream buffer
}ms_info_t;

typedef struct hwsw_resampler_lib
{
   // Flag which tells us which resampler is being used
   hwsw_rs_using_resampler_t this_resampler_instance_using;

   // Copy of Input Media format in lib
   hwsw_rs_media_fmt_t media_fmt;

   /* sw related structs */
   // Resampling mode (dynamic or non-dynamic) (High Delay or Low Delay)
   hwsw_rs_sw_mode_config_t sw_rs_config_param;

   // Sw resampler Library config structure.
   hwsw_rs_sw_lib_config_t* sw_lib_config_ptr[MAX_MULTI_STAGES_SUPPORTED];

   // info about allocated memory for sw resampler.
   hwsw_rs_sw_memory_t* sw_rs_mem_ptr[MAX_MULTI_STAGES_SUPPORTED];

   /* hw related structs */
   // hw resampler structure
   hwsw_rs_hw_resampler_t hw_resampler;

   /* dm related params set to the lib through set param handling when receiving dm mode and num samples
    * dm_mode decides if lib should operate in fixed out or fall back to default operation*/
   uint32_t dm_mode;
   /* output num of samples are provided to lib to be used when dm mode is set to fixed out mode
    * Lib by default only has info of output buffers max data length
    * For lib default behavior this variable wont be used */
   uint32_t output_fixed_samples;

   //flag to check if it is a multi_stage_process
   bool_t is_multi_stage_process;

   //multi stage information structure
   ms_info_t ms_info;
#ifdef HWSW_RESAMPLER_DEBUG_MSG
  FILE *fp_intm;
  FILE *fp_out;
#endif
} hwsw_resampler_lib_t;

/*------------------------------------------------------------------------
 * Function definitions
 * -----------------------------------------------------------------------*/
void hwsw_rs_lib_init(hwsw_resampler_lib_t *hwsw_rs_lib);

ar_result_t hwsw_rs_lib_deinit(hwsw_resampler_lib_t *hwsw_rs_ptr);

void hwsw_rs_hw_resampler_get_process_info(hwsw_rs_hw_resampler_t *hwrs_ptr,
                                           uint32_t                input_actual_samples,
                                           uint32_t                output_max_samples,
                                           uint32_t *              input_samples_to_consume_ptr,
                                           uint32_t *              output_samples_to_generate_ptr);

void hwsw_rs_lib_process_get_hw_process_info(hwsw_resampler_lib_t *hwsw_rs_ptr,
                                             uint32_t              input_actual_samples,
                                             uint32_t              output_max_samples,
                                             uint32_t *            input_samples_to_consume_ptr,
                                             uint32_t *            output_samples_to_generate_ptr);

ar_result_t hwsw_rs_lib_process(hwsw_resampler_lib_t *hwsw_rs_ptr,
                                capi_buf_t *       input_buf_ptr,
                                capi_buf_t *       output_buf_ptr,
                                uint32_t              input_num_bufs,
                                uint32_t              output_num_bufs,
								uint32_t              heap_id);

ar_result_t hwsw_rs_lib_check_create_resampler_instance(hwsw_resampler_lib_t *hwsw_rs_ptr, uint32_t heap_id);

void hwsw_rs_set_lib_mf(hwsw_resampler_lib_t *hwsw_rs_ptr, hwsw_rs_media_fmt_t *inp_mf);

void hwsw_rs_lib_set_dm_config(hwsw_resampler_lib_t *hwsw_rs_ptr, uint32_t dm_mode, uint32_t fixed_out_samples);

void hwsw_rs_lib_set_config(uint32_t              use_hwrs,
                            uint16_t              dyn_mode,
                            uint16_t              delay,
                            hwsw_resampler_lib_t *hwsw_rs_ptr,
                            bool_t *              update_rs);

ar_result_t hwsw_rs_lib_set_hwrs_suspend_resume(bool_t is_suspend, hwsw_resampler_lib_t *hwsw_rs_ptr, uint32_t *create_rs);

uint32_t hwsw_rs_lib_get_kpps(hwsw_resampler_lib_t *hwsw_rs_ptr);

uint32_t hwsw_rs_lib_get_bw(hwsw_resampler_lib_t *hwsw_rs_ptr);

uint32_t hwsw_rs_lib_get_alg_delay(hwsw_resampler_lib_t *hwsw_rs_ptr);

void hwsw_rs_lib_get_process_check(hwsw_resampler_lib_t *hwsw_rs_ptr, uint32_t *process_check);

ar_result_t hwsw_rs_lib_algo_reset(hwsw_resampler_lib_t *hwsw_rs_ptr);

bool_t hwsw_rs_lib_is_multi_stage_supported (hwsw_resampler_lib_t *hwsw_rs_ptr);

void hwsw_rs_lib_init_sw_rs_mem_inst(hwsw_resampler_lib_t *hwsw_rs_ptr, uint32_t stage_idx);

ar_result_t hwsw_rs_lib_create_lib_inst(hwsw_resampler_lib_t *hwsw_rs_ptr, uint32_t stage_idx, uint32_t req_size_per_instance,
                                        uint32_t heap_id);


#ifdef __cplusplus
}
#endif //__cplusplus

#endif // HWSW_RS_LIB_H
