/*========================================================================

file pcm_converter.h
This file contains functions for compression-decompression container

   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
======================================================================*/

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#ifndef PCM_CONVERTER_H
#define PCM_CONVERTER_H
#include "audio_basic_op_ext.h"
#include "ar_error_codes.h"
#include "capi.h"
#include "capi_cmn.h"
#include "shared_lib_api.h"
#include "shared_aud_cmn_lib.h"
#include "common_enc_dec_api.h"
#include "pcm_converter_api.h"
#include "pcm_encoder_api.h"
#include "pcm_decoder_api.h"
#include "ChannelMixerLib.h"
#include "hwsw_rs_lib.h"
#include "iir_rs_lib.h"
#include "spf_interleaver.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/*--------------------------------------------------------------*/
/* Macro definitions                                            */
/* -------------------------------------------------------------*/
//#define USE_Q6_SPECIFIC_CODE
//#define PCM_CNV_LIB_DEBUG 1
#define MIID_UNKNOWN 0

#define CNV_MSG_PREFIX "MFC_PCM_CNV:[%lX] "
#define CNV_MSG(ID, xx_ss_mask, xx_fmt, ...) AR_MSG(xx_ss_mask, CNV_MSG_PREFIX xx_fmt, ID, ##__VA_ARGS__)

#define CNV_IIR_RS_MIN_FRAME_SIZE_US 1000
#define MEM_ALIGN_EIGHT_BYTE         8

/*----------------------------------------------------------------------------
 * Global Data Definitions
 * -------------------------------------------------------------------------*/
static const uint32_t MAX_Q23                         = ((1 << PCM_Q_FACTOR_23) - 1);
static const uint32_t MIN_Q23                         = (-(1 << PCM_Q_FACTOR_23));
static const uint32_t MAX_Q27                         = ((1 << PCM_Q_FACTOR_27) - 1);
static const uint32_t MIN_Q27                         = (-(1 << PCM_Q_FACTOR_27));
static const uint32_t MAX_Q31                         = 0x7FFFFFFF;
static const uint32_t MIN_Q31                         = 0x80000000;
static const uint32_t NUMBER_OF_PROCESS               = 11;
static const uint32_t MAX_NUMBER_OF_BUFFERING_PROCESS = 7;
static const uint32_t NUMBER_OF_CHANNEL_MAPS          = 2;
static const uint32_t NUMBER_OF_REMAP_BUFFERS         = 4;
static const uint32_t MAX_BW_IDX                      = 3;
// All the kpps values are scaled based on sample word size
// Indices to WS map is as follows,
// 0 - > 16 bit
// 1 - > 24 bit
// 2 - > 32 bit
static const uint32_t endianness_kpps_per_ch[MAX_BW_IDX] = { 230, 180, 130 };
static const uint32_t intl_dintl_kpps_per_ch[MAX_BW_IDX] = { 110, 640, 110 };
static const uint32_t byte_cnv_kpps_per_ch[MAX_BW_IDX]   = { 100, 300, 120 };
static const uint32_t ch_mixer_kpps_per_ch[MAX_BW_IDX]   = { 450, 500, 500 };
static const uint32_t memcpy_kpps_per_ch[MAX_BW_IDX]     = { 100, 100, 100 };
static const uint32_t fw_kpps                            = 100; // just a little extra kpps for logic

static const uint32_t base_sample_rate = 48000;

#define PC_CEIL(x, y) (((x) + (y)-1) / (y))

// Enlists possible combinations of media_fmt for a given buf
// BW- bit width, W - word size, Q - q-factor
typedef enum pc_mf_combo_t {
   PC_INVALID_CMBO = 0,
   PC_BW16_W16_Q15,
   PC_BW24_W24_Q23,
   PC_BW24_W32_Q23,
   PC_BW24_W32_Q27,
   PC_BW24_W32_Q31,
   PC_BW32_W32_Q31,
   PC_BW32_W32_FLOAT,
   PC_BW64_W64_DOUBLE,
} pc_mf_combo_t;

typedef enum pc_endian_t {
   PC_UNKNOWN_ENDIAN = PARAM_VAL_INVALID,
   PC_LITTLE_ENDIAN  = PCM_LITTLE_ENDIAN,
   PC_BIG_ENDIAN     = PCM_BIG_ENDIAN
} pc_endian_t;

typedef enum pc_alignment_t {
   PC_UNKNOWN_ALIGNMENT = PARAM_VAL_INVALID,
   PC_LSB_ALIGNED       = PCM_LSB_ALIGNED,
   PC_MSB_ALIGNED       = PCM_MSB_ALIGNED
} pc_alignment_t;

/* PC handles unpacked data, in the CAPI_DEINTERLEVED_UNPACKED_V2 format,
   i.e it reads/updates only first chs lengths */
typedef enum pc_interleaving_t
{
   PC_UNKNOWN_INTERLEAVING      = PARAM_VAL_INVALID,
   PC_INTERLEAVED               = PCM_INTERLEAVED,
   PC_DEINTERLEAVED_PACKED      = PCM_DEINTERLEAVED_PACKED,
   PC_DEINTERLEAVED_UNPACKED_V2 = PCM_DEINTERLEAVED_UNPACKED
} pc_interleaving_t;

// Enlisting data format to the pc module
typedef enum pc_data_format_t {
   PC_FIXED_FORMAT = 0,
   PC_FLOATING_FORMAT,
   PC_INVALID_FORMAT,
} pc_data_format_t;

// Media Format used for
typedef struct pc_media_fmt_t
{
   uint32_t          sampling_rate;
   pc_endian_t       endianness;
   pc_interleaving_t interleaving;
   pc_mf_combo_t     byte_combo;
   pc_alignment_t    alignment;
   pc_data_format_t  data_format;
   uint16_t          bit_width;
   uint16_t          word_size;
   uint16_t          q_factor;
   uint16_t          num_channels;
   uint16_t *        channel_type;
} pc_media_fmt_t;

// Process modules which can be present in the PCM pipeline
// UP - comes before channel map module
// Down - comes after channel map module
typedef union pc_process_flags_t
{
   struct
   {
      uint32_t ENDIANNESS_PRE : 1;
      uint32_t DATA_CNV_FLOAT_TO_FIXED : 1;
      uint32_t INT_DEINT_PRE : 1;
      uint32_t BYTE_CNV_PRE : 1;
      uint32_t RESAMPLER_PRE : 1;
      uint32_t CHANNEL_MIXER : 1;
      uint32_t RESAMPLER_POST : 1;
      uint32_t BYTE_CNV_POST : 1;
      uint32_t INT_DEINT_POST : 1;
      uint32_t DATA_CNV_FIXED_TO_FLOAT : 1;
      uint32_t ENDIANNESS_POST : 1;
   };
   uint32_t word;
} pc_process_flags_t;

typedef enum pc_proc_flags_t {
   ENDIANNESS_PRE = 0,
   DATA_CNV_FLOAT_TO_FIXED,
   INT_DEINT_PRE,
   BYTE_CNV_PRE,
   RESAMPLER_PRE,
   CHANNEL_MIXER,
   RESAMPLER_POST,
   BYTE_CNV_POST,
   INT_DEINT_POST,
   DATA_CNV_FIXED_TO_FLOAT,
   ENDIANNESS_POST,
} pc_proc_flags_t;

typedef enum pc_resampler_type_t { FIR_RESAMPLER = 0, IIR_RESAMPLER = 1, IIR_PREFERRED } pc_resampler_type_t;

typedef enum pc_rs_position_flag_t { RESAMPLER_POSITION_PRE, RESAMPLER_POSITION_POST } pc_rs_position_flag_t;

typedef enum pcm_mf_cnv_type_t { PCM_CNV = 0, PCM_ENC, PCM_DEC, MFC } pcm_mf_cnv_type_t;
// This structure is used as a v-table for process calls for modules in the pcm pipeline
typedef struct pc_proc_info_t
{
   // Pointer to the process function
   ar_result_t (*process)(void *          me_ptr,
                          capi_buf_t *    input_buf_ptr,
                          capi_buf_t *    output_buf_ptr,
                          pc_media_fmt_t *input_media_fmt_ptr,
                          pc_media_fmt_t *output_media_fmt_ptr);

   // Pointer to the output buffer.
   // We point this to the de-interleaved-unpacked buffers part of the pc_lib_t struct, because
   // 1.Actual buffers are not available in the init phase
   // 2.Actual buffer ptr values may change
   // 3.We might have to re-map the buffers run time since most of the process functions take deint-unpacked fmt
   capi_buf_t *output_buffer_ptr;

   // Output media format for the module. Input media fmt is obtained form the previous process's output media fmt
   pc_media_fmt_t output_media_fmt;

} pc_proc_info_t;

typedef struct pc_core_lib_t
{
   // Memory pointer to the memory required for channel maps and remap buffers
   void *pc_mem_ptr;

   // Max of input and output number of channels
   uint16_t max_num_channels;

   // Input Media format
   pc_media_fmt_t input_media_fmt;

   // Output media format
   pc_media_fmt_t output_media_fmt;

   // Flags to hold information on which module is enabled in the pipeline
   pc_process_flags_t flags;

   // Number of modules with cannot do processing in-place
   uint32_t no_of_buffering_stages;

   // Process info v-table required to call process of each module
   pc_proc_info_t pc_proc_info[NUMBER_OF_PROCESS];

   // Temporary de-interleaved-unpacked buffer ptr holder for fwk input
   capi_buf_t *remap_input_buf_ptr;

   // Temporary de-interleaved-unpacked buffer ptr holder for scratch buffer 1
   capi_buf_t *remap_scratch_buf1_ptr;

   // Temporary de-interleaved-unpacked buffer ptr holder for scratch buffer 2
   capi_buf_t *remap_scratch_buf2_ptr;

   // Temporary de-interleaved-unpacked buffer ptr holder for fwk output buffer
   capi_buf_t *remap_output_buf_ptr;
} pc_core_lib_t;

// Main library structure for pcm converter
typedef struct pc_lib_t
{
   pc_core_lib_t core_lib;

   // log id used for printing
   uint32_t miid;

   // Channel mixer library structure
   ChMixerStateStruct *ch_lib_ptr;

   // either fir or iir resampler
   pc_resampler_type_t resampler_type;

   // fir resampler config
   uint32_t use_hw_rs;

   // fir resampler dyn mode
   uint16_t dynamic_mode;

   // for resampler delay type
   uint16_t delay_type;

   // hw-sw (fir) resampler lib
   hwsw_resampler_lib_t *hwsw_rs_lib_ptr;

   // previous resampler position
   pc_rs_position_flag_t resampler_used;

   // iir resampler lib
   iir_rs_lib_t *iir_rs_lib_ptr;

   // For IIR_RS use cases, this is used to send the library the fixed frame size. It is also
   // used to ensure we only consume one frame_size worth of data per process call. Capi client
   // will send aggregated frame size (ex 20ms, or 1ms when threshold is disabed).
   uint32_t cntr_frame_size_us;
   uint32_t frame_size_us;
   uint32_t in_frame_samples_per_ch; // To avoid process call divisions.
   uint32_t out_frame_samples_per_ch;
   uint32_t heap_id;
   bool_t   consume_partial_input;
   bool_t   iir_pref_set;
} pc_lib_t;

// Process function

ar_result_t pc_endianness_process(void *          me_ptr,
                                  capi_buf_t *    input_buf_ptr,
                                  capi_buf_t *    output_buf_ptr,
                                  pc_media_fmt_t *input_media_fmt_ptr,
                                  pc_media_fmt_t *output_media_fmt_ptr);

ar_result_t pc_interleaving_process(void *          me_ptr,
                                    capi_buf_t *    input_buf_ptr,
                                    capi_buf_t *    output_buf_ptr,
                                    pc_media_fmt_t *input_media_fmt_ptr,
                                    pc_media_fmt_t *output_media_fmt_ptr);

ar_result_t pc_byte_morph_process(void *          me_ptr,
                                  capi_buf_t *    input_buf_ptr,
                                  capi_buf_t *    output_buf_ptr,
                                  pc_media_fmt_t *input_media_fmt_ptr,
                                  pc_media_fmt_t *output_media_fmt_ptr);

ar_result_t pc_channel_mix_process(void *          me_ptr,
                                   capi_buf_t *    input_buf_ptr,
                                   capi_buf_t *    output_buf_ptr,
                                   pc_media_fmt_t *input_media_fmt_ptr,
                                   pc_media_fmt_t *output_media_fmt_ptr);

ar_result_t pc_dyn_resampler_process(void *          me_ptr,
                                     capi_buf_t *    input_buf_ptr,
                                     capi_buf_t *    output_buf_ptr,
                                     pc_media_fmt_t *input_media_fmt_ptr,
                                     pc_media_fmt_t *output_media_fmt_ptr);

ar_result_t pc_iir_resampler_process(void *          me_ptr,
                                     capi_buf_t *    input_buf_ptr,
                                     capi_buf_t *    output_buf_ptr,
                                     pc_media_fmt_t *input_media_fmt_ptr,
                                     pc_media_fmt_t *output_media_fmt_ptr);

ar_result_t pc_float_to_fixed_conv_process(void *          me_ptr,
                                           capi_buf_t *    input_buf_ptr,
                                           capi_buf_t *    output_buf_ptr,
                                           pc_media_fmt_t *input_media_fmt_ptr,
                                           pc_media_fmt_t *output_media_fmt_ptr);

ar_result_t pc_fixed_to_float_conv_process(void *          me_ptr,
                                           capi_buf_t *    input_buf_ptr,
                                           capi_buf_t *    output_buf_ptr,
                                           pc_media_fmt_t *input_media_fmt_ptr,
                                           pc_media_fmt_t *output_media_fmt_ptr);

ar_result_t pc_intlv_16_out(capi_buf_t *input_buf_ptr,
                            capi_buf_t *output_buf_ptr,
                            uint16_t    word_size_in,
                            uint16_t    q_factor_in);

ar_result_t pc_intlv_24_out(capi_buf_t *input_buf_ptr,
                            capi_buf_t *output_buf_ptr,
                            uint16_t    word_size_in,
                            uint16_t    q_factor_in);

ar_result_t pc_intlv_32_out(capi_buf_t *input_buf_ptr,
                            capi_buf_t *output_buf_ptr,
                            uint16_t    word_size_in,
                            uint16_t    q_factor_in,
                            uint16_t    q_factor_out);

/** Generates DEINTERLEAVED UNPACKED V1 output i.e updates all the chs data len's*/
ar_result_t pc_deintlv_16_out(capi_buf_t *input_buf_ptr,
                              capi_buf_t *output_buf_ptr,
                              uint16_t    num_channels,
                              uint16_t    word_size_in,
                              uint16_t    q_factor_in);

/** Generates DEINTERLEAVED UNPACKED V1 output i.e updates all the chs data len's*/
ar_result_t pc_deintlv_24_out(capi_buf_t *input_buf_ptr,
                              capi_buf_t *output_buf_ptr,
                              uint16_t    num_channels,
                              uint16_t    word_size_in,
                              uint16_t    q_factor_in);

/** Generates DEINTERLEAVED UNPACKED V1 output i.e updates all the chs data len's*/
ar_result_t pc_deintlv_32_out(capi_buf_t *input_buf_ptr,
                              capi_buf_t *output_buf_ptr,
                              uint16_t    num_channels,
                              uint16_t    word_size_in,
                              uint16_t    q_factor_in,
                              uint16_t    q_factor_out);

/** Generates DEINTERLEAVED UNPACKED V2 output i.e updates only first chs data len's*/
ar_result_t pc_deintlv_unpacked_v2_16_out(capi_buf_t *input_buf_ptr,
                                          capi_buf_t *output_buf_ptr,
                                          uint16_t    num_channels,
                                          uint16_t    word_size_in,
                                          uint16_t    q_factor_in);

/** Generates DEINTERLEAVED UNPACKED V2 output i.e updates only first chs data len's*/
ar_result_t pc_deintlv_unpacked_v2_24_out(capi_buf_t *input_buf_ptr,
                                          capi_buf_t *output_buf_ptr,
                                          uint16_t    num_channels,
                                          uint16_t    word_size_in,
                                          uint16_t    q_factor_in);

/** Generates DEINTERLEAVED UNPACKED V2 output i.e updates only first chs data len's*/
ar_result_t pc_deintlv_unpacked_v2_32_out(capi_buf_t *input_buf_ptr,
                                          capi_buf_t *output_buf_ptr,
                                          uint16_t    num_channels,
                                          uint16_t    word_size_in,
                                          uint16_t    q_factor_in,
                                          uint16_t    q_factor_out);

ar_result_t pc_change_endianness(int8_t   *src_ptr,
                                 int8_t   *dest_ptr,
                                 uint32_t  src_actual_len,
                                 uint32_t *dest_actual_len,
                                 uint32_t  word_size);
/* -----------------------------------------------------------------------
 ** API Func
 ** ----------------------------------------------------------------------- */
bool_t pc_is_floating_point_data_format_supported();
bool_t pc_is_ch_mixer_needed(pc_lib_t *      pc_ptr,
                             int16_t *       coef_set_ptr,
                             pc_media_fmt_t *input_media_fmt_ptr,
                             pc_media_fmt_t *output_media_fmt_ptr);

ar_result_t pc_init(pc_lib_t *      pc_ptr,
                    pc_media_fmt_t *input_media_fmt_ptr,
                    pc_media_fmt_t *output_media_fmt_ptr,
                    void *          coef_set_ptr,
                    uint32_t        heap_id,
                    bool_t *        lib_enable,
                    bool_t          fir_iir_rs_switch,
                    uint32_t        miid,
                    uint8_t         type);

void pc_deinit(pc_lib_t *pc_ptr);

void pc_get_kpps_and_bw(pc_lib_t *pc_ptr, uint32_t *kpps, uint32_t *bw);

uint32_t pc_get_algo_delay(pc_lib_t *pc_ptr);

pc_mf_combo_t pc_classify_mf(pc_media_fmt_t *mf_ptr);

ar_result_t pc_process(pc_lib_t *  me_ptr,
                       capi_buf_t *input_buf_ptr,
                       capi_buf_t *output_buf_ptr,
                       capi_buf_t *scratch_buf_ptr_1,
                       capi_buf_t *scratch_buf_ptr_2);

uint32_t pc_get_fixed_out_samples(pc_lib_t *pc_ptr, uint32_t req_out_samples);

void pc_set_frame_size(pc_lib_t *pc_ptr, uint32_t frame_size_us);

void pc_set_cntr_frame_size(pc_lib_t *pc_ptr, uint32_t cntr_frame_size_us);
void pc_set_consume_partial_input(pc_lib_t *pc_ptr, bool_t consume_partial_input);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif // PCM_CONVERTER_H
