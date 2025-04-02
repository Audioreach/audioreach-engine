/* ======================================================================== */
/**
   @file capi_iir_mbdrc_utils.h

   Header file to implement utilities for MBDRC audio Post
   Processing module
 */

/* =========================================================================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause
   ========================================================================== */

#ifndef CAPI_IIR_MBDRC_UTILS_H
#define CAPI_IIR_MBDRC_UTILS_H

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "capi.h"
#include "capi_cmn.h"
#ifdef CAPI_STANDALONE
#include "capi_util.h"
#else
#include "posal.h"
#endif

#include "capi_iir_mbdrc.h"
#include "iir_mbdrc_api.h"
#include "iir_mbdrc_calibration_api.h"
#include "mbdrc_api.h"
#include "stringl.h"
/*------------------------------------------------------------------------
 * Macros, Defines, Type declarations
 * -----------------------------------------------------------------------*/

#define MIID_UNKNOWN 0

#define IIR_MBDRC_MSG_PREFIX "IIR_MBDRC:[%lX] "
#define IIR_MBDRC_MSG(ID, xx_ss_mask, xx_fmt, ...) AR_MSG(xx_ss_mask, IIR_MBDRC_MSG_PREFIX xx_fmt, ID, ##__VA_ARGS__)

// Default values for iir_mbdrc static parameters
static const uint32 CAPI_IIR_MBDRC_DEFAULT_SAMPLING_RATE = 48000; // sampling rate
static const uint32 CAPI_IIR_MBDRC_DEFAULT_NUM_CHANNELS  = 1;     // Mono channel
static const uint32 CAPI_IIR_MBDRC_DEFAULT_BLOCK_SIZE    = 480;   // 480 = 10ms samples at 48KHz

#define IIR_MBDRC_MAX_MSIIR_STATES_MEM (32)
// max # of even allpass stage
#define IIR_MBDRC_EVEN_AP_STAGE 3
// max # of odd allpass stage
#define IIR_MBDRC_ODD_AP_STAGE 2

#define CAPI_IIR_MBDRC_ALIGN_TO_8(x) ((((uint32_t)(x) + 7) >> 3) << 3)
#define ALIGN8(o) (((o) + 7) & (~7))
#define CAPI_IIR_MBDRC_ALIGN_4_BYTE(x) (((x) + 3) & (0xFFFFFFFC))

// Limiter default values
static const int32 CAPI_IIR_MBDRC_LIM_THRESH_16BIT_VAL      = 93945856;
static const int32 CAPI_IIR_MBDRC_LIM_MAKEUP_GAIN_VAL       = 256;
static const int32 CAPI_IIR_MBDRC_LIM_GC_VAL                = 32440;
static const int32 CAPI_IIR_MBDRC_LIM_MAX_WAIT_VAL          = 82;
static const int32 CAPI_IIR_MBDRC_LIM_GAIN_ATTACK_VAL       = 188099735;
static const int32 CAPI_IIR_MBDRC_LIM_GAIN_RELEASE_VAL      = 32559427;
static const int32 CAPI_IIR_MBDRC_LIM_RELEASE_COEF_VAL      = 32768;
static const int32 CAPI_IIR_MBDRC_LIM_ATTACK_COEF_VAL       = 32768;
static const int32 CAPI_IIR_MBDRC_LIM_HARD_THRESH_16BIT_VAL = 93945856;
static const int32 CAPI_IIR_MBDRC_LIM_HISTORY_WINLEN_VAL    = 262;
static const int32 CAPI_IIR_MBDRC_LIM_DELAY_VAL             = 262;

static const int32 CAPI_IIR_MBDRC_KPPS_CH1_DW8_VAL          = 587860;
static const int32 CAPI_IIR_MBDRC_KPPS_CH2_DW8_VAL          = 607254;
static const int32 CAPI_IIR_MBDRC_KPPS_CH6_DW8_VAL          = 569251;
static const int32 CAPI_IIR_MBDRC_KPPS_CH8_DW8_VAL          = 539941;
static const int32 CAPI_IIR_MBDRC_KPPS_CH1_DW8_band_1_2_VAL = 503647;
static const int32 CAPI_IIR_MBDRC_KPPS_CH2_DW8_band_1_2_VAL = 508655;
static const int32 CAPI_IIR_MBDRC_KPPS_CH6_DW8_band_1_2_VAL = 455077;
static const int32 CAPI_IIR_MBDRC_KPPS_CH8_DW8_band_1_2_VAL = 445501;

static const int32 CAPI_IIR_MBDRC_KPPS_CH1_DW1_link1_VAL          = 865519;
static const int32 CAPI_IIR_MBDRC_KPPS_CH2_DW1_link1_VAL          = 740111;
static const int32 CAPI_IIR_MBDRC_KPPS_CH6_DW1_link1_VAL          = 609267;
static const int32 CAPI_IIR_MBDRC_KPPS_CH8_DW1_link1_VAL          = 591798;
static const int32 CAPI_IIR_MBDRC_KPPS_CH1_DW1_link1_band_1_2_VAL = 781348;
static const int32 CAPI_IIR_MBDRC_KPPS_CH2_DW1_link1_band_1_2_VAL = 653223;
static const int32 CAPI_IIR_MBDRC_KPPS_CH6_DW1_link1_band_1_2_VAL = 500676;
static const int32 CAPI_IIR_MBDRC_KPPS_CH8_DW1_link1_band_1_2_VAL = 480502;

static const int32 CAPI_IIR_MBDRC_KPPS_CH1_DW1_link0_VAL          = 865519;
static const int32 CAPI_IIR_MBDRC_KPPS_CH2_DW1_link0_VAL          = 854974;
static const int32 CAPI_IIR_MBDRC_KPPS_CH6_DW1_link0_VAL          = 826218;
static const int32 CAPI_IIR_MBDRC_KPPS_CH8_DW1_link0_VAL          = 843466;
static const int32 CAPI_IIR_MBDRC_KPPS_CH1_DW1_link0_band_1_2_VAL = 792501;
static const int32 CAPI_IIR_MBDRC_KPPS_CH2_DW1_link0_band_1_2_VAL = 792973;
static const int32 CAPI_IIR_MBDRC_KPPS_CH6_DW1_link0_band_1_2_VAL = 738497;
static const int32 CAPI_IIR_MBDRC_KPPS_CH8_DW1_link0_band_1_2_VAL = 758090;

static const uint32 CAPI_IIR_MBDRC_limiter_threshold_min = 1; // 16-bit - -96db , 24-bit - -162db , 32-bit - -162db
static const uint32 CAPI_IIR_MBDRC_limiter_threshold_32bit_max = 2127207634; // 24-bit - 24db , 32-bit - 0db

/*------------------------------------------------------------------------
 * Structure definitions
 * -----------------------------------------------------------------------*/
typedef struct capi_iir_mbdrc_common_config_struct_t
{
   iir_mbdrc_num_band_t       iir_mbdrc_num_band;
   iir_mbdrc_limiter_mode_t   iir_mbdrc_limiter_mode;
   iir_mbdrc_limiter_bypass_t iir_mbdrc_limiter_bypass;
   uint32                     version;
} capi_iir_mbdrc_common_config_struct_t;

typedef struct capi_iir_mbdrc_crossover_freqs_t
{
   uint32 ch_idx;                                   // channel index
   uint32 crossover_freqs[IIR_MBDRC_MAX_BANDS - 1]; // cross-over frequencies;
} capi_iir_mbdrc_crossover_freqs_t;

typedef struct capi_iir_mbdrc_per_config_crossover_freqs_struct_t
{
   uint32_t                         channel_mask_lsb;
   uint32_t                         channel_mask_msb;
   capi_iir_mbdrc_crossover_freqs_t iir_mbdrc_crossover_freqs;
} capi_iir_mbdrc_per_config_crossover_freqs_struct_t;

typedef struct capi_iir_mbdrc_config_struct_t
{
   iir_mbdrc_mute_flags_t iir_mbdrc_mute_flags;
   iir_mbdrc_drc_feature_mode_struct_t
      *                           iir_mbdrc_drc_feature_mode_struct; // iir_mbdrc_drc_feature_mode_struct[num_bands]
   iir_mbdrc_drc_config_struct_t *iir_mbdrc_drc_cfg;                 // iir_mbdrc_drc_cfg[num_bands]
   iir_mbdrc_limiter_tuning_t     iir_mbdrc_limiter_cfg;
   iir_mbdrc_iir_config_struct_t *iir_mbdrc_iir_cfg; // iir_mbdrc_iir_cfg[num_bands-1]
} capi_iir_mbdrc_config_struct_t;

typedef struct capi_iir_mbdrc_per_config_struct_t
{
   uint32_t channel_mask_lsb;                       // Contains Channel type info
                                                    // Least significant bit = 0 => per channel
                                                    // calibration disable Least significant bit = 1 =>
                                                    // per channel calibration enable
   uint32_t                       channel_mask_msb; // Contains Channel type info
   capi_iir_mbdrc_config_struct_t iir_mbdrc_cfg;    // Has calibration for the above mapped channels.

} capi_iir_mbdrc_per_config_struct_t;

typedef struct capi_iir_mbdrc_per_config_t
{
   uint32_t                                            num_config;
   uint32_t                                            num_crossover_freqs_config;
   capi_iir_mbdrc_per_config_struct_t *                iir_mbdrc_per_cfg; // iir_mbdrc_per_cfg[num_config]
   capi_iir_mbdrc_per_config_crossover_freqs_struct_t *iir_mbdrc_crossover_freqs_per_cfg;
   // iir_mbdrc_crossover_freqs_per_cfg[num_crossover_freqs_config]
} capi_iir_mbdrc_per_config_t;

typedef struct capi_iir_mbdrc_per_config_crossover_freqs_struct_v2_t
{
   uint32_t                         channel_map_mask_list[CAPI_CMN_MAX_CHANNEL_MAP_GROUPS];
   capi_iir_mbdrc_crossover_freqs_t iir_mbdrc_crossover_freqs;
} capi_iir_mbdrc_per_config_crossover_freqs_struct_v2_t;

typedef struct capi_iir_mbdrc_per_config_struct_v2_t
{
   uint32_t                       channel_map_mask_list[CAPI_CMN_MAX_CHANNEL_MAP_GROUPS];
   capi_iir_mbdrc_config_struct_t iir_mbdrc_cfg; // Has calibration for the above mapped channels.
} capi_iir_mbdrc_per_config_struct_v2_t;

typedef struct capi_iir_mbdrc_per_config_v2_t
{
   uint32_t                                               num_config;
   uint32_t                                               num_crossover_freqs_config;
   capi_iir_mbdrc_per_config_struct_v2_t *                iir_mbdrc_per_cfg; // iir_mbdrc_per_cfg[num_config]
   capi_iir_mbdrc_per_config_crossover_freqs_struct_v2_t *iir_mbdrc_crossover_freqs_per_cfg;
   uint32_t                                               cached_xover_freqs_v2_param_size;
   uint32_t                                               cached_mbdrc_config_v2_param_size;
} capi_iir_mbdrc_per_config_v2_t;

typedef struct capi_iir_mbdrc_memory_struct_t
{
   iir_mbdrc_lib_mem_requirements_t mem_req;  // MBDRC memory requirements structure
   void *                           mem_ptr;  // MBDRC memory pointer
   capi_heap_id_t                   heap_mem; // Heap ID to be used for memory allocation
} iir_mbdrc_memory_struct_t;

typedef struct capi_iir_mbdrc_event_report_t
{
   capi_event_callback_info_t cb_info;
   uint32_t                   iir_mbdrc_bw;
   uint32_t                   iir_mbdrc_kpps;
   uint32_t                   iir_mbdrc_delay_in_us;
   bool_t                     iir_mbdrc_enable;
} iir_mbdrc_event_report_t;

typedef enum capi_iir_mbdrc_config_version_t
{
   DEFAULT    = 0,
   VERSION_V1 = 1,
   VERSION_V2 = 2
} capi_iir_mbdrc_config_version_t;

#define CAPI_IIR_MBDRC_MF_V2_MIN_SIZE (sizeof(capi_standard_data_format_v2_t) + sizeof(capi_set_get_media_format_t))

typedef struct capi_iir_mbdrc_t
{
   capi_vtbl_t *                  vtbl;
   capi_media_fmt_v2_t            media_fmt[CAPI_IIR_MBDRC_MAX_PORT];
   capi_iir_mbdrc_event_report_t  iir_mbdrc_event_report;
   capi_iir_mbdrc_memory_struct_t iir_mbdrc_memory;              // contains memory information
   iir_mbdrc_lib_t                iir_mbdrc_lib;                 // contains ptr to the total chunk of lib mem
   iir_mbdrc_static_struct_t      iir_mbdrc_static_cfg;          // Stores static config information from media
                                                                 // format and calibration received during Set param
   iir_mbdrc_static_struct_t iir_mbdrc_static_cfg_of_defult_cfg; // Stores static config information
                                                                 // from media format and default
                                                                 // calibration
   iir_mbdrc_feature_mode_t iir_mbdrc_feature_mode;              // Stores MBDRC mode
   bool_t                   is_iir_mbdrc_set_cfg_param_received; // This flag is set to 1 if Set
                                                                 // param is received or else it is
                                                                 // set to 0
   bool_t is_iir_mbdrc_media_fmt_received;                       // This flag is set to 1 if Media format
                                                                 // is received or else it is set to 0
   bool_t is_iir_mbdrc_library_set_with_default_cfg;             // This flag is set to 1 if
                                                                 // library is set with default
                                                                 // config, due to non
                                                                 // availability of calibration
                                                                 // for all the channel type
                                                                 // received through Media
                                                                 // format in Calibration
                                                                 // received through Set Param
   iir_mbdrc_per_channel_calib_mode_t iir_mbdrc_per_channel_calib_mode; // Stores information if per-channel
                                                                        // calibration is enabled or disabled
   capi_iir_mbdrc_common_config_struct_t iir_mbdrc_common_cfg;
   capi_iir_mbdrc_per_config_t           iir_mbdrc_main_cfg; // Stores the calibration received through set param
   capi_iir_mbdrc_per_config_t
      iir_mbdrc_default_cfg; // Stores the default calibratio   capi_iir_mbdrc_per_config_t iir_mbdrc_main_cfg;    //
                             // Stores the calibration received through set param
   capi_iir_mbdrc_per_config_v2_t iir_mbdrc_main_cfg_v2;    // Stores the calibration received through set param
   capi_iir_mbdrc_per_config_v2_t iir_mbdrc_default_cfg_v2; // Stores the calibration received through set param
   uint32_t cached_mbdrc_config_v2_param_size;
   uint32_t cached_mbdrc_xover_freqs_v2_param_size;
   uint32_t miid;                // Module Instance ID
   uint64_t input_mf_ch_mask_v1; // Input channel mask
   // Input channel mask for v2 config
   uint32_t input_mf_ch_mask_list_v2[CAPI_CMN_MAX_CHANNEL_MAP_GROUPS];
   // flag to detect if higher than 63 channel map received or not
   bool_t   higher_channel_map_present;
   
   capi_iir_mbdrc_config_version_t cfg_version;

} capi_iir_mbdrc_t;

/*------------------------------------------------------------------------
 * Function declarations
 * -----------------------------------------------------------------------*/
// Utility functions

// Functions to be called from init function.
void       capi_iir_mbdrc_init_media_fmt(capi_iir_mbdrc_t *const me_ptr);
void       capi_iir_mbdrc_init_events(capi_iir_mbdrc_t *const me_ptr);
capi_err_t capi_iir_mbdrc_init_config(capi_iir_mbdrc_t *const me_ptr);

// Set and Get property functions.
capi_err_t capi_iir_mbdrc_process_set_properties(capi_iir_mbdrc_t *me_ptr, capi_proplist_t *proplist_ptr);
capi_err_t capi_iir_mbdrc_process_get_properties(capi_iir_mbdrc_t *me_ptr, capi_proplist_t *proplist_ptr);

// Set and Get Config params functions.
capi_err_t capi_iir_mbdrc_set_config_params(capi_iir_mbdrc_t *me_ptr, const uint32_t param_id, capi_buf_t *params_ptr);
capi_err_t capi_iir_mbdrc_get_config_params(capi_iir_mbdrc_t *me_ptr, const uint32_t param_id, capi_buf_t *params_ptr);

// Function to raise the events
void       capi_iir_mbdrc_raise_events(capi_iir_mbdrc_t *const me_ptr, bool_t media_fmt_update);
capi_err_t capi_iir_mbdrc_check_channel_map_cfg(iir_mbdrc_per_ch_filter_xover_freqs_t *cfg_ptr, uint32_t num_config);
capi_err_t capi_iir_mbdrc_init_default_xover_config(capi_iir_mbdrc_t *me_ptr);
uint64_t   capi_iir_mbdrc_calculate_channel_mask(uint32_t channel_mask_lsb, uint32_t channel_mask_msb);
iir_mbdrc_per_channel_calib_mode_t capi_iir_mbdrc_configure_per_channel_calib_mode(
   capi_iir_mbdrc_t *me_ptr,
   uint64_t          channel_map_from_set_config);

capi_err_t capi_iir_mbdrc_allocate_lib_memory(capi_iir_mbdrc_t *me_ptr,
                                              bool_t            force_reinit,
                                              bool_t            use_default_static_cfg);

capi_err_t capi_iir_mbdrc_set_xover_freq_v2_params(capi_iir_mbdrc_t *me_ptr,
                                                   const uint32_t    param_id,
                                                   capi_buf_t *      params_ptr);

capi_err_t capi_iir_mbdrc_validate_multichannel_v2_payload(capi_iir_mbdrc_t *me_ptr,
                                                           int8_t *          data_ptr,
                                                           uint32_t          param_id,
                                                           uint32_t          param_size,
                                                           uint32_t          num_cfg,
                                                           uint32_t *        req_payload_size,
                                                           uint32_t          base_payload_size,
                                                           uint32_t          per_cfg_base_payload_size);

capi_err_t capi_iir_mbdrc_validate_per_channel_v2_payload(capi_iir_mbdrc_t *me_ptr,
                                                          uint32_t          num_cfg,
                                                          int8_t *          data_ptr,
                                                          uint32_t          param_size,
                                                          uint32_t *        required_size_ptr,
                                                          uint32_t          param_id,
                                                          uint32_t          base_payload_size,
                                                          uint32_t          per_cfg_base_payload_size);

bool_t capi_iir_mbdrc_check_multi_ch_channel_mask_v2_param(uint32_t miid,
                                                           uint32_t num_config,
                                                           uint32_t param_id,
                                                           int8_t * param_ptr,
                                                           uint32_t base_payload_size,
                                                           uint32_t per_cfg_base_payload_size);

capi_err_t capi_iir_mbdrc_set_config_v2_params(capi_iir_mbdrc_t *me_ptr,
                                               const uint32_t    param_id,
                                               capi_buf_t *      params_ptr);

iir_mbdrc_per_channel_calib_mode_t capi_iir_mbdrc_configure_per_channel_calib_mode_v2(capi_iir_mbdrc_t *me_ptr,
                                                                                      uint32_t *channel_mask_list,
                                                                                      uint32_t  channel_group_mask);

bool_t capi_iir_mbdrc_check_if_non_vol_cal_changed_v2(
   capi_iir_mbdrc_t *                 me_ptr,
   capi_buf_t *                       params_ptr,
   int32_t *                          memory_needs_to_change,
   iir_mbdrc_per_channel_calib_mode_t iir_mbdrc_per_channel_calib_mode);

void capi_iir_mbdrc_get_channel_mask_list_v2(uint32_t  temp_channel_mask_list[],
                                             uint32_t *channel_mask_list,
                                             uint32_t  channel_group_mask);

capi_err_t capi_iir_mbdrc_get_xover_freq_v2_params(capi_iir_mbdrc_t *me_ptr,
                                                   const uint32_t    param_id,
                                                   capi_buf_t *      params_ptr);

capi_err_t capi_iir_mbdrc_init_default_xover_config_v2(capi_iir_mbdrc_t *me_ptr);

capi_err_t capi_iir_mbdrc_init_default_config_v2(capi_iir_mbdrc_t *const me_ptr,
                                                 bool_t                  cache_defaults_to_iir_mbdrc_main_cfg);

void capi_iir_mbdrc_get_calib_mode_for_v2_cfg(capi_iir_mbdrc_t *me_ptr);

void capi_iir_mbdrc_check_allocate_and_initialize_lib_v2_cfg(capi_iir_mbdrc_t *me_ptr,
                                                             uint32_t          num_cfg,
                                                             uint32_t          channels_allocated,
                                                             uint32_t          *matched_ptr);

capi_err_t capi_iir_mbdrc_init_lib_v1(capi_iir_mbdrc_t *me_ptr, uint32_t matched);

capi_err_t capi_iir_mbdrc_init_lib_v2(capi_iir_mbdrc_t *me_ptr, uint32_t matched);

void capi_iir_mbdrc_post_process_lib_initialization_v1(capi_iir_mbdrc_t *me_ptr);

void capi_iir_mbdrc_post_process_lib_initialization_v2(capi_iir_mbdrc_t *me_ptr);

capi_err_t capi_iir_mbdrc_set_params_to_library_v2(capi_iir_mbdrc_t *me_ptr, bool_t use_default_static_cfg);

void capi_iir_mbdrc_update_drc_flags_and_ch_mask_v2(capi_iir_mbdrc_t *me_ptr,
                                                    const uint32_t    num_bands,
                                                    bool_t *          downsample_level_flag_1,
                                                    bool_t *          drc_linked_flag,
                                                    const uint32_t    num_channel);

void capi_iir_mbdrc_check_allocate_and_initialize_lib_v2_cfg(capi_iir_mbdrc_t *me_ptr,
                                                             uint32_t          channels_allocated,
                                                             uint32_t *        matched);

capi_err_t capi_iir_mbdrc_get_config_params_v2(capi_iir_mbdrc_t *const me_ptr,
                                               uint32_t param_id,
                                               capi_buf_t *params_ptr);

uint32_t  capi_iir_mbdrc_calculate_cached_size_for_default_v2_param(uint32_t  num_bands, 
                                                                    uint32_t  num_config);
#endif
