#ifndef CAPI_MULTISTAGEIIR_UTILS_H
#define CAPI_MULTISTAGEIIR_UTILS_H
/* ======================================================================== */
/**
@file capi_multistageiir_utils.h

   Header file to implement the CAPI Interface for Multi-Stage IIR filter
   object (MSIIR).
*/

/* =========================================================================
   * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   * SPDX-License-Identifier: BSD-3-Clause-Clear
  ========================================================================= */

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/


#ifndef CAPI_STANDALONE
#include "shared_lib_api.h"
#else
#include "module_cmn_api.h"
#endif

/* Only internal dependencies should be present here, others should be covered in shared_lib_api.h */
#include "capi_multistageiir.h"
#include "msiir_api.h"
#include "api_msiir.h"
#include "crossfade_api.h"

#include "capi_cmn.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef PROD_SPECIFIC_MAX_CH
#define MSIIR_MAX_CHANNEL PROD_SPECIFIC_MAX_CH
#else
#define MSIIR_MAX_CHANNEL 32
#endif

#define CAPI_MSIIR_DEBUG_MSG 0
#define DEFAULT_FRAME_SIZE_IN_US                 5000

#define NUM_PACKETS_PER_SAMPLE_16BIT                 8
#define NUM_PACKETS_PER_STAGE_WITH_ZERO_SAMPLE_16BIT 19
#define NUM_PACKETS_WITH_ZERO_STAGE_16BIT            220
#define NUM_PACKETS_PER_SAMPLE_32BIT                 7
#define NUM_PACKETS_PER_STAGE_WITH_ZERO_SAMPLE_32BIT 17
#define NUM_PACKETS_WITH_ZERO_STAGE_32BIT            220

#define CAPI_MSIIR_ALIGN_4_BYTE(x) (((x) + 3) & (0xFFFFFFFC))

/*------------------------------------------------------------------------
 * Static constants
 * -----------------------------------------------------------------------*/
//#define CAPI_MSIIR_DEBUG_MSG
static const uint32_t MSIIR_MAX_STAGES            = 20;
static const uint32_t MSIIR_FILTER_STATES         = 2;

static const uint32_t MSIIR_CURRENT_INSTANCE      = 0;
static const uint32_t MSIIR_NEW_INSTANCE          = 1;

static const uint32_t MSIIR_Q_PREGAIN_LEGACY      = 13;
static const uint32_t MSIIR_DEFAULT_SAMPLE_RATE   = 48000;

typedef enum
{
    MULTISTAGE_IIR_LEFT_PAN=0,
    MULTISTAGE_IIR_CENTER_PAN,
    MULTISTAGE_IIR_RIGHT_PAN,
} MultiStageIIRPan;


typedef enum
{
    MULTISTAGE_IIR_INVALID_PARAM=0,
    MULTISTAGE_IIR_MCHAN_PARAM,
} MultiStageIIRParamType;
/*------------------------------------------------------------------------
 * Macros, Defines, Type declarations
 * -----------------------------------------------------------------------*/

/* debug message */
#define MIID_UNKNOWN 0
#define MSIIR_MSG_PREFIX "CAPI MULTISTAGE IIR:[%lX] "
#define MSIIR_MSG(ID, xx_ss_mask, xx_fmt, ...)\
           AR_MSG(xx_ss_mask, MSIIR_MSG_PREFIX xx_fmt, ID, ##__VA_ARGS__)

typedef struct capi_one_chan_msiir_config_max_t
{
    int32_t          num_stages;
    msiir_coeffs_t   coeffs_struct[MSIIR_MAX_STAGES];
} capi_one_chan_msiir_config_max_t;

typedef struct capi_cached_params_t
{
    uint32_t         param_id_type;
    capi_buf_t    params_ptr;
} capi_cached_params_t;

typedef enum capi_msiir_config_version_t
{
   DEFAULT    = 0,
   VERSION_V1 = 1,
   VERSION_V2 = 2
} capi_msiir_config_version_t;

typedef struct capi_one_chan_msiir_config_static_param_t
{
   uint16_t reserved;
   uint16_t num_biquad_stages;
} capi_one_chan_msiir_config_static_param_t;

typedef struct capi_multistageiir_t
{
    // capi
    const capi_vtbl_t                  *vtbl;               // capi function table pointer
    capi_event_callback_info_t         cb_info;
    uint32_t                              heap_id;
    uint32_t                              kpps;
    uint32_t                              bw;
    uint32_t                              delay;
    //media format
    capi_media_fmt_v2_t                media_fmt[CAPI_MSIIR_MAX_OUT_PORTS];
    int32_t                               channel_map_to_index[PCM_MAX_CHANNEL_MAP_V2+1]; //converts channel_type to index, +1 to array size so that channel_type can be used directly to index into this array
    // core library
    msiir_lib_t                           msiir_lib[IIR_TUNING_FILTER_MAX_CHANNELS_V2];
    msiir_lib_t                           msiir_new_lib[IIR_TUNING_FILTER_MAX_CHANNELS_V2];
    msiir_static_vars_t                   msiir_static_vars;
    msiir_mem_req_t                       per_chan_mem_req;
    uint32_t                              enable_flag[IIR_TUNING_FILTER_MAX_CHANNELS_V2];
    uint32_t                              num_channels_allocated;
    // cross fade
    cross_fade_lib_t                      cross_fade_lib[IIR_TUNING_FILTER_MAX_CHANNELS_V2];
    cross_fade_static_t                   cross_fade_static_vars;
    cross_fade_lib_mem_req_t              per_chan_cross_fade_mem_req;
    uint32_t                              cross_fade_flag[IIR_TUNING_FILTER_MAX_CHANNELS_V2];
    // config params
    msiir_pregain_t                       per_chan_msiir_pregain[IIR_TUNING_FILTER_MAX_CHANNELS_V2];
    capi_one_chan_msiir_config_max_t   *per_chan_msiir_cfg_max;
    capi_one_chan_msiir_config_max_t   default_per_chan_msiir_cfg_max;
    // flags
    bool_t                                is_first_frame;
    bool_t                                media_fmt_received;
    bool_t                                start_cross_fade;
    bool_t                                enable; // specifies the state of the module
    //This is used for storing Capi Module instance id
    uint32_t                              miid;
    //this flag is to detect if higher than 63 channel maps received in media format
    bool_t                                higher_channel_map_present;
    // cache params
    capi_cached_params_t               enable_params;
    capi_cached_params_t               pregain_params;
    capi_cached_params_t               config_params;
    uint32_t                           cntr_frame_size_us;
    capi_msiir_config_version_t        cfg_version;
} capi_multistageiir_t;

bool_t check_channel_mask_msiir(uint8_t *iir_param_ptr,uint32_t param_id) ;

capi_err_t capi_msiir_update_delay_event(capi_multistageiir_t *me);

capi_err_t capi_msiir_set_enable_disable_per_channel(capi_multistageiir_t *me,
                                                           const capi_buf_t *    params_ptr);

capi_err_t capi_msiir_set_pregain_per_channel(capi_multistageiir_t *me, const capi_buf_t *params_ptr);

capi_err_t capi_msiir_set_config_per_channel(
        capi_multistageiir_t   *me,
        const capi_buf_t       *params_ptr);

capi_err_t capi_msiir_get_enable_disable_per_channel(
        capi_multistageiir_t   *me,
        capi_buf_t             *params_ptr);

capi_err_t capi_msiir_get_pregain_per_channel(
        capi_multistageiir_t   *me,
        capi_buf_t             *params_ptr);

capi_err_t capi_msiir_get_config_per_channel(
        capi_multistageiir_t   *me,
        capi_buf_t             *params_ptr);

capi_err_t capi_msiir_get_default_pregain_config(
        capi_multistageiir_t   *me,
        uint32_t ch);

capi_err_t capi_msiir_start_cross_fade(
        capi_multistageiir_t   *me);

capi_err_t capi_msiir_process_set_properties(
        capi_multistageiir_t   *me,
        capi_proplist_t        *props_ptr);

capi_err_t capi_msiir_process_get_properties(
        capi_multistageiir_t   *me,
        capi_proplist_t        *props_ptr);

capi_err_t capi_msiir_init_media_fmt(
        capi_multistageiir_t   *me);

capi_err_t capi_msiir_cache_params(
        capi_multistageiir_t   *me,
        capi_buf_t             *params_ptr,
        uint32_t                  param_id);

capi_err_t capi_msiir_check_raise_kpps_event(
        capi_multistageiir_t   *me,
        uint32_t                  val);

uint32_t capi_msiir_get_kpps(
        capi_multistageiir_t   *me);

int32_t capi_msiir_s32_min_s32_s32(
        int32_t                   x,
        int32_t                   y);

capi_err_t capi_msiir_raise_process_check_event(capi_multistageiir_t *me_ptr);

capi_err_t capi_msiir_set_enable_disable_per_channel_v2(capi_multistageiir_t *me_ptr,
                                                       const capi_buf_t      *params_ptr,
                                                       uint32_t               param_id);

capi_err_t capi_msiir_validate_multichannel_v2_payload(capi_multistageiir_t *me_ptr,
                                                      int8_t *               data_ptr,
                                                      uint32_t               param_id,
                                                      uint32_t               param_size,
                                                      uint32_t *             req_payload_size,
                                                      uint32_t               base_payload_size,
                                                      uint32_t               per_cfg_base_payload_size);

capi_err_t capi_msiir_validate_per_channel_v2_payload(capi_multistageiir_t *me_ptr,
                                                     uint32_t               num_cfg,
                                                     int8_t *               payload_cfg_ptr,
                                                     uint32_t               param_size,
                                                     uint32_t *             required_size_ptr,
                                                     uint32_t               param_id,
                                                     uint32_t               base_payload_size,
                                                     uint32_t               per_cfg_base_payload_size);

capi_err_t capi_msiir_set_enable_disable_v2_payload(capi_multistageiir_t *me_ptr,
                                                   int8_t*                data_ptr,
                                                   uint32_t               per_cfg_base_payload_size);

capi_err_t capi_msiir_set_pregain_per_channel_v2(capi_multistageiir_t *me_ptr,
                                                 const capi_buf_t     *params_ptr,
                                                 uint32_t              param_id);;

capi_err_t capi_msiir_set_pregain_v2_payload(capi_multistageiir_t *me_ptr,
                                             int8_t*               data_ptr,
                                             uint32_t              per_cfg_base_payload_size);

capi_err_t capi_msiir_set_config_per_channel_v2(capi_multistageiir_t *me_ptr,
                                                    const capi_buf_t *params_ptr,
                                                    uint32_t          param_id);

capi_err_t capi_msiir_set_config_v2_payload(capi_multistageiir_t *me_ptr,
                                            int8_t*               payload_ptr);

capi_err_t capi_msiir_get_enable_disable_per_channel_v2(capi_multistageiir_t   *me_ptr,
                                                        capi_buf_t             *params_ptr);

capi_err_t capi_msiir_get_pregain_per_channel_v2(
        capi_multistageiir_t   *me_ptr,
        capi_buf_t             *params_ptr);

capi_err_t capi_msiir_get_config_per_channel_v2(
        capi_multistageiir_t   *me_ptr,
        capi_buf_t             *params_ptr);

bool_t capi_msiir_check_multi_ch_channel_mask_v2_param(uint32_t miid,
                                            uint32_t    num_config,
                                            uint32_t    param_id,
                                            int8_t *    param_ptr,
                                            uint32_t    base_payload_size,
                                            uint32_t    per_cfg_base_payload_size);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif //__CAPI_MULTISTAGEIIR_UTILS_H

