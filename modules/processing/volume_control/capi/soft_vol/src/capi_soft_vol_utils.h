#ifndef CAPI_SOFT_VOL_UTILS_H
#define CAPI_SOFT_VOL_UTILS_H
/* =========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 * @file capi_soft_vol_utils.h
 *
 * Utility header for soft_vol module.
 */

/*=====================================================================
  Includes
 ======================================================================*/
#include "capi.h"
#include "SoftVolumeControls.h"
#include "soft_vol_api.h"
#include "posal.h"
#include "capi_cmn.h"

//For imcl to AVC Tx in voice path
#include "capi_cmn_ctrl_port_list.h"
#include "imcl_module_gain_api.h"
#include "imcl_p_eq_vol_api.h"
#include "imcl_mute_api.h"

#define CHECK_THROW_ERROR(result, error_msg, ...)                                                                      \
   {                                                                                                                   \
      if (CAPI_FAILED(result))                                                                                      \
      {                                                                                                                \
         AR_MSG(DBG_ERROR_PRIO, error_msg, ##__VA_ARGS__);                                                             \
         return result;                                                                                                \
      }                                                                                                                \
   \
}

/*=====================================================================
  Macros
 ======================================================================*/
#define MAX_PERIOD 15000
#define MIN_PERIOD 0
#define MAX_STEP 15000000
#define MIN_STEP 0
#define MULT_CH_GAIN_BASE_PYLD_SIZE sizeof(volume_ctrl_channels_gain_config_v2_t)
#define MULT_CH_MUTE_BASE_PYLD_SIZE sizeof(volume_ctrl_channels_mute_config_v2_t)

#ifdef PROD_SPECIFIC_MAX_CH
#define VOLUME_CONTROL_MAX_CHANNELS_V2 128
#else
#define VOLUME_CONTROL_MAX_CHANNELS_V2 32
#endif

#define HEADROOM_GAIN_Q28_MIN 4254
#define HEADROOM_GAIN_Q28_MAX 268435456

#define SOFT_VOL_MASK_WIDTH 32
#define SOFT_VOL_MAX_CHANNEL_TYPE PCM_MAX_CHANNEL_MAP_V2

const uint32_t UNITY_GAIN_Q28 = 1 << 28;
const uint16_t UNITY_GAIN_Q13 = 1 << 13;

const uint16_t MIN_CHANNEL_TYPE = PCM_CHANNEL_L;
const uint16_t MAX_CHANNEL_TYPE = PCM_MAX_CHANNEL_MAP_V2;
#define SOFT_VOL_DEBUG 0
/* debug message */
#define MIID_UNKNOWN 0
#define SOFT_VOL_MSG_PREFIX "CAPI SOFT_VOL:[%lX] "
#define SOFT_VOL_MSG(ID, xx_ss_mask, xx_fmt, ...)\
         AR_MSG(xx_ss_mask, SOFT_VOL_MSG_PREFIX xx_fmt, ID, ##__VA_ARGS__)

#ifdef __qdsp6__
#define s64_mult_fp_s32_s16_shift(var1, var2, shift) Q6_P_asl_PR(Q6_P_mpy_RR(var1, (int32)var2), shift - 16)
#else
#define s64_mult_fp_s32_s16_shift(var1, var2, shift) s64_mult_s32_s16_shift(var1, var2, shift)
#endif

#define CAPI_SOFT_VOL_ALIGN_4_BYTE(x) (((x) + 3) & (0xFFFFFFFC))

/*=====================================================================
  Structure definitions
 ======================================================================*/
typedef enum {
   SOFT_VOL_MIN_IN_CHANNELS  = 1,
   SOFT_VOL_MIN_OUT_CHANNELS = 1,
#ifdef PROD_SPECIFIC_MAX_CH
   SOFT_VOL_MAX_IN_CHANNELS  = PROD_SPECIFIC_MAX_CH,
   SOFT_VOL_MAX_OUT_CHANNELS = PROD_SPECIFIC_MAX_CH,
#else
   SOFT_VOL_MAX_IN_CHANNELS  = 32,
   SOFT_VOL_MAX_OUT_CHANNELS = 32,
#endif
} soft_vol_ch_t;

typedef enum { SOFT_VOL_WAITING_FOR_MEDIA_FORMAT = -1, SOFT_VOL_DISABLE, SOFT_VOL_ENABLE } soft_vol_state_t;

#ifdef DO_SOFT_VOL_PROFILING
typedef struct soft_vol_prof_info
{
   uint32_t sample_rate;
   uint64_t start_cycles;
   uint64_t end_cycles;
   uint64_t total_cycles;
   uint64_t sample_count;
   uint64_t average_kpps;
   uint64_t peak_kpps;
} soft_vol_prof_info_t;
#endif

#define CAPI_SOFT_VOL_MF_V2_MIN_SIZE                                                                                \
   (sizeof(capi_standard_data_format_v2_t) + sizeof(capi_set_get_media_format_t))

typedef struct capi_soft_vol_power_info
{
   uint32_t kpps;
   uint32_t code_bw;
   uint32_t data_bw;
} capi_soft_vol_power_info_t;

typedef struct capi_soft_vol_multich_gain_info
{
   uint32_t gain;
   uint32_t group_ch_mask;
   uint32_t multch_gain_mask_list[CAPI_CMN_MAX_CHANNEL_MAP_GROUPS];
} capi_soft_vol_multich_gain_info;

typedef struct soft_vol_lib_struct
{
   uint16_t masterGain;

   /* +1 to be able to index the array directly by channel type. */
   uint32_t channelGain[MAX_CHANNEL_TYPE + 1];
   void *   pPerChannelData[MAX_CHANNEL_TYPE + 1];

   int32_t  headroom_mB;
   int32_t  topoReqHeadroom;
   uint32_t numChannels;
   uint16_t channelMapping[VOLUME_CONTROL_MAX_CHANNELS_V2];

   bool_t softPauseEnable;
} soft_vol_lib_struct_t;

typedef struct capi_soft_vol
{
   const capi_vtbl_t *        vtbl_ptr;
   uint32_t                      heap_id;
   capi_event_callback_info_t cb_info;
   capi_media_fmt_v2_t        input_media_fmt;
   capi_media_fmt_v2_t        output_media_fmt;
   capi_soft_vol_power_info_t power_info;
   soft_vol_lib_struct_t         soft_vol_lib;
   CSoftVolumeControlsLib        SoftVolumeControlsLib;
   soft_vol_state_t              soft_vol_state;

   //for imcl to AVC-Tx
   ctrl_port_list_handle_t       ctrl_port_info;
   bool_t                        update_gain_over_imcl;
   int8_t                        adjust_volume_based_on_headroom;
#ifdef DO_SOFT_VOL_PROFILING
   soft_vol_prof_info_t soft_vol_kpps_data;
#endif
   uint32_t                      miid;
   //this flag is to detect if greater than 63 channels received in media format
   bool_t                        higher_channel_map_present;
} capi_soft_vol_t;

capi_err_t capi_vol_imc_set_param_handler(capi_soft_vol *me_ptr, capi_buf_t *intent_buf_ptr);

capi_err_t capi_set_default_ramp_params(capi_soft_vol_t *me_ptr);

capi_err_t capi_soft_vol_process_set_properties(capi_soft_vol_t *me_ptr, capi_proplist_t *proplist_ptr);

capi_err_t capi_soft_vol_process_get_properties(capi_soft_vol_t *me_ptr, capi_proplist_t *proplist_ptr);

void capi_soft_vol_profiling(capi_soft_vol_t *me_ptr, uint32_t num_samples);

void capi_set_soft_stepping_param(const capi_soft_vol_t *me_ptr,
                          SoftSteppingParams *      pParams,
                          uint32_t                  period,
                          uint32_t                  step,
                          uint32_t                  rampingCurve);

void capi_get_soft_stepping_param(const capi_soft_vol_t *me_ptr,
                          const SoftSteppingParams *pParams,
                          uint32_t *                pPeriod,
                          uint32_t *                pStep,
                          uint32_t *                pRampingCurve);

bool_t check_channel_mask_softvol(uint8_t *iir_param_ptr,uint32_t param_id) ;

capi_err_t capi_soft_vol_set_multichannel_gain(capi_soft_vol_t *             me_ptr,
                                                     volume_ctrl_multichannel_gain_t *pVolumePkt);

bool_t check_unique_gain(uint32_t *gain_list, uint32_t gain, uint32_t *num_unique_gains_ptr, uint32_t *position_ptr);

uint32_t capi_soft_vol_get_multichannel_gain(capi_soft_vol_t *me_ptr, volume_ctrl_multichannel_gain_t *pVolumePkt);

capi_err_t capi_soft_vol_set_multichannel_mute(capi_soft_vol_t *             me_ptr,
                                                     volume_ctrl_multichannel_mute_t *pMutePkt);

void capi_soft_vol_get_multichannel_mute(capi_soft_vol_t *me_ptr, volume_ctrl_multichannel_mute_t *pMutePkt);

void capi_soft_vol_set_mute_for_all_channels(capi_soft_vol_t *me_ptr, const uint32_t mute_flag);

void capi_soft_vol_reinit_new_channels(capi_soft_vol_t *me_ptr,
                                          const uint32_t      numNewChannels,
                                          const uint8_t       newMapping[]);

uint32 capi_soft_vol_calc_gain_q28(const uint16_t masterGainQ13,const uint32_t channelGainQ28);

void capi_soft_vol_set_sample_rate(capi_soft_vol_t *me_ptr, const uint32_t sample_rate);

int32 capi_soft_vol_convert_headroom_linear_gain(const int32 reqHeadroom);

capi_err_t capi_soft_vol_headroom_gain(capi_soft_vol_t *me_ptr, const int32_t headroom_mB);

capi_err_t capi_soft_vol_init_lib_memory(capi_soft_vol_t *me_ptr);

void capi_soft_vol_send_gain_over_imcl(capi_soft_vol_t *me_ptr);

capi_err_t capi_soft_vol_validate_multichannel_payload(capi_soft_vol_t *me_ptr,
                                                       uint32_t        num_cfg,
                                                       int8_t          *payload_cfg_ptr,
                                                       uint32_t        param_size,
                                                       uint32_t        *required_size_ptr,
                                                       uint32_t        param_id);

capi_err_t capi_soft_vol_set_multichannel_gain_v2(capi_soft_vol_t                    *me_ptr,
                                                  volume_ctrl_multichannel_gain_v2_t *vol_payload_ptr);

capi_err_t capi_soft_vol_set_multichannel_mute_v2(capi_soft_vol_t                    *me_ptr,
                                                  volume_ctrl_multichannel_mute_v2_t *mute_payload_ptr);

void capi_soft_vol_get_multichannel_mute_v2(capi_soft_vol_t *me_ptr,
                                            int8_t          *get_payload_ptr,
                                            uint32_t        *mute_chamask_list_ptr);

uint32_t capi_soft_vol_get_multichannel_gain_v2_payload_size(capi_soft_vol_t                 *me_ptr,
                                                             capi_soft_vol_multich_gain_info *gain_list,
                                                             uint32_t                        *req_size_ptr );

capi_err_t capi_soft_vol_get_multichannel_gain_v2(capi_soft_vol_t                 *me_ptr,
                                                  int8_t                          *get_payload_ptr,
                                                  capi_soft_vol_multich_gain_info *gain_list,
                                                  uint32_t                        payload_size,
                                                  uint32_t                        num_unique_gains);

void capi_soft_vol_update_mult_ch_gain_info(capi_soft_vol_t *                me_ptr,
                                            capi_soft_vol_multich_gain_info *gain_list,
                                            uint32_t                        gain,
                                            uint32_t                        *num_unique_gains_ptr,
                                            uint32_t                        *position_ptr,
                                            uint32_t                        channel_type,
                                            uint32_t                        *req_size_ptr);

void capi_soft_vol_update_gain_ch_mask_list(capi_soft_vol_t *                me_ptr,
                                            capi_soft_vol_multich_gain_info *gain_list,
                                            uint32_t                        *position_ptr,
                                            uint32_t                        channel_type,
                                            uint32_t                        *req_size_ptr);

capi_err_t capi_soft_vol_get_multichannel_mute_v2_payload_size(capi_soft_vol_t *me_ptr,
                                                               uint32_t        *mute_chmask_list,
                                                               uint32_t        *req_size_ptr );

bool_t capi_soft_vol_check_multi_ch_channel_mask_v2_param(uint32_t miid,
                                                          uint32_t num_config,
                                                          uint32_t param_id,
                                                          int8_t  *param_ptr,
                                                          uint32_t base_payload_size,
                                                          uint32_t per_cfg_base_payload_size);

capi_err_t capi_soft_vol_process(capi_t *     _pif,
                                              capi_stream_data_t *input[],
                                              capi_stream_data_t *output[]);

capi_err_t capi_soft_vol_end(capi_t *_pif);

capi_err_t capi_soft_vol_set_param(capi_t *     _pif,
                                                uint32_t                   param_id,
                                                const capi_port_info_t *port_info_ptr,
                                                capi_buf_t *            params_ptr);

capi_err_t capi_soft_vol_get_param(capi_t *     _pif,
                                                uint32_t                   param_id,
                                                const capi_port_info_t *port_info_ptr,
                                                capi_buf_t *            params_ptr);

capi_err_t capi_soft_vol_set_properties(capi_t *_pif, capi_proplist_t *props_ptr);

capi_err_t capi_soft_vol_get_properties(capi_t *_pif, capi_proplist_t *props_ptr);

capi_vtbl_t *capi_soft_vol_get_vtbl();

#endif /* CAPI_SOFT_VOL_UTILS_H */
