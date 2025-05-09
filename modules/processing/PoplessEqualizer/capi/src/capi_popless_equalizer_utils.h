/* =========================================================================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 * @file capi_popless_equalizer_utils.h
 *
 * Common Audio Processor Interface Utility header for Popless Equalizer.
 */

#ifndef CAPI_POPLESS_EQUALIZER_UTILS_H
#define CAPI_POPLESS_EQUALIZER_UTILS_H
#include "capi.h"
#include "capi_cmn.h"
#include "imcl_p_eq_vol_api.h"
#ifndef CAPI_STANDALONE
/* For shared libraries. */
#include "shared_lib_api.h"
#else
#include "capi_util.h"
#endif

#ifndef POSAL_ASSERT
#define POSAL_ASSERT(...) {;}
#endif

#include "audpp_util.h"
#include "equalizer_api.h"
#include "capi_cmn_imcl_utils.h"
#include "imcl_p_eq_vol_api.h"
#include "capi_intf_extn_imcl.h"
#include "capi_cmn_ctrl_port_list.h"

#define CAPI_P_EQ_MAX_IN_PORTS     1
#define CAPI_P_EQ_MAX_OUT_PORTS    1
#define CAPI_P_EQ_MAX_CTRL_OUT_PORTS    1

#define MIID_UNKNOWN 0
 
#define P_EQ_MSG_PREFIX "P_EQ:[%lX] "
#define P_EQ_MSG(ID, xx_ss_mask, xx_fmt, ...) AR_MSG(xx_ss_mask, P_EQ_MSG_PREFIX xx_fmt, ID, ##__VA_ARGS__)

static const uint32_t EQUALIZER_MAX_CHANNELS = 8;
static const uint32_t EQUALIZER_CROSS_FADE_CONVERGE_NUM_SAMPLES = 32;
static const uint32_t EQUALIZER_CROSS_FADE_TOTAL_PERIOD = 20;

static const uint32_t P_EQ_STACK_SIZE = 2000;
/* Votes are fined tuned based on use cases based on channels and Sampling rate */
const uint32_t P_EQ_BW_LOW = 1*1024*1024;
const uint32_t P_EQ_BW_CH_GREATER_THAN_2 = 1*1024*1024;
const uint32_t P_EQ_BW_FS_GREATER_THAN_48k = 1*1024*1024;
const uint32_t P_EQ_BW_HIGHEST = 3*1024*1024;
const int32_t P_EQ_INIT_PREGAIN = 134217728;
#define P_EQ_VOL_MAX_NUM_RECURRING_BUFS  2

#define P_EQ_VOL_MAX_RECURRING_BUF_SIZE  64

typedef enum
{
	P_EQ_CTRL_PORT_INFO_NOT_RCVD = 0,
	P_EQ_CTRL_PORT_INFO_RCVD
}is_ctrl_port_info_rcvd_t;
typedef enum
{
    P_EQ_WAITING_FOR_MEDIA_FORMAT = -1,
    P_EQ_DISABLE,
    P_EQ_VOL_CTRL,
    P_EQ_PRESET_VOL_RAMP,
    P_EQ_OFF_ON_CROSS_FADE,
    P_EQ_ON_OFF_CROSS_FADE,
    P_EQ_PRESET_CROSS_FADE,
    P_EQ_ENABLE
} p_eq_state_t;

typedef struct _max_band_levels
{
    uint32      num_bands;
    eq_level_t  levels[12];
} max_band_levels;

typedef struct _max_band_freqs
{
    uint32      num_bands;
    eq_freq_t   freqs[12];
} max_band_freqs;

typedef struct _max_eq_lib_cfg_t
{
    eq_config_t      eq_lib_cfg;
    eq_band_specs_t  eq_per_band_spec[12];
} max_eq_lib_cfg_t;

#ifdef DO_POPLESSEQUALIZER_PROFILING
typedef struct p_eq_prof_info
{
   uint32_t sample_rate;
   uint64_t start_cycles;
   uint64_t end_cycles;
   uint64_t total_cycles;
   uint64_t sample_count;
   uint64_t average_kpps;
   uint64_t peak_kpps;
} p_eq_prof_info_t;
#endif

typedef struct imcl_port_info_t
{
   uint32_t     port_id;
   imcl_port_state_t state;
   uint32_t     num_intents;
   uint32_t     intent_list_arr[1];

} imcl_port_info_t;

typedef struct capi_p_eq
{
    const capi_vtbl_t           *vtbl_ptr;
    capi_event_callback_info_t  cb_info;
    capi_media_fmt_v2_t         input_media_fmt;
    eq_lib_t                       lib_instances[EQUALIZER_MAX_CHANNELS][TOTAL_INST];
    eq_static_struct_t             lib_static_vars;
    max_eq_lib_cfg_t               max_eq_cfg;
    p_eq_state_t                   p_eq_state;
    uint32_t                       present_eq_state;
    uint32_t                       transition_num_samples;
    uint32_t                       transition_num_samples_preset;
    uint32_t                       is_new_config_pending;
    uint32_t                       volume_ramp;
    uint32_t                       update_headroom;
    uint32_t                       popless_eq_headroom;
    uint32_t                       is_first_frame;
    uint32_t                       lib_size;
    uint32_t                       num_channels;
    uint32_t                       enable_flag;
    uint32_t                       band_idx;
    uint32_t                       band_freq_millihertz;
    uint32_t                       kpps;
    uint32_t                       code_bw;
    uint32_t                       data_bw;
    capi_heap_id_t 			   heap_info;
    int32_t						   process_check;
    is_ctrl_port_info_rcvd_t   	   ctrl_port_received;
    ctrl_port_list_handle_t 	   ctrl_port_info;
    int32_t					   	   temp_enable_flag;
    uint32_t 					   pending_disable_event;

    uint32_t                       prev_transition_num_samples;
    uint32_t                       vol_ctrl_to_peq_state;
	uint32_t					   miid;
	
#ifdef DO_POPLESSEQUALIZER_PROFILING
    p_eq_prof_info_t               p_eq_kpps_data;
#endif
} capi_p_eq_t;

capi_err_t capi_p_eq_process_set_properties(
        capi_p_eq_t*     me_ptr,
        capi_proplist_t* proplist_ptr);

capi_err_t capi_p_eq_process_get_properties(
        capi_p_eq_t*     me_ptr,
        capi_proplist_t* proplist_ptr);

capi_err_t capi_p_eq_init_media_fmt(
        capi_p_eq_t*     me_ptr);

capi_err_t capi_p_eq_set_config(
        capi_p_eq_t*     me_ptr,
        int8_t*             param_payload_ptr,
        uint32_t            param_size);

capi_err_t equalizer_adjust_lib_size_reinit(
        capi_p_eq_t *me_ptr,
        uint32_t new_num_channels);

capi_err_t capi_p_eq_set_config(
        capi_p_eq_t *me_ptr,
        int8_t* param_payload_ptr,
        uint32_t param_size);

capi_err_t capi_p_eq_check_get_param(
        capi_p_eq_t *me_ptr,
        uint32_t param_id,
        int8_t *param_ptr,
        uint32_t max_data_len,
        uint32_t required_size,
        uint32 *actual_size_ptr);

capi_err_t capi_p_eq_cross_fade_init(
        capi_p_eq_t *me_ptr);

capi_err_t capi_p_eq_create_new_equalizers(
        capi_p_eq_t *me_ptr,
        EQ_INST inst_id);

void capi_p_eq_free_eq_instance(
        capi_p_eq_t *me_ptr,
        EQ_INST inst_id);

capi_err_t capi_p_eq_replace_current_equalizers(
        capi_p_eq_t *me_ptr);

capi_err_t capi_p_eq_set_enable(capi_p_eq_t* me_ptr);

bool_t equalizer_is_cross_fade_active(
        capi_p_eq_t *me_ptr);

void capi_p_eq_copy_config(
        capi_p_eq_t *me_ptr,
        int8_t*  param_payload_ptr);

#ifdef DO_POPLESSEQUALIZER_PROFILING
void capi_p_eq_print_kpps(
        capi_p_eq_t *me_ptr);

void capi_p_eq_profiling(
        capi_p_eq_t *me_ptr,
        uint32_t num_samples);
#endif

capi_err_t get_eq_lib_api_info(
        uint32_t param_id,
        uint32_t *param_size,
        uint32_t *eq_lib_api);

void capi_p_eq_process_in_vol_ctrl(
        capi_p_eq_t *me_ptr,
        capi_stream_data_t* input[],
        capi_stream_data_t* output[]);

capi_err_t capi_p_eq_process_eq(
        capi_p_eq_t *me_ptr,
        capi_stream_data_t*  input[],
        capi_stream_data_t*  output[]);

bool_t is_new_eq_instance_crossfade_finished(
        capi_p_eq_t *me_ptr);

capi_err_t capi_p_eq_update_headroom(
        capi_p_eq_t *me_ptr);

void capi_p_eq_update_delay(
        capi_p_eq_t* me_ptr);

capi_err_t capi_p_eq_update_eq_state(
        capi_p_eq_t *me_ptr);

uint32_t capi_p_eq_get_kpps(
		capi_p_eq_t* me_ptr);

capi_err_t capi_p_eq_update_process_check(capi_p_eq_t* me_ptr, uint32_t process_check);

#endif /* CAPI_POPLESS_EQUALIZER_UTILS_H */

