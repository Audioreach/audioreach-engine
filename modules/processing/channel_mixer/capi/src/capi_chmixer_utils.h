/* ======================================================================== */
/**
   @file capi_chmixer_utils.h

   Header file to implement utilities for Channel Mixer audio Post
   Processing module
*/

/* =========================================================================
  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
   ========================================================================== */

#ifndef CAPI_CHMIXER_UTILS_H
#define CAPI_CHMIXER_UTILS_H

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "capi.h"

#include "audpp_util.h"
#include "audio_dsp32.h"

#ifndef CAPI_STANDALONE
#include "shared_lib_api.h"
#endif

#include "ChannelMixerLib.h"
#include "capi_cmn.h"
/*------------------------------------------------------------------------
 * Macros, Defines, Type declarations
 * -----------------------------------------------------------------------*/
//Enable this flag for average and peak kpps analysis (only on sim)
//#define CAPI_CHMIXER_KPPS_PROFILING 1

//Enable this flag for input and output buffer logging (only on sim)
//#define CAPI_CHMIXER_DATA_LOG 1

//Enable this flag for debug messages
//#define CAPI_CHMIXER_DEBUG_MSG 1

/* debug message */
#define MIID_UNKNOWN 0
#define CHMIXER_MSG_PREFIX "CAPI CHMIXER:[%lX] "
#define CHMIXER_MSG(ID, xx_ss_mask, xx_fmt, ...)\
         AR_MSG(xx_ss_mask, CHMIXER_MSG_PREFIX xx_fmt, ID, ##__VA_ARGS__)
		 

static const uint16_t CAPI_CHMIXER_MAX_IN_PORTS = 1;
static const uint16_t CAPI_CHMIXER_MAX_OUT_PORTS = 1;
static const uint16_t CAPI_CHMIXER_DEFAULT_PORT = 0;

static const uint32_t CAPI_CHMIXER_STACK_SIZE = 2048;

static const uint32_t CAPI_CHMIXER_MAX_CHANNELS = CH_MIXER_MAX_NUM_CH;

static const int16_t CAPI_CHMIXER_UNITY_Q14 = (1<<14);

static const uint32_t CAPI_CHMIXER_ENABLE = 1;

#define CAPI_CHMIXER_ALIGN_4_BYTE(x) (((x) + 3) & (0xFFFFFFFC))

#define CAPI_CHMIXER_V2_MIN_SIZE   (sizeof(capi_standard_data_format_v2_t) + sizeof(capi_set_get_media_format_t))

#define CH_MIXER_16BIT_Q_FORMAT 15

#define MEM_ALIGN_EIGHT_BYTE         8
/*------------------------------------------------------------------------
 * Structure definitions
 * -----------------------------------------------------------------------*/
typedef struct capi_chmixer_events_config
{
    bool_t   chmixer_enable;
    uint32_t chmixer_KPPS;
    uint32_t chmixer_delay_in_us;
    uint32_t chmixer_bandwidth;
} capi_chmixer_events_config_t;

typedef struct capi_chmixer_coef_set_t
{
   uint16_t      num_in_ch;
   uint16_t      *in_ch_map;
   uint16_t      num_out_ch;
   uint16_t      *out_ch_map;
   int16_t       *coef_ptr;
} capi_chmixer_coef_set;

typedef struct capi_chmixer_config
{
    bool_t lib_enable;
    uint32_t num_coef_sets;
    capi_chmixer_coef_set *coef_sets_ptr;
}capi_chmixer_config_t;

typedef struct capi_chmixer_profiler
{
    uint64_t start_cycles;
    uint64_t end_cycles;
    uint64_t total_cycles;
    uint64_t total_sample_count;
    uint64_t frame_sample_count;
    uint64_t frame_kpps;
    uint64_t average_kpps;
    uint64_t peak_kpps;
} capi_chmixer_profiler_t;

typedef struct capi_chmixer_data_log_info
{
    uint16_t num_in_channel;
    uint16_t num_out_channel;
    uint32_t samp_rate;
    uint64_t curr_timestamp;
    uint64_t split_timestamp;
    uint32_t instance_id;
} capi_chmixer_data_log_info_t;

typedef struct capi_chmixer
{
    capi_t vtbl;
    capi_event_callback_info_t cb_info;
    capi_heap_id_t heap_mem;
    capi_chmixer_events_config event_config;
    capi_media_fmt_v2_t input_media_fmt[CAPI_CHMIXER_MAX_IN_PORTS];
    capi_media_fmt_v2_t output_media_fmt[CAPI_CHMIXER_MAX_OUT_PORTS];
    bool_t                      use_default_channel_info[CAPI_CHMIXER_MAX_OUT_PORTS];

    capi_chmixer_config_t config;
    void *lib_ptr;
    uint32_t lib_instance_size;
    uint32_t client_enable;

    bool_t is_native_mode;
    bool_t inp_media_fmt_received;
    uint16_t configured_num_channels;
    uint16_t * configured_ch_map;
    uint32_t coef_payload_size;
#ifdef CAPI_CHMIXER_KPPS_PROFILING
    capi_chmixer_profiler_t profiler;
#endif
#ifdef CAPI_CHMIXER_DATA_LOG
    capi_chmixer_data_log_info_t logger;
#endif
    uint32_t miid;
} capi_chmixer_t;

/*------------------------------------------------------------------------
 * Function declarations
 * -----------------------------------------------------------------------*/
uint32_t capi_get_init_mem_req();
void capi_chmixer_init_media_fmt(capi_chmixer_t *const me_ptr);
void capi_chmixer_init_config(capi_chmixer_t *const me_ptr);
void capi_chmixer_init_events(capi_chmixer_t *const me_ptr);

capi_err_t capi_chmixer_check_ch_type(const uint16_t *channel_type,const uint32_t array_size);
capi_err_t capi_chmixer_set_output_media_fmt(capi_chmixer_t *const me_ptr, const capi_media_fmt_v2_t *const out_media_fmt);

capi_err_t capi_chmixer_process_set_properties(capi_chmixer_t * me_ptr,  capi_proplist_t * proplist_ptr);
capi_err_t capi_chmixer_process_get_properties(capi_chmixer_t * me_ptr, capi_proplist_t * proplist_ptr);

capi_err_t capi_chmixer_check_init_lib_instance(capi_chmixer_t *const me_ptr);

capi_err_t capi_chmixer_raise_events(capi_chmixer_t *const me_ptr, bool_t media_fmt_update);

#ifdef CAPI_CHMIXER_KPPS_PROFILING
void capi_chmixer_kpps_profiler_reset(capi_chmixer_t* me_ptr);
void capi_chmixer_kpps_profiler(capi_chmixer_t *me_ptr);
void capi_chmixer_kpps_print(capi_chmixer_t* me_ptr);
#endif

#ifdef CAPI_CHMIXER_DATA_LOG
void capi_chmixer_data_logger(capi_chmixer_t *me_ptr, const capi_stream_data_t *input, const capi_stream_data_t *output);
#endif

#endif // CAPI_CHMIXER_UTILS_H
