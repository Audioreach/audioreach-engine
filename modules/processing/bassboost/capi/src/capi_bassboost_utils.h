/* =========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/
/**
 * @file capi_bassboost_utils.h
 *
 * Header file for utilities implement the Audio Post Processor Interface for
 * Acoustic Bass Enhancement
 */

#ifndef CAPI_BASSBOOST_UTILS_H
#define CAPI_BASSBOOST_UTILS_H

#ifndef CAPI_STANDALONE
/* For shared libraries. */
#include "shared_lib_api.h"
#else
#include "capi_util.h"
#endif

#ifndef POSAL_ASSERT
#define POSAL_ASSERT(...)                                                                                              \
   {                                                                                                                   \
      ;                                                                                                                \
   }
#endif

#include "capi.h"
#include "capi_cmn.h"
#include "audio_basic_op_ext.h"

#include "api_bassboost.h"
#include "capi_bassboost.h"

#include "audpp_util.h"
#include "bassboost_api.h"
#include "bassboost_calibration_api.h"

#ifdef DO_BASSBOOST_PROFILING
#include <q6sim_timer.h>
#endif

#define MIID_UNKNOWN 0
 
#define BASSBOOST_MSG_PREFIX "BASSBOOST:[%lX] "
#define BASSBOOST_MSG(ID, xx_ss_mask, xx_fmt, ...) AR_MSG(xx_ss_mask, BASSBOOST_MSG_PREFIX xx_fmt, ID, ##__VA_ARGS__)

const uint32_t        BASS_BOOST_MAX_CHANNELS           = 8;
const int32_t         LIMITER_DEFAULT_DELAY_MS          = 10;
const int32_t         BASS_BOOST_MAX_FRAME_SIZE         = 240;
static const uint32_t BASSBOOST_STACK_SIZE              = 4096;
const uint32_t        BASS_BOOST_BY_PASS_BW             = 0;
const uint32_t        BASS_BOOST_BY_PASS_BW_HIGH        = 1 * 1024 * 1024;
const uint32_t        BASS_BOOST_BW_LOW                 = 2 * 1024 * 1024;
const uint32_t        BASS_BOOST_BW_CH_GREATER_THAN_2   = 2 * 1024 * 1024;
const uint32_t        BASS_BOOST_BW_FS_GREATER_THAN_48K = 2 * 1024 * 1024;
const uint32_t        BASS_BOOST_BW_HIGHEST             = 45 * 1024 * 1024;
const uint32_t        BASSBOOST_MAX_SAMPLE_RATE         = 192000;
/*------------------------------------------------------------------------
 * Function declarations
 * -----------------------------------------------------------------------*/
typedef struct capi_bassboost_profiler
{
   uint64_t start_cycles;
   uint64_t end_cycles;
   uint64_t total_cycles;
   uint64_t sample_count;
   uint64_t average_kpps;
   uint64_t peak_kpps;
   uint32_t sample_rate;
} capi_bassboost_profiler_t;

typedef struct capi_bassboost_events_config
{
   uint32_t enable;
   uint32_t kpps;
   uint32_t delay_in_us;
   uint32_t code_bw;
   uint32_t data_bw;
} capi_bassboost_events_config_t;

typedef struct capi_bassboost_lib_config
{
   bool_t               is_enable_set;
   bool_t               is_mode_set;
   bool_t               is_strength_set;
   bassboost_enable_t   enable;
   bassboost_strength_t strength;
   uint32_t             mode;
} capi_bassboost_lib_config_t;

typedef struct capi_bassboost_t
{
   capi_t               vtbl;
   capi_heap_id_t       heap; // added here
   bassboost_lib_t         lib_instance;
   bassboost_static_vars_t lib_static_vars;
   bassboost_mem_req_t     lib_mem_req;

   capi_event_callback_info_t     cb_info;
   capi_media_fmt_v2_t            input_media_fmt[CAPI_BASSBOOST_MAX_IN_PORTS];
   capi_bassboost_events_config_t events_config;
   capi_bassboost_lib_config_t    lib_config;
   bool_t is_disabled;
   uint32_t                       miid;
#ifdef DO_BASSBOOST_PROFILING
   capi_bassboost_profiler_t kpps_profile_data;
#endif
} capi_bassboost_t;

capi_err_t capi_bassboost_process_set_properties(capi_bassboost_t *me_ptr, capi_proplist_t *proplist_ptr);

capi_err_t capi_bassboost_process_get_properties(capi_bassboost_t *me_ptr, capi_proplist_t *proplist_ptr);

capi_err_t capi_bassboost_update_raise_event(capi_bassboost_t *me_ptr);

void capi_bassboost_init_media_fmt(capi_bassboost_t *me_ptr);

capi_err_t capi_bassboost_check_set_param(capi_bassboost_t *me_ptr,
                                                uint32_t             param_id,
                                                void *               param_ptr,
                                                uint32_t             param_size,
                                                uint32_t             required_size);

capi_err_t capi_bassboost_check_get_param(capi_bassboost_t *me_ptr,
                                                uint32_t             param_id,
                                                void *               param_ptr,
                                                uint32_t             max_data_len,
                                                uint32_t             required_size,
                                                uint32_t *           actual_size_ptr);

void capi_bassboost_profiling(capi_bassboost_t *me_ptr, uint32_t num_samples);

void capi_bassboost_print_kpps(capi_bassboost_t *me_ptr);

uint32_t capi_bassboost_get_kpps(capi_bassboost_t *me_ptr);

#endif /* CAPI_BASSBOOST_UTILS_H */
