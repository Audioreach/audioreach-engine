/* ======================================================================== */
/**
@file capiv2_gain_utils.h

   Header file to implement the Gain block
*/

/* =========================================================================
  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
  ========================================================================== */

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/

#ifndef CAPI_GAIN_UTILS_H
#define CAPI_GAIN_UTILS_H

#include "gain_api.h"
#include "capi_gain.h"
#include "capi_cmn.h"
#include "audpp_util.h"
#include "apply_gain.h"

#define GAIN_BW  (1*1024*1024);

// KPPS number for 8KHz sampling rate and mono channel
//For now the values are same
#define CAPI_GAIN_KPPS_8KHZ_MONO_CH_16BIT  (30)

#define CAPI_GAIN_KPPS_8KHZ_MONO_CH_32BIT_G1  (30)
#define CAPI_GAIN_KPPS_8KHZ_MONO_CH_32BIT_L1  (30)


/* debug message */
#define MIID_UNKNOWN 0
#define GAIN_MSG_PREFIX "CAPI GAIN:[%lX] "
#define GAIN_MSG(ID, xx_ss_mask, xx_fmt, ...)\
         AR_MSG(xx_ss_mask, GAIN_MSG_PREFIX xx_fmt, ID, ##__VA_ARGS__)

/*------------------------------------------------------------------------
 * Function declarations
 * -----------------------------------------------------------------------*/
typedef struct capi_gain_events_config
{
    uint32_t enable;
    uint32_t kpps;
    uint32_t delay_in_us;
    uint32_t code_bw;
    uint32_t data_bw;
} capi_gain_events_config_t;

typedef struct capi_gain_module_config
{
    uint32_t  enable;
    uint16_t  gain_q13;
} capi_gain_module_config_t;

typedef struct capi_gain_t
{
    capi_t                        vtbl;
    capi_event_callback_info_t    cb_info;
    capi_heap_id_t                heap_info;
    capi_media_fmt_v2_t           inp_media_fmt;
    capi_gain_events_config_t     events_config;
    capi_gain_module_config_t     lib_config;
    uint16_t                         gain_q12;      ///< gain level at Q12
	uint32_t miid;
} capi_gain_t;

capi_err_t capi_gain_process_set_properties(
        capi_gain_t *me_ptr,
        capi_proplist_t *proplist_ptr);

capi_err_t capi_gain_process_get_properties(
        capi_gain_t *me_ptr,
        capi_proplist_t *proplist_ptr);

void capi_gain_init_config(
      capi_gain_t *me_ptr);

capi_err_t gain_process(
      capi_t* _pif,
      capi_stream_data_t* input[],
      capi_stream_data_t* output[]);

capi_err_t capi_gain_raise_event(
        capi_gain_t *me_ptr);

capi_err_t capi_gain_raise_process_event(
        capi_gain_t *me_ptr);

#endif //CAPI_GAIN_UTILS_H


