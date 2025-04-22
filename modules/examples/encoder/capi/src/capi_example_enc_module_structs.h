#ifndef C_EXAMPLE_ENC_MODULE_STRUCTS_H
#define C_EXAMPLE_ENC_MODULE_STRUCTS_H

/**
 * \file example_encoder_module_structs.h
 *  
 * \brief
 *  
 *     Example Encoder Module
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* =======================================================================
 *                       INCLUDES
 * ====================================================================== */
#ifndef CAPI_STANDALONE
#include "shared_lib_api.h"
#else
#include "capi_util.h"
#endif

#include "common_enc_dec_api.h"
#include "example_encoder_module_api.h"
#include "capi_example_enc_module.h"
#include "module_cmn_api.h"
#include "example_enc_lib.h"

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

/* =======================================================================
 *                       DEFINITIONS
 * ====================================================================== */
// Default kpps and bw values
#define CAPI_EXAMPLE_ENC_DEFAULT_CODE_BW 0
#define CAPI_EXAMPLE_ENC_DEFAULT_DATA_BW (4 * 1024 * 1024)
#define CAPI_EXAMPLE_ENC_DEFAULT_KPPS 40000

// Maximum number of channels
#define EXAMPLE_ENC_MAX_NUM_CHANNELS 2

// Input buffer size which decides input threshold of the encoder
#define EXAMPLE_ENC_INP_BUF_SIZE 2048

// Output buffer size which decides output threshold of the encoder
#define EXAMPLE_ENC_OUT_BUF_SIZE 2048

// stack size for example encoder
#define EXAMPLE_ENC_STACK_SIZE 4096

// default abc config for encoder
#define EXAMPLE_ENC_DEFAULT_ABC 1

// default xyz config for encoder
#define EXAMPLE_ENC_DEFAULT_XYZ 2

// Input/operating media format struct
typedef struct capi_example_enc_inp_media_fmt_t
{
   capi_set_get_media_format_t    header;
   capi_standard_data_format_v2_t format;
   uint16_t                          channel_type[CAPI_MAX_CHANNELS_V2];
} capi_example_enc_inp_media_fmt_t;

typedef struct capi_example_enc_out_media_fmt_t
{
   capi_set_get_media_format_t    header;
   capi_raw_compressed_data_format_t format;
} capi_example_enc_out_media_fmt_t;


// Capi struct for encoder module
typedef struct capi_example_enc_module_t
{
   /* Function table for the example enc module */
   capi_t vtbl;

   /* callback structure used to raise events to framework
    * from the example encoder module*/
   capi_event_callback_info_t cb_info;

   /* kpps for example encoder module */
   uint32_t kpps;

   /* code bw for example encoder module */
   uint32_t code_bw;

   /* data bw for example encoder module */
   uint32_t data_bw;

   /* threshold for example encoder module*/
   uint32_t port_threshold;

   /* heap id for example encoder module */
   capi_heap_id_t heap_info;

   /* bitrate of the example encoder module */
   uint32_t bitrate;

   /* encoder modules input media format*/
   capi_example_enc_inp_media_fmt_t input_media_format;

   /* flag to indicate init done for the capi  */
   bool_t capi_init_done;

   /* example encoder cfg parameters */
   uint32_t enc_cfg_abc;
   uint32_t enc_cfg_xyz;
} capi_example_enc_module_t;

#if defined(__cplusplus)
}
#endif // __cplusplus

/* =======================================================================
 *                       FUNCTION DECLARATIONS
 * ====================================================================== */
/* For all functions in capi_example_enc_module_utils being called from capi_example enc_module.cpp*/

capi_err_t capi_example_enc_module_module_init_media_fmt_v2(
   capi_example_enc_inp_media_fmt_t *media_fmt_ptr);

capi_err_t capi_example_enc_module_init_capi(capi_example_enc_module_t *me_ptr);

capi_err_t capi_example_enc_module_set_bit_rate(capi_example_enc_module_t *me_ptr, capi_buf_t *params_ptr);

capi_err_t capi_example_enc_module_set_enc_cfg_blk(capi_example_enc_module_t *me_ptr,
                                                         capi_buf_t *               params_ptr);

capi_err_t capi_example_enc_module_process_set_properties(capi_example_enc_module_t *me_ptr,
                                                                capi_proplist_t *          proplist_ptr);

capi_err_t capi_example_enc_module_process_get_properties(capi_example_enc_module_t *me_ptr,
                                                                capi_proplist_t *          proplist_ptr);

capi_err_t capi_example_enc_module_raise_output_media_fmt_event_v2(capi_event_callback_info_t *cb_info_ptr);

capi_err_t capi_example_enc_module_raise_kpps_event(capi_example_enc_module_t *me_ptr, uint32_t kpps);

capi_err_t capi_example_enc_module_raise_bandwidth_event(capi_example_enc_module_t *me_ptr,
                                                               uint32_t                      code_bandwidth,
                                                               uint32_t                      data_bandwidth);

capi_err_t capi_example_enc_module_raise_port_data_threshold_event(capi_example_enc_module_t *me_ptr,
                                                                         uint32_t                      threshold_bytes,
                                                                         bool_t                        is_input_port,
                                                                         uint32_t                      port_index);

#endif /* C_EXAMPLE_ENC_MODULE_STRUCTS_H */
