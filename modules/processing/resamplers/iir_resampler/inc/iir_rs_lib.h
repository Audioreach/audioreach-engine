/*=========================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
========================================================================= */

#ifndef IIR_RS_LIB_H
#define IIR_RS_LIB_H

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/

#ifndef CAPI_UNIT_TEST
#include "shared_lib_api.h"
#else
#include "Elite_intf_extns_change_media_fmt.h"
#include "capi.h"
#endif

#include "posal.h"
#include "iir_resampler.h"
#include "module_cmn_api.h"



#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/*------------------------------------------------------------------------
 * Macros, Defines, Type declarations
 * -----------------------------------------------------------------------*/
//#define IIR_RS_DBG_TAP

#define IIR_RS_8K_SAMPLING_RATE   (8000)
#define IIR_RS_16K_SAMPLING_RATE  (16000)
#define IIR_RS_24K_SAMPLING_RATE  (24000)
#define IIR_RS_32K_SAMPLING_RATE  (32000)
#define IIR_RS_48K_SAMPLING_RATE  (48000)
#define IIR_RS_96K_SAMPLING_RATE  (96000)
#define IIR_RS_192K_SAMPLING_RATE (192000)
#define IIR_RS_384K_SAMPLING_RATE (384000)


#define IIR_RS_FRAME_LEN_10_MS  (10)
#define IIR_RS_FRAME_LEN_20_MS  (20)
#define IIR_RS_FRAME_LEN_1_MS   (1)
#define IIR_RS_FRAME_LEN_MAX_MS (IIR_RS_FRAME_LEN_20_MS + IIR_RS_FRAME_LEN_1_MS)

#define IIR_RESAMPLER_ROUNDTO8(x) ((((uint32_t)(x) + 7) >> 3) << 3)

// To print all lib status messages
//#define IIR_RESAMPLER_DEBUG_MSG 1

// Max number of ports supported by one instance of resampler CAPI
#define MAX_NUM_PORTS (1)
/*------------------------------------------------------------------------
 * Structure definitions
 * -----------------------------------------------------------------------*/
typedef struct iir_rs_lib_instance_t
{
   iir_resampler_io_config_t     lib_io_config;  // resampler io config
   iir_resampler_memory_config_t lib_mem_config; // resampler mem config
   iir_resampler_t *             lib_mem_ptr;    // resampler mem instance
} iir_rs_lib_instance_t;



typedef struct iir_rs_lib_t
{
   uint32_t                num_ports;                                // active port numbers
   iir_rs_lib_instance_t   lib_instance_per_port_ptr[MAX_NUM_PORTS]; // pointer to lib instance per port
} iir_rs_lib_t;

/*------------------------------------------------------------------------
 * Function definitions
 * -----------------------------------------------------------------------*/
void iir_rs_lib_deinit(iir_rs_lib_t *iir_rs_ptr);

ar_result_t iir_rs_process(iir_rs_lib_t *iir_rs_ptr, int8** input_data_ptr, int8** output_data_ptr,uint32 num_in_samples, uint32 num_out_samples);

ar_result_t iir_rs_get_param(iir_rs_lib_t *iir_rs_ptr, uint32_t param_id, int8_t *param_data_ptr, uint32_t param_size);

ar_result_t iir_rs_lib_clear_algo_memory(iir_rs_lib_t *iir_rs_ptr);

ar_result_t iir_rs_lib_allocate_memory(iir_rs_lib_t         * iir_rs_ptr, uint32_t inp_sampling_rate, uint32_t out_sampling_rate,
                                                uint32_t num_channels, uint32_t bits_per_sample, uint32_t frame_length_ms, uint32_t heap_id);

uint32_t iir_rs_lib_get_kpps(iir_rs_lib_t *iir_rs_ptr, uint32_t input_samp_rate, uint32_t output_samp_rate);

uint32_t iir_rs_lib_get_delay(iir_rs_lib_t *iir_rs_ptr, uint32_t input_samp_rate, uint32_t output_samp_rate);

uint32_t iir_rs_lib_get_bw(iir_rs_lib_t *iir_rs_ptr, uint32_t input_samp_rate);


#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif // iir_rs_lib_H
