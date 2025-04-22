/**
 * \file capi_demuxer_utils.h
 * \brief
 *        Header file to implement utilities for DEMUXER
 *  
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef CAPI_DEMUXER_UTILS_H
#define CAPI_DEMUXER_UTILS_H

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "capi_cmn.h"
#include "ar_error_codes.h"
#include "capi_intf_extn_metadata.h"
#include "demuxer_api.h"
#include "posal.h"
#include "capi_intf_extn_data_port_operation.h"
#include "capi_fwk_extns_trigger_policy.h"
#include "capi_intf_extn_prop_is_rt_port_property.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/* clang-format off */

/*------------------------------------------------------------------------
 * Macros
 * -----------------------------------------------------------------------*/

/*------------------------------------------------------------------------
 * Type definitions
 * -----------------------------------------------------------------------*/
typedef struct capi_demuxer_events_config
{
   uint32_t        kpps;
} capi_demuxer_events_config_t;


typedef struct demuxer_cached_out_config_t
{
   uint32_t output_port_id;
   uint32_t num_channels;
   uint16_t input_channel_index[CAPI_MAX_CHANNELS_V2];
   uint16_t output_channel_type[CAPI_MAX_CHANNELS_V2];
}demuxer_cached_out_config_t;

typedef struct capi_demuxer_out_port_state_t
{
   uint32_t                    port_id;
   uint32_t num_channels;
   intf_extn_data_port_state_t state;  // Port state, based on port operation.
   uint32_t is_cfg_received;
   capi_media_fmt_v2_t   operating_out_mf;
   uint16_t input_channel_index[CAPI_MAX_CHANNELS_V2];
   uint16_t output_channel_type[CAPI_MAX_CHANNELS_V2];
} capi_demuxer_out_port_state_t;


#ifdef PROD_SPECIFIC_MAX_CH
static const uint16_t default_channel_index[CAPI_MAX_CHANNELS_V2] =
{0, 1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11,12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22,23, 24, 25, 26, 27, 28, 29, 30, 31,
32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 
63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 
94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 
120, 121, 122, 123, 124, 125, 126, 127};
#else	
static const uint16_t default_channel_index[CAPI_MAX_CHANNELS_V2] =
{ 0, 1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11,12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22,23, 24, 25, 26, 27, 28, 29, 30, 31};
#endif

typedef struct capi_demuxer
{
   capi_t                              vtbl;
   /* v-table pointer */
   capi_heap_id_t                      heap_mem;
   /* Heap id, used to allocate memory */
   capi_event_callback_info_t          cb_info;
   /* Call back info for event raising */
   capi_demuxer_events_config_t       events_config;
   /* Event information struct */
   capi_demuxer_out_port_state_t      *out_port_state_arr;
   /*Array to maintain state information (active/inactive)*/
   demuxer_cached_out_config_t      *cached_out_cfg_arr;
   /*Array to maintain state information (active/inactive)*/
   capi_media_fmt_v2_t                 operating_mf;
   /* Operating Media Format of the custom DEMUXER Module*/
   bool_t                                 is_num_ports_set;
   /* boolean to indicate if the number of ports has been set*/
   bool_t                                 is_in_media_fmt_set;
   /* boolean to indicate if the input media format has been set*/
   uint32_t                               num_out_ports;
   /* Number of output ports on the DEMUXER Module*/
   uint32_t miid;
} capi_demuxer_t;

/* clang-format on */
/*------------------------------------------------------------------------
 * Function Declarations
 * -----------------------------------------------------------------------*/
/* Utility functions not a part of the function table  (vtbl) */
/*Calculates and stores the current KPPS requirement of the Demuxer module*/
capi_err_t capi_demuxer_update_and_raise_kpps_event(capi_demuxer_t *me_ptr);

capi_err_t capi_demuxer_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[]);

capi_err_t capi_demuxer_end(capi_t *_pif);

capi_err_t capi_demuxer_set_param(capi_t *                _pif,
                                   uint32_t                param_id,
                                   const capi_port_info_t *port_info_ptr,
                                   capi_buf_t *            params_ptr);

capi_err_t capi_demuxer_get_param(capi_t *                _pif,
                                   uint32_t                param_id,
                                   const capi_port_info_t *port_info_ptr,
                                   capi_buf_t *            params_ptr);

capi_err_t capi_demuxer_set_properties(capi_t *_pif, capi_proplist_t *props_ptr);

capi_err_t capi_demuxer_get_properties(capi_t *_pif, capi_proplist_t *props_ptr);

capi_vtbl_t *capi_demuxer_get_vtbl();

void capi_demuxer_get_port_index(capi_demuxer_t *me_ptr,uint32_t port_id,uint32_t *port_index);

capi_err_t capi_demuxer_validate_out_cfg_and_raise_out_mf(capi_demuxer_t *me_ptr,uint32_t port_index);

capi_err_t capi_demuxer_search_cached_cfg_and_update_out_cfg(capi_demuxer_t *me_ptr,uint32_t port_index,uint32_t port_id);


#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif // CAPI_DEMUXER_UTILS_H
