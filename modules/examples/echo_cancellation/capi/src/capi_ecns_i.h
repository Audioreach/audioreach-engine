/**
 * \file capi_ecns_i.h
 *  
 * \brief
 *  
 *     Example Echo Cancellation
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef CAPI_ECNS_I_H
#define CAPI_ECNS_I_H

// include files
#include "ecns_calibration_api.h"
#include "capi_ecns.h"
#include "shared_lib_api.h"
#include "capi_fwk_extns_ecns.h"
#include "capi_types.h"
#include "posal.h"
#include "capi_intf_extn_data_port_operation.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

/* Debug log flags */
#define CAPI_ENCS_DEBUG_VERBOSE

#define INVALID ((int32_t)(-1))

// Frame size in msec
#define ECNS_PROCESS_FRAME_SIZE_IN_10MS (10)

// Bandwidth numbers
// These numbers must be profiled and assigned.
#define ECNS_CODE_BW (0)
#define ECNS_DATA_BW (0)

// KPPS
#define ECNS_KPPS_REQUIREMENT 0

/* Delay in usec*/
#define ECNS_DEFAULT_DELAY_IN_US (0)

/* ECNS debug message */
#define ECNS_MSG_PREFIX "CAPI_ECNS[0x%X]: "
#define ECNS_DBG(ID, xx_ss_mask, xx_fmt, ...) AR_MSG(xx_ss_mask, ECNS_MSG_PREFIX xx_fmt, ID, ##__VA_ARGS__)

/* Capi media format event payload structure */
typedef struct capi_ecns_media_fmt_v2_t
{
   capi_set_get_media_format_t    header;
   capi_standard_data_format_v2_t format;
   uint16_t                       channel_type[CAPI_MAX_CHANNELS_V2];
} capi_ecns_media_fmt_v2_t;

/* Capi input port state structure */
typedef struct ecns_input_port_info_t
{
   uint32_t                    port_id;
   uint32_t                    port_index;
   intf_extn_data_port_state_t port_state;
   bool_t                      is_media_fmt_received;
   capi_ecns_media_fmt_v2_t    media_fmt;
   module_cmn_md_list_t *      md_list_ptr;
} ecns_input_port_info_t;


/* Capi output port state structure */
typedef struct ecns_output_port_info_t
{
   uint32_t                    port_id;
   uint32_t                    port_index;
   intf_extn_data_port_state_t port_state;
   capi_ecns_media_fmt_v2_t    media_fmt;
} ecns_output_port_info_t;


/* Module capi handle structure */
typedef struct capi_ecns_t
{
   const capi_vtbl_t *        vtbl_ptr;
   uint32_t                   miid;
   capi_event_callback_info_t cb_info;
   POSAL_HEAP_ID              heap_id;
   capi_port_num_info_t       num_port_info;
   bool_t                     is_enabled;
   bool_t                     is_calibration_received;
   bool_t                     is_library_init_done;
   uint32_t                   kpps;
   uint32_t                   delay_us;

   /* Port info */
   ecns_input_port_info_t  in_port_info[ECNS_MAX_INPUT_PORTS];
   ecns_output_port_info_t out_port_info[ECNS_MAX_OUTPUT_PORTS];

   /* calibration config */
   int32_t feature_mask;

   /* metadata info */
   intf_extn_param_id_metadata_handler_t metadata_handler;

} capi_ecns_t;

/********** Capi helper functions **************/
capi_err_t capi_ecns_handle_get_properties(capi_ecns_t *me, capi_proplist_t *proplist_ptr);
capi_err_t capi_ecns_handle_set_properties(capi_ecns_t *me, capi_proplist_t *proplist_ptr);
capi_err_t capi_ecns_raise_output_media_format_event(capi_ecns_t *me);

/********** Library helper functions **************/
capi_err_t capi_ecns_init_library(capi_ecns_t *me);
capi_err_t capi_ecns_deinit_library(capi_ecns_t *me_ptr);

/********** Metadata propagation helper functions **************/
capi_err_t capi_ecns_propagate_metadata(capi_ecns_t *       me_ptr,
                                        capi_stream_data_t *input[],
                                        capi_stream_data_t *output[],
                                        uint32_t            pri_in_bytes_before,
                                        uint32_t            pri_bytes_consumed_per_ch,
                                        uint32_t            ref_in_bytes_before,
                                        uint32_t            ref_in_bytes_consumed_per_ch);

capi_err_t capi_ecns_destroy_md_list(capi_ecns_t *me_ptr, module_cmn_md_list_t **md_list_pptr);

capi_err_t capi_ecns_handle_metadata_node(capi_ecns_t *me_ptr, module_cmn_md_t *md_ptr);

capi_err_t capi_ecns_drop_input_md(capi_ecns_t *me_ptr,
                                   capi_stream_data_t *input[]);

/********** Data port operation helper functions **************/
capi_err_t capi_ecns_handle_intf_extn_data_port_operation(capi_ecns_t *me_ptr, capi_buf_t *params_ptr);
capi_err_t capi_encs_handle_port_open(capi_ecns_t *me_ptr, uint32_t port_id, uint32_t port_idx, bool_t is_input);
capi_err_t capi_encs_handle_port_close(capi_ecns_t *me_ptr, uint32_t port_idx, bool_t is_input);
capi_err_t capi_encs_handle_port_start(capi_ecns_t *me_ptr, uint32_t port_idx, bool_t is_input);
capi_err_t capi_encs_handle_port_stop(capi_ecns_t *me_ptr, uint32_t port_idx, bool_t is_input);

void       capi_encs_destroy_input_port(capi_ecns_t *me_ptr, uint32_t port_idx);
void       capi_encs_destroy_output_port(capi_ecns_t *me_ptr, uint32_t port_idx);

// Port ID to index mapping functions
capi_err_t ecns_get_input_port_arr_idx(capi_ecns_t *me_ptr, uint32_t input_port_id, uint32_t *input_port_idx_ptr);
capi_err_t ecns_get_output_port_arr_idx(capi_ecns_t *me_ptr, uint32_t output_port_id, uint32_t *output_port_idx_ptr);

// Returns if the input port state is as expected
static inline bool_t is_ecns_input_port_state(capi_ecns_t *               me_ptr,
                                         uint32_t                    port_index,
                                         intf_extn_data_port_state_t port_state)
{
   return (me_ptr->in_port_info[port_index].port_state == port_state);
}

// Returns if the output port state is as expected
static inline bool_t is_ecns_output_port_state(capi_ecns_t *               me_ptr,
                                          uint32_t                    port_index,
                                          intf_extn_data_port_state_t port_state)
{
   return (me_ptr->out_port_info[port_index].port_state == port_state);
}


#endif /* CAPI_ECNS_I_H */
