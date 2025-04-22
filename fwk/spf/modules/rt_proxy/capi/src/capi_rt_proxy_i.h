#ifndef __CAPI_RT_PROXY_H_I__
#define __CAPI_RT_PROXY_H_I__

/**
 *   \file capi_rt_proxy_i.h
 *   \brief
 *        This file contains CAPI API's published by RT Proxy module API.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi.h"
#include "capi_rt_proxy.h"
#include "capi_cmn.h"
#include "ar_defs.h"
#include "shared_lib_api.h"
#include "capi_cmn_imcl_utils.h"
#include "capi_fwk_extns_multi_port_buffering.h"
#include "capi_fwk_extns_trigger_policy.h"
#include "rt_proxy_api.h"
#include "rt_proxy_driver.h"
#include "imcl_timer_drift_info_api.h"
#include "capi_intf_extn_metadata.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/*==============================================================================
   Local Defines
==============================================================================*/

// Debug flag
#define DEBUG_RT_PROXY_DRIVER_DRIFT_ADJ

#ifdef DEBUG_RT_PROXY_DRIVER
#define DEBUG_RT_PROXY_DRIVER_DRIFT_ADJ
#endif

/* Stack size requirement from the module */
#define CAPI_RT_PROXY_MODULE_STACK_SIZE (2048)

/* Chunk size used for fragmented circular buffer */
#define DEFAULT_CIRC_BUF_CHUNK_SIZE (2 * 1024)

#define RT_PROXY_MAX_INTENTS_PER_CTRL_PORT 1

#define RT_PROXY_MAX_NUM_RECURRING_BUFS 8

#define RT_PROXY_MAX_RECURRING_BUF_SIZE 64

#define RT_PROXY_NUM_FRAMEWORK_EXTENSIONS 3

#define RT_PROXY_CIRC_BUF_PREFERRED_CHUNK_SIZE 1024

#define RT_PROXY_MAX_CTRL_PORTS 1

#define CTRL_PORT_IDX_0 0

#define RT_PROXY_MAX_INTENTS_PER_CTRL_PORT 1

#define IS_RX_MODULE(me_ptr) (!me_ptr->is_tx_module)

#define IS_TX_MODULE(me_ptr) (me_ptr->is_tx_module)

typedef struct rt_proxy_ctrl_port_info_t
{
   uint32_t          port_id;
   imcl_port_state_t state;
   uint32_t          num_intents;
   uint32_t          intent_list_arr[RT_PROXY_MAX_INTENTS_PER_CTRL_PORT];
} rt_proxy_ctrl_port_info_t;

/** Drift struct used by rt_proxy module to accumulate drift based on buffer fullness */
typedef struct rt_proxy_drift_info_t
{
   imcl_tdi_hdl_t drift_info_hdl;
   /**< Shared drift info handle */

   imcl_tdi_acc_drift_t acc_drift;
   /**< Current drift info  */

   posal_mutex_t drift_info_mutex;
   /**< Mutex to protect read/write between rt_proxy and rat */

} rt_proxy_drift_info_t;

/* CAPI structure  */
typedef struct capi_rt_proxy_t
{
   const capi_vtbl_t *vtbl;
   /* pointer to virtual table */

   POSAL_HEAP_ID heap_id;
   /* Heap id received from framework*/

   capi_event_callback_info_t event_cb_info;
   /* Event Call back info received from Framework*/

   bool_t is_tx_module;
   /* Type of module, tells if module is in TX or RX path */

   capi_media_fmt_v2_t operating_mf;
   // Media format of the module.

   bool_t is_input_mf_received;
   // indicate a valid media format has been recieved by the module.

   bool_t is_calib_set;
   // indicate if a valid calibration has been set on the module.

   param_id_rt_proxy_config_t cfg;
   // Buffer config

   rt_proxy_ctrl_port_info_t ctrl_port_info;
   // Only one control port is supported by rt_proxy module.

   rt_proxy_drift_info_t drift_info;
   // Drift correction thats send to RAT module.

   fwk_extn_param_id_trigger_policy_cb_fn_t policy_chg_cb;
   // call back for changing trigger policy

   intf_extn_param_id_metadata_handler_t metadata_handler;
   // meta data handler utility call back handle.

   fwk_extn_port_trigger_policy_t trigger_policy;
   // current operating mode of the module.

   uint32_t frame_duration_in_us;
   // Container frame duration in us.

   bool_t first_write_frame_done;
   // Indicate if the first write is done.

   rt_proxy_driver_t driver_hdl;
   /* RT proxy driver handle. */

} capi_rt_proxy_t;

/*==============================================================================
   Function declarations
==============================================================================*/

capi_err_t rt_proxy_vaildate_and_cache_input_media_format(capi_rt_proxy_t *me_ptr, capi_buf_t *media_fmt_ptr);

capi_err_t rt_proxy_raise_output_media_format_event(capi_rt_proxy_t *me_ptr, capi_media_fmt_v2_t *mf_info_ptr);

capi_err_t rt_proxy_raise_mpps_and_bw_events(capi_rt_proxy_t *me_ptr);

/*==============================================================================
   IMCL utility functions
==============================================================================*/

capi_err_t capi_rt_proxy_handle_imcl_port_operation(capi_rt_proxy_t *me_ptr, capi_buf_t *params_ptr);

capi_err_t capi_rt_proxy_send_drift_info_hdl_to_rat(capi_rt_proxy_t *me_ptr);

capi_err_t capi_rt_proxy_init_out_drift_info(rt_proxy_drift_info_t *     drift_info_ptr,
                                             imcl_tdi_get_acc_drift_fn_t get_drift_fn_ptr);

capi_err_t capi_rt_proxy_deinit_out_drift_info(rt_proxy_drift_info_t *drift_info_ptr);

// Clients read accumulated drift using this function.
ar_result_t rt_proxy_imcl_read_acc_out_drift(imcl_tdi_hdl_t *      drift_info_hdl_ptr,
                                             imcl_tdi_acc_drift_t *acc_drift_out_ptr);

// rt_proxy writes accumulated drift using this function.
ar_result_t rt_proxy_update_accumulated_drift(rt_proxy_drift_info_t *shared_drift_ptr,
                                              int64_t                current_drift_adjustment,
                                              uint64_t               timestamp);

void rt_proxy_change_trigger_policy(capi_rt_proxy_t *me_ptr, fwk_extn_port_trigger_policy_t new_policy);

capi_err_t capi_rt_proxy_propagate_metadata(capi_rt_proxy_t *me_ptr, capi_stream_data_t *input);

capi_err_t capi_rt_proxy_destroy_md_list(capi_rt_proxy_t *me_ptr, module_cmn_md_list_t **md_list_pptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*__CAPI_RT_PROXY_H_I__ */
