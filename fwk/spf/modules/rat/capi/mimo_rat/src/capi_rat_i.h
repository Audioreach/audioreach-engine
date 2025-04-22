/* ======================================================================== */
/**
 @file capi_rat_i.h

 Header file to implement the Rate Adapted Timer Utilities
 */
/* =========================================================================
 Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 SPDX-License-Identifier: BSD-3-Clause-Clear
 ========================================================================== */
// clang-format off
// clang-format on
/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#ifndef CAPI_RAT_I_H
#define CAPI_RAT_I_H

#include "capi_cmn.h"
#include "rate_adapted_timer_api.h"
#include "spf_list_utils.h"
#include "capi_intf_extn_imcl.h"
#include "capi_cmn_imcl_utils.h"
#include "capi_fwk_extns_signal_triggered_module.h"
#include "capi_fwk_extns_thresh_cfg.h"
#include "imcl_timer_drift_info_api.h"
#include "other_metadata.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/**------------------------------------------------------------------------
 * Macro Definitions
 * ----------------------------------------------------------------------*/
//#define RAT_DEBUG

/* Number of milliseconds in a second*/
#define NUM_MS_PER_SEC 1000

/* Number of microseconds in a millisecond*/
#define NUM_US_PER_MS 1000

/* Number of microseconds in a second*/
#define NUM_US_PER_SEC 1000000

/* Number of CAPI Framework extension needed */
#define RAT_NUM_FRAMEWORK_EXTENSIONS 2

/**< Max number of control ports per rat end point */
#define RAT_MAX_INTENTS_PER_CTRL_PORT 1

/**< Macro to determine the frame_size in samples per Mill second */
#define RAT_MS_FRAME_SIZE(sample_rate) (sample_rate / NUM_MS_PER_SEC)

/**
 * maximum correction in us that is applied per trigger of the module
 */
#define MAX_CORR_US 50

/* Maximum number of static input control ports*/
#define RAT_MAX_NUM_INP_CONTROL_PORTS 3

// Constant used to check if port id/index mapping is valid.
#define RAT_PORT_INDEX_INVALID 0xFFFFFFFF

#define RAT_PORT_ID_INVALID 0
/**------------------------------------------------------------------------
 * Enums
 * ----------------------------------------------------------------------*/
/* Enum to decide if IMCL is opening or closing a port*/
typedef enum rat_ctrl_port_op_cl { RAT_CTRL_PORT_CLOSE = 0, RAT_CTRL_PORT_OPEN = 1 } rat_ctrl_port_op_cl;

/* Indexing for the static array which stores the incoming ctrl port info*/
typedef enum capi_rat_input_port_index_t {
   QT_HWEP_PORT_IDX              = 0,
   HWEP_BT_PORT_IDX              = 1,
   QT_REMOTE_PORT_IDX            = 2,
   RAT_MAX_NUM_CONTROL_PORTS_INP = 3
} capi_rat_input_port_index_t;

/* Enum to decide if siso or mimo rat is used*/
typedef enum capi_rat_type_t {
   CAPI_MIMO_RAT = 0,
   CAPI_SISO_RAT,
} capi_rat_type_t;

typedef enum rat_data_flow_state {
   RAT_DF_STOPPED  = 0,
   RAT_DF_STARTED  = 1,
   RAT_DF_STOPPING = 2,
} rat_data_flow_state;
/**------------------------------------------------------------------------
 * Structure Definitions
 * ----------------------------------------------------------------------*/

/* IMCL STRUCTS */
/** Drift struct used by RAT module to keep track of its current rat_acc_drift
 *  and allow other modules to access this info through control links*/
typedef struct rat_drift_info_t
{
   imcl_tdi_hdl_t drift_info_hdl;
   /**< Shared drift info  handle */

   imcl_tdi_acc_drift_t rat_acc_drift;
   /**< Current accumulated drift info  */

   posal_mutex_t drift_info_mutex;
   /**< Mutex for the shared rat_acc_drift info with
    rate matching modules  */

} rat_drift_info_t;

/** RAT ctrl port struct which is dynamically created for
 *  each output control link */
typedef struct rat_out_ctrl_port_base_t
{
   uint32_t port_id;
   /**< Control port ID */

   imcl_port_state_t state;
   /**< Control port state */

   uint32_t intent;
   /**< Intent ID */

} rat_out_ctrl_port_base_t;

/** Overall RAT ctrl port struct which stores info for
 *  all output control ports */
typedef struct rat_out_ctrl_port_info_t
{
   uint32_t num_out_active_ports;
   /**< Number of active ports */

   spf_list_node_t *port_list_ptr;
   /**< Control port list. List node is
    of type rat_out_ctrl_port_base_t */

} rat_out_ctrl_port_info_t;

/** RAT ctrl port struct for
 *  each input control link */
typedef struct rat_inp_ctrl_port_info_t
{
   uint32_t port_id;
   /**< Control port ID */

   uint32_t num_intents;
   /**< Num intent ID's */

   uint32_t intent_id_list[RAT_MAX_INTENTS_PER_CTRL_PORT];
   /**< Intent ID list */

   imcl_port_state_t state;
   /**< Control port state */

   imcl_tdi_hdl_t *timer_drift_info_hdl_ptr;
   /**< Incoming drift handle */

   imcl_tdi_acc_drift_t acc_drift;
   /* To store the acc_drift for each port */

} rat_inp_ctrl_port_info_t;

/* PORT STRUCTS for mimo rat*/
typedef struct capi_rat_cmn_port_t
{
   uint32_t self_port_id; // Value of port ID provided in the module definition
   uint32_t self_index;   // Which index [0 to MAX_PORTS-1] corresponds to this port
   uint32_t conn_port_id; // Value of port ID to which this is connected
   uint32_t conn_index;   // Value of the port index to which this port is connected.
   // Closed, stopped, started. Stopped and started are only used on input ports.
   intf_extn_data_port_state_t port_state;
} capi_rat_cmn_port_t;

typedef struct capi_rat_in_port_t
{
   // Must be first field so we can cast common port to input port.
   capi_rat_cmn_port_t cmn;

   // Input media format for this port
   bool_t              inp_mf_received;
   capi_media_fmt_v2_t media_fmt;

   // enum maintaining data flow state for each input
   rat_data_flow_state df_state;

} capi_rat_in_port_t;

typedef struct capi_rat_out_port_t
{
   // Must be first field so we can cast common port to output port.
   capi_rat_cmn_port_t cmn;

   // When input port is closed without eos sent yet, this flag is used to indicate eos/silence md
   // should be sent on the output port on the next process call.
   bool_t begin_silence_insertion_md;

   // Indicates if the current state on output port is inserted silence or not
   bool_t is_state_inserted_silence;

   // Stores underrun counter for each output port, with 1:1 mapping with input
   capi_cmn_underrun_info_t underrun_info;

} capi_rat_out_port_t;

typedef struct capi_rat_events_config_t
{
   uint32_t kpps;
   uint32_t code_bw;
   uint32_t data_bw;
} capi_rat_events_config_t;

/** capi struct for the RAT module*/
typedef struct capi_rat
{
   /* V-table pointer */
   capi_t vtbl;

   /* Heap id, used to allocate memory */
   capi_heap_id_t heap_mem;

   /* Call back info for event raising */
   capi_event_callback_info_t cb_info;

   // Operating media format, applicable for static output port
   capi_media_fmt_v2_t configured_media_fmt;

   /* Struct to track all the performance variables*/
   capi_rat_events_config_t raised_events_config;

   /* To indicate if it is siso or mimo rat*/
   capi_rat_type_t type;

   /* Metadata handler*/
   intf_extn_param_id_metadata_handler_t metadata_handler;

   /* Output control Port Info */
   rat_out_ctrl_port_info_t out_ctrl_port_info;

   /* Input control port info*/
   rat_inp_ctrl_port_info_t inp_ctrl_port_arr[RAT_MAX_NUM_INP_CONTROL_PORTS];

   /* Module output drift info*/
   rat_drift_info_t rat_out_drift_info;

   /* Cache the timer duration configuration */
   param_id_rat_timer_duration_config_t rat_timer_duration_config;

   /* Signal sent by the container*/
   void *signal_ptr;

   /* Module timer*/
   posal_timer_t timer;

   /* Container has enabled the timer */
   bool_t is_timer_enable;

   /* Flag which is set when timer is created */
   bool_t is_timer_created;

   // Set when static port of rat receives mandatory media fmt config
   bool_t configured_media_fmt_received;

   /* Pending drift to be corrected */
   int64_t rat_pending_drift_us;

   /* Save the initial start time to be used in every process call*/
   int64_t absolute_start_time_us;

   /* Save (sampling rate/NUM_MS_PER_SEC)*NUM_US_PER_SEC to prevent recalc in process*/
   uint64_t integ_sr_us;

   /* Counter which increments on each process call*/
   uint64_t counter;

   /* Threshold in bytes for the module*/
   uint32_t threshold;

   /* Frame duration in us*/
   uint32_t frame_dur_us;

   /* frame size in sample */
   uint32_t frame_size_in_samples;

   /* SG based Frame duration in us*/
   uint32_t sg_frame_dur_us;

   /* sample rate used to determine the resolution of timer duration */
   uint32_t resolution_control_sample_rate;

   /* ID unique to this instance of the RAT module*/
   uint32_t iid;

   /* Indicates number of input ports opened (port num info in set prop)*/
   uint32_t num_in_ports;

   /* Indicates number of output ports opened (port num info in set prop)*/
   uint32_t num_out_ports;

   /* Allocates and stores input port information in an array, based on num_in_ports*/
   capi_rat_in_port_t *in_port_info_ptr;

   /* Allocates and stores output port information in an array, based on num_out_ports */
   capi_rat_out_port_t *out_port_info_ptr;

} capi_rat_t;

#define RAT_MSG_PREFIX "RAT: 0x%lx: "
#define RAT_MSG(ID, xx_ss_mask, xx_fmt, ...) AR_MSG_ISLAND(xx_ss_mask, RAT_MSG_PREFIX xx_fmt, ID, ##__VA_ARGS__)
#define RAT_MSG_ISLAND(ID, xx_ss_mask, xx_fmt, ...) AR_MSG_ISLAND(xx_ss_mask, RAT_MSG_PREFIX xx_fmt, ID, ##__VA_ARGS__)

/**------------------------------------------------------------------------
 * Function Definitions
 * ----------------------------------------------------------------------*/
/* IMCL functions */
capi_err_t capi_rat_imcl_port_operation(capi_rat_t *            me_ptr,
                                        const capi_port_info_t *port_info_ptr,
                                        capi_buf_t *            params_ptr);

rat_inp_ctrl_port_info_t *capi_rat_get_inp_ctrl_port_info_ptr(capi_rat_t *me_ptr, uint32_t ctrl_port_id);

rat_out_ctrl_port_base_t *capi_rat_get_out_ctrl_port_info_ptr(rat_out_ctrl_port_info_t *ctrl_port_info_ptr,
                                                              uint32_t                  ctrl_port_id);

capi_err_t capi_rat_init_out_drift_info(rat_drift_info_t *drift_info_ptr, imcl_tdi_get_acc_drift_fn_t get_drift_fn_ptr);

capi_err_t capi_rat_deinit_out_drift_info(rat_drift_info_t *drift_info_ptr);

capi_err_t capi_rat_reset_out_drift_info(rat_drift_info_t *drift_info_ptr);

ar_result_t rat_read_acc_out_drift(imcl_tdi_hdl_t *drift_info_hdl_ptr, imcl_tdi_acc_drift_t *acc_drift_out_ptr);

capi_err_t capi_rat_get_inp_drift(rat_inp_ctrl_port_info_t *port_base_ptr);

capi_err_t capi_rat_send_out_drift_info_hdl(capi_event_callback_info_t *event_cb_info_ptr,
                                            rat_drift_info_t *          rat_drift_info_ptr,
                                            uint32_t                    ctrl_port_id);

capi_err_t capi_rat_deinit_out_control_ports(rat_out_ctrl_port_info_t *control_port_info_ptr);

/* RAT vtbl functions*/
capi_err_t capi_rat_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[]);

capi_err_t capi_rat_end(capi_t *_pif);
capi_err_t capi_rat_set_param(capi_t *                _pif,
                              uint32_t                param_id,
                              const capi_port_info_t *port_info_ptr,
                              capi_buf_t *            params_ptr);

capi_err_t capi_rat_get_param(capi_t *                _pif,
                              uint32_t                param_id,
                              const capi_port_info_t *port_info_ptr,
                              capi_buf_t *            params_ptr);

capi_err_t capi_rat_set_properties(capi_t *_pif, capi_proplist_t *props_ptr);

capi_err_t capi_rat_get_properties(capi_t *_pif, capi_proplist_t *props_ptr);

capi_vtbl_t *capi_rat_get_vtbl();

/* Mimo rat port utilitites */
capi_err_t capi_rat_process_port_num_info(capi_rat_t *me_ptr, capi_prop_t *prop_ptr);

capi_err_t capi_rat_handle_port_op(capi_rat_t *me_ptr, capi_buf_t *payload_ptr);

capi_rat_cmn_port_t *capi_rat_get_port_cmn_from_index(capi_rat_t *me_ptr, uint32_t index, bool_t is_input);

capi_rat_cmn_port_t *capi_rat_get_port_cmn_from_port_id(capi_rat_t *me_ptr, uint32_t port_id, bool_t is_input);

capi_rat_in_port_t *capi_rat_get_in_port_from_port_id(capi_rat_t *me_ptr, uint32_t port_id);

capi_rat_out_port_t *capi_rat_get_out_port_from_port_id(capi_rat_t *me_ptr, uint32_t port_id);

uint32_t capi_rat_get_port_id_from_port_idx(capi_rat_t *me_ptr, uint32_t port_idx, bool_t is_input);

uint32_t capi_rat_get_port_idx_from_port_id(capi_rat_t *me_ptr, uint32_t port_idx, bool_t is_input);

bool_t capi_rat_is_output_static_port(capi_rat_cmn_port_t *port_cmn_ptr);

/* Media format functions*/
bool_t capi_rat_is_sample_rate_accepted(uint32_t operating_mf_sr, uint32_t out_mf_sr);

capi_err_t capi_rat_accept_op_mf(capi_rat_t *me_ptr, bool_t is_static_port_mf, uint32_t port_index);

/* Misc functions */
void capi_rat_process_md_with_no_output(capi_rat_t *me_ptr, capi_stream_data_t *input[]);

capi_err_t capi_rat_timer_enable(capi_rat_t *me_ptr);

capi_err_t capi_rat_raise_kpps_bw_events(capi_rat_t *me_ptr);

capi_err_t capi_rat_raise_thresh_event(capi_rat_t *         me_ptr,
                                       capi_media_fmt_v2_t *mf_ptr,
                                       uint32_t             inp_port_index,
                                       uint32_t             out_port_index);

capi_err_t capi_rat_update_frame_duration(capi_rat_t *me_ptr);

capi_err_t capi_rat_validate_timer_duration_cfg(capi_rat_t *                          me_ptr,
                                                param_id_rat_timer_duration_config_t *rat_timer_duration_cfg_ptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* CAPI_RAT_I_H */