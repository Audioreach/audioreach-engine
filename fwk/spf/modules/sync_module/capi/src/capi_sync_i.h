/**
 * \file capi_sync_i.h
 * \brief
 *        Internal header file for the Sync module.
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef _CAPI_SYNC_I_H_
#define _CAPI_SYNC_I_H_

#ifndef CAPI_STANDALONE
/* For shared libraries. */
#include "shared_lib_api.h"
#else
#include "capi_util.h"
#endif

#include "capi.h"
#include "capi_sync.h"
#include "capi_cmn.h"
#include "sync_api.h"

#include "capi_fwk_extns_sync.h"
#include "capi_fwk_extns_container_frame_duration.h"
#include "capi_intf_extn_mimo_module_process_state.h"
#include "spf_list_utils.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/* Number of max input/outputs for Generic Sync Module, defined in api file. */
// #define SYNC_MAX_IN_PORTS  8
// #define SYNC_MAX_OUT_PORTS 8

/* Number of max input/output ports for EC Sync Module */
#define EC_SYNC_MAX_IN_PORTS 2
#define EC_SYNC_MAX_OUT_PORTS 2

// TODO(claguna): Define static port.
#define SYNC_EC_PRIMARY_IN_PORT_ID 0x4
#define SYNC_EC_PRIMARY_OUT_PORT_ID 0x1

#define SYNC_EC_SECONDARY_IN_PORT_ID 0x2
#define SYNC_EC_SECONDARY_OUT_PORT_ID 0x3

// Identify non-EC ports
#define SYNC_PORT_ID_RANGE_BEGIN 0x5

// Constant used to check if port id/index mapping is valid.
#define SYNC_PORT_INDEX_INVALID 0xFFFFFFFF

#define SYNC_PORT_ID_INVALID 0

// FWK_EXTN_SYNC,FWK_EXTN_CONTAINER_FRAME_DURATION
#define SYNC_NUM_FRAMEWORK_EXTENSIONS 2

// Upstream should be sending 1ms frames. Error in sync time is +- 1 frame.
#define SYNC_UPSTREAM_FRAME_SIZE_US 1000

// Enable for debug logs.
//#define CAPI_SYNC_DEBUG   TRUE
//#define CAPI_SYNC_VERBOSE TRUE

/*------------------------------------------------------------------------
 * Function declarations
 * -----------------------------------------------------------------------*/
typedef struct capi_sync_events_config_t
{
   uint32_t kpps;
   uint32_t delay_in_us;
   uint32_t code_bw;
   uint32_t data_bw;
   bool_t   enable;
} capi_sync_events_config_t;

typedef enum capi_sync_state_t { STATE_SYNCED, STATE_STARTING } capi_sync_state_t;

typedef enum capi_port_state_t {
   CAPI_PORT_STATE_CLOSED,
   CAPI_PORT_STATE_STOPPED,
   CAPI_PORT_STATE_STARTED
} capi_port_state_t;

typedef enum capi_sync_mode_t {
   MODE_EC_PRIO_INPUT,       // Indicates EC use case where a single input port is marked as high priority
   MODE_ALL_EQUAL_PRIO_INPUT // Indicates General Sync Module where all input ports have equal priority
} capi_sync_mode_t;

typedef struct capi_sync_config_t
{
   uint32_t frame_len_us; // Container's operating frame len in us
} capi_sync_config_t;

typedef struct capi_sync_cmn_port_t
{
   uint32_t          self_port_id; // Value of port ID provided in the module definition
   uint32_t          self_index;   // Which index [0 to MAX_PORTS-1] corresponds to this port
   uint32_t          conn_index;   // Value of the port index to which this port is connected.
   uint32_t          conn_port_id; // Value of port ID to which this is connected
   capi_port_state_t state;        // Closed, stopped, started. Stopped and started are only used on input ports.

} capi_sync_cmn_port_t;

typedef struct capi_sync_in_port_t
{
   capi_sync_cmn_port_t cmn;          // Must be first field so we can cast common port to input port.
   capi_buf_t *         int_bufs_ptr; // Data that is buffered up while starting or stopping. Capi bufs are stored
                                      // here as an array and this memory also includes space for the buffers
                                      // themselves in one contiguous block.
   capi_stream_data_v2_t int_stream; // Internal structure used to hold the stream state info for propagating metadata
                                     // from input to internal list, and then from the internal list to the output.

   /* Meta data information */
   int64_t buffer_timestamp; // Time stamp at the head of data buffered at this input port. This will be passed to
                             // the output. The timestamp will be adjusted if initial zeros are added.

   bool_t is_timestamp_valid;    // Whether buffer_timestamp is valid or not.
   bool_t pending_data_flow_gap; // True only for secondary port, if we buffered data along with a data flow gap.
                                 // Next time we send out data we should also send the data gap. Also we should
                                 // send the data gap when we clear the internal buffer while this is set.
   bool_t pending_eos;           // Indicates if EOS is to be delivered from this input port.
   bool_t pending_eof;           // Indicates if EOF occurs on this input stream.
   bool_t is_eos_rcvd;           // Indicates if this port has delivered EOS

   /* Port configuration */
   bool_t is_output_sent_once;       // Indicates if this port has delivered output at least once.
                                     // Reset when this input needs to sync again.
   bool_t is_threshold_disabled;     // Indicates if this port requested for a threshold disable.
                                     // Reset when this input needs to sync again.
   bool_t will_start_rcvd;           // Indicates if this port is started from the container context.
                                     // Reset when this input has been stopped.
   capi_media_fmt_v2_t media_fmt;    // Input media format for this port. It is required that both ports are
                                     // configured to the same media format. This is stored per port to verify that.
                                     // RR: TODO conclude on this
   uint32_t peer_frame_size_us;      // Frame size of the up stream container in us for this port
   uint32_t frame_size_bytes_per_ch; // Frame size of upstream. Need to allocate buffers to account for
                                     // one extra frame. For EC case especially, in the case that
                                     // secondary error is 1ms extra. Then we need to allow for secondary
                                     // to have extra frame instead of dropping.
   uint32_t threshold_bytes_per_ch;  // Threshold in bytes per channel.

} capi_sync_in_port_t;

typedef struct capi_sync_out_port_t
{
   capi_sync_cmn_port_t cmn; // Must be first field so we can cast common port to output port.

   int64_t expected_timestamp; //next expected timestamp of the output data.
   bool_t   is_ts_valid; //set if expected timestamp is valid.

   // When input port is closed without eos sent yet, this flag is used to create eos on the
   // output port on the next process call. It doesn't cover the case of last input port was closed
   // since process won't be called in that case.
   bool_t needs_eos_at_close;

   // Output media format is the same as the connected input media format
} capi_sync_out_port_t;

// Common structure to contain the mask of input/output ports to be handled
typedef struct capi_sync_mask_t
{
   uint32_t input;
   uint32_t output;

} capi_sync_mask_t;

typedef struct capi_sync_proc_info_t
{
   capi_sync_mask_t capi_data_avail_mask;  // Mask of ports on which CAPIv2 buffers have data/space to process
   capi_sync_mask_t ports_to_process_mask; // Mask of sync module ports to be processed
   capi_sync_mask_t ports_processed_mask;  // Mask of sync module ports that were processed

   uint32_t synced_ports_mask;  // Mask of sync module input ports that are sync'ed
   uint32_t ready_to_sync_mask; // Mask of sync module input ports that are ready to sync
   uint32_t stopped_ports_mask; // Mask of sync module input ports in stop state

   uint32_t num_synced;        // Number of sync'ed input ports
   uint32_t num_ready_to_sync; // Number of input ports waiting to sync
   uint32_t num_stopped;       // Number of stopped input ports
   uint32_t process_counter;   // Running counter to track number of process calls

} capi_sync_proc_info_t;

typedef struct capi_sync_t
{
   capi_t                     vtbl;
   capi_event_callback_info_t cb_info;
   capi_heap_id_t             heap_info;
   uint32_t                   miid; // Module-Instance ID value

   // Module configuration.
   capi_sync_events_config_t events_config;
   capi_sync_config_t        module_config;

   // Port information.
   uint32_t             num_in_ports;
   uint32_t             num_out_ports;
   uint32_t             num_opened_in_ports;
   capi_sync_in_port_t *in_port_info_ptr;   // Allocates and stores input port information in an array fashion
                                            // based on the value of num_in_ports
   capi_sync_out_port_t *out_port_info_ptr; // Allocates and stores output port information in an array fashion
                                            // based on the value of num_in_ports

   // State information
   bool_t                threshold_is_disabled; // Checks if sync has disabled or enabled the threshold.
   capi_sync_mode_t      mode;                  // Specifies the mode of operation of the Sync Module
   capi_sync_state_t     synced_state;          // Sync'ing, steady state, closing.
   capi_sync_proc_info_t proc_ctx;              // Sync Module Process Info

   // Metadata interface extension info.
   intf_extn_param_id_metadata_handler_t metadata_handler;

   bool_t is_mimo_process_state_intf_ext_supported; // flag is set if fwk supports #INTF_EXTN_MIMO_MODULE_PROCESS_STATE

} capi_sync_t;

// capi_sync utils
capi_err_t capi_sync_ext_input_threshold_change(capi_sync_t *me_ptr);
void capi_sync_init_config(capi_sync_t *me_ptr, capi_sync_mode_t mode);
capi_err_t capi_sync_calc_threshold_bytes(capi_sync_t *me_ptr, capi_sync_in_port_t *in_port_ptr);

// capi_sync port utils
capi_err_t capi_sync_init_in_out_ports(capi_sync_t *me_ptr);
uint32_t capi_sync_get_supported_num_ports(capi_sync_t *me_ptr, bool_t is_input);
bool_t capi_sync_port_id_is_valid(capi_sync_t *me_ptr, uint32_t port_id, bool_t is_input);
capi_sync_out_port_t *capi_sync_get_out_port_from_index(capi_sync_t *me_ptr, uint32_t port_index);
capi_sync_in_port_t *capi_sync_get_in_port_from_index(capi_sync_t *me_ptr, uint32_t port_index);
capi_sync_cmn_port_t *capi_sync_get_port_cmn_from_port_id(capi_sync_t *me_ptr, uint32_t port_id, bool_t is_input);
capi_sync_in_port_t *capi_sync_get_in_port_from_port_id(capi_sync_t *me_ptr, uint32_t port_id);
capi_sync_out_port_t *capi_sync_get_out_port_from_port_id(capi_sync_t *me_ptr, uint32_t port_id);

// capi_sync_control_utils
capi_err_t capi_sync_process_set_properties(capi_sync_t *me_ptr, capi_proplist_t *proplist_ptr);
capi_err_t capi_sync_process_get_properties(capi_sync_t *me_ptr, capi_proplist_t *proplist_ptr);
capi_err_t capi_sync_process_set_param(capi_sync_t *           me_ptr,
                                       uint32_t                param_id,
                                       const capi_port_info_t *port_info_ptr,
                                       capi_buf_t *            params_ptr);
capi_err_t capi_sync_process_get_param(capi_sync_t *           me_ptr,
                                       uint32_t                param_id,
                                       const capi_port_info_t *port_info_ptr,
                                       capi_buf_t *            params_ptr);
capi_err_t capi_sync_process_port_will_start(capi_sync_t *me_ptr);
capi_err_t capi_sync_in_port_stop(capi_sync_t *me_ptr, capi_sync_in_port_t *in_port_ptr, capi_stream_data_t *output[]);

// capi_sync_data_utils function declarations.
capi_err_t sync_module_process(capi_sync_t *me_ptr, capi_stream_data_t *input[], capi_stream_data_t *output[]);
capi_err_t ec_sync_mode_process(capi_sync_t *me_ptr, capi_stream_data_t *input[], capi_stream_data_t *output[]);
capi_err_t generic_sync_mode_process(capi_sync_t *me_ptr, capi_stream_data_t *input[], capi_stream_data_t *output[]);
bool_t capi_sync_port_meets_threshold(capi_sync_t *        me_ptr,
                                      capi_sync_in_port_t *in_port_ptr,
                                      capi_stream_data_t * input[],
                                      bool_t               check_process_buf);
capi_err_t capi_sync_allocate_port_buffer(capi_sync_t *me_ptr, capi_sync_in_port_t *in_port_ptr);
void capi_sync_deallocate_port_buffer(capi_sync_t *me_ptr, capi_sync_in_port_t *in_port_ptr);
capi_err_t capi_sync_clear_buffered_data(capi_sync_t *me_ptr, capi_sync_in_port_t *in_port_ptr);
capi_err_t capi_sync_validate_io_bufs(capi_sync_t *       me_ptr,
                                      capi_stream_data_t *input[],
                                      capi_stream_data_t *output[],
                                      bool_t *            does_any_port_have_data);
capi_err_t capi_sync_handle_dfg(capi_sync_t *       me_ptr,
                                capi_stream_data_t *input[],
                                capi_stream_data_t *output[],
                                bool_t *            data_gap_found_ptr);
capi_err_t capi_sync_mark_input_unconsumed(capi_sync_t *        me_ptr,
                                           capi_stream_data_t * input[],
                                           capi_sync_in_port_t *in_port_ptr);
capi_err_t capi_sync_buffer_new_data(capi_sync_t *        me_ptr,
                                     capi_stream_data_t * input[],
                                     capi_sync_in_port_t *in_port_ptr);
capi_err_t capi_sync_send_buffered_data(capi_sync_t *        me_ptr,
                                        capi_stream_data_t * output[],
                                        capi_sync_state_t    synced_state,
                                        capi_sync_in_port_t *in_port_ptr,
                                        capi_stream_data_t * input[]);
bool_t capi_sync_should_disable_thresh(capi_sync_t *me_ptr);
capi_err_t capi_sync_pass_through_metadata_single_port(capi_sync_t *        me_ptr,
                                                       capi_sync_in_port_t *in_port_ptr,
                                                       capi_stream_data_t * output[],
                                                       capi_stream_data_t * input[]);
uint32_t generic_sync_calc_conn_mask(capi_sync_t *me_ptr, uint32_t incoming_mask, bool_t is_in_to_out);
uint32_t generic_sync_calc_synced_input_mask(capi_sync_t *me_ptr);
uint32_t generic_sync_calc_waiting_input_mask(capi_sync_t *me_ptr);
uint32_t generic_sync_calc_stopped_input_mask(capi_sync_t *me_ptr, capi_stream_data_t *input[]);
bool_t capi_sync_sdata_has_dfg(capi_sync_t *me_ptr, capi_stream_data_v2_t *sdata_ptr);
bool_t capi_sync_sdata_has_any_md(capi_stream_data_v2_t *sdata_ptr);

// capi_sync_event_utils function declarations.
capi_err_t capi_sync_raise_event(capi_sync_t *me_ptr);
void capi_sync_raise_enable_disable_event(capi_sync_t *me_ptr, bool_t is_enable);
capi_err_t capi_sync_raise_event_toggle_threshold(capi_sync_t *me_ptr, bool_t enable_threshold);
capi_err_t capi_sync_check_raise_out_port_active_inactive_event(capi_sync_t *me_ptr);
// Validate given port index
static inline bool_t capi_sync_is_valid_port_index(capi_sync_t *me_ptr, uint32_t index, bool_t is_input)
{
   bool_t is_valid = FALSE;
   if (!me_ptr)
   {
      return FALSE;
   }
   uint32_t max_ports = is_input ? me_ptr->num_in_ports : me_ptr->num_out_ports;

   // port index starts from zero
   is_valid = (index < max_ports) ? TRUE : FALSE;
   return is_valid;
}

// Given port index, fetch the cmn port structure
static inline capi_sync_cmn_port_t *capi_sync_get_port_cmn_from_index(capi_sync_t *me_ptr,
                                                                      uint32_t     index,
                                                                      bool_t       is_input)
{
   capi_sync_cmn_port_t *port_cmn_ptr = NULL;
   if (!capi_sync_is_valid_port_index(me_ptr, index, is_input))
   {
      return NULL;
   }

   port_cmn_ptr = is_input ? &me_ptr->in_port_info_ptr[index].cmn : &me_ptr->out_port_info_ptr[index].cmn;
   return port_cmn_ptr;
}

// Check if input port is in started state
static inline bool_t capi_sync_is_path_running(capi_sync_t *me_ptr, uint32_t index)
{
   return (CAPI_PORT_STATE_STARTED == me_ptr->in_port_info_ptr[index].cmn.state);
}

// Check if we have set media format yet on a given input port. This will be true if the port's
// minor version is not invalid (among other things!).
static inline bool_t capi_sync_media_fmt_is_valid(capi_sync_in_port_t *in_port_ptr)
{
   // RR: check for num_channels should be sufficient?
   return (CAPI_DATA_FORMAT_INVALID_VAL != in_port_ptr->media_fmt.format.num_channels);
}

// Check if there is any input on the given input port.
static inline bool_t capi_sync_input_has_data(capi_stream_data_t *input[], uint32_t input_index)
{
   return input[input_index] && input[input_index]->buf_ptr && input[input_index]->buf_ptr[0].actual_data_len;
}

static inline bool_t capi_sync_output_has_space(capi_stream_data_t *output[], uint32_t output_index)
{
   return output[output_index] && output[output_index]->buf_ptr &&
          ((output[output_index]->buf_ptr[0].max_data_len - output[output_index]->buf_ptr[0].actual_data_len) > 0);
}

static inline bool_t capi_sync_input_buffer_has_space(capi_sync_in_port_t *in_port_ptr)
{
   return in_port_ptr && in_port_ptr->int_bufs_ptr &&
          (in_port_ptr->int_bufs_ptr[0].max_data_len > in_port_ptr->int_bufs_ptr[0].actual_data_len);
}

/**
 *  Checks if input port will start can be ignored for the given input port
 */
static inline bool_t can_sync_in_port_start_be_ignored(capi_sync_t *me_ptr, capi_sync_in_port_t *in_port_ptr)
{
   // if port is not opened or is already in run state, then ignore the start
   return ((CAPI_PORT_STATE_CLOSED == in_port_ptr->cmn.state) ||
           ((TRUE == in_port_ptr->is_threshold_disabled) && (CAPI_PORT_STATE_STARTED == in_port_ptr->cmn.state)) ||
           ((CAPI_PORT_STATE_STARTED == in_port_ptr->cmn.state) && (TRUE == in_port_ptr->is_output_sent_once)));
}

/**
 *  Set the 'pos' bit in the given mask
 */
static inline void capi_sync_set_bit(uint32_t *mask_ptr, uint32_t pos)
{
   uint32_t mask = 1 << pos;
   *mask_ptr |= mask;
}

/**
 *  Set the 'pos' bit in the given mask
 */
static inline void capi_sync_clear_bit(uint32_t *mask_ptr, uint32_t pos)
{
   uint32_t mask = 1 << pos;
   *mask_ptr &= ~mask;
}

static inline uint32_t align_to_8_byte(const uint32_t num)
{
   return ((num + 7) & (0xFFFFFFF8));
}

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif //_CAPI_SYNC_I_H_
