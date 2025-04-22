/**
 * \file capi_smart_sync_i.h
 * \brief
 *        Internal header file for the Smart Sync module.
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef _CAPI_SMART_SYNC_UTILS_H_
#define _CAPI_SMART_SYNC_UTILS_H_

#ifndef CAPI_STANDALONE
/* For shared libraries. */
#include "shared_lib_api.h"
#else
#include "capi_util.h"
#endif

#include "capi.h"
#include "capi_smart_sync.h"
#include "capi_cmn.h"
#include "smart_sync_api.h"
#include "capi_fwk_extns_sync.h"
#include "capi_fwk_extns_voice_delivery.h"
#include "capi_fwk_extns_container_frame_duration.h"
#include "capi_intf_extn_data_port_operation.h"
#include "other_metadata.h"
#include "ar_msg.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/* clang-format off */

// FWK_EXTN_SYNC, FWK_EXTN_VOICE_DELIVERY,FWK_EXTN_CONTAINER_FRAME_DURATION
#define SMART_SYNC_NUM_FRAMEWORK_EXTENSIONS 3

// Constant used to check if port id/index mapping is valid.
#define SMART_SYNC_PORT_INDEX_INVALID       0xFFFFFFFF

#define SMART_SYNC_PRIMARY_IN_PORT_ID       0x2
#define SMART_SYNC_PRIMARY_OUT_PORT_ID      0x1

#define SMART_SYNC_SECONDARY_IN_PORT_ID     0x4
#define SMART_SYNC_SECONDARY_OUT_PORT_ID    0x3

#define VFR_CYCLE_DURATION_20MS             20
#define VFR_CYCLE_DURATION_40MS             40

// Enable for debug logs.
//#define SMART_SYNC_DEBUG TRUE
#define SMART_SYNC_DEBUG_HIGH

#define ALIGN_8_BYTES(a) ((a + 7) & (0xFFFFFFF8))

// if processing after first proc tick is delayed more than 2 ms then it will be ignored.
//increased to 2ms to allow for RT-RT jitter(0.99999 ms) and scheduling (1 ms)
#define SMART_SYNC_FIRST_PROC_TICK_SCHEDULING_JITTER_THRESHOLD_US (2000)

/*------------------------------------------------------------------------
 * Function declarations
 * -----------------------------------------------------------------------*/
typedef struct capi_smart_sync_events_config_t
{
   uint32_t kpps;
   uint32_t delay_in_us;
   uint32_t code_bw;
   uint32_t data_bw;
   bool_t   enable; // Always true.
} capi_smart_sync_events_config_t;

typedef enum capi_smart_sync_state_t
{
   // Before the first proc tick, the container runs in buffer triggered mode, accepting RT input
   // as soon as it comes in and buffering immediately. When the first proc tick occurs, we are guaranteed
   // to have buffered all data coming before the first proc tick (besides very small potential jitter: The case
   // where a buffer comes slightly before the proc tic while pp is sleeping or handling another command).
   // Thus at that point we can send out all buffered data with zeros padded up to proc_samples, and output one
   // threshold worth of data. When this happens, the module moves into FIRST_VFR_CYCLE state.
   SMART_SYNC_STATE_BEFORE_FIRST_PROC_TICK = 0,

   // This state signifies cases where there are multiple frames per vfr cycle. In this case part of the input will
   // be buffered in smart sync and the remaining will come from the PP local buffer. Smart sync needs to buffer data until
   // threshold is met on both ports (no more). At that point it can exhaust the local buffer, and move to steady state
   // where data is passed through without having to move through smart sync's buffer.
   SMART_SYNC_STATE_SYNCING,

   // After the first vfr cycle, smart sync can act like a pass-through threshold module, accepting a full frame of data on
   // the input and copying to output.
   SMART_SYNC_STATE_STEADY_STATE
} capi_smart_sync_state_t;

typedef struct capi_smart_sync_circ_buf_t
{
	int8_t*     bufs_ptr;
	/* Circular buffer, one contiguous block. Data that is buffered up during buffering mode. */

	uint32_t    max_data_len_per_ch;
	/* Circular buffer size per channel. Calculated based on proc samples info. */

	uint32_t    actual_data_len_per_ch;
	/* total number of proc samples buffered per channel */

	uint32_t    read_index;
	/* read marker (used in drain mode) in the circular
	 * buffer, same for all channels */

	uint32_t    write_index;
	/* write (used in buffering mode) in the circular
	 * buffer, same for all channels */
}capi_smart_sync_circ_buf_t;

typedef enum capi_smart_sync_port_state_t {
   CAPI_PORT_STATE_CLOSED,
   CAPI_PORT_STATE_STOPPED,
   CAPI_PORT_STATE_STARTED
} capi_smart_sync_port_state_t;

typedef struct capi_smart_sync_cmn_port_t
{
   uint32_t                    index;
   /* Which index [0 to MAX_PORTS-1] corresponds to this port, since
    * primary/secondary is stored by ID not index. */

   capi_smart_sync_port_state_t state;
   /* Closed, stopped, started. Stopped and started are only used on input ports.
    * Secondary ports only use this to validate against multiple open/closes. */

} capi_smart_sync_cmn_port_t;

typedef struct capi_smart_sync_in_port_t
{
	capi_smart_sync_cmn_port_t             cmn;

   capi_media_fmt_v2_t                    media_fmt;
   /* Input media format for this port. IT is required that both ports are
    * configured to the same media format. This is stored per port to verify that. */

   capi_smart_sync_circ_buf_t circ_buf;

   bool_t                                 is_ts_valid;
   int64_t                                cur_out_buf_timestamp_us;

   bool_t                                 is_input_unconsumed;
   bool_t                                 zeros_were_padded;

   // Stored during EOF to calculate amount of timestamp discontinuity. If this value is small, zeros will be
   // padded to ensure continuous data (make up for data drop scenario).
   bool_t                                 is_next_expected_ts_valid;
   int64_t                                next_expected_ts;
} capi_smart_sync_in_port_t;

typedef struct capi_smart_sync_out_port_t
{
	capi_smart_sync_cmn_port_t cmn;
} capi_smart_sync_out_port_t;

/* TODO(CG): add description for each parameter in the struct */
typedef struct capi_smart_sync_t
{
   capi_t                             vtbl;

   capi_event_callback_info_t         cb_info;

   capi_heap_id_t                     heap_info;

   // Module configuration.
   capi_smart_sync_events_config_t    events_config;

   volatile uint64_t                  *vfr_timestamp_us_ptr;
   volatile uint32_t                  *resync_status_ptr;
   volatile bool_t                    *first_vfr_occurred_ptr;

   // Cached versions of each shared pointer. At the beginning of each process call, if we are subscribed
   // to vt we will update these values, thereby avoiding shared pointer reads throughout the process context.
   // This is necessary for handling of unsubscribing from vt while VPTX is still running, especially if
   // VPTX is in buffer trigger mode when the unsubscribe occurrs.
   uint64_t                           vfr_timestamp_at_cur_proc_tick;
   bool_t                             cached_first_vfr_occurred;

   uint32_t                           module_instance_id;

   uint32_t                           log_id;

   uint32_t                           threshold_us;
   uint32_t                           threshold_bytes_per_ch;

   // These variables are used to track where we are in the vfr cycle. When out_generated == out_required,
   // we know we reached the end of the vfr cycle.
   uint32_t                           out_required_per_vfr_cycle;
   uint32_t                           out_generated_this_vfr_cycle;
   uint32_t						      out_to_drop_from_next_vfr_cycle;

   // For TTR delivery. TODO(claguna): We can use out_generated_this_vfr_cycle for this.
   uint32_t                           num_bytes_copied_to_output;

   uint32_t                           current_packet_token;

   capi_smart_sync_state_t            state;

   smart_sync_voice_proc_info_t       voice_proc_info;
   /* voice processing config received from VCPM */

   // Port information.
   capi_smart_sync_in_port_t          primary_in_port_info;
   capi_smart_sync_out_port_t         primary_out_port_info;

   capi_smart_sync_in_port_t          secondary_in_port_info;
   capi_smart_sync_out_port_t         secondary_out_port_info;

   void*                              voice_proc_start_signal_ptr;
   /* signal ptr for voice timer expiry (for voice proc start tick) */

   void*                              voice_resync_signal_ptr;
   /* signal ptr for voice resync */

   intf_extn_param_id_metadata_handler_t metadata_handler;
   /* Metadata Handler Object*/

   container_trigger_policy_t          trigger_policy;

   bool_t                              is_threshold_disabled;

   bool_t                              skip_send_ttr_metadata;

   bool_t                              is_proc_tick_notif_rcvd;
   /*container notifies us of the first proc tick after which smart sync starts
   syncing since it should have enough data buffered.*/

   bool_t                              can_process;
   /* smart sync is supposed to generate output once per topo process.
    * if topo call smart-sync module multiple times due to bkwd kick processing then smart sync shouldn't generate more output more than once.
    * This is to avoid frame getting stuck in topo.
    */

   // event FWK_EXTN_VOICE_DELIVERY_EVENT_ID_UPDATE_SYNC_STATE client info
   uint64_t                            evt_dest_address;
   uint32_t                            evt_dest_token;

   bool_t                              received_eof; // Indicates on last process call EOF w/out EOS was received.
   bool_t                              disable_ts_disc_handling;  // Set this to FALSE to enable ts disc handling.

} capi_smart_sync_t;

/* clang-format on */
/* Check if we have set media format yet (on the primary port). This will be true if the primary port's
 * minor version is not invalid (among other things!). */
static inline bool_t capi_smart_sync_media_fmt_is_valid(capi_smart_sync_t *me_ptr, bool_t is_primary)
{
   return is_primary ? CAPI_DATA_FORMAT_INVALID_VAL != me_ptr->primary_in_port_info.media_fmt.format.num_channels
                     : CAPI_DATA_FORMAT_INVALID_VAL != me_ptr->secondary_in_port_info.media_fmt.format.num_channels;
}

/* Returns true if internal buffered data is drained for primary/secondary port. */
static inline bool_t capi_smart_sync_is_int_buf_empty(capi_smart_sync_t *me_ptr, bool_t is_primary)
{
   return is_primary ? 0 == me_ptr->primary_in_port_info.circ_buf.actual_data_len_per_ch
                     : 0 == me_ptr->secondary_in_port_info.circ_buf.actual_data_len_per_ch;
}

// Check if there is any input on the given input port.
static inline bool_t capi_smart_sync_input_has_data(capi_stream_data_t *input[], uint32_t input_index)
{
   return input[input_index] && input[input_index]->buf_ptr && input[input_index]->buf_ptr[0].actual_data_len;
}

/**
 * Check if a vfr cycle is complete by comparing the amount of output generated to amount of output required per vfr
 * cycle.
 */
static inline bool_t capi_smart_sync_vfr_cycle_is_complete(capi_smart_sync_t *me_ptr)
{
   return (me_ptr->out_generated_this_vfr_cycle >= me_ptr->out_required_per_vfr_cycle);
}

static inline bool_t capi_smart_sync_is_first_frame_of_vfr_cycle(capi_smart_sync_t *me_ptr)
{
   return (0 == me_ptr->out_generated_this_vfr_cycle);
}

static inline bool_t capi_smart_sync_should_move_to_steady_state(capi_smart_sync_t *me_ptr)
{
   return (me_ptr->secondary_in_port_info.zeros_were_padded && capi_smart_sync_vfr_cycle_is_complete(me_ptr));
}

static inline uint32_t align_to_8_byte(const uint32_t num)
{
   return ((num + 7) & (0xFFFFFFF8));
}

bool_t capi_smart_sync_can_subscribe_to_vt(capi_smart_sync_t *me_ptr);

capi_err_t capi_smart_sync_raise_process_event(capi_smart_sync_t *me_ptr);

capi_err_t capi_smart_sync_raise_event_toggle_threshold_n_sync_state( capi_smart_sync_t *me_ptr,
                                                                      bool_t enable_threshold);
capi_err_t capi_smart_sync_raise_event(capi_smart_sync_t *me_ptr);

capi_err_t capi_smart_sync_raise_event_change_container_trigger_policy(
   capi_smart_sync_t *        me_ptr,
   container_trigger_policy_t container_trigger_policy);

capi_err_t capi_smart_sync_pad_initial_zeroes(capi_smart_sync_t *me_ptr,
                                              bool_t             is_primary,
                                              uint32_t           expected_in_actual_data_len_per_ch);

capi_err_t capi_smart_sync_buffer_new_data(capi_smart_sync_t * me_ptr,
                                           capi_stream_data_t *input[],
                                           bool_t              is_primary,
                                           bool_t *            is_eof_dropped_ptr);

capi_err_t capi_smart_sync_underrun_secondary_port(capi_smart_sync_t * me_ptr,
                                                   capi_stream_data_t *output[]);

capi_err_t capi_smart_sync_output_buffered_data(capi_smart_sync_t * me_ptr,
                                                capi_stream_data_t *output[],
                                                bool_t              is_primary);

capi_err_t capi_smart_sync_pass_through_data(capi_smart_sync_t * me_ptr,
                                             capi_stream_data_t *input[],
                                             capi_stream_data_t *output[],
                                             bool_t              is_primary,
                                             bool_t             *is_eof_propagated_ptr);

capi_err_t capi_smart_sync_subscribe_to_voice_timer(capi_smart_sync_t *me_ptr);

capi_err_t capi_smart_sync_unsubscribe_to_voice_timer(capi_smart_sync_t *me_ptr);

void capi_smart_sync_deallocate_internal_circ_buf(capi_smart_sync_t *me_ptr, capi_smart_sync_in_port_t *in_port_ptr);

capi_err_t capi_smart_sync_allocate_internal_circ_buf(capi_smart_sync_t *        me_ptr,
                                                      capi_smart_sync_in_port_t *in_port_ptr);

bool_t capi_smart_sync_is_supported_media_type(const capi_media_fmt_v2_t *format_ptr);

capi_err_t capi_smart_sync_port_will_start(capi_smart_sync_t *me_ptr);

capi_err_t capi_smart_sync_set_properties_port_op(capi_smart_sync_t *me_ptr, capi_buf_t *payload_ptr);

capi_err_t capi_smart_sync_init_cmn_port(capi_smart_sync_t *me_ptr, capi_smart_sync_cmn_port_t *cmn_port_ptr);

capi_err_t capi_smart_sync_resync_module_state(capi_smart_sync_t *me_ptr);

capi_err_t capi_smart_sync_reset_module_state(capi_smart_sync_t *me_ptr, bool_t reset_buffer);

capi_err_t capi_smart_sync_circ_buf_adjust_for_overflow(capi_smart_sync_t *me_ptr, bool_t is_primary, uint32_t write_index);

// Timestamp discontinuity robustness handling
capi_err_t capi_smart_sync_check_handle_ts_disc(capi_smart_sync_t * me_ptr,
                                                capi_stream_data_t *input[],
                                                capi_stream_data_t *output[],
                                                bool_t *            skip_processing_ptr,
                                                bool_t *            eof_found_ptr);

capi_err_t capi_smart_sync_check_pad_ts_disc_zeros(capi_smart_sync_t * me_ptr,
                                                   capi_stream_data_t *input[],
                                                   capi_stream_data_t *output[]);

void capi_smart_sync_reset_buffers(capi_smart_sync_t *me_ptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif //_CAPI_SMART_SYNC_UTILS_H_
