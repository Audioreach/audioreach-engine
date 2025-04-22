/**
 * \file capi_priority_sync_i.h
 * \brief
 *        Internal header file for the Priority Sync module.
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef _CAPI_PRIORITY_SYNC_I_H_
#define _CAPI_PRIORITY_SYNC_I_H_

#ifndef CAPI_STANDALONE
/* For shared libraries. */
#include "shared_lib_api.h"
#else
#include "capi_util.h"
#endif

#include "capi.h"
#include "capi_priority_sync.h"
#include "capi_cmn.h"
#include "priority_sync_api.h"
#include "capi_fwk_extns_sync.h"
#include "capi_fwk_extns_container_frame_duration.h"
#include "capi_fwk_extns_trigger_policy.h"
#include "capi_intf_extn_prop_is_rt_port_property.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/** Macros */
#ifndef MIN
#define MIN(m, n) (((m) < (n)) ? (m) : (n))
#endif

#ifndef MAX
#define MAX(m, n) (((m) > (n)) ? (m) : (n))
#endif

/* debug message */
#define MIID_UNKNOWN 0
#define PS_MSG_PREFIX "[%lX]: "
#define PS_MSG(ID, xx_ss_mask, xx_fmt, ...) AR_MSG(xx_ss_mask, PS_MSG_PREFIX xx_fmt, ID, ##__VA_ARGS__)

// Constant used to check if port id/index mapping is valid.
#define PRIORITY_SYNC_PORT_INDEX_INVALID 0xFFFFFFFF

// FWK_EXTN_SYNC,FWK_EXTN_CONTAINER_FRAME_DURATION, FWK_EXTN_TRIGGER_POLICY
#define PRIORITY_SYNC_NUM_FRAMEWORK_EXTENSIONS 3

// Upstream should be sending 1ms frames. Error in sync time is +- 1 frame.
#define PRIORITY_SYNC_UPSTREAM_FRAME_SIZE_US 1000

//maximum delay added to time synchronize the primary and secondary paths.
#define PRIORITY_SYNC_TS_SYNC_WINDOW_US (5000)

#ifdef USES_DEBUG_DEV_ENV
// Enable for debug logs.
#define CAPI_PRIORITY_SYNC_DEBUG TRUE
#endif

/*------------------------------------------------------------------------
 * Function declarations
 * -----------------------------------------------------------------------*/

typedef enum capi_priority_sync_state_t {
   PRIORITY_SYNC_STATE_SYNCED,
   PRIORITY_SYNC_STATE_STARTING
} capi_priority_sync_state_t;

typedef enum capi_priority_sync_flow_state_t {
   PRIORITY_SYNC_FLOW_STATE_AT_GAP = 0,
   PRIORITY_SYNC_FLOW_STATE_FLOWING
} capi_priority_sync_flow_state_t;

typedef struct capi_priority_sync_config_t
{
   uint32_t threshold_us; // Container's external input threshold.
} capi_priority_sync_config_t;

typedef enum capi_priority_sync_is_rt_state_t {
   PRIORITY_SYNC_FTRT = 0, // Default
   PRIORITY_SYNC_RT
} capi_priority_sync_is_rt_state_t;

// structure to store rt/nrt port property.
typedef struct capi_priority_sync_data_port_property_t
{
   capi_priority_sync_is_rt_state_t us_rt; // upstream RT/FTRT
   capi_priority_sync_is_rt_state_t ds_rt; // downstream RT/FTRT
} capi_priority_sync_data_port_property_t;

typedef struct capi_priority_sync_cmn_port_t
{
   uint32_t index; // Which index [0 to MAX_PORTS-1] corresponds to this port,
                   // since primary/secondary is stored by ID not index.
   intf_extn_data_port_state_t     state;
   capi_priority_sync_flow_state_t flow_state;

   capi_priority_sync_data_port_property_t prop_state;

} capi_priority_sync_cmn_port_t;

typedef union capi_priority_sync_in_port_proc_ctx_flags_t
{
   struct
   {
      uint32_t
         proc_ctx_sent_eos_dfg : 1;  // Cleared every process call, stores whether eos/dfg was sent during this process
                                     // call on this stream or not.
      uint32_t proc_ctx_has_eos : 1; // Cleared every process call, stores whether eos was found on the input stream.
      uint32_t proc_ctx_has_eof : 1; // Cleared every process call, stores whether eof was found on the input stream.
      uint32_t proc_ctx_has_dfg : 1; // Cleared every process call, stores whether dfg was found on the input stream.

      uint32_t proc_ctx_sec_cleared_eos_dfg : 1; // Cleared every process call, stores whether eos or dfg was found on
                                                 // secondary port and removed from the metadata list at the beginning
                                                 // of the process call.
   };
   uint32_t word;
} capi_priority_sync_in_port_proc_ctx_flags_t;

typedef struct capi_priority_sync_in_port_t
{
   capi_priority_sync_cmn_port_t cmn; // Must be first field so we can cast common port to input port.

   capi_stream_data_v2_t int_stream; // capi stream structure to hold internal buffer, flags, timestamp and metadata.

   capi_media_fmt_v2_t media_fmt;              // Input media format for this port.
   uint32_t            threshold_bytes_per_ch; // Threshold in bytes per channel.
   uint32_t            delay_bytes_per_ch;
   /* ^^
    * for non-ts based sync:
    * 		extra 1ms space is added in secondary path to account for rt jitter between primary upstream and secondary
    * 		upstream. here assuming that upstream are running on 1ms frame size.
    *
    * for ts based sync:
    * 		extra space for the delay added during timestamp based synchronization.
    */

   capi_priority_sync_in_port_proc_ctx_flags_t flags;

   bool_t data_continous; // flag indicates if data-flow is continuous, if true then discontinuity
                          // by appending zeros can't be added.
} capi_priority_sync_in_port_t;

typedef struct capi_priority_sync_out_port_t
{
   capi_priority_sync_cmn_port_t cmn; // Must be first field so we can cast common port to output port.

   // Output media format is the same as input media format (primary in -> primary out, secondary in -> seconadary out),
   // so no need to store here.
} capi_priority_sync_out_port_t;

typedef enum capi_priority_sync_tg_policy_t {
   PRIORITY_SYNC_DEFAULT_TRIGGER = 0,
   PRIORITY_SYNC_PRIMARY_OPTIONAL,
   PRIORITY_SYNC_SECONDARY_OPTIONAL,
   PRIORITY_SYNC_ALL_OPTIONAL = (PRIORITY_SYNC_PRIMARY_OPTIONAL + PRIORITY_SYNC_SECONDARY_OPTIONAL),
   PRIORITY_SYNC_ALL_MANDATORY
} capi_priority_sync_tg_policy_t;

typedef struct capi_priority_sync_t
{
   capi_t                     vtbl;
   capi_event_callback_info_t cb_info;
   capi_heap_id_t             heap_info;

   // Module configuration.
   capi_priority_sync_config_t        module_config;

   // Port information.
   capi_priority_sync_in_port_t  primary_in_port_info;
   capi_priority_sync_out_port_t primary_out_port_info;

   capi_priority_sync_in_port_t  secondary_in_port_info;
   capi_priority_sync_out_port_t secondary_out_port_info;

   capi_priority_sync_state_t synced_state;          // Syncing, steady state, closing.

   // Metadata interface extension info.
   intf_extn_param_id_metadata_handler_t metadata_handler;

   // FWK Trigger policy callback handle.
   fwk_extn_param_id_trigger_policy_cb_fn_t tg_policy_cb;
   capi_priority_sync_tg_policy_t           tg_policy;

   bool_t   threshold_is_disabled;  // Checks if priority sync has disabled or enabled the threshold.
   bool_t   avoid_sec_buf_overflow; // flag to avoid buffer overflow on secondary path when primary is not running.
   bool_t is_ts_based_sync;
   uint32_t miid;
} capi_priority_sync_t;

capi_err_t capi_priority_sync_process_set_properties(capi_priority_sync_t *me_ptr, capi_proplist_t *proplist_ptr);
capi_err_t capi_priority_sync_process_get_properties(capi_priority_sync_t *me_ptr, capi_proplist_t *proplist_ptr);

capi_err_t capi_priority_sync_init_cmn_port(capi_priority_sync_t *me_ptr, capi_priority_sync_cmn_port_t *cmn_port_ptr);

// capi_priority_sync_control_utils function declarations.
capi_err_t capi_priority_sync_port_will_start(capi_priority_sync_t *me_ptr);
capi_err_t capi_priority_sync_in_port_flow_gap(capi_priority_sync_t *me_ptr, bool_t is_primary);

capi_err_t capi_priority_sync_handle_is_rt_property(capi_priority_sync_t *me_ptr);

intf_extn_data_port_state_t capi_priority_sync_get_downgraded_port_state(capi_priority_sync_t *         me_ptr,
                                                                         capi_priority_sync_cmn_port_t *port_cmn_ptr);

// capi_priority_sync_data_utils function declarations.
capi_err_t capi_priority_sync_allocate_port_buffer(capi_priority_sync_t *        me_ptr,
                                                   capi_priority_sync_in_port_t *in_port_ptr);
void capi_priority_sync_deallocate_port_buffer(capi_priority_sync_t *me_ptr, capi_priority_sync_in_port_t *in_port_ptr);
capi_err_t capi_priority_sync_clear_buffered_data(capi_priority_sync_t *me_ptr, bool_t primary_path);

capi_err_t priority_sync_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[]);
capi_err_t priority_ts_sync_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[]);

//md related functions
void capi_priority_sync_adj_offset(capi_media_fmt_v2_t * med_fmt_ptr,
                                   module_cmn_md_list_t *md_list_ptr,
                                   uint32_t              bytes_consumed,
                                   bool_t                true_add_false_sub);
void capi_priority_sync_do_md_offset_math(uint32_t *           offset_ptr,
                                          uint32_t             bytes,
                                          capi_media_fmt_v2_t *med_fmt_ptr,
                                          bool_t               need_to_add);
capi_err_t capi_priority_sync_destroy_md_within_range(capi_priority_sync_t * me_ptr,
                                                      capi_media_fmt_v2_t *  med_fmt_ptr,
                                                      module_cmn_md_list_t **md_list_pptr,
                                                      uint32_t               offset_end);
capi_err_t capi_priority_sync_destroy_md_list(capi_priority_sync_t *me_ptr, module_cmn_md_list_t **md_list_pptr);
capi_err_t capi_priority_sync_manual_metadata_prop_single_port(capi_priority_sync_t *        me_ptr,
                                                               capi_priority_sync_in_port_t *in_port_ptr,
                                                               capi_stream_data_v2_t *       in_stream_ptr,
                                                               capi_stream_data_v2_t *       out_stream_ptr);

// capi_priority_sync_event_utils function declarations.
capi_err_t capi_priority_sync_raise_event(capi_priority_sync_t *me_ptr);

capi_err_t capi_priority_sync_raise_event_toggle_threshold(capi_priority_sync_t *me_ptr, bool_t enable_threshold);

capi_err_t priority_sync_update_port_data_threshold_event(capi_priority_sync_t *      me_ptr,
                                                          capi_event_callback_info_t *cb_info_ptr,
                                                          capi_media_fmt_v2_t *       media_fmt,
                                                          bool_t                      is_input_port,
                                                          uint32_t                    port_index);

capi_err_t capi_priority_sync_handle_tg_policy(capi_priority_sync_t *me_ptr);

//process helper functions
capi_err_t priority_sync_check_for_started_ports(capi_priority_sync_t *me_ptr,
                                                 capi_stream_data_t *  input[],
                                                 capi_stream_data_t *  output[],
                                                 bool_t *              any_data_found_ptr);

capi_err_t capi_priority_sync_mark_input_unconsumed(capi_priority_sync_t *me_ptr,
                                                    capi_stream_data_t *  input[],
                                                    bool_t                is_primary);

capi_err_t capi_priority_sync_buffer_new_data(capi_priority_sync_t *me_ptr,
                                              capi_stream_data_t *  input[],
                                              bool_t                buffer_primary,
                                              bool_t                buffer_secondary);

capi_err_t capi_priority_sync_send_buffered_data(capi_priority_sync_t *     me_ptr,
                                                 capi_stream_data_t *       output[],
                                                 capi_priority_sync_state_t synced_state);

capi_err_t capi_priority_sync_check_handle_flow_gap(capi_priority_sync_t *me_ptr,
                                                    capi_stream_data_t *  input[],
                                                    capi_stream_data_t *  output[]);



static inline capi_priority_sync_cmn_port_t *capi_priority_sync_get_port_cmn(capi_priority_sync_t *me_ptr,
                                                                             bool_t                is_primary,
                                                                             bool_t                is_input)
{
   capi_priority_sync_cmn_port_t *port_cmn_ptr = NULL;
   if (is_primary)
   {
      if (is_input)
      {
         port_cmn_ptr = &me_ptr->primary_in_port_info.cmn;
      }
      else
      {
         port_cmn_ptr = &me_ptr->primary_out_port_info.cmn;
      }
   }
   else
   {
      if (is_input)
      {
         port_cmn_ptr = &me_ptr->secondary_in_port_info.cmn;
      }
      else
      {
         port_cmn_ptr = &me_ptr->secondary_out_port_info.cmn;
      }
   }
   return port_cmn_ptr;
}

// Check if primary is running or secondary is running (input port is in started state)
static inline bool_t capi_priority_sync_is_path_running(capi_priority_sync_t *me_ptr, bool_t is_primary)
{
   return is_primary ? (DATA_PORT_STATE_STARTED ==
                        capi_priority_sync_get_downgraded_port_state(me_ptr, &me_ptr->primary_in_port_info.cmn))
                     : (DATA_PORT_STATE_STARTED ==
                        capi_priority_sync_get_downgraded_port_state(me_ptr, &me_ptr->secondary_in_port_info.cmn));
}

// Check if we have set media format yet (on the primary port). This will be true if the primary port's
// minor version is not invalid (among other things!).
static inline bool_t capi_priority_sync_media_fmt_is_valid(capi_priority_sync_t *me_ptr, bool_t is_primary)
{
   return is_primary ? CAPI_DATA_FORMAT_INVALID_VAL != me_ptr->primary_in_port_info.media_fmt.format.num_channels
                     : CAPI_DATA_FORMAT_INVALID_VAL != me_ptr->secondary_in_port_info.media_fmt.format.num_channels;
}

// Check if there is any input on the given input port.
static inline bool_t capi_priority_sync_input_has_data(capi_stream_data_t *input[], uint32_t input_index)
{
   return input[input_index] && input[input_index]->buf_ptr && input[input_index]->buf_ptr[0].actual_data_len;
}

static inline uint32_t align_to_8_byte(const uint32_t num)
{
   return ((num + 7) & (0xFFFFFFF8));
}

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif //_CAPI_PRIORITY_SYNC_I_H_
