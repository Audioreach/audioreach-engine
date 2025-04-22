/**
 * \file capi_gapless_i.h
 * \brief
 *        Header file to implement the gapless module.
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef CAPI_GAPLESS_I_H
#define CAPI_GAPLESS_I_H

#ifndef CAPI_STANDALONE

/* For shared libraries. */
#include "shared_lib_api.h"
#else
#include "capi_util.h"

#endif

#include "capi.h"
#include "capi_gapless.h"
#include "capi_cmn.h"
#include "capi_intf_extn_metadata.h"
#include "other_metadata.h"
#include "gapless_api.h"
#include "module_cmn_api.h"
#include "spf_circular_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

// Constant used to check if port id/index mapping is valid.
#define GAPLESS_PORT_INDEX_INVALID (0xFFFFFFFF)
#define GAPLESS_PORT_ID_INVALID (0)
#define GAPLESS_NUM_PORTS_INVALID (0xFFFFFFFF)
#define GAPLESS_INVALID_CNTR_FRAME_SIZE (0xFFFFFFFF)

// TODO(claguna): Determine these
#define GAPLESS_BW (1 * 1024 * 1024)
#define GAPLESS_KPPS (30)

// #define CAPI_GAPLESS_DEBUG TRUE

// Preferred chunk size of the fragmented circular buffer
#define GAPLESS_PREFERRED_CHUNK_SIZE (2048)

#define GAPLESS_MAX_EARLY_EOS_DELAY_MS (500)

/*------------------------------------------------------------------------
 * Function declarations
 * -----------------------------------------------------------------------*/
typedef struct capi_gapless_events_config_t
{
   uint32_t enable;
   uint32_t kpps;
   uint32_t delay_in_us;
   uint32_t code_bw;
   uint32_t data_bw;
} capi_gapless_events_config_t;

typedef enum capi_gapless_port_trigger_policy_t {
   PORT_TRIGGER_POLICY_PASS_THROUGH, // (input 0 || input 1) && (output)
   PORT_TRIGGER_POLICY_LISTEN,       // Accept data (Optional).
   PORT_TRIGGER_POLICY_DONT_LISTEN,  // Don't accept data (inputs: Nontriggerable-Blocked, outputs:
                                     // Nontriggerable-Optional).
   PORT_TRIGGER_POLICY_CLOSED // Port is closed, so marked as INVALID in nontriggerable group and NONE in triggerable
                              // group.
} capi_gapless_port_trigger_policy_t;

typedef struct capi_gapless_cmn_port_t
{
   uint32_t                    port_id; // Value of port ID provided in the module definition
   uint32_t                    index;   // Which index [0 to MAX_PORTS-1] corresponds to this port
   intf_extn_data_port_state_t state;   // Closed, stopped, started. Stopped and started are only used on input ports.
   capi_gapless_port_trigger_policy_t port_trigger_policy;

} capi_gapless_cmn_port_t;

typedef struct capi_gapless_in_port_t
{
   capi_gapless_cmn_port_t cmn;                // Must be first field so we can cast common port to input port.
   capi_media_fmt_v2_t     media_fmt;          // Media format of the newest data in the circular buffer.
   void *                  sdata_circ_buf_ptr; // Internal delay buffer. TODO(claguna)

   // delay buffer handles
   spf_circ_buf_t *       stream_drv_ptr; // Stream buffer driver handle
   spf_circ_buf_client_t *writer_handle;  // writer handle, used for buffering stream into delay buffer.
   spf_circ_buf_client_t *reader_handle;  // reader handle used for reading data from the delay buffer.

   // Used to avoid printing that timestamps become invalid every process call.
   bool_t found_valid_timestamp;
} capi_gapless_in_port_t;

typedef struct capi_gapless_out_port_t
{
   capi_gapless_cmn_port_t cmn; // Must be first field so we can cast common port to output port.

} capi_gapless_out_port_t;

/**
 * Pre-allocated trigger policy structures.
 */
typedef struct capi_gapless_trigger_policy_t
{
   fwk_extn_port_nontrigger_policy_t in_port_nontrigger_policies[GAPLESS_MAX_INPUT_PORTS];
   fwk_extn_port_nontrigger_policy_t out_port_nontrigger_policies[GAPLESS_MAX_OUTPUT_PORTS];

   fwk_extn_port_trigger_affinity_t in_port_trigger_policies[GAPLESS_MAX_INPUT_PORTS];
   fwk_extn_port_trigger_affinity_t out_port_trigger_policies[GAPLESS_MAX_OUTPUT_PORTS];

   // Affinities will get init to 0 which are FWK_EXTN_PORT_TRIGGER_AFFINITY_NONE. Used for
   // pass_through_mode.
   fwk_extn_port_trigger_affinity_t none_in_port_trigger_policies[GAPLESS_MAX_INPUT_PORTS];
   fwk_extn_port_trigger_affinity_t none_out_port_trigger_policies[GAPLESS_MAX_OUTPUT_PORTS];

   fwk_extn_port_trigger_affinity_t  default_trigger_affinity;
   fwk_extn_port_nontrigger_policy_t default_nontrigger_policy;

} capi_gapless_trigger_policy_t;

typedef struct capi_gapless_t
{
   capi_t                                vtbl;
   capi_event_callback_info_t            cb_info;
   capi_heap_id_t                        heap_info;
   capi_gapless_in_port_t                in_ports[GAPLESS_MAX_INPUT_PORTS];
   capi_gapless_out_port_t               out_ports[GAPLESS_MAX_OUTPUT_PORTS];
   capi_gapless_events_config_t          events_config;
   intf_extn_param_id_metadata_handler_t metadata_handler;

   uint32_t num_in_ports;         // Usually would be 2, could be 1.
   uint32_t num_out_ports;        // Must be 1.
   uint32_t active_in_port_index; // Port index (cmn.index) of the active input port.

   // Holds the callback function to invoke when changing the trigger policy.
   fwk_extn_param_id_trigger_policy_cb_fn_t trigger_policy_cb_fn;
   capi_gapless_trigger_policy_t            trigger_policy;
   bool_t                                   sent_trigger_policy_at_least_once;

   bool_t pass_through_mode; // Config for delay buffer never came, copy input directly from sdata to output.

   bool_t   client_registered;  // TRUE if a client registered for the early eos event.
   uint64_t event_dest_address; // Registration info for the early eos event.
   uint32_t event_token;        // Registration info for the early eos event.
   uint32_t early_eos_delay_ms; // Client configured early eos delay in ms.

   // Media format that's currently being output. Comes from the media format of the active input.
   capi_media_fmt_v2_t operating_media_fmt;
   // If we switch streams when inputs have differing media formats, we need to hold off on raising output media
   // format until the next process call since we can only raise events when not producing output.
   bool_t has_pending_operating_media_format;

   // The container frame size. Needed for the stream buffer configuration.
   uint32_t cntr_frame_size_us;

   // Need to send reset session time metadata on the process call (where output is generated) after eos is sent on the
   // output.
   bool_t sent_eos_this_process_call;
   bool_t send_rst_md;

   bool_t trigger_policy_sent_once;
   bool_t is_gapless_cntr_duty_cycling;
} capi_gapless_t;

static inline uint32_t align_to_8_byte(const uint32_t num)
{
   return ((num + 7) & (0xFFFFFFF8));
}

/**----------------------------- capi_gapless.cpp -------------------------------*/
void capi_gapless_init_config(capi_gapless_t *me_ptr);

/**---------------------- capi_gapless_control_utils.cpp ------------------------*/
bool_t capi_gapless_is_supported_media_type(capi_media_fmt_v2_t *format_ptr);
bool_t capi_gapless_should_set_operating_media_format(capi_gapless_t *me_ptr, capi_gapless_in_port_t *in_port_ptr);
capi_err_t capi_gapless_get_properties(capi_t *_pif, capi_proplist_t *props_ptr);
capi_err_t capi_gapless_set_properties(capi_t *_pif, capi_proplist_t *props_ptr);

capi_err_t capi_gapless_get_param(capi_t *                _pif,
                                  uint32_t                param_id,
                                  const capi_port_info_t *port_info_ptr,
                                  capi_buf_t *            params_ptr);

capi_err_t capi_gapless_set_param(capi_t *                _pif,
                                  uint32_t                param_id,
                                  const capi_port_info_t *port_info_ptr,
                                  capi_buf_t *            params_ptr);

capi_err_t gapless_check_create_delay_buffer(capi_gapless_t *me_ptr);
capi_err_t gapless_check_update_trigger_policy(capi_gapless_t *me_ptr);
capi_err_t gapless_send_trigger_policy(capi_gapless_t *me_ptr, bool_t all_policies_closed);
capi_err_t capi_gapless_set_operating_media_format(capi_gapless_t *me_ptr, capi_media_fmt_v2_t *media_fmt_ptr);

/**----------------------- capi_gapless_data_utils.cpp --------------------------*/
bool_t gapless_sdata_has_data(capi_gapless_t *me_ptr, capi_stream_data_v2_t *sdata_ptr);
capi_err_t gapless_process(capi_gapless_t *me_ptr, capi_stream_data_t *input[], capi_stream_data_t *output[]);
capi_err_t gapless_pass_through(capi_gapless_t *        me_ptr,
                                capi_gapless_in_port_t *in_port_ptr,
                                capi_stream_data_v2_t * in_sdata_ptr,
                                capi_stream_data_v2_t * out_sdata_ptr,
                                module_cmn_md_t **      eos_md_pptr);
capi_err_t gapless_buffer_input(capi_gapless_t *me_ptr, capi_stream_data_t *input[]);
capi_err_t gapless_write_output(capi_gapless_t *me_ptr, capi_stream_data_t *input[], capi_stream_data_t *output[]);
bool_t capi_gapless_other_stream_has_data(capi_gapless_t *me_ptr, capi_gapless_in_port_t *other_in_port_ptr);
module_cmn_md_t *gapless_find_eos(capi_stream_data_v2_t *in_sdata_ptr);
capi_err_t gapless_propagate_metadata(capi_gapless_t *       me_ptr,
                                      capi_stream_data_v2_t *in_stream_ptr,
                                      capi_media_fmt_v2_t *  in_media_fmt_ptr,
                                      capi_stream_data_v2_t *out_stream_ptr,
                                      bool_t                 is_input_fully_consumed,
                                      uint32_t               in_size_before);
capi_err_t capi_gapless_destroy_md_list(capi_gapless_t *me_ptr, module_cmn_md_list_t **md_list_pptr);
capi_err_t gapless_setup_output_sdata(capi_gapless_t *       me_ptr,
                                      capi_stream_data_v2_t *temp_sdata_ptr,
                                      capi_stream_data_v2_t *out_sdata_ptr,
                                      uint32_t *             prev_actual_data_len_per_ch_ptr);
capi_err_t gapless_adjust_output_sdata(capi_gapless_t *       me_ptr,
                                       capi_stream_data_v2_t *temp_sdata_ptr,
                                       capi_stream_data_v2_t *out_sdata_ptr,
                                       uint32_t               prev_actual_data_len_per_ch);
void gapless_metadata_adj_offset(capi_media_fmt_v2_t * med_fmt_ptr,
                                 module_cmn_md_list_t *md_list_ptr,
                                 uint32_t              bytes_consumed,
                                 bool_t                true_add_false_sub);
void gapless_do_md_offset_math(uint32_t *           offset_ptr,
                               uint32_t             bytes,
                               capi_media_fmt_v2_t *med_fmt_ptr,
                               bool_t               need_to_add);

/**----------------------- capi_gapless_event_utils.cpp -------------------------*/
capi_err_t gapless_raise_event(capi_gapless_t *me_ptr);
capi_err_t gapless_raise_process_event(capi_gapless_t *me_ptr);
capi_err_t gapless_raise_early_eos_event(capi_gapless_t *        me_ptr,
                                         capi_gapless_in_port_t *in_port_ptr,
                                         capi_stream_data_v2_t * in_sdata_ptr,
                                         module_cmn_md_t *       md_ptr);
capi_err_t gapless_raise_allow_duty_cycling_event(capi_gapless_t *me_ptr, bool_t allow_duty_cycling);

/**-------------------- capi_gapless_delay_buffer_utils.cpp ---------------------*/
bool_t capi_gapless_does_delay_buffer_exist(capi_gapless_t *me_ptr, capi_gapless_in_port_t *in_port_ptr);
bool_t capi_gapless_is_delay_buffer_empty(capi_gapless_t *me_ptr, capi_gapless_in_port_t *in_port_ptr);
bool_t capi_gapless_is_delay_buffer_full(capi_gapless_t *me_ptr, capi_gapless_in_port_t *in_port_ptr);
capi_err_t capi_gapless_destroy_delay_buffer(capi_gapless_t *me_ptr, capi_gapless_in_port_t *in_port_ptr);

capi_err_t capi_gapless_get_delay_buffer_media_fmt(capi_gapless_t *        me_ptr,
                                                   capi_gapless_in_port_t *in_port_ptr,
                                                   capi_media_fmt_v2_t *   ret_mf_ptr);

capi_err_t capi_gapless_set_delay_buffer_media_fmt(capi_gapless_t *        me_ptr,
                                                   capi_gapless_in_port_t *in_port_ptr,
                                                   capi_media_fmt_v2_t *   media_fmt_ptr,
                                                   uint32_t                cntr_frame_size_us);

capi_err_t capi_gapless_create_delay_buffer(capi_gapless_t *        me_ptr,
                                            capi_gapless_in_port_t *in_port_ptr,
                                            capi_media_fmt_v2_t *   media_fmt_ptr,
                                            uint32_t                size_ms);
capi_err_t gapless_read_delay_buffer(capi_gapless_t *        me_ptr,
                                     capi_gapless_in_port_t *in_port_ptr,
                                     capi_stream_data_v2_t * out_sdata_ptr,
                                     module_cmn_md_t **      sent_eos_md_pptr);

capi_err_t gapless_write_delay_buffer(capi_gapless_t *        me_ptr,
                                      capi_gapless_in_port_t *in_port_ptr,
                                      capi_stream_data_v2_t * in_sdata_ptr);

capi_err_t gapless_circular_buffer_event_cb(void *          context_ptr,
                                            spf_circ_buf_t *circ_buf_ptr,
                                            uint32_t        event_id,
                                            void *          event_info_ptr);

/**------------------------ capi_gapless_port_utils.cpp -------------------------*/
capi_err_t capi_gapless_init_all_ports(capi_gapless_t *me_ptr);
bool_t capi_gapless_port_id_is_valid(capi_gapless_t *me_ptr, uint32_t port_id, bool_t is_input);
capi_gapless_in_port_t *capi_gapless_get_in_port_from_index(capi_gapless_t *me_ptr, uint32_t port_index);
capi_gapless_cmn_port_t *capi_gapless_get_port_cmn_from_index(capi_gapless_t *me_ptr, uint32_t index, bool_t is_input);
capi_gapless_out_port_t *capi_gapless_get_out_port(capi_gapless_t *me_ptr);
capi_gapless_in_port_t *capi_gapless_get_active_in_port(capi_gapless_t *me_ptr);
capi_gapless_in_port_t *capi_gapless_get_other_in_port(capi_gapless_t *me_ptr);
bool_t capi_gapless_is_in_port_active(capi_gapless_t *me_ptr, capi_gapless_in_port_t *in_port_ptr);
capi_err_t capi_gapless_check_assign_new_active_in_port(capi_gapless_t *me_ptr, capi_stream_data_t *input[]);
capi_err_t capi_gapless_port_open(capi_gapless_t *me_ptr, bool_t is_input, uint32_t port_index, uint32_t port_id);
capi_err_t capi_gapless_port_close(capi_gapless_t *me_ptr, uint32_t port_index, bool_t is_input);
capi_err_t capi_gapless_port_start(capi_gapless_t *me_ptr, uint32_t port_index, bool_t is_input);
capi_err_t capi_gapless_port_stop(capi_gapless_t *me_ptr, uint32_t port_index, bool_t is_input);

// clang-format on

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif // CAPI_GAPLESS_I_H
