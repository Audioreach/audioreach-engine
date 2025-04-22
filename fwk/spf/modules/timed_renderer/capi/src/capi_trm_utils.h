/**
 *   \file capi_trm_utils.h
 *   \brief
 *        Header file of utilities for TRM
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef CAPI_TRM_UTILS_H
#define CAPI_TRM_UTILS_H

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#ifndef CAPI_STANDALONE
/* For shared libraries. */
#include "shared_lib_api.h"
#else
#include "capi_util.h"
#endif
#include "capi.h"
#include "capi_cmn.h"
#include "capi_intf_extn_metadata.h"
#include "capi_intf_extn_data_port_operation.h"
#include "module_cmn_api.h"
#include "other_metadata.h"
#include "spf_list_utils.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

// 86 is determined in the following way:
// 1. Maximum amount of data required to hold in worst case scenario.
//    - worst case scenario is CDRX 40 case with 40ms processing jitter
//    - using formula below, 84 ms is found
//    max_hold(period, hold) = (duration of # total frames arrived by steady state) - (data rendered by steady state)
//
//   duration of # total frames arrived by steady state = (ceil(hold/period) + 1) * max_of_variable_frame_size
//   data rendered by steady state = (ceil(hold/period) * period) - hold
//   Check with claguna for questions.
//
// 2. 2ms for zero padding done within TRM
//    - 1ms for rt jitter prebuffer
//    - 1ms for worst case partial hold
#define NUM_FRAMES_IN_LOCAL_BUFFER (86 + 1) //+1 is for regular prebuffer (rate matching jitter)

#define CAPI_TRM_KPPS (1)    // TODO: profile and update kpps
#define CAPI_TRM_CODE_BW (0) // TODO: profile and update bw
#define CAPI_TRM_DATA_BW (0) // TODO: profile and update bw
#define CAPI_TRM_JITTER_TOLERANCE 100 // Jitter tolerance of 100us when wall clock is past ttr

//#define TRM_DEBUG

typedef enum capi_trm_render_decision_t {
   CAPI_TRM_DECISION_PENDING = 0,
   CAPI_TRM_DECISION_HOLD,
   CAPI_TRM_DECISION_RENDER,
   CAPI_TRM_DECISION_DROP,
} capi_trm_render_decision_t;

typedef enum capi_trm_tgp_state_t {
   CAPI_TRM_TGP_PENDING = 0,
   CAPI_TRM_TGP_BEFORE_SYNC,
   CAPI_TRM_TGP_AFTER_SYNC
} capi_trm_tgp_state_t;

typedef struct capi_trm_tgp_t
{
   fwk_extn_param_id_trigger_policy_cb_fn_t tg_policy_cb;
} capi_trm_tgp_t;

typedef struct capi_trm_circ_buf_frame_t
{
   capi_stream_data_v2_t sdata;
   uint16_t              actual_data_len_per_ch;
   uint16_t              read_offset;
   /* filled bytes in the frame */
} capi_trm_circ_buf_frame_t;

typedef struct capi_trm_circ_buf_t
{
   capi_trm_circ_buf_frame_t *frame_ptr;
   /* Circular buffer, per channel. Data that is buffered up during buffering mode. */

   uint32_t frame_len_per_ch;

   uint32_t read_index;
   /* read marker in the circular buffer, same for all channels */

   uint32_t write_index;
   /* write in the circular buffer, same for all channels */

   bool_t is_eos_buffered;
   /* True while there is eos in the pipeline*/

   uint32_t actual_data_len_all_ch;
   /* bytes per channel currently buffered in the circular buffer. */
} capi_trm_circ_buf_t;

/* CAPI structure  */
typedef struct capi_trm_t
{
   const capi_vtbl_t *vtbl;
   /* pointer to virtual table */

   capi_event_callback_info_t event_cb_info;
   /* Event Call back info received from Framework*/

   capi_heap_id_t heap_mem;
   /* Heap id received from framework*/

   capi_media_fmt_v2_t media_format;
   /* Media format set from upstream module */

   bool_t is_input_mf_received;
   /* flag to determine if input media format has been received */

   uint64_t curr_ttr;
   /* current ttr value */

   uint64_t wall_clock_at_trigger;
   /*wall clock time at recent signal trigger.*/

   bool_t first_ttr_received;
   /* flag to check if first ttr has been received */

   capi_trm_render_decision_t render_decision;
   /* Current ttr decision has been render. This means we will continue to render until next TTR comes. */

   intf_extn_param_id_metadata_handler_t metadata_handler;
   /* handler for metadata operations */

   // module_cmn_md_list_t *int_md_list_ptr;
   /* internal metadata list for propagation from input to held buffer: NOT needed. Although we report some algo delay,
    * it is handed as buffering delay from within the module therefore algo delay passed to metadata_propagate is always
    * 0.
    */

   capi_trm_circ_buf_t held_input_buf;
   /* local buffer to hold input data in module */

   capi_trm_tgp_t tgp;
   /* Trigger policy structure */

   uint32_t frame_size_us;
   /* Frame size, also equal to STM period of host container - cached from NOMINAL_FRAME_DURATION fwk extn*/

   uint32_t total_zeros_to_pad_us;
   /*
    * How many zeros to pad before sending first data of a new stream. TRM needs to pad zeros for two reasons:
    * 1. 1 frame worth of zeros for prebuffering due to upstream container providing variable sized data (fixed-in
    * sample slip in VPRX)
    * 2. Precise holding time - TRM will start rendering data halfway through a frame if possible.
    */

   uint32_t remaining_zeros_to_pad_us;
   /*
    * Remaining zeros to pad, needed since we may need to pad > 1 frame of zeros.
    */

   uint32_t input_bytes_to_drop_per_ch;
   /* Due to early VFR (resync), packet reaches earlier than expected so to align the rendering time
    * we need to drop some data from the new packet.
    */

   capi_trm_tgp_state_t tgp_state;

   uint32_t process_count;
   uint64_t prev_err_time;

   // Uint8s to save memory. Something is wrong if err count rises above 255 in 10ms.
   uint8_t bad_args_err_count;
   uint8_t no_held_buf_err_count;
   uint8_t output_not_empty_err_count;
   uint8_t not_enough_op_err_count;
   uint8_t unexp_render_decision_err_count;
   uint8_t unexp_underrun_err_count;
   /* Used to print steady state messages only once every 20 process calls. */
} capi_trm_t;

// Used in process context to tell which flags propagated to the output.
typedef struct capi_trm_propped_flags_t
{
   uint32_t marker_eos : 1;
   // uint32_t end_of_frame : 1;
} capi_trm_propped_flags_t;

static inline bool_t capi_trm_is_held_input_buffer_empty(capi_trm_t *me_ptr)
{
   return me_ptr->held_input_buf.read_index == me_ptr->held_input_buf.write_index;
}

static inline uint32_t align_to_8_byte(const uint32_t num)
{
   return ((num + 7) & (0xFFFFFFF8));
}

/*------------------------------------------------------------------------
 * Function Definitions
 * -----------------------------------------------------------------------*/
capi_err_t capi_trm_drop_all_metadata(capi_trm_t *me_ptr, capi_stream_data_t *input[]);

capi_err_t capi_trm_handle_metadata_b4_process(capi_trm_t *        me_ptr,
                                               capi_stream_data_t *input[],
                                               capi_stream_data_t *output[],
                                               bool_t *            is_resync_ptr);

capi_err_t capi_trm_check_alloc_held_input_buffer(capi_trm_t *me_ptr);

void capi_trm_dealloc_held_input_buffer(capi_trm_t *me_ptr);

capi_err_t capi_trm_buffer_zeros(capi_trm_t *me_ptr, uint32_t zeros_to_pad_bytes_per_ch);

capi_err_t capi_trm_buffer_input_data(capi_trm_t *me_ptr, capi_stream_data_t *input[]);

capi_err_t capi_trm_render_data_from_held_input_buffer(capi_trm_t *              me_ptr,
                                                       capi_stream_data_t *      output[],
                                                       capi_trm_propped_flags_t *propped_flags_ptr);

capi_err_t capi_trm_free_held_metadata(capi_trm_t *               me_ptr,
                                       capi_trm_circ_buf_frame_t *frame_ptr,
                                       module_cmn_md_list_t **    stream_associated_md_list_pptr,
                                       bool_t                     force_free);

capi_err_t capi_trm_flush_held_buffer(capi_trm_t *me_ptr, module_cmn_md_list_t **    stream_associated_md_list_pptr, bool_t force_free_md);

capi_err_t capi_trm_raise_event_data_trigger_in_st_cntr(capi_trm_t *me_ptr);

void capi_trm_clear_ttr(capi_trm_t *me_ptr);

capi_err_t capi_trm_update_tgp_after_sync(capi_trm_t *me_ptr);
capi_err_t capi_trm_update_tgp_before_sync(capi_trm_t *me_ptr);

void capi_trm_raise_rt_port_prop_event(capi_trm_t *me_ptr, bool_t is_input);

void capi_trm_metadata_destroy_handler(capi_stream_data_v2_t *in_stream_ptr, capi_trm_t *me_ptr);
void capi_trm_metadata_b4_process_nlpi(capi_trm_t *me_ptr, capi_stream_data_v2_t *in_stream_ptr, bool_t **is_resync_ptr);

///////////////////////////////////////CAPI VTBL/////////////////////////////////////////
capi_err_t capi_trm_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[]);
capi_err_t capi_trm_end(capi_t *_pif);
capi_err_t capi_trm_set_param(capi_t *                _pif,
                              uint32_t                param_id,
                              const capi_port_info_t *port_info_ptr,
                              capi_buf_t *            params_ptr);
capi_err_t capi_trm_get_param(capi_t *                _pif,
                              uint32_t                param_id,
                              const capi_port_info_t *port_info_ptr,
                              capi_buf_t *            params_ptr);
capi_err_t capi_trm_set_properties(capi_t *_pif, capi_proplist_t *props_ptr);
capi_err_t capi_trm_get_properties(capi_t *_pif, capi_proplist_t *props_ptr);
capi_vtbl_t *capi_trm_get_vtbl();

static inline bool_t capi_trm_in_has_data(capi_stream_data_t *input[])
{
   return input && input[0] && input[0]->buf_ptr && input[0]->buf_ptr[0].data_ptr &&
          ((0 != input[0]->buf_ptr[0].actual_data_len) || (input[0]->flags.marker_eos) ||
           (input[0]->flags.end_of_frame));
}

static inline bool_t capi_trm_out_has_space(capi_stream_data_t *output[])
{
   return output && output[0] && output[0]->buf_ptr && output[0]->buf_ptr[0].data_ptr &&
          (0 != output[0]->buf_ptr[0].max_data_len);
}

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif
