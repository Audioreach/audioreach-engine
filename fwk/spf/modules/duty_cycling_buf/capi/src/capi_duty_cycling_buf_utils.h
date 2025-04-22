/**
 *   \file capi_duty_cycling_buf_utils.h
 *   \brief
 *        Header file of utilities for duty cycling buffering module
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef CAPI_DUTY_CYCLING_BUF_UTILS_H
#define CAPI_DUTY_CYCLING_BUF_UTILS_H

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
#include "spf_circular_buffer.h"
#include "platform_internal_dcm_if.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/



#define DUTY_CYCLING_BUF_MAX_INPUT_PORTS 1
#define DUTY_CYCLING_BUF_MAX_OUTPUT_PORTS 1
#define DEFAULT_PORT_INDEX 0
#define MAX_32 (0x7fffffffL)



// Preferred chunk size of the fragmented circular buffer
#define DUTY_CYCLING_BUF_PREFERRED_CHUNK_SIZE (2048)


// TODO: profile and update
#define CAPI_DUTY_CYCLING_BUF_KPPS (30)
#define CAPI_DUTY_CYCLING_BUF_CODE_BW (1*1024*1024)
#define CAPI_DUTY_CYCLING_BUF_DATA_BW (0)





typedef struct capi_duty_cycling_buf_events_config_t
{
   uint32_t enable;
   uint32_t kpps;
   uint32_t delay_in_us;
   uint32_t code_bw;
   uint32_t data_bw;
} capi_duty_cycling_buf_events_config_t;



typedef struct capi_duty_cycling_buf_tgp_t
{
   fwk_extn_param_id_trigger_policy_cb_fn_t tg_policy_cb;
} capi_duty_cycling_buf_tgp_t;


typedef enum capi_duty_cycling_buf_staus
{
	DUTY_CYCLING_WHILE_BUFFERING = 0, //State to indicate that module is buffering data received in input. usually in non-island
	DUTY_CYCLING_AFTER_BUFFERING = 1, //State to indicate that sufficient data is buffered
} capi_duty_cycling_buf_staus;


/* CAPI structure  */
typedef struct capi_duty_cycling_buf_t
{
   const capi_vtbl_t *vtbl;
   /* pointer to virtual table */
   capi_event_callback_info_t event_cb_info;
   /* Event Call back info received from Framework*/
   capi_heap_id_t heap_info;
   /* Heap id received from framework*/
   capi_duty_cycling_buf_tgp_t tgp;
   /* Trigger policy structure */
   intf_extn_param_id_metadata_handler_t metadata_handler;
   /* handler for metadata operations */
   uint32_t cntr_frame_size_us;
   /*frame size of container in us*/
   uint32_t expected_input_bytes;
   /*This value is used for tracking of data received. Typically = container frame size. Always based on latest in MF*/
   uint32_t expected_output_bytes;
   /*This value is used for tracking of data pulled from circ buffer.Typically = container frame size Always based on MF raised by circ buf*/
   bool_t   cntr_frame_size_received;
   /*Flag to know if circular buffer raised output media fmt */
   uint32_t num_in_ports;
   /*Number of input ports*/
   uint32_t num_out_ports;
   /*Number of output ports*/
   capi_duty_cycling_buf_events_config_t          events_config;
   /*Events related structure*/
   bool_t is_buf_cfg_received;
   /* flag to determine if buffer configuration has been received */
   uint32_t buffer_size_in_ms;
   /* Size of circular buffer */
   uint32_t lower_threshold_in_ms;
   /* Lower threshold used to decide on exiting island.Container frame size and threshold received in set_param decides  value*/
   uint32_t lower_threshold_in_ms_rcvd_in_set_param;
   /* Lower threshold received in set_param  */
   uint32_t data_rcvd_in_ms_while_buffering;
   /*data in ms received in input while buffering.  */
   uint32_t data_remaining_in_ms_in_circ_buf;
   /*data in ms remaining in circular  buffer */
   capi_duty_cycling_buf_staus current_status;
   /*Indiacate the current status of the moduel i.e., while buffering or buffering is complete */
   capi_duty_cycling_buf_staus tgp_status;
   /*Indiacate the current trigger policy raised status of the module i.e., while buffering or buffering is complete */
   bool_t              cir_buffer_raised_out_mf;
   /*Flag to know if circular buffer raised output media fmt */
   capi_media_fmt_v2_t out_media_fmt;
   /*output media fmt raised by circular buffer */
   bool_t is_input_mf_received;
   /* flag to determine if input media format has been received */
   capi_media_fmt_v2_t in_media_fmt;
   /* Media format of the newest data in the circular buffer*/
   void *              sdata_circ_buf_ptr; // Internal circular buffer.
   /* circular buffer handles*/
   spf_circ_buf_t *       stream_drv_ptr;
   /* Stream buffer driver handle*/
   spf_circ_buf_client_t *writer_handle;
   /* writer handle, used for buffering stream into circular buffer*/
   spf_circ_buf_client_t *reader_handle;
   /* reader handle used for reading data from the circular buffer */
   dcm_island_control_payload_t payload;
   /*Payload used for sending entry or exit to dcm*/
} capi_duty_cycling_buf_t;

static inline uint32_t align_to_8_byte(const uint32_t num)
{
  return ((num + 7) & (0xFFFFFFF8));
}

/*------------------------------------------------------------------------
 * Function Definitions
 * -----------------------------------------------------------------------*/
///////////////////////////////////////CAPI VTBL/////////////////////////////////////////
capi_err_t capi_duty_cycling_buf_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[]);
capi_err_t capi_duty_cycling_buf_end(capi_t *_pif);
capi_err_t capi_duty_cycling_buf_set_param(capi_t *                _pif,
                                           uint32_t                param_id,
                                           const capi_port_info_t *port_info_ptr,
                                           capi_buf_t *            params_ptr);
capi_err_t capi_duty_cycling_buf_get_param(capi_t *                _pif,
                                           uint32_t                param_id,
                                           const capi_port_info_t *port_info_ptr,
                                           capi_buf_t *            params_ptr);
capi_err_t capi_duty_cycling_buf_set_properties(capi_t *_pif, capi_proplist_t *props_ptr);
capi_err_t capi_duty_cycling_buf_get_properties(capi_t *_pif, capi_proplist_t *props_ptr);
capi_vtbl_t *capi_duty_cycling_buf_get_vtbl();

capi_err_t capi_duty_cycling_buf_raise_event_data_trigger_in_st_cntr(capi_duty_cycling_buf_t *me_ptr);
capi_err_t capi_duty_cycling_buf_create_buffer(capi_duty_cycling_buf_t *me_ptr);

capi_err_t capi_duty_cycling_buf_set_prop_input_media_fmt(capi_duty_cycling_buf_t *me_ptr, capi_prop_t *prop_ptr);

bool_t capi_duty_cycling_buf_does_circular_buffer_exist(capi_duty_cycling_buf_t *        me_ptr);
capi_err_t capi_duty_cycling_buf_create_circular_buffer(capi_duty_cycling_buf_t *        me_ptr,
                                                        capi_media_fmt_v2_t *            media_fmt_ptr,
                                                        uint32_t                         size_ms);
capi_err_t duty_cycling_buf_circular_buffer_event_cb(void *          context_ptr,
                                                     spf_circ_buf_t *circ_buf_ptr,
                                                     uint32_t        event_id,
                                                     void *          event_info_ptr);

capi_err_t capi_duty_cycling_buf_set_circular_buffer_media_fmt(capi_duty_cycling_buf_t *        me_ptr,
                                                               capi_media_fmt_v2_t *            media_fmt_ptr,
                                                               uint32_t                         cntr_frame_size_us);
capi_err_t duty_cycling_buf_check_create_circular_buffer(capi_duty_cycling_buf_t *me_ptr);
capi_err_t capi_duty_cycling_buf_get_circular_buffer_media_fmt(capi_duty_cycling_buf_t *        me_ptr,
                                                               capi_media_fmt_v2_t *            ret_mf_ptr);
void capi_duty_cycling_buf_get_unread_bytes(capi_duty_cycling_buf_t *        me_ptr,
                                            uint32_t *                       unread_bytes);
bool_t capi_duty_cycling_buf_is_circular_buffer_empty(capi_duty_cycling_buf_t *        me_ptr);
bool_t capi_duty_cycling_buf_is_circular_buffer_full(capi_duty_cycling_buf_t *        me_ptr);
capi_err_t capi_duty_cycling_buf_raise_rt_port_prop_event(capi_duty_cycling_buf_t *me_ptr,
                                                          bool_t                   is_input,
                                                          bool_t                   is_rt,
                                                          bool_t                   port_index);
capi_err_t capi_duty_cycling_buf_underrun(capi_duty_cycling_buf_t *me_ptr, capi_stream_data_t *output[]);
capi_err_t capi_duty_cycling_buf_island_entry_exit_util(capi_duty_cycling_buf_t *me_ptr);

capi_err_t capi_duty_cycling_buf_destroy_circular_buffer(capi_duty_cycling_buf_t *        me_ptr);
capi_err_t capi_duty_cycling_buf_update_tgp_after_buffering(capi_duty_cycling_buf_t *me_ptr);
capi_err_t capi_duty_cycling_buf_update_tgp_while_buffering(capi_duty_cycling_buf_t *me_ptr);
capi_err_t capi_duty_cycling_buf_set_data_port_property(capi_duty_cycling_buf_t *me_ptr, capi_buf_t *payload_ptr);

capi_err_t capi_duty_cycling_buf_set_param_port_op(capi_duty_cycling_buf_t *me_ptr, capi_buf_t *payload_ptr);

static inline bool_t capi_duty_cycling_buf_in_has_data(capi_stream_data_t *input[])
{
   return input && input[DEFAULT_PORT_INDEX] && input[DEFAULT_PORT_INDEX]->buf_ptr &&
          input[DEFAULT_PORT_INDEX]->buf_ptr[0].data_ptr &&
          ((0 != input[DEFAULT_PORT_INDEX]->buf_ptr[0].actual_data_len)) && ( input[DEFAULT_PORT_INDEX]->flags.erasure == FALSE);
}

static inline bool_t capi_duty_cycling_buf_out_has_space(capi_stream_data_t *output[])
{
   return output && output[DEFAULT_PORT_INDEX] && output[DEFAULT_PORT_INDEX]->buf_ptr &&
          output[DEFAULT_PORT_INDEX]->buf_ptr[0].data_ptr &&
          (0 !=
           output[DEFAULT_PORT_INDEX]->buf_ptr[0].max_data_len -
              (output[DEFAULT_PORT_INDEX]->buf_ptr[0].actual_data_len));
}

static inline bool_t capi_duty_cycling_buf_out_has_partial_data(capi_stream_data_t *output[])
{
   return output && output[DEFAULT_PORT_INDEX] && output[DEFAULT_PORT_INDEX]->buf_ptr &&
          output[DEFAULT_PORT_INDEX]->buf_ptr[0].data_ptr &&
          (0 != output[DEFAULT_PORT_INDEX]->buf_ptr[0].actual_data_len);
}


#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif
