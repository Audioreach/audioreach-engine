/* ======================================================================== */
/**
  @file push_lab.h
  @brief This file contains function declarations internal to CAPI
         Push Lab module

  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/
#ifndef PUSH_LAB_H
#define PUSH_LAB_H

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#ifndef CAPI_STANDALONE
/* For shared libraries. */
#include "shared_lib_api.h"
#else
#include "capi_util.h"
#endif
#include "posal_timer.h"
#include "sh_mem_push_lab_api.h"
#include "ar_error_codes.h"
#include "capi_types.h"
#include "capi_cmn.h"
#include "capi_cmn_imcl_utils.h"
#include "imcl_dam_detection_api.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

#define PUSH_LAB_MAX_CHANNELS (32)

#define CAPI_PUSH_LAB_MAX_PORTS (4)

// TODO: check if required
#define CAPI_PUSH_LAB_KPPS 1800

#define UMAX_32 0xFFFFFFFFL

// Check if the port index is valid.
#define IS_INVALID_PORT_INDEX(port_id) (port_id == UMAX_32)

/*------------------------------------------------------------------------
 * Structure definitions
 * ----------------------------------------------------------------------*/
#define QFORMAT_TO_BIT_WIDTH(q) ((PCM_Q_FACTOR_15 == q) ? 16 : ((PCM_Q_FACTOR_27 == q) ? 24 : 32))

typedef struct push_lab_media_fmt_t
{
   uint32_t            fmt_id;
   uint32_t            sample_rate;
   uint16_t            num_channels;
   uint16_t            bit_width; // 16,24,32
   uint16_t            is_signed;
   uint16_t            Q_format;
   uint32_t            endianness;
   uint32_t            bits_per_sample; // sample word size, 16, 32
   uint32_t            alignment;
   capi_interleaving_t data_interleaving;
   uint8_t             channel_map[PUSH_LAB_MAX_CHANNELS];
} push_lab_media_fmt_t;

typedef struct push_lab_watermark_event_client_info_t
{
   uint64_t watermark_event_dest_address;
   uint32_t watermark_event_token;
} push_lab_watermark_event_client_info_t;

typedef struct push_lab_watermark_level_t
{
   uint32_t start_index;
   /**< Start index of keyword*/

   uint32_t current_write_position_index;
   /**< Current write index*/

} push_lab_watermark_level_t;

typedef struct push_lab_t
{
   uint32_t                               mode;
   uint32_t                               mem_map_client;
   uint32_t                               num_water_mark_levels;
   uint32_t                               circ_buf_mem_map_handle;
   uint32_t                               shared_circ_buf_size;
   push_lab_watermark_event_client_info_t watermark_event_client_info;
   uint8_t *                              shared_circ_buf_start_ptr;
   push_lab_media_fmt_t                   media_fmt;     /**< input media fmt */
   push_lab_media_fmt_t                   cfg_media_fmt; /**< configured media fmt for push lab */
   capi_buf_t                             scratch_buf[PUSH_LAB_MAX_CHANNELS];
   bool_t                                 is_disabled;
   uint32_t                               watermark_interval_in_us; /**< populated on getting param */
   uint32_t watermark_interval_in_bytes; /**< populated when both param and media format available */
   bool_t   is_gate_opened;              /**< Indicates if Data flow should happen (TRUE) or not (FALSE) */
   uint32_t detection_offset_in_bytes;
   uint32_t start_index;         /**< start of keyword */
   uint32_t current_write_index; /**< current write position */
   uint32_t prev_write_index;    /**< previous write position */
   uint32_t pos_buf_write_index;
   uint32_t last_watermark_level_index; /**< last watermark level */
   bool_t   is_media_fmt_populated;     /**< flag for media format set */
   bool_t   circ_buf_allocated;         /**< flag for circ buf allocated */
   uint32_t resize_in_us;               /**< Populated with param for buf resize */
   uint32_t resize_in_bytes;            /**< Used to verify if set circ buf size fits requirement */
   uint32_t acc_data;

   param_id_audio_dam_output_ch_cfg_t *dam_output_ch_cfg_received; /**< output channel cfg received over control link */

   param_id_audio_dam_output_ch_cfg_t
      *dam_output_ch_cfg; /**< Updated channel cfg based on received and input media format*/

} push_lab_t;

typedef struct capi_push_lab_media_fmt_t
{
   capi_set_get_media_format_t main;
   capi_standard_data_format_t std;
} capi_push_lab_media_fmt_t;

#define MAX_CHANNELS_PER_STREAM 16

#define PUSH_LAB_MAX_CTRL_PORT 1

#define PUSH_LAB_MAX_INTENTS_PER_CTRL_PORT 1

#define PUSH_LAB_MAX_RECURRING_BUF_SIZE 32

#define PUSH_LAB_MAX_NUM_RECURRING_BUFS 2

typedef struct imcl_port_info_t
{
   uint32_t          port_id;
   imcl_port_state_t state;
   uint32_t          num_intents;
   uint32_t          intent_list_arr[PUSH_LAB_MAX_INTENTS_PER_CTRL_PORT];

} imcl_port_info_t;

typedef struct capi_push_lab_t
{
   /* v-table pointer */
   capi_t vtbl;

   /* Heap id, used to allocate memory */
   capi_heap_id_t heap_mem;

   /* Call back info for event raising */
   capi_event_callback_info_t cb_info;

   push_lab_t push_lab_info;

   uint32_t num_active_input_ports;
   /* Number of input ports currently active */

   uint32_t max_input_ports;
   /*Maximum number of input ports */

   uint32_t num_active_output_ports;
   /* Number of input ports currently active */

   uint32_t max_output_ports;
   /* Maximum number of output ports*/

   imcl_port_info_t *imcl_port_info_arr;
   /* Pointers to ctrl port info structures. Size of the array is max_output_ports. */

   capi_port_num_info_t port_info;
   /* Port info received from framework*/

   capi_event_callback_info_t event_cb_info;
   /* Event Call back info received from Framework*/

} capi_push_lab_t;

/*------------------------------------------------------------------------
 * Function declarations
 * ----------------------------------------------------------------------*/

capi_err_t push_lab_init(push_lab_t *pm_ptr, sh_mem_push_lab_cfg_t *init_ptr);

capi_err_t push_lab_check_buf_size(push_lab_t *pm_ptr, push_lab_media_fmt_t *media_fmt_ptr);

capi_err_t push_lab_set_inp_media_fmt(push_lab_t *          pm_ptr,
                                      media_format_t *      media_fmt,
                                      push_lab_media_fmt_t *dst_media_fmt_ptr);

bool_t push_lab_check_media_fmt_validity(push_lab_t *pm_ptr);

capi_err_t push_lab_set_fwk_ext_inp_media_fmt(push_lab_t *pm_ptr, fwk_extn_pcm_param_id_media_fmt_extn_t *extn_ptr);

capi_err_t capi_push_lab_populate_payload_raise_watermark_event(capi_push_lab_t *me);

capi_err_t capi_push_lab_raise_watermark_event(capi_push_lab_t *                      me,
                                               event_cfg_sh_mem_push_lab_watermark_t *water_mark_event);

void push_lab_deinit(push_lab_t *pm_ptr);

capi_err_t push_lab_write_output(capi_push_lab_t *capi_ptr, capi_buf_t *module_buf_ptr, uint64_t timestamp);

uint32_t sh_mem_push_lab_calculate_start_index(capi_push_lab_t *me);

capi_err_t capi_sh_mem_push_lab_imc_set_param_handler(capi_push_lab_t *me_ptr,
                                                      uint32_t         ctrl_port_id,
                                                      capi_buf_t *     intent_buf_ptr);

uint32_t sh_mem_push_lab_us_to_bytes(capi_push_lab_t *me_ptr, uint64_t time_us);

uint32_t capi_push_lab_get_ctrl_port_arr_idx_from_ctrl_port_id(capi_push_lab_t *me_ptr, uint32_t ctrl_port_id);

capi_err_t capi_push_lab_imcl_register_for_recurring_bufs(capi_push_lab_t *me_ptr,
                                                          uint32_t         port_id,
                                                          uint32_t         buf_size,
                                                          uint32_t         num_bufs);

capi_err_t capi_push_lab_imcl_send_unread_len(capi_push_lab_t *me_ptr,
                                              uint32_t         unread_len_in_us,
                                              uint32_t         ctrl_port_id);

capi_err_t push_lab_get_num_out_channels_to_write(push_lab_t *me_ptr);

capi_err_t push_lab_update_dam_output_ch_cfg(capi_push_lab_t *capi_ptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* PUSH_LAB_H */
