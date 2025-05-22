/* ======================================================================== */
/**
   @file capi_sal_utils.h

   Header file to define types internal to the CAPI interface for the SAL module
*/

/* =========================================================================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
   ========================================================================== */

#ifndef CAPI_SAL_UTILS_H
#define CAPI_SAL_UTILS_H

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "posal.h"
#include "capi_cmn.h"
#include "limiter_calibration_api.h"
#include "limiter_api.h"
#include "spf_list_utils.h"
#include "capi_intf_extn_metadata.h"
#include "capi_intf_extn_data_port_operation.h"
#include "capi_intf_extn_mimo_module_process_state.h"
#include "sal_metadata_api.h"
/*------------------------------------------------------------------------
 * Macros
 * -----------------------------------------------------------------------*/
// Uncomment the below line to enable low level logs
//#define SAL_DBG_LOW

#if ((defined __hexagon__) || (defined __qdsp6__))
#define QDSP_ADD
#endif

#ifndef SAL_MAX
#define SAL_MAX(m, n) (((m) > (n)) ? (m) : (n))
#endif

#ifndef SAL_MIN
#define SAL_MIN(m, n) (((m) < (n)) ? (m) : (n))
#endif
#define SAL_CEIL(x, y) (((x) + (y)-1) / (y))

#define CAPI_SAL_PORT_THRESHOLD 1 // ms
#define CAPI_SAL_LIM_BLOCK_SIZE_1_MS 1
#define SAL_INTERLEAVING_DEFAULT CAPI_DEINTERLEAVED_UNPACKED

/** Configured parameter (like bps, num_channels, etc) is considered valid
    and should be used as the configuration for the module. */
#define SAL_PARAM_VALID 1
#define SAL_PARAM_NATIVE PARAM_VAL_NATIVE
/*Q-Formats corresponding to Bits per sample*/
#define QF_BPS_16 PCM_Q_FACTOR_15
#define QF_BPS_24 PCM_Q_FACTOR_27
#define QF_BPS_32 PCM_Q_FACTOR_31
#define QF_CNV_4 4
#define QF_CNV_12 12
#define QF_CNV_16 16

static const uint32_t MAX_Q27 = ((1 << PCM_Q_FACTOR_27) - 1);
static const uint32_t MIN_Q27 = (-(1 << PCM_Q_FACTOR_27));
static const uint32_t MAX_Q31 = 0x7FFFFFFF;
static const uint32_t MIN_Q31 = 0x80000000;

//#define SAL_LIMITER_MAX_DELAY 3            27 // delay in seconds in Q15. 10ms = 0.01 in Q15 format
/* SAL Limiter Threshold for 16bit output in Q27 format*/
//#define SAL_LIM_16BIT_THRESHOLD_Q27        0x05998000 // 0.7
/* AFE Limiter Threshold for 16bit output in Q15 format -0.02 dB - used in AFE*/
#define SAL_LIM_16BIT_THRESHOLD_Q15 0x7FB5
/* AFE Limiter Threshold for 32bit output in Q27 format -0.02 dB - used in AFE*/
#define SAL_LIM_32BIT_THRESHOLD_Q27 0x7FB4A2C
/* SAL limiter delay in seconds: 1ms in Q15 */
#define SAL_LIM_DELAY_1MS_Q15 33
/* SAL Limiter recovery const in Q15*/
#define SAL_LIM_GC_Q15 0x7EB8
/* SAL Limiter Make up gain in Q8 format*/
#define SAL_LIM_MAKEUP_GAIN_Q8 0x0100
/*10ms = 0.01 in Q15 format*/
#define SAL_LIM_GAIN_ATTACK_CONSTANT_Q31 0xB362C80
/* SAL Limiter Gain release time const in Q31*/
#define SAL_LIM_GAIN_RELEASE_CONSTANT_Q31 0x1F0D180
/* SAL Limiter Gain attack time coefficient value in Q15*/
#define SAL_LIM_GAIN_ATTACK_COEFFICIENT_Q15 0x8000
/* SAL Limiter Gain release time coefficient value in Q15*/
#define SAL_LIM_GAIN_RELEASE_COEFFICIENT_Q15 0x8000
/* SAL Hard Limiter Threshold for 16bit output in Q15 format*/
#define SAL_LIM_16BIT_HARD_THRESHOLD_Q15 0x8000 // 0db
/* SAL Hard Limiter Threshold for 16bit output in Q27 format*/
#define SAL_LIM_32BIT_HARD_THRESHOLD_Q27 0x8000000 // 0 db

#define SAL_LIM_DISABLED_ALWAYS 0
#define SAL_LIM_ENABLED_IF_REQ 1
#define SAL_LIM_ENABLED_ALWAYS 2

#define SAL_INVALID_PORT_IDX 0xFFFFFFFF
#define SAL_MSG_PREFIX "SAL: 0x%lx: "
#define SAL_MSG(ID, xx_ss_mask, xx_fmt, ...) AR_MSG(xx_ss_mask, SAL_MSG_PREFIX xx_fmt, ID, ##__VA_ARGS__)
#define SAL_MSG_ISLAND(ID, xx_ss_mask, xx_fmt, ...) AR_MSG_ISLAND(xx_ss_mask, SAL_MSG_PREFIX xx_fmt, ID, ##__VA_ARGS__)
/*------------------------------------------------------------------------
 * Type definitions
 * -----------------------------------------------------------------------*/
typedef struct capi_limiter_memory_struct_t
{
   void *        mem_ptr; // malloc ptr for both lib and scratch if needed
   uint32_t      mem_req; // limiter memory requirements
   POSAL_HEAP_ID heap_id; // Heap ID to be used for memory allocation
} limiter_memory_struct_t;

typedef struct capi_sal_events_config_t
{
   uint32_t sal_kpps;
   uint32_t sal_bw;
} capi_sal_events_config_t;

/* Struct type used to cache the input media format that is
required by the SAL module*/
typedef struct sal_port_cache_mf_t
{
   uint32_t q_factor; /*Qformat of the data in the stream*/
   uint32_t word_size_bytes;
} sal_cache_mf_t;

typedef enum capi_sal_data_port_state_t
{
   DATA_PORT_CLOSED  = 0,
   DATA_PORT_OPEN    = 1,
   DATA_PORT_STARTED = 2,
   DATA_PORT_STOPPED = 3
} capi_sal_data_port_state_t;

typedef struct sal_port_flag_t
{
   uint32_t is_ref_port : 1;
   /*reference port - if MF changes on this port, we reset the OMF*/
   uint32_t data_drop : 1;
   /*If a port receives IMF different from OMF - we set this flag to TRUE and drop data*/
   uint32_t at_gap : 1;
   /** see notes in capi_sal_check_process_metadata */
   uint32_t is_algo_proc : 1;
   /*used to reset the algo*/ // why cant we use at_gap ?
   uint32_t mf_rcvd : 1;
   /*indicates if valid MF was received on this port*/
   uint32_t proc_check_pass : 1;
   /*indicates proc readiness for this port (STARTED + input stream validation) - should be
   refreshed every process call. Not to be used outside the process call*/
} sal_port_flag_t;

typedef struct sal_in_port_array_t
{
   capi_sal_data_port_state_t state;
   /* enum */
   sal_port_flag_t port_flags;
   /* port flags*/
   capi_media_fmt_v2_t mf;
   /*Media Format received on this port*/
   uint32_t pending_zeros_at_eos;
   /* Pending zeros on the PORT in Bytes per channel */
   module_cmn_md_list_t *md_list_ptr;
   /* internal metadata list. will be used if output buf is larger than algo delay*/
   uint32_t proc_ctx_prev_actual_data_len;
   /* Amount of data on the input port at the beginning of the process call. */
} sal_in_port_array_t;

typedef struct sal_flag_t
{
   uint32_t any_valid_mf_rcvd : 1;
   /*For Path delay purposes, we need to raise mf before even determining OMF*/
   /* Configuration mode: Native, unset, invalid */
   uint32_t is_lim_set_cfg_rcvd : 1;
   /*boolean to indicate if the limiter set cfg is received*/
   uint32_t op_mf_requires_limiting : 1;
   /*boolean to indicate if limiting is necessary based on operating input media format*/
   /*Note: Only if both limiter_enabled(configured, TRUE by default) AND limiting required (decided based on MF, etc),
    * will the limiter be invoked*/
   /*Boolean to indicate if the limiter was configured*/
   uint32_t dtmf_tone_started : 1;

   uint32_t is_inplace : 1;

   uint32_t unmixed_output : 1;
   /* If unmixed_output is TRUE, SAL copies data from unmixed_output_port_index to output */
   uint32_t process_other_input_md : 1;
   /* If unmixed output is TRUE, SAL attempts to process the other inputs with a gain of 0 to propagate MD */
   uint32_t raise_mf_on_next_process : 1;
   /* Flag to raise media format on next process call */
   uint32_t insert_int_eos : 1;
   /** when all inputs are closed, a flushing EOS is inserted in the next process call if any input was not at-gap when
    * getting closed.*/
   uint32_t all_ports_at_gap : 1;
   /** when all ports are at gap */
} sal_flag_t;

typedef struct capi_sal_input_media_process_info_t
{
   void (*accumulate_func_ptr)(int8_t *, int8_t *, uint32_t);
   /* ptr to accumulate function */
   uint8_t alignment;
   /* alignment required for input buf ptr */
   bool_t upconvert_flag;
   /* upconvert flag */
} capi_sal_input_media_process_info_t;

typedef struct capi_sal_t
{
   capi_t vtbl;
   /* v-table pointer */
   capi_heap_id_t heap_mem;
   /* Heap id, used to allocate memory */
   capi_event_callback_info_t cb_info;
   /* Callback info for event raising */
   capi_sal_events_config_t events_config;
   /* Event information struct */
   capi_media_fmt_v2_t *operating_mf_ptr;
   /* Operating Media Format */
   capi_sal_input_media_process_info_t input_process_info;
   /* Input Media Format Process Info */
   sal_flag_t module_flags;
   /*maintains various states/flags for the SAL module*/
   int32_t bps_cfg_mode;
   /* bps configuration mode */
   sal_in_port_array_t *in_port_arr;
   /*contains details like port state, mf rcv state and bps and qf for each input port*/
   int32_t *started_in_port_index_arr;
   /* array of port indexes in the started state  */
   uint32_t num_in_ports_started;
   /* number of input ports in the started state */
   uint32_t num_ports_at_gap;
   /* number of input ports at gap */
#if __qdsp6__
   capi_buf_t acc_in_scratch_buf;
   /*  scratch buffer which is eight byte aligned, used for vector optimization */
#endif
   capi_buf_t *acc_out_scratch_arr;
   /*  memory to store as many channel buf pointers as there are input ports during process */
   sal_cache_mf_t out_port_cache_cfg;
   /*to cache configured output bps and QF*/
   uint32_t ref_acc_out_buf_len;
   /* variable updated if we will be needing more memory than initially allocated for scratch buffers*/
   uint32_t num_in_ports;
   /*Number of input ports to the SAL module*/
   limiter_tuning_v2_t limiter_params;
   /*Limiter tuning parameters exposed in the API*/
   int32_t lim_bypass;
   /*indicates if lim needs to process in bypass mode*/
   limiter_lib_t lib_mem;
   /*Memory for the limiter instance*/
   limiter_static_vars_v2_t limiter_static_vars;

   limiter_memory_struct_t limiter_memory;
   int32_t **              lim_in_ptr;
   /* Array of input pointers to be passed to algorithm.
    * Array length is number of channels*/
   int32_t **lim_out_ptr;
   /* Array of output pointers to be passed to algorithm.
    * Array length is number of channels*/
   intf_extn_param_id_metadata_handler_t metadata_handler;

   uint32_t algo_delay_us;

   uint32_t unmixed_output_port_index;
   /* The input port index from which data is copied over to the output port in case of unmixed_output = TRUE */
   uint32_t iid;
   /*ID unique to this instance of the SAL module*/
   uint16_t cfg_lim_block_size_ms;
   /*This paramter is used to allocate the limiter lib instance's max block size (analogous to frame size)
   If SAL receives data less than this, limiter can directly nbe invoked. If more, SAL will loop over
   and invoke limiter with this much data each time. - Default = 1 ms*/
   module_cmn_md_list_t *md_list_ptr;
   /** internal EOS list (this is used only for the internal EOS inserted by SAL. Rest use per port struct */
   capi_sal_data_port_state_t output_state;
   /** The last output media format which was raised. Cached to avoid redundant omf events, and to avoid leaving
       data unconsumed during data flow start handling. */
   capi_media_fmt_v2_t last_raised_out_mf;
   /* This parameter indicate the limiter is enabled/desabled */
   uint32_t limiter_enabled;
} capi_sal_t;

/*------------------------------------------------------------------------
 * Function Declarations
 * -----------------------------------------------------------------------*/

////////////////////////////////////////////////General _utils.cpp////////////////////////////////////////////////
/* Utility functions not a part of the function table  (vtbl) */
/*initializes limiter configuration*/
void capi_sal_init_limiter_cfg(capi_sal_t *me_ptr);
/*Allocates the scratch memory required for the BW convertor and accumulator*/
capi_err_t capi_sal_bw_conv_allocate_scratch_ptr_buf(capi_sal_t *me_ptr, uint32_t data_len);
/*Destroys the scratch memory used by the BW convertor and the accumulator*/
capi_err_t capi_sal_destroy_scratch_ptr_buf(capi_sal_t *me_ptr);
/*Calculates and stores the current KPPS and BW requirement of the SAL module*/
capi_err_t capi_sal_update_and_raise_kpps_bw_event(capi_sal_t *me_ptr);
/*Allocates and partitions limiter library memory*/
capi_err_t capi_sal_limiter_allocate_lib_memory(capi_sal_t *me_ptr);
/*Allocs scratch, limiter memory and raises the output media format updated event to the callback function when theres a
 * change in OMF*/
capi_err_t capi_sal_alloc_scratch_lim_mem_and_raise_events(capi_sal_t *me_ptr,
                                                           uint32_t    lim_data_width,
                                                           uint32_t    q_factor);
/*Prints OMF*/
void capi_sal_print_operating_mf(capi_sal_t *me_ptr);

capi_err_t capi_sal_check_and_raise_process_state_events(capi_sal_t *met_ptr);

void       capi_sal_update_raise_delay_event(capi_sal_t *me_ptr);
capi_err_t capi_sal_accept_omf_alloc_mem_and_raise_events(capi_sal_t *me_ptr,
                                                          uint32_t    port_index,
                                                          bool_t      data_produced,
                                                          bool_t *    mf_raised_ptr);

/*Does what needs to be done to reset the limiter.*/
capi_err_t capi_sal_algo_reset(capi_sal_t *me_ptr);
/*Returns the corresponding Qfactor to the provided bits per sample*/
uint32_t capi_sal_bps_to_qfactor(uint32_t bps);
/*Returns the corresponding bit-width to the provided q_factor*/
uint32_t   capi_sal_qf_to_bps(uint32_t qf);
capi_err_t capi_sal_raise_out_mf(capi_sal_t *me_ptr, capi_media_fmt_v2_t *mf_ptr, bool_t *mf_raised_ptr);
////////////////////////////////////////////////md_utils.cpp////////////////////////////////////////////////
void               capi_sal_process_metadata_after_process(capi_sal_t *        me_ptr,
                                                           capi_stream_data_t *input[],
                                                           capi_stream_data_t *output[],
                                                           uint32_t            max_num_samples_per_ch,
                                                           uint32_t            input_word_size_bytes);
capi_err_t         capi_sal_destroy_md_list(capi_sal_t *me_ptr, module_cmn_md_list_t **md_list_pptr);
void               capi_sal_push_zeros_for_flushing_eos(capi_sal_t *           me_ptr,
                                                        capi_stream_data_v2_t *in_stream_ptr,
                                                        sal_in_port_array_t *  in_port_ptr,
                                                        uint32_t *             num_samples_per_ch_ptr,
                                                        uint32_t *             max_num_samples_per_ch_ptr,
                                                        uint32_t               out_max_samples_per_ch,
                                                        uint32_t               num_active_inputs_to_acc_without_eos);
static inline void capi_sal_check_push_zeros_for_flushing_eos(capi_sal_t *           me_ptr,
                                                              capi_stream_data_v2_t *in_stream_ptr,
                                                              sal_in_port_array_t *  in_port_ptr,
                                                              uint32_t *             num_samples_per_ch_ptr,
                                                              uint32_t *             max_num_samples_per_ch_ptr,
                                                              uint32_t               out_max_samples_per_ch,
                                                              uint32_t num_active_inputs_to_acc_without_eos)
{
   // if not flushing EOS return
   if (!in_stream_ptr->flags.marker_eos)
   {
      return;
   }

   capi_sal_push_zeros_for_flushing_eos(me_ptr,
                                        in_stream_ptr,
                                        in_port_ptr,
                                        num_samples_per_ch_ptr,
                                        max_num_samples_per_ch_ptr,
                                        out_max_samples_per_ch,
                                        num_active_inputs_to_acc_without_eos);
}

void   capi_sal_check_process_metadata(capi_sal_t *           me_ptr,
                                       capi_stream_data_v2_t *in_stream_ptr,
                                       capi_stream_data_v2_t *out_stream_ptr,
                                       uint32_t               num_samples_per_ch,
                                       sal_in_port_array_t *  in_port_ptr,
                                       uint32_t               data_port_index,
                                       uint32_t *             num_ports_flush_eos_ptr);
void   capi_sal_process_metadata_after_process(capi_sal_t *        me_ptr,
                                               capi_stream_data_t *input[],
                                               capi_stream_data_t *output[],
                                               uint32_t            max_num_samples_per_ch,
                                               uint32_t            input_word_size_bytes,
                                               uint32_t            num_active_inputs);
void   capi_sal_insert_eos_for_us_gap(capi_sal_t *me_ptr, capi_stream_data_t *output[]);
bool_t capi_sal_sdata_has_dfg(capi_sal_t *me_ptr, capi_stream_data_v2_t *sdata_ptr);
void   capi_sal_destroy_all_md(capi_sal_t *me_ptr, capi_stream_data_t *input[]);

////////////////////////////////////////////////port_utils.cpp////////////////////////////////////////////////
void capi_sal_check_and_update_lim_bypass_mode(capi_sal_t *me_ptr);

ar_result_t capi_sal_compare_port_mfs_to_omf_and_asign_data_drops(capi_sal_t *me_ptr);
ar_result_t capi_sal_evaluate_non_ref_port_imf(capi_sal_t *         me_ptr,
                                               uint32_t             size_to_copy,
                                               capi_media_fmt_v2_t *media_fmt_ptr,
                                               uint32_t             actual_data_len,
                                               uint32_t             port_index);

ar_result_t capi_sal_evaluate_ref_port_imf(capi_sal_t *         me_ptr,
                                           uint32_t             size_to_copy,
                                           capi_media_fmt_v2_t *media_fmt_ptr,
                                           uint32_t             actual_data_len,
                                           uint32_t             port_index);

uint32_t   capi_sal_get_new_ref_port_index(capi_sal_t *me_ptr, uint32_t curr_ref_port_idx);
bool_t     capi_sal_handle_data_flow_start(capi_sal_t *me_ptr, uint32_t data_port_index);
bool_t     capi_sal_handle_data_flow_start_force_ref(capi_sal_t *me_ptr, uint32_t force_ref_port_idx);
capi_err_t capi_sal_handle_data_flow_stop(capi_sal_t *me_ptr, uint32_t data_port_index, bool_t data_produced);
capi_err_t capi_sal_assign_ref_port(capi_sal_t *me_ptr, uint32_t ref_port_index);
capi_err_t capi_sal_handle_metadata_b4_process(capi_sal_t *        me_ptr,
                                               capi_stream_data_t *input[],
                                               capi_stream_data_t *output[]);

//////////////////////////////////////////////vtbl functions////////////////////////////////////////////////////////
capi_err_t capi_sal_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[]);

capi_err_t capi_sal_end(capi_t *_pif);

capi_err_t capi_sal_set_param(capi_t *                _pif,
                              uint32_t                param_id,
                              const capi_port_info_t *port_info_ptr,
                              capi_buf_t *            params_ptr);

capi_err_t capi_sal_get_param(capi_t *                _pif,
                              uint32_t                param_id,
                              const capi_port_info_t *port_info_ptr,
                              capi_buf_t *            params_ptr);

capi_err_t capi_sal_set_properties(capi_t *_pif, capi_proplist_t *props_ptr);

capi_err_t capi_sal_get_properties(capi_t *_pif, capi_proplist_t *props_ptr);

capi_vtbl_t *capi_sal_get_vtbl();
capi_err_t   capi_sal_dtmf(capi_sal_t *        me_ptr,
                           capi_stream_data_t *input[],
                           capi_stream_data_t *output[],
                           bool_t              any_port_has_md_n_flags,
                           uint32_t            max_num_samples_per_ch,
                           uint32_t            input_word_size_bytes);
capi_err_t capi_sal_lim_loop_process(capi_sal_t *me_ptr, uint32_t max_num_samples_per_ch, capi_stream_data_t *output[]);
void       downconvert_ws_32(int8_t *input_ch_buf, uint16_t shift_factor, uint32_t num_samp_per_ch);
bool_t     capi_sal_inqf_greater_than_outqf(capi_sal_t *        me_ptr,
                                            uint32_t            input_qf,
                                            uint32_t            output_qf,
                                            capi_stream_data_t *output[],
                                            uint32_t            max_num_samples_per_ch,
                                            capi_err_t *        err_code_ptr);
static inline uint32_t capi_sal_get_num_active_in_ports(capi_sal_t *me_ptr)
{
   uint32_t num_active_in_ports = 0;
   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      if ((DATA_PORT_STARTED == me_ptr->in_port_arr[i].state) && me_ptr->in_port_arr[i].port_flags.mf_rcvd &&
          !me_ptr->in_port_arr[i].port_flags.at_gap)
      {
         num_active_in_ports++;
      }
   }

   return num_active_in_ports;
}

static inline bool_t capi_sal_check_limiting_required(capi_sal_t *me_ptr)
{
   return me_ptr->module_flags.op_mf_requires_limiting && me_ptr->limiter_enabled;
}

static inline void capi_sal_process_metadata_handler(bool_t              any_port_has_md_n_flags,
                                                     capi_sal_t *        me_ptr,
                                                     capi_stream_data_t *input[],
                                                     capi_stream_data_t *output[],
                                                     uint32_t            max_num_samples_per_ch,
                                                     uint32_t            input_word_size_bytes)
{
   if (FALSE == any_port_has_md_n_flags)
   {
      return;
   }

   uint32_t num_active_in_ports = capi_sal_get_num_active_in_ports(me_ptr);
   posal_island_trigger_island_exit();
   capi_sal_process_metadata_after_process(me_ptr,
                                           input,
                                           output,
                                           max_num_samples_per_ch,
                                           input_word_size_bytes,
                                           num_active_in_ports);
}
#endif // CAPI_SAL_UTILS_H
