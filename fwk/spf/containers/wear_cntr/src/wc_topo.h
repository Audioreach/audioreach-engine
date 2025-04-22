#ifndef WC_TOPO_H_
#define WC_TOPO_H_

/**
 * \file wc_topo.h
 *
 * \brief
 *
 *     Basic Topology header file.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_guids.h"
#include "spf_utils.h"
#include "spf_macros.h"
#include "spf_svc_utils.h"
#include "gpr_packet.h"
#include "gpr_api_inline.h"
#include "shared_lib_api.h"
#include "amdb_static.h"
#include "wc_graph_utils.h"
#include "apm_cntr_if.h"
#include "cntr_cntr_if.h"
#include "capi_cmn.h"
#include "capi_intf_extn_data_port_operation.h"
#include "rd_sh_mem_ep_api.h"


#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

// clang-format off


//#define VERBOSE_DEBUGGING
//#define VERBOSE_LOGGING


#define WCNTR_UNITY_Q4 0x10
#define WCNTR_MAX_NUM_CHANNELS 32
#define WCNTR_TOPO_QFORMAT_TO_BIT_WIDTH(q) ((PCM_Q_FACTOR_15 == q) ? 16 : ((PCM_Q_FACTOR_23 == q) ? 24 : ((PCM_Q_FACTOR_27 == q) ? 24 : 32)))


#define WCNTR_TOPO_PERF_MODE_TO_FRAME_DURATION_MS(perf_mode) ((APM_SG_PERF_MODE_LOW_POWER == perf_mode) ?                    \
                                                      WCNTR_PERF_MODE_LOW_POWER_FRAME_DURATION_MS :                          \
                                                      WCNTR_PERF_MODE_LOW_LATENCY_FRAME_DURATION_MS)


typedef struct wcntr_topo_t                    wcntr_topo_t;
typedef struct wcntr_topo_module_t             wcntr_topo_module_t;
typedef struct wcntr_topo_input_port_t         wcntr_topo_input_port_t;
typedef struct wcntr_topo_output_port_t        wcntr_topo_output_port_t;
typedef struct wcntr_topo_ctrl_port_t          wcntr_topo_ctrl_port_t;
typedef struct wcntr_topo_process_context_t	 wcntr_topo_process_context_t;
typedef struct wcntr_topo_module_bypass_t      wcntr_topo_module_bypass_t;
typedef struct wcntr_topo_common_port_t        wcntr_topo_common_port_t;
typedef struct wcntr_topo_graph_init_t         wcntr_topo_graph_init_t;


typedef struct wcntr_topo_to_cntr_vtable_t
{
  
   /* Topo callback to containers to handle events to DSP service. */
   ar_result_t (*raise_data_to_dsp_service_event)(wcntr_topo_module_t *module_context_ptr,
												              capi_event_info_t *event_info_ptr);

   /* Topo callback to containers to handle events from DSP service. */
   ar_result_t (*raise_data_from_dsp_service_event)(wcntr_topo_module_t *module_context_ptr,
                                                  capi_event_info_t *event_info_ptr);

   ar_result_t (*handle_capi_event)(wcntr_topo_module_t *module_context_ptr,
                                              capi_event_id_t    id,
                                              capi_event_info_t *event_info_ptr);

   ar_result_t (*destroy_module)(wcntr_topo_t *       topo_ptr,
                                 wcntr_topo_module_t *module_ptr,
                                 bool_t             reset_capi_dependent_dont_destroy);
} wcntr_topo_to_cntr_vtable_t;


typedef enum wcntr_topo_trigger_t
{
   WCNTR_TOPO_INVALID_TRIGGER = 0,
   /**< when we come out of signal or buf trigger, invalid trigger is set.
    *  This helps if we check curr_trigger in some other context. */
   WCNTR_TOPO_DATA_TRIGGER=1,
   /**< data/buffer trigger caused current topo process - NOT SUPPORTED FOR WCNTR */
   WCNTR_TOPO_SIGNAL_TRIGGER=2
   /**< signal/interrupt/timer trigger caused current topo process THE ONLY TRIGGER FOR WCNTR*/
} wcntr_topo_trigger_t;

// TOPO utils begin

#define WCNTR_TOPO_MSG_PREFIX "WCNTR_TU  :%08X: "
#define WCNTR_TOPO_MSG(ID, xx_ss_mask, xx_fmt, ...) AR_MSG(xx_ss_mask, WCNTR_TOPO_MSG_PREFIX xx_fmt, ID, ##__VA_ARGS__)

#define WCNTR_TOPO_PRINT_PCM_MEDIA_FMT(log_id, media_fmt_ptr, IOTYPE)                                                        \
   do                                                                                                                  \
   {                                                                                                                   \
      WCNTR_TOPO_MSG(log_id,                                                                                                 \
               DBG_HIGH_PRIO,                                                                                          \
               SPF_LOG_PREFIX IOTYPE                                                                                   \
               " media format: data format %lu, ch=%lu, SR=%lu, bit_width=%lu, bits_per_sample=%lu, Q_fct=%lu, "       \
               "endianness %lu, interleaving %lu (1->int,  2->deint pack, 3->deint unpack)",                           \
               media_fmt_ptr->data_format,                                                                             \
               media_fmt_ptr->pcm.num_channels,                                                                        \
               media_fmt_ptr->pcm.sample_rate,                                                                         \
               media_fmt_ptr->pcm.bit_width,                                                                           \
               media_fmt_ptr->pcm.bits_per_sample,                                                                     \
               media_fmt_ptr->pcm.q_factor,                                                                            \
               media_fmt_ptr->pcm.endianness,                                                                          \
               media_fmt_ptr->pcm.interleaving);                                                                       \
                                                                                                                       \
      uint8_t   c[WCNTR_MAX_NUM_CHANNELS];                                                                                   \
      uint32_t *c_ptr = (uint32_t *)c;                                                                                 \
      for (uint32_t i = 0; i < media_fmt_ptr->pcm.num_channels; i += 4)                                                \
      {                                                                                                                \
         for (int32_t j = 3, k = 0; j >= 0; j--, k++)                                                                  \
         {                                                                                                             \
            c[i + k] = media_fmt_ptr->pcm.chan_map[i + j];                                                             \
         }                                                                                                             \
      }                                                                                                                \
                                                                                                                       \
      WCNTR_TOPO_MSG(log_id,                                                                                                 \
               DBG_HIGH_PRIO,                                                                                          \
               SPF_LOG_PREFIX IOTYPE                                                                                   \
               " media format:  channel mapping (hex bytes)  %08lx %08lx %08x %08x  %08lx %08lx %08x %08x",            \
               c_ptr[0],                                                                                               \
               c_ptr[1],                                                                                               \
               c_ptr[2],                                                                                               \
               c_ptr[3],                                                                                               \
               c_ptr[4],                                                                                               \
               c_ptr[5],                                                                                               \
               c_ptr[6],                                                                                               \
               c_ptr[7]);                                                                                              \
                                                                                                                       \
   } while (0)

#define WCNTR_TOPO_PRINT_MEDIA_FMT(log_id, module_ptr, out_port_ptr, media_fmt_ptr, IOTYPE)                                  \
   do                                                                                                                  \
   {                                                                                                                   \
      WCNTR_TOPO_MSG(log_id,                                                                                                 \
               DBG_HIGH_PRIO,                                                                                          \
               SPF_LOG_PREFIX "Module 0x%lX (0x%lX): Port Id 0x%lx, propagating? %u, " IOTYPE                          \
                              " media format: data_format %lu, fmt_id 0x%lX",                                          \
               module_ptr->gu.module_instance_id,                                                                      \
               module_ptr->gu.module_id,                                                                               \
               out_port_ptr->gu.cmn.id,                                                                                \
               out_port_ptr->common.flags.media_fmt_event,                                                             \
               media_fmt_ptr->data_format,                                                                             \
               media_fmt_ptr->fmt_id);                                                                                 \
      if (SPF_IS_PACKETIZED_OR_PCM(media_fmt_ptr->data_format))                                                        \
      {                                                                                                                \
         WCNTR_TOPO_PRINT_PCM_MEDIA_FMT(log_id, media_fmt_ptr, IOTYPE);                                                      \
      }                                                                                                                \
   } while (0)
   
   
typedef enum wcntr_topo_sg_state_t
{
   WCNTR_TOPO_SG_STATE_STOPPED   = 0,
   WCNTR_TOPO_SG_STATE_PREPARED  = 1,   
   WCNTR_TOPO_SG_STATE_STARTED   = 2,
   WCNTR_TOPO_SG_STATE_SUSPENDED = 3,   //Not supported currently
   WCNTR_TOPO_SG_STATE_INVALID  = 0XFFFFFFFF,
} wcntr_topo_sg_state_t;

/**
 * Codes for subgraph operations.
 */
typedef enum wcntr_topo_sg_operation_t
{
   WCNTR_TOPO_SG_OP_FLUSH      = 0x00000001,  //Not supported currently
   WCNTR_TOPO_SG_OP_START      = 0x00000004,
   WCNTR_TOPO_SG_OP_PREPARE    = 0x00000008,
   WCNTR_TOPO_SG_OP_STOP       = 0x00000010,
   WCNTR_TOPO_SG_OP_CLOSE      = 0x00000020,
   WCNTR_TOPO_SG_OP_DISCONNECT = 0x00000040,  //Not supported currently
   WCNTR_TOPO_SG_OP_SUSPEND    = 0x00000080,  //Not supported currently
} wcntr_topo_sg_operation_t;

/**
 * Port states. Applicable to internal 
 * Note: the port state have a one to one mapping with wcntr_topo_sg_state_t. 
 * refer wcntr_topo_sg_state_to_port_state()
 */
typedef enum wcntr_topo_port_state_t
{
   WCNTR_TOPO_PORT_STATE_STOPPED = 0,
   WCNTR_TOPO_PORT_STATE_PREPARED,
   WCNTR_TOPO_PORT_STATE_STARTED,
   WCNTR_TOPO_PORT_STATE_SUSPENDED,  //Not supported currently
   WCNTR_TOPO_PORT_STATE_INVALID = 0xFFFFFFFF
} wcntr_topo_port_state_t;

typedef enum wcntr_topo_endianness_t { WCNTR_TOPO_UNKONWN_ENDIAN = 0, WCNTR_TOPO_LITTLE_ENDIAN, WCNTR_TOPO_BIG_ENDIAN } wcntr_topo_endianness_t;

typedef enum wcntr_topo_interleaving_t
{
   WCNTR_TOPO_INTERLEAVING_UNKNOWN = 0,
   WCNTR_TOPO_INTERLEAVED,
   WCNTR_TOPO_DEINTERLEAVED_PACKED,
   WCNTR_TOPO_DEINTERLEAVED_UNPACKED,
} wcntr_topo_interleaving_t;

typedef enum wcntr_topo_port_type_t
{
   WCNTR_TOPO_DATA_INVALID_PORT_TYPE=0,
   WCNTR_TOPO_DATA_INPUT_PORT_TYPE,
   WCNTR_TOPO_DATA_OUTPUT_PORT_TYPE,
   WCNTR_TOPO_CONTROL_PORT_TYPE,
} wcntr_topo_port_type_t;

typedef struct wcntr_topo_buf_t wcntr_topo_buf_t;

struct wcntr_topo_buf_t
{
   int8_t   *data_ptr;
   uint32_t actual_data_len;
   uint32_t max_data_len;           /**< max len is nonzero only when data_ptr is set */
};

/**
 * Media format for PCM and packetized data
 */
typedef struct wcntr_topo_pcm_pack_med_fmt_t
{
   uint32_t                sample_rate;
   uint8_t                 bit_width;        /**< 16, 24, 32 (actual bit width independent of the word size)*/
   uint8_t                 bits_per_sample;  /**< bits per sample 16 or 32. This is actually word size in bits*/
   uint8_t                 q_factor;         /**< 15, 27, 31 */
   uint8_t                 num_channels;
   wcntr_topo_interleaving_t     interleaving;
   wcntr_topo_endianness_t       endianness;
   uint8_t                 chan_map[CAPI_MAX_CHANNELS_V2];
} wcntr_topo_pcm_pack_med_fmt_t;

/**
 * zero means not set with specific value
 *
 */
typedef struct wcntr_topo_media_fmt_t
{
   spf_data_format_t       data_format;
   uint32_t                fmt_id;
   wcntr_topo_pcm_pack_med_fmt_t pcm; /**< when SPF_IS_PACKETIZED_OR_PCM(data_fmt is true)*/
} wcntr_topo_media_fmt_t;

/**
 *  Defined this for the 64 bit destination address used
 *  for registering events with modules
 */
typedef union wcntr_topo_evt_dest_addr_t
{
   uint64_t address;
   struct
   {
      uint32_t src_port;
      uint8_t  src_domain_id;
      uint8_t  dest_domain_id;
   } a;
} wcntr_topo_evt_dest_addr_t;

/**
 *  Used to pass the register event data to containers
 */
typedef struct wcntr_topo_reg_event_t
{
   uint32_t          token;
   uint32_t          event_id;
   capi_buf_t        event_cfg;
   uint32_t          src_port;
   uint8_t           src_domain_id;
   uint8_t           dest_domain_id;
} wcntr_topo_reg_event_t;


/**
 * Convert subgraph state to port state.
 */
static inline wcntr_topo_port_state_t wcntr_topo_sg_state_to_port_state(wcntr_topo_sg_state_t sg_state)
{
   wcntr_topo_port_state_t port_state_array[] = {
      WCNTR_TOPO_PORT_STATE_STOPPED,  // WCNTR_TOPO_SG_STATE_STOPPED
      WCNTR_TOPO_PORT_STATE_PREPARED, // WCNTR_TOPO_SG_STATE_PREPARED
      WCNTR_TOPO_PORT_STATE_STARTED,   // WCNTR_TOPO_SG_STATE_STARTED
      WCNTR_TOPO_PORT_STATE_SUSPENDED  // WCNTR_TOPO_SG_STATE_SUSPENDED
   };

   if (sg_state > SIZE_OF_ARRAY(port_state_array))
   {
      return WCNTR_TOPO_PORT_STATE_INVALID;
   }

   return port_state_array[sg_state];
}


#define WCNTR_TOPO_IS_POW_OF_2(x) (!((x)&((x)-1)))


#define WCNTR_LOG_ID_LOG_MODULE_INSTANCES_SHIFT    6
#define WCNTR_LOG_ID_LOG_DISCONTINUITY_SHIFT       0

/** 64 log modules (only 32 supported now) in a container */
#define WCNTR_LOG_ID_LOG_MODULE_INSTANCES_MASK     0x00000FC0
/** 64 EOS / flush & other discontinuities (see Elite_fwk_extns_logging.h)*/
#define WCNTR_LOG_ID_LOG_DISCONTINUITY_MASK        0x0000003F

bool_t wcntr_topo_is_valid_media_fmt(wcntr_topo_media_fmt_t *med_fmt_ptr);


/**
 * this structure stores hist info only inside a process trigger. not across 2 process triggers.
 */
typedef struct wcntr_topo_process_context_t
{
   uint8_t                           num_in_ports;              /**< max of max input ports across any module */
   uint8_t                           num_out_ports;             /**< max of max output ports across any module */
   capi_stream_data_v2_t              **in_port_sdata_pptr;   /**< array of ptr to sdata for calling capi process. size num_in_ports. Must index with in_port_ptr->gu.index*/
   capi_stream_data_v2_t              **out_port_sdata_pptr;  /**< array of ptr to sdata for calling capi process. size num_out_ports. Must index with  out_port_ptr->gu.index */
   wcntr_topo_trigger_t                 curr_trigger;           /**< current trigger that caused process frames */
} wcntr_topo_process_context_t;


typedef struct wcntr_topo_init_data_t
{
   /** input */
   const wcntr_topo_to_cntr_vtable_t            *wcntr_topo_to_cntr_vtble_ptr;  /**< vtable for topo to call back containers */

 } wcntr_topo_init_data_t;

typedef uint32_t (*wcntr_gpr_callback_t)(gpr_packet_t *, void *) ;

typedef capi_err_t (*wcntr_topo_capi_callback_f)(void *context_ptr, capi_event_id_t id, capi_event_info_t *event_info_ptr);

/** when graph is created, it results in output such as required stack size etc*/
typedef struct wcntr_topo_graph_init_t
{
   /** input */
   spf_handle_t *             spf_handle_ptr;
   wcntr_gpr_callback_t             gpr_cb_fn;

   wcntr_topo_capi_callback_f       capi_cb;          /**< CAPI callback function */

   /** output */
   uint32_t                   max_stack_size;   /**< max stack size required for the topo (max of all CAPIs) */

} wcntr_topo_graph_init_t;


typedef union wcntr_topo_capi_event_flag_t
{
   struct
   {
      uint32_t    kpps: 1;                /**< Set when kpps and kpss scale factor are changed by the module */
      uint32_t    bw : 1;                 /**< Set when code or data BW are changed by the module */
      uint32_t    port_thresh : 1;        /**< at least one threshold event is pending; thresh prop is pending */
      uint32_t    process_state : 1;
      uint32_t    media_fmt_event : 1;    /**< to track any change in output media fmt of a module and if so, to trigger media fmt dependent global
                                                operations such as threshold, kpps/bw. etc */
   };
   uint32_t word;
} wcntr_topo_capi_event_flag_t;

typedef struct wcntr_topo_flags_t
{
   uint32_t any_data_trigger_policy: 1;   /**< is there any module which raised data trigger policy extn FWK_EXTN_TRIGGER_POLICY
                                                                  (note: currently if trigger policy extn is removed this is not reset.*/

   uint32_t is_signal_triggered : 1;      /**< whether container is signal triggered */
 } wcntr_topo_flags_t;

typedef struct wcntr_topo_t
{
   wcntr_gu_t                          gu;                        /**< Graph utils. */
   wcntr_topo_process_context_t    proc_context;              /**< history data required for data processing */
   wcntr_topo_capi_event_flag_t    capi_event_flag;
   const wcntr_topo_to_cntr_vtable_t   *topo_to_cntr_vtable_ptr;  /**< vtable with topo to container functions*/
   uint8_t                       module_count;              /**< tracks modules created in this container for logging purposes (only increasing count).
                                                                 See WCNTR_LOG_ID_LOG_MODULE_INSTANCES_MASK)
                                                                 Every module create adds one count.
                                                                 This uniquely identifies a module in this container */
   wcntr_topo_flags_t              flags;
   POSAL_HEAP_ID                 heap_id;                   /*Heap ID used for all memory allocations in the topology*/
   bool_t mf_propagation_done;
} wcntr_topo_t;


/**
 * Container level subgraph - holds state information.
 */
typedef struct wcntr_topo_sg_t
{
   wcntr_gu_sg_t           gu; /**<  Must be the first element. */
   wcntr_topo_sg_state_t   state;
   bool_t can_mf_be_propagated;
} wcntr_topo_sg_t;



/**
 * A bitfield for module flags
 */
typedef union wcntr_topo_module_flags_t
{
   struct
   {
      uint32_t inplace             : 1;
      uint32_t requires_data_buf   : 1;
      uint32_t disabled            : 1;         /**< by default modules are enabled.*/
      uint32_t active              : 1;         /**< caches module active flag: active = SG started && ports started (src,sink,siso cases) && (module bypassed||fwk module||!disabled) etc*/

      uint32_t need_data_trigger_in_st  : 2;    /**< 2 bit flag to indicate if data_trigger is needed in signal triggered container.
                                                          By default data triggers are not handled in signal triggered container unless this flag is set by the event or propagation.
                                                          #FWK_EXTN_EVENT_ID_DATA_TRIGGER_IN_ST_CNTR
                                                          This flag is used to evaluate the topo trigger policy of signal trigger container when a subgraph opens/closes.*/

      uint32_t is_dm_disabled           : 1;    /**< Indicates if modules has enabled/disabled dm >*/
      uint32_t dm_mode                  : 2;    /**< Indicates operating dm mode [value range is enum gen_topo_dm_mode_t] */

      uint32_t is_nblc_boundary_module	 : 1;    /**< Indicates if module is at the nblc boundary >*/

      /** Flags which indicate whether framework extension is required for the module */
      uint32_t need_stm_extn            : 1;    /**< FWK_EXTN_STM (SIGNAL TRIGGERED MODULE)*/
      uint32_t need_pcm_extn            : 1;    /**< FWK_EXTN_PCM */
      uint32_t need_mp_buf_extn         : 1;    /**< FWK_EXTN_MULTI_PORT_BUFFERING */
      uint32_t need_trigger_policy_extn : 1;    /**< FWK_EXTN_TRIGGER_POLICY */
      uint32_t need_cntr_frame_dur_extn : 1;    /**< FWK_EXTN_CONTAINER_FRAME_DURATION*/
      uint32_t need_thresh_cfg_extn     : 1;    /**< FWK_EXTN_THRESHOLD_CONFIGURATION*/
      uint32_t need_proc_dur_extn       : 1;    /**< FWK_EXTN_PARAM_ID_CONTAINER_PROC_DELAY */
      uint32_t need_dm_extn             : 1;    /**< FWK_EXTN_DM */

      /** Flags which record the interface extensions supported by the module
       * Other extensions such as INTF_EXTN_IMCL, INTF_EXTN_PATH_DELAY are not stored */
      uint32_t supports_metadata       : 1;       /**< INTF_EXTN_METADATA: this also means module propagates metadata */
      uint32_t supports_data_port_ops  : 1;       /**< INTF_EXTN_DATA_PORT_OPERATION: this also means module requires data port states. */
      uint32_t supports_prop_port_ds_state : 1;   /**< INTF_EXTN_PROP_PORT_DS_STATE */
      uint32_t supports_prop_is_rt_port_prop : 1; /**< INTF_EXTN_PROP_IS_RT_PORT_PROPERTY */

   };
   uint32_t word;
} wcntr_topo_module_flags_t;

typedef struct wcntr_topo_cached_param_node_t
{
   spf_cfg_data_type_t payload_type; /**< persistent, shared persistent and default*/
   uint32_t           param_id;     /**< cached Param ID*/
   uint32_t           param_size;   /**< size of the payload*/
   int8_t *           payload_ptr;  /**< pointer to the payload*/
} wcntr_topo_cached_param_node_t;

typedef struct wcntr_topo_cached_event_node_t
{
   capi_register_event_to_dsp_client_v2_t reg_event_payload;
} wcntr_topo_cached_event_node_t;



typedef struct wcntr_topo_module_t
{
   wcntr_gu_module_t                               gu;                  /**< Must be the first element. */
   capi_t                                    *capi_ptr;
   wcntr_topo_t                                *topo_ptr;
   uint32_t                                  kpps;                /**< Total kpps of this capi for all its ports. needed to check for change. */
   uint32_t                                  algo_delay;          /**< us, for single port modules */
   uint32_t                                  code_bw;             /**< BW in Bytes per sec */
   uint32_t                                  data_bw;             /**< BW in Bytes per sec */
   wcntr_topo_module_flags_t                   flags;
   wcntr_topo_module_bypass_t *                bypass_ptr;          /**< if module is disabled & can be bypassed, then it will use this mem to park prev media fmt, kpps etc.
                                                                       note: most SISO modules are bypassed when disabled. MIMO modules cannot be bypassed.
                                                                       If bypass is not possible, then module proc will be called and module needs to take care of out media, kpps etc */
   uint32_t                                  num_proc_loops;      /**< in case LCM thresh is used, then num process loop that need to be called per module */

   /** per port properties applicable to SISO modules only (hence stored in module)*/

   uint8_t                                   serial_num;          /**< serial number assigned to the module, when it's first created (See WCNTR_LOG_ID_LOG_MODULE_INSTANCES_MASK) */
   uint32_t                                  kpps_scale_factor_q4;   /**< Multiply by scale factor for running at higher speeds if module sets this flag, qfactor Q28.4 */
   bool_t can_process_be_called;

} wcntr_topo_module_t;



typedef struct wcntr_topo_module_debug_t
{
   uint32_t  module_id;   
   uint32_t  module_instance_id;
   uint32_t  module_type;                            
   uint32_t  itype;      
   uint32_t  max_input_ports;
   uint32_t  max_output_ports;
   uint32_t  num_input_ports;
   uint32_t  num_output_ports;
   uint32_t  num_ctrl_ports;
   uint32_t  kpps;                
   uint32_t  algo_delay;          
   uint32_t  code_bw;             
   uint32_t  data_bw;            
   uint32_t  module_bypassed;            
   uint32_t  can_process_be_called;

} wcntr_topo_module_debug_t;


typedef union wcntr_topo_port_flags_t
{
   struct
   {
      /** Permanent    */
      uint32_t       port_has_threshold : 1;       /**< module has threshold if it returns thresh >= 1. */
      uint32_t       requires_data_buf : 1;        /**< TRUE if CAPI requires framework to do buffering.
                                                        If FALSE and module has threshold, then module will not be called unless input threshold is met
                                                        (encoder, module like EC) & for output, output must have thresh amount of empty space.
                                                        Enc may also use requires_data_buf = TRUE and do thresh check inside. See comment in wcntr_topo_capi_get_port_thresh*/
      
      uint32_t       is_mf_valid : 1;              /**< cached value of wcntr_topo_is_valid_media_fmt(wcntr_topo_common_port_t::media_fmt) for minimizing run time overhead.
                                                         CU ports dont have this flag as they are not frequently checked for validity.*/

      /** temporary: cleared after event handling */
      uint32_t       media_fmt_event : 1;
     };
   uint32_t          word;

} wcntr_topo_port_flags_t;


typedef struct wcntr_topo_common_port_t
{
   wcntr_topo_port_state_t             state;                     /**< Downgraded state = downgrade of (self state, connected port state)*/
 
   wcntr_topo_media_fmt_t              media_fmt;                 /**< Media format from CAPI*/

   wcntr_topo_buf_t                     *bufs_ptr; ;                     
   uint32_t                      max_buf_len;               /**< this is the max length of the buffer, assigned based on thresh propagation,
                                                                  not necessarily same as module raised threshold.
                                                                  max_data_len in buf struct is for current data_ptr.*/
   uint32_t						max_buf_len_per_buf;
   capi_stream_data_v2_t         sdata;                     /**< buf in sdata points to a buf from process context*/
   wcntr_topo_port_flags_t         flags;

   /** Events update the below fields temporarily */
   uint32_t                      port_event_new_threshold;  /**< thresh from module is assigned here.
                                                                  thresh propagation also assigned here. 0 means no change in thresh.
                                                                  1 means no buffer needed.
                                                                  cleared after event is handled. flags.create_buf decides if the buf is created.*/
   intf_extn_data_port_opcode_t  last_issued_opcode;        /**< Last issued port operation is stored to avoid consecutively
                                                              setting same operation.*/


   bool_t state_updated_during_propagation;
} wcntr_topo_common_port_t;


typedef union wcntr_topo_input_port_flags_t
{
   struct
   {
           uint32_t       media_fmt_received : 1;          /**< indicates if this input port has received media format at least once */  
   };
   uint32_t          word;
} wcntr_topo_input_port_flags_t;


typedef struct wcntr_topo_input_port_t
{
   wcntr_gu_input_port_t               gu;                  /**< Must be the first element */
   wcntr_topo_common_port_t        common;
   wcntr_topo_input_port_flags_t   flags;
} wcntr_topo_input_port_t;

typedef struct wcntr_topo_output_port_t
{
   wcntr_gu_output_port_t              gu;                /**< Must be the first element */
   wcntr_topo_common_port_t        common;
  } wcntr_topo_output_port_t;

typedef struct wcntr_topo_ctrl_port_t
{
   wcntr_gu_ctrl_port_t               gu;                /**< Must be the first element */
   wcntr_topo_port_state_t            state;
   
   intf_extn_imcl_port_opcode_t last_issued_opcode; /* Cache the last issued opcode to protect against repetitive issues to module.
                                                        For example, we cannot issue CLOSE twice to a module. */
} wcntr_topo_ctrl_port_t;


static inline spf_data_format_t wcntr_topo_convert_public_data_fmt_to_spf_data_format(uint32_t data_format)
{
   const spf_data_format_t mapper[] = {
         SPF_UNKNOWN_DATA_FORMAT,
         SPF_FIXED_POINT,         //DATA_FORMAT_FIXED_POINT = 1
         SPF_IEC61937_PACKETIZED, //DATA_FORMAT_IEC61937_PACKETIZED = 2
         SPF_IEC60958_PACKETIZED, //DATA_FORMAT_IEC60958_PACKETIZED = 3
         SPF_DSD_DOP_PACKETIZED,  //DATA_FORMAT_DSD_OVER_PCM = 4
         SPF_GENERIC_COMPRESSED,  //DATA_FORMAT_GENERIC_COMPRESSED = 5
         SPF_RAW_COMPRESSED,      //DATA_FORMAT_RAW_COMPRESSED = 6
         SPF_UNKNOWN_DATA_FORMAT, //To be added: SPF_FLOATING_POINT,
         SPF_UNKNOWN_DATA_FORMAT, //To be added: SPF_COMPR_OVER_PCM_PACKETIZED,
   };
   if (data_format >= SIZE_OF_ARRAY(mapper))
   {
      return SPF_UNKNOWN_DATA_FORMAT;
   }
   else
   {

      return mapper[data_format];
   }
}

static inline uint32_t wcntr_topo_convert_spf_data_fmt_public_data_format(spf_data_format_t spf_fmt)
{
   const uint32_t mapper[] = {
      0,                               // SPF_UNKNOWN_DATA_FORMAT = 0,
      DATA_FORMAT_FIXED_POINT,         // SPF_FIXED_POINT = 1,
      0,                               // SPF_FLOATING_POINT = 2,
      DATA_FORMAT_RAW_COMPRESSED,      // SPF_RAW_COMPRESSED = 3,
      DATA_FORMAT_IEC61937_PACKETIZED, // SPF_IEC61937_PACKETIZED = 4,
      DATA_FORMAT_DSD_OVER_PCM,        // SPF_DSD_DOP_PACKETIZED = 5,
      0,                               // SPF_COMPR_OVER_PCM_PACKETIZED = 6,
      DATA_FORMAT_GENERIC_COMPRESSED,  // SPF_GENERIC_COMPRESSED = 7,
      0,                               // SPF_IEC60958_PACKETIZED = 8,
      0,                               // SPF_MAX_FORMAT_TYPE = 0x7FFFFFFF
   };

   if (spf_fmt >= SIZE_OF_ARRAY(mapper))
   {
      return 0;
   }
   else
   {

      return mapper[spf_fmt];
   }
}

// clang-format on

static inline wcntr_topo_interleaving_t wcntr_topo_convert_public_interleaving_to_topo_interleaving(
   uint16_t pcm_interleaving)
{
   wcntr_topo_interleaving_t topo_interleaving = WCNTR_TOPO_INTERLEAVING_UNKNOWN;
   switch (pcm_interleaving)
   {
      case PCM_INTERLEAVED:
         topo_interleaving = WCNTR_TOPO_INTERLEAVED;
         break;
      case PCM_DEINTERLEAVED_PACKED:
         topo_interleaving = WCNTR_TOPO_DEINTERLEAVED_PACKED;
         break;
      case PCM_DEINTERLEAVED_UNPACKED:
         topo_interleaving = WCNTR_TOPO_DEINTERLEAVED_UNPACKED;
         break;
      default:
         break;
   }
   return topo_interleaving;
}

static inline uint16_t gen_topo_convert_gen_wcntr_topo_interleaving_to_public_interleaving(
   wcntr_topo_interleaving_t topo_interleaving)
{
   uint16_t pcm_interleaving = PCM_INTERLEAVED;
   switch (topo_interleaving)
   {
      case WCNTR_TOPO_INTERLEAVED:
         pcm_interleaving = PCM_INTERLEAVED;
         break;
      case WCNTR_TOPO_DEINTERLEAVED_PACKED:
         pcm_interleaving = PCM_DEINTERLEAVED_PACKED;
         break;
      case WCNTR_TOPO_DEINTERLEAVED_UNPACKED:
         pcm_interleaving = PCM_DEINTERLEAVED_UNPACKED;
         break;
      default:
         break;
   }
   return pcm_interleaving;
}

static inline wcntr_topo_endianness_t wcntr_topo_convert_public_endianness_to_topo_endianness(uint16_t pcm_endianness)
{
   wcntr_topo_endianness_t gen_topo_endianness = WCNTR_TOPO_UNKONWN_ENDIAN;
   switch (pcm_endianness)
   {
      case PCM_LITTLE_ENDIAN:
         gen_topo_endianness = WCNTR_TOPO_LITTLE_ENDIAN;
         break;
      case PCM_BIG_ENDIAN:
         gen_topo_endianness = WCNTR_TOPO_BIG_ENDIAN;
         break;
      default:
         break;
   }
   return gen_topo_endianness;
}

static inline uint16_t wcntr_topo_convert_wcntr_topo_endianness_to_public_endianness(wcntr_topo_endianness_t gen_topo_endianness)
{
   uint16_t pcm_endianness = 0;
   switch (gen_topo_endianness)
   {
      case WCNTR_TOPO_LITTLE_ENDIAN:
         pcm_endianness = PCM_LITTLE_ENDIAN;
         break;
      case WCNTR_TOPO_BIG_ENDIAN:
         pcm_endianness = PCM_BIG_ENDIAN;
         break;
      default:
         break;
   }
   return pcm_endianness;
}

/**
 * index by capiv2 data format (index cannot be MAX_FORMAT_TYPE)
 */
static inline spf_data_format_t wcntr_topo_convert_capi_data_format_to_spf_data_format(data_format_t capi_data_format)
{
   if (capi_data_format > CAPI_IEC60958_PACKETIZED_NON_LINEAR)
   {
      return SPF_UNKNOWN_DATA_FORMAT;
   }
   else
   {
      const spf_data_format_t mapper[] =
         { SPF_FIXED_POINT,         SPF_FLOATING_POINT,      SPF_RAW_COMPRESSED,
           SPF_IEC61937_PACKETIZED, SPF_DSD_DOP_PACKETIZED,  SPF_COMPR_OVER_PCM_PACKETIZED,
           SPF_GENERIC_COMPRESSED,  SPF_IEC60958_PACKETIZED, SPF_IEC60958_PACKETIZED_NON_LINEAR };

      return mapper[capi_data_format];
   }
}

/**
 * index by capiv2 data format (index cannot be MAX_FORMAT_TYPE)
 */
static inline data_format_t wcntr_topo_convert_spf_data_format_to_capi_data_format(spf_data_format_t spf_data_fmt)
{
   if (spf_data_fmt > SPF_IEC60958_PACKETIZED_NON_LINEAR)
   {
      return CAPI_MAX_FORMAT_TYPE;
   }
   else
   {
      const data_format_t mapper[] = { CAPI_MAX_FORMAT_TYPE,
                                       CAPI_FIXED_POINT,
                                       CAPI_FLOATING_POINT,
                                       CAPI_RAW_COMPRESSED,
                                       CAPI_IEC61937_PACKETIZED,
                                       CAPI_DSD_DOP_PACKETIZED,
                                       CAPI_COMPR_OVER_PCM_PACKETIZED,
                                       CAPI_GENERIC_COMPRESSED,
                                       CAPI_IEC60958_PACKETIZED,
                                       CAPI_IEC60958_PACKETIZED_NON_LINEAR };

      return mapper[spf_data_fmt];
   }
}

static inline capi_interleaving_t wcntr_topo_convert_wcntr_topo_interleaving_to_capi_interleaving(
   wcntr_topo_interleaving_t gen_topo_int)
{
   capi_interleaving_t capi_int = CAPI_INVALID_INTERLEAVING;
   switch (gen_topo_int)
   {
      case WCNTR_TOPO_INTERLEAVED:
         capi_int = CAPI_INTERLEAVED;
         break;
      case WCNTR_TOPO_DEINTERLEAVED_PACKED:
         capi_int = CAPI_DEINTERLEAVED_PACKED;
         break;
      case WCNTR_TOPO_DEINTERLEAVED_UNPACKED:
         capi_int = CAPI_DEINTERLEAVED_UNPACKED;
         break;
      default:
         break;
   }

   return capi_int;
}

static inline wcntr_topo_interleaving_t wcntr_topo_convert_capi_interleaving_to_topo_interleaving(
   capi_interleaving_t capi_int)
{
   wcntr_topo_interleaving_t gen_topo_int = WCNTR_TOPO_INTERLEAVING_UNKNOWN;
   switch (capi_int)
   {
      case CAPI_INTERLEAVED:
         gen_topo_int = WCNTR_TOPO_INTERLEAVED;
         break;
      case CAPI_DEINTERLEAVED_PACKED:
         gen_topo_int = WCNTR_TOPO_DEINTERLEAVED_PACKED;
         break;
      case CAPI_DEINTERLEAVED_UNPACKED:
         gen_topo_int = WCNTR_TOPO_DEINTERLEAVED_UNPACKED;
         break;
      default:
         break;
   }

   return gen_topo_int;
}

static inline void wcntr_topo_set_bits(uint32_t *x_ptr, uint32_t val, uint32_t mask, uint32_t shift)
{
   val    = (val << shift) & mask;
   *x_ptr = (*x_ptr & ~mask) | val;
}

static inline uint32_t wcntr_topo_get_bits(uint32_t x, uint32_t mask, uint32_t shift)
{
   return (x & mask) >> shift;
}

static inline uint32_t wrcntr_topo_get_total_max_len(wcntr_topo_common_port_t *cmn_port_ptr)
{
   if (cmn_port_ptr->bufs_ptr)
   {
      return (cmn_port_ptr->bufs_ptr[0].max_data_len * cmn_port_ptr->sdata.bufs_num);
   }
   else
   {
      return 0;
   }
}

static inline uint32_t wrcntr_topo_get_total_actual_len(wcntr_topo_common_port_t *cmn_port_ptr)
{
   uint32_t total_actual_len=0;
   if(cmn_port_ptr->bufs_ptr)
   	{
   total_actual_len = cmn_port_ptr->bufs_ptr[0].actual_data_len * cmn_port_ptr->sdata.bufs_num;
   	}  
   return total_actual_len;
}

static inline uint32_t wcntr_topo_get_out_port_data_len(void *topo_ctx_ptr, void *out_port_ctx_ptr, bool_t is_max)
{
   wcntr_topo_output_port_t *out_port_ptr = (wcntr_topo_output_port_t *)out_port_ctx_ptr;
   wcntr_topo_common_port_t *cmn_port_ptr = &out_port_ptr->common;
   return is_max ? wrcntr_topo_get_total_max_len(cmn_port_ptr) : wrcntr_topo_get_total_actual_len(cmn_port_ptr);
}


static inline void wcntr_topo_set_all_bufs_len_to_zero(wcntr_topo_common_port_t *cmn_port_ptr)
{
   for (uint32_t b = 0; b < cmn_port_ptr->sdata.bufs_num; b++)
   {
      cmn_port_ptr->bufs_ptr[b].actual_data_len = 0;
   }
}


///* =======================================================================
// Public Function Declarations
//========================================================================== */
//

ar_result_t wcntr_topo_init_topo(wcntr_topo_t *topo_ptr, wcntr_topo_init_data_t *topo_init_data_ptr, POSAL_HEAP_ID heap_id);
ar_result_t wcntr_topo_destroy_topo(wcntr_topo_t *topo_ptr);

void wcntr_topo_reset_top_level_flags(wcntr_topo_t *topo_ptr);
ar_result_t wcntr_topo_reset_module(wcntr_topo_t *topo_ptr, wcntr_topo_module_t *module_ptr);
ar_result_t wcntr_topo_basic_reset_input_port(wcntr_topo_t *me_ptr, wcntr_topo_input_port_t *in_port_ptr);
ar_result_t wcntr_topo_shared_reset_input_port(void *topo_ptr, void *topo_out_port_ptr);
static inline ar_result_t wcntr_topo_reset_input_port(void *topo_ptr, void *topo_in_port_ptr)
{
   return wcntr_topo_shared_reset_input_port(topo_ptr, topo_in_port_ptr);
}
ar_result_t wcntr_topo_basic_reset_output_port(wcntr_topo_t *me_ptr, wcntr_topo_output_port_t *out_port_ptr);
ar_result_t wcntr_topo_shared_reset_output_port(void *topo_ptr, void *topo_out_port_ptr);
static inline ar_result_t wcntr_topo_reset_output_port(void *topo_ptr, void *topo_out_port_ptr)
{
   return wcntr_topo_shared_reset_output_port(topo_ptr, topo_out_port_ptr);
}

ar_result_t wcntr_topo_reset_all_out_ports(wcntr_topo_module_t *module_ptr);
ar_result_t wcntr_topo_reset_all_in_ports(wcntr_topo_module_t *module_ptr);

ar_result_t wcntr_topo_algo_reset(void *   topo_module_ptr,
                                uint32_t log_id,
                                bool_t   is_port_valid,
                                bool_t   is_input,
                                uint16_t port_index);

wcntr_topo_sg_state_t wcntr_topo_get_sg_state(wcntr_gu_sg_t *sg_ptr);

void wcntr_topo_set_sg_state(wcntr_gu_sg_t *sg_ptr, wcntr_topo_sg_state_t state);

ar_result_t wcntr_topo_update_ctrl_port_state(void *           vtopo_ptr,
                                            wcntr_topo_port_type_t port_type,
                                            void *           port_ptr,
                                            wcntr_topo_sg_state_t  state);

/**-------------------------------- gen_topo_pm ---------------------------------*/
ar_result_t wcntr_topo_aggregate_kpps_bandwidth(wcntr_topo_t *topo_ptr,
                                              bool_t      only_aggregate,
                                              uint32_t *  aggregate_kpps_ptr,
                                              uint32_t *  aggregate_bw_ptr,
                                              uint32_t *  scaled_kpps_q10_agg_ptr);

/*
 * Basic topo process call that can be used from CdC and WCNTR
 * */
ar_result_t wcntr_topo_topo_process(wcntr_topo_t *topo_ptr, wcntr_gu_module_list_t **start_module_list_pptr);

//////////////////////////////////////  GEN_TOPO_TOPO_HANDLER_H_
ar_result_t wcntr_topo_query_and_create_capi(wcntr_topo_t *           topo_ptr,
                                           wcntr_topo_graph_init_t *graph_init_ptr,
                                           wcntr_topo_module_t *    module_ptr);

ar_result_t wcntr_topo_set_get_data_port_properties(wcntr_topo_module_t *module_ptr,
                                                  wcntr_topo_t *       topo_ptr);

ar_result_t wcntr_topo_create_modules(wcntr_topo_t *topo_ptr, wcntr_topo_graph_init_t *graph_init_ptr);

ar_result_t wcntr_topo_destroy_modules(wcntr_topo_t *topo_ptr, spf_cntr_sub_graph_list_t *spf_sg_list_ptr);
void wcntr_topo_destroy_module(wcntr_topo_t *topo_ptr, wcntr_topo_module_t *module_ptr);

ar_result_t wcntr_topo_propagate_media_fmt(void *topo_ptr, bool_t is_data_path);
ar_result_t wcntr_topo_propagate_media_fmt_from_module(void *            cxt_ptr,
                                                     bool_t            is_data_path,
                                                     wcntr_gu_module_list_t *start_module_list_ptr);

bool_t wcntr_topo_is_module_sg_stopped(wcntr_topo_module_t *module_ptr);
bool_t wcntr_topo_is_module_sg_stopped_or_suspended(wcntr_topo_module_t *module_ptr);
bool_t wcntr_topo_is_module_sg_started(wcntr_topo_module_t *module_ptr);

uint32_t wcntr_topo_get_curr_port_threshold(wcntr_topo_common_port_t *port_ptr);

uint32_t wcntr_topo_get_one_ms_buffer_size_in_bytes(wcntr_topo_common_port_t *port_ptr);

uint32_t wcntr_topo_get_default_port_threshold(wcntr_topo_module_t *module_ptr, wcntr_topo_media_fmt_t *media_fmt_ptr);

ar_result_t wcntr_topo_check_and_set_default_port_threshold(wcntr_topo_module_t *     module_ptr,
                                                          wcntr_topo_common_port_t *cmn_port_ptr);

///////////////////////////////////////// GEN_TOPO_TOPO_HANDLER_H_

//////////////////////////////////////////// GEN_TOPO_FWK_EXTN_UTILS_H

ar_result_t wcntr_topo_fmwk_extn_handle_at_init(wcntr_topo_t *topo_ptr, wcntr_topo_module_t *module_ptr);
ar_result_t wcntr_topo_fmwk_extn_handle_at_deinit(wcntr_topo_t *topo_ptr, wcntr_topo_module_t *module_ptr);

//////////////////////////////////////////////// GEN_TOPO_FWK_EXTN_UTILS_H

//////////////////////////////////////////////// GEN_TOPO_INTF_EXTN_UTILS_H

ar_result_t wcntr_topo_intf_extn_handle_at_init(wcntr_topo_t *topo_ptr, wcntr_topo_module_t *module_ptr);

/////////////////////////////////////////////// gen_topo_get_capi_callback_handler

capi_err_t wcntr_topo_handle_process_state_event(wcntr_topo_t *       topo_ptr,
                                               wcntr_topo_module_t *module_ptr,
                                               capi_event_info_t *event_info_ptr);

capi_err_t wcntr_topo_handle_output_media_format_event(void *             ctxt_ptr,
                                                     void *             module_ctxt_ptr,
                                                     capi_event_info_t *event_info_ptr,
                                                     bool_t             is_std_fmt_v2,
                                                     bool_t             is_pending_data_valid);

// Topo_cmn capi v2 event callback handling.
capi_err_t wcntr_topo_capi_callback_non_island(void *context_ptr, capi_event_id_t id, capi_event_info_t *event_info_ptr);

capi_err_t wcntr_topo_capi_callback(void *context_ptr, capi_event_id_t id, capi_event_info_t *event_info_ptr);

/////////////////////////////////////////////// gen_topo_get_capi_callback_handler

/////////////////////////////////////////////// gen_topo_internal_buffer_utils

ar_result_t wcntr_topo_destroy_cmn_port(wcntr_topo_module_t *     module_ptr,
                                        wcntr_topo_common_port_t *cmn_port_ptr,
                                        uint32_t                  log_id,
                                        uint32_t                  port_id,
                                        bool_t                    is_input);

ar_result_t wcntr_topo_destroy_input_port(wcntr_topo_t *me_ptr, wcntr_topo_input_port_t *in_port_ptr);

ar_result_t wcntr_topo_destroy_output_port(wcntr_topo_t *me_ptr, wcntr_topo_output_port_t *out_port_ptr);

/////////////////////////////////////////////// gen_topo_internal_buffer_utils
ar_result_t wcntr_topo_set_event_reg_prop_to_capi_modules(uint32_t           log_id,
                                                        capi_t *           capi_ptr,
                                                        wcntr_topo_module_t *module_ptr,
                                                        wcntr_topo_reg_event_t * payload_ptr,
                                                        bool_t             is_register,
                                                        bool_t *           store_client_info_ptr);

////////////////////////////////////////////// metadata

//////////////////////////////// gen_topo_data_process.cpp

uint32_t wcntr_topo_get_bufs_num_from_med_fmt(wcntr_topo_media_fmt_t *med_fmt_ptr);


ar_result_t wcntr_topo_initialize_bufs_sdata(wcntr_topo_t *            topo_ptr,
                                           wcntr_topo_common_port_t *cmn_port_ptr,
                                           uint32_t                miid,
                                           uint32_t                port_id,bool_t update_data_ptr,
                                           int8_t *data_ptr);


/////////////// TOPO propagation ////////////////////////////

static inline bool_t wcntr_topo_is_stm_module(wcntr_topo_module_t *module_ptr)
{
   return ((module_ptr->flags.need_stm_extn > 0) ? TRUE : FALSE);
}

/** ------------------------------------------- data flow state -----------------------------------------------------*/
bool_t wcntr_topo_check_if_all_src_are_ftrt_n_at_gap(wcntr_topo_t *topo_ptr, bool_t *is_ftrt_ptr);

// Module bypass

typedef struct wcntr_topo_module_bypass_t
{
   wcntr_topo_media_fmt_t media_fmt; /**< output media fmt */
   uint32_t         kpps;
   uint32_t         algo_delay; // us
   uint32_t         code_bw;
   uint32_t         data_bw;
   uint32_t         in_thresh_bytes_all_ch;
   uint32_t         out_thresh_bytes_all_ch;
} wcntr_topo_module_bypass_t;

ar_result_t wcntr_topo_check_create_bypass_module(wcntr_topo_t *topo_ptr, wcntr_topo_module_t *module_ptr);
ar_result_t wcntr_topo_check_destroy_bypass_module(wcntr_topo_t *topo_ptr, wcntr_topo_module_t *module_ptr);

//

ar_result_t wcntr_topo_set_ctrl_port_properties(wcntr_topo_module_t *module_ptr,
                                              wcntr_topo_t *       topo_ptr,
                                              bool_t             is_placeholder_replaced);

ar_result_t wcntr_topo_handle_incoming_ctrl_intent(void *   topo_ctrl_port_ptr,
                                                 void *   intent_buf,
                                                 uint32_t max_size,
                                                 uint32_t actual_size);

ar_result_t wcntr_topo_set_ctrl_port_operation(wcntr_gu_ctrl_port_t *             gu_ctrl_port_ptr,
                                             intf_extn_imcl_port_opcode_t opcode,
                                             POSAL_HEAP_ID                heap_id);

ar_result_t wcntr_topo_check_set_connected_ctrl_port_operation(uint32_t                   log_id,
                                                             wcntr_topo_module_t *        this_module_ptr,
                                                             wcntr_topo_ctrl_port_t *     connected_port_ptr,
                                                             spf_cntr_sub_graph_list_t *spf_sg_list_ptr,
                                                             uint32_t                   sg_ops);

ar_result_t wcntr_topo_check_set_self_ctrl_port_operation(uint32_t              log_id,
                                                        wcntr_topo_ctrl_port_t *topo_ctrl_port_ptr,
                                                        uint32_t              sg_ops);

ar_result_t wcntr_topo_set_ctrl_port_state(void *ctx_ptr, wcntr_topo_port_state_t state);

ar_result_t wcntr_topo_from_sg_state_set_ctrl_port_state(void *ctx_ptr, wcntr_topo_sg_state_t new_state);

ar_result_t wcntr_topo_intf_extn_data_ports_hdl_at_init(wcntr_topo_t *topo_ptr, wcntr_topo_module_t *module_ptr);

ar_result_t wcntr_topo_capi_set_data_port_op(wcntr_topo_module_t *           module_ptr,
                                           intf_extn_data_port_opcode_t  opcode,
                                           intf_extn_data_port_opcode_t *last_issued_opcode_ptr,
                                           bool_t                        is_input,
                                           uint32_t                      port_index,
                                           uint32_t                      port_id);

ar_result_t wcntr_topo_capi_set_data_port_op_from_state(wcntr_topo_module_t *           module_ptr,
                                                      wcntr_topo_port_state_t             downgraded_state,
                                                      intf_extn_data_port_opcode_t *last_issued_opcode_ptr,
                                                      bool_t                        is_input,
                                                      uint32_t                      port_index,
                                                      uint32_t                      port_id);

ar_result_t wcntr_topo_capi_set_data_port_op_from_sg_state(wcntr_topo_module_t *           module_ptr,
                                                         wcntr_topo_sg_state_t               sg_state,
                                                         intf_extn_data_port_opcode_t *last_issued_opcode_ptr,
                                                         bool_t                        is_input,
                                                         uint32_t                      port_index,
                                                         uint32_t                      port_id);

ar_result_t wcntr_topo_capi_set_data_port_op_from_sg_ops(wcntr_topo_module_t *         module_ptr,
                                                         uint32_t                      sg_ops,
                                                         intf_extn_data_port_opcode_t *last_issued_opcode_ptr,
                                                         bool_t                        is_input,
                                                         uint32_t                      port_index,
                                                         uint32_t                      port_id);

ar_result_t wcntr_topo_capi_set_data_port_op_from_data_port_state(wcntr_topo_module_t *         module_ptr,
                                                                  wcntr_topo_port_state_t       data_port_state,
                                                                  intf_extn_data_port_opcode_t *last_issued_opcode_ptr,
                                                                  bool_t                        is_input,
                                                                  uint32_t                      port_index,
                                                                  uint32_t                      port_id);
intf_extn_imcl_port_opcode_t wcntr_topo_port_state_to_ctrl_port_opcode(wcntr_topo_port_state_t topo_state);
intf_extn_data_port_opcode_t wcntr_topo_port_state_to_data_port_opcode(wcntr_topo_port_state_t topo_state);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef WC_TOPO_H_
