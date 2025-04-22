#ifndef WCNTR_I_H
#define WCNTR_I_H
/**
 * \file wear_cntr_i.h
 * \brief
 *   This file contains structure definitions for cd
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "apm_container_api.h"
#include "apm_api.h"
#include "apm_sub_graph_api.h"
#include "apm_container_api.h"
#include "apm_module_api.h"
#include "apm_cntr_if.h"
#include "cntr_cntr_if.h"

#include "module_cmn_api.h"
#include "capi_mm_error_code_converter.h"

#include "wc_graph_utils.h"

#include "posal_power_mgr.h"
#include "posal_intrinsics.h"

#include "wc_topo.h"
#include "wc_topo_capi.h"
#include "wear_cntr_utils.h"
#include "wear_cntr_events.h"

#include "ar_msg.h"
#include "shared_lib_api.h"
#include "spf_macros.h"

#include "gpr_packet.h"
#include "gpr_api_inline.h"
#include "ar_guids.h"
#include "spf_svc_calib.h"
#include "spf_ref_counter.h"
#include "spf_svc_utils.h"


#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */

#define PATH_DELAY_DEBUGGING

#define WCNTR_UNITY_Q4 0x10
#define WCNTR_ALIGN_8_BYTES(a)   ((a + 7) & (0xFFFFFFF8))
#define WCNTR_ALIGN_4_BYTES(a)   ((a + 3) & (0xFFFFFFFC))


#define WCNTR_GET_BASE_PTR(base_type, member_type, member_ptr)  (base_type *)(((int8_t *)member_ptr - offsetof(base_type, member_type)))


#define WCNTR_MSG_PREFIX "WCNTR  :%08lX: "
#define WCNTR_MSG(ID, xx_ss_mask, xx_fmt, ...) AR_MSG(xx_ss_mask, WCNTR_MSG_PREFIX xx_fmt, ID, ##__VA_ARGS__)
#ifdef SIM
#define WCNTR_SIM_DEBUG(dbg_log, msg, ...)                                                                          \
   do                                                                                                                  \
   {                                                                                                                   \
      AR_MSG(DBG_LOW_PRIO, WCNTR_MSG_PREFIX msg, dbg_log, ##__VA_ARGS__);                                           \
   } while (0)

#else
#define WCNTR_SIM_DEBUG(dbg_log, msg, ...)                                                                          \
   do                                                                                                                  \
   {                                                                                                                   \
   } while (0)
#endif

#define WCNTR_FRAME_SIZE_DONT_CARE     -1
#define WCNTR_FRAME_SIZE_1_MS           1
#define WCNTR_FRAME_SIZE_5_MS           5
#define WCNTR_FRAME_SIZE_10_MS         10

#define WCNTR_FRAME_LEN_1000_US        (1000)
#define WCNTR_FRAME_LEN_5000_US        (5000)
#define WCNTR_FRAME_LEN_20000_US       (20000)
#define WCNTR_FRAME_LEN_40000_US       (40000)

#define WCNTR_LATENCY_VOTE_MAX         (0xFFFFFFFF)
#define WCNTR_LATENCY_VOTE_RT_FACTOR   (3)  // 3% tolerance
#define WCNTR_LATENCY_VOTE_NRT_FACTOR  (70) // 70% tolerance

#define WCNTR_MAX_INP_DATA_Q_ELEMENTS              (128)
#define WCNTR_MAX_OUT_BUF_Q_ELEMENTS               (128)

#define WCNTR_LOG_ID_CNTR_TYPE_SHIFT              28
#define WCNTR_LOG_ID_SEQUENCE_SHIFT               12

/** 16 container container type */
#define WCNTR_LOG_ID_CNTR_TYPE_MASK                0xF0000000
/** sequence-id will wrap around after 65k */
#define WCNTR_LOG_ID_SEQUENCE_MASK                 0x0FFFF000

// maximum intent messages in intent queue.
static const uint32_t WCNTR_MAX_INTENT_Q_ELEMENTS = 16;
// number of intent buffers (default).
static const uint32_t WCNTR_DEFAULT_NUM_INTENT_BUFFERS = 1;

//Don't use first 8bits from MSB
#define WCNTR_CMD_BIT_MASK   0x00400000
#define WCNTR_TIMER_BIT_MASK 0x00800000

#define WCNTR_STATIC static

#ifndef ALIGN_8_BYTES
#define ALIGN_8_BYTES(a)    ((a + 7) & (0xFFFFFFF8))
#endif

#ifndef ALIGNED_SIZE_W_QUEUES
#define ALIGNED_SIZE_W_QUEUES(t, n) (ALIGN_8_BYTES(sizeof(t)) + posal_queue_get_size() * n)
#endif

/** 
 * Defines for inserting posal Q's after the container structures 
 * memory for the wcntr is organized as follows:

wcntr_t     posal_queue_t posal_queue_t 
            cmdq          ctrl port q  
to reduce malloc overheads we do one single allocation 
*/
#define WCNTR_SIZE_W_QS (ALIGNED_SIZE_W_QUEUES(wcntr_t, 2))
#define WNTR_INT_CTRL_PORT_SIZE_W_QS (ALIGNED_SIZE_W_QUEUES(wcntr_int_ctrl_port_t, 1))
#define WCNTR_CMDQ_OFFSET (ALIGN_8_BYTES(sizeof(wcntr_t)))
#define WCNTR_CTRL_PORT_Q_OFFSET (ALIGNED_SIZE_W_QUEUES(wcntr_t, 1))
#define WCNTR_INT_CTRL_PORT_Q_OFFSET (ALIGN_8_BYTES(sizeof(wcntr_int_ctrl_port_t)))

#define WCNTR_GET_CMDQ_OFFSET_ADDR(x)  ((void*) ((uint8_t*)x + WCNTR_CMDQ_OFFSET))
#define WCNTR_GET_CTRL_PORT_Q_OFFSET_ADDR(x)  ((void*) ((uint8_t*)x + WCNTR_CTRL_PORT_Q_OFFSET))
#define WCNTR_GET_INT_CTRL_PORT_Q_OFFSET_ADDR(x) ((void*) ((uint8_t*)x + WCNTR_INT_CTRL_PORT_Q_OFFSET))

typedef struct wcntr_base_t         wcntr_base_t;

typedef ar_result_t (*wcntr_queue_handler_t)(wcntr_base_t *me_ptr, uint32_t ch_bit_index);
typedef ar_result_t (*wcntr_msg_handler_func_t)(wcntr_base_t *me_ptr);
typedef ar_result_t (*wcntr_handle_rest_of_fn_t)(wcntr_base_t *me_ptr, void *ctx_ptr);

typedef struct wcntr_msg_handler_t
{
      uint32_t                opcode;     /**< Opcode of the message */
      wcntr_msg_handler_func_t   fn;         /**< function pointer to the handler */
} wcntr_msg_handler_t;

/** Container power manager information struct */
typedef struct wcntr_pm_info_t
{
   posal_pm_handle_t      pm_handle_ptr;           /**< Pointer to PM handle */
   posal_pm_register_t    register_info;           /**< PM register information */
   bool_t				  is_request_async;		   /**< Determines if pm request/releases are done synchronously */
   uint32_t               cntr_kpps;               /**< KPPS contribution of the container framework. */
   uint32_t               cntr_bw;                 /**< BW contribution of the container framework. Bytes per sec */
   uint32_t               prev_kpps_vote;          /**< total KPPS of the decoder. */
   uint64_t               prev_floor_clock_vote;   /**< previous floor clock vote in Hz*/
   uint32_t               prev_bw_vote;            /**< previous bw vote in bytes per sec inclu capi vote */
   uint32_t               prev_latency_vote;       /**< previous latency vote in us */
   uint32_t               weighted_kpps_scale_factor_q4;/**< weighted KPPS scale factor in q4, non-unity (0x10) indicates faster or slower processing. */
} wcntr_pm_info_t;



typedef struct wcntr_handle_rest_ctx_for_wcntr_set_cfg_t
{
   int8_t *           param_payload_ptr;           /**< param_payload_ptr at which the set-cfg processing is swapped with thread re-launch (points to mem after apm_module_param_data_t) */
   void *             module_ptr;                  /**< module pointer (gen_topo) */
   ar_result_t        overall_result;              /**< aggregated result */
} wcntr_handle_rest_ctx_for_wcntr_set_cfg_t;

typedef union wcntr_event_flags_t
{
   struct
   {
      uint32_t sg_state_change : 1;                /**< Indicates if any self or peer SG state has changed to started, suspended or stopped. */
      uint32_t cntr_run_state_change : 1;              /**< when first sg is started, last SG is stopped/suspended. */
      uint32_t kpps_bw_scale_factor_change : 1;    /**< KPPS, BW or KPPS scale factor have changed */      
   };
   uint32_t word;
} wcntr_event_flags_t;

typedef struct cu_flags_t
{   
   uint32_t        is_cntr_started : 1;              /**< cntr is started if at least one SG is in start state */
} cu_flags_t;

typedef struct wcu_module_t
{
   spf_list_node_t     *event_list_ptr;       /**< Module events list */
} wcu_module_t;
/**
 * @brief The base structure of a container
 *
 * Only the container owning this memory must use the elements of this struct
 * Other containers or APM must not use.
 */
typedef struct wcntr_base_t
{
   spf_handle_t           spf_handle;              /**< Handle to this container. Must be the first element. q_ptr points to response queue (no longer supported).*/
   spf_cmd_handle_t       cmd_handle;             /**< Memory for the cmd_handle_ptr referred to in spf_handle. */
   wcntr_gu_t *                 gu_ptr;
   void *                 topo_ptr;               /**< Opaque pointer to the topology, to be passed into public topo functions. */
   uint16_t               int_ctrl_port_cu_offset;/**< Offset of cu component from gu, used to derive cu from gu_ptr. */
   uint16_t               module_cu_offset;       /**< Offset of cu component from gu, used to derive cu from gu_ptr. */
   uint32_t               cntr_instance_id;       /**< Instance ID of the container */
   uint32_t               cntr_type;
   uint32_t               position;
   uint32_t               proc_domain;
   uint32_t               actual_stack_size;       /**< Actual size of the thread stack */
   uint32_t               configured_stack_size;
   uint32_t               configured_thread_prio;  /**< Thread priority configured by the client */
   posal_channel_t        channel_ptr;
   posal_signal_t		  gp_signal_ptr;			   /**< General purpose signal */
   POSAL_HEAP_ID          heap_id;
   uint32_t               curr_chan_mask;          /**< Current channel mask */
   uint32_t               available_bit_mask;      /**< Available bits in the bit mask of the channel */
   wcntr_queue_handler_t     qftable[32];
   spf_msg_t              cmd_msg;                 /**< Command message */
   const wcntr_msg_handler_t *cmd_handler_table_ptr;  /**< Pointer to a function table for command handling */
   uint16_t               cmd_handler_table_size;
   posal_thread_t         thread_id_to_exit;       /**< ID of thread need to be destroyed. a thread exists if this matches its ID */
   wcntr_handle_rest_of_fn_t handle_rest_fn;          /**< Thread might have re-launched after handling a command partially.
                                                        If so, this function ptr can be set to handle the rest of the functionality.
                                                        this is set only when thread is re-launched.*/
   void                   *handle_rest_ctx_ptr;    /**< Any data needed to complete the incomplete handling. The interpreter can decide how to 
                                                        handle this pointer. this is set only when thread is re-launched. */
   wcntr_pm_info_t			  pm_info;                 /**< power manager information */
   posal_channel_t        ctrl_channel_ptr;       /**< Current control channel */
   uint32_t               curr_ctrl_chan_mask;     /**< Current control channel mask */
   uint32_t               available_ctrl_chan_mask;/**< Available bits in the bit mask of the control commands channel */
   uint32_t               cntr_proc_duration;      /**< us, processing duration of the container per frame = scaled frame_size*/
   cu_flags_t             flags;                   /**< general flags */
   
   spf_list_node_t        *event_list_ptr;         /**< List of events registered with the container instance
                                                        object = tu_event_info_t */
   wcntr_event_flags_t       fwk_evt_flags;           /**< Event flags */
   spf_handle_t           cntr_cmn_imcl_handle;    /*< Queue which is common to all the internal control port messages within this container*/  														
} wcntr_base_t;

typedef struct wcu_int_ctrl_port_t
{
   /** Used for intra-container IMCL  */
   posal_queue_t         *buffer_q_ptr;       // Outgoing ctrl buffer queue.

   posal_channel_t       buf_q_channel_ptr; // outgoing ctrl buffer queue's channel.
   
   uint32_t              num_bufs;           // Number of buffers requested by the module.

   uint32_t              buf_size;           // Current buffer size of the allocated outgoing buffers.

   uint32_t              new_event_num_bufs; // New event num_bufs from the module event.

   uint32_t              new_event_buf_size; // New recurring buffer size from the module event.

   spf_msg_t             recurring_msg;      // Current outgoing recurring ctrl msg.

   spf_msg_t             one_time_msg;       // Current outgoing one time msg.  

   uint32_t              outgoing_intent_cnt;// outgoing intent counter.

} wcu_int_ctrl_port_t;


/**
 * Please note that in memory a posal queue will immediatly follow this structure, to reduce memory overhead from allocations. 
 * */
typedef struct wcntr_int_ctrl_port_t
{
   wcntr_topo_ctrl_port_t      topo;        /** Must be the first element */
   wcu_int_ctrl_port_t        cu;

} wcntr_int_ctrl_port_t;

typedef struct wcntr_module_t
{
   wcntr_topo_module_t             topo;
   wcu_module_t                   cu;
} wcntr_module_t;

/** instance struct of WCNTR */
typedef struct wcntr_t
{
   wcntr_base_t         cu;                        /**< Container utility. Must be first element. */
   wcntr_topo_t        topo;
   posal_signal_t    trigger_signal_ptr;        /**< non-NULL if signal triggered (after start)  */
   uint32_t op_frame_in_ms; /**< Operating frame size. Updated during threshold calculation*/
   uint32_t signal_trigger_count;
   uint32_t signal_miss_count;
} wcntr_t;



typedef struct wcntr_debug_info_t
{    
   uint32_t max_size; /*Total size allocated for the container*/
   uint32_t size_filled; /* Debug info filled by the container*/
   uint32_t op_frame_in_ms;
   uint32_t signal_trigger_count;
   uint32_t signal_miss_count;
   uint32_t actual_stack_size;      
   uint32_t configured_stack_size;
   uint32_t configured_thread_prio;
   uint32_t curr_chan_mask;
   uint32_t num_subgraphs; /* Number of Subgraphs present in container*/
   uint32_t num_sg_debug_info_filled;  /* Debug information of number of filled r*/
} wcntr_debug_info_t;


/**---------------------------------- cu -----------------------------------*/
ar_result_t wcntr_init(wcntr_base_t *me_ptr);

ar_result_t wcntr_deinit(wcntr_base_t *me_ptr);

ar_result_t wcntr_check_launch_thread(wcntr_base_t *            me_ptr,
                                   uint32_t               new_stack_size,
                                   posal_thread_prio_t    thread_priority,
                                   char *                 thread_name,
                                   bool_t *               thread_launched_ptr);

ar_result_t wcntr_process_cmd_queue(wcntr_base_t *me_ptr, uint32_t channel_bit_index);
ar_result_t wcntr_parse_and_get_heap_id(apm_container_cfg_t *container_cfg_ptr, POSAL_HEAP_ID *heap_id_ptr);
uint32_t wcntr_get_next_unique_id(wcntr_base_t *base_ptr);
ar_result_t wcntr_set_cntr_type_bits_in_log_id(uint32_t cntr_type, uint32_t *log_id_ptr);

/**--------------------------- General utilities ---------------------------*/
/**
 * returns conventional bit index (bit 0 for right most bit)
 */
static inline uint32_t wcntr_get_bit_index_from_mask(uint32_t mask)
{
   return (31 - s32_cl0_s32(mask)); // count leading zeros starting from MSB
   // (subtracting from 31 gives index of the 1 from right, (conventional bit index))
}

static inline void wcntr_set_handler_for_bit_mask(wcntr_base_t *me_ptr, uint32_t bit_mask, wcntr_queue_handler_t fn)
{
   uint32_t n         = wcntr_get_bit_index_from_mask(bit_mask);
   me_ptr->qftable[n] = fn;
}

static inline void wcntr_stop_listen_to_mask(wcntr_base_t *me_ptr, uint32_t mask)
{
   me_ptr->curr_chan_mask &= ~mask;
}

static inline void wcntr_start_listen_to_mask(wcntr_base_t *me_ptr, uint32_t mask)
{
   me_ptr->curr_chan_mask |= mask;
}

static inline void wcntr_set_bit_in_bit_mask(wcntr_base_t *me_ptr, uint32_t mask)
{
   me_ptr->available_bit_mask &= ~mask;
}

static inline void wcntr_release_bit_in_bit_mask(wcntr_base_t *me_ptr, uint32_t mask)
{
   // add it back to available mask
   me_ptr->available_bit_mask |= mask;
   wcntr_set_handler_for_bit_mask(me_ptr, mask, NULL);
}

static inline uint32_t wcntr_get_bits(uint32_t x, uint32_t mask, uint32_t shift)
{
   return (x & mask) >> shift;
}

static inline void wcntr_set_bits(uint32_t *x_ptr, uint32_t val, uint32_t mask, uint32_t shift)
{
   val    = (val << shift) & mask;
   *x_ptr = (*x_ptr & ~mask) | val;
}

static inline uint32_t wcntr_request_bit_in_bit_mask(uint32_t *available_bit_mask)
{
   uint32_t n    = wcntr_get_bit_index_from_mask(*available_bit_mask);
   uint32_t mask = 1 << n;
   *available_bit_mask &= ~mask;

   return mask;
}

static inline void wcntr_set_bits_in_x(uint32_t *x_ptr, uint32_t flags)
{
   *x_ptr |= flags;
}

static inline void wcntr_clear_bits_in_x(uint32_t *x_ptr, uint32_t flags)
{
   *x_ptr &= ~flags;
}

static inline bool_t wcntr_is_any_handle_rest_pending(wcntr_base_t *base_ptr)
{
   return (NULL != base_ptr->handle_rest_fn);
}

ar_result_t wcntr_workloop_entry(void *instance_ptr);
ar_result_t wcntr_workloop(wcntr_base_t *me_ptr);

static inline void wcntr_reset_handle_rest(wcntr_base_t *base_ptr)
{
   MFREE_NULLIFY(base_ptr->handle_rest_ctx_ptr);
   base_ptr->handle_rest_fn = NULL;
}

ar_result_t wcntr_init_queue(wcntr_base_t *		 base_ptr,
                                  char *             data_q_name,
                                  uint32_t           num_elements,
                                  uint32_t           bit_mask,
                                  wcntr_queue_handler_t q_func_ptr,
                                  posal_channel_t    channel_ptr,
                                  posal_queue_t **   q_pptr,
                                  void *             dest,
                                  POSAL_HEAP_ID      heap_id);

void wcntr_dump_debug_info(spf_handle_t *handle, int8_t *start_address,uint32_t max_size);



#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef WCNTR_I_H
