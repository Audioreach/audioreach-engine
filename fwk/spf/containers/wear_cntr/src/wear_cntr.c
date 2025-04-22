/**
 * \file wear_cntr.c
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "wear_cntr.h"
#include "wear_cntr_i.h"
#include "spf_svc_utils.h"
#include "posal_queue.h"

// maximum number of commands expected ever in command queue.
static const uint32_t MAX_CMD_Q_ELEMENTS = 128;
#define CU_MIN_QUEUE_ELEMENTS_TO_PRE_ALLOC 8
#define SIZE_OF_ARRAY(a) (sizeof(a) / sizeof((a)[0]))

// Global counter to return a unique id.
static uint32_t wr_global_unique_id = 1; // 0 reserved for not initialized.

/**
 * Create function of a container
 */
typedef ar_result_t (*spf_cntr_create_function_t)(cntr_cmn_init_params_t *init_param_ptr, spf_handle_t **cntr_handle);

typedef void (*spf_cntr_dump_debug_info_t)(spf_handle_t *cntr_handle, int8_t *start_address, uint32_t max_size);

const spf_cntr_dump_debug_info_t global_dump_debug_info_ftable[] = {
   NULL, // APM_CONTAINER_TYPE_ID_SC
   NULL, // APM_CONTAINER_TYPE_ID_GC
   NULL, // APM_CONTAINER_TYPE_ID_OLC
   wcntr_dump_debug_info,
};

/**
 * the index is used for getting entry point function
 * and for getting the index for logging.
 */
const uint32_t global_cntr_list_t[] = {
   APM_CONTAINER_TYPE_ID_SC, // 0
   APM_CONTAINER_TYPE_ID_GC, // 1
   APM_CONTAINER_TYPE_ID_OLC, // 2
   APM_CONTAINER_TYPE_ID_WC
};

const spf_cntr_create_function_t global_cntr_create_ftable[] = {
   NULL, // APM_CONTAINER_TYPE_ID_SC
   NULL, // APM_CONTAINER_TYPE_ID_GC
   NULL, // APM_CONTAINER_TYPE_ID_OLC
   wear_cntr_create,
};

// most frequent commands be on top
const wcntr_msg_handler_t wcntr_cmd_handler_table[] = {
   { SPF_MSG_CMD_GPR, wcntr_gpr_cmd },
   { SPF_MSG_CMD_SET_CFG, wcntr_set_get_cfg },
   { SPF_MSG_CMD_GET_CFG, wcntr_set_get_cfg },
   { SPF_MSG_CMD_INFORM_ICB_INFO, wcntr_cmd_icb_info_from_downstream },
   { SPF_MSG_CMD_CTRL_PORT_TRIGGER_MSG, wcntr_handle_ctrl_port_trigger_cmd },
   { SPF_MSG_CMD_PEER_PORT_PROPERTY_UPDATE, wcntr_handle_peer_port_property_update_cmd },
   { SPF_MSG_CMD_UPSTREAM_STOPPED_ACK, wcntr_handle_upstream_stop_cmd },
   { SPF_MSG_CMD_GRAPH_OPEN, wcntr_graph_open },
   { SPF_MSG_CMD_GRAPH_PREPARE, wcntr_graph_prepare },
   { SPF_MSG_CMD_GRAPH_START, wcntr_graph_start },
   { SPF_MSG_CMD_GRAPH_SUSPEND, wcntr_graph_suspend },
   { SPF_MSG_CMD_GRAPH_STOP, wcntr_graph_stop },
   { SPF_MSG_CMD_GRAPH_FLUSH, wcntr_graph_flush },
   { SPF_MSG_CMD_GRAPH_CLOSE, wcntr_graph_close },
   { SPF_MSG_CMD_GRAPH_CONNECT, wcntr_graph_connect },
   { SPF_MSG_CMD_GRAPH_DISCONNECT, wcntr_graph_disconnect },
   { SPF_MSG_CMD_DESTROY_CONTAINER, wcntr_destroy_container },
   { SPF_MSG_CMD_MEDIA_FORMAT, wcntr_ctrl_path_media_fmt_cmd },
   { SPF_MSG_CMD_REGISTER_CFG, wcntr_set_get_cfg },
   { SPF_MSG_CMD_DEREGISTER_CFG, wcntr_set_get_cfg },
};

/* CU call back functions for container specific handling */

const wcntr_topo_to_cntr_vtable_t topo_to_wcntr_vtable = {

   .raise_data_to_dsp_service_event             = wcntr_handle_event_to_dsp_service_topo_cb,
   .raise_data_from_dsp_service_event           = wcntr_handle_event_from_dsp_service_topo_cb,
   .handle_capi_event                           = wcntr_handle_capi_event,
   .destroy_module                              = wcntr_destroy_module,
};

const uint32_t g_sizeof_wcntr_cmd_handler_table = (SIZE_OF_ARRAY(wcntr_cmd_handler_table));

typedef ar_result_t (*posal_cmd_q_handler_f)(wcntr_t *me_ptr);
extern const wcntr_msg_handler_t      wcntr_cmd_handler_table[];
extern const uint32_t              g_sizeof_wcntr_cmd_handler_table;
extern const wcntr_topo_to_cntr_vtable_t topo_to_wcntr_vtable;

ar_result_t wcntr_unsupported_cmd(wcntr_base_t *me_ptr);


static ar_result_t wear_cntr_create_channel_and_queues(wcntr_t *me_ptr)
{
   ar_result_t result = AR_EOK;

   char_t cmdQ_name[POSAL_DEFAULT_NAME_LEN]; // command queue name

   snprintf(cmdQ_name, POSAL_DEFAULT_NAME_LEN, "%s%8lX", "CWC", me_ptr->topo.gu.log_id);

   if (AR_EOK != (result = posal_channel_create(&me_ptr->cu.channel_ptr, me_ptr->cu.heap_id)))
   {
      AR_MSG(DBG_ERROR_PRIO, "WCNTR: Unable to create channel, result: %lu", result);
      return result;
   }

   wcntr_init_queue(&me_ptr->cu,
                   cmdQ_name,
                   MAX_CMD_Q_ELEMENTS,
                   WCNTR_CMD_BIT_MASK,
                   wcntr_process_cmd_queue,
                   me_ptr->cu.channel_ptr,
                   &me_ptr->cu.cmd_handle.cmd_q_ptr,
                   WCNTR_GET_CMDQ_OFFSET_ADDR(me_ptr),
                   me_ptr->cu.heap_id);

   // Assign command handle pointer to point to the correct memory.
   me_ptr->cu.spf_handle.cmd_handle_ptr = &me_ptr->cu.cmd_handle;

   /* Set the available bit mask for gen_cntr
    *  Important Note: Carefully set the mask availability based on the command handling priority.
    *  When the multiple signals are set on the channel, commands are handled based on the set bit position.
    *  The higher the bit position higher the priority. Need to decide if the control takes priority over data or
    *  vice versa and set the WCNTR_CMD_BIT_MASK. For example, in End point case data takes higher priority than .
    *  control.
    */
   me_ptr->cu.available_bit_mask = 0x00FFFFFF & (~(WCNTR_CMD_BIT_MASK | WCNTR_TIMER_BIT_MASK));

   AR_MSG(DBG_HIGH_PRIO, "WCNTR: available_bit_mask  0x%lX", me_ptr->cu.available_bit_mask);

   /* Intialize control channel mask*/
   if (AR_EOK != (result = posal_channel_create(&me_ptr->cu.ctrl_channel_ptr, me_ptr->cu.heap_id)))
   {
      AR_MSG(DBG_ERROR_PRIO, "WCNTR: Unable to create ctrl channel, result: %lu", result);
      return result;
   }

   /* Intialize the available ctrl chan mask- MSB 8 bits dont use due to free RTOS restrictions*/
   me_ptr->cu.available_ctrl_chan_mask = 0x00FFFFFF;

    if (AR_EOK !=(result= wcntr_init_int_ctrl_port_queue(&me_ptr->cu, WCNTR_GET_CTRL_PORT_Q_OFFSET_ADDR(me_ptr))))
    {
	AR_MSG(DBG_ERROR_PRIO, "WCNTR: Unable to init ctrl port queue, result: %lu", result);
	}

   return result;
}

/**
 * Prerequisite tasks for thread launching. Must call this function before
 * launching/relaunching thread.
 *
 * stack_size [out]: returns the new thread's stack size.
 * priority [out]: returns the new thread's priority.
 */
ar_result_t wcntr_prepare_to_launch_thread(wcntr_t *         me_ptr,
                                              uint32_t *           stack_size,
                                              posal_thread_prio_t *priority_ptr,
                                              char *               thread_name,
                                              uint32_t             name_length)
{
   snprintf(thread_name, name_length, "WC_%lX", me_ptr->cu.cntr_instance_id);

   wcntr_get_thread_stack_size(me_ptr, stack_size);
   wcntr_get_set_thread_priority(me_ptr, priority_ptr, FALSE /*should_set*/);

   return AR_EOK;
}

ar_result_t wear_cntr_create(cntr_cmn_init_params_t *init_params_ptr, spf_handle_t **handle)
{

   AR_MSG(DBG_HIGH_PRIO, "wear_cntr_create START ");
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   wcntr_t *me_ptr                        = NULL;
   *handle                                = NULL;
   uint32_t               stack_size      = 0;
   posal_thread_prio_t    thread_priority = 0;
   char_t                 thread_name[POSAL_DEFAULT_NAME_LEN];
   bool_t                 thread_launched = FALSE;
   wcntr_topo_init_data_t topo_init;
   POSAL_HEAP_ID          heap_id = POSAL_HEAP_DEFAULT;

   AR_MSG(DBG_HIGH_PRIO, "WCNTR: Not parsing init cfg for heap_id. Assuming default POSAL_HEAP_DEFAULT");
   me_ptr = (wcntr_t *)posal_memory_malloc(WCNTR_SIZE_W_QS, heap_id);
   if (NULL == me_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "WCNTR: Malloc failed from heap ID %u. Required Size %lu",
             (uint32_t)heap_id,
             sizeof(wcntr_t));
      return AR_ENOMEMORY;
   }
   memset(me_ptr, 0, sizeof(wcntr_t));

   /*Initialize gu_ptr, such that if wcntr_destroy is called due to some failure, we don't crash for this being null
    */
   me_ptr->cu.gu_ptr = &me_ptr->topo.gu;

   TRY(result, wcntr_parse_container_cfg(me_ptr, init_params_ptr->container_cfg_ptr));

   TRY(result, wcntr_set_cntr_type_bits_in_log_id(me_ptr->cu.cntr_type, &me_ptr->topo.gu.log_id));

   wcntr_set_bits(&me_ptr->topo.gu.log_id,
               (uint32_t)init_params_ptr->log_seq_id,
               WCNTR_LOG_ID_SEQUENCE_MASK,
               WCNTR_LOG_ID_SEQUENCE_SHIFT);

   /*Assign heap ID */
   me_ptr->cu.heap_id                               = heap_id;
   me_ptr->cu.cmd_handler_table_ptr                 = wcntr_cmd_handler_table;
   me_ptr->cu.cmd_handler_table_size                = g_sizeof_wcntr_cmd_handler_table;
   me_ptr->cu.pm_info.weighted_kpps_scale_factor_q4 = WCNTR_UNITY_Q4;
   me_ptr->cu.configured_thread_prio                = 0; // Assume configured priority, to be updated by tools

   /* Init the topo and setup cu pointers to topo and gu fields. */
   memset(&topo_init, 0, sizeof(topo_init));
   topo_init.wcntr_topo_to_cntr_vtble_ptr = &topo_to_wcntr_vtable;
   TRY(result, wcntr_topo_init_topo(&me_ptr->topo, &topo_init, me_ptr->cu.heap_id));
   me_ptr->cu.topo_ptr      = (void *)&me_ptr->topo;
   me_ptr->cu.int_ctrl_port_cu_offset = offsetof(wcntr_int_ctrl_port_t, cu);
   me_ptr->cu.module_cu_offset        = offsetof(wcntr_module_t, cu);


   // Only command queue and IMCL queue is created.
   TRY(result, wear_cntr_create_channel_and_queues(me_ptr));

   // Intialize the wait mask only with the CMD handler mask
   me_ptr->cu.curr_chan_mask = WCNTR_CMD_BIT_MASK;

   TRY(result, wcntr_init(&me_ptr->cu));

   TRY(result,
       wcntr_prepare_to_launch_thread(me_ptr, &stack_size, &thread_priority, thread_name, POSAL_DEFAULT_NAME_LEN));

   TRY(result, wcntr_check_launch_thread(&me_ptr->cu, stack_size, thread_priority, thread_name, &thread_launched));

   *handle = (spf_handle_t *)&me_ptr->cu.spf_handle;

   WCNTR_MSG(me_ptr->topo.gu.log_id,
                DBG_HIGH_PRIO,
             SPF_LOG_PREFIX "Created WCNTR with container ID 0x%08lX, from heap_id %u",
                me_ptr->cu.cntr_instance_id,
                (uint32_t)heap_id);

   /** Register with GPR */
   if (AR_EOK != (result = __gpr_cmd_register(me_ptr->cu.cntr_instance_id, wcntr_gpr_callback, &me_ptr->cu.spf_handle)))
   {
      WCNTR_MSG(me_ptr->topo.gu.log_id,
                   DBG_ERROR_PRIO,
                   "WCNTR Create: Failed to register with GPR, result: 0x%8x",
                   result);
   }

   CATCH(result, WCNTR_MSG_PREFIX, me_ptr->topo.gu.log_id)
   {
      WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_ERROR_PRIO, "Create failed");
      wcntr_destroy(me_ptr);
   }


     WCNTR_MSG(me_ptr->topo.gu.log_id,
                   DBG_HIGH_PRIO,
                   "wear_cntr_create is successful with result 0x%lX ",result);

   return result;
}

ar_result_t wcntr_destroy(wcntr_t *me_ptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   uint32_t log_id = 0;

   VERIFY(result, NULL != me_ptr);

   log_id = me_ptr->topo.gu.log_id;

   /** De-register with  GPR    */
   if (AR_EOK != (result = __gpr_cmd_deregister(me_ptr->cu.cntr_instance_id)))
   {
      WCNTR_MSG(log_id, DBG_ERROR_PRIO, "WCNTR: Failed to de-register with GPR, result: %lu", result);
   }

   wcntr_delete_all_event_nodes(&me_ptr->cu.event_list_ptr);

   (void)wcntr_deregister_with_pm(&me_ptr->cu);

   TRY(result, wcntr_deinit(&me_ptr->cu));

   wcntr_deinit_internal_ctrl_ports(&me_ptr->cu, NULL);

   wcntr_topo_destroy_modules(&me_ptr->topo, NULL);

  wcntr_topo_destroy_topo(&me_ptr->topo);

   wcntr_gu_destroy_graph(me_ptr->cu.gu_ptr, NULL, me_ptr->cu.heap_id);

   /*Deinit the intra container IMCL Queue */
   wcntr_deinit_int_ctrl_port_queue(&me_ptr->cu);

   if (me_ptr->cu.spf_handle.q_ptr)
   {
      posal_queue_deinit(me_ptr->cu.spf_handle.q_ptr);
      me_ptr->cu.spf_handle.q_ptr = NULL;
   }

   if (me_ptr->cu.cmd_handle.cmd_q_ptr)
   {
      spf_svc_deinit_cmd_queue(me_ptr->cu.cmd_handle.cmd_q_ptr);
      me_ptr->cu.cmd_handle.cmd_q_ptr = NULL;
      // shouldn't be made NULL as cntr_cmn_destroy uses this to get handle->thread_id
      // me_ptr->cu.spf_handle.cmd_handle_ptr = NULL;

      // if cmdQ is created then ch is also init'ed.
      posal_channel_destroy(&me_ptr->cu.channel_ptr);

      /** Destroy the control port channel */
      posal_channel_destroy(&me_ptr->cu.ctrl_channel_ptr);
   }

   // If the thread is not launched, free up the me ptr as
   // APM cannot send destroy messsage without a thread context
   if (!me_ptr->cu.cmd_handle.thread_id)
   {
      MFREE_NULLIFY(me_ptr);
   }

   WCNTR_MSG(log_id, DBG_HIGH_PRIO, "WC destroyed");

   CATCH(result, WCNTR_MSG_PREFIX, log_id)
   {
      WCNTR_MSG(log_id, DBG_HIGH_PRIO, "WC destroy failed");
   }

   return AR_ETERMINATED;
}

ar_result_t wcntr_init_queue(wcntr_base_t *        base_ptr,
                            char *             data_q_name,
                            uint32_t           num_elements,
                            uint32_t           bit_mask,
                            wcntr_queue_handler_t q_func_ptr,
                            posal_channel_t    channel_ptr,
                            posal_queue_t **   q_pptr,
                            void *             dest,
                            POSAL_HEAP_ID      heap_id)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result = AR_EOK;
   uint32_t    min_elements =
      (num_elements <= CU_MIN_QUEUE_ELEMENTS_TO_PRE_ALLOC) ? num_elements : CU_MIN_QUEUE_ELEMENTS_TO_PRE_ALLOC;

   VERIFY(result, q_pptr);


   if (*q_pptr)
   {
      WCNTR_MSG(base_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Queue already present");
      return AR_EFAILED;
   }

   posal_queue_init_attr_t q_attr;
   posal_queue_attr_init(&q_attr);
   posal_queue_attr_set_heap_id(&q_attr, heap_id);
   posal_queue_attr_set_max_nodes(&q_attr, num_elements);
   posal_queue_attr_set_prealloc_nodes(&q_attr, min_elements);
   posal_queue_attr_set_name(&q_attr, data_q_name);
   if (AR_DID_FAIL(result = posal_queue_init(dest, &q_attr)))
   {
      WCNTR_MSG(base_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Failed to create message queues, result: %lu", result);
      return result;
   }
   *q_pptr = dest;

   if (q_func_ptr)
   {
      wcntr_set_handler_for_bit_mask(base_ptr, bit_mask, q_func_ptr);
   }

   /* Add queue to the channel only if valid bitmask and channel is passed in the arguments */
   if ((bit_mask > 0) && (NULL != channel_ptr))
   {
      if (AR_DID_FAIL(result = posal_channel_addq(channel_ptr, *q_pptr, bit_mask)))
      {
         WCNTR_MSG(base_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Failed to add Q to channel, result = %d", result);
         return result;
      }
   }
   else
   {
      WCNTR_MSG(base_ptr->gu_ptr->log_id,
             DBG_ERROR_PRIO,
             "Queue not added to the channel, bit_mask: 0x%lX, ch_ptr: 0x%lX ",
             bit_mask,
             channel_ptr);
	  return AR_EFAILED;
   }

   CATCH(result, WCNTR_MSG_PREFIX, base_ptr->gu_ptr->log_id)
   {
   }
   return result;
}

static ar_result_t wcntr_get_index_for_cntr_type(uint32_t cntr_type_id, uint32_t *cntr_index_ptr)
{
   *cntr_index_ptr = (uint32_t)-1;
   for (uint32_t i = 0; i < SIZE_OF_ARRAY(global_cntr_list_t); i++)
   {
      if (global_cntr_list_t[i] == cntr_type_id)
      {
         *cntr_index_ptr = i;
         return AR_EOK;
      }
   }
   return AR_EFAILED;
}
static ar_result_t wcntr_cmn_get_container_type_id(apm_container_cfg_t *container_cfg_ptr, uint32_t *type_id_ptr)
{
   ar_result_t result = AR_EOK;
   *type_id_ptr       = 0;

   apm_prop_data_t *cntr_prop_ptr = (apm_prop_data_t *)(container_cfg_ptr + 1);

   for (uint32_t i = 0; i < container_cfg_ptr->num_prop; i++)
   {
      switch (cntr_prop_ptr->prop_id)
      {
         case APM_CONTAINER_PROP_ID_CONTAINER_TYPE:
         {
            if (cntr_prop_ptr->prop_size < sizeof(apm_cont_prop_id_type_t))
            {
               return AR_ENORESOURCE;
            }

            apm_cont_prop_id_type_t *type_ptr = (apm_cont_prop_id_type_t *)(cntr_prop_ptr + 1);
            if (1 != type_ptr->version)
            {
               AR_MSG(DBG_ERROR_PRIO, "version must be one");
               return AR_EBADPARAM;
            }
            if (type_ptr->type_id.type != APM_CONTAINER_TYPE_ID_WC)
            {
               AR_MSG(DBG_ERROR_PRIO, "container type 0x%lX not supported ", type_ptr->type_id.type);
               return AR_EBADPARAM;
            }
            *type_id_ptr = type_ptr->type_id.type;
            AR_MSG(DBG_HIGH_PRIO, "Received type_id 0x%lX", *type_id_ptr);
         }
         break;
      }

      cntr_prop_ptr =
         (apm_prop_data_t *)((uint8_t *)cntr_prop_ptr + cntr_prop_ptr->prop_size + sizeof(apm_prop_data_t));
   }

   if (0 == *type_id_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Container Type IDs not given");
      return AR_EFAILED;
   }
   return result;
}

/* =======================================================================
API Function Definitions
========================================================================== */

ar_result_t wcntr_init(wcntr_base_t *me_ptr)
{
   ar_result_t result = AR_EOK;

   /* Create general purpose signal */
   if (AR_EOK != (result = posal_signal_create(&me_ptr->gp_signal_ptr, me_ptr->heap_id)))
   {
      WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_HIGH_PRIO, "Failed to create gp signal with result: ", result);
      return result;
   }

   uint32_t bit_mask = wcntr_request_bit_in_bit_mask(&me_ptr->available_bit_mask);
   wcntr_set_bit_in_bit_mask(me_ptr, bit_mask);

   /* Add general purpose signal to channel */
   posal_channel_add_signal(me_ptr->channel_ptr, me_ptr->gp_signal_ptr, bit_mask);

   return result;
}

ar_result_t wcntr_deinit(wcntr_base_t *me_ptr)
{

   /* Release signal bit in mask */
   wcntr_release_bit_in_bit_mask(me_ptr, posal_signal_get_channel_bit(me_ptr->gp_signal_ptr));

   /* Destroy general purpose signal */
   posal_signal_destroy(&me_ptr->gp_signal_ptr);

   return AR_EOK;
}

ar_result_t cntr_cmn_create(cntr_cmn_init_params_t *init_param_ptr, spf_handle_t **cntr_handle)
{
   ar_result_t result  = AR_EOK;
   uint32_t    type_id = 0;

   if (AR_DID_FAIL(result = wcntr_cmn_get_container_type_id(init_param_ptr->container_cfg_ptr, &type_id)))
   {
      return result;
   }
   uint32_t cntr_index = 0;

   if (AR_DID_FAIL(result = wcntr_get_index_for_cntr_type(type_id, &cntr_index)))
   {
      return result;
   }

#ifdef HEAP_PROFILING
   WCNTR_MSG(0, DBG_HIGH_PRIO, "APM allocation for containers started, seq_id 0x%lx", init_param_ptr->log_seq_id);
#endif

   result = global_cntr_create_ftable[cntr_index](init_param_ptr, cntr_handle);


#ifdef HEAP_PROFILING
   if (*cntr_handle)
   {
      WCNTR_MSG(0,
             DBG_HIGH_PRIO,
             "APM allocation for containers done, seq_id 0x%lx, cmd_handle 0x%lx",
             init_param_ptr->log_seq_id,
             (*cntr_handle)->cmd_handle_ptr);
   }
#endif
   return result;
}

ar_result_t cntr_cmn_destroy(spf_handle_t *cntr_handle)
{
   ar_result_t result = AR_EOK;

#ifdef HEAP_PROFILING
   WCNTR_MSG(0, DBG_HIGH_PRIO, "APM deallocation for containers started, cmd_handle 0x%lx", cntr_handle->cmd_handle_ptr);
#endif

   posal_thread_join(cntr_handle->cmd_handle_ptr->thread_id, &result);
   posal_memory_free(cntr_handle);

#ifdef HEAP_PROFILING
   WCNTR_MSG(0, DBG_HIGH_PRIO, "APM deallocation for containers done, cmd_handle 0x%lx", cntr_handle->cmd_handle_ptr);
#endif

   return result;
}

/* =======================================================================
Internal Function Definitions
========================================================================== */

ar_result_t wcntr_check_launch_thread(wcntr_base_t *me_ptr,
                                   uint32_t   new_stack_size,
                                   int32_t    thread_priority,
                                   char *     thread_name,
                                   bool_t *   thread_launched_ptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING

   posal_thread_t old_thread_id = me_ptr->cmd_handle.thread_id;

   // Can reuse thread if stack size did not change.
   if (me_ptr->actual_stack_size != new_stack_size)
   {
      WCNTR_MSG(me_ptr->gu_ptr->log_id,
             DBG_HIGH_PRIO,
             "Creating new thread. old stack size %lu, new stack size %lu, priority %lu",
             me_ptr->actual_stack_size,
             new_stack_size,
             thread_priority);

      // If there was a previous thread, it must exit. The newly created thread
      // won't exit looking at me_ptr->thread_id_to_exit as the id will be
      // different.
      me_ptr->thread_id_to_exit = old_thread_id;

      if (AR_DID_FAIL(result = posal_thread_launch(&(me_ptr->cmd_handle.thread_id),
                                                   thread_name,
                                                   new_stack_size,
                                                   thread_priority,
                                                   wcntr_workloop_entry,
                                                   (void *)me_ptr,
                                                   me_ptr->heap_id)))
      {
         // So that the thread doesn't exit, under failure.
         me_ptr->thread_id_to_exit = 0;

         WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Failed to launch thread!");

         THROW(result, result);
      }

      me_ptr->actual_stack_size = new_stack_size;
      *thread_launched_ptr      = TRUE;

      WCNTR_MSG(me_ptr->gu_ptr->log_id,
             DBG_HIGH_PRIO,
             "old thread id = 0x%lX, new thread id = 0x%lX",
             posal_thread_get_tid(old_thread_id),
             posal_thread_get_tid(me_ptr->cmd_handle.thread_id));
   }

   CATCH(result, WCNTR_MSG_PREFIX, me_ptr->gu_ptr->log_id)
   {
      WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "cntr_lauch_thread failed result = %d", result);
   }

   return result;
}

// Process the command queue: call the proper handler function depending on
// the message op code.
ar_result_t wcntr_process_cmd_queue(wcntr_base_t *me_ptr, uint32_t channel_bit_index)
{
   ar_result_t result = AR_EOK;

   // Take next msg from the queue.
   result = posal_queue_pop_front(me_ptr->cmd_handle.cmd_q_ptr, (posal_queue_element_t *)&(me_ptr->cmd_msg));

   // Process the message.
   if (AR_EOK == result)
   {
      bool_t handler_found = FALSE;
      for (uint32_t i = 0; i < me_ptr->cmd_handler_table_size; i++)
      {
         if (me_ptr->cmd_msg.msg_opcode == me_ptr->cmd_handler_table_ptr[i].opcode)
         {
#ifdef VERBOSE_LOGGING
            WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_HIGH_PRIO, "wcntr_process_cmd_queue : cmd_handler_table_ptr index %u ", i);
#endif
            result        = me_ptr->cmd_handler_table_ptr[i].fn(me_ptr);
            handler_found = TRUE;
            break;
         }
      }

      if (!handler_found)
      {
         return wcntr_unsupported_cmd(me_ptr);
      }
   }

   return result;
}

/*If client doesn't send the heap ID property, we assume Default heap.
We have to validate only if the property is received*/
ar_result_t wcntr_parse_and_get_heap_id(apm_container_cfg_t *container_cfg_ptr, POSAL_HEAP_ID *heap_id_ptr)
{
   apm_prop_data_t *cntr_prop_ptr;
   *heap_id_ptr = POSAL_HEAP_DEFAULT;

   if (!container_cfg_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Container cfg not given during create. Error");
      return AR_EFAILED;
   }

   cntr_prop_ptr = (apm_prop_data_t *)(container_cfg_ptr + 1);

   for (uint32_t i = 0; i < container_cfg_ptr->num_prop; i++)
   {
      if (APM_CONTAINER_PROP_ID_HEAP_ID == cntr_prop_ptr->prop_id)
      {
         if (cntr_prop_ptr->prop_size < sizeof(apm_cont_prop_id_heap_id_t))
         {
            return AR_EBADPARAM;
         }
         apm_cont_prop_id_heap_id_t *heap_cfg_ptr = (apm_cont_prop_id_heap_id_t *)(cntr_prop_ptr + 1);

         *heap_id_ptr = gu_get_heap_id_from_heap_prop(heap_cfg_ptr->heap_id);
         break;
      }
      cntr_prop_ptr =
         (apm_prop_data_t *)((uint8_t *)cntr_prop_ptr + cntr_prop_ptr->prop_size + sizeof(apm_prop_data_t));
   }
   return AR_EOK;
}

uint32_t wcntr_get_next_unique_id(wcntr_base_t *base_ptr)
{
   uint32_t unique_id = wr_global_unique_id;
   wr_global_unique_id++;
   return unique_id;
}

ar_result_t wcntr_set_cntr_type_bits_in_log_id(uint32_t cntr_type, uint32_t *log_id_ptr)
{
   ar_result_t result     = AR_EOK;
   uint32_t    cntr_index = 0;

   if (AR_DID_FAIL(result = wcntr_get_index_for_cntr_type(cntr_type, &cntr_index)))
   {
      return result;
   }
   // cntr_index +1 for backward compatibility
   wcntr_set_bits(log_id_ptr, cntr_index + 1, WCNTR_LOG_ID_CNTR_TYPE_MASK, WCNTR_LOG_ID_CNTR_TYPE_SHIFT);
   return result;
}

ar_result_t wcntr_unsupported_cmd(wcntr_base_t *me_ptr)
{
   WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_HIGH_PRIO, "Unsupported command with opcode 0x%lx!!", me_ptr->cmd_msg.msg_opcode);
   spf_msg_ack_msg(&me_ptr->cmd_msg, AR_EUNSUPPORTED);
   return AR_EUNSUPPORTED;
}

// Pre-workloop tasks.
ar_result_t wcntr_workloop_entry(void *instance_ptr)
{
   ar_result_t result = AR_EOK;
   wcntr_base_t * me_ptr = (wcntr_base_t *)instance_ptr;

   // If there was a previous thread, join it. me_ptr->cntr_handle.thread_id
   // is assigned only in workloop.
   if (me_ptr->thread_id_to_exit)
   {
      WCNTR_MSG(me_ptr->gu_ptr->log_id,
             DBG_HIGH_PRIO,
             "Joining thread ID 0x%lX",
             posal_thread_get_tid(me_ptr->thread_id_to_exit));

      posal_thread_join(me_ptr->thread_id_to_exit, &result);
      me_ptr->thread_id_to_exit = 0;
   }

   // If any command handling was done partially, complete the rest now.
   if (wcntr_is_any_handle_rest_pending(me_ptr))
   {
      me_ptr->handle_rest_fn(me_ptr, me_ptr->handle_rest_ctx_ptr);
   }

   // if handle rest is again set, don't enter workloop, just exit
   if (!wcntr_is_any_handle_rest_pending(me_ptr))
   {
      // Call the workloop.
      result = wcntr_workloop(me_ptr);
   }

   return result;
}

ar_result_t wcntr_workloop(wcntr_base_t *me_ptr)
{
   ar_result_t result = AR_EFAILED;
   uint32_t    channel_status;

   // Start off waiting on only the command queue. Must be done before launch_thread.
   // me_ptr->curr_chan_mask = CMD_BIT_MASK;

   // Loop until termination.
   for (;;)
   {

#ifdef VERBOSE_LOGGING
      WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_HIGH_PRIO, "Entering wait with mask 0x%lx ", me_ptr->curr_chan_mask);
#endif
      // Block on any selected queues to get a msg.
      (void)posal_channel_wait(me_ptr->channel_ptr, me_ptr->curr_chan_mask);

      for (;;)
      {
         // Check for signals.
         channel_status = posal_channel_poll(me_ptr->channel_ptr, me_ptr->curr_chan_mask);

         if (channel_status == 0)
         {
            break;
         }

         int32_t bit_index = wcntr_get_bit_index_from_mask(channel_status);
#ifdef VERBOSE_LOGGING
         WCNTR_MSG(me_ptr->gu_ptr->log_id,
                DBG_HIGH_PRIO,
                "Out of wait. channel_status 0x%lx, bit_index %d ",
                channel_status,
                bit_index);
#endif
         if (NULL == me_ptr->qftable[bit_index])
         {
            WCNTR_MSG(me_ptr->gu_ptr->log_id,
                   DBG_ERROR_PRIO,
                   "No handler at bit position %lu, mask 0x%lx. Not listening "
                   "to this bit anymore",
                   bit_index,
                   channel_status);

            // Clear the bit in the me_ptr->curr_chan_mask.
            me_ptr->curr_chan_mask &= (~(1 << bit_index));
            continue;
         }

         result = me_ptr->qftable[bit_index](me_ptr, bit_index);

         if (result == AR_ETERMINATED)
         {
            return AR_EOK;
         }

         // In case new thread got created.
         if (posal_thread_get_tid(me_ptr->thread_id_to_exit) == posal_thread_get_curr_tid())
         {
            WCNTR_MSG(me_ptr->gu_ptr->log_id,
                   DBG_MED_PRIO,
                   "Thread ID 0x%lX exited",
                   posal_thread_get_tid(me_ptr->thread_id_to_exit));
            return AR_EOK;
         }
      }
   }
   return result;
}

void cntr_cmn_dump_debug_info(spf_handle_t *cntr_handle, uint32_t container_type, int8_t *start_address, uint32_t max_size)
{
   ar_result_t result     = AR_EOK;
   uint32_t cntr_index=0;
   if (AR_DID_FAIL(result = wcntr_get_index_for_cntr_type((uint32_t)container_type, &cntr_index)))
   {
     return ;
   }
   global_dump_debug_info_ftable[cntr_index](cntr_handle,start_address,max_size);
   return;

}

void wcntr_dump_debug_info(spf_handle_t *handle, int8_t *start_address,uint32_t max_size)
{

   AR_MSG(DBG_HIGH_PRIO, "wcntr_dump_debug_info %X %u ",start_address,max_size);

   wcntr_t *           me_ptr         = (wcntr_t *)handle;
   wcntr_debug_info_t *debug_info     = (wcntr_debug_info_t *)start_address;
   int8_t *            next_address   = start_address;
   uint32_t            remaining_size = max_size;

   if (me_ptr == NULL || start_address == NULL)
   {
      return;
   }

   /*
   Packing is done in the following order
   Container info.
   Subgraph info S1.
   Modules present in subgraph
   Subgraph info S2.
   Modules present in subgraph
   */

   if (remaining_size < sizeof(wcntr_debug_info_t))
   {
      return;
   }

   debug_info->size_filled             = 0;
   debug_info->num_sg_debug_info_filled=0;
   debug_info->max_size               = max_size;
   debug_info->op_frame_in_ms         = me_ptr->op_frame_in_ms;
   debug_info->signal_trigger_count   = me_ptr->signal_trigger_count;
   debug_info->signal_miss_count      = me_ptr->signal_miss_count;
   debug_info->actual_stack_size      = me_ptr->cu.actual_stack_size;
   debug_info->configured_stack_size  = me_ptr->cu.configured_stack_size;
   debug_info->configured_thread_prio = me_ptr->cu.configured_thread_prio;
   debug_info->curr_chan_mask         = me_ptr->cu.curr_chan_mask;
   debug_info->num_subgraphs          = me_ptr->cu.gu_ptr->num_subgraphs;
   debug_info->size_filled            = debug_info->size_filled + sizeof(wcntr_debug_info_t);

   remaining_size = remaining_size - sizeof(wcntr_debug_info_t);
   next_address   = next_address + sizeof(wcntr_debug_info_t);
   wcntr_topo_t *topo_ptr=(wcntr_topo_t *)me_ptr->cu.topo_ptr;

   for (wcntr_gu_sg_list_t *sg_list_ptr = topo_ptr->gu.sg_list_ptr; (NULL != sg_list_ptr);
        LIST_ADVANCE(sg_list_ptr))
   {

      if (remaining_size < sizeof(wcntr_sg_debug_info_t))
      {
         return;
      }

      wcntr_gu_sg_t *sub_graph_ptr = sg_list_ptr->sg_ptr;

      if (sub_graph_ptr == NULL)
      {
         continue;
      }

      wcntr_sg_debug_info_t *sg_debug_info = (wcntr_sg_debug_info_t *)next_address;

      sg_debug_info->id                        = sub_graph_ptr->id;
      sg_debug_info->direction                 = sub_graph_ptr->direction;
      sg_debug_info->num_modules               = sub_graph_ptr->num_modules;
      sg_debug_info->num_boundary_in_ports     = sub_graph_ptr->num_boundary_in_ports;
      sg_debug_info->num_boundary_out_ports    = sub_graph_ptr->num_boundary_out_ports;
      sg_debug_info->num_mod_debug_info_filled = 0;

      debug_info->size_filled = debug_info->size_filled + sizeof(wcntr_sg_debug_info_t);
      remaining_size          = remaining_size - sizeof(wcntr_sg_debug_info_t);
      next_address            = next_address + sizeof(wcntr_sg_debug_info_t);

      for (wcntr_gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != module_list_ptr);
           LIST_ADVANCE(module_list_ptr))
      {
         wcntr_topo_module_t *module_ptr = (wcntr_topo_module_t *)module_list_ptr->module_ptr;

         if (module_ptr == NULL)
         {
            continue;
         }

         if (remaining_size < sizeof(wcntr_topo_module_debug_t))
         {
            return;
         }

         wcntr_topo_module_debug_t *module_debug_info = (wcntr_topo_module_debug_t *)next_address;

         module_debug_info->module_id          = module_ptr->gu.module_id;
         module_debug_info->module_instance_id = module_ptr->gu.module_instance_id;
         module_debug_info->module_type        = module_ptr->gu.module_type;
         module_debug_info->itype = module_ptr->gu.itype;
         module_debug_info->num_input_ports  = module_ptr->gu.num_input_ports;
         module_debug_info->num_output_ports = module_ptr->gu.num_output_ports;
         module_debug_info->num_ctrl_ports   = module_ptr->gu.num_ctrl_ports;
         module_debug_info->kpps             = module_ptr->kpps;
         module_debug_info->algo_delay       = module_ptr->algo_delay;
         module_debug_info->code_bw          = module_ptr->code_bw;
         module_debug_info->data_bw          = module_ptr->data_bw;

         if (module_ptr->bypass_ptr)
         {
            module_debug_info->module_bypassed = 1;
         }
         module_debug_info->can_process_be_called = module_ptr->can_process_be_called;

         sg_debug_info->num_mod_debug_info_filled++;
         debug_info->size_filled = debug_info->size_filled + sizeof(wcntr_topo_module_debug_t);
         remaining_size          = remaining_size - sizeof(wcntr_topo_module_debug_t);
         next_address            = next_address + sizeof(wcntr_topo_module_debug_t);
      }

      debug_info->num_sg_debug_info_filled++;
   }

   return;
}
