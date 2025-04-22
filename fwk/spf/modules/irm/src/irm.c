/**
@file irm.cpp

@brief Main file for Integrated Resource Monitor (IRM).

================================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/
// clang-format off
// clang-format on

#include "irm_i.h"
#include "irm_api.h"
#include "spf_macros.h"
#include "spf_svc_calib.h"
#include "ar_guids.h"
#include "irm_offload_utils.h"
#include "posal_mem_prof.h"

static irm_t g_irm_info;

/** Pointer to irm module global struct */
irm_t *g_irm_ptr;

static char           irm_cmd_q_name[]      = "irm_cmd_q";
static char           irm_rsp_q_name[]      = "irm_rsp_q";
static const uint32_t irm_cmd_q_mask        = 0x00000001UL;
static const uint32_t irm_rsp_q_mask        = 0x00000002UL;
static const uint32_t irm_report_timer_mask = 0x00000004UL;
static const uint32_t irm_sys_q_mask        = 0x00000008UL;
static const uint32_t irm_pm_mask           = 0x00000010UL;
static char           IRM_THREAD_NAME[]     = "IRM";

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/

void irm_clean_up_memory(irm_t *irm_ptr)
{
   if (irm_ptr->core.is_profiling_started)
   {
      posal_timer_destroy(&irm_ptr->irm_report_timer);
      irm_ptr->core.is_profiling_started = FALSE;
   }
   irm_clean_up_all_nodes(irm_ptr);
   posal_mem_prof_stop();

   irm_ptr->core.is_cntr_or_mod_prof_enabled = FALSE;

   if (NULL != irm_ptr->core.report_payload_ptr)
   {
      posal_memory_free((void *)irm_ptr->core.report_payload_ptr);
      irm_ptr->core.report_payload_ptr  = NULL;
      irm_ptr->core.report_payload_size = 0;
   }
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
uint32_t irm_gpr_call_back_f(gpr_packet_t *gpr_pkt_ptr, void *cb_ctx_ptr)
{
   uint32_t       result = AR_EOK;
   spf_msg_t      msg;
   uint32_t       cmd_opcode;
   spf_handle_t  *dst_handle_ptr = NULL;
   posal_queue_t *temp_ptr       = NULL;

   /* Validate GPR packet pointer */
   if (!gpr_pkt_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "IRM GPR CB: GPR pkt ptr is NULL");

      return AR_EFAILED;
   }

   /* Get the GPR command opcode */
   cmd_opcode           = gpr_pkt_ptr->opcode;
   uint32_t opcode_type = ((cmd_opcode & AR_GUID_TYPE_MASK) >> AR_GUID_TYPE_SHIFT);

   AR_MSG(DBG_HIGH_PRIO, "IRM GPR CB, rcvd cmd opcode[0x%08lX]", cmd_opcode);

   /* Validate GPR callback context pointer */
   if (!cb_ctx_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "IRM GPR CB: CB ctxt ptr is NULL");
      goto __bailout;
   }

   /* Get the destination module handle */
   dst_handle_ptr = (spf_handle_t *)cb_ctx_ptr;
   temp_ptr       = dst_handle_ptr->cmd_handle_ptr->cmd_q_ptr;

   if ((((opcode_type & AR_GUID_TYPE_CONTROL_CMD_RSP) == AR_GUID_TYPE_CONTROL_CMD_RSP)) &&
       ((posal_queue_t *)g_irm_ptr->irm_cmd_q_ptr == dst_handle_ptr->cmd_handle_ptr->cmd_q_ptr))
   {
      dst_handle_ptr->cmd_handle_ptr->cmd_q_ptr = (posal_queue_t *)g_irm_ptr->irm_rsp_q_ptr;
   }

   /** Compose the GK message payload to be routed to
    *  destination module */
   msg.msg_opcode  = SPF_MSG_CMD_GPR;
   msg.payload_ptr = gpr_pkt_ptr;

   /* Push msg to the destination module queue */
   if (AR_DID_FAIL(result = spf_msg_send_cmd(&msg, dst_handle_ptr)))
   {
      AR_MSG(DBG_HIGH_PRIO, "IRM : Failed to push gpr msg to irm cmd_q");
      goto __bailout;
   }

   dst_handle_ptr->cmd_handle_ptr->cmd_q_ptr = temp_ptr; // replace it with the actual qptr
   return result;

__bailout:

   /* End the GPR command */
   __gpr_cmd_end_command(gpr_pkt_ptr, result);

   return AR_EOK;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
ar_result_t irm_process_cmd_q(irm_t *irm_ptr)
{
   ar_result_t result;
   spf_msg_t   msg_pkt;

   /** Pop the message buffer from the cmd queue */
   if (AR_EOK !=
       (result = posal_queue_pop_front((posal_queue_t *)irm_ptr->irm_cmd_q_ptr, (posal_queue_element_t *)&msg_pkt)))
   {
      AR_MSG(DBG_ERROR_PRIO, "irm_process_cmd_q(): Failed to pop buf from svc cmd_q, result: 0x%lx", result);
      return result;
   }

   if (SPF_MSG_CMD_GPR == msg_pkt.msg_opcode)
   {
      AR_MSG(DBG_HIGH_PRIO, "IRM: irm_process_cmd_q(): GPR CMD received");
      /** Call the irm svc command queue handler */
      result |= irm_cmdq_gpr_cmd_handler(irm_ptr, &msg_pkt);

      return result;
   }
   else
   {
      AR_MSG(DBG_HIGH_PRIO, "IRM: irm_process_cmd_q(): CMD received 0x%lx", msg_pkt.msg_opcode);
      result |= irm_cmdq_apm_cmd_handler(irm_ptr, &msg_pkt);
      return result;
   }
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
ar_result_t irm_process_rsp_q(irm_t *irm_ptr)
{
   ar_result_t result;
   spf_msg_t   msg_pkt;

   /** Pop the message buffer from the cmd rsp queue */
   if (AR_EOK !=
       (result = posal_queue_pop_front((posal_queue_t *)irm_ptr->irm_rsp_q_ptr, (posal_queue_element_t *)&msg_pkt)))
   {
      AR_MSG(DBG_ERROR_PRIO, "irm_process_cmd_q(): Failed to pop buf from irm cmd_rsp_q, result: 0x%lx", result);
      return result;
   }

   if (SPF_MSG_CMD_GPR == msg_pkt.msg_opcode)
   {
      AR_MSG(DBG_HIGH_PRIO, "IRM: irm_process_rsp_q(): GPR CMD received");
      /** Call the irm svc command queue handler */
      result |= irm_rspq_gpr_rsp_handler(irm_ptr, &msg_pkt);

      return result;
   }
   else
   {
      AR_MSG(DBG_HIGH_PRIO, "IRM: irm_process_rsp_q(): CMD received 0x%lx", msg_pkt.msg_opcode);
      result |= irm_rspq_spf_rsp_handler(irm_ptr, &msg_pkt);
      return result;
   }

   /** Call the IRM svc response queue handler */
   return result;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
ar_result_t irm_process_timer_tick(irm_t *irm_ptr)
{
   ar_result_t result = AR_EOK;
   AR_MSG(DBG_HIGH_PRIO, "IRM: Report payload timer tick");
   result = irm_timer_tick_handler(irm_ptr);

   posal_signal_clear(irm_ptr->timer_signal);
   return result;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
ar_result_t irm_handle_close_all(void *close_all_ptr)
{
   ar_result_t                    result  = AR_EOK;
   irm_t                         *irm_ptr = &g_irm_info;
   spf_msg_t                      msg_pkt;
   uint32_t                       curr_wait_mask = irm_cmd_q_mask;
   uint32_t                       channel_status;
   param_id_sys_util_close_all_t *close_all_paylaod_ptr = (param_id_sys_util_close_all_t *)close_all_ptr;

   AR_MSG(DBG_HIGH_PRIO, "IRM: Flushing CMD queue");

   if (close_all_paylaod_ptr->is_flush_needed)
   {
      while (irm_cmd_q_mask & (channel_status = posal_channel_poll(irm_ptr->cmd_channel, curr_wait_mask)))
      {
         /** Pop the message buffer from the cmd queue */
         if (AR_EOK != (result = posal_queue_pop_front((posal_queue_t *)irm_ptr->irm_cmd_q_ptr,
                                                       (posal_queue_element_t *)&msg_pkt)))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "irm_handle_flush_and_reset(): Failed to pop buf from svc cmd_q, result: 0x%lx",
                   result);
            return result;
         }
         if (SPF_MSG_CMD_GPR == msg_pkt.msg_opcode)
         {
            // Get the pointer to GPR command
            gpr_packet_t *gpr_pkt_ptr = (gpr_packet_t *)msg_pkt.payload_ptr;

            if (NULL != gpr_pkt_ptr)
            {
               // Flush the cmd by failing it, but return success
               __gpr_cmd_end_command(gpr_pkt_ptr, AR_EFAILED);
               return AR_EOK;
            }
         }
         else
         {
            // flush the command saying failed
            result = spf_msg_ack_msg(&msg_pkt, AR_EFAILED);
         }
      }
   }

   if (close_all_paylaod_ptr->is_reset_needed)
   {
      irm_clean_up_memory(irm_ptr);
   }
   return result;
}
/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
ar_result_t irm_handle_sat_pd_info(uint32_t num_proc_domain_ids, uint32_t *proc_domain_id_list)
{
   return spf_sys_util_ssr_register(g_irm_ptr->sys_util_handle_ptr, num_proc_domain_ids, proc_domain_id_list);
}
/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
ar_result_t irm_process_q(irm_t *irm_ptr, uint32_t channel_status)
{
   ar_result_t result = AR_EOK;

   if (!channel_status)
   {
      return AR_EOK;
   }

   if (irm_ptr->sys_util_handle_ptr)
   {
      if (channel_status & irm_ptr->sys_util_handle_ptr->sys_queue_mask)
      {
         result = spf_sys_util_handle_cmd(irm_ptr->sys_util_handle_ptr);
         return result;
      }
   }

   /** Process service command Q */
   if (channel_status & irm_cmd_q_mask)
   {
      result |= irm_process_cmd_q(irm_ptr);
   }

   /** Process service response Q */
   if (channel_status & irm_rsp_q_mask)
   {
      result |= irm_process_rsp_q(irm_ptr);
   }

   if (channel_status & irm_report_timer_mask)
   {
      result |= irm_process_timer_tick(irm_ptr);
   }

   return result;
}
#define TEST_SSR_PD_INFO 0
#if TEST_SSR_PD_INFO
#include "gpr_proc_info_api.h"
#endif
/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
static ar_result_t irm_work_loop(void *arg_ptr)
{
   ar_result_t result         = AR_EOK;
   uint32_t    kill_sig_mask  = 0;
   uint32_t    curr_wait_mask = 0;
   uint32_t    sys_queue_mask = 0;
   uint32_t    channel_status;

   /** Service Instance */
   irm_t *irm_ptr = (irm_t *)arg_ptr;

   /** Get the signal bit for kill signal */
   kill_sig_mask = posal_signal_get_channel_bit(irm_ptr->kill_signal);

   sys_queue_mask = (NULL != irm_ptr->sys_util_handle_ptr) ? irm_ptr->sys_util_handle_ptr->sys_queue_mask : 0;
   /** Set up mask for listening to the service cmd,rsp queues and
    *  kill signal for IRM thread */
   curr_wait_mask = (irm_cmd_q_mask | irm_rsp_q_mask | kill_sig_mask | irm_report_timer_mask | sys_queue_mask);

   AR_MSG(DBG_HIGH_PRIO, "IRM: Entering irm workloop...");

#if TEST_SSR_PD_INFO // test code only on sim..
   posal_timer_sleep(10000);
   AR_MSG(DBG_HIGH_PRIO, "PBM: inside irm_work_loop, testing pd id - string gpr apis");
   char_t service_name[GPR_PD_SUBSTRING_MAX_SIZE + 1];
   gpr_get_audio_service_name(service_name, GPR_PD_SUBSTRING_MAX_SIZE + 1);
   AR_MSG(DBG_HIGH_PRIO, "PBM: service name = %s", service_name);

   char_t   domain_name[GPR_PD_SUBSTRING_MAX_SIZE + 1];
   uint32_t pd_id = 1;
   result         = gpr_get_pd_str_from_id(pd_id, domain_name, GPR_PD_SUBSTRING_MAX_SIZE + 1);
   AR_MSG(DBG_HIGH_PRIO, "PBM: PDID = %lu name = %s", pd_id, domain_name);

   pd_id  = 0;
   result = gpr_get_pd_id_from_str(&pd_id, domain_name, GPR_PD_SUBSTRING_MAX_SIZE + 1);
   AR_MSG(DBG_HIGH_PRIO, "PBM: PDID = %lu name = %s", pd_id, domain_name);

#endif

   //   posal_timer_sleep(50000);
   //   uint32_t host_domain_id = 0xFFFFFFFF;
   //   __gpr_cmd_get_host_domain_id(&host_domain_id);
   //
   //   if (0x2 == host_domain_id)
   //   {
   //      irm_ptr->core.profiling_period_us     = 20000;
   //      irm_ptr->core.num_profiles_per_report = 5;
   //      typedef struct set_enable_disable_t
   //      {
   //         param_id_enable_disable_metrics_t enable_disable;
   //         irm_enable_disable_block_t        enable_payload;
   //         uint32_t                          metric_ids[5];
   //
   //      } set_enable_disable_t;
   //
   //      set_enable_disable_t set_enable_disable;
   //      set_enable_disable.enable_disable.is_enable      = 1;
   //      set_enable_disable.enable_disable.num_blocks     = 1;
   //      set_enable_disable.enable_disable.proc_domain    = host_domain_id;
   //      set_enable_disable.enable_payload.block_id       = IRM_BLOCK_ID_PROCESSOR;
   //      set_enable_disable.enable_payload.instance_id    = 0;
   //      set_enable_disable.enable_payload.num_metric_ids = 5;
   //
   //      set_enable_disable.metric_ids[0] = IRM_METRIC_ID_MEM_TRANSACTIONS;
   //      set_enable_disable.metric_ids[1] = IRM_METRIC_ID_PACKET_COUNT;
   //      set_enable_disable.metric_ids[2] = IRM_METRIC_ID_HEAP_INFO;
   //      set_enable_disable.metric_ids[3] = IRM_BASIC_METRIC_ID_CURRENT_CLOCK;
   //      set_enable_disable.metric_ids[4] = IRM_METRIC_ID_PROCESSOR_CYCLES;
   //      (void)irm_handle_set_enable(irm_ptr, (param_id_enable_disable_metrics_t *)&set_enable_disable);
   //   }
   /** Enter forever loop */
   for (;;)
   {
      /** Block on any one or more of selected queues to get a msg */
      channel_status = posal_channel_wait(irm_ptr->cmd_channel, curr_wait_mask);

      /** Check if the KILL signal is received */
      if (channel_status & kill_sig_mask)
      {
         posal_signal_clear(irm_ptr->kill_signal);

         /** Return from the workloop */
         return AR_EOK;
      }

      result = irm_process_q(irm_ptr, channel_status);
   } /** forever loop */

   return result;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
ar_result_t irm_handle_svc_status(void *status_ptr)
{
   ar_result_t                     result          = AR_EOK;
   irm_t                          *irm_ptr         = &g_irm_info;
   param_id_sys_util_svc_status_t *proc_status_ptr = (param_id_sys_util_svc_status_t *)status_ptr;
   AR_MSG(DBG_ERROR_PRIO,
          "IRM: SSR notified to IRM, cleaning up cached cmd ctrl, proc_domain_id = %lu, status = %lu",
          proc_status_ptr->proc_domain_id,
          proc_status_ptr->status);

   if (SPF_SYS_UTIL_SSR_STATUS_DOWN == proc_status_ptr->status)
   {
      result = irm_clean_up_proc_id_cmd_ctrl(irm_ptr, proc_status_ptr->proc_domain_id);
   }

   return result;
}

/*----------------------------------------------------------------------------------------------------------------------
 Creates the necessary Queue, workloop, bufpool, signal etc required for IRM
----------------------------------------------------------------------------------------------------------------------*/
ar_result_t irm_init(POSAL_HEAP_ID heap_id)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result  = AR_EOK;
   irm_t      *irm_ptr = &g_irm_info;

   // fixme
   heap_id          = IRM_INTERNAL_STATIC_HEAP_ID;
   irm_ptr->heap_id = heap_id;

   posal_queue_init_attr_t q_attr;
   posal_thread_prio_t     irm_thread_prio = 0;
   uint32_t sched_policy = 0, affinity_mask = 0;
   prio_query_t            query_tbl;

   AR_MSG(DBG_HIGH_PRIO, "Entering irm_init() ...");

   memset(irm_ptr, 0, sizeof(irm_t));
   g_irm_ptr = irm_ptr;

   TRY(result, posal_channel_create(&irm_ptr->cmd_channel, heap_id));

   /** Create service irm cmd queue */
   posal_queue_attr_init(&q_attr);
   posal_queue_attr_set_heap_id(&q_attr, irm_ptr->heap_id);
   posal_queue_attr_set_max_nodes(&q_attr, IRM_MAX_CMD_Q_ELEMENTS);
   posal_queue_attr_set_prealloc_nodes(&q_attr, IRM_MAX_CMD_Q_ELEMENTS);
   posal_queue_attr_set_name(&q_attr, irm_cmd_q_name);
   TRY(result,
       posal_queue_set_attributes(&q_attr,
                                  irm_ptr->heap_id,
                                  IRM_MAX_CMD_Q_ELEMENTS,
                                  IRM_MAX_CMD_Q_ELEMENTS,
                                  irm_cmd_q_name));
   TRY(result, posal_queue_create_v1(&(irm_ptr->irm_cmd_q_ptr), &q_attr));

   /** Create service irm rsp queue */
   TRY(result,
       posal_queue_set_attributes(&q_attr,
                                  irm_ptr->heap_id,
                                  IRM_MAX_RSP_Q_ELEMENTS,
                                  IRM_MAX_RSP_Q_ELEMENTS,
                                  irm_rsp_q_name));
   TRY(result, posal_queue_create_v1(&(irm_ptr->irm_rsp_q_ptr), &q_attr));

   TRY(result, posal_signal_create(&irm_ptr->kill_signal, heap_id));

   TRY(result, posal_signal_create(&irm_ptr->timer_signal, heap_id));

   TRY(result, posal_signal_create(&irm_ptr->pm_info.pm_signal, heap_id));

   TRY(result, posal_channel_addq(irm_ptr->cmd_channel, (posal_queue_t *)irm_ptr->irm_cmd_q_ptr, irm_cmd_q_mask));

   TRY(result, posal_channel_addq(irm_ptr->cmd_channel, (posal_queue_t *)irm_ptr->irm_rsp_q_ptr, irm_rsp_q_mask));

   TRY(result, posal_channel_add_signal(irm_ptr->cmd_channel, irm_ptr->kill_signal, POSAL_CHANNEL_MASK_DONT_CARE));

   TRY(result, posal_channel_add_signal(irm_ptr->cmd_channel, irm_ptr->timer_signal, irm_report_timer_mask));

   TRY(result, posal_channel_add_signal(irm_ptr->cmd_channel, irm_ptr->pm_info.pm_signal, irm_pm_mask));

   spf_sys_util_vtable vtable = { .spf_sys_util_handle_svc_status  = irm_handle_svc_status,
                                  .spf_sys_util_handle_close_all   = irm_handle_close_all,
                                  .spf_sys_util_handle_sat_pd_info = irm_handle_sat_pd_info };

   irm_ptr->sys_util_handle_ptr = NULL;

   TRY(result,
       spf_sys_util_get_handle(&irm_ptr->sys_util_handle_ptr,
                               &vtable,
                               irm_ptr->cmd_channel,
                               "irm_sys_q",
                               irm_sys_q_mask,
                               heap_id,
                               IRM_MAX_SYS_Q_ELEMENTS,
                               IRM_PREALLOC_SYS_Q_ELEMENTS));

   uint32_t handle = posal_bufpool_pool_create(sizeof(irm_node_obj_t),
                                               irm_ptr->heap_id,
                                               IRM_NUM_BUF_POOL_ARRAYS,
                                               FOUR_BYTE_ALIGN,
                                               IRM_NODES_PER_ARRAY);
   if (POSAL_BUFPOOL_INVALID_HANDLE == handle)
   {
      AR_MSG(DBG_FATAL_PRIO, "Received invalid handle from posal bufpool!");
   }

   irm_ptr->core.irm_bufpool_handle = handle;
   TRY(result, irm_profiler_init(irm_ptr)); // To avoid mem leak during sim testing

   // Initialize in the IRM core variables
   irm_ptr->core.is_profiling_started    = FALSE;
   irm_ptr->core.profiling_period_us     = IRM_MIN_PROFILING_PERIOD_1MS;
   irm_ptr->core.num_profiles_per_report = IRM_MIN_PROFILES_PER_REPORT_1;
   TRY(result, posal_mutex_create(&irm_ptr->core.cntr_mod_prof_enable_mutex, irm_ptr->heap_id));
   irm_ptr->core.is_cntr_or_mod_prof_enabled = FALSE;
   irm_ptr->enable_all                       = FALSE;

   irm_ptr->pm_info.register_info.mode = PM_MODE_DEFAULT;
   /** Register with power manager */
   TRY(result,
       posal_power_mgr_register(irm_ptr->pm_info.register_info,
                                &irm_ptr->pm_info.pm_handle_ptr,
                                irm_ptr->pm_info.pm_signal,
                                IRM_MODULE_INSTANCE_ID));

   query_tbl.frame_duration_us = 0; // frame duration is dont care since its static thread prio
   query_tbl.is_interrupt_trig = FALSE;
   query_tbl.static_req_id     = SPF_THREAD_STAT_PRM_ID;
   TRY(result, posal_thread_determine_attributes(&query_tbl, &irm_thread_prio, &sched_policy, &affinity_mask));

   /** Launch the thread */
   TRY(result,
       posal_thread_launch3(&irm_ptr->irm_cmd_handle.thread_id,
                           IRM_THREAD_NAME,
                           IRM_THREAD_STACK_SIZE,
                           0,
                           irm_thread_prio,
                           irm_work_loop,
                           (void *)irm_ptr,
                           irm_ptr->heap_id,
                           sched_policy,
                           affinity_mask));

   irm_ptr->thread_launched = TRUE;

   posal_queue_t *sys_queue_ptr =
      (NULL != irm_ptr->sys_util_handle_ptr) ? irm_ptr->sys_util_handle_ptr->sys_queue_ptr : NULL;
   /** Update command handle */
   spf_msg_init_cmd_handle(&irm_ptr->irm_cmd_handle,
                           irm_ptr->irm_cmd_handle.thread_id,
                           (posal_queue_t *)irm_ptr->irm_cmd_q_ptr,
                           sys_queue_ptr);

   /** Update irm handle */
   spf_msg_init_handle(&irm_ptr->irm_handle, &irm_ptr->irm_cmd_handle, (posal_queue_t *)irm_ptr->irm_rsp_q_ptr);
   TRY(result,
       irm_register_static_module(IRM_MODULE_INSTANCE_ID,
                                  heap_id,
                                  posal_thread_get_tid_v2(irm_ptr->irm_cmd_handle.thread_id)));

   TRY(result, __gpr_cmd_register(IRM_MODULE_INSTANCE_ID, irm_gpr_call_back_f, &irm_ptr->irm_handle));

   AR_MSG(DBG_HIGH_PRIO, "IRM: thread launched successfully");

   return result;

   CATCH(result, IRM_MSG_PREFIX)
   {
      AR_MSG(DBG_ERROR_PRIO, "IRM: create failed, destroying ...");
   }

   /** Destroy irm and free-up all partially allocated resource */
   irm_deinit();

   return result;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
ar_result_t irm_reset(bool_t is_flush_needed, bool_t is_reset_needed) // TODO:pbm have stub implementation
{
   ar_result_t            result              = AR_EOK;
   irm_t                 *irm_ptr             = &g_irm_info;
   spf_sys_util_handle_t *sys_util_handle_ptr = irm_ptr->sys_util_handle_ptr;
   AR_MSG(DBG_HIGH_PRIO, "IRM: Reset called, flush = %lu, reset = %lu", is_flush_needed, is_reset_needed);

   if (NULL != sys_util_handle_ptr)
   {
      result =
         spf_sys_util_sync_push_close_all(sys_util_handle_ptr, irm_ptr->heap_id, is_flush_needed, is_reset_needed);
   }
   return result;
}

/*----------------------------------------------------------------------------------------------------------------------
 Destroys everything created as part of Init, also, return any pending bufpool buffers.
----------------------------------------------------------------------------------------------------------------------*/
ar_result_t irm_deinit()
{
#if defined(SIM)
   ar_result_t result  = AR_EOK;
   irm_t      *irm_ptr = &g_irm_info;

   if (irm_ptr->thread_launched)
   {
      /** Send Kill Signal to irm service workloop */
      posal_signal_send(irm_ptr->kill_signal);

      /** Wait for the thread join */
      posal_thread_join(irm_ptr->irm_cmd_handle.thread_id, &result);
   }

   /** Deregister with power manager */
   if (posal_power_mgr_is_registered(irm_ptr->pm_info.pm_handle_ptr))
   {
      result = posal_power_mgr_deregister(&irm_ptr->pm_info.pm_handle_ptr, IRM_MODULE_INSTANCE_ID);
   }

   irm_clean_up_memory(irm_ptr);

   irm_profiler_deinit(irm_ptr);
   posal_mutex_destroy(&irm_ptr->core.cntr_mod_prof_enable_mutex);


   posal_bufpool_pool_destroy(irm_ptr->core.irm_bufpool_handle);

   spf_sys_util_release_handle(&irm_ptr->sys_util_handle_ptr);

   if (irm_ptr->irm_cmd_q_ptr)
   {
      /** De-init the service CMD queue */
      posal_queue_destroy((posal_queue_t *)irm_ptr->irm_cmd_q_ptr);
   }

   if (irm_ptr->irm_rsp_q_ptr)
   {
      /** De-init the service RSP queue */
      posal_queue_destroy((posal_queue_t *)irm_ptr->irm_rsp_q_ptr);
   }

   /** Destroy the service timer signal */
   if (NULL != irm_ptr->timer_signal)
   {
      posal_signal_destroy(&irm_ptr->timer_signal);
   }

   /** Destroy the service Kill signal */
   if (NULL != irm_ptr->kill_signal)
   {
      posal_signal_destroy(&irm_ptr->kill_signal);
   }

   /** Destroy the channel */
   if (NULL != irm_ptr->cmd_channel)
   {
      posal_channel_destroy(&irm_ptr->cmd_channel);
   }

   /** De-register with GPR */
   if (AR_EOK != (result = __gpr_cmd_deregister(IRM_MODULE_INSTANCE_ID)))
   {
      AR_MSG(DBG_ERROR_PRIO, "Failed to de-register with GPR, result: %lu", result);
   }

   memset((void *)irm_ptr, 0, sizeof(irm_t));
   AR_MSG(DBG_HIGH_PRIO, "IRM: Completed irm_deinit() ...");
#endif
   return AR_EOK;
}

ar_result_t irm_get_spf_handle(void **spf_handle_pptr)
{
   ar_result_t    result      = AR_EOK;
   spf_handle_t **handle_pptr = (spf_handle_t **)spf_handle_pptr;

   if (NULL == handle_pptr)
   {
      return AR_EBADPARAM;
   }

   if (0 != g_irm_ptr->thread_launched)
   {
      *handle_pptr = &(g_irm_ptr->irm_handle);
   }
   else
   {
      return AR_ENOTREADY;
   }

   return result;
}

void irm_buf_pool_reset()
{
   irm_t *irm_ptr = &g_irm_info;

   AR_MSG(DBG_HIGH_PRIO, "IRM: Bufpool reset called ");
   posal_bufpool_pool_reset_to_base(irm_ptr->core.irm_bufpool_handle);
}

bool_t irm_is_cntr_or_mod_prof_enabled()
{
   irm_t *irm_ptr = &g_irm_info;
   bool_t rv;
   posal_mutex_lock(irm_ptr->core.cntr_mod_prof_enable_mutex);
   rv = irm_ptr->core.is_cntr_or_mod_prof_enabled;
   posal_mutex_unlock(irm_ptr->core.cntr_mod_prof_enable_mutex);
   return rv;
}
