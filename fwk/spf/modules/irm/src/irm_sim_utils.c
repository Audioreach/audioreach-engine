/**
@file irm_sim_utils.cpp

@brief Profiler driver and Dev cfg file for IRM q6 processor.

================================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/
// clang-format off
// clang-format on

#include "irm_i.h"


#if defined(SIM)
#include "amssheap.h"
#include "irm_sysmon_util.h"
#include "spf_svc_calib.h"
#include "qurt_cycles.h"

#include "vcpm_tst_fwk_api.h"
#include "qurt_event.h"

typedef struct irm_test_fwk_event_t
{
   gpr_cmd_alloc_ext_t cb_pkt;
} irm_test_fwk_event_t;

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
int32_t irm_wrapper_profile_enable(uint32_t block_id, int32_t enable)
{
#if defined(AVS_USES_QURT_PROF)
   qurt_profile_enable(enable);
#endif
   return 0;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
int32_t irm_query_processor_metrics(sysmon_audio_query_t *query_ptr, uint64_t *idle_pcycles_ptr)
{

   query_ptr->sysclocktick            = 0;
   query_ptr->busVoteinAb             = 0;
   query_ptr->busVoteinIb             = 0;
   query_ptr->coreClkVoteinkHz        = 0;
   query_ptr->avsheap_total_bytes     = amssheap_get_current_heap_usages();
   query_ptr->avsheap_available_total = 0;
   query_ptr->avsheap_available_max   = 0;
   query_ptr->pktcnt                  = 0;
   query_ptr->axi_rd_cnt              = 0;
   query_ptr->axi_wr_cnt              = 0;
   query_ptr->pcycles                 = qurt_get_core_pcycles();
   qurt_profile_get_idle_pcycles(idle_pcycles_ptr);
   qurt_profile_reset_idle_pcycles();

   return 0;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
void irm_get_q6_hw_info(irm_metric_id_q6_hw_info_t *q6_hw_info_ptr)
{
   qurt_sysenv_max_hthreads_t hw_thread_info;
   qurt_sysenv_get_max_hw_threads(&hw_thread_info);
   q6_hw_info_ptr->num_hw_threads = hw_thread_info.max_hthreads;

   qurt_arch_version_t arch_version_info;
   qurt_sysenv_get_arch_version(&arch_version_info);
   q6_hw_info_ptr->q6_arch_version = arch_version_info.arch_version;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
void irm_calculate_non_idle_cycles(irm_metric_id_q6_hw_info_t *q6_hw_info_ptr,
                                   uint32_t *                  pcyles_ptr,
                                   uint64_t *                  idle_cycles_ptr)
{
   uint32_t non_idle_cycles = 0;
   for (uint32_t hw_thread = 0; ((hw_thread < q6_hw_info_ptr->num_hw_threads) && (hw_thread < IRM_MAX_NUM_HW_THREADS));
        hw_thread++)
   {
      non_idle_cycles += (*pcyles_ptr) - idle_cycles_ptr[hw_thread];
   }
   *pcyles_ptr = non_idle_cycles;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
ar_result_t irm_process_register_module_event(irm_t *irm_ptr, gpr_packet_t *gpr_pkt_ptr)
{
   ar_result_t result = 0;

   // check contents of header here - break here & inpsect header (should be all
   // 0's except for size)
   apm_cmd_header_t *    in_apm_cmd_header = GPR_PKT_GET_PAYLOAD(apm_cmd_header_t, gpr_pkt_ptr);
   irm_test_fwk_event_t *testfwk_event     = NULL;
   uint32_t              payload_size      = in_apm_cmd_header->payload_size;

   gpr_packet_t *dummy_pkt_ptr;
   uint32_t      byte_aligned_size;
   uint8_t *     payload = NULL;

   result = spf_svc_get_cmd_payload_addr(IRM_MODULE_INSTANCE_ID,
                                         gpr_pkt_ptr,
                                         &dummy_pkt_ptr,
                                         (uint8_t **)&payload,
                                         &byte_aligned_size,
                                         NULL,
                                         apm_get_mem_map_client());

   if (AR_EOK != result || NULL == payload)
   {
      AR_MSG(DBG_ERROR_PRIO, "IRM: irm_process_register_module_event: failed to get command payload");
      return result;
   }

   while (payload_size > 0)
   {
      apm_module_register_events_t *current_payload = (apm_module_register_events_t *)payload;
      if (sizeof(apm_module_register_events_t) > payload_size)
      {
         break;
      }

      uint32_t one_event_size = sizeof(apm_module_register_events_t) + current_payload->event_config_payload_size;

      if (payload_size >= one_event_size)
      {
         // Deregister - free up the event ptr if it is allocated
         if (!current_payload->is_register)
         {
            AR_MSG(DBG_LOW_PRIO, "IRM: irm_process_register_module_event: De-registering event");
            if (NULL != irm_ptr->core.test_fwk_event_ptr)
            {
               posal_memory_free(irm_ptr->core.test_fwk_event_ptr);
               irm_ptr->core.test_fwk_event_ptr = NULL;
            }
            break;
         }

         if (NULL == irm_ptr->core.test_fwk_event_ptr)
         {
            // Assume that if it's null then it just hasn't been allocated yet
            irm_ptr->core.test_fwk_event_ptr = posal_memory_malloc(sizeof(irm_test_fwk_event_t), irm_ptr->heap_id);
            if (NULL == irm_ptr->core.test_fwk_event_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "IRM: irm_process_register_module_event: failed to allocate memory for IRM testwfk event");
               return AR_ENOMEMORY;
            }
         }
         memset(irm_ptr->core.test_fwk_event_ptr, 0, sizeof(irm_test_fwk_event_t));
         testfwk_event = (irm_test_fwk_event_t *)irm_ptr->core.test_fwk_event_ptr;

         testfwk_event->cb_pkt.dst_domain_id = gpr_pkt_ptr->src_domain_id;
         testfwk_event->cb_pkt.src_domain_id = gpr_pkt_ptr->dst_domain_id;
         testfwk_event->cb_pkt.client_data   = gpr_pkt_ptr->client_data;
         testfwk_event->cb_pkt.src_port      = gpr_pkt_ptr->dst_port;
         testfwk_event->cb_pkt.dst_port      = gpr_pkt_ptr->src_port;
         testfwk_event->cb_pkt.token         = gpr_pkt_ptr->token;
         testfwk_event->cb_pkt.opcode        = APM_EVENT_MODULE_TO_CLIENT;
         AR_MSG(DBG_LOW_PRIO, "IRM: irm_process_register_module_event: event registered");
      }
      payload += ALIGN_8_BYTES(one_event_size);
      payload_size -= one_event_size;
   }

   return result;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
ar_result_t irm_tst_fwk_override_event_send(irm_t *irm_ptr)
{
   ar_result_t           result        = AR_EOK;
   irm_test_fwk_event_t *testfwk_event = (irm_test_fwk_event_t *)irm_ptr->core.test_fwk_event_ptr;
   if (NULL != testfwk_event)
   {
      gpr_packet_t *      event_packet_ptr = NULL;
      apm_module_event_t *event_payload;
      testfwk_event->cb_pkt.payload_size = sizeof(apm_module_event_t) + sizeof(uint64_t);
      testfwk_event->cb_pkt.ret_packet   = &event_packet_ptr;
      (void)__gpr_cmd_alloc_ext(&testfwk_event->cb_pkt);

      if (NULL == event_packet_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO, "IRM: irm_tst_fwk_event_send: allocate gpr packet failed");
         return AR_EFAILED;
      }

      AR_MSG(DBG_HIGH_PRIO, "IRM: irm_tst_fwk_override_event_send: Sending island sleep override");

      event_payload = GPR_PKT_GET_PAYLOAD(apm_module_event_t, event_packet_ptr);

      event_payload->event_id                             = EVENT_ID_TESTFWK_ISLAND_SLEEP_OVERRIDE;
      event_payload->event_payload_size                   = sizeof(event_id_island_sleep_override_t);
      event_id_island_sleep_override_t *event_payload_ptr = ((event_id_island_sleep_override_t *)(event_payload + 1));
      event_payload_ptr->do_not_allow_island_sleep        = TRUE;

      if (AR_EOK != __gpr_cmd_async_send(event_packet_ptr))
      {
         AR_MSG(DBG_ERROR_PRIO, "IRM: irm_tst_fwk_event_send: async send gpr packet failed");
         __gpr_cmd_free(event_packet_ptr);
         result = AR_EFAILED;
      }
   }
   return result;
}
#else

#ifdef __qdsp6__
void irm_get_q6_hw_info(irm_metric_id_q6_hw_info_t *q6_hw_info_ptr)
{
   return;
}

void irm_calculate_non_idle_cycles(irm_metric_id_q6_hw_info_t *q6_hw_info_ptr,
                                   uint32_t *                  pcyles_ptr,
                                   uint64_t *                  idle_cycles_ptr)
{
   return;
}

#endif //__qdsp6__

ar_result_t irm_tst_fwk_override_event_send(irm_t *irm_ptr)
{
   return AR_EOK;
}

ar_result_t irm_process_register_module_event(irm_t *irm_ptr, gpr_packet_t *gpr_pkt_ptr)
{
   return AR_EOK;
}
#endif
