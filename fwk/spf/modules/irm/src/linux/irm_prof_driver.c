/**
@file irm_prof_drv_q6.cpp

@brief Profiler driver and Dev cfg file for IRM q6 processor.

================================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/

#include "irm_dev_cfg.h"
#include "irm_api.h"
#include "irm_i.h"
#include "spf_macros.h"
#include "spf_svc_calib.h"
#include "irm_prev_metric_info.h"
#include "posal_mem_prof.h"
#include "private_irm_api.h"
#include "posal_thread_profiling.h"
#include "posal_island.h"

#include <sys/time.h>
#include <sys/times.h>
#include <sys/resource.h>
#include <unistd.h>

#include <pthread.h>

#define IRM_DEBUG 1


irm_system_capabilities_t g_irm_cmn_capabilities = { .processor_type               = IRM_PROCESSOR_TYPE_ARM,
                                                     .min_profiling_period_us      = 200000,
                                                     .min_profile_per_report       = IRM_MIN_PROFILES_PER_REPORT_1,
                                                     .max_num_containers_supported = IRM_MAX_NUM_CONTAINERS_SUPPORTED,
                                                     .max_module_supported         = IRM_MAX_NUM_MODULES_SUPPORTED };

uint32_t g_irm_processor_metric_capabilities[] = { IRM_METRIC_ID_PROCESSOR_CYCLES
				};

uint32_t g_irm_container_metric_capabilities[]  = {  };
uint32_t g_irm_module_metric_capabilities[] = {  };

uint32_t g_irm_pool_metric_capabilities[] = {  };

uint32_t g_irm_static_mod_metric_capabilities[] = {  };


irm_capability_node_t g_capability_list[] = {
   { .block_id       = IRM_BLOCK_ID_PROCESSOR,
     .num_metrics    = sizeof(g_irm_processor_metric_capabilities) / sizeof(uint32_t),
     .capability_ptr = (uint32_t *)&g_irm_processor_metric_capabilities },
   { .block_id       = IRM_BLOCK_ID_CONTAINER,
     .num_metrics    = sizeof(g_irm_container_metric_capabilities) / sizeof(uint32_t),
     .capability_ptr = (uint32_t *)&g_irm_container_metric_capabilities },
   { .block_id       = IRM_BLOCK_ID_MODULE,
     .num_metrics    = sizeof(g_irm_module_metric_capabilities) / sizeof(uint32_t),
     .capability_ptr = (uint32_t *)&g_irm_module_metric_capabilities },
   { .block_id       = IRM_BLOCK_ID_POOL,
     .num_metrics    = sizeof(g_irm_pool_metric_capabilities) / sizeof(uint32_t),
     .capability_ptr = (uint32_t *)&g_irm_pool_metric_capabilities },
   { .block_id       = IRM_BLOCK_ID_STATIC_MODULE,
     .num_metrics    = sizeof(g_irm_static_mod_metric_capabilities) / sizeof(uint32_t),
     .capability_ptr = (uint32_t *)&g_irm_static_mod_metric_capabilities },
};

irm_capability_node_t *g_capability_list_ptr   = &g_capability_list[0];
uint32_t               g_num_capability_blocks = sizeof(g_capability_list) / sizeof(irm_capability_node_t);

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
bool_t irm_is_supported_metric(uint32_t block_id, uint32_t metric_id)
{
   uint32_t *capabilities_ptr = NULL;
   uint32_t  num_elements     = 0;
   if (block_id == IRM_BLOCK_ID_PROCESSOR)
   {
      capabilities_ptr = g_irm_processor_metric_capabilities;
      num_elements     = sizeof(g_irm_processor_metric_capabilities) / sizeof(uint32_t);
   }
   else if (block_id == IRM_BLOCK_ID_CONTAINER)
   {
      capabilities_ptr = (uint32_t *)&g_irm_container_metric_capabilities;
      num_elements     = sizeof(g_irm_container_metric_capabilities) / sizeof(uint32_t);
   }
   else if (block_id == IRM_BLOCK_ID_MODULE)
   {
      capabilities_ptr = (uint32_t *)&g_irm_module_metric_capabilities;
      num_elements     = sizeof(g_irm_module_metric_capabilities) / sizeof(uint32_t);
   }
   else if (block_id == IRM_BLOCK_ID_POOL)
   {
      capabilities_ptr = (uint32_t *)&g_irm_pool_metric_capabilities;
      num_elements     = sizeof(g_irm_pool_metric_capabilities) / sizeof(uint32_t);
   }
   else if (block_id == IRM_BLOCK_ID_STATIC_MODULE)
   {
      capabilities_ptr = (uint32_t *)&g_irm_static_mod_metric_capabilities;
      num_elements     = sizeof(g_irm_static_mod_metric_capabilities) / sizeof(uint32_t);
   }
   else
   {
      return FALSE;
   }

   for (uint32_t i = 0; i < num_elements; i++)
   {
      if (metric_id == capabilities_ptr[i])
      {
         AR_MSG(DBG_HIGH_PRIO, "IRM: is supported metric id = 0x%X", capabilities_ptr[i]);
         return TRUE;
      }
   }
   return FALSE;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
void irm_get_supported_metric_arr(uint32_t block_id, uint32_t **metric_id_list, uint32_t *num_elements)
{

   if (block_id == IRM_BLOCK_ID_PROCESSOR)
   {
      *metric_id_list = g_irm_processor_metric_capabilities;
      *num_elements   = sizeof(g_irm_processor_metric_capabilities) / sizeof(uint32_t);
   }
   else if (block_id == IRM_BLOCK_ID_CONTAINER)
   {
      *metric_id_list = (uint32_t *)&g_irm_container_metric_capabilities;
      *num_elements   = sizeof(g_irm_container_metric_capabilities) / sizeof(uint32_t);
   }
   else if (block_id == IRM_BLOCK_ID_MODULE)
   {
      *metric_id_list = (uint32_t *)&g_irm_module_metric_capabilities;
      *num_elements   = sizeof(g_irm_module_metric_capabilities) / sizeof(uint32_t);
   }
   else if (block_id == IRM_BLOCK_ID_POOL)
   {
      *metric_id_list = (uint32_t *)&g_irm_pool_metric_capabilities;
      *num_elements   = sizeof(g_irm_pool_metric_capabilities) / sizeof(uint32_t);
   }
   else if (block_id == IRM_BLOCK_ID_STATIC_MODULE)
   {
      *metric_id_list = (uint32_t *)&g_irm_static_mod_metric_capabilities;
      *num_elements   = sizeof(g_irm_static_mod_metric_capabilities) / sizeof(uint32_t);
   }
   else
   {
      AR_MSG(DBG_HIGH_PRIO, "IRM: Illigal block id= 0x%X", block_id);
   }
}

static ar_result_t irm_fill_processor_metric(irm_t                *irm_ptr,
                                             irm_node_obj_t       *metric_obj_ptr,
                                             uint32_t              frame_size_ms)
{
   ar_result_t result = AR_EOK;
   if (NULL == metric_obj_ptr)
   {
      result = AR_EFAILED;
      return result;
   }
   irm_report_metric_t    *report_metric_ptr = (irm_report_metric_t *)metric_obj_ptr->metric_info.metric_payload_ptr;
   report_metric_ptr->num_metric_payloads    = irm_ptr->core.timer_tick_counter + 1;
   report_metric_ptr++;

   uint32_t metric_size = irm_get_metric_payload_size(metric_obj_ptr->id);

   irm_report_metric_payload_t *report_metric_payload_ptr =
      (irm_report_metric_payload_t *)(((uint8_t *)report_metric_ptr) +
                                      (irm_ptr->core.timer_tick_counter *
                                       (sizeof(irm_report_metric_payload_t) + metric_size)));

   report_metric_payload_ptr->is_valid      = 1;
   report_metric_payload_ptr->frame_size_ms = frame_size_ms;
   report_metric_payload_ptr->payload_size  = metric_size;

   switch (metric_obj_ptr->id)
   {
      case IRM_METRIC_ID_PROCESSOR_CYCLES:
      {
         irm_metric_id_processor_cycles_t *payload_ptr =
            (irm_metric_id_processor_cycles_t *)(report_metric_payload_ptr + 1);
         irm_prev_metric_processor_cycles_t *prev_ptr =
            (irm_prev_metric_processor_cycles_t *)metric_obj_ptr->metric_info.prev_statistic_ptr;
		uint32_t total_time_active = 0;
		
		{
			//https://pubs.opengroup.org/onlinepubs/009604499/basedefs/sys/time.h.html
			struct rusage usage = {0};
			int rc = getrusage(RUSAGE_SELF, &usage);
			if (rc)
			{
				AR_MSG(DBG_HIGH_PRIO, "getrusage returned error %d\n", rc);
			}
			else
			{
				total_time_active = (uint32_t) ((usage.ru_utime.tv_sec*1000000.0+usage.ru_utime.tv_usec)/1000000.0 + 
				(usage.ru_stime.tv_sec*1000000.0+usage.ru_stime.tv_usec)/1000000.0);
				
				AR_MSG(DBG_HIGH_PRIO," User time %f sec\n System time %f sec. total_time_active %lu\n", \
						(usage.ru_utime.tv_sec*1000000.0+usage.ru_utime.tv_usec)/1000000.0, \
						(usage.ru_stime.tv_sec*1000000.0+usage.ru_stime.tv_usec)/1000000.0,
						total_time_active
						);
			}
		}
		//for now, report amount of time we are active
        payload_ptr->processor_cycles = (uint32_t)(total_time_active - prev_ptr->processor_cycles);

#if IRM_DEBUG
         AR_MSG(DBG_HIGH_PRIO, "IRM: pcyles = %lu", payload_ptr->processor_cycles);
#endif
         prev_ptr->processor_cycles = total_time_active;
         break;
      }
      case IRM_BASIC_METRIC_ID_CURRENT_CLOCK:
      {
         break;
      }
      case IRM_METRIC_ID_HEAP_INFO:
      {
        break;
      }
      case IRM_METRIC_ID_PACKET_COUNT:
      {
         break;
      }
      case IRM_METRIC_ID_MEM_TRANSACTIONS:
      {
         break;
      }
      case IRM_METRIC_ID_Q6_HW_INFO:
      {
         break;
      }
      case IRM_METRIC_ID_TIMESTAMP:
      {
         break;
      }
      default:
      {
         break;
      }
   }
   return result;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
static ar_result_t irm_handle_processor_metrics(irm_t *irm_ptr, irm_node_obj_t *block_obj_ptr, uint32_t frame_size_ms)
{
   spf_list_node_t     *instance_node_ptr = NULL;
   irm_node_obj_t      *instance_obj_ptr  = NULL;
   spf_list_node_t     *metric_node_ptr   = NULL;

   instance_node_ptr = block_obj_ptr->head_node_ptr;
   if (NULL == instance_node_ptr || NULL == instance_node_ptr->obj_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "IRM: Null instance node");
      return AR_EFAILED;
   }

   instance_obj_ptr = (irm_node_obj_t *)instance_node_ptr->obj_ptr;
   metric_node_ptr  = instance_obj_ptr->head_node_ptr;

   for (; NULL != metric_node_ptr; metric_node_ptr = metric_node_ptr->next_ptr)
   {
      irm_node_obj_t *metric_obj_ptr = (irm_node_obj_t *)metric_node_ptr->obj_ptr;
      irm_fill_processor_metric(irm_ptr, metric_obj_ptr, frame_size_ms);
   }
   return AR_EOK;
}


ar_result_t irm_profiler_init(irm_t *irm_ptr)
{
   return AR_EOK;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
void irm_profiler_deinit(irm_t *irm_ptr)
{
   if (NULL != irm_ptr->core.profiler_handle_ptr)
   {
      posal_memory_free(irm_ptr->core.profiler_handle_ptr);
      AR_MSG(DBG_HIGH_PRIO, "IRM: profiler memory deallocated");
   }
   irm_ptr->core.profiler_handle_ptr = NULL;
}
/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
ar_result_t irm_collect_and_fill_info(irm_t *irm_ptr, uint32_t frame_size_ms)
{
   ar_result_t result = AR_EOK;
   if (!irm_ptr)
   {
      return AR_EFAILED;
   }
   spf_list_node_t *block_node_ptr = irm_ptr->core.block_head_node_ptr;
   for (; NULL != block_node_ptr; block_node_ptr = block_node_ptr->next_ptr)
   {
      irm_node_obj_t *block_obj_ptr = (irm_node_obj_t *)block_node_ptr->obj_ptr;
      if (NULL != block_obj_ptr)
      {
         switch (block_obj_ptr->id)
         {
            case IRM_BLOCK_ID_PROCESSOR:
            {
               result |= irm_handle_processor_metrics(irm_ptr, block_obj_ptr, frame_size_ms);
               break;
            }
            case IRM_BLOCK_ID_CONTAINER:
            {
               //result |= irm_handle_container_metrics(irm_ptr, block_obj_ptr, frame_size_ms);
               break;
            }
            case IRM_BLOCK_ID_STATIC_MODULE:
            {
               //result |= irm_handle_static_module_metrics(irm_ptr, block_obj_ptr, frame_size_ms);
               break;
            }

            case IRM_BLOCK_ID_MODULE:
            {
               //result |= irm_handle_module_metrics(irm_ptr, block_obj_ptr, frame_size_ms);
               break;
            }
            case IRM_BLOCK_ID_POOL:
            {
               //result |= irm_handle_pool_metrics(irm_ptr, block_obj_ptr, frame_size_ms);
               break;
            }
            default:
            {
               AR_MSG(DBG_ERROR_PRIO, "IRM: Block ID %lu not supported", block_obj_ptr->id);
               break;
            }
         }
      }
   }
   return result;
}
