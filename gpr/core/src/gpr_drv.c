/**
 * \file gpr_drv.c
 * \brief
 *  	This file contains GPR driver implementation
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */
/******************************************************************************
 * Includes                                                                    *
 *****************************************************************************/
#include "gpr_drv_i.h"
#include "gpr_heap_i.h"

/*****************************************************************************
 * Local function definitions                                                *
 ****************************************************************************/

GPR_INTERNAL uint32_t gpr_ipc_send_done(void *buf, uint32_t length);

GPR_INTERNAL uint32_t gpr_ipc_receive(void *buf, uint32_t length);

/*****************************************************************************
 * Global variables                                                          *
 ****************************************************************************/

#ifndef HEAP1_SESSION_ARRAY_SIZE
#define HEAP1_SESSION_ARRAY_SIZE 2
#endif

// clang-format off
gpr_proc_info_t gpr_proc_info[GPR_PL_NUM_TOTAL_DOMAINS_V] = {
                                                               { GPR_IDS_DOMAIN_ID_MODEM_V, "modem" },
                                                               { GPR_IDS_DOMAIN_ID_ADSP_V, "adsp" },
                                                               { GPR_IDS_DOMAIN_ID_APPS_V, "apps" },
                                                               { GPR_IDS_DOMAIN_ID_SDSP_V, "slpi" },
                                                               { GPR_IDS_DOMAIN_ID_CDSP_V, "cdsp" },
                                                               { GPR_IDS_DOMAIN_ID_CC_DSP_V, "ccdsp" },
                                                               { GPR_IDS_DOMAIN_ID_RESERVED_1, "res1" },
                                                               { GPR_IDS_DOMAIN_ID_RESERVED_2, "res2" },
                                                               { GPR_IDS_DOMAIN_ID_RESERVED_3, "res3" },
                                                               { GPR_IDS_DOMAIN_ID_GDSP0_V, "gpdsp0" },
                                                               { GPR_IDS_DOMAIN_ID_GDSP1_V, "gpdsp1" },
                                                               { GPR_IDS_DOMAIN_ID_APPS2_V, "adsp" },
                                                            };
// clang-format on

/* Local GPR IPC table containing init,deinit functions and vtbl function pointer to be populated for all domains*/
extern struct ipc_dl_local_t local_gpr_ipc_dl_table[GPR_PL_NUM_TOTAL_DOMAINS_V];

/* local ipc vtbl functions */
extern ipc_to_gpr_vtbl_t local_vtbl;

// global context variable
extern gpr_ctxt_struct_type gpr_ctxt_struct_t;

// de-init context flag
static bool_t gpr_deinit_flag = FALSE;

/*****************************************************************************
 * Helper functions                                                          *
 ****************************************************************************/

static uint32_t gpr_alloc_and_get_proc_domain_list(ar_osal_servreg_entry_type **domain_list_pptr,
                                                   uint32_t                    *num_domains_ptr,
                                                   ar_heap_info                *heap_info_ptr)
{
   uint32_t                    result           = AR_EOK;
   uint32_t                    domain_list_size = 2;
   ar_osal_servreg_entry_type *domain_list_ptr  = NULL;
   ar_osal_servreg_entry_type  service;

   if (NULL == domain_list_pptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "GPR: NULL domain_list_pptr");
      return AR_EBADPARAM;
   }

   ar_mem_set((void *)&service, 0, sizeof(ar_osal_servreg_entry_type));
   result = gpr_get_audio_service_name(&service.name[0], AR_OSAL_SERVREG_NAME_LENGTH_MAX);
   if (AR_EOK != result)
   {
      AR_MSG(DBG_ERROR_PRIO, "GPR: failed to get audio service name from ar osal %lu", result);
      return result;
   }

   // With domain list null, this api return the size for number of domain list
   result = ar_osal_servreg_get_domainlist(&service, NULL /*domain_list*/, &domain_list_size);
   if (AR_EOK != result)
   {
      AR_MSG(DBG_ERROR_PRIO, "GPR: failed to get size for num of domains from ar osal %lu", result);
      return result;
   }

   AR_MSG(DBG_HIGH_PRIO, "GPR: rcved size %d (%d) ", domain_list_size, result);

   domain_list_ptr = (ar_osal_servreg_entry_type *)ar_heap_malloc(domain_list_size * sizeof(ar_osal_servreg_entry_type),
                                                                  heap_info_ptr);
   if (NULL == domain_list_ptr)
   {
      AR_MSG(DBG_HIGH_PRIO, "GPR: failed to malloc");
      return AR_ENOMEMORY;
   }

   ar_mem_set((void *)domain_list_ptr, 0, (domain_list_size * sizeof(ar_osal_servreg_entry_type)));

   result = ar_osal_servreg_get_domainlist(&service, domain_list_ptr, &domain_list_size);
   if (AR_EOK != result)
   {
      AR_MSG(DBG_ERROR_PRIO, "GPR: failed to get domains with ar_osal %lu", result);
      ar_heap_free(domain_list_ptr, heap_info_ptr);
      return result;
   }
   for (uint32_t i = 0; i < domain_list_size; i++)
   {
      AR_MSG(DBG_MED_PRIO, "GPR: Domain list string[%lu] %s", i, domain_list_ptr[i].name);
      // MSG_SPRINTF_1(MSG_SSID_QDSP6,MSG_LEGACY_HIGH,"Domain name fetched  %s ", domain_list_ptr[i].name);
   }

   *domain_list_pptr = domain_list_ptr;
   *num_domains_ptr  = domain_list_size;
   return result;
}

static inline bool_t gpr_is_sub_string(char_t *main_string, char_t *sub_string)
{
   return (NULL != ar_strstr(main_string, sub_string));
}

GPR_EXTERNAL uint32_t gpr_get_audio_service_name(char_t *service_name, uint32_t string_size)
{
   uint32_t result = AR_EOK;
   if (NULL != service_name)
   {
      char_t audio_service_name[] = "avs/audio";
      if (string_size < sizeof(audio_service_name))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "GPR: Service string size not enough %lu, required %lu",
                string_size,
                sizeof(audio_service_name));
         return result;
      }
      result = ar_mem_cpy(service_name, string_size, audio_service_name, sizeof(audio_service_name));
   }
   else
   {
      AR_MSG(DBG_ERROR_PRIO, "GPR: Service name is NULL");
      result = AR_EBADPARAM;
   }
   return result;
}

GPR_EXTERNAL uint32_t gpr_get_pd_str_from_id(uint32_t proc_domain_id,
                                             char_t  *proc_domain_string,
                                             uint32_t proc_domain_string_size)
{
   uint32_t                    result          = AR_EOK;
   uint32_t                    num_proc_domain = 0;
   ar_osal_servreg_entry_type *domain_list_ptr = NULL;
   int32_t                     audio_pd_idx    = -1;
   int32_t                     root_pd_idx     = -1;
   ar_heap_info                heap_info;
   char_t                      audio_pd_string[] = "audio_pd";
   char_t                      root_pd_string[]  = "root_pd";

   if (NULL == proc_domain_string)
   {
      AR_MSG(DBG_ERROR_PRIO, "GPR: null proc domain string ");
      return AR_EBADPARAM;
   }

   if ((GPR_IDS_DOMAIN_ID_INVALID_V >= proc_domain_id) || (GPR_PL_MAX_DOMAIN_ID_V < proc_domain_id))
   {
      AR_MSG(DBG_ERROR_PRIO, "GPR: Invalid proc domain id, %lu ", proc_domain_id);
      return AR_EBADPARAM;
   }

   ar_mem_set((void *)&heap_info, 0, sizeof(ar_heap_info));
   heap_info.tag         = AR_HEAP_TAG_DEFAULT;
   heap_info.align_bytes = AR_HEAP_ALIGN_8_BYTES;

   result = gpr_alloc_and_get_proc_domain_list(&domain_list_ptr, &num_proc_domain, &heap_info);
   if (AR_EOK != result)
   {
      AR_MSG(DBG_ERROR_PRIO, "GPR: Failed to get proc domain list,result %lu ", result);
      return result;
   }

   // Get the proc name from the proc id..
   for (uint32_t cur_pd_id = GPR_IDS_DOMAIN_ID_INVALID_V; cur_pd_id <= GPR_PL_MAX_DOMAIN_ID_V; cur_pd_id++)
   {
      if (gpr_proc_info[cur_pd_id].proc_id == (int32_t)proc_domain_id)
      {
         // search for the string "gpr_proc_info[i].proc_name"
         for (uint32_t domain_list_idx = 0; domain_list_idx < num_proc_domain; domain_list_idx++)
         {
            // Check if "core"(adsp, modem, slpi) string related to proc domain ID is present.
            if (gpr_is_sub_string(domain_list_ptr[domain_list_idx].name, gpr_proc_info[cur_pd_id].proc_name))
            {
               // Check if the domain string containing "core"(adsp, modem, slpi) string
               // has audio_pd or root_pd and saave the idx
               if (gpr_is_sub_string(domain_list_ptr[domain_list_idx].name, &audio_pd_string[0]))
               {
                  audio_pd_idx = domain_list_idx;
               }
               else if (gpr_is_sub_string(domain_list_ptr[domain_list_idx].name, &root_pd_string[0]))
               {
                  root_pd_idx = domain_list_idx;
               }
            }
         }
      }
   }

   char_t *pd_string = NULL;
   if (-1 != audio_pd_idx) // Found the string with "core"(adsp, modem, slpi) substring and "audio_pd"
   {
      AR_MSG(DBG_MED_PRIO, "GPR: matching string with audio_pd found ");
      pd_string = &domain_list_ptr[audio_pd_idx].name[0];
   }
   else if (-1 != root_pd_idx) // Found the string with "core"(adsp, modem, slpi) substring and "root_pd"
   {
      AR_MSG(DBG_MED_PRIO, "GPR: matching string with root_pd found ");
      pd_string = &domain_list_ptr[root_pd_idx].name[0];
   }
   else // Nothing found, don't immediately return, free the domain_list_ptr
   {
      AR_MSG(DBG_ERROR_PRIO, "GPR: Failed to get proc domain string from id %lu ", proc_domain_id);
      result = AR_EFAILED;
   }

   if (NULL != pd_string)
   {
      uint32_t string_len = (uint32_t)ar_strlen(pd_string, AR_OSAL_SERVREG_NAME_LENGTH_MAX);

      if (proc_domain_string_size < string_len)
      {
         AR_MSG(DBG_ERROR_PRIO, "GPR: Inp string size %lu not enough, need %lu ", proc_domain_string_size, string_len);
         result = AR_EFAILED;
      }
      ar_mem_cpy(proc_domain_string, proc_domain_string_size, pd_string, string_len + 1);
   }
   ar_heap_free((void *)domain_list_ptr, &heap_info);

   return result;
}

GPR_EXTERNAL uint32_t gpr_get_pd_id_from_str(uint32_t *proc_domain_id_ptr,
                                             char_t   *proc_domain_string,
                                             uint32_t  proc_domain_string_size)
{
   uint32_t                    result          = AR_EOK;
   uint32_t                    num_proc_domain = 0;
   ar_osal_servreg_entry_type *domain_list_ptr = NULL;
   ar_heap_info                heap_info;

   if (NULL == proc_domain_string)
   {
      AR_MSG(DBG_ERROR_PRIO, "GPR: null proc domain string ");
      return AR_EBADPARAM;
   }

   if (NULL == proc_domain_id_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "GPR: null proc domain id ");
      return AR_EBADPARAM;
   }

   ar_mem_set((void *)&heap_info, 0, sizeof(ar_heap_info));
   heap_info.tag         = AR_HEAP_TAG_DEFAULT;
   heap_info.align_bytes = AR_HEAP_ALIGN_8_BYTES;

   result = gpr_alloc_and_get_proc_domain_list(&domain_list_ptr, &num_proc_domain, &heap_info);
   if (AR_EOK != result)
   {
      AR_MSG(DBG_ERROR_PRIO, "GPR: Failed to get proc domain list,result %lu ", result);
      return result;
   }

   *proc_domain_id_ptr = GPR_IDS_DOMAIN_ID_INVALID_V;
   bool_t found        = FALSE;
   // search for the string "gpr_proc_info[i].proc_name"
   for (uint32_t domain_list_idx = 0; (domain_list_idx < num_proc_domain) && (FALSE == found); domain_list_idx++)
   {
      if (0 == ar_strcmp(domain_list_ptr[domain_list_idx].name, proc_domain_string, proc_domain_string_size))
      {
         for (uint32_t cur_pd_id = GPR_IDS_DOMAIN_ID_INVALID_V; cur_pd_id <= GPR_PL_MAX_DOMAIN_ID_V; cur_pd_id++)
         {
            if (gpr_is_sub_string(domain_list_ptr[domain_list_idx].name, gpr_proc_info[cur_pd_id].proc_name))
            {
               *proc_domain_id_ptr = gpr_proc_info[cur_pd_id].proc_id;
               found               = TRUE;
               break;
            }
         }
      }
   }

   if (FALSE == found)
   {
      result = AR_EFAILED;
      AR_MSG(DBG_ERROR_PRIO, "GPR: Failed to find pd id from string %s ", proc_domain_string);
   }

   ar_heap_free((void *)domain_list_ptr, &heap_info);
   return result;
}

/*****************************************************************************
 * IPC Callback Implementation                                                  *
 ****************************************************************************/
/*@brief Callback function in gpr that datalink layer calls when it receives a packet
  @param[in] domain_id  Index to retrieve remote ssid, corresponding channel name and datalink type
  @param[in] buf        Buffer received by datalink layer
  @param[in] length       Size of buffer

  @return
  #AR_EOK when successful.
*/
GPR_INTERNAL uint32_t gpr_ipc_receive(void *buf, uint32_t length)
{
   uint32_t      rc          = AR_EOK;
   gpr_packet_t *packet      = (gpr_packet_t *)buf;
   uint32_t      packet_size = GPR_PKT_GET_PACKET_BYTE_SIZE(packet->header);

   if ((NULL == buf) || (length != packet_size))
   {
      AR_MSG(DBG_ERROR_PRIO, "gpr received null buffer or invalid buffer size ");
      return AR_EBADPARAM;
   }

#ifdef GPR_DEBUG_MSG
   AR_MSG(DBG_HIGH_PRIO,
          "GPR received message over datalink layer with opcode %lx, dst port %lx, \
             src_port %lx token \
             0x%lx ",
          packet->opcode,
          packet->dst_port,
          packet->src_port,
          packet->token);
#endif

   rc = __gpr_cmd_async_send(packet);
   if (rc)
   {
      AR_MSG(DBG_ERROR_PRIO, "gpr send failed");
      __gpr_cmd_free(packet);
   }
   return AR_EOK;
}

/*@brief Callback function in gpr that datalink layer calls once it finishes sending GPR packet across
  @param[in] domain_id  Domain id of destination
  @param[in] buf        Buffer sent by datalink layer
  @param[in] length     Size of buffer

  @return
  #AR_EOK when successful.
*/
GPR_INTERNAL uint32_t gpr_ipc_send_done(void *buf, uint32_t length)
{
   gpr_packet_t *packet      = (gpr_packet_t *)buf;
   uint32_t      packet_size = GPR_PKT_GET_PACKET_BYTE_SIZE(packet->header);

   if ((NULL == buf) || (length != packet_size))
   {
      AR_MSG(DBG_ERROR_PRIO, "Received NULL buffer or buffer of incorrect size");
      return AR_EBADPARAM;
   }

#ifdef GPR_DEBUG_MSG
   AR_MSG(DBG_HIGH_PRIO,
          "GPR finished sending message over datalink layer with opcode %lx, dst port %lx src port %lx token %lx",
          packet->opcode,
          packet->dst_port,
          packet->src_port,
          packet->token);
#endif

   __gpr_cmd_free(packet);
   return AR_EOK;
}

// Utility to create packet pool info arrays, called at the time of init
static uint32_t gpr_drv_util_create_packet_pool_info_arrs(uint32_t                  num_packet_pools,
                                                          gpr_packet_pool_info_v2_t packet_pool_info[])
{
   // find num static and dynamic pools and allocate packet pool info structure arrays.
   // Also, find out the heap index from which array pointer should be allocated.
   uint32_t         num_static_packet_pools = 0;
   uint32_t         num_dyn_packet_pools    = 0;
   gpr_heap_index_t gpr_heap_index          = GPR_HEAP_INDEX_DEFAULT;
   for (uint32_t idx = 0; idx < num_packet_pools; idx++)
   {
      if (packet_pool_info[idx].is_dynamic)
      {
         num_dyn_packet_pools++;
      }
      else
      {
         num_static_packet_pools++;
      }

      // check validity of heap index
      if (packet_pool_info[idx].heap_index > GPR_HEAP_INDEX_1)
      {
         return AR_EFAILED;
      }

      // if atleast one heap is from index 1, then use index 1 for allocating the array
      if (GPR_HEAP_INDEX_1 == packet_pool_info[idx].heap_index)
      {
         gpr_heap_index = GPR_HEAP_INDEX_1;
      }
   }

   // cache heap index
   gpr_ctxt_struct_t.drv_heap_index = gpr_heap_index;

   // assign appropriate ar heap id
   ar_heap_info heap_info;
   ar_mem_set((void *)&heap_info, 0, sizeof(ar_heap_info));
   gpr_populate_ar_heap_info(gpr_ctxt_struct_t.drv_heap_index, AR_HEAP_ALIGN_DEFAULT, &heap_info);

   // malloc static pool array if needed
   if (num_static_packet_pools)
   {
      gpr_ctxt_struct_t.static_pool_arr = (gpr_drv_pkt_static_pool_info_t *)
         ar_heap_malloc(num_static_packet_pools * sizeof(gpr_drv_pkt_static_pool_info_t), &heap_info);
      if (NULL == gpr_ctxt_struct_t.static_pool_arr)
      {
         return AR_EFAILED;
      }
      ar_mem_set((void *)gpr_ctxt_struct_t.static_pool_arr,
                 0,
                 num_static_packet_pools * sizeof(gpr_drv_pkt_static_pool_info_t));
   }

   // malloc dynamic pool array if needed
   if (num_dyn_packet_pools)
   {
      gpr_ctxt_struct_t.dyn_pool_arr = (gpr_drv_pkt_dynamic_pool_info_t *)
         ar_heap_malloc(num_dyn_packet_pools * sizeof(gpr_drv_pkt_dynamic_pool_info_t), &heap_info);
      if (NULL == gpr_ctxt_struct_t.dyn_pool_arr)
      {
         return AR_EFAILED;
      }
      ar_mem_set((void *)gpr_ctxt_struct_t.dyn_pool_arr,
                 0,
                 num_dyn_packet_pools * sizeof(gpr_drv_pkt_dynamic_pool_info_t));
   }

   /* Allocate Default heap CB lists */
   gpr_populate_ar_heap_info(GPR_HEAP_INDEX_DEFAULT, AR_HEAP_ALIGN_DEFAULT, &heap_info);

   uint32_t default_list_size     = sizeof(gpr_module_node_list_t) + (SESSION_ARRAY_SIZE * sizeof(gpr_module_node_t *));
   gpr_ctxt_struct_t.default_list = (gpr_module_node_list_t *)ar_heap_malloc(default_list_size, &heap_info);
   if (NULL == gpr_ctxt_struct_t.default_list)
   {
      return AR_EFAILED;
   }
   ar_mem_set((void *)gpr_ctxt_struct_t.default_list, 0, default_list_size);
   gpr_ctxt_struct_t.default_list->max_cb_list_size = SESSION_ARRAY_SIZE;
   gpr_ctxt_struct_t.default_list->heap_index       = GPR_HEAP_INDEX_DEFAULT;

   /* Allocate Non-Default heap CB list if required */
   gpr_populate_ar_heap_info(GPR_HEAP_INDEX_1, AR_HEAP_ALIGN_DEFAULT, &heap_info);

   uint32_t list_size = sizeof(gpr_module_node_list_t) + (HEAP1_SESSION_ARRAY_SIZE * sizeof(gpr_module_node_t *));
   gpr_ctxt_struct_t.heap1_list = (gpr_module_node_list_t *)ar_heap_malloc(list_size, &heap_info);
   if (NULL == gpr_ctxt_struct_t.heap1_list)
   {
      return AR_EFAILED;
   }
   ar_mem_set((void *)gpr_ctxt_struct_t.heap1_list, 0, list_size);
   gpr_ctxt_struct_t.heap1_list->max_cb_list_size = HEAP1_SESSION_ARRAY_SIZE;
   gpr_ctxt_struct_t.heap1_list->heap_index       = GPR_HEAP_INDEX_1;

   return AR_EOK;
}

#ifndef DISABLE_DEINIT
// Utility to destory the packet pool info arrays, called at the time of de-init
static uint32_t gpr_drv_util_free_packet_pool_info_arrs(void)
{
   // Get the heap index in which the structures have been allocated
   gpr_heap_index_t gpr_heap_index = gpr_ctxt_struct_t.drv_heap_index;

   ar_heap_info heap_info;
   ar_mem_set((void *)&heap_info, 0, sizeof(ar_heap_info));
   gpr_populate_ar_heap_info(gpr_heap_index, AR_HEAP_ALIGN_DEFAULT, &heap_info);

   // free the pointers from corresponding heap.
   if (gpr_ctxt_struct_t.static_pool_arr)
   {
      ar_heap_free((void *)gpr_ctxt_struct_t.static_pool_arr, &heap_info);
   }

   if (gpr_ctxt_struct_t.dyn_pool_arr)
   {
      ar_heap_free((void *)gpr_ctxt_struct_t.dyn_pool_arr, &heap_info);
   }

   if (gpr_ctxt_struct_t.default_list)
   {
      gpr_populate_ar_heap_info(AR_HEAP_ID_DEFAULT, AR_HEAP_ALIGN_DEFAULT, &heap_info);
      ar_heap_free((void *)gpr_ctxt_struct_t.default_list, &heap_info);
   }

   if (gpr_ctxt_struct_t.heap1_list)
   {
      gpr_populate_ar_heap_info(AR_HEAP_ID_1, AR_HEAP_ALIGN_DEFAULT, &heap_info);
      ar_heap_free((void *)gpr_ctxt_struct_t.heap1_list, &heap_info);
   }

   return AR_EOK;
}
#endif
/*@brief Creating memory for GPR packets and initializing all datalink layers

  @return
  #AR_EOK when successful.
*/

GPR_EXTERNAL uint32_t gpr_drv_internal_init_v2(uint32_t                  default_domain_id,
                                               uint32_t                  num_domains,
                                               struct ipc_dl_v2_t        gpr_ipc_dl_v2_table[],
                                               uint32_t                  num_packet_pools,
                                               gpr_packet_pool_info_v2_t packet_pool_info[])
{
   uint32_t rc;
   uint32_t port_index = 0;
   uint32_t domain_id  = 0;
   memset(&gpr_ctxt_struct_t, 0, sizeof(gpr_ctxt_struct_t));
   gpr_deinit_flag = FALSE;

   // DSP gpr supports upto 3 packet pools.
   if (num_packet_pools > MAX_GPR_PKT_POOLS)
   {
      goto bailout;
   }

   /* Create gpr mutexes */
   rc = ar_osal_mutex_create((ar_osal_mutex_t *)&gpr_ctxt_struct_t.gpr_drv_task_lock);
   if (rc)
   {
      goto bailout;
   }
   rc = ar_osal_mutex_create((ar_osal_mutex_t *)&gpr_ctxt_struct_t.gpr_drv_isr_lock);
   if (rc)
   {
      goto bailout;
   }

   // allocate packet pool info arrays.
   rc = gpr_drv_util_create_packet_pool_info_arrs(num_packet_pools, packet_pool_info);
   if (rc)
   {
      goto bailout;
   }

   // allocate packets for static pools and cache packet size info for dynamic allocation.
   for (uint32_t idx = 0; idx < num_packet_pools; idx++)
   {
      if (packet_pool_info[idx].is_dynamic)
      {
         // if there is dynamic pool in the input args, the arr must have been allocated in fn
         // gpr_drv_util_create_packet_pool_info_arrs
         if (NULL == gpr_ctxt_struct_t.dyn_pool_arr)
         {
            goto bailout;
         }

         gpr_ctxt_struct_t.num_dyn_packet_pools++;
         uint32_t new_pool_index = gpr_ctxt_struct_t.num_dyn_packet_pools - 1;

         gpr_ctxt_struct_t.dyn_pool_arr[new_pool_index].max_num_packets  = packet_pool_info[idx].num_packets;
         gpr_ctxt_struct_t.dyn_pool_arr[new_pool_index].buf_size         = packet_pool_info[idx].packet_size;
         gpr_ctxt_struct_t.dyn_pool_arr[new_pool_index].curr_num_packets = 0;
         gpr_ctxt_struct_t.dyn_pool_arr[new_pool_index].heap_index       = packet_pool_info[idx].heap_index;
      }
      else
      {

         // if there is static pool in the input args, the arr must have been allocated in fn
         // gpr_drv_util_create_packet_pool_info_arrs
         if (NULL == gpr_ctxt_struct_t.static_pool_arr)
         {
            goto bailout;
         }

         gpr_ctxt_struct_t.num_static_packet_pools++;
         uint32_t new_pool_index = gpr_ctxt_struct_t.num_static_packet_pools - 1;

         /* static packets info*/
         uint32_t buf_size    = packet_pool_info[idx].packet_size;
         uint32_t num_packets = packet_pool_info[idx].num_packets;

         ar_heap_info heap_info;
         ar_mem_set((void *)&heap_info, 0, sizeof(ar_heap_info));
         gpr_populate_ar_heap_info(packet_pool_info[idx].heap_index, AR_HEAP_ALIGN_8_BYTES, &heap_info);

         gpr_ctxt_struct_t.static_pool_arr[new_pool_index].buf_size    = buf_size;
         gpr_ctxt_struct_t.static_pool_arr[new_pool_index].num_packets = num_packets;
         gpr_ctxt_struct_t.static_pool_arr[new_pool_index].heap_index  = packet_pool_info[idx].heap_index;

         uint32_t gpr_memq_size_per_packet =
            GPR_MEMQ_UNIT_OVERHEAD_V + buf_size + (GPR_DRV_METADATA_ITEMS_V * GPR_MEMQ_BYTES_PER_METADATA_ITEM_V);

         uint32_t gpr_memq_size = (num_packets * gpr_memq_size_per_packet);

         /* Allocate memory for GPR packets of different sizes */
         gpr_ctxt_struct_t.static_pool_arr[new_pool_index].packet_heap =
            (char *)ar_heap_malloc((gpr_memq_size * sizeof(char)), &heap_info);
         if (NULL == gpr_ctxt_struct_t.static_pool_arr[new_pool_index].packet_heap)
         {
            goto bailout;
         }

         gpr_ctxt_struct_t.static_pool_arr[new_pool_index].packet_heap_end =
            (gpr_ctxt_struct_t.static_pool_arr[new_pool_index].packet_heap) + (gpr_memq_size - 1);

         gpr_ctxt_struct_t.static_pool_arr[new_pool_index].free_packets_memq =
            (gpr_memq_block_t *)ar_heap_malloc(sizeof(gpr_memq_block_t), &heap_info);
         if (NULL == gpr_ctxt_struct_t.static_pool_arr[new_pool_index].free_packets_memq)
         {
            goto bailout;
         }

         rc = gpr_memq_init(gpr_ctxt_struct_t.static_pool_arr[new_pool_index].free_packets_memq,
                            gpr_ctxt_struct_t.static_pool_arr[new_pool_index].packet_heap,
                            (gpr_memq_size * sizeof(char)),
                            gpr_memq_size_per_packet,
                            GPR_DRV_METADATA_ITEMS_V,
                            gpr_drv_isr_lock_fn,
                            gpr_drv_isr_unlock_fn,
                            gpr_ctxt_struct_t.static_pool_arr[new_pool_index].heap_index);
         if (rc)
         {
            goto bailout;
         }
      }
   }

   /* Save default domain id and pass it through init*/
   gpr_ctxt_struct_t.default_domain_id = default_domain_id;

   /* Check for max number of domains */
   if (GPR_PL_NUM_TOTAL_DOMAINS_V < num_domains)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Init failed for number of domain ids %lu > max supported %lu",
             num_domains,
             (GPR_PL_NUM_TOTAL_DOMAINS_V));
      goto bailout;
   }

   /* Copy contents of user defined table to local table */
   gpr_ctxt_struct_t.num_user_defined_domains = num_domains;
   for (port_index = 0; port_index < gpr_ctxt_struct_t.num_user_defined_domains; port_index++)
   {
      domain_id                                             = gpr_ipc_dl_v2_table[port_index].domain_id;
      local_gpr_ipc_dl_table[domain_id].domain_id           = gpr_ipc_dl_v2_table[port_index].domain_id;
      local_gpr_ipc_dl_table[domain_id].init_fn             = gpr_ipc_dl_v2_table[port_index].init_fn;
      local_gpr_ipc_dl_table[domain_id].deinit_fn           = gpr_ipc_dl_v2_table[port_index].deinit_fn;
      local_gpr_ipc_dl_table[domain_id].supports_shared_mem = gpr_ipc_dl_v2_table[port_index].supports_shared_mem;
   }

   /* Populate GPR CB functions */
   gpr_to_ipc_vtbl_t gpr_to_ipc_tbl;
   gpr_to_ipc_tbl.receive   = &gpr_ipc_receive;
   gpr_to_ipc_tbl.send_done = &gpr_ipc_send_done;

   /* Call init for all user defined dl's */
   for (domain_id = 0; domain_id < GPR_PL_NUM_TOTAL_DOMAINS_V; domain_id++)
   {
      if (NULL != (local_gpr_ipc_dl_table[domain_id].init_fn))
      {
         rc = local_gpr_ipc_dl_table[domain_id].init_fn(default_domain_id,
                                                        domain_id,
                                                        &gpr_to_ipc_tbl,
                                                        &local_gpr_ipc_dl_table[domain_id].fn_ptr);
         if (rc)
         {
            AR_MSG(DBG_ERROR_PRIO, "Init failed for domain id %lu", domain_id);
            goto bailout;
         }
      }
   }

   /* Initialize list of gpr packets allocated dynamically */
   if (gpr_dynamic_packet_init())
   {
      AR_MSG(DBG_ERROR_PRIO, "Dynamic packet initialization failed");
      goto bailout;
   }

   return AR_EOK;

bailout:
#ifndef DISABLE_DEINIT
   rc = gpr_drv_deinit();
#endif
   return AR_EFAILED;
}

/*@brief Creating memory for GPR packets and initializing all datalink layers

  @return
  #AR_EOK when successful.
*/
#define MAX_PKT_POOLS_V1 2
GPR_INTERNAL uint32_t gpr_drv_internal_init(struct ipc_dl_t gpr_ipc_dl_table[],
                                            uint32_t        num_domains,
                                            uint32_t        default_domain_id,
                                            uint32_t        num_packets_1,
                                            uint32_t        buf_size_1,
                                            uint32_t        num_packets_2,
                                            uint32_t        buf_size_2)
{
   gpr_packet_pool_info_v2_t packet_pool_info[MAX_PKT_POOLS_V1];
   packet_pool_info[0].heap_index  = GPR_HEAP_INDEX_DEFAULT;
   packet_pool_info[0].num_packets = num_packets_1;
   packet_pool_info[0].packet_size = buf_size_1;
   packet_pool_info[0].is_dynamic  = FALSE;
   packet_pool_info[0].reserved    = 0;

   packet_pool_info[1].heap_index  = GPR_HEAP_INDEX_DEFAULT;
   packet_pool_info[1].num_packets = num_packets_2;
   packet_pool_info[1].packet_size = buf_size_2;
   packet_pool_info[1].is_dynamic  = FALSE;
   packet_pool_info[1].reserved    = 0;

   struct ipc_dl_v2_t gpr_ipc_dl_v2_table[GPR_PL_NUM_TOTAL_DOMAINS_V];
   memset(&gpr_ipc_dl_v2_table[0], 0, (sizeof(struct ipc_dl_v2_t) * GPR_PL_NUM_TOTAL_DOMAINS_V));

   /* Check for max number of domains */
   if (GPR_PL_NUM_TOTAL_DOMAINS_V < num_domains)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Init failed for number of domain ids %lu > max supported %lu",
             num_domains,
             (GPR_PL_NUM_TOTAL_DOMAINS_V));
      return AR_EBADPARAM;
   }

   for (uint32_t idx = 0; idx < num_domains; idx++)
   {
      gpr_ipc_dl_v2_table[idx].domain_id           = gpr_ipc_dl_table[idx].domain_id;
      gpr_ipc_dl_v2_table[idx].init_fn             = gpr_ipc_dl_table[idx].init_fn;
      gpr_ipc_dl_v2_table[idx].deinit_fn           = gpr_ipc_dl_table[idx].deinit_fn;
      gpr_ipc_dl_v2_table[idx].supports_shared_mem = TRUE;
   }

   return gpr_drv_internal_init_v2(default_domain_id,
                                   num_domains,
                                   gpr_ipc_dl_v2_table,
                                   MAX_PKT_POOLS_V1,
                                   packet_pool_info);
}

/*@brief Freeing memory for GPR packets and de-initializing all datalink layers

  @return
  #AR_EOK when successful.
*/
#ifndef DISABLE_DEINIT

GPR_INTERNAL uint32_t gpr_drv_deinit(void)
{
   uint32_t domain_id;
   uint32_t rc = AR_EOK;

   if (gpr_deinit_flag)
   {
      return AR_EOK;
   }

   /* Calling ipc dl de-init for user defined domains */
   for (domain_id = 0; domain_id < GPR_PL_NUM_TOTAL_DOMAINS_V; domain_id++)
   {
      if (NULL != (local_gpr_ipc_dl_table[domain_id].deinit_fn))
      {
         rc = local_gpr_ipc_dl_table[domain_id].deinit_fn(gpr_ctxt_struct_t.default_domain_id, domain_id);
         if (rc)
         {
            AR_MSG(DBG_ERROR_PRIO, "de-init failed for domain id %lu", domain_id);
         }
      }
   }

   // Free static packet pool heaps
   for (uint32_t idx = 0; idx < gpr_ctxt_struct_t.num_static_packet_pools; idx++)
   {
      /* Freeing linked list nodes*/
      if (NULL != gpr_ctxt_struct_t.static_pool_arr[idx].free_packets_memq)
      {
         gpr_memq_deinit(gpr_ctxt_struct_t.static_pool_arr[idx].free_packets_memq,
                         gpr_ctxt_struct_t.static_pool_arr[idx].heap_index);
      }

      /*Freeing packet memory*/
      ar_heap_info heap_info;
      ar_mem_set((void *)&heap_info, 0, sizeof(ar_heap_info));
      gpr_populate_ar_heap_info(gpr_ctxt_struct_t.static_pool_arr[idx].heap_index, AR_HEAP_ALIGN_8_BYTES, &heap_info);

      if (NULL != gpr_ctxt_struct_t.static_pool_arr[idx].free_packets_memq)
      {
         ar_heap_free((void *)gpr_ctxt_struct_t.static_pool_arr[idx].free_packets_memq, &heap_info);
      }

      if (NULL != gpr_ctxt_struct_t.static_pool_arr[idx].packet_heap)
      {
         ar_heap_free((void *)gpr_ctxt_struct_t.static_pool_arr[idx].packet_heap, &heap_info);
      }
   }

   gpr_dynamic_packet_deinit();

   // free packet pool info arrays
   gpr_drv_util_free_packet_pool_info_arrs();

   /* Free mutexes*/
   if (NULL != gpr_ctxt_struct_t.gpr_drv_isr_lock)
   {
      (void)ar_osal_mutex_destroy(gpr_ctxt_struct_t.gpr_drv_isr_lock);
      gpr_ctxt_struct_t.gpr_drv_isr_lock = NULL;
   }
   if (NULL != gpr_ctxt_struct_t.gpr_drv_task_lock)
   {
      (void)ar_osal_mutex_destroy(gpr_ctxt_struct_t.gpr_drv_task_lock);
      gpr_ctxt_struct_t.gpr_drv_task_lock = NULL;
   }

   // memset the global gpr driver structure memory
   memset(&gpr_ctxt_struct_t, 0, sizeof(gpr_ctxt_struct_t));
   gpr_deinit_flag = TRUE;

   return AR_EOK;
}

#endif
/*****************************************************************************
 * Local IPC Implementation                                       *
 ****************************************************************************/
/*@brief Local datalink init
  @param[in] domain_id          Domain id of destination
  @param[in] gpr_to_ipc_vtbl_t  Pointer to function table sent from GPR to IPC datalink layers
  @param[in] ipc_to_gpr_vtbl_t  Double pointer to function table to be populated by IPC datalink layer

  @return
  #AR_EOK when successful.
*/
GPR_INTERNAL uint32_t ipc_dl_local_init(uint32_t                                   src_domain_id,
                                        uint32_t                                   dest_domain_id,
                                        const gpr_to_ipc_vtbl_t *p_gpr_to_ipc_vtbl __UNUSED,
                                        ipc_to_gpr_vtbl_t                        **pp_ipc_to_gpr_vtbl)
{

   __UNREFERENCED_PARAM(p_gpr_to_ipc_vtbl);
   AR_MSG(DBG_LOW_PRIO,
          "GPR Local init called with same src domain id %d, dest domain id %d",
          src_domain_id,
          dest_domain_id);
   *pp_ipc_to_gpr_vtbl = &local_vtbl;

   return AR_EOK;
}

/*@brief Local datalink de-init
  @param[in] domain_id  Domain id

  @return
  #AR_EOK when successful.
*/
GPR_INTERNAL uint32_t ipc_dl_local_deinit(uint32_t src_domain_id, uint32_t dest_domain_id)
{
   AR_MSG(DBG_LOW_PRIO,
          "GPR Local de-init called with same src domain id %d, dest domain id %d",
          src_domain_id,
          dest_domain_id);
   return AR_EOK;
}

/*****************************************************************************
 * Fundamental Controls Implementation                                       *
 ****************************************************************************/
/**
  @brief Registers a module, by domain id and module port/address, to the GPR service.
  Module must pass its heap index.

  @param[in] src_port        Address of registering module
  @param[in] rcallback_fn    callback function to register.
  @param[in] rcallback_data  Client-supplied data pointer for the callback
                             function.
  @param[in] heap_index      Heap index

  @detdesc
  This function is called by modules once during system bring-up or during
  runtime to register its presence to the GPR service. The client and server
  services must be registered with the GPR service before any messages can
  be passed between them.
  @return
  #AR_EOK when successful.
*/
uint32_t __gpr_cmd_register_v2(uint32_t          src_port,
                               gpr_callback_fn_t callback_fn,
                               void             *callback_data,
                               gpr_heap_index_t  heap_index)
{
   uint32_t            rc      = 0;
   gpr_module_entry_t *session = NULL;

   if (NULL == callback_fn)
   {
      return AR_EBADPARAM;
   }

#ifdef GPR_DEBUG_MSG
   AR_MSG(DBG_HIGH_PRIO, "GPR Register: Source Port is %lu", src_port);
#endif

   AR_MSG(DBG_HIGH_PRIO, "GPR Register: Source Port is %lu heap_index: %lu", src_port, heap_index);

   gpr_module_node_list_t *list_handle = gpr_ctxt_struct_t.default_list;
   if (GPR_HEAP_INDEX_1 == heap_index)
   {
      list_handle = gpr_ctxt_struct_t.heap1_list;
   }

   ar_osal_mutex_lock(gpr_ctxt_struct_t.gpr_drv_task_lock);

   // saves src port in cb_list and returns pointer to session
   rc = gpr_init_session(src_port, &session, list_handle);

   /* Save cb function and argument if any */
   if (session)
   {
      session->callback_fn = callback_fn;
      if (callback_data)
      {
         session->callback_data = callback_data;
      }
   }

   ar_osal_mutex_unlock(gpr_ctxt_struct_t.gpr_drv_task_lock);
   return rc;
}

/**
  @brief Registers a module, by domain id and module port/address, to the GPR service.

  @param[in] src_port        Address of registering module
  @param[in] rcallback_fn    callback function to register.
  @param[in] rcallback_data  Client-supplied data pointer for the callback
                             function.
  @detdesc
  This function is called by modules once during system bring-up or during
  runtime to register its presence to the GPR service. The client and server
  services must be registered with the GPR service before any messages can
  be passed between them.
  @return
  #AR_EOK when successful.
*/
uint32_t __gpr_cmd_register(uint32_t src_port, gpr_callback_fn_t callback_fn, void *callback_data)
{
   return __gpr_cmd_register_v2(src_port, callback_fn, callback_data, GPR_HEAP_INDEX_DEFAULT);
}

/**
  @brief De-registers a module from the GPR service.

  @param[in] src_port      src_port
  @detdesc
  This function is called by modules once during system teardown or during
  runtime to deregister its presence from the GPR service.

  @return
  #AR_EOK when successful.
*/
uint32_t __gpr_cmd_deregister_v2(uint32_t src_port, gpr_heap_index_t heap_index)
{
   uint32_t rc = AR_EOK;

   gpr_module_node_list_t *list_handle = gpr_ctxt_struct_t.default_list;
   if (GPR_HEAP_INDEX_1 == heap_index)
   {
      list_handle = gpr_ctxt_struct_t.heap1_list;
   }

   ar_osal_mutex_lock(gpr_ctxt_struct_t.gpr_drv_task_lock);

   // free memory for node corresponding to the port
   (void)gpr_deinit_session(src_port, list_handle);

   ar_osal_mutex_unlock(gpr_ctxt_struct_t.gpr_drv_task_lock);

   return rc;
}

/**
  @brief De-registers a module from the GPR service.

  @param[in] src_port      src_port
  @detdesc
  This function is called by modules once during system teardown or during
  runtime to deregister its presence from the GPR service.

  @return
  #AR_EOK when successful.
*/
uint32_t __gpr_cmd_deregister(uint32_t src_port)
{
   return __gpr_cmd_deregister_v2(src_port, GPR_HEAP_INDEX_DEFAULT);
}

/**
  @brief Checks if a module is registered to the GPR service.

  @param[in]  port         port
  @param[out] bool_t       is_registered
  @detdesc
  This function is called by framework to check if module is still registered
  to the GPR service.

  @return
  #AR_EOK always with result in is_registered flag
*/
uint32_t __gpr_cmd_is_registered(uint32_t port, bool_t *is_registered)
{
   gpr_module_entry_t *session = NULL;
   uint32_t            rc      = AR_EOK;

   if (NULL == is_registered)
   {
      return AR_EBADPARAM;
   }

   ar_osal_mutex_lock(gpr_ctxt_struct_t.gpr_drv_task_lock);

   rc = gpr_get_session_util(port, &session);

   ar_osal_mutex_unlock(gpr_ctxt_struct_t.gpr_drv_task_lock);

   if (NULL == session)
   {
      *is_registered = FALSE;
   }
   else
   {
      *is_registered = TRUE;
   }

   return rc;
}

uint32_t __gpr_cmd_is_shared_mem_supported(uint32_t domain_id, bool_t *supports_shared_mem)
{
   if (NULL == supports_shared_mem)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Flag passed is NULL OR Domain ID out of bounds %lu, Max ID supported %lu",
             domain_id,
             GPR_PL_MAX_DOMAIN_ID_V);
      return AR_EBADPARAM;
   }

   *supports_shared_mem = local_gpr_ipc_dl_table[domain_id].supports_shared_mem;

   return AR_EOK;
}

uint32_t __gpr_cmd_get_dest_domain_ids(uint32_t *num_domains, uint32_t *domain_ids)
{
   uint32_t domain_index = 0;

   if (NULL == num_domains)
   {
      return AR_EBADPARAM;
   }
   else if (*num_domains == 0)
   {
      *num_domains = gpr_ctxt_struct_t.num_user_defined_domains - 1;
      return AR_EOK;
   }
   else
   {
      if (domain_ids != NULL)
      {
         for (domain_index = 0; domain_index < *num_domains; domain_index++)
         {
            if (local_gpr_ipc_dl_table[domain_index].domain_id != gpr_ctxt_struct_t.default_domain_id &&
                local_gpr_ipc_dl_table[domain_index].domain_id != GPR_IDS_DOMAIN_ID_INVALID_V)
            {
               domain_ids[domain_index] = local_gpr_ipc_dl_table[domain_index].domain_id;
            }
         }
         return AR_EOK;
      }
      else
      {
         return AR_EBADPARAM;
      }
   }
}

/**
  @brief Retrieves gpr packet info.

  @detdesc
  This function is called to query the gpr packet pool information
  which returns the minimum and maximum size(in bytes) of packets
  allocated by gpr as well as maximum number of each such packets.

  This is a V1 function so dynamic pools are not supported here. Hence we iterate
  through only static pools to get the max and min sizes.

  If the client is using V2 api to allocate pools i.e gpr_drv_internal_init_v2(),
  then this function should not be used, client must use __gpr_cmd_get_gpr_packet_info_v2() only.

  This info is sent to gpr through platform wrappers at the time of init
  @return
  #AR_EOK when successful.
*/
uint32_t __gpr_cmd_get_gpr_packet_info(gpr_cmd_gpr_packet_pool_info_t *args)
{
   if (NULL == args)
   {
      return AR_EBADPARAM;
   }

   uint32_t min_buf_size = 0xFFFFFFFF;
   uint32_t max_buf_size = 0;
   for (uint32_t idx = 0; idx < gpr_ctxt_struct_t.num_static_packet_pools; idx++)
   {
      // Consider only non-island heap index for V1 get packet pool info function
      if (GPR_HEAP_INDEX_DEFAULT != gpr_ctxt_struct_t.static_pool_arr[idx].heap_index)
      {
         continue;
      }

      // track max
      if (gpr_ctxt_struct_t.static_pool_arr[idx].buf_size > max_buf_size)
      {
         args->num_max_size_packets = gpr_ctxt_struct_t.static_pool_arr[idx].num_packets;

         args->bytes_per_max_size_packet = gpr_ctxt_struct_t.static_pool_arr[idx].buf_size;
         max_buf_size                    = gpr_ctxt_struct_t.static_pool_arr[idx].buf_size;
      }

      // track min
      if (gpr_ctxt_struct_t.static_pool_arr[idx].buf_size < min_buf_size)
      {
         args->num_min_size_packets = gpr_ctxt_struct_t.static_pool_arr[idx].num_packets;

         args->bytes_per_min_size_packet = gpr_ctxt_struct_t.static_pool_arr[idx].buf_size;
         min_buf_size                    = gpr_ctxt_struct_t.static_pool_arr[idx].buf_size;
      }
   }

#ifdef GPR_DEBUG_MSG
   AR_MSG(DBG_HIGH_PRIO,
          "GPR: Min packet pool (size,num_pkts)= (%lu,%lu) Max packet pool (size,num_pkts)= (%lu,%lu)",
          args->bytes_per_min_size_packet,
          args->num_min_size_packets,
          args->bytes_per_max_size_packet,
          args->num_max_size_packets);
#endif

   return AR_EOK;
}

uint32_t __gpr_cmd_get_gpr_packet_info_v2(uint32_t *num_packet_pools, gpr_packet_pool_info_v2_t *packet_pool_info_arr)
{
   if (num_packet_pools)
   {
      *num_packet_pools = gpr_ctxt_struct_t.num_static_packet_pools + gpr_ctxt_struct_t.num_dyn_packet_pools;
   }

   if (packet_pool_info_arr)
   {
      uint32_t idx = 0;
      // populate static pool info
      for (idx = 0; idx < gpr_ctxt_struct_t.num_static_packet_pools; idx++)
      {
         packet_pool_info_arr[idx].is_dynamic  = FALSE;
         packet_pool_info_arr[idx].heap_index  = gpr_ctxt_struct_t.static_pool_arr[idx].heap_index;
         packet_pool_info_arr[idx].num_packets = gpr_ctxt_struct_t.static_pool_arr[idx].num_packets;
         packet_pool_info_arr[idx].packet_size = gpr_ctxt_struct_t.static_pool_arr[idx].buf_size;
         packet_pool_info_arr[idx].reserved    = 0;
      }

      // populate dynamic pool info
      for (; idx < gpr_ctxt_struct_t.num_dyn_packet_pools; idx++)
      {
         packet_pool_info_arr[idx].is_dynamic  = TRUE;
         packet_pool_info_arr[idx].heap_index  = gpr_ctxt_struct_t.dyn_pool_arr[idx].heap_index;
         packet_pool_info_arr[idx].num_packets = gpr_ctxt_struct_t.dyn_pool_arr[idx].max_num_packets;
         packet_pool_info_arr[idx].packet_size = gpr_ctxt_struct_t.dyn_pool_arr[idx].buf_size;
         packet_pool_info_arr[idx].reserved    = 0;
      }
   }

   return AR_EOK;
}
//end of file