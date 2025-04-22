#ifndef __GPR_DRV_I_H__
#define __GPR_DRV_I_H__
/**
 * \file gpr_drv_i.h
 * \brief
 *  	This file contains GPR driver internal API header.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/******************************************************************************
 * Includes                                                                    *
 *****************************************************************************/
#include "gpr_proc_info_api.h"
#include "ar_osal_mem_op.h"
#include "ar_osal_mutex.h"
#include "ar_osal_heap.h"
#include "ar_guids.h"
#include "gpr_api_i.h"
#include "gpr_msg_if.h"
#include "gpr_memq.h"
#include "gpr_api_inline.h"
#include "gpr_log.h"
#include "gpr_session.h"
#include "ar_osal_servreg.h"
#include "ar_osal_string.h"
#include "gpr_dynamic_allocation.h"

#if defined(WIN64) || defined(WIN32)
#include <windows.h>
#else
#include "gpr_private_api.h"
#endif

/*****************************************************************************
 * Structure definitions                                                     *
 ****************************************************************************/

/* Structure for Local GPR IPC table containing init,deinit functions
 * and vtbl function pointer to be populated for each datalink during init*/
struct ipc_dl_local_t
{
   uint16_t           domain_id; // store domain id for readability
   ipc_dl_init_fn_t   init_fn;   // store the call back function here
   ipc_dl_deinit_fn_t deinit_fn;
   ipc_to_gpr_vtbl_t *fn_ptr;
   bool_t             supports_shared_mem;
};

/*****************************************************************************
 * Defines                                                                   *
 ****************************************************************************/
//#define GPR_DEBUG_MSG 1
#define GPR_DRV_METADATA_ITEMS_V (1)

/*****************************************************************************
 * Global variables                                                          *
 ****************************************************************************/
/* GPR context structure contains all GPR globals */

// GPR driver internal MAX number of pools limit.
#define MAX_GPR_PKT_POOLS 128

// Info related to each of the static packet pool
typedef struct gpr_drv_pkt_static_pool_info_t
{
   char             *packet_heap;
   char             *packet_heap_end;
   gpr_memq_block_t *free_packets_memq;
   uint32_t          buf_size;
   uint32_t          num_packets;
   gpr_heap_index_t  heap_index;
} gpr_drv_pkt_static_pool_info_t;

/* Info related to each of the dynamic packet pool, currently only one dynamic pool is supported.*/
typedef struct gpr_drv_pkt_dynamic_pool_info_t
{
   uint32_t         buf_size;
   uint32_t         max_num_packets;
   uint32_t         curr_num_packets;
   gpr_heap_index_t heap_index;
} gpr_drv_pkt_dynamic_pool_info_t;

typedef struct gpr_ctxt_struct_t
{
   ar_osal_mutex_t         gpr_drv_isr_lock;
   ar_osal_mutex_t         gpr_drv_task_lock;
   gpr_module_node_list_t *default_list; /* handle to manage registered module nodes for default heap  */
   gpr_module_node_list_t *heap1_list;   /* handle to manage registered module nodes for GPR heap index 1 */

   uint32_t num_user_defined_domains;
   uint32_t default_domain_id;

   /** Driver heap index is the index used for allocating,
    *   1. globla structure arrays static_pool_arr and dyn_pool_arr
    *   2. Creating session entry memory.
    *
    * This will allow accessing the client nodes and packet pool info structures in low power heaps.
    *
    * Note that this index is not used for allocating the packets. Packets are allocated
    * based on per pool heap index.
    * */
   gpr_heap_index_t drv_heap_index;

   /* Packets in each of static pool are allocated statically at init time.
     The structure contains statically allocated num_packets, each packets size, packet heap info. */
   uint32_t                        num_static_packet_pools;
   gpr_drv_pkt_static_pool_info_t *static_pool_arr;

   /* Packets in the dynamic pool are malloced when packet_alloc() is called. the info struct contains
       max packets that can be dynamically allocated, current num of malloced packets and each packets size. */
   uint32_t                         num_dyn_packet_pools;
   gpr_drv_pkt_dynamic_pool_info_t *dyn_pool_arr;
} gpr_ctxt_struct_type;

/*****************************************************************************
 * function definitions                                                *
 ****************************************************************************/

GPR_INTERNAL uint32_t gpr_local_send(uint32_t domain_id, void *buf, uint32_t length);

GPR_INTERNAL uint32_t gpr_local_receive_done(uint32_t domain_id, void *buf);

void gpr_drv_isr_lock_fn(void);

void gpr_drv_isr_unlock_fn(void);

GPR_INTERNAL uint32_t gpr_get_session_util(uint32_t my_module_port, gpr_module_entry_t **ret_entry);

#endif /* __GPR_DRV_I_H__ */
