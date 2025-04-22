/**
 * \file apm_close_all_utils.c
 *
 * \brief
 *
 *     This file contains APM_CMD_CLOSE_ALL processing utilities.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_error_codes.h"
#include "apm_debug_info_dump.h"
#include "apm_internal.h"

extern apm_t g_apm_info;

#define ALIGN8(o)         (((o)+7)&(~7))

/**==============================================================================
   Function Declaration
==============================================================================*/

void apm_cntr_dump_debug_info(void *callback_data,int8_t *start_address,uint32_t max_size)
{

   apm_t *           apm_info_ptr   = &g_apm_info;

   if(start_address==NULL||max_size==0)
   {
       return;
   }

   uint32_t          total_size     = max_size;
   apm_graph_info_t *graph_info_ptr = &apm_info_ptr->graph_info;
   apm_cont_graph_t *cont_graph_ptr;
   apm_container_t * container_node_ptr;
   spf_list_node_t * curr_graph_node_ptr;
   uint32_t          cntr_type;
   uint32_t *container_start_address;
   uint32_t container_index=0;
   curr_graph_node_ptr = graph_info_ptr->cont_graph_list_ptr;
   spf_handle_t *cont_hdl_ptr;


   int8_t * aligned_start_address = start_address;
   if(total_size < sizeof(apm_debug_info_t))
   {
      return;
   }

   apm_debug_info_t *apm_debug_info =(apm_debug_info_t *)aligned_start_address;
   apm_debug_info->apm_container_debug_size=total_size;
   apm_debug_info->channel_status=apm_info_ptr->channel_status;
   apm_debug_info->curr_wait_mask=apm_info_ptr->curr_wait_mask;

   if(apm_info_ptr->curr_cmd_ctrl_ptr)
   {
      apm_debug_info->cmd_ctrl_debug.cmd_opcode=apm_info_ptr->curr_cmd_ctrl_ptr->cmd_opcode;
      apm_debug_info->cmd_ctrl_debug.cmd_pending=apm_info_ptr->curr_cmd_ctrl_ptr->cmd_pending;
      apm_debug_info->cmd_ctrl_debug.cmd_status=apm_info_ptr->curr_cmd_ctrl_ptr->cmd_status;
      apm_debug_info->cmd_ctrl_debug.agg_rsp_status=apm_info_ptr->curr_cmd_ctrl_ptr->agg_rsp_status;
   }


   total_size=total_size-ALIGN8(sizeof(apm_debug_info_t));
   uint32_t total_container_count =graph_info_ptr->num_containers ;
   apm_debug_info->num_containers=total_container_count;

   if (total_container_count == 0)
   {
      return;
   }

   //Storing container start address. Can be removed later
   if(total_size<((total_container_count)*sizeof(container_start_address)))
   {
      return;
   }

   total_size=total_size-ALIGN8(((total_container_count)*sizeof(container_start_address)));
   //Storing container start address. Can be removed later
   container_start_address=(uint32_t*)((uintptr_t)aligned_start_address+ALIGN8(sizeof(apm_debug_info_t)));
   aligned_start_address =(int8_t*)((uintptr_t)container_start_address +ALIGN8(((total_container_count)*sizeof(container_start_address))));


   uint32_t max_size_per_container = total_size / (total_container_count);
   uint32_t aligned_size           = ALIGN8((uint32_t)max_size_per_container);

   if (aligned_size > max_size_per_container)
   {
      aligned_size = aligned_size - 8;
   }

   apm_debug_info->size_per_container=aligned_size;
   while (curr_graph_node_ptr)
   {
      cont_graph_ptr                 = (apm_cont_graph_t *)curr_graph_node_ptr->obj_ptr;
      spf_list_node_t *curr_node_ptr = cont_graph_ptr->container_list_ptr;
      while (curr_node_ptr)
      {
         //Should not hit the following condition
         if(container_index>=total_container_count)
         {
            return;
         }
         container_node_ptr = (apm_container_t *)curr_node_ptr->obj_ptr;
         cntr_type          = container_node_ptr->prop.cntr_type.type;
         cont_hdl_ptr       = container_node_ptr->cont_hdl_ptr;
         cntr_cmn_dump_debug_info(cont_hdl_ptr, cntr_type, aligned_start_address, aligned_size);
         container_start_address[container_index]=(uintptr_t)aligned_start_address;
         aligned_start_address = aligned_start_address + aligned_size;
         curr_node_ptr         = curr_node_ptr->next_ptr;
         container_index++;
      }

      curr_graph_node_ptr = curr_graph_node_ptr->next_ptr;
   }
   return;
}

