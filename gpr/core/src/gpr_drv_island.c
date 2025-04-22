/**
 * \file gpr_drv_island.c
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

/*****************************************************************************
 * Global variables                                                          *
 ****************************************************************************/

/* Local GPR IPC table containing init,deinit functions and vtbl function pointer to be populated for all domains*/
struct ipc_dl_local_t local_gpr_ipc_dl_table[GPR_PL_NUM_TOTAL_DOMAINS_V] = {
   { 0, NULL, NULL, NULL, true }, { 0, NULL, NULL, NULL, true }, { 0, NULL, NULL, NULL, true },
   { 0, NULL, NULL, NULL, true }, { 0, NULL, NULL, NULL, true }, { 0, NULL, NULL, NULL, true },
   { 0, NULL, NULL, NULL, true },
};

/* local ipc vtbl functions */
ipc_to_gpr_vtbl_t local_vtbl = {
   gpr_local_send,
   gpr_local_receive_done,
};

// global context variable
gpr_ctxt_struct_type gpr_ctxt_struct_t;

/*****************************************************************************
 * Helper functions                                                          *
 ****************************************************************************/

void gpr_drv_isr_lock_fn(void)
{
   (void)ar_osal_mutex_lock(gpr_ctxt_struct_t.gpr_drv_isr_lock);
}

void gpr_drv_isr_unlock_fn(void)
{
   (void)ar_osal_mutex_unlock(gpr_ctxt_struct_t.gpr_drv_isr_lock);
}

/**
  @brief Sends an asynchronous message to other modules.

  @param[in] packet  Packet (message) to send.

  @detdesc
  Calling #GPR_CMDID_ALLOC or #GPR_CMDID_ALLOC_EXT allocates free messages
  for sending. The caller can try to resend messages, or abort and free
  messages when delivery fails.

  This GPR_CMDID_ASYNC_SEND function provides the caller with low-level
  control over the sending process. For general use, consider using a
  simplified helper function, such as #GPR_CMDID_ALLOC_SEND.

  @return
  #AR_EOK when successful.
*/
uint32_t __gpr_cmd_async_send(gpr_packet_t *packet)
{
   uint32_t rc = AR_EOK;
   if (NULL == packet)
   {
      AR_MSG(DBG_ERROR_PRIO, "Param pointer is NULL");
      return AR_EBADPARAM;
   }

   uint32_t          domain_id  = packet->dst_domain_id;
   uint32_t          packet_len = GPR_PKT_GET_PACKET_BYTE_SIZE(packet->header);
   gpr_memq_block_t *block;

#ifdef GPR_DEBUG_MSG
   AR_MSG(DBG_HIGH_PRIO,
          "GPR async Send: Destination Domain ID %hhu, Destination Port %ld opcode 0x%lx token 0x%lx",
          packet->dst_domain_id,
          packet->dst_port,
          packet->opcode,
          packet->token);
#endif

   if (packet->src_domain_id != packet->dst_domain_id)
   {
      // Log incoming and outgoing packets
      gpr_log_packet(packet);
   }

   if (GPR_PL_MAX_DOMAIN_ID_V < domain_id)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "GPR send: Domain ID out of bounds %lu, Max ID supported %lu",
             domain_id,
             GPR_PL_MAX_DOMAIN_ID_V);
      return AR_EBADPARAM;
   }

   if ((NULL == local_gpr_ipc_dl_table[domain_id].fn_ptr) || (NULL == local_gpr_ipc_dl_table[domain_id].fn_ptr->send))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "NULL Fn ptr, = 0x%lx -> if this is not NULL, the send ptr is NULL",
             local_gpr_ipc_dl_table[domain_id].fn_ptr);
      return AR_EFAILED;
   }

   // check if the packet is from a static packet pool.
   // if so, set memq metadata and then send the packet
   // else, just call send
   bool_t pkt_is_from_the_static_pool = false;
   for (uint32_t idx = 0; idx < gpr_ctxt_struct_t.num_static_packet_pools; idx++)
   {
      /* Freeing linked list nodes*/
      if ((char *)packet > gpr_ctxt_struct_t.static_pool_arr[idx].packet_heap &&
          (char *)packet < gpr_ctxt_struct_t.static_pool_arr[idx].packet_heap_end)
      {
         pkt_is_from_the_static_pool = true;

#ifdef GPR_DEBUG_MSG
         AR_MSG(DBG_HIGH_PRIO,
                "gpr packet send: Destination Domain ID %hhu, Destination Port %ld",
                packet->dst_domain_id,
                packet->dst_port);
#endif
         if (packet_len <= gpr_ctxt_struct_t.static_pool_arr[idx].buf_size)
         {
            block = gpr_ctxt_struct_t.static_pool_arr[idx].free_packets_memq;
         }
         else
         {
            AR_MSG(DBG_ERROR_PRIO, "Send error %lu", packet->dst_port);
            return AR_EFAILED;
         }

         /* Sets the packet ownership to destination before sending */
         gpr_memq_node_set_metadata(block, packet, 0, packet->dst_port);

         rc = local_gpr_ipc_dl_table[domain_id].fn_ptr->send(domain_id, packet, packet_len);
         if (rc)
         {
            /* Sets the packet owner to source if send fails for any reason */
            gpr_memq_node_set_metadata(block, packet, 0, packet->src_port);
            AR_MSG(DBG_ERROR_PRIO,
                   "gpr packet send failed rc %d: Destination Domain ID %hhu, Destination Port %ld Opcode %lx token "
                   "%lx",
                   rc,
                   packet->dst_domain_id,
                   packet->dst_port,
                   packet->opcode,
                   packet->token);
         }

         // stop iterating the loop
         break;
      }
   }

   // If the packet is not from static pool, it could be from datalink packet or dynamic packet pool.
   if (!pkt_is_from_the_static_pool)
   {
#ifdef GPR_DEBUG_MSG
      AR_MSG(DBG_HIGH_PRIO,
             "GPR datalink packet send: Destination Domain ID %hhu, Destination Port %ld",
             packet->dst_domain_id,
             packet->dst_port);
#endif
      rc = local_gpr_ipc_dl_table[domain_id].fn_ptr->send(domain_id, packet, packet_len);

      if (rc)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "gpr packet send failed rc %d: Destination Domain ID %hhu, Destination Port %ld Opcode %lx token %lx",
                rc,
                packet->dst_domain_id,
                packet->dst_port,
                packet->opcode,
                packet->token);
      }
   }

   return rc;
}
/**
  @brief Allocates a free message for delivery.

  @param[in]  alloc_size  Amount of memory required for allocation, in bytes.
  @param[out] ret_packet  Returns the pointer to the allocated packet.

  @detdesc
  This function allocates a packet from the indicated module's free packet
  queue, and it provides the caller with low-level control over the allocation
  process. For general use, consider using a simplified helper function, such
  as #GPR_CMDID_ALLOC_EXT.

  @return
  #AR_EOK when successful.
*/
uint32_t __gpr_cmd_alloc(uint32_t alloc_size, gpr_packet_t **ret_packet)
{
   return __gpr_cmd_alloc_v2(alloc_size, GPR_HEAP_INDEX_DEFAULT, ret_packet);
}

/**
  @brief Allocates a free message for delivery.

  @param[in]  alloc_size  Amount of memory required for allocation, in bytes.
  @param[out] ret_packet  Returns the pointer to the allocated packet.

  @detdesc
  This function allocates a packet from the indicated module's free packet
  queue, and it provides the caller with low-level control over the allocation
  process. For general use, consider using a simplified helper function, such
  as #GPR_CMDID_ALLOC_EXT.

  @return
  #AR_EOK when successful.
*/
uint32_t __gpr_cmd_alloc_v2(uint32_t alloc_size, gpr_heap_index_t heap_index, gpr_packet_t **ret_packet)
{
   gpr_memq_block_t *block;
   gpr_packet_t     *new_packet  = NULL;
   uint32_t          packet_size = (GPR_PKT_HEADER_BYTE_SIZE_V + alloc_size);

   if (NULL == ret_packet)
   {
      AR_MSG(DBG_ERROR_PRIO, "alloc_error, NULL packet");
      return AR_EBADPARAM;
   }

   bool_t found_packet_pool = FALSE;
   for (uint32_t idx = 0; idx < gpr_ctxt_struct_t.num_static_packet_pools; idx++)
   {
      if (packet_size <= gpr_ctxt_struct_t.static_pool_arr[idx].buf_size &&
          (heap_index == gpr_ctxt_struct_t.static_pool_arr[idx].heap_index))
      {
         found_packet_pool = TRUE;

         block      = gpr_ctxt_struct_t.static_pool_arr[idx].free_packets_memq;
         new_packet = (gpr_packet_t *)gpr_memq_alloc(block);
         break;
      }
   }

   // If packet couldnt not be allocated in static pool, check dynamic pool
   if (NULL == new_packet)
   {
      for (uint32_t idx = 0; idx < gpr_ctxt_struct_t.num_dyn_packet_pools; idx++)
      {
         if ((packet_size <= gpr_ctxt_struct_t.dyn_pool_arr[idx].buf_size) &&
             (gpr_ctxt_struct_t.dyn_pool_arr[idx].curr_num_packets <
              gpr_ctxt_struct_t.dyn_pool_arr[idx].max_num_packets) &&
             (heap_index == gpr_ctxt_struct_t.dyn_pool_arr[idx].heap_index))
         {
            found_packet_pool = TRUE;

            gpr_allocate_dynamic_packet(&new_packet, packet_size);
            if (new_packet == NULL)
            {
               AR_MSG(DBG_ERROR_PRIO, "alloc_error unsupported size %lu, heap_index: %lu", alloc_size, heap_index);
               return AR_ENORESOURCE;
            }
            gpr_ctxt_struct_t.dyn_pool_arr[idx].curr_num_packets++;
         }
      }
   }

   /* Check if packet has been allocated from a dynamic/static pool.*/
   if (new_packet == NULL)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "alloc_error, NULL packet found_packet_pool %lu, alloc size %lu heap_index %lu",
             found_packet_pool,
             alloc_size,
             heap_index);
      return AR_ENORESOURCE;
   }

   ar_mem_set(new_packet, 0, sizeof(gpr_packet_t));
   new_packet->header = GPR_SET_FIELD(GPR_PKT_VERSION, GPR_PKT_VERSION_V) |
                        GPR_SET_FIELD(GPR_PKT_HEADER_SIZE, GPR_PKT_HEADER_WORD_SIZE_V) |
                        GPR_SET_FIELD(GPR_PKT_PACKET_SIZE, packet_size);
   new_packet->opcode      = GPR_UNDEFINED_ID_V;
   new_packet->client_data = GPR_PKT_INIT_CLIENT_DATA_V;
   new_packet->reserved    = GPR_PKT_INIT_RESERVED_V;
   *ret_packet             = new_packet;

   return AR_EOK;
}

/**
  @brief Frees a packet from the indicated module's free packet queue.

  @param[in] packet  Packet to return.

  @return
  #AR_EOK when successful.
*/
uint32_t __gpr_cmd_free(gpr_packet_t *packet)
{
   if (NULL == packet)
   {
      AR_MSG(DBG_ERROR_PRIO, "Param pointer is NULL");
      return AR_EBADPARAM;
   }

   gpr_memq_block_t *block       = NULL;
   uint32_t          packet_size = GPR_PKT_GET_PACKET_BYTE_SIZE(packet->header);
   uint32_t          domain_id   = packet->src_domain_id;

   bool_t pkt_is_from_static_pool = FALSE;
   /* If the packet is from Static pool, mark it Free*/
   for (uint32_t idx = 0; idx < gpr_ctxt_struct_t.num_static_packet_pools; idx++)
   {
      if ((char *)packet > gpr_ctxt_struct_t.static_pool_arr[idx].packet_heap &&
          (char *)packet < gpr_ctxt_struct_t.static_pool_arr[idx].packet_heap_end)
      {
         /* If buffer is allocated by GPR*/
         if (packet_size <= gpr_ctxt_struct_t.static_pool_arr[idx].buf_size)
         {
            block = gpr_ctxt_struct_t.static_pool_arr[idx].free_packets_memq;

            /* Sets the packet owner to 0 before free the packet. */
            gpr_memq_node_set_metadata(block, packet, 0, 0);
            gpr_memq_free(block, packet);
            pkt_is_from_static_pool = TRUE;
            return AR_EOK;
         }
         else
         {
            return AR_EBADPARAM;
         }
      }
   }

   // If packet is not from static pool, it could be from Dynamic pool.
   if (FALSE == pkt_is_from_static_pool)
   {
      for (uint32_t idx = 0; idx < gpr_ctxt_struct_t.num_dyn_packet_pools; idx++)
      {
         if ((packet_size <= gpr_ctxt_struct_t.dyn_pool_arr[idx].buf_size) &&
             (gpr_ctxt_struct_t.dyn_pool_arr[idx].curr_num_packets > 0))
         {
            if (gpr_check_and_free_dynamic_packet(packet) == AR_EOK)
            {
               gpr_ctxt_struct_t.dyn_pool_arr[idx].curr_num_packets--;
               return AR_EOK;
            }
         }
      }
   }

   if (GPR_PL_MAX_DOMAIN_ID_V < domain_id)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "GPR free: Domain ID out of bounds %lu, Max ID supported %lu",
             domain_id,
             GPR_PL_MAX_DOMAIN_ID_V);
      return AR_EBADPARAM;
   }

   /* If buffer belongs to datalink layer*/
   if ((NULL != local_gpr_ipc_dl_table[domain_id].fn_ptr) &&
       (NULL != local_gpr_ipc_dl_table[domain_id].fn_ptr->receive_done))
   {
      return local_gpr_ipc_dl_table[domain_id].fn_ptr->receive_done(domain_id, (void *)packet);
   }
   else
   {
      AR_MSG(DBG_HIGH_PRIO, "Datalink free function pointer is NULL");
      return AR_EFAILED;
   }
}

/*****************************************************************************
 * Utility Control Implementations                                           *
 ****************************************************************************/
/**
  @brief Allocates a pre-formatted free message for delivery.

  @param[in]  src_domain_id   Domain id of the sender.
  @param[in]  src_port        Port address of the sender.
  @param[in]  dst_domain_id   Domain id of the receiver
  @param[in]  dst_port        Port address of the receiver.
  @param[in]  token           Value attached by the sender to determine when
                              command messages have been processed by the receiver
                              after having received response messages.
  @param[in]  opcode          Defines both the action and the payload structure to
                              the receiver.
  @param[in]  payload_size    Actual number of bytes needed for the payload.
  @param[out] ret_packet      Return pointer to the allocated packet.

  @detdesc
  This helper function partially creates a message for delivery. The message
  allocation and initialization is performed by this function, but the message
  payload is dependent on the caller to fill in. The two-step process allows
  the caller to avoid multiple memcpy() operations on the message payload.

  @consid
  The helper function allocates from the indicated service's free packet pool
  based on the msg_type.
    @par
  - For CMDRSP messages, #GPR_ALLOC_TYPE_RESPONSE is used.
  - For EVENT, SEQCMD, and NSEQCMD messages, #GPR_ALLOC_TYPE_COMMAND is used.

  @return
  #AR_EOK when successful.
 */
uint32_t __gpr_cmd_alloc_ext(gpr_cmd_alloc_ext_t *args)
{
   gpr_cmd_alloc_ext_v2_t args_v2;
   args_v2.heap_index    = GPR_HEAP_INDEX_DEFAULT;
   args_v2.dst_port      = args->dst_port;
   args_v2.dst_domain_id = args->dst_domain_id;
   args_v2.src_port      = args->src_port;
   args_v2.src_domain_id = args->src_domain_id;
   args_v2.token         = args->token;
   args_v2.opcode        = args->opcode;
   args_v2.client_data   = args->client_data;
   args_v2.payload_size  = args->payload_size;
   args_v2.ret_packet    = args->ret_packet;

   return __gpr_cmd_alloc_ext_v2(&args_v2);
}

uint32_t __gpr_cmd_alloc_ext_v2(gpr_cmd_alloc_ext_v2_t *args)
{
   uint32_t          rc;
   gpr_packet_t     *new_packet;
   gpr_memq_block_t *block;

   if ((NULL == args) || (NULL == args->ret_packet))
   {
      return AR_EBADPARAM;
   }

   rc = __gpr_cmd_alloc_v2(args->payload_size, args->heap_index, &new_packet);
   if (rc)
   {
      return rc;
   }

   new_packet->dst_port      = args->dst_port;
   new_packet->dst_domain_id = args->dst_domain_id;
   new_packet->src_port      = args->src_port;
   new_packet->src_domain_id = args->src_domain_id;
   new_packet->token         = args->token;
   new_packet->opcode        = args->opcode;
   new_packet->client_data   = args->client_data;

   // set metadata in the corresponding packets memq.
   for (uint32_t idx = 0; idx < gpr_ctxt_struct_t.num_static_packet_pools; idx++)
   {
      if ((char *)new_packet > gpr_ctxt_struct_t.static_pool_arr[idx].packet_heap &&
          (char *)new_packet < gpr_ctxt_struct_t.static_pool_arr[idx].packet_heap_end)
      {
         block = gpr_ctxt_struct_t.static_pool_arr[idx].free_packets_memq;
         gpr_memq_node_set_metadata(block, new_packet, 0, new_packet->src_port);
         break;
      }
   }

   *args->ret_packet = new_packet;

   return AR_EOK;
}

/**
  @brief Allocates and sends a formatted message.

  @param[in] src_domain_id   Domain id of the sender.
  @param[in] src_port        Port of the sender.
  @param[in] dst_domain_id   Domain id of the receiver
  @param[in] dst_port        Port of the receiver.
  @param[in] token           Value attached by the sender to determine when
                             command messages have been processed by the
                             receiver after having received response messages.
  @param[in] opcode          Defines both the action and the payload structure
                             to the receiver.
  @param[in] payload         Payload to send.
  @param[in] payload_size    Actual number of bytes needed for the payload.

  @detdesc
  This helper function fully creates and sends a message. The message
  allocation and initialization is performed by this function, and the message
  payload is supplied up front by the caller. See the examples in
  #GPR_CMDID_ASYNC_SEND and #GPR_CMDID_FORWARD for comparison.

  @return
  #AR_EOK when successful.
*/
uint32_t __gpr_cmd_alloc_send(gpr_cmd_alloc_send_t *args)
{
   uint32_t            rc;
   gpr_cmd_alloc_ext_t alloc_args;
   gpr_packet_t       *packet;

   if ((NULL == args) || ((args->payload_size > 0) && (NULL == args->payload)))
   {
      return AR_EBADPARAM;
   }

   alloc_args.src_port      = args->src_port;
   alloc_args.src_domain_id = args->src_domain_id;
   alloc_args.dst_port      = args->dst_port;
   alloc_args.dst_domain_id = args->dst_domain_id;
   alloc_args.client_data   = args->client_data;
   alloc_args.token         = args->token;
   alloc_args.opcode        = args->opcode;
   alloc_args.payload_size  = args->payload_size;
   alloc_args.ret_packet    = &packet;

   rc = __gpr_cmd_alloc_ext(&alloc_args);
   if (rc)
   {
      return rc;
   }

   if (args->payload_size > 0)
   {
      (void)ar_mem_cpy(GPR_PKT_GET_PAYLOAD(void, packet), args->payload_size, args->payload, args->payload_size);
   }

   rc = __gpr_cmd_async_send(packet);
   if (rc)
   {
      __gpr_cmd_free(packet);
   }

   return rc;
}

/**
  @brief Accepts a command message by replying with an #GPR_IBASIC_EVT_ACCEPTED
  message to the sender.

  @param[in] packet  Command packet to accept.

  @detdesc
  The same procedure can be performed manually by swapping the source and
  destination fields, and then inserting an #GPR_IBASIC_EVT_ACCEPTED payload.

  @return
  #AR_EOK when successful.
*/
uint32_t __gpr_cmd_accept_command(gpr_packet_t *packet)
{
   gpr_cmd_alloc_send_t      send_args;
   gpr_ibasic_evt_accepted_t rsp;

   if (NULL == packet)
   {
      return AR_EBADPARAM;
   }

   // Reverse the source and destination addresses to send a command accepted event.
   send_args.dst_port      = packet->src_port;
   send_args.dst_domain_id = packet->src_domain_id;
   send_args.src_port      = packet->dst_port;
   send_args.src_domain_id = packet->dst_domain_id;
   send_args.token         = packet->token;
   send_args.client_data   = packet->client_data;
   send_args.opcode        = GPR_IBASIC_EVT_ACCEPTED;
   rsp.opcode              = packet->opcode;
   send_args.payload       = &rsp;
   send_args.payload_size  = sizeof(rsp);

   (void)__gpr_cmd_alloc_send(&send_args);

   return AR_EOK;
}

/**
  @brief Completes a command message by replying with an #GPR_IBASIC_RSP_RESULT
  command response message to the sender. The indicated packet is then freed.

  @param[in] packet  Command message to complete.
  @param[in] status  Completion/error status (see Section
                     @xref{dox:AprCoreStatusErrorcodes} for available values).

  @detdesc
  The same procedure can be done manually by swapping the source and
  destination fields and then inserting an #GPR_IBASIC_RSP_RESULT command
  response payload.

  @return
  #AR_EOK when successful.
*/
// NOTE: The command packet is freed irrespective of failing to send the command response.
uint32_t __gpr_cmd_end_command(gpr_packet_t *packet, uint32_t status)
{
   uint32_t                rc = AR_EOK;
   gpr_cmd_alloc_ext_v2_t  alloc_args;
   gpr_ibasic_rsp_result_t rsp;
   gpr_packet_t           *rsp_pkt_ptr = NULL;

   if (NULL == packet)
   {
      return AR_EBADPARAM;
   }

   uint32_t opcode_type = ((packet->opcode & AR_GUID_TYPE_MASK) >> AR_GUID_TYPE_SHIFT);

   if (!(((opcode_type & AR_GUID_TYPE_CONTROL_CMD_RSP) == AR_GUID_TYPE_CONTROL_CMD_RSP) ||
         ((opcode_type & AR_GUID_TYPE_DATA_CMD_RSP) == AR_GUID_TYPE_DATA_CMD_RSP) ||
         ((opcode_type & AR_GUID_TYPE_CONTROL_EVENT) == AR_GUID_TYPE_CONTROL_EVENT) ||
         ((opcode_type & AR_GUID_TYPE_DATA_EVENT) == AR_GUID_TYPE_DATA_EVENT)))
   {
      // get GPR packet heap index
      gpr_heap_index_t gpr_heap_index = GPR_HEAP_INDEX_DEFAULT;
      for (uint32_t idx = 0; idx < gpr_ctxt_struct_t.num_static_packet_pools; idx++)
      {
         /* Freeing linked list nodes*/
         if ((char *)packet > gpr_ctxt_struct_t.static_pool_arr[idx].packet_heap &&
             (char *)packet < gpr_ctxt_struct_t.static_pool_arr[idx].packet_heap_end)
         {
            gpr_heap_index = gpr_ctxt_struct_t.static_pool_arr[idx].heap_index;
         }
      }

      // Reverse the source and destination addresses to send a command response.
      alloc_args.dst_port      = packet->src_port;
      alloc_args.dst_domain_id = packet->src_domain_id;
      alloc_args.src_port      = packet->dst_port;
      alloc_args.src_domain_id = packet->dst_domain_id;
      alloc_args.token         = packet->token;
      alloc_args.client_data   = packet->client_data;
      alloc_args.opcode        = GPR_IBASIC_RSP_RESULT;
      alloc_args.ret_packet    = &rsp_pkt_ptr;
      alloc_args.heap_index    = gpr_heap_index;
      alloc_args.payload_size  = sizeof(rsp);

      rc = __gpr_cmd_alloc_ext_v2(&alloc_args);
      if (rc)
      {
         AR_MSG(DBG_ERROR_PRIO, "gpr_unable to allocate packet");
         goto __bailout;
      }

      rsp.opcode = packet->opcode;
      rsp.status = status;
      (void)ar_mem_cpy(GPR_PKT_GET_PAYLOAD(void, rsp_pkt_ptr), alloc_args.payload_size, &rsp, sizeof(rsp));

      rc = __gpr_cmd_async_send(rsp_pkt_ptr);
      if (rc)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "GPR_unable to send to dst domain %d, dst port %d, src domain %d src port %d",
                packet->dst_domain_id,
                packet->dst_port,
                packet->src_domain_id,
                packet->src_port);
         __gpr_cmd_free(rsp_pkt_ptr);
         goto __bailout;
      }
   }
   else
   {
      AR_MSG(DBG_ERROR_PRIO, "Opcode 0x%lx is of unsupported type. Not sending basic response", packet->opcode);
   }

__bailout:
   __gpr_cmd_free(packet);
   return rc;
}
/*@brief Function to send gpr packet if its within the same domain
  @param[in] domain_id  Domain id of destination
  @param[in] buf        Buffer to be sent
  @param[in] length     Size of buffer

  @return
  #AR_EOK when successful.
*/
GPR_INTERNAL uint32_t gpr_local_send(uint32_t domain_id __UNUSED, void *buf, uint32_t length __UNUSED)
{
   __UNREFERENCED_PARAM(domain_id);
   __UNREFERENCED_PARAM(length);

   uint32_t            rc, result;
   gpr_module_entry_t *session = NULL;
   gpr_packet_t       *packet  = (gpr_packet_t *)buf;

#ifdef GPR_DEBUG_MSG
   AR_MSG(DBG_HIGH_PRIO,
          "GPR local send: Destination Domain ID %hhu, Destination Port %ld opcode %lx, token %lx",
          packet->dst_domain_id,
          packet->dst_port,
          packet->opcode,
          packet->token);
#endif

   ar_osal_mutex_lock(gpr_ctxt_struct_t.gpr_drv_task_lock);

   result = gpr_get_session_util(packet->dst_port, &session);

   ar_osal_mutex_unlock(gpr_ctxt_struct_t.gpr_drv_task_lock);

   if (AR_ENOTEXIST == result)
   {
      AR_MSG(DBG_ERROR_PRIO, "Session not found, returning");
      /*GPR will return error to the caller if session is not found to ensure synchronous error reporting*/
      rc = AR_ENOTEXIST;
   }
   else if ((NULL != session) && (NULL != session->callback_fn))
   {
      rc = session->callback_fn(packet, session->callback_data);
   }
   else
   {
      AR_MSG(DBG_ERROR_PRIO, "Sending packet failed, retrieved session is null");
      rc = AR_EFAILED;
   }
   return rc;
}

/*@brief Local free function
  @param[in] domain_id  Domain id of destination
  @param[in] buf        Buffer to be sent

  @return
  #AR_EOK when successful.
*/
GPR_INTERNAL uint32_t gpr_local_receive_done(uint32_t domain_id __UNUSED, void *buf __UNUSED)
{
   __UNREFERENCED_PARAM(domain_id);
   __UNREFERENCED_PARAM(buf);

   return AR_EOK;
}

/*@brief Internal utility to get the session handle for the given port. It searches
         all the available cb lists in the gpr driver.
  @param[in] port          Module port number
  @param[out] ret_entry    Return node ptr

  @return
  #AR_EOK when successful.
  #AR_ENOTEXIST when the given port is not found in the lists.
*/
GPR_INTERNAL uint32_t gpr_get_session_util(uint32_t port, gpr_module_entry_t **ret_entry)
{
   uint32_t result = AR_EOK;

   // Imp: Always check the non-deafult list first and then default.
   if (AR_EOK != (result = gpr_get_session(port, ret_entry, gpr_ctxt_struct_t.heap1_list)))
   {
      result = gpr_get_session(port, ret_entry, gpr_ctxt_struct_t.default_list);
   }

   if (AR_FAILED(result))
   {
      AR_MSG(DBG_ERROR_PRIO, "No such module port (%lu) found in linked list", port);
   }

   return result;
}

uint32_t __gpr_cmd_get_host_domain_id(uint32_t *host_domain_id)
{
   if (NULL == host_domain_id)
   {
      return AR_EBADPARAM;
   }
   *host_domain_id = gpr_ctxt_struct_t.default_domain_id;

   return AR_EOK;
}
