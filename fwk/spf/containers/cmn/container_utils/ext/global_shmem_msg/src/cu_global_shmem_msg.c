
/**
 * \file cu_global_shmem_msg.c
 *
 * \brief
 *
 *     CU utility for global shared  memory message handling.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "cu_global_shmem_msg.h"
#include "container_utils.h"
#include "gpr_packet.h"
#include "gpr_api_inline.h"
#include "apm.h"

void cu_handle_global_shmem_msg(cu_base_t *cu_ptr, gpr_packet_t *packet_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result = AR_EOK;

   switch (packet_ptr->opcode)
   {
      case AR_SPF_MSG_GLOBAL_SH_MEM:
      {
         ar_spf_msg_global_sh_mem_t *cmd_header_ptr = GPR_PKT_GET_PAYLOAD(ar_spf_msg_global_sh_mem_t, packet_ptr);
         uint32_t                    mem_map_handle = 0;
         uint64_t                    virtual_addr   = 0;

         if (!cu_ptr->topo_vtbl_ptr->set_global_sh_mem_msg)
         {
            THROW(result, AR_EUNSUPPORTED);
         }

         if (cmd_header_ptr->shmem_size)
         {
            TRY(result,
                posal_memorymap_get_mem_map_handle(apm_get_mem_map_client(),
                                                   cmd_header_ptr->shmem_id,
                                                   &mem_map_handle));

            TRY(result,
                posal_memorymap_get_virtual_addr_from_shm_handle_v2(apm_get_mem_map_client(),
                                                                    mem_map_handle,
                                                                    cmd_header_ptr->shmem_addr_lsw,
                                                                    cmd_header_ptr->shmem_addr_msw,
                                                                    cmd_header_ptr->shmem_size,
                                                                    FALSE, // ref count not increased, see the comment
                                                                           // below
                                                                    (void *)&(virtual_addr)));
         }

         if (packet_ptr->dst_port == cu_ptr->gu_ptr->container_instance_id)
         {
            // todo:
            result = AR_EUNSUPPORTED;
         }
         else
         {
            result = cu_ptr->topo_vtbl_ptr->set_global_sh_mem_msg(cu_ptr->topo_ptr,
                                                                  packet_ptr->dst_port,
                                                                  (void *)virtual_addr,
                                                                  (void *)cmd_header_ptr);
         }

         /* Not managing ref count in this command handling context in the interest of optimization.
          * A corner case may come when APM tries to unmap the shared memory while this command is being handled by the
          * module. Since ref count is not updated therefore APM will be able to unmap the memory successfully.
          * */
         // posal_memorymap_shm_decr_refcount(apm_get_mem_map_client(), mem_map_handle);

         if (AR_FAILED(result) ||
             !(cmd_header_ptr->property_flag & AR_SPF_MSG_GLOBAL_SH_MEM_BIT_MASK_IS_ACK_NOT_REQUIRED))
         {
            __gpr_cmd_end_command(packet_ptr, result);
            return;
         }

         break;
      }

      default:
         result = AR_EUNSUPPORTED;
         break;
   }

   CATCH(result, CU_MSG_PREFIX, cu_ptr->gu_ptr->log_id)
   {
      __gpr_cmd_end_command(packet_ptr, result);
   }
}
