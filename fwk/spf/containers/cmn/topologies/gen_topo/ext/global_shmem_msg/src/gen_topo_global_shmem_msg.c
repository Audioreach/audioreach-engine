/**
 * \file gen_topo_global_shmem_msg.c
 *
 * \brief
 *
 *     topo utility for global shared  memory message handling.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_topo.h"
#include "gen_topo_capi.h"
#include "capi_fwk_extns_global_shmem_msg.h"
#include "apm.h"

static capi_err_t gen_topo_capi_cb_global_sh_mem_cmd_handler(void                          *context_ptr,
                                                             uint32_t                       shmem_id,
                                                             fwk_extn_global_shmem_cb_cmd_t cmd,
                                                             capi_buf_t                     buffer)
{
   gen_topo_t *me_ptr         = (gen_topo_t *)context_ptr;
   capi_err_t  result         = CAPI_EOK;
   uint32_t    mem_map_handle = 0;
   uint32_t    client_token   = apm_get_mem_map_client();

   if (AR_FAILED(posal_memorymap_get_mem_map_handle(client_token, shmem_id, &mem_map_handle)))
   {
      TOPO_MSG(me_ptr->gu.log_id, DBG_ERROR_PRIO, "memmap not found for shmem_id %lu", shmem_id);
      return CAPI_EFAILED;
   }

   switch (cmd)
   {
      case FWK_EXTN_INC_SHMEM_REF_COUNT:
      {
         posal_memorymap_shm_incr_refcount(apm_get_mem_map_client(), mem_map_handle);
         break;
      }
      case FWK_EXTN_DEC_SHMEM_REF_COUNT:
      {
         posal_memorymap_shm_decr_refcount(apm_get_mem_map_client(), mem_map_handle);
         break;
      }
      default:
         return CAPI_EUNSUPPORTED;
   }

   return result;
}

ar_result_t gen_topo_init_global_sh_mem_extn(void *topo_ptr, gen_topo_module_t *module_ptr)
{
   gen_topo_t                           *me_ptr = (gen_topo_t *)topo_ptr;
   ar_result_t                           result = AR_EOK;
   capi_param_set_global_shmem_cb_func_t cb     = { .cb_func     = gen_topo_capi_cb_global_sh_mem_cmd_handler,
                                                    .context_ptr = topo_ptr };

   TOPO_MSG(me_ptr->gu.log_id, DBG_HIGH_PRIO, "MIID 0x%x global shmem fwk ext init", module_ptr->gu.module_instance_id);

   result = gen_topo_capi_set_param(me_ptr->gu.log_id,
                                    module_ptr->capi_ptr,
                                    FWK_EXTN_PARAM_ID_SET_GLOBAL_SHMEM_CB_FUNC,
                                    (int8_t *)&cb,
                                    sizeof(cb));

   return result;
}

ar_result_t gen_topo_set_global_sh_mem_msg(void *topo_ptr, uint32_t miid, void *virt_addr_ptr, void *cmd_header_ptr)
{
   ar_result_t                       result    = AR_EOK;
   gen_topo_t                       *me_ptr    = (gen_topo_t *)topo_ptr;
   capi_param_set_global_shmem_msg_t shmem_msg = { .payload_virtual_addr = virt_addr_ptr,
                                                   .cmd_header_addr      = cmd_header_ptr };

   gen_topo_module_t *module_ptr = (gen_topo_module_t *)gu_find_module(&me_ptr->gu, miid);

   if (!module_ptr)
   {
      TOPO_MSG(me_ptr->gu.log_id, DBG_ERROR_PRIO, "MIID 0x%x not found", miid);

      return AR_EFAILED;
   }

   TOPO_MSG(me_ptr->gu.log_id, DBG_HIGH_PRIO, "MIID 0x%x received global shmem fwk ext msg", module_ptr->gu.module_instance_id);

   result = gen_topo_capi_set_param(me_ptr->gu.log_id,
                                    module_ptr->capi_ptr,
                                    FWK_EXTN_PARAM_ID_SET_GLOBAL_SHMEM_MSG,
                                    (int8_t *)&shmem_msg,
                                    sizeof(shmem_msg));

   return result;
}
