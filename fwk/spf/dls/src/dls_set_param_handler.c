/**
 * \file dls_set_param_handler.c
 * \brief
 *  	This file contains DLS set param handling API function definitions
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* ----------------------------------------------------------------------------
 * Include Files
 * ------------------------------------------------------------------------- */
#include "dls_i.h"

/*------------------------------------------------------------------------------
 *  Global Definitions/Externs
 *----------------------------------------------------------------------------*/
extern dls_param_id_config_buffer_t g_dls_config_buffer_info;
extern uint32_t g_dls_current_buf_count;
extern uint32_t g_dls_total_buf_count;
extern uint32_t g_dls_log_code[DLS_MAX_NUM_LOG_CODES];

/*------------------------------------------------------------------------------
 *  Function Definitions
 *----------------------------------------------------------------------------*/
/**
  This is an utility function used to read the command payload data

  @param[in]     packet             pointer to the GPR packet
  @param[in]     min_payload_size   minimum payload size
  @param[in]     param_ptr          pointer to the parameter payload
  @param[in]     param_size         total parameter payload size

  @param[out]    param_ptr          pointer to the parameter payload
  @param[out]    param_size         total parameter payload size

  @return        result             status error code

  @dependencies  None
 */
ar_result_t dls_get_set_cmd_param_payload(gpr_packet_t *pkt_ptr,
                                          uint32_t      min_payload_size,
                                          void        **param_ptr,
                                          uint32_t     *param_size)
{
   ar_result_t result = AR_EOK;
   apm_cmd_header_t *cmd_hdr_ptr = GPR_PKT_GET_PAYLOAD(apm_cmd_header_t, pkt_ptr);
   if (cmd_hdr_ptr->payload_size < min_payload_size)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "DLS: Failed to get apm cmd payload. payload_len: %lu, min_size: %lu, result: 0x%lx",
             cmd_hdr_ptr->payload_size,
             min_payload_size,
             AR_EBADPARAM);
      result = AR_EBADPARAM;
      goto __dls_get_set_cmd_param_payload_bail_out;
   }

   uint8_t *     payload_ptr      = NULL;
   gpr_packet_t *temp_gpr_pkt_ptr = NULL; // dont care var, used only for get cfg and DLS doesnt support APM get cfg
   uint32_t      alignment_size   = 0;

   result = spf_svc_get_cmd_payload_addr(DLS_MODULE_INSTANCE_ID,
                                         pkt_ptr,
                                         &temp_gpr_pkt_ptr,
                                         (uint8_t **)&payload_ptr,
                                         &alignment_size,
                                         NULL,
                                         apm_get_mem_map_client());

   // populate return variables
   *param_ptr  = (void *)payload_ptr;
   *param_size = cmd_hdr_ptr->payload_size;

__dls_get_set_cmd_param_payload_bail_out:
   return result;
}

/**
  This function will handle the DLS_PARAM_ID_CONFIG_BUFFER param received from
  the RTM client and configure the buffer information, like, size, address, etc.

  @param[in]     dls_info_ptr             pointer to the DLS service information
  @param[in]     param_ptr                pointer to the buffer configuration info
  @param[in]     param_size               total parameter payload size

  @param[out]    g_dls_config_buffer_info buffer configuration information

  @return        result                   status error code

  @dependencies  before receiving this set param, RTM client must create the
  memory for data logging
 */
static ar_result_t dls_set_param_config_buffer(dls_t *dls_info_ptr,
                                               void *param_ptr,
                                               uint32_t param_size)
{
   ar_result_t result = AR_EOK;
   if (param_size < sizeof(dls_param_id_config_buffer_t))
   {
      AR_MSG(DBG_ERROR_PRIO, "dls_set_param_config_buffer: param_size 0x%d is less than the"
             "sizeof(dls_param_id_config_buffer_t): 0x%d", param_size, sizeof(dls_param_id_config_buffer_t));
      result = AR_EBADPARAM;
      goto __dls_set_param_cfg_buf_bail_out;
   }

   dls_param_id_config_buffer_t *cfg_buf_ptr = (dls_param_id_config_buffer_t *)param_ptr;
   memcpy(&g_dls_config_buffer_info, cfg_buf_ptr, sizeof(dls_param_id_config_buffer_t));

   //map the physical address to the virtual address
   uint64_t buf_start_virt_addr = 0;
   if (AR_DID_FAIL(result = posal_memorymap_get_virtual_addr_from_shm_handle_v2(apm_get_mem_map_client(),
                                                                                g_dls_config_buffer_info.mem_map_handle,
                                                                                g_dls_config_buffer_info.buf_start_addr_lsw,
                                                                                g_dls_config_buffer_info.buf_start_addr_msw,
                                                                                g_dls_config_buffer_info.max_log_pkt_size,
                                                                                TRUE, // is_ref_counted
                                                                                &buf_start_virt_addr)))
   {
      AR_MSG(DBG_ERROR_PRIO, "Phy to Virt Failed(paddr,vaddr)-->(%lx %lx, 0x%p)\n for the data buffer address",
                   g_dls_config_buffer_info.buf_start_addr_lsw,
                   g_dls_config_buffer_info.buf_start_addr_msw,
                   (buf_start_virt_addr));

      result = AR_EFAILED;
      goto __dls_set_param_cfg_buf_bail_out;
   }

   //store the virtual address in the global struct
   g_dls_config_buffer_info.buf_start_addr_lsw = GET_LSW_FROM_64BIT_WORD(buf_start_virt_addr);
   g_dls_config_buffer_info.buf_start_addr_msw = GET_MSW_FROM_64BIT_WORD(buf_start_virt_addr);
   AR_MSG(DBG_LOW_PRIO,
          "dls_set_param_config_buffer: Received buffer config command. buf_start_addr_lsw=0x%p,"
          " buf_start_addr_msw=0x%p, max_log_pkt_size=%d, total_buf_size=%d, is_cfg=%d",
          g_dls_config_buffer_info.buf_start_addr_lsw,
          g_dls_config_buffer_info.buf_start_addr_msw,
          g_dls_config_buffer_info.max_log_pkt_size,
          g_dls_config_buffer_info.total_buf_size,
          g_dls_config_buffer_info.is_cfg);

   if (!buf_start_virt_addr)
   {
      AR_MSG(DBG_ERROR_PRIO, "dls_config_buffer: buffer stating address is NULL");
      result = AR_EBADPARAM;
      goto __dls_set_param_cfg_buf_bail_out;
   }

   /** Check if the buffer start address is 8 byte aligned */
   if (!IS_ALIGN_8_BYTE(buf_start_virt_addr))
   {
      AR_MSG(DBG_ERROR_PRIO, "dls_config_buffer: configured buffer start address: (0x%p)"
             " is not 8 byte aligned", buf_start_virt_addr);
      result = AR_EBADPARAM;
      goto __dls_set_param_cfg_buf_bail_out;
   }

   if (g_dls_config_buffer_info.is_cfg == 1)
   {
      /* check the max single buffer size and if it is set to default then update it to maximum log packet size */
      if (g_dls_config_buffer_info.max_log_pkt_size == DLS_SET_BUF_SIZE_TO_DEFAULT)
      {
         g_dls_config_buffer_info.max_log_pkt_size = DLS_DEFAULT_MAX_LOG_PKT_SIZE;
      }

      /* go through the buffers and mark all buffers status to available */
      uint32_t buf_offset = 0;
      while ((buf_offset + g_dls_config_buffer_info.max_log_pkt_size) <= g_dls_config_buffer_info.total_buf_size)
      {
         uint64_t buf_offset_addr = buf_start_virt_addr + buf_offset;
         uint8_t *buf = (uint8_t *)buf_offset_addr;
         dls_buf_hdr_t *dls_buf = (dls_buf_hdr_t *)buf;
         dls_buf->buf_state = (dls_buf_state_t)DLS_BUF_AVAILABLE;

         /* update the buffer offset address to get the next buffer start address */
         buf_offset = buf_offset + g_dls_config_buffer_info.max_log_pkt_size;
         g_dls_total_buf_count++; // increment the buffer counter
      }
      AR_MSG(DBG_LOW_PRIO, "dls_config_buffer: total number of buffers configured are %d", g_dls_total_buf_count);

      // calculate the buffer utilization just to make sure there is no memory goes unused and print a message
      if ((g_dls_total_buf_count * g_dls_config_buffer_info.max_log_pkt_size) < g_dls_config_buffer_info.total_buf_size)
      {
         AR_MSG(DBG_LOW_PRIO, "dls_config_buffer: total memory that is unused in total buffer is %d bytes",
                g_dls_config_buffer_info.total_buf_size - (g_dls_total_buf_count * g_dls_config_buffer_info.max_log_pkt_size));
      }

      AR_MSG(DBG_LOW_PRIO, "dls_config_buffer: buffer configuration is success");
   }
   else
   {
      //TODO:in future, if there is a requirement to de-configure for any reason then add it here.
      AR_MSG(DBG_LOW_PRIO, "dls_config_buffer: buffer de-configuration has nothing to do here");
   }

   // assign the total buf count to current buf count to track the available buf count
   g_dls_current_buf_count = g_dls_total_buf_count;

__dls_set_param_cfg_buf_bail_out:
   return result;
}

/**
  This function will handle the DLS_PARAM_ID_ENABLE_DISABLE_LOG_CODE param
  received from the RTM client and update the log code information

  @param[in]     dls_info_ptr    pointer to the DLS service information
  @param[in]     param_ptr       pointer to the log code info
  @param[in]     param_size      total parameter payload size

  @param[out]    g_dls_log_code  global log code information

  @return        result          status error code

  @dependencies  None
 */
static ar_result_t dls_set_param_log_code(dls_t *dls_info_ptr,
                                          void *param_ptr,
                                          uint32_t param_size)
{
   ar_result_t result = AR_EOK;
   int32_t log_code_idx = DLS_LOG_CODE_NOT_FOUND;

   if (param_size < sizeof(dls_param_id_enable_disable_log_code_t))
   {
      AR_MSG(DBG_ERROR_PRIO, "dls_set_param_log_code: param_size 0x%d is less than the"
             "sizeof(dls_param_id_enable_disable_log_code_t): 0x%d", param_size, sizeof(dls_param_id_enable_disable_log_code_t));
      result = AR_EBADPARAM;
      goto __dls_set_param_log_code;
   }

   dls_param_id_enable_disable_log_code_t *log_code_info = (dls_param_id_enable_disable_log_code_t *)param_ptr;

   uint32_t num_log_codes = log_code_info->num_log_codes;
   if (param_size < (sizeof(dls_param_id_enable_disable_log_code_t) + (sizeof(uint32_t) * num_log_codes)))
   {
      AR_MSG(DBG_ERROR_PRIO, "dls_set_param_log_code: param_size 0x%d is less than the"
             "(sizeof(dls_param_id_enable_disable_log_code_t) + (sizeof(uint32_t) * num_log_codes)): 0x%d",
             param_size,
             (sizeof(dls_param_id_enable_disable_log_code_t) + (sizeof(uint32_t) * num_log_codes)));
      result = AR_ENEEDMORE;
      goto __dls_set_param_log_code;
   }

   AR_MSG(DBG_HIGH_PRIO, "dls_set_param_log_code: received %d number of log codes with is_enable set to %d",
          log_code_info->num_log_codes,
          log_code_info->is_enabled);

   /* check if the received number of log codes are not greater than the maximum */
   if (log_code_info->num_log_codes > DLS_MAX_NUM_LOG_CODES)
   {
      AR_MSG(DBG_ERROR_PRIO, "dls_set_param_log_code: number of log codes %d to enable/disable should be <= the max log codes %d",
             log_code_info->num_log_codes, DLS_MAX_NUM_LOG_CODES);
   }

   /* if is_enabled flag is set to 1 then add the log codes to the g_dls_log_code array*/
   if (log_code_info->is_enabled)
   {
      for (int32_t inp_log_code_idx = 0; inp_log_code_idx < log_code_info->num_log_codes; inp_log_code_idx++)
      {
         // check if the log code already exists in the global list
         if(!log_code_info->log_codes[inp_log_code_idx])
         {
            AR_MSG(DBG_ERROR_PRIO, "dls_set_param_log_code(): log code cannot be zero");
            result = AR_EBADPARAM;
            goto __dls_set_param_log_code;
         }
         log_code_idx = dls_is_log_code_exists(log_code_info->log_codes[inp_log_code_idx]);
         if (log_code_idx != DLS_LOG_CODE_NOT_FOUND)//if log code available in the list
         {
            AR_MSG(DBG_MED_PRIO, "dls_set_param_log_code(): log code 0x%p already present "
                  "in the global list",
                  log_code_info->log_codes[inp_log_code_idx]);
            continue; // if the log code exists then move on to add the next log code in the list
         }
         else
         {
            for (int32_t global_log_code_idx = 0; global_log_code_idx < DLS_MAX_NUM_LOG_CODES; global_log_code_idx++)
            {
               if (0 == g_dls_log_code[global_log_code_idx]) // go through the global list and add it when an empty location is found
               {
                  g_dls_log_code[global_log_code_idx] = log_code_info->log_codes[inp_log_code_idx];
                  break;
               }
            }
         }
      }
   }
   else
   {
      for (int32_t inp_log_code_idx = 0; inp_log_code_idx < log_code_info->num_log_codes; inp_log_code_idx++)
      {
         // check if the log code already exists in the global list
         log_code_idx = dls_is_log_code_exists(log_code_info->log_codes[inp_log_code_idx]);
         if (log_code_idx == DLS_LOG_CODE_NOT_FOUND)//if log code not available in the list
         {
            AR_MSG(DBG_MED_PRIO, "dls_set_param_log_code(): log code 0x%p not found in the global list",
                  log_code_info->log_codes[inp_log_code_idx]);
            continue; // if the log code doesn't exists then move on to the next one
         }
         else
         {
            g_dls_log_code[log_code_idx] = 0; // when the log code is found then just remove it by setting it to zero
            AR_MSG(DBG_LOW_PRIO, "dls_set_param_log_code(): log code 0x%p found in the global list at index %d",
                  log_code_info->log_codes[inp_log_code_idx], log_code_idx);
         }
      }
   }

__dls_set_param_log_code:
   return result;
}

/**
  This function will handle the DLS set param commands received from the RTM
  client

  @param[in]     dls_info_ptr    pointer to the DLS service information
  @param[in]     param_id        parameter ID
  @param[in]     payload_ptr     pointer to the parameter payload
  @param[in]     param_size      total parameter payload size

  @param[out]    g_dls_log_code  global log code information

  @return        result          status error code

  @dependencies  None
 */
ar_result_t dls_handle_set_param(dls_t *dls_info_ptr,
                                     uint32_t param_id,
                                     void *payload_ptr,
                                     uint32_t param_size)
{
   ar_result_t result = AR_EOK;

   switch (param_id)
   {
      case DLS_PARAM_ID_CONFIG_BUFFER:
      {
         result = dls_set_param_config_buffer(dls_info_ptr, payload_ptr, param_size);
         if(AR_FAILED(result))
         {
            AR_MSG(DBG_ERROR_PRIO, "dls_handle_apm_set_param: config buffer set param failed."
                  "param_id: 0x%lx result: 0x%lx", param_id, result);
         }
         else
         {
            AR_MSG(DBG_HIGH_PRIO, "dls_handle_apm_set_param: config buffer set param success."
                  " param_id: 0x%lx result: 0x%lx", param_id, result);
         }
         break;
      }
      case DLS_PARAM_ID_ENABLE_DISABLE_LOG_CODE:
      {
         result = dls_set_param_log_code(dls_info_ptr, payload_ptr, param_size);
         if(AR_FAILED(result))
         {
            AR_MSG(DBG_ERROR_PRIO, "dls_handle_apm_set_param: log code set param failed."
                  " param_id: 0x%lx result: 0x%lx", param_id, result);
         }
         else
         {
            AR_MSG(DBG_HIGH_PRIO, "dls_handle_apm_set_param: log code set param success."
                  " param_id: 0x%lx result: 0x%lx", param_id, result);
         }
         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO, "dls_handle_apm_set_param: unsupported param id is received."
               " param_id: 0x%lx result: 0x%lx", param_id, result);
         result = AR_EUNSUPPORTED;
         break;
      }
   }

   return result;
}

/**
  This function will handle the APM_SET_CFG_CMD received from the RTM client
  to set the parameters

  @param[in]     dls_info_ptr    pointer to the DLS service information
  @param[in]     gpr_pkt_ptr     pointer ot the GPR packet

  @return        result          status error code

  @dependencies  None
 */
ar_result_t dls_handle_set_cfg_cmd(dls_t *dls_info_ptr,
                                       gpr_packet_t *gpr_pkt_ptr)
{
   ar_result_t result = AR_EOK;

   // set config payload from the gpr packets
   int8_t * payload_ptr = NULL;
   uint32_t payload_len = 0;
   if (AR_EOK != (result = dls_get_set_cmd_param_payload(gpr_pkt_ptr,
                                                          sizeof(apm_module_param_data_t), // min length of the payload
                                                          (void **)&payload_ptr,
                                                          &payload_len)))
   {
      AR_MSG(DBG_ERROR_PRIO, "DLS: failed to get the param payload");
      return result;
   }

   uint32_t payload_len_counter = payload_len;
   while (payload_len_counter >= sizeof(apm_module_param_data_t))
   {
      apm_module_param_data_t *param_data_hdr_ptr = (apm_module_param_data_t *)payload_ptr;
      void *                   param_ptr          = (void *)(param_data_hdr_ptr + 1);

      AR_MSG(DBG_HIGH_PRIO,
             "DLS: Received set param_id: 0x%lx param_size: %lu ",
             param_data_hdr_ptr->param_id,
             param_data_hdr_ptr->param_size);

      result = dls_handle_set_param(dls_info_ptr, param_data_hdr_ptr->param_id, param_ptr, param_data_hdr_ptr->param_size);

      // break and return only for badparam, else continue
      if (AR_EBADPARAM == result)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "DLS: Failed to set param_id: 0x%lx param_size: %lu  result: 0x%lx",
                param_data_hdr_ptr->param_id,
                param_data_hdr_ptr->param_size,
                result);
         return result;
      }

      AR_MSG(DBG_HIGH_PRIO,
             "DLS: set param_id: 0x%lx param_size: %lu  result: 0x%lx",
             param_data_hdr_ptr->param_id,
             param_data_hdr_ptr->param_size,
             result);

      // move to next param header and adjust pointer as per alignment requirements.
      uint32_t cur_param_size = (sizeof(apm_module_param_data_t) + ALIGN_8_BYTES(param_data_hdr_ptr->param_size));
      payload_ptr += cur_param_size;
      payload_len_counter = (payload_len_counter > cur_param_size) ? (payload_len_counter - cur_param_size) : 0;
   }

   return result;
}
