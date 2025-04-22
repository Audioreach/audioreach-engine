/**
 * \file gpr_log_diag.c
 * \brief
 *    This file contains helper functions to log gpr msgs in diag
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/******************************************************************************
 * Includes                                                                    *
 *****************************************************************************/
#include "ar_osal_mem_op.h"
#include "ar_osal_heap.h"
#include "ar_osal_error.h"
#include "ar_osal_timer.h"
#include "ar_guids.h"
#include "ar_msg.h"
#include "gpr_api_i.h"
#include "gpr_api_inline.h"
#include "gpr_log.h"
#include "gpr_msg_if.h"
#include "gpr_ids_domains.h"
#include "gpr_log_i.h"
#include "log.h"
#include "log_codes.h"

#ifdef GPR_DIAG_LOG_CODES
// clang-format off
   // If we are on sim the log codes are not defined in core
   #ifndef SIM
      #if defined GPR_UPD
         #define GPR_ADSP_APPS_LOG_CODE LOG_GPR_ADSP_USERPD_APPS_C
         #define GPR_ADSP_MODEM_LOG_CODE LOG_GPR_ADSP_USERPD_MODEM_C
         #define GPR_MODEM_LOG_CODE LOG_GPR_MODEM_USERPD_C
         #define GPR_SLPI_LOG_CODE LOG_GPR_SLPI_USERPD_C
         #define GPR_CDSP_LOG_CODE LOG_GPR_CDSP_USERPD_C
      #else
         #define GPR_ADSP_APPS_LOG_CODE LOG_GPR_ADSP_APPS_C
         #define GPR_ADSP_MODEM_LOG_CODE LOG_GPR_ADSP_MODEM_C
         #define GPR_MODEM_LOG_CODE LOG_GPR_MODEM_C
         #define GPR_SLPI_LOG_CODE LOG_GPR_SLPI_C
         #define GPR_CDSP_LOG_CODE LOG_GPR_CDSP_C
      #endif
   #else
      // Log codes are not defined on sim core label 
      #define GPR_ADSP_APPS_LOG_CODE 0x1D68
      #define GPR_ADSP_MODEM_LOG_CODE 0x1D6E
   #endif

   // Figure out which generic log code to use based on target
   #if defined modem_proc
      #define GPR_LOG_CODE_GENERIC GPR_MODEM_LOG_CODE
   #elif defined slpi_proc
      #define GPR_LOG_CODE_GENERIC GPR_SLPI_LOG_CODE
   #elif defined cdsp_proc
      #define GPR_LOG_CODE_GENERIC GPR_CDSP_LOG_CODE
   #else
      #define GPR_LOG_CODE_GENERIC GPR_ADSP_APPS_LOG_CODE
   #endif
// clang-format on
#endif

typedef PACK(struct)
{
   log_hdr_type hdr;
   uint32_t     version;
   gpr_packet_t gpr_pkt;
}
gpr_log_diag_log_pkt;

static uint32_t gpr_log_get_diag_log_code(uint32_t src_domain_id, uint32_t dst_domain_id)
{
   uint32_t host_domain_id = 0;
   __gpr_cmd_get_host_domain_id(&host_domain_id);
   if (GPR_IDS_DOMAIN_ID_ADSP_V == host_domain_id)
   {
      if ((src_domain_id == GPR_IDS_DOMAIN_ID_MODEM_V) || (dst_domain_id == GPR_IDS_DOMAIN_ID_MODEM_V))
      {
         return GPR_ADSP_MODEM_LOG_CODE;
      }
      else
      {
         return GPR_ADSP_APPS_LOG_CODE;
      }
   }
   else
   {
      return GPR_LOG_CODE_GENERIC;
   }
}

void gpr_log_diag_packet(gpr_packet_t *packet, uint32_t packet_size)
{
   uint32_t src_domain_id = packet->src_domain_id;
   uint32_t dst_domain_id = packet->dst_domain_id;

   uint32_t log_code = gpr_log_get_diag_log_code(src_domain_id, dst_domain_id);

   gpr_log_diag_log_pkt *diag_pkt_ptr =
      log_alloc(log_code, sizeof(gpr_log_diag_log_pkt) + packet_size - sizeof(gpr_packet_t));
   if (NULL != diag_pkt_ptr)
   {
      ar_mem_cpy(((void *)&diag_pkt_ptr->gpr_pkt), sizeof(gpr_packet_t), packet, sizeof(gpr_packet_t));
      ar_mem_set(((void *)&diag_pkt_ptr->version), 0, sizeof(diag_pkt_ptr->version));
      log_commit(diag_pkt_ptr);
   }
}
