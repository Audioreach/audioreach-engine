#ifndef _DLS_I_H_
#define _DLS_I_H_
/**
 * \file dls_i.h
 * \brief
 *  	This file contains private declarations of the DLS service
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* ----------------------------------------------------------------------------
 * Include Files
 * ------------------------------------------------------------------------- */
#include "posal.h"
#include "gpr_api_inline.h"
#include "gpr_packet.h"
#include "gpr_api.h"
#include "gpr_ids_domains.h"
#include "dls_api.h"
#include "spf_svc_calib.h"
#include "apm.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/* ----------------------------------------------------------------------------
 * Global Definitions
 * ------------------------------------------------------------------------- */
#define DLS_MAX_CMD_Q_ELEMENTS   8
#define DLS_CMD_Q_BYTE_SIZE      (POSAL_QUEUE_GET_REQUIRED_BYTES(DLS_MAX_CMD_Q_ELEMENTS))

#define DLS_MAX_RSP_Q_ELEMENTS   16
#define DLS_RSP_Q_BYTE_SIZE      (POSAL_QUEUE_GET_REQUIRED_BYTES(DLS_MAX_RSP_Q_ELEMENTS))

#define DLS_DEFAULT_MAX_LOG_PKT_SIZE 3360
#define DLS_SET_BUF_SIZE_TO_DEFAULT  -1
/** Maximum number of log codes that can be enabled or disabled */
#define DLS_MAX_NUM_LOG_CODES    32
#define DLS_LOG_CODE_NOT_FOUND   -1

#define MAX_DLS_EVENT_CLIENTS    4

#define ALIGN_8_BYTES(a) ((a + 7) & (0xFFFFFFF8))

#define IS_ALIGN_8_BYTE(a) (!((a) & (uint32_t)0x7))

#define IS_ALIGN_4_BYTE(a) (!((a) & (uint32_t)0x3))

#define GET_MSW_FROM_64BIT_WORD(word64) ((word64 & 0xFFFFFFFF00000000) >> 32)
#define GET_LSW_FROM_64BIT_WORD(word64) (word64 & 0x00000000FFFFFFFF)

#define  DLS_SELF_GPR_PORT   DLS_MODULE_INSTANCE_ID

/* ----------------------------------------------------------------------------
 * Structure Declarations
 * ------------------------------------------------------------------------- */
/* DLS Client info */
typedef struct dls_event_client_info_t
{
   uint32_t event_id;

   // gpr addr
   uint32_t gpr_domain;
   uint32_t gpr_port;
   uint32_t gpr_client_token;

} dls_event_client_info_t;

/** DLS Module Struct */
typedef struct dls_t
{
   spf_handle_t dls_handle;
   /**< DLS thread handle */

   spf_cmd_handle_t dls_cmd_handle;
   /**< DLS thread command handle */

   dls_event_client_info_t    client_info[MAX_DLS_EVENT_CLIENTS];
   /**< table to cache RTM register event info */

   posal_mutex_t   buf_acquire_mutex;
   /**< Mutex to acquire the available buffer */

   posal_mutex_t   buf_commit_mutex;
   /**< Mutex to commit the logged buffer */

   posal_mutex_t   buf_return_mutex;
   /**< Mutex to clean the buffer */

   uint32_t dls_cmd_q_wait_mask;
   /**< Signal wait mask for cmd Q */

   uint32_t dls_rsp_q_wait_mask;
   /**< Signal wait mask for rsp Q */

   posal_channel_t channel_ptr;
   /**< Mask for Q's owned by this obj */

   posal_signal_t kill_signal_ptr;
   /**< Signal to destroy HRM module thread */

   posal_queue_t *p_dls_cmd_q;
   /**< DLS Command queue */

   posal_queue_t *p_dls_rsp_q;
   /**< DLS Command queue */

   uint32_t gpr_handle;
   /**< GPR handle */
}dls_t;

/* ----------------------------------------------------------------------------
 *  Function Declarations
 * ------------------------------------------------------------------------- */
uint32_t dls_gpr_call_back_f(gpr_packet_t *gpr_pkt_ptr,
                             void *cb_ctxt_ptr);

dls_event_client_info_t *dls_get_event_client_info(uint32_t event_id);

ar_result_t dls_handle_set_cfg_cmd(dls_t *dls_info_ptr,
                                       gpr_packet_t *gpr_pkt_ptr);

ar_result_t dls_is_log_code_exists(uint32_t log_code);

static inline gpr_packet_t *dls_alloc_gpr_pkt(uint32_t dest_domain_id,
                                              uint32_t dest_port,
                                              uint32_t token,
                                              uint32_t opcode,
                                              uint32_t payload_size)
{
   gpr_packet_t *         pkt_ptr = NULL;
   gpr_cmd_alloc_ext_v2_t args;
   uint32_t               host_domain_id     = 0xFFFFFFFF;
   __gpr_cmd_get_host_domain_id(&host_domain_id);

   /* Currently GPR msgs are sent from ASPS only in non-island mode. So use default heap*/
   args.heap_index    = GPR_HEAP_INDEX_DEFAULT;
   args.src_domain_id = host_domain_id;
   args.src_port      = DLS_SELF_GPR_PORT;
   args.dst_domain_id = dest_domain_id;
   args.dst_port      = dest_port;
   args.token         = token;
   args.opcode        = opcode;
   args.payload_size  = payload_size;
   args.client_data   = 0;
   args.ret_packet    = &pkt_ptr;
   ar_result_t result = __gpr_cmd_alloc_ext_v2(&args);
   if (NULL == pkt_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Failed to allocate gpr packet opcode 0x%lX payload_size %lu result 0x%lx",
             opcode,
             payload_size,
             result);
   }

   return pkt_ptr;
}

static inline uint32_t dls_gpr_async_send_packet(gpr_packet_t *pkt_ptr)
{
  ar_result_t rc = __gpr_cmd_async_send(pkt_ptr);
  if(rc)
  {
     __gpr_cmd_free(pkt_ptr);
  }

  return rc;
}

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* _DLS_I_H_ */
