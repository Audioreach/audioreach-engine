/**
 * \file spf_svc_calib.h
 * \brief
 *     This file contains structures and message ID's for communication between
 *  spf services.
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*===========================================================================
NOTE: The @brief description above does not appear in the PDF.
      The description that displays in the PDF is located in the
      spf_mainpage.dox file. Contact Tech Pubs for assistance.
===========================================================================*/

#ifndef _SPF_SVC_CALIB_H_
#define _SPF_SVC_CALIB_H_

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "spf_utils.h"
#include "gpr_api.h"
#include "gpr_api_inline.h"
#include "amdb_api.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/** @ingroup spf_svc_alloc_get_cfg_payload
  Called by spf_svc_get_cmd_payload_addr for in band get param case. Here separate packet is allocated and
  returned back along with the ptr to the payload

  @param[in] log_id                 log id for printing
  @param[in] gpr_pkt_ptr            Pointer to the incoming gpr packet
  @param[in] gpr_rsp_pkt_pptr       Double pointer to the gpr response packet, which is allocated here
  @param[in] cmd_payload_size       Size of the payload to be allocated
  @param[in] cmd_rsp_payload_pptr   Double pointer to the payload, which is allocated here as well
  @param[in] gpr_cmd_rsp_ptr        Pointer to the struct of type spf_msg_t. NULL can be sent if you don't want to save
                                    resulting gpr_rsp_pkt_pptr as its payload
  @return
  Result

  @dependencies
  None.
*/

ar_result_t spf_svc_alloc_rsp_payload(uint32_t       log_id,
                                      gpr_packet_t * gpr_pkt_ptr,
                                      gpr_packet_t **gpr_rsp_pkt_pptr,
                                      uint32_t       cmd_payload_size,
                                      uint8_t **     cmd_rsp_payload_pptr,
                                      spf_msg_t *    gpr_cmd_rsp_ptr);

/** @ingroup spf_svc_alloc_get_cfg_payload
  Called by service (apm or containers) for both in band get and out of band case. Can be called to get the payload
  pointer from the incoming packet. In case of in band get_param, a new gpr packet with payload same as incoming
  gpr packet will be allocated along with apm_cmd_rsp_get_cfg_t in the top of the payload.

  For other in band cases, no allocation is done
  For out of band cases, virtual memory is obtained from the physical memory and mem_map_handle

  @param[in] log_id                 log id for printing
  @param[in] gpr_pkt_ptr            Pointer to the incoming gpr packet
  @param[in] gpr_rsp_pkt_pptr       Double pointer to the gpr response packet, which is allocated here
  @param[in] cmd_payload_size       Size of the payload to be allocated
  @param[in] cmd_rsp_payload_pptr   Double pointer to the payload, which is allocated here as well
  @param[in] gpr_cmd_rsp_ptr        Pointer to the struct of type spf_msg_t. NULL can be sent if you don't want to save
                                    resulting gpr_rsp_pkt_pptr as its payload
  @param[in] memory_map_client      memory map client for mapping physical memory to virtual memory
  @return
  Result

  @dependencies
  None.
*/
ar_result_t spf_svc_get_cmd_payload_addr(uint32_t       log_id,
                                         gpr_packet_t * gpr_pkt_ptr,
                                         gpr_packet_t **gpr_rsp_pkt_pptr,
                                         uint8_t **     cmd_payload_pptr,
                                         uint32_t *     byte_aligned_size_ptr,
                                         spf_msg_t *    gpr_cmd_rsp_ptr,
                                         uint32_t       memory_map_client);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif // #ifndef _SPF_SVC_CALIB_H_
