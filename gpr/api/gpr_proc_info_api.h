#ifndef __GPR_PROC_INFO_H__
#define __GPR_PROC_INFO_H__

/**
 * @file  gpr_packet.h
 * @brief This file contains GPR packet definitions
 *
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/******************************************************************************
 * Includes                                                                    *
 *****************************************************************************/
#include "gpr_comdef.h"
#include "ar_guids.h"
#include "gpr_ids_domains.h"
/** @addtogroup gpr_macros
@{ */
/** @name GPR Packet Definitions
@{ */

/****************************************************************************/
// Header Field Definitions
/****************************************************************************/
#define GPR_PD_SUBSTRING_MAX_SIZE 20
typedef struct gpr_proc_info_t
{
   int32_t proc_id;
   char_t  proc_name[GPR_PD_SUBSTRING_MAX_SIZE];
} gpr_proc_info_t;

/**
  Returns the proc name, if proc id is a input.
  Returns the proc id, if proc name is a input.

  @param[in] gpr_proc_info_t structure


  @return
  #proc_id, proc_name -- When successful.

  @dependencies
  None.
*/
GPR_EXTERNAL uint32_t gpr_get_audio_service_name(char_t *service_name, uint32_t string_size);

GPR_EXTERNAL uint32_t gpr_get_pd_str_from_id(uint32_t proc_domain_id,
                                             char_t * proc_domain_string,
                                             uint32_t proc_domain_string_size);

GPR_EXTERNAL uint32_t gpr_get_pd_id_from_str(uint32_t *proc_domain_id_ptr,
                                             char_t *  proc_domain_string,
                                             uint32_t  proc_domain_string_size);

#endif /* __GPR_PACKET_H__ */
