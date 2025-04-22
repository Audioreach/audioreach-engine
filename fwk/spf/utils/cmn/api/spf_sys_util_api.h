/**
 * \file spf_sys_util_api.h
 * \brief
 *      API header for Integrated Resource Monitor (IRM).
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef _SPF_SYS_UTIL_API_H_
#define _SPF_SYS_UTIL_API_H_

/*----------------------------------------------------------------------------------------------------------------------
 Include files
 ---------------------------------------------------------------------------------------------------------------------*/
#include "ar_defs.h"

typedef enum spf_sys_util_status_t
{
   SPF_SYS_UTIL_SSR_STATUS_DOWN = 0,
   SPF_SYS_UTIL_SSR_STATUS_UP   = 1,
   SPF_SSYS_UTIL_SSR_STATUS_UNINIT
} spf_sys_util_status_t;

#define PARAM_ID_SYS_UTIL_SVC_STATUS 0x08001213

/**< payload for PARAM_ID_SYS_UTIL_SVC_STATUS. as always spf_msg_header_t precedes this. */
#include "spf_begin_pack.h"
struct param_id_sys_util_svc_status_t
{
   uint32_t proc_domain_id;
   /*<< process domain id of the service reporting */

   spf_sys_util_status_t status;
   /*<< Service status */
}
#include "spf_end_pack.h"
;
typedef struct param_id_sys_util_svc_status_t param_id_sys_util_svc_status_t;

#define PARAM_ID_SYS_UTIL_CLOSE_ALL 0x08001236

/**< Payload for PARAM_ID_SYS_UTIL_CLOSE_ALL */
#include "spf_begin_pack.h"
struct param_id_sys_util_close_all_t
{
   bool_t is_flush_needed;
   /*<< flag to indicate if command queue flush is needed */

   bool_t is_reset_needed;
   /** Flag to indicate if reset is needed.
       If this flag is enabled, the callee should
       reset itself to the boot up state.*/

   bool_t set_done_signal;
   /*<< Sets close all done signal if required. */

   uint8_t reserved;
}
#include "spf_end_pack.h"
;
typedef struct param_id_sys_util_close_all_t param_id_sys_util_close_all_t;

#define PARAM_ID_SYS_UTIL_KILL  0x08001363    

#endif //_SPF_SYS_UTIL_API_H_
