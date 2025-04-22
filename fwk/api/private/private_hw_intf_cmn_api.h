#ifndef _PRIVATE_HW_INTF_CMN_API_H_
#define _PRIVATE_HW_INTF_CMN_API_H_

/**
 * \file private_hw_intf_cmn_api.h
 * \brief
 *     This file contains declarations of the private API for hw interfaces.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "ar_defs.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

/** Param ID to configure Signal Miss insatnces for HW EP to debug signal miss handling logic on SIM*/
#define PARAM_ID_HW_EP_SIGNAL_MISS_CFG 0x0800131A

/** Configures signal miss parameters for debubging signal miss handling logic */

#include "spf_begin_pack.h"
/** Payload for parameter PARAM_ID_HW_EP_SIGNAL_MISS_CFG */
struct param_id_hw_ep_signal_miss_cfg_t
{

   uint32_t is_periodic_signal_miss;
   /**<  This flag is to be set for periodic signal miss testing
                     periodic signal miss enable =  1
                     periodic signal miss disable = 0
   */

   uint16_t intr_count_post;
   /**< Interrupt count value when a post proc. signal miss needs to occur
        default     0
   */
   uint16_t intr_count_pre;
   /**< Interrupt count value when a pre proc. signal miss needs to occur }
        default     0
   */
}
#include "spf_end_pack.h"
;
/* Structure type def for above payload. */
typedef struct param_id_hw_ep_signal_miss_cfg_t param_id_hw_ep_signal_miss_cfg_t;

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* #ifdef _PRIVATE_HW_INTF_CMN_API_H_ */