#ifndef _APM_CONTAINER_DEBUG_IF_H_
#define _APM_CONTAINER_DEBUG_IF_H_

/**
 * \file apm_cntr_debug_if.h
 *
 * \brief
 *     This file defines APM to container functions.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "apm_api.h"
#include "apm_module_api.h"
#include "ar_guids.h"
#include "spf_utils.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/*====================================================================================================================*/
/*====================================================================================================================*/
/**
 *This param ID is used to forward port media format enable value received from #SPF_MSG_CMD_SET_CFG to all conatiners
 *
 Payload: cntr_port_mf_param_data_cfg_t
 */
#define CNTR_PARAM_ID_DATA_PORT_MEDIA_FORMAT 0x08001508

#define DEBUG_INFO_LOG_CODE AR_NON_GUID(0x1E0B)

#define DEBUG_INFO_TAP_ID 0x000000F3

#include "spf_begin_pack.h"

struct cntr_port_mf_param_data_cfg_t
{
   uint32_t enable;

   /** enable/disable sending port media format info */
   /*#< h2xmle_description {pointer pointing to the container id.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */
}
#include "spf_end_pack.h"
;
typedef struct cntr_port_mf_param_data_cfg_t cntr_port_mf_param_data_cfg_t;

/*====================================================================================================================*/
/*====================================================================================================================*/

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* _APM_CONTAINER_DEBUG_IF_H_ */
