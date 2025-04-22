/**
 *   \file rd_sh_mem_ep_ext_api.h
 *   \brief
 *        This file contains Shared mem module APIs extension for Read shared memory Endpoint module
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef RD_SH_MEM_EP_EXT_API_H_
#define RD_SH_MEM_EP_EXT_API_H_

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "ar_defs.h"
#include "sh_mem_ep_metadata_api.h"
#include "ar_guids.h"
#include "rd_sh_mem_ep_api.h"
#include "offload_def_api.h"

/**
  @h2xml_title1          {Read Shared Memory End Point Module API Extension}
  @h2xml_title_agile_rev {Read Shared Memory End Point Module API Extension}
  @h2xml_title_date      {August 5, 2019}
*/

/**
   @h2xmlx_xmlNumberFormat {int}
*/

/*==============================================================================
   Param ID
==============================================================================*/
/**
   Configuration of the Read Shared Memory Module.
*/
#define PARAM_ID_RD_EP_CLIENT_CFG 0x0800114D
/*==============================================================================
   Param structure definitions
==============================================================================*/

/** @h2xmlp_parameter   {"PARAM_ID_RD_EP_CLIENT_CFG", PARAM_ID_RD_EP_CLIENT_CFG}
    @h2xmlp_description {Parameter for setting the client configuration to the
                   read shared memory end point module.\n }
    @h2xmlp_toolPolicy  {Calibration} */
#include "spf_begin_pack.h"
struct param_id_rd_ep_client_cfg_t
{
   uint32_t client_id;
   /**< @h2xmle_description {Specifies the information about the client
                       0 indicates external client (HLOS)
                       1 indicates internal client (OLC) }
        @h2xmle_default     { 0 - HLOS}
        @h2xmle_rangelist   { 0 - HLOS,
                              1 - OLC }
        @h2xmle_policy      {advanced}
   */
   uint32_t gpr_port_id;
   /**< @h2xmle_description {Specifies the ID used for registration with GPR by the specified Client
                             For CLIENT_ID_OLC this would be the container ID of OLC}
        @h2xmle_default     {0}
        @h2xmle_rangelist   {0 - 0x7FFFFFFF}
        @h2xmle_policy      {advanced}
   */
}
#include "spf_end_pack.h"
;

/* Type definition for the above structure. */
typedef struct param_id_rd_ep_client_cfg_t param_id_rd_ep_client_cfg_t;


/**
   Configuration of the metadata that is rendered in Master to be set to Satellite RD EP module
*/
#define PARAM_ID_RD_EP_MD_RENDERED_CFG 0x08001A99
/*==============================================================================
   Param structure definitions
==============================================================================*/

/** @h2xmlp_parameter   {"PARAM_ID_RD_EP_MD_RENDERED_CFG", PARAM_ID_RD_EP_MD_RENDERED_CFG}
    @h2xmlp_description {Parameter for setting the Rendered Metadata identifier in master SPF and
                         release it from the cached list of Satellite RD_EP module.\n }
    @h2xmlp_toolPolicy  {Calibration} */
#include "spf_begin_pack.h"
struct param_id_rd_ep_md_rendered_cfg_t
{
   uint32_t md_node_address_lsw;
   /**< @h2xmle_description {specifies the lower 32 bit address of the metadata node pointer }
        @h2xmle_default     { 0 }
        @h2xmle_policy      {advanced}
   */
   uint32_t md_node_address_msw;
   /**< @h2xmle_description {specifies the higher 32 bit address of the metadata node pointer}
        @h2xmle_default     {0}
        @h2xmle_policy      {advanced}
   */
   uint32_t md_rendered_port_id;
   /**< @h2xmle_description {specifies the module instance id which was raised the MD tracking event}
        @h2xmle_default     {0}
        @h2xmle_policy      {advanced}
   */
   uint32_t md_rendered_domain_id;
   /**< @h2xmle_description {specifies the domain id of the module instance id which was raised the MD tracking event}
        @h2xmle_default     {0}
        @h2xmle_policy      {advanced}
   */
   uint32_t is_last_instance;
   /**< @h2xmle_description {specifies if this is the last instance of the tracking metadata }
        @h2xmle_default     {0}
        @h2xmle_policy      {advanced}
        @h2xmle_rangeList     { "FALSE" = 0,
                      	  	    "TRUE"  = 1}
   */
   uint32_t is_md_dropped;
   /**< @h2xmle_description {specifies if this is the metadata is dropped or consumed by the sink module}
        @h2xmle_default     {0}
        @h2xmle_policy      {advanced}
        @h2xmle_rangeList     { "CONSUME" = 0,
                      	  	    "DROPPED"  = 1}
   */
}
#include "spf_end_pack.h"
;

/* Type definition for the above structure. */
typedef struct param_id_rd_ep_md_rendered_cfg_t param_id_rd_ep_md_rendered_cfg_t;

#endif // RD_SH_MEM_EP_EXT_API_H_
