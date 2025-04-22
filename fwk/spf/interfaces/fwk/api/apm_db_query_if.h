#ifndef _APM_DB_QUERY_IF_H_
#define _APM_DB_QUERY_IF_H_

/**
 * \file apm_db_query_if.h
 *
 * \brief
 *     This file defines APM to container functions.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "posal.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

typedef struct spf_handle_t spf_handle_t;

/**********************************************************************************************************************/
/**
  @ingroup APM_PARAM_ID_SET_CNTR_HANDLES
  This param is used to by APM to send the instance id to
  spf handle mapping for containers when it had a new information.
  For ex: When a new graph open comes to APM, it will have info
  about the new containers or modules.

  @gpr_hdr_fields
  Opcode -- APM_PARAM_ID_SET_CNTR_HANDLES

  @msg_payload
  param_id_cntr_instance_handles_t

  @return
  None

  @dependencies
  None
 */
#define APM_PARAM_ID_SET_CNTR_HANDLES 0x08001315
/**********************************************************************************************************************/
/**
  @ingroup APM_PARAM_ID_RESET_CNTR_HANDLES
  This param is used to by APM to send the instance id to
  of the containers or modules which are being closed.
  For ex: When a new graph open comes to APM, it will have info
  about the new containers or modules.

  @gpr_hdr_fields
  Opcode -- APM_PARAM_ID_SET_CNTR_HANDLES

  @msg_payload
  param_id_cntr_instance_handles_t

  @return
  None

  @dependencies
  None
 */
#define APM_PARAM_ID_RESET_CNTR_HANDLES 0x0800134B
/**********************************************************************************************************************/

/**
  @ingroup APM_PARAM_ID_GET_CNTR_HANDLES
  This param is used to request container instance handles from APM from IRM

  @gpr_hdr_fields
  Opcode -- APM_PARAM_ID_GET_CNTR_HANDLES

  @msg_payload
  param_id_cntr_instance_handles_t

  @return
  None

  @dependencies
  None
 */
#define APM_PARAM_ID_GET_CNTR_HANDLES 0x0800135B
/**********************************************************************************************************************/

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/** Following payload can be used for
   APM_PARAM_ID_SET_CNTR_HANDLES - APM uses this to send handle info of containers and modules to IRM during graph open
   APM_PARAM_ID_RESET_CNTR_HANDLES- APM sent containers and modules being to IRM during graph close
   APM_PARAM_ID_GET_CNTR_HANDLES - IRM uses this to get container and module handle info from IRM
 */
struct param_id_cntr_instance_handles_t
{
   uint32_t num_instance_ids;
   /**< Number of instance ids */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct param_id_cntr_instance_handles_t param_id_cntr_instance_handles_t;

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"


struct param_id_cntr_instance_handles_payload_t
{
   uint32_t container_instance_id;
   /**< Container instance ID, if this is 0, then the
    *   query is for container*/

   uint32_t module_instance_id;
   /**< Instance id of a module, if it is 0, then the
    *   query is for container */

   spf_handle_t *handle_ptr;
   /**< Spf handle corresponding to the instance_id */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct param_id_cntr_instance_handles_payload_t param_id_cntr_instance_handles_payload_t;

/**
  @ingroup APM_PARAM_ID_GET_ALL_CNTR_HANDLES
  This param is used to request container instance handles from APM from IRM

  @gpr_hdr_fields
  Opcode -- APM_PARAM_ID_GET_ALL_CNTR_HANDLES

  @msg_payload
  param_id_cntr_instance_handles_out_of_band_t

  @return
  None

  @dependencies
  None
 */
#define APM_PARAM_ID_GET_ALL_CNTR_HANDLES 0x080014EB 
/**********************************************************************************************************************/

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct param_id_cntr_instance_handles_out_of_band_t
{
   param_id_cntr_instance_handles_t* payload_ptr; 
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct param_id_cntr_instance_handles_out_of_band_t param_id_cntr_instance_handles_out_of_band_t;

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef _APM_DB_QUERY_IF_H_
