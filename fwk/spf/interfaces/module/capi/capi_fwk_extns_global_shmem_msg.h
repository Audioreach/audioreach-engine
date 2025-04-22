#ifndef _CAPI_FWK_EXTNS_GLOBAL_SHMEM_MSG_H_
#define _CAPI_FWK_EXTNS_GLOBAL_SHMEM_MSG_H_

/**
 *   \file capi_fwk_extns_global_shmem_msg.h
 *   \brief
 *        This file contains global shared memory MSG extension.
 *
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_types.h"

/** @addtogroup capi_fwk_ext_global_shmem_msg
@{ */

/*==============================================================================
     Constants
==============================================================================*/

/** Unique identifier of the framework extension.

 This extension supports the following param and parameter ID:
 - #FWK_EXTN_PARAM_ID_SET_GLOBAL_SHMEM_MSG
 - #FWK_EXTN_PARAM_ID_SET_GLOBAL_SHMEM_CB_FUNC
*/
#define FWK_EXTN_GLOBAL_SHMEM_MSG 0x0A001BB1

/*==============================================================================
   Constants
==============================================================================*/
/** ID of the custom param used to set a callback function in module to manage global shared memory.

 @msgpayload{capi_param_set_global_shmem_cb_func_t}
 @table{weak__capi__param__set__global__shmem__cb__func__t}
 */
#define FWK_EXTN_PARAM_ID_SET_GLOBAL_SHMEM_CB_FUNC 0x0A001BB2

/** ID of the custom param used to send the global shared memory message to the module.

   Clients can manage the global shared memory using following commands.
   #APM_CMD_GLOBAL_SHARED_MEM_MAP_REGIONS
   #APM_CMD_GLOBAL_SHARED_MEM_UNMAP_REGIONS

   Client can use following command to send the out of band message in the global shared memory to the module.
   #AR_SPF_MSG_GLOBAL_SH_MEM

   As part of #AR_SPF_MSG_GLOBAL_SH_MEM command handling. Framework maps the out of band physical memory to virtual
   address and sends it to the module using this param.

 @msgpayload{capi_param_set_global_shmem_msg_t}
 @table{weak__capi__param__set__global__shmem__msg__t}
 */
#define FWK_EXTN_PARAM_ID_SET_GLOBAL_SHMEM_MSG 0x0A001BB3

/*==============================================================================
   Type definitions
==============================================================================*/

/** Types of module's commands on global_shmem.
 */
typedef enum fwk_extn_global_shmem_cb_cmd_t
{
   FWK_EXTN_INC_SHMEM_REF_COUNT = 0,
   /**< Increase the shared memory usage ref count.
    * This indicates that the module is going to use the shmem outside the context of
    * #FWK_EXTN_PARAM_ID_SET_GLOBAL_SHMEM_MSG. No buffer required for this command.*/

   FWK_EXTN_DEC_SHMEM_REF_COUNT = 1,
   /**< Decrease the shared memory usage ref count indicating module is no longer going to use it.
    * No buffer required for this command.*/

} /** @cond */ fwk_extn_global_shmem_cb_cmd_t /** @endcond */;

/**
 Callback function to manage global shared memory from module.


 @param[in] context_ptr     Pointer to the context given by the container in
                             #FWK_EXTN_PARAM_ID_SET_GLOBAL_SHMEM_CB_FUNC.
 @param[in] shmem_id        Unique ID to refer the global shared memory map handle.
 @param[in] cmd             command on global shared memory.
 @param[in] buffer          capi buffer specific to cmd
 */
typedef capi_err_t (*fwk_extn_global_shmem_cb_fn)(void                          *context_ptr,
                                                  uint32_t                       shmem_id,
                                                  fwk_extn_global_shmem_cb_cmd_t cmd,
                                                  capi_buf_t                     buffer);

typedef struct capi_param_set_global_shmem_cb_func_t capi_param_set_global_shmem_cb_func_t;

/** @weakgroup capi_param_set_global_shmem_cb_func_t
 @{ */
struct capi_param_set_global_shmem_cb_func_t
{
   fwk_extn_global_shmem_cb_fn cb_func;
   /* callback function for module to manage global_shmem*/

   void *context_ptr;
   /* callback context ptr. */
};
/** @} */ /* end_weakgroup capi_param_set_global_shmem_cb_func_t */

typedef struct capi_param_set_global_shmem_msg_t capi_param_set_global_shmem_msg_t;

/** @weakgroup capi_param_set_global_shmem_msg_t
@{ */
struct capi_param_set_global_shmem_msg_t
{
   void* cmd_header_addr;
   /* Address of the cmd #AR_SPF_MSG_GLOBAL_SH_MEM sent from the client.
    * This is of type ar_spf_msg_global_sh_mem_t.*/

   void* payload_virtual_addr;
   /*Virtual address of the out of band payload.
    * Cache flush/invalidate has to be managed by the module.*/
};
/** @} */ /* end_weakgroup capi_param_set_global_shmem_msg_t */

/** @} */ /* end_addtogroup capi_fwk_ext_global_shmem_msg */

#endif /* _CAPI_FWK_EXTNS_GLOBAL_SHMEM_MSG_H_ */
