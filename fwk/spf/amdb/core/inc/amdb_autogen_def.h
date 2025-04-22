#ifndef _AMDB_AUTOGEN_DEF_H_
#define _AMDB_AUTOGEN_DEF_H_
/**
 * \file amdb_autogen_def.h
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "posal.h"
#include "capi.h"
#include "spf_hashtable.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

// These structs are used in json parsing
typedef struct amdb_static_capi_module_t
{
   uint32_t mtype;
   uint32_t mid;

   capi_get_static_properties_f get_static_prop_fn;
   capi_init_f                  init_fn;

} amdb_static_capi_module_t;

typedef struct amdb_dynamic_capi_module_t
{
   uint32_t mtype;
   uint32_t mid;
   uint32_t revision;

   const char *filename;
   const char *tag;

} amdb_dynamic_capi_module_t;

typedef struct amdb_module_id_t
{
   uint32_t mtype;
   uint32_t mid;
   // revision is top available one
} amdb_module_id_t;

typedef struct amdb_context_t
{
   uint8_t *payload_ptr;
   uint8_t *gpr_rsp_pkt_ptr;
   uint8_t *gpr_pkt_ptr;
} amdb_context_t;

typedef struct amdb_load_handle_t
{
   uint8_t *       module_handle_ptr;
   spf_hash_node_t hn;
} amdb_load_handle_t;
#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* _AMDB_AUTOGEN_DEF_H_ */
