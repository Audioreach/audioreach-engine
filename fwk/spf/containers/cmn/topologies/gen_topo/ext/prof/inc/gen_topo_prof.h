#ifndef GEN_TOPO_PROF_H
#define GEN_TOPO_PROF_H
/**
 * \file gen_topo_prof.h
 * \brief
 *     This file contains utility functions profiling
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "topo_utils.h"
#include "ar_defs.h"

// clang-format off

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

typedef struct gen_topo_module_prof_flags_t
{
   uint32_t is_pcycles_enabled: 1;
   /**< Flag to indicate pcycles profiling is enabled */

   uint32_t is_pktcnt_enabled: 1;
   /**< Flag to indicate pktcnt profiling is enabled */

} gen_topo_module_prof_flags_t;

typedef struct gen_topo_module_prof_info_t
{
   uint64_t accum_pcylces;
   /**< processor cycles info */

   uint64_t accum_pktcnt;
   /**< packet count info */

   gen_topo_module_prof_flags_t flags;
   /** Flags to indicate what profiling was enabled */
}gen_topo_module_prof_info_t;

ar_result_t gen_topo_get_prof_info(void *vtopo_ptr, int8_t *param_payload_ptr, uint32_t *param_size_ptr);
void gen_topo_prof_handle_deinit(gen_topo_t *topo_ptr, gen_topo_module_t *module_ptr);

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif /* GEN_TOPO_PROF_H */
