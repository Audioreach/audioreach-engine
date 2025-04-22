#ifndef GEN_TOPO_DISABLED_MODULE_H
#define GEN_TOPO_DISABLED_MODULE_H
/**
 * \file gen_topo_module_bypass.h
 * \brief
 *     This file contains utility functions for handling disabled modules
 *
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

typedef struct gen_topo_t gen_topo_t;
typedef struct gen_topo_module_t gen_topo_module_t;

ar_result_t gen_topo_capi_set_media_fmt(gen_topo_t *       topo_ptr,
                                        gen_topo_module_t *module_ptr,
                                        topo_media_fmt_t * media_fmt_ptr,
                                        bool_t             is_input_mf,
                                        uint16_t           port_index);

/**
 * this struct stores back-up values of media fmt, kpps etc before bypassing; so that
 * we can enable them back once bypass is disabled.
 * Further, if module raises events when its disabled, then these values are updated.
 *
 */
typedef struct gen_topo_module_bypass_t
{
   topo_media_fmt_t *media_fmt_ptr; /**< Output Port's media format.
					media format pointers are shared with other ports as well, it should not be updated directly.
					read can be done directly.
					to write, first update the media format in a temporary variable (stack) and then update using
					"gen_topo_port_mf_utils_set_media_fmt" function.*/
   uint32_t         kpps;
   uint32_t         algo_delay; //us
   uint32_t         hw_acc_proc_delay; //us
   uint32_t         code_bw;
   uint32_t         data_bw;
   uint32_t         in_thresh_bytes_all_ch;
   uint32_t         out_thresh_bytes_all_ch;
} gen_topo_module_bypass_t;


ar_result_t gen_topo_check_create_bypass_module(gen_topo_t *topo_ptr, gen_topo_module_t *module_ptr);
ar_result_t gen_topo_check_destroy_bypass_module(gen_topo_t *topo_ptr, gen_topo_module_t *module_ptr, bool_t is_module_destroying);


#if defined(__cplusplus)
}
#endif // __cplusplus

#endif /* GEN_TOPO_DISABLED_MODULE_H */
