#ifndef GEN_CNTR_CMN_SH_MEM_EP_H
#define GEN_CNTR_CMN_SH_MEM_EP_H
/**
 * \file gen_cntr_cmn_sh_mem_ep.h
 * \brief
 *     This file contains utility functions for offloading
 *  
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_cntr.h"
#include "gen_cntr_cmn_utils.h"
#include "gen_topo.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

ar_result_t gen_cntr_shmem_cmn_process_and_apply_peer_client_property_configuration(cu_base_t *   base_ptr,
                                                                                    spf_handle_t *dst_handle_ptr,
                                                                                    int8_t *      payload_ptr,
                                                                                    uint32_t      param_size);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef GEN_CNTR_RD_SH_MEM_EP_H
