#ifndef SPL_TOPO_MIMO_PROC_STATE_INTF_EXTN_H
#define SPL_TOPO_MIMO_PROC_STATE_INTF_EXTN_H
/**
 * \file spl_topo_mimo_proc_state_intf_extn.h
 * \brief
 *     This file contains function definitions for INTF_EXTN_MIMO_MODULE_PROCESS_STATE

 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_intf_extn_mimo_module_process_state.h"
#include "topo_utils.h"
#include "ar_defs.h"

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

typedef struct spl_topo_t             spl_topo_t;
typedef struct spl_topo_module_t      spl_topo_module_t;
typedef struct spl_topo_input_port_t  spl_topo_input_port_t;
typedef struct spl_topo_output_port_t spl_topo_output_port_t;

ar_result_t spl_topo_intf_extn_mimo_module_process_state_handle_event(spl_topo_t *                      topo_ptr,
                                                                      spl_topo_module_t *               module_ptr,
                                                                      capi_event_data_to_dsp_service_t *event_info_ptr);

bool_t spl_topo_intf_extn_mimo_module_process_state_is_module_bypassable(spl_topo_t *             topo_ptr,
                                                                         spl_topo_module_t *      module_ptr,
                                                                         spl_topo_input_port_t ** ip_port_pptr,
                                                                         spl_topo_output_port_t **ou_port_pptr);

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif /* SPL_TOPO_MIMO_PROC_STATE_INTF_EXTN_H */
