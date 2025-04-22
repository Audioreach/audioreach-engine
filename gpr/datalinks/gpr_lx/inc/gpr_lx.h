/*
 * gpr_lx.h
 *
:wq * This file has implementation platform wrapper for the GPR datalink layer
 *
 * Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gpr_comdef.h"
#include "ipc_dl_api.h"

/******************************************************************************
 * Defines                                                                    *
 *****************************************************************************/
/*IPC datalink init function called from gpr layer for glink*/
GPR_INTERNAL uint32_t ipc_dl_lx_init(uint32_t                 src_domain_id,
                                        uint32_t                 dest_domain_id,
                                        const gpr_to_ipc_vtbl_t *p_gpr_to_ipc_vtbl,
                                        ipc_to_gpr_vtbl_t **     pp_ipc_to_gpr_vtbl);

/*IPC datalink de-init function called from gpr layer for glink*/
GPR_INTERNAL uint32_t ipc_dl_lx_deinit (uint32_t src_domain_id, uint32_t dest_domain_id);
