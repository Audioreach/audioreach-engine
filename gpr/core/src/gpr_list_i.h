#ifndef __GPR_LIST_I_H__
#define __GPR_LIST_I_H__

/**
 * \file gpr_list_i.h
 * \brief
 *  	This file contains GPR list internal APIs
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gpr_list.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

uint32_t gpr_list_raw_assert_node_is_linked(gpr_list_node_t *node);

void gpr_list_raw_delete_node(gpr_list_node_t *node);

uint32_t gpr_list_raw_insert_node_before(gpr_list_t *list, gpr_list_node_t *ref_node, gpr_list_node_t *new_node);

uint32_t gpr_list_raw_assert_secure_node_is_unlinked(gpr_list_t *list, gpr_list_node_t *node);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* __GPR_LIST_I_H__ */
