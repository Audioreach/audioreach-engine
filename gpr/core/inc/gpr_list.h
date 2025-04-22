#ifndef __GPR_LIST_H__
#define __GPR_LIST_H__

/**
 * \file gpr_list.h
 * \brief
 *  	This file contains GPR list APIs
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gpr_comdef.h"
#include "ar_osal_error.h"
#include "ar_msg.h"
#include "ar_types.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

typedef struct gpr_list_node_t gpr_list_node_t;
typedef struct gpr_list_t      gpr_list_t;

typedef void (*gpr_list_lock_enter_fn_t)(void);
typedef void (*gpr_list_lock_leave_fn_t)(void);

struct gpr_list_node_t
{
   gpr_list_t *list;
   /**< Indicates the last known list for which this node belonged. */
   gpr_list_node_t *prev;
   /**< The previous node. */
   gpr_list_node_t *next;
   /**< The next node. */
};

struct gpr_list_t
{
   gpr_list_node_t dummy;
   /**< A special dummy node to implement the NULL object pattern. */
   uint32_t size;
   /**< The number of nodes linked to this linked list. */
   gpr_list_lock_enter_fn_t lock_fn;
   /**< The user supplied lock implementation function. */
   gpr_list_lock_leave_fn_t unlock_fn;
   /**< The user supplied unlock implementation function. */
   bool_t en_secure;
   /**< Enable secure linked list flag. */
};

GPR_EXTERNAL uint32_t gpr_list_init(gpr_list_t              *list,
                                    gpr_list_lock_enter_fn_t lock_fn,
                                    gpr_list_lock_leave_fn_t unlock_fn);

GPR_EXTERNAL uint32_t gpr_list_init_v2(gpr_list_t              *list,
                                       gpr_list_lock_enter_fn_t lock_fn,
                                       gpr_list_lock_leave_fn_t unlock_fn);

GPR_EXTERNAL bool_t gpr_list_raw_is_empty(gpr_list_t *list);

GPR_EXTERNAL uint32_t gpr_list_init_node(gpr_list_node_t *node);

GPR_EXTERNAL uint32_t gpr_list_add_tail(gpr_list_t *list, gpr_list_node_t *node);

GPR_EXTERNAL uint32_t gpr_list_remove_head(gpr_list_t *list, gpr_list_node_t **ret_node);

GPR_EXTERNAL uint32_t gpr_list_get_head(gpr_list_t *list, gpr_list_node_t **ret_node);

GPR_EXTERNAL uint32_t gpr_list_delete(gpr_list_t *list, gpr_list_node_t *node);

GPR_EXTERNAL uint32_t gpr_list_clear(gpr_list_t *list);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* __GPR_LIST_H__ */
