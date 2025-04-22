#ifndef _GPR_SESSION_H_
#define _GPR_SESSION_H_

/**
 * \file gpr_session.h
 * \brief
 *  	This file contains GPR drv session APIs
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_osal_mem_op.h"
#include "ar_osal_mutex.h"
#include "ar_osal_heap.h"
#include "ar_guids.h"
#include "gpr_api_i.h"
#include "gpr_msg_if.h"
#include "gpr_api_inline.h"
#include "ar_msg.h"

//#define SESSION_ARRAY_SIZE 200
#define SESSION_HASH(RANGE, x) ((x) % (RANGE - 1)) // always a prime number

/* Forward declaration for the gpr_module_entry_t structure. */
typedef struct gpr_module_entry_t gpr_module_entry_t;

/* Structure for a session for each (domain id, port) set*/
struct gpr_module_entry_t
{
   gpr_callback_fn_t callback_fn;
   void             *callback_data;
};

/* Forward declaration for the gpr_module_node_t structure. */
typedef struct gpr_module_node_t gpr_module_node_t;

/*Structure for each linked list node to store callback function and
 * corresponding argument*/
struct gpr_module_node_t
{
   uint32_t           my_module_port;
   gpr_module_entry_t session;
   gpr_module_node_t *next;
};

/* Structures to store the registered module nodes, heap index provided will be used to
   allocate the nodes. Client must allocate memory for the cb_list[] */
struct gpr_module_node_list_t
{
   gpr_heap_index_t   heap_index;       // heap index from which module nodes need to be allocated
   uint32_t           max_cb_list_size; // max size of the cb_list array
   gpr_module_node_t *cb_list[1];       // array containing the module node pointers.
};
typedef struct gpr_module_node_list_t gpr_module_node_list_t;

GPR_INTERNAL uint32_t gpr_init_session(uint32_t                my_module_port,
                                       gpr_module_entry_t    **ret_entry,
                                       gpr_module_node_list_t *list_handle);

GPR_INTERNAL uint32_t gpr_get_session(uint32_t                my_module_port,
                                      gpr_module_entry_t    **ret_entry,
                                      gpr_module_node_list_t *list_handle);

GPR_INTERNAL uint32_t gpr_deinit_session(uint32_t my_module_port, gpr_module_node_list_t *list_handle);

#endif /* _GPR_SESSION_H_ */
