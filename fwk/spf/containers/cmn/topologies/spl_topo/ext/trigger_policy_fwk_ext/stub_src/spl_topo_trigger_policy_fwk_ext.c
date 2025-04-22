/**
 * \file spl_topo_trigger_policy.c
 *
 * \brief
 *
 *     Topo 2 functions for managing trigger policy/processing decision stubs.
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spl_topo_trigger_policy_fwk_ext.h"

/* =======================================================================
Public Function Definitions
========================================================================== */

/**
 * Topo layer check that the trigger policy is satisfied for this port.
 */
ar_result_t spl_topo_int_in_port_is_trigger_present(void *  ctx_topo_ptr,
                                                    void *  ctx_in_port_ptr,
                                                    bool_t *is_ext_trigger_not_satisfied_ptr,
                                                    bool_t *has_trigger_ptr)
{
   return FALSE;
}

/**
 * Function returns TRUE if the trigger policy is satisfied for this input port. Different buffer fullness
 * checks are required for framework/topo level checks. For example, consider a configuration where the
 * nominal frame size is 10ms but the external input port's module has a threshold of 1ms. We need the
 * framework check to make sure the buffer is filled to 10ms, but the topo check needs to make sure the
 * buffer is filled to 1ms.
 */
bool_t spl_topo_in_port_is_trigger_present(void *  ctx_topo_ptr,
                                                void *  ctx_in_port_ptr,
                                                bool_t *is_ext_trigger_not_satisfied_ptr,
                                                bool_t  is_internal_check)
{
   return FALSE;
}

/**
 * This is used to determine if a port's trigger policy is satisfied on the output side when the policy is ABSENT.
 * It's just the inverse of is_trigger_present.
 */
bool_t spl_topo_in_port_is_trigger_absent(void *ctx_topo_ptr, void *ctx_in_port_ptr)
{
   return TRUE;
}

/**
 * During the spl_topo_process loop, this function is used to check if setup/adjust is necessary for this input port.
 * This also returns the connected output port/connected external input port.
 * Blocks input if:
 * 1. Media format isn't valid.
 * 2. There's no proper upstream connection.
 * 3. The input port is not started.
 * 4. For external ports, if send_to_topo is FALSE.
 * 5. Nontrigger policy is blocked.
 * 6. Trigger is not present.
 */
bool_t spl_topo_in_port_trigger_blocked(spl_topo_t *             topo_ptr,
                                        spl_topo_input_port_t *  in_port_ptr,
                                        spl_topo_output_port_t **connected_out_port_pptr,
                                        spl_topo_input_port_t ** ext_in_port_pptr)
{
   return FALSE;
}

// This is used to determine if a port's trigger policy is satisfied on the output side.
bool_t spl_topo_out_port_is_trigger_present(void *  ctx_topo_ptr,
                                                 void *  ctx_out_port_ptr,
                                                 bool_t *is_ext_trigger_not_satisfied_ptr)

{
   return FALSE;
}

/**
 * This is used to determine if a port's trigger policy is satisfied on the output side when the policy is ABSENT.
 * It's just the inverse of is_trigger_present.
 */
bool_t spl_topo_out_port_is_trigger_absent(void *ctx_topo_ptr, void *ctx_out_port_ptr)
{
   return TRUE;
}

/**
 * During the spl_topo_process loop, this function is used to check if setup/adjust is necessary for this output port.
 * Blocks output if:
 * 1. Media format isn't valid.
 * 2. For external ports, if send_to_topo is FALSE.
 * 3. The port's nontrigger policy is blocked.
 * 4. If output port trigger isn't present.
 */
bool_t spl_topo_out_port_trigger_blocked(spl_topo_t *topo_ptr, spl_topo_output_port_t *out_port_ptr)
{
   return FALSE;
}
