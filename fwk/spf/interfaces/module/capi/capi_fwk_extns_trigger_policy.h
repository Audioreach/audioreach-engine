#ifndef _CAPI_FWK_EXTN_TRIGGER_POLICY_H_
#define _CAPI_FWK_EXTN_TRIGGER_POLICY_H_

/**
 *   \file capi_fwk_extns_trigger_policy.h
 *   \brief
 *        This file contains extension for trigger policy
 *
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_types.h"

/** @weakgroup weakf_capi_chapter_trigger_policy
The Trigger Policy framework extension (#FWK_EXTN_TRIGGER_POLICY) determines
when the capi_vtbl_t::process() function is called for a module. Most modules
are called when all input ports have data and output ports have buffers (the
default policy of the framework). Input data and output buffers are defined as
follows:
- Data buffer, or data, refers to a buffer that has some data. In the context
  of a process() call, input ports have data.
- Empty buffer, or buffer, refers to a buffer that is ready to accept data.
  In the context of a process() call, output ports have a buffer.

For multiport and buffering modules, complex triggers are possible (for
example, when process() is called because input data is available, or because
an output buffer is available).

@heading2{Types of Triggers}
@xreflabel{hdr:triggerTypes}

Containers are triggered in two ways:
- Data or buffer trigger -- If a container thread is awakened by data or a
  buffer, the current trigger for processing is called a <i>data trigger</i>.
- Signal trigger -- Certain containers can have signal-triggered
  (timer-triggered) modules. If a container is awakened by a signal, the
  current trigger is called a <i>signal trigger</i>.

The policy used to call the module is based on the current trigger. If the
current trigger is based on signals, the signal trigger policy is used;
otherwise, the data trigger policy is used.

@note1hang A trigger policy is only one of the conditions for calling modules.
           Other conditions for calling the modules (such as meeting a
           threshold or if ports are started) must also be satisfied
           independently.

A module can leave either or both policies as NULL. In this case, the default
policy is used, which means all ports are mandatory:
- All input ports get input data
- All output ports get a buffer when the timer trigger causes a graph
to be processed.

If input data is not present, an underrun (underflow) occurs (erasure flag is
set). If output is not present, an overrun (overflow) occurs.

A signal trigger policy is not useful if there is no signal trigger module in
the container. Only under special conditions is a module required to implement
a signal trigger policy: when the module is used in a signal-triggered
container and the default policy does not work. Typically, the default policy
works for most modules, for example, a SISO module might behave as a source
during calibration time. @newpage

If a module requires a data trigger policy in a signal-triggered container,
the module must explicitly enable the policy through
#FWK_EXTN_EVENT_ID_DATA_TRIGGER_IN_ST_CNTR. Data triggers are handled in the
middle of signal triggers.

The schema for defining a trigger policy is the same for both signal triggers
and data triggers, but the actual callbacks are different.

@heading2{Triggerable Ports}
@xreflabel{hdr:triggerablePorts}

The trigger policy is described in two levels, ports and group of ports.

@note1hang A port in a triggerable group can belong to multiple groups.

@heading3{Mandatory Policy}

For the mandatory policy (#FWK_EXTN_PORT_TRIGGER_POLICY_MANDATORY), ports in
each group are ANDed. That is, all ports in the group must satisfy the trigger
condition (present or absent).

Multiple groups are ORed. That is, a module process() is called as long as
at least one group has a trigger.

Using the ports/groups and present/absent notion, any Boolean expression can be
satisfied. For example:
- The module process() might be called when either of the inputs (a or b) and
  output (c) are present: ac + bc, where ac forms the first group, and bc forms
  the second group.
- The module process() might be called in an XOR condition of inputs
  a^b = (!a)b + a(!b), where (!a) indicates the absence of input a.
- The module process() might be called when either inputs (a, b) or output (c)
  is present. There are three groups: a+b+c.

@heading3{Optional Policy}

For the optional policy (#FWK_EXTN_PORT_TRIGGER_POLICY_OPTIONAL), ports in each
group are ORed and multiple groups are ANDed. For example, (a+c)(b+c). Thus, the
module process() is called for a module when a timer trigger occurs OR all
ports in at least one group have a trigger.

The framework calls capi_vtbl_t::process() if any one of the OR conditions is
satisfied. In this case, the module also must check which OR condition is
actually satisfied before processing. For example, if the module asks for the
(abc + def) trigger policy, when process() is called, the module must check
that either abc or def is satisfied.

@newpage
@heading2{Non-triggerable Ports and Blocked Ports}
@xreflabel{hdr:nontriggerPortsBlockedPorts}

Apart from groups, there are optional non-triggerable ports and blocked
ports. Both non-triggerable and blocked ports belong to a non-triggerable group
that is ignored when the framework determines whether to call
capi_vtbl_t::process() on a module.

@note1hang A port cannot belong to both triggerable and non-triggerable groups.

@heading3{Non-triggerable Ports}

Optional non-triggerable ports never trigger a capi_vtbl_t::process() call.
However, if a module is triggered due to other ports, and if these ports also
have a trigger at that time, the ports carry the data and output.

@heading3{Blocked Ports}

An input or output port must not be given when calling capi_vtbl_t::process()
on the module, even though buffer or data might be present.

@note1hang Blocked ports do not apply for timer (signal) triggers.

@heading2{Default Trigger Policy}

The default data or buffer trigger policy for all modules is <i>All ports must
have triggers</i>. This policy is the same as having all groups in one group.

Upon an algorithm reset, port reset, or other resets, the trigger policy is not
reset. Also, for module enable and disable operations, modules must explicitly
issue a callback.

In a group, if a port is mandatory but it is stopped, the module will not get a
call unless the stopped port is removed from the group.
*/

/** @addtogroup capi_fw_ext_trigger_pol
@{ */

/*==============================================================================
   Constants
==============================================================================*/

/** Unique identifier of the framework extension that modules use to decide on
    a trigger policy. (For more information, see Section
    @xref{chp:triggerPolicyChap}.)
 */
#define FWK_EXTN_TRIGGER_POLICY 0x0A00103A

/** ID of the parameter a module uses to decide when its process() function is
    to be called.

    @msgpayload{fwk_extn_param_id_trigger_policy_cb_fn_t}
    @table{weak__fwk__extn__param__id__trigger__policy__cb__fn__t}
*/
#define FWK_EXTN_PARAM_ID_TRIGGER_POLICY_CB_FN 0x0A00103B

/*==============================================================================
   Type definitions
==============================================================================*/

/** Types of trigger groups that indicate whether multiple ports in a group
    must be ANDED or ORed. (For more information, see Section
    @xref{hdr:triggerablePorts}.)
 */
typedef enum fwk_extn_port_trigger_policy_t {
   FWK_EXTN_PORT_TRIGGER_POLICY_MANDATORY = 0,
   /**< All ports in a group must satisfy the trigger specified through
        #fwk_extn_port_trigger_affinity_t. */

   FWK_EXTN_PORT_TRIGGER_POLICY_OPTIONAL,
   /**< Any port in a group is sufficient to trigger a #capi_vtbl_t::process()
        call. */
} /** @cond */ fwk_extn_port_trigger_policy_t /** @endcond */;

/** Types of non-trigger groups. (For details, see Section
    @xref{hdr:nontriggerPortsBlockedPorts}.)
 */
typedef enum fwk_extn_port_nontrigger_policy_t {
   FWK_EXTN_PORT_NON_TRIGGER_INVALID = 0,
   /**< Invalid value (default). */

   FWK_EXTN_PORT_NON_TRIGGER_OPTIONAL,
   /**< Optional non-triggerable port that never triggers a
        #capi_vtbl_t::process() call. */

   FWK_EXTN_PORT_NON_TRIGGER_BLOCKED
   /**< Blocked port that does not trigger the module even if there is data
        or a buffer. */
} /** @cond */ fwk_extn_port_nontrigger_policy_t /** @endcond */;

typedef struct fwk_extn_port_nontrigger_group_t fwk_extn_port_nontrigger_group_t;

/** Defines a non-triggerable group of ports.
 */
struct fwk_extn_port_nontrigger_group_t
{
   fwk_extn_port_nontrigger_policy_t *in_port_grp_policy_ptr;
   /**< Pointer to the array that contains a value at the input port index.
        This value indicates that the port belongs to the optional
        non-triggerable or blocked group.

        The array is as big as the maximum number of input ports (as indicated
        in #CAPI_PORT_NUM_INFO). */

   fwk_extn_port_nontrigger_policy_t *out_port_grp_policy_ptr;
   /**< Pointer to the array that contains a value at the output port index.
        This value indicates that the port belongs to the optional
        non-triggerable or blocked group.

        The array is as big as the maximum number of output ports (as indicated
        in #CAPI_PORT_NUM_INFO). */
};

/** Types of affinity modes for a port.
 */
typedef enum fwk_extn_port_trigger_affinity_t {
   FWK_EXTN_PORT_TRIGGER_AFFINITY_NONE = 0,
   /**< Indicates that the port does not belong to this group. */

   FWK_EXTN_PORT_TRIGGER_AFFINITY_PRESENT,
   /**< Indicates that the presence of a trigger on the port contributes to the
        group trigger. */

   FWK_EXTN_PORT_TRIGGER_AFFINITY_ABSENT,
   /**< Currently not supported.

        Indicates that the absence of a trigger on the port contributes to the
        group trigger (the port must still be started). */
} /** @cond */ fwk_extn_port_trigger_affinity_t /** @endcond */;

typedef struct fwk_extn_port_trigger_group_t fwk_extn_port_trigger_group_t;

/** Defines a triggerable group of ports.
 */
struct fwk_extn_port_trigger_group_t
{
   fwk_extn_port_trigger_affinity_t *in_port_grp_affinity_ptr;
   /**< Pointer to the array that contains a value at the input port index, if
        the input port belongs to the group.

        The array is as big as the maximum number of input ports (as indicated
        in #CAPI_PORT_NUM_INFO). */

   fwk_extn_port_trigger_affinity_t *out_port_grp_affinity_ptr;
   /**< Pointer to the array that contains a value at the output port index, if
        the output port belongs to the group.

        The array is as big as the maximum number of output ports (as indicated
        in #CAPI_PORT_NUM_INFO). @newpagetable */
};

/**
  Callback function that changes the trigger policy.

  @datatypes
  #fwk_extn_port_nontrigger_group_t \n
  #fwk_extn_port_trigger_policy_t \n
  #fwk_extn_port_trigger_group_t

  @param[in] context_ptr  Pointer to the context given by the container in
                          #FWK_EXTN_PARAM_ID_TRIGGER_POLICY_CB_FN.
  @param[in] nontriggerable_ports_ptr  Pointer to the
                          #fwk_extn_port_nontrigger_policy_t structure that
                          indicates which ports are optional non-triggerable
                          and which ports are blocked. \n @vertspace{3}
                          The value can be NULL if there are no nontriggerable
                          or blocked ports.
  @param[in] port_trigger_policy  Type of trigger policy for a port: mandatory
                          or optional.
  @param[in] num_groups   Number of elements in the array.
  @param[in] triggerable_groups_ptr  Pointer to the array of length num_groups,
                          where each element is of type
                          #fwk_extn_port_trigger_policy_t. \n @vertspace{3}
                          Any call to this function replaces the previous
                          call's values for all ports.\n @vertspace{3}
                          For the signal trigger policy,
                          nontriggerable_ports_ptr must be NULL because
                          non-trigger policies are not yet supported.

  Modules may require trigger control only in transient states and may not require it in steady states. In that case,
  a module can remove custom non-trigger and trigger policies by setting the nontriggerable_ports_ptr and
  triggerable_groups_ptr to NULL. In this case, the framework switches to the default policies.

  Modules should try to remove policies as long as they can, as this removes overhead from the framework.

  Modules are also allowed to disable themselves after removing the trigger policy.

  @newpage
 */
typedef capi_err_t (*fwk_extn_change_trigger_policy_fn)(void *                            context_ptr,
                                                        fwk_extn_port_nontrigger_group_t *nontriggerable_ports_ptr,
                                                        fwk_extn_port_trigger_policy_t    port_trigger_policy,
                                                        uint32_t                          num_groups,
                                                        fwk_extn_port_trigger_group_t *   triggerable_groups_ptr);

/* Structure defined for above Property */
typedef struct fwk_extn_param_id_trigger_policy_cb_fn_t fwk_extn_param_id_trigger_policy_cb_fn_t;

/** @weakgroup weak_fwk_extn_param_id_trigger_policy_cb_fn_t
@{ */
struct fwk_extn_param_id_trigger_policy_cb_fn_t
{
   uint32_t version;
   /**< Version of this payload (currently 1).

        In subsequent versions, extra fields might be present, but no fields
        will be removed. */

   void *context_ptr;
   /**< Pointer to the argument that must be passed to
        fwk_extn_change_trigger_policy_fn(). */

   fwk_extn_change_trigger_policy_fn change_data_trigger_policy_cb_fn;
   /**< Callback function to change the data trigger policy.

        The policy affects future #capi_vtbl_t::process() calls. The callback
        can also be made from a process() call. */

   fwk_extn_change_trigger_policy_fn change_signal_trigger_policy_cb_fn;
   /**< Callback function to change the signal trigger policy.

        The policy affects future #capi_vtbl_t::process() calls. The callback
        can also be made from a process() call. */
};
/** @} */ /* end_weakgroup weak_fwk_extn_param_id_trigger_policy_cb_fn_t */

/*==============================================================================
   Constants
==============================================================================*/

/** ID of the event the trigger policy module raises to enable or disable a
    data trigger if the module is in a signal-triggered container.

    The module process() is called only when a signal trigger occurs and its
    data trigger policies are ignored.

    The signal-triggered container's topology process cannot be called with a
    data trigger unless a module that can buffer or drop the data is before the
    STM module. If the module is a buffering module or if it handles the data
    dropping, it can raise this event to allow process() to be called with a
    data trigger in the signal-triggered container.

    Most module are not required to raise this event, such as a module that is
    to change a signal trigger policy.

    For more details, see Section @xref{hdr:triggerTypes}. @newpage

    @msgpayload{fwk_extn_event_id_data_trigger_in_st_cntr_t}
    @table{weak__fwk__extn__event__id__data__trigger__in__st__cntr__t}
 */
#define FWK_EXTN_EVENT_ID_DATA_TRIGGER_IN_ST_CNTR 0x0A00104C

/*==============================================================================
   Type definitions
==============================================================================*/

/* Structure defined for above event */
typedef struct fwk_extn_event_id_data_trigger_in_st_cntr_t fwk_extn_event_id_data_trigger_in_st_cntr_t;

/** @weakgroup weak_fwk_extn_event_id_data_trigger_in_st_cntr_t
@{ */
struct fwk_extn_event_id_data_trigger_in_st_cntr_t
{
   uint32_t is_enable;
   /**< Indicates whether to allow a topology process with a data trigger to be
        in an ST container.

        @valuesbul
        - 0 -- FALSE (error; do not allow topology process)
        - 1 -- TRUE (allow topology process) @tablebulletend */

   uint32_t needs_input_triggers;
   /**< Indicates whether this module consumes input during data triggers in STM containers.

        @valuesbul
        - 0 -- FALSE; module doesn't consume input for data triggers in STM containers
        - 1 -- TRUE; module consumes input for data triggers in STM containers @tablebulletend */

   uint32_t needs_output_triggers;
   /**< Indicates whether this module generates output during data triggers in STM containers.

        @valuesbul
        - 0 -- FALSE; module doesn't generate output for data triggers in STM containers
        - 1 -- TRUE; module generates output for data triggers in STM containers @tablebulletend */
};
/** @} */ /* end_weakgroup weak_fwk_extn_event_id_data_trigger_in_st_cntr_t */

/** @} */ /* end_addtogroup capi_fw_ext_trigger_pol */

#endif /* _CAPI_FWK_EXTN_TRIGGER_POLICY_H_ */
