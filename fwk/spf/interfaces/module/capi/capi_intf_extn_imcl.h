#ifndef CAPI_INTF_EXTN_IMCL_H
#define CAPI_INTF_EXTN_IMCL_H

/**
 *   \file capi_intf_extn_imcl.h
 *   \brief
 *        intf_extns related to the Inter-Module-Control-Links
 *
 *    This file defines interface extensions that would help modules relay information
 *    to the framework about their IMCL support, and related capabilities.
 *
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/*------------------------------------------------------------------------------
 * Include Files
 *----------------------------------------------------------------------------*/
#include "capi_types.h"

/** @weakgroup weakf_capi_chapter_imcl
The #INTF_EXTN_IMCL interface extension allows two modules to talk to each
other. Any module that requires a control link must implement INTF_EXTN_IMCL.
The framework can use this information to perform control port ID-based link
handling, buffer management, queue management, and so on.

IMCL is bidirectional and point-to-point.

@heading2{Intents}

Although IMCL provides the pipe for communication, it does not design the
parameters or protocol to be used between the modules. It is up to the modules
to determine the information they want to exchange with each other.

A control link can support multiple intents. An intent is an abstract concept
that groups a set of interactions between two modules. Module developers can
define their own intents. Intent IDs are GUIDs.

For example, a timer drift intent defines the APIs required for some modules to
query drifts from other modules. The protocols (when, what API is called, and
so on) are completely defined within the intent. As long as the connection
exists, the modules can talk to each other.

@heading2{Types of Ports}

Static control ports are labeled and have fixed meaning. They support only a
fixed list of intents defined in the h2xmlm_ctrlStaticPort tag.

Other ports are defined through h2xmlm_ctrlDynamicPortIntent, where intents
and the maximum number of possible usages of that intent are provided. Graph
designers assign the appropriate intents to the links in the QACT GUI.

@heading2{Control Link Port Operations}

Like data port operations, control port operations are associated with
connections being created, connected, disconnected, or closed. For more
information, see Chapter @xref{dox:Imclchap}. @newpage

@heading2{Types of Messages}

@heading3{One Time vs. Repeating}

Repeating messages use a queue to create a pool of buffers up front.

@heading3{Triggerable or Polling}

Most messages are required to be read only once per frame. Such messages are
handled through polling. Occasionally, messages might be sent when data
processing is not occurring. For such scenarios, triggerable messages are
suitable.

For every message, a flag can be set to help route messages appropriately.

@heading2{Typical Operation}

-# Create control ports with #INTF_EXTN_IMCL_PORT_OPEN, where the number of
   ports and required intents are mentioned. \n @vertspace{2}
   If necessary, the module raises #INTF_EXTN_EVENT_ID_IMCL_RECURRING_BUF_INFO
   to the framework to create recurring buffers.
-# The module creates the memory for the control ports after any validations.
   \n @vertspace{2}
   When the peer is connected, the container notifies the module through
   #INTF_EXTN_IMCL_PORT_PEER_CONNECTED.
-# After the peer is connected, the module does the following:@vertspace{2}
   -# Sends messages by first getting recurring buffers
      (#INTF_EXTN_EVENT_ID_IMCL_GET_RECURRING_BUF) or one-time buffers
      (#INTF_EXTN_EVENT_ID_IMCL_GET_ONE_TIME_BUF).
   -# Uses #INTF_EXTN_EVENT_ID_IMCL_OUTGOING_DATA to send messages to the peer.
   -# Uses #INTF_EXTN_PARAM_ID_IMCL_INCOMING_DATA to receive parameters from
      the peer.
-# The framework issues #INTF_EXTN_IMCL_PORT_PEER_DISCONNECTED to indicate that
   the peer is disconnected. \n @vertspace{2}
   The module must not send any more messages. If the module is holding any
   buffers, they must be returned (#INTF_EXTN_EVENT_ID_IMCL_OUTGOING_DATA must
   be sent as FALSE).
-# When #INTF_EXTN_IMCL_PORT_CLOSE is issued to the module, memory can be
   freed. All the intents cease to exist.
*/

/** @addtogroup capi_if_ext_imcl
The Intermodule Control Link interface extension (INTF_EXTN_IMCL) allows
modules to tell the framework that they support communication via IMCL.

Modules use #CAPI_EVENT_DATA_TO_DSP_SERVICE to communicate with the framework.
The param_id field is populated with one of the event IDs and the capi_bufs
data pointer to the corresponding event payload.
*/

/** @addtogroup capi_if_ext_imcl
@{ */

/** Unique identifier of the IMCL interface extension.

     This extension supports the following events and parameter IDs:
    - #INTF_EXTN_EVENT_ID_IMCL_RECURRING_BUF_INFO
    - #INTF_EXTN_EVENT_ID_IMCL_GET_RECURRING_BUF
    - #INTF_EXTN_EVENT_ID_IMCL_GET_ONE_TIME_BUF
    - #INTF_EXTN_EVENT_ID_IMCL_OUTGOING_DATA
    - #INTF_EXTN_PARAM_ID_IMCL_PORT_OPERATION
    - #INTF_EXTN_PARAM_ID_IMCL_INCOMING_DATA
*/
#define INTF_EXTN_IMCL 0x0A001012

/** ID of the event a module raises to tell the framework that it requires a
    recurring buffer of a specific size on a specific control port. The
    framework will allocate the queues accordingly.

    @msgpayload{event_id_imcl_recurring_buf_info_t}
    @table{weak__event__id__imcl__recurring__buf__info__t}
*/
#define INTF_EXTN_EVENT_ID_IMCL_RECURRING_BUF_INFO 0x0A001013

/* Payload corresponding to the event INTF_EXTN_EVENT_ID_IMCL_RECURRING_BUF_INFO */
typedef struct event_id_imcl_recurring_buf_info_t event_id_imcl_recurring_buf_info_t;

/** @weakgroup weak_event_id_imcl_recurring_buf_info_t
@{ */
struct event_id_imcl_recurring_buf_info_t
{
   uint32_t port_id;
   /**< Identifies the port.

        @values Control port IDs exposed by the module that is visible to
                the QACT tools */

   uint32_t buf_size;
   /**< Size (in bytes) of the recurring packet that the module expects on
        the specified port ID. */

   uint32_t num_bufs;
   /**< Number of recurring packets that the module expects on the specified
        port ID. @newpagetable */
};
/** @} */ /* end_weakgroup weak_event_id_imcl_recurring_buf_info_t */

/** ID of the event a module raises to ask the framework for a recurring
    buffer. The parameter provides a pointer that the framework populates from
    the appropriate queue.

    @msgpayload{event_id_imcl_get_recurring_buf_t}
    @table{weak__event__id__imcl__get__recurring__buf__t}
*/
#define INTF_EXTN_EVENT_ID_IMCL_GET_RECURRING_BUF 0x0A001014

/* Payload corresponding to the event INTF_EXTN_EVENT_ID_IMCL_GET_RECURRING_BUF */
typedef struct event_id_imcl_get_recurring_buf_t event_id_imcl_get_recurring_buf_t;

/** @weakgroup weak_event_id_imcl_get_recurring_buf_t
@{ */
struct event_id_imcl_get_recurring_buf_t
{
   uint32_t port_id;
   /**< Identifies the port.

        @values Control port IDs exposed by the module that is visible to
                the QACT tools */

   capi_buf_t buf;
   /**< CAPI buffer for the requested buffer.

        The framework populates the pointer to the buffer in data_ptr, in which
        the module then copies the data. The framework also populates
        max_data_len when the buffer is returned. */
};
/** @} */ /* end_weakgroup weak_event_id_imcl_get_recurring_buf_t */

/** ID of the event a module raises to ask the framework for a one-time
    buffer. This parameter provides a pointer that the framework populates from
    the appropriate source, like a buffer manager.

    @msgpayload{event_id_imcl_get_one_time_buf_t}
    @table{weak__event__id__imcl__get__one__time__buf__t} @newpage
*/
#define INTF_EXTN_EVENT_ID_IMCL_GET_ONE_TIME_BUF 0x0A001015

/* Payload corresponding to the event INTF_EXTN_EVENT_ID_IMCL_GET_ONE_TIME_BUF */
typedef struct event_id_imcl_get_one_time_buf_t event_id_imcl_get_one_time_buf_t;

/** @weakgroup weak_event_id_imcl_get_one_time_buf_t
@{ */
struct event_id_imcl_get_one_time_buf_t
{
   uint32_t port_id;
   /**< Identifies the port.

        @values Control port IDs exposed by the module that is visible to
                the QACT tools */

   capi_buf_t buf;
   /**< CAPI buffer for the requested buffer.

        The actual_data_len field contains the requested size of the one-time
        buffer.

        The framework populates data_ptr, into which the module then copies the
        data. @newpagetable */
};
/** @} */ /* end_weakgroup weak_event_id_imcl_get_one_time_buf_t */

typedef struct imcl_outgoing_data_flag_t imcl_outgoing_data_flag_t;

/** @weakgroup weak_imcl_outgoing_data_flag_t
@{ */
struct imcl_outgoing_data_flag_t
{
   uint32_t should_send : 1;
   /**< Indicates to the framework whether the buffer is to be sent to the
        peer.

        @valuesbul
        - 0 -- FALSE; the buffer returns to the buffer source
        - 1 -- TRUE; the buffer is sent to the peer @tablebulletend */

   uint32_t is_trigger : 1;
   /**< Indicates whether the module is to send a trigger or polling message.

        @valuesbul
        - 0 -- FALSE; a polling message is pushed to the destination control
               port buffer queue, which is handled at the processed boundary
        - 1 -- TRUE; a trigger message is pushed to the command queue of the
               destination container @tablebulletend */
};
/** @} */ /* end_weakgroup weak_imcl_outgoing_data_flag_t */

/** ID of the event a module raises to inform the framework that it is
    ready to send data to its peer. The framework pushes the data to the module
    on the other end of the control link.

    @msgpayload{event_id_imcl_outgoing_data_t}
    @table{weak__event__id__imcl__outgoing__data__t}

    @par Outgoing data control flags (imcl_outgoing_data_flag_t)
    @table{weak__imcl__outgoing__data__flag__t} @newpage
*/
#define INTF_EXTN_EVENT_ID_IMCL_OUTGOING_DATA 0x0A001016

/* Payload corresponding to the event INTF_EXTN_EVENT_ID_IMCL_OUTGOING_DATA */
typedef struct event_id_imcl_outgoing_data_t event_id_imcl_outgoing_data_t;

/** @weakgroup weak_event_id_imcl_outgoing_data_t
@{ */
struct event_id_imcl_outgoing_data_t
{
   uint32_t port_id;
   /**< Identifies the port.

        @values Control port IDs exposed by the module that is visible to
                the QACT tools */

   capi_buf_t buf;
   /**< CAPI buffer for the requested buffer.

        The module must populate the data_ptr and actual_data_len fields. */

   imcl_outgoing_data_flag_t flags;
   /**< Flags that control the buffer destination and type of message. */
};
/** @} */ /* end_weakgroup weak_event_id_imcl_outgoing_data_t */

/** ID of the parameter a module uses to control port operations associated
    with connections being created, connected, disconnected, or closed.

    @msgpayload{intf_extn_param_id_imcl_port_operation_t}
    @table{weak__intf__extn__param__id__imcl__port__operation__t}
 */
#define INTF_EXTN_PARAM_ID_IMCL_PORT_OPERATION 0x0A001018

typedef struct intf_extn_imcl_id_intent_map_t intf_extn_imcl_id_intent_map_t;

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
/** Contains the intents that are supported over the specified port.
 */
struct intf_extn_imcl_id_intent_map_t
{
   uint32_t port_id;
   /**< Identifies the port.

        @values Control port IDs exposed by the module that is visible to
                the QACT tools */

   uint32_t peer_module_instance_id;
   /**< Identifies the peer module instance. */

   uint32_t peer_port_id;
   /**< Identifies the peer port. */

   uint32_t num_intents;
   /**< Number of elements in the array. The intents are supported by the
        control port (port_id). */

   uint32_t intent_arr[0];
   /**< Array of intents of size num_intents. @tablebulletend */
}
#include "spf_end_pack.h"
#include "spf_end_pragma.h"
;

typedef struct intf_extn_imcl_port_open_t intf_extn_imcl_port_open_t;

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
/** Payload for the #INTF_EXTN_IMCL_PORT_OPEN operation.
 */
struct intf_extn_imcl_port_open_t
{
   uint32_t num_ports;
   /**< Number of elements in the array. */

   intf_extn_imcl_id_intent_map_t intent_map[0];
   /**< Array of intents (list of #intf_extn_imcl_id_intent_map_t structures)
        supported by each opened control port. */
}
#include "spf_end_pack.h"
#include "spf_end_pragma.h"
;

typedef struct intf_extn_imcl_port_close_t intf_extn_imcl_port_close_t;

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
/** Payload for the #INTF_EXTN_IMCL_PORT_CLOSE operation.
 */
struct intf_extn_imcl_port_close_t
{
   uint32_t num_ports;      /**< Number of elements in the array. */
   uint32_t port_id_arr[0]; /**< Array of control port IDs to be closed. */
}
#include "spf_end_pack.h"
#include "spf_end_pragma.h"
;

typedef struct intf_extn_imcl_port_start_t intf_extn_imcl_port_start_t;

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
/** Payload for the #INTF_EXTN_DATA_PORT_START operation.
 */
struct intf_extn_imcl_port_start_t
{
   uint32_t num_ports;      /**< Number of elements in the array. */
   uint32_t port_id_arr[0]; /**< Array of control port IDs to be started. */
}
#include "spf_end_pack.h"
#include "spf_end_pragma.h"
;

typedef struct intf_extn_imcl_port_stop_t intf_extn_imcl_port_stop_t;

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
/** Payload for the #INTF_EXTN_DATA_PORT_STOP operation.
 */
struct intf_extn_imcl_port_stop_t
{
   uint32_t num_ports;      /**< Number of elements in the array. */
   uint32_t port_id_arr[0]; /**< Array of control port IDs to be stopped. */
}
#include "spf_end_pack.h"
#include "spf_end_pragma.h"
;

/** Types of port operation codes (opcodes) used by
    #INTF_EXTN_PARAM_ID_IMCL_PORT_OPERATION.
 */
typedef enum intf_extn_imcl_port_opcode_t {
   INTF_EXTN_IMCL_PORT_OPEN = 0x1,
   /**< Indicates the port open operation on specific control port IDs.
        @vertspace{4}

        Information provided to the modules includes the number of ports being
        opened, control port IDs, peer module instance IDs, peer port IDs, and
        the array of intents. (Each port ID can have multiple intents.)
        @vertspace{4}

        Modules must not send any messages when the control port is in this
        state. @vertspace{4}

        Payload: #intf_extn_imcl_port_open_t @vertspace{6} */

   INTF_EXTN_IMCL_PORT_PEER_CONNECTED = 0x2,
   /**< Indicates that the peer port is connected and ready to handle incoming
        messages. @vertspace{4}

        As soon as a control port is opened, a module might try to send a
        message, however, the other side might not be ready yet. The module
        must wait for the connected state before it can send any messages over
        the control link. @vertspace{6} */

   INTF_EXTN_IMCL_PORT_PEER_DISCONNECTED = 0x3,
   /**< Indicates that the peer port is stopped and is not ready to receive
        messages. @vertspace{4}

        Modules must not send any messages when the control port is in this
        state. @vertspace{6} */

   INTF_EXTN_IMCL_PORT_CLOSE = 0x4,
   /**< Indicates the port close operation on specific port IDs. Each port ID
        can have multiple intents. @vertspace{4}

        A close operation is atomic: all intents associated with the port are
        closed. @vertspace{4}

        Modules must not send any control messages when the control port is in
        this state. @vertspace{4}

        Payload: #intf_extn_imcl_port_close_t @vertspace{6} */

   INTF_EXTN_IMCL_PORT_STATE_INVALID = 0XFFFFFFFF
   /**< Port opcode is not valid. */
} /** @cond */ intf_extn_imcl_port_opcode_t /** @endcond */;

typedef struct intf_extn_param_id_imcl_port_operation_t intf_extn_param_id_imcl_port_operation_t;

/** @weakgroup weak_intf_extn_param_id_imcl_port_operation_t
@{ */
struct intf_extn_param_id_imcl_port_operation_t
{
   intf_extn_imcl_port_opcode_t opcode;
   /**< Operation code that indicates the type of operation to be done on the
        control ports. */

   capi_buf_t op_payload;
   /**< CAPI buffer element specific to the operation code. One buffer is to be
        used per opcode.

        This element can contain a NULL data pointer if the operation does not
        require a specific payload. */
};
/** @} */ /* end_weakgroup weak_intf_extn_param_id_imcl_port_operation_t */

/** ID of the parameter a module uses to receive the IMCL buffer from its
    peer.

    The framework uses this ID and does a capi_vtbl_t::set_param() on the
    destination port ID. The module then parses the payload based on the intent
    code, which the IMCL peers understand.

    This parameter follows the typical set_param() routine with the payload
    pointing to the data buffer sent by the IMCL Peer.

    @msgpayload{intf_extn_param_id_imcl_incoming_data_t}
    @table{weak__intf__extn__param__id__imcl__incoming__data__t} @newpage
*/
#define INTF_EXTN_PARAM_ID_IMCL_INCOMING_DATA 0x0A001019

typedef struct intf_extn_param_id_imcl_incoming_data_t intf_extn_param_id_imcl_incoming_data_t;

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/** @weakgroup weak_intf_extn_param_id_imcl_incoming_data_t
@{ */
struct intf_extn_param_id_imcl_incoming_data_t
{
   uint32_t port_id;  /**< Identifies the port that is receiving data. */
   uint32_t reserved; /**< Maintains 8-byte alignment. */
   uint64_t buf[0];   /**< Array of buffers. */
}
#include "spf_end_pack.h"
#include "spf_end_pragma.h"
;
/** @} */ /* end_weakgroup weak_intf_extn_param_id_imcl_incoming_data_t */

/** @} */ /* end_addtogroup capi_if_ext_imcl */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* CAPI_INTF_EXTN_IMCL_H*/
