#ifndef CAPI_INTF_EXTN_PORT_OP_H
#define CAPI_INTF_EXTN_PORT_OP_H

/**
 *   \file capi_intf_extn_data_port_operation.h
 *   \brief
 *        intf_extns related to data port state operation.
 *
 *    This file defines interface extensions that would allow modules get the data
 *    port states from framework.
 *
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/*------------------------------------------------------------------------------
 * Include Files
 *----------------------------------------------------------------------------*/
#include "capi_types.h"

/** @weakgroup weakf_capi_chapter_dataport_op
The Data Port Operation interface extension (#INTF_EXTN_DATA_PORT_OPERATION)
defines port operations (open, start, stop, close).

Most simple PP modules might not be required to implement this extension.
Modules such as EC, buffering modules, mixer, splitter, and so on might be
required to implement it.

@heading2{Open}

The open operation (#INTF_EXTN_DATA_PORT_OPEN) communicates the
port ID-to-index mapping that the modules might want to cache for future use.

When a new data connection is made to a module, a data port is opened. This
operation is set for ports that were opened immediately when the module was
created as well as for any ports that are opened after module creation.

@heading2{Start}

The start operation (#INTF_EXTN_DATA_PORT_START) indicates that the framework
started providing buffers on the given ports.

On an input port, the start operation indicates that the subgraph containing
the module and the upstream operations of the module on this port are all
started.

On an output port, the start operation indicates that the subgraph containing
the module and the downstream operations of the module on this port are all
started.

@heading2{Stop}

The start operation (#INTF_EXTN_DATA_PORT_STOP) indicates that the framework
stopped providing buffers on the stopped port.

On an input port, the stop operation indicates that the subgraph containing the
module is stopped. Upstream stop is indicated through metadata (EOS), not
through port operation. The metadata method helps to drain data instead of
dropping it at once.

On an output port, the stop operation indicates that the subgraph containing
the module or any downstream operations of the module on this port are stopped.

@heading2{Close}

The close operation (#INTF_EXTN_DATA_PORT_CLOSE) is issued when a module is
closing or when the connection to an input or output port is removed. If a stop
was not issued before this close, a stop is also issued before the close.

When an input port in the data flowing state is closed, modules that handle
metadata must insert an internal EOS on all corresponding outputs. This tells
downstream operations about the upstream gap.

Open ports are not required to be closed for symmetry. For example,
#INTF_EXTN_DATA_PORT_OPEN need not be completed by #INTF_EXTN_DATA_PORT_CLOSE.

When the input port of a metadata handling module (which implements
#INTF_EXTN_METADATA) is closed, and if the data flow state of the port is not
already at-gap, an internal EOS might be required to be inserted at this input
port and eventually propagated to corresponding outputs. This internal EOS
serves as a way to indicate upstream data flow gap. The framework takes care of
this for modules that do not handle metadata.

@heading2{Data Flow State vs Port State}

@latexonly
\setlength\LTleft\parindent
\begin{longtable}[Htb]{|>{\raggedright}p{3.0in}|>{\raggedright}p{3.2in}|}
\hline
\multicolumn{1}{|c|}{\sf\small\textbf{Port state}} & \multicolumn{1}{c|}{\sf\small\textbf{Data flow state}} \\
\hline
\endhead
Related to the data port operations: closed, opened, started, stopped,
   suspended. & States are: Data is Flowing and Data Flow is at Gap (DFG).
   \tn \hline
Directly related to the port operations. & State change is due to data
   arrival at a port, or EOS or DFG metadata departure from a port.
   \tn \hline
State change is due to an SPF client sending a subgraph management command on
   the self or downstream peers. & State change is due to any gap in the data
   flow. For example: an SPF client sends a subgraph management command on the
   self or upstream peers; or an EOS either comes from the client or is due to
   an upstream pause.
   \tn \hline
\end{longtable}
@endlatexonly
*/

/** @addtogroup capi_if_ext_data_port_op
The Data Port Operation interface extension (INTF_EXTN_DATA_PORT_OPERATION)
allows modules to tell the framework that they require data port state
information from the framework.
*/

/** @addtogroup capi_if_ext_data_port_op
@{ */

/** Unique identifier of the Data Port Operation interface extension. */
#define INTF_EXTN_DATA_PORT_OPERATION 0x0A001023

/** ID of the parameter the framework uses to inform the module if there is a
    port state change.

    @msgpayload{intf_extn_data_port_operation_t}
    @tablens{weak__intf__extn__data__port__operation__t}
*/
#define INTF_EXTN_PARAM_ID_DATA_PORT_OPERATION 0x0A001031

typedef struct intf_extn_data_port_id_idx_map_t intf_extn_data_port_id_idx_map_t;

/** ID-to-index map used in any #intf_extn_data_port_operation_t instance.
 */
struct intf_extn_data_port_id_idx_map_t
{
   uint32_t port_id;
   /**< Identifies the port.

        @values Port IDs exposed by the module that is visible to the QACT
                tools */

   uint32_t port_index;
   /**< Index that is mapped to the port ID.

        @values Between 0 and the maximum number of ports as sent to the module
                using #CAPI_PORT_NUM_INFO */
};

/** Operation code (opcode) used in any field of #intf_extn_data_port_opcode_t
    to indicate that the value is unspecified.
 */
#define INTF_EXTN_PORT_OPCODE_INVALID_VAL AR_NON_GUID(0xFFFFFFFF)

/** Valid values for the port operation code. For more information, see
    Chapter @xref{chp:dataPortOp}.
 */
typedef enum intf_extn_data_port_opcode_t {
   INTF_EXTN_DATA_PORT_OPEN = 1,
   /**< Port open operation on specific ID-to-index mappings. \n @vertspace{4}

        This operation does not require a payload. @vertspace{6} */

   INTF_EXTN_DATA_PORT_START = 2,
   /**< Port start operation on specific ID-to-index mappings. This operation
        indicates that the framework starts providing buffers on the given
        ports. \n @vertspace{4}

        This operation does not require a payload. @vertspace{6} */

   INTF_EXTN_DATA_PORT_STOP = 3,
   /**< Port stop operation on specific ID-to-index mappings. This operation
        indicates that the framework stops providing buffers on the stopped
        port. \n @vertspace{4}

        This operation does not require a payload. @vertspace{6} */

   INTF_EXTN_DATA_PORT_CLOSE = 4,
   /**< Port close operation on specific ID-to-index mappings. \n @vertspace{4}

        Open ports are not required to be closed for symmetry. For example,
        INTF_EXTN_DATA_PORT_CLOSE is not required to close
        INTF_EXTN_DATA_PORT_OPEN. \n @vertspace{4}

        When the input port of a metadata handling module (that is, it
        implements #INTF_EXTN_METADATA) is closed, and if the data flow state
        of the port is not already at-gap, an internal EOS might need to be
        inserted at this input port and eventually propagated to the
        corresponding outputs. This internal EOS is a way to indicate the
        upstream data flow gap. The framework takes care of this EOS for
        modules that do not handle metadata. \n @vertspace{4}

        This operation does not require a payload. @newpage */

   INTF_EXTN_DATA_PORT_SUSPEND = 5,
   /**< Port suspend operation on specific IDs-Indices. \n @vertspace{4}

        This operation indicates that data flow is paused and the framework
        will not provide buffers on the suspended ports.
        The module must not reset the port state at suspend. \n @vertspace{4}

        This operation does not require a payload. @vertspace{6} */

   INTF_EXTN_DATA_PORT_OP_INVALID = INTF_EXTN_PORT_OPCODE_INVALID_VAL
   /**< Port operation code is not valid. */
} /** @cond */ intf_extn_data_port_opcode_t /** @endcond */;

/** Types of port states.
 */
typedef enum intf_extn_data_port_state_t {
   DATA_PORT_STATE_CLOSED = 0x0,          /**< Data port is destroyed and there is
                                               no further data flow. */
   DATA_PORT_STATE_OPENED  = 0x1,         /**< Data port is opened. */
   DATA_PORT_STATE_STARTED = 0x2,         /**< Data port is started. It can expect
                                               incoming data or it can output data
                                               on this port. */
   DATA_PORT_STATE_STOPPED   = 0x4,       /**< Data port is stopped. */
   DATA_PORT_STATE_SUSPENDED = 0x8,       /**< Data port is suspended. */
   DATA_PORT_STATE_INVALID   = 0xFFFFFFFF /**< Data port state is not valid. */
} /** @cond */ intf_extn_data_port_state_t /** @endcond */;

typedef struct intf_extn_data_port_operation_t intf_extn_data_port_operation_t;

#include "spf_begin_pragma.h"

/** @weakgroup weak_intf_extn_data_port_operation_t
@{ */
/** Following this structure is the port ID-index mapping array of size
    num_ports.
 */
struct intf_extn_data_port_operation_t
{
   bool_t is_input_port;
   /**< Indicates the type of port.

         @valuesbul
         - TRUE -- Input port
         - FALSE -- Output port @tablebulletend */

   intf_extn_data_port_opcode_t opcode;
   /**< Indicates the type of operation to be done on the input or output
        port: open, close, start, stop, suspend. */

   capi_buf_t opcode_payload_buf;
   /**< CAPI buffer element specific to the opcode. One buffer is to be used
        per opcode for the entire port ID-to-index map.

        This element can contain a NULL data pointer if the operation does not
        require a specific payload. */

   uint32_t num_ports;
   /**< Number of elements in the array. */

   intf_extn_data_port_id_idx_map_t id_idx[0];
   /**< Array of port ID-to-index mappings.

        This array is of variable length and depends on the number of ports to
        operate on. This payload has the ID-to-index couplet for each of the
        elements in num_ports. */
}
#include "spf_end_pragma.h"
;
/** @} */ /* end_weakgroup weak_intf_extn_data_port_operation_t */

/** @} */ /* end_addtogroup capi_if_ext_data_port_op */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* CAPI_INTF_EXTN_PORT_OP_H*/
