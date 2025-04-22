#ifndef CAPI_INTF_EXTN_METADATA_H
#define CAPI_INTF_EXTN_METADATA_H

/**
 * \file capi_intf_extn_metadata.h
 * \brief
 *    Common Audio Processing Interface v2 stream metadata.
 *
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_defs.h"
#include "module_cmn_metadata.h"

/** @weakgroup weakf_capi_chapter_metadata
Metadata is information about the data in a buffer. The Metadata interface
extension (#INTF_EXTN_METADATA) must be implemented by modules that are
required to inject, modify, use, or propagate metadata:

- All multi-port modules
- All buffering modules
- Any single-port modules

The framework does not help to propagate metadata for modules that implement
this extension. A module implementing this extension is responsible for all
metadata, not just metadata the module might be interested in. It is
responsible for propagating metadata from input to output, including all flags
in #capi_stream_flags_t (end_of_frame, timestamp, EOS, and so on).

After a capi_init_f() call, a vtable and context pointer are passed to the
module that is implementing this extension. The vtable includes callback
functions that help in common metadata operations.

Metadata transfer is performed using doubly linked lists
(#module_cmn_md_list_t).

Single-port modules that implement this extension must ensure that they send or
destroy all the internally held metadata when they disable themselves.

Sink modules that implement this extension must destroy all metadata after the
metadata goes through internal algorithm delays.

For most SISO modules, the framework's default implementation should be
sufficient. A SISO module implementing this extension must clear the internally
held metadata before moving to the Disable Process state. When such a module
is disabled, the framework propagates the metadata.

@heading2{Common Metadata Interfaces}

The module_cmn_metadata.h header file defines the common metadata structures.

All metadata must use the #module_cmn_md_t structure. It contains a
metadata ID (GUID), flags, size, offset, and either in-band or out-band data
for the actual metadata.

@heading3{Flags}

Metadata flags are defined in #module_cmn_md_flags_t. @newpage

@heading4{Out-of-band}

The following figure illustrates in-band and out-of-band flags.

@inputfigcap{0.65,metadata_inband_oob_memory,In-band and out-of-band flags,ibOobFlags}

- For in-band, #module_cmn_md_t and the metadata-specific payload are in one
  contiguous memory buffer).
- For out-of-band, metadata-specific memory is elsewhere and module_cmn_md_t
  has a pointer to it.

Metadata-specific memory cannot contain any pointers.

@heading4{Buffer Association}

Metadata can be sample- or buffer-associated (via #module_cmn_md_flags_t).
- Sample-associated metadata always sticks to the same position in the signal,
  even when the signal is processed by an algorithm with delay. Thus, when the
  signal is processed by a module, the offset is adjusted by algorithmic delay.
  \n @vertspace{3}
  Sample-associated metadata suffers both algorithmic and buffering delay.
  \n @vertspace{3}
  Example: EOS is sample-associated because EOS cannot be propagated ahead of
  the last sample.
  The following diagram shows metadata propagation for sample-associated
  metadata.\n @vertspace{3}
  @inputfig{0.75,metadata_propagation_sample_assoc}
- Buffer-associated metadata does not suffer algorithmic delay, but it does
  suffer from any buffering delay. Buffering delay is typically zero for simple
  PP modules. \n @vertspace{3}
  Some modules might have internal data buffered, which might be used to delay
  some metadata. In the absence of a buffering delay, even when a signal
  suffers delay, metadata comes out quicker. \n @vertspace{3}
  For example, a DFG is buffer-associated metadata because it must propagate
  even if data is delayed by an algorithmic delay.

@heading3{Offset}

An offset in #module_cmn_md_t indicates the position in the data buffer from or
at which metadata is applicable. For example, when a stream gain metadata is
applicable from the 50th sample onwards, the offset is 50.

@heading3{Lists}

Metadata transfers are done using doubly linked lists (via #module_cmn_md_list_t).

@heading2{EOS Metadata}

@heading3{Flags}

@heading4{Flushing EOS}

Flushing EOS causes all stream data to be rendered, as shown in the following
figure. To send all the signals to the output, zeroes worth of algorithmic
delay are pushed through the module: zeroes worth = zero samples equal to the
amount of algorithmic delay.

@inputfigcap{0.75,metadata_eos_flushing,Stream data rendered due to flushing EOS,flushingEosDataRendering}

@newpage
When data follows the external EOS, the EOS stops it from being flushed. The
incoming data itself can send data. Hence, a flushing EOS is converted to
non-flushing if there is any data follows the EOS.

@heading4{Internal EOS}

Internal EOS is used to indicate data flow stoppage due to upstream stops or
flushes. If any data follows the internal EOS, the internal EOS is not useful
and can be dropped.

@heading3{EOS Payload}

The modules that propagate metadata must keep #module_cmn_md_eos_t intact.

@heading2{DFG Metadata}

DFG metadata indicates that the upstream data flow has a data flow gap
(possibly due to a stream pause operation).

@heading2{Virtual Function Table}

After initialization, a virtual function table (vtable) and context pointer
(both in #intf_extn_param_id_metadata_handler_t) are passed to the module that
is implementing this extension. The vtable includes callback functions that
help in common metadata operations: create, clone, destroy, propagate, and
modify at DFG.
*/

/** @addtogroup capi_if_ext_metadata
@{ */

/** Unique identifier of the interface extension for metadata definitions and
    their methods.
 */
#define INTF_EXTN_METADATA 0x0A00101F

/** Version details of the metadata handler supported by the framework.
 *  Will be incremented for every update to the supported features.
 */
#define INTF_EXTN_METADATA_HANDLER_VERSION 0x00000001

/** ID of the parameter the containers or services use to set the handlers for
    different methods of operating on metadata lists.
 */
#define INTF_EXTN_PARAM_ID_METADATA_HANDLER 0x0A001020

typedef struct intf_extn_md_propagation_t intf_extn_md_propagation_t;

/** Defines the input and output data formats.
 */
struct intf_extn_md_propagation_t
{
   data_format_t df;
   /**< Data format.

        For CAPI_RAW_COMPRESSED: bits_per_sample and sample_rate are not
        applicable. In all other cases (PCM and packetized), they are
        applicable.

        For PCM and packetized: all lengths are given in bytes_per_channel.

        For raw compressed: per_channel qualifier is not applicable. */

   uint32_t initial_len_per_ch_in_bytes;
   /**< Amount of data in the buffer (in bytes per channel) when
        capi_vtbl_t::process() is entered.

        For inputs: after process() returns, the data remaining in the buffer
        = (initial_len_per_ch_in_bytes  - len_per_ch_in_bytes) >= 0.

        For outputs: after process returns, the data in the output buffer
        = (initial_len_per_ch_in_bytes + len_per_ch_in_bytes).

        Also, after moving metadata to the output list, offsets are adjusted by
        adding initial_len_per_ch_in_bytes. */

   uint32_t len_per_ch_in_bytes;
   /**< Length (in bytes) of data consumed for input and data produced for
        output per channel.

        For inputs and outputs, this value corresponds to the actual length
        after the capi_vtbl_t::process() call. */

   uint32_t buf_delay_per_ch_in_bytes;
   /**< Delay (length in bytes) of data per channel already buffered in the
        input or output channel before the capi_vtbl_t::process() call.

        @valuesbul
        - 0 -- For most modules
        - Nonzero -- For modules with internal buffering that is not accounted
          for by the algorithm delay.

          This buffering delay must not be included in a module's reported
          algorithm delay. It will lead to incorrect calculations in
          #intf_extn_param_id_metadata_handler_t::metadata_propagate() for
          adjusting metadata offsets. */

   uint32_t bits_per_sample;
   /**< Bits per sample if the data is PCM or packetized.

        @values 16, 32 */

   uint32_t sample_rate;
   /**< Sample rate (in Hertz) if the data is PCM or packetized. */
};

/** Specifies the control flags for tracking metadata. */
struct module_cmn_md_tracking_flags_t
{
   union
   {
      struct
      {
         uint32_t use_only_specified_heap : 1;
         /**< Indicates if the tracking information needs to be allocated only from the specified heap.

              @valuesbul
              - #MODULE_CMN_MD_TRACKING_USE_SPECIFIED_HEAP_OPTIONAL
              - #MODULE_CMN_MD_TRACKING_USE_SPECIFIED_HEAP_MANDATORY @tablebulletend
       */

         uint32_t enable_cloning_event : 1;
         /**< Indicates if the client needs to be notified through an event when the tracking MD
              is cloned in the framework.

              @valuesbul
              - #MODULE_CMN_MD_TRACKING_DISABLE_CLONING_EVENT
              - #MODULE_CMN_MD_TRACKING_ENABLE_CLONING_EVENT @tablebulletend
       */

         uint32_t requires_custom_event : 1;
         /**< Indicates if the metadata needs custom tracking event handling.

              @valuesbul
              - #MODULE_CMN_MD_TRACKING_USE_GENERIC_EVENT
              - #MODULE_CMN_MD_TRACKING_USE_CUSTOM_EVENT @tablebulletend
       */
      };

      uint32_t word;
      /**< Entire 32-bit word for easy access to read or write an entire word
           in one shot. */
   };
};

typedef struct module_cmn_md_tracking_flags_t module_cmn_md_tracking_flags_t;

/** Specifies that it is optional to allocate the metadata tracking information
 *  in the client-specified heap.
 */
#define MODULE_CMN_MD_TRACKING_USE_SPECIFIED_HEAP_OPTIONAL 0

/** Specifies that it is mandatory to allocate the metadata tracking information
 *  in the client-specified heap.
 */
#define MODULE_CMN_MD_TRACKING_USE_SPECIFIED_HEAP_MANDATORY 1

/** Specifies that client does not need any cloning events. */
#define MODULE_CMN_MD_TRACKING_DISABLE_CLONING_EVENT 0

/** Specifies that client requires a cloning event from the framework
 *  when the tracking MD is cloned. */
#define MODULE_CMN_MD_TRACKING_ENABLE_CLONING_EVENT 1

/** Specifies that the tracking event uses a generic implementation. */
#define MODULE_CMN_MD_TRACKING_USE_GENERIC_EVENT 0

/** Specifies that the tracking event needs custom implementation.
 *
 *  example : EOS from the HLOS client uses custom implementation and EOS
 *            from OLC uses generic implementation. */
#define MODULE_CMN_MD_TRACKING_USE_CUSTOM_EVENT 1

/** Payload structure specific to the metadata tracking information. */
struct module_cmn_md_tracking_payload_t
{
   module_cmn_md_tracking_flags_t flags;
   /**< Indicates the metadata tracking flag. */

   uint16_t src_domain_id;
   /**< Domain ID of the packet's source.

        Bits 8 to 15 (eight bits) in the core header structure, gpr_packet_t
        (see gpr_packet.h). */

   uint16_t dst_domain_id;
   /**< Domain ID of the destination where the packet is to be delivered.

        Bits 0 to 7 (eight bits) in the core header structure, gpr_packet_t
        (see gpr_packet.h). */

   uint32_t src_port;
   /**< Identifies the service from where the packet came.

        Bits 31 to 0 (thirty-two bits) in the core header structure,
        gpr_packet_t (see gpr_packet.h). */

   uint32_t dest_port;
   /**< Identifies the service where the packet is to be delivered.

        Bits 31 to 0 (thirty-two bits) in the core header structure,
        gpr_packet_t (see gpr_packet.h). */

   uint32_t token_lsw;
   /**< Client transaction ID provided by the sender.
        Lower 32 bits of the token.
        This value is populated from the metadata header sent by the client. */

   uint32_t token_msw;
   /**< Client transaction ID provided by the sender.
        Higher 32 bits of the token.
        This value is populated from the metadata header sent by the client. */
};

typedef struct module_cmn_md_tracking_payload_t module_cmn_md_tracking_payload_t;

/** The metadata can create a tracking reference based on the client configuration.
 *  The metadata tracking would have the payload to raise an event when the
 *  metadata is either dropped/rendered based on the client configuration.
 *  The payload also specifies the control flags and heap ID to
 *  create the tracking reference.
 */
struct module_cmn_md_tracking_t
{
   module_cmn_md_tracking_payload_t tracking_payload;
   /**< Metadata tracking payload information. */

   capi_heap_id_t heap_info;
   /**< Specifies the heap ID for allocating the tracking payload. */
};

typedef struct module_cmn_md_tracking_t module_cmn_md_tracking_t;

typedef struct intf_extn_param_id_metadata_handler_t intf_extn_param_id_metadata_handler_t;

/** Function wrapper for callback functions that help in common metadata
    operations. This wrapper includes the following functions:
    - metadata_create()
    - metadata_clone()
    - metadata_destroy()
    - metadata_propagate()
    - metadata_modify_at_data_flow_start()
    - metadata_create_with_tracking()
 */
struct intf_extn_param_id_metadata_handler_t
{
   /** Version of this structure (currently v1).

       In subsequent versions, more fields might be present, but no fields will
       be removed.

       The version supported by the framework would be configured to the module.
       The module should only use features supported by the specified version.

       example : If the module is compiled for version 2 and framework uses version 1,
                 the module should only use API features from version 1 specified by
                 framework to the module.
   */
   uint32_t version;

   /** Context pointer passed to the module implementing this extension.
   */
   void *context_ptr;

   /**
     Allocates memory for metadata, creates a list node, and inserts the new
     list node to the tail of a given list.

     @datatypes
     #capi_heap_id_t

     @param[in] context_ptr   Pointer to the context of the handler.
     @param[in] md_list_pptr  Double pointer to the list to which the new
                              metadata object is inserted.
     @param[in] size          Size of the metadata object to be created
     @param[in] heap_id       ID of the heap on which the metadata object is to
                              be created.
     @param[in] is_out_band   Indicates whether the metadata object is
                              out-of-band in the module_cmn_md_t structure.
     @param[in] md_pptr       Double pointer to the new metadata object that is
                              inserted.

     @detdesc
     This function handles only one metadata object per call.
     @par
     The module initializes the metadata with the ID, flag, and other details.
     @par
     The metadata payload cannot contain references, for example, metadata_buf
     or metadata_ptr, that cannot contain a pointer to another memory.

     @returns
     Error code (see Section @xref{hdr:errorCodes}).
   */
   capi_err_t (*metadata_create)(void *                 context_ptr,
                                 module_cmn_md_list_t **md_list_pptr,
                                 uint32_t               size,
                                 capi_heap_id_t         heap_id,
                                 bool_t                 is_out_band,
                                 module_cmn_md_t **     md_pptr);

   /**
     Creates a clone and (deep) copies the payload contents of the given
     metadata (even for out-of-band, which is inherited when cloning).

     @datatypes
     #capi_heap_id_t

     @param[in] context_ptr   Pointer to the context of the handler.
     @param[in] md_ptr        Pointer to the metadata to be cloned.
     @param[in] md_list_pptr  Double pointer to the metadata list in which the
                              cloned object is to be inserted.
     @param[in] heap_id       ID of the heap on which the metadata object is to
                              be created when cloning.

     @detdesc
     This function handles only one metadata object per call. For EOS and the
     data flow gap (DFG), special handling is automatically used.
     @par
     Cloning is useful for modules such as splitters that replicate data from
     an input to multiple outputs.
     @par
     However, cloning is costly due to mallocs. Use this function only if the
     input metadata list and objects cannot be reused for output. For example,
     the first output port can reuse metadata from input, but other output
     ports require clones.
     @par
     The metadata payload cannot contain references, for example, metadata_buf
     or metadata_ptr, that cannot contain a pointer to another memory.

     @returns
     Error code (see Section @xref{hdr:errorCodes}).

     @dependencies
     All input arguments must be valid. @newpage
    */
   capi_err_t (*metadata_clone)(void *                 context_ptr,
                                module_cmn_md_t *      md_ptr,
                                module_cmn_md_list_t **md_list_pptr,
                                capi_heap_id_t         heap_id);

   /**
     Destroys a metadata object and updates the head pointer of the stream
     data's metadata list.

     @param[in] context_ptr  Pointer to the context of the handler.
     @param[in] md_list_ptr  Pointer to the list of metadata.
     @param[in] is_dropped   Indicates whether some metadata (EOS) can result
                             in events when not dropped.
     @param[in] head_pptr    Pointer to the metadata list head pointer. If the
                             current head is being destroyed, it is updated
                             with the next pointer.

     @detdesc
     This destroy function is different from simply freeing memory. It changes
     reference counters and thus can affect when certain events are raised.
     @par
     This function handles only one metadata per call. It updates the list
     when an object and list node are removed. For EOS, special handling is
     automatically used. For out-of-band, the payload is also freed.
     @par
     If the caller is iterating over a list, the next pointer must be read
     before calling this function because the node will be freed by this
     call.

     @returns
     Error code (see Section @xref{hdr:errorCodes}).

     @dependencies
     All input arguments must be valid. @newpage
    */
   capi_err_t (*metadata_destroy)(void *                 context_ptr,
                                  module_cmn_md_list_t * md_list_ptr,
                                  bool_t                 is_dropped,
                                  module_cmn_md_list_t **head_pptr);

   /**
     Propagates metadata from the input stream to an internal list and the
     output stream while considering algorithmic and buffering delays.

     @datatypes
     #capi_stream_data_v2_t \n
     #intf_extn_md_propagation_t

     @param[in] context_ptr         Pointer to the context of the handler.
     @param[in] input_stream_ptr    Pointer to the input stream provided in the
                                    capi_vtbl_t::process() call. \n
                                    @vertspace{3}
                                    This function uses only
                                    #capi_stream_data_v2_t::flags and
                                    metadata_list_ptr. It does not use
                                    #capi_stream_data_v2_t::buf_ptr
                                    or #capi_buf_t::actual_data_len.
     @param[in] output_stream_ptr   Pointer to the output stream pointer
                                    provided in the capi_vtbl_t::process()
                                    call. \n @vertspace{3}
                                    This function uses only
                                    #capi_stream_data_v2_t::flags and
                                    #capi_stream_data_v2_t::metadata_list_ptr.
                                    It does not use
                                    #capi_stream_data_v2_t::buf_ptr or
                                    #capi_buf_t::actual_data_len.
     @param[in] internal_md_list_pptr  Double pointer to the list internal to
                                    the module. \n @vertspace{3}
                                    This list stores the metadata that could
                                    not get to the output due to an algorithm
                                    delay. \n @vertspace{3}
                                    For decoders, the internal list must be
                                    provided even if the algorithm delay is
                                    zero, because decoders must be called
                                    repeatedly during a forced capi_vtbl_t::process() call.
     @param[in] algo_delay_us       Algorithm delay in microseconds. This value
                                    must not include the buffering delay.
     @param[in] input_md_info_ptr   Pointer to the input metadata information.
     @param[in] output_md_info_ptr  Pointer to the output metadata information.

     @detdesc
     This function handles all metadata per call. Following is the simplified
     algorithm:
     - Metadata whose offset is within the input data being consumed is removed
       from the input stream metadata list and is moved to internal list.
     - From the internal list, metadata whose offset is beyond the algorithmic
       delay (plus buffering delay) is moved to the output metadata list.
     - Stream data flags except EOF are updated. @newpage
     @par
     Typically, modules call this function from capi_vtbl_t::process() after
     the module's algorithm is processed and the amount of input consumed and
     amount of output produced are known.
     @par
     While adjusting metadata offsets, this function segregates the input
     metadata list into the output metadata list or a module internal metadata
     list.
     @par
     This function is suitable for one-to-one or one-to-many transfers of
     metadata from input to output (pairwise). It is not suitable for
     many-to-one because the input list is cleared after the first copy.
     @par
     Multiport modules can use the function input-output pairwise. Metadata
     handling modules must take care of EOF because this function does not
     propagate EOF.
     @par
     This function does not propagate end_of_frame (in #capi_stream_data_v2_t).
     Typically, end_of_frame must be propagated only after the module cannot
     produce any more outputs with the given inputs.
     @par
     When end_of_frame is set, the given input can also be dropped if no output
     can be produced (for example, the threshold modules might require fixed
     length input to produce any output).
     @par
     For modules that generate or consume metadata, the recommended order is:
     -# Consume the input metadata.
     -# Propagate the rest of the metadata using this function.
     -# Add new metadata for the output.
     @par
     If data is dropped, this function must be called after dropping data.

     @sa marker_eos flag (#capi_stream_flags_t)

     @returns
     Error code (see Section @xref{hdr:errorCodes}).

     @dependencies
     All input arguments must be valid. @newpage
    */
   capi_err_t (*metadata_propagate)(void *                      context_ptr,
                                    capi_stream_data_v2_t *     input_stream_ptr,
                                    capi_stream_data_v2_t *     output_stream_ptr,
                                    module_cmn_md_list_t **     internal_md_list_pptr,
                                    uint32_t                    algo_delay_us,
                                    intf_extn_md_propagation_t *input_md_info_ptr,
                                    intf_extn_md_propagation_t *output_md_info_ptr);

   /**
     Checks and modifies any metadata when data flow starts.

     @datatypes
     #module_cmn_md_list_t \n
     #intf_extn_md_propagation_t

     @param[in] context_ptr   Pointer to the context of the handler.
     @param[in] md_node_pptr  Pointer to the metadata node to be handled.
     @param[in] head_pptr     Double pointer to the head pointer to be updated.

     @detdesc
     When data flow starts, any internal EOS or DFG in the buffer can be
     destroyed. Flushing EOS must be converted to non-flushing EOS. To achieve
     this, modules can call this function.
     @par
     Only modules that modify the data flow state should implement this
     function. Most modules are not required to use this function because the
     framework typically takes care of this operation. For example, a module
     inserts zeroes when upstream data flow is stopped; it must call this
     function to change flushing EOS to non-flushing EOS. Thus, the metadata is
     modified as follows:
     -# If it is flushing external EOS, make it non-flushing.
     -# If it is flushing internal EOS, it is destroyed.
     -# If it is DFG, it is destroyed.
     @par
     This function handles only one metadata node and updates the head pointer
     if necessary.

     @returns
     Error code (see Section @xref{hdr:errorCodes}). @newpage
    */
   capi_err_t (*metadata_modify_at_data_flow_start)(void *                 context_ptr,
                                                    module_cmn_md_list_t * md_node_pptr,
                                                    module_cmn_md_list_t **head_pptr);

   /**
     Allocates memory for metadata with tracking, creates a list node, inserts the new
     list node to the tail of a given list, and creates a reference counter for tracking.

     @datatypes
     #capi_heap_id_t

     @param[in] context_ptr        Pointer to the context of the handler.
     @param[in] md_list_pptr       Double pointer to the list to which the new
                                   metadata object is inserted.
     @param[in] size               Size of the metadata object to be created.
     @param[in] heap_id            ID of the heap on which the metadata object is to
                                   be created.
     @param[in] metadata_id        Metadata GUID.
     @param[in] flags              Specifies the metadata flags.
                                   variable type : module_cmn_md_flags_t
     @param[in] tracking_info_ptr  Specifies the tracking information.
                                   variable type : module_cmn_md_tracking_t
     @param[in] md_pptr            Double pointer to the new metadata object that is
                                   inserted.

     @detdesc
     This function handles only one metadata object per call.
     @par
     The module initializes the metadata with the offset and other details.
     This create would update the metadata ID and the flags.

     @par
     The metadata creates a tracking reference. The tracking reference
     contains the payload to raise an event when the metadata is
     dropped/rendered based on the configuration specified during metadata create.

     @par
     The metadata payload cannot contain references, for example, metadata_buf
     or metadata_ptr, that cannot contain a pointer to another memory.

     @returns
     Error code (see @xref{hdr:errorCodes}).
   */

   capi_err_t (*metadata_create_with_tracking)(void *                    context_ptr,
                                               module_cmn_md_list_t **   md_list_pptr,
                                               uint32_t                  size,
                                               capi_heap_id_t            heap_id,
                                               uint32_t                  metadata_id,
                                               module_cmn_md_flags_t     flags,
                                               module_cmn_md_tracking_t *md_tracking_ptr,
                                               module_cmn_md_t **        md_pptr);
};

/** @} */ /* end_addtogroup capi_if_ext_metadata */

#endif // CAPI_INTF_EXTN_METADATA_H
