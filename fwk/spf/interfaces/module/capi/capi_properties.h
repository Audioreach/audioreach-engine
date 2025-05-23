#ifndef CAPI_PROPERTIES_H
#define CAPI_PROPERTIES_H

/**
 * \file capi_properties.h
 * \brief
 *      This file defines the data structures and ids for getting and setting properties in the Common Audio Processing
 *  Interface.
 *
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_types.h"
#include "capi_events.h"

/** @addtogroup capi_property_ids
  Properties are used to set and get information to and from the module.
  Properties are identified by IDs and have corresponding payloads. Their usage
  is similar to parameters, but parameters are module specific:
  - Parameters are defined by the implementer of the module
  - Parameters are used to control aspects that are specific to the underlying
    algorithm
  - Properties are generic and are defined in the CAPI interface.
  @vertspace{-6}

  @par Categories of properties
   - Properties that can be queried statically using
     capi_get_static_properties_f():
      - #CAPI_INIT_MEMORY_REQUIREMENT
      - #CAPI_STACK_SIZE
      - #CAPI_MAX_METADATA_SIZE
      - #CAPI_IS_INPLACE
      - #CAPI_REQUIRES_DATA_BUFFERING
      - #CAPI_NUM_NEEDED_FRAMEWORK_EXTENSIONS
      - #CAPI_NEEDED_FRAMEWORK_EXTENSIONS
      - #CAPI_INTERFACE_EXTENSIONS
      - #CAPI_MAX_STATIC_PROPERTIES
      - #CAPI_IS_ELEMENTARY
      - #CAPI_MIN_PORT_NUM_INFO
  @par
    - Properties that can be set at initialization and at any time after
      initialization:
      - #CAPI_EVENT_CALLBACK_INFO
      - #CAPI_PORT_NUM_INFO
      - #CAPI_HEAP_ID
      - #CAPI_INPUT_MEDIA_FORMAT
      - #CAPI_LOG_CODE
      - #CAPI_CUSTOM_INIT_DATA
      - #CAPI_SESSION_IDENTIFIER
      - #CAPI_INPUT_MEDIA_FORMAT_V2
      - #CAPI_MAX_INIT_PROPERTIES
  @newpage
  @par
    - Properties that can be set only after initialization:
      - #CAPI_ALGORITHMIC_RESET
      - #CAPI_EXTERNAL_SERVICE_ID
      - #CAPI_REGISTER_EVENT_DATA_TO_DSP_CLIENT
      - #CAPI_REGISTER_EVENT_DATA_TO_DSP_CLIENT_V2
      - #CAPI_PARAM_PERSISTENCE_INFO
      - #CAPI_MAX_SET_PROPERTIES
  @par
    - Properties that can be queried using capi_vtbl_t::get_properties():
      - #CAPI_METADATA
      - #CAPI_PORT_DATA_THRESHOLD
      - #CAPI_OUTPUT_MEDIA_FORMAT_SIZE
      - #CAPI_MAX_GET_PROPERTIES
  @par
    - Properties that can be set using capi_vtbl_t::set_properties() and
      queried using capi_vtbl_t::get_properties():
      - #CAPI_OUTPUT_MEDIA_FORMAT
      - #CAPI_CUSTOM_PROPERTY
      - #CAPI_OUTPUT_MEDIA_FORMAT_V2
      - #CAPI_MAX_SET_GET_PROPERTIES
      - #CAPI_MAX_PROPERTY

  @latexonly
  \subsection{Enumeration Type Documentation}
  @endlatexonly

  @inputfile{group__weakb__capi__property__ids__enum.tex}

  @inputfile{group__weakb__capi__persistence__type__enum.tex}
*/

/** @weakgroup weakb_capi_property_ids_enum
@{ */
/** Valid property IDs.
 */
typedef enum capi_property_id_t {
   //----------- Properties that can be queried statically using get_static_properties ---------------------------------
   CAPI_INIT_MEMORY_REQUIREMENT,
   /**< Amount of memory (in bytes) to be passed into the capi_init_f()
        function. @vertspace{4}

        Payload: #capi_init_memory_requirement_t @vertspace{6} */

   CAPI_STACK_SIZE,
   /**< Stack size required by this module (in bytes). @vertspace{4}

        Payload: #capi_stack_size_t @vertspace{6} */

   CAPI_MAX_METADATA_SIZE,
   /**< Deprecated. See #module_cmn_md_t. @vertspace{4}

        Maximum size of metadata generated by this module after each call
        to capi_vtbl_t::process(). @vertspace{4}

        If this value is zero, the module does not generate any metadata.
        The value includes the sizes of different structures used to pack
        metadata (see #CAPI_METADATA). @vertspace{4}

        Payload: #capi_max_metadata_size_t @vertspace{6} @newpage */

   CAPI_IS_INPLACE,
   /**< Indicates whether the module can perform in-place computation.
        @vertspace{4}

        If this value is TRUE, the caller can provide the same pointers for
        input and output (but this is not guaranteed). In this case, the input
        and output data format must be the same, and the
        requires_data_buffering property must be set to FALSE. @vertspace{4}

        Payload: #capi_is_inplace_t @vertspace{6}  */

   CAPI_REQUIRES_DATA_BUFFERING,
   /**< Specifies whether data buffering is TRUE or FALSE. @vertspace{4}

        If this value is FALSE and the module does not require a port data
        threshold (#CAPI_PORT_DATA_THRESHOLD), the module must behave as
        follows: @vertspace{4}
        - The number of output samples on all output ports must always be the
          same as the number of input samples. The caller must ensure that
          the number of input samples is the same on all input ports.
        - All of the input must be consumed. The caller must ensure that the
          output buffers have enough space.
        - The module must be able to handle any number of samples. @vertspace{4}

        If this value is FALSE and the module requires a port data threshold
        (#CAPI_PORT_DATA_THRESHOLD), the module must behave as follows:
        @vertspace{6}
        - The number of samples per buffer on all input and output ports
          must be the same as the threshold.
        - The caller must call the module with the required number of samples.
        - All of the input must be consumed. The caller must ensure that the
          output buffers have enough space. @vertspace{4}

        If this value is TRUE, the module must behave as follows:
        @vertspace{6}
        - The module must define a threshold in number of bytes for each
          input port and each output port.
        - The module must consume data on its input ports and fill data on its
          output ports until one of the following occurs: @vertspace{4}
            - The amount of remaining data in each buffer of at least one
              input port is less than its threshold @vertspace{-2}
            - Or the amount of free space in each buffer of at least one
              output port is less than its threshold. @vertspace{4}

        When this value is set to TRUE, significant overhead is added. Use the
        TRUE setting only under the following circumstances: @vertspace{6}
        - The module performs encoding/decoding of data.
        - The module performs rate conversion between the input and output.
        @vertspace{4}

        Payload: #capi_requires_data_buffering_t @vertspace{6} */

   CAPI_NUM_NEEDED_FRAMEWORK_EXTENSIONS,
   /**< Number of framework extensions required by this module. @vertspace{4}

        Payload: #capi_num_needed_framework_extensions_t @vertspace{6} @newpage */

   CAPI_NEEDED_FRAMEWORK_EXTENSIONS,
   /**< List of framework extensions that are required by the module.
        @vertspace{4}

        Payload: Array of #capi_framework_extension_id_t structures
        @vertspace{6}
        - Each value is the ID of a framework extension.
        - The number of elements of the array is the number returned in the
          query for the #CAPI_NUM_NEEDED_FRAMEWORK_EXTENSIONS property. */

   CAPI_INTERFACE_EXTENSIONS,
   /**< List of interface extensions provided by the client.
        @vertspace{4}

        The module must set the is_supported flag to TRUE or FALSE to indicate
        whether the module supports this extension.
        Additional data can also be exchanged using an optional buffer per
        extension. @vertspace{4}

        Payload: #capi_interface_extns_list_t @vertspace{6} */

   CAPI_IS_ELEMENTARY,
   /**< Indicates whether the module is an elementary module.
        @vertspace{4}

        To qualify as an elementary module, the module must meet the following
        criteria:
        - The module must be SISO
        - The module must have no requirements about trigger policy; it will
          get called every time the upstream module produces data
        - For each process call, the module must consume all data provided to it
          and produce the exact same amount of data
        - The module must not modify timestamps
        - The module must have 0 algorithmic delay
        - The module cannot raise the DM framework extension
        - The module cannot modify or write metadata; metadata reads are allowed
        as long as the metadata list is not modified
        - The module cannot reject media format; if the module can't handle a
          media format, it must externally accept the module and disable itself
          until a valid media format is received
        - The module cannot raise any propagated values (port state, is_realtime)
        - The module cannot modify any propagated values (port state, is_realtime)
        - Any information propagated along the data path will NOT be sent to
          the tap point module
            - One exception -- The media format will be propagated to the tap point
              module. @vertspace{4}

        Payload: #capi_is_elementary_t @vertspace{6} */

   CAPI_MIN_PORT_NUM_INFO,
   /**< Indicates the minimum number of input and output data ports for the module.
        Along with SISO if the module can also behave as sink or source then this property must be implemented and:
          a. if sink behavior is supported then minimum output port must be zero
          b. if source behavior is supported then minimum input port must be zero
        If the minimum number of output port is zero:
          a. Then framework doesn't propagate the downstream port state by default.
          b. If the module is acting as SISO and the output port is stopped then framework will treat it as a sink
      module,
             if this is not the desired behavior then the module should implement the #INTF_EXTN_PROP_PORT_DS_STATE
      extension
          and propagate the output state to the input.

          @vertspace{4}

          Following are the types of limits on the number of data ports:
          @vertspace{6}
          -# Maximum number of ports as defined by the module
          -# Minimum number of ports as defined by the module
          -# Maximum number of ports as defined by the client (if the module
             defines an infinite number of ports)
          -# Number of currently active ports @vertspace{4}

          Control ports are handled through interface extensions. @vertspace{4}

          Payload: #capi_min_port_num_info_t @vertspace{6} */

   CAPI_MODULE_VERSION_INFO,
   /**< Indicates the module version information of the module. @vertspace{4}

   Payload: #capi_module_version_info_t @vertspace{6} */

   CAPI_MAX_STATIC_PROPERTIES = 0x10000,
   /**< Dummy value that indicates the range of properties.
        @vertspace{6} */

   //------------ End of properties that can be queried statically -------------------------

   //-------------- Properties that can be set at init and at any time after init ----------------------------
   CAPI_EVENT_CALLBACK_INFO,
   /**< Function pointer and state required for raising events to the
        framework. @vertspace{4}

        Payload: #capi_event_callback_info_t @vertspace{6} */

   CAPI_PORT_NUM_INFO,
   /**< Sets the maximum number of input and output data ports for the module.
        @vertspace{4}

        Following are the types of limits on the number of data ports:
        @vertspace{6}
        -# Maximum number of ports as defined by the module
        -# Minimum number of ports as defined by the module
        -# Maximum number of ports as defined by the client (if the module
           defines an infinite number of ports)
        -# Number of currently active ports @vertspace{4}

        For example, a mixer can support infinite inputs (1) with four
        possible inputs in a given use \n case (2), but only two inputs are
        currently active (3). This property sets number 2. @vertspace{4}

        Control ports are handled through interface extensions. @vertspace{4}

        Payload: #capi_port_num_info_t @vertspace{6} */

   CAPI_HEAP_ID,
   /**< Provides the heap IDs for allocating memory. @vertspace{4}
         Heap ID must not be interpreted by the modules; it can only be used as an
       argument for malloc and other functions.

        Payload: #capi_heap_id_t @vertspace{6} */

   CAPI_INPUT_MEDIA_FORMAT,
   /**< Sets the media format for an input port.
        The caller must set the port ID in the payload. @vertspace{4}

        Payload: #capi_set_get_media_format_t @vertspace{6} */

   CAPI_LOG_CODE,
   /**< Provides the logging code to the module for logging module data.
        @vertspace{4}

        Payload: #capi_log_code_t @vertspace{6} */

   CAPI_CUSTOM_INIT_DATA,
   /**< Provides module-specific initialization data.
        This property ID is typically set only at initialization time.
        @vertspace{4}

        Payload: Module-specific @vertspace{6} */

   CAPI_SESSION_IDENTIFIER,
   /**< Deprecated. @vertspace{4}

        Provides values that allow the module to uniquely identify its
        placement and session. @vertspace{4}

        Payload: #capi_session_identifier_t @vertspace{6} */

   CAPI_INPUT_MEDIA_FORMAT_V2,
   /**< Sets media format version 2 for an input port.
        The caller must set the port ID in the payload. @vertspace{4}

        Payload: #capi_set_get_media_format_t followed by
        #capi_standard_data_format_v2_t followed by the channel type
        information @vertspace{6} @newpage */

   CAPI_MODULE_INSTANCE_ID,
   /**< Provides the instance ID of the module. This ID uniquely identifies the
        module in a graph. @vertspace{4}

        Payload: #capi_module_instance_id_t */

   CAPI_LOGGING_INFO,
   /**< Information provided by the containers to the module to help with
        logging, including text and PCM/binary dump logging. @vertspace{4}

        Payload: #capi_logging_info_t @vertspace{6} */

   CAPI_MAX_INIT_PROPERTIES = 0x20000,
   /**< Dummy value that indicates the range of properties.
        @vertspace{6} */

   //-------------- End of properties that can be set at init and at any time after init ----------------------

   //-------------- Properties that can only be set after init -------------
   CAPI_ALGORITHMIC_RESET,
   /**< Specifies whether the module is to reset any internal buffers and the
        algorithm state to zero. @vertspace{4}

        Settings are not required to be reset, and memory allocated by the
        module is not required to be freed. @vertspace{4}

        Payload: Empty buffer @vertspace{6} */

   CAPI_EXTERNAL_SERVICE_ID,
   /**< Currently not used. @vertspace{6} */

   CAPI_REGISTER_EVENT_DATA_TO_DSP_CLIENT,
   /**< Specifies the registration information for an event of type
        #CAPI_EVENT_DATA_TO_DSP_CLIENT_V2. @vertspace{4}

        Payload: #capi_register_event_to_dsp_client_t @vertspace{6} */

   CAPI_REGISTER_EVENT_DATA_TO_DSP_CLIENT_V2,
   /**< Specifies the registration information for an event of type
        #CAPI_EVENT_DATA_TO_DSP_CLIENT_V2. @vertspace{4}

         Payload: #capi_register_event_to_dsp_client_v2_t @vertspace{6} */

   CAPI_PARAM_PERSISTENCE_INFO,
   /**< Specifies the type of persistent memory that is associated with a
        certain parameter. @vertspace{4}

        A capi_vtbl_t::set_properties() of this property ID is immediately
        followed by a capi_vtbl_t::set_param() of the corresponding parameter.
        Using this property, the module can do the following: @vertspace{6}
        - Perform error checks. For example, when module's expectation is to
          receive a persistent memory for a parameter, but the parameter is
          specified as non-persistent.
        - Decide to copy the pointer or copy the actual memory. @vertspace{4}

        If this property is not set, the module should, by default, assume that
        the parameter's payload is non-persistent. However, it is up to module
        implementation to support persistent, non-persistent, or both.
        @vertspace{4}

        If memory is persistent, this property also specifies whether memory is
        shared with other modules. @vertspace{6}
        - Persistent: Specifies whether the memory is exclusively available for
          the module. The module is not required to copy the contents of the
          set_param() payload; it can just store the pointer.
        - Global persistent: Specifies whether memory is shared with other
          modules. The module must never write to the memory. @vertspace{4}

        Payload: #capi_param_persistence_info_t @vertspace{6} */

   CAPI_MAX_SET_PROPERTIES = 0x30000,
   /**< Dummy value that indicates the range of properties. @vertspace{6} @newpage */

   //-------------- End of properties that can be set any time -------------

   //-------------- Properties that can only be queried only using get properties ---------------
   CAPI_METADATA,
   /**< Deprecated. See #module_cmn_md_t.

        Specifies that the module must fill in any metadata that it generated
        during a capi_vtbl_t::process() call when the caller queries this
        parameter. The query is typically done after the module raises
        #CAPI_EVENT_METADATA_AVAILABLE. @vertspace{4}

        Payload: #capi_metadata_t */

   CAPI_PORT_DATA_THRESHOLD,
   /**< Threshold of an input or output port (in bytes). @vertspace{4}

        This property ID is used only for modules that require a specific
        amount of data on any port. The module behavior depends on whether
        the module requires data buffering.
        For more information, see #CAPI_REQUIRES_DATA_BUFFERING.
        @vertspace{6} */

   CAPI_OUTPUT_MEDIA_FORMAT_SIZE,
   /**< Size of the media format payload for an output port.
        The size excludes the size of capi_data_format_header_t.
        @vertspace{4}

        Payload: #capi_output_media_format_size_t @vertspace{6} */

   CAPI_MAX_GET_PROPERTIES = 0x40000,
   /**< Dummy value that indicates the range of properties.
        @vertspace{6} */
   //------------ End of properties that can only be queried using get properties -------------------------

   //-------------- Properties that can be set or queried using set/get properties only ------------
   CAPI_OUTPUT_MEDIA_FORMAT,
   /**< Queries the media format for a specified output port. @vertspace{4}

        This property ID can also be used to set the output media format for
        modules that support control of the output media format. @vertspace{4}

        If a module supports control of some aspects such as the sample rate
        only, all other fields can be set to #CAPI_DATA_FORMAT_INVALID_VAL.
        The caller must set the port ID in the payload. @vertspace{4}

        Payload: #capi_set_get_media_format_t @vertspace{6} */

   CAPI_CUSTOM_PROPERTY,
   /**< Sets and gets a property that is specific to a framework extension.
        This property ID can also be used to statically get a
        module-specific property. @vertspace{4}

        The framework extension must define a secondary property ID and
        corresponding payload structure (%capi_custom_property_t) that are
        specific to the information the property is to set or get.
        @vertspace{4}

        Payload: #capi_custom_property_t @vertspace{6} */

   CAPI_OUTPUT_MEDIA_FORMAT_V2,
   /**< Queries media format version 2 for a specified output port.
        @vertspace{4}

        This property ID can also be used to set the output media format for
        modules that support control of the output media format. @vertspace{4}

        If a module supports control of some aspects, such as the sample rate
        only, all other fields can be set to #CAPI_DATA_FORMAT_INVALID_VAL.
        The caller must set the port ID in the payload. @vertspace{4}

        Payload: #capi_set_get_media_format_t @vertspace{6} */

   CAPI_MAX_SET_GET_PROPERTIES = 0x50000,
   /**< Dummy value that indicates the range of properties. @vertspace{6} */

   CAPI_MAX_PROPERTY = 0x7FFFFFFF
   /**< Maximum value that a property ID can take. @newpage */
} /** @cond */ capi_property_id_t /** @endcond */;
/** @} */ /* end_weakgroup weakb_capi_property_ids_enum */

typedef struct capi_prop_t capi_prop_t;

/** @addtogroup capi_property_ids
@{ */

/** Contains properties that can be sent to a module.

  Properties are used for generic set and get commands, which are independent
  of the underlying module.
 */
struct capi_prop_t
{
   capi_property_id_t id;
   /**< Identifies the property that is being sent. */

   capi_buf_t payload;
   /**< Payload buffer.

        The buffer must contain the payload corresponding to the property value
        for the #capi_vtbl_t::set_properties() call, and it must be
        sufficiently large to contain the payload for the set_properties()
        call. */

   capi_port_info_t port_info;
   /**< Information about the port for which the property is applicable.

        If the property is applicable to any port, the is_valid flag must be
        set to FALSE in the port information */
};

typedef struct capi_proplist_t capi_proplist_t;

/** Contains a list of CAPI properties. This structure can be used to send
  a list of properties to the module or query for the properties.
 */
struct capi_proplist_t
{
   uint32_t     props_num; /**< Number of elements in the array. */
   capi_prop_t *prop_ptr;  /**< Array of CAPI property elements. @newpagetable */
};

// Payloads for the properties
typedef struct capi_init_memory_requirement_t capi_init_memory_requirement_t;

/** Payload for the #CAPI_INIT_MEMORY_REQUIREMENT property.
*/
struct capi_init_memory_requirement_t
{
   uint32_t size_in_bytes; /**< Amount of memory. */
};

typedef struct capi_stack_size_t capi_stack_size_t;

/** Payload for the #CAPI_STACK_SIZE property.
*/
struct capi_stack_size_t
{
   uint32_t size_in_bytes; /**< Size of the stack. */
};

typedef struct capi_max_metadata_size_t capi_max_metadata_size_t;

/** Deprecated. see #module_cmn_md_t.

    Payload for the #CAPI_MAX_METADATA_SIZE property.
*/
struct capi_max_metadata_size_t
{
   uint32_t output_port_index; /**< Index of the output port for which this
                                    property applies. */
   uint32_t size_in_bytes;     /**< Size of the metadata. */
};

typedef struct capi_is_inplace_t capi_is_inplace_t;

/** Payload for the #CAPI_IS_INPLACE property.
*/
struct capi_is_inplace_t
{
   bool_t is_inplace;
   /**< Indicates whether a module is capable of doing in-place processing.

        @valuesbul
        - 0 -- Does not support in-place processing
        - 1 -- Supports in-place processing @tablebulletend */
};

typedef struct capi_requires_data_buffering_t capi_requires_data_buffering_t;

/** Payload for the #CAPI_REQUIRES_DATA_BUFFERING property.
*/
struct capi_requires_data_buffering_t
{
   bool_t requires_data_buffering; /**< Specifies whether data buffering
                                        is set to TRUE. @newpagetable */
};

typedef struct capi_is_elementary_t capi_is_elementary_t;

/** Payload for the #CAPI_IS_ELEMENTARY property.
*/
struct capi_is_elementary_t
{
   bool_t is_elementary;
   /**< Indicates whether the module can operate as an elementary module.

        @valuesbul
        - 0 -- This module is an elementary module
        - 1 -- This module is not an elementary module
        @tablebulletend */
};

typedef struct capi_min_port_num_info_t capi_min_port_num_info_t;

/** Payload for the #CAPI_MIN_PORT_NUM_INFO property.
*/
struct capi_min_port_num_info_t
{
   uint32_t num_min_input_ports;  /**< Mininum number of input ports. */
   uint32_t num_min_output_ports; /**< Mininum number of output ports. */
};

typedef struct capi_event_callback_info_t capi_event_callback_info_t;

/** Payload for the #CAPI_EVENT_CALLBACK_INFO property.
*/
struct capi_event_callback_info_t
{
   capi_event_cb_f event_cb;      /**< Callback function used to raise an event. */
   void *          event_context; /**< Opaque pointer value used as the context
                                       for this callback function. */
};

typedef struct capi_port_num_info_t capi_port_num_info_t;

/** Payload for the #CAPI_PORT_NUM_INFO property.
*/
struct capi_port_num_info_t
{
   uint32_t num_input_ports;  /**< Number of input ports. */
   uint32_t num_output_ports; /**< Number of output ports. */
};

typedef struct capi_heap_id_t capi_heap_id_t;

/** Payload for the #CAPI_HEAP_ID property.
*/
struct capi_heap_id_t
{
   uint32_t heap_id; /**< Heap ID for allocating memory. */
};

typedef struct capi_metadata_t capi_metadata_t;

/** Deprecated. See #module_cmn_md_t.

    Payload for the #CAPI_METADATA property.
*/
struct capi_metadata_t
{
   capi_buf_t payload; /**< Contains the metadata. */
};

typedef struct capi_port_data_threshold_t capi_port_data_threshold_t;

/** Payload for the #CAPI_PORT_DATA_THRESHOLD property.
*/
struct capi_port_data_threshold_t
{
   uint32_t threshold_in_bytes; /**< Threshold of an input or output port. */
};

typedef struct capi_output_media_format_size_t capi_output_media_format_size_t;

/** Payload for the #CAPI_OUTPUT_MEDIA_FORMAT_SIZE property.
*/
struct capi_output_media_format_size_t
{
   uint32_t size_in_bytes; /**< Size of the media format payload for an
                                output port. @tablebulletend */
};

typedef struct capi_num_needed_framework_extensions_t capi_num_needed_framework_extensions_t;

/** Payload for the #CAPI_NUM_NEEDED_FRAMEWORK_EXTENSIONS property.
*/
struct capi_num_needed_framework_extensions_t
{
   uint32_t num_extensions; /**< Number of framework extensions. */
};

typedef struct capi_framework_extension_id_t capi_framework_extension_id_t;

/** Payload for the #CAPI_NEEDED_FRAMEWORK_EXTENSIONS property.
*/
struct capi_framework_extension_id_t
{
   uint32_t id; /**< Identifies the framework extension. */
};

typedef struct capi_log_code_t capi_log_code_t;

/** Payload for the #CAPI_LOG_CODE property.
*/
struct capi_log_code_t
{
   uint32_t code; /**< Code for logging module data. */
};

typedef struct capi_session_identifier_t capi_session_identifier_t;

/** Deprecated. Payload for the #CAPI_SESSION_IDENTIFIER property.
 */
struct capi_session_identifier_t
{
   uint16_t service_id;
   /**< Identifies the service in which the module is contained.

        This ID is an opaque value that is not guaranteed to be
        backward compatible. As such, modules are not to determine their
        behavior based on this value. */

   uint16_t session_id;
   /**< Identifies the session within the service as indicated by
        service_id.

        Modules can use this value together with service_id to generate unique
        IDs for setting up intermodule communication within the same service
        session or for debug messaging. @newpagetable */
};

typedef struct capi_custom_property_t capi_custom_property_t;

/** Payload for the #CAPI_CUSTOM_PROPERTY property.
*/
struct capi_custom_property_t
{
   uint32_t secondary_prop_id;
   /**< Secondary property ID that indicates the format of the rest of the
        payload.

        Following this ID is the custom payload defined by the service. If a
        module does not support a custom property or a secondary property ID,
        it must return 0 in payload.actual_data_len of capi_prop_t. */
}
#include "capi_struct_align.h"
;

typedef struct capi_interface_extns_list_t capi_interface_extns_list_t;

/** Payload for the #CAPI_INTERFACE_EXTENSIONS property.

  Following this structure is an array of capi_interface_extn_desc_t
  structures with num_extensions elements.
 */
struct capi_interface_extns_list_t
{
   uint32_t num_extensions;
   /**< Number of interface extensions for which the client is querying.
        The client must provide this value. */
}
#include "capi_struct_align.h"
;

typedef struct capi_interface_extn_desc_t capi_interface_extn_desc_t;

/** Data type of each element in an array of
    capi_interface_extns_list_t::num_extensions elements (for the
    #CAPI_INTERFACE_EXTENSIONS property).
 */
struct capi_interface_extn_desc_t
{
   uint32_t id;
   /**< Identifies the interface extension being queried. The client must
        provide this value. */

   bool_t is_supported;
   /**< Indicates whether this extension is supported.

        @valuesbul
        - 0 -- Not supported
        - 1 -- Supported

        The module must provide this value. */

   capi_buf_t capabilities;
   /**< Optional buffer containing a structure that can be used for further
        negotiation of capabilities related to this extension.

        The structure is defined in the interface extension file. If it is not
        defined in this file, the interface extension does not have a
        capabilities structure. @newpagetable */
};

typedef struct capi_register_event_to_dsp_client_t capi_register_event_to_dsp_client_t;

/** Payload for the #CAPI_REGISTER_EVENT_DATA_TO_DSP_CLIENT_V2 event.
 */
struct capi_register_event_to_dsp_client_t
{
   uint32_t event_id;
   /**< Identifies the event to be registered. */

   bool_t is_registered;
   /**< Indicates whether a registered client exists for the event.

        @valuesbul
        - TRUE --  Event is registered
        - FALSE -- Event is not registered @tablebulletend */
};

typedef struct capi_register_event_to_dsp_client_v2_t capi_register_event_to_dsp_client_v2_t;

/** Payload for the #CAPI_REGISTER_EVENT_DATA_TO_DSP_CLIENT_V2 event.
 */
struct capi_register_event_to_dsp_client_v2_t
{
   uint64_t dest_address;
   /**< Address to which this event must be sent. */

   uint32_t token;
   /**< Token to be used when raising this event. */

   uint32_t event_id;
   /**< Identifies the event to be registered. */

   bool_t is_register;
   /**< Indicates whether a registered client exists for this event.

        @valuesbul
        - 0 -- FALSE (event is not registered)
        - 1 -- TRUE  (event is registered) @tablebulletend */

   capi_buf_t event_cfg;
   /**< Event configuration. Data is interpreted based on the event ID. */
};

/** Macro that defines an ID for an invalid persistence type. */
#define CAPI_PERSISTENCE_INVALID_VAL CAPI_INVALID_VAL

/** @} */ /* end_addtogroup capi_property_ids */

/** @weakgroup weakb_capi_persistence_type_enum
@{ */
/** Valid types of persistence memory associated with parameters.
 */
typedef enum capi_persistence_type_t {
   CAPI_PERSISTENT_MEM,
   /**< Indicates that the memory is persistent. @vertspace{4}

        This property indicates that this global memory is valid only for as
        long as the module is valid or until deregistration (whichever is
        first). @vertspace{4}

        This memory is unique to each module instance. Therefore, the module
        is free to read from or write to this memory region. */

   CAPI_GLOBAL_PERSISTENT,
   /**< Indicates that the memory is global persistent. @vertspace{4}

        This global memory is valid only for as long as the module is valid or
        until deregistration (whichever is first). @vertspace{4}

        This memory is shared with other modules. Therefore, the module must
        only read from this region; it is prohibited from writing into it.
        @vertspace{6} */

   CAPI_INVALID_PERSISTENCE = CAPI_PERSISTENCE_INVALID_VAL
   /**< Persistence type is invalid. @newpage */
} /** @cond */ capi_persistence_type_t /** @endcond */;
/** @} */ /* end_weakogroup weakb_capi_persistence_type_enum */

typedef struct capi_param_persistence_info_t capi_param_persistence_info_t;

/** @addtogroup capi_property_ids
@{ */

/** Payload header of the #CAPI_PARAM_PERSISTENCE_INFO property. Following
    this structure is the parameter ID payload.
 */
struct capi_param_persistence_info_t
{
   bool_t is_register;
   /**< Indicates whether the property is intended for registration or
        deregistration of memory.

        @valuesbul
        - TRUE  -- Registration
        - FALSE -- Deregistration @tablebulletend */

   capi_persistence_type_t mem_type;
   /**< Indicates the type of persistence memory associated with the parameter
        ID payload that follows.

        @valuesbul
        - #CAPI_PERSISTENT_MEM
        - #CAPI_GLOBAL_PERSISTENT @tablebulletend */

   uint32_t param_id;
   /**< Identifies the parameter. @newpagetable */
};

typedef struct capi_module_instance_id_t capi_module_instance_id_t;

/** Payload for the #CAPI_MODULE_INSTANCE_ID property.
 */
struct capi_module_instance_id_t
{
   uint32_t module_id;          /**< Identifies the module. */
   uint32_t module_instance_id; /**< Identifies the module instance. */
};

typedef struct capi_logging_info_t capi_logging_info_t;

/** Payload for the #CAPI_LOGGING_INFO property.
 */
struct capi_logging_info_t
{
   uint32_t log_id;
   /**< Valid log ID for the module.

        Any message printed with this ID uniquely identifies messages from this
        instance of the module. */

   uint32_t log_id_mask;
   /**< Bits reserved in the log_id field for the module to modify.

        The module can use these bits for changing the log ID during a
        discontinuity such as EOS or flush. */
};

typedef struct capi_module_version_info_t capi_module_version_info_t;

/** Payload for the #CAPI_MODULE_VERSION property. */
struct capi_module_version_info_t
{
   uint16_t version_major;
   /**< Major version of the module*/
   uint16_t version_minor;
   /**< Minor version of the module*/
};

/** @} */ /* end_addtogroup capi_property_ids */

#endif /* #ifndef CAPI_PROPERTIES_H */
