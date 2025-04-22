#ifndef CAPI_EVENTS_H
#define CAPI_EVENTS_H

/**
 * \file capi_events.h
 * \brief
 *      This file defines the events that can be raised by a module using the CAPI interface.
 *
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_types.h"

/** @addtogroup capi_events
    @xreflabel{hdr:events}

Modules use events to send asynchronous notifications to the framework.
During initialization, the framework provides a callback function and a
context pointer. The module can call this function any time to raise an event.
The appropriate payload must be sent based on the event ID.

The callback function is not thread safe, so it must be called from the same
thread context as the interface functions unless mentioned otherwise in the
event description. The payload data is copied before the function returns.

For example, raising the kilo packets per second (KPPS) change event:

@code
capi_event_KPPS_t payload;
payload.KPPS = 10000;

capi_event_info_t payload_buffer;
payload_buffer.port_info.is_valid = FALSE;
payload_buffer.payload.data_ptr = (int8_t*)(&payload);
payload_buffer.payload.actual_data_len = payload_buffer.payload.max_data_len = sizeof(payload);

capi_err_t result = event_cb_ptr(context_ptr, CAPI_EVENT_KPPS, &payload_buffer);

@endcode

  @latexonly
  \subsection{Enumeration Type Documentation}
  @endlatexonly
  @inputfile{group__weakb__capi__events__enum.tex}

  @latexonly
  \subsection{Typedef Documentation}
  @endlatexonly
  @inputfile{group__weakb__capi__func__event__callback.tex}
 */

/** @weakgroup weakb_capi_events_enum
@{ */
/** Valid IDs for the events that can be raised by a module.
*/
typedef enum capi_event_id_t {
   CAPI_EVENT_KPPS = 0,
   /**< Indicates the KPPS requirement of the module.
        This value must be calculated assuming zero cache miss. @vertspace{4}

        This event must be raised when the module is enabled and when the KPPS
        requirement changes. @vertspace{4}

         Kilo-Packets-Per-Second (KPPS), where 'packets' is a Hexagon processor specific
         terminology referring to multiple instructions grouped for parallel execution.
         When CAPI is used for non-Hexagon processors, this event must be re-interpreted as
         Kilo-Instructions-Per-Second (KIPS) or a similar property that indicates processing load per second.
         The platform specific layers (POSAL) and CAPI modules must be in sync with respect to the meaning of this
         property.

        Payload: #capi_event_KPPS_t @vertspace{6} */

   CAPI_EVENT_BANDWIDTH = 1,
   /**< Indicates the bandwidth requirement of the module (in bytes per
        second). @vertspace{4}

        This event must be raised when the module is enabled and when the
        bandwidth requirement changes. The bandwidth must be specified
        separately for code and data, and it must be calculated assuming no
        cache. @vertspace{4}

        Payload: #capi_event_bandwidth_t @vertspace{6} */

   CAPI_EVENT_DATA_TO_DSP_CLIENT = 2,
   /**< Deprecated. Use #CAPI_EVENT_DATA_TO_DSP_CLIENT_V2. @vertspace{4}

        Sends data to the client framework. @vertspace{4}

        The module must specify a parameter ID to indicate the type of data
        that is present in the payload. The module can also specify an
        optional token to indicate additional information such as an instance
        identifier. The module must provide the payload in a buffer. The buffer
        can be safely destroyed or reused once the callback returns.
        @vertspace{4}

        Payload: #capi_event_data_to_dsp_client_t @vertspace{6} */

   CAPI_EVENT_DATA_TO_OTHER_MODULE = 3,
   /**< Currently not supported. @vertspace{6} */

   CAPI_EVENT_OUTPUT_MEDIA_FORMAT_UPDATED = 4,
   /**< Indicates that the output media format from the module has changed.
        @vertspace{4}

        If this event is raised during a call to
        #capi_vtbl_t::process(), no data can be output in the process() call.
        @vertspace{4}

        Subsequent calls to the process function output the data of the new
        media type. If the module is to immediately output the data of the
        new media type, it must exit the process function with zero output
        and wait for the process function to be called again. @vertspace{4}

        Payload: #capi_set_get_media_format_t @vertspace{6} */

   CAPI_EVENT_PROCESS_STATE = 5,
   /**< Sent to the client to specify whether the module is enabled or
        disabled. The module is considered enabled by default. @vertspace{4}

        We recommend that the module disable itself if it is not doing any
        processing. The #capi_vtbl_t::process() function of the module is
        not called until it raises this event again to enable itself.
        @vertspace{4}

        If this event is raised during a call to process(), no data can be
        output in the process() call. @vertspace{4}

        Single port modules are bypassed when they are disabled. However,
        multi-port modules cannot be bypassed. This event might not be
        suitable for multi-port modules. @vertspace{4}

        Payload: #capi_event_process_state_t @vertspace{6} */

   CAPI_EVENT_ALGORITHMIC_DELAY = 6,
   /**< Indicates the algorithmic delay (in microseconds) caused by the
        module. @vertspace{4}

        This event must be raised when the delay changes. The value
        must include any buffering delay. @vertspace{4}

        Payload: #capi_event_algorithmic_delay_t @vertspace{6} */

   CAPI_EVENT_HEADROOM = 7,
   /**< Deprecated. Use control links instead. @vertspace{4}

        Indicates the head room requirement (in millibels) of the module (the
        default is zero). @vertspace{4}

        Payload: #capi_event_headroom_t @vertspace{6} */

   CAPI_EVENT_PORT_DATA_THRESHOLD_CHANGE = 8,
   /**< Indicates to the client that the threshold for a port has changed.
        @vertspace{4}

        One event must be raised for each port whose threshold has changed.
        For more information on port thresholds, see
        #CAPI_REQUIRES_DATA_BUFFERING. @vertspace{6}

        If this event is raised during a call to #capi_vtbl_t::process(),
        no data can be output in the process() call. @vertspace{4}

        Payload: #capi_port_data_threshold_change_t @vertspace{6} */

   CAPI_EVENT_METADATA_AVAILABLE = 9,
   /**< Deprecated. Use #capi_metadata_t. @vertspace{4}

        Indicates to the client that metadata is available for an output port.
        @vertspace{4}

        One event must be raised for each port whose threshold has changed.
        For more information, see #CAPI_METADATA. @newpage */

   CAPI_EVENT_DATA_TO_DSP_SERVICE = 10,
   /**< Sends data to the framework. @vertspace{4}

        The module must specify a parameter ID to indicate the type of data
        that is present in the payload. It can also specify an optional token
        to indicate additional information such as an instance identifier.
        It must then provide the payload in a buffer. The buffer can be
        safely destroyed or reused once the callback returns. @vertspace{4}

        Payload: #capi_event_data_to_dsp_service_t @vertspace{6} */

   CAPI_EVENT_GET_LIBRARY_INSTANCE = 11,
   /**< Queries for an instance of a supporting library. @vertspace{4}

        The module must provide the ID of the library. The service returns a
        pointer to an instance of the library. The instance contains a pointer
        to the virtual function table of the library interface as the first
        element. Every library virtual function table has
        #capi_library_base_t as its first element. @vertspace{4}

        Payload: #capi_event_get_library_instance_t @vertspace{6}

        For example, the following definitions are in the library header file:
        @vertspace{4}
  @code
  #define LIB_GUID <GUID>

  struct lib_obj;

  struct lib_vtable
  {
     capi_library_base_t b; // Should be the first element of every library vtable
     capi_err_t lib_func1(lib_obj *obj, uint32_t param);
     capi_err_t lib_func2(lib_obj *obj, uint32_t param);
     capi_err_t lib_func3(lib_obj *obj, uint32_t param);
  };

  struct lib_obj
  {
     lib_vtable *vtble;
  };

  @endcode
  @vertspace{12}

  The following code is in the module: @vertspace{4}
  @code
   capi_event_get_library_instance_t payload;
   payload.id = LIB_GUID;
   payload.ptr = NULL;

   capi_event_info_t payload_buffer;
   payload_buffer.port_info.is_valid = FALSE; // May be a valid value based on the use case.
   payload_buffer.payload.data_ptr = (int8_t*)(&payload);
   payload_buffer.payload.actual_data_len = payload_buffer.payload.max_data_len = sizeof(payload);

   capi_err_t result = event_cb_ptr(context_ptr, CAPI_EVENT_GET_LIBRARY_INSTANCE, &payload_buffer);

   lib_obj *lib_ptr = (lib_obj*)payload.ptr;

   lib_ptr->vtbl.lib_func1(lib_ptr, param);
   ...
   lib_ptr->vtbl.lib_func2(lib_ptr, param);
   ...
   lib_ptr->vtbl.lib_func3(lib_ptr, param);
   ...

   lib_ptr->vtbl.b.end(lib_ptr); // The pointer is freed here.
   lib_ptr = NULL;

   //The GUID can be queried from the lib object itself. This allows the code to determine the type of the object if it
  is not known:
   void *unknown_lib_ptr = get_lib(); // Some function that returns a stored lib pointer whose type is unknown.
   uint32_t interface_id = ((capi_library_base_t**)unknown_lib_ptr)->get_interface_id();
   switch(interface_id)
   {
   case LIB_GUID:
      lib_obj *lib_ptr = (lib_obj*)unknown_lib_ptr;
      ...
   ...
   }


   @endcode @vertspace{6} */

   CAPI_EVENT_GET_DLINFO = 12,
   /**< Queries the dynamic load information if the module is part of an SO \n
        file. @vertspace{4}

        Payload: #capi_event_dlinfo_t @vertspace{6} */

   CAPI_EVENT_OUTPUT_MEDIA_FORMAT_UPDATED_V2 = 13,
   /**< Indicates that media format version 2 output from the module has
        changed. @vertspace{4}

        If this event is raised during a call to capi_vtbl_t::process(),
        any data output from this process function is assumed to use
        the old media type. @vertspace{4}

        Subsequent calls to the process function output the data of the new
        media type. If the module is to immediately output the data of the
        new media type, it must exit the process function with zero output
        and wait for the process function to be called again. @vertspace{4}

        Payload: #capi_set_get_media_format_t @vertspace{6} */

   CAPI_EVENT_GET_DATA_FROM_DSP_SERVICE = 14,
   /**< Gets data from the framework. @vertspace{4}

        The module must specify a parameter ID to indicate the type of data
        that is required from the framework. The ID can also specify an
        optional token to indicate additional information (like an instance
        identifier). The ID must provide the pointer to the data in the
        payload of the buffer. The buffer can be safely destroyed or reused
        once the callback returns. @vertspace{4}

        The event is a blocking call. To update and return the callback, it
        must wait for the framework to get the required information.
        Because the request for data is a blocking call to the framework, the
        event handling in the framework must ensure that the response is
        instantaneous and does not involve any heavy processing. @vertspace{4}

        For modules offloaded using the MDF framework, this event waits for
        the master processor to send the details. @vertspace{4}

        Payload: #capi_event_get_data_from_dsp_service_t @vertspace{6} */

   CAPI_EVENT_DATA_TO_DSP_CLIENT_V2 = 15,
   /**< Sends data to the SPF client application processor. @vertspace{4}

        The module must specify a parameter ID to indicate the type of data
        that is present in the payload. It can also specify an optional token
        to indicate additional information such as an instance identifier.
        The module must then provide the payload in a buffer. The buffer can be
        safely destroyed or reused once the callback returns. @vertspace{4}

        Payload: #capi_event_data_to_dsp_client_v2_t @vertspace{6} @newpage*/

   CAPI_EVENT_DYNAMIC_INPLACE_CHANGE = 16,
   /**< Event ID used by the module to inform dynamic changes to the inplace property.

        Module changing itself to inplace must be:
          - SISO as long as its inplace, if it's MIMO when the event is raised it will error out.
          - Input media format and output media format must be equal.
          - Port thresholds must be equal.

        Module must change itself to non-inplace if:
          - It becomes MIMO at any point in time.
          - Input and output media formats are different.
          - Input and output thresholds are different.

      Payload: #capi_event_dynamic_inplace_change_t @vertspace{6} */

   CAPI_EVENT_HW_ACCL_PROC_DELAY = 17,
   /**< Event ID used by the module to indicate hardware accelerator delay.

      This delay applies to the modules utilizing hardware accelerators for off-loading partial/complete processing. It
      is the maximum time in microseconds taken by the hardware accelerator for the off-loaded processing for the
      provided
      input frame. Hardware accelerator processing happens in a synchronous manner such that the caller waits for the
      frame
      processing to finish. This delay does not include algorithmic delay due to signal processing in the hardware
      accelerator.

      Default value: zero.

      Payload: #capi_event_hw_accl_proc_delay_t */

   CAPI_EVENT_ISLAND_VOTE = 18,
   /**< Event ID used by the module to indicate if it wants to vote for or against island.

      If this event is raised by a module placed in a non-island container, it will be ignored.
      A vote against will be honored immediately.

      Default value: Vote for island (island entry) 0.

      Payload: #capi_event_island_vote_t */

   CAPI_EVENT_DEINTERLEAVED_UNPACKED_V2_SUPPORTED = 19,
   /**< Event ID used by the module to indicate if it supports CAPI_DEINTERLEAVED_UNPACKED_V2 interleaving format
      on the input ports. Module needs to raise this event once during init, to indicate if all the input ports
      accept processing the CAPI_DEINTERLEAVED_UNPACKED_V2 interleaving format.

      Note that if a module raise this event, and if module is generating unpacked output data then it must make 
      sure to raise output media format with CAPI_DEINTERLEAVED_UNPACKED_V2 interleaving format.

      Payload: No payload required.*/

   CAPI_MAX_EVENT = 0x7FFFFFFF
   /**< Maximum value that an event ID can take. @newpage */
} /** @cond */ capi_event_id_t /** @endcond */;
/** @} */ /* end_weakgroup weak_weakb_capi_events_enum */

typedef struct capi_event_info_t capi_event_info_t;

/** @ingroup capi_events
    Contains information about an event.
 */
struct capi_event_info_t
{
   capi_port_info_t port_info;
   /**< Port for which this event is raised.

        Set this field to an invalid value for events that are not specific
        to any port or if the payload contains the port information. */

   capi_buf_t payload;
   /**< Buffer that holds the payload for the event. */
};

/** @weakgroup weakb_capi_func_event_callback
@{ */
/**
  Signature of the callback function used to raise an event to the client.

  @datatypes
  #capi_event_id_t \n
  capi_event_info_t

  @param [in] context_ptr    Context pointer value provided by the framework
                             when the callback function was passed.
  @param [in] id             ID for the event being raised.
  @param [in] event_info_ptr Information about the event being raised.

  @detdesc
  The client provides the pointer to this function and the context_ptr in the
  call to capi_init_f().

  @returns
  #CAPI_EOK -- Success
  @par
  Error code -- Failure (see Section @xref{hdr:errorCodes})

  @dependencies
  This function must be called from the same thread context as the interface
  functions. @newpage
 */
typedef capi_err_t (*capi_event_cb_f)(void *context_ptr, capi_event_id_t id, capi_event_info_t *event_info_ptr);
/** @} */ /* end_weakgroup weak_weakb_capi_func_event_callback */

/** @addtogroup capi_events
@{ */

// Payloads for the events
typedef struct capi_event_KPPS_t capi_event_KPPS_t;

/** Payload for the #CAPI_EVENT_KPPS event.
 */
struct capi_event_KPPS_t
{
   uint32_t KPPS; /**< Kilo packets per second requirement of the module. */
};

typedef struct capi_event_bandwidth_t capi_event_bandwidth_t;

/** Payload for the #CAPI_EVENT_BANDWIDTH event.
 */
struct capi_event_bandwidth_t
{
   uint32_t code_bandwidth;
   /**< Code bandwidth of the module (in bytes per second). */

   uint32_t data_bandwidth;
   /**< Data bandwidth of the module (in bytes per second). */
};

typedef struct capi_event_data_to_dsp_client_t capi_event_data_to_dsp_client_t;

/** Deprecated. See #capi_event_data_to_dsp_client_v2_t.

    Payload for the #CAPI_EVENT_DATA_TO_DSP_CLIENT event.
 */
struct capi_event_data_to_dsp_client_t
{
   uint32_t param_id;
   /**< Indicates the type of data that is present in the payload. */

   uint32_t token;
   /**< Optional token that indicates additional information, such as an
        instance identifier. */

   capi_buf_t payload;
   /**< Buffer that contains the payload.

        This buffer can be safely destroyed or reused once the callback
        returns. @newpagetable */
};

typedef struct capi_event_dynamic_inplace_change_t capi_event_dynamic_inplace_change_t;

/** Deprecated.

    Payload for the #CAPI_EVENT_DYNAMIC_INPLACE_CHANGE event.
 */
struct capi_event_dynamic_inplace_change_t
{
   uint32_t is_inplace;
   /**< @valuesbul
        - 0 -- Indicates module changed to non-inplace
        - Non zero -- Indicates module changed to inplace @tablebulletend */
};

typedef struct capi_event_data_to_dsp_client_v2_t capi_event_data_to_dsp_client_v2_t;

/** Payload for the #CAPI_EVENT_DATA_TO_DSP_CLIENT_V2 event.
 */
struct capi_event_data_to_dsp_client_v2_t
{
   uint64_t dest_address;
   /**< Address to which this event is to be sent. The address that was
        provided during registration must be used. */

   uint32_t token;
   /**< Optional token that indicates additional information, such as an
        instance identifier. */

   uint32_t event_id;
   /**< Identifies the event. */

   capi_buf_t payload;
   /**< Buffer that contains the payload.

        This buffer can be safely destroyed or reused once the callback
        returns. */
};

typedef struct capi_event_data_to_dsp_service_t capi_event_data_to_dsp_service_t;

/** Payload for the #CAPI_EVENT_DATA_TO_DSP_SERVICE event.
 */
struct capi_event_data_to_dsp_service_t
{
   uint32_t param_id;
   /**< Indicates the type of data that is present in the payload. */

   uint32_t token;
   /**< Optional token that indicates additional information, such as an
        instance identifier. */

   capi_buf_t payload;
   /**< Buffer that contains the payload.

        This buffer can be safely destroyed or reused once the callback
        returns. */
};

typedef struct capi_event_get_data_from_dsp_service_t capi_event_get_data_from_dsp_service_t;

/** Payload for the #CAPI_EVENT_GET_DATA_FROM_DSP_SERVICE event.
 */
struct capi_event_get_data_from_dsp_service_t
{
   uint32_t param_id;
   /**< Indicates the type of data that is required from the framework. */

   uint32_t token;
   /**< Optional token that indicates additional information, such as an
        instance identifier. */

   capi_buf_t payload;
   /**< Buffer that contains the payload.

        This buffer can be safely destroyed or reused once the callback
        returns. @newpagetable */
};

typedef struct capi_event_process_state_t capi_event_process_state_t;

/** Payload for the #CAPI_EVENT_PROCESS_STATE event.
 */
struct capi_event_process_state_t
{
   bool_t is_enabled;
   /**< Specifies whether the module is enabled. If a module is disabled, its
        capi_vtbl_t::process() function is not called.

        @valuesbul
        - 0 -- Disabled
        - 1 -- Enabled (Default) @tablebulletend */
};

typedef struct capi_event_algorithmic_delay_t capi_event_algorithmic_delay_t;

/** Payload for the #CAPI_EVENT_ALGORITHMIC_DELAY event.
 */
struct capi_event_algorithmic_delay_t
{
   uint32_t delay_in_us;
   /**< Algorithmic delay in microseconds caused by the module.

        This value <b>must not</b> include a buffering delay. Otherwise,
        metadata offset adjustments will be calculated incorrectly. */
};

typedef struct capi_event_headroom_t capi_event_headroom_t;

/** Deprecated. Use control links instead.

    Payload for the #CAPI_EVENT_HEADROOM event.
 */
struct capi_event_headroom_t
{
   uint32_t headroom_in_millibels;
   /**< Headroom requirement of the module. The default is zero. */
};

typedef struct capi_port_data_threshold_change_t capi_port_data_threshold_change_t;

/** Payload for the #CAPI_EVENT_PORT_DATA_THRESHOLD_CHANGE event.
 */
struct capi_port_data_threshold_change_t
{
   uint32_t new_threshold_in_bytes; /**< Value of the threshold for a port.
                                         @newpagetable */
};

typedef struct capi_library_base_t capi_library_base_t;

/** Function pointers that are the first element of every library virtual
    function table.
*/
struct capi_library_base_t
{
   uint32_t (*get_interface_id)(void *obj_ptr);
   /**< Returns the ID associated with the interface that this object
        implements. */

   void (*end)(void *obj_ptr);
   /**< De-initializes the object and frees the memory associated with it. The
        object pointer is not valid after this call. */
};

typedef struct capi_event_get_library_instance_t capi_event_get_library_instance_t;

/** Payload for the #CAPI_EVENT_GET_LIBRARY_INSTANCE event.
*/
struct capi_event_get_library_instance_t
{
   uint32_t id;  /**< Identifies the library. */
   void *   ptr; /**< Pointer to the instance of the library. */
};

typedef struct capi_event_dlinfo_t capi_event_dlinfo_t;

/** Payload for the #CAPI_EVENT_GET_DLINFO event.
*/
struct capi_event_dlinfo_t
{
   uint32_t is_dl;
   /**< Indicates whether the SO file is dynamically loaded.

        @valuesbul
        - TRUE -- File is dynamically loaded
        - FALSE -- Otherwise

        The rest of this payload is applicable only if the SO file is loaded. */

   uint32_t load_addr_lsw;
   /**< Lower 32 bits of the physical address where the SO file is loaded. */

   uint32_t load_addr_msw;
   /**< Upper 32 bits of the physical address where the SO file is loaded.

        The 64-bit number formed by load_addr_lsw and load_addr_msw must
        be 32-byte aligned and must have been previously mapped. */

   uint32_t load_size;
   /**< Size (in bytes) of the loaded SO file. */
};

typedef struct capi_event_hw_accl_proc_delay_t capi_event_hw_accl_proc_delay_t;

/** Payload for the #CAPI_EVENT_HW_ACCL_PROC_DELAY event.
*/

struct capi_event_hw_accl_proc_delay_t
{
   uint32_t delay_in_us;
   /**< Hardware  requirement of the module. Default value is 0. */
};

typedef struct capi_event_island_vote_t capi_event_island_vote_t;

/** Payload for the #CAPI_EVENT_ISLAND_VOTE event.
*/

struct capi_event_island_vote_t
{
   uint32_t island_vote;
   /**< Island vote of the module placed in an LPI container.

        @valuesbul
        - 0 -- Vote for island entry
        - 1 -- Vote against island entry @tablebulletend
   */
};

/** @} */ /* end_addtogroup capi_events */

#endif /* #ifndef CAPI_EVENTS_H */
