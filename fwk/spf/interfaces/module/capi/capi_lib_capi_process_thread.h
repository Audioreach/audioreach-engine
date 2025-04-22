#ifndef ELITE_LIB_CAPI_PROCESS_THREAD_H
#define ELITE_LIB_CAPI_PROCESS_THREAD_H

/**
 *   \file capi_lib_capi_process_thread.h
 *   \brief
 *        A CAPI supporting library for launching the core process in a separate thread.
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
#include "capi.h"

/** @addtogroup capi_lib_process_thread
The CAPI Process Thread library (ELITE_LIB_CAPI_PROCESS_THREAD) is used to
create a separate thread and launch the core process (CAPI process) in that
thread.
*/

/** @addtogroup capi_lib_process_thread
@{ */

/** Unique identifier of the CAPI Process Thread library.

  @subhead{To use this library}

   -# Get an object of type elite_lib_capi_process_thread_util_t using
      #CAPI_EVENT_GET_LIBRARY_INSTANCE. \n @vertspace{1.5}
      The object is an instance of library, along with the virtual function
      table (#elite_lib_capi_process_thread_util_vtable) for the library
      interface.
   -# Call capi_pu_launch_thread() to launch a thread with a defined stack
      size.
   -# Call capi_pu_set_process() to set the core library process function to be
      launched in a separate thread. \n @vertspace{1.5}
      The core process function must take in frame size samples and provide
      frame size samples in every call.
   -# Call capi_pu_set_media_type() to set the number of channels and channel
      map. \n @vertspace{1.5}
      The library creates data buffers.
   -# Call capi_pu_process() to instantiate library processing.
   -# Call capi_pu_wait_for_processing_done() to tell the calling thread to
      wait before changing any configuration.
   -# Protect the media format changes and set or get parameters to ensure that
      there are no race conditions. \n @vertspace{1.5}
      One method is to wait for the thread to be idle.
   -# Call capi_pu_reset() to clear the stale data in the internal buffers of
      the thread utility library.
*/
#define ELITE_LIB_CAPI_PROCESS_THREAD 0x00010915

/*------------------------------------------------------------------------------
 * Interface definition
 *----------------------------------------------------------------------------*/

/**
  Function pointer that invokes the process call of the utility thread
  process.

  @datatypes
  capi_t \n
  capi_stream_data_t

  @param[in,out] _pif   Pointer to the module object.
  @param[in,out] input  Array of pointers to the input data for each input
                        port. \n @vertspace{3}
                        The length of the array is the number of input ports.
                        The client uses the #CAPI_PORT_NUM_INFO property to
                        set the number of ports. \n @vertspace{3}
                        The function must modify the actual_data_len field
                        to indicate how many bytes were consumed.
  @param[out] output    Array of pointers to the output data for each output
                        port. \n @vertspace{3}
                        The client uses the #CAPI_PORT_NUM_INFO property to
                        set the number of output ports. \n @vertspace{3}
                        The function sets the actual_data_len field to
                        indicate how many bytes were generated.

  @return
  None.

  @dependencies
  This separate thread process performs only core processing.
  @par
  All initializations must have been performed by the parent module that
  launches this thread.
 */
typedef void (*capi_pu_process_fn_t)(capi_t *_pif, const capi_stream_data_t *input, capi_stream_data_t *output);

typedef struct elite_lib_capi_process_thread_util_vtable elite_lib_capi_process_thread_util_vtable;

/** Virtual function table of the library interface.
 */
struct elite_lib_capi_process_thread_util_vtable
{
   capi_library_base_t b;
   capi_err_t (*capi_pu_process)(void *obj_ptr, const capi_stream_data_t *input, capi_stream_data_t *output);
   capi_err_t (*capi_pu_launch_thread)(uint32_t stack_size, void *obj_ptr, char *thread_name);
   capi_err_t (*capi_pu_set_media_type)(void *         obj_ptr,
                                        uint32_t       frame_size_in_bytes,
                                        uint32_t       num_input_buffers,
                                        const uint16_t input_channel_map[],
                                        uint32_t       num_output_buffers,
                                        const uint16_t output_channel_map[]);
   void (*capi_pu_set_process_function)(void *obj_ptr, capi_t *module_ptr, capi_pu_process_fn_t fn);
   void (*capi_pu_reset)(void *obj_ptr);
   void (*capi_pu_wait_for_processing_done)(void *obj_ptr);
};

typedef struct elite_lib_get_capi_t  elite_lib_get_capi_t;

/** Contains the pointer to the virtual table of the thread
  process utility defined in #elite_lib_capi_process_thread_util_vtable.
 */
struct elite_lib_capi_process_thread_util_t
{
   const elite_lib_capi_process_thread_util_vtable *vtable;
   /**< Pointer to the virtual function table. @newpagetable */
};

/** @} */ /* end_addtogroup capi_lib_process_thread */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* #ifndef ELITE_LIB_CAPI_PROCESS_THREAD_H */
