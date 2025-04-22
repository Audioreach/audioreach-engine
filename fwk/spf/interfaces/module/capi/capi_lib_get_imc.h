#ifndef ELITE_LIB_GET_IMC_H
#define ELITE_LIB_GET_IMC_H

/**
 *   \file capi_lib_get_imc.h
 *   \brief
 *        A CAPI supporting library for inter-module communication with other CAPI modules
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

/** @addtogroup capi_lib_imc
    @xreflabel{hdr:ImcLibrary}

The Intermodule Communication (IMC) library (ELITE_LIB_GET_IMC) is used to
communicate with other CAPI modules that are available in the system.

To establish intermodule communication between a destination and one or more
sources, the sources must use a key to register with the IMC database. The key
must be unique for each communication instance, and it must be same for both
destination and sources to communicate with each other.
 - The first four bytes of the key correspond to the communication ID that is
   specific to the type of modules.
 - The bytes following the first four bytes must distinguish between different
   instances of the same communication.

When registering for IMC, use the following information to determine the key
(id0&nbsp;=&nbsp;first 4 bytes, id1&nbsp;=&nbsp;next 4 bytes, and so on).

@tbl_lib_imc_regkey{
One destination and one source & Unique communication ID &
   Destination module ID       & Source module ID        &
   Distinguishes between different instances of the same communication.
   A service session ID can be used.
   @tblendline
One destination and multiple sources & Unique communication ID &
   Destination module ID             & Session ID & ~~~~~ --
   @tblendline
Multiple destinations & \multicolumn{4}{l|}{Source module must open
   communication with different destinations using} \tn
   and one source     & \multicolumn{4}{l|}{the corresponding keys.
   \vspace{1pt}} \tn \hline}
 */

/** @addtogroup capi_lib_imc
@{ */

/** Unique identifier of the IMC supporting library.

  @subhead{To use this library for a source module}

  -# Get an object of type #elite_lib_get_imc_t using
     #CAPI_EVENT_GET_LIBRARY_INSTANCE with id = #ELITE_LIB_GET_IMC. \n
     @vertspace{1.5}
     The object is an instance of the library, together with the virtual
     function table (#elite_lib_get_imc_vtable_t) for the library interface.
  -# Call src_register() to register with the IMC database and get the source
     handle. \n @vertspace{1.5}
     Provide the source buffer queue length (which must be a power of 2).
  -# Call src_open_comm() to open communication
     with a key and get the destination handle. \n @vertspace{1.5}
     Once a source is registered, it can open multiple connections with
     different destinations. @newpage
  -# Allocate and transfer buffers to the destination queue in one of two ways:
     @vertspace{1.5}
      - Call src_allocate_packet() to allocate a single buffer and push it to
        the destination. \n @vertspace{1.5}
        When the destination returns the buffer, the source must free or resend
        the buffer with the new payload to the destination (src_return_buf()).
        @vertspace{-1}
      - Call src_allocate_push_self_buf() to allocate a required number of
        buffers with a specific buffer size and push them to their own queue.
        The number of buffers must be less than the source queue length. \n
        @vertspace{-1}
  -# When required, call src_pop_buf() to pop the buffers from their queue and
     send them to the destination number of buffers. \n @vertspace{1.5}
     If the source failed to push the buffers to the destination, it must push
     the buffers to their own queue (src_push_buf()).
  -# Once communication is done, call src_close_connection() to close the
     connection for each destination.
  -# Call src_deregister() to deregister the source queue from the IMC
     database.
  -# Call b.end() (#capi_vtbl_t::end()) to return the #elite_lib_get_imc_t
     object.

  @subhead{To use this library for a destination module}

  -# Get an object of type #elite_lib_get_imc_t using
     #CAPI_EVENT_GET_LIBRARY_INSTANCE with id = #ELITE_LIB_GET_IMC.
     \n @vertspace{1.5}
     The object is an instance of the library, together with the virtual
     function table (#elite_lib_get_imc_vtable_t) for the library interface.
  -# Call dest_register() to register with the IMC database and get a
     destination handle.
  -# Call dest_pop_buf() to pop the buffers from the queue.
  -# Process the buffers.
  -# Call dest_return_buf() to return the buffers to the source with the
     results.
  -# Call dest_deregister() to deregister the destination queue from the IMC
     database.
  -# Call b.end() (#capi_vtbl_t::end()) to return the #elite_lib_get_imc_t
     object.
 */
#define ELITE_LIB_GET_IMC 0x00010E5D

/*------------------------------------------------------------------------------
 * Interface definition
 *----------------------------------------------------------------------------*/

typedef struct imc_src_info_t  imc_src_info_t;
typedef struct imc_dest_info_t imc_dest_info_t;

typedef struct elite_lib_get_imc_vtable_t elite_lib_get_imc_vtable_t;

/** Virtual table of the IMC library interface.
 */
struct elite_lib_get_imc_vtable_t
{
   capi_library_base_t b;
   capi_err_t (*src_register)(imc_src_info_t **src_handle, uint32_t src_bufq_len);
   capi_err_t (*src_open_comm)(imc_src_info_t *  src_handle,
                               imc_dest_info_t **dst_handle,
                               const int8_t *    keyptr,
                               const uint32_t    keylen);
   capi_err_t (*src_allocate_push_self_buf)(imc_src_info_t *src_handle, uint32_t payload_size, uint32_t num_buffers);
   capi_err_t (*src_allocate_packet)(imc_src_info_t *src_handle, void **payload_ptr, uint32_t payload_size);
   capi_err_t (*src_pop_buf)(imc_src_info_t *src_handle,
                             void **         payload_ptr,
                             capi_err_t *    resp_result_ptr,
                             bool_t *        is_queue_empty);
   capi_err_t (*src_push_buf)(imc_src_info_t * src_handle,
                              imc_dest_info_t *dst_handle,
                              void *           payload_ptr,
                              uint32_t         payload_size);
   capi_err_t (*src_return_buf)(imc_src_info_t *src_handle, void *payload_ptr, capi_err_t response_result);
   capi_err_t (*src_close_connection)(imc_src_info_t *src_handle, imc_dest_info_t *dst_handle);
   capi_err_t (*src_deregister)(imc_src_info_t *src_handle);
   capi_err_t (*dest_register)(imc_dest_info_t **dst_handle,
                               const int8_t *    keyptr,
                               const uint32_t    keylen,
                               uint32_t          dest_dataq_len);
   capi_err_t (*dest_pop_buf)(imc_dest_info_t *dst_handle,
                              void **          payload_ptr,
                              uint32_t *       payload_actual_size_ptr,
                              uint32_t *       payload_max_size_ptr,
                              bool_t *         is_queue_empty);
   capi_err_t (*dest_return_buf)(imc_dest_info_t *dst_handle, void *payload_ptr, capi_err_t response_result);
   capi_err_t (*dest_deregister)(imc_dest_info_t *dst_handle);
};

typedef struct elite_lib_get_imc_t elite_lib_get_imc_t;

/** Contains a pointer to the IMC virtual function table that is defined in
    #elite_lib_get_imc_vtable_t.
*/
struct elite_lib_get_imc_t
{
   const elite_lib_get_imc_vtable_t *vtable;
   /**< Pointer to the virtual function table. */
};

/** @} */ /* end_addtogroup capi_lib_imc */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* ELITE_LIB_GET_IMC_H */
