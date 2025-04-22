#ifndef ELITE_LIB_GET_CAPI_MODULE_H
#define ELITE_LIB_GET_CAPI_MODULE_H

/**
 *   \file capi_lib_get_capi_module.h
 *   \brief
 *        A CAPI supporting library for creating other CAPI modules
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

/** @addtogroup capi_lib_module_create
The Module Creation library (ELITE_LIB_GET_CAPI_MODULE) is used to create
instances of other CAPI modules that are available in the system.

When working with CAPI modules, use the following table to determine the
values of id1 and id2.

@tbl_lib_modcreate{
Generic    & Set module ID & Set to zero   @tblendline
Encoder    & Output format & Set to zero   @tblendline
Decoder    & Input format  & Set to zero   @tblendline
Converter  & Input format  & Output format @tblendline
Packetizer & Input format  & Set to zero   @tblendline}
*/

/** @addtogroup capi_lib_module_create
@{ */

/** Unique identifier of the Module Creation supporting library.

  @subhead{To use this library}
  -# Get an object of type #elite_lib_get_capi_t by using
     #CAPI_EVENT_GET_LIBRARY_INSTANCE. \n @vertspace{1.5}
     The object is an instance of the library, together with the virtual
     function table (#elite_lib_get_capi_vtable) for the library interface.
  -# Call get_handle() with the appropriate values of type, id1, and id2 to
     get a handle that can be used to call the capi_get_static_properties_f()
     and capi_init_f() functions. @vertspace{1.5}
     - Use the handle to call get_static_properties_f() of the module
       using call_get_static_properties(). @vertspace{-1}
     - Use the handle to call the init() function of the module using
       call_init(). @vertspace{-1}
  -# Call call_init() to create multiple instances of the module with multiple
     calls to init().
  -# After the required instances are created, call release_handle() to
     release the handle. @vertspace{1.5}
     - Do not use the handle after this call. @vertspace{-1}
     - The module instances might continue to exist after the handle is
       released. @vertspace{-1}
  -# Query multiple capi handles using a single library instance.
     Repeat steps 2 to 4 for all modules that are to be created.
  -# Once all the modules are created and all the handles are closed, call
     b.end() (#capi_vtbl_t::end()) to destroy the library instance. \n
     @vertspace{1.5}
     The module instances might continue to exist after the library instance
     is destroyed. @newpage
*/
#define ELITE_LIB_GET_CAPI_MODULE 0x00010914

/*------------------------------------------------------------------------------
 * Interface definition
 *----------------------------------------------------------------------------*/
typedef struct elite_lib_capi_handle elite_lib_capi_handle;

/** Library type is Generic.

  Use this library type when there is no information about a module or when the
  module functionality is not one of the defined library types.
 */
#define ELITE_LIB_CAPI_TYPE_GENERIC 0

/** Library type is a Decoder. */
#define ELITE_LIB_CAPI_TYPE_DECODER 1

/** Library type is an Encoder. */
#define ELITE_LIB_CAPI_TYPE_ENCODER 2

/** Library type is a Converter.

  For example, the converter might be a transcoder that converts one bitstream
  input format to another bitstream format.
 */
#define ELITE_LIB_CAPI_TYPE_CONVERTER 3

/** Library type is a Packetizer.

    Data is packetized into one of the IEC 61937 or IEC 60958 packetization
    formats.
 */
#define ELITE_LIB_CAPI_TYPE_PACKETIZER 4

/** Library type is a Depacketizer.

    Data is depacketized into raw format from IEC 61937 or other packetization
    formats. @newpage
 */
#define ELITE_LIB_CAPI_TYPE_DEPACKETIZER 5

typedef struct elite_lib_get_capi_vtable elite_lib_get_capi_vtable;
typedef struct elite_lib_get_capi_t elite_lib_get_capi_t;

/** Virtual function table of the library interface.
*/
struct elite_lib_get_capi_vtable
{
   capi_library_base_t b;
   capi_err_t (*get_handle)(elite_lib_get_capi_t *  obj_ptr,
                            uint32_t                type,
                            uint32_t                id1,
                            uint32_t                id2,
                            elite_lib_capi_handle **h_ptr);
   capi_err_t (*call_get_static_properties)(elite_lib_get_capi_t * obj_ptr,
                                            elite_lib_capi_handle *h,
                                            capi_proplist_t *      init_set_properties,
                                            capi_proplist_t *      static_properties);
   capi_err_t (*call_init)(elite_lib_get_capi_t * obj_ptr,
                           elite_lib_capi_handle *h,
                           capi_t *               _pif,
                           capi_proplist_t *      init_set_properties);
   capi_err_t (*release_handle)(elite_lib_get_capi_t *obj_ptr, elite_lib_capi_handle *h);
};


/** Contains a pointer to the CAPI virtual function table that is defined in
    #elite_lib_get_capi_vtable.
 */
struct elite_lib_get_capi_t
{
   const elite_lib_get_capi_vtable *vtable;
   /**< Pointer to the virtual function table. */
};

/** @} */ /* end_addtogroup capi_lib_module_create */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* #ifndef ELITE_LIB_GET_CAPI_MODULE_H */
