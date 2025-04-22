#ifndef _AMDB_CNTR_IF_H_
#define _AMDB_CNTR_IF_H_
/**
 * \file amdb_cntr_if.h
 *
 * \brief
 *     This file defines AMDB to container functions.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "amdb_api.h"

/*----------------------------------------------------------------------------------------------------------------------
 * Interface types not exposed in the API
 *--------------------------------------------------------------------------------------------------------------------*/
#define AMDB_INTERFACE_TYPE_STUB (1)

/*----------------------------------------------------------------------------------------------------------------------
 * Module types not exposed in the API
 *--------------------------------------------------------------------------------------------------------------------*/
/** Module is of type framework-module. */
#define AMDB_MODULE_TYPE_FRAMEWORK (1)

/*----------------------------------------------------------------------------------------------------------------------
 * Handle information that is provided by the AMDB per module.
 *--------------------------------------------------------------------------------------------------------------------*/

typedef struct amdb_module_handle_info_t
{
   uint16_t interface_type; /*<< To be filled by AMDB. AMDB will return the interface type of the module
                                  if it returns a valid handle. */
   uint16_t    module_type; /*<< returned by AMDB. */
   int         module_id;   /*<< To be filled by the caller for CAPI and CAPIv2 modules. */
   void *      handle_ptr;
   ar_result_t result; /*<< To be filled by AMDB. If result is not AR_EOK, the handle is not valid. */
} amdb_module_handle_info_t;

/*<< Function signature for the callback function that is called when all the module handles are ready. */
typedef void (*amdb_get_modules_callback_f)(void *callback_context);


/** API to get the handles for list of modules. Modules should be registered
 *    @module_handle_info_list_ptr  : A list of elements, for the modules whose handles are required
 *    @callback_function            : The callback func that is called when the handles are available for all modules.
 *                                     If NULL, this function will be blocking.
 *    @callback_context             : Context to be passed to the callback function.*/
void amdb_request_module_handles(spf_list_node_t *           module_handle_info_list_ptr,
                                 amdb_get_modules_callback_f callback_function,
                                 void *                      callback_context);

/** API to release the handles acquired from amdb_get_modules_request_list
 *    @module_handle_info_list_ptr  : Array of num_modules_elements, whose handles are to be released.
 *                                     If the handle value is NULL or if the 'result' value is not EOK,
 *                                     the corresponding entry will be ignored.*/
void amdb_release_module_handles(spf_list_node_t *module_handle_info_list_ptr); //


/** function to get the dlinfo for any module. only dynamically loaded modules have dlinfo.
 *    @module_handle_info: handle to the amdb.
 *    @is_dl:              is module dynamically loaded.
 *    @start_addr:         start address (virtual) where the lib is loaded.*/
void amdb_get_dl_info(amdb_module_handle_info_t *module_handle_info,
                      bool_t *                   is_dl,
                      uint32_t **                start_addr,
                      uint32_t *                 so_size);

/*----------------------------------------------------------------------------------------------------------------------
 * CAPI APIs
 *--------------------------------------------------------------------------------------------------------------------*/
capi_err_t amdb_capi_get_static_properties_f(void *           handle_ptr,
                                             capi_proplist_t *init_set_properties,
                                             capi_proplist_t *static_properties);

capi_err_t amdb_capi_init_f(void *handle_ptr, capi_t *_pif, capi_proplist_t *init_set_properties);



#endif //_IRM_CNTR_IF_H_
