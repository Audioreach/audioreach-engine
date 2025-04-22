/**
 * \file amdb_static_loading.c
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "amdb_internal.h"

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  None
  --------------------------------------------------------------------------------------------------------------------*/
ar_result_t amdb_init_dynamic(POSAL_HEAP_ID heap_id)
{
   return AR_EOK;
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : result

  None
  --------------------------------------------------------------------------------------------------------------------*/
void amdb_deinit_dynamic()
{
}
/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
void amdb_node_inc_dl_ref(amdb_node_t *me)
{
   return;
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
void amdb_node_dec_dl_ref(amdb_node_t *me)
{
   return;
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
ar_result_t node_init_dynamic(amdb_node_t * me,
                              bool_t        is_built_in,
                              uint32_t      tag_len,
                              const char *  tag_str,
                              uint32_t      filename_len,
                              const char *  filename_str,
                              POSAL_HEAP_ID heap_id)
{
   return AR_EOK;
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
ar_result_t check_dynamic_loading(amdb_node_t *me, bool_t *to_be_loaded)
{
   return AR_EOK;
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
void handle_get_modules_request_list(spf_list_node_t *           module_handle_info_list_ptr,
                                     amdb_get_modules_callback_f callback_function,
                                     void *                      callback_context)
{
   while (module_handle_info_list_ptr)
   {
      bool_t                     to_be_loaded = FALSE;
      amdb_module_handle_info_t *h_info_ptr   = (amdb_module_handle_info_t *)module_handle_info_list_ptr->obj_ptr;
      h_info_ptr->result                      = amdb_get_node(h_info_ptr, &to_be_loaded);
      module_handle_info_list_ptr             = module_handle_info_list_ptr->next_ptr;
   }
   if (NULL != callback_function)
   {
      callback_function(callback_context);
   }
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
void amdb_dynamic_loader(uint64_t task_info)
{
   return;
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
ar_result_t node_wrapper_dlopen(amdb_node_t *me, const char *filename_str)
{
   return AR_EOK;
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
void amdb_get_dl_info(amdb_module_handle_info_t *module_handle_info, // handle to the amdb.
                      bool_t *                   is_dl,              // is module dynamically loaded.
                      uint32_t **                start_addr,         // start address (virtual) where the lib is loaded.
                      uint32_t *                 so_size)                             // size of the loaded library.
{
   return;
}

ar_result_t  amdb_resolve_symbols(amdb_node_t *node_ptr)
{
	return AR_EOK;
}

ar_result_t amdb_dyn_module_version(amdb_node_t *node_ptr, amdb_module_version_info_payload_t *module_version_ptr)
{
    return AR_EOK;
}