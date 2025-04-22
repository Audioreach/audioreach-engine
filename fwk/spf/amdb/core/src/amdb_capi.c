/**
 * \file amdb_capi.c
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "amdb_internal.h"

extern capi_vtbl_t capi_vtbl_wrapper;
extern amdb_t *g_amdb_ptr;
/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
capi_err_t capi_wrapper_end(capi_t *icapi)
{
   capi_wrapper_t *me = (capi_wrapper_t *)icapi;

   me->icapi->vtbl_ptr->end(me->icapi);
   amdb_node_dec_dl_ref(&me->capi_ptr->node);
   amdb_node_dec_mem_ref(&me->capi_ptr->node);
   return CAPI_EOK;
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
ar_result_t capi_wrapper_init(capi_wrapper_t *me, capi_t *icapi, amdb_capi_t *capi_ptr)
{
   me->pvtbl    = &capi_vtbl_wrapper;
   me->icapi    = icapi;
   me->capi_ptr = capi_ptr;


   if (!capi_ptr->node.flags.is_static)
   {
      amdb_dynamic_t* dyn = amdb_node_get_dyn(&capi_ptr->node);

      posal_mutex_lock_inline(&dyn->dl_mutex);
      (void)amdb_node_inc_dl_ref(&capi_ptr->node);
      posal_mutex_unlock_inline(&dyn->dl_mutex);
   }
   amdb_node_inc_mem_ref(&capi_ptr->node);
   return AR_EOK;
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
capi_err_t amdb_capi_get_static_properties_f(void *           handle_ptr,
                                             capi_proplist_t *init_set_properties,
                                             capi_proplist_t *static_properties)
{
   capi_err_t   err      = CAPI_EOK;
   amdb_capi_t *me_ptr   = (amdb_capi_t *)handle_ptr;
   capi_prop_t *prop_ptr = 0;
   uint32_t     size     = 0;
   uint32_t     i;

   err = me_ptr->get_static_properties_f(init_set_properties, static_properties);
   if (CAPI_FAILED(err))
   {
      return err;
   }

   for (i = 0; i < static_properties->props_num; i++)
   {
      if (static_properties->prop_ptr[i].id == CAPI_INIT_MEMORY_REQUIREMENT)
      {
         prop_ptr = &static_properties->prop_ptr[i];
      }
   }
   if (0 != prop_ptr)
   {
      size = ((capi_init_memory_requirement_t *)prop_ptr->payload.data_ptr)->size_in_bytes;
      size = align_to_8_byte(size);
      size += align_to_8_byte(sizeof(capi_wrapper_t));
      ((capi_init_memory_requirement_t *)prop_ptr->payload.data_ptr)->size_in_bytes = size;
   }

   return AR_EOK;
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
capi_err_t amdb_capi_init_f(void *handle_ptr, capi_t *icapi, capi_proplist_t *init_set_properties)
{
   amdb_capi_t *   me_ptr      = (amdb_capi_t *)handle_ptr;
   capi_wrapper_t *wrapper_ptr = (capi_wrapper_t *)icapi;
   capi_err_t      err         = CAPI_EOK;

   icapi = (capi_t *)((char *)wrapper_ptr + align_to_8_byte(sizeof(capi_wrapper_t)));

   err = me_ptr->init_f(icapi, init_set_properties);

   if ((err != CAPI_EOK) && (err != CAPI_EUNSUPPORTED))
   {
      return err;
   }

   return capi_wrapper_init(wrapper_ptr, icapi, me_ptr);
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None
  If a function name is NULL, the internally stored names will be used
  --------------------------------------------------------------------------------------------------------------------*/
ar_result_t capi_resolve_symbols(amdb_node_t *node_ptr)
{
   ar_result_t  result    = AR_EOK;

   result = amdb_resolve_symbols(node_ptr);
   return result;
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
void capi_store_static_functions(amdb_node_t *node_ptr, void *f1_ptr, void *f2_ptr)
{
   amdb_capi_t *me = (amdb_capi_t *)(node_ptr);

   me->get_static_properties_f = (capi_get_static_properties_f)(f1_ptr);
   me->init_f                  = (capi_init_f)(f2_ptr);
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
void capi_clear_symbols(amdb_node_t *node_ptr)
{
   amdb_capi_t *me = (amdb_capi_t *)(node_ptr);

   me->get_static_properties_f = NULL;
   me->init_f                  = NULL;
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
void capi_end(amdb_node_t *node_ptr)
{
   amdb_capi_t *me = (amdb_capi_t *)(node_ptr);


   amdb_node_vtbl[me->node.flags.interface_type - 1].clear_symbols(&me->node);
   if (!node_ptr->flags.is_static)
   {
      amdb_dynamic_t* dyn = amdb_node_get_dyn(node_ptr);
      dyn->tag_str = NULL;
   }
}

/*** stub ******************************************************************/

// If a function name is NULL, the internally
/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
ar_result_t stub_resolve_symbols(amdb_node_t *node_ptr)
{

   return AR_EOK;
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
void stub_store_static_functions(amdb_node_t *node_ptr, void *f1_ptr, void *f2_ptr)
{
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
void stub_clear_symbols(amdb_node_t *node_ptr)
{
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
void stub_end(amdb_node_t *node_ptr)
{
   amdb_stub_t *me = (amdb_stub_t *)(node_ptr);
   amdb_node_vtbl[me->node.flags.interface_type - 1].clear_symbols(&me->node);
}
