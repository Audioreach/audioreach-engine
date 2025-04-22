/**
 * \file amdb_capi_island.c
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "amdb_internal.h"

capi_vtbl_t capi_vtbl_wrapper = {
   capi_wrapper_process,        capi_wrapper_end,
   capi_wrapper_set_param,      capi_wrapper_get_param,
   capi_wrapper_set_properties, capi_wrapper_get_properties,
};

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
capi_err_t capi_wrapper_process(capi_t *icapi, capi_stream_data_t *input[], capi_stream_data_t *output[])
{
   capi_wrapper_t *me = (capi_wrapper_t *)icapi;
   return me->icapi->vtbl_ptr->process(me->icapi, input, output);
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
capi_err_t capi_wrapper_set_param(capi_t *                icapi,
                                  uint32_t                param_id,
                                  const capi_port_info_t *port_info_ptr,
                                  capi_buf_t *            params_ptr)
{
   capi_wrapper_t *me = (capi_wrapper_t *)icapi;
   return me->icapi->vtbl_ptr->set_param(me->icapi, param_id, port_info_ptr, params_ptr);
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
capi_err_t capi_wrapper_get_param(capi_t *                icapi,
                                  uint32_t                param_id,
                                  const capi_port_info_t *port_info_ptr,
                                  capi_buf_t *            params_ptr)
{
   capi_wrapper_t *me = (capi_wrapper_t *)icapi;
   return me->icapi->vtbl_ptr->get_param(me->icapi, param_id, port_info_ptr, params_ptr);
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
capi_err_t capi_wrapper_set_properties(capi_t *icapi, capi_proplist_t *props_ptr)
{
   capi_wrapper_t *me = (capi_wrapper_t *)icapi;
   return me->icapi->vtbl_ptr->set_properties(me->icapi, props_ptr);
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
capi_err_t capi_wrapper_get_properties(capi_t *icapi, capi_proplist_t *props_ptr)
{
   capi_wrapper_t *me = (capi_wrapper_t *)icapi;
   return me->icapi->vtbl_ptr->get_properties(me->icapi, props_ptr);
}
