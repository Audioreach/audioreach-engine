#ifndef AMDB_STATIC_H
#define AMDB_STATIC_H
/**
 * \file amdb_static.h
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "amdb_api.h"
#include "posal.h"
#include "capi.h"
#include "spf_list_utils.h"
#include "amdb_cntr_if.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus


/*----------------------------------------------------------------------------------------------------------------------
 * AMDB API
 *--------------------------------------------------------------------------------------------------------------------*/

/** Initializes the global AMDB */
ar_result_t amdb_init(POSAL_HEAP_ID heap_id, bool_t init_cmd_thread);

/** De-Initializes the global AMDB */
void amdb_deinit(bool_t deinit_cmd_thread);

/** Details of the module will be added to AMDB data base.
    Can be used to add custom modules as well (public API) */
ar_result_t amdb_register(uint32_t    module_type,
                          uint32_t    module_id,
                          void *      get_static_properties_f,
                          void *      init_f,
                          uint32_t    filename_len,
                          const char *filename_str,
                          uint32_t    tag_len,
                          const char *tag_str,
                          bool_t      is_built_in);

/** Used de-register module data from AMDB.
    If the module is not in use, it's data will be removed from AMDB */
ar_result_t amdb_deregister(uint32_t module_id);


/** function to unload all dynamic modules registered using ar_register_custom_modules. */
ar_result_t amdb_reset(bool_t is_flush_needed, bool_t is_reset_needed);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // AMDB_STATIC_H
