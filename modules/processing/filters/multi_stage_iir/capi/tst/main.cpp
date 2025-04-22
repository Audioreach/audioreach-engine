/*==========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *==========================================================================*/

/**
 * @file main.cpp
 *
 * Standalone Setup for MSIIR Modules module
 */

#ifndef CAPI_MODULE_HEADER
#error "Please specify the include header for module"
#endif

#ifndef CAPI_STATIC_PROP_FUNCTION
#error "Please specify the capi static property function in makefile"
#endif

#ifndef CAPI_INIT_FUNCTION
#error "Please specify the capi init function in makefile"
#endif


#include "capi_test.h"
#include "media_fmt_api.h"
#include CAPI_MODULE_HEADER


int main(int argc, char* argv[])
{
    capi_err_t result = CAPI_EOK;

    module_info_t module;
    memset(&module, 0, sizeof(module_info_t));

    /* Read input arguments */
    args_t input_args;
    get_eargs(argc, argv, &input_args);

    if ((module.finp = fopen(input_args.input_filename, "rb")) == NULL)
    {
        fprintf(stderr, "%s:  ERROR CODE 2- Cannot open input file '%s'.\n",
                argv[0],
                input_args.input_filename);
        exit(-1);
    }

    if ((module.fout = fopen(input_args.output_filename, "wb")) == NULL)
    {
        fprintf(stderr, "%s:  ERROR CODE 3 - Cannot open output file '%s'.\n",
                argv[0],
                input_args.output_filename);
        fclose(module.finp);
        exit(-1);
    }
    if ((module.fCfg = fopen(input_args.config_filename, "rb")) == NULL)
    {
        fprintf(stderr, "%s:  ERROR CODE 3 - Cannot open config file '%s'.\n",
                argv[0],
                input_args.config_filename);
        fclose(module.finp);
        fclose(module.fout);
        exit(-1);
    }

    /* STEP 1: Get size requirements of CAPI */
    AR_MSG(DBG_HIGH_PRIO,"MAIN: -----------------");
    AR_MSG(DBG_HIGH_PRIO,"MAIN: Initialize module");
    AR_MSG(DBG_HIGH_PRIO,"MAIN: -----------------");

    /* Query for CAPI size */
    capi_proplist_t static_properties;
    static_properties.props_num = 1;
    capi_prop_t prop_ptr[static_properties.props_num];
    static_properties.prop_ptr = prop_ptr;

    /* Populate INIT_MEMORY_REQUIREMENT query */
    capi_init_memory_requirement_t mem_req;
    prop_ptr[0].id = CAPI_INIT_MEMORY_REQUIREMENT;
    prop_ptr[0].payload.data_ptr = (int8_t *)&mem_req;
    prop_ptr[0].port_info.is_valid = FALSE;

    result = CAPI_STATIC_PROP_FUNCTION(
            NULL, &static_properties);
    if (CAPI_EOK != result)
    {
        AR_MSG(DBG_HIGH_PRIO,
            "Failed to query for module size");
        fclose(module.finp);
        fclose(module.fout);
        fclose(module.fCfg);
        return CAPI_EFAILED;
    }

    AR_MSG(DBG_HIGH_PRIO,
          "%lu bytes required for module.",
          mem_req.size_in_bytes);

    /* STEP 2: Allocate memory */
    uint8_t *ptr = (uint8_t*)posal_memory_malloc(
            mem_req.size_in_bytes, POSAL_HEAP_DEFAULT);
    if (NULL == ptr)
    {
        AR_MSG(DBG_ERROR_PRIO,
            "MAIN: Memory allocation error");
        fclose(module.finp);
        fclose(module.fout);
        fclose(module.fCfg);
        return CAPI_ENOMEMORY;
    }
    module.module_ptr = (capi_t *)ptr;
    AR_MSG(DBG_HIGH_PRIO,
          "Module allocated for %lu bytes of memory at location 0x%p.",
          mem_req.size_in_bytes,
          ptr);

    /* STEP 3: Initialize module */
    capi_event_callback_info_t cb_info = capi_tst_get_cb_info(&module);
    capi_prop_t cb_prop;
    cb_prop.id = CAPI_EVENT_CALLBACK_INFO;
    cb_prop.payload.max_data_len = sizeof(cb_info);
    cb_prop.payload.actual_data_len = sizeof(cb_info);
    cb_prop.payload.data_ptr = reinterpret_cast<int8_t*>(&cb_info);

    capi_proplist_t init_proplist;
    init_proplist.props_num = 1;
    init_proplist.prop_ptr = &cb_prop;

    AR_MSG(DBG_HIGH_PRIO, "capiv2 init");
    result = CAPI_INIT_FUNCTION((capi_t *)ptr, &init_proplist);

    if (CAPI_EOK != result)
    {
        AR_MSG(DBG_ERROR_PRIO, "MAIN: Initialization error");
        posal_memory_free(ptr);
        fclose(module.finp);
        fclose(module.fout);
        fclose(module.fCfg);
        return result;
    }

    module.out_buffer_len = module.in_buffer_len;
    module.requires_data_buffering = FALSE;
    module.is_enabled = TRUE;
    module.alg_delay = 0;

    /* Run config file */
    AR_MSG(DBG_HIGH_PRIO,"MAIN: ----------------");
    AR_MSG(DBG_HIGH_PRIO,"MAIN: Run config file ");
    AR_MSG(DBG_HIGH_PRIO,"MAIN: ----------------");

    result = RunTest(&module);
    if(CAPI_EOK != result)
    {
        AR_MSG(DBG_ERROR_PRIO, "MAIN: Error in RunTest");
    }

    /* Destroy CAPI V2 and free memory */
    module.module_ptr->vtbl_ptr->end(module.module_ptr);

    posal_memory_free(ptr);

    fclose (module.finp);
    fclose (module.fout);
    fclose (module.fCfg);

    AR_MSG(DBG_HIGH_PRIO,"MAIN: Done");
    return result;
}

