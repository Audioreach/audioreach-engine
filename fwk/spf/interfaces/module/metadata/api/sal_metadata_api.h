#ifndef SAL_METADATA_API_H
#define SAL_METADATA_API_H

/**
 *   \file sal_metadata_api.h
 *   \brief
 *        This file Metadata ID and payloads for SAL module
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/** @h2xml_title1           {SAL Metadata API}
    @h2xml_title_agile_rev  {SAL Metadata API}
    @h2xml_title_date       {June 7, 2019} */

/* When SAL receives this metadata, if unmixed output is TRUE,
   SAL copies data from input port to output port without mixing
  other input ports data */
#define MODULE_MD_ID_SAL_UNMIXED_OUTPUT 0x0A001034

typedef struct module_md_sal_unmixed_output_t module_md_sal_unmixed_output_t;
struct module_md_sal_unmixed_output_t
{
   // SAL copies data from input port to output port without mixing
   bool_t unmixed_output;
   // in unmixed mode, SAL attempts to process md from other inputs if this is set
   bool_t process_other_input_md;
};

#endif // SAL_METADATA_API_H
