#ifndef DTMF_GEN_METADATA_API_H
#define DTMF_GEN_METADATA_API_H

/**
 *   \file dtmf_gen_metadata_api.h
 *   \brief
 *        This file Metadata ID and payloads for DTMF Generator
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "dtmf_gen_api.h"

/** @h2xml_title1           {DTMF Gen Metadata API}
    @h2xml_title_agile_rev  {DTMF Gen Metadata API}
    @h2xml_title_date       {June 17, 2019} */

/** Metadata ID for DTMF generation */
#define MODULE_MD_ID_DTMF_GEN 0x0A001033

typedef struct module_md_dtmf_gen_t module_md_dtmf_gen_t;
struct module_md_dtmf_gen_t
{
   // configuration for DTMF Generator
   param_id_dtmf_gen_tone_cfg_t dtmf_gen_tone_cfg;

   // flag to determine if DTMF is mixed or replaces output
   bool_t unmixed_dtmf_output;
};

#endif // DTMF_GEN_METADATA_API_H
