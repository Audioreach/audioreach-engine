#ifndef _AUDIOSS_DSP_MEM_API_H_
#define _AUDIOSS_DSP_MEM_API_H_
/**
 * \file audioss_dsp_mem_api.h
 * \brief
 *    This file contains Audioss DSP memory util definitions
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */
// clang-format off
/*
*/
// clang-format on

#include "ar_defs.h"

/** This param is used to allocate or free the DSP memory.
The client can request for memory from the DSP and once the
memory allocation is done client can read/write to that memory section.
Client can free the memory using the same parameter.
This can be considered as a remote malloc by client.
 */
#define PARAM_ID_RSC_AUDIOSS_DSP_MEM 0x0800151E

/** The macro AUDIOSS_DRAM_MEMORY can be used to mention the type of
 * memory that is to be allocated or freed. Currently only DRAM type is
 * supported.
 */
#define AUDIOSS_DRAM_MEMORY 1

/* This structure details the payload for requesting the memory from DSP*/
#include "spf_begin_pack.h"
struct audioss_dsp_mem_request_t
{
   uint32_t mem_type;
   /**< type of memory, from where client is requesting memory
   Currently on DRAM memory type(1) is supported. */
   uint32_t size_in_bytes;
   /**< size of memory required in bytes */
}
#include "spf_end_pack.h"
;
typedef struct audioss_dsp_mem_request_t  audioss_dsp_mem_request_t ;

/* This structure details the payload for the response sent to the client
after allocating the DSP memory. */
#include "spf_begin_pack.h"
struct audioss_dsp_mem_request_rsp_t
{
   uint32_t size_in_bytes;
   /**< size of memory to be released in bytes */
   uint32_t phy_addr_lsw;
   /** <least significant word of the adress */
   uint32_t phy_addr_msw;
   /** <most significant word of the adress */
}
#include "spf_end_pack.h"
;
typedef struct audioss_dsp_mem_request_rsp_t audioss_dsp_mem_request_rsp_t;

/* This structure details the payload of the request sent from client
to free the DSP memory. */
#include "spf_begin_pack.h"
struct audioss_dsp_mem_release_t
{
   uint32_t mem_type;
   /**< type of memory, from where client is requesting memory
   Currently on DRAM memory type(1) is supported. */
   uint32_t size_in_bytes;
   /**< size of memory to be released in bytes */
   uint32_t phy_addr_lsw;
   /** <least significant word of the adress */
   uint32_t phy_addr_msw;
   /** <most significant word of the adress */
}
#include "spf_end_pack.h"
;
typedef struct audioss_dsp_mem_release_t audioss_dsp_mem_release_t;

#endif /* _AUDIOSS_DSP_MEM_API_H_ */
