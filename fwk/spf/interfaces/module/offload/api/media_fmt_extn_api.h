#ifndef MEDIA_FMT_EXTN_API_H
#define MEDIA_FMT_EXTN_API_H

/**
 *   \file media_fmt_extn_api.h
 *   \brief
 *        This file contains private definition relevant to media format
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/** @h2xml_title1           {Media Format Private APIs}
    @h2xml_title_agile_rev  {Media Format Private APIs}
    @h2xml_title_date       {July 12, 2019} */
/**
   @h2xmlx_xmlNumberFormat {int}
*/

/*************************************************** Internal PCM Media Format
 * *************************************************/

/**
 * Media format ID for identifying internal PCM streams with extended media format.
 */
#define MEDIA_FMT_ID_PCM_EXTN 0x09001011

/**
 * Operating frame size of the module to the client.
 *
 * Payload shmem_ep_frame_size_t
 *
 * Event must be registered with APM_CMD_REGISTER_MODULE_EVENTS
 */
#define OFFLOAD_EVENT_ID_SH_MEM_EP_OPERATING_FRAME_SIZE 0x03001004

/**
 *  Param that indicates required frame-size of the future buffers in the stream.

    Payload is of type shmem_ep_frame_size_t
 */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
/**
 *  Indicates the required buffer-size of the future buffers in the stream.
 *  Payload is of type shmem_ep_frame_size_t
 *
 * Some opcodes that use this payload are:
 * -DATA_EVENT_ID_SH_MEM_EP_OPERATING_FRAME_SIZE
 *
 */
struct shmem_ep_frame_size_t
{
   uint32_t buf_size_in_bytes;
   /**< @h2xmle_description {buffer size in bytes}
        @h2xmle_default     {960}
        @h2xmle_range       {1 - 1000000}
        @h2xmle_policy      {Basic} */

   uint32_t ep_module_type;
   /**< @h2xmle_description {module type of the EP module}
        @h2xmle_default     {0-WR_SHARED_MEM_EP}
        @h2xmle_range       {0-WR_SHARED_MEM_EP
                             1-RD_SHARED_MEM_EP}
        @h2xmle_policy      {Basic} */

   uint32_t ep_miid;
   /**< @h2xmle_description {module instance ID of the EP module}
        @h2xmle_default     {960}
        @h2xmle_range       {0-7FFFFFFF}
        @h2xmle_policy      {Basic} */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct shmem_ep_frame_size_t shmem_ep_frame_size_t;


/**
 * Media format sent from this module to the client.
 *
 * Payload media_format_t
 *
 * Event must be registered with APM_CMD_REGISTER_MODULE_EVENTS
 */
#define OFFLOAD_EVENT_ID_RD_SH_MEM_EP_MEDIA_FORMAT                        0x03001009

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* MEDIA_FMT_EXTN_API_H */
