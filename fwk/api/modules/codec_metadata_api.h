#ifndef CODEC_METADATA_API_H
#define CODEC_METADATA_API_H
/**
 * \file codec_metadata_api.h
 * \brief
 *    This file contains codec related metadata IDs.
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_defs.h"

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

/*# @h2xml_title1           {Metadata IDs}
    @h2xml_title_agile_rev  {Metadata IDs}
    @h2xml_title_date       {Apr 30 2019} */

/*# @h2xmlx_xmlNumberFormat {int} */


/** @ingroup spf_mods_metadata
    Metadata ID for encoder frame information.

    @msgpayload
    module_cmn_md_encoder_per_frame_info_t
*/
#define MODULE_CMN_MD_ID_ENCODER_FRAME_INFO              0x0A001052


/** @ingroup spf_mods_metadata
    Payload for #MODULE_CMN_MD_ID_ENCODER_FRAME_INFO.
    This structure contains metadata for the encoder frame information to be
    put in the read metadata buffer.
 */
#include "spf_begin_pack.h"
struct module_cmn_md_encoder_per_frame_info_t
{
   uint32_t          frame_size;
   /**< Size of each frame in bytes (for example, an encoded frame size).

        @values > 0 */

   uint32_t          timestamp_lsw;
   /**< Lower 32 bits of the 64-bit session time (microseconds) of the
        first sample in the frame. */

   uint32_t          timestamp_msw;
   /**< Upper 32 bits of the 64-bit session time (microseconds) of the
        first sample in the frame. */

   uint32_t          flags;
   /**< Frame flags.

        @valuesbul{for bit 31}
        - 1 -- Timestamp is valid
        - 0 -- Timestamp is not valid

        To set bit 31, use RD_SH_MEM_EP_BIT_MASK_TIMESTAMP_VALID_FLAG and
        RD_SH_MEM_EP_SHIFT_TIMESTAMP_VALID_FLAG.

        All other bits are reserved; the SPF sets them to 0. */
}
#include "spf_end_pack.h"
;
typedef struct module_cmn_md_encoder_per_frame_info_t module_cmn_md_encoder_per_frame_info_t;


/* Metadata ID for Codec.2.0 requirement. */

/** @ingroup spf_mods_metadata
    Identifier for the metadata that specifies the start of a buffer.

    @msgpayload
    module_cmn_md_buffer_start_t
 */
#define MODULE_CMN_MD_ID_BUFFER_START          0x0A001053


/** @ingroup spf_mods_metadata 
    Payload for #MODULE_CMN_MD_ID_BUFFER_START.
    This structure contains metadata to specify the start of the buffer.
 */
#include "spf_begin_pack.h"
struct module_cmn_md_buffer_start_t
{
   uint32_t buffer_index_lsw;
   /**< Lower 32 bits of the 64-bit unique buffer index. */

   uint32_t buffer_index_msw;
   /**< Upper 32 bits of the 64-bit unique buffer index. @newpagetable */
}
#include "spf_end_pack.h"
;
typedef struct module_cmn_md_buffer_start_t module_cmn_md_buffer_start_t;


/** @ingroup spf_mods_metadata
    Identifier for the metadata that specifies the end of the buffer.

    @msgpayload
    module_cmn_md_buffer_end_t @newpage
 */
#define MODULE_CMN_MD_ID_BUFFER_END            0x0A001054


/** @ingroup spf_mods_metadata
    Bitmask for the metadata error status/result flag. */
#define MD_END_PAYLOAD_FLAGS_BIT_MASK_ERROR_RESULT                AR_NON_GUID(0x00000001)

/** @ingroup spf_mods_metadata
    Shift value for the metadata error status/result flag. */
#define MD_END_PAYLOAD_FLAGS_SHIFT_ERROR_RESULT                   0

/** @ingroup spf_mods_metadata
    Bitmask for the metadata error recovery done flag. */
#define MD_END_PAYLOAD_FLAGS_BIT_MASK_ERROR_RECOVERY_DONE         AR_NON_GUID(0x00000002)

/** @ingroup spf_mods_metadata
    Shift value for the metadata error status/result flag. */
#define MD_END_PAYLOAD_FLAGS_SHIFT_ERROR_RECOVERY_DONE            1

/** @ingroup spf_mods_metadata
    Metadata error result value for a failure. */
#define MD_END_RESULT_FAILED                                      1

/** @ingroup spf_mods_metadata
    Metadata error result value for when an error recovery is done. @newpage */
#define MD_END_RESULT_ERROR_RECOVERY_DONE                         1


/** @ingroup spf_mods_metadata
    Payload for #MODULE_CMN_MD_ID_BUFFER_END.
    This structure contains metadata to specify the end of the buffer.
 */
#include "spf_begin_pack.h"
struct module_cmn_md_buffer_end_t
{
   uint32_t buffer_index_lsw;
   /**< Lower 32 bits of the 64-bit unique buffer index. */

   uint32_t buffer_index_msw;
   /**< Upper 32 bits of the 64-bit unique buffer index. */

   uint32_t flags;
   /**< Specifies related information to or from the client.

        Bit 0 specifies whether any frame in the buffer has an error:
        @vertspace{-4}
         - 0 -- No error
         - 1 -- At least one frame in the buffer has an error

        Bit 1 specifies whether an error recovery was done for any frame in
        the buffer: @vertspace{-4}
         - 0 -- No error recovery
         - 1 -- At least one frame in the buffer was recovered

        Other bitfield values are reserved for future use and must be set to 0. */
}
#include "spf_end_pack.h"
;
typedef struct module_cmn_md_buffer_end_t module_cmn_md_buffer_end_t;


#ifdef __cplusplus
}
#endif /*__cplusplus*/


#endif /* CODEC_METADATA_API_H */
