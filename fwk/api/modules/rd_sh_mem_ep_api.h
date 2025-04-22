#ifndef RD_SH_MEM_EP_API_H_
#define RD_SH_MEM_EP_API_H_
/**
 * \file rd_sh_mem_ep_api.h
 * \brief
 *    This file contains Shared mem module APIs
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*------------------------------------------------------------------------
  Include files
  -----------------------------------------------------------------------*/
#include "ar_defs.h"
#include "common_enc_dec_api.h"
#include "module_cmn_shmem_api.h"

/*# @h2xml_title1          {APIs of Read Shared Memory Endpoint Module}
    @h2xml_title_agile_rev {APIs of Read Shared Memory Endpoint Module}
    @h2xml_title_date      {August 13, 2018} */

/*# @h2xmlx_xmlNumberFormat {int} */


/** @ingroup ar_spf_mod_ep_rdshmemep_macros
    Bitmask for the timestamp validity information. */
#define RD_SH_MEM_EP_BIT_MASK_TIMESTAMP_VALID_FLAG                AR_NON_GUID(0x80000000)

/** @ingroup ar_spf_mod_ep_rdshmemep_macros
    Shift value for the timestamp validity information. */
#define RD_SH_MEM_EP_SHIFT_TIMESTAMP_VALID_FLAG                   31

/** @ingroup ar_spf_mod_ep_rdshmemep_macros
    Bitmask for the frame metadata. */
#define RD_SH_MEM_EP_BIT_MASK_FRAME_METADATA_FLAG                 AR_NON_GUID(0x60000000)

/** @ingroup ar_spf_mod_ep_rdshmemep_macros
    Shift value for the  frame metadata. */
#define RD_SH_MEM_EP_SHIFT_FRAME_METADATA_FLAG                    29

/** @ingroup ar_spf_mod_ep_rdshmemep_mods
    Payload of the metadata that can be put in the data read buffer.
*/
#include "spf_begin_pack.h"
struct data_event_rd_sh_mem_ep_metadata_t
{
   uint32_t          offset;
   /**< Offset from the buffer address in
        %data_cmd_rsp_rd_sh_mem_ep_data_buffer_done_v2_t to the frame
        associated with the metadata. */

   uint32_t          frame_size;
   /**< Size of each frame in bytes (for example, the encoded frame size). */

   uint32_t          pcm_length;
   /**< Number of PCM samples per channel that corresponding to each frame_size
        (for example, the number of PCM samples per channel used for encoding
        a frame).

       This field is always set to 0 because no use case requires it. */

   uint32_t          timestamp_lsw;
   /**< Lower 32 bits of the 64-bit session time of the first sample in the
        frame (in microseconds). */

   uint32_t          timestamp_msw;
   /**< Upper 32 bits of the 64-bit session time of the first sample in the
        frame (in microseconds). */

   uint32_t          flags;
   /**< Frame flags.

        @valuesbul{for bit 31}
        - 1 -- Timestamp is valid
        - 0 -- Timestamp is not valid

        To set this bit, use #RD_SH_MEM_EP_BIT_MASK_TIMESTAMP_VALID_FLAG and
        #RD_SH_MEM_EP_SHIFT_TIMESTAMP_VALID_FLAG

        All other bits are reserved; the SPF sets them to 0. */
}
#include "spf_end_pack.h"
;
typedef struct data_event_rd_sh_mem_ep_metadata_t data_event_rd_sh_mem_ep_metadata_t;


/** @ingroup ar_spf_mod_ep_rdshmemep_mods
    Sends an empty buffer to the #MODULE_ID_RD_SHARED_MEM_EP module. The module
    writes data and metadata to this buffer, and the client reads it.

    This command can be used to read PCM data, Raw compressed data, and
    packetized streams.

    Once the buffer is filled, the SPF returns
    #DATA_CMD_RSP_RD_SH_MEM_EP_DATA_BUFFER_DONE_V2.

    @msgpayload
    data_cmd_rd_sh_mem_ep_data_buffer_v2_t
 */
#define DATA_CMD_RD_SH_MEM_EP_DATA_BUFFER_V2                         0x0400100B

/** @ingroup ar_spf_mod_ep_rdshmemep_mods
    Payload for #DATA_CMD_RD_SH_MEM_EP_DATA_BUFFER_V2.
*/
#include "spf_begin_pack.h"
struct data_cmd_rd_sh_mem_ep_data_buffer_v2_t
{
   uint32_t data_buf_addr_lsw;
   /**< Lower 32 bits of the address of the data buffer. */

   uint32_t data_buf_addr_msw;
   /**< Upper 32 bits of the address of the data buffer.

        The data buffer address for each frame must be a valid address that was
        mapped via APM_CMD_SHARED_MEM_MAP_REGIONS. For more information about
        APM_CMD_SHARED_MEM_MAP_REGIONS, see the AudioReach SPF API
        Reference (80-VN500-5).

        The 64-bit number formed by data_buf_addr_lsw and data_buf_addr_msw
        must be aligned to the cache-line of the processor where the SPF runs
        (64-byte alignment for Hexagon).

        @valuesbul
        - For a 32-bit shared memory address, this field must be set to 0.
        - For a 36-bit address, bits 31 to 4 of this field must be set to 0.
        @tablebulletend */

   uint32_t data_mem_map_handle;
   /**< Unique identifier for the shared memory address that corresponds to the
        data buffer.

        The SPF returns this handle through APM_CMD_RSP_SHARED_MEM_MAP_REGIONS
        (see the AudioReach SPF CAPI API Reference (80-VN500-6)). */

   uint32_t data_buf_size;
   /**< Number of bytes available for the SPF to write data. The first byte
        starts at the specified data buffer address.

        @values @ge 0 */

   uint32_t md_buf_addr_lsw;
   /**< Lower 32 bits of the address of the metadata buffer. */

   uint32_t md_buf_addr_msw;
   /**< Upper 32 bits of the address of the metadata buffer.

        The metadata buffer address for each frame must be a valid address
        that was mapped via APM_CMD_SHARED_MEM_MAP_REGIONS. For more
        information about APM_CMD_SHARED_MEM_MAP_REGIONS, see the AudioReach
        SPF API Reference (80-VN500-5).

        The 64-bit number formed by md_buf_addr_lsw and md_buf_addr_msw must be
        aligned to the cache line of the processor where the SPF runs (64-byte
        alignment for Hexagon).

        @valuesbul
        - For a 32-bit shared memory address, this field must be set to 0.
        - For a 36-bit address, bits 31 to 4 of this field must be set to 0.
        @tablebulletend */

   uint32_t md_mem_map_handle;
   /**< Unique identifier for the shared memory address that corresponds to
        the metadata buffer.

        The SPF returns this handle through APM_CMD_RSP_SHARED_MEM_MAP_REGIONS
        (see the AudioReach SPF CAPI API Reference (80-VN500-6)). */

   uint32_t md_buf_size;
   /**< Number of bytes available for the SPF to write metadata.

        The first byte starts at the specified metadata buffer address.

        @values @ge 0 */
}
#include "spf_end_pack.h"
;
typedef struct data_cmd_rd_sh_mem_ep_data_buffer_v2_t data_cmd_rd_sh_mem_ep_data_buffer_v2_t;


/** @ingroup ar_spf_mod_ep_rdshmemep_mods
    Sent by the SPF in response to #DATA_CMD_RD_SH_MEM_EP_DATA_BUFFER_V2 when
    the referenced buffer is filled and is available to the client for
    reading.

    @msgpayload
    data_cmd_rsp_rd_sh_mem_ep_data_buffer_done_v2_t
 */
#define DATA_CMD_RSP_RD_SH_MEM_EP_DATA_BUFFER_DONE_V2                0x05001005


/** @ingroup ar_spf_mod_ep_rdshmemep_mods
    Payload for #DATA_CMD_RSP_RD_SH_MEM_EP_DATA_BUFFER_DONE_V2.
*/
#include "spf_begin_pack.h"
struct data_cmd_rsp_rd_sh_mem_ep_data_buffer_done_v2_t
{
   uint32_t                data_status;
   /**< Status message for data buffer (error code).

        @values See the AudioReach SPF Generic Packet Router API
                Reference(80-VN500-10) */

   uint32_t                data_buf_addr_lsw;
   /**< Lower 32 bits of the address of the data buffer being returned. */

   uint32_t                data_buf_addr_msw;
   /**< Upper 32 bits of the data address of the buffer being returned.

        The valid, mapped, 64-bit address is the same address that
        the client provides in #DATA_CMD_RD_SH_MEM_EP_DATA_BUFFER_V2. */

   uint32_t                 data_mem_map_handle;
   /**< Unique identifier for the shared memory address that corresponds to
        the data buffer.

        The SPF returns this handle through APM_CMD_RSP_SHARED_MEM_MAP_REGIONS
        (see the >AudioReach SPF CAPI API Reference (80-VN500-6)). */

   uint32_t                 data_size;
   /**< Total size of the data frames in bytes. */

   uint32_t                 timestamp_lsw;
   /**< Lower 32 bits of the 64-bit session time in microseconds of the
        first sample in the data buffer. */

   uint32_t                 timestamp_msw;
   /**< Upper 32 bits of the 64-bit session time in microseconds of the
        first sample in the data buffer.

        The 64-bit timestamp must be interpreted as a signed number. The
        source of the timestamp depends on the source that is feeding data to
        this module. */

   uint32_t                 flags;
   /**< Bit field of flags.

        @valuesbul{for bit 31}
        - 1 -- Timestamp is valid
        - 0 -- Timestamp is invalid

        To set this bit, use #RD_SH_MEM_EP_BIT_MASK_TIMESTAMP_VALID_FLAG and
        #RD_SH_MEM_EP_SHIFT_TIMESTAMP_VALID_FLAG.

        All other bits are reserved; the SPF sets them to 0. */

   uint32_t                num_frames;
   /**< Number of data frames in the data buffer. */

   uint32_t                md_status;
   /**< Status message for metadata buffer(error code).

        @values See the AudioReach SPF Generic Packet Router API
                Reference (80-VN500-10) */

   uint32_t                md_buf_addr_lsw;
   /**< Lower 32 bits of the address of the metadata buffer being returned. */

   uint32_t                md_buf_addr_msw;
   /**< Upper 32 bits of the data address of the metadata buffer being
        returned.

        The valid, mapped, 64-bit address is the same address that
        the client provides in #DATA_CMD_RD_SH_MEM_EP_DATA_BUFFER_V2. */

   uint32_t                md_mem_map_handle;
   /**< Unique identifier for the shared memory address that corresponds to
        the metadata buffer.

        The SPF returns this handle through APM_CMD_RSP_SHARED_MEM_MAP_REGIONS
        (see the AudioReach SPF CAPI API Reference (80-VN500-6)). */

   uint32_t                md_size;
   /**< Total size of the metadata frames in bytes. */
}
#include "spf_end_pack.h"
;
typedef struct data_cmd_rsp_rd_sh_mem_ep_data_buffer_done_v2_t data_cmd_rsp_rd_sh_mem_ep_data_buffer_done_v2_t;


/** @ingroup ar_spf_mod_ep_rdshmemep_mods
    Media format sent from #MODULE_ID_RD_SHARED_MEM_EP to the client.

    This event must be registered with APM_CMD_REGISTER_MODULE_EVENTS (see the
    AudioReach SPF API Reference (80-VN500-5)).

    @msgpayload
    media_format_t
 */
#define DATA_EVENT_ID_RD_SH_MEM_EP_MEDIA_FORMAT                        0x06001000

/** @ingroup ar_spf_mod_ep_rdshmemep_mods
    EOS event sent from #MODULE_ID_RD_SHARED_MEM_EP to the client.

    This event serves as a discontinuity marker and must be registered with
    APM_CMD_REGISTER_MODULE_EVENTS (see the AudioReach SPF API Reference
    (80-VN500-5)).

    @msgpayload
    data_event_rd_sh_mem_ep_eos_t
*/
#define DATA_EVENT_ID_RD_SH_MEM_EP_EOS                                 0x06001001

/** @ingroup ar_spf_mod_ep_rdshmemep_mods
    Payload for #DATA_EVENT_ID_RD_SH_MEM_EP_EOS.
 */
#include "spf_begin_pack.h"
struct data_event_rd_sh_mem_ep_eos_t
{
   uint32_t    eos_reason;
   /**< EoS is raised due to a pause or another discontinuity. */
}
#include "spf_end_pack.h"
;
typedef struct data_event_rd_sh_mem_ep_eos_t data_event_rd_sh_mem_ep_eos_t;

/** @ingroup ar_spf_mod_ep_rdshmemep_mods
    Timestamp Discontinuity event sent from #MODULE_ID_RD_SHARED_MEM_EP to the client.

    This event serves as a timestamp discontinuity notification and must be registered with
    APM_CMD_REGISTER_MODULE_EVENTS (see the AudioReach SPF API Reference
    (80-VN500-5)).

    This will be received as payload of the event APM_EVENT_MODULE_TO_CLIENT.

    @msgpayload
    event_id_rd_sh_mem_ep_timestamp_disc_detection_t
*/
#define EVENT_ID_RD_SH_MEM_EP_TIMESTAMP_DISC_DETECTION 0x08001A98

/** @ingroup ar_spf_mod_ep_rdshmemep_macros
    Bitmask for the timestamp discontinuity duration validity information. */
#define RD_SH_MEM_EP_BIT_MASK_TIMESTAMP_DISC_DURATION_VALID_FLAG AR_NON_GUID(0x80000000)

/** @ingroup ar_spf_mod_ep_rdshmemep_macros
    Shift Value for the timestamp discontinuity duration validity information. */
#define RD_SH_MEM_EP_SHIFT_TIMESTAMP_DISC_DURATION_VALID_FLAG 31

#include "spf_begin_pack.h"
struct event_id_rd_sh_mem_ep_timestamp_disc_detection_t
{

   uint32_t flags;
   /**< Frame flags.

        @valuesbul{for bit 31}
        - 1 -- Timestamp is valid
        - 0 -- Timestamp is not valid

        To set this bit, use #RD_SH_MEM_EP_BIT_MASK_TIMESTAMP_DISC_DURATION_VALID_FLAG and
        #RD_SH_MEM_EP_SHIFT_TIMESTAMP_DISC_DURATION_VALID_FLAG

        All other bits are reserved; the SPF sets them to 0. */

   uint32_t timestamp_disc_duration_us_msw;
   /**< MSW of difference in timestamp detected that led to the discontinuity
    * in microseconds.  */

   uint32_t timestamp_disc_duration_us_lsw;
   /**< LSW of difference in timestamp detected that led to the discontinuity
    * in microseconds.  */
}
#include "spf_end_pack.h"
;
typedef struct event_id_rd_sh_mem_ep_timestamp_disc_detection_t event_id_rd_sh_mem_ep_timestamp_disc_detection_t;

/** @ingroup ar_spf_mod_ep_rdshmemep_macros
    Identifier for the input port of the #MODULE_ID_RD_SHARED_MEM_EP module. */
#define PORT_ID_RD_SHARED_MEM_EP_INPUT                            0x2

/** @ingroup ar_spf_mod_ep_rdshmemep_macros
    Identifier for the output port of the #MODULE_ID_RD_SHARED_MEM_EP module.
    This port is connected only in offload (MDF) use cases.
 */
#define PORT_ID_RD_SHARED_MEM_EP_OUTPUT                            0x1

/** @ingroup ar_spf_mod_ep_rdshmemep_mods
    Identifier for the Read Shared Memory Endpoint module, which reads data
    from the SPF into the host through a packet exchange mechanism.

    For a regular use case, this module has only one static input port with
    ID 2.

    For MDF use cases where the module is automatically inserted by the QACT\tm
    Platform, this module supports one static output port with ID 1 and one
    static input port with ID 2.

    This module supports any input media format.
 */
#define MODULE_ID_RD_SHARED_MEM_EP                                0x07001001

/*# @h2xmlm_module             {"MODULE_ID_RD_SHARED_MEM_EP",
                                 MODULE_ID_RD_SHARED_MEM_EP}
    @h2xmlm_displayName        {"Read Shared Memory Endpoint"}
    @h2xmlm_modSearchKeys      {software}
    @h2xmlm_description        {ID for the module that reads data from the SPF
                                into the host through a packet exchange
                                mechanism. \n
                                For a regular use case, this module has only
                                one static input port with ID 2. \n
                                For MDF use cases where the module is
                                automatically inserted by the QACT\tm
                                Platform, this module supports one static
                                output port with ID 1 and one static input
                                port with ID 2.}
    @h2xmlm_offloadInsert      {RD_EP}
    @h2xmlm_dataInputPorts     {IN=PORT_ID_RD_SHARED_MEM_EP_INPUT}
    @h2xmlm_dataOutputPorts    {OUT=PORT_ID_RD_SHARED_MEM_EP_OUTPUT}
    @h2xmlm_dataMaxInputPorts  {1}
    @h2xmlm_dataMaxOutputPorts {1}
    @h2xmlm_supportedContTypes {APM_CONTAINER_TYPE_GC,APM_CONTAINER_TYPE_WC}
    @h2xmlm_isOffloadable      {false}
    @h2xmlm_stackSize          {1024}

    @{                      <-- Start of the Module --> */

/** @ingroup ar_spf_mod_ep_rdshmemep_macros
    Special value of #param_id_rd_sh_mem_cfg_t::num_frames_per_buffer to
    indicate that as many buffers as possible must be filled.
 */
#define RD_SH_MEM_NUM_FRAMES_IN_BUF_AS_MUCH_AS_POSSIBLE               0

/*==============================================================================
   Param ID
==============================================================================*/

/** @ingroup ar_spf_mod_ep_rdshmemep_mods
    Identifier of the parameter that configures the #MODULE_ID_RD_SHARED_MEM_EP
    module.

    @msgpayload
    param_id_rd_sh_mem_cfg_t
*/
#define PARAM_ID_RD_SH_MEM_CFG                                       0x08001007

/** @ingroup ar_spf_mod_ep_rdshmemep_macros
    Bitmask that indicates whether encoder frame metadata is enabled. */
#define RD_EP_CFG_MD_CNTRL_FLAGS_BIT_MASK_ENABLE_ENCODER_FRAME_MD AR_NON_GUID(0x00000001)

/** @ingroup ar_spf_mod_ep_rdshmemep_macros
    Shift value for encoder frame metadata. */
#define RD_EP_CFG_MD_CNTRL_FLAGS_SHIFT_ENABLE_ENCODER_FRAME_MD 0

/** @ingroup ar_spf_mod_ep_rdshmemep_macros
    Bitmask that indicates whether media format metadata is enabled. */
#define RD_EP_CFG_MD_CNTRL_FLAGS_BIT_MASK_ENABLE_MEDIA_FORMAT_MD AR_NON_GUID(0x00000002)

/** @ingroup ar_spf_mod_ep_rdshmemep_macros
    Shift value for media format metadata. */
#define RD_EP_CFG_MD_CNTRL_FLAGS_SHIFT_ENABLE_MEDIA_FORMAT_MD 1

/*==============================================================================
   Param structure definitions
==============================================================================*/

/*# @h2xmlp_parameter   {"PARAM_ID_RD_SH_MEM_CFG", PARAM_ID_RD_SH_MEM_CFG}
    @h2xmlp_description {ID for the parameter that configures the
                         MODULE_ID_RD_SHARED_MEM_EP module.}
    @h2xmlp_toolPolicy  {Calibration} */

/** @ingroup ar_spf_mod_ep_rdshmemep_mods
    Payload for #PARAM_ID_RD_SH_MEM_CFG.
*/
#include "spf_begin_pack.h"
struct param_id_rd_sh_mem_cfg_t
{
   uint32_t       num_frames_per_buffer;
   /**<Number of frames per buffer that must be populated in the read buffers.

       It is considered an error if even one frame cannot be filled. If this value is
       0, as many frames as possible are to be filled (see
       #RD_SH_MEM_NUM_FRAMES_IN_BUF_AS_MUCH_AS_POSSIBLE).

       Only an integral number of frames can be filled in every buffer. */

   /*#< @h2xmle_description {Number of frames per buffer that must be populated
                             in the read buffers. It is considered an error if
							 even one frame cannot be filled. \n
                             If this value is 0, as many frames as possible are
                             to be filled. Only an integral number of frames
                             can be filled in every buffer.}
        @h2xmle_default     {0}
        @h2xmle_range       {0..0xFFFFFFFF}
        @h2xmle_policy      {Basic} */

   uint32_t metadata_control_flags;
   /**< Specifies the control flags for the metadata for the module. */

   /*#< @h2xmle_description {Specifies the control flags for the metadata for
                             the module.}
        @h2xmle_default     {0}
        @h2xmle_range       {0..3}
        @h2xmle_policy      {Basic}

        @h2xmle_bitfield    {0x00000001}
        @h2xmle_default     {0}
        @h2xmle_bitName     {Bit_0_ENABLE_ENCODER_FRAME_MD}
        @h2xmle_rangeList   {"Disable"=0; "Enable"=1}
        @h2xmle_description {Specifies the control to enable or disable the
                             per frame encoder information that is communicated
                             using MODULE_CMN_MD_ID_ENCODER_FRAME_INFO.}
        @h2xmle_bitfieldEnd

        @h2xmle_bitfield    {0x00000002}
        @h2xmle_default     {1}
        @h2xmle_bitName     {Bit_1_ENABLE_MEDIA_FORMAT_MD}
        @h2xmle_description {Specifies the control to enable or disable the
                             media format communication from the SPF to the
                             client as metatdata, which is communicated using
                             MODULE_CMN_MD_ID_MEDIA_FORMAT. \n
                             If enabled, the media format is communicated as
                             metadata in the read buffer.}
        @h2xmle_bitfieldEnd

        @h2xmle_bitfield    {0xfffffffC}
        @h2xmle_bitName     {Bit_31_2_Reserved}
        @h2xmle_description {Reserved bit [31:2].}
        @h2xmle_visibility  {hide}
        @h2xmle_bitfieldEnd */
}
#include "spf_end_pack.h"
;
typedef struct param_id_rd_sh_mem_cfg_t param_id_rd_sh_mem_cfg_t;

/*# @}                      <-- End of the Module --> */


#endif // RD_SH_MEM_EP_API_H_
