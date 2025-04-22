#ifndef WR_SH_MEM_EP_API_H_
#define WR_SH_MEM_EP_API_H_
/**
 * \file wr_sh_mem_ep_api.h
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
#include "ar_guids.h"

/*# @h2xml_title1          {Write Shared Memory End Point Module API}
    @h2xml_title_agile_rev {Write Shared Memory End Point Module API}
    @h2xml_title_date      {August 13, 2018} */

/*# @h2xmlx_xmlNumberFormat {int} */


/** @ingroup ar_spf_mod_ep_wrshmemep_macros
    Bitmask for the timestamp validity information. */
#define WR_SH_MEM_EP_BIT_MASK_TIMESTAMP_VALID_FLAG    AR_NON_GUID(0x80000000)

/** @ingroup ar_spf_mod_ep_wrshmemep_macros
    Shift value for the timestamp validity information. */
#define WR_SH_MEM_EP_SHIFT_TIMESTAMP_VALID_FLAG       31

/** @ingroup ar_spf_mod_ep_wrshmemep_macros
    Bitmask for the last buffer indicator. */
#define WR_SH_MEM_EP_BIT_MASK_LAST_BUFFER_FLAG        AR_NON_GUID(0x40000000)

/** @ingroup ar_spf_mod_ep_wrshmemep_macros
    Shift value for the last buffer indicator. */
#define WR_SH_MEM_EP_SHIFT_LAST_BUFFER_FLAG           30

/** @ingroup ar_spf_mod_ep_wrshmemep_macros
    Bitmask for the timestamp continues indicator. */
#define WR_SH_MEM_EP_BIT_MASK_TS_CONTINUE_FLAG        AR_NON_GUID(0x20000000)

/** @ingroup ar_spf_mod_ep_wrshmemep_macros
    Shift value for the timestamp continues indicator. */
#define WR_SH_MEM_EP_SHIFT_TS_CONTINUE_FLAG           29

/** @ingroup ar_spf_mod_ep_wrshmemep_macros
    Bitmask for the End of Frame (EOF) indicator. */
#define WR_SH_MEM_EP_BIT_MASK_EOF_FLAG                AR_NON_GUID(0x00000010)

/** @ingroup ar_spf_mod_ep_wrshmemep_macros
    Shift value for the EOF indicator. */
#define WR_SH_MEM_EP_SHIFT_EOF_FLAG                   4


/** @ingroup ar_spf_mod_ep_wrshmemep_mods
    Writes a buffer and the associated metadata to the Shared Memory Endpoint
    module.

    This command can be used for PCM Raw compressed bit streams and packetized
    streams. Before issuing this command, the graph must be opened and a media
    format issued (command or data). For PCM, the buffer must contain same
    number of samples on all channels.

    The client writes data and metadata to a buffer (defined by
    data_cmd_wr_sh_mem_ep_data_buffer_v2_t), and the
    #MODULE_ID_WR_SHARED_MEM_EP module reads that data from the buffer. The
    metadata must be filled in the specified format at the corresponding
    address.

    Once the buffer is consumed, the module raises a
    #DATA_CMD_RSP_WR_SH_MEM_EP_DATA_BUFFER_DONE_V2 event.

    @msgpayload
    data_cmd_wr_sh_mem_ep_data_buffer_v2_t
 */
#define DATA_CMD_WR_SH_MEM_EP_DATA_BUFFER_V2             0x0400100A

/** @ingroup ar_spf_mod_ep_wrshmemep_mods
    Payload of the #DATA_CMD_WR_SH_MEM_EP_DATA_BUFFER_V2 command.
*/
#include "spf_begin_pack.h"
struct data_cmd_wr_sh_mem_ep_data_buffer_v2_t
{
   uint32_t data_buf_addr_lsw;
   /**< Lower 32 bits of the address of the buffer containing the data. */

   uint32_t data_buf_addr_msw;
   /**< Upper 32 bits of the address of the buffer containing the data.

        The 64-bit number formed by data_buf_addr_lsw and data_buf_addr_msw
        must be aligned to the cache line of the processor where the SPF runs
        (64-byte alignment for Hexagon). This buffer address must be a valid
        address that was mapped via APM_CMD_SHARED_MEM_MAP_REGIONS (see the
        AudioReach SPF API Reference (80-VN500-5)).

        @values
        - For a 32-bit shared memory address, this data_buf_addr_msw field must
          be set to 0.
        - For a 36-bit address, bits 31 to 4 of this data_buf_addr_msw field
          must be set to 0. @tablebulletend */

   uint32_t data_mem_map_handle;
   /**< Unique identifier for the shared memory that corresponds to the data
        buffer address

        The SPF returns this handle through APM_CMD_RSP_SHARED_MEM_MAP_REGIONS
        (see the AudioReach SPF CAPI API Reference (80-VN500-6)). */

   uint32_t data_buf_size;
   /**< Number of valid bytes available in the buffer for processing. The
        first byte starts at the data buffer address. */

   uint32_t timestamp_lsw;
   /**< Lower 32 bits of the 64-bit timestamp in microseconds of the
        first data buffer sample. */

   uint32_t timestamp_msw;
   /**< Upper 32 bits of the 64-bit timestamp in microseconds of the
        first data buffer sample.

        The 64-bit number of interpreted as signed number. */

   uint32_t flags;
   /**< Bitfield of flags.

        @valuesbul{for bit 31}
        - 1 -- Valid timestamp
        - 0 -- Invalid timestamp
        - To set this bit, use #WR_SH_MEM_EP_BIT_MASK_TIMESTAMP_VALID_FLAG and
          #WR_SH_MEM_EP_SHIFT_TIMESTAMP_VALID_FLAG

        @contcell
        @valuesbul{for bit 30}
        - 1 -- Last data buffer
        - 0 -- Not the last data buffer
        - To set this bit, use #WR_SH_MEM_EP_BIT_MASK_LAST_BUFFER_FLAG and
          #WR_SH_MEM_EP_SHIFT_LAST_BUFFER_FLAG

        @valuesbul{for bit 29}
        - 1 -- Continue the timestamp from the previous data buffer
        - 0 -- Timestamp of the current buffer is not related to the timestamp
               of the previous data buffer
        - To set this bit, use #WR_SH_MEM_EP_BIT_MASK_TS_CONTINUE_FLAG and
          #WR_SH_MEM_EP_SHIFT_TS_CONTINUE_FLAG

        @valuesbul{for bit 4 (end-of-frame flag)}
        - 1 -- End of the frame
        - 0 -- Not the end of frame, or this information is not known
        - To set this bit, use #WR_SH_MEM_EP_BIT_MASK_EOF_FLAG as the bitmask
          and #WR_SH_MEM_EP_SHIFT_EOF_FLAG

        All other bits are reserved; clients must set them to 0.

        @tblsubhd{If bit 31=0 and bit 29=1}
        The timestamp of the first sample in this data buffer continues from
        the timestamp of the last sample in the previous data buffer.

        The samples in the current data buffer do not have a valid timestamp in
        the following situations: @lstsp2
        - If there is no previous data buffer (this buffer is the first
          data buffer sent after opening and starting the graph or after a
          flush operation)
        - If the previous data buffer does not have a valid timestamp

        In these cases, the samples in the current data buffer are played out
        as soon as possible.

        @tblsubhd{If bit 31=0 and bit 29=0}
        No timestamp is associated with the first sample in this data buffer.
        The samples are played out as soon as possible.

        @tblsubhd{If bit 31=1 and bit 29 is ignored}
        The timestamp specified in this payload is honored.

        @tblsubhd{If bit 30=0}
        This buffer is not the last data buffer. This case is useful
        in removing trailing samples in gapless use cases.

        @tblsubhd{For bit 4}
        The client can set this end-of-frame flag for every data buffer sent in
        which the last byte is the end of a frame. If this flag is set, the
        data buffer can contain data from multiple frames, but it must always
        end at a frame boundary. Restrictions allow the SPF to detect an end of
        frame without requiring additional processing. */

   uint32_t md_buf_addr_lsw;
   /**< Lower 32 bits of the address of the buffer containing the metadata. */

   uint32_t md_buf_addr_msw;
   /**< Upper 32 bits of the address of the buffer containing the metadata.

        The 64-bit number formed by md_buf_addr_lsw and md_buf_addr_msw must be
        aligned to the cache line of the processor where the SPF runs (64-byte
        alignment for Hexagon). This buffer address must be a valid address
        that was mapped via APM_CMD_SHARED_MEM_MAP_REGIONS (see the
        AudioReach SPF API Reference (80-VN500-5)).

        @values
        - For a 32-bit shared memory address, this md_buf_addr_msw field must
          be set to 0.
        - For a 36-bit address, bits 31 to 4 of this md_buf_addr_msw field must
          be set to 0. @tablebulletend */

   uint32_t md_mem_map_handle;
   /**< Unique identifier for the shared memory that corresponds to the
        metadata buffer address.

        The SPF returns this handle through APM_CMD_RSP_SHARED_MEM_MAP_REGIONS
        (see the AudioReach SPF CAPI API Reference (80-VN500-6)). */

   uint32_t md_buf_size;
   /**< Number of valid bytes available in the metadata buffer for processing.
        The first byte starts at the metadata buffer address. */
}
#include "spf_end_pack.h"
;
typedef struct data_cmd_wr_sh_mem_ep_data_buffer_v2_t data_cmd_wr_sh_mem_ep_data_buffer_v2_t;


/** @ingroup ar_spf_mod_ep_wrshmemep_mods
    Indicates that the referenced write buffer has been fully consumed and
    is available to the client.

    @msgpayload
    data_cmd_rsp_wr_sh_mem_ep_data_buffer_done_v2_t
 */
#define DATA_CMD_RSP_WR_SH_MEM_EP_DATA_BUFFER_DONE_V2    0x05001004

/** @ingroup ar_spf_mod_ep_wrshmemep_mods
    Payload of the #DATA_CMD_RSP_WR_SH_MEM_EP_DATA_BUFFER_DONE_V2 command
    response.
*/
#include "spf_begin_pack.h"
struct data_cmd_rsp_wr_sh_mem_ep_data_buffer_done_v2_t
{
   uint32_t data_buf_addr_lsw;
   /**< Lower 32 bits of the address of the data buffer being returned. */

   uint32_t data_buf_addr_msw;
   /**< Upper 32 bits of the address of the data buffer being returned.

        The valid, mapped, 64-bit data buffer address is the same address that
        the client provides in #DATA_CMD_WR_SH_MEM_EP_DATA_BUFFER_V2. */

   uint32_t data_mem_map_handle;
   /**< Unique identifier for the shared memory that corresponds to the data
        buffer address.

        The SPF returns this handle through APM_CMD_RSP_SHARED_MEM_MAP_REGIONS
        (see the AudioReach SPF CAPI API Reference (80-VN500-6)). */

   uint32_t data_status;
   /**< Status message (error code) that indicates whether the referenced
        data buffer has been successfully consumed.

        @values See the AudioReach SPF Generic Packet Router API
                Reference (80-VN500-10) */

   uint32_t md_buf_addr_lsw;
   /**< Lower 32 bits of the address of the metadata buffer being returned. */

   uint32_t md_buf_addr_msw;
   /**< Upper 32 bits of the address of the metadata buffer being returned.

        The valid, mapped, 64-bit metadata buffer address is the same address
        that the client provides in #DATA_CMD_WR_SH_MEM_EP_DATA_BUFFER_V2. */

   uint32_t md_mem_map_handle;
   /**< Unique identifier for the shared memory that corresponds to the
        metadata buffer address

        The SPF returns this handle through APM_CMD_RSP_SHARED_MEM_MAP_REGIONS
        (see the AudioReach SPF CAPI API Reference (80-VN500-6)). */

   uint32_t md_status;
   /**< Status message (error code) that indicates whether the referenced
        metadata buffer has been successfully consumed.

        @values See the AudioReach SPF Generic Packet Router API
                Reference (80-VN500-10) */
}
#include "spf_end_pack.h"
;
typedef struct data_cmd_rsp_wr_sh_mem_ep_data_buffer_done_v2_t data_cmd_rsp_wr_sh_mem_ep_data_buffer_done_v2_t;


/** @ingroup ar_spf_mod_ep_wrshmemep_mods
    Indicates an update to the media format, which applies to future buffers
    written to this stream.

    This command is accepted only when subgraph is in START state.

    Some decoders might not work without receiving either
    #PARAM_ID_MEDIA_FORMAT or #DATA_CMD_WR_SH_MEM_EP_MEDIA_FORMAT.

    @msgpayload
    media_format_t @newpage
 */
#define DATA_CMD_WR_SH_MEM_EP_MEDIA_FORMAT         0x04001001

/** @ingroup ar_spf_mod_ep_wrshmemep_mods
    Communicates an EoS marker, which indicates that the last buffer in a
    stream was delivered.

    Once the stream is received by another endpoint (such as a hardware
    endpoint), the SPF raises a #DATA_CMD_RSP_WR_SH_MEM_EP_EOS_RENDERED event.

    For correlation, the token in the GPR payload is used as a token in
    DATA_CMD_RSP_WR_SH_MEM_EP_EOS_RENDERED.

    @msgpayload
    data_cmd_wr_sh_mem_ep_eos_t
 */
#define DATA_CMD_WR_SH_MEM_EP_EOS                  0x04001002

/** @ingroup ar_spf_mod_ep_wrshmemep_macros
    EoS policy whereby the last final endpoint raises
    #DATA_CMD_RSP_WR_SH_MEM_EP_EOS_RENDERED, or the last container that drops
    or renders the EoS raises DATA_CMD_RSP_WR_SH_MEM_EP_EOS_RENDERED.
 */
#define WR_SH_MEM_EP_EOS_POLICY_LAST               1

/** @ingroup ar_spf_mod_ep_wrshmemep_macros
    EoS policy whereby each final endpoint raises
    #DATA_CMD_RSP_WR_SH_MEM_EP_EOS_RENDERED, or each container that drops the
    EoS raises DATA_CMD_RSP_WR_SH_MEM_EP_EOS_RENDERED.
 */
#define WR_SH_MEM_EP_EOS_POLICY_EACH                2

/** @ingroup ar_spf_mod_ep_wrshmemep_mods
    Payload of the #DATA_CMD_WR_SH_MEM_EP_EOS command.
*/
#include "spf_begin_pack.h"
struct data_cmd_wr_sh_mem_ep_eos_t
{
   uint32_t          policy;
   /**< Policy used to raise #DATA_CMD_RSP_WR_SH_MEM_EP_EOS_RENDERED.

        @valuesbul
        - #WR_SH_MEM_EP_EOS_POLICY_LAST
        - #WR_SH_MEM_EP_EOS_POLICY_EACH @tablebulletend */
}
#include "spf_end_pack.h"
;
typedef struct data_cmd_wr_sh_mem_ep_eos_t data_cmd_wr_sh_mem_ep_eos_t;

/** @ingroup ar_spf_mod_ep_wrshmemep_mods
    Indicates that #DATA_CMD_WR_SH_MEM_EP_EOS has been received by the final
    endpoint. No more data is left to be rendered.

    This command response uses the same token that was sent in
    DATA_CMD_WR_SH_MEM_EP_EOS. The source port in the GPR packet contains the
    shared memory endpoint that received DATA_CMD_WR_SH_MEM_EP_EOS.

    @msgpayload
    data_cmd_rsp_wr_sh_mem_ep_eos_rendered_t
 */
#define DATA_CMD_RSP_WR_SH_MEM_EP_EOS_RENDERED          0x05001001

/** @ingroup ar_spf_mod_ep_wrshmemep_macros
    Render status of the EoS, where the EoS message is rendered on the final
    end device. */
#define WR_SH_MEM_EP_EOS_RENDER_STATUS_RENDERED       1

/** @ingroup ar_spf_mod_ep_wrshmemep_macros
    Render status of the EoS, where the EoS message is dropped before it
    reaches the final end device. */
#define WR_SH_MEM_EP_EOS_RENDER_STATUS_DROPPED        2

/** @ingroup ar_spf_mod_ep_wrshmemep_mods
    Payload of the #DATA_CMD_RSP_WR_SH_MEM_EP_EOS_RENDERED command response.
*/
#include "spf_begin_pack.h"
struct data_cmd_rsp_wr_sh_mem_ep_eos_rendered_t
{
   uint32_t module_instance_id;
   /**< Identifier for the module instance from which
        #DATA_CMD_RSP_WR_SH_MEM_EP_EOS_RENDERED is being raised.

        This value is invalid (0) when the EoS metadata is dropped. */

   uint32_t render_status;
   /**< Render status of the EoS.

        @valuesbul
        - #WR_SH_MEM_EP_EOS_RENDER_STATUS_RENDERED
        - #WR_SH_MEM_EP_EOS_RENDER_STATUS_DROPPED @tablebulletend */
}
#include "spf_end_pack.h"
;
typedef struct data_cmd_rsp_wr_sh_mem_ep_eos_rendered_t  data_cmd_rsp_wr_sh_mem_ep_eos_rendered_t;

/** @ingroup ar_spf_mod_ep_wrshmemep_macros
    Indicates the output port ID of the #MODULE_ID_WR_SHARED_MEM_EP module. */
#define PORT_ID_WR_SHARED_MEM_EP_OUTPUT               0x1

/** @ingroup ar_spf_mod_ep_wrshmemep_macros
    Indicates the input port ID of the #MODULE_ID_WR_SHARED_MEM_EP module.
    This port is connected only in Offload (MDF) use cases.
 */
#define PORT_ID_WR_SHARED_MEM_EP_INPUT               0x2

/** @ingroup ar_spf_mod_ep_wrshmemep_mods
    Identifier for the Write Shared Memory Endpoint module, which writes data
    to the SPF from the host through a packet exchange mechanism.

    For a regular use case, this module supports only one static output port
    with ID 1.

    For MDF use cases where the module is automatically inserted by the
    QACT@tm Platform, this module supports one static output port with ID 1 and
    one static input port with ID 2.

    This module supports any input media format. @newpage
 */
#define MODULE_ID_WR_SHARED_MEM_EP                    0x07001000

/*# @h2xmlm_module             {"MODULE_ID_WR_SHARED_MEM_EP",
                                 MODULE_ID_WR_SHARED_MEM_EP}
    @h2xmlm_displayName        {"Write Shared Memory Endpoint"}
    @h2xmlm_modSearchKeys      {software}
    @h2xmlm_description        {ID for the module that writes data to the SPF
                                from the host through a packet exchange
                                mechanism. For a regular use case, this module
                                supports only one static output port with
                                ID 1. For MDF use cases where the module is
                                automatically inserted by the QACT\tm
                                Platform, this module supports one static
                                output port with ID 1 and one static input
                                port with ID 2.}
    @h2xmlm_offloadInsert      {WR_EP}
    @h2xmlm_dataInputPorts     {IN=PORT_ID_WR_SHARED_MEM_EP_INPUT}
    @h2xmlm_dataOutputPorts    {OUT=PORT_ID_WR_SHARED_MEM_EP_OUTPUT}
    @h2xmlm_dataMaxInputPorts  {1}
    @h2xmlm_dataMaxOutputPorts {1}
    @h2xmlm_supportedContTypes {APM_CONTAINER_TYPE_GC}
    @h2xmlm_isOffloadable      {false}
    @h2xmlm_stackSize          {1024}

    @{                      <-- Start of the Module -->
    @h2xml_Select           {"media_format_t"}
    @h2xmlm_InsertParameter
    @}                      <-- End of the Module   --> */


#endif // WR_SH_MEM_EP_API_H_
