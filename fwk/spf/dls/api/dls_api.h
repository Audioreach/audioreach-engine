#ifndef _DLS_API_H_
#define _DLS_API_H_
/**
 * \file dls_api.h
 * \brief
 *   This file contains Data Logging Service command/param/event ID's and public
 *   data structures.
 *   This file should be used only when the DIAG service is not supported.
 *
 * \copyright
 *    Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *    SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*------------------------------------------------------------------------------
 *  Include Files
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

/*------------------------------------------------------------------------------
 *  Global Definitions/Declarations
 *----------------------------------------------------------------------------*/
/** @ingroup spf_mods_dls
    Identifier for an instance of the Audio Logging Server module. */

#define DLS_MODULE_INSTANCE_ID         0x0000000A

/*# h2xmlm_module             {"DLS_MODULE_INSTANCE_ID",
                                 DLS_MODULE_INSTANCE_ID}
    h2xmlm_displayName        {"Audio Logging Server"}
    h2xmlm_description        {Identifier for data logging service instance}
    h2xmlm_dataMaxInputPorts  {0}
    h2xmlm_dataMaxOutputPorts {0}
    h2xmlm_stackSize          {0}
    h2xmlm_ToolPolicy         {Calibration}

    {                   <-- Start of the Module -->
    }                   <-- End of the Module   --> */

/** @ingroup spf_dls_commands
    Used to configure the data logging service buffer information.
    DLS client will allocate the memory and sends this command to the DLS to
    configure the buffer information, like, buffer address, size, etc.
    This will be the first command from DLS client to the server. is_cfg field
    shall be set to 1, to configure the buffer information in the beginning of
    the data logging process. At the end of the process this flag shall be set
    to 0, to deconfigure [zeroing out all the fields] the buffer information.

    @msgpayload
    dls_param_id_config_buffer_t @newpage
*/
#define DLS_PARAM_ID_CONFIG_BUFFER 		   0x08001A83

/** @ingroup spf_dls_commands
    Payload for #DLS_CMD_CONFIG_BUFFER.

    Consists the 64-bit buffer starting address in buf_start_addr_lsw and
    buf_start_addr_msw. Followed by a single buffer size and the total
    allocated buffer size for DLS. Finally a configuration flag to config
    or deconfig the buffer information in the DLS
*/
#include "spf_begin_pack.h"
struct dls_param_id_config_buffer_t
{
	uint32_t buf_start_addr_lsw;
   /**< Lower 32-bits of the 64-bit buffer starting address.
        This field is an input configuration parameter. */

	uint32_t buf_start_addr_msw;
   /**< Upper 32-bits of the 64-bit buffer starting address.
        This field is an input configuration parameter. */

   uint32_t mem_map_handle;
   /**< Unique identifier for a shared memory address.

        @values
        - NULL -- buffer address is the physical memory address and it doesn't
        need mapping.
        - Non-NULL -- when buffers starting address needs to be mapped to fetch
        the physical address. */

	uint32_t total_buf_size;
   /**< Total allocated buffer size for the data logging service.
        This field is an input configuration parameter. */

   uint32_t max_log_pkt_size;
   /**< Maximum one single log packet size.
        This field is an input configuration parameter.
        -1 - default: maximum single buffer size will be used the internal
        from the DLS internal define */

	uint32_t	 is_cfg;
   /**< This parameter is used to differentiate between the buffer configuration
        before starting the data logging and deconfiguration when the data
        logging is complete
        1 - Configuration at the beginning to use the given memory space for the
            data logging.
        0 - Deconfigiuration, to not use the given memory space for the data
        logging */
}
#include "spf_end_pack.h"
;
typedef struct dls_param_id_config_buffer_t dls_param_id_config_buffer_t;

/** @ingroup spf_dls_commands
    Used to configure the log code to start/stop the data logging.
    This command will be issued by the DLS client to enable/disable logging the
    data on a specific log code. This log code will be used to prepare the
    buffer header when the buffer is committed.

    @msgpayload
    dls_param_id_enable_disable_log_code_t @newpage
*/
#define DLS_PARAM_ID_ENABLE_DISABLE_LOG_CODE   0x08001A84

/** @ingroup spf_dls_commands
    Payload for #DLS_PARAM_ID_ENABLE_DISABLE_LOG_CODE.

    Consists the 32-bit log_code to start/stop the data logging and a flag to
    to enable/disable the log code.
*/
#include "spf_begin_pack.h"
struct dls_param_id_enable_disable_log_code_t
{
	uint32_t is_enabled;
   /**< Flag to indicate the log code is requested to be enabled or disabled.
        1 - Enable log code
        0 - Disable log code
        This field is an input configuration parameter. */

   uint32_t num_log_codes;
   /**< Number of log codes to enable/disable.
        This field is an input configuration parameter. */

	uint32_t log_codes[0];
   /**< This is a Flexible Array Member to list the log codes to be
        enabled/disabled.
        This field is an input configuration parameter. */
}
#include "spf_end_pack.h"
;
typedef struct dls_param_id_enable_disable_log_code_t dls_param_id_enable_disable_log_code_t;

/** @ingroup spf_dls_commands
    Used to let the client know that a buffer of data is logged and is ready to
    be read at client side.
    This data event will be raised by the DLS to notify the client.

    @msgpayload
    dls_data_event_id_commit_log_buffer_t @newpage
*/
#define DLS_DATA_EVENT_ID_COMMIT_LOG_BUFFER 	   0x06001006

#include "spf_begin_pack.h"
struct dls_buf_start_addr_t
{
	uint32_t buf_addr_lsw;
   /**< This field contains the lower 32-bits of the 64-bit buffer starting
        address. */

	uint32_t buf_addr_msw;
   /**< This field contains the upper 32-bits of the 64-bit buffer starting
        address. */

   uint32_t mem_map_handle;
   /**< Unique identifier for a shared memory address.

        @values
        - NULL -- buffer address is the physical memory address and it doesn't
        need mapping.
        - Non-NULL -- when buffer starting address is a mapped address. */
}
#include "spf_end_pack.h"
;
typedef struct dls_buf_start_addr_t dls_buf_start_addr_t;

/** @ingroup spf_dls_commands
    Payload for #DLS_DATA_CMD_BUFFER_RETURN

    Consists the lower 32-bits and upper 32-bits of the logged buffer starting
    address. This address will be read by the client to parse the header and to
    work with the logged data.
*/
#include "spf_begin_pack.h"
struct dls_data_event_id_commit_log_buffer_t
{
   uint32_t num_bufs;
   /**< Total number of logged buffers that are committed. */

   dls_buf_start_addr_t buf_start_addr[0];
   /**< This is a flexible array member and it contains the following:
        lower 32-bits of the 64-bit buffer starting addresses
        upper 32-bits of the 64-bit buffer starting addresses */
}
#include "spf_end_pack.h"
;
typedef struct dls_data_event_id_commit_log_buffer_t dls_data_event_id_commit_log_buffer_t;

/** @ingroup spf_dls_commands
    Used to return the logged buffer. This data command will be issued by the
    client, after reading the logged buffer [after finishing the process of
    DLS_DATA_CMD_BUFFER_RETURN data event]. After receiving this command, the
    DLS server will clean the buffer and make it available for the next data
    logging.

    @msgpayload
    dls_data_cmd_buffer_return_t @newpage
*/
#define DLS_DATA_CMD_BUFFER_RETURN          0x0400100D

/** @ingroup spf_dls_commands
    Payload for #DLS_DATA_CMD_BUFFER_RETURN

    Consists the lower 32-bits and upper 32-bits of the buffer starting address
    to clear the buffer and to make it available for next data logging.
    It also includes a flag to indicate to clean the entire buffer.
*/
#include "spf_begin_pack.h"
struct dls_data_cmd_buffer_return_t
{
   uint32_t num_bufs;
   /**< Total number of buffers that need to be returned. */

   dls_buf_start_addr_t buf_start_addr[0];
   /**< This field contains the 64-bit buffer starting addresses */
}
#include "spf_end_pack.h"
;
typedef struct dls_data_cmd_buffer_return_t dls_data_cmd_buffer_return_t;
/** Used to identify the state of the buffer. When the buffer is first created
   implicit state would be DLS_BUF_AVAILABLE. When DLS Server requests for an
   available buffer [via posal data logging], the state will be updated to
   DLS_BUF_ACQUIRED. When the data written to buffer, the state will be updated
   to DLS_BUF_READY. Once the DLS client read the buffer, the buffer shall be
   reset using DLS_DATA_CMD_BUFFER_RETURN command and the server will update the
   buffer state to DLS_BUF_AVAILABLE.
 */
typedef enum
{
	DLS_BUF_UNAVAILABLE = 0,
	DLS_BUF_AVAILABLE,
	DLS_BUF_ACQUIRED,
	DLS_BUF_READY
} dls_buf_state_t;

#include "spf_begin_pack.h"
struct dls_buf_hdr_t
{
   uint32_t buf_size;
   /**< It indicates the size of the data logged into the buffer. This will be
        used at the client side to read buf_size number of bytes from the
        buffer */

   dls_buf_state_t buf_state;
   /**< It indicates the current state of a data logging buffer. This will be
        used in server to update/know the buffer state. This is useful to find
        the available buffer.
        The buffer state is not required at the client side but the client
        needs to skip this data to read the actual logged buffer. */

   uint32_t buf_in_use;
   /**< It indicates the available buffer space in percentage. This varies
        between 0 and 100. This is used at the client end and when the available
        buffer size goes below a threshold limit [decided by the client] client
        may take decision to reconfigure the buffer information. */
}
#include "spf_end_pack.h"
;
typedef struct dls_buf_hdr_t dls_buf_hdr_t;

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif // #ifdef _DLS_API_H_
