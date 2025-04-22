#ifndef _APM_DEBUG_INFO_DUMP__
#define _APM_DEBUG_INFO_DUMP__

/**
 * \file apm_debug_info_dump.h
 *
 * \brief
 *     This file contains function declaration for APM utilities for APM_CMD_CLOSE_ALL handling
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */



#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/**------------------------------------------------------------------------------
 *  Structure Definition
 *----------------------------------------------------------------------------*/

typedef struct apm_cmd_ctrl_debug_info_t
{

     uint32_t   cmd_opcode;
     /**< Command opcode under process */
     ar_result_t    cmd_status;
     /**< Current status of command*/
     bool_t         cmd_pending;
     /**< Overall command in pending status */
     ar_result_t         agg_rsp_status;
     /**< Aggregated response status of one step of a command */

}apm_cmd_ctrl_debug_info_t;

typedef struct apm_debug_info_t
{

    uint32_t    curr_wait_mask;
    /**< Channel mask or signals to act */

    uint32_t    channel_status;
    /**< Current signal received */
    uint32_t apm_container_debug_size;
    /**< Size allocated for both container and containerr*/

    uint32_t size_per_container;

    uint32_t num_containers;

    apm_cmd_ctrl_debug_info_t cmd_ctrl_debug;

    //uint32_t container_start_address_array[0];
    /**< array of container start address will be present following the above variable cmd_ctrl_debug*/

}apm_debug_info_t;

/**------------------------------------------------------------------------------
 *  Function Declaration
 *----------------------------------------------------------------------------*/

void apm_cntr_dump_debug_info(void *callback_data,int8_t *start_address,uint32_t max_size);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* _APM_DEBUG_INFO_DUMP__ */
