#ifndef _APM_CONTAINER_IF_H_
#define _APM_CONTAINER_IF_H_

/**
 * \file apm_cntr_if.h
 *  
 * \brief
 *     This file defines APM to container functions.
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "apm_api.h"
#include "apm_sub_graph_api.h"
#include "apm_container_api.h"
#include "apm_module_api.h"

#include "spf_utils.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

typedef struct spf_handle_t spf_handle_t;
typedef struct apm_imcl_peer_domain_info_t apm_imcl_peer_domain_info_t;
typedef struct  apm_mxd_heap_id_link_cfg_t apm_mxd_heap_id_link_cfg_t;

/**< Ths structure is ised to store the data/control link information
    between modules with different host containers. 
    The link info is stored in the context of the container 
    with default heapID connected to the non-default heap ID */
struct apm_mxd_heap_id_link_cfg_t
{
   void *conn_ptr;
   /*Pointer to the connection payload
     apm_module_conn_cfg_t** or
     apm_module_ctrl_link_cfg_t** */
   apm_cont_prop_id_heap_id_t heap_id;
   /*Heap id of the non-default container*/
};

/********************************************************************************************************/
/*                                          Messages                                                    */
/********************************************************************************************************/


typedef struct spf_cntr_sub_graph_list_t spf_cntr_sub_graph_list_t;

struct spf_cntr_sub_graph_list_t
{
   uint32_t num_sub_graph;
   /**< Number of sub-graphs addressed */

   uint32_t *sg_id_list_ptr;
   /**< Pointer to start of the sub-graph ID
        variable  length array.
        Size of the array is determined by
        number of sub-graphs. */
};

typedef enum spf_module_port_type_t
{
   PORT_TYPE_DATA_IP = 0,
   /**< Port Type Input Data*/

   PORT_TYPE_DATA_OP = 1,
   /**< Port Type Output Data*/

   PORT_TYPE_CTRL_IO = 2,
   /**< Port Type Control*/

   PORT_TYPE_MAX = 3
   /**< Max Container Port Type */

} spf_module_port_type_t;

typedef enum spf_module_link_type_t
{
   LINK_TYPE_DATA = 0,
   /**< Data connections between modules*/

   LINK_TYPE_CTRL = 1,
   /**< IMCL between modules*/

   LINK_TYPE_MAX
   /**< Max Module Link Types*/
}spf_module_link_type_t;

typedef struct spf_module_port_handle_t
{
   spf_handle_t *port_ctx_hdl;
   /**< Port context handle */

   uint32_t module_inst_id;
   /**< Module instance ID */

   uint32_t module_port_id;
   /**< Module port ID */

   uint32_t sub_graph_id;
   /**< subgraph ID of this module */

   uint32_t domain_id;
   /**< domain ID of this module */

   spf_module_port_type_t port_type;
   /**< Module port type */

} spf_module_port_handle_t;

typedef struct spf_module_port_conn_t spf_module_port_conn_t;

struct spf_module_port_conn_t
{
   spf_module_port_handle_t self_mod_port_hdl;
   /**< Module port self handle */

   spf_module_port_handle_t peer_mod_port_hdl;
   /**< Module port peer handle */
};

/**
   This payload structure is used for the following opcodes
     -#SPF_MSG_RSP_GRAPH_OPEN
     -#SPF_MSG_CMD_GRAPH_CONNECT

   Immediately following this structure contains the variable
   length array of structure objects #module_port_conn_t
   corresponding to input and output port handles for the
   container interacting with APM.

   As part of #SPF_MSG_RSP_GRAPH_OPEN, container allocates this
   payloads and populates the source handles corresponding to
   output ports and destination handles corresponding to input
   port.

   APM executes #SPF_MSG_CMD_GRAPH_CONNECT in response to
   #SPF_MSG_RSP_GRAPH_OPEN for a given container.

   As part of #SPF_MSG_CMD_GRAPH_CONNECT, APM reuses the same
   payload and populates the destination handles corresponding
   to output port and source handles corresponding to input port
   and sends the updated payload to container.

*/

typedef struct spf_cntr_port_connect_info_t spf_cntr_port_connect_info_t;

struct spf_cntr_port_connect_info_t
{
   uint32_t num_ip_data_port_conn;
   /**< Number of input data port connections */

   uint32_t num_op_data_port_conn;
   /**< Number of output data port connections */

   uint32_t num_ctrl_port_conn;
   /**< Number of control port connections */

   spf_module_port_conn_t *ip_data_port_conn_list_ptr;
   /**< Module input data port connection list.
        Variable length array.
        Size of the array is determined by
        number of input port connections. */

   spf_module_port_conn_t *op_data_port_conn_list_ptr;
   /**< Module output data port connection list.
        Variable length array.
        Size of the array is determined by
        number of output port connections. */

   spf_module_port_conn_t *ctrl_port_conn_list_ptr;
   /**< List of control port connection list.
        Variable length array.
        Size of the array is determined by
        number of control port connections */
};

/**
   Immediately following this structure contains the variable
   length array of structure objects #spf_handle_t
   corresponding to input and output port handles for the
   container interacting with APM.
*/

typedef struct spf_cntr_port_info_list_t spf_cntr_port_info_list_t;

struct spf_cntr_port_info_list_t
{
   uint32_t num_ip_port_handle;
   /**< Number of input port handles */

   uint32_t num_op_port_handle;
   /**< Number of output port handles */

   uint32_t num_ctrl_port_handle;
   /**< Number of output port handles */

   uint32_t num_data_links;
   /**< Number of data links configured */

   uint32_t num_ctrl_links;
   /**< Number of control links configured */

   spf_handle_t **ip_port_handle_list_pptr;
   /**< Container input port handle list.
        Variable length array.
        Size of the array is determined by
        number of input port handles */

   spf_handle_t **op_port_handle_list_pptr;
   /**< Container output port handle list.
        Variable length array.
        Size of the array is determined by
        number of outout port handles */

   spf_handle_t **ctrl_port_handle_list_pptr;
   /**< Container control port handle list.
        Variable length array.
        Size of the array is determined by
        number of control port handles */

   apm_module_conn_cfg_t **data_link_list_pptr;
   /**< Array of pointers for module port connection objects.
        Length of array is determined using the number
        of port connections being configured */

   apm_module_ctrl_link_cfg_t **ctrl_link_list_pptr;
   /**< Array of pointers for module control port connection objects.
        Length of array is determined using the number
        of control port connections being configured */
};

/**
   Payload structure of the parameters passed on by APM to
   container as part of SPF_MSG_CMD_GRAPH_OPEN message.

   Immediately following this structure contains the array of
   pointers which points to various configuration blocks
   received by APM as part of GRAPH_OPEN command from the
   client.

   Overall payload size includes the variable length array of
   pointers. Each array of pointers contains the configuration
   block for sub-graph, module list, module propety and module
   connection info as received from the client for this
   container.
*/

typedef struct spf_msg_cmd_graph_open_t spf_msg_cmd_graph_open_t;

struct spf_msg_cmd_graph_open_t
{
   apm_container_cfg_t *container_cfg_ptr;
   /**< Pointer to container config payload */

   uint32_t num_sub_graphs;
   /**< Number of sub-graphs being configured */

   apm_sub_graph_cfg_t **sg_cfg_list_pptr;
   /**< Array of pointers for sub-graph config
        payload. Length of array is determined
        using the number of sub-graphs. */

   uint32_t num_modules_list;
   /**< Number of modules being configured */

   apm_modules_list_t **mod_list_pptr;
   /**< Array of pointers for module list
        payload. Length of array is determined
        using the number of modules being
        configured */

   uint32_t num_mod_prop_cfg;
   /**< Number of module property configuration */

   apm_module_prop_cfg_t **mod_prop_cfg_list_pptr;
   /**< Array of pointers for module property object
        payload. Length of array is determined
        using the number of property objects being
        configured. Each object contains the module
        instance ID and property list for that module */

   uint32_t num_module_conn;
   /**< Number of connections */

   apm_module_conn_cfg_t **mod_conn_list_pptr;
   /**< Array of pointers for module port connection objects.
        Length of array is determined using the number
        of port connections being configured */

   uint32_t num_offloaded_imcl_peers;
   /**< Number of unique inter-proc ctrl peers */

   apm_imcl_peer_domain_info_t **imcl_peer_domain_info_pptr;
   /**< Array of pointers for module control port connection objects.
       Gives the info about the domain IDs of the peers.
       Length of array is determined using the number
       of unique inter-proc ctrl peers */

   uint32_t num_module_ctrl_links;
   /**< Number of module control links */

   apm_module_ctrl_link_cfg_t **mod_ctrl_link_cfg_list_pptr;
   /**< Array of pointers for module control port connection objects.
        Length of array is determined using the number
        of control port connections being configured */

   uint32_t num_param_id_cfg;
   /**< Number of parameters being configured */

   void **param_data_pptr;
   /**< Array of pointers for module instance param data
        payload. Variable length array of pointers.
        Length of array is determined by number
        of paramter ID's being configured.
        Each index points to param data payload. */

   uint32_t num_satellite_containers;
   /**< Number of satellite conatiners configured */

   apm_container_cfg_t **sat_cnt_config_pptr;
   /**< Array of pointers for satellite configuration data
        payload. Variable length array of pointers.
        Length of array is determined by number
        of satellite containers being configured. */

   uint32_t num_mxd_heap_id_data_links;
   /**< Number of data links between modules with different host containers. 
       The link info is stored in the context of the container 
       with default heapID connected to the non-default heap ID */

   apm_mxd_heap_id_link_cfg_t **mxd_heap_id_data_links_cfg_pptr;
   /**< Array of pointers for data link connections across this container and a 
        non-default heap container.*/

   uint32_t num_mxd_heap_id_ctrl_links;
  /**< Number of control links between modules with different host containers. 
       The link info is stored in the context of the container 
       with default heapID connected to the non-default heap ID */

   apm_mxd_heap_id_link_cfg_t **mxd_heap_id_ctrl_links_cfg_pptr;
   /**< Array of pointers for data link connections across this container and a 
        non-default heap container.*/
};

/** Graph Open Msg Opcode */
#define SPF_MSG_CMD_GRAPH_OPEN   0x0100102B

/** Graph Open Response Msg Opcode */
#define SPF_MSG_RSP_GRAPH_OPEN 0x0200100A

/**
   Payload structure of the parameters passed on by APM to
   container as part of following message ID's

    #SPF_MSG_CMD_SET_CFG
    #SPF_MSG_CMD_REGISTER_CFG
    #SPF_MSG_CMD_DEREGISTER_CFG

   Immediately following this structure contains the array of
   pointers which points to module's parameter ID data block as
   Received by APM as part of SET_CFG or
   REGISTER_CFG/DEREGISTER_CFG command from the client.

   Overall payload size includes the variable length array of
   pointers. Each pointer pointes to param data structure for
   each of the parameter ID configured for module instance ID
   present inside this container.
*/

typedef enum spf_cfg_data_type_t {
   SPF_CFG_DATA_TYPE_DEFAULT = 0,
   /**< Config data is neither persistent
     nor shared */

   SPF_CFG_DATA_PERSISTENT = 1,
   /**< Config data is persistent only */

   SPF_CFG_DATA_SHARED_PERSISTENT = 2,
   /**< Config data is persistent and shared
     across multiple instance of same module */

   SPF_CFG_DATA_INVALID = 0xFFFFFFFF
   /**< Invalid Config type */
} spf_cfg_data_type_t;

typedef struct spf_msg_cmd_param_data_cfg_t spf_msg_cmd_param_data_cfg_t;

struct spf_msg_cmd_param_data_cfg_t
{
   uint32_t num_param_id_cfg;
   /**< Number of parameters being configured */

   void **param_data_pptr;
   /**< Array of pointers for module instance param data
        payload. Variable length array of pointers.
        Length of array is determined by number
        of paramter ID's being configured.
        Each index points to param data payload. */
};
/** Set Config Msg Opcode */
#define SPF_MSG_CMD_SET_CFG      0x0100102C
/** Get Config Msg Opcode */
#define SPF_MSG_CMD_GET_CFG      0x0100102D
/** Register Config Msg Opcode */
#define SPF_MSG_CMD_REGISTER_CFG 0x0100102E
/** Deregister Config Msg Opcode */
#define SPF_MSG_CMD_DEREGISTER_CFG 0x0100102F



/**
   Payload structure of the parameters passed on by APM to
   container as part of following message ID's.

   #SPF_MSG_CMD_GRAPH_PREPARE
   #SPF_MSG_CMD_GRAPH_START
   #SPF_MSG_CMD_GRAPH_STOP
   #SPF_MSG_CMD_GRAPH_FLUSH
   #SPF_MSG_CMD_GRAPH_DISCONNECT
   #SPF_MSG_CMD_GRAPH_CLOSE

   Immediately following this structure contains the variable
   length array of pointers corresponding to list of container
   input and output port and list of sub-graph ID's present
   within the container.

   For the graph management command, if the sub-graph id
   addressed is present within the container then it is
   populated under sub-graph ID list.

   If the sub-graph ID addressed is not present within the
   container but it contains the list of container IO ports
   which are connected to this sub-graph ID, then these ports
   are populted under IO port handle list.

   For a given container, sub-graph ID and port ID list is
   populated as per availability and either of the list could be
   empty based upon the sub-graph ID list received as part of
   sub-graph command.

*/

typedef struct spf_msg_cmd_graph_mgmt_t spf_msg_cmd_graph_mgmt_t;

struct spf_msg_cmd_graph_mgmt_t
{
   spf_cntr_sub_graph_list_t sg_id_list;
   /**< List of sub-graph ID's being processed. */

   spf_cntr_port_info_list_t cntr_port_hdl_list;
   /**< List of container port handles */
};
/** Graph Prepare Msg Opcode  */
#define SPF_MSG_CMD_GRAPH_PREPARE 0x01001030
/** Graph Start Msg Opcode */
#define SPF_MSG_CMD_GRAPH_START  0x01001031
/** Graph Stop Msg Opcode */
#define SPF_MSG_CMD_GRAPH_STOP   0x01001032
/** Graph Flush Msg Opcode */
#define SPF_MSG_CMD_GRAPH_FLUSH  0x01001033
/** Graph Close Msg Opcode */
#define SPF_MSG_CMD_GRAPH_CLOSE  0x01001034
/** Graph Connect Msg Opcode */
#define SPF_MSG_CMD_GRAPH_CONNECT 0x01001035
/** Graph Disconnect Msg Opcode */
#define SPF_MSG_CMD_GRAPH_DISCONNECT 0x01001036
/** Graph Destroy Msg Opcode */
#define SPF_MSG_CMD_DESTROY_CONTAINER 0x01001037
/** Graph Suspend Msg Opcode */
#define SPF_MSG_CMD_GRAPH_SUSPEND  0x01001044


/********************************************************************************************************/
/*                                          Functional calls                                             */
/********************************************************************************************************/
/**
 @brief Data structure for initializing a container
 */
typedef struct cntr_cmn_init_params_t
{
   apm_container_cfg_t *container_cfg_ptr;
   /**< container config from open command */
   uint16_t log_seq_id;
   /**< An ID for logging (Serial number incremented
        by APM every time a container is created).
        The container pads its own 16 bits to form 32 bit ID.
        */

} cntr_cmn_init_params_t;

ar_result_t cntr_cmn_create(cntr_cmn_init_params_t *init_param_ptr, spf_handle_t **cntr_handle);

/**
 * Container thread can re-launch itself whenever stack size requirement changes.
 * Example: graph open, set-cfg commands can potentially change stack size requirements
 *
 * When APM gets graph-close cmd, it's split into graph-disconnect and close command to the containers.
 * When close is being processed, if last graph goes away, the container thread kills itself.
 * The container must not free it's me_ptr or handle at this time.
 * The APM finally makes cntr_cmn_destroy call once close is acked by the container.
 * APM cannot cache the thread-id before sending close cmd because, a module may receive set-cfg in the mean time
 * and thread-id could potentially change.
 *
 */
ar_result_t cntr_cmn_destroy(spf_handle_t *cntr_handle);

void cntr_cmn_dump_debug_info(spf_handle_t *cntr_handle, uint32_t type, int8_t *start_address, uint32_t max_size);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef _APM_CONTAINER_IF_H_
