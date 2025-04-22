#ifndef _APM_MODULE_API_H_
#define _APM_MODULE_API_H_
/**
 * \file apm_module_api.h
 * \brief
 *    This file contains APM module related commands and events structures definitions
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_defs.h"
#include "apm_graph_properties.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/*--------------------------------------------------------------------------------------------------------------------*/
/** @ingroup spf_apm_module_props
    Identifier for the parameter that configures the modules list for a given
    combination of sub-graph and container IDs. This parameter ID must be used
    with #APM_CMD_GRAPH_OPEN.

    @msgpayload
    apm_param_id_modules_list_t \n
    @indent{12pt} apm_modules_list_t \n
    @indent{12pt} apm_module_cfg_t
 */
#define APM_PARAM_ID_MODULES_LIST 0x08001002

/*# h2xmlp_parameter   {"APM_PARAM_ID_MODULES_LIST",
                          APM_PARAM_ID_MODULES_LIST}
    h2xmlp_description {ID for the parameter that configures the modules list
                         for a given combination of sub-graph and container
                         IDs.}
    h2xmlp_toolPolicy  {RTC; Calibration} */

/*# h2xmlgp_configType   {"MOD_CFG"}
    @h2xmlgp_config      {"MOD_CFG", "APM_PARAM_ID_MODULES_LIST", APM_PARAM_ID_MODULES_LIST}
    @h2xmlgp_description {Parameter that configures the modules list
                          for a given combination of sub-graph and container IDs.}
    @{                   <-- Start of module_cfg --> */

/** @ingroup spf_apm_module_props
    Payload for #APM_PARAM_ID_MODULES_LIST.

    Immediately following this structure is a variable length array of sub-graphs
    (apm_modules_list_t).
 */
#include "spf_begin_pack.h"
struct apm_param_id_modules_list_t
{
   uint32_t num_modules_list;
   /**< Number of module lists being configured. */

   /*#< h2xmle_description {Number of sub-graphs being configured.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */
}
#include "spf_end_pack.h"
;
typedef struct apm_param_id_modules_list_t apm_param_id_modules_list_t;

/** @ingroup spf_apm_module_props
    Payload for #APM_PARAM_ID_MODULES_LIST. This structure contains information
    for configuring sub-graphs and containers.

    Immediately following this structure is a variable length array of the modules
    (apm_module_cfg_t).
 */
#include "spf_begin_pack.h"
struct apm_modules_list_t
{
   uint32_t sub_graph_id;
   /**< Unique identifier for the sub-graph ID being configured. */

   /*#< h2xmle_description {Unique ID for the sub-graph ID being configured.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */

   uint32_t container_id;
   /**< Unique identifier for the container being configured. */

   /*#< h2xmle_description {Unique ID for the container being configured.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */

   uint32_t num_modules;
   /**< Number of modules being configured. @newpagetable */

   /*#< h2xmle_description {Number of modules being configured.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */
}
#include "spf_end_pack.h"
;
typedef struct apm_modules_list_t apm_modules_list_t;

/** @ingroup spf_apm_module_props
    Payload for #APM_PARAM_ID_MODULES_LIST. This structure contains information
    for configuring modules.
 */
#include "spf_begin_pack.h"
struct apm_module_cfg_t
{
   uint32_t module_id;
   /**< Unique identifier for the module. */

   /*#< h2xmle_description {Unique ID for the module.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */

   uint32_t instance_id;
   /**< Instance identifier for the module. */

   /*#< h2xmle_description {Instance ID for the module.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */
}
#include "spf_end_pack.h"
;
typedef struct apm_module_cfg_t apm_module_cfg_t;

/*# @h2xmlp_insertSubStruct {apm_module_cfg_t} */

/*--------------------------------------------------------------------------------------------------------------------*/

/** @ingroup spf_apm_module_props
    Identifier for the parameter that configures the module properties.

    @msgpayload
    apm_param_id_module_prop_t \n
    @indent{12pt} apm_module_prop_cfg_t
 */
#define APM_PARAM_ID_MODULE_PROP 0x08001003

/*# h2xmlp_parameter   {"APM_PARAM_ID_MODULE_PROP",
                          APM_PARAM_ID_MODULE_PROP}
    h2xmlp_description {ID for the parameter that configures the module properties.}
    h2xmlp_toolPolicy  {RTC; Calibration} */

/*# @h2xmlp_property   {"Module Property", APM_PARAM_ID_MODULE_PROP}
    @h2xmlp_description {ID for the parameter that configures the module properties.} */

/** @ingroup spf_apm_module_props
    Payload for #APM_PARAM_ID_MODULE_PROP. This structure contains property
    configuration objects.

    Immediately following this structure is a variable length array of property
    objects.
 */
#include "spf_begin_pack.h"
struct apm_param_id_module_prop_t
{
   uint32_t num_module_prop_cfg;
   /**< Number of module property configuration objects. */

   /*#< h2xmle_description {Number of module property configuration objects.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */
}
#include "spf_end_pack.h"
;
typedef struct apm_param_id_module_prop_t apm_param_id_module_prop_t;

/** @ingroup spf_apm_module_props
    Payload for #APM_PARAM_ID_MODULE_PROP.

    Immediately following this structure is a variable length array of structure
    objects that correspond to each module property. The length of the array is
    determined by the number of module properties being configured.
 */
#include "spf_begin_pack.h"
struct apm_module_prop_cfg_t
{
   uint32_t instance_id;
   /**< Instance identifier for the module. */

   /*#< h2xmle_description {Instance identifier for the module.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */

   uint32_t num_props;
   /**< Number of properties for the module (default = 0). */

   /*#< h2xmle_description {Number of properties for the module.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */
}
#include "spf_end_pack.h"
;
typedef struct apm_module_prop_cfg_t apm_module_prop_cfg_t;

/*--------------------------------------------------------------------------------------------------------------------*/
/** @ingroup spf_apm_module_props
    Property identifier for the parameter used to provide port information for
    the module.

    @msgpayload
    apm_module_prop_id_port_info_t
 */
#define APM_MODULE_PROP_ID_PORT_INFO 0x08001015

/*# h2xmlp_parameter   {"APM_MODULE_PROP_ID_PORT_INFO",
                          APM_MODULE_PROP_ID_PORT_INFO}
    h2xmlp_description {Property ID for the parameter used to provide port
                         information for the module.}
    h2xmlp_toolPolicy  {RTC; Calibration} */

/*# @h2xmlp_property    {"Module Port Info", APM_MODULE_PROP_ID_PORT_INFO}
    @h2xmlp_description {Property ID used to provide port information for the module.} */

/** @ingroup spf_apm_module_props
    Payload for #APM_MODULE_PROP_ID_PORT_INFO.
 */
#include "spf_begin_pack.h"
struct apm_module_prop_id_port_info_t
{
   uint32_t max_ip_port;
   /**< Maximum number of input ports. */

   /*#< h2xmle_description {Maximum number of input ports.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */

   uint32_t max_op_port;
   /**< Maximum number of output ports. @newpagetable */

   /*#< h2xmle_description {Maximum number of output ports.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */
}
#include "spf_end_pack.h"
;
typedef struct apm_module_prop_id_port_info_t apm_module_prop_id_port_info_t;

/*--------------------------------------------------------------------------------------------------------------------*/
/** @ingroup spf_apm_module_props
    Identifier for the parameter that configures the connection between module
    ports. This parameter ID must be used with #APM_CMD_GRAPH_OPEN.

    Configure connection links between each input and output port of the
    modules in the sub-graph. The connection between source and destination
    module ports is one-to-one. That is, the source module's output cannot be
    routed to multiple input ports of one or more destination modules.

    Similarly, the destination module's input port cannot accept data from
    multiple output ports of one or more source modules. Any such connection is
    treated as invalid configuration and is flagged as an error to the client.

    @msgpayload
    apm_param_id_module_conn_t \n
    @indent{12pt} apm_module_conn_cfg_t
 */
#define APM_PARAM_ID_MODULE_CONN 0x08001004

/*# h2xmlp_parameter   {"APM_PARAM_ID_MODULE_CONN", APM_PARAM_ID_MODULE_CONN}
    h2xmlp_description {ID for the parameter that configures the connection
                         between module ports. This parameter ID must be used
                         with APM_CMD_GRAPH_OPEN. \n
                         Configure connection links between each of the input
                         and output ports of modules present in the sub-graph.
                         The connection between source and destination module
                         ports is one-to-one. That is, the source module's
                         output cannot be routed to multiple input ports of
                         one or more destination modules. \n
                         Similarly, the destination module's input port cannot
                         accept data from multiple output ports of one or more
                         source modules. Any such connection is treated as
                         invalid configuration and is flagged as an error to
                         the client.}
    h2xmlp_toolPolicy  {RTC; Calibration} */

/** @ingroup spf_apm_module_props
    Payload for #APM_PARAM_ID_MODULE_CONN.

    Immediately following this structure is a variable length array of
    connection objects. The length of the array is determined by the number of
    connections.
 */
#include "spf_begin_pack.h"
struct apm_param_id_module_conn_t
{
   uint32_t num_connections;
   /**< Number of connections between the input and output ports. */

   /*#< h2xmle_description {Number of connections.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */
}
#include "spf_end_pack.h"
;
typedef struct apm_param_id_module_conn_t apm_param_id_module_conn_t;

/** @ingroup spf_apm_module_props
    Payload for #APM_PARAM_ID_MODULE_CONN. This structure contains connection
    objects.
 */
#include "spf_begin_pack.h"
struct apm_module_conn_cfg_t
{
   uint32_t src_mod_inst_id;
   /**< Instance identifier for the source module. */

   /*#< h2xmle_description {Instance ID for the source module.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */

   uint32_t src_mod_op_port_id;
   /**< Output port identifier for the source module. */

   /*#< h2xmle_description {Output port ID for the source module.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */

   uint32_t dst_mod_inst_id;
   /**< Instance identifier for the destination module. */

   /*#< h2xmle_description {Instance ID for the destination module.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */

   uint32_t dst_mod_ip_port_id;
   /**< Input port identifier for the destination module. */

   /*#< h2xmle_description {Input port ID for the destination module.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */
}
#include "spf_end_pack.h"
;
typedef struct apm_module_conn_cfg_t apm_module_conn_cfg_t;

/*--------------------------------------------------------------------------------------------------------------------*/
/** @ingroup spf_apm_module_props
    Identifier for the parameter that configures the list of control links
    across module control ports.

    @msgpayload
    apm_param_id_module_ctrl_link_cfg_t \n
    @indent{12pt} apm_module_ctrl_link_cfg_t
 */
#define APM_PARAM_ID_MODULE_CTRL_LINK_CFG 0x08001061

/*# h2xmlp_parameter   {"APM_PARAM_ID_MODULE_CTRL_LINK_CFG",
                          APM_PARAM_ID_MODULE_CTRL_LINK_CFG}
    h2xmlp_description {ID for the parameter that configures the list of
                         control links across module control ports.}
    h2xmlp_toolPolicy  {RTC; Calibration} */

/** @ingroup spf_apm_module_props
    Payload for #APM_PARAM_ID_MODULE_CTRL_LINK_CFG.

    Immediately following this structure is a variable length array of
    structure objects that correspond to configuration of the module control
    port link. The length of the array is determined by the number of control
    links.
 */
#include "spf_begin_pack.h"
struct apm_param_id_module_ctrl_link_cfg_t
{
   uint32_t num_ctrl_link_cfg;
   /**< Number of module control link objects being configured. @newpagetable */

   /*#< h2xmle_description {Number of module control link objects being
                             configured.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */
}
#include "spf_end_pack.h"
;
typedef struct apm_param_id_module_ctrl_link_cfg_t apm_param_id_module_ctrl_link_cfg_t;

/** @ingroup spf_apm_module_props
    Payload for #APM_PARAM_ID_MODULE_CTRL_LINK_CFG. This structure contains
    configuration parameters.

    Immediately following this structure is a variable length array of
    structure objects that correspond to each module control link property.
    The length of the array is determined by number of control link properties
    being configured.
 */
#include "spf_begin_pack.h"
struct apm_module_ctrl_link_cfg_t
{
   uint32_t peer_1_mod_iid;
   /**< Instance identifier for the first peer module. */

   /*#< h2xmle_description {Instance ID for the first peer module.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */

   uint32_t peer_1_mod_ctrl_port_id;
   /**< Control port identifier for the first peer module. */

   /*#< h2xmle_description {Control port ID for the first peer module.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */

   uint32_t peer_2_mod_iid;
   /**< Instance identifier for the second peer module. */

   /*#< h2xmle_description {Instance ID for the second peer module.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */

   uint32_t peer_2_mod_ctrl_port_id;
   /**< Control port identifier for the second peer module. */

   /*#< h2xmle_description {Control port ID for the second peer module.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */

   uint32_t num_props;
   /**< Number of link properties (default = 0). */

   /*#< h2xmle_description {Number of link properties.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */
}
#include "spf_end_pack.h"
;
typedef struct apm_module_ctrl_link_cfg_t apm_module_ctrl_link_cfg_t;

/*--------------------------------------------------------------------------------------------------------------------*/
/** @ingroup spf_apm_module_props
    Property identifier for control link intent information.

    @msgpayload
    apm_module_ctrl_link_prop_id_intent_list_t
 */
#define APM_MODULE_PROP_ID_CTRL_LINK_INTENT_LIST 0x08001062

/*# h2xmlp_parameter   {"APM_MODULE_PROP_ID_CTRL_LINK_INTENT_LIST",
                          APM_MODULE_PROP_ID_CTRL_LINK_INTENT_LIST}
    h2xmlp_description {Property ID for control link intent information.}
    h2xmlp_toolPolicy  {RTC; Calibration} */

/** @ingroup spf_apm_module_props
    Payload for #APM_MODULE_PROP_ID_CTRL_LINK_INTENT_LIST.

    Immediately following this structure is a variable length array of intent
    IDs for each control link. The length of the array is determined by the
    number of control link intents.
 */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct apm_module_ctrl_link_prop_id_intent_list_t
{
   uint32_t num_intents;
   /**< Number of intents in the array (default = 0). */

   /*#< h2xmle_description {Number of intents in the array.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */

   uint32_t intent_id_list[0];
   /**< Array of intent IDs associated with a control link. */

   /*#< h2xmle_description {Array of intent IDs associated with a control
                             link.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct apm_module_ctrl_link_prop_id_intent_list_t apm_module_ctrl_link_prop_id_intent_list_t;

/*--------------------------------------------------------------------------------------------------------------------*/
/** @ingroup spf_apm_module_props
    Property identifier for the control link heap id.

    @msgpayload
    apm_module_prop_id_ctrl_link_heap_id_t
 */
#define APM_MODULE_PROP_ID_CTRL_LINK_HEAP_ID 0x0800136F

/*# h2xmlp_parameter   {"APM_MODULE_PROP_ID_CTRL_LINK_HEAP_ID",
                          APM_MODULE_PROP_ID_CTRL_LINK_HEAP_ID}
    h2xmlp_description {Property ID for control link heap ID.} */

/** @ingroup spf_apm_module_props
    Payload for #APM_MODULE_PROP_ID_CTRL_LINK_HEAP_ID.

 */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct apm_module_prop_id_ctrl_link_heap_id_t
{
   uint32_t heap_id;
   /**< Control link heap ID.

        @valuesbul
        - #APM_HEAP_ID_DEFAULT
        - #APM_HEAP_ID_LOW_POWER
        - #APM_HEAP_ID_LOW_POWER_2 @tablebulletend */

   /*#< @h2xmle_rangeList   {"Default"=APM_HEAP_ID_DEFAULT,
                             "Low Power"=APM_HEAP_ID_LOW_POWER,
                             "Low Power 2"=APM_HEAP_ID_LOW_POWER_2}
        @h2xmle_default     {APM_HEAP_ID_DEFAULT}
        @h2xmle_description {Control link heap ID.} */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct apm_module_prop_id_ctrl_link_heap_id_t apm_module_prop_id_ctrl_link_heap_id_t;

/*--------------------------------------------------------------------------------------------------------------------*/
/** @ingroup spf_apm_module_props
    Property identifier for the module heap id.

    @msgpayload
    apm_module_prop_id_module_heap_id_t
 */
#define APM_MODULE_PROP_ID_HEAP_ID 0x08001A9A

/*# h2xmlp_parameter   {"APM_MODULE_PROP_ID_HEAP_ID",
                          APM_MODULE_PROP_ID_HEAP_ID}
    h2xmlp_description {Property ID used to provide heap id for the module.}
    h2xmlp_toolPolicy  {RTC; Calibration} */

/*# @h2xmlp_property    {"Module Heap ID", APM_MODULE_PROP_ID_HEAP_ID}
    @h2xmlp_description {Property ID used to provide heap id for the module.} */

/** @ingroup spf_apm_module_props
    Payload for #APM_MODULE_PROP_ID_HEAP_ID.
*/
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct apm_module_prop_id_heap_id_t
{
   uint32_t heap_id;
   /**< Module heap ID.

        @valuesbul
        - #APM_HEAP_ID_DEFAULT
        - #APM_HEAP_ID_LOW_POWER
        - #APM_HEAP_ID_LOW_POWER_2	   @tablebulletend */

   /*#< @h2xmle_rangeList   {"Default"=APM_HEAP_ID_DEFAULT,
                             "Low Power"=APM_HEAP_ID_LOW_POWER,
                             "Low Power 2"=APM_HEAP_ID_LOW_POWER_2}
        @h2xmle_default     {APM_HEAP_ID_DEFAULT}
        @h2xmle_description {Module heap ID.} */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct apm_module_prop_id_heap_id_t apm_module_prop_id_heap_id_t;

/*--------------------------------------------------------------------------------------------------------------------*/
/*# @}                   <-- End of module_cfg -->*/

/*====================================================================================================================*/
/*====================================================================================================================*/

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* _APM_MODULE_API_H_ */
