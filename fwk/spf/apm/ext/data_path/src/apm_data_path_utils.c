/**
 * \file apm_data_path_utils.c
 *
 * \brief
 *     This file contains utility functions for APM Data Path Handling utility functions
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/****************************************************************************
 * INCLUDE HEADER FILES                                                     *
 ****************************************************************************/

#include "apm_i.h"
#include "apm_graph_utils.h"
#include "apm_data_path_utils_i.h"
#include "posal_intrinsics.h"
#include "spf_list_utils.h"
#include "apm_cmd_utils.h"
#include "spf_utils.h"
#include "offload_path_delay_api.h"

#define APM_DEBUG_DATA_PATH 1

/**==============================================================================
   Function Declaration
==============================================================================*/

ar_result_t apm_data_path_config_cmn_sequencer(apm_t *apm_info_ptr, bool_t data_path_create);

ar_result_t apm_close_data_path_list(apm_t *apm_info_ptr);

ar_result_t apm_update_get_data_path_cfg_rsp_payload(apm_t *                  apm_info_ptr,
                                                     spf_msg_header_t *       msg_hdr_ptr,
                                                     apm_module_param_data_t *cont_param_data_hdr_ptr);

ar_result_t apm_cont_path_delay_msg_rsp_hdlr(apm_t *apm_info_ptr);

ar_result_t apm_path_delay_event_hdlr(apm_t *apm_info_ptr, spf_msg_t *msg_ptr);

ar_result_t apm_process_get_cfg_path_delay(apm_t *apm_info_ptr, apm_module_param_data_t *get_cfg_rsp_payload_ptr);

ar_result_t apm_destroy_one_time_data_paths(apm_t *apm_info_ptr);

ar_result_t apm_update_data_path_list(apm_t *          apm_info_ptr,
                                      apm_module_t *   self_module_node_ptr,
                                      spf_list_node_t *data_link_list_ptr,
                                      bool_t           is_module_close);

ar_result_t apm_graph_open_cmd_cache_data_link(apm_module_conn_cfg_t *data_link_cfg_ptr,
                                               apm_module_t **        module_node_pptr);

ar_result_t apm_compute_cntr_path_delay_param_payload_size(uint32_t                    container_id,
                                                           apm_cont_set_get_cfg_hdr_t *set_get_cfg_hdr_ptr,
                                                           uint32_t *                  msg_payload_size_ptr);

ar_result_t apm_populate_cntr_path_delay_params(uint32_t                    container_id,
                                                apm_cont_set_get_cfg_hdr_t *set_get_cfg_hdr_ptr,
                                                apm_module_param_data_t *   param_data_ptr);

ar_result_t apm_data_path_clear_cached_cont_cfg_params(spf_list_node_t *param_node_ptr);

ar_result_t apm_db_get_data_path_obj(spf_list_node_t *            data_path_list_ptr,
                                     uint32_t                     path_id,
                                     apm_data_path_delay_info_t **data_path_delay_info_pptr);

ar_result_t apm_clear_module_data_port_conn(apm_t *apm_info_ptr, apm_module_t *self_module_node_ptr);

ar_result_t apm_clear_module_single_port_conn(apm_t *apm_info_ptr, spf_module_port_conn_t *module_port_conn_ptr);

ar_result_t apm_clear_closed_cntr_from_data_paths(apm_t *apm_info_ptr, apm_container_t *closing_container_node_ptr);

/**==============================================================================
   Global Defines
==============================================================================*/

apm_data_path_utils_vtable_t data_path_util_funcs = {
   .apm_data_path_cfg_cmn_seqncer_fptr = apm_data_path_config_cmn_sequencer,

   .apm_close_data_path_list_fptr = apm_close_data_path_list,

   .apm_update_get_data_path_cfg_rsp_payload_fptr = apm_update_get_data_path_cfg_rsp_payload,

   .apm_cont_path_delay_msg_rsp_hdlr_fptr = apm_cont_path_delay_msg_rsp_hdlr,

   .apm_path_delay_event_hdlr_fptr = apm_path_delay_event_hdlr,

   .apm_process_get_cfg_path_delay_fptr = apm_process_get_cfg_path_delay,

   .apm_destroy_one_time_data_paths_fptr = apm_destroy_one_time_data_paths,

   .apm_graph_open_cmd_cache_data_link_fptr = apm_graph_open_cmd_cache_data_link,

   .apm_compute_cntr_path_delay_param_payload_size_fptr = apm_compute_cntr_path_delay_param_payload_size,

   .apm_populate_cntr_path_delay_params_fptr = apm_populate_cntr_path_delay_params,

   .apm_data_path_clear_cached_cont_cfg_params_fptr = apm_data_path_clear_cached_cont_cfg_params,

   .apm_clear_module_data_port_conn_fptr = apm_clear_module_data_port_conn,

   .apm_clear_module_single_port_conn_fptr = apm_clear_module_single_port_conn,

   .apm_clear_closed_cntr_from_data_paths_fptr = apm_clear_closed_cntr_from_data_paths
};

ar_result_t apm_clear_closed_cntr_from_data_paths(apm_t *apm_info_ptr, apm_container_t *closing_container_node_ptr)
{
   ar_result_t                 result = AR_EOK;
   spf_list_node_t *           curr_data_path_node_ptr;
   apm_data_path_delay_info_t *delay_data_path_ptr;
   spf_list_node_t *           curr_container_node_ptr;
   apm_container_t *           curr_container_ptr;
   spf_list_node_t *           next_container_node_ptr;

   /** Get the pointer to available data paths */
   curr_data_path_node_ptr = apm_info_ptr->graph_info.data_path_list_ptr;

   /** Iterate over the the list of available data paths */
   while (curr_data_path_node_ptr)
   {
      /** Get the pointer to path delay obj */
      delay_data_path_ptr = (apm_data_path_delay_info_t *)curr_data_path_node_ptr->obj_ptr;

      curr_container_node_ptr = (spf_list_node_t *)delay_data_path_ptr->data_path.container_list.list_ptr;

      while (curr_container_node_ptr)
      {
         curr_container_ptr = (apm_container_t *)curr_container_node_ptr->obj_ptr;

         next_container_node_ptr = curr_container_node_ptr->next_ptr;

         if (curr_container_ptr->container_id == closing_container_node_ptr->container_id)
         {
            if (AR_EOK !=
                (result = apm_db_remove_node_from_list(&delay_data_path_ptr->data_path.container_list.list_ptr,
                                                       curr_container_ptr,
                                                       &delay_data_path_ptr->data_path.container_list.num_nodes)))
            {
#ifdef APM_DEBUG_DATA_PATH
               AR_MSG(DBG_ERROR_PRIO,
                      "apm_clear_closed_cntr_from_data_paths(): Failed to Remove CONT_ID[0x%lX] from path_id[%lu] "
                      "which is getting "
                      "closed, total containers left in this path [%lu]",
                      closing_container_node_ptr->container_id,
                      delay_data_path_ptr->data_path.path_id,
                      delay_data_path_ptr->data_path.container_list.num_nodes);
#endif
            }
            else
            {
#ifdef APM_DEBUG_DATA_PATH
               AR_MSG(DBG_HIGH_PRIO,
                      "apm_clear_closed_cntr_from_data_paths(): Removed CONT_ID[0x%lX] from path_id[%lu] which is "
                      "getting "
                      "closed, total containers left in this path [%lu]",
                      closing_container_node_ptr->container_id,
                      delay_data_path_ptr->data_path.path_id,
                      delay_data_path_ptr->data_path.container_list.num_nodes);
#endif
            }
         }

         curr_container_node_ptr = next_container_node_ptr;
      }

      /** Advance to next node in the list */
      curr_data_path_node_ptr = curr_data_path_node_ptr->next_ptr;
   }

   return result;
}

ar_result_t apm_update_data_path_list(apm_t *          apm_info_ptr,
                                      apm_module_t *   self_module_node_ptr,
                                      spf_list_node_t *data_link_list_ptr,
                                      bool_t           is_module_close)
{
   ar_result_t                 result = AR_EOK;
   spf_list_node_t *           curr_path_node_ptr, *next_path_node_ptr;
   spf_list_node_t *           curr_vrtx_node_ptr, *next_vrtx_node_ptr;
   spf_list_node_t *           curr_data_link_node_ptr;
   apm_data_path_delay_info_t *data_path_info_ptr;
   cntr_graph_vertex_t *       graph_vertex_ptr;
   uint32_t                    path_id;
   apm_module_data_link_t *    mod_data_link_ptr;

   if (!self_module_node_ptr || !data_link_list_ptr)
   {
      AR_MSG(DBG_HIGH_PRIO,
             "apm_update_data_path_list(): Invalid i/p arguments, self_module_node_ptr[0x%lX], "
             "data_link_list_ptr[0x%lX]",
             self_module_node_ptr,
             data_link_list_ptr);

      return AR_EFAILED;
   }

   /** Get the pointer to the global list of data paths */
   curr_path_node_ptr = apm_info_ptr->graph_info.data_path_list_ptr;

   /** Iterate over the list of data paths */
   while (curr_path_node_ptr)
   {
      /** Get the pointer to current data path info object */
      data_path_info_ptr = (apm_data_path_delay_info_t *)curr_path_node_ptr->obj_ptr;

      /** Get pointer to next node in the list   */
      next_path_node_ptr = curr_path_node_ptr->next_ptr;

      /** Get the path ID */
      path_id = data_path_info_ptr->data_path.path_id;

      /** Get the pointer to data links being removed */
      curr_data_link_node_ptr = data_link_list_ptr;

      /** Iterate over the list of data links to be removed  */
      while (curr_data_link_node_ptr)
      {
         /** Get the pointer to data link object */
         mod_data_link_ptr = (apm_module_data_link_t *)curr_data_link_node_ptr->obj_ptr;

         /** Get the pointer to list of graph vertices for current
          *  data path */
         curr_vrtx_node_ptr = data_path_info_ptr->data_path.vertices_list_ptr;

         /** Iterate over the list of vertices, find the matching
          *  module ID and remove it from the list */
         while (curr_vrtx_node_ptr)
         {
            /** Get the pointer to current vertex object */
            graph_vertex_ptr = (cntr_graph_vertex_t *)curr_vrtx_node_ptr->obj_ptr;

            /** Cache the next node pointer */
            next_vrtx_node_ptr = curr_vrtx_node_ptr->next_ptr;

            /** Check if the module instance ID matches with the module
             *  being destroyed as part of the close command */
            if (((graph_vertex_ptr->module_instance_id == mod_data_link_ptr->src_port.module_instance_id) &&
                 (graph_vertex_ptr->port_id == mod_data_link_ptr->src_port.port_id)) ||
                ((graph_vertex_ptr->module_instance_id == mod_data_link_ptr->dstn_port.module_instance_id) &&
                 (graph_vertex_ptr->port_id == mod_data_link_ptr->dstn_port.port_id)))
            {
               /** Delete this node from the list */
               spf_list_find_delete_node(&data_path_info_ptr->data_path.vertices_list_ptr,
                                         curr_vrtx_node_ptr->obj_ptr,
                                         TRUE);

               /** Decrement the number of vertices in the data path */
               data_path_info_ptr->data_path.num_vertices--;

#ifdef APM_DEBUG_DATA_PATH
               AR_MSG(DBG_HIGH_PRIO,
                      "apm_update_data_path_list(): Removed vrtx m_iid[0x%lX], port_id[0x%lX], remaining vrtx[%lu], "
                      "path id[%lu]",
                      graph_vertex_ptr->module_instance_id,
                      graph_vertex_ptr->port_id,
                      data_path_info_ptr->data_path.num_vertices,
                      path_id);
#endif

               /** Set the data path to be invalid */
               if (data_path_info_ptr->data_path.flags.path_valid)
               {
                  data_path_info_ptr->data_path.flags.path_valid = FALSE;

                  /** Set the flag to inform containers about data path being
                   *  invalid */
                  data_path_info_ptr->data_path.flags.container_path_update = TRUE;

                  /** Clear the number of containers */
                  data_path_info_ptr->data_path.num_containers = 0;
               }
            }

            /** Advance to next node in the list */
            curr_vrtx_node_ptr = next_vrtx_node_ptr;

         } /** End of while (curr data path vertices) */

         /** Advance to next node in the list */
         curr_data_link_node_ptr = curr_data_link_node_ptr->next_ptr;

      } /** End of while (data links) */

      /** This routine gets called for both data links close and
       *  module close. Flags below should be set only if
       *  module is getting closed */
      if (is_module_close)
      {
         /** If the source module in the data path is being closed, mark
          *  this path object for removal */
         if (self_module_node_ptr->instance_id == data_path_info_ptr->data_path.path_dfn.src_module_instance_id)
         {
            data_path_info_ptr->data_path.flags.src_module_closed = TRUE;
         }
         else if (self_module_node_ptr->instance_id == data_path_info_ptr->data_path.path_dfn.dst_module_instance_id)
         {
            data_path_info_ptr->data_path.flags.dstn_module_closed = TRUE;
         }
      }

      /** If all the vertices are removed from the data path and
       *  source module is also closed, free up the allocated object
       *  for this data path and remove it from APM global data path
       *  list */
      if (!data_path_info_ptr->data_path.num_vertices && data_path_info_ptr->data_path.flags.src_module_closed)
      {
         /** Free up the allocated shared mem for delay pointer list */
         if (data_path_info_ptr->delay_shmem_list_ptr)
         {
            posal_memory_free(data_path_info_ptr->delay_shmem_list_ptr);
         }

         if (data_path_info_ptr->data_path.container_list.list_ptr)
         {
            /* As the data path is getting destroyed, empty container list for this particular data path.*/
            spf_list_delete_list(&data_path_info_ptr->data_path.container_list.list_ptr, TRUE /** Pool used*/);
         }
         data_path_info_ptr->data_path.container_list.num_nodes = 0;

         spf_list_delete_node_and_free_obj(&curr_path_node_ptr, &apm_info_ptr->graph_info.data_path_list_ptr, TRUE);

         /** Decrement the number of data path objects */
         apm_info_ptr->graph_info.num_data_paths--;

         AR_MSG(DBG_HIGH_PRIO,
                "apm_update_data_path_list(): Destroyed path id[%lu], cmd_opcode[0x%lX]",
                path_id,
                apm_info_ptr->curr_cmd_ctrl_ptr->cmd_opcode);
      }

      /** Advance to next data path in the list */
      curr_path_node_ptr = next_path_node_ptr;

   } /** End of while (data path list) */

   /** If all the data paths have been removed, clear the list
    *  pointer */
   if (!apm_info_ptr->graph_info.num_data_paths)
   {
      apm_info_ptr->graph_info.data_path_list_ptr = NULL;
   }

   return result;
}

static void apm_dfs_clear_visited_nodes(spf_list_node_t *visited_node_list_ptr)
{
   spf_list_node_t *       curr_node_ptr;
   apm_module_data_link_t *mod_out_data_link_ptr;

   /** Clear the "visted flag" for all the visted links */
   curr_node_ptr = visited_node_list_ptr;

   /** Iterate over the list of visited nodes */
   while (curr_node_ptr)
   {
      /** Get the connection object */
      mod_out_data_link_ptr = (apm_module_data_link_t *)curr_node_ptr->obj_ptr;

      /** Clear the node visted flag */
      mod_out_data_link_ptr->node_visted = FALSE;

      /** Delete this node and advance to curr pointer to next node
       *  in the list */
      spf_list_delete_node(&curr_node_ptr, TRUE);
   }

   return;
}

static ar_result_t apm_dfs_get_module_port(apm_graph_info_t *    graph_info_ptr,
                                           uint32_t              module_instance_id,
                                           uint32_t              port_id,
                                           uint32_t              port_dir,
                                           cntr_graph_vertex_t **graph_vertex_pptr)
{
   ar_result_t             result = AR_EFAILED;
   spf_list_node_t *       curr_node_ptr;
   apm_module_data_link_t *mod_data_link_ptr;
   apm_module_t *          module_node_ptr;

   /** Init the return pointer */
   *graph_vertex_pptr = NULL;

   /** Get the module node corresponding to current module iid */
   if (AR_EOK !=
       (result = apm_db_get_module_node(graph_info_ptr, module_instance_id, &module_node_ptr, APM_DB_OBJ_QUERY)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_dfs_get_module_port(): Failed to get module node: [M_IID]:[0x%lX]",
             module_instance_id);

      return result;
   }

   /** Check if the module node exists */
   if (!module_node_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "apm_dfs_get_module_port() Module node: [M_IID]:[0x%lX] not found", module_instance_id);

      return AR_EFAILED;
   }

   /** Get the pointer to the list of module data links as per
    *  port direction */
   if (AR_PORT_DIR_TYPE_INPUT == port_dir)
   {
      /** Get the pointer the input data-link list */
      curr_node_ptr = module_node_ptr->input_data_link_list_ptr;
   }
   else /** Output port */
   {
      /** Check if the destination module is a sink module based
       *  upon available connections at the time of this function
       *  invocatoin. It may not be a real sink but that is don't
       *  care */
      if (!module_node_ptr->num_output_data_link)
      {
         return AR_EOK;
      }

      /** Else, get the pointer the output data-link list */
      curr_node_ptr = module_node_ptr->output_data_link_list_ptr;
   }

   /** Iterate over the data links */
   while (curr_node_ptr)
   {
      /** Get the pointer to data link object */
      mod_data_link_ptr = (apm_module_data_link_t *)curr_node_ptr->obj_ptr;

      /** Break if the port ID match found */
      if ((AR_PORT_DIR_TYPE_INPUT == port_dir) && (mod_data_link_ptr->dstn_port.port_id == port_id))
      {
         *graph_vertex_pptr = &mod_data_link_ptr->dstn_port;
         result             = AR_EOK;
         break;
      }
      else if ((AR_PORT_DIR_TYPE_OUTPUT == port_dir) && (!port_id || (mod_data_link_ptr->src_port.port_id == port_id)))
      {
         /** For output port, if the port ID is not provided, return
          *  the first available port object. If the port ID is
          *  provided, need to find the matching port object and
          *  return */

         *graph_vertex_pptr = &mod_data_link_ptr->src_port;
         result             = AR_EOK;
         break;
      }

      /** Else, keep traversing */
      curr_node_ptr = curr_node_ptr->next_ptr;
   }

   return result;
}

static ar_result_t apm_dfs_validate_pre_process_path_dfn(apm_graph_info_t *    graph_info_ptr,
                                                         apm_data_path_info_t *path_info_ptr,
                                                         cntr_graph_vertex_t **graph_vertex_src_pptr,
                                                         cntr_graph_vertex_t **graph_vertex_dstn_pptr)
{
   ar_result_t          result = AR_EOK;
   spf_data_path_dfn_t *path_dfn_ptr;

   /** Populate return pointers */
   *graph_vertex_src_pptr  = NULL;
   *graph_vertex_dstn_pptr = NULL;

   /** Get the pointer to path definition being queried */
   path_dfn_ptr = &path_info_ptr->path_dfn;

   /** Validate the source and destination module instance ID's */
   if (!path_dfn_ptr->src_module_instance_id || !path_dfn_ptr->dst_module_instance_id)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_dfs_validate_pre_process_path_dfn(): Src module iid[0x%lX] and/or dst module iid[0x%lX] is/are NULL",
             path_dfn_ptr->src_module_instance_id,
             path_dfn_ptr->dst_module_instance_id);

      return AR_EBADPARAM;
   }

   /** Check if the source and destination module ID's are same */
   if (path_dfn_ptr->src_module_instance_id == path_dfn_ptr->dst_module_instance_id)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_dfs_validate_pre_process_path_dfn(): src module iid[0x%lX] and dst module iid[0x%lX] are same"
             "cmd_opcode[0x%08lx]",
             path_dfn_ptr->src_module_instance_id,
             path_dfn_ptr->dst_module_instance_id);

      return AR_EBADPARAM;
   }

   /** If source module port ID is provided and it is input
    *  port, cache it separately to be inserted in the DFS
    *  output later */
   if (path_dfn_ptr->src_port_id &&
       (AR_PORT_DIR_TYPE_INPUT ==
        spf_get_bits(path_dfn_ptr->src_port_id, AR_PORT_DIR_TYPE_MASK, AR_PORT_DIR_TYPE_SHIFT)))
   {
      if (AR_EOK != (result = apm_dfs_get_module_port(graph_info_ptr,
                                                      path_dfn_ptr->src_module_instance_id,
                                                      path_dfn_ptr->src_port_id,
                                                      AR_PORT_DIR_TYPE_INPUT,
                                                      graph_vertex_src_pptr)))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_dfs_validate_pre_process_path_dfn(): Failed to find port_id[0x%lx], src module iid[0x%lX]",
                path_dfn_ptr->src_module_instance_id,
                path_dfn_ptr->src_port_id);

         return result;
      }
   }

   /** If Either the destination module port ID is not provided,
    *  \n
    *  or if provided, and it's output port then cache the port
    *  object to be inserted in the DFS output later */
   if (!path_dfn_ptr->dst_port_id ||
       (path_dfn_ptr->dst_port_id &&
        (AR_PORT_DIR_TYPE_OUTPUT ==
         spf_get_bits(path_dfn_ptr->dst_port_id, AR_PORT_DIR_TYPE_MASK, AR_PORT_DIR_TYPE_SHIFT))))
   {
      if (AR_EOK != (result = apm_dfs_get_module_port(graph_info_ptr,
                                                      path_dfn_ptr->dst_module_instance_id,
                                                      path_dfn_ptr->dst_port_id,
                                                      AR_PORT_DIR_TYPE_OUTPUT,
                                                      graph_vertex_dstn_pptr)))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_dfs_validate_pre_process_path_dfn(): Failed to find port_id[0x%lx], dstn module iid[0x%lX]",
                path_dfn_ptr->dst_module_instance_id,
                path_dfn_ptr->dst_port_id);

         return result;
      }
   }

   return result;
}

#ifdef APM_DEBUG_DATA_PATH
static void apm_print_data_path(apm_data_path_info_t *path_info_ptr)
{
   spf_list_node_t *    curr_node_ptr;
   cntr_graph_vertex_t *graph_vertex_ptr;

   /** Get the pointer to the list of graph nodes. It's a stack
    *  so get the tail node pointer for the source module */
   if (AR_EOK != spf_list_get_tail_node(path_info_ptr->vertices_list_ptr, &curr_node_ptr))
   {
      return;
   }

   /** Iterate over the list of graph nodes and print values */
   while (curr_node_ptr)
   {
      /** Get the pointer to graph vertex object */
      graph_vertex_ptr = (cntr_graph_vertex_t *)curr_node_ptr->obj_ptr;

      AR_MSG(DBG_MED_PRIO,
             "apm_print_data_path(): Path ID[%lu], m_iid[0x%lX], port_id[0x%lX], num_vertices[%lu]",
             path_info_ptr->path_id,
             graph_vertex_ptr->module_instance_id,
             graph_vertex_ptr->port_id,
             path_info_ptr->num_vertices);

      curr_node_ptr = curr_node_ptr->prev_ptr;
   }

   return;
}
#endif

ar_result_t apm_data_path_dfs(apm_t *apm_info_ptr, apm_data_path_info_t *path_info_ptr)
{
   ar_result_t             result = AR_EOK;
   spf_list_node_t *       curr_out_conn_node_ptr, *curr_node_ptr;
   apm_module_t *          curr_module_node_ptr;
   uint32_t                curr_module_iid;
   bool_t                  path_found = FALSE;
   apm_graph_info_t *      graph_info_ptr;
   spf_list_node_t *       visited_node_list_ptr = NULL;
   spf_data_path_dfn_t *   path_dfn_ptr;
   bool_t                  unvisited_node_found         = FALSE;
   bool_t                  all_links_visited            = FALSE;
   cntr_graph_vertex_t *   graph_vertex_cached_src_ptr  = NULL;
   cntr_graph_vertex_t *   graph_vertex_cached_dstn_ptr = NULL;
   cntr_graph_vertex_t *   graph_vertex_ptr             = NULL;
   apm_module_data_link_t *mod_out_data_link_ptr;

   /** Depth First Search traversal rules. \n
    *
    *  -If the source port ID is not provided then always start
    *   from the first available output port of the source
    *   module.\n
    *
    *  -If the source port ID is provided and it's an output port
    *   then DFS search starts from that point. \n
    *
    *  -If the source port ID is provided and it's an input port,
    *   then port object is cached and inserted in the DFS output
    *   separately. DFS traversal continues assuming source output
    *   port is not provided \n
    *
    *  -If the Destination port ID is not provided then destination
    *   port object with first available output port is cached. If
    *   it is sink module, then nothing is cached and DFS goes upto
    *   module's input port only \n
    *
    *  -If the destination port is provided and it's an output
    *   port, then output port object is cached and inserted in the
    *   DFS output separately \n
    *
    *  -If the destination port is provided and it's an input port
    *   then DFS traversal continues until the matching port ID is
    *   found.
    *   */

   /** Get the APM graph info pointer */
   graph_info_ptr = &apm_info_ptr->graph_info;

   /** Get the path definition pointer */
   path_dfn_ptr = &path_info_ptr->path_dfn;

   /** Validate the data path definition. This function also
    *  caches the source and destination port objects which are
    *  not the part of DFS traversal as mentioned above */
   if (AR_EOK != (result = apm_dfs_validate_pre_process_path_dfn(graph_info_ptr,
                                                                 path_info_ptr,
                                                                 &graph_vertex_cached_src_ptr,
                                                                 &graph_vertex_cached_dstn_ptr)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_data_path_dfs(): Path dfn validation failed src: m_iid[0x%lX], port_id[0x%lX], dstn: m_iid[0x%lX], "
             "port_id[0x%lX]",
             path_dfn_ptr->src_module_instance_id,
             path_dfn_ptr->src_port_id,
             path_dfn_ptr->dst_module_instance_id,
             path_dfn_ptr->dst_port_id);

      return result;
   }

   /** Start the traversal from the source module. Init the current
    *  module IID being processed */
   curr_module_iid = path_dfn_ptr->src_module_instance_id;

   for (;;)
   {
      /** Get the module node corresponding to current module iid */
      if (AR_EOK !=
          (result = apm_db_get_module_node(graph_info_ptr, curr_module_iid, &curr_module_node_ptr, APM_DB_OBJ_QUERY)))
      {
         AR_MSG(DBG_HIGH_PRIO, "apm_data_path_dfs(): Failed to get module node: [M_IID]:[0x%lX]", curr_module_iid);

         goto __bail_out_apm_dfs;
      }

      /** Check if the module node exists */
      if (!curr_module_node_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO, "apm_data_path_dfs() Module node: [M_IID]:[0x%lX] not found", curr_module_iid);

         goto __bail_out_apm_dfs;
      }

      /** If the current module has output port connections, get
       *  the first un-visted connection and traverse further until
       *   the destination module IID is found  */
      if (curr_module_node_ptr->output_data_link_list_ptr && !all_links_visited)
      {
         /** Get the pointer to output port connections for the current
          *  module under process  */
         curr_out_conn_node_ptr = curr_module_node_ptr->output_data_link_list_ptr;

         /** Iterate over this output port connections */
         while (curr_out_conn_node_ptr)
         {
            /** Get the connection object */
            mod_out_data_link_ptr = (apm_module_data_link_t *)curr_out_conn_node_ptr->obj_ptr;

            /** If the current module is source, path definition port ID
             *  is valid, then need to keep iterating until the source
             *  module's output port ID match is found and source module
             *  port is not pre-identified */
            if ((curr_module_iid = path_dfn_ptr->src_module_instance_id) && (path_dfn_ptr->src_port_id) &&
                (mod_out_data_link_ptr->src_port.port_id != path_dfn_ptr->src_port_id) && !graph_vertex_cached_src_ptr)
            {
               /** Advance to next output port */
               curr_out_conn_node_ptr = curr_out_conn_node_ptr->next_ptr;

               /** Skip further processing for this iteration */
               continue;
            }

            /** Check if this connection has not been visited previously */
            if (!mod_out_data_link_ptr->node_visted)
            {
               /** Link as a whole is pushed to the top of stack,
                *  source goes first followed by destination */

               /** Push src node to the at the top of DFS stack */
               spf_list_insert_head(&path_info_ptr->vertices_list_ptr,
                                    &mod_out_data_link_ptr->src_port,
                                    APM_INTERNAL_STATIC_HEAP_ID,
                                    TRUE);

               /** Push dstn node to the at the top of DFS stack */
               spf_list_insert_head(&path_info_ptr->vertices_list_ptr,
                                    &mod_out_data_link_ptr->dstn_port,
                                    APM_INTERNAL_STATIC_HEAP_ID,
                                    TRUE);

               /** Increment the number of vertices */
               path_info_ptr->num_vertices += 2;

               /** Mark the node as visited */
               mod_out_data_link_ptr->node_visted = TRUE;

               AR_MSG(DBG_MED_PRIO,
                      "apm_data_path_dfs(): Pushed src: m_iid[0x%lX], port_id[0x%lX], dstn: m_iid[0x%lX], "
                      "port_id[0x%lX], num_vertices[%lu]",
                      mod_out_data_link_ptr->src_port.module_instance_id,
                      mod_out_data_link_ptr->src_port.port_id,
                      mod_out_data_link_ptr->dstn_port.module_instance_id,
                      mod_out_data_link_ptr->dstn_port.port_id,
                      path_info_ptr->num_vertices);

               /** Add this link to visted node list */
               spf_list_insert_tail(&visited_node_list_ptr, mod_out_data_link_ptr, APM_INTERNAL_STATIC_HEAP_ID, TRUE);

               /** Get the destination module instance ID */
               curr_module_iid = mod_out_data_link_ptr->dstn_port.module_instance_id;

               /** If the destination module instance ID matches, return the
                *  path */
               if (curr_module_iid == path_dfn_ptr->dst_module_instance_id)
               {
                  /** Match found */
                  path_found = TRUE;

                  /** If the destination port type is not pre-identified, and port
                   *  ID is provided, then for path to complete the port ID must
                   *  match with module's input port, else keep traversing  */
                  if (!graph_vertex_cached_dstn_ptr && path_dfn_ptr->dst_port_id &&
                      (mod_out_data_link_ptr->dstn_port.port_id != path_dfn_ptr->dst_port_id))
                  {
                     /** Not a Match */
                     path_found = FALSE;
                  }
               }

               /** Set this flag if the while() loop is exited because an
                *  unvisted node is found */
               unvisited_node_found = TRUE;

               /** If path not found, continue traversing the conection list
                *  depth wise. If found, also break from the outer for{;;}
                *  loop */
               break;
            }

            /** Advance to next data link */
            curr_out_conn_node_ptr = curr_out_conn_node_ptr->next_ptr;

         } /** End of while (out connection) */

         /** Check if the while loop ends without finding any
          *  un-visited node */
         if (!unvisited_node_found)
         {
            all_links_visited = TRUE;
         }

         /** Clear the unvisited node flag */
         unvisited_node_found = FALSE;

      } /** End of if */
      else
      {
         cntr_graph_vertex_t *graph_vertex_src_ptr;
         cntr_graph_vertex_t *graph_vertex_dstn_ptr;

         /** Clear the flag  */
         all_links_visited = FALSE;

         /** Check if the stack is non-empty */
         if (path_info_ptr->vertices_list_ptr)
         {
            /** Pop the dstn node from stack */
            if (NULL == (graph_vertex_dstn_ptr =
                            (cntr_graph_vertex_t *)spf_list_pop_head(&path_info_ptr->vertices_list_ptr, TRUE)))
            {
               AR_MSG(DBG_ERROR_PRIO, "apm_data_path_dfs(): Failed to get the vertex list head");

               goto __bail_out_apm_dfs;
            }

            /** Pop the src node from stack */
            if (NULL == (graph_vertex_src_ptr =
                            (cntr_graph_vertex_t *)spf_list_pop_head(&path_info_ptr->vertices_list_ptr, TRUE)))
            {
               AR_MSG(DBG_ERROR_PRIO, "apm_data_path_dfs(): Failed to get the vertex list head");

               goto __bail_out_apm_dfs;
            }

            /** Get the source instance ID and continue traversal */
            curr_module_iid = graph_vertex_src_ptr->module_instance_id;

            /** Deccrement the number of vertices */
            path_info_ptr->num_vertices -= 2;

#ifdef APM_DEBUG_DATA_PATH
            AR_MSG(DBG_MED_PRIO,
                   "apm_data_path_dfs(): Popped src: m_iid[0x%lx], port_id[0x%lX], dstn: m_iid[0x%lx], "
                   "port_id[0x%lX], num_vertices[%lu]",
                   graph_vertex_src_ptr->module_instance_id,
                   graph_vertex_src_ptr->port_id,
                   graph_vertex_dstn_ptr->module_instance_id,
                   graph_vertex_dstn_ptr->port_id,
                   path_info_ptr->num_vertices);
#endif
         }
         else /** If the stack is empty */
         {
            /** Execution reaches here is no match found. Return error */
            AR_MSG(DBG_HIGH_PRIO,
                   "apm_data_path_dfs():: WARNING:: No connected data path exists between "
                   "src_module_iid[0x%lX], src_mod_port_id[0x%lX]"
                   "dstn_module_iid[0x%lX], dstn_mod_port_id[0x%lX]",
                   path_dfn_ptr->src_module_instance_id,
                   path_dfn_ptr->src_port_id,
                   path_dfn_ptr->dst_module_instance_id,
                   path_dfn_ptr->dst_port_id);

            /** Invalidate the path ID and return */
            path_info_ptr->path_id = APM_DATA_PATH_ID_INVALID;

            /** Return success, further error handling is taken care by the
             *  caller based upon the context of calling the DFS. For
             *  client command or container event, this should result in
             *  error. However, if the data path is attempted to being
             *  reconfigured during GRAPH OPEN, the DFS search may fail, in
             *  which case it should not be flagged as error. The caller
             *  knows the operations is successful if valid non-zero path ID
             *  is assigned to the data path */

            result = AR_EOK;

            /** Abort processing */
            goto __bail_out_apm_dfs;
         }
      }

      /** If path found break from the outer for(;;) loop */
      if (path_found)
      {
         break;
      }

   } /** End of for(;;) */

   /** Update the source port ID if not provided, need to update
    *  the identified port in the path definition */
   if (!path_dfn_ptr->src_port_id)
   {
      /** Get the pointer to source module, using tail node as the
       *  graph is stored as stack */
      if (AR_EOK != (result = spf_list_get_tail_node(path_info_ptr->vertices_list_ptr, &curr_node_ptr)))
      {
         goto __bail_out_apm_dfs;
      }

      /** Get the pointer to data link object */
      graph_vertex_ptr = (cntr_graph_vertex_t *)curr_node_ptr->obj_ptr;

      /** Populate the source port ID in the path definition  */
      path_info_ptr->path_dfn.src_port_id = graph_vertex_ptr->port_id;
   }
   else if (graph_vertex_cached_src_ptr) /** Port ID for source module is provided and it's an input port */
   {
      /** Insert the pre-identified port object in the DFS output */
      spf_list_insert_tail(&path_info_ptr->vertices_list_ptr, graph_vertex_cached_src_ptr, APM_INTERNAL_STATIC_HEAP_ID, TRUE);

      /** Increment the number of vertices */
      path_info_ptr->num_vertices++;
   }

   /** Update the destination port ID if not provided, destination
    *  is not a sink module,\n
    *  or if provided but it's an output port, then insert the
    *  pre-identified port in the DFS output */
   if (graph_vertex_cached_dstn_ptr)
   {
      /** Push this node to the at the top of DFS stack */
      spf_list_insert_head(&path_info_ptr->vertices_list_ptr, graph_vertex_cached_dstn_ptr, APM_INTERNAL_STATIC_HEAP_ID, TRUE);

      /** Increment the number of vertices */
      path_info_ptr->num_vertices++;

      /** Populate the destination port ID in the path definition  */
      path_info_ptr->path_dfn.dst_port_id = graph_vertex_cached_dstn_ptr->port_id;
   }
   else
   {
      /** Either dstn module port ID is provided and it's and input
       *  port or \n
       *  dstn port ID is not provided and it's a sink module */

      /** Head node in the list points to the destination link. */
      graph_vertex_ptr = (cntr_graph_vertex_t *)path_info_ptr->vertices_list_ptr->obj_ptr;

      /** Populate the destination port ID in the path definition  */
      path_info_ptr->path_dfn.dst_port_id = graph_vertex_ptr->port_id;
   }

   /** Clear the "visted flag" for all the visted data links */
   apm_dfs_clear_visited_nodes(visited_node_list_ptr);

   /** Assign the unique ID for this data path */
   path_info_ptr->path_id = apm_get_next_uid(apm_info_ptr);

   /** Set the validity flag for this path */
   path_info_ptr->flags.path_valid = TRUE;

#ifdef APM_DEBUG_DATA_PATH
   apm_print_data_path(path_info_ptr);
#endif

   AR_MSG(DBG_HIGH_PRIO,
          "apm_data_path_dfs(): Assigned path ID: 0x%lx, for data path : src_module_iid[0x%lX], src_mod_port_id[0x%lX] "
          "dstn_module_iid[0x%lX], dstn_mod_port_id[0x%lX]",
          path_info_ptr->path_id,
          path_info_ptr->path_dfn.src_module_instance_id,
          path_info_ptr->path_dfn.src_port_id,
          path_info_ptr->path_dfn.dst_module_instance_id,
          path_info_ptr->path_dfn.dst_port_id);

   return result;

__bail_out_apm_dfs:

   /** Clear the visited nodes */
   apm_dfs_clear_visited_nodes(visited_node_list_ptr);

   /** Clear the DFS output stack */
   spf_list_delete_list(&path_info_ptr->vertices_list_ptr, TRUE);

   return result;
}

/** Check if the cached config struct have been allocated for
 *  the current path ID */
ar_result_t apm_get_cont_cached_path_delay_cfg_obj(apm_cont_cmd_ctrl_t *       cont_cmd_ctrl_ptr,
                                                   apm_data_path_info_t *      path_info_ptr,
                                                   apm_cont_path_delay_cfg_t **path_delay_cfg_pptr,
                                                   bool_t                      data_path_create_op,
                                                   bool_t                      create_new_obj,
                                                   bool_t *                    data_path_obj_created_ptr)
{
   ar_result_t                 result = AR_EOK;
   apm_cont_path_delay_cfg_t * cached_path_delay_cfg_ptr;
   spf_list_node_t *           curr_node_ptr;
   apm_cont_set_get_cfg_hdr_t *param_data_hdr_ptr;
   uint32_t                    cont_param_id;
   uint32_t                    path_id;
   apm_container_t *           host_container_ptr;

   /** Init return pointer */
   *path_delay_cfg_pptr = NULL;

   /** Init return pointer   */
   *data_path_obj_created_ptr = FALSE;

   /** Get the pointer to list of cached container specific cfg
    *  parameters */
   curr_node_ptr = cont_cmd_ctrl_ptr->cached_cfg_params.cont_cfg_params.param_data_list_ptr;

   /** Get the container param ID based upon if data path is
    *  being created or destroyed */
   if (data_path_create_op)
   {
      cont_param_id = CNTR_PARAM_ID_PATH_DELAY_CFG;
   }
   else
   {
      cont_param_id = CNTR_PARAM_ID_PATH_DESTROY;
   }

   /** Get the path ID */
   path_id = path_info_ptr->path_id;

   /** Get the host container pointer */
   host_container_ptr = (apm_container_t *)cont_cmd_ctrl_ptr->host_container_ptr;

   /** For the given path ID, check if the cached data structure
    *  is already allocated  */

   /** Iterate over the list of data path config list */
   while (curr_node_ptr)
   {
      /** Get the param data header */
      param_data_hdr_ptr = (apm_cont_set_get_cfg_hdr_t *)curr_node_ptr->obj_ptr;

      /** If the param ID matches, check for the matching path ID */
      if (cont_param_id == param_data_hdr_ptr->param_id)
      {
         cached_path_delay_cfg_ptr = (apm_cont_path_delay_cfg_t *)curr_node_ptr->obj_ptr;

         /** Check the message opcode and path ID are matching with
          *  input, then return this object */
         if (cached_path_delay_cfg_ptr->data_path.path_id == path_id)
         {
            /** Populate the output pointer */
            *path_delay_cfg_pptr = cached_path_delay_cfg_ptr;

            return result;
         }
      }

      /** Advance to next node in the list */
      curr_node_ptr = curr_node_ptr->next_ptr;

   } /** End of while(cached delay cfg node list) */

   /** If execution falls through, check if new object creation
    *  is required, if not return */
   if (!create_new_obj)
   {
      AR_MSG(DBG_MED_PRIO,
             "apm_cache_container_data_path_info(): Data path node not found, path_id[%lu], CONT_ID[0x%lX]",
             path_id,
             host_container_ptr->container_id);

      return AR_EOK;
   }

   /** Execution falls through if the obj has not be allocated
    *  for given path ID, allocate one now */

   if (NULL ==
       (cached_path_delay_cfg_ptr =
           (apm_cont_path_delay_cfg_t *)posal_memory_malloc(sizeof(apm_cont_path_delay_cfg_t), APM_INTERNAL_STATIC_HEAP_ID)))
   {
      AR_MSG(DBG_ERROR_PRIO, "apm_cache_container_data_path_info(): Failed to allocate the path delay node");

      return AR_EBADPARAM;
   }

   /** Clear the allocated struct */
   memset(cached_path_delay_cfg_ptr, 0, sizeof(apm_cont_path_delay_cfg_t));

   /** Init the path ID */
   cached_path_delay_cfg_ptr->data_path.path_id = path_id;

   /** Init the one time query flag */
   cached_path_delay_cfg_ptr->data_path.flags.one_time_query = path_info_ptr->flags.one_time_query;

   /** Populate the param ID associated with this cached config */

   cached_path_delay_cfg_ptr->header.param_id = cont_param_id;

   /** Cache this node to be sent to containers */
   if (AR_EOK !=
       (result = apm_db_add_node_to_list(&cont_cmd_ctrl_ptr->cached_cfg_params.cont_cfg_params.param_data_list_ptr,
                                         cached_path_delay_cfg_ptr,
                                         &cont_cmd_ctrl_ptr->cached_cfg_params.cont_cfg_params.num_cfg_params)))
   {
      AR_MSG(DBG_ERROR_PRIO, "apm_cache_container_data_path_info(): Failed to cache the allocated path delay node");

      /** Free up the allocated object */
      posal_memory_free(cached_path_delay_cfg_ptr);
   }

#ifdef APM_DEBUG_DATA_PATH
   AR_MSG(DBG_HIGH_PRIO,
          "apm_cache_container_data_path_info(): Cached param_id[0x%lX], CONT_ID[0x%lX]",
          cont_param_id,
          host_container_ptr->container_id);
#endif

   /** Set the flag to indicate that new data path object has
    *  been created */
   *data_path_obj_created_ptr = TRUE;

   /** Update the return pointer */
   *path_delay_cfg_pptr = cached_path_delay_cfg_ptr;

   return result;
}

ar_result_t apm_cache_container_data_path_info(apm_t *               apm_info_ptr,
                                               apm_data_path_info_t *path_info_ptr,
                                               bool_t                data_path_create)
{
   ar_result_t                result = AR_EOK;
   spf_list_node_t *          curr_node_ptr;
   cntr_graph_vertex_t *      graph_vertex_ptr;
   apm_module_t *             module_node_ptr           = NULL;
   apm_container_t *          curr_host_container_ptr   = NULL;
   apm_cont_cmd_ctrl_t *      cont_cmd_ctrl_ptr         = NULL;
   apm_cont_path_delay_cfg_t *cached_path_delay_cfg_ptr = NULL;
   bool_t                     CREATE_NEW_DATA_PATH_OBJ  = TRUE;
   bool_t                     data_path_obj_created     = FALSE;

   /** Get the tail node for the source module as the data path
    *  is on stack */
   spf_list_get_tail_node(path_info_ptr->vertices_list_ptr, &curr_node_ptr);

   /** Iterate the list from tail to head */
   while (curr_node_ptr)
   {
      graph_vertex_ptr = (cntr_graph_vertex_t *)curr_node_ptr->obj_ptr;

      /** Clear the flag to indicate if data path object is being
       *  first time allocated */
      data_path_obj_created = FALSE;

      /** Get the source module node in the data link  */
      if (AR_EOK != (result = apm_db_get_module_node(&apm_info_ptr->graph_info,
                                                     graph_vertex_ptr->module_instance_id,
                                                     &module_node_ptr,
                                                     APM_DB_OBJ_QUERY)))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_cache_container_data_path_info(): Failed to get cont module_iid[0x%lX]",
                graph_vertex_ptr->module_instance_id);

         return AR_EFAILED;
      }

      if (!module_node_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_cache_container_data_path_info(): module_iid[0x%lX] does not exist",
                graph_vertex_ptr->module_instance_id);

         return AR_EFAILED;
      }

      /** Get the current module's host container command control
       *  handle */
      if (curr_host_container_ptr != module_node_ptr->host_cont_ptr)
      {
         /** Update the current host container pointer */
         curr_host_container_ptr = module_node_ptr->host_cont_ptr;

         /** Get the pointer to current container command control
          *  pointer */
         apm_get_cont_cmd_ctrl_obj(curr_host_container_ptr, apm_info_ptr->curr_cmd_ctrl_ptr, &cont_cmd_ctrl_ptr);

         if (!cont_cmd_ctrl_ptr)
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "apm_cache_container_data_path_info(): Failed to get cont cmd ctrl obj, CONT_ID[0x%lX]",
                   curr_host_container_ptr->container_id);

            return AR_EFAILED;
         }

         /** Check if the cached config struct have been allocated for
          *  the current path ID */
         if (AR_EOK != (result = apm_get_cont_cached_path_delay_cfg_obj(cont_cmd_ctrl_ptr,
                                                                        path_info_ptr,
                                                                        &cached_path_delay_cfg_ptr,
                                                                        data_path_create,
                                                                        CREATE_NEW_DATA_PATH_OBJ,
                                                                        &data_path_obj_created)))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "apm_cache_container_data_path_info(): Failed to get path delay cached object, CONT_ID[0x%lX]",
                   curr_host_container_ptr->container_id);

            return result;
         }

         /** If data path is being created, and container's data path obj
          *  for keeping is first time allocated, then increment the
          *  number of participating containers */
         if (data_path_create && data_path_obj_created)
         {
            /** Increment the number of participating containers */
            path_info_ptr->num_containers++;

            /*Add the container in the data path list, if it is not an one time query.*/
            if (!path_info_ptr->flags.one_time_query)
            {
               if (AR_EOK != (result = apm_db_search_and_add_node_to_list(&path_info_ptr->container_list.list_ptr,
                                                                          module_node_ptr->host_cont_ptr,
                                                                          &path_info_ptr->container_list.num_nodes)))
               {
                  AR_MSG(DBG_ERROR_PRIO,
                         "apm_cache_container_data_path_info(): Failed to add container in data path container list, "
                         "CONT_ID[0x%lX]",
                         module_node_ptr->host_cont_ptr->container_id);
                  return result;
               }
               else
               {
                  AR_MSG(DBG_MED_PRIO,
                         "apm_cache_container_data_path_info(): Added container in data path container list, "
                         "CONT_ID[0x%lX], path_id[%lu] num_node %d",
                         module_node_ptr->host_cont_ptr->container_id,
                         path_info_ptr->path_id,
                         path_info_ptr->container_list.num_nodes);
               }
            }
         }
      }

      /** If path is being created, cache the data path graph
       *  vertex */
      if (data_path_create && cached_path_delay_cfg_ptr)
      {
         /** Cached the path delay config structure to the runnning
          *  list to be sent to containers */
         if (AR_EOK != (result = apm_db_add_node_to_list(&cached_path_delay_cfg_ptr->data_path.vertices_list_ptr,
                                                         graph_vertex_ptr,
                                                         &cached_path_delay_cfg_ptr->data_path.num_vertices)))
         {
            return result;
         }

#ifdef APM_DEBUG_DATA_PATH

         AR_MSG(DBG_HIGH_PRIO,
                "apm_cache_container_data_path_info(): path_id[%lu], CONT_ID[0x%lX], "
                "module_iid[0x%lX], port_id[0x%lX], total containers[%lu]",
                path_info_ptr->path_id,
                curr_host_container_ptr->container_id,
                graph_vertex_ptr->module_instance_id,
                graph_vertex_ptr->port_id,
                path_info_ptr->num_containers);
#endif
      }

      if (cont_cmd_ctrl_ptr)
      {
         /** Add this container to pending message send list */
         apm_add_cont_to_pending_msg_send_list(apm_info_ptr->curr_cmd_ctrl_ptr,
                                               curr_host_container_ptr,
                                               cont_cmd_ctrl_ptr);
      }

      /** Continue backward traversal */
      curr_node_ptr = curr_node_ptr->prev_ptr;
   }

   return result;
}

ar_result_t apm_cache_and_destroy_container_data_path_info(apm_t *apm_info_ptr, apm_data_path_info_t *path_info_ptr)
{
   ar_result_t                result = AR_EOK;
   spf_list_node_t *          curr_node_ptr;
   apm_container_t *          curr_container_ptr   = NULL;
   apm_cont_cmd_ctrl_t *      cont_cmd_ctrl_ptr         = NULL;
   apm_cont_path_delay_cfg_t *cached_path_delay_cfg_ptr = NULL;
   bool_t                     CREATE_NEW_DATA_PATH_OBJ  = TRUE;
   bool_t                     data_path_obj_created     = FALSE;

   curr_node_ptr = path_info_ptr->container_list.list_ptr;

   /** Iterate the container list */
   while (curr_node_ptr)
   {
      curr_container_ptr = (apm_container_t *)curr_node_ptr->obj_ptr;

      /** Clear the flag to indicate if data path object is being
       *  first time allocated */
      data_path_obj_created = FALSE;

      /** Get the pointer to current container command control
       *  pointer */
      apm_get_cont_cmd_ctrl_obj(curr_container_ptr, apm_info_ptr->curr_cmd_ctrl_ptr, &cont_cmd_ctrl_ptr);

      if (!cont_cmd_ctrl_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_cache_and_destroy_container_data_path_info(): Failed to get cont cmd ctrl obj, CONT_ID[0x%lX]",
                curr_container_ptr->container_id);

         return AR_EFAILED;
      }

      /** Check if the cached config struct have been allocated for
       *  the current path ID */
      if (AR_EOK != (result = apm_get_cont_cached_path_delay_cfg_obj(cont_cmd_ctrl_ptr,
                                                                     path_info_ptr,
                                                                     &cached_path_delay_cfg_ptr,
                                                                     FALSE /* Destroy the data path*/,
                                                                     CREATE_NEW_DATA_PATH_OBJ,
                                                                     &data_path_obj_created)))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_cache_and_destroy_container_data_path_info(): Failed to get path delay cached object, "
                "CONT_ID[0x%lX]",
                curr_container_ptr->container_id);

         return result;
      }

#ifdef APM_DEBUG_DATA_PATH
      AR_MSG(DBG_HIGH_PRIO,
             "apm_cache_and_destroy_container_data_path_info(): sending destroy data path path_id[%lu], CONT_ID[0x%lX]",
             path_info_ptr->path_id,
             curr_container_ptr->container_id);
#endif

      if (cont_cmd_ctrl_ptr)
      {
         /** Add this container to pending message send list */
         apm_add_cont_to_pending_msg_send_list(apm_info_ptr->curr_cmd_ctrl_ptr,
                                               curr_container_ptr,
                                               cont_cmd_ctrl_ptr);
      }

      /** Continue traversal */
      curr_node_ptr = curr_node_ptr->next_ptr;
   }

   if (path_info_ptr->container_list.list_ptr)
   {
      /* As the data path is getting destroyed, empty container list for this particular data path.*/
      spf_list_delete_list(&path_info_ptr->container_list.list_ptr, TRUE /** Pool used*/);
   }
   path_info_ptr->container_list.num_nodes = 0;

   return result;
}

ar_result_t apm_allocate_delay_list_shmem(apm_t *apm_info_ptr, apm_data_path_delay_info_t *path_delay_info_ptr)
{
   ar_result_t                result      = AR_EOK;
   uint32_t                   malloc_size = 0;
   spf_list_node_t *          curr_node_ptr;
   apm_path_delay_shmem_t *   shmem_list_ptr                 = NULL;
   apm_cont_path_delay_cfg_t *cont_cached_path_delay_cfg_ptr = NULL;
   apm_container_t *          container_node_ptr;
   apm_cont_cmd_ctrl_t *      cont_cmd_ctrl_ptr;
   uint32_t                   arr_idx                   = 0;
   bool_t                     DATA_PATH_CREATE_OP       = TRUE;
   bool_t                     DONT_CREATE_DATA_PATH_OBJ = FALSE;
   bool_t                     data_path_obj_created     = FALSE;

   /** Compute the memory to be allocated */
   malloc_size = path_delay_info_ptr->data_path.num_containers * sizeof(apm_path_delay_shmem_t);

   /** Allocate memory for accumulated drift */
   if (NULL == (shmem_list_ptr = (apm_path_delay_shmem_t *)posal_memory_malloc(malloc_size, APM_INTERNAL_STATIC_HEAP_ID)))
   {
      AR_MSG(DBG_ERROR_PRIO, "apm_path_delay_event_hdlr: Failed to allocated shmem for delay storage");

      goto __bailout_alloc_dl_shmem;
   }

   /** Clear the allocated struct */
   memset(shmem_list_ptr, 0, malloc_size);

   /** Store the allocated pointer */
   path_delay_info_ptr->delay_shmem_list_ptr = shmem_list_ptr;

   /** Get the pointer to the list of container pending send
    *  message */
   curr_node_ptr = apm_info_ptr->curr_cmd_ctrl_ptr->rsp_ctrl.pending_cont_list_ptr;

   arr_idx = 0;

   /** Iterate over the list of pending containers */
   while (curr_node_ptr)
   {
      /** Get the container object */
      container_node_ptr = (apm_container_t *)curr_node_ptr->obj_ptr;

      /** Initalize the delay memory with container ID */
      shmem_list_ptr[arr_idx].container_id = container_node_ptr->container_id;

      /** Get the pointer to current container command control
       *  pointer */
      apm_get_cont_cmd_ctrl_obj(container_node_ptr, apm_info_ptr->curr_cmd_ctrl_ptr, &cont_cmd_ctrl_ptr);

      /** Get the cached config struct have been allocated for
       *  the current path ID */
      if (AR_EOK != (result = apm_get_cont_cached_path_delay_cfg_obj(cont_cmd_ctrl_ptr,
                                                                     &path_delay_info_ptr->data_path,
                                                                     &cont_cached_path_delay_cfg_ptr,
                                                                     DATA_PATH_CREATE_OP,
                                                                     DONT_CREATE_DATA_PATH_OBJ,
                                                                     &data_path_obj_created /** Don't care */)))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_path_delay_event_hdlr(): Failed to get path delay cached object, CONT_ID[0x%lX]",
                container_node_ptr->container_id);

         goto __bailout_alloc_dl_shmem;
      }

      /** If data path exists, populate the delay pointer */
      if (cont_cached_path_delay_cfg_ptr)
      {
         /** Cached the delay pointer */
         cont_cached_path_delay_cfg_ptr->delay_ptr = &shmem_list_ptr[arr_idx++].delay_us;

#ifdef APM_DEBUG_DATA_PATH

         AR_MSG(DBG_HIGH_PRIO,
                "apm_allocate_delay_list_shmem(): path_id[%lu], CONT_ID[0x%lX],"
                "delay_ptr[0x%lX]",
                path_delay_info_ptr->data_path.path_id,
                container_node_ptr->container_id,
                cont_cached_path_delay_cfg_ptr->delay_ptr);
#endif
      }

      /** Advance to next node in the list */
      curr_node_ptr = curr_node_ptr->next_ptr;

   } /** End of while (pending container list) */

   return result;

__bailout_alloc_dl_shmem:

   if (shmem_list_ptr)
   {
      posal_memory_free(shmem_list_ptr);
   }

   return result;
}

ar_result_t apm_configure_delay_data_path(apm_t *                      apm_info_ptr,
                                          spf_data_path_dfn_t *        path_dfn_ptr,
                                          apm_data_path_delay_info_t **path_delay_info_pptr,
                                          bool_t                       is_client_query,
                                          bool_t                       is_reconfig)
{
   ar_result_t                 result              = AR_EOK;
   apm_data_path_delay_info_t *path_delay_info_ptr = NULL;

   /** If first time configuration, allocate the memory for delay
    *  path object */
   if (!is_reconfig)
   {
      /** Clear the return pointer */
      *path_delay_info_pptr = NULL;

      /** Allocate memory for storing the data path info from the APM
       *  internal client */
      if (NULL ==
          (path_delay_info_ptr = (apm_data_path_delay_info_t *)posal_memory_malloc(sizeof(apm_data_path_delay_info_t),
                                                                                   APM_INTERNAL_STATIC_HEAP_ID)))
      {
         AR_MSG(DBG_ERROR_PRIO, "apm_configure_delay_data_path(): Failed to memory for caching the delay info");

         return AR_ENOMEMORY;
      }

      /** Clear the allocated struct */
      memset(path_delay_info_ptr, 0, sizeof(apm_data_path_delay_info_t));

      /** Cache the path definition */
      path_delay_info_ptr->data_path.path_dfn = *path_dfn_ptr;

      /** For the event from container, the data path info needs to
       *  be maintained until the data path exists  */
      path_delay_info_ptr->data_path.flags.one_time_query = is_client_query;
   }
   else /** Data path reconfiguration, delay path object is already allocated */
   {
      if (!path_delay_info_pptr)
      {
         AR_MSG(DBG_ERROR_PRIO, "apm_configure_delay_data_path(): Path delay info ptr is NULL");

         return AR_EFAILED;
      }

      path_delay_info_ptr = *path_delay_info_pptr;
   }

   /** Perform DFS */
   if (AR_EOK != (result = apm_data_path_dfs(apm_info_ptr, &path_delay_info_ptr->data_path)))
   {
      AR_MSG(DBG_ERROR_PRIO, "apm_configure_delay_data_path(): Failed to perform DFS search, result: 0x%lX", result);

      goto __bailout_data_path_dfn;
   }

   /** If the path ID returned is invalid, then error reporting
    *  dpeends upon the caller context of first time vs
    *  re-configuratoin of the delay data path */
   if (APM_DATA_PATH_ID_INVALID == path_delay_info_ptr->data_path.path_id)
   {
      /** If first time configuration, path ID must be valid, else it
       *  is flagged as error */
      if (!is_reconfig)
      {
         result = AR_EFAILED;
      }
      else /** Reconfiguration */
      {
         /** For reconfiguration, the path ID could be invalid as the
          *  data path might not be complete when reconfig is
          *  attempted */
         result = AR_EOK;
      }

      goto __bailout_data_path_dfn;
   }

   /** Cache per container info */
   if (AR_EOK != (result = apm_cache_container_data_path_info(apm_info_ptr,
                                                              &path_delay_info_ptr->data_path,
                                                              TRUE /** Data path create*/)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_configure_delay_data_path(): Failed to cache container params, result: 0x%lX",
             result);

      goto __bailout_data_path_dfn;
   }

   if (AR_EOK != (result = apm_allocate_delay_list_shmem(apm_info_ptr, path_delay_info_ptr)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_configure_delay_data_path(): Failed to allocate delay list shmem, result: 0x%lX",
             result);

      goto __bailout_data_path_dfn;
   }

   if (!is_reconfig)
   {
      /** Update the return pointer */
      *path_delay_info_pptr = path_delay_info_ptr;
   }

   return result;

__bailout_data_path_dfn:

   /** Free up the allocated node, if any failure occured during
    *  first time path configuration */
   if (!is_reconfig && path_delay_info_ptr)
   {
      posal_memory_free(path_delay_info_ptr);
   }

   return result;
}

ar_result_t apm_process_get_cfg_path_delay_local(apm_t *apm_info_ptr, apm_module_param_data_t *get_cfg_rsp_payload_ptr)
{
   ar_result_t                 result = AR_EOK;
   uint32_t                    overall_payload_size;
   uint32_t                    num_data_paths;
   apm_param_id_path_delay_t * pid_cmd_payload_ptr;
   apm_path_defn_for_delay_t * curr_path_dfn_cmd_ptr;
   apm_module_t *              module_node_ptr_list[2]   = { NULL };
   bool_t                      DANGLING_LINK_NOT_ALLOWED = FALSE;
   gpr_packet_t *              gpr_pkt_ptr;
   apm_param_id_path_delay_t * path_delay_pid_rsp_ptr;
   apm_path_defn_for_delay_t * curr_path_dfn_rsp_ptr = NULL;
   apm_module_param_data_t *   module_param_data_ptr;
   spf_data_path_dfn_t         data_path_dfn;
   apm_data_path_delay_info_t *path_delay_info_ptr = NULL;
   bool_t                      ONE_TIME_QUERY      = TRUE;
   bool_t                      FIRST_TIME_CFG      = FALSE;

   /** Validate the input arguments. This is a special type of
    *  get param payload which is the combination of input and
    *  output arguments. In this case, input argument is the
    *  path information pre-filled by the client and output
    *  arugment is the delay pointer */

   /** Get the pointer to GPR command packet */
   gpr_pkt_ptr = (gpr_packet_t *)apm_info_ptr->curr_cmd_ctrl_ptr->cmd_msg.payload_ptr;

   /** Get the pointer to command payload pointer */
   module_param_data_ptr =
      (apm_module_param_data_t *)(GPR_PKT_GET_PAYLOAD(uint8_t, gpr_pkt_ptr) + sizeof(apm_cmd_header_t));

   /** Validate the min expected size of the payload */
   if (module_param_data_ptr->param_size < sizeof(apm_param_id_path_delay_t))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "APM_PARAM_ID_PATH_DELAY: Insufficient payload size[%lu]",
             module_param_data_ptr->param_size);

      return AR_EBADPARAM;
   }

   /** Get the pointer to the path delay query pointer */
   pid_cmd_payload_ptr = (apm_param_id_path_delay_t *)(module_param_data_ptr + 1);

   /** Get the number of data paths queried */
   num_data_paths = pid_cmd_payload_ptr->num_paths;

   /** If the number of paths queried is 0, print a warning and
    *  return  */
   if (!num_data_paths)
   {
      AR_MSG(DBG_HIGH_PRIO, "APM_PARAM_ID_PATH_DELAY ::WARNING:: Num data path queried is 0");

      /** Returning EOK as it is not a critical error */
      return AR_EOK;
   }

   /** Validate overall payload size  including all the data
    *  paths */
   overall_payload_size = sizeof(apm_param_id_path_delay_t) + (num_data_paths * sizeof(apm_path_defn_for_delay_t));

   /** Validate the overall payload size */
   if (module_param_data_ptr->param_size < overall_payload_size)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "APM_PARAM_ID_PATH_DELAY: Insufficient payload size[%lu], expected size[%lu]",
             module_param_data_ptr->param_size,
             overall_payload_size);

      return AR_EBADPARAM;
   }

   /** Get the pointer to the start of param ID payload */
   path_delay_pid_rsp_ptr = (apm_param_id_path_delay_t *)(get_cfg_rsp_payload_ptr + 1);

   /** Update the number of data paths in response payload */
   path_delay_pid_rsp_ptr->num_paths = num_data_paths;

   /** Get the pointer to start of the path definition object
    *  array in response payload   */
   curr_path_dfn_rsp_ptr = (apm_path_defn_for_delay_t *)(path_delay_pid_rsp_ptr + 1);

   /** Get the pointer to start of the path definition object
    *  array in cmd payload   */
   curr_path_dfn_cmd_ptr = (apm_path_defn_for_delay_t *)(pid_cmd_payload_ptr + 1);

   /** Iterate over the list of data paths being queried */
   while (num_data_paths)
   {
      if (AR_EOK != (result = apm_validate_module_instance_pair(&apm_info_ptr->graph_info,
                                                                curr_path_dfn_cmd_ptr->src_module_instance_id,
                                                                curr_path_dfn_cmd_ptr->dst_module_instance_id,
                                                                module_node_ptr_list,
                                                                DANGLING_LINK_NOT_ALLOWED)))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "APM_PARAM_ID_PATH_DELAY: Failed to validate path dfn, src: MIID[0x%lX] and/or dstn module: "
                "MIID[0x%lX] does not exist",
                curr_path_dfn_cmd_ptr->src_module_instance_id,
                curr_path_dfn_cmd_ptr->dst_module_instance_id);

         return result;
      }

      AR_MSG(DBG_HIGH_PRIO,
             "APM_PARAM_ID_PATH_DELAY: Rcvd path dfn src: m_iid[0x%lx], port_id[0x%lX], dstn: m_iid[0x%lx], "
             "port_id[0x%lX]",
             curr_path_dfn_cmd_ptr->src_module_instance_id,
             curr_path_dfn_cmd_ptr->src_port_id,
             curr_path_dfn_cmd_ptr->dst_module_instance_id,
             curr_path_dfn_cmd_ptr->dst_port_id);

      /** Populate the data path info  */
      data_path_dfn.src_module_instance_id = curr_path_dfn_cmd_ptr->src_module_instance_id;
      data_path_dfn.src_port_id            = curr_path_dfn_cmd_ptr->src_port_id;
      data_path_dfn.dst_module_instance_id = curr_path_dfn_cmd_ptr->dst_module_instance_id;
      data_path_dfn.dst_port_id            = curr_path_dfn_cmd_ptr->dst_port_id;

      /** Configure delay data path as per the data definition
       *  received */
      if (AR_EOK != (result = apm_configure_delay_data_path(apm_info_ptr,
                                                            &data_path_dfn,
                                                            &path_delay_info_ptr,
                                                            ONE_TIME_QUERY,
                                                            FIRST_TIME_CFG)))
      {
         AR_MSG(DBG_ERROR_PRIO, "APM_PARAM_ID_PATH_DELAY: Failed to configure data path");

         result = AR_EFAILED;

         return result;
      }

      /** Add this node to APM global data path list */
      if (AR_EOK != (result = apm_db_add_node_to_list(&apm_info_ptr->graph_info.data_path_list_ptr,
                                                      path_delay_info_ptr,
                                                      &apm_info_ptr->graph_info.num_data_paths)))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "APM_PARAM_ID_PATH_DELAY : Failed to add event info node to the data path list, result: 0x%lX",
                result);

         return result;
      }

      /** Connection information is cached within each container.
       *  DFS traversal output list can be freed up */
      spf_list_delete_list(&path_delay_info_ptr->data_path.vertices_list_ptr, TRUE);

      /** Clear the number of vertices */
      path_delay_info_ptr->data_path.num_vertices = 0;

      /** Update the port ID's in the command payload */
      curr_path_dfn_cmd_ptr->src_port_id = path_delay_info_ptr->data_path.path_dfn.src_port_id;
      curr_path_dfn_cmd_ptr->dst_port_id = path_delay_info_ptr->data_path.path_dfn.dst_port_id;

      /** Copy the contents from cmd payload to response payload */
      *curr_path_dfn_rsp_ptr = *curr_path_dfn_cmd_ptr;

      /** Clear the accumulated delay in response payload */
      curr_path_dfn_rsp_ptr->delay_us = 0;

      /** Increment the current path definition cmd payload pointer */
      curr_path_dfn_cmd_ptr++;

      /** Increment the current path definition rsp payload pointer */
      curr_path_dfn_rsp_ptr++;

      /** Decrement number of data paths */
      num_data_paths--;

   } /** End of while (num_data_paths) */

   return result;
}

ar_result_t apm_graph_open_cmd_cache_data_link(apm_module_conn_cfg_t *data_link_cfg_ptr,
                                               apm_module_t **        module_node_pptr)
{
   ar_result_t             result        = AR_EOK;
   apm_module_data_link_t *data_link_ptr = NULL;

   enum
   {
      SRC_MODULE  = 0,
      DSTN_MODULE = 1
   };

   /** Allocate memory for storing this data connection */
   if (NULL == (data_link_ptr =
                   (apm_module_data_link_t *)posal_memory_malloc(sizeof(apm_module_data_link_t), APM_INTERNAL_STATIC_HEAP_ID)))
   {
      AR_MSG(DBG_ERROR_PRIO, "apm_graph_open_cmd_cache_data_link(): Failed to allocate memory for data link");

      return AR_ENOMEMORY;
   }

   /** Clear the allocated memory */
   memset(data_link_ptr, 0, sizeof(apm_module_data_link_t));

   /** Copy the data link content */
   data_link_ptr->src_port.module_instance_id  = data_link_cfg_ptr->src_mod_inst_id;
   data_link_ptr->src_port.port_id             = data_link_cfg_ptr->src_mod_op_port_id;
   data_link_ptr->dstn_port.module_instance_id = data_link_cfg_ptr->dst_mod_inst_id;
   data_link_ptr->dstn_port.port_id            = data_link_cfg_ptr->dst_mod_ip_port_id;

   /** For the source module, add this data link to list of it's
    *  output port connections */
   if (AR_EOK != (result = apm_db_add_node_to_list(&module_node_pptr[SRC_MODULE]->output_data_link_list_ptr,
                                                   data_link_ptr,
                                                   &module_node_pptr[SRC_MODULE]->num_output_data_link)))
   {
      goto __bail_out_dlink_cache;
   }

   /** For the destination module, add this data link to list of
    *  it's input port connections */
   if (AR_EOK != (result = apm_db_add_node_to_list(&module_node_pptr[DSTN_MODULE]->input_data_link_list_ptr,
                                                   data_link_ptr,
                                                   &module_node_pptr[DSTN_MODULE]->num_input_data_link)))
   {
	  apm_db_remove_node_from_list(&module_node_pptr[SRC_MODULE]->output_data_link_list_ptr,
                                                data_link_ptr,
                                                &module_node_pptr[SRC_MODULE]->num_output_data_link);

      goto __bail_out_dlink_cache;
   }

   return result;

__bail_out_dlink_cache:

   /** Free up the allocated memory */
   if (data_link_ptr)
   {
      posal_memory_free(data_link_ptr);
   }

   return result;
}

ar_result_t apm_destroy_one_time_data_paths(apm_t *apm_info_ptr)
{
   ar_result_t                 result = AR_EOK;
   spf_list_node_t *           curr_node_ptr;
   apm_data_path_delay_info_t *delay_data_path_ptr;
   uint32_t                    path_id;

   /** Get the pointer to available data paths */
   curr_node_ptr = apm_info_ptr->graph_info.data_path_list_ptr;

   /** Iterate over the the list of available data paths */
   while (curr_node_ptr)
   {
      /** Get the pointer to path delay obj */
      delay_data_path_ptr = (apm_data_path_delay_info_t *)curr_node_ptr->obj_ptr;

      /** if not one time queried, skip this data path */
      if (!delay_data_path_ptr->data_path.flags.one_time_query)
      {
         /** Advance to next node in the list */
         curr_node_ptr = curr_node_ptr->next_ptr;

         /** Skip current iteration */
         continue;
      }

      /** Cache the path ID */
      path_id = delay_data_path_ptr->data_path.path_id;

      /** Free up the memory shared with containers for delay
       *  reporting */
      if (delay_data_path_ptr->delay_shmem_list_ptr)
      {
         posal_memory_free(delay_data_path_ptr->delay_shmem_list_ptr);
      }

      /** Free up the list of verticesm if present */
      if (delay_data_path_ptr->data_path.vertices_list_ptr)
      {
         spf_list_delete_list(&delay_data_path_ptr->data_path.vertices_list_ptr, TRUE);
      }

      /** Remove this node from the list and free up the obj. This
       *  all also increments the list pointer */
      spf_list_delete_node_and_free_obj(&curr_node_ptr, &apm_info_ptr->graph_info.data_path_list_ptr, TRUE);

      /** Decrement the data path counter */
      apm_info_ptr->graph_info.num_data_paths--;

      AR_MSG(DBG_HIGH_PRIO,
             "apm_destroy_one_time_data_paths(): Destroyed path id[%lu], cmd_opcode[0x%lX]",
             path_id,
             apm_info_ptr->curr_cmd_ctrl_ptr->cmd_opcode);

   } /** End of while(data path list) */

   return result;
}

ar_result_t apm_cache_cont_data_path_src_info(apm_t *apm_info_ptr, apm_data_path_delay_info_t *delay_data_path_ptr)
{
   ar_result_t                           result = AR_EOK;
   apm_module_t *                        src_module_node_ptr;
   apm_container_t *                     src_mod_cont_ptr = NULL;
   void *                                malloc_ptr;
   apm_cont_cmd_ctrl_t *                 cont_cmd_ctrl_ptr;
   apm_cont_path_delay_src_module_cfg_t *cached_path_delay_src_mod_cfg_ptr;
   apm_data_path_info_t *                data_path_info_ptr;

   /** Get the pointer to data path info */
   data_path_info_ptr = &delay_data_path_ptr->data_path;

   /** Try to get module node corresponding to source in the
    *  path definition  */
   if (AR_EOK != (result = apm_db_get_module_node(&apm_info_ptr->graph_info,
                                                  data_path_info_ptr->path_dfn.src_module_instance_id,
                                                  &src_module_node_ptr,
                                                  APM_DB_OBJ_QUERY)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_cache_cont_data_path_src_info(): Failed to get src module_iid[0x%lx]",
             data_path_info_ptr->path_dfn.src_module_instance_id);

      return result;
   }

   /** Source module already closed */
   if (!src_module_node_ptr)
   {
      AR_MSG(DBG_MED_PRIO,
             "apm_cache_cont_data_path_src_info(): src module_iid[0x%lX], already closed",
             data_path_info_ptr->path_dfn.src_module_instance_id);

      return AR_EOK;
   }

   /** Execution falls through if source module is present. Get
    *  the pointer to the host container of source module. */
   src_mod_cont_ptr = src_module_node_ptr->host_cont_ptr;

   /** Get the container's command control object corresponding
    *  to current APM command under process */
   apm_get_cont_cmd_ctrl_obj(src_mod_cont_ptr, apm_info_ptr->curr_cmd_ctrl_ptr, &cont_cmd_ctrl_ptr);

   /** Clear the cached configuration for this container */
   apm_release_cont_msg_cached_cfg(&cont_cmd_ctrl_ptr->cached_cfg_params, apm_info_ptr->curr_cmd_ctrl_ptr);

   /** Allocate memory for caching the source module config info
    *  to be sent to containers */
   if (NULL == (malloc_ptr = posal_memory_malloc(sizeof(apm_cont_path_delay_src_module_cfg_t), APM_INTERNAL_STATIC_HEAP_ID)))
   {
      AR_MSG(DBG_ERROR_PRIO, "apm_cache_cont_data_path_src_info(): Failed to allocate the path delay node");

      return AR_ENOMEMORY;
   }

   cached_path_delay_src_mod_cfg_ptr = (apm_cont_path_delay_src_module_cfg_t *)malloc_ptr;

   /** Clear the allocated struct */
   memset(cached_path_delay_src_mod_cfg_ptr, 0, sizeof(apm_cont_path_delay_src_module_cfg_t));

   /** Init the param ID */
   cached_path_delay_src_mod_cfg_ptr->header.param_id = CNTR_PARAM_ID_DESTROY_SRC_MOD_DELAY_LIST;

   /** Cache the delay data path info pointer */
   cached_path_delay_src_mod_cfg_ptr->delay_path_info_ptr = delay_data_path_ptr;

   /** Cache this node to be sent to containers */
   if (AR_EOK !=
       (result = apm_db_add_node_to_list(&cont_cmd_ctrl_ptr->cached_cfg_params.cont_cfg_params.param_data_list_ptr,
                                         cached_path_delay_src_mod_cfg_ptr,
                                         &cont_cmd_ctrl_ptr->cached_cfg_params.cont_cfg_params.num_cfg_params)))
   {
      AR_MSG(DBG_ERROR_PRIO, "apm_cache_cont_data_path_src_info(): Failed to cache the allocated path delay node");

      /** Free up the allocated object */
      posal_memory_free(cached_path_delay_src_mod_cfg_ptr);

      return result;
   }

#ifdef APM_DEBUG_DATA_PATH
   AR_MSG(DBG_HIGH_PRIO,
          "apm_cache_cont_data_path_src_info(): Cached param_id[0x%lX], CONT_ID[0x%lX]",
          cached_path_delay_src_mod_cfg_ptr->header.param_id,
          src_mod_cont_ptr->container_id);
#endif

   /** Add this container to the list of containers pending send
    *  message */
   apm_add_cont_to_pending_msg_send_list(apm_info_ptr->curr_cmd_ctrl_ptr, src_mod_cont_ptr, cont_cmd_ctrl_ptr);

   return result;
}

static ar_result_t apm_cache_boundary_mod_host_cont_data_path_info(apm_t *                     apm_info_ptr,
                                                                   apm_data_path_delay_info_t *delay_data_path_ptr)
{
   ar_result_t                result = AR_EOK;
   apm_module_t *             curr_module_node_ptr;
   uint32_t                   curr_module_iid;
   apm_container_t *          host_container_ptr;
   apm_cont_cmd_ctrl_t *      cont_cmd_ctrl_ptr;
   apm_cont_path_delay_cfg_t *cached_path_delay_cfg_ptr      = NULL;
   uint32_t                   mod_iid_list[2]                = { 0 };
   uint32_t                   num_modules_to_process         = 0;
   uint32_t                   list_idx                       = 0;
   bool_t                     DATA_PATH_DESTROY              = FALSE;
   bool_t                     CREATE_NEW_CONT_CACHED_CFG_OBJ = TRUE;
   bool_t                     data_path_obj_created;

   /** Check if the source module is not closed, then add host
    *  container to path destroy command list */
   if (!delay_data_path_ptr->data_path.flags.src_module_closed)
   {
      mod_iid_list[list_idx++] = delay_data_path_ptr->data_path.path_dfn.src_module_instance_id;
      num_modules_to_process++;
   }

   /** Check if the destination module is not closed, then add host
    *  container to path destroy command list */
   if (!delay_data_path_ptr->data_path.flags.dstn_module_closed)
   {
      mod_iid_list[list_idx++] = delay_data_path_ptr->data_path.path_dfn.dst_module_instance_id;
      num_modules_to_process++;
   }

   for (list_idx = 0; list_idx < num_modules_to_process; list_idx++)
   {
      /** Get current module IID */
      curr_module_iid = mod_iid_list[list_idx];

      /** Try to get module node corresponding to source in the
       *  path definition  */
      if (AR_EOK != (result = apm_db_get_module_node(&apm_info_ptr->graph_info,
                                                     curr_module_iid,
                                                     &curr_module_node_ptr,
                                                     APM_DB_OBJ_QUERY)))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_cache_boundary_mod_host_cont_data_path_info(): Failed to get src module_iid[0x%lx]",
                curr_module_iid);

         return result;
      }

      /** Source module already closed */
      if (!curr_module_node_ptr)
      {
         AR_MSG(DBG_MED_PRIO,
                "apm_cache_boundary_mod_host_cont_data_path_info(): module_iid[0x%lx], already closed",
                curr_module_iid);

         continue;
      }

      /** Get the host container of the current module */
      host_container_ptr = curr_module_node_ptr->host_cont_ptr;

      /** Get the pointer to current container command control
       *  pointer */
      apm_get_cont_cmd_ctrl_obj(host_container_ptr, apm_info_ptr->curr_cmd_ctrl_ptr, &cont_cmd_ctrl_ptr);

      if (!cont_cmd_ctrl_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_cache_boundary_mod_host_cont_data_path_info(): Failed to get cont cmd ctrl obj, CONT_ID[0x%lX]",
                host_container_ptr->container_id);

         return AR_EFAILED;
      }

      /** Cache the path destroy cfg for current container.  */
      if (AR_EOK != (result = apm_get_cont_cached_path_delay_cfg_obj(cont_cmd_ctrl_ptr,
                                                                     &delay_data_path_ptr->data_path,
                                                                     &cached_path_delay_cfg_ptr,
                                                                     DATA_PATH_DESTROY,
                                                                     CREATE_NEW_CONT_CACHED_CFG_OBJ,
                                                                     &data_path_obj_created)))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_cache_boundary_mod_host_cont_data_path_info(): Failed to get path delay cached object, "
                "CONT_ID[0x%lX]",
                host_container_ptr->container_id);

         return result;
      }

      if (data_path_obj_created)
      {
         /** Add this container to pending message send list */
         apm_add_cont_to_pending_msg_send_list(apm_info_ptr->curr_cmd_ctrl_ptr, host_container_ptr, cont_cmd_ctrl_ptr);
      }

   } /** End of for() */

   return result;
}

ar_result_t apm_cache_container_data_path_destroy_info(apm_t *                     apm_info_ptr,
                                                       apm_data_path_delay_info_t *delay_data_path_ptr)
{
   ar_result_t result = AR_EOK;

   /** If the boundary modules in the data path are not closed,
    *  prepare destroy path ID payloads for them */
   if (!delay_data_path_ptr->data_path.flags.src_module_closed ||
       !delay_data_path_ptr->data_path.flags.dstn_module_closed)
   {
      result = apm_cache_boundary_mod_host_cont_data_path_info(apm_info_ptr, delay_data_path_ptr);
   }

   /** For rest of the data path, iterate over the container list in this data path
    *  and send path destroy command to the containers */
   result = apm_cache_and_destroy_container_data_path_info(apm_info_ptr, &delay_data_path_ptr->data_path);

   return result;
}

ar_result_t apm_update_dangling_data_paths_to_containers(apm_t *apm_info_ptr)
{
   ar_result_t                 result = AR_EOK;
   spf_list_node_t *           curr_data_path_node_ptr;
   apm_data_path_delay_info_t *delay_data_path_ptr;
   bool_t                      dangling_path_present = FALSE;

   /** Get the pointer to available data paths */
   curr_data_path_node_ptr = apm_info_ptr->graph_info.data_path_list_ptr;

   /** Iterate over the the list of available data paths */
   while (curr_data_path_node_ptr)
   {
      /** Get the pointer to path delay obj */
      delay_data_path_ptr = (apm_data_path_delay_info_t *)curr_data_path_node_ptr->obj_ptr;

      /** if no updates required to containers, skip this data path */
      if (!delay_data_path_ptr->data_path.flags.container_path_update)
      {
         /** Advance to next node in the list */
         curr_data_path_node_ptr = curr_data_path_node_ptr->next_ptr;

         /** Skip current iteration */
         continue;
      }

      /** For the container hosting source module, first cache the
       *  list of source module(s) to be sent to invalidate the list
       *  of delay pointers. If the source module port is closed
       *  because of partial data path close, module will ignore
       *  this message */

      if (!delay_data_path_ptr->data_path.flags.src_module_closed)
      {
         if (AR_EOK != (result = apm_cache_cont_data_path_src_info(apm_info_ptr, delay_data_path_ptr)))
         {
            return result;
         }
      }

      /** Iterate over the list of remaining vertices in the data
       *  path graph and inform the host containers about data path
       *  being destroyed. For the container hosting the source module
       *  in the data path also needs to be informed to invalidate
       *  the list of delay pointers in the data path */

      if (AR_EOK != (result = apm_cache_container_data_path_destroy_info(apm_info_ptr, delay_data_path_ptr)))
      {
         return result;
      }

      /** At least 1 dangling data path is present */
      dangling_path_present = TRUE;

      /** Advance to next node in the list */
      curr_data_path_node_ptr = curr_data_path_node_ptr->next_ptr;

   } /** End of while (data path list) */

   /** If the dangling data path is present send message to
    *  impacted containers */
   if (dangling_path_present)
   {
      /** If the dangling data path is present and message needs
       *  to be sent to atleast 1 container. If list is empty flag
       *  error */
      if (!apm_info_ptr->curr_cmd_ctrl_ptr->rsp_ctrl.num_pending_container)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_update_dangling_data_paths_to_containers(): Pending container list is empty, cmd_opcode[0x%lx]",
                apm_info_ptr->curr_cmd_ctrl_ptr->cmd_opcode);

         return AR_EFAILED;
      }

   } /** if (dangling_path_present) */

   return result;
}

ar_result_t apm_close_data_path_list(apm_t *apm_info_ptr)
{
   ar_result_t                 result = AR_EOK;
   spf_list_node_t *           curr_node_ptr;
   apm_data_path_delay_info_t *delay_path_info_ptr;
   uint32_t                    path_id;

   /** Get the pointer to list of container pending send message */
   curr_node_ptr = apm_info_ptr->graph_info.data_path_list_ptr;

   /** Iterate the list of available data paths */
   while (curr_node_ptr)
   {
      /** Get the pointer delay path object */
      delay_path_info_ptr = (apm_data_path_delay_info_t *)curr_node_ptr->obj_ptr;

      /** If the path is marked for updating the containers, then
       *  free up the vertices graph and shmem for delay pointer
       *  list */
      if (delay_path_info_ptr->data_path.flags.container_path_update)
      {
         if (delay_path_info_ptr->delay_shmem_list_ptr)
         {
            posal_memory_free(delay_path_info_ptr->delay_shmem_list_ptr);
            delay_path_info_ptr->delay_shmem_list_ptr = NULL;
         }

         /** Clear the  data path graph  */
         if (delay_path_info_ptr->data_path.vertices_list_ptr)
         {
            spf_list_delete_list(&delay_path_info_ptr->data_path.vertices_list_ptr, TRUE);

            delay_path_info_ptr->data_path.vertices_list_ptr = NULL;
            delay_path_info_ptr->data_path.num_vertices      = 0;
         }

         /** Clear the container path update */
         delay_path_info_ptr->data_path.flags.container_path_update = FALSE;

         /** Clear the path validity flag */
         delay_path_info_ptr->data_path.flags.path_valid = FALSE;

         AR_MSG(DBG_MED_PRIO,
                "apm_close_data_path_list(): Freed vertices list and list pointer for path id[%lu]",
                delay_path_info_ptr->data_path.path_id);
      }

      /** If the source module is closed or if vertices list is
       *  empty, delete the data path object */
      if (delay_path_info_ptr->data_path.flags.src_module_closed || !delay_path_info_ptr->data_path.vertices_list_ptr)
      {
         /** Cache the data path ID */
         path_id = delay_path_info_ptr->data_path.path_id;

         if (delay_path_info_ptr->data_path.container_list.list_ptr)
         {
            /*As the data path is getting destroyed, empty container list for this particular data path.*/
            spf_list_delete_list(&delay_path_info_ptr->data_path.container_list.list_ptr, TRUE /** Pool used*/);
         }
         delay_path_info_ptr->data_path.container_list.num_nodes = 0;

         /** Remove the node from the list. This call also incrementes
          *  the list pointer */
         spf_list_delete_node_and_free_obj(&curr_node_ptr, &apm_info_ptr->graph_info.data_path_list_ptr, TRUE);

         /** Decrement the number of data paths */
         apm_info_ptr->graph_info.num_data_paths--;

         /** If number of data paths become zero, clear the list
          *  pointer */
         if (!apm_info_ptr->graph_info.num_data_paths)
         {
            apm_info_ptr->graph_info.data_path_list_ptr = NULL;
         }

         AR_MSG(DBG_HIGH_PRIO, "apm_close_data_path_list(): Closed data path id[%lu]", path_id);
      }
      else /** Source module not closed */
      {
         /** Continue iterating the list */
         curr_node_ptr = curr_node_ptr->next_ptr;
      }

   } /** End of while (data path list) */

   return result;
}

ar_result_t apm_reconfigure_delay_data_path(apm_t *apm_info_ptr)
{
   ar_result_t                 result = AR_EOK;
   spf_list_node_t *           curr_node_ptr;
   apm_data_path_delay_info_t *data_path_info_ptr;
   apm_module_t *              module_node_ptr      = NULL;
   bool_t                      PERSISTENT_DATA_PATH = FALSE;
   bool_t                      RECONFIG_DATA_PATH   = TRUE;

   /** Get the pointer to the list of available data paths */
   curr_node_ptr = apm_info_ptr->graph_info.data_path_list_ptr;

   /** Iterate over the data path lists */
   while (curr_node_ptr)
   {
      /** Get the pointer to delay data path object */
      data_path_info_ptr = (apm_data_path_delay_info_t *)curr_node_ptr->obj_ptr;

      /** If data path is not valid, this implies that at least the
       *  source module is still present */
      if (!data_path_info_ptr->data_path.flags.path_valid)
      {
         /** Check if the destination module is also opened, then attempt
          *  to configure the path. Note that this may still fail if
          *  either the intermediate graph is not opened or no path
          *  exists to destination module. */
         apm_db_get_module_node(&apm_info_ptr->graph_info,
                                data_path_info_ptr->data_path.path_dfn.dst_module_instance_id,
                                &module_node_ptr,
                                APM_DB_OBJ_QUERY);

         if (NULL != module_node_ptr)
         {
            if (AR_EOK != (result = apm_configure_delay_data_path(apm_info_ptr,
                                                                  &data_path_info_ptr->data_path.path_dfn,
                                                                  &data_path_info_ptr,
                                                                  PERSISTENT_DATA_PATH,
                                                                  RECONFIG_DATA_PATH)))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "apm_reconfigure_delay_data_path(): Delay path reconfig failed"
                      "cmd_opcode[0x%lx], result: 0x%lX",
                      apm_info_ptr->curr_cmd_ctrl_ptr->cmd_opcode,
                      result);

               return result;
            }

         } /** End of if (dstn module exists) */

      } /** End of if (data path valid) */

      /** Advance to next data path object */
      curr_node_ptr = curr_node_ptr->next_ptr;

   } /** End of while(delay data path list) */

   /** Check if the message needs to be sent to atleast 1
    *  container, if not flag error */
   if (!apm_info_ptr->curr_cmd_ctrl_ptr->rsp_ctrl.num_pending_container)
   {
      AR_MSG(DBG_MED_PRIO,
             "apm_reconfigure_delay_data_path(): No dangling data paths could be reconfigured"
             "cmd_opcode[0x%lx]",
             apm_info_ptr->curr_cmd_ctrl_ptr->cmd_opcode);

      return AR_EOK;
   }

   return result;
}

ar_result_t apm_cfg_src_module_delay_list(apm_t *                  apm_info_ptr,
                                          apm_container_t *        rsp_container_ptr,
                                          apm_module_param_data_t *param_data_hdr_ptr,
                                          bool_t *                 cont_processed_ptr)
{
   ar_result_t                           result = AR_EOK;
   apm_cont_cmd_ctrl_t *                 cont_cmd_ctrl_ptr;
   apm_cont_path_delay_src_module_cfg_t *src_mod_cfg_ptr;
   apm_data_path_delay_info_t *          delay_data_path_info_ptr = NULL;
   cntr_param_id_path_delay_cfg_t *      path_delay_pid_payload_ptr;
   apm_container_t *                     host_container_ptr = NULL;
   apm_module_t *                        src_module_ptr     = NULL;

   /** Clear the return pointer */
   *cont_processed_ptr = FALSE;

   /** Validate the input pointers */
   if (!param_data_hdr_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "apm_cfg_src_module_delay_list(): Param data pointer is NULL");

      return AR_EFAILED;
   }

   /** Get the PID payload pointer */
   path_delay_pid_payload_ptr = (cntr_param_id_path_delay_cfg_t *)(param_data_hdr_ptr + 1);

   /** Get the data path object corresponding to container param
    *  ID */
   if (AR_EOK != (result = apm_db_get_data_path_obj(apm_info_ptr->graph_info.data_path_list_ptr,
                                                    path_delay_pid_payload_ptr->path_id,
                                                    &delay_data_path_info_ptr)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_cfg_src_module_delay_list(): Failed to get path delay global obj, path_id[0x%lx]",
             path_delay_pid_payload_ptr->path_id);

      return result;
   }

   /** Get the source module node */
   apm_db_get_module_node(&apm_info_ptr->graph_info,
                          delay_data_path_info_ptr->data_path.path_dfn.src_module_instance_id,
                          &src_module_ptr,
                          APM_DB_OBJ_QUERY);

   /** Validate the source module */
   if (!src_module_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_cfg_src_module_delay_list(): SRC module IID [0x%lX] not found",
             delay_data_path_info_ptr->data_path.path_dfn.src_module_instance_id);

      return AR_EFAILED;
   }

   /** Get the host container for the source module  */
   host_container_ptr = src_module_ptr->host_cont_ptr;

   /** If the current container ID does not match with the host
    *  container of the source module in the data path, return */
   if (rsp_container_ptr->container_id != host_container_ptr->container_id)
   {
      *cont_processed_ptr = FALSE;
      return AR_EOK;
   }

   /** Get the command control object for the host container
    *  corresponding to current APM command under process */
   apm_get_cont_cmd_ctrl_obj(host_container_ptr, apm_info_ptr->curr_cmd_ctrl_ptr, &cont_cmd_ctrl_ptr);

   /** Allocate memory for caching the delay pointer list for
    *  source module */
   if (NULL == (src_mod_cfg_ptr = (apm_cont_path_delay_src_module_cfg_t *)
                   posal_memory_malloc(sizeof(apm_cont_path_delay_src_module_cfg_t), APM_INTERNAL_STATIC_HEAP_ID)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_cfg_src_module_delay_list(): Failed to allocate memory for caching cont info, path_id[0x%lx], "
             "CONT_ID[0x%lX]",
             path_delay_pid_payload_ptr->path_id,
             host_container_ptr->container_id);

      return AR_ENOMEMORY;
   }

   /** Populate the allocated cached object */
   src_mod_cfg_ptr->header.param_id     = CNTR_PARAM_ID_CFG_SRC_MOD_DELAY_LIST;
   src_mod_cfg_ptr->delay_path_info_ptr = delay_data_path_info_ptr;

   /** Add the allocated to running list of cached objects */
   apm_db_add_node_to_list(&cont_cmd_ctrl_ptr->cached_cfg_params.cont_cfg_params.param_data_list_ptr,
                           src_mod_cfg_ptr,
                           &cont_cmd_ctrl_ptr->cached_cfg_params.cont_cfg_params.num_cfg_params);

   /** Set the container processed flag */
   *cont_processed_ptr = TRUE;

   /** Clear the container path update flag */
   delay_data_path_info_ptr->data_path.flags.container_path_update = FALSE;

   AR_MSG(DBG_HIGH_PRIO,
          "apm_cfg_src_module_delay_list(): Cached delay pointer list for CONT_ID[0x%lx],"
          "num delay pointers[%lu], ptr_arr[0x%lX] ",
          host_container_ptr->container_id,
          delay_data_path_info_ptr->data_path.num_containers,
          delay_data_path_info_ptr->delay_shmem_list_ptr);

   return result;
}

ar_result_t apm_get_delay_path_rsp_payload_offset(apm_t *                     apm_info_ptr,
                                                  spf_data_path_dfn_t *       cont_path_dfn_ptr,
                                                  apm_module_param_data_t **  param_hdr_pptr,
                                                  apm_path_defn_for_delay_t **pid_path_dfn_pptr)
{
   ar_result_t              result = AR_EFAILED;
   uint32_t                 num_data_paths;
   spf_list_node_t *        curr_node_ptr;
   apm_module_param_data_t *param_hdr_ptr;

   /** Clear the return pointers */
   *pid_path_dfn_pptr = NULL;
   *param_hdr_pptr    = NULL;

   /** Get the pointer to the list of param ID payloads
    *  corresponding to containers */
   curr_node_ptr = apm_info_ptr->curr_cmd_ctrl_ptr->get_cfg_cmd_ctrl.pid_payload_list_ptr;

   /** Iterate over this list */
   while (curr_node_ptr)
   {
      /** Get the pointer to param data header */
      param_hdr_ptr = (apm_module_param_data_t *)curr_node_ptr->obj_ptr;

      /** Handle as per the param ID */
      switch (param_hdr_ptr->param_id)
      {
         case APM_PARAM_ID_PATH_DELAY:
         {
            apm_path_defn_for_delay_t *cmd_path_dfn_ptr;
            apm_param_id_path_delay_t *pid_payload_ptr;

            /** Get the pointer to path delay param ID payload */
            pid_payload_ptr = (apm_param_id_path_delay_t *)(param_hdr_ptr + 1);

            /** Get the total number of data paths */
            num_data_paths = pid_payload_ptr->num_paths;

            /** Get the pointer to start of path definition object array */
            cmd_path_dfn_ptr = (apm_path_defn_for_delay_t *)(pid_payload_ptr + 1);

            /** Iterate over the list of data path definition object
             *  list, check for the matching path defintion and return
             *  it's offset */
            for (uint32_t idx = 0; idx < num_data_paths; idx++)
            {
               if ((cmd_path_dfn_ptr->src_module_instance_id == cont_path_dfn_ptr->src_module_instance_id) &&
                   (cmd_path_dfn_ptr->src_port_id == cont_path_dfn_ptr->src_port_id) &&
                   (cmd_path_dfn_ptr->dst_module_instance_id == cont_path_dfn_ptr->dst_module_instance_id) &&
                   (cmd_path_dfn_ptr->dst_port_id == cont_path_dfn_ptr->dst_port_id))
               {
                  /** Match found, populate the return pointer and return */
                  *pid_path_dfn_pptr = cmd_path_dfn_ptr;

                  /** Populate the corresponding header pointer */
                  *param_hdr_pptr = param_hdr_ptr;

                  return AR_EOK;
               }

               /** Increment the list pointer */
               cmd_path_dfn_ptr++;

            } /** End of for (data paths) */

            break;
         }

         case APM_PARAM_ID_OFFLOAD_GRAPH_PATH_DELAY:
         {
            apm_offload_graph_path_defn_for_delay_t *cmd_path_dfn_ptr;
            apm_param_id_offload_graph_path_delay_t *pid_payload_ptr;
            /** Get the pointer to path delay param ID payload */
            pid_payload_ptr = (apm_param_id_offload_graph_path_delay_t *)(param_hdr_ptr + 1);

            /** Get the total number of data paths */
            num_data_paths = pid_payload_ptr->num_paths;

            /** Get the pointer to start of path definition object array */
            cmd_path_dfn_ptr = (apm_offload_graph_path_defn_for_delay_t *)(pid_payload_ptr + 1);

            /** Iterate over the list of data path definition object
             *  list, check for the matching path defintion and return
             *  it's offset */
            for (uint32_t idx = 0; idx < num_data_paths; idx++)
            {
               if ((cmd_path_dfn_ptr->src_module_instance_id == cont_path_dfn_ptr->src_module_instance_id) &&
                   (cmd_path_dfn_ptr->src_port_id == cont_path_dfn_ptr->src_port_id) &&
                   (cmd_path_dfn_ptr->dst_module_instance_id == cont_path_dfn_ptr->dst_module_instance_id) &&
                   (cmd_path_dfn_ptr->dst_port_id == cont_path_dfn_ptr->dst_port_id))
               {
                  /** Match found, populate the return pointer and return */
                  *pid_path_dfn_pptr = (apm_path_defn_for_delay_t *)cmd_path_dfn_ptr;

                  /** Populate the corresponding header pointer */
                  *param_hdr_pptr = param_hdr_ptr;

                  return AR_EOK;
               }

               /** Increment the list pointer */
               cmd_path_dfn_ptr++;

            } /** End of for (data paths) */
            break;
         }
         default:
         {
            break;
         }

      } /** End of switch (param_hdr_ptr->param_id) */

      curr_node_ptr = curr_node_ptr->next_ptr;

   } /** End of while (get param payload pointers) */

   return result;
}

ar_result_t apm_compute_cntr_path_delay_param_payload_size(uint32_t                    container_id,
                                                           apm_cont_set_get_cfg_hdr_t *set_get_cfg_hdr_ptr,
                                                           uint32_t *                  msg_payload_size_ptr)
{
   ar_result_t result           = AR_EOK;
   uint32_t    msg_payload_size = 0;
   uint32_t    num_ptr_objs;

   apm_cont_path_delay_src_module_cfg_t *path_delay_src_module_cfg_ptr;

   /** Clear return pointer */
   *msg_payload_size_ptr = 0;

   switch (set_get_cfg_hdr_ptr->param_id)
   {
      case CNTR_PARAM_ID_PATH_DELAY_CFG:
      {
         /** List of vertices is OOB, need not be accounted in the
          *  overall payload size */
         msg_payload_size += sizeof(apm_module_param_data_t) + sizeof(cntr_param_id_path_delay_cfg_t);

         break;
      }
      case CNTR_PARAM_ID_CFG_SRC_MOD_DELAY_LIST:
      {
         /** Get the pointer to cached delay pointer for the source
          *  module for a given path ID */
         path_delay_src_module_cfg_ptr = (apm_cont_path_delay_src_module_cfg_t *)set_get_cfg_hdr_ptr;

         /** Get the total number of containers in the data path */
         num_ptr_objs = path_delay_src_module_cfg_ptr->delay_path_info_ptr->data_path.num_containers;

         /** Update the overall payload size */
         msg_payload_size += (sizeof(apm_module_param_data_t) + sizeof(cntr_param_id_cfg_src_mod_delay_list_t) +
                              (SIZE_OF_PTR() * num_ptr_objs));

         break;
      }
      case CNTR_PARAM_ID_PATH_DESTROY:
      {
         msg_payload_size += sizeof(apm_module_param_data_t) + sizeof(cntr_param_id_path_destroy_t);

         break;
      }
      case CNTR_PARAM_ID_DESTROY_SRC_MOD_DELAY_LIST:
      {
         msg_payload_size += sizeof(apm_module_param_data_t) + sizeof(cntr_param_id_destroy_src_mod_delay_list_t);
         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_compute_cntr_path_delay_param_payload_size(): Un-supported param ID[0x%lx], CONT_ID[0x%lX]",
                set_get_cfg_hdr_ptr->param_id,
                container_id);

         result = AR_EUNSUPPORTED;

         break;
      }

   } /** End of switch (param_id) */

   /** Update the return pointer with calculaed size */
   *msg_payload_size_ptr = msg_payload_size;

   return result;
}

ar_result_t apm_populate_cntr_path_delay_params(uint32_t                    container_id,
                                                apm_cont_set_get_cfg_hdr_t *set_get_cfg_hdr_ptr,
                                                apm_module_param_data_t *   param_data_ptr)
{
   ar_result_t                                 result = AR_EOK;
   cntr_param_id_path_delay_cfg_t *            delay_msg_pid_payload_ptr;
   cntr_param_id_cfg_src_mod_delay_list_t *    delay_list_pid_payload_ptr;
   apm_cont_path_delay_cfg_t *                 cached_cont_delay_cfg_ptr;
   apm_cont_path_delay_src_module_cfg_t *      cached_src_mod_delay_cfg_ptr;
   cntr_param_id_path_destroy_t *              path_destroy_pid_payload_ptr;
   cntr_param_id_destroy_src_mod_delay_list_t *src_mod_delay_list_destroy_payload_ptr;

   switch (set_get_cfg_hdr_ptr->param_id)
   {
      case CNTR_PARAM_ID_PATH_DELAY_CFG:
      {
         cached_cont_delay_cfg_ptr = (apm_cont_path_delay_cfg_t *)set_get_cfg_hdr_ptr;

         apm_update_cont_set_cfg_msg_hdr(container_id,
                                         CNTR_PARAM_ID_PATH_DELAY_CFG,
                                         sizeof(cntr_param_id_path_delay_cfg_t),
                                         param_data_ptr);

         delay_msg_pid_payload_ptr = (cntr_param_id_path_delay_cfg_t *)(param_data_ptr + 1);

         delay_msg_pid_payload_ptr->is_one_time_query = cached_cont_delay_cfg_ptr->data_path.flags.one_time_query;

         delay_msg_pid_payload_ptr->path_id      = cached_cont_delay_cfg_ptr->data_path.path_id;
         delay_msg_pid_payload_ptr->delay_us_ptr = cached_cont_delay_cfg_ptr->delay_ptr;
         delay_msg_pid_payload_ptr->path_def_ptr =
            (cntr_list_node_t *)cached_cont_delay_cfg_ptr->data_path.vertices_list_ptr;

         break;
      }
      case CNTR_PARAM_ID_CFG_SRC_MOD_DELAY_LIST:
      {
         cached_src_mod_delay_cfg_ptr = (apm_cont_path_delay_src_module_cfg_t *)set_get_cfg_hdr_ptr;

         apm_update_cont_set_cfg_msg_hdr(container_id,
                                         CNTR_PARAM_ID_CFG_SRC_MOD_DELAY_LIST,
                                         sizeof(cntr_param_id_cfg_src_mod_delay_list_t),
                                         param_data_ptr);

         /** Get the pointer to PID payload start */
         delay_list_pid_payload_ptr = (cntr_param_id_cfg_src_mod_delay_list_t *)(param_data_ptr + 1);

         /** Populate the path ID */
         delay_list_pid_payload_ptr->path_id = cached_src_mod_delay_cfg_ptr->delay_path_info_ptr->data_path.path_id;

         /** Populate Path definition */
         delay_list_pid_payload_ptr->src_module_instance_id =
            cached_src_mod_delay_cfg_ptr->delay_path_info_ptr->data_path.path_dfn.src_module_instance_id;

         delay_list_pid_payload_ptr->src_port_id =
            cached_src_mod_delay_cfg_ptr->delay_path_info_ptr->data_path.path_dfn.src_port_id;

         delay_list_pid_payload_ptr->dst_module_instance_id =
            cached_src_mod_delay_cfg_ptr->delay_path_info_ptr->data_path.path_dfn.dst_module_instance_id;

         delay_list_pid_payload_ptr->dst_port_id =
            cached_src_mod_delay_cfg_ptr->delay_path_info_ptr->data_path.path_dfn.dst_port_id;

         /** Populate number of delay pointers */
         delay_list_pid_payload_ptr->num_delay_ptrs =
            cached_src_mod_delay_cfg_ptr->delay_path_info_ptr->data_path.num_containers;

         /** Init the pointer to array of pointers */
         delay_list_pid_payload_ptr->delay_us_pptr = (volatile uint32_t **)(delay_list_pid_payload_ptr + 1);

         /** Populate the list of delay pointers */
         for (uint32_t idx = 0; idx < delay_list_pid_payload_ptr->num_delay_ptrs; idx++)
         {
            delay_list_pid_payload_ptr->delay_us_pptr[idx] =
               &cached_src_mod_delay_cfg_ptr->delay_path_info_ptr->delay_shmem_list_ptr[idx].delay_us;
         }

         break;
      }
      case CNTR_PARAM_ID_DESTROY_SRC_MOD_DELAY_LIST:
      {
         cached_src_mod_delay_cfg_ptr = (apm_cont_path_delay_src_module_cfg_t *)set_get_cfg_hdr_ptr;

         apm_update_cont_set_cfg_msg_hdr(container_id,
                                         CNTR_PARAM_ID_DESTROY_SRC_MOD_DELAY_LIST,
                                         sizeof(cntr_param_id_destroy_src_mod_delay_list_t),
                                         param_data_ptr);

         /** Get thE PID payload pointer */
         src_mod_delay_list_destroy_payload_ptr = (cntr_param_id_destroy_src_mod_delay_list_t *)(param_data_ptr + 1);

         /** Populate the Path ID */
         src_mod_delay_list_destroy_payload_ptr->path_id =
            cached_src_mod_delay_cfg_ptr->delay_path_info_ptr->data_path.path_id;

         /** Populate the source IID and port ID */
         src_mod_delay_list_destroy_payload_ptr->src_module_instance_id =
            cached_src_mod_delay_cfg_ptr->delay_path_info_ptr->data_path.path_dfn.src_module_instance_id;
         src_mod_delay_list_destroy_payload_ptr->src_port_id =
            cached_src_mod_delay_cfg_ptr->delay_path_info_ptr->data_path.path_dfn.src_port_id;

         break;
      }
      case CNTR_PARAM_ID_PATH_DESTROY:
      {
         cached_cont_delay_cfg_ptr = (apm_cont_path_delay_cfg_t *)set_get_cfg_hdr_ptr;

         apm_update_cont_set_cfg_msg_hdr(container_id,
                                         CNTR_PARAM_ID_PATH_DESTROY,
                                         sizeof(cntr_param_id_path_destroy_t),
                                         param_data_ptr);

         path_destroy_pid_payload_ptr = (cntr_param_id_path_destroy_t *)(param_data_ptr + 1);

         /** Populate the path ID to be destroyed */
         path_destroy_pid_payload_ptr->path_id = cached_cont_delay_cfg_ptr->data_path.path_id;

         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_config_cntr_path_delay_params(): Un-supported param ID[0x%lx], CONT_ID[0x%lX]",
                set_get_cfg_hdr_ptr->param_id,
                container_id);

         result = AR_EUNSUPPORTED;

         break;
      }

   } /** End of switch (set_get_cfg_hdr_ptr->param_id) */

   return result;
}

ar_result_t apm_cont_path_delay_msg_rsp_hdlr(apm_t *apm_info_ptr)
{
   ar_result_t                   result = AR_EOK;
   spf_msg_header_t *            rsp_msg_hdr_ptr;
   apm_cmd_ctrl_t *              apm_cmd_ctrl_ptr;
   spf_list_node_t *             curr_cont_node_ptr, *next_cont_node_ptr;
   apm_container_t *             container_obj_ptr;
   apm_cont_cmd_ctrl_t *         cont_cmd_ctrl_ptr = NULL;
   spf_msg_cmd_param_data_cfg_t *spf_set_cfg_msg_ptr;
   apm_module_param_data_t *     param_data_hdr_ptr;
   bool_t                        is_cont_processed = FALSE;

   /** Get the pointer to APM current command control */
   apm_cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   /** Validate if the pending container list is non-empty */
   if (!apm_cmd_ctrl_ptr->rsp_ctrl.pending_cont_list_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_cont_path_delay_msg_rsp_hdlr(): Container list is empty, cmd_opcode[0x%lX]",
             apm_cmd_ctrl_ptr->cmd_opcode);

      return AR_EFAILED;
   }

   /** Get the pointer to pending container list */
   curr_cont_node_ptr = apm_cmd_ctrl_ptr->rsp_ctrl.pending_cont_list_ptr;

   /** Iterate over the list of pending containers */
   while (curr_cont_node_ptr)
   {
      /** Get the pointer to current container object */
      container_obj_ptr = (apm_container_t *)curr_cont_node_ptr->obj_ptr;

      /** Get the next list node   */
      next_cont_node_ptr = curr_cont_node_ptr->next_ptr;

      /** Get the pointer to command control object for this
       *  container corresponding to current APM command under
       *  process */
      if (AR_EOK !=
          (result = apm_get_allocated_cont_cmd_ctrl_obj(container_obj_ptr, apm_cmd_ctrl_ptr, &cont_cmd_ctrl_ptr)))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_cont_path_delay_msg_rsp_hdlr(): Failed to get allocated cmd ctrl obj for CONT_ID[0x%lX], "
                "cmd_opcode[0x%lX]",
                container_obj_ptr->container_id,
                apm_cmd_ctrl_ptr->cmd_opcode);

         return AR_EFAILED;
      }

      /** Get the pointer to container's response message header */
      if (NULL == (rsp_msg_hdr_ptr = (spf_msg_header_t *)cont_cmd_ctrl_ptr->rsp_ctrl.rsp_msg.payload_ptr))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_cont_path_delay_msg_rsp_hdlr(): Cont rsp msg payload is NULL, CONT_ID[0x%lX], "
                "cmd_opcode[0x%lX]",
                container_obj_ptr->container_id,
                apm_cmd_ctrl_ptr->cmd_opcode);

         return AR_EFAILED;
      }

      /** Get the pointer to the message payload */
      spf_set_cfg_msg_ptr = (spf_msg_cmd_param_data_cfg_t *)&rsp_msg_hdr_ptr->payload_start;

      /** Loop over the list of all the configuration parameters,
       *  handle the param ID's pertaining to the container */
      for (uint32_t arr_idx = 0; arr_idx < spf_set_cfg_msg_ptr->num_param_id_cfg; arr_idx++)
      {
         /** Get the pointer to message header */
         param_data_hdr_ptr = (apm_module_param_data_t *)spf_set_cfg_msg_ptr->param_data_pptr[arr_idx];

         /** Check if the instance ID matches with the current
          *  container */
         if (param_data_hdr_ptr->module_instance_id == container_obj_ptr->container_id)
         {
            switch (param_data_hdr_ptr->param_id)
            {
               case CNTR_PARAM_ID_PATH_DELAY_CFG:
               {
                  /** For the host container, configure the list of delay
                   *  pointers for the data path */
                  if (AR_EOK != (result = apm_cfg_src_module_delay_list(apm_info_ptr,
                                                                        container_obj_ptr,
                                                                        param_data_hdr_ptr,
                                                                        &is_cont_processed)))
                  {
                     AR_MSG(DBG_ERROR_PRIO,
                            "apm_cont_path_delay_msg_rsp_hdlr(): Failed to configure delay pointer list, "
                            "CONT_ID[0x%lx]",
                            container_obj_ptr->container_id);
                  }

                  break;
               }
               case CNTR_PARAM_ID_CFG_SRC_MOD_DELAY_LIST:
               {
                  /** Nothing to do */
                  break;
               }
               default:
               {
                  break;
               }
            } /** End of switch(param ID) */

         } /** End if (param inst ID == container ID)*/

      } /** End of for(num set params)*/

      /** Clear the response control for this container */
      apm_clear_cont_cmd_rsp_ctrl(cont_cmd_ctrl_ptr, TRUE /** Release response msg buffer */);

      /** If the container to be retained for sending further
       *  message, done't remove it from pending list */
      if (!is_cont_processed)
      {
         /** Remove this node from the list of pending containers */
         apm_db_remove_node_from_list(&apm_cmd_ctrl_ptr->rsp_ctrl.pending_cont_list_ptr,
                                      container_obj_ptr,
                                      &apm_cmd_ctrl_ptr->rsp_ctrl.num_pending_container);

         /** Release the container command control object */
         apm_release_cont_cmd_ctrl_obj(container_obj_ptr, cont_cmd_ctrl_ptr);
      }

      /** Reset the flag */
      is_cont_processed = FALSE;

      /** Advance to next node in the list */
      curr_cont_node_ptr = next_cont_node_ptr;

   } /** End of while (pending container list) */

   return result;
}

ar_result_t apm_path_delay_event_hdlr(apm_t *apm_info_ptr, spf_msg_t *msg_ptr)
{
   ar_result_t                      result = AR_EOK;
   spf_msg_header_t *               spf_msg_hdr_ptr;
   spf_evt_to_apm_for_path_delay_t *path_delay_evt_payload_ptr;
   apm_data_path_delay_info_t *     path_delay_info_ptr       = NULL;
   apm_module_t *                   module_node_ptr_list[2]   = { NULL };
   bool_t                           DANGLING_LINK_NOT_ALLOWED = FALSE;
   bool_t                           PERSISTENT_DATA_PATH      = FALSE;
   bool_t                           FIRST_TIME_CFG            = FALSE;
   apm_cmd_ctrl_t *                 curr_cmd_ctrl_ptr;

   /** Get the pointer to GK message header */
   spf_msg_hdr_ptr = (spf_msg_header_t *)msg_ptr->payload_ptr;

   /** Get the pointer to path delay event payload */
   path_delay_evt_payload_ptr = (spf_evt_to_apm_for_path_delay_t *)&spf_msg_hdr_ptr->payload_start;

   /** Validate the source and destination module IID's specified
    *  in the path definition query  */
   if (AR_EOK !=
       (result = apm_validate_module_instance_pair(&apm_info_ptr->graph_info,
                                                   path_delay_evt_payload_ptr->path_def.src_module_instance_id,
                                                   path_delay_evt_payload_ptr->path_def.dst_module_instance_id,
                                                   module_node_ptr_list,
                                                   DANGLING_LINK_NOT_ALLOWED)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_path_delay_event_hdlr(): Failed to validate path dfn, src: MIID[0x%lX] and/or dstn module: "
             "MIID[0x%lX] does not exist",
             path_delay_evt_payload_ptr->path_def.src_module_instance_id,
             path_delay_evt_payload_ptr->path_def.dst_module_instance_id);

      return result;
   }

   AR_MSG(DBG_HIGH_PRIO,
          "apm_path_delay_event_hdlr(): Received path delay event params, src: MIID[0x%lX], port_id[0x%lX], "
          "dstn: MIID[0x%lX], port_id[0x%lX]",
          path_delay_evt_payload_ptr->path_def.src_module_instance_id,
          path_delay_evt_payload_ptr->path_def.src_port_id,
          path_delay_evt_payload_ptr->path_def.dst_module_instance_id,
          path_delay_evt_payload_ptr->path_def.dst_port_id);

   /** Get the pointer to current command control object */
   curr_cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   /** Configure delay data path as per the data definition
    *  received */
   if (AR_EOK != (result = apm_configure_delay_data_path(apm_info_ptr,
                                                         &path_delay_evt_payload_ptr->path_def,
                                                         &path_delay_info_ptr,
                                                         PERSISTENT_DATA_PATH,
                                                         FIRST_TIME_CFG)))
   {
      AR_MSG(DBG_ERROR_PRIO, "apm_path_delay_event_hdlr(): Failed to configure data path");

      result = AR_EFAILED;

      goto __bailout_path_delay_evt_hdlr;
   }

   /** Check if there is at least 1 container pending send
    *  message, if not error out */
   if (!curr_cmd_ctrl_ptr->rsp_ctrl.num_pending_container)
   {
      AR_MSG(DBG_ERROR_PRIO, "apm_path_delay_event_hdlr(): No Containers to be processed");

      result = AR_EFAILED;

      goto __bailout_path_delay_evt_hdlr;
   }

   AR_MSG(DBG_MED_PRIO,
          "apm_path_delay_event_hdlr(): Performed DFS and cached container config for path ID: 0x%lX",
          path_delay_info_ptr->data_path.path_id);

   /** Add this node to APM global data path list */
   if (AR_EOK != (result = apm_db_add_node_to_list(&apm_info_ptr->graph_info.data_path_list_ptr,
                                                   path_delay_info_ptr,
                                                   &apm_info_ptr->graph_info.num_data_paths)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_path_delay_event_hdlr(): Failed to add event info node to the data path list, result: 0x%lX",
             result);

      goto __bailout_path_delay_evt_hdlr;
   }

   return result;

__bailout_path_delay_evt_hdlr:

   /** Free up the memory allocated for delay memory shmem list
    *  and path delay object */

   if (path_delay_info_ptr)
   {
      if (path_delay_info_ptr->delay_shmem_list_ptr)
      {
         posal_memory_free(path_delay_info_ptr->delay_shmem_list_ptr);
      }

      /** Free up the path delay info node */
      posal_memory_free(path_delay_info_ptr);
   }

   return result;
}

ar_result_t apm_graph_close_cmd_post_proc(apm_t *apm_info_ptr)
{
   ar_result_t     result = AR_EOK;
   apm_cmd_ctrl_t *apm_cmd_ctrl_ptr;

   apm_cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   /** Check if any of the data paths became invalid after Graph
    *  close command, if so inform the participating containers */
   if (apm_info_ptr->graph_info.data_path_list_ptr)
   {
      result = apm_update_dangling_data_paths_to_containers(apm_info_ptr);

      apm_cmd_ctrl_ptr->cmd_status |= result;

      /** If managed to send message to at least 1 container,
       *  return and wait for response */
      if (apm_cmd_ctrl_ptr->rsp_ctrl.cmd_rsp_pending)
      {
         /** Set the command pending status to true */
         apm_cmd_ctrl_ptr->cmd_pending = TRUE;

         result = AR_EOK;
      }
   }

   return result;
}

ar_result_t apm_update_get_data_path_cfg_rsp_payload(apm_t *                  apm_info_ptr,
                                                     spf_msg_header_t *       msg_hdr_ptr,
                                                     apm_module_param_data_t *cont_param_data_hdr_ptr)
{
   ar_result_t                     local_result = AR_EOK, result = AR_EOK;
   cntr_param_id_path_delay_cfg_t *path_delay_pid_payload_ptr;
   apm_path_defn_for_delay_t *     pid_path_dfn_offset_ptr;
   apm_data_path_delay_info_t *    delay_data_path_info_ptr;
   apm_module_param_data_t *       get_cfg_rsp_param_hdr_ptr = NULL;

   switch (cont_param_data_hdr_ptr->param_id)
   {
      case CNTR_PARAM_ID_PATH_DELAY_CFG:
      {
         path_delay_pid_payload_ptr = (cntr_param_id_path_delay_cfg_t *)(cont_param_data_hdr_ptr + 1);

         if (AR_EOK != (local_result = apm_db_get_data_path_obj(apm_info_ptr->graph_info.data_path_list_ptr,
                                                                path_delay_pid_payload_ptr->path_id,
                                                                &delay_data_path_info_ptr)))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "CNTR_PARAM_ID_PATH_DELAY_CFG: Failed to get delay obj for path id[%lu]",
                   path_delay_pid_payload_ptr->path_id);

            result |= local_result;

            break;
         }

         if (AR_EOK !=
             (local_result = apm_get_delay_path_rsp_payload_offset(apm_info_ptr,
                                                                   &delay_data_path_info_ptr->data_path.path_dfn,
                                                                   &get_cfg_rsp_param_hdr_ptr,
                                                                   &pid_path_dfn_offset_ptr)))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "CNTR_PARAM_ID_PATH_DELAY_CFG: Failed to get rsp payload offset for path id[%lu]",
                   path_delay_pid_payload_ptr->path_id);

            result |= local_result;

            break;
         }

         /** If the container returned success */
         if (AR_EOK == cont_param_data_hdr_ptr->error_code)
         {
            /** Accumulate the delay */
            pid_path_dfn_offset_ptr->delay_us += (*path_delay_pid_payload_ptr->delay_us_ptr);

            AR_MSG(DBG_HIGH_PRIO,
                   "CNTR PID PATH_DELAY_CFG, path_id[%lu], CONT_ID[0x%lX] "
                   "cont_path_delay[%lu], total path_delay[%lu], ",
                   path_delay_pid_payload_ptr->path_id,
                   cont_param_data_hdr_ptr->module_instance_id,
                   *path_delay_pid_payload_ptr->delay_us_ptr,
                   pid_path_dfn_offset_ptr->delay_us);

            get_cfg_rsp_param_hdr_ptr->error_code = AR_EOK;
         }
         else /** Aggregate the error code */
         {
            /** Aggregate the param ID based error code */
            get_cfg_rsp_param_hdr_ptr->error_code |= cont_param_data_hdr_ptr->error_code;

            /** Aggregate the container overall response */
            result |= msg_hdr_ptr->rsp_result;
         }

         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_parse_get_cfg_rsp_payload(): Un-supported param ID[0x%lX]",
                cont_param_data_hdr_ptr->param_id);

         result |= AR_EUNSUPPORTED;
         break;
      }

   } /** End of switch (param_data_hdr_ptr->param_id) */

   return result;
}

ar_result_t apm_data_path_config_cmn_sequencer(apm_t *apm_info_ptr, bool_t data_path_create)
{
   ar_result_t       result = AR_EOK;
   apm_cmd_ctrl_t *  cmd_ctrl_ptr;
   apm_cmd_seq_idx_t curr_seq_idx;
   apm_op_seq_t *    curr_op_seq_ptr;

   cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   /** Get the current op seq obj ptr   */
   curr_op_seq_ptr = cmd_ctrl_ptr->cmd_seq.curr_op_seq_ptr;

   /** This sequencer should get called only in the context of open
    *  and close commands.
    *  Check if any data paths are present to be processed  */
   if ((!curr_op_seq_ptr->curr_cmd_op_pending) &&
       (!apm_info_ptr->graph_info.data_path_list_ptr ||
        (APM_CMD_GRAPH_OPEN != cmd_ctrl_ptr->cmd_opcode && APM_CMD_GRAPH_CLOSE != cmd_ctrl_ptr->cmd_opcode)))
   {
      return AR_EOK;
   }

   if (APM_CMD_SEQ_IDX_INVALID == curr_op_seq_ptr->curr_seq_idx)
   {
      if (data_path_create)
      {
         curr_seq_idx = APM_SEQ_RECONFIG_DATA_PATHS;
      }
      else
      {
         curr_seq_idx = APM_SEQ_DESTROY_DATA_PATHS;
      }
   }
   else
   {
      curr_seq_idx = curr_op_seq_ptr->curr_seq_idx;
   }

   /** Init the seq index if not already initialized  */
   apm_init_next_cmd_op_seq_idx(curr_op_seq_ptr, curr_seq_idx);

   switch (curr_seq_idx)
   {
      case APM_SEQ_RECONFIG_DATA_PATHS:
      {
         if (AR_EOK == (result = apm_reconfigure_delay_data_path(apm_info_ptr)))
         {
            /** Check if the reconfiguration needs to be sent to atleast
             *  1 container depending upon new graph open. */
            if (cmd_ctrl_ptr->rsp_ctrl.num_pending_container)
            {
               /** Set the next seq idx */
               curr_op_seq_ptr->curr_seq_idx = APM_SEQ_SET_UP_CONT_MSG_SEQ;
            }
            else /** No commands need to be sent to containers */
            {
               /** Clear the current operation pending status */
               apm_clear_curr_cmd_op_pending_status(curr_op_seq_ptr);
            }
         }

         break;
      }
      case APM_SEQ_DESTROY_DATA_PATHS:
      {
         if (AR_EOK == (result = apm_update_dangling_data_paths_to_containers(apm_info_ptr)))
         {
            /** Check if the reconfiguration needs to be sent to atleast
             *  1 container depending upon new graph open. */
            if (cmd_ctrl_ptr->rsp_ctrl.num_pending_container)
            {
               /** Set the next seq idx */
               curr_op_seq_ptr->curr_seq_idx = APM_SEQ_SET_UP_CONT_MSG_SEQ;
            }
            else /** No commands need to be sent to containers */
            {
               /** Clear the current operation pending status */
               apm_clear_curr_cmd_op_pending_status(curr_op_seq_ptr);
            }
         }

         break;
      }
      case APM_SEQ_SET_UP_CONT_MSG_SEQ:
      case APM_SEQ_SEND_MSG_TO_CONTAINERS:
      case APM_SEQ_CONT_SEND_MSG_COMPLETED:
      {
         result = apm_cmd_cmn_sequencer(apm_info_ptr);
      }
      default:
      {
         break;
      }

   } /** End of switch (cmd_ctrl_ptr->cmd_seq.curr_seq_idx) */

   return result;
}

ar_result_t apm_data_path_clear_cached_cont_cfg_params(spf_list_node_t *param_node_ptr)
{
   ar_result_t                 result = AR_EOK;
   apm_cont_path_delay_cfg_t * cntr_path_delay_cfg_ptr;
   apm_cont_set_get_cfg_hdr_t *param_data_hdr_ptr;

   param_data_hdr_ptr = (apm_cont_set_get_cfg_hdr_t *)param_node_ptr->obj_ptr;

   switch (param_data_hdr_ptr->param_id)
   {
      case CNTR_PARAM_ID_PATH_DELAY_CFG:
      {
         cntr_path_delay_cfg_ptr = (apm_cont_path_delay_cfg_t *)param_node_ptr->obj_ptr;

         /** Delete the list of cached vertices */
         spf_list_delete_list(&cntr_path_delay_cfg_ptr->data_path.vertices_list_ptr, TRUE);

         break;
      }
      default:
      {
         break;
      }
   }

   return result;
}

ar_result_t apm_db_get_data_path_obj(spf_list_node_t *            data_path_list_ptr,
                                     uint32_t                     path_id,
                                     apm_data_path_delay_info_t **data_path_delay_info_pptr)
{
   ar_result_t                 result = AR_EFAILED;
   spf_list_node_t *           curr_node_ptr;
   apm_data_path_delay_info_t *delay_path_info_ptr;

   /** Init the return pointer */
   *data_path_delay_info_pptr = NULL;

   /** Get the pointer to the list of data paths */
   curr_node_ptr = data_path_list_ptr;

   /** Iterate over the list of data paths */
   while (curr_node_ptr)
   {
      /** Get the pointer to current data path object */
      delay_path_info_ptr = (apm_data_path_delay_info_t *)curr_node_ptr->obj_ptr;

      /** If the path ID matches with the query, return the object */
      if (delay_path_info_ptr->data_path.path_id == path_id)
      {
         /** Populate the return pointer */
         *data_path_delay_info_pptr = delay_path_info_ptr;

         result = AR_EOK;

         break;
      }

      /** Advance to next node in the list */
      curr_node_ptr = curr_node_ptr->next_ptr;
   }

   return result;
}

ar_result_t apm_process_get_cfg_offload_path_delay(apm_t *                  apm_info_ptr,
                                                   apm_module_param_data_t *get_cfg_rsp_payload_ptr)

{
   ar_result_t                              result = AR_EOK;
   uint32_t                                 num_data_paths;
   apm_offload_graph_path_defn_for_delay_t *curr_path_dfn_cmd_ptr;
   apm_module_t *                           module_node_ptr_list[2]   = { NULL };
   bool_t                                   DANGLING_LINK_NOT_ALLOWED = FALSE;

   apm_param_id_offload_graph_path_delay_t *path_delay_pid_rsp_ptr;
   spf_data_path_dfn_t                      data_path_dfn;
   apm_data_path_delay_info_t *             path_delay_info_ptr = NULL;
   bool_t                                   FIRST_TIME_CFG      = FALSE;

   /** Validate the input arguments. This is a special type of
    *  get param payload which is the combination of input and
    *  output arguments. In this case, input argument is the
    *  path information pre-filled by the client and output
    *  arugment is the delay pointer */

   /** Get the pointer to the start of param ID payload */
   path_delay_pid_rsp_ptr = (apm_param_id_offload_graph_path_delay_t *)(get_cfg_rsp_payload_ptr + 1);

   /** Update the number of data paths in response payload */
   num_data_paths = path_delay_pid_rsp_ptr->num_paths;

   /** Get the pointer to start of the path definition object
    *  array in cmd payload   */
   curr_path_dfn_cmd_ptr = (apm_offload_graph_path_defn_for_delay_t *)(path_delay_pid_rsp_ptr + 1);

   /** Iterate over the list of data paths being queried */
   while (num_data_paths)
   {
      if (AR_EOK != (result = apm_validate_module_instance_pair(&apm_info_ptr->graph_info,
                                                                curr_path_dfn_cmd_ptr->src_module_instance_id,
                                                                curr_path_dfn_cmd_ptr->dst_module_instance_id,
                                                                module_node_ptr_list,
                                                                DANGLING_LINK_NOT_ALLOWED)))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "APM_PARAM_ID_OFFLOAD_GRAPH_PATH_DELAY: Failed to validate path dfn, src: MIID[0x%lX]"
                "and/or dstn module: MIID[0x%lX] does not exist",
                curr_path_dfn_cmd_ptr->src_module_instance_id,
                curr_path_dfn_cmd_ptr->dst_module_instance_id);

         return result;
      }

      AR_MSG(DBG_HIGH_PRIO,
             "APM_PARAM_ID_OFFLOAD_GRAPH_PATH_DELAY: Rcvd path dfn src: m_iid[0x%lx], "
             "port_id[0x%lX], dstn: m_iid[0x%lx], port_id[0x%lX]",
             curr_path_dfn_cmd_ptr->src_module_instance_id,
             curr_path_dfn_cmd_ptr->src_port_id,
             curr_path_dfn_cmd_ptr->dst_module_instance_id,
             curr_path_dfn_cmd_ptr->dst_port_id);

      /** Populate the data path info  */
      data_path_dfn.src_module_instance_id = curr_path_dfn_cmd_ptr->src_module_instance_id;
      data_path_dfn.src_port_id            = curr_path_dfn_cmd_ptr->src_port_id;
      data_path_dfn.dst_module_instance_id = curr_path_dfn_cmd_ptr->dst_module_instance_id;
      data_path_dfn.dst_port_id            = curr_path_dfn_cmd_ptr->dst_port_id;

      /** Configure delay data path as per the data definition
       *  received */
      if (AR_EOK != (result = apm_configure_delay_data_path(apm_info_ptr,
                                                            &data_path_dfn,
                                                            &path_delay_info_ptr,
                                                            curr_path_dfn_cmd_ptr->is_client_query,
                                                            FIRST_TIME_CFG)))
      {
         AR_MSG(DBG_ERROR_PRIO, "APM_PARAM_ID_OFFLOAD_GRAPH_PATH_DELAY: Failed to configure data path");

         result = AR_EFAILED;

         return result;
      }

      /** Add this node to APM global data path list */
      if (AR_EOK != (result = apm_db_add_node_to_list(&apm_info_ptr->graph_info.data_path_list_ptr,
                                                      path_delay_info_ptr,
                                                      &apm_info_ptr->graph_info.num_data_paths)))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "APM_PARAM_ID_OFFLOAD_GRAPH_PATH_DELAY : Failed to add event info node to "
                "the data path list, result: 0x%lX",
                result);

         return result;
      }

      /** Connection information is cached within each container.
       *  DFS traversal output list can be freed up */
      spf_list_delete_list(&path_delay_info_ptr->data_path.vertices_list_ptr, TRUE);

      /** Clear the number of vertices */
      path_delay_info_ptr->data_path.num_vertices = 0;

      /** Update the port ID's in the command payload */
      curr_path_dfn_cmd_ptr->src_port_id = path_delay_info_ptr->data_path.path_dfn.src_port_id;
      curr_path_dfn_cmd_ptr->dst_port_id = path_delay_info_ptr->data_path.path_dfn.dst_port_id;
      /** Update the path in the command payload */
      curr_path_dfn_cmd_ptr->get_sat_path_id = path_delay_info_ptr->data_path.path_id;

      /** Clear the accumulated delay in response payload */
      curr_path_dfn_cmd_ptr->delay_us = 0;

      /** Increment the current path definition cmd payload pointer */
      curr_path_dfn_cmd_ptr++;

      /** Decrement number of data paths */
      num_data_paths--;

   } /** End of while (num_data_paths) */

   return result;
}

ar_result_t apm_process_get_cfg_path_delay(apm_t *apm_info_ptr, apm_module_param_data_t *mod_data_ptr)
{
   ar_result_t result = AR_EOK;

   switch (mod_data_ptr->param_id)
   {
      case APM_PARAM_ID_PATH_DELAY:
      {
         result = apm_process_get_cfg_path_delay_local(apm_info_ptr, mod_data_ptr);

         break;
      }
      case APM_PARAM_ID_OFFLOAD_GRAPH_PATH_DELAY:
      {
         result = apm_process_get_cfg_offload_path_delay(apm_info_ptr, mod_data_ptr);

         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_process_get_cfg_path_delay :: WARNING :: Un-supported PID: 0x%lX",
                mod_data_ptr->param_id);

         result = AR_EUNSUPPORTED;
         break;
      }
   }

   return result;
}

static ar_result_t apm_clear_src_module_output_port_conn(apm_t *apm_info_ptr, apm_module_t *src_module_node_ptr)
{
   ar_result_t             result = AR_EOK, local_result = AR_EOK;
   spf_list_node_t *       curr_out_port_conn_node_ptr;
   spf_list_node_t *       curr_in_port_conn_node_ptr, *next_in_port_conn_node_ptr;
   apm_module_data_link_t *mod_out_data_link_ptr;
   apm_module_data_link_t *mod_in_data_link_ptr;
   uint32_t                dstn_module_iid;
   apm_module_t *          dstn_module_node_ptr;
   spf_list_node_t         local_conn_node_to_remove;
   /** Get  the pointer to output port connnection list of the
    *  self module */
   curr_out_port_conn_node_ptr = src_module_node_ptr->output_data_link_list_ptr;
   /** For the self module as source, remove the input link from
    *  all the peer desination modules */
   while (curr_out_port_conn_node_ptr)
   {
      /** Get the pointer to connection object */
      mod_out_data_link_ptr = (apm_module_data_link_t *)curr_out_port_conn_node_ptr->obj_ptr;

      /** Get the instance ID of the peer module, destination module
       *  in case of output port */
      dstn_module_iid = mod_out_data_link_ptr->dstn_port.module_instance_id;

      /** Get the peer module node object */
      if (AR_EOK != (local_result = apm_db_get_module_node(&apm_info_ptr->graph_info,
                                                           dstn_module_iid,
                                                           &dstn_module_node_ptr,
                                                           APM_DB_OBJ_QUERY)))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_clear_src_module_output_port_conn(): Failed to get dstn module node with instance_id[0x%lX]",
                dstn_module_iid);

         /** Aggregate the overall result */
         result |= local_result;

         /** Advance to next node in the list */
         curr_out_port_conn_node_ptr = curr_out_port_conn_node_ptr->next_ptr;

         /** Skip current iteration */
         continue;
      }

      /** Check if the peer module node exists. The module may been
       *  closed while the link is being destroyed due to a prior
       *  sub-graph close */
      if (!dstn_module_node_ptr)
      {
         /** Advance to next node in the list */
         curr_out_port_conn_node_ptr = curr_out_port_conn_node_ptr->next_ptr;

         /** Skip current iteration */
         continue;
      }

      AR_MSG(DBG_HIGH_PRIO,
             "apm_clear_src_module_output_port_conn(): SRC [M_IID, SGID]: [0x%lX, 0x%lX], DSTN [M_IID, SGID]: [0x%lX, "
             "0x%lX]",
             src_module_node_ptr->instance_id,
             src_module_node_ptr->host_sub_graph_ptr->sub_graph_id,
             dstn_module_node_ptr->instance_id,
             dstn_module_node_ptr->host_sub_graph_ptr->sub_graph_id);

      /** Get the pointer to peer module's input connection list */
      curr_in_port_conn_node_ptr = dstn_module_node_ptr->input_data_link_list_ptr;

      /** Iterate over the list of the input port connection for
       *  peer module */
      while (curr_in_port_conn_node_ptr)
      {
         /** Get the peer module input port connection object */
         mod_in_data_link_ptr = (apm_module_data_link_t *)curr_in_port_conn_node_ptr->obj_ptr;

         /** Get pointer to next node in the list   */
         next_in_port_conn_node_ptr = curr_in_port_conn_node_ptr->next_ptr;

         /** If this object matches with the output port connection of
          *  the source module, delete this node from the input list
          *  of peer module */
         if (mod_in_data_link_ptr == mod_out_data_link_ptr)
         {
            /** clear the local list node */
            memset(&local_conn_node_to_remove, 0, sizeof(spf_list_node_t));

            /** Copy the obj pointer */
            local_conn_node_to_remove.obj_ptr = curr_in_port_conn_node_ptr->obj_ptr;

            /** Update the data paths */
            apm_update_data_path_list(apm_info_ptr,
                                      src_module_node_ptr,
                                      &local_conn_node_to_remove,
                                      TRUE /** Module Close*/);

            /** Remove current node from the list, update head pointer   */
            spf_list_find_delete_node(&dstn_module_node_ptr->input_data_link_list_ptr,
                                      curr_in_port_conn_node_ptr->obj_ptr,
                                      TRUE);

            /** Decrement the number of available input connections */
            dstn_module_node_ptr->num_input_data_link--;

            /** If the number of input data link for peer module reaches
             *  zero, set the list pointer to NULL */
            if (!dstn_module_node_ptr->num_input_data_link)
            {
               dstn_module_node_ptr->input_data_link_list_ptr = NULL;
            }
         }

         /** Advance to next node in the peer module input conn list */
         curr_in_port_conn_node_ptr = next_in_port_conn_node_ptr;

      } /** End of while(peer module input conn list) */

      /** Advance to next node in the output connection list */
      curr_out_port_conn_node_ptr = curr_out_port_conn_node_ptr->next_ptr;

   } /** End of while(self module output conn list) */

   /** Free up the output connection list for this module. Since
    *  source module owns the connection object memory, also
    *  frees up the allocated memory for connection object. */
   if (src_module_node_ptr->output_data_link_list_ptr)
   {
      /** Update the data paths */
      apm_update_data_path_list(apm_info_ptr,
                                src_module_node_ptr,
                                src_module_node_ptr->output_data_link_list_ptr,
                                TRUE /** Module Close*/);

      spf_list_delete_list_and_free_objs(&src_module_node_ptr->output_data_link_list_ptr, TRUE);

      /** Clear the number of output connections */
      src_module_node_ptr->num_output_data_link = 0;
   }

   return result;
}

static ar_result_t apm_clear_dstn_module_input_port_conn(apm_t *apm_info_ptr, apm_module_t *dstn_module_node_ptr)
{
   ar_result_t             result = AR_EOK, local_result = AR_EOK;
   spf_list_node_t *       curr_out_port_conn_node_ptr, *next_out_port_conn_node_ptr;
   spf_list_node_t *       curr_in_port_conn_node_ptr;
   apm_module_data_link_t *mod_out_data_link_ptr;
   apm_module_data_link_t *mod_in_data_link_ptr;
   uint32_t                src_module_iid;
   apm_module_t *          src_module_node_ptr;
   spf_list_node_t         local_conn_node_to_remove;
   //apm_ext_utils_t *       ext_utils_ptr;

   /** Get  the pointer to output port connnection list of the
    *  self module */
   curr_in_port_conn_node_ptr = dstn_module_node_ptr->input_data_link_list_ptr;

   /** For the self module as destination, remove the output links
    *  from all the peer source modules */
   while (curr_in_port_conn_node_ptr)
   {
      /** Get the pointer to connection object */
      mod_in_data_link_ptr = (apm_module_data_link_t *)curr_in_port_conn_node_ptr->obj_ptr;

      /** Get the instance ID of the source module, destination module
       *  in case of output port */
      src_module_iid = mod_in_data_link_ptr->src_port.module_instance_id;

      /** Get the peer module node object */
      if (AR_EOK != (local_result = apm_db_get_module_node(&apm_info_ptr->graph_info,
                                                           src_module_iid,
                                                           &src_module_node_ptr,
                                                           APM_DB_OBJ_QUERY)))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_clear_dstn_module_input_port_conn(): Failed to get src module node with instance_id[0x%lX]",
                src_module_iid);

         /** Aggregate the overall result */
         result |= local_result;

         /** Advance to next node in the list */
         curr_in_port_conn_node_ptr = curr_in_port_conn_node_ptr->next_ptr;

         /** Skip current iteration */
         continue;
      }

      /** Check if the peer source module node exists. The module may
       *  been closed while the link is being destroyed due to a prior
       *  sub-graph close */
      if (!src_module_node_ptr)
      {
         /** Advance to next node in the list */
         curr_in_port_conn_node_ptr = curr_in_port_conn_node_ptr->next_ptr;

         /** Skip current iteration */
         continue;
      }

      AR_MSG(DBG_MED_PRIO,
             "apm_clear_dstn_module_input_port_conn(): SRC [M_IID, SGID]: [0x%lX, 0x%lX], DSTN [M_IID, SGID]: "
             "[0x%lX, 0x%lX]",
             src_module_node_ptr->instance_id,
             src_module_node_ptr->host_sub_graph_ptr->sub_graph_id,
             dstn_module_node_ptr->instance_id,
             dstn_module_node_ptr->host_sub_graph_ptr->sub_graph_id);

      /** Get the pointer to source module's output connection list */
      curr_out_port_conn_node_ptr = src_module_node_ptr->output_data_link_list_ptr;

      /** Iterate over the list of the input port connection for
       *  peer module */
      while (curr_out_port_conn_node_ptr)
      {
         /** Get the src module output port connection object */
         mod_out_data_link_ptr = (apm_module_data_link_t *)curr_out_port_conn_node_ptr->obj_ptr;

         /** Get the pointer to next node in the list */
         next_out_port_conn_node_ptr = curr_out_port_conn_node_ptr->next_ptr;

         /** If this object matches with the input port connection of
          *  the destination module, delete this node from the output
          *  list of source module */
         if (mod_in_data_link_ptr == mod_out_data_link_ptr)
         {
            /** clear the local list node */
            memset(&local_conn_node_to_remove, 0, sizeof(spf_list_node_t));

            /** Copy the obj pointer */
            local_conn_node_to_remove.obj_ptr = curr_out_port_conn_node_ptr->obj_ptr;

            /** Update the data paths */
            apm_update_data_path_list(apm_info_ptr,
                                      dstn_module_node_ptr,
                                      &local_conn_node_to_remove,
                                      TRUE /** Module Close*/);

            /** Delete current node from the input list. This call also
             *  increments the current list pointer. Source module owns
             *  the connection memory, hence it is also freed */
            spf_list_delete_node_and_free_obj(&curr_out_port_conn_node_ptr,
                                              &src_module_node_ptr->output_data_link_list_ptr,
                                              TRUE);

            /** Decrement the number of available output connections */
            src_module_node_ptr->num_output_data_link--;
         }

         /** Advance to next node in the source module output conn
          *  list */
         curr_out_port_conn_node_ptr = next_out_port_conn_node_ptr;

      } /** End of while(source module output conn list) */

      /** Advance to next node in the input connection list */
      curr_in_port_conn_node_ptr = curr_in_port_conn_node_ptr->next_ptr;

   } /** End of while(destination module input conn list) */

   /** Free up the input connection list for this module.
    *  Connection object memory is freed in the source module's
    *  context. */
   if (dstn_module_node_ptr->input_data_link_list_ptr)
   {
      /** Update the data paths */
      apm_update_data_path_list(apm_info_ptr,
                                dstn_module_node_ptr,
                                dstn_module_node_ptr->input_data_link_list_ptr,
                                TRUE /** Module Close*/);

      /** Delete list */
      spf_list_delete_list(&dstn_module_node_ptr->input_data_link_list_ptr, TRUE);

      /** Clear the number of input connections */
      dstn_module_node_ptr->num_input_data_link = 0;
   }

   return result;
}

ar_result_t apm_clear_module_data_port_conn(apm_t *apm_info_ptr, apm_module_t *self_module_node_ptr)
{
   ar_result_t result = AR_EOK;

   if (!self_module_node_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "apm_clear_module_data_port_conn(): Module port pointer is NULL");

      return AR_EFAILED;
   }

   /** Free up the output connection list for this module. Since
    *  source module owns the connection object memory, also
    *  frees up the allocated memory for connection object. */
   if (self_module_node_ptr->output_data_link_list_ptr)
   {
      result = apm_clear_src_module_output_port_conn(apm_info_ptr, self_module_node_ptr);
   }

   /** Free up the input connection list for this module.
    *  Connection object memory is freed in the source module's
    *  context. */
   if (self_module_node_ptr->input_data_link_list_ptr)
   {
      result |= apm_clear_dstn_module_input_port_conn(apm_info_ptr, self_module_node_ptr);
   }

   return result;
}

ar_result_t apm_update_module_closed_state(apm_t *       apm_info_ptr,
                                           apm_module_t *src_module_obj_ptr,
                                           apm_module_t *dst_module_obj_ptr)
{
   ar_result_t                 result = AR_EOK;
   apm_cmd_ctrl_t *            cmd_ctrl_ptr;
   spf_list_node_t *           curr_node_ptr;
   apm_sub_graph_t *           sg_ob_ptr;
   bool_t                      is_src_module_close = FALSE, is_dst_module_close = FALSE;
   apm_data_path_delay_info_t *data_path_obj_ptr;

   if (!src_module_obj_ptr || !dst_module_obj_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_update_module_closed_state(): src[0x%lX] and/or dst[0x%lX] obj ptr is NULL",
             src_module_obj_ptr,
             dst_module_obj_ptr);

      return AR_EFAILED;
   }

   /** Get the pointer to cmd ctrl pointr for current command in
    *  process */
   cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   /** Pointer to the list of sub-graphs getting processed as
    *  part of current graph mgmt cmd step */
   curr_node_ptr = cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_proc_info.processed_sg_list_ptr;

   /** Iterate over the list of sub-graphs. If the source and
    *  destination modules are part of the list of sub-graphs
    *  getting closed, set a flag to keep track */
   while (curr_node_ptr)
   {
      sg_ob_ptr = (apm_sub_graph_t *)curr_node_ptr->obj_ptr;

      if (sg_ob_ptr->sub_graph_id == src_module_obj_ptr->host_sub_graph_ptr->sub_graph_id)
      {
         is_src_module_close = TRUE;
      }
      else if (sg_ob_ptr->sub_graph_id == dst_module_obj_ptr->host_sub_graph_ptr->sub_graph_id)
      {
         is_dst_module_close = TRUE;
      }

      /** Advance to next node in the list   */
      curr_node_ptr = curr_node_ptr->next_ptr;

   } /** End of while(graph mgmt cmd sg list being processsed) */

   /** Get the pointer to list of all the available data paths   */
   curr_node_ptr = apm_info_ptr->graph_info.data_path_list_ptr;

   /** Iterate over the list of data paths and update the src
    *  and/or dst module closed status as per the tracking done
    *  above */
   while (curr_node_ptr)
   {
      data_path_obj_ptr = (apm_data_path_delay_info_t *)curr_node_ptr->obj_ptr;

      /**  If src module is marked for close and it matches with
       *   data path src, then update the data path obj flag    */
      if (is_src_module_close &&
          (src_module_obj_ptr->instance_id == data_path_obj_ptr->data_path.path_dfn.src_module_instance_id))
      {
         data_path_obj_ptr->data_path.flags.src_module_closed = TRUE;
      }

      /**  If dst module is marked for close and it matches with
       *   data path dst, then update the data path obj flag    */
      if (is_dst_module_close &&
          (dst_module_obj_ptr->instance_id == data_path_obj_ptr->data_path.path_dfn.dst_module_instance_id))
      {
         data_path_obj_ptr->data_path.flags.dstn_module_closed = TRUE;
      }

      /** Advance to next node in the list   */
      curr_node_ptr = curr_node_ptr->next_ptr;

   } /** End of while (list of all the available data paths) */

   return result;
}

ar_result_t apm_clear_module_single_port_conn(apm_t *apm_info_ptr, spf_module_port_conn_t *module_port_conn_ptr)
{
   ar_result_t             result             = AR_EOK;
   apm_module_t *          src_module_obj_ptr = NULL, *dst_module_obj_ptr = NULL;
   spf_list_node_t *       curr_op_node_ptr, *curr_ip_node_ptr;
   spf_list_node_t *       next_op_node_ptr, *next_ip_node_ptr;
   apm_module_data_link_t *src_data_link_obj_ptr;
   spf_list_node_t         local_conn_node_to_remove;
   bool_t                  data_lnk_removed = FALSE;

   if (!module_port_conn_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "apm_clear_module_single_port_conn(): port conn ptr is NULL");

      return AR_EFAILED;
   }

   if (AR_EOK != (result = apm_db_get_module_node(&apm_info_ptr->graph_info,
                                                  module_port_conn_ptr->self_mod_port_hdl.module_inst_id,
                                                  &src_module_obj_ptr,
                                                  APM_DB_OBJ_QUERY)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_clear_module_single_port_conn(): Failed to get self module with MIID[0x%lX]",
             module_port_conn_ptr->self_mod_port_hdl.module_inst_id);

      return result;
   }

   if (AR_EOK != (result = apm_db_get_module_node(&apm_info_ptr->graph_info,
                                                  module_port_conn_ptr->peer_mod_port_hdl.module_inst_id,
                                                  &dst_module_obj_ptr,
                                                  APM_DB_OBJ_QUERY)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_clear_module_single_port_conn(): Failed to get peer module with MIID[0x%lX]",
             module_port_conn_ptr->peer_mod_port_hdl.module_inst_id);

      return result;
   }

   /** Additional validation to statisfy static analysis   */
   if (!src_module_obj_ptr || !dst_module_obj_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_clear_module_single_port_conn(): Self[0x%lX]/peer[0x%lX] module obj ptr is/are NULL",
             src_module_obj_ptr,
             dst_module_obj_ptr);

      return AR_EFAILED;
   }

   /** This routine should be called only in the source module's
    *  output port context */
   if (PORT_TYPE_DATA_OP != module_port_conn_ptr->self_mod_port_hdl.port_type)
   {
      AR_MSG(DBG_MED_PRIO,
             "apm_clear_module_single_port_conn(): Self module with MIID[0x%lX] is not src",
             module_port_conn_ptr->self_mod_port_hdl.module_inst_id);

      return AR_EOK;
   }

   /** Data path structure keeps track of source and/or
    *  destination module getting closed to take specific
    *  actions. Since this routine gets called in the context of
    *  container external ports, first determine if the host
    *  sub-graph of those modules are getting closed. */
   if (AR_EOK != (result = apm_update_module_closed_state(apm_info_ptr, src_module_obj_ptr, dst_module_obj_ptr)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_clear_module_single_port_conn(): Failed to update module closed state src miid[0x%lX], dst "
             "miid[0x%lX]",
             src_module_obj_ptr->instance_id,
             dst_module_obj_ptr->instance_id);

      return result;
   }

   /** Get the pointer to self module output data link list  */
   curr_op_node_ptr = src_module_obj_ptr->output_data_link_list_ptr;

   /** Iterate over the list of src module output data links   */
   while (curr_op_node_ptr)
   {
      /** Get next node pointer in the list   */
      next_op_node_ptr = curr_op_node_ptr->next_ptr;

      /** Get the pointer to current data link object   */
      src_data_link_obj_ptr = (apm_module_data_link_t *)curr_op_node_ptr->obj_ptr;

      /** If this link matches with the container output port
       *  handle being closed, then identify the peer modules input
       *  port and link object */
      if (src_data_link_obj_ptr->src_port.port_id == module_port_conn_ptr->self_mod_port_hdl.module_port_id)
      {
         /** Get the pointer to peer(dstn) module's input data link
          *  list */
         curr_ip_node_ptr = dst_module_obj_ptr->input_data_link_list_ptr;

         /** Iterate over the list of dstn module input data link list */
         while (curr_ip_node_ptr)
         {
            /** Get the pointer to next node in the list   */
            next_ip_node_ptr = curr_ip_node_ptr->next_ptr;

            /** Data link memory is shared across src and dstn modules.
             *  Obj pointer of data link node must be same across src and
             *  dstn */
            if (src_data_link_obj_ptr == curr_ip_node_ptr->obj_ptr)
            {
               /** First remvove this data port from data path book keeping */

               /** Clear the local variable   */
               memset(&local_conn_node_to_remove, 0, sizeof(spf_list_node_t));

               /** Copy the obj pointer */
               local_conn_node_to_remove.obj_ptr = curr_op_node_ptr->obj_ptr;

               /** Update the data paths to remove this vertex */
               apm_update_data_path_list(apm_info_ptr, src_module_obj_ptr, &local_conn_node_to_remove, FALSE);

               /** Delete this data link object from input data lins list of
                *  dst module. Obj memory is owned by the source. */
               spf_list_delete_node_update_head(&curr_ip_node_ptr, &dst_module_obj_ptr->input_data_link_list_ptr, TRUE);

               /**Decrement the number of input data links */
               dst_module_obj_ptr->num_input_data_link--;

               AR_MSG(DBG_MED_PRIO,
                      "apm_clear_module_single_port_conn(): Cleared IP [MIID, port_id]:[0x%lX, 0x%lX]",
                      module_port_conn_ptr->peer_mod_port_hdl.module_inst_id,
                      module_port_conn_ptr->peer_mod_port_hdl.module_port_id);

               /** Delete this node from the src module. Freeing up obj
                *  memory as well since it is owned by src module */
               spf_list_delete_node_and_free_obj(&curr_op_node_ptr,
                                                 &src_module_obj_ptr->output_data_link_list_ptr,
                                                 TRUE);

               /** Decrement the number of output data links */
               src_module_obj_ptr->num_output_data_link--;

               AR_MSG(DBG_MED_PRIO,
                      "apm_clear_module_single_port_conn(): Cleared OP [MIID, port_id]:[0x%lX, 0x%lX]",
                      module_port_conn_ptr->self_mod_port_hdl.module_inst_id,
                      module_port_conn_ptr->self_mod_port_hdl.module_port_id);

               data_lnk_removed = TRUE;

               /** Break the inner while loop  */
               break;
            }

            /** Advance to next node in the input data links of dst
             *  module */
            curr_ip_node_ptr = next_ip_node_ptr;

         } /** End of while (dstn module input data link list) */

      } /** End of if (src module op data link matches container port hdl getting closed)*/

      /** If data link removed, then break from the outer loop */
      if (data_lnk_removed)
      {
         break;
      }

      /** Advance to next node in the list */
      curr_op_node_ptr = next_op_node_ptr;

   } /** End of while (src module output data link list) */

   return result;
}

ar_result_t apm_data_path_utils_init(apm_t *apm_info_ptr)
{
   ar_result_t result = AR_EOK;

   apm_info_ptr->ext_utils.data_path_vtbl_ptr = &data_path_util_funcs;

   return result;
}
