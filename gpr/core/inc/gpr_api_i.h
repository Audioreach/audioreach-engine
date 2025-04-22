#ifndef __GPR_API_I_H__
#define __GPR_API_I_H__

/**
 * @file  gpr_api_i.h
 * \brief This file contains GPR internal APIs
 *
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/******************************************************************************
 * Includes                                                                    *
 *****************************************************************************/
#include "gpr_api.h"
#include "ipc_dl_api.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/*****************************************************************************
 * Core Routines                                                             *
 ****************************************************************************/

/** @addtogroup gpr_core_routines
@{ */

/**
  Called from gpr_init() to initialize the GPR infrastructure.

  This function is defined in each platform wrapper( see Section
  @xref{mpg:PlatformSpecificCfgWrappers}) and further calls
  gpr_drv_internal_init() to perform GPR internal initialization.

  @return
  #AR_EOK -- When successful.

  @dependencies
  None.
 */
GPR_EXTERNAL uint32_t gpr_drv_init(void);

/**
  Called from gpr_init_domain(domain_id) to GPR infrastructure for
  particular domain.

  @return
  #AR_EOK -- When successful.

  @dependencies
  None.
*/
GPR_EXTERNAL uint32_t gpr_drv_init_domain(uint32_t domain_id);

/**
  Called from gpr_deinit() to deinitialize the GPR infrastructure.

  @return
  #AR_EOK -- When successful.

  @dependencies
  None.
*/
GPR_INTERNAL uint32_t gpr_drv_deinit(void);

/** @} */ /* end_addtogroup gpr_core_routines */

/*****************************************************************************
 * IPC Implementations                                                       *
 ****************************************************************************/

/* IPC datalink table structure*/
typedef struct ipc_dl_t ipc_dl_t;

/** @addtogroup ipc_platform_cfg_wrappers
The platform-specific configuration wrappers allow each platform to provide
specific configuration and perform GPR instantiation with this configuration.
GPR instantiation is done by calling gpr_drv_internal_init().

The platform wrapper provides control over the following aspects.
- Number and size of GPR packets to be allocated by the GPR instance on this
  platform. The GPR currently supports creation and maintenance of two such
  packet pools.
- Domains with which the current platform or domain wants to establish
  communication with and the corresponding data link layer to be used.
- This information is provided through the gpr_ipc_dl_table (see
  gpr_drv_internal_init()), which has multiple entries, each of type #ipc_dl_t.
- The gpr_ipc_dl_table has as many entries as the domain IDs with which the
  given processor is to establish communication.
- The maximum size of the table is the maximum number of processor domains that
  are present.
*/

/** @addtogroup ipc_platform_cfg_wrappers
@{ */
/**
  Contains the domain ID to which communication is to be established through
  a specified IPC data link layer. This structure includes the corresponding
  data link initialization and deinitialization functions to be called.
 */
struct ipc_dl_t
{
   uint16_t           domain_id;
   /**< Domain ID to establish communication with. */

   ipc_dl_init_fn_t   init_fn;
   /**< Initialization function in the IPC data link layer. */

   ipc_dl_deinit_fn_t deinit_fn;
   /**< Deinitialization function in the IPC data link layer. @newpagetable */
};

/* IPC datalink table structure*/
typedef struct ipc_dl_v2_t ipc_dl_v2_t;

/** @addtogroup ipc_platform_cfg_wrappers
The platform-specific configuration wrappers allow each platform to provide
specific configuration and perform GPR instantiation with this configuration.
GPR instantiation is done by calling gpr_drv_internal_init().

The platform wrapper provides control over the following aspects.
- Number and size of GPR packets to be allocated by the GPR instance on this
  platform. The GPR currently supports creation and maintenance of two such
  packet pools.
- Domains with which the current platform or domain wants to establish
  communication with and the corresponding data link layer to be used.
  And also indicates if shared memory is supported with the destination domain.
- This information is provided through the gpr_ipc_dl_table (see
  gpr_drv_internal_init()), which has multiple entries, each of type #ipc_dl_v2_t.
- The gpr_ipc_dl_table has as many entries as the domain IDs with which the
  given processor is to establish communication.
- The maximum size of the table is the maximum number of processor domains that
  are present.
*/

/** @addtogroup ipc_platform_cfg_wrappers
@{ */
/**
  Contains the domain ID to which communication is to be established through
  a specified IPC data link layer. This structure includes the corresponding
  data link initialization and deinitialization functions to be called.
 */
struct ipc_dl_v2_t
{
   uint16_t           domain_id;
   /**< Domain ID to establish communication with. */

   ipc_dl_init_fn_t   init_fn;
   /**< Initialization function in the IPC data link layer. */

   ipc_dl_deinit_fn_t deinit_fn;
   /**< Deinitialization function in the IPC data link layer. @newpagetable */

   bool_t             supports_shared_mem;
   /**< Indicates availability of shared memory with the destination domain. */
};

/**
  Performs internal initialization of the GPR infrastructure.

  @datatypes
  #ipc_dl_t

  @param[in] gpr_ipc_dl_table   Array of domain IDs to which communication is to
                                be established.
  @param[in] num_domains        Number of domain IDs in the array.
  @param[in] default_domain_id  Host domain ID.
  @param[in] buf_size_1         Size (in bytes) of each packet to be created in
                                the first packet pool.
  @param[in] num_packets_1      Number of packets of size buf_size_1 to be
                                created.
  @param[in] buf_size_2         Size (in bytes) of each packet to be created in
                                the second packet pool.
  @param[in] num_packets_2      Number of packets of size buf_size_2 to be
                                created.

  @detdesc
  This function must be called from all platform wrappers to perform GPR
  internal initialization, which sets up the entire GPR functionality.

  @return
  #AR_EOK -- When successful.

  @dependencies
  None.

  @codeexample
  @lstlisting
//Example platform wrapper for GPR in ADSP domain.

#include "ipc_dl_api.h"
#include "gpr_api_i.h"
#include "gpr_glink.h"
#include "gpr_qsocket.h"

GPR_EXTERNAL uint32_t ipc_dl_local_init(uint32_t                 src_domain_id,
                                        uint32_t                 dest_domain_id,
                                        const gpr_to_ipc_vtbl_t *p_gpr_to_ipc_vtbl,
                                        ipc_to_gpr_vtbl_t **   pp_ipc_to_gpr_vtbl);

GPR_EXTERNAL uint32_t ipc_dl_local_deinit(uint32_t src_domain_id, uint32_t dest_domain_id);

GPR_EXTERNAL uint32_t ipc_dl_glink_init(uint32_t                src_domain_id,
                                        uint32_t                 dest_domain_id,
                                        const gpr_to_ipc_vtbl_t *p_gpr_to_ipc_vtbl,
                                        ipc_to_gpr_vtbl_t **   pp_ipc_to_gpr_vtbl);

GPR_EXTERNAL uint32_t ipc_dl_glink_deinit(uint32_t src_domain_id,
                                           uint32_t dest_domain_id);

GPR_EXTERNAL uint32_t ipc_dl_qsocket_init(uint32_t                src_domain_id,
                                        uint32_t                 dest_domain_id,
                                        const gpr_to_ipc_vtbl_t *p_gpr_to_ipc_vtbl,
                                        ipc_to_gpr_vtbl_t **   pp_ipc_to_gpr_vtbl);

GPR_EXTERNAL uint32_t ipc_dl_qsocket_deinit(uint32_t src_domain_id,
                                           uint32_t dest_domain_id);

//This example creates the following gpr packet pools:
// 100 packets of size 512 bytes and 4 packets of size 4096 bytes.
#define GPR_NUM_PACKETS_1 ( 100 )
#define GPR_DRV_BYTES_PER_PACKET_1 ( 512 )
#define GPR_NUM_PACKETS_2 ( 4 )
#define GPR_DRV_BYTES_PER_PACKET_2 ( 4096 )

//In this example, ADSP wants to open communication with MODEM domain(through qsocket datalink layer),
//with APPS domain (through glink datalink layer) as well as
//set up local transmission of packets within the same domain.

//Thus the GPR IPC table has four entries, two for each domain and one for the local domain
//containing initialization, deinitialization functions for each of their datalink layers.
static struct ipc_dl_t  gpr_adsp_ipc_dl_table[GPR_PL_NUM_TOTAL_DOMAINS_V]= {
         {GPR_IDS_DOMAIN_ID_MODEM_V, ipc_dl_qsocket_init, ipc_dl_qsocket_deinit},
         {GPR_IDS_DOMAIN_ID_ADSP_V, ipc_dl_local_init, ipc_dl_local_deinit},
         {GPR_IDS_DOMAIN_ID_APPS_V, ipc_dl_glink_init, ipc_dl_glink_deinit}};

GPR_EXTERNAL uint32_t gpr_drv_init(void)
{
   uint32_t rc;
   uint32_t num_domains = sizeof(gpr_adsp_ipc_dl_table) / sizeof(ipc_dl_t);

   rc = gpr_drv_internal_init(gpr_adsp_ipc_dl_table,
                              num_domains,
                              GPR_IDS_DOMAIN_ID_ADSP_V, //host_domain_id
                              GPR_NUM_PACKETS_1,
                              GPR_DRV_BYTES_PER_PACKET_1,
                              GPR_NUM_PACKETS_2,
                              GPR_DRV_BYTES_PER_PACKET_2);

   if ( rc )
   {
      printf( "Could not initialize the GPR infrastructure and datalink layers");
   }
   return rc;
}
  @endlstlisting
  @newpage
*/
GPR_EXTERNAL uint32_t gpr_drv_internal_init(struct ipc_dl_t gpr_ipc_dl_table[],
                                            uint32_t        num_domains,
                                            uint32_t        default_domain_id,
                                            uint32_t        buf_size_1,
                                            uint32_t        num_packets_1,
                                            uint32_t        buf_size_2,
                                            uint32_t        num_packets_2);

/**
  Performs internal initialization of the GPR infrastructure.

  @datatypes
  #ipc_dl_t

  @param[in] default_domain_id     Host domain ID.
  @param[in] num_domains           Number of destiantion domain IDs that are supported.
  @param[in] gpr_ipc_dl_v2_table   Array of domain IDs specific info to which communication is to
                                   be established. Size of the array is specified in the previous agrument.
  @param[in] num_packet_pools      Number of packet pools that need to allocated.
  @param[in] packet_pool_info      List of packet pool info. Includes heap_index, if the packet pool
                                   needs to be statically or dynamically allocated, num packets and
                                   packet size of each of the pool to be allocated.

  @detdesc
  This function must be called from all platform wrappers to perform GPR
  internal initialization, which sets up the entire GPR functionality.

  @return
  #AR_EOK -- When successful.

  @dependencies
  None.

  @codeexample
  @lstlisting
//Example platform wrapper for GPR in ADSP domain.

#include "ipc_dl_api.h"
#include "gpr_api_i.h"
#include "gpr_glink.h"
#include "gpr_qsocket.h"

GPR_EXTERNAL uint32_t ipc_dl_local_init(uint32_t                 src_domain_id,
                                        uint32_t                 dest_domain_id,
                                        const gpr_to_ipc_vtbl_t *p_gpr_to_ipc_vtbl,
                                        ipc_to_gpr_vtbl_t **   pp_ipc_to_gpr_vtbl);

GPR_EXTERNAL uint32_t ipc_dl_local_deinit(uint32_t src_domain_id, uint32_t dest_domain_id);

GPR_EXTERNAL uint32_t ipc_dl_glink_init(uint32_t                src_domain_id,
                                        uint32_t                 dest_domain_id,
                                        const gpr_to_ipc_vtbl_t *p_gpr_to_ipc_vtbl,
                                        ipc_to_gpr_vtbl_t **   pp_ipc_to_gpr_vtbl);

GPR_EXTERNAL uint32_t ipc_dl_glink_deinit(uint32_t src_domain_id,
                                           uint32_t dest_domain_id);

GPR_EXTERNAL uint32_t ipc_dl_qsocket_init(uint32_t                src_domain_id,
                                        uint32_t                 dest_domain_id,
                                        const gpr_to_ipc_vtbl_t *p_gpr_to_ipc_vtbl,
                                        ipc_to_gpr_vtbl_t **   pp_ipc_to_gpr_vtbl);

GPR_EXTERNAL uint32_t ipc_dl_qsocket_deinit(uint32_t src_domain_id,
                                           uint32_t dest_domain_id);

//This example creates the following gpr packet pools:
// 100 packets of size 512 bytes and 4 packets of size 4096 bytes, and these
// two pools are allocated statically. So set is_dynamic to FALSE.
#define GPR_NUM_PACKET_POOLS 2
#define GPR_NUM_PACKETS_1 ( 100 )
#define GPR_DRV_BYTES_PER_PACKET_1 ( 512 )
#define GPR_NUM_PACKETS_2 ( 4 )
#define GPR_DRV_BYTES_PER_PACKET_2 ( 4096 )

//In this example, ADSP wants to open communication with MODEM domain(through qsocket datalink layer),
//with APPS domain (through glink datalink layer) as well as
//set up local transmission of packets within the same domain.

//Thus the GPR IPC table has four entries, two for each domain and one for the local domain
//containing initialization, deinitialization functions for each of their datalink layers.
// Since all the domains support shared memory in this case, set support_shared_mem to TRUE.
static struct ipc_dl_v2_t  gpr_adsp_ipc_dl_v2_table[GPR_PL_NUM_TOTAL_DOMAINS_V]= {
         {GPR_IDS_DOMAIN_ID_MODEM_V, ipc_dl_qsocket_init, ipc_dl_qsocket_deinit, TRUE},
         {GPR_IDS_DOMAIN_ID_ADSP_V,  ipc_dl_local_init,   ipc_dl_local_deinit,   TRUE},
         {GPR_IDS_DOMAIN_ID_APPS_V,  ipc_dl_glink_init,   ipc_dl_glink_deinit,   TRUE}};

GPR_EXTERNAL uint32_t gpr_drv_init(void)
{
   uint32_t rc;
   uint32_t num_domains = sizeof(gpr_adsp_ipc_dl_v2_table) / sizeof(ipc_dl_v2_t);

    gpr_packet_pool_info_v2_t packet_pool_info[GPR_NUM_PACKET_POOLS];

    // init pool info
    packet_pool_info[0].heap_index = GPR_HEAP_INDEX_DEFAULT;
    packet_pool_info[0].is_dynamic = FALSE;
    packet_pool_info[0].num_packets =GPR_NUM_PACKETS_1;
    packet_pool_info[0].packet_size =GPR_DRV_BYTES_PER_PACKET_1;
    packet_pool_info[0].reserved = 0;

    packet_pool_info[1].heap_index = GPR_HEAP_INDEX_DEFAULT;
    packet_pool_info[1].is_dynamic = FALSE;
    packet_pool_info[1].num_packets =GPR_NUM_PACKETS_2;
    packet_pool_info[1].packet_size =GPR_DRV_BYTES_PER_PACKET_2;
    packet_pool_info[1].reserved = 0;

   rc = gpr_drv_internal_init_v2(GPR_IDS_DOMAIN_ID_ADSP_V, //host_domain_id
                                 num_domains,
                                 gpr_adsp_ipc_dl_v2_table,
                                 GPR_NUM_PACKET_POOLS,
                                 &packet_pool_info[0]);

   if ( rc )
   {
      printf( "Could not initialize the GPR infrastructure and datalink layers");
   }
   return rc;
}
  @endlstlisting
  @newpage
*/
GPR_EXTERNAL uint32_t gpr_drv_internal_init_v2(uint32_t                  default_domain_id,
                                               uint32_t                  num_domains,
                                               struct ipc_dl_v2_t        gpr_ipc_dl_v2_table[],
                                               uint32_t                  num_packet_pools,
                                               gpr_packet_pool_info_v2_t packet_pool_info[]);

/** @} */ /* end_addtogroup ipc_platform_cfg_wrappers */

/** @addtogroup ipc_gpr_function_loc_rout
GPR functions used for local routing, which is routing between two services on
the same domain, should also follow IPC data link function prototype (see
Section @xref{mpg:IpcDlinkFunctionProtos}).
*/

/** @addtogroup ipc_gpr_function_loc_rout
@{ */
/**
  Performs initialization of the GPR functionality used for local routing of
  packets.

  @datatypes
  #gpr_to_ipc_vtbl_t \n
  #ipc_to_gpr_vtbl_t

  @param[in] src_domain_id       Domain ID for the source.
  @param[in] dest_domain_id      Domain ID for the destination.
  @param[in] p_gpr_to_ipc_vtbl   Pointer to the GPR-to-IPC virtual function
                                 table.
  @param[in] pp_ipc_to_gpr_vtbl  Double pointer to the IPC-to-GPR virtual
                                 function table.

  @detdesc
  This function is of type ipc_dl_init_fn_t(). It sets up the GPR for
  transferring and receiving packets within the same domain or processor, when
  there is no need for a data link layer.
  @par
  This function must be specified as the initialization function for the local
  or host domain in the array of type ipc_dl_t in the platform-specific
  configuration wrappers (see Section @xref{mpg:PlatformSpecificCfgWrappers}).

  @return
  #AR_EOK -- When successful.

  @dependencies
  None.

  @codeexample
  See the code example for gpr_drv_internal_init(). @newpage
*/
GPR_EXTERNAL uint32_t ipc_dl_local_init(uint32_t                 src_domain_id,
                                        uint32_t                 dest_domain_id,
                                        const gpr_to_ipc_vtbl_t *p_gpr_to_ipc_vtbl,
                                        ipc_to_gpr_vtbl_t      **pp_ipc_to_gpr_vtbl);

/**
  Performs deinitialization of the GPR functionality used for local routing of
  packets.

  @param[in] src_domain_id   Domain ID for the source.
  @param[in] dest_domain_id  Domain ID for the destination.

  @detdesc
  This function is of type ipc_dl_deinit_fn_t(). It deinitializes the GPR for
  transferring and receiving packets within the same domain or processor.
  @par
  This function must be specified as the initialization function for the local
  or host domain in the array of type ipc_dl_t in the platform-specific
  configuration wrappers (see Section @xref{mpg:PlatformSpecificCfgWrappers}).

  @return
  #AR_EOK -- When successful.

  @dependencies
  None.

  @codeexample
  See the code example for gpr_drv_internal_init().
*/
GPR_EXTERNAL uint32_t ipc_dl_local_deinit(uint32_t src_domain_id, uint32_t dest_domain_id);

/** @} */ /* end_addtogroup ipc_gpr_function_loc_rout */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* __GPR_API_I_H__ */
