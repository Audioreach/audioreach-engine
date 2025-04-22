#ifndef _IPC_DL_API_H_
#define _IPC_DL_API_H_

/**
 * @file  ipc_dl_api.h
 * @brief This file contains IPC APIs
 *
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */


/******************************************************************************
 * Includes                                                                    *
 *****************************************************************************/
#include "gpr_comdef.h"
#include "gpr_ids_domains.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/******************************************************************************
 * Defines                                                                   *
 *****************************************************************************/
typedef struct gpr_to_ipc_vtbl_t gpr_to_ipc_vtbl_t;

/** @addtogroup ipc_gpr_ipc_callbk_functions
To enable remote communication, both the GPR and IPC data link layers must
implement and exchange callback functions during initialization.
*/

/** @ingroup ipc_gpr_ipc_callbk_functions
  Table of callback functions exposed by the GPR to the IPC data link layers.
  The GPR sends this table to the data link layers during data link layer
  initialization.
*/
struct gpr_to_ipc_vtbl_t
{
    /**
     Prototype of the %receive() callback function for the GPR.

     @param[in] buf     Pointer to the packet.
     @param[in] length  Size of the packet.

     @detdesc
     When the data link layer receives a packet, it calls this GPR
     function to handle and route the packet to its final destination.

     @return
     #AR_EOK -- When successful.

     @dependencies
     None. @newpage
   */
   uint32_t (*receive)(void* buf, uint32_t length);

    /**
     Prototype of the %send_done() callback function for the GPR.

     @param[in] buf     Pointer to the packet.
     @param[in] length  Size of the packet.

     @detdesc
     When the data link layer finishes sending a packet, it uses this function
     to signal the GPR to free the packet.

     @return
     #AR_EOK -- When successful.

     @dependencies
     None.
   */
   uint32_t (*send_done)(void *buf, uint32_t length);
};

typedef struct ipc_to_gpr_vtbl_t ipc_to_gpr_vtbl_t;

/** @addtogroup ipc_ipc_gpr_func_wrapper
To enable remote communication, both the GPR and IPC data link layers must
implement and exchange callback functions during initialization time.
*/

/** @ingroup ipc_ipc_gpr_func_wrapper
  Table of functions exposed by data link layers to the GPR.
  The functions must be populated by every data link layer when gpr_init() calls
  for data link layer initialization.
*/
struct ipc_to_gpr_vtbl_t
{
    /**
     Prototype of the %send() function for a data link layer.

     @param[in] domain_id  ID of the domain to which packet is being sent.
     @param[in] buf        Pointer to the packet.
     @param[in] length     Size of the packet.

     @detdesc
     When the GPR sends a packet to a destination domain, it calls this
     function of the corresponding data link layer.

     @return
     #AR_EOK -- When successful.

     @dependencies
     None. @newpage
   */
    uint32_t (*send)(uint32_t domain_id, void *buf, uint32_t length);

   /**
     Prototype of the %receive_done() function for a data link layer.

     @param[in] domain_id  ID of the domain from which packet was sent.
     @param[in] buf        Pointer to the packet.

     @detdesc
     When the GPR finishes processing a packet it received from the data link
     layer, it returns the packet to the data link layer for it to free the
     buffer.

     @return
     #AR_EOK -- When successful.

     @dependencies
     None.
   */
    uint32_t (*receive_done)(uint32_t domain_id, void *buf);
};


/** @addtogroup ipc_dlink_function_protos
A data link layer is used for two-way communication between a given source
domain and a destination domain. The source domain can set up communication
with multiple destination domains through different data link layers.

This section outlines the function prototypes used for initialization and
de-initialization of any data link layer.
*/

/** @addtogroup ipc_dlink_function_protos
@{ */
/**
  Initializes a data link or IPC layer that communicates between two domains.

  @datatypes
  #gpr_to_ipc_vtbl_t \n
  #ipc_to_gpr_vtbl_t

  @param[in] src_domain_id        Domain ID of the source.
  @param[in] dest_domain_id       Domain ID of the destination with which the
                                  given src_domain_id is to establish
                                  communication.
  @param[in] p_gpr_to_ipc_vtbl    Pointer to the function table that contains
                                  the callback functions the GPR provides to
                                  each IPC data link layer during
                                  initialization.
                                  This function table argument is defined as a
                                  constant because the same GPR callback
                                  functions are sent to each data link layer.
  @param[out] pp_ipc_to_gpr_vtbl  Double pointer to the function table that
                                  contains the functions each IPC data link
                                  layer provides to the GPR. It is populated by
                                  each data link during initialization.

  @detdesc
  This function is called for each data link layer during GPR initialization. It
  sets up all the necessary function handlers to receive and send packets in the
  GPR and data link layers.
  @par
  The GPR registers callback functions to each data link layer, and in turn, a
  data link layer registers its callback functions with the GPR.

  @return
  #AR_EOK -- When successful.

  @dependencies
  None.

  @codeexample
  @lstlisting
#include "ipc_dl_api.h"

//Example of initializing a datalink layer to communicate between
// two domains - ADSP and APPS.

//Datalink layer entry point function.
GPR_EXTERNAL uint32_t ipc_datalink_init(uint32_t                 src_domain_id,
                                        uint32_t                 dest_domain_id,
                                        const gpr_to_ipc_vtbl_t *p_gpr_to_ipc_vtbl,
                                        ipc_to_gpr_vtbl_t **   pp_ipc_to_gpr_vtbl);

//Functions are defined in gpr.
static gpr_to_ipc_vtbl_t gpr_to_datalink_vtbl = {gpr_receive,
                                                 gpr_send_done,
                                                };

//Functions are defined inside the datalink layer and to be populated and
//sent back to gpr in the ipc_datalink_init function.
static ipc_to_gpr_vtbl_t datalink_layer_to_gpr_vtbl = {datalink_send,
                                                       datalink_receive_done,
                                                      };

int main()
{
   uint32_t src_domain_id = GPR_IDS_DOMAIN_ID_ADSP_V;
   uin32_t dest_domain_id = GPR_IDS_DOMAIN_ID_APPS_V;

   rc = ipc_datalink_init(src_domain_id,
                          dest_domain_id,
                          &gpr_to_datalink_vtbl,
                          &datalink_layer_to_gpr_vtbl);
   if (rc)
   {
      AR_MSG(DBG_ERROR_PRIO, "Initialization failed for destination domain id %lu", dest_domain_id);
   }
   return rc;
 }
  @endlstlisting @newpage
*/
typedef uint32_t (*ipc_dl_init_fn_t)(uint32_t src_domain_id,
                                    uint32_t dest_domain_id,
                                    const gpr_to_ipc_vtbl_t *p_gpr_to_ipc_vtbl,
                                    ipc_to_gpr_vtbl_t **pp_ipc_to_gpr_vtbl);

/**
  Deinitializes a data link or IPC layer that was set up to communicate between
  two domains.

  @param[in] src_domain_id   Domain ID of the source.
  @param[in] dest_domain_id  Domain ID of the destination with which the
                             src_domain_id must close communication.

  @detdesc
  This function is called for each data link layer during GPR deinitialization.

  @return
  #AR_EOK -- When successful.

  @dependencies
  None.

  @codeexample
  @lstlisting
#include "ipc_dl_api.h"

//Example of deinitializing a datalink layer to close communication between
//two domains - ADSP and APPS.

GPR_EXTERNAL uint32_t ipc_datalink_deinit (uint32_t src_domain_id,
                                           uint32_t dest_domain_id);

int main()
{
   uint32_t src_domain_id = GPR_IDS_DOMAIN_ID_ADSP_V;
   uin32_t dest_domain_id = GPR_IDS_DOMAIN_ID_APPS_V;

   rc = ipc_datalink_deinit(src_domain_id,dest_domain_id);
   if (rc)
   {
      AR_MSG(DBG_ERROR_PRIO, "Deinitialization failed for destination domain id %lu", dest_domain_id);
   }
   return rc;
}
  @endlstlisting
*/
typedef uint32_t (*ipc_dl_deinit_fn_t)(uint32_t src_domain_id, uint32_t dest_domain_id);

/** @} */ /* end_addtogroup ipc_dlink_function_protos */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif //_IPC_DL_API_H_
