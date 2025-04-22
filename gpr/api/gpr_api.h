#ifndef __GPR_API_H__
#define __GPR_API_H__

/**
 * @file  gpr_api.h
 * @brief This file contains GPR APIs
 *
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/******************************************************************************
 * Includes                                                                   *
 *****************************************************************************/
#include "gpr_packet.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/*****************************************************************************
 * Definitions                                                               *
 ****************************************************************************/

/* Deafult heap index - its an 8 Bit value */
#define GPR_HEAP_INDEX_DEFAULT 0

/* Heap index 1 - its an 8 Bit value */
#define GPR_HEAP_INDEX_1       1

/* GPR Heap index type definition
   Supported Values:
    1. GPR_HEAP_INDEX_DEFAULT
    2. GPR_HEAP_INDEX_1  */
typedef uint8_t gpr_heap_index_t;

/** @ingroup gpr_func_proto_callback
  Prototype of a packet callback function.

  @datatypes
  #gpr_packet_t

  @param[in] packet         Pointer to the incoming packet.
                            The packet is guaranteed to have a non-NULL value.
  @param[in] callback_data  Client-supplied data pointer that the service
                            provided at registration time.

  @return
  #AR_EOK -- When successful, indicates that the callee has taken ownership of
  the packet.
  @par
  Otherwise, an error (see @xref{hdr:StatusCodes}) -- The packet ownership is
  returned to the caller.
*/
typedef uint32_t (*gpr_callback_fn_t)(gpr_packet_t *packet, void *callback_data);

/* Structure to define the information related to a single packet pool. */
typedef struct gpr_packet_pool_info_v2_t gpr_packet_pool_info_v2_t;

struct gpr_packet_pool_info_v2_t
{
   gpr_heap_index_t heap_index;
   /* heap index from which packet pool has to be allocated.
        @valid_values
          0 - GPR_HEAP_INDEX_DEFAULT
          1 - GPR_HEAP_INDEX_1 */

   uint8_t is_dynamic;
   /*  Flag to indicate if the packets in the pool need to be allocated dynamically or statcially at init.*/

   uint16_t reserved;
    /* Reserved field for alignment, must be set to 0*/

   uint32_t num_packets;
   /* Max number of packets in the pool.

      If is_dynamic is set to 0: indicates number of packets to be statically allocated 
      If is_dynamic > 0: indicates maximum number of packets that can be dynamically allocated at any given time.
      */

   uint32_t packet_size;
   /* Size of each packet in the pool.*/
};

/*****************************************************************************
 * Core Routines                                                             *
 ****************************************************************************/

/** @addtogroup gpr_core_routines
@{ */

/**
  Performs external initialization of the GPR infrastructure.

  Each supported domain calls this function once during system bring-up or
  during runtime to initialize the GPR infrastructure for that domain. The GPR
  infrastructure must be initialized before any other GPR APIs can be called.

  @return
  #AR_EOK -- When successful.

  @dependencies
  None.

  @codeexample
  @lstlisting
#include "gpr_api.h"

int32_t rc = gpr_init();
if ( rc )
{
  printf( "Could not initialize the GPR infrastructure" );
}
  @endlstlisting
  @newpage
*/
GPR_EXTERNAL uint32_t gpr_init(void);

/**
  To initialize GPR with particular domain.

  @return
  #AR_EOK -- When successful.

  @dependencies
  None.
*/
GPR_EXTERNAL uint32_t gpr_init_domain(uint32_t domain_id);

/**
  Performs external deinitialization of the GPR infrastructure.

  Each supported domain calls this function once during system shutdown or
  during runtime to deinitialize the GPR infrastructure for that domain. No
  functions except for gpr_init() can be called after the GPR infrastructure is
  deinitialized.

  @return
  #AR_EOK -- When successful.

  @dependencies
  None.

  @codeexample
  @lstlisting
#include "gpr_api.h"

int32_t rc = gpr_deinit();
if ( rc )
{
  printf( "Could not deinitialize the GPR infrastructure" );
}
  @endlstlisting @newpage
*/
GPR_EXTERNAL uint32_t gpr_deinit(void);

/** @} */ /* end_addtogroup gpr_core_routines */


/*****************************************************************************
 * Fundamental Controls                                                      *
 ****************************************************************************/
/* Queries for the GPR's packet pool information. */

typedef struct gpr_cmd_gpr_packet_pool_info_t gpr_cmd_gpr_packet_pool_info_t;
/** @endcond */

/** @weakgroup weakb_gpr_cmd_gpr_packet_pool_info_t
@{ */
/** Contains the packet pool information for __gpr_cmd_get_gpr_packet_info().
*/
struct gpr_cmd_gpr_packet_pool_info_t
{
   uint32_t bytes_per_min_size_packet;
   /**< Minimum size (in bytes) of a GPR packet. */

   uint32_t num_min_size_packets;
   /**< Number of packets of the minimum size allocated at initialization. */

   uint32_t bytes_per_max_size_packet;
   /**< Maximum size (in bytes) of a GPR packet. */

   uint32_t num_max_size_packets;
   /**< Number of packets of the maximum size allocated at initialization. */
};
/** @} */ /* end_weakgroup weakb_gpr_cmd_gpr_packet_pool_info_t */


/*****************************************************************************
 * Utility Controls                                                          *
 ****************************************************************************/
/** @cond */
/* Allocates a preformatted free packet for delivery. */

typedef struct gpr_cmd_alloc_ext_t gpr_cmd_alloc_ext_t;
/** @endcond */

/** @weakgroup weakb_gpr_cmd_alloc_ext_t
@{ */
/** Contains the allocated packet information for __gpr_cmd_alloc_ext().
*/
struct gpr_cmd_alloc_ext_t
{
   uint8_t src_domain_id;     /**< Domain ID of the sender service. */
   uint32_t src_port;         /**< Registered unique ID of the sender service. */
   uint8_t dst_domain_id;     /**< Domain ID of the receiver service. */
   uint32_t dst_port;          /**< Registered unique ID of the receiver service. */
   uint8_t client_data;       /**< Reserved for use by client. */
   uint32_t token;            /**< Value attached by the sender to determine
                                   when command messages are processed by the
                                   receiver after they receive response messages. */
   uint32_t opcode;           /**< Defines both the action and the payload
                                   structure to the receiver. */
   uint32_t payload_size;     /**< Actual number of bytes required for the
                                   payload. */
   gpr_packet_t **ret_packet; /**< Double pointer to the formatted packet
                                   returned by the function. */
};
/** @} */ /* end_weakgroup weak_gpr_cmd_alloc_ext_t */

typedef struct gpr_cmd_alloc_ext_v2_t gpr_cmd_alloc_ext_v2_t;
/** @endcond */

/** @weakgroup weakb_gpr_cmd_alloc_ext_v2_t
@{ */
/** Contains the allocated packet information for __gpr_cmd_alloc_ext_v2().
*/
struct gpr_cmd_alloc_ext_v2_t
{
   uint8_t  src_domain_id;     /**< Domain ID of the sender service. */
   uint32_t src_port;          /**< Registered unique ID of the sender service. */
   uint8_t  dst_domain_id;     /**< Domain ID of the receiver service. */
   uint32_t dst_port;          /**< Registered unique ID of the receiver service. */
   uint8_t  client_data;       /**< Reserved for use by client. */
   uint32_t token;             /**< Value attached by the sender to determine
                                    when command messages are processed by the
                                    receiver after they receive response messages. */
   gpr_heap_index_t heap_index; /**< heap index of the packet pool. '0' is default value.
                                    Must be set to 0 if not applicable. */
   uint32_t opcode;             /**< Defines both the action and the payload
                                     structure to the receiver. */
   uint32_t payload_size;     /**< Actual number of bytes required for the
                                   payload. */
   gpr_packet_t **ret_packet; /**< Double pointer to the formatted packet
                                   returned by the function. */
};
/** @} */ /* end_weakgroup weakb_gpr_cmd_alloc_ext_v2_t */

/** @cond */
/* Allocates and sends a formatted packet. */

typedef struct gpr_cmd_alloc_send_t gpr_cmd_alloc_send_t;
/** @endcond */

/** @weakgroup weakb_gpr_cmd_alloc_send_t
@{ */
/** Contains the allocated packet information for __gpr_cmd_alloc_send().
*/
struct gpr_cmd_alloc_send_t
{
   uint8_t src_domain_id;  /**< Domain ID of the sender service. */
   uint32_t src_port;      /**< Registered unique ID of the sender service. */
   uint8_t dst_domain_id;  /**< Domain ID of the receiver service. */
   uint32_t dst_port;      /**< Registered unique ID of the receiver service. */
   uint8_t client_data;    /**< Reserved for use by the client. */
   uint32_t token;         /**< Value attached by the sender to determine when
                                command messages have been processed by the
                                receiver after having received response messages. */
   uint32_t opcode;        /**< Operation code defines both the action and the
                                payload structure to the receiver. */
   uint32_t payload_size;  /**< Actual number of bytes needed for the payload. */
   void *payload;          /**< Pointer to the payload to send. */
};
/** @} */ /* end_weakgroup weakb_gpr_cmd_alloc_send_t */


#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* __GPR_API_H__ */
