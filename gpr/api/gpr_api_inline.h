#ifndef __GPR_API_INLINE_H__
#define __GPR_API_INLINE_H__

/**
 * @file gpr_api_inline.h
 * @brief This file contains GPR APIs
 *
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/******************************************************************************
 * Includes                                                                    *
 *****************************************************************************/
#include "ar_osal_error.h"
#include "gpr_api.h"
#include "gpr_msg_if.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

//#define GPR_DEBUG_MSG
/*****************************************************************************
 * Inline Fundamental Controls                                               *
 ****************************************************************************/

/** @ingroup gpr_cmd_register
  Registers a service, by its unique service ID, with the GPR.

  @datatypes
  #gpr_callback_fn_t

  @param[in] src_port       Unique ID (within a domain) of the service to be
                            registered.
  @param[in] callback_fn    Callback function of the service to be registered.
  @param[in] callback_data  Pointer to the client-supplied data pointer for the
                            callback function.

  @detdesc
  Services call this command once during system bring-up or runtime to register
  their presence with the GPR.
  @par
  A service is any functional block, such as a module, that is required to send
  or receive GPR packets. Services must be registered with the GPR before they
  can send or receive any messages.

  @return
  #AR_EOK -- When successful.
  @par
  #AR_EALREADY -- Service has already been registered.

  @dependencies
  GPR initialization must be completed via gpr_init(). @newpage

  @codeexample
  @lstlisting
#include "gpr_api_inline.h"

//Example of a test client service (with service ID GPR_TESTCLIENT_SERVICE_ID)
//trying to register with GPR.
uint32_t service_callback_fn(gpr_packet_t *packet, void *callback_data);
void * callback_data = NULL;

int main ( void )
{
  int32_t rc = __gpr_cmd_register(GPR_TESTCLIENT_SERVICE_ID,
                                  callback_fn,
                                  callback_data);
  if ( rc )
  {
    printf( "Could not register the test client service with GPR" );
  }
  return 0;
}

//Example of a callback function for a service. It is invoked by GPR
//every time a message is sent or received, to or from that service.
static int32_t service_callback_fn( gpr_packet_t* packet,
                                    void* callback_data )
{
   // Accept command by replying to the sender that the message is accepted.
   // The usage is optional.
   __gpr_cmd_accept_command( packet );

   switch ( packet->opcode )
   {
     case TEST_CLIENT_CMD_FUNCTION:
     {
       // Handle accordingly based on operation code and send response to
       // the sender that command is completed.
       __gpr_cmd_end_command( packet, AR_EOK );
       break;
     }
     case TEST_CLIENT_RSP_FUNCTION:
     {
       // Notify command completion.
       // Response may contain payload, free the packet after handling.
       __gpr_cmd_free( packet );
       break;
      }
     case GPR_IBASIC_RSP_RESULT:
     {
       // Notify command completion, response contains the error status.
       __gpr_cmd_free( packet );
       break;
     }
     default:
     {
       // Free unsupported events and command responses.
       __gpr_cmd_free( packet );
       break;
     }
   }
   return AR_EOK;
   // AR_EOK tells the caller that the packet was consumed (freed).
}
  @endlstlisting
 */
uint32_t __gpr_cmd_register(uint32_t src_port, gpr_callback_fn_t callback_fn, void *callback_data);

/** @ingroup gpr_cmd_deregister
  Deregisters a service from the GPR.

  @param[in] src_port  Unique ID of the service.

  @detdesc
  Services call this function once during system teardown or runtime to
  deregister their presence from the GPR.

  @return
  #AR_EOK -- When successful.

  @dependencies
  GPR initialization must be completed via gpr_init().

  @codeexample
  @lstlisting
#include "gpr_api_inline.h"

//Example of a test client service (with service ID GPR_TESTCLIENT_SERVICE_ID)
//trying to deregister from GPR.
uint32_t callback_fn(gpr_packet_t *packet, void *callback_data);
void * callback_data = NULL;

int32_t rc = __gpr_cmd_register(GPR_TESTCLIENT_SERVICE_ID,
                                callback_fn,
                                callback_data);
if ( rc )
{
  printf( "Could not register the client service with GPR" );
}
...
rc =  __gpr_cmd_deregister(GPR_TESTCLIENT_SERVICE_ID);
if ( rc )
{
  printf( "Could not deregister the client service from GPR" );
}
  @endlstlisting
 */
uint32_t __gpr_cmd_deregister(uint32_t src_port);

/** @ingroup gpr_cmd_is_registered
  Called by the framework to check whether a service is registered with the GPR.

  @param[in]  port            Unique ID of the service.
  @param[out] is_registered   Pointer to the client-supplied flag that returns
                              TRUE if service is registered and FALSE if the
                              service is not registered.

  @return
  #AR_EOK.

  @dependencies
  GPR initialization must be completed via gpr_init().

  @codeexample
  @lstlisting
#include "gpr_api_inline.h"

//Example to check if test client service (with service ID GPR_TESTCLIENT_SERVICE_ID)
// is registered with GPR.
bool_t is_registered = FALSE;
__gpr_cmd_is_registered(GPR_TESTCLIENT_SERVICE_ID, &is_registered);
if(TRUE == is_registered)
{
   printf( "Client service registered with GPR" );
}
else
{
   printf( "Client service not registered with GPR" );
}
  @endlstlisting @newpage
 */
uint32_t __gpr_cmd_is_registered(uint32_t port, bool_t *is_registered);

/** @ingroup gpr_cmd_get_host_dom_id
  Queries the GPR to get the local or host domain ID.

  @param[out] host_domain_id  Pointer to the GPR's host domain ID.

  @return
  #AR_EOK always.- Returns the host domain ID

  @dependencies
  GPR initialization must be completed via gpr_init().

  @codeexample
  @lstlisting
#include "gpr_api_inline.h"

uint32_t host_domain_id;
__gpr_cmd_host_domain_id( &host_domain_id );
printf( "GPR is in domain ID: %d ", host_domain_id );
  @endlstlisting
 */
uint32_t __gpr_cmd_get_host_domain_id(uint32_t *host_domain_id);

/** @ingroup gpr_cmd_get_dest_domain_ids
Utility to query list of destination domain ids with which GPR communication is supported from the current domain.

@param[out] num_domains - Pointer to the num of destination domains.
@param[out] domain_ids  - Pointer to the GPR's destination domain IDs.

@return
#AR_EOK always.- Returns the number of destination domains on requesting first time.
#AR_EOK always.- When called again with memory allocated for domain array, fills destination domain ids info.

@dependencies
GPR initialization must be completed via gpr_init().

@codeexample
@lstlisting

#include "gpr_api_inline.h"

uint32_t num_domains = 0;
uint32_t *dest_domains = NULL;
__gpr_cmd_get_dest_domain_ids(&num_domains, dest_domains);
dest_domains = (uint32_t *)malloc(sizeof(num_domains*sizeof(uint32_t)));
__gpr_cmd_get_dest_domain_ids(&num_domains, dest_domains);

 @endlstlisting
*/
uint32_t __gpr_cmd_get_dest_domain_ids(uint32_t *num_domains, uint32_t *domain_ids);

/** @ingroup gpr_cmd_is_shared_mem_supported
  Utility to query if shared memory is supported from the current domain to a given dest_domain_id.

 @param[in]  dest_domain_id              -  Destination domain id.
 @param[out] supports_shared_mem   -  Pointer to know if domain supports shared memory

 @return
 #AR_EOK always.- Returns if a destination domain supports shared memory.

 @dependencies
 GPR initialization must be completed via gpr_init().

 @codeexample
 @lstlisting
  #include "gpr_api_inline.h"

  uint32_t dest_domain_id = GPR_IDS_DOMAIN_ID_ADSP_V;
  bool_t supports_shared_mem = true;
  __gpr_cmd_is_shared_mem_supported(dest_domain_id, &supports_shared_mem);
  if (supports_shared_mem)
  // do something
  else
  // do something
   @endlstlisting
*/
uint32_t __gpr_cmd_is_shared_mem_supported(uint32_t dest_domain_id, bool_t *supports_shared_mem);

/** @ingroup gpr_cmd_get_pkt_pool
  Queries for the GPR's packet pool information.

  @datatypes
  #gpr_cmd_gpr_packet_pool_info_t

  @param[out] args  Pointer to the packet pool information, such as the number
                    and sizes of the packets.

  @detdesc
  This function returns following information:
  - Minimum and maximum sizes (in bytes) of the packets allocated by the GPR.
  - Maximum number of each packet.
  @par
  The number of packets does not translate to the packets that are currently
  available. The number is the total or maximum number of packets allocated
  during initialization.
  @par
  Clients can use this information to decide whether to send inband or
  out-of-band commands.

  @return
  #AR_EOK -- Returns the GPR packet pool information.

  @dependencies
  GPR initialization must be completed via gpr_init().

  @codeexample
  @lstlisting
#include "gpr_api_inline.h"

gpr_cmd_gpr_packet_pool_info_t packet_info;
__gpr_cmd_get_gpr_packet_info( &packet_info );

printf( "GPR packet pool information:
         Bytes in minimum-sized gpr packet: %d,
         Number of minimum-sized gpr packets allocated at initialization: %d,
         Bytes in maximum-sized gpr packet: %d,
         Number of maximum-sized gpr packets allocated at initialization: %d ",
         packet_info.bytes_per_min_size_packet,
         packet_info.num_min_size_packets,
         packet_info.bytes_per_max_size_packet,
         packet_info.num_max_size_packets);
  @endlstlisting

  @inputfile{group__weakb__gpr__cmd__gpr__packet__pool__info__t.tex}
 */
uint32_t __gpr_cmd_get_gpr_packet_info(gpr_cmd_gpr_packet_pool_info_t *args);

/** @ingroup gpr_cmd_get_pkt_pool_v2
  Queries for the GPR's V2 packet pool information.

  @datatypes
  #gpr_packet_pool_info_v2_t

  @param[out] num_packet_pools       Number of packet pools that are created.
  @param[out] packet_pool_info_arr   Packet pool info array, with each element in array containing pool related info like
                                     number of packets, packet size, heap index and if the packets are dynamically/statically
                                     allocated.

  @detdesc
    1.	Number of packet pools present
    2.	Arrays of packet pool info. Each element of the array has info related to the pool like
          •	Size(in bytes) of each packet in the pool
          •	Max number of packets in the pool.
          •	Heap index of the packet pool.
          •	IS_DYNAMIC – flag indicating if the packet in the pool are created dynamically or statically at init.

  @par
  The number of packets in each pool does not translate to the packets that are currently
  available. The number is the total or maximum number of packets allocated during initialization.

  @par
  Clients can use this information to decide whether to send inband or
  out-of-band commands.

  @return
  #AR_EOK always.- Returns the number of packet pools on requesting first time.
  #AR_EOK always.- When called again with memory allocated for Packet pool info array, fills the array elements with the pool info.

  @dependencies
  GPR initialization must be completed via gpr_init().

  @codeexample
  @lstlisting
#include "gpr_api_inline.h"

gpr_packet_pool_info_v2_t *packet_pool_info_arr  = NULL;
uint32_t                        num_packet_pools = 0;

// Calling first time, to gets number of packet pools and allocates the array accordingly.
__gpr_cmd_get_gpr_packet_info_v2( &num_packet_pools, packet_pool_info_arr );
packet_pool_info_arr = (gpr_packet_pool_info_v2_t *)malloc(sizeof(num_packet_pools*sizeof(gpr_packet_pool_info_v2_t)));

// Calling second time to the get the populated packet_pool_info_arr
__gpr_cmd_get_gpr_packet_info_v2( &num_packet_pools, packet_pool_info_arr );

  @endlstlisting

  @inputfile{group__weakb__gpr__cmd__gpr__packet__pool__info__v2__t.tex}
 */
uint32_t __gpr_cmd_get_gpr_packet_info_v2(uint32_t *num_packet_pools, gpr_packet_pool_info_v2_t *packet_pool_info_arr);

/** @ingroup gpr_cmd_send_async
  Sends an asynchronous message to other services.

  @datatypes
  #gpr_packet_t

  @param[in] packet  Pointer to the packet (message) to send.

  @detdesc
  This function provides the caller with low-level control over the sending
  process. For general use, consider using a simplified helper function, such as
  __gpr_cmd_alloc_send().
  @par
  Before calling this function, use __gpr_cmd_alloc() or __gpr_cmd_alloc_ext()
  to allocate free messages for sending.  If delivery fails, the caller must
  either try to resend the messages or abort-and-free the messages

  @par Notes
  The sender must always anticipate failures, even when this function returns no
  errors. The #GPR_IBASIC_RSP_RESULT response messages are to be checked for
  any error statuses that are returned.
  @par
  The sender can locally abort any remotely pending operations by implementing
  timeouts. The sender must still expect and handle receipt of response messages
  for those aborted operations.

  @return
  #AR_EOK -- When successful.

  @dependencies
  GPR initialization must be completed via gpr_init().
  @par
  The source and destination services must be registered with the GPR.

  @codeexample
  @lstlisting
#include "gpr_api_inline.h"

int32_t rc;
gpr_packet_t* packet_ptr;
uint32_t payload_size;
uint32_t packet_size;

//Example of a payload structure that needs to be populated and sent.
test_client_cmd_function_t payload;
payload_size = sizeof( test_client_cmd_function_t );
packet_size = payload_size + GPR_PKT_HEADER_WORD_SIZE_V;

// Allocate a free packet.
rc = __gpr_cmd_alloc( payload_size, &packet );
if ( rc )
{
   return AR_ENORESOURCE;
}

// Fill in the packet details.
packet->header GPR_SET_FIELD(GPR_PKT_VERSION, GPR_PKT_VERSION_V) |
               GPR_SET_FIELD(GPR_PKT_HEADER_SIZE, GPR_PKT_HEADER_WORD_SIZE_V) |
               GPR_SET_FIELD(GPR_PKT_PACKET_SIZE, packet_size);
packet->dst_domain = GPR_CLIENT_SERVICE_DOMAIN_ID_DESTINATION;
packet->src_domain = GPR_CLIENT_SERVICE_DOMAIN_ID_SOURCE;
packet->dst_port = GPR_CLIENT_SERVICE_PORT_ID_DESTINATION;
packet->src_port = GPR_CLIENT_SERVICE_PORT_ID_SOURCE;
packet->token = 0x12345678;
packet->opcode = TEST_CLIENT_CMD_FUNCTION;

// Fill in the payload.
payload.param1 = 1;
payload.param2 = 2;
memscpy( GPR_PKT_GET_PAYLOAD( void, packet ), payload_size, payload, payload_size );

// Send the packet.
rc = __gpr_cmd_async_send( packet_ptr );
if ( rc )
{
  // Free the packet when delivery fails.
  ( void ) __gpr_cmd_free( packet_ptr );
  return rc;
}
  @endlstlisting
 */
uint32_t __gpr_cmd_async_send(gpr_packet_t *packet);

/** @ingroup gpr_cmd_alloc
  Allocates a free message for delivery.

  @datatypes
  #gpr_packet_t

  @param[in]  alloc_size  Amount of memory (in bytes) required for allocation.
  @param[out] ret_packet  Double pointer to the allocated packet that is
                          returned by this function.

  @detdesc
  This function allocates a packet from the GPR's free packet queue. It provides
  the caller with low-level control over the allocation process.
  @par
  For general use, consider using a simplified helper function, such as
  __gpr_cmd_alloc_ext().

  @return
  #AR_EOK -- When successful.

  @dependencies
  GPR initialization must be completed via gpr_init().

  @codeexample
  See the code example for __gpr_cmd_async_send().
 */
uint32_t __gpr_cmd_alloc(uint32_t alloc_size, gpr_packet_t **ret_packet);

/** @ingroup gpr_cmd_alloc
  Allocates a free message for delivery from a specified heap.

  @datatypes
  #gpr_packet_t

  @param[in]  alloc_size  Amount of memory (in bytes) required for allocation.
  @param[in]  heap_index  Heap index of the packet pool.
  @param[out] ret_packet  Double pointer to the allocated packet that is
                          returned by this function.

  @detdesc
  This function allocates a packet from the GPR's free packet queue from the specified heap.
  It provides the caller with low-level control over the allocation process.
  @par
  For general use, consider using a simplified helper function, such as
  __gpr_cmd_alloc_ext_v2().

  @return
  #AR_EOK -- When successful.

  @dependencies
  GPR initialization must be completed via gpr_init().

  @codeexample
  See the code example for __gpr_cmd_async_send().
 */
uint32_t __gpr_cmd_alloc_v2(uint32_t alloc_size, gpr_heap_index_t heap_index, gpr_packet_t **ret_packet);

/** @ingroup gpr_cmd_free
  Frees a specified GPR packet and returns it to the owner.

  @datatypes
  #gpr_packet_t

  @param[in] packet  Pointer to the GPR packet to be freed.

  @return
  #AR_EOK -- When successful.

  @dependencies
  GPR initialization must be completed via gpr_init().

  @codeexample
  See the code example for __gpr_cmd_async_send().
 */
uint32_t __gpr_cmd_free(gpr_packet_t *packet);

/*****************************************************************************
 * Inline Utility Controls                                                   *
 ****************************************************************************/


/** @ingroup gpr_cmd_alloc_ext
  Allocates a formatted free packet for delivery.

  @datatypes
  #gpr_cmd_alloc_ext_t

  @param[in] args  Pointer to the allocated packet information, such as domain
                   and port IDs, token and opcode values, and payload size.

  @detdesc
  This helper function partially creates a packet for delivery. It performs the
  packet allocation and initialization, but the packet payload is dependent on
  the caller to fill in.
  @par
  This two-step process allows the caller to avoid multiple memscpy() operations
  on the packet payload.

  @return
  #AR_EOK -- When successful.

  @dependencies
  GPR initialization must be completed via gpr_init().
  @par
  The source and destination services must be registered with the GPR.

  @codeexample
  @lstlisting
#include "gpr_api_inline.h"

int32_t rc;
gpr_packet_t* packet_ptr;
uint32_t payload_size;

//Example payload required to be sent.
test_client_cmd_function_t * payload;

gpr_cmd_alloc_ext_t alloc_args;
alloc_args.src_domain_id = GPR_CLIENT_SERVICE_DOMAIN_ID_SOURCE;
alloc_args.src_port = GPR_CLIENT_SERVICE_PORT_ID_SOURCE;
alloc_args.dst_domain_id = GPR_CLIENT_SERVICE_DOMAIN_ID_DESTINATION;
alloc_args.dst_port = GPR_CLIENT_SERVICE_PORT_ID_DESTINATION;
alloc_args.token = 0x12345678;
alloc_args.opcode = TEST_CLIENT_CMD_FUNCTION;
alloc_args.payload_size = sizeof( test_client_cmd_function_t );
alloc_args.ret_packet = &packet_ptr;

// Allocate memory for the packet.
rc = __gpr_cmd_alloc_ext(alloc_args);
if ( rc )
{
   printf( "Packet allocation failed" );
   return AR_EFAILED;
}

// Fill in the payload.
payload = GPR_PKT_GET_PAYLOAD( test_client_cmd_function_t, packet_ptr );
payload->param1 = 1;
payload->param2 = 2;

// Send the packet.
rc = __gpr_cmd_async_send( packet_ptr );
if ( rc )
{
   printf( "Could not send the packet.\n" );
}
  @endlstlisting

  @inputfile{group__weakb__gpr__cmd__alloc__ext__t.tex}
 */
uint32_t __gpr_cmd_alloc_ext(gpr_cmd_alloc_ext_t *args);

/** @ingroup gpr_cmd_alloc_ext_v2
  Allocates a formatted free packet for delivery from a specified heap.

  @datatypes
  #gpr_cmd_alloc_ext_v2

  @param[in] args  Pointer to the allocated packet information, such as domain
                   and port IDs, token and opcode values, heap_index and payload size.

  @detdesc
  This helper function partially creates a packet for delivery from a specified heap. It performs the
  packet allocation and initialization, but the packet payload is dependent on
  the caller to fill in.

  @par
  This two-step process allows the caller to avoid multiple memscpy() operations
  on the packet payload.

  @return
  #AR_EOK -- When successful.

  @dependencies
  GPR initialization must be completed via gpr_init().
  @par
  The source and destination services must be registered with the GPR.

  @codeexample
  @lstlisting
#include "gpr_api_inline.h"

int32_t rc;
gpr_packet_t* packet_ptr;
uint32_t payload_size;

//Example payload required to be sent.
test_client_cmd_function_t * payload;

gpr_cmd_alloc_ext_v2_t alloc_args;
alloc_args.src_domain_id = GPR_CLIENT_SERVICE_DOMAIN_ID_SOURCE;
alloc_args.src_port = GPR_CLIENT_SERVICE_PORT_ID_SOURCE;
alloc_args.dst_domain_id = GPR_CLIENT_SERVICE_DOMAIN_ID_DESTINATION;
alloc_args.dst_port = GPR_CLIENT_SERVICE_PORT_ID_DESTINATION;
alloc_args.token = 0x12345678;
alloc_args.heap_index = GPR_HEAP_INDEX_DEFAULT;
alloc_args.opcode = TEST_CLIENT_CMD_FUNCTION;
alloc_args.payload_size = sizeof( test_client_cmd_function_t );
alloc_args.ret_packet = &packet_ptr;

// Allocate memory for the packet.
rc = __gpr_cmd_alloc_ext_v2(alloc_args);
if ( rc )
{
   printf( "Packet allocation failed" );
   return AR_EFAILED;
}

// Fill in the payload.
payload = GPR_PKT_GET_PAYLOAD( test_client_cmd_function_t, packet_ptr );
payload->param1 = 1;
payload->param2 = 2;

// Send the packet.
rc = __gpr_cmd_async_send( packet_ptr );
if ( rc )
{
   printf( "Could not send the packet.\n" );
}
  @endlstlisting

  @inputfile{group__weakb__gpr__cmd__alloc__ext__v2__t.tex}
 */
uint32_t __gpr_cmd_alloc_ext_v2(gpr_cmd_alloc_ext_v2_t *args);

/** @ingroup gpr_cmd_alloc_send
  Allocates and sends a formatted free packet.

  @datatypes
  #gpr_cmd_alloc_send_t

  @param[in] args  Pointer to the allocated packet information, such as domain
                   and port IDs, token and opcode values, and payload size.

  @detdesc
  This helper function fully creates the packet, and it performs the packet
  allocation and initialization. The caller supplies the packet payload as
  an input if you do not want to it use up front. For comparison, see
  __gpr_cmd_async_send().

  @return
  #AR_EOK -- When successful.

  @dependencies
  GPR initialization must be completed via gpr_init().
  @par
  The source and destination services must be registered with the GPR.

  @codeexample
  @lstlisting
#include "gpr_api_inline.h"

int32_t rc;
gpr_packet_t* packet_ptr;
uint32_t payload_size;

//Example payload required to be sent.
test_client_cmd_function_t payload;

// Fill in the payload.
payload.param1 = 1;
payload.param2 = 2;

gpr_cmd_alloc_send_t alloc_send_args;
alloc_send_args.src_domain_id = GPR_CLIENT_SERVICE_DOMAIN_ID_SOURCE;
alloc_send_args.src_port = GPR_CLIENT_SERVICE_PORT_ID_SOURCE;
alloc_send_args.dst_domain_id = GPR_CLIENT_SERVICE_DOMAIN_ID_DESTINATION;
alloc_send_args.dst_port = GPR_CLIENT_SERVICE_PORT_ID_DESTINATION;
alloc_send_args.token = 0x12345678;
alloc_send_args.opcode = TEST_CLIENT_CMD_FUNCTION;
alloc_send_args.payload_size = sizeof( test_client_cmd_function_t );
alloc_send_args.payload = &payload;

// Create and send packet.
rc = __gpr_cmd_alloc_send(alloc_send_args);
if ( rc )
{
   printf( "Packet allocation and send failed" );
   return AR_EFAILED;
}
  @endlstlisting

  @inputfile{group__weakb__gpr__cmd__alloc__send__t.tex}
 */
uint32_t __gpr_cmd_alloc_send(gpr_cmd_alloc_send_t *args);

/** @ingroup gpr_cmd_accept_cmd
  Accepts a command packet by replying with a #GPR_IBASIC_EVT_ACCEPTED
  message to the sender.

  @datatypes
  #gpr_packet_t

  @param[in] packet  Pointer to the command packet to be accepted.

  @detdesc
  The required routing information is extracted from the specified
  packet.
  @par
  The same procedure can be performed manually by swapping the source and
  destination fields, and then inserting a GPR_IBASIC_EVT_ACCEPTED
  payload.

  @return
  #AR_EOK -- When successful.

  @dependencies
  GPR initialization must be completed via gpr_init().
  @par
  The source and destination services must be registered with the GPR.

  @codeexample
  See the code example for __gpr_cmd_register().
 */
uint32_t __gpr_cmd_accept_command(gpr_packet_t *packet);
/** @ingroup gpr_cmd_end_cmd
  Completes a command message by replying with a #GPR_IBASIC_RSP_RESULT command
  response message to the sender. The indicated packet is then freed.

  @datatypes
  #gpr_packet_t

  @param[in] packet  Pointer to the command message to complete.
  @param[in] status  Completion or error status to respond to the client (see
                     Section&nbsp;@xref{hdr:StatusCodes}).

  @detdesc
  This function both sends an GPR_IBASIC_RSP_RESULT command response back to
  the sender and frees the specified command packet. The required routing
  information is extracted from the command packet.
  @par
  The same procedure can be done manually by swapping the source and
  destination fields and then inserting an #GPR_IBASIC_RSP_RESULT command
  response payload.

  @return
  #AR_EOK -- When successful.
  @par
  #AR_EBADPARAM -- When the input parameter is invalid. The packet to be
  delivered is not freed.

  @dependencies
  GPR initialization must be completed via gpr_init().
  @par
  The source and destination services must be registered with the GPR.

  @codeexample
  See the code example for __gpr_cmd_register().

 */
uint32_t __gpr_cmd_end_command(gpr_packet_t *packet, uint32_t status);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* __GPR_API_INLINE_H__ */
