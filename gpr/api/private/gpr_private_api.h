#ifndef __GPR_PRIVATE_API_H__
#define __GPR_PRIVATE_API_H__

/**
 * @file  gpr_private_api.h
 * @brief This file contains GPR Private APIs
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
 * Function Definitions                                                      *
 ****************************************************************************/

/** @ingroup gpr_cmd_register_v2
  Registers a service, by its unique service ID, with the GPR.

  @datatypes
  #gpr_callback_fn_t

  @param[in] src_port       Unique ID (within a domain) of the service to be
                            registered.
  @param[in] callback_fn    Callback function of the service to be registered.
  @param[in] callback_data  Pointer to the client-supplied data pointer for the
                            callback function.
  @param[in] heap_index     Heap index of the client registering with GPR.

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
  int32_t rc = __gpr_cmd_register_v2(GPR_TESTCLIENT_SERVICE_ID,
                                  callback_fn,
                                  callback_data,
                                  GPR_HEAP_INDEX_DEFAULT);
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
uint32_t __gpr_cmd_register_v2(uint32_t          src_port,
                               gpr_callback_fn_t callback_fn,
                               void *            callback_data,
                               gpr_heap_index_t  heap_index);

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

int32_t rc = __gpr_cmd_register_v2(GPR_TESTCLIENT_SERVICE_ID,
                                callback_fn,
                                callback_data,
                                GPR_HEAP_INDEX_DEFAULT);
if ( rc )
{
  printf( "Could not register the client service with GPR" );
}
...
rc =  __gpr_cmd_deregister_v2(GPR_TESTCLIENT_SERVICE_ID, GPR_HEAP_INDEX_DEFAULT);
if ( rc )
{
  printf( "Could not deregister the client service from GPR" );
}
  @endlstlisting
 */
uint32_t __gpr_cmd_deregister_v2(uint32_t src_port, gpr_heap_index_t heap_index);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* __GPR_PRIVATE_API_H__ */
