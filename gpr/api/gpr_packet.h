#ifndef __GPR_PACKET_H__
#define __GPR_PACKET_H__

/**
 * @file  gpr_packet.h
 * @brief This file contains GPR packet definitions
 *
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */


/******************************************************************************
 * Includes                                                                    *
 *****************************************************************************/
#include "gpr_comdef.h"
#include "ar_guids.h"

/** @addtogroup gpr_macros
@{ */
/** @name GPR Packet Definitions
@{ */

/****************************************************************************/
//Header Field Definitions
/****************************************************************************/

/** Defines the layout of the packet structure. @hideinitializer */
#define GPR_PKT_VERSION_V ( 0 )

/** Uninitialized port value. @hideinitializer */
#define GPR_PKT_INIT_PORT_V  ( ( uint32_t ) 0 )

/** Uninitialized reserved field value. @hideinitializer */
#define GPR_PKT_INIT_RESERVED_V ( ( uint16_t ) 0 )

/** Uninitialized domain ID value. @hideinitializer */
#define GPR_PKT_INIT_DOMAIN_ID_V ( ( uint8_t ) 0 )

/** Header size in number of 32-bit words. @hideinitializer */
#define GPR_PKT_HEADER_WORD_SIZE_V ( ( sizeof( gpr_packet_t ) + 3 ) >> 2 )

/** Header size in number of bytes. @hideinitializer */
#define GPR_PKT_HEADER_BYTE_SIZE_V ( GPR_PKT_HEADER_WORD_SIZE_V << 2 )

/** Undefined value where a valid GUID is expected. @hideinitializer */
#define GPR_UNDEFINED_ID_V  AR_NON_GUID( ( uint32_t ) 0xFFFFFFFF )

/** Uninitialized client data value. @hideinitializer */
#define GPR_PKT_INIT_CLIENT_DATA_V ( ( uint8_t ) 0 )

/****************************************************************************/
// GPR Packet Bitfield Macros for Header Fields
/****************************************************************************/

/** Bitmask of the version field. */
#define GPR_PKT_VERSION_MASK  AR_NON_GUID( 0x0000000F )

/** Bit shift of the version field. */
#define GPR_PKT_VERSION_SHFT ( 0 )

/** Bitmask of the header size field. */
#define GPR_PKT_HEADER_SIZE_MASK AR_NON_GUID( 0x000000F0 )

/** Bit shift of the header size field. */
#define GPR_PKT_HEADER_SIZE_SHFT ( 4 )

/** Bitmask of the reserved field. Includes four reserved bits from the client
    data field. */
#define GPR_PKT_RESERVED_MASK  AR_NON_GUID( 0xFFF00000 )

/** Bit shift of the reserved field. */
#define GPR_PKT_RESERVED_SHFT ( 20 )

/** Bitmask of the packet size field. */
#define GPR_PKT_PACKET_SIZE_MASK  AR_NON_GUID( 0xFFFFFF00 )

/** Bit shift of the packet size field. */
#define GPR_PKT_PACKET_SIZE_SHFT ( 8 )


/****************************************************************************/
//Packet Helper Macros
/****************************************************************************/

/** Gets the value of a field, including the specified mask and shift.
    @hideinitializer
*/
#define GPR_GET_BITMASK( mask, shift, value ) \
  ( ( ( value ) & ( mask ) ) >> ( shift ) )

/** Sets a value in a field, including the specified mask and shift.
    @hideinitializer
*/
#define GPR_SET_BITMASK( mask, shift, value ) \
  ( ( ( value ) << ( shift ) ) & ( mask ) )

/** Gets the value of a field. @hideinitializer
*/
#define GPR_GET_FIELD( field, value ) \
  GPR_GET_BITMASK( ( field##_MASK ), ( field##_SHFT ), ( value ) )

/** Sets a value in a field. @hideinitializer */
#define GPR_SET_FIELD( field, value ) \
  GPR_SET_BITMASK( ( field##_MASK ), ( field##_SHFT ), ( value ) )

/** Returns an 8-bit aligned pointer to a base address pointer plus an
    offset in bytes.
    @hideinitializer
*/
#define GPR_PTR_END_OF( base_ptr, offset ) \
  ( ( ( uint8_t* ) base_ptr ) + ( offset ) )

/** Given the packet header, returns the packet's current size in bytes.

    The current packet byte size is the sum of the base packet structure and
    the used portion of the payload.
    @hideinitializer
*/
#define GPR_PKT_GET_PACKET_BYTE_SIZE( header ) \
  ( GPR_GET_FIELD( GPR_PKT_PACKET_SIZE, header ) )

/** Given the packet header, returns the header's current size in bytes.
    @hideinitializer
*/
#define GPR_PKT_GET_HEADER_BYTE_SIZE( header ) \
  ( (size_t)GPR_GET_FIELD( GPR_PKT_HEADER_SIZE, header ) << 2 )

/** Given the packet header, returns the payload's current size in bytes.

    The current payload byte size is the difference between the packet size and
    the header size.
    @hideinitializer
*/
#define GPR_PKT_GET_PAYLOAD_BYTE_SIZE( header ) \
  ( GPR_PKT_GET_PACKET_BYTE_SIZE( header ) - \
    GPR_PKT_GET_HEADER_BYTE_SIZE( header ) )

/** Given the packet, returns a pointer to the beginning of the packet's
    payload.
    @hideinitializer
*/
#define GPR_PKT_GET_PAYLOAD( type, packet_ptr ) \
  ( ( type* ) GPR_PTR_END_OF( packet_ptr, \
                GPR_PKT_GET_HEADER_BYTE_SIZE( \
                  ( ( struct gpr_packet_t* ) packet_ptr )->header ) ) )

/** @} */ /* end_name GPR Packet Definitions */
/** @} */ /* end_addtogroup gpr_macros */


/***************************************************************************
Packet Structure Definition
***************************************************************************/

/* Packet structure type definition. */
typedef struct gpr_packet_t gpr_packet_t;

/** @ingroup gpr_packet_struct
  Core header structure necessary to route a packet from a source to a
  destination.
*/
struct gpr_packet_t
{
  uint32_t header;
  /**< Contains the following subfield information for the header field.

       @subhead{version}
        - Bits 3 to 0 (four bits).
        - Defines the layout of the packet structure.

       @subhead{header\_size}
        - Bits 7 to 4 (four bits).
        - The header size is the number of 32-bit header words starting.
          from the beginning of the packet.

       @subhead{packet\_size}
        - Bits 31 to 8 (twenty-four bits).
        - The total packet size in bytes.
        - The total packet size includes both the header and payload size.
        @tablebulletend */

  uint8_t dst_domain_id;
  /**< Domain ID of the destination where the packet is to be delivered.

       - Bits 0 to 7 (eight bits).
       - The domain ID refers to a process, a processor, or an off-target
         location that houses GPR.
       - The transport layer used may differ depending on the physical
         link between domains.
       - All domain ID values are reserved by GPR. @tablebulletend */

  uint8_t src_domain_id;
  /**< Domain ID of the source from where the packet came.

       - Bits 8 to 15 (eight bits).
       - The domain ID refers to a process, a processor, or an off-target
         location that houses GPR.
       - The transport layer used may differ depending on the physical
         link between domains.
       - All domain ID values are reserved by the GPR. @tablebulletend */

  uint8_t client_data;
  /**< Used for client-specific needs.

       - Bits 16 to 23 (eight bits).
       - 4 bits for use by client.
       - 4 bits are reserved and to be set to 0.
       - Default value to be set to 0. @tablebulletend */

  uint8_t reserved;
  /**< Reserved field.

       - Bits 24 to 31 (eight bits).
       - Set the value to 0. @tablebulletend */

  uint32_t src_port;
  /**< Unique ID of the service from where the packet came.

       - Bits 31 to 0 (thirty-two bits).
       - The src_port refers to the ID that the sender service registers with
         the GPR. This ID must be unique to the service within a given domain.
       - Service refers to any functional block, such as a module, that is
         required to send or receive packets through the GPR. @tablebulletend */

  uint32_t dst_port;
  /**<  Unique ID of the service where the packet is to be delivered.

       - Bits 31 to 0 (thirty-two bits).
       - The dst_port refers to the ID that the receiver service registers with
         the GPR. This ID must be unique to the service within a given domain.
       - Service refers to any functional block, fsuch as a module, that is
         required to send or receive packets through the GPR. @tablebulletend */

  uint32_t token;
  /**< Client transaction ID provided by the sender.

       Bits 31 to 0 (thirty-two bits). */

  uint32_t opcode;
  /**< Defines both the action and the payload structure.

       @valuesbul
       - Bits 31 to 0 (thirty-two bits).
       - This operation code is a Globally Unique ID (GUID) and must be a valid
         value. @tablebulletend */
};


#endif /* __GPR_PACKET_H__ */

