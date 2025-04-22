/**
 * \file ar_spf_cmd.h
 * \brief
 *    This file contains the API definitions for the commands related to the global shared memory
 *    used between client<->module and module<->module
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef _AR_SPF_CMD_API_H_
#define _AR_SPF_CMD_API_H_

#include "ar_defs.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/** @ingroup spf_apm_commands
 This command carries an OOB payload from global shared memory (#APM_CMD_GLOBAL_SHARED_MEM_MAP_REGIONS)
 and used by the client to signal module to read the payload and handle accordingly.

 OOB payload structure and the communication protocol is private to client and module.

 @gpr_hdr_fields
 Opcode -- AR_SPF_MSG_GLOBAL_SH_MEM

 @msgpayload
 ar_spf_msg_global_sh_mem_t

 @par
 This command can be issued for all the module instance IDs that were
 configured at least once using #APM_CMD_GRAPH_OPEN.

 @return
 GPR_IBASIC_RSP_RESULT
 */
#define AR_SPF_MSG_GLOBAL_SH_MEM 0x0100105B

/** Definition of the is_ack_required flag bitmask. */
#define AR_SPF_MSG_GLOBAL_SH_MEM_BIT_MASK_IS_ACK_NOT_REQUIRED (0x00000001UL)

/** Definition of the is_memory_release flag bitmask. */
#define AR_SPF_MSG_GLOBAL_SH_MEM_BIT_MASK_IS_MEM_RELEASE (0x00000002UL)

/** @ingroup spf_apm_commands
 Payload for #AR_SPF_MSG_GLOBAL_SH_MEM.
 */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct ar_spf_msg_global_sh_mem_t
{
   uint32_t shmem_id;
   /**< A unique ID used by the client to map the global memory regions using
    command #APM_CMD_GLOBAL_SHARED_MEM_MAP_REGIONS.
    */

   uint32_t property_flag;
   /**< Configures property for this command.

    @values 0x00000000 through 0x00000001

    @contcell
    @subhead{Bit 0 -- is_ack_not_required flag}

    Indicates if acknowledgment is needed for this command:
    - 1 -- Ack is not needed. This is useful if client only want to send the information and not waiting for the
    response. Client may still receive #GPR_IBASIC_RSP_RESULT in response to this command if command fails to reach the
    destination.

    - 0 -- Ack is needed. This is a common case where client wants to wait for the response.

    @subhead{Bit 1 -- is_release_memory flag}

    Indicates if client/HLOS wants module to release the specified shared memory. This can be used for cleanup or during
    error handling.
    - 1 -- release; used when client/HLOS wants module to stop reading/writing from/into the specified shared memory.
    If shmem_size is zero then client/HLOS wants module to stop using the entire shared memory region. This can be used
    just before shared memory is going to be unmapped.
    If shmem_size is non zero then client/HLOs wants module to stop using the memory block specified with msw,lsw.

    - 0 -- Default value; used when client wants module to read/use the specified shared memory.

    @subhead{Bits 31 to 2}

    Reserved and must be set to 0. */
   uint32_t shmem_addr_lsw;
   /**< Lower 32 bits of the physical/Virtual/byte-offset address value.

    This refers to the memory which is used by client/module to communicate.

    If global shared memory was mapped with offset property (Bit 2 is set)
    then this value is a byte offset from the base virtual address of
    global shared memory.

    If global shared memory was not mapped with offset property (Bit 2 is unset)
    then this value either refers to a physical or a virtual address depending on the
    Bit 0 configuration in the global shared memory map property.

    @values Non-zero 32-bit value */

   uint32_t shmem_addr_msw;
   /**< Upper 32 bits of the physical/Virtual address value.

    @valuesbul
    - For offset based addressing, this field must be set to 0.
    - For a 32-bit shared memory address, this field must be set to 0.
    - For a 36-bit address, bits 31 to 4 must be set to 0.
    - For a 64-bit shared memory address, non-zero 32-bit value.
    @tablebulletend */

   uint32_t shmem_size;
   /**< Number of bytes in the shared memory used by the client/modules to communicate.
    Base virtual address of this memory will be retrieved from the
    physical/virtual/byte-offset address value specified in this payload. */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct ar_spf_msg_global_sh_mem_t ar_spf_msg_global_sh_mem_t;

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* _AR_SPF_CMD_API_H_ */
