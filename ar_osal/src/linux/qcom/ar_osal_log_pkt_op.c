/**
* \file ar_osal_log_pkt_op.c
* \brief
*    Defines platform agnostic APIs for sending log packets to
*    the PC.
* \copyright
*  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
*  SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include "ar_osal_log_pkt_op.h"
#include "ar_osal_error.h"
#include "ar_osal_log.h"
#include "comdef.h"
#include "diag_lsm.h"
#include "log.h"

/**<
Length of the DIAG log packet header in bytes. The QXDM Pro log
code, and system timestamp. Each log packet must begin with a
member of type log_hdr_type which is 12 bytes. */
#define AR_OSAL_LOG_DIAG_HEADER_LENGTH 12

#define ar_osal_log_alloc(code, length) log_alloc(code, length)
#define ar_osal_log_commit(ptr) log_commit(ptr)
#define ar_osal_log_free(ptr) log_free(ptr)
#define ar_osal_log_status(code) log_status(code)

/**
* \brief ar_log_pkt_op_init
*
*        Initialzes the data transport mechanism
*
* \param[in] info: pointer to info structure containing heap and data
*                  transport information that is used to initialize the pai
*
*
* \return
*  Success -- 0
*  Failure -- non-zero
*/
int32_t ar_log_pkt_op_init(ar_log_pkt_op_init_info_t *info)
{
    __UNREFERENCED_PARAM(info);

    if(Diag_LSM_Init(NULL))
    {
        return AR_EOK;
    }

    return AR_EFAILED;
}

/**
* \brief ar_log_pkt_op_deinit
*
*        Deinitializes the utility and release any/all
*        resources aquired
*
* \depends
* ar_log_pkt_op_init(...) must be called before this function
*
* \return
*  Success -- returns a pointer to the allocated memory
*  Failure -- returns null if the allocation fails
*/
int32_t ar_log_pkt_op_deinit()
{
    if(Diag_LSM_DeInit())
    {
        return AR_EOK;
    }

    return AR_EFAILED;
}

/**
* \brief ar_log_pkt_get_max_size
*
*        Retrieve the max size of a log packet that can be sent through
*        the data transport
*
* \depends
* ar_log_pkt_op_init(...) must be called before this function
*
* \return
*  return the max log packet size
*/
uint32_t ar_log_pkt_get_max_size()
{
    return 3360;//4k
}

/**
* \brief ar_log_pkt_alloc
*
*        Allocates a log packet and returns a pointer to the memory
*
* \param[in] logcode: the log code used for the packet
* \param[in] length: the size of the packet to allocate
*
* \depends
*  commit or free operations must be called ASAP after this call since
*  the allocation is performed in the transport layer shared resource pool
*
* \return
*  Success -- returns a pointer to the allocated memory
*  Failure -- returns null if the allocation fails
*/
void *ar_log_pkt_alloc(uint16_t logcode, uint32_t length)
{
    uint8_t *ptr = (uint8_t*)ar_osal_log_alloc(
         logcode,
         AR_OSAL_LOG_DIAG_HEADER_LENGTH + length);

    return ptr + AR_OSAL_LOG_DIAG_HEADER_LENGTH;
}

/**
* \brief ar_log_pkt_commit
*
*        Commits the log packet and then frees the memory pointed to by ptr
*
* \param[in] ptr: pointer to memory allocated through ar_log_pkt_alloc
*
* \return
*  n/a
*/
int32_t ar_log_pkt_commit(void *ptr)
{
    ar_osal_log_commit(
        (uint8_t*)ptr - AR_OSAL_LOG_DIAG_HEADER_LENGTH);

    return AR_EOK;
}

/**
* \brief ar_log_pkt_free
*
*        Releases resources aquired through ar_log_pkt_alloc
*
* \param[in] ptr: pointer to memory allocated through ar_log_pkt_alloc
*
* \return
*  n/a
*/
void ar_log_pkt_free(void *ptr)
{
    if (!ptr) return;

     ar_osal_log_free(
         (uint8_t*)ptr - AR_OSAL_LOG_DIAG_HEADER_LENGTH);
}

/**
* \brief ar_log_code_status
*
*        Checks whether a log code is enable
*
* \param[in] logcode: the log code to check.
*
* \return
*  TRUE -- ENABLED
*  FALSE -- DISABLED
*/
bool_t ar_log_code_status(uint16_t logcode)
{
    return ar_osal_log_status(logcode);
}