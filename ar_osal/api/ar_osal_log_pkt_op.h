#ifndef AR_OSAL_LOG_PKT_OP_H
#define AR_OSAL_LOG_PKT_OP_H

/**
* \file ar_osal_log_pkt_op.h
* \brief
*    Defines platform agnostic APIs for sending log packets to
*    the PC.
* \copyright
*  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
*  SPDX-License-Identifier: BSD-3-Clause-Clear
*/

#include "ar_osal_types.h"
#include "ar_osal_heap.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

    /**< An enumeration of supported transports */
    typedef enum ar_log_data_transport_t
    {
        AR_LOG_DATA_TRANSPORT_UNKNOWN   = 0x0,
        AR_LOG_DATA_TRANSPORT_DIAG      = 0x1,
        AR_LOG_DATA_TRANSPORT_TCPIP     = 0x2,
    }ar_log_data_transport_t;

    typedef struct ar_log_pkt_op_init_info_t
    {
        ar_log_data_transport_t
            data_transport;     /**< The data transport to use when sending
                                     log packets */
        ar_heap_info heap_info; /**< Heap information used to identify memory
                                     allocated by the data transport */
    }ar_log_pkt_op_init_info_t, *par_log_pkt_op_init_info_t;

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
    int32_t ar_log_pkt_op_init(ar_log_pkt_op_init_info_t *info);

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
    int32_t ar_log_pkt_op_deinit(void);

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
    uint32_t ar_log_pkt_get_max_size(void);

    /**
    * \brief ar_log_pkt_alloc
    *
    *        Allocates a log packet and returns a pointer to the memory
    *
    * \param[in] logcode: the log code used for the packet. See
    *                     ar_util_data_log_codes.h in the AR Util
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
    void *ar_log_pkt_alloc(uint16_t logcode, uint32_t length);

    /**
    * \brief ar_log_pkt_commit
    *
    *        Commits the log packet and then frees the memory pointed to by ptr
    *
    * \param[in] ptr: pointer to memory allocated through ar_log_pkt_alloc
    *
    * \return
    *  Success -- 0
    *  Failure -- non-zero
    */
    int32_t ar_log_pkt_commit(void *ptr);

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
    void ar_log_pkt_free(void *ptr);

    /**
    * \brief ar_log_code_status
    *
    *        Checks whether a log code is enable
    *
    * \param[in] logcode: the log code to check. See ar_util_data_log_codes.h
    *                     in the AR Util
    *
    * \return
    *  TRUE -- ENABLED
    *  FALSE -- DISABLED
    */
    bool_t ar_log_code_status(uint16_t logcode);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* AR_OSAL_LOG_PKT_OP_H */