#ifndef _POSAL_DATA_LOG_I_H_
#define _POSAL_DATA_LOG_I_H_
/**
 *  \file posal_data_log_i.h
 * \brief
 *       Internal definitions needed for data logging utility
 *
 * \copyright
 *   Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *   SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* ----------------------------------------------------------------------------
 * Include Files
 * ------------------------------------------------------------------------- */
#include "ar_error_codes.h"
#include "posal_types.h"

//#define DATA_LOGGING_DBG

/* ----------------------------------------------------------------------------
 * Global Declarations/Definitions
 * ------------------------------------------------------------------------- */
// diag max packet size is 8736 but diag adds their header which is around 104 bytes so we cannot use entire 8736
// bytes hence we are subtracting 200 bytes from 8736 which gives the max num of bytes available for spf to use .
#ifdef _DIAG_MAX_TX_PKT_SIZE
#define MAX_LOG_PKT_SIZE (_DIAG_MAX_TX_PKT_SIZE - 200)
#else
// retaining 3360 for variants which do not diag macro defined
#define MAX_LOG_PKT_SIZE 3360
#endif

/* Log container tags */
#define AUDIOLOG_CONTAINER_LOG_HEADER 0x0001      /** Container tag for the log header. */
#define AUDIOLOG_CONTAINER_USER_SESSION 0x0002    /** Container tag for the user session. */
#define AUDIOLOG_CONTAINER_PCM_DATA_FORMAT 0x0004 /** Container tag for the PCM data format. */
#define AUDIOLOG_CONTAINER_BS_DATA_FORMAT 0x0005  /** Container tag for the bitstream data format. */

/** POSAL log codes for audio/voice data path logging. */
#define POSAL_LOG_CODE_UNUSED 0x0            /**< Unused. */
#define POSAL_LOG_CODE_AUD_DEC_IN 0x152E     /**<  audio decoder input. */
#define POSAL_LOG_CODE_AUD_POPP_IN 0x152F    /**<  audio POPP input. */
#define POSAL_LOG_CODE_AUD_COPP_IN 0x1531    /**<  audio COPP input. */
#define POSAL_LOG_CODE_AUD_POPREP_IN 0x1534  /**<  audio POPREP input. */
#define POSAL_LOG_CODE_AUD_COPREP_IN 0x1532  /**<  audio COPREP input. */
#define POSAL_LOG_CODE_AUD_MTMX_RX_IN 0x1530 /**<  audio matrix Rx input. */
#define POSAL_LOG_CODE_AUD_MTMX_TX_IN 0x1533 /**<  audio matrix Tx input. */
#define POSAL_LOG_CODE_AUD_ENC_IN 0x1535     /**<  audio encoder input. */
#define POSAL_LOG_CODE_AUD_ENC_OUT 0x1536    /**<  audio encoder output. */
#define POSAL_LOG_CODE_AFE_RX_TX_OUT 0x1586  /**<  AFE Rx output and AFE Tx input. */
#define POSAL_LOG_CODE_VPTX 0x158A           /**<  vptx near and far input and output. */
#define POSAL_LOG_CODE_VPRX 0x158A           /**< Logs PCM data at vprx output. */
#define POSAL_LOG_CODE_VOC_ENC_IN 0x158B     /**<  voice encoder input  */
#define POSAL_LOG_CODE_VOC_PKT_OUT 0x14EE    /**<  voice encoder output . */
#define POSAL_LOG_CODE_VOC_PKT_IN 0x14EE     /**<  voice decoder input. */
#define POSAL_LOG_CODE_VOC_DEC_OUT 0x158B    /**<  voice decoder output. */
#define POSAL_LOG_CODE_AFE_ALGO_CALIB 0x17D8 /**< Logs run-time parameters of algorithms in AFE. */
#define POSAL_LOG_CODE_PP_RTM 0x184B         /**< Logs RTM parameters of algorithms in PP. */
#define POSAL_LOG_CODE_LSM_OUTPUT 0x1882     /**<  LSM output. */
#define POSAL_LOG_CODE_AUD_DEC_OUT 0x19AF    /**<  audio Decoder output. */
#define POSAL_LOG_CODE_AUD_COPP_OUT 0x19B0   /**<  audio COPP output. */
#define POSAL_LOG_CODE_AUD_POPP_OUT 0x19B1   /**<  audio POPP output. */

#define SEG_PKT (0xDEADBEEF) /** Magic number for indicating the segmented packet. */
#define SPF_PACK(x) x __attribute__((__packed__))
#define POSAL_PINE_VERSION_LATEST                                                                                  \
   POSAL_PINE_VERSION_PCM_V2 /**< Indicates the current version of the packet in the PiNE database */

//#define POSAL_PINE_VERSION_RAW_V1 0x30
//#define POSAL_PINE_VERSION_PCM_V1 0x44
#define POSAL_PINE_VERSION_PCM_V2 0x59

#endif // POSAL_DATA_LOG_I_H
