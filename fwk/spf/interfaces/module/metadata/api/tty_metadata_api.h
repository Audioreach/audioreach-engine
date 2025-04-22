#ifndef TTY_METADATA_API_H
#define TTY_METADATA_API_H

/**
 *   \file tty_metadata_api.h
 *   \brief
 *        This file Metadata ID and payloads for TTY detection status transmission
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/** @h2xml_title1           {TTY Metadata API}
    @h2xml_title_agile_rev  {TTY Metadata API}
    @h2xml_title_date       {August 14, 2019} */

/**************************************** 1X_TTY - Begin ***********************************/

/**
    Metadata ID for 1x TTY detection. module_cmn_md_t structure
    has to set the metadata_id field to this id when the metadata
    is related to 1x TTY detections.  The module also should check this ID before operating on
    1x TTY structures.

    1x TTY detection metadata is transmitted by 1x voice decoders to the the 1x TTY module.
    This 1x TTY module upon receiving this information, transmits this information to the Soft Volume
    Module through IMC in the Tx path

 */
#define METADATA_ID_1X_TTY 0x0A001045

typedef struct metadata_1x_tty_t metadata_1x_tty_t;

/** Data structure for the stream's metadata */
struct metadata_1x_tty_t
{
   uint32_t is_tty_active;
   /**< This is set to TRUE when TTY detection happens
     else set to FALSE */
};
/**************************************** 1X_TTY - End *************************************/

#endif // TTY_METADATA_API_H
