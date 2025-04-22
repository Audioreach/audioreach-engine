/**
@file sysmon.c

@brief 

================================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/

#include "sysmon_audio_query.h"
#include "ar_msg.h"

int sysmon_audio_register(unsigned char enable)
{
    AR_MSG(DBG_ERROR_PRIO, "sysmon_audio_register is stubbed in MDF/CDSP");
    return 0;
}

int sysmon_audio_query(sysmon_audio_query_t *query)
{
    AR_MSG(DBG_ERROR_PRIO, "sysmon_audio_query is stubbed in MDF/CDSP");
    return 0;
}