/* ========================================================================
  @file alsa_device_driver_i.h
  @brief This file contains contains tinyalsa device related internal
  macros and structures.

  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/

#ifndef _ALSA_DEVICE_DRIVER_I_H_
#define _ALSA_DEVICE_DRIVER_I_H_

#include "alsa_device_api.h"
#include <tinyalsa/asoundlib.h>

typedef struct alsa_device_driver
{
   /* Card ID */
   uint32_t card_id;
   /* Device ID */
   uint32_t device_id;
   /* PCM handle */
   struct pcm *pcm;
   /* Encapsulates the hardware and software parameters of a PCM */
   struct pcm_config config;
} alsa_device_driver_t;

#endif // (_ALSA_DEVICE_DRIVER_I_H_)