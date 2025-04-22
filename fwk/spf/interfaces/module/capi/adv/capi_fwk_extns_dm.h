#ifndef ELITE_FWK_EXTNS_DM_H
#define ELITE_FWK_EXTNS_DM_H

/**
 *   \file capi_fwk_extns_dm.h
 *   \brief
 *        fwk extension to take care of modules with variable input consumption rate or output production rate.
 *
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/*-----------------------------------------------------------------------------
 * Include Files
 *----------------------------------------------------------------------------*/

#include "capi_types.h"

/** @addtogroup capi_fw_ext_data_dur_mod
The data duration modifying (DM) framework extension (#FWK_EXTN_DM) is used to
take care of data duration modifying modules with variable input consumption
rates or output production rates.

DM modules change the duration of input data by a small amount relative to
output, or vice versa. These modules do not act as buffering modules, and they
always produce one output for one input. Examples include sample slipping,
Asynchronous Sample Rate Converter (ASRC), and fractional resampling.

This DM extension, used for rate corrections through sample slipping/stuffing
or fractional sample rate conversion, has the following requirements:
- Prebuffering
- Setting of fixed input or output mode
- Allocation of appropriately sized input and output buffers after querying
  the module
*/

/** @addtogroup capi_fw_ext_data_dur_mod
@{ */

/** Unique identifier of the framework extension used to specify a data
    duration modifying module.

    This extension supports the following parameter and event IDs:
    - #FWK_EXTN_DM_PARAM_ID_CHANGE_MODE
    - #FWK_EXTN_DM_PARAM_ID_SET_SAMPLES
    - #FWK_EXTN_DM_EVENT_ID_REPORT_SAMPLES
    - #FWK_EXTN_DM_PARAM_ID_SET_MAX_SAMPLES
    - #FWK_EXTN_DM_EVENT_ID_REPORT_MAX_SAMPLES
    - #FWK_EXTN_DM_EVENT_ID_DISABLE_DM
 */
#define FWK_EXTN_DM 0x0A001027

/*------------------------------------------------------------------------------
 * Parameter IDs
 *----------------------------------------------------------------------------*/

/** Defines the data duration modifying modes.
 */
typedef enum {
   FWK_EXTN_DM_INVALID_MODE      = 0, /**< Invalid value. */
   FWK_EXTN_DM_FIXED_INPUT_MODE  = 1, /**< Module runs in Fixed Input mode. */
   FWK_EXTN_DM_FIXED_OUTPUT_MODE = 2, /**< Module runs in Fixed Output mode. */
} fwk_extn_dm_mode_t;

/** ID of the parameter used to inform the DM module whether it should consume
    partial input or keep it unconsumed while configured for Fixed Output mode.

    This parameter doesn't need to be implemented if this module won't be placed in the same container upstream
    of a module implementing FWK_EXTN_SYNC.

    When a DM module is placed upstream of a module implementing FWK_EXTN_SYNC, it must be configured to
    Fixed Output mode and it is expected to be able to process data even when less than the expected input
    amount is provided. When less than the expected input is provided, this module is allowed to produce for any
    amount of output to be generated (less than the fixed output threshold). This extra requirement is necessary for
    proper sync module behavior when the threshold is disabled.

    This parameter allows the framework to inform the DM module of whether it should or should not consume data
    when less than the expected input is provided.
    @newpage

    @msgpayload{fwk_extn_dm_param_id_consume_partial_input_t}
    @table{weak__fwk__extn__dm__param__id__consume__partial__input__t}
 */
#define FWK_EXTN_DM_PARAM_ID_CONSUME_PARTIAL_INPUT 0x080012EE

typedef struct fwk_extn_dm_param_id_consume_partial_input_t fwk_extn_dm_param_id_consume_partial_input_t;

/** @weakgroup weak_fwk_extn_dm_param_id_consume_partial_input_t
@{ */
struct fwk_extn_dm_param_id_consume_partial_input_t
{
   uint32_t should_consume_partial_input;
   /**< @valuesbul
        - 1 -- The module should consume data even if less than expected input is provided.
        - 0 -- The module should not consume data if less than expected input is provided.@tablebulletend
      */
};
/** @} */ /* end_weakgroup weak_fwk_extn_dm_param_id_consume_partial_input_t */

/** ID of the parameter used to configure a module to run in Fixed Input or
    Fixed Output mode.

    In Fixed Input mode, the module consumes all data on the input side but it
    does not necessarily fill the entire output buffer. If the output buffer
    passed for processing is not large enough to contain all data that is
    produced when consuming the entire input, the module fills the output
    buffer entirely, although it does not consume the entire input.

    In Fixed Output mode, the module produces enough data to completely fill
    the output buffer but does not necessarily consume all the input data.
    If the input buffer passed for processing is not large enough such that
    enough data is produced to fill the output buffer, the module consumes all
    input, although it does not fill the entire output buffer. @newpage

    @msgpayload{fwk_extn_dm_param_id_change_mode_t}
    @table{weak__fwk__extn__dm__param__id__change__mode__t}
 */
#define FWK_EXTN_DM_PARAM_ID_CHANGE_MODE 0x0A001028

typedef struct fwk_extn_dm_param_id_change_mode_t fwk_extn_dm_param_id_change_mode_t;

/** @weakgroup weak_fwk_extn_dm_param_id_change_mode_t
@{ */
struct fwk_extn_dm_param_id_change_mode_t
{
   uint32_t dm_mode;
   /**< Type of data duration modifying mode.

        @valuesbul
        - #FWK_EXTN_DM_FIXED_INPUT_MODE
        - #FWK_EXTN_DM_FIXED_OUTPUT_MODE @tablebulletend */
};
/** @} */ /* end_weakgroup weak_fwk_extn_dm_param_id_change_mode_t */

/* Structure that provides sample count per port based on index
 */
typedef struct fwk_extn_dm_port_samples_t fwk_extn_dm_port_samples_t;

/** @weakgroup weak_fwk_extn_dm_port_samples_t
@{ */
struct fwk_extn_dm_port_samples_t
{
   uint32_t port_index;          /**< Port index for which samples are being
                                      set. */
   uint32_t samples_per_channel; /**< Number of samples per channel for the
                                      port. */
};
/** @} */ /* end_weakgroup weak_fwk_extn_dm_port_samples_t */

/** ID of the parameter used to set the number of samples that are either
    required on output or are provided on input to the module.

    The module responds to this parameter ID with
    #FWK_EXTN_DM_EVENT_ID_REPORT_SAMPLES.

    @msgpayload{fwk_extn_dm_param_id_req_samples_t}
    @table{weak__fwk__extn__dm__param__id__req__samples__t}

    @msgpayload{fwk_extn_dm_port_samples_t}
    @table{weak__fwk__extn__dm__port__samples__t} @newpage
 */
#define FWK_EXTN_DM_PARAM_ID_SET_SAMPLES 0x0A001029

/** ID of the event raised in response to #FWK_EXTN_DM_PARAM_ID_SET_SAMPLES or
    when the sample requirement of a module changes.

    For modules configured in Fixed Input mode, this event is raised for output
    ports.

    For modules configured in Fixed Output mode, this event is raised for input
    ports.

    @msgpayload{fwk_extn_dm_param_id_req_samples_t}
    @table{weak__fwk__extn__dm__param__id__req__samples__t}

    @msgpayload{fwk_extn_dm_port_samples_t}
    @table{weak__fwk__extn__dm__port__samples__t} @newpage
 */
#define FWK_EXTN_DM_EVENT_ID_REPORT_SAMPLES 0x0A00102A

/** ID of the parameter used to set either the maximum number of samples that
    a module can provide when required on input, or the maximum space required
    on output. Usage depends on the mode.

    The module responds to this setting with
    #FWK_EXTN_DM_EVENT_ID_REPORT_MAX_SAMPLES.

    @msgpayload{fwk_extn_dm_param_id_req_samples_t}
    @table{weak__fwk__extn__dm__param__id__req__samples__t}
 */
#define FWK_EXTN_DM_PARAM_ID_SET_MAX_SAMPLES 0x0A00102B

/** ID of the event used in response to #FWK_EXTN_DM_PARAM_ID_SET_MAX_SAMPLES.

    @msgpayload{fwk_extn_dm_param_id_req_samples_t}
    @table{weak__fwk__extn__dm__param__id__req__samples__t} @newpage
 */
#define FWK_EXTN_DM_EVENT_ID_REPORT_MAX_SAMPLES 0x0A00102C

typedef struct fwk_extn_dm_param_id_req_samples_t fwk_extn_dm_param_id_req_samples_t;

/** @weakgroup weak_fwk_extn_dm_param_id_req_samples_t
@{ */
struct fwk_extn_dm_param_id_req_samples_t
{
   uint16_t is_input;
   /**< Indicates whether samples are being set for input or output ports. */

   uint16_t num_ports;
   /**< Number of ports for which samples are being set. */

   fwk_extn_dm_port_samples_t req_samples[1];
   /**< Array that contains the required samples.

        For #FWK_EXTN_DM_PARAM_ID_SET_MAX_SAMPLES, input port samples are
        samples to be provided to the module, and output port samples are
        samples that are required from the module.

        For #FWK_EXTN_DM_EVENT_ID_REPORT_MAX_SAMPLES, input port samples are
        samples required by the module, and output port samples indicate that
        output buffer space is required. */
};
/** @} */ /* end_weakgroup weak_fwk_extn_dm_param_id_req_samples_t */

/** ID of the event a module raises to disable or enable DM mode, which the
    framework sets with #FWK_EXTN_DM_PARAM_ID_CHANGE_MODE.

    Depending on the output media configuration or input media format, the
    module can raise disable = 1 to indicate that it will not act as a DM
    module. For example, a disabled rate matching module or a resampler
    currently performing integer sample rate conversion.

    The module can enable itself by raising disable = 0 when the it starts
    fractional resampling.

    @msgpayload{fwk_extn_dm_event_id_disable_dm_t}
    @table{weak__fwk__extn__dm__event__id__disable__dm__t}
 */
#define FWK_EXTN_DM_EVENT_ID_DISABLE_DM 0x0A00102D

/** Defines the DM modes.
*/
typedef enum event_id_disable_dm_supported_values_t {
   FWK_EXTN_DM_ENABLED_DM = 0,
   /**< Module can raise an event to enable the DM mode (if it is currently in
        Disabled mode). \n @vertspace{4}

        By default, the module is assumed to be in Enabled mode. Once the
        module is enabled, it can operate in fixed in/fixed out mode as set by
        the framework. @vertspace{6} */

   FWK_EXTN_DM_DISABLED_DM = 1,
   /**< Module does not modify data duration in the context of input media
        format or output media format configuration. */
} /** @cond */ event_id_disable_dm_supported_values_t /** @endcond */;

typedef struct fwk_extn_dm_event_id_disable_dm_t fwk_extn_dm_event_id_disable_dm_t;

/** @weakgroup weak_fwk_extn_dm_event_id_disable_dm_t
@{ */
struct fwk_extn_dm_event_id_disable_dm_t
{
   uint32_t disabled;
   /**< Indicates whether the DM mode is disabled.

        @valuesbul
        - 0 -- #FWK_EXTN_DM_ENABLED_DM
        - 1 -- #FWK_EXTN_DM_DISABLED_DM @tablebulletend */
};
/** @} */ /* end_weakgroup weak_fwk_extn_dm_event_id_disable_dm_t */

/** @} */ /* end_addtogroup capi_fw_ext_data_dur_mod */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* #ifndef ELITE_FWK_EXTNS_DM_H*/
