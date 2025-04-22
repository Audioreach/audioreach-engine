#ifndef AR_OSAL_SERVREG_H
#define AR_OSAL_SERVREG_H

/**
 * \file ar_osal_servreg.h
 * \brief
 *      Defines public APIs for service location,
 *      notification and state registration.
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_osal_types.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/**  Max length of the domain name i.e "soc/domain/subdomain" or
 *   service name i.e "provider/service" is 64 bytes
 *   e.g. msm/adsp/audio_pd or avs/audio or audio/avs_mdf_sdsp or
 *    audio/avs_mdf_mdsp
 */
#define AR_OSAL_SERVREG_NAME_LENGTH_MAX              (64)

/** ar osal servreg type object.
*/
typedef void *ar_osal_servreg_t;

/**  Struct representing the name of the service or domain and the instance id
 */
typedef struct {
    /**<   Name of the service or domain. */
    char_t name[AR_OSAL_SERVREG_NAME_LENGTH_MAX + 1];

    /**<   Instance ID. */
    uint32_t instance_id;
}ar_osal_servreg_entry_type;

/** Service state up/down indicator.
 */
typedef enum ar_osal_service_state {
	/* service is in down state */
	AR_OSAL_SERVICE_STATE_DOWN = 0,
	/* service is in up state */
	AR_OSAL_SERVICE_STATE_UP = 1,
} ar_osal_service_state_type;

/** Servreg client type: listener or service provider.
 */
typedef enum ar_osal_client {
	/* invalid client */
	AR_OSAL_CLIENT_INVALID	= 0,
	/* listener client */
	AR_OSAL_CLIENT_LISTENER	= 1,
	/* service provide client */
	AR_OSAL_CLIENT_SERVICE_PROVIDER	= 2,
} ar_osal_client_type;

/** Servreg callback notify events.
*/
typedef enum ar_osal_servreg_cb_event
{
    /* service state notify event */
    AR_OSAL_SERVICE_STATE_NOTIFY = 1,
} ar_osal_servreg_cb_event_type;

/** servreg service state notify callback payload.
 */
typedef struct ar_osal_servreg_state_notify_payload {
    /* Service notification for */
    ar_osal_servreg_entry_type service;
    /* Service domain notification for */
    ar_osal_servreg_entry_type domain;
    /* service state on a domain */
    ar_osal_service_state_type  service_state;
} ar_osal_servreg_state_notify_payload_type;

/**
* \brief ar_osal_servreg_init
*        Initialize servreg interface.
*........Note:This API has to be called before any other API in this interface.
*........Should be called at least once and is expected to be serialized if called
*        multiple times.
* \return
*  0 -- Success
*  Nonzero -- Failure
*/
int32_t ar_osal_servreg_init(void);

/**
* \brief ar_osal_servreg_deinit
*        Uninitialize servreg interface.
*........Should be called in pair with ar_osal_servreg_init() and
*........should be a serialized call.
* \return
*  0 -- Success
*  Nonzero -- Failure
*/
int32_t ar_osal_servreg_deinit(void);

/**
* \brief ar_osal_servreg_get_domainlist
*        Client to call this API to get a list of domains(msm/domain/subdomain) on which
*        a given service(provider/service) is supported.
*
* \param[in]  service: service(provider/service) for which domain(s) list is required.
* \param[in out] domain_list: service supported in domain(s), client to provide
*.                            payload buffer pointer.
* \param[in out] num_domains: Client to provide the num_domains to get the domain list.
*.                            If num_domains is zero and domain_list is NULL, API will return
*.                            the number of domains for the given service if available.
*
* \return
*  0 -- Success
*  Nonzero -- Failure
*  AR_ENOMEMORY- Failed due to insufficient memory, client to call the API again
*                  with required size as returned in num_domains.
*.
*/
int32_t ar_osal_servreg_get_domainlist(ar_osal_servreg_entry_type *service,
    ar_osal_servreg_entry_type *domain_list,
    uint32_t *num_domains);

/**
* \brief ar_osal_servreg_callback
*        Callback function to notify clients for any changes in
*        service state(up/down).
* \param[in] servreg_handle: Servreg handle returned with ar_osal_servreg_register() for the given service.
* \param[in] event_id: callback event id supported by ar_osal_servreg_cb_event_type .
* \param[in] cb_context:  payload/context provided by client in ar_osal_servreg_register() .
*.\param[in] payload: payload provided by the callback.
*.                    service state UP/DOWN payload ar_osal_servreg_state_notify_payload_type.
*.\param[in] payload_size: payload size in bytes.
* \return
*  none
*/
typedef void(*ar_osal_servreg_callback)(ar_osal_servreg_t servreg_handle,
    ar_osal_servreg_cb_event_type event_id, void *cb_context, void *payload,
    uint32_t payload_size);

/**
* \brief ar_osal_servreg_register
*        Service client(s) to register for the domain service state change notifications.
*
* \param[in]  client_type: indicates registering client is a listener or service provider.
* \param[in opt]  cb_func: callback function pointer to get notifications on.
*.                         This is parameter is optional to Service provider registration.
* \param[in opt]  cb_context: callback function payload/context provided by client.
*.                         This is parameter is optional to Service provider registration.
* \param[in]  domain: domain of the service(msm/domain/subdomain) for which the
*.            state change notifications to be provided.
* \param[in]  service: service(provider/service) for which the
*.            state change notifications to be provided.
* \return
*  servreg_handle on success.
*  null on failure.
*/
ar_osal_servreg_t ar_osal_servreg_register(ar_osal_client_type  client_type,
    ar_osal_servreg_callback cb_func,
    void *cb_context,
    ar_osal_servreg_entry_type *domain,
    ar_osal_servreg_entry_type *service);

/**
* \brief ar_osal_servreg_deregister
*        Service client(s) to deregister for the service state change notifications.
*
* \param[in]  servreg_handle: interface handle returned by ar_osal_servreg_register().
*
* \return
*  0 -- Success
*  Nonzero -- Failure
*/
int32_t ar_osal_servreg_deregister(ar_osal_servreg_t servreg_handle);

/**
* \brief ar_osal_servreg_set_state
*        Service provider to call this API to register its service states(UP/DOWN).
*        This API to be used only by the service provider(msm/domain/subdomain/provider/service)
*        and not by service client(s).
*
* \param[in]  servreg_handle: interface handle returned by ar_osal_servreg_register().
* \param[in]  state: new service state for service registered using ar_osal_servreg_register().
*.
* \return
*  0 -- Success
*  Nonzero -- Failure
*/
int32_t ar_osal_servreg_set_state(ar_osal_servreg_t servreg_handle,
        ar_osal_service_state_type state);

/**
* \brief ar_osal_servreg_restart_service
*        HLOS calls this API to trigger a restart (PDR or SSR) on a given
*        processor
*
* \param[in]  servreg_handle: interface handle returned by
*             ar_osal_servreg_register() which identifies the desired processor
*
* \return
*  0 -- Success
*  Nonzero -- Failure
*/
int32_t ar_osal_servreg_restart_service(ar_osal_servreg_t servreg_handle);

/**
* \brief induce panic to crash the system
*
*/
void ar_osal_panic();

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif //AR_OSAL_SERVREG_H
