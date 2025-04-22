/**
 * \file ar_osal_serverg.c
 *
 * \brief
 *       This file has implementation of service location, notification, and
 *       state registration.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#define AR_OSAL_SERVREG_TAG    "COSR"

#include "ar_osal_types.h"
#include "ar_osal_servreg.h"
#include "ar_osal_mutex.h"
#include "ar_osal_error.h"
#include "ar_osal_log.h"
#include "ar_osal_mem_op.h"
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#ifdef AR_OSAL_USE_CUTILS
#include <cutils/properties.h>
#endif

 #ifdef __cplusplus
 extern "C" {
 #endif /* __cplusplus */

#ifdef AR_OSAL_USE_PD_NOTIFIER
#include <libpdmapper.h>
#include <libpdnotifier.h>
#include <service_registry_notifier_v01.h>
SR_DL_Handle*       pd_mapper_handle;
#else
#include "ar_osal_sys_id.h"
char_t domain_name[AR_SUB_SYS_ID_LAST + 1][24] = { "msm/adsp/audio_pd", "msm/mdsp/audio_pd", "msm/adsp/audio_pd","msm/apss/audio_pd","msm/sdsp/audio_pd","msm/cdsp/audio_pd" };
#endif /* AR_OSAL_USE_PD_NOTIFIER */

#define ADSP_LOADER_PATH    "/sys/kernel/boot_adsp/ssr"
#define PROC_PANIC_PATH     "/proc/sysrq-trigger"

#define AR_OSAL_SERVREG_CLIENT_NAME "apps/ar_osal"

int32_t g_init_done = 0;
struct ar_osal_service_node
{

    ar_osal_servreg_t          srv_handle;
    ar_osal_servreg_entry_type service;
    ar_osal_servreg_entry_type domain;
    void                         *cb_context;
    ar_osal_servreg_callback   cb_func;
    ar_osal_service_state_type srv_state;
#ifdef AR_OSAL_USE_PD_NOTIFIER
    PD_Notifier_Handle           *pd_handle;
#endif /* AR_OSAL_USE_PD_NOTIFIER */
};

typedef struct ar_osal_service_node ar_osal_service_node;
ar_osal_servreg_t serv_reg_handle;

#ifdef AR_OSAL_USE_PD_NOTIFIER
static ar_osal_service_state_type pd_event_to_ar_osal_pd_state(enum pd_event event)
{
    ar_osal_service_state_type ret = AR_OSAL_SERVICE_STATE_DOWN;
    switch (event) {
    case EVENT_PD_DOWN:
        ret = AR_OSAL_SERVICE_STATE_DOWN;
        break;
    case EVENT_PD_UP:
        ret = AR_OSAL_SERVICE_STATE_UP;
        break;
    default:
        ret = AR_OSAL_SERVICE_STATE_DOWN;
        break;
    }
    return ret;
}

static ar_osal_service_state_type pd_state_to_ar_osal_pd_state(pd_state state)
{
    ar_osal_service_state_type ret = AR_OSAL_SERVICE_STATE_DOWN;

    switch (state) {
    case SERVREG_NOTIF_SERVICE_STATE_DOWN_V01:
        ret = AR_OSAL_SERVICE_STATE_DOWN;
        break;
    case SERVREG_NOTIF_SERVICE_STATE_UP_V01:
        ret = AR_OSAL_SERVICE_STATE_UP;
        break;
    default:
        ret = AR_OSAL_SERVICE_STATE_DOWN;
    }
    return ret;
}

static void ar_osal_pd_notifier_cb(void *data, enum pd_event event)
{
    ar_osal_service_node *entry = (ar_osal_service_node *)data;
    ar_osal_service_state_type state = pd_event_to_ar_osal_pd_state(event);
    ar_osal_servreg_state_notify_payload_type notify_state;

    AR_LOG_INFO(AR_OSAL_SERVREG_TAG, "ar_osal_pd_notifier_cb state(%d) service(%s)",
	                                      state, entry->service.name );
    AR_LOG_INFO(AR_OSAL_SERVREG_TAG, "ar_osal_pd_notifier_cb state(%d) domain(%s)",
	                                      state, entry->domain.name );

    notify_state.service_state = state;
    memcpy((void*)&notify_state.service, (const void*)&entry->service, sizeof(ar_osal_servreg_entry_type));
    memcpy((void*)&notify_state.domain, (const void*)&entry->domain, sizeof(ar_osal_servreg_entry_type));


    if (entry)
    {
        if (entry->cb_func)
        {
            entry->cb_func((ar_osal_servreg_t)entry,
                AR_OSAL_SERVICE_STATE_NOTIFY,
                entry->cb_context,
                (void*)&notify_state,
                sizeof(notify_state));
        }
        entry->srv_state = state;
    }
}
#endif /* AR_OSAL_USE_PD_NOTIFIER */


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
_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_osal_servreg_init(void)
{
    int32_t status = AR_EOK;
    // ar_osal_servreg_type_t* handle = NULL;
    if (g_init_done)
    {
        g_init_done++;
        return status;
    }
    g_init_done = 1;
#ifdef AR_OSAL_USE_PD_NOTIFIER
    pd_mapper_handle = servreg_alloc_DLHandle();
    if (!pd_mapper_handle)
    {
        status = AR_ENOMEMORY;
        AR_LOG_ERR(AR_OSAL_SERVREG_TAG, "servreg allocation error status(%d)", status);
        goto end;
    }
#endif /* AR_OSAL_USE_PD_NOTIFIER */
    // strlcpy(handle->client_name, AR_OSAL_SERVREG_TAG, sizeof(handle->client_name));
    AR_LOG_INFO(AR_OSAL_SERVREG_TAG, "ar_osal_servreg_init success status(%d)", status);
end:
    return status;
}

/**
* \brief ar_osal_servreg_deinit
*        Uninitialize servreg interface.
*........Should be called in pair with ar_osal_servreg_init() and
*........should be a serialized call.
* \return
*  0 -- Success
*  Nonzero -- Failure
*/

_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_osal_servreg_deinit(void)
{
    //uint32_t status = AR_EOK;
    // ar_osal_servreg_type_t* handle = (ar_osal_servreg_type_t*)pd_handle;
    g_init_done--;
    if (0 != g_init_done)
    {
        goto end;
    }
#ifdef AR_OSAL_USE_PD_NOTIFIER
    if (pd_mapper_handle)
    {
        servreg_free_DLHandle(pd_mapper_handle);
        pd_mapper_handle = NULL;
    }
#endif /* AR_OSAL_USE_PD_NOTIFIER */
end:
    return AR_EOK;
}

/**
* \brief ar_osal_servreg_get_domainlist
*        Client to call this API to get a list of domains(msm/domain/subdomain) on which
*        a given service(provider/service) is supported.
*
* \param[in]  service: service(provider/service) for which domain(s) list is required.
* \param[in out] domain_list: service supported in domain(s), client to provide
*.                            payload buffer pointer.
* \param[in out] num_domains: Client to provide the num_domains to get the domain list.
*.                            Input NULL domain_list to get the number of domains
*                             for the given service if available in num_domains.
*
* \return
*  0 -- Success
*  Nonzero -- Failure
*  AR_ENOMEMORY- Failed due to insufficient memory, client to call the API again
*                  with required size as returned in num_domains.
*.
*/

_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_osal_servreg_get_domainlist(_In_ ar_osal_servreg_entry_type *service,
    _Inout_opt_ ar_osal_servreg_entry_type *domain_list,
    _Inout_ uint32_t *num_domains)
{
    int32_t status = AR_EOK;
    if (NULL == service || NULL == num_domains)
    {
        status = AR_EBADPARAM;
        goto end;
    }
#ifdef AR_OSAL_USE_PD_NOTIFIER
    if (NULL == domain_list)
    {
        enum SR_Result_Enum rc = SR_RESULT_SUCCESS;
        rc = servreg_get_domainlist((char *)service->name, pd_mapper_handle);
        if (rc != SR_RESULT_SUCCESS)
        {
            status = AR_EFAILED;
            AR_LOG_ERR(AR_OSAL_SERVREG_TAG, "servreg_get_domainlist error status(%d)"
                "servreg status(%d)", status, rc);
            goto end;
        }

        //return single domain
        *num_domains = servreg_get_numentries(pd_mapper_handle);
        status = AR_ENOMEMORY;
    }
    else if ((domain_list != NULL) && (num_domains > 0))
    {
        //copy domain details
        enum SR_Result_Enum rc = SR_RESULT_SUCCESS;
        uint32_t i = 0;
        //memcpy_s(domain_list->name, sizeof(domain_list->name), domain_name, sizeof(domain_name));
        for (i = 0; i < *num_domains; i++)
        {
            char* name;
            int instance = 0;
            int service_data_valid = 0;
            int service_data = 0;
            rc = servreg_get_entry(pd_mapper_handle,
                &name,
                &instance,
                &service_data_valid,
                &service_data,
                i);
            if (rc != SR_RESULT_SUCCESS)
            {
                status = AR_EFAILED;
                AR_LOG_ERR(AR_OSAL_SERVREG_TAG,
                    "servreg_get_domainlist entry(%d) error status(%d)"
                    "servreg status(%d)",
                    i, status, rc);
                goto end;
            }

            strlcpy((domain_list + i)->name, name, sizeof(domain_list->name));
            (domain_list + i)->instance_id = instance;

            AR_LOG_INFO(AR_OSAL_SERVREG_TAG, "servreg_get_domainlist entry(%d)"
                " instance(%d) domain name(%s)",
                i, instance, (domain_list + i)->name);
        }
    }
    else
    {
        // service not found.
        status = AR_EBADPARAM;
    }
#else
    if (NULL == domain_list)
    {
        //return supported domains
        *num_domains = AR_SUB_SYS_ID_LAST;
        status = AR_ENOMEMORY;
        goto end;
    }

    //copy domain details
    strlcpy(domain_list[0].name, domain_name[AR_MODEM_DSP], sizeof(domain_list[0].name));
    strlcpy(domain_list[1].name, domain_name[AR_AUDIO_DSP], sizeof(domain_list[1].name));
    strlcpy(domain_list[2].name, domain_name[AR_APSS], sizeof(domain_list[2].name));
    strlcpy(domain_list[3].name, domain_name[AR_SENSOR_DSP], sizeof(domain_list[3].name));
    strlcpy(domain_list[4].name, domain_name[AR_COMPUTE_DSP], sizeof(domain_list[4].name));
    domain_list[0].instance_id = 74;
    domain_list[1].instance_id = 74;
    domain_list[2].instance_id = 74;
    domain_list[3].instance_id = 74;
    domain_list[4].instance_id = 74;
    *num_domains = AR_SUB_SYS_ID_LAST;
#endif

end:
    AR_LOG_INFO(AR_OSAL_SERVREG_TAG,
        "ar_osal_servreg_get_domainlist service(%s) status(0x%x)",
     service->name, status);
    return status;
}

/**
* \brief ar_osal_servreg_register
*        Service client(s) to register for the domain service state change notifications.
*
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
_IRQL_requires_max_(PASSIVE_LEVEL)
ar_osal_servreg_t ar_osal_servreg_register(_In_ ar_osal_client_type  client_type __unused,
    _In_opt_ ar_osal_servreg_callback cb_func,
    _In_opt_ void *cb_context,
    _In_ ar_osal_servreg_entry_type *domain,
    _In_ ar_osal_servreg_entry_type *service)
{
#ifndef AR_OSAL_USE_PD_NOTIFIER
    return 1;
#else
    int32_t status = AR_EOK;
    //ar_osal_servreg_t* handle = NULL;
    ar_osal_service_node* srv_reg_handle = NULL;
    enum pd_rcode pd_rc = PD_NOTIFIER_FAIL;
    pd_state state = SERVREG_NOTIF_SERVICE_STATE_DOWN_V01;

    if (NULL == domain || NULL == service || NULL == cb_func)
    {
        status = AR_EBADPARAM;
        goto end;
    }
    srv_reg_handle = (ar_osal_service_node*)malloc(sizeof(ar_osal_service_node));
    if (!srv_reg_handle)
    {
        status = AR_ENOMEMORY;
        AR_LOG_ERR(AR_OSAL_SERVREG_TAG,
         "handle allocation error status(%d)", status);
        goto end;
    }

    memset((void*)srv_reg_handle, 0, sizeof(ar_osal_service_node));
    memcpy(&srv_reg_handle->service, service, sizeof(ar_osal_servreg_entry_type));
    memcpy(&srv_reg_handle->domain, domain, sizeof(ar_osal_servreg_entry_type));

    srv_reg_handle->cb_func = cb_func;
    srv_reg_handle->cb_context = cb_context;

    srv_reg_handle->pd_handle = pd_notifier_alloc(domain->name,
        AR_OSAL_SERVREG_CLIENT_NAME,
        domain->instance_id,
        ar_osal_pd_notifier_cb,
        (void*)srv_reg_handle);

    if (!srv_reg_handle->pd_handle)
    {
        status = AR_EFAILED;
        AR_LOG_ERR(AR_OSAL_SERVREG_TAG, "pd_notifier_alloc error status(%d)", status);
        free(srv_reg_handle);
        srv_reg_handle = NULL;
        goto end;
    }

    pd_rc = pd_notifier_register(srv_reg_handle->pd_handle, &state);

    if (pd_rc != PD_NOTIFIER_SUCCESS) {
        status = AR_EFAILED;
        AR_LOG_ERR(AR_OSAL_SERVREG_TAG, "pd_notifier_register error: status(%d)"
            "pd_notifier status(%d)",
            status, pd_rc );
        pd_notifier_free(srv_reg_handle->pd_handle);
        free(srv_reg_handle);
        srv_reg_handle = NULL;
        goto end;
    }
    else {
        AR_LOG_INFO(AR_OSAL_SERVREG_TAG,
            "Successfully registered.  Curr state is %s state (0x%08x)",
            (state == EVENT_PD_UNKNOWN) ? "unknown" :
            ((state == EVENT_PD_UP) ? "up" :
            ((state == EVENT_PD_DOWN) ? "down" : "out of range")),
            state);
        srv_reg_handle->srv_state = pd_state_to_ar_osal_pd_state(state);
    }

end:
    return (ar_osal_servreg_t)srv_reg_handle;
#endif
}

/**
* \brief ar_osal_servreg_deregister
*        Service client(s) to deregister for the service state change notifications.
*
* \param[in]  servreg_handle: interface handle returned by ar_osal_servreg_allocate_handle().
*
* \return
*  0 -- Success
*  Nonzero -- Failure
*/
_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_osal_servreg_deregister(_In_ ar_osal_servreg_t servreg_handle)
{
#ifndef AR_OSAL_USE_PD_NOTIFIER
    return 1;
#else
    int32_t status = AR_EOK;
    enum pd_rcode pd_rc = PD_NOTIFIER_FAIL;
    ar_osal_service_node* srv_reg_handle = (ar_osal_service_node*)servreg_handle;
    if (NULL == servreg_handle)
    {
        status = AR_EBADPARAM;
        goto end;
    }

    pd_rc = pd_notifier_deregister(srv_reg_handle->pd_handle);
    if (pd_rc != PD_NOTIFIER_SUCCESS) {
        status = AR_EFAILED;
        AR_LOG_ERR(AR_OSAL_SERVREG_TAG, "pd_notifier_deregister error: status(%d)"
            "pd_notifier status(%d)",
            status, pd_rc);
        pd_notifier_free(srv_reg_handle->pd_handle);
        free(srv_reg_handle);
        srv_reg_handle = NULL;
        goto end;
    }
    pd_notifier_free(srv_reg_handle->pd_handle);
    free(srv_reg_handle);
    srv_reg_handle = NULL;
end:
    return status;
#endif
}

/**
* \brief ar_osal_servreg_set_state
*        Service provider to call this API to register its service states(UP/DOWN).
*        This API to be used only by the service provider(msm/domain/subdomain/provider/service)
*        and not by service client(s).
*
* \param[in]  servreg_handle: interface handle returned by ar_osal_servreg_allocate_handle().
* \param[in]  state: new service state for service registered using ar_osal_servreg_register().
*.
* \return
*  0 -- Success
*  Nonzero -- Failure
*/
_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_osal_servreg_set_state(_In_ ar_osal_servreg_t servreg_handle __unused,
    _In_ ar_osal_service_state_type state __unused)
{
    return AR_ENOTIMPL;
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

_IRQL_requires_max_(PASSIVE_LEVEL)
static int32_t ar_osal_servreg_ssr() {
#ifdef PROPERTY_TRIGGER
    if (property_set("vendor.audio.ssr.trigger", "1")) {
        AR_LOG_ERR(AR_OSAL_SERVREG_TAG, "%s: set property failed", __func__);
        return -1;
    }
    return 0;
#else
    int fd_dsplder;
    int32_t rc = 0;

    fd_dsplder = open(ADSP_LOADER_PATH, O_WRONLY);

    if(fd_dsplder < 0) {
        AR_LOG_ERR(AR_OSAL_SERVREG_TAG, "%s: open (%s) fail - %s (%d)",
		   __func__, ADSP_LOADER_PATH, strerror(errno), errno);
	rc = errno;
    } else if (write(fd_dsplder, "1", 1) < 0) {
        AR_LOG_ERR(AR_OSAL_SERVREG_TAG, "%s: write (%s) fail - %s (%d)",
		   __func__, ADSP_LOADER_PATH, strerror(errno), errno);
	rc = errno;
    }

    return rc;
#endif
}

/**
* \brief induce panic to crash the system
*
*/
void ar_osal_panic()
{
#ifdef PROPERTY_TRIGGER
    if (property_set("vendor.audio.crash.trigger", "1"))
        AR_LOG_ERR(AR_OSAL_SERVREG_TAG, "%s: set property failed", __func__);
#else
    char panic_set ='c';
    int fd_sysrq = 0;

    fd_sysrq = open(PROC_PANIC_PATH, O_WRONLY);

    if(fd_sysrq < 0) {
        AR_LOG_ERR(AR_OSAL_SERVREG_TAG, "%s: open (%s) fail - %s (%d)", __func__,
                   PROC_PANIC_PATH, strerror(errno), errno);
	//ignore if panic path can't be opened
    } else if (write(fd_sysrq, &panic_set, 1) < 0) {
        AR_LOG_ERR(AR_OSAL_SERVREG_TAG,"%s: write (%s) fail - %s (%d)", __func__,
		   PROC_PANIC_PATH, strerror(errno), errno);
    }
#endif
}

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
int32_t ar_osal_servreg_restart_service(ar_osal_servreg_t servreg_handle)
{
#ifndef AR_OSAL_USE_PD_NOTIFIER
    return 1;
#else
    int32_t rc = AR_EOK;
    int32_t ssr_rc;
    enum pd_rcode pd_rc;
    ar_osal_service_node* serv_reg_handle = (ar_osal_service_node*)servreg_handle;
#ifdef AR_OSAL_USE_CUTILS
    char value[256] = {0};

    property_get("persist.vendor.audio.induce_crash", value, "");
    if (!strncmp("true", value, sizeof("true")))
        ar_osal_panic();

    property_get("persist.vendor.audio.spf_restart", value, "");
    if (strncmp("true", value, sizeof("true"))) {
        AR_LOG_INFO(AR_OSAL_SERVREG_TAG, "SPF restart feature disabled\n");
        return rc;
    }
#endif

    AR_LOG_INFO(AR_OSAL_SERVREG_TAG, "Restarting service %d", servreg_handle);

    if (serv_reg_handle == NULL)
    {
        rc = AR_EBADPARAM;
        goto end;
    }

    pd_rc = pd_notifier_restart_pd(serv_reg_handle->pd_handle);
    if (pd_rc != PD_NOTIFIER_SUCCESS) {
        AR_LOG_ERR(AR_OSAL_SERVREG_TAG,
		   "Failed to restart audio service rc(%d) perform SSR\n", pd_rc);
	ssr_rc = ar_osal_servreg_ssr();
        if (ssr_rc) {
	    AR_LOG_ERR(AR_OSAL_SERVREG_TAG,
                       "Failed to restart audio DSP rc(%d)\n", ssr_rc);
	    rc = AR_EUNEXPECTED;
	}
    }

end:
    return rc;
#endif
}
