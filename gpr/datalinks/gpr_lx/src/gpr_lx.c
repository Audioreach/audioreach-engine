/*
 * gpr_lx.c
 *
 * This file has implementation platform wrapper for the GPR datalink layer
 *
 * Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#define LOG_TAG "gpr_dl_lx"

#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "ar_osal_log.h"

#ifdef GPR_USE_CUTILS
#include <cutils/list.h>
#include <sys/poll.h>
#else
#include "list.h"
#include "poll.h"
#endif

#include <pthread.h>
#include "gpr_comdef.h"
#include "ipc_dl_api.h"
#include "gpr_ids_domains.h"
#include "ar_osal_error.h"

#define GPR_DL_LX_ADSP_DRV "/dev/aud_pasthru_adsp"
#define GPR_DL_LX_CC_DSP_DRV "/dev/gpr_channel"
#define GPR_DL_LX_MODEM_DRV "/dev/aud_pasthru_modem"
#define GPR_DL_LX_APPS_SPF_DRV "/dev/aud_pasthru_apps"
#define GPR_DL_LX_BUF_SIZE 4096 /*bytes*/
#define GPR_DL_LX_NO_OF_BUFFERS 8

/** Data receive notification callback type*/
typedef uint32_t (*gpr_dl_lx_receive_cb)(void *ptr, uint32_t length);

/** Data send done notification callback type*/
typedef uint32_t (*gpr_dl_lx_send_done_cb)(void *ptr, uint32_t length);

typedef struct gpr_dl_lx_buf{
    struct listnode node;
    void *buffer;
}gpr_dl_lx_buf_t;

typedef struct gpr_dl_lx_port{
    uint32_t domain_id;
    pthread_t receiver_thread;
    bool thread_exit;
    gpr_dl_lx_receive_cb rx_cb;
    gpr_dl_lx_send_done_cb send_done;
    int drv_fd;
    int intpipe[2];
    struct listnode buff_list;
    pthread_mutex_t buff_list_lock;
    int buf_cnt;
} gpr_dl_lx_port_t;

/*Array of structure pointers each member pointer corresponds to one domain*/
gpr_dl_lx_port_t *gpr_dl_lx_ports[GPR_PL_NUM_TOTAL_DOMAINS_V]={NULL};

static uint32_t gpr_dl_lx_send(uint32_t domain_id, void *buf, uint32_t size);

static uint32_t gpr_dl_lx_receive_done(uint32_t domain_id, void *buf);

/*ipc datalink function table*/
static ipc_to_gpr_vtbl_t gpr_dl_lx_vtbl =
{
   gpr_dl_lx_send,
   gpr_dl_lx_receive_done,
};

void deallocate_buffers(gpr_dl_lx_port_t *dl_lx_port)
{
    gpr_dl_lx_buf_t *buffer_node = NULL;
    struct listnode *item = NULL;

    pthread_mutex_lock(&dl_lx_port->buff_list_lock);
    while (!list_empty(&dl_lx_port->buff_list)) {
        item = list_head(&dl_lx_port->buff_list);
        list_remove(item);
        buffer_node = node_to_item(item, gpr_dl_lx_buf_t, node);
        if (buffer_node->buffer)
            free(buffer_node->buffer);
        free(buffer_node);
        buffer_node = NULL;
    }
    pthread_mutex_unlock(&dl_lx_port->buff_list_lock);
}

uint32_t allocate_buffers(gpr_dl_lx_port_t *dl_lx_port,
                          size_t buf_sz, size_t no_of_buffers)
{
    uint32_t status;
    unsigned int i;

    pthread_mutex_lock(&dl_lx_port->buff_list_lock);
    list_init(&dl_lx_port->buff_list);

    for (i = 0; i < no_of_buffers; i++) {

        gpr_dl_lx_buf_t *buffer_node =
                          (gpr_dl_lx_buf_t *)malloc(sizeof(gpr_dl_lx_buf_t));
        if (buffer_node == NULL) {
            AR_LOG_ERR(LOG_TAG,"%s:%d malloc failed", __func__, __LINE__);
            status = AR_ENOMEMORY;
            goto error;
        }

        buffer_node->buffer = calloc(buf_sz, sizeof(int8_t));
        if (buffer_node->buffer == NULL) {
            AR_LOG_ERR(LOG_TAG,"%s:%d malloc for buf failed", __func__, __LINE__);
            free (buffer_node);
            status = AR_ENOMEMORY;
            goto error;
        }
        list_add_tail(&dl_lx_port->buff_list, &buffer_node->node);
        dl_lx_port->buf_cnt++;
    }
    AR_LOG_VERBOSE(LOG_TAG,"%s:%d buf_cnt = %d", __func__, __LINE__, dl_lx_port->buf_cnt);
    pthread_mutex_unlock(&dl_lx_port->buff_list_lock);
    return AR_EOK;
error:
    pthread_mutex_unlock(&dl_lx_port->buff_list_lock);
    deallocate_buffers(dl_lx_port);
    return status;
}

uint32_t get_buffer(gpr_dl_lx_port_t *dl_lx_port, void **buf)
{
    gpr_dl_lx_buf_t *buffer_node;
    struct listnode *item;
    pthread_mutex_lock(&dl_lx_port->buff_list_lock);
    if (list_empty(&dl_lx_port->buff_list)) {
        AR_LOG_ERR(LOG_TAG,"%s:%d No free buffers available", __func__, __LINE__);
        pthread_mutex_unlock(&dl_lx_port->buff_list_lock);
        return AR_ENORESOURCE;
    }
    item = list_head(&dl_lx_port->buff_list);
    if (item != NULL) {
        list_remove(item);
        buffer_node = node_to_item(item, gpr_dl_lx_buf_t, node);
        if (buffer_node->buffer) {
            *buf = buffer_node->buffer;
            free(buffer_node);
        } else {
            AR_LOG_ERR(LOG_TAG,"%s:%d buffer node invalid", __func__, __LINE__);
            pthread_mutex_unlock(&dl_lx_port->buff_list_lock);
            return AR_ENOTEXIST;
        }
        AR_LOG_VERBOSE(LOG_TAG,"%s:%d buf_cnt = %d", __func__, __LINE__, --dl_lx_port->buf_cnt);
    }
    pthread_mutex_unlock(&dl_lx_port->buff_list_lock);
    return AR_EOK;
}

uint32_t put_buffer(gpr_dl_lx_port_t *dl_lx_port, void *buf)
{
    gpr_dl_lx_buf_t *buffer_node = NULL;
    struct listnode *item = NULL;
    struct listnode *temp_node = NULL;

    pthread_mutex_lock(&dl_lx_port->buff_list_lock);
    list_for_each_safe(item, temp_node, &dl_lx_port->buff_list) {
        if (item != NULL) {
            buffer_node = node_to_item(item, gpr_dl_lx_buf_t, node);
            if (buffer_node && buffer_node->buffer == buf) {
                AR_LOG_ERR(LOG_TAG,"%s:%d buffer already put error case", __func__, __LINE__);
                pthread_mutex_unlock(&dl_lx_port->buff_list_lock);
                return AR_EALREADY;
            }
        }
    }
    buffer_node = (gpr_dl_lx_buf_t *)malloc(sizeof(gpr_dl_lx_buf_t));
    if (buffer_node == NULL) {
        AR_LOG_ERR(LOG_TAG,"%s:%d malloc failed", __func__, __LINE__);
        pthread_mutex_unlock(&dl_lx_port->buff_list_lock);
        return AR_ENOMEMORY;
    }
    buffer_node->buffer = buf;
    list_add_tail(&dl_lx_port->buff_list, &buffer_node->node);
    AR_LOG_VERBOSE(LOG_TAG,"%s:%d buf_cnt = %d", __func__, __LINE__, ++dl_lx_port->buf_cnt);
    pthread_mutex_unlock(&dl_lx_port->buff_list_lock);
    return AR_EOK;
}

#define NUM_FDS 2

void *receiver_thread_loop(void *priv_data)
{
    uint32_t status;
    int32_t receive_size;
    void *buf;
    uint32_t *temp;
    gpr_dl_lx_port_t *dl_lx_port = (gpr_dl_lx_port_t *)priv_data;
    struct pollfd *pfd;
    if (dl_lx_port == NULL) {
        AR_LOG_ERR(LOG_TAG,"%s:%d invalid port instance", __func__, __LINE__);
        return NULL;
    }
    pfd = (struct pollfd *)calloc(NUM_FDS, sizeof(struct pollfd));
    if (pfd == NULL) {
        AR_LOG_ERR(LOG_TAG,"%s:%d calloc failed for poll fd", __func__, __LINE__);
        return NULL;
    }
    if (dl_lx_port->drv_fd) {
        pfd[0].fd = dl_lx_port->drv_fd;
        pfd[0].events = POLLIN|POLLPRI|POLLERR|POLLHUP|POLLNVAL;
        pfd[1].fd = dl_lx_port->intpipe[0];
        pfd[1].events = POLLIN|POLLPRI|POLLERR|POLLHUP|POLLNVAL;
    } else {
        AR_LOG_ERR(LOG_TAG,"%s:%d invalid driver inst exit resp thread", __func__, __LINE__);
        return NULL;
    }

    while (1) {
        if (dl_lx_port->thread_exit) {
            AR_LOG_DEBUG(LOG_TAG,"%s:%d exiting receiver thread", __func__, __LINE__);
            break;
        }

        /*Implement poll related functionality here*/
        if (poll(pfd, NUM_FDS, -1) < 0) {
            /*Poll errored out, treat it as a fatal error bail out*/
            int error = errno;
            AR_LOG_ERR(LOG_TAG,"Poll failed error %s", strerror(error));
            break;
        }

        AR_LOG_DEBUG(LOG_TAG,"Out of poll");
        if (pfd[0].revents & (POLLIN|POLLPRI)) {
            /*
             * Get a buffer from buffer queue, it is a finite queue
             * So if the client holds the received buffers for long
             * we would run out of buffers.
             */
            status = get_buffer(dl_lx_port, &buf);
            if (status != 0) {
                AR_LOG_ERR(LOG_TAG,"%s:%d get_buffer failed", __func__, __LINE__);
                /*Shall we break out or continue ?*/
                continue;
            }
            memset(buf, 0 , GPR_DL_LX_BUF_SIZE);
            receive_size = read(dl_lx_port->drv_fd, buf, GPR_DL_LX_BUF_SIZE);
            temp = (uint32_t *) buf;
            AR_LOG_DEBUG(LOG_TAG,"recieved buffer %x %x %x %x size %d", temp[0], temp[1], temp[2], temp[3], receive_size);
            if ((receive_size <= 0) || (receive_size > GPR_DL_LX_BUF_SIZE)) {
                AR_LOG_ERR(LOG_TAG,"%s:%d read failed %d", __func__, __LINE__, errno);
                put_buffer(dl_lx_port, buf);
            } else {
                if (dl_lx_port->rx_cb) {
                    status = dl_lx_port->rx_cb(buf, receive_size);
                    if (status != AR_EOK) {
                        AR_LOG_ERR(LOG_TAG,"%s:%d receive callback failed", __func__, __LINE__);
                        continue;
                    }
                }
             }
        } else if (pfd[0].revents & (POLLERR|POLLHUP|POLLNVAL)) {
            /*
             *We should hit this case when we are trying to exit
             *dl layer will close the driver, which inturn should
             *unblock poll with an error mask;
             */
            AR_LOG_INFO(LOG_TAG,"%s:%d Poll errored", __func__, __LINE__);
            continue;
        } else if (pfd[1].revents & (POLLIN|POLLPRI)) {
            break;
        }
    }
    return NULL;
}


static gpr_dl_lx_port_t * gpr_dl_lx_local_init(uint32_t src_domain_id, uint32_t dst_domain_id)
{
    gpr_dl_lx_port_t *dl_lx_port;
    uint32_t status = 0;
    char *drv_name = NULL;
    pthread_attr_t tattr;
    struct sched_param param = { .sched_priority = 3 };

    if ((dst_domain_id < 0) || (dst_domain_id >= GPR_PL_NUM_TOTAL_DOMAINS_V)
        || (src_domain_id < 0) || (src_domain_id >= GPR_PL_NUM_TOTAL_DOMAINS_V)) {
        AR_LOG_ERR(LOG_TAG,"%s:%d invalid domain(src domain id %d, dst domain id %d)",
                __func__, __LINE__, src_domain_id, dst_domain_id);
        return NULL;
    }

    AR_LOG_INFO(LOG_TAG,"%s:%d port setup for src domain id %d and dst domain id %d",
            __func__, __LINE__, src_domain_id, dst_domain_id);

    if (gpr_dl_lx_ports[dst_domain_id] != NULL){
        AR_LOG_ERR(LOG_TAG,"%s:%d port already setup for domain id:%d", __func__, __LINE__,
               dst_domain_id);
        return gpr_dl_lx_ports[dst_domain_id];
    }
    dl_lx_port = (gpr_dl_lx_port_t *)calloc(1, sizeof(gpr_dl_lx_port_t));
    if (dl_lx_port == NULL){
        AR_LOG_ERR(LOG_TAG,"%s:%d malloc failed", __func__, __LINE__);
        return NULL;
    }
    dl_lx_port->domain_id = dst_domain_id;
    dl_lx_port->thread_exit = false;

    if (dst_domain_id == GPR_IDS_DOMAIN_ID_ADSP_V) {
        if (src_domain_id == GPR_IDS_DOMAIN_ID_APPS2_V) {
            drv_name = GPR_DL_LX_APPS_SPF_DRV;
            dl_lx_port->drv_fd = open(drv_name, O_RDWR);
            AR_LOG_INFO(LOG_TAG,"%s:%d open drv_name:%s, drv_fd:%d", __func__, __LINE__, drv_name, dl_lx_port->drv_fd);
        } else {
            drv_name = GPR_DL_LX_ADSP_DRV;
            dl_lx_port->drv_fd = open(drv_name, O_RDWR);
        }
    } else if (dst_domain_id == GPR_IDS_DOMAIN_ID_MODEM_V) {
        drv_name = GPR_DL_LX_MODEM_DRV;
        dl_lx_port->drv_fd = open(drv_name, O_RDWR);
    } else if (dst_domain_id == GPR_IDS_DOMAIN_ID_CC_DSP_V) {
        drv_name = GPR_DL_LX_CC_DSP_DRV;
        dl_lx_port->drv_fd = open(drv_name, O_RDWR);
    } else if (dst_domain_id == GPR_IDS_DOMAIN_ID_APPS2_V) {
        drv_name = GPR_DL_LX_APPS_SPF_DRV;
        dl_lx_port->drv_fd = open(drv_name, O_RDWR);
    } else {
        dl_lx_port->drv_fd = -1;
    }

    if ((dl_lx_port->drv_fd < 0) || (pipe(dl_lx_port->intpipe) < 0) ) {
        AR_LOG_ERR(LOG_TAG,"%s:%d driver open failed %d for %s", __func__, __LINE__, errno, drv_name);
        free(dl_lx_port);
        return NULL;
    }

    pthread_mutex_init(&dl_lx_port->buff_list_lock,
                          (const pthread_mutexattr_t *) NULL);

    status = allocate_buffers(dl_lx_port, GPR_DL_LX_BUF_SIZE,
                             GPR_DL_LX_NO_OF_BUFFERS);
    if (status) {
        AR_LOG_ERR(LOG_TAG,"%s:%d buffer allocation failed", __func__, __LINE__);
        free(dl_lx_port);
        return NULL;
    }
    pthread_attr_init (&tattr);
    pthread_attr_setschedparam (&tattr, &param);
    pthread_attr_setschedpolicy(&tattr, SCHED_FIFO);
    status = pthread_create(&dl_lx_port->receiver_thread, &tattr,
                    receiver_thread_loop, dl_lx_port);
    if (status) {
        AR_LOG_ERR(LOG_TAG,"%s:%d error:%d pthread_create fail", __func__, __LINE__, status);
        free(dl_lx_port);
        return NULL;
    }
    return dl_lx_port;
}


static uint32_t gpr_dl_lx_local_deinit(uint32_t src_domain_id, uint32_t dst_domain_id)
{
    uint32_t status = AR_EOK;
    gpr_dl_lx_port_t *dl_lx_port;

    if (gpr_dl_lx_ports[dst_domain_id] == NULL) {
        AR_LOG_ERR(LOG_TAG,"%s:%d deinit already done", __func__, __LINE__);
        return AR_EOK;
    }
    dl_lx_port = gpr_dl_lx_ports[dst_domain_id];
    gpr_dl_lx_ports[dst_domain_id] = NULL;
    /*
     * Set thread exit to true and then close the driver instance
     * this should unblock the poll and then we do a pthread_join
     * to ensure that the receiver_thread has exited.
     */
    dl_lx_port->thread_exit = true;
    status = write(dl_lx_port->intpipe[1], "Q", 1);
    if(status < 0) {
        /* proceed regardless with a error print */
        AR_LOG_ERR(LOG_TAG,"%s:%d write to driver failed %d", __func__, __LINE__, errno);
    }

    status = pthread_join(dl_lx_port->receiver_thread, NULL);
    if (status < 0){
        AR_LOG_ERR(LOG_TAG,"%s:%d pthread_join failed", __func__, __LINE__);
    }
    close(dl_lx_port->drv_fd);
    dl_lx_port->drv_fd = 0;
    free(dl_lx_port);
    return status;
}

uint32_t ipc_dl_lx_init(uint32_t src_domain_id,
                        uint32_t dest_domain_id,
                        const gpr_to_ipc_vtbl_t *p_gpr_to_ipc_vtbl,
                        ipc_to_gpr_vtbl_t ** pp_ipc_to_gpr_vtbl)
{
    gpr_dl_lx_port_t *dl_lx_port;

    if ((dest_domain_id < 0) || (dest_domain_id >= GPR_PL_NUM_TOTAL_DOMAINS_V)
        || (src_domain_id < 0) || (src_domain_id >= GPR_PL_NUM_TOTAL_DOMAINS_V)) {
        AR_LOG_ERR(LOG_TAG,"%s:%d invalid domain(src domain id %d, dst domain id %d)",
                __func__, __LINE__, src_domain_id, dest_domain_id);
        return AR_EBADPARAM;
    }

    dl_lx_port = gpr_dl_lx_local_init(src_domain_id, dest_domain_id);
    if (dl_lx_port == NULL) {
        AR_LOG_ERR(LOG_TAG,"%s:%d local_init failed", __func__, __LINE__);
        return AR_EFAILED;
    }
    *pp_ipc_to_gpr_vtbl = &gpr_dl_lx_vtbl;
    if (p_gpr_to_ipc_vtbl->receive && p_gpr_to_ipc_vtbl->send_done) {
        dl_lx_port->rx_cb = p_gpr_to_ipc_vtbl->receive;
        dl_lx_port->send_done = p_gpr_to_ipc_vtbl->send_done;
    } else {
        AR_LOG_ERR(LOG_TAG,"%s:%d no gpr cbs error out", __func__, __LINE__);
        return AR_EBADPARAM;
    }
    gpr_dl_lx_ports[dest_domain_id] = dl_lx_port;

    return AR_EOK;
}

uint32_t ipc_dl_lx_deinit(uint32_t src_domain_id, uint32_t dest_domain_id)
{
   uint32_t status = AR_EOK;

   status = gpr_dl_lx_local_deinit(src_domain_id, dest_domain_id);
   return status;
}

static uint32_t gpr_dl_lx_send(uint32_t domain_id, void *buf, uint32_t size)
{
    int32_t status;
    gpr_dl_lx_port_t *dl_lx_port;

    if ((dl_lx_port = gpr_dl_lx_ports[domain_id]) == NULL) {
        AR_LOG_ERR(LOG_TAG,"%s:%d port domain %d not initialized", __func__, __LINE__,
              domain_id);
        return AR_ENOTEXIST;
    }
    AR_LOG_DEBUG(LOG_TAG,"%s:Sending buffer of size %d to driver",__func__, size);
    status = write(dl_lx_port->drv_fd, buf, size);
    if (status < 0) {
        AR_LOG_ERR(LOG_TAG,"%s:%d write to driver failed %d", __func__, __LINE__, errno);
        if (errno == ENETRESET)
            return AR_ESUBSYSRESET;
        else
            return AR_EFAILED;
    }
    dl_lx_port->send_done(buf, size);
    AR_LOG_DEBUG(LOG_TAG,"%s:Send done",__func__);
    return AR_EOK;
}

static uint32_t gpr_dl_lx_receive_done(uint32_t domain_id, void *buf)
{
    uint32_t status;
    gpr_dl_lx_port_t *dl_lx_port;

    if ((dl_lx_port = gpr_dl_lx_ports[domain_id]) == NULL) {
        AR_LOG_ERR(LOG_TAG,"%s:%d port domain %d not initialized", __func__, __LINE__,
              domain_id);
        return AR_ENOTEXIST;
    }
    status = put_buffer(dl_lx_port, buf);
    return status;
}
