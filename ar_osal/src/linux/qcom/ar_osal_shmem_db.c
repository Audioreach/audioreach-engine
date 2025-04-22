/**
 *
 * \file ar_osal_shmem.c
 *
 * \brief
 *      This file has implementation of shared memory allocation for DSP.

 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#define  AR_OSAL_SHMEM_LOG_TAG     "COSH"
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/msm_audio.h>
#ifdef AR_OSAL_USE_CUTILS
#include <cutils/properties.h>
#endif
#include "ar_osal_shmem.h"
#include "ar_osal_error.h"
#include "ar_osal_log.h"

#define  SHMEM_4K_ALIGNMENT       0x1000
#define AR_MEM_DRIVER_PATH "/dev/msm_audio_ion"
#define AR_MEM_DRIVER_PATH_CMA "/dev/msm_audio_ion_cma"
#define DMABUF_SYS_HEAP_PATH "/dev/dma_heap/system"
#define DMABUF_SYS_HEAP_PATH_UNCACHED "/dev/dma_heap/qcom,system-uncached"
#define DMABUF_SYS_HEAP_PATH_CMA "/dev/dma_heap/qcom,audio-ml"


typedef struct ar_shmem_handle_data {
  int32_t heap_fd;
  bool dma_sync_flag;
} ar_shmem_handle_data_t;

typedef struct ar_shmem_pdata {
  int dmabuf_fd;
  int dmabuf_fd_cma;
  int armem_fd;
  int armem_fd_cma;
  bool dmabuf_cma_mem_enabled;
} ar_shmem_pdata_t;

static ar_shmem_pdata_t *pdata = NULL;

static pthread_mutex_t ar_shmem_lock = PTHREAD_MUTEX_INITIALIZER;

/**
 * \brief ar_shmem_validate_sys_id
 *       internal function to validate supported SYS IDs.
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 */
int32_t ar_shmem_validate_sys_id(uint8_t num_sys_id, uint8_t *sys_id)
{
    int32_t status = AR_EOK;
    if (0 == num_sys_id || NULL == sys_id)
    {
        status = AR_EBADPARAM;
        goto end;
    }

    for (uint8_t i = 0; i < num_sys_id; i++)
    {
        if (AR_AUDIO_DSP != sys_id[i] &&
            AR_MODEM_DSP != sys_id[i] &&
            AR_SENSOR_DSP != sys_id[i] &&
            AR_COMPUTE_DSP != sys_id[i] &&
            AR_APSS != sys_id[i] &&
            AR_APSS2 != sys_id[i])
        {
            status = AR_EBADPARAM;
            break;
        }
    }

end:
    return status;
}

/*
 *\brief ar_shmem_init
 *        initialize shared memory interface.
 * \return
 * 0 -- Success
 *  Nonzero -- Failure
 */
int32_t ar_shmem_init(void)
{
    int32_t status = AR_EOK;
    bool use_uncached_heap = false;

    pthread_mutex_lock(&ar_shmem_lock);
    if (NULL != pdata) {
      AR_LOG_INFO(AR_OSAL_SHMEM_LOG_TAG,"%s: init already done\n", __func__);
      goto end;
    }

    pdata = (ar_shmem_pdata_t *) calloc(1, sizeof(ar_shmem_pdata_t));
    if (NULL == pdata) {
      AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s: calloc failed\n", __func__);
      status = AR_ENOMEMORY;
      goto end;
    }

#ifdef AR_OSAL_USE_CUTILS
    use_uncached_heap = property_get_bool("vendor.audio.gsl.shmem.dmaheap.uncached", false);
#endif
    if (use_uncached_heap) {
        AR_LOG_INFO ("%s: Using dmaheap path: qcom,system-uncached\n", __func__);
        pdata->dmabuf_fd = open(DMABUF_SYS_HEAP_PATH_UNCACHED, O_RDONLY | O_CLOEXEC);
    } else {
        AR_LOG_INFO ("%s: Using dmaheap path: system\n", __func__);
        pdata->dmabuf_fd = open(DMABUF_SYS_HEAP_PATH, O_RDONLY | O_CLOEXEC);
    }
    /*Check if dma_buf heap initialize the dmabuf_handle to invalid handle */
    if (pdata->dmabuf_fd < 0) {
      AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:dmabuf dev open failed errno:%d\n", __func__, pdata->dmabuf_fd);
      status = AR_ENOTEXIST;
      goto end;
    }

    pdata->armem_fd = open(AR_MEM_DRIVER_PATH, O_RDWR);
    if (pdata->armem_fd < 0) {
      status = AR_ENOTEXIST;
      AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s armem fd open failed %s status %d\n", __func__, AR_MEM_DRIVER_PATH, status);
      close(pdata->dmabuf_fd);
      goto end;
    }
    AR_LOG_INFO(AR_OSAL_SHMEM_LOG_TAG,"%s armem fd open success %s\n", __func__, AR_MEM_DRIVER_PATH);

#ifdef AR_OSAL_USE_CUTILS
    pdata->dmabuf_cma_mem_enabled =
        property_get_bool("vendor.audio.feature.dmabuf.cma.memory.enable", false);
#endif
    if(pdata->dmabuf_cma_mem_enabled) {
        /*Check if dma_buf heap initialize the dmabuf_handle to invalid handle */
        pdata->dmabuf_fd_cma = open(DMABUF_SYS_HEAP_PATH_CMA, O_RDONLY | O_CLOEXEC);
        if (pdata->dmabuf_fd_cma <= 0) {
          AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:dmabuf fd cma dev open failed errno:%d\n",
            __func__, pdata->dmabuf_fd_cma);
          status = AR_ENOTEXIST;
          close(pdata->armem_fd);
          close(pdata->dmabuf_fd);
          goto end;
        }

        pdata->armem_fd_cma = open(AR_MEM_DRIVER_PATH_CMA, O_RDWR);
        if (pdata->armem_fd_cma <= 0) {
          status = AR_ENOTEXIST;
          AR_LOG_ERR("%s armem fd cma open failed %s status %d\n",
            __func__, AR_MEM_DRIVER_PATH_CMA, status);
          close(pdata->armem_fd);
          close(pdata->dmabuf_fd);
          close(pdata->dmabuf_fd_cma);
          goto end;
        }

    }

 end:
    pthread_mutex_unlock(&ar_shmem_lock);
    return status;
}

/*
 * \brief Allocates shared memory.
 *  Only non cached memory allocation supported.
 *  Size if multiple of 4KB and the returned is aligned to 4KB boundary.
 *  Buffer start address should be atleast 64bit multiple aligned.
 *
 * \param[in_out] info: pointer to ar_shmem_info.
 *
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 *
 */
int32_t ar_shmem_alloc(_Inout_ ar_shmem_info *info)
{
    int32_t status = AR_EOK;
    ar_shmem_handle_data_t *shmem_handle = NULL;
    struct dma_heap_allocation_data heap_data;
    struct dma_buf_sync sync;
    memset(&heap_data, 0, sizeof heap_data);
    memset(&sync, 0, sizeof sync);

    pthread_mutex_lock(&ar_shmem_lock);

    if (NULL == pdata) {
      AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:not in init state", __func__);
      status = AR_EUNSUPPORTED;
      goto end;
    }

    if (NULL == info || 0 == info->buf_size) {
      AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"Error: info(NULL)|buf_size(0) passed");
      status = AR_EBADPARAM;
      goto end;
    }

    if (AR_EOK != ar_shmem_validate_sys_id(info->num_sys_id,info->sys_id)) {
      AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG, "Error: invalid sys_id(%d) passed", info->sys_id);
      status = AR_EBADPARAM;
      goto end;
    }
    shmem_handle = (ar_shmem_handle_data_t *)calloc(1, sizeof(ar_shmem_handle_data_t));
    if (NULL == shmem_handle) {
      AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:malloc for shmem handle failed\n", __func__);
      status = AR_ENOMEMORY;
      goto end;
    }

    info->ipa_lsw = 0;
    info->ipa_msw = 0;
    info->metadata = 0;
    info->pa_lsw = 0;
    info->pa_msw = 0;
    info->index_type = AR_SHMEM_BUFFER_OFFSET;
    info->mem_type = AR_SHMEM_PHYSICAL_MEMORY;

    heap_data.len = info->buf_size;
    heap_data.fd_flags = O_RDWR;

    if (((info->flags) & (1 << AR_SHMEM_SHIFT_HW_ACCELERATOR_FLAG)) && pdata->dmabuf_cma_mem_enabled) {
        status = ioctl(pdata->dmabuf_fd_cma, DMA_HEAP_IOCTL_ALLOC, &heap_data);
        if (status) {
          AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s: DMA heap alloc failed, errno: %d\n",
             __func__, status);
          status = AR_ENOMEMORY;
          goto end;
        }
    }
    else {
        status = ioctl(pdata->dmabuf_fd, DMA_HEAP_IOCTL_ALLOC, &heap_data);
        if (status) {
          AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s: DMA heap alloc failed, errno: %d\n",
             __func__, status);
          status = AR_ENOMEMORY;
          goto end;
        }
    }

    info->vaddr = mmap(0, info->buf_size,
		       PROT_READ | PROT_WRITE, MAP_SHARED,
		       heap_data.fd, 0);
    if (info->vaddr == MAP_FAILED) {
      AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s: mmap failed \n", __func__);
      status = AR_ENOMEMORY;
      goto err1;
    }

    if ((uint64_t)info->vaddr % SHMEM_4K_ALIGNMENT) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG, "Error: 4k alignment check failed vaddr(0x%pK)", info->vaddr);
        info->vaddr = NULL;
        status = AR_EUNEXPECTED;
        goto err2;
    }

    sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW;
    status = ioctl(heap_data.fd, DMA_BUF_IOCTL_SYNC, &sync);
    if (status) {
      AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s: DMA heap set access failed, errno: %d\n",
		 __func__, status);
      status = AR_EUNEXPECTED;
      goto err2;
    }

    shmem_handle->heap_fd = heap_data.fd;
    shmem_handle->dma_sync_flag = true;
    info->metadata = (uint64_t)shmem_handle;
    info->ipa_lsw = shmem_handle->heap_fd;
    AR_LOG_VERBOSE(AR_OSAL_SHMEM_LOG_TAG," SHMEM: cached|memory_type(0x%x)|index_type(0x%x) buf_size(0x%x)|vaddr(0x%pK)| ipa_lsw(0x%x)| status(0x%x) ",
		   info->mem_type, info->index_type, info->buf_size, info->vaddr, info->ipa_lsw, status);
     if (((info->flags) & (1 << AR_SHMEM_SHIFT_HW_ACCELERATOR_FLAG)) && pdata->dmabuf_cma_mem_enabled) {
        if (pdata->armem_fd_cma) {
            status = ioctl(pdata->armem_fd_cma, IOCTL_MAP_PHYS_ADDR, info->ipa_lsw);
        } else {
            status = AR_ENOTEXIST;
            AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:ION CMA memory does not exist  %d\n",
                __func__, status);
        }
     } else
         status = ioctl(pdata->armem_fd, IOCTL_MAP_PHYS_ADDR, info->ipa_lsw);
    if (status) {
      AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:Physical address map failed status %d\n",
        __func__, status);
      goto err2;
    }
    goto end;

 err2:
    munmap(info->vaddr, info->buf_size);
    shmem_handle->dma_sync_flag = false;

 err1:
    close(shmem_handle->heap_fd);
    free(shmem_handle);

 end:
    pthread_mutex_unlock(&ar_shmem_lock);
    return status;
}

/*
 * Frees shared memory.
 *
 * \param[in] info: pointer to ar_shmem_info.
 *
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 */
int32_t ar_shmem_free(_In_ ar_shmem_info *info)
{
    int32_t status = AR_EOK;
    ar_shmem_handle_data_t *shmem_handle = NULL;
    struct dma_buf_sync sync;

    pthread_mutex_lock(&ar_shmem_lock);

    if (NULL == pdata) {
      AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:not in i state\n", __func__);
      status = AR_EBADPARAM;
      goto end;
    }

    if (NULL == info || NULL == info->vaddr){
      AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:invalid info %pK\n", __func__, info);
      status = AR_EBADPARAM;
      goto end;
    }

    if (0 == info->metadata || 0 == info->buf_size){
      AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:invalid params handle %pK buf_size %zu\n",
		 __func__, info->metadata, info->buf_size);
      status = AR_EBADPARAM;
      goto end;
    }

    if (AR_EOK != ar_shmem_validate_sys_id(info->num_sys_id,info->sys_id)) {
      AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG, "Error: invalid sys_id(%d) passed", info->sys_id);
      status = AR_EBADPARAM;
      goto end;
    }

    shmem_handle = (ar_shmem_handle_data_t *)(intptr_t)info->metadata;

    if (NULL == shmem_handle) {
       AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s: shmem_handle is NULL\n",
           __func__);
       status = AR_EBADPARAM;
       goto end;
    }
    if (shmem_handle && shmem_handle->dma_sync_flag) {
      sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW;
      status = ioctl(shmem_handle->heap_fd, DMA_BUF_IOCTL_SYNC, &sync);
      if (status) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s: DMA heap set access failed, errno: %d\n",
           __func__, status);
      }
      shmem_handle->dma_sync_flag = false;
    }
    munmap(info->vaddr, info->buf_size);
    if (shmem_handle->heap_fd) {
        if (((info->flags) & (1 << AR_SHMEM_SHIFT_HW_ACCELERATOR_FLAG)) && pdata->dmabuf_cma_mem_enabled) {
            if (pdata->armem_fd_cma) {
                status = ioctl(pdata->armem_fd_cma, IOCTL_UNMAP_PHYS_ADDR, shmem_handle->heap_fd);
            } else {
                status = AR_ENOTEXIST;
                AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:ION CMA memory does not exist  %d\n",
                    __func__, status);
            }
        } else {
            status = ioctl(pdata->armem_fd, IOCTL_UNMAP_PHYS_ADDR, shmem_handle->heap_fd);
        }
        if (status) {
            AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:unmap failed. status %d\n", __func__, status);
        }
        close(shmem_handle->heap_fd);
    } else {
      AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s Invalid dmabuf_fd \n", __func__);
    }
    if (shmem_handle)
      free(shmem_handle);
    info->metadata = 0;
    info->vaddr = NULL;
 end:
    pthread_mutex_unlock(&ar_shmem_lock);
    return status;
}

/**
 * \brief Helps map memory with SMMU an already allocated shared memory
 * for a give sub system.
 * Size should be multiple of 4KB boundary.
 * Buffer start address should be 64bit aligned.
 *
 * \param[in_out] info: pointer to ar_shmem_info.
 *                     required input parameters in ar_shmem_info
 *                     ar_shmem_info_t.cache_type
 *                     ar_shmem_info_t.buf_size
 *                     ar_shmem_info_t.pa_lsw
 *                     ar_shmem_info_t.pa_msw
 *                     ar_shmem_info_t.num_sys_id
 *                     ar_shmem_info_t.sys_id
 *                     required output parameters in ar_shmem_info
 *                     ar_shmem_info_t.ipa_lsw
 *                     ar_shmem_info_t.ipa_msw
 *
 *\return
 * 0 -- Success
 * Nonzero -- Failure
 *
 */
int32_t ar_shmem_map(_Inout_ ar_shmem_info *info)
{
    int32_t status = AR_EOK;
    ar_shmem_handle_data_t *shmem_handle = NULL;

    pthread_mutex_lock(&ar_shmem_lock);
    if (NULL == pdata) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:not in init state", __func__);
        status = AR_EUNSUPPORTED;
        goto end;
    }

    if (NULL == info || 0 == info->buf_size) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"Error: info(NULL)|buf_size(0) passed");
        status = AR_EBADPARAM;
        goto end;
    }

    if (0 == info->pa_lsw && 0 == info->pa_msw) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"Error: pa_lsw:0x%x|pa_msw:0x%x passed",
                   info->pa_lsw, info->pa_msw);
        status = AR_EBADPARAM;
        goto end;
    }

    if (AR_EOK != ar_shmem_validate_sys_id(info->num_sys_id,info->sys_id)) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG, "Error: invalid sys_id(%d) passed", info->sys_id);
        status = AR_EBADPARAM;
        goto end;
    }

    shmem_handle = (ar_shmem_handle_data_t *)
                    malloc(sizeof(ar_shmem_handle_data_t));
    if (NULL == shmem_handle) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:malloc for shmem handle failed\n", __func__);
        status = AR_ENOMEMORY;
        goto end;
    }

    info->ipa_lsw = 0;
    info->ipa_msw = 0;
    info->metadata = 0;
    info->index_type = AR_SHMEM_BUFFER_OFFSET;
    info->mem_type = AR_SHMEM_PHYSICAL_MEMORY;

    /*
     *This api is called by clients when the clients already have allocated shared memory
     *and now we just need to map it with DSP. On LX platforms what we recieve from clients
     *is the ion fd, which is passed in the form of pa_lsw and pa_msw. Hence we extract the same
     *and pass it on to the audio ion driver.
     */
    shmem_handle->heap_fd = (((int64_t)info->pa_msw << 32) | info->pa_lsw);

    info->vaddr = mmap(0, info->buf_size,
                  PROT_READ | PROT_WRITE, MAP_SHARED,
                  shmem_handle->heap_fd, 0);
    if (info->vaddr == MAP_FAILED) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s: mmap failed \n", __func__);
        status = AR_ENOMEMORY;
        goto err2;
    }

    if ((uint64_t)info->vaddr % SHMEM_4K_ALIGNMENT) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG, "Error: 4k alignment check failed vaddr(0x%pK)", info->vaddr);
        info->vaddr = NULL;
        status = AR_EUNEXPECTED;
        goto err2;
    }
    info->metadata = (uint64_t)shmem_handle;
    info->ipa_lsw = info->pa_lsw;
    AR_LOG_VERBOSE(AR_OSAL_SHMEM_LOG_TAG," SHMEM: memory_type(0x%x)|index_type(0x%x)"
                   "buf_size(0x%x)|vaddr(0x%pK)| ipa_lsw(0x%x)| status(0x%x) ",info->mem_type,
                    info->index_type, info->buf_size, info->vaddr, info->ipa_lsw, status);
    if (((info->flags) & (1 << AR_SHMEM_SHIFT_HW_ACCELERATOR_FLAG)) && pdata->dmabuf_cma_mem_enabled) {
        if (pdata->armem_fd_cma) {
            status = ioctl(pdata->armem_fd_cma, IOCTL_MAP_PHYS_ADDR, info->ipa_lsw);
        } else {
            status = AR_ENOTEXIST;
            AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:ION CMA memory does not exist  %d\n",
                __func__, status);
        }
    } else
        status = ioctl(pdata->armem_fd, IOCTL_MAP_PHYS_ADDR, info->ipa_lsw);
    if (status) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:Physical address map failed status %d\n", __func__, status);
        goto err2;
    }
    goto end;

err2:
    free(shmem_handle);
end:
    pthread_mutex_unlock(&ar_shmem_lock);
    return status;
}

/**
 *\brief Helps unmap the shared memory allocated externally with SMMU
 *
 *\param[in] info: pointer to ar_shmem_info.
 *            required input parameters in ar_shmem_info
 *                     ar_shmem_info_t.cache_type
 *                     ar_shmem_info_t.buf_size
 *                     ar_shmem_info_t.pa_lsw
 *                     ar_shmem_info_t.pa_msw
 *                     ar_shmem_info_t.num_sys_id
 *                     ar_shmem_info_t.sys_id
 *
 *\return
 * 0 -- Success
 * Nonzero -- Failure
 *
 */
int32_t ar_shmem_unmap(_In_ ar_shmem_info *info)
{
    int32_t status = AR_EOK;
    ar_shmem_handle_data_t *shmem_handle;
    pthread_mutex_lock(&ar_shmem_lock);
    if (NULL == pdata) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:not in init state\n", __func__);
        status = AR_EBADPARAM;
        goto end;
    }

    if (NULL == info || NULL == info->vaddr){
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:invalid info %pK\n", __func__, info);
        status = AR_EBADPARAM;
        goto end;
    }

    if (0 == info->metadata || 0 == info->buf_size){
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:invalid params handle %pK buf_size %zu\n",
                     __func__, info->metadata, info->buf_size);
        status = AR_EBADPARAM;
        goto end;
    }

    if (AR_EOK != ar_shmem_validate_sys_id(info->num_sys_id,info->sys_id)) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG, "Error: invalid sys_id(%d) passed", info->sys_id);
        status = AR_EBADPARAM;
        goto end;
    }

    shmem_handle = (ar_shmem_handle_data_t *)(intptr_t)info->metadata;
    if (NULL == shmem_handle) {
       AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s: shmem_handle is NULL\n",
           __func__);
       status = AR_EBADPARAM;
       goto end;
    }

    munmap(info->vaddr, info->buf_size);
    if (shmem_handle->heap_fd) {
        if (((info->flags) & (1 << AR_SHMEM_SHIFT_HW_ACCELERATOR_FLAG) && pdata->dmabuf_cma_mem_enabled)) {
            if (pdata->armem_fd_cma) {
                status = ioctl(pdata->armem_fd_cma, IOCTL_UNMAP_PHYS_ADDR, shmem_handle->heap_fd);
            } else {
                status = AR_ENOTEXIST;
                AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:ION CMA memory does not exist  %d\n",
                    __func__, status);
            }
        } else
            status = ioctl(pdata->armem_fd, IOCTL_UNMAP_PHYS_ADDR, shmem_handle->heap_fd);
        if (status) {
            AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:unmap failed. status %d\n", __func__, status);
        }
        AR_LOG_VERBOSE(AR_OSAL_SHMEM_LOG_TAG,"%s:shmem heap fd %d\n", __func__, shmem_handle->heap_fd);
        close(shmem_handle->heap_fd);
    } else {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s Invalid dmabuf_fd \n", __func__);
    }
    if (shmem_handle)
        free(shmem_handle);
    info->metadata = 0;
    info->vaddr = NULL;
end:
    pthread_mutex_unlock(&ar_shmem_lock);
    return status;
}

/**
 * Helps to hyp assign physical memory between source and destination sub systems.
 *
 * \param[in] info: info: pointer to ar_shmem_hyp_assign_phys_info.
 *
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 */
int32_t ar_shmem_hyp_assign_phys(ar_shmem_hyp_assign_phys_info *info)
{
    int32_t status = AR_EOK;
    uint32_t shm_addr_lsw = 0;
    ar_shmem_handle_data_t *shmem_handle = NULL;
    struct dma_buf_sync sync;

    if (NULL == pdata) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:not in init state\n", __func__);
        status = AR_EBADPARAM;
        goto end;
    }

    if(!(pdata->dmabuf_cma_mem_enabled)) {
        return AR_EOK;
    }
    for (int i = 0; i < info->phys_addr_list_size; i++) {
        shm_addr_lsw = (info->phys_addr_list->phys_addr) & 0xFFFFFFFF;
        for (int j = 0; j < info->src_sys_list_size; j++) {
            if (info->src_sys_list[j] == AR_APSS) {
                if (info->metadata) {
                    shmem_handle = (ar_shmem_handle_data_t *)(intptr_t)info->metadata;
                    if (shmem_handle && shmem_handle->dma_sync_flag) {
                      sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW;
                      status = ioctl(shmem_handle->heap_fd, DMA_BUF_IOCTL_SYNC, &sync);
                      if (status) {
                        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s: DMA heap set access failed, errno: %d\n",
                          __func__, status);
                        status = AR_EBADPARAM;
                        goto end;
                      }
                      shmem_handle->dma_sync_flag = false;
                    }
                    AR_LOG_VERBOSE(AR_OSAL_SHMEM_LOG_TAG,"%s:IOCTL_MAP_HYP_ASSIGN\n", __func__);
                    status = ioctl(pdata->armem_fd_cma, IOCTL_MAP_HYP_ASSIGN, shm_addr_lsw);
                    if (status) {
                        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:hyp_assign failed. status %d\n",
                         __func__, status);
                    }
                } else {
                    AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:invalid params handle \n", __func__);
                    status = AR_EBADPARAM;
                    goto end;
                }
            } else {
                AR_LOG_VERBOSE(AR_OSAL_SHMEM_LOG_TAG,"%s:IOCTL_UNMAP_HYP_ASSIGN\n", __func__);
                status = ioctl(pdata->armem_fd_cma, IOCTL_UNMAP_HYP_ASSIGN, shm_addr_lsw);

                if (status) {
                     AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:unmap_hyp_assign failed. status %d\n",
                         __func__, status);
                }
            }
         }
         info->phys_addr_list++;
    }
end:
    return status;
}

/*
 * \brief ar_shmem_deinit.
 *
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 */
int32_t ar_shmem_deinit(void)
{

  pthread_mutex_lock(&ar_shmem_lock);

  if (pdata == NULL) {
    AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s not in init state\n", __func__);
    return AR_EOK;
  }

  if (pdata->dmabuf_fd){
    close(pdata->dmabuf_fd);
  }

  if (pdata->armem_fd){
    close(pdata->armem_fd);
  }

  if (pdata->dmabuf_fd_cma){
    close(pdata->dmabuf_fd_cma);
  }

  if (pdata->armem_fd_cma){
    close(pdata->armem_fd_cma);
  }

  free(pdata);
  pdata = NULL;
  pthread_mutex_unlock(&ar_shmem_lock);

  return AR_EOK;
}
