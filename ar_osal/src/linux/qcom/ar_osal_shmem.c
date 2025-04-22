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
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <linux/msm_ion.h>
#include <sys/ioctl.h>
#include "ar_osal_log.h"
#include <linux/msm_audio.h>

#if TARGET_ION_ABI_VERSION >= 2
#include <ion/ion.h>
#endif

#include "ar_osal_shmem.h"
#include "ar_osal_error.h"

/* Memory pointer to be 4K aligned for ADSP, MDSP,SDSP and CDSP */
#define  SHMEM_4K_ALIGNMENT       0x1000
#define ION_DRIVER_PATH "/dev/msm_audio_ion"
#define ION_DRIVER_PATH_CMA "/dev/msm_audio_ion_cma"

/* Systems without SMMU use AUDIO_HEAP for ion allocations */
#ifdef USE_AUDIO_HEAP_ID
#define HEAP_MASK   ION_HEAP(ION_AUDIO_HEAP_ID)
#else
#define HEAP_MASK   ION_HEAP(ION_SYSTEM_HEAP_ID)
#endif
#ifdef TARGET_USES_ION_CMA_MEMORY
#define HEAP_MASK_CMA   ION_HEAP(ION_AUDIO_ML_HEAP_ID)
#endif

typedef struct ar_shmem_handle_data {
   /*ion fd created on ion memory allocation*/
   int64_t ion_mem_fd;
#ifndef TARGET_ION_ABI_VERSION
   /* ion userspace handle used in legagcy ion implementation*/
   int32_t ion_user_handle;
#endif
} ar_shmem_handle_data_t;

typedef struct ar_shmem_pdata {
    int32_t ion_handle;
    int ion_fd;
    int ion_fd_cma;
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
_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_shmem_init(void)
{
    int32_t status = AR_EOK;
    pthread_mutex_lock(&ar_shmem_lock);
    if (NULL != pdata) {
        AR_LOG_INFO(AR_OSAL_SHMEM_LOG_TAG,"%s: init already done\n", __func__);
        goto end;
    }

    pdata = (ar_shmem_pdata_t *) malloc(sizeof(ar_shmem_pdata_t));
    if (NULL == pdata) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s: malloc failed\n", __func__);
        status = AR_ENOMEMORY;
        goto end;
    }
    /*initialize the ion_handle to invalid handle */
    pdata->ion_handle = -1;
#if TARGET_ION_ABI_VERSION >= 2
    pdata->ion_handle = ion_open();
#else
    pdata->ion_handle = open("/dev/ion", O_RDONLY | O_DSYNC);
#endif
    if (pdata->ion_handle < 0) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:ion dev open failed errno:%d\n", __func__, pdata->ion_handle);
        status = AR_ENOTEXIST;
        free(pdata);
        goto end;
    }
    pdata->ion_fd = open(ION_DRIVER_PATH, O_RDWR);
    if (pdata->ion_fd < 0) {
        status = AR_ENOTEXIST;
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s ion fd open failed %s status %d\n", __func__, ION_DRIVER_PATH, status);
        goto end;
    }
    AR_LOG_INFO(AR_OSAL_SHMEM_LOG_TAG,"%s ion fd open success %s\n", __func__, ION_DRIVER_PATH);
#ifdef  TARGET_USES_ION_CMA_MEMORY
    pdata->ion_fd_cma = open(ION_DRIVER_PATH_CMA, O_RDWR);
    if (pdata->ion_fd_cma < 0) {
        status = AR_ENOTEXIST;
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s ion fd open failed %s status %d\n",
        __func__, ION_DRIVER_PATH_CMA, status);
        goto end;
    }
    AR_LOG_INFO(AR_OSAL_SHMEM_LOG_TAG,"%s ion fd open success %s\n", __func__, ION_DRIVER_PATH_CMA);
#else
    pdata->ion_fd_cma = 0;
    AR_LOG_INFO(AR_OSAL_SHMEM_LOG_TAG,"%s ion CMA memory not used in this  platform", __func__);
#endif
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
_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_shmem_alloc(_Inout_ ar_shmem_info *info)
{
    int32_t status = AR_EOK;
    ar_shmem_cache_type_t ctype;
    ar_shmem_handle_data_t *shmem_handle = NULL;
    uint32_t heap_mask;
#ifndef TARGET_ION_ABI_VERSION
    struct ion_fd_data fd_data;
    struct ion_allocation_data alloc_data;
#endif

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
    info->pa_lsw = 0;
    info->pa_msw = 0;
    info->index_type = AR_SHMEM_BUFFER_OFFSET;
    info->mem_type = AR_SHMEM_PHYSICAL_MEMORY;

    if (info->cache_type == AR_SHMEM_CACHED) {
        ctype = 1;
    } else {
        ctype = 0;
    }

#ifdef TARGET_USES_ION_CMA_MEMORY
    heap_mask = ((info->flags) & (1 << AR_SHMEM_SHIFT_HW_ACCELERATOR_FLAG)) ? HEAP_MASK_CMA : HEAP_MASK;
#else
    heap_mask = HEAP_MASK;
#endif

#if TARGET_ION_ABI_VERSION >= 2
    status = ion_alloc_fd(pdata->ion_handle, info->buf_size,
                          SHMEM_4K_ALIGNMENT, heap_mask,
                          ctype, &shmem_handle->ion_mem_fd);
    if (status) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s: ION alloc failed errno: %d\n",
                     __func__, status);
        shmem_handle->ion_mem_fd = -1;
        status = AR_ENOMEMORY;
        goto end;
    }
#else
    alloc_data.len = info->buf_size;
    alloc_data.align = SHMEM_4K_ALIGNMENT;
    alloc_data.heap_id_mask = heap_mask;
    alloc_data.flags = ctype;
        /* Initialize handle to 0 */
    alloc_data.handle = 0;
    status = ioctl(pdata->ion_handle, ION_IOC_ALLOC, &alloc_data);
    if (status) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s: ION alloc failed, errno: %d\n",
                     __func__, status);
        status = AR_ENOMEMORY;
        goto end;
    }
    fd_data.handle = alloc_data.handle;
    shmem_handle->ion_user_handle = (uint32_t)alloc_data.handle;
    status = ioctl(pdata->ion_handle, ION_IOC_SHARE, &fd_data);
    if (status) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s: ION_IOC_SHARE, errno: %d\n",
                 __func__, status);
        status = AR_ENOMEMORY;
        goto err1;
    }
    shmem_handle->ion_mem_fd = fd_data.fd;
#endif
    info->vaddr = mmap(0, info->buf_size,
                  PROT_READ | PROT_WRITE, MAP_SHARED,
                  shmem_handle->ion_mem_fd, 0);
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
    info->ipa_lsw = shmem_handle->ion_mem_fd;
    AR_LOG_VERBOSE(AR_OSAL_SHMEM_LOG_TAG," SHMEM: cache_type(0x%x)|memory_type(0x%x)|index_type(0x%x) buf_size(0x%x)|vaddr(0x%pK)| ipa_lsw(0x%x)| status(0x%x) ",
                     ctype, info->mem_type, info->index_type, info->buf_size, info->vaddr, info->ipa_lsw, status);

    if ((info->flags) & (1 << AR_SHMEM_SHIFT_HW_ACCELERATOR_FLAG)) {
       if (pdata->ion_fd_cma) {
           status = ioctl(pdata->ion_fd_cma, IOCTL_MAP_PHYS_ADDR, info->ipa_lsw);
       } else {
          status = AR_ENOTEXIST;
          AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:ION CMA memory does not exist  %d\n",
          __func__, status);
       }
    } else {
       status = ioctl(pdata->ion_fd, IOCTL_MAP_PHYS_ADDR, info->ipa_lsw);
    }
    if (status) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:Physical address map failed status %d\n", __func__, status);
        goto err2;
    }
    goto end;

err2:
    close(shmem_handle->ion_mem_fd);
#ifndef TARGET_ION_ABI_VERSION
err1:
    status = ioctl(pdata->ion_handle, ION_IOC_FREE,
                   &shmem_handle->ion_user_handle);
    if (status < 0)
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s: ION Free failed %d\n", __func__, status);
#endif
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
_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_shmem_free(_In_ ar_shmem_info *info)
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
    munmap(info->vaddr, info->buf_size);
    if ((info->flags) & (1 << AR_SHMEM_SHIFT_HW_ACCELERATOR_FLAG)) {
       if (pdata->ion_fd_cma) {
           status = ioctl(pdata->ion_fd_cma, IOCTL_UNMAP_PHYS_ADDR, shmem_handle->ion_mem_fd);
       } else {
          status = AR_ENOTEXIST;
          AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:ION CMA memory does not exist  %d\n",
           __func__, status);
       }
    } else {
       status = ioctl(pdata->ion_fd, IOCTL_UNMAP_PHYS_ADDR, shmem_handle->ion_mem_fd);
    }
    if (status) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:unmap failed. status %d\n", __func__, status);
    }
    if (shmem_handle->ion_mem_fd) {
        close(shmem_handle->ion_mem_fd);
    } else {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s Invalid ion_mem_fd \n", __func__);
    }
#ifndef TARGET_ION_ABI_VERSION
    status = ioctl(pdata->ion_handle, ION_IOC_FREE,
                   &shmem_handle->ion_user_handle);
    if (status < 0)
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s: ION Free failed %d\n", __func__, status);
#endif
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
_IRQL_requires_max_(PASSIVE_LEVEL)
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
    shmem_handle->ion_mem_fd = (((int64_t)info->pa_msw << 32) | info->pa_lsw);

    info->vaddr = mmap(0, info->buf_size,
                  PROT_READ | PROT_WRITE, MAP_SHARED,
                  shmem_handle->ion_mem_fd, 0);
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
    status = ioctl(pdata->ion_fd, IOCTL_MAP_PHYS_ADDR, info->ipa_lsw);
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
_IRQL_requires_max_(PASSIVE_LEVEL)
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
    munmap(info->vaddr, info->buf_size);
    status = ioctl(pdata->ion_fd, IOCTL_UNMAP_PHYS_ADDR, shmem_handle->ion_mem_fd);
    if (status) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:unmap failed. status %d\n", __func__, status);
    }
    AR_LOG_VERBOSE(AR_OSAL_SHMEM_LOG_TAG,"%s:shmem heap fd %d\n", __func__, shmem_handle->ion_mem_fd);
    close(shmem_handle->ion_mem_fd);
    free(shmem_handle);
    info->metadata = 0;
    info->vaddr = NULL;
end:
    pthread_mutex_unlock(&ar_shmem_lock);
    return status;
}

/*
 * Helps to hyp assign physical memory between source and destination sub systems.
 *
 * \param[in] info: info: pointer to ar_shmem_hyp_assign_phys_info.
 *
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 */
_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_shmem_hyp_assign_phys(ar_shmem_hyp_assign_phys_info *info)
{
    int32_t status = AR_EOK;
    uint32_t shm_addr_lsw = 0;
    if (NULL == pdata) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:not in init state\n", __func__);
        status = AR_EBADPARAM;
        goto end;
    }
    for (int i = 0; i < info->phys_addr_list_size; i++) {
         shm_addr_lsw = (info->phys_addr_list->phys_addr) & 0xFFFFFFFF;
         for (int j = 0; j < info->src_sys_list_size; j++) {
#ifdef TARGET_USES_ION_CMA_MEMORY
             if (pdata->ion_fd_cma) {
                if (info->src_sys_list[j] == AR_APSS) {
                    status = ioctl(pdata->ion_fd_cma, IOCTL_MAP_HYP_ASSIGN, shm_addr_lsw);
                } else {
                   status = ioctl(pdata->ion_fd_cma, IOCTL_UNMAP_HYP_ASSIGN, shm_addr_lsw);
                }
                if (status) {
                    AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:hyp_assign failed. status %d\n",
                    __func__, status);
                }
             }
             else
#endif
             {
                status = AR_ENOTEXIST;
                AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:ION CMA memory does not exist  %d\n",
                __func__, status);
             }
        }
         info->phys_addr_list++;
    }
end:
    return status;
}

/**
* \brief ar_shmem_get_uid.
* Get associated unique identifier(UID) for the shared memory
* pointed by alloc_handle, platform which doesn`t support UID
* should return alloc_handle as UID with expectation of
* alloc_handle being unique.
*
* \param[in]  alloc_handle: handle for the shared memory.
* \param[out] uid: unique identifier to the shmem.
* \return
*  0 -- Success
*  Nonzero -- Failure
*/
_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_shmem_get_uid(uint64_t alloc_handle, uint64_t *uid)
{
    int32_t status = AR_EOK;
    struct stat file_stat;

    if (NULL == uid) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:%d Invalid parameter passed \n",
                    __func__, __LINE__);
        status = AR_EBADPARAM;
        goto end;
    }
    /*Note that on LX platform alloc_handle should always be a file descriptor*/
    status = fstat(alloc_handle, &file_stat);
    if (status == -1) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s:%d fstat api failed %d\n",
                   __func__, __LINE__, errno);
        status = AR_EFAILED;
        goto end;
    }
    *uid = file_stat.st_ino;

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
_IRQL_requires_max_(PASSIVE_LEVEL)
int32_t ar_shmem_deinit(void)
{
    pthread_mutex_lock(&ar_shmem_lock);
    if (pdata == NULL) {
        AR_LOG_ERR(AR_OSAL_SHMEM_LOG_TAG,"%s not in init state\n", __func__);
        return AR_EOK;
    }
    if (pdata->ion_handle){
#if TARGET_ION_ABI_VERSION >= 2
        ion_close(pdata->ion_handle);
#else
        close(pdata->ion_handle);
#endif
    }
    if (pdata->ion_fd)
        close(pdata->ion_fd);
    if (pdata->ion_fd_cma)
        close(pdata->ion_fd_cma);
    free(pdata);
    pdata = NULL;
    pthread_mutex_unlock(&ar_shmem_lock);
    return AR_EOK;
}
