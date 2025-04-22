#ifndef AR_OSAL_FILE_IO_H
#define AR_OSAL_FILE_IO_H

/**
 * \file ar_osal_file_io.h
 * \brief
 *      Defines public APIs for file IO operations.
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_osal_types.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/**
* Structures and Typedefs
*/
typedef void *ar_fhandle;

/** Opens for reading, file position is at the beginning. 
  * If the file does not exist, the ar_fopen call fails.
  */
#define AR_FOPEN_READ_ONLY                      (0x00000001)
/** Opens an empty file for writing. If the given file exists, its contents are destroyed,
  * file position is at the beginning.
  */
#define AR_FOPEN_WRITE_ONLY                     (0x00000002)
/** Opens an empty file for both reading and writing. If the file exists,
  * its contents are destroyed, file position is at the beginning.
  */
#define AR_FOPEN_READ_WRITE                     (AR_FOPEN_READ_ONLY|AR_FOPEN_WRITE_ONLY)
/** Open for appending (writing at end of file), this parameter alone cannot be used with ar_fopen.
  */
#define AR_FOPEN_APPEND                         (0x00000004)
/** Opens for writing at the end of the file(appending). Creates the file if it does not exist.
  */
#define AR_FOPEN_WRITE_ONLY_APPEND              (AR_FOPEN_WRITE_ONLY|AR_FOPEN_APPEND)
/** Opens for reading and appending. Creates the file if it does not exist. The file
  * position for reading is at the beginning of the file, but write data is always appended
  * to the end of the file.
  */
#define AR_FOPEN_READ_WRITE_APPEND              (AR_FOPEN_READ_ONLY|AR_FOPEN_WRITE_ONLY|AR_FOPEN_APPEND)
/** Opens for both reading and writing. The file must exist, file position is at the beginning of file.
  */
#define AR_FOPEN_READ_ONLY_WRITE                (0x00000008)

typedef enum ar_fseek_reference {
    /**
     *The starting point is zero or the beginning of the file.
     */
    AR_FSEEK_BEGIN = 0,
    /**
     *The starting point is the current end-of-file position.
     */
    AR_FSEEK_END = 1,
    /**
     *The start point is the current value of the file pointer.
     */
    AR_FSEEK_CURRENT = 2
} ar_fseek_reference_t ;

/**
 * \brief ar_fopen
 *        open a file or create if does not exist. 
 *  
 * \param[out] handle: Handle to the file.
 * \param[in]  path: Absolute file path.
 * \param[in]  access: Type of access 
 *                 AR_FILE_OPEN_READ_ONLY
 *                 AR_FILE_OPEN_WRITE_ONLY
 *                 AR_FILE_OPEN_READ_WRITE
 *                 AR_FOPEN_WRITE_ONLY_APPEND
 *                 AR_FOPEN_READ_WRITE_APPEND
 *                 AR_FOPEN_READ_ONLY_WRITE
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 */
int32_t ar_fopen(ar_fhandle *handle,
                          const char_t *path,
                          uint32_t access);

/**
 * \brief ar_fsize
 *        Return file size.
 *   
 * \param[in] handle: Handle to the file.
 * 
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 */
size_t ar_fsize(ar_fhandle handle);

/**
 * \brief ar_fmap
 *          Map a file into Data Memory for Read Only access
 *
 * Whether this allocates heap memory for the buffer or not
 * is platform dependent. On some platforms with low memory
 * this may map non-volatile storage into a readable memory window instead.
 * To free any possible resources allocated by this call, the caller
 * MUST call ar_funmap
 *
 * \param[in] handle: Handle to the file
 * \param[out] fbuffer: A pointer to the read-only Data memory buffer
 * 
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 */
int32_t ar_fmap(ar_fhandle handle, 
                const void **fbuffer);

/**
 * \brief ar_funmap
 *          Un-map a file from Data Memory
 *
 * This call releases a buffer obtained by a previous call to ar_fmap
 * and frees any resources that may be in use by it.
 * The file still needs to be closed by a call to ar_fclose
 *
 * \param[in] fbuffer: The pointer to the file buffer obtained by ar_fmap
 * 
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 */
int32_t ar_funmap(const void *fbuffer);

/**
 * \brief  ar_fseek
 *           Move the file pointer for read/write to the required offset.
 *   
 * \param[in] handle: Handle to the file.
 * \param[in] offset: The number of bytes to move the file pointer.
 * \param[in] ref: Refer to ar_fseek_reference_t for options.
 * \return
 * 0 -- Success
 * Nonzero -- Failure
 */
int32_t ar_fseek(ar_fhandle handle, 
                   size_t offset, 
                   ar_fseek_reference_t ref);

/**
 * \brief  ar_fread
 *            Read from file.
 *   
 * \param[in] handle: Handle to the file.
 * \param[in_out] buf_ptr: Buffer pointer to read data into.
 * \param[in] read_size: Data size to read from file.
 * \param[in_out] bytes_read: Actual data size read from file.
 *
 * \return
 * 0 -- Success
 * Nonzero -- Failure
 */
int32_t ar_fread(ar_fhandle handle,
                           void *buf_ptr,
                           size_t read_size,
                           size_t *bytes_read);

/**
 * \brief  ar_fwrite
 *           write to file.
 *   
 * \param[in] handle: Handle to the file.
 * \param[in] buf_ptr: Buffer pointer to write data from.
 * \param[in] write_size: Data size to write into file.
 * \param[in_out] bytes_written: Actual data size written into file.
 * 
 * \return
 * 0 -- Success
 * Nonzero -- Failure
 */
int32_t ar_fwrite(ar_fhandle handle,
                            void *buf_ptr,
                            size_t write_size,
                            size_t *bytes_written);

/**
 * \brief  ar_fclose 
 * \param[in] handle: Handle to the file.  
 * \return
 * 0 -- Success
 * Nonzero -- Failure
 */
int32_t ar_fclose(ar_fhandle handle);

/**
 *  \brief  ar_fdelete
 *  \param[in]  path: Absolute file path.
 *  \return
 *  0 -- Success
 *  Nonzero -- Failure
 */
int32_t ar_fdelete(const char_t *path);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif //AR_OSAL_FILE_IO_H
