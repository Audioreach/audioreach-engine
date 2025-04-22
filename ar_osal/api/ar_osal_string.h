#ifndef AR_OSAL_STRING_H
#define AR_OSAL_STRING_H

/**
 * \file ar_osal_string.h
 * \brief
 *        Defines public APIs for string operations.
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_osal_types.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/**
 * \brief ar_strlen
 *        Get the length of the string.
 * \param[in] str: NULL terminated string buffer pointer.
 * \param[in] size: String buffer size in bytes.
 * \return
 *  Number of characters in the string, not including the terminating null
 *  character. If there is no null terminator in the buffer, provided
 *  string buffer size is returned to indicate the error.
 */
size_t ar_strlen(const char_t* str, size_t size);

/**
 * \brief ar_strcpy
 *        Copies character from source to destination string.
 * \param[in_out] str_dest: Destination string buffer.
 * \param[in] str_dest_size: Size in characters.
 * \param[in] str_src: Source string to copy from.
 * \param[in] cpy_size: Size in characters to copy from source string.
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 * note: Always null terminator is appended to the destination and string
 * truncation will occur if destination size is smaller or equal to characters copy size.
 */
int32_t ar_strcpy(char_t* str_dest, size_t str_dest_size, const char_t* str_src, size_t cpy_size);

/**
 * \brief ar_strcmp
 *        Compare characters in two given strings.
 * \param[in] str1: Null terminated string 1
 * \param[in] str2: Null terminated string 2
 * \param[in] num: number of characters to compare.
 * \return
 *  < 0   str1 < str2
 *    0   str1 == str2
 *  > 0   str1 > str2
 */
int32_t ar_strcmp(const char_t* str1, const char_t* str2, size_t num);

/**
* \brief ar_strcat
*        Append character from source to destination string.
* \param[in_out] str_dest: Destination string buffer.
* \param[in] str_dest_size: Size in characters.
* \param[in] str_src: Source string to copy from.
* \param[in] apnd_size: Size in characters to append from source string.
* \return
*  0 -- Success
*  Nonzero -- Failure
* note: Always null terminator is appended to the destination after concatenation and string
* truncation will occur if destination size is smaller or equal to source.
*/
int32_t ar_strcat(char_t* str_dest, size_t str_dest_size, const char_t* str_src, size_t apnd_size);

/**
 * \brief ar_strstr
 *        Search string in string.
 * \param[in] str: Null terminated string
 * \param[in] str_search: Null terminated string to search for.
 * \return
 *         Pointer to the first occurrence of str_search in str.
 *         NULL- if not found.
 */
char_t* ar_strstr(const char_t* str, const char_t* str_search);

/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------------------------*/
/**
* \brief ar_sprintf
*       Write formated data to a string.
* \param[in_out] str_dest: Destination string buffer.
* \param[in] str_dest_size: Size in bytes.
* \param[in] format: Format control string.
* \param[in] "..." : variable arguments.
* \return
*   Number of characters written to the buffer(null terminator not counted). 
*   Destination is appended with null terminator always, if data is greater than the
*   destination buffer size, truncation will occur with appended null terminator.
*/

/**
* \brief ar_sscanf
*       Read formated data from a string.
* \param[in] str_src: Stored string buffer.
* \param[in] format: Format control string.
* \param[in] "..." : variable arguments.
* \return
*  Number of fields successfully converted and assigned, the return value does not include 
*  fields that were read but not assigned. A return value of 0 indicates that no fields were assigned.
*/

#if defined(_WIN64) || defined(_WIN32) /* WIN32/WIN64 and Windows kernel.*/

#define ar_sprintf(str_dest, str_dest_size, format, ...)  _snprintf_s(str_dest, str_dest_size, _TRUNCATE, format, __VA_ARGS__)
#define ar_sscanf(str_src, format, ...)   sscanf_s(str_src, format, __VA_ARGS__)

#else /* all others */

#define ar_sprintf(str_dest, str_dest_size, format, ...)  snprintf(str_dest, str_dest_size, format, __VA_ARGS__)
#define ar_sscanf(str_src, format, ...)   sscanf(str_src, format, __VA_ARGS__)

#endif


#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* #ifndef AR_OSAL_STRING_H */

