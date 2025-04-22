/**
 * \file ar_osal_string.c
 *
 * \brief
 *      Defines public APIs for string operations.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "ar_osal_types.h"
#include "ar_osal_error.h"
#include <string.h>

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
_IRQL_requires_max_(DISPATCH_LEVEL)
size_t ar_strlen(_In_ const char_t* str, _In_ size_t size)
{
    return strnlen(str, size);
}

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
_IRQL_requires_max_(DISPATCH_LEVEL)
int32_t ar_strcpy(_Inout_ char_t* str_dest, _In_ size_t str_dest_size, _In_ const char_t* str_src, _In_ size_t cpy_size)
{
    size_t final_cpy_size = 0;
    if (!str_dest || !str_src)
        return AR_EFAILED;

    if (str_dest_size == 0 || cpy_size == 0)
        return AR_EFAILED;

    if (cpy_size < str_dest_size)
        final_cpy_size = cpy_size + 1;
    else if (cpy_size >= str_dest_size )
        final_cpy_size = str_dest_size;

    strlcpy(str_dest, str_src, final_cpy_size);

    return AR_EOK;
}

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
_IRQL_requires_max_(DISPATCH_LEVEL)
int32_t ar_strcmp(_In_ const char_t* str1, _In_ const char_t* str2, _In_ size_t num)
{
    return strncmp(str1, str2, num);
}

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
_IRQL_requires_max_(DISPATCH_LEVEL)
int32_t ar_strcat(_Inout_ char_t* str_dest, _In_ size_t str_dest_size, _In_ const char_t* str_src, _In_ size_t apnd_size)
{
    if (str_dest_size <
         ar_strlen(str_dest, str_dest_size) +
         apnd_size + 1)
         return AR_EFAILED;

    strlcat(str_dest, str_src, str_dest_size);
    return AR_EOK;
}

/**
 * \brief ar_strstr
 *        Search string in string.
 * \param[in] str: Null terminated string
 * \param[in] str_search: Null terminated string to search for.
 * \return
 *         Pointer to the first occurrence of str_search in str.
 *         NULL- if not found.
 */
_IRQL_requires_max_(DISPATCH_LEVEL)
char_t* ar_strstr(_In_ const char_t* str, _In_ const char_t* str_search)
{
    return strstr(str, str_search);
}
