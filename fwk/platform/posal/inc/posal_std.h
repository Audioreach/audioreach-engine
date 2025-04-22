#ifndef POSAL_STD_H
#define POSAL_STD_H
/**
 * \file posal_std.h
 * \brief 
 *  	 This file contains standard C functions
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "posal_types.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus


#define AR_STD_MIN( a, b ) ( ( a ) < ( b ) ? ( a ) : ( b ) )
#define AR_STD_MAX( a, b ) ( ( a ) < ( b ) ? ( b ) : ( a ) )

/** @ingroup posal_std
  Copies the string from source pointer to a destination pointer.

  @param[in] dest_ptr  Pointer to the destination string.
  @param[in] src_ptr   Pointer to the source string.
  @param[in] dest_len  Length of the destination buffer.

  @return
  src_len -- Source string size.

  @dependencies
  None.
*/
uint32_t posal_strlcpy(char_t *dest_ptr, const char_t *src_ptr, uint32_t dest_len);

/** @ingroup posal_std
  Determines the length of a string with a fixed maximum size.

  @param[in] src_ptr  Pointer to the destination string.
  @param[in] size   Pointer to the source string.

  @return
  string length.

  @dependencies
  None.
*/
uint32_t posal_strnlen (const char_t* src_ptr, uint32_t size);

/** @ingroup posal_std
  Compares two strings, character wise bounded by length of the strings.

  @param[in] s1  Pointer to the destination string.
  @param[in] s1_size   Pointer to the source string.
  @param[in] s2  Length of the destination buffer.
  @param[in] s2_size  Pointer to the destination string.

  @return
  <0 - if the first character that doesnt match has lower ASCII value in s1 than in s2
   0 - if two strings are same
  >0 - if the first character that doesnt match has greater ASCII value in s1 than in s2

  @dependencies
  None.
*/
int32_t posal_strncmp (const char* s1, uint32_t s1_size, const char* s2, uint32_t s2_size);

/** @ingroup posal_std
  Copies src_size bytes from source pointer to the destination pointer. The number of bytes
  actually copied is bounded by dst_size, this avoids destination memory corruption if dst_size
  is less than src_size.

  @param[in] dst      - destination pointer.
  @param[in] dst_size - size of destination pointer in bytes.
  @param[in] src      - source pointer
  @param[in] src_size - number of bytes to be copied from the source pointer.

  @return
  returns copy of destination pointer.

  @dependencies
  None.
*/
void* posal_memcpy (void* dst, uint32_t dst_size, const void* src, uint32_t src_size);

/** @ingroup posal_std
  Sets the first size bytes of the memory pointed by dst to the value 'c' [interpreted
  as unsigned char].

  @param[in] dst        - destination pointer
  @param[in] c          - Value to be set. Pass as int32 but interpreted as unsigned char.
  @param[in] num_bytes  - number of bytes to be set to value 'c'

  @return
  returns a copy of the input pointer.

  @dependencies
  None.
*/
void* posal_memset (void* dst, int32_t c, uint32_t num_bytes);

/** @ingroup posal_std
  Prints formated string in the destination pointer, the maximum number of characters
  printed is bounded by the size.

  @param[in] dst    - destination pointer where the string is printed.
  @param[in] size   - maximum number of characters that could be printed.
  @param[in] format - pointer to the format string.

  @return
  Number of character that have been actually printed.

  @dependencies
  None.
*/
int32_t posal_snprintf (char_t* dst, uint32_t size,const char_t* format, ...);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif /* #ifndef POSAL_STD_H */
