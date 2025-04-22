/**
 * \file posal_std.c
 * \brief
 *
 *
 * \copyright
 *    Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *    SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/****************************************************************************
 * INCLUDE HEADER FILES                                                     *
 ****************************************************************************/

#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#if defined( WIN32 )
  #include <windows.h>
#else
  #include <stringl/stringl.h>
#endif /* WIN32 */

#include "posal_types.h"
#include "posal_std.h"

/****************************************************************************
 * ROUTINES                                                                 *
 ****************************************************************************/
/* The src must be size or less. */
uint32_t posal_strnlen (const char_t* src, uint32_t size)
{
  uint32_t count = 0;
  while ( ( count < size ) && ( *src != 0 ) )
  {
    ++src;
    ++count;
  }

  return count;
}

int32_t posal_strncmp (const char* s1, uint32_t s1_size, const char* s2, uint32_t s2_size)
{
  return strncmp( s1, s2, AR_STD_MIN( s1_size, s2_size ) );
}

/* NULL counts towards the size. Locale comparison is not yet supported. */
uint32_t posal_strlcpy (char_t* dst, const char_t* src, uint32_t size)
{
#if defined( WIN32 )
  strncpy_s( dst, size, src, _TRUNCATE );
  return posal_strnlen( dst, size );
#else
  return strlcpy( dst, src, size );
#endif /* WIN32 */
}

void* posal_memcpy (void* dst, uint32_t dst_size, const void* src, uint32_t src_size)
{
#if defined( WIN32 )
  return ( ( memcpy_s( dst, dst_size, src, src_size ) == 0 ) ? dst : NULL );
#else
  return ( ( memscpy( dst, dst_size, src, src_size ) == src_size ) ? dst : NULL );
#endif /* WIN32 */
}

void* posal_memset (void* dst, int32_t c, uint32_t size)
{
  return memset( dst, c, size );
}

int32_t posal_snprintf (char_t* dst, uint32_t size, const char_t* format, ...)
{
  int32_t rc;
  va_list args = {0};
  va_start( args, format );

#if defined( WIN32 )
  rc = vsnprintf_s( dst, size, _TRUNCATE, format, args );
#else
  rc = vsnprintf( dst, size, format, args );
#endif /* WIN32 */

  va_end( args );

  return rc;
}
