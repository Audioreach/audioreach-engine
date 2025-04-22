#ifndef _POSAL_TYPES_H
#define _POSAL_TYPES_H
/**
 * \file posal_types.h
 * \brief
 *  	 This file contains basic types and pre processor macros.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#if /* Supported Compilers */                                                                                          \
   defined(__ARMCC_VERSION) ||                                                                                         \
   defined(__GNUC__)

/* If we're hosted, fall back to the system's stdint.h, which might have
 * dditional definitions.
 */
#include "stdint.h"

#elif defined(_WIN64)

#include <stdint.h>
#include <sal.h>

#else /* Unsupported Compilers */

/* The following definitions are the same accross platforms.  This first
 * group are the sanctioned types.
 */
typedef unsigned long long uint64_t; /* Unsigned 64 bit value */
typedef unsigned long int  uint32_t; /* Unsigned 32 bit value */
typedef unsigned short     uint16_t; /* Unsigned 16 bit value */
typedef unsigned char      uint8_t;  /* Unsigned 8  bit value */

typedef signed long long int64_t; /* Signed 64 bit value */
typedef signed long int  int32_t; /* Signed 32 bit value */
typedef signed short     int16_t; /* Signed 16 bit value */
typedef signed char      int8_t;  /* Signed 8  bit value */

/* ------------------------------------------------------------------------
 ** Constants
 ** ------------------------------------------------------------------------ */
#undef TRUE
#undef FALSE

#define TRUE (1)  /* Boolean true value. */
#define FALSE (0) /* Boolean false value. */

#ifndef NULL
#define NULL (0)
#endif

#endif /* __ARMCC_VERSION || __GNUC__ */

/* ------------------------------------------------------------------------
 ** Character and boolean
 ** ------------------------------------------------------------------------ */
typedef char          char_t;    /* Character type */
typedef unsigned char bool_t;    /* Boolean value type. */
typedef float         float32_t; /*floating point 32 bit value*/
typedef double        float64_t; /*floating point 64 bit value*/

#if defined(COMDEF_H) /* guards against a known re-definer */
#define _BOOLEAN_DEFINED
#define _UINT32_DEFINED
#define _UINT16_DEFINED
#define _UINT8_DEFINED
#define _INT32_DEFINED
#define _INT16_DEFINED
#define _INT8_DEFINED
#define _UINT64_DEFINED
#define _INT64_DEFINED
#define _BYTE_DEFINED
#endif /* #if !defined(COMDEF_H) */

/* -----------------------------------------------------------------------
 ** Standard Types
 ** ----------------------------------------------------------------------- */

/* The following definitions are the same accross platforms.  This first
 ** group are the sanctioned types.
 */
#ifndef boolean
#ifndef _BOOLEAN_DEFINED
typedef bool_t boolean;  /* Boolean value type. */
#define _BOOLEAN_DEFINED /* Boolean value type flag. */
#endif
#endif

#ifndef uint32
#ifndef _UINT32_DEFINED
typedef uint32_t uint32; /* Unsigned 32-bit value. */
#define _UINT32_DEFINED  /* Unsigned 32-bit value flag. */
#endif
#endif

#ifndef uint16
#ifndef _UINT16_DEFINED
typedef uint16_t uint16; /* Unsigned 16-bit value. */
#define _UINT16_DEFINED  /* Unsigned 16-bit value flag. */
#endif
#endif

#ifndef uint8
#ifndef _UINT8_DEFINED
typedef uint8_t uint8; /* Unsigned 8-bit value. */
#define _UINT8_DEFINED /* Unsigned 8-bit value flag. */
#endif
#endif

#ifndef int32
#ifndef _INT32_DEFINED
typedef int32_t int32; /* Signed 32-bit value */
#define _INT32_DEFINED /* Signed 32-bit value flag. */
#endif
#endif

#ifndef int16
#ifndef _INT16_DEFINED
typedef int16_t int16; /* Signed 16-bit value. */
#define _INT16_DEFINED /* Signed 16-bit value flag. */
#endif
#endif

#ifndef int8
#ifndef _INT8_DEFINED
typedef int8_t int8;  /* Signed 8-bit value. */
#define _INT8_DEFINED /* Signed 8-bit value flag. */
#endif
#endif

#ifndef uint64
#ifndef _UINT64_DEFINED
typedef uint64_t uint64; /* Unsigned 64-bit value. */
#define _UINT64_DEFINED  /* Unsigned 64-bit value flag. */
#endif
#endif

#ifndef int64
#ifndef _INT64_DEFINED
typedef int64_t int64; /* Signed 64-bit value. */
#define _INT64_DEFINED /* Signed 64-bit value flag. */
#endif
#endif

#ifndef byte
#ifndef _BYTE_DEFINED
typedef uint8_t byte; /* Byte type. */
#define _BYTE_DEFINED /* Byte type flag. */
#endif
#endif

/* -----------------------------------------------------------------------
 ** Function Calling Conventions
 ** ----------------------------------------------------------------------- */
#ifndef CDECL
#ifdef _MSC_VER
#define CDECL __cdecl
#else
#define CDECL
#endif /* _MSC_VER */
#endif /* CDECL */

/* -----------------------------------------------------------------------
 ** Preprocessor helpers
 ** ----------------------------------------------------------------------- */
#define __STR__(x) #x
/**< Macro wrapper for the preprocessing operator number. */

#define __TOSTR__(x) __STR__(x)
/**< Macro wrapper for converting a number to a string. */

#define __FILE_LINE__ __FILE__ ":" __TOSTR__(__LINE__)
/**< Macro wrapper for generating a filename and line. */

#if defined(__GNUC__) || defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 220000)
#define POSAL_ALIGN(t, a) t __attribute__((aligned(a)))
/**< Macro for compiler pragma align. */

#elif defined(_MSC_VER) || defined(_WIN64)
#define POSAL_ALIGN(t, a) __declspec(align(a)) t
/**< Macro for compiler pragma align. */

#elif defined(__ARMCC_VERSION)
#define POSAL_ALIGN(t, a) __align(a) t
/**< Macro for compiler pragma align. */

#else
#error Unknown compiler
#endif

#define POSAL_ARRAY_SIZE(a) ((size_t)((sizeof((a)) / sizeof((a)[0]))))
/**< Macro that gets the array size. */

/** Macros */
#ifndef MIN
#define MIN(m, n) (((m) < (n)) ? (m) : (n))
#endif

#ifndef MAX
#define MAX(m, n) (((m) > (n)) ? (m) : (n))
#endif

#define POSAL_MIN(a, b) ((a) < (b) ? (a) : (b))
/**< Macro that gets the minimum of two numbers. */

#define POSAL_MAX(a, b) ((a) > (b) ? (a) : (b))
/**< Macro that gets the maximum of two numbers. */

#define POSAL_ZEROAT(p) memset((p), 0, sizeof(*p))
/**< Macro that clears the buffer to all zeros. */ // lint -emacro(545,STD_ZEROAT)

/** Define POSAL_ASSERT macro */
#if defined(DEBUG) || defined(SIM)
#include <assert.h>
/* For sim */
#define POSAL_ASSERT(x) assert(x)

#else /* TARGET */
/*
If the definition of POSAL_ASSERT for on target builds is set to blank, the compiler simply removes any code inside the
POSAL_ASSERT macro.
This may cause an unused variable error. We can fix this by doing:

#define POSAL_ASSERT(x) (x)

However, this will trigger another warning from the compiler saying that this statement does not have any effect. This
warning can be suppressed by doing:

#define POSAL_ASSERT(x) (void)(x)

This works, but may cause the compiler to insert code that evaluates x in some cases. We want the compiler to not
generate any code at all.
The C++ standard states that contents of sizeof() are evaluated at compile time and never at run time. So we can use
that property and do the following:

#define POSAL_ASSERT(x) (void)sizeof(x)

This eliminates the compiler warnings and ensures that no code is generated.
*/
#define POSAL_ASSERT(x) (void)sizeof(x)

#endif /* POSAL_ASSERT */

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/* =======================================================================
 Global definitions/forward declarations
========================================================================== */
/* Define this keyword for GNU C. */
#ifndef restrict
#ifdef __GNUC__
#define restrict __restrict__ /**< Macro for a compiler pragma restriction. */
#else
#define restrict /**< Macro for a compiler pragma restriction. */
#endif
#endif

/**
Macro that rounds a number to the nearest multiple of 4 (approaching
infinity).
 */
#define POSAL_ROUNDUP_MULTIPLE4(x) (((uint32_t)x + 3) & -4)

/**
Macro that rounds a number to the nearest multiple of 8 (approaching
infinity).
 */
#define POSAL_ROUNDUP_MULTIPLE8(x) (((uint32_t)x + 7) & -8)

#ifdef __cplusplus
}
#endif //__cplusplus
#endif /* #ifndef POSAL_TYPES_H */
