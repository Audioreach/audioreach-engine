#ifndef AR_OSAL_TYPES_H
#define AR_OSAL_TYPES_H

/**
 * \file ar_osal_types.h
 * \brief
 *        This file contains basic types and pre processor macros.
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#if defined(__H2XML__)

typedef signed char          int8_t;
typedef unsigned char        uint8_t;
typedef short                int16_t;
typedef unsigned short       uint16_t;
typedef int                  int32_t;
typedef unsigned int         uint32_t;
typedef signed long long     int64_t;
typedef unsigned long long   uint64_t;

#else /** else of defined(__H2XML__) */

/* -----------------------------------------------------------------------
** COMMON to all  ---> START
** ----------------------------------------------------------------------- */
    /* -----------------------------------------------------------------------
    ** Standard Types
    ** ----------------------------------------------------------------------- */

    #include <stdio.h>
    #include <stdint.h>
    #include <string.h>

#ifdef __INTEGRITY
    #include "mmdefs.h"
#endif
    /* ------------------------------------------------------------------------
    ** Character and boolean
    ** ------------------------------------------------------------------------ */
#ifndef __INTEGRITY /* defined only on greenhills compiler */
        typedef char char_t;           /**< Character type */
        typedef unsigned char bool_t;  /**< Boolean value type. */
#endif

    #ifndef FALSE
        #define FALSE               0
    #endif

    #ifndef TRUE
        #define TRUE                1
    #endif
/* -----------------------------------------------------------------------
** COMMON to all  ---> END
** ----------------------------------------------------------------------- */

/* -----------------------------------------------------------------------
** COMMON to all except windows kernel  ---> START
** ----------------------------------------------------------------------- */
#if !defined(_KERNEL_MODE)

    /** PAGING functiona and data*/

    /** By default, the linker assigns names such as ".text" and ".data" to the code and data sections
      *of a driver image file. When the driver is loaded, the I/O manager makes these sections nonpaged.
      *A nonpaged section is always memory-resident.
      *A driver developer has the option to make designated parts of a driver pageable so that Windows
      *can move these parts to the paging file when they are not in use. To make a code or data section
      *pageable, the driver developer assigns a name that begins with "PAGE" to the section. The I/O manager
      *checks the names of the sections when it loads a driver. If a section name begins with "PAGE",
      *the I/O manager makes the section pageable.
      *Code that runs at IRQL >= DISPATCH_LEVEL must be memory-resident. That is, this code must be either
      *in a nonpageable segment, or in a pageable segment that is locked in memory. If code that is running
      *at IRQL >= DISPATCH_LEVEL causes a page fault, a bug check occurs. Drivers can use the PAGED_CODE macro
      *to verify that pageable functions are called only at appropriate IRQLs.
      */

      /** Paged functions*/

      /** Should be used at the start of functions group to be paged */
      #define PAGED_FUNCTIONS_START

      /** Should be used at the end of functions group to be paged */
      #define PAGED_FUNCTIONS_END

      /** Should be used at the start of the paged function definition */
      #define PAGED_FUNCTION()

      /** Paged Data*/

      /** Should be used at the start of the vairables group to be paged */
      #define PAGED_DATA_START

      /** Should be used at the end of the variables group to be paged */
      #define PAGED_DATA_END

/* -----------------------------------------------------------------------
** COMMON to all except windows kernel  ---> END
** ----------------------------------------------------------------------- */
#endif

/* -----------------------------------------------------------------------
** Windows kernel and user mode  ---> START
** ----------------------------------------------------------------------- */
#if defined(_WIN64) || defined(_WIN32)

    #include <sal.h>

/* -----------------------------------------------------------------------
** Windows kernel mode only ---> START
** ----------------------------------------------------------------------- */
    #if defined(_KERNEL_MODE)

        #include <stdlib.h>
        #include <basetsd.h>
        #include <ntdef.h>
        #include <kernelspecs.h>
        #include <wdm.h>

        #define snprintf(X,Y,...) _snprintf_s(X, Y, _TRUNCATE, __VA_ARGS__)



        #ifndef false
            #define false               0
        #endif

        #ifndef true
            #define true                1
        #endif


        /** Paged functions*/

        /** Should be used at the start of functions group to be paged */
        #define PAGED_FUNCTIONS_START      __pragma (code_seg("PAGEAR"))

        /** Should be used at the end of functions group to be paged */
        #define PAGED_FUNCTIONS_END        __pragma (code_seg())

        /** Should be used at the start of the paged function definition */
        #define PAGED_FUNCTION()             PAGED_CODE();

        /** Paged Data*/

        /** Should be used at the start of the vairables group to be paged */
        #define PAGED_DATA_START           __pragma (data_seg("PAGEAR"))

        /** Should be used at the end of the variables group to be paged */
        #define PAGED_DATA_END            __pragma (data_seg())

/* -----------------------------------------------------------------------
** Windows kernel mode only ---> END
** ----------------------------------------------------------------------- */
    #else

/* -----------------------------------------------------------------------
** Windows user mode only ---> START
** ----------------------------------------------------------------------- */
        #include <driverspecs.h>
        #include <stdbool.h>
/* -----------------------------------------------------------------------
** Windows user mode only ---> END
** ----------------------------------------------------------------------- */
    #endif


        #define __UNREFERENCED_PARAM(x)  (x) /*UNREFERENCED_PARAMETER is not used here, getting conflict including windows.h with INFINITE redefined in DSP(media_fmt_api.h)*/
        #define __UNUSED

/* -----------------------------------------------------------------------
** Windows kernel and user mode  ---> END
** ----------------------------------------------------------------------- */
#else

/* -----------------------------------------------------------------------
** Rest of the Platforms   ---> START
** ----------------------------------------------------------------------- */

    #include <stdbool.h>
    #define __UNREFERENCED_PARAM(x)

    #if defined (__GNUC__)
        #define __UNUSED  __attribute__((unused))
    #else
        #define __UNUSED
    #endif

    /** Function parameter annotations
      */
    /** This parameter must have a valid value and will not be changed by the function
      */
    #define _In_

    /** This parameter may not have valid value and will not be changed by the function.
      */
    #define _In_opt_

    /** This parameter does not have a valid value, but must be a valid value after the
      * function call returns.
      */
    #define _Out_

    /** This parameter does not have a valid value before and after the function call returns.
      */
    #define _Out_opt_

    /** This parameter must have a valid value before and should have a different value after
      * the function call returns.
      */
    #define _Inout_

    /** This parameter may not have a valid value before and after the function call returns.
      */
    #define _Inout_opt_

    /** This Parameter cannot be null and should have a valid pointer to location after the
      * function call returns.
      */
    #define _Outptr_

     /** This Parameter can be null and should have a valid pointer to location after the
       * function call returns.
       */
    #define _Outptr_opt_

    /** Interrupt request level(IRQL)  */
    /** Driver code or system APIs can only run on certain IRQLs. Developer must ensure IRQL requirements are met.
      *Code with in spin lock usage must not be paged.
      *Code with in spin lock must be minimum possible.
      *Interrupts/callbacks function running at DISPATCH IRQL (high priorities) should not be paged.
      *Identify functions which could run at PASSIVE IRQL( low priority).
      */
    #define _IRQL_requires_max_(irql)

    #define _IRQL_requires_min_(irql)

    /** Normal priority level */
    #define PASSIVE_LEVEL

    /** Runs at high priority level, these functions should not be paged.*/
    #define DISPATCH_LEVEL

/* -----------------------------------------------------------------------
** Rest of the Platforms   ---> END
** ----------------------------------------------------------------------- */
#endif

#endif /* __H2XML__ */
#endif /* #ifndef AR_OSAL_TYPES_H */
