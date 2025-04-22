/**
 * \file posal_err_fatal.c
 * \brief
 *    Contains API for force crash.
 *
 *
 * \copyright
 *    Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *    SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 * Copyright (c) 1998 Todd C. Miller <Todd.Miller@courtesan.com>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 * 
*/

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */

#include "posal_err_fatal.h"
#include "ar_msg.h"
#include "posal_types.h"
#include <stringl.h>

#ifdef ADSPMODE
#include "adsp_err_fatal.h"
#endif

bool_t is_force_crash_enabled = TRUE;

#if ARSDK_BUILD_ENABLED
//strlcpy definition is available in gpr/platform/win/osal/src/ar_osal_strlcpy.c
//but this file is not compiled for ARSDK.
//This will be cleanedup when ar_osal is unified with the top-level ar_osal
size_t strlcpy(char *dst, const char *src, size_t dsize)
{
	const char *osrc = src;
	size_t nleft = dsize;

	/* Copy as many bytes as will fit. */
	if (nleft != 0) {
		while (--nleft != 0) {
			if ((*dst++ = *src++) == '\0')
				break;
		}
	}

	/* Not enough room in dst, add NUL and traverse rest of src. */
	if (nleft == 0) {
		if (dsize != 0)
			*dst = '\0';		/* NUL-terminate dst */
		while (*src++)
			;
	}

	return(src - osrc - 1);	/* count does not include NUL */
}
#endif //#if ARSDK_BUILD_ENABLED

void posal_err_fatal(const char *err_str)
{
   if (TRUE == is_force_crash_enabled)
   {
      uint16_t err_str_len = strlen(err_str);
      char     err_fatal_str[err_str_len];
      strlcpy(err_fatal_str, err_str, err_str_len);
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "Posal Error fatal called Reason : %s", err_fatal_str);
#if defined(SIM)
      *((volatile uint32_t *)0) = 0;
#else
#if defined(__qdsp6__) && defined(ADSPMODE)
      return (void)AdspfatalerrApi((char *)err_str, err_str_len);
#else
      *((volatile uint32_t *)0) = 0;
#endif
#endif
   }

}
