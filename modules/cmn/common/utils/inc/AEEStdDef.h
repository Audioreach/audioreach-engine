#ifndef AEESTDDEF_H
#define AEESTDDEF_H
/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
FILE:         AEEStdDef.h

DESCRIPTION:  Legacy file kept for compatibility. Redirects to posal_types.h */

#include "posal_types.h"

/* ------------------------------------------------------------------------
** Constants
** ------------------------------------------------------------------------ */
#undef TRUE
#undef FALSE

#define TRUE   (1)  /* Boolean true value. */
#define FALSE  (0)  /* Boolean false value. */

#ifndef NULL
  #define NULL (0)
#endif



#endif /* #ifndef AEESTDDEF_H */