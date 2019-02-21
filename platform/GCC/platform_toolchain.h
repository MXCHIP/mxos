/**
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 *
 */

#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************
 *                      Macros
 ******************************************************/

#ifndef WEAK
#ifndef __MINGW32__
#define WEAK             __attribute__((weak))
#else
/* MinGW doesn't support weak */
#define WEAK
#endif
#endif

#ifndef USED
#define USED             __attribute__((used))
#endif

#ifndef MAY_BE_UNUSED
#define MAY_BE_UNUSED    __attribute__((unused))
#endif

#ifndef NORETURN
#define NORETURN         __attribute__((noreturn))
#endif

//#ifndef ALIGNED
//#define ALIGNED(size)    __attribute__((aligned(size)))
//#endif

#ifndef SECTION
#define SECTION(name)    __attribute__((section(name)))
#endif

#ifndef NEVER_INLINE
#define NEVER_INLINE     __attribute__((noinline))
#endif

#ifndef ALWAYS_INLINE
#define ALWAYS_INLINE    __attribute__((always_inline))
#endif

#ifndef MXOS_PACKED
#define MXOS_PACKED(struct) struct __attribute__((packed))
#endif

#ifndef MXOS_ALIGN
#define MXOS_ALIGN(size) __attribute__((aligned(size)))
#endif

#ifndef MXOS_UNUSED
#define MXOS_UNUSED      __attribute__((__unused__))
#endif

#ifndef MXOS_WEAK
#define MXOS_WEAK        __attribute__((weak))
#endif

#ifndef MXOS_FORCEINLINE
#define MXOS_FORCEINLINE static inline __attribute__((always_inline))
#endif

#ifndef MXOS_NORETURN
#define MXOS_NORETURN   __attribute__((noreturn))
#endif

#ifndef MXOS_UNREACHABLE
#define MXOS_UNREACHABLE __builtin_unreachable()
#endif

#ifndef MXOS_CALLER_ADDR
#define MXOS_CALLER_ADDR() __builtin_extract_return_addr(__builtin_return_address(0))
#endif

#ifndef MXOS_DEPRECATED
#define MXOS_DEPRECATED(M) __attribute__((deprecated(M)))
#endif

#define MXOS_DEPRECATED_SINCE(D, M) MXOS_DEPRECATED(M " [since " D "]")

#ifndef MXOS_SECTION
#define MXOS_SECTION(name) __attribute__ ((section (name)))
#endif


// Backwards compatibility
#ifndef WEAK
#define WEAK MXOS_WEAK
#endif

#ifndef PACKED
#define PACKED MXOS_PACKED()
#endif

#ifndef EXTERN
#define EXTERN extern
#endif


/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

void *memrchr( const void *s, int c, size_t n );


/* Windows doesn't come with support for strlcpy */
#ifdef WIN32
size_t strlcpy (char *dest, const char *src, size_t size);
#endif /* WIN32 */

#ifdef __cplusplus
} /* extern "C" */
#endif

