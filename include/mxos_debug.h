/**
 ******************************************************************************
 * @file    Debug.h
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This header contains defines, macros, and functions to aid in
 *          debugging the MXOS project.
 ******************************************************************************
 *
 *  The MIT License
 *  Copyright (c) 2014 MXCHIP Inc.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is furnished
 *  to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 *  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ******************************************************************************
 */

#ifndef __MXOS_Debug_h__
#define __MXOS_Debug_h__


#ifdef __cplusplus
extern "C" {
#endif

// ==== LOGGING ====
#ifdef __GNUC__
#define SHORT_FILE __FILENAME__
#else
#define SHORT_FILE strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__
#endif

#define YesOrNo(x) (x ? "YES" : "NO")

#ifdef _MXOS_DEBUG_
   extern int mxos_debug_enabled;
   extern mos_mutex_id_t stdio_tx_mutex;

    #define custom_log(N, M, ...) do {if (mxos_debug_enabled==0)break;\
                                      mos_mutex_lock(stdio_tx_mutex );\
                                      printf("[%ld][%s: %s:%4d] " M "\r\n", mos_time(), N, SHORT_FILE, __LINE__, ##__VA_ARGS__);\
                                      mos_mutex_unlock(stdio_tx_mutex );}while(0==1)

    #define app_log(M, ...)      do {if (mxos_debug_enabled==0)break;\
                                      mos_mutex_lock(stdio_tx_mutex );\
                                      printf("[%ld][%s:%4d] " M "\r\n", mos_time(), SHORT_FILE, __LINE__, ##__VA_ARGS__);\
                                      mos_mutex_unlock(stdio_tx_mutex );}while(0==1)
                                      
    #define custom_print(M, ...) do {if (mxos_debug_enabled==0)break;\
                                  mos_mutex_lock(stdio_tx_mutex );\
                                  printf( M, ##__VA_ARGS__);\
                                  mos_mutex_unlock(stdio_tx_mutex );}while(0==1)
                    
    #ifndef MXOS_ASSERT_INFO_DISABLE
        #define debug_print_assert(A,B,C,D,E,F) do {if (mxos_debug_enabled==0)break;\
                                                     mos_mutex_lock(stdio_tx_mutex );\
                                                     printf("[%ld][MXOS:%s:%s:%4d] **ASSERT** %s""\r\n", mos_time(), D, F, E, (C!=NULL) ? C : "" );\
                                                     mos_mutex_unlock(stdio_tx_mutex );}while(0==1)
    #else  // !MXOS_ASSERT_INFO_ENABLE
        #define debug_print_assert(A,B,C,D,E,F)
    #endif  // MXOS_ASSERT_INFO_ENABLE

    #ifdef TRACE
        #define custom_log_trace(N) do {if (mxos_debug_enabled==0)break;\
                                        mos_mutex_lock(stdio_tx_mutex );\
                                        printf("[%s: [TRACE] %s] %s()\r\n", N, SHORT_FILE, __PRETTY_FUNCTION__);\
                                        mos_mutex_unlock(stdio_tx_mutex );}while(0==1)
    #else  // !TRACE
        #define custom_log_trace(N)
    #endif // TRACE  
                                
#else // DEBUG = 0
    // IF !DEBUG, make the logs NO-OP
    #define custom_log(N, M, ...)
    #define custom_print(M, ...)
    #define custom_log_trace(N)
    #define app_log(M, ...) 
    #define debug_print_assert(A,B,C,D,E,F)
#endif // DEBUG


/** Debug level: ALL messages*/
#define MXOS_DEBUG_LEVEL_ALL     0x00
/** Debug level: Warnings. bad checksums, dropped packets, ... */
#define MXOS_DEBUG_LEVEL_WARNING 0x01
/** Debug level: Serious. memory allocation failures, ... */
#define MXOS_DEBUG_LEVEL_SERIOUS 0x02
/** Debug level: Severe */
#define MXOS_DEBUG_LEVEL_SEVERE  0x03

#define MXOS_DEBUG_MASK_LEVEL    0x03
/* compatibility define only */
#define MXOS_DEBUG_LEVEL_OFF     MXOS_DEBUG_LEVEL_ALL

/** flag for LWIP_DEBUGF to enable that debug message */
#define MXOS_DEBUG_ON            0x80U
/** flag for LWIP_DEBUGF to disable that debug message */
#define MXOS_DEBUG_OFF           0x00U

#ifdef _MXOS_DEBUG_
#define MXOS_LOG(D, T, M, ...) do { \
                                   if ( ((D) & MXOS_DEBUG_ON) && \
                                        ((D) & MXOS_DEBUG_TYPES_ON) && \
                                        ((int16_t)((D) & MXOS_DEBUG_MASK_LEVEL) >= MXOS_DEBUG_MIN_LEVEL)) { \
                                        custom_log(T, M, ##__VA_ARGS__); \
                                   } \
                               } while(0)
#define MXOS_PRINT(D, M, ...) do { \
                                   if ( ((D) & MXOS_DEBUG_ON) && \
                                        ((D) & MXOS_DEBUG_TYPES_ON) && \
                                        ((int16_t)((D) & MXOS_DEBUG_MASK_LEVEL) >= MXOS_DEBUG_MIN_LEVEL)) { \
                                        custom_print(M, ##__VA_ARGS__); \
                                   } \
                               } while(0)
#define MXOS_LOG_TRACE(T) custom_log_trace(T)
#else
#define MXOS_LOG(D, T, M, ...)
#define MXOS_LOG_TRACE(T)
#endif

// ==== BRANCH PREDICTION & EXPRESSION EVALUATION ====
#if( !defined( unlikely ) )
    //#define unlikely( EXPRESSSION )     __builtin_expect( !!(EXPRESSSION), 0 )
   #define unlikely( EXPRESSSION )     !!(EXPRESSSION)
#endif

//---------------------------------------------------------------------------------------------------------------------------
/*! @defined    check
    @abstract   Check that an expression is true (non-zero).
    @discussion
    
    If expression evalulates to false, this prints debugging information (actual expression string, file, line number, 
    function name, etc.) using the default debugging output method.
    
    Code inside check() statements is not compiled into production builds.
*/

#if( !defined( check ) )
    #define check( X )                                                                                  \
        do                                                                                              \
        {                                                                                               \
            if( unlikely( !(X) ) )                                                                      \
            {                                                                                           \
                debug_print_assert( 0, #X, NULL, SHORT_FILE, __LINE__, __PRETTY_FUNCTION__ );             \
            }                                                                                           \
                                                                                                        \
        }   while( 1==0 )
#endif
              
//---------------------------------------------------------------------------------------------------------------------------
/*! @defined    check_string
    @abstract   Check that an expression is true (non-zero) with an explanation.
    @discussion
    
    If expression evalulates to false, this prints debugging information (actual expression string, file, line number, 
    function name, etc.) using the default debugging output method.
    
    Code inside check() statements is not compiled into production builds.
*/

#if( !defined( check_string ) )
    #define check_string( X, STR )                                                                                  \
        do                                                                                              \
        {                                                                                               \
            if( unlikely( !(X) ) )                                                                      \
            {                                                                                           \
                debug_print_assert( 0, #X, STR, SHORT_FILE, __LINE__, __PRETTY_FUNCTION__ );              \
                MXOS_ASSERTION_FAIL_ACTION();                                                           \
            }                                                                                           \
                                                                                                        \
        }   while( 1==0 )
#endif              

//---------------------------------------------------------------------------------------------------------------------------
/*! @defined    require
    @abstract   Requires that an expression evaluate to true.
    @discussion
    
    If expression evalulates to false, this prints debugging information (actual expression string, file, line number, 
    function name, etc.) using the default debugging output method then jumps to a label.
*/

#if( !defined( require ) )
    #define require( X, LABEL )                                                                             \
        do                                                                                                  \
        {                                                                                                   \
            if( unlikely( !(X) ) )                                                                          \
            {                                                                                               \
                debug_print_assert( 0, #X, NULL, SHORT_FILE, __LINE__, __PRETTY_FUNCTION__ );                 \
                goto LABEL;                                                                                 \
            }                                                                                               \
                                                                                                            \
        }   while( 1==0 )
#endif

//---------------------------------------------------------------------------------------------------------------------------
/*! @defined    require_string
    @abstract   Requires that an expression evaluate to true with an explanation.
    @discussion
    
    If expression evalulates to false, this prints debugging information (actual expression string, file, line number, 
    function name, etc.) and a custom explanation string using the default debugging output method then jumps to a label.
*/

#if( !defined( require_string ) )
    #define require_string( X, LABEL, STR )                                                                 \
        do                                                                                                  \
        {                                                                                                   \
            if( unlikely( !(X) ) )                                                                          \
            {                                                                                               \
                debug_print_assert( 0, #X, STR, SHORT_FILE, __LINE__, __PRETTY_FUNCTION__ );                  \
                goto LABEL;                                                                                 \
            }                                                                                               \
                                                                                                            \
        }   while( 1==0 )
#endif

//---------------------------------------------------------------------------------------------------------------------------
/*! @defined    require_quiet
    @abstract   Requires that an expression evaluate to true.
    @discussion
    
    If expression evalulates to false, this jumps to a label. No debugging information is printed.
*/

#if( !defined( require_quiet ) )
    #define require_quiet( X, LABEL )                                                                       \
        do                                                                                                  \
        {                                                                                                   \
            if( unlikely( !(X) ) )                                                                          \
            {                                                                                               \
                goto LABEL;                                                                                 \
            }                                                                                               \
                                                                                                            \
        }   while( 1==0 )
#endif

//---------------------------------------------------------------------------------------------------------------------------
/*! @defined    require_noerr
    @abstract   Require that an error code is noErr (0).
    @discussion
    
    If the error code is non-0, this prints debugging information (actual expression string, file, line number, 
    function name, etc.) using the default debugging output method then jumps to a label.
*/

#if( !defined( require_noerr ) )
    #define require_noerr( ERR, LABEL )                                                                     \
        do                                                                                                  \
        {                                                                                                   \
            merr_t        localErr;                                                                       \
                                                                                                            \
            localErr = (merr_t)(ERR);                                                                     \
            if( unlikely( localErr != 0 ) )                                                                 \
            {                                                                                               \
                debug_print_assert( localErr, NULL, NULL, SHORT_FILE, __LINE__, __PRETTY_FUNCTION__ );        \
                goto LABEL;                                                                                 \
            }                                                                                               \
                                                                                                            \
        }   while( 1==0 )
#endif

//---------------------------------------------------------------------------------------------------------------------------
/*! @defined    require_noerr_string
    @abstract   Require that an error code is noErr (0).
    @discussion
    
    If the error code is non-0, this prints debugging information (actual expression string, file, line number, 
    function name, etc.), and a custom explanation string using the default debugging output method using the 
    default debugging output method then jumps to a label.
*/

#if( !defined( require_noerr_string ) )
    #define require_noerr_string( ERR, LABEL, STR )                                                         \
        do                                                                                                  \
        {                                                                                                   \
            merr_t        localErr;                                                                       \
                                                                                                            \
            localErr = (merr_t)(ERR);                                                                     \
            if( unlikely( localErr != 0 ) )                                                                 \
            {                                                                                               \
                debug_print_assert( localErr, NULL, STR, SHORT_FILE, __LINE__, __PRETTY_FUNCTION__ );         \
                goto LABEL;                                                                                 \
            }                                                                                               \
                                                                                                            \
        }   while( 1==0 )
#endif

//---------------------------------------------------------------------------------------------------------------------------
/*! @defined    require_noerr_action_string
    @abstract   Require that an error code is noErr (0).
    @discussion
    
    If the error code is non-0, this prints debugging information (actual expression string, file, line number, 
    function name, etc.), and a custom explanation string using the default debugging output method using the 
    default debugging output method then executes an action and jumps to a label.
*/

#if( !defined( require_noerr_action_string ) )
    #define require_noerr_action_string( ERR, LABEL, ACTION, STR )                                          \
        do                                                                                                  \
        {                                                                                                   \
            merr_t        localErr;                                                                       \
                                                                                                            \
            localErr = (merr_t)(ERR);                                                                     \
            if( unlikely( localErr != 0 ) )                                                                 \
            {                                                                                               \
                debug_print_assert( localErr, NULL, STR, SHORT_FILE, __LINE__, __PRETTY_FUNCTION__ );         \
                { ACTION; }                                                                                 \
                goto LABEL;                                                                                 \
            }                                                                                               \
                                                                                                            \
        }   while( 1==0 )
#endif

//---------------------------------------------------------------------------------------------------------------------------
/*! @defined    require_noerr_quiet
    @abstract   Require that an error code is noErr (0).
    @discussion
    
    If the error code is non-0, this jumps to a label. No debugging information is printed.
*/

#if( !defined( require_noerr_quiet ) )
    #define require_noerr_quiet( ERR, LABEL )                                                               \
        do                                                                                                  \
        {                                                                                                   \
            if( unlikely( (ERR) != 0 ) )                                                                    \
            {                                                                                               \
                goto LABEL;                                                                                 \
            }                                                                                               \
                                                                                                            \
        }   while( 1==0 )
#endif

//---------------------------------------------------------------------------------------------------------------------------
/*! @defined    require_noerr_action
    @abstract   Require that an error code is noErr (0) with an action to execute otherwise.
    @discussion
    
    If the error code is non-0, this prints debugging information (actual expression string, file, line number, 
    function name, etc.) using the default debugging output method then executes an action and jumps to a label.
*/

#if( !defined( require_noerr_action ) )
    #define require_noerr_action( ERR, LABEL, ACTION )                                                      \
        do                                                                                                  \
        {                                                                                                   \
            merr_t        localErr;                                                                       \
                                                                                                            \
            localErr = (merr_t)(ERR);                                                                     \
            if( unlikely( localErr != 0 ) )                                                                 \
            {                                                                                               \
                debug_print_assert( localErr, NULL, NULL, SHORT_FILE, __LINE__, __PRETTY_FUNCTION__ );        \
                { ACTION; }                                                                                 \
                goto LABEL;                                                                                 \
            }                                                                                               \
                                                                                                            \
        }   while( 1==0 )
#endif

//---------------------------------------------------------------------------------------------------------------------------
/*! @defined    require_noerr_action_quiet
    @abstract   Require that an error code is noErr (0) with an action to execute otherwise.
    @discussion
    
    If the error code is non-0, this executes an action and jumps to a label. No debugging information is printed.
*/

#if( !defined( require_noerr_action_quiet ) )
    #define require_noerr_action_quiet( ERR, LABEL, ACTION )                                                \
        do                                                                                                  \
        {                                                                                                   \
            if( unlikely( (ERR) != 0 ) )                                                                    \
            {                                                                                               \
                { ACTION; }                                                                                 \
                goto LABEL;                                                                                 \
            }                                                                                               \
                                                                                                            \
        }   while( 1==0 )
#endif

//---------------------------------------------------------------------------------------------------------------------------
/*! @defined    require_action
    @abstract   Requires that an expression evaluate to true with an action to execute otherwise.
    @discussion
    
    If expression evalulates to false, this prints debugging information (actual expression string, file, line number, 
    function name, etc.) using the default debugging output method then executes an action and jumps to a label.
*/

#if( !defined( require_action ) )
    #define require_action( X, LABEL, ACTION )                                                              \
        do                                                                                                  \
        {                                                                                                   \
            if( unlikely( !(X) ) )                                                                          \
            {                                                                                               \
                debug_print_assert( 0, #X, NULL, SHORT_FILE, __LINE__, __PRETTY_FUNCTION__ );                 \
                { ACTION; }                                                                                 \
                goto LABEL;                                                                                 \
            }                                                                                               \
                                                                                                            \
        }   while( 1==0 )
#endif

//---------------------------------------------------------------------------------------------------------------------------
/*! @defined    require_action_string
    @abstract   Requires that an expression evaluate to true with an explanation and action to execute otherwise.
    @discussion
    
    If expression evalulates to false, this prints debugging information (actual expression string, file, line number, 
    function name, etc.) and a custom explanation string using the default debugging output method then executes an
    action and jumps to a label.
*/

#if( !defined( require_action_string ) )
    #define require_action_string( X, LABEL, ACTION, STR )                                                  \
        do                                                                                                  \
        {                                                                                                   \
            if( unlikely( !(X) ) )                                                                          \
            {                                                                                               \
                debug_print_assert( 0, #X, STR, SHORT_FILE, __LINE__, __PRETTY_FUNCTION__ );                  \
                { ACTION; }                                                                                 \
                goto LABEL;                                                                                 \
            }                                                                                               \
                                                                                                            \
        }   while( 1==0 )
#endif

//---------------------------------------------------------------------------------------------------------------------------
/*! @defined    require_action_quiet
    @abstract   Requires that an expression evaluate to true with an action to execute otherwise.
    @discussion
    
    If expression evalulates to false, this executes an action and jumps to a label. No debugging information is printed.
*/

#if( !defined( require_action_quiet ) )
    #define require_action_quiet( X, LABEL, ACTION )                                                        \
        do                                                                                                  \
        {                                                                                                   \
            if( unlikely( !(X) ) )                                                                          \
            {                                                                                               \
                { ACTION; }                                                                                 \
                goto LABEL;                                                                                 \
            }                                                                                               \
                                                                                                            \
        }   while( 1==0 )
#endif


#endif // __Debug_h__

