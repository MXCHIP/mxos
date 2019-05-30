/**
 ******************************************************************************
 * @file    URLUtils.c
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This file contains function that aid in parsing URLs.
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

#include "mxos_common.h"
#include "mdebug.h"

#include "StringUtils.h"

#include "URLUtils.h"

#define url_utils_log(M, ...) custom_log("URLUtils", M, ##__VA_ARGS__)

//===========================================================================================================================
//  URLParseComponents
//===========================================================================================================================

merr_t    URLParseComponents( const char *inSrc, const char *inEnd, URLComponents *outComponents, const char **outSrc )
{
    const char *        ptr;
    const char *        schemePtr;
    const char *        schemeEnd;
    const char *        userPtr;
    const char *        userEnd;
    const char *        passwordPtr;
    const char *        passwordEnd;
    const char *        hostPtr;
    const char *        hostEnd;
    const char *        pathPtr;
    const char *        pathEnd;
    const char *        queryPtr;
    const char *        queryEnd;
    const char *        fragmentPtr;
    const char *        fragmentEnd;
    char                c;
    
    /*
        URL breakdown from RFC 3986.
    
         foo://example.com:8042/over/there?name=ferret#nose
         \_/   \______________/\_________/ \_________/ \__/
          |           |            |            |        |
        scheme    authority       path        query   fragment
          |   _____________________|__
         / \ /                        \
         urn:example:animal:ferret:nose
    */
    
    if( inEnd == NULL ) inEnd = inSrc + strlen( inSrc );
    
    // Parse an optional scheme (the "ftp" in "ftp://tom:secret@abc.com/test?x#y").
    
    schemePtr = NULL;
    schemeEnd = NULL;
    
    c = '\0';
    ptr = inSrc;
    while( ( ptr < inEnd ) && ( ( c = *ptr ) != ':' ) && ( c != '/' ) && ( c != '?' ) && ( c != '#' ) ) ++ptr;
    if( c == ':' )
    {
        schemePtr = inSrc;
        schemeEnd = ptr;
        inSrc = ptr + 1;
    }
    
    // Parse an optional authority (the "tom:secret@abc.com" in "ftp://tom:secret@abc.com/test?x#y").
    
    userPtr     = NULL;
    userEnd     = NULL;
    passwordPtr = NULL;
    passwordEnd = NULL;
    hostPtr     = NULL;
    hostEnd     = NULL;
    
    if( ( ( inEnd - inSrc ) >= 2 ) && ( inSrc[ 0 ] == '/' ) && ( inSrc[ 1 ] == '/' ) )
    {
        const char *        authorityPtr;
        const char *        authorityEnd;
        const char *        userInfoPtr;
        const char *        userInfoEnd;
        
        inSrc += 2;
        authorityPtr = inSrc;
        while( ( inSrc < inEnd ) && ( ( c = *inSrc ) != '/' ) && ( c != '?' ) && ( c != '#' ) ) ++inSrc;
        authorityEnd = inSrc;
        
        // Parse an optional userinfo (the "tom:secret" in the above URL).
        
        userInfoPtr = authorityPtr;
        userInfoEnd = userInfoPtr;
        while( ( userInfoEnd < authorityEnd ) && ( *userInfoEnd != '@' ) ) ++userInfoEnd;
        if( userInfoEnd < authorityEnd )
        {
            // Parse the username (the "tom" in the above URL).
            
            userPtr = userInfoPtr;
            userEnd = userPtr;
            while( ( userEnd < userInfoEnd ) && ( *userEnd != ':' ) ) ++userEnd;
            if( userEnd < userInfoEnd )
            {
                // The rest is password/auth info. Note: passwords in URLs are deprecated (see RFC 3986 section 3.2.1).
                
                passwordPtr = userEnd + 1;
                passwordEnd = userInfoEnd;
            }
            
            // The host is the rest of the authority (the "abc.com" in "ftp://tom:secret@abc.com/test?x#y").
            
            hostPtr = userInfoEnd + 1;
            hostEnd = authorityEnd;
        }
        else
        {
            // The host is the entire authority (the "abc.com" in "ftp://tom:secret@abc.com/test?x#y").
            
            hostPtr = authorityPtr;
            hostEnd = authorityEnd;
        }
    }
    
    // Parse the path (the "/test" in "ftp://tom:secret@abc.com/test?x#y").
    
    c = '\0';
    pathPtr = inSrc;
    while( ( inSrc < inEnd ) && ( ( c = *inSrc ) != '?' ) && ( c != '#' ) ) ++inSrc;
    pathEnd = inSrc;
    
    // Parse an optional query (the "x" in "ftp://tom:secret@abc.com/test?x#y").
    
    queryPtr = NULL;
    queryEnd = NULL;    
    if( c == '?' )
    {
        queryPtr = ++inSrc;
        while( ( inSrc < inEnd ) && ( ( c = *inSrc ) != '#' ) ) ++inSrc;
        queryEnd = inSrc;
    }
    
    // Parse an optional fragment  (the "y" in "ftp://tom:secret@abc.com/test?x#y").
    
    fragmentPtr = NULL;
    fragmentEnd = NULL;
    if( c == '#' )
    {
        fragmentPtr = ++inSrc;
        fragmentEnd = inEnd;
        inSrc = inEnd;
    }
    
    outComponents->schemePtr    = schemePtr;
    outComponents->schemeLen    = (size_t)( schemeEnd - schemePtr );
    outComponents->userPtr      = userPtr;
    outComponents->userLen      = (size_t)( userEnd - userPtr );
    outComponents->passwordPtr  = passwordPtr;
    outComponents->passwordLen  = (size_t)( passwordEnd - passwordPtr );
    outComponents->hostPtr      = hostPtr;
    outComponents->hostLen      = (size_t)( hostEnd - hostPtr );
    outComponents->pathPtr      = pathPtr;
    outComponents->pathLen      = (size_t)( pathEnd - pathPtr );
    outComponents->queryPtr     = queryPtr;
    outComponents->queryLen     = (size_t)( queryEnd - queryPtr );
    outComponents->fragmentPtr  = fragmentPtr;
    outComponents->fragmentLen  = (size_t)( fragmentEnd - fragmentPtr );
    outComponents->segmentPtr   = ( ( pathPtr < pathEnd ) && ( *pathPtr      == '/' ) ) ? ( pathPtr + 1 ) : pathPtr;
    outComponents->segmentEnd   = ( ( pathPtr < pathEnd ) && ( pathEnd[ -1 ] == '/' ) ) ? ( pathEnd - 1 ) : pathEnd;
    if( outSrc ) *outSrc = inSrc;
    return kNoErr;
}

void PrintURL( URLComponents *inURL )
{
    (void)inURL;
#if _MXOS_DEBUG_
    char *scheme   = DataToCString( (uint8_t*)inURL->schemePtr, inURL->schemeLen );
    char *user     = DataToCString( (uint8_t*)inURL->userPtr, inURL->userLen );
    char *password = DataToCString( (uint8_t*)inURL->passwordPtr, inURL->passwordLen );
    char *host     = DataToCString( (uint8_t*)inURL->hostPtr, inURL->hostLen );
    char *path     = DataToCString( (uint8_t*)inURL->pathPtr, inURL->pathLen );
    char *query    = DataToCString( (uint8_t*)inURL->queryPtr, inURL->queryLen );
    char *fragment = DataToCString( (uint8_t*)inURL->fragmentPtr, inURL->fragmentLen );
    char *segment  = DataToCString( (uint8_t*)inURL->segmentPtr, inURL->segmentEnd - inURL->segmentPtr );

    url_utils_log("scheme: %s", scheme);
    url_utils_log("user: %s", user);
    url_utils_log("password: %s", password);
    url_utils_log("host: %s", host);
    url_utils_log("path: %s", path);
    url_utils_log("query: %s", query);
    url_utils_log("fragment: %s", fragment);
    url_utils_log("segment: %s", segment);

    free( scheme );
    free( user );
    free( password );
    free( host );
    free( path );
    free( query );
    free( fragment );
    free( segment );
#endif // _MXOS_DEBUG_
}


