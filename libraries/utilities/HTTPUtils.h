/**
 ******************************************************************************
 * @file    HTTPUtils.h
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This header contains function prototypes. These functions assist
 *          with interacting with HTTP clients and servers.
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


#ifndef __HTTPUtils_h__
#define __HTTPUtils_h__

#include "mxos_common.h"
#include "mxos_socket.h"

#include "URLUtils.h"
#include "stdbool.h"

#define Boolean bool

#define kHTTPPostMethod     "POST"

      // Status-Code    =
      //       "100"  ; Section 10.1.1: Continue
      //     | "101"  ; Section 10.1.2: Switching Protocols
      //     | "200"  ; Section 10.2.1: OK
      //     | "201"  ; Section 10.2.2: Created
      //     | "202"  ; Section 10.2.3: Accepted
      //     | "203"  ; Section 10.2.4: Non-Authoritative Information
      //     | "204"  ; Section 10.2.5: No Content
      //     | "205"  ; Section 10.2.6: Reset Content
      //     | "206"  ; Section 10.2.7: Partial Content
      //     | "207"  ; ???           : Multi-Status
      //     | "300"  ; Section 10.3.1: Multiple Choices
      //     | "301"  ; Section 10.3.2: Moved Permanently
      //     | "302"  ; Section 10.3.3: Found
      //     | "303"  ; Section 10.3.4: See Other
      //     | "304"  ; Section 10.3.5: Not Modified
      //     | "305"  ; Section 10.3.6: Use Proxy
      //     | "307"  ; Section 10.3.8: Temporary Redirect
      //     | "400"  ; Section 10.4.1: Bad Request
      //     | "401"  ; Section 10.4.2: Unauthorized
      //     | "402"  ; Section 10.4.3: Payment Required
      //     | "403"  ; Section 10.4.4: Forbidden
      //     | "404"  ; Section 10.4.5: Not Found
      //     | "405"  ; Section 10.4.6: Method Not Allowed
      //     | "406"  ; Section 10.4.7: Not Acceptable


#define kStatusAccept               202
#define kStatusOK                   200
#define kStatusNoConetnt            204
#define kStatusPartialContent       206
#define kStatusBadRequest           400
#define kStatusNotFound             404
#define kStatusMethodNotAllowed     405
#define kStatusForbidden            403  
#define kStatusAuthenticationErr    470  
#define kStatusInternalServerErr    500      

#define kMIMEType_Binary                "application/octet-stream"
#define kMIMEType_DMAP                  "application/x-dmap-tagged"
#define kMIMEType_ImagePrefix           "image/"
#define kMIMEType_JSON                  "application/json"
#define kMIMEType_SDP                   "application/sdp"
#define kMIMEType_TextHTML              "text/html"
#define kMIMEType_TextParameters        "text/parameters"
#define kMIMEType_TextPlain             "text/plain"
#define kMIMEType_TLV8                  "application/x-tlv8" // 8-bit type, 8-bit length, N-byte value.
#define kMIMEType_Pairing_TLV8          "application/pairing+tlv8" // 8-bit type, 8-bit length, N-byte value.
#define kMIMEType_MXCHIP_OTA            "application/ota-stream"
#define kMIMEType_Stream                "application/octet-stream"
#define kTransferrEncodingType_CHUNKED  "chunked"

#define OTA_Data_Length_per_read        1024


typedef struct _HTTPHeader_t
{
    char *              buf;                //! Buffer holding the start line and all headers.
    size_t              bufLen;             //! The size of the buffer.
    size_t              len;                //! Number of bytes in the header.
    char *              extraDataPtr;       //! Ptr for any extra data beyond the header, it is alloced when http header is received.
    char *              otaDataPtr;         //! Ptr for any OTA data beyond the header, it is alloced when one OTA package is received.
    size_t              extraDataLen;       //! Length of any extra data beyond the header.

    const char *        methodPtr;          //! Request method (e.g. "GET"). "$" for interleaved binary data.
    size_t              methodLen;          //! Number of bytes in request method.
    const char *        urlPtr;             //! Request absolute or relative URL or empty if not a request.
    size_t              urlLen;             //! Number of bytes in URL.
    URLComponents       url;                //! Parsed URL components.
    const char *        protocolPtr;        //! Request or response protocol (e.g. "HTTP/1.1").
    size_t              protocolLen;        //! Number of bytes in protocol.
    int                 statusCode;         //! Response status code (e.g. 200 for HTTP OK).
    const char *        reasonPhrasePtr;    //! Response reason phrase (e.g. "OK" for an HTTP 200 OK response).
    size_t              reasonPhraseLen;    //! Number of bytes in reason phrase.

    uint8_t             channelID;          //! Interleaved binary data channel ID. 0 for other message types.
    uint64_t            contentLength;      //! Number of bytes following the header. May be 0.
    bool                persistent;         //! true=Do not close the connection after this message.

    int                 firstErr;           //! First error that occurred or kNoErr.

    bool                dataEndedbyClose;
    bool                chunkedData;        //! true=Application should read the next chunked data.
    char *              chunkedDataBufferPtr;     //! Ptr for any extra data beyond the header, it is alloced when http header is received.
    size_t              chunkedDataBufferLen; //! Total buffer length that stores the chunkedData, private use only

    void *              userContext;
    bool                isCallbackSupported;
    mret_t            (*onReceivedDataCallback) ( struct _HTTPHeader_t * , uint32_t, uint8_t *, size_t, void * ); 
    void                (*onClearCallback) ( struct _HTTPHeader_t * httpHeader, void * userContext );



} HTTPHeader_t;

typedef mret_t (*onReceivedDataCallback) ( struct _HTTPHeader_t * httpHeader, uint32_t pos, uint8_t * data, size_t len, void * userContext );

typedef void (*onClearCallback) ( struct _HTTPHeader_t * httpHeader, void * userContext );

void PrintHTTPHeader( HTTPHeader_t *inHeader );

bool findHeader ( HTTPHeader_t *inHeader,  char **  outHeaderEnd);

int HTTPScanFHeaderValue( const char *inHeaderPtr, size_t inHeaderLen, const char *inName, const char *inFormat, ... );

int findCRLF( const char *inDataPtr , size_t inDataLen, char **  nextDataPtr );

int findChunkedDataLength( const char *inChunkPtr , size_t inChunkLen, char **  chunkedDataPtr, const char *inFormat, ... );

int SocketReadHTTPHeader( int inSock, HTTPHeader_t *inHeader );

int SocketReadHTTPBody( int inSock, HTTPHeader_t *inHeader );

int SocketReadHTTPSHeader( mxos_ssl_t ssl, HTTPHeader_t *inHeader );

int SocketReadHTTPSBody( mxos_ssl_t ssl, HTTPHeader_t *inHeader );

int HTTPHeaderParse( HTTPHeader_t *ioHeader );

int HTTPHeaderMatchMethod( HTTPHeader_t *inHeader, const char *method );

int HTTPHeaderMatchURL( HTTPHeader_t *inHeader, const char *url );

char* HTTPHeaderMatchPartialURL( HTTPHeader_t *inHeader, const char *url );


int HTTPGetHeaderField( const char *inHeaderPtr, 
                             size_t     inHeaderLen, 
                             const char *inName, 
                             const char **outNamePtr, 
                             size_t     *outNameLen, 
                             const char **outValuePtr, 
                             size_t     *outValueLen, 
                             const char **outNext );

HTTPHeader_t * HTTPHeaderCreate( size_t bufLen );

HTTPHeader_t * HTTPHeaderCreateWithCallback( size_t bufLen, onReceivedDataCallback , onClearCallback , void * context );

void HTTPHeaderClear( HTTPHeader_t *inHeader );

void HTTPHeaderDestory( HTTPHeader_t **inHeader );

int CreateSimpleHTTPOKMessage( uint8_t **outMessage, size_t *outMessageSize );

mret_t CreateSimpleHTTPMessage      ( const char *contentType, uint8_t *inData, size_t inDataLen, uint8_t **outMessage, size_t *outMessageSize );
mret_t CreateSimpleHTTPMessageNoCopy( const char *contentType, size_t inDataLen, uint8_t **outMessage, size_t *outMessageSize );

mret_t CreateHTTPRespondMessageNoCopy( int status, const char *contentType, size_t inDataLen, uint8_t **outMessage, size_t *outMessageSize );


mret_t CreateHTTPMessage( const char *methold, const char *url, const char *contentType, uint8_t *inData, size_t inDataLen, uint8_t **outMessage, size_t *outMessageSize );

mret_t CreateHTTPMessageWithHost( const char *methold, const char *url,
                           const char* host, uint16_t port, 
                           const char *contentType, 
                           uint8_t *inData, size_t inDataLen, 
                           uint8_t **outMessage, size_t *outMessageSize );

#endif // __HTTPUtils_h__

