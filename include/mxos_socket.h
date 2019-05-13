/**
 ******************************************************************************
 * @file    mxos_socket.h
 * @author  William Xu
 * @version V1.0.0
 * @date    16-Sep-2014
 * @brief   This file provides all the headers of BSD socket APIs.
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
#ifndef __MXOSSOCKET_H__
#define __MXOSSOCKET_H__

#if defined __GNUC__
#include <sys/time.h>
#include <sys/select.h>
#endif
#include "mxos_errno.h"
#include "mxos_poll.h"

#ifdef __cplusplus
extern "C" {
#endif



/** @addtogroup MXOS_Core_APIs
  * @{
*/

/** @defgroup MXOS_SOCKET MXOS Socket Operations
  * @brief Communicate with other device using TCP or UDP over MXOS network
  * @{
  */

#define INADDR_NONE         ((uint32_t)0xffffffffUL)     /**< 255.255.255.255 */
#define INADDR_LOOPBACK     ((uint32_t)0x7f000001UL)     /**< 127.0.0.1 */
#define INADDR_ANY          ((uint32_t)0x00000000UL)     /**< 0.0.0.0 */
#define INADDR_BROADCAST    ((uint32_t)0xffffffffUL)     /**< 255.255.255.255 */

/** This macro can be used to initialize a variable of type struct in6_addr
    to the IPv6 wildcard address. */
#define IN6ADDR_ANY_INIT {{{0,0,0,0}}}

#define IS_IPV4_MAPPED_IPV6(psockaddr_in6) (((psockaddr_in6)->sin6_addr.un.u32_addr[0] == 0) && \
                                            ((psockaddr_in6)->sin6_addr.un.u32_addr[1] == 0) && \
                                            ((psockaddr_in6)->sin6_addr.un.u32_addr[2] == htonl(0x0000FFFFUL)))

#define UNMAP_IPV4_MAPPED_IPV6(psockaddr_in4, psockaddr_in6) \
  (psockaddr_in4)->sin_addr.s_addr = (psockaddr_in6)->sin6_addr.un.u32_addr[3]; \
  (psockaddr_in4)->sin_family = AF_INET; \
  (psockaddr_in4)->sin_len = sizeof(struct sockaddr_in); \

struct in_addr {
    uint32_t s_addr;
};

struct in6_addr {
  union {
      uint32_t u32_addr[4];
      uint8_t  u8_addr[16];
  } un;
#define s6_addr  un.u8_addr
};

/* members are in network byte order */
struct sockaddr {
    uint8_t sa_len;
    uint8_t sa_family;
    uint8_t sa_data[14];
};

struct sockaddr_in {
    uint8_t sin_len;
    uint8_t sin_family;
    uint16_t sin_port;
    struct in_addr sin_addr;
    char sin_zero[8];
};

struct sockaddr_in6 {
  uint8_t         sin6_len;      /* length of this structure    */
  uint8_t         sin6_family;   /* AF_INET6                    */
  uint16_t        sin6_port;     /* Transport layer port #      */
  uint32_t        sin6_flowinfo; /* IPv6 flow information       */
  struct in6_addr sin6_addr;     /* IPv6 address                */
  uint32_t        sin6_scope_id; /* Set of interfaces for scope */
};

struct sockaddr_storage {
    uint8_t     s2_len;
    uint8_t     ss_family;
    char        s2_data1[2];
    uint32_t    s2_data2[3];
    uint32_t    s2_data3[3];
};

extern struct in_addr in_addr_any;
extern struct in6_addr in6_addr_any;

#ifndef socklen_t
#  define socklen_t uint32_t
#endif

struct hostent {
    char  *h_name;      /* Official name of the host. */
    char **h_aliases;   /* A pointer to an array of pointers to alternative host names,
                           terminated by a null pointer. */
    int    h_addrtype;  /* Address type. */
    int    h_length;    /* The length, in bytes, of the address. */
    char **h_addr_list; /* A pointer to an array of pointers to network addresses (in
                           network byte order) for the host, terminated by a null pointer. */
#define h_addr h_addr_list[0] /* for backward compatibility */
};


struct addrinfo {
    int               ai_flags;      /* Input flags. */
    int               ai_family;     /* Address family of socket. */
    int               ai_socktype;   /* Socket type. */
    int               ai_protocol;   /* Protocol of socket. */
    socklen_t         ai_addrlen;    /* Length of socket address. */
    struct sockaddr  *ai_addr;       /* Socket address of socket. */
    char             *ai_canonname;  /* Canonical name of service location. */
    struct addrinfo  *ai_next;       /* Pointer to next in list. */
};

/* Socket protocol types (TCP/UDP/RAW) */
#define SOCK_STREAM     1
#define SOCK_DGRAM      2
#define SOCK_RAW        3

#define  SOL_SOCKET  0xfff    /* options for socket level */

#define AF_UNSPEC       0
#define AF_INET         2
#define AF_INET6        10

#define PF_UNSPEC       AF_UNSPEC
#define PF_INET         AF_INET
#define PF_INET6        AF_INET6

#define IPPROTO_IP      0
#define IPPROTO_ICMP    1
#define IPPROTO_TCP     6
#define IPPROTO_UDP     17
#define IPPROTO_IPV6    41
#define IPPROTO_ICMPV6  58
#define IPPROTO_UDPLITE 136

#define F_GETFL 3
#define F_SETFL 4



/*
 * Options for level IPPROTO_IP
 */
#define IP_TOS             1
#define IP_TTL             2


#ifndef INET_ADDRSTRLEN
#define INET_ADDRSTRLEN     16
#endif

#ifndef INET6_ADDRSTRLEN
#define INET6_ADDRSTRLEN    46
#endif

typedef struct ip_mreq {
    struct in_addr imr_multiaddr; /* IP multicast address of group */
    struct in_addr imr_interface; /* local IP address of interface */
} ip_mreq;


/**
  * @brief  Socket option types, level: SOL_SOCKET
  */
typedef enum {
    SO_DEBUG              = 0x0001,     /**< Unimplemented: turn on debugging info recording */
    SO_ACCEPTCONN         = 0x0002,     /**< socket has had listen() */
    SO_REUSEADDR          = 0x0004,     /**< Allow local address reuse */
    SO_KEEPALIVE          = 0x0008,     /**< keep connections alive */
    SO_DONTROUTE          = 0x0010,     /**< Just use interface addresses */
    SO_BROADCAST          = 0x0020,     /**< Permit to send and to receive broadcast messages */
    SO_USELOOPBACK        = 0x0040,     /**< Bypass hardware when possible */
    SO_LINGER             = 0x0080,     /**< linger on close if data present */
    SO_OOBINLINE          = 0x0100,     /**< Leave received OOB data in line */
    SO_REUSEPORT          = 0x0200,     /**< Allow local address & port reuse */
    SO_BLOCKMODE          = 0x1000,     /**< set socket as block(optval=0)/non-block(optval=1) mode.
                                             Default is block mode. */
    SO_SNDBUF             = 0x1001,
    SO_SNDTIMEO           = 0x1005,     /**< Send timeout in block mode. block for ever in dafault mode. */
    SO_RCVTIMEO           = 0x1006,     /**< Recv timeout in block mode. block 1 second in default mode. */
    SO_ERROR              = 0x1007,     /**< Get socket error number. */
    SO_TYPE               = 0x1008,     /**< Get socket type. */
    SO_NO_CHECK           = 0x100a      /**< Don't create UDP checksum. */

} SOCK_OPT_VAL;


/**
  * @brief  IP option types, level: IPPROTO_IP
  */
typedef enum {
    IP_ADD_MEMBERSHIP       = 0x0003,     /**< Join multicast group. */
    IP_DROP_MEMBERSHIP      = 0x0004,     /**< Leave multicast group. */
    IP_MULTICAST_TTL        = 0x0005,
    IP_MULTICAST_IF         = 0x0006,
    IP_MULTICAST_LOOP       = 0x0007
} IP_OPT_VAL;

/**
  * @brief  IPv6 option types, level: IPPROTO_IPV6
  */
#define IPV6_UNICAST_HOPS   4  /* int; IP6 hops */
#define IPV6_MULTICAST_IF   9  /* __uint8_t; set/get IP6 multicast i/f  */
#define IPV6_MULTICAST_HOPS 10 /* __uint8_t; set/get IP6 multicast hops */
#define IPV6_MULTICAST_LOOP 11 /* __uint8_t; set/get IP6 mcast loopback */
#define IPV6_JOIN_GROUP     12 /* ip6_mreq; join a group membership */
#define IPV6_LEAVE_GROUP    13 /* ip6_mreq; leave a group membership */

/*
 * Argument structure for IPV6_JOIN_GROUP and IPV6_LEAVE_GROUP.
 */
struct ipv6_mreq {
    struct in6_addr ipv6mr_multiaddr;
    unsigned int    ipv6mr_interface;
};

/**
  * @brief  TCP option types, level: IPPROTO_TCP
  */
typedef enum {
	TCP_NODELAY             = 0x0001,
	TCP_KEEPALIVE           = 0x0002,
    TCP_CONN_NUM            = 0x0006,     /**< Read the current connected TCP client number. */
    TCP_MAX_CONN_NUM        = 0x0007,     /**< Set the max number of TCP client that server can support. */
    TCP_KEEPIDLE            = 0x0003,     /**< set pcb->keep_idle - send KEEPALIVE probes when idle for pcb->keep_idle milliseconds */
    TCP_KEEPINTVL           = 0x0004,     /**< set pcb->keep_intvl - Use seconds for get/setsockopt */
    TCP_KEEPCNT             = 0x0005,     /**< set pcb->keep_cnt - Use number of probes sent for get/setsockopt */
} TCP_OPT_VAL;

/* Flags we can use with send and recv. */
#define MSG_PEEK       0x01    /* Peeks at an incoming message */
#define MSG_WAITALL    0x02    /* Unimplemented: Requests that the function block until the full amount of data requested can be returned */
#define MSG_OOB        0x04    /* Unimplemented: Requests out-of-band data. The significance and semantics of out-of-band data are protocol-specific */
#define MSG_DONTWAIT   0x08    /* Nonblocking i/o for this operation only */
#define MSG_MORE       0x10    /* Sender will send more */


#if !defined(FIONREAD) || !defined(FIONBIO)
#define IOCPARM_MASK    0x7fU           /* parameters must be < 128 bytes */
#define IOC_VOID        0x20000000UL    /* no parameters */
#define IOC_OUT         0x40000000UL    /* copy out parameters */
#define IOC_IN          0x80000000UL    /* copy in parameters */
#define IOC_INOUT       (IOC_IN|IOC_OUT)
                                        /* 0x20000000 distinguishes new &
                                           old ioctl's */
#define _IO(x,y)        (IOC_VOID|((x)<<8)|(y))

#define _IOR(x,y,t)     (IOC_OUT|(((long)sizeof(t)&IOCPARM_MASK)<<16)|((x)<<8)|(y))

#define _IOW(x,y,t)     (IOC_IN|(((long)sizeof(t)&IOCPARM_MASK)<<16)|((x)<<8)|(y))
#endif /* !defined(FIONREAD) || !defined(FIONBIO) */

#ifndef FIONREAD
#define FIONREAD    _IOR('f', 127, unsigned long) /* get # bytes to read */
#endif
#ifndef FIONBIO
#define FIONBIO     _IOW('f', 126, unsigned long) /* set/clear non-blocking i/o */
#endif

typedef void* mxos_ssl_t;

/**
  * @brief  Supported SSL protocol version
  */
enum ssl_version_type_e
{
  SSL_V3_MODE   = 1,
  TLS_V1_0_MODE = 2,
  TLS_V1_1_MODE = 3,
  TLS_V1_2_MODE = 4,
};
typedef uint8_t ssl_version_type_t;

#if !defined __GNUC__

struct timeval {
    long      tv_sec;     /* seconds */
    long      tv_usec;    /* and microseconds */
};

#define O_NONBLOCK 1

#define FD_SETSIZE        64    /**< MAX fd number is 64 in MXOS. */
#define howmany(x, y)   (((x) + ((y) - 1)) / (y))

#define NBBY              8     /**< number of bits in a byte. */
#define NFDBITS (sizeof(unsigned long) * NBBY)        /**< bits per mask */

#define _fdset_mask(n)    ((unsigned long)1 << ((n) % NFDBITS))

typedef struct fd_set {
  unsigned long   fds_bits[howmany(FD_SETSIZE, NFDBITS)];
} fd_set;

#define FD_SET(n, p)      ((p)->fds_bits[(n)/NFDBITS] |= _fdset_mask(n))  /**< Add a fd to FD set. */
#define FD_CLR(n, p)      ((p)->fds_bits[(n)/NFDBITS] &= ~_fdset_mask(n)) /**< Remove fd from FD set. */
#define FD_ISSET(n, p)    ((p)->fds_bits[(n)/NFDBITS] & _fdset_mask(n))   /**< Check if the fd is set in FD set. */
#define FD_ZERO(p)        memset(p, 0, sizeof(*(p)))                      /**< Clear FD set. */

#endif

#ifndef SHUT_RD
  #define SHUT_RD   1
  #define SHUT_WR   2
  #define SHUT_RDWR 3
#endif

#define MAX_TCP_CLIENT_PER_SERVER  5


/** @defgroup MXOS_SOCKET_GROUP_1 MXOS BSD-like Socket Functions
  * @brief Provide basic APIs for socket function
  * @{
  */

/**
  * @brief  Create an endpoint for communication
  * @attention  Never doing operations on one socket in different MXOS threads
  * @param  domain: Specifies a communication domain; this selects the protocol
  *         family which will be used for communication. 
  *           This parameter can be one value:
  *             @arg AF_INET: IPv4 Internet protocols.
  * @param  type: Specifies the communication semantics.
  *           This parameter can be one of the following values:
  *             @arg SOCK_STREAM: Provides sequenced, reliable, two-way, 
  *                    connection-based byte streams.  An out-of-band data 
  *                    transmission mechanism may be supported. (TCP)
  *             @arg SOCK_DGRAM: Supports datagrams (connectionless, unreliable
  *                    messages of a fixed maximum length).(UDP)
  * @param  protocol: specifies a particular protocol to be used with the socket.
  *            This parameter can be one of the following values:
  *             @arg IPPROTO_TCP: TCP protocol
  *             @arg IPPROTO_UDP: UDP protocol
  * @retval On success, a file descriptor for the new socket is returned.  
            On error, -1 is returned.
  */
int socket(int domain, int type, int protocol);

/**
  * @brief  Set options on sockets
  * @attention  Never doing operations on one socket in different MXOS threads
  * @param  socket: A file descriptor
  * @param  level: This parameter can be : IPPROTO_IP, SOL_SOCKET, IPPROTO_TCP, IPPROTO_UDP
  * @param  optname: This parameter is defined in SOCK_OPT_VAL
  * @param  optval: address of buffer in which the value for the requested option(s) 
  *         are to be set.
  * @param  optlen: containing the size of the buffer pointed to by optval
  * @retval On success, zero is returned.  On error, -1 is returned.
  */
int setsockopt (int socket, int level, int optname, void *optval, socklen_t optlen);

/**
  * @brief  Get options on sockets
  * @attention  Never doing operations on one socket in different MXOS threads
  * @param  socket: A file descriptor
  * @param  level: This parameter can be : IPPROTO_IP, SOL_SOCKET, IPPROTO_TCP, IPPROTO_UDP
  * @param  optname: This parameter is defined in SOCK_OPT_VAL
  * @param  optval: address of buffer in which the value for the requested option(s)
  *         are to be returned.
  * @param  optlen_ptr: This is a value-result argument, initially containing the size
  *         of the buffer pointed to by optval, and modified on return to indicate 
  *         the actual size of the value returned. 
  * @retval On success, zero is returned.  On error, -1 is returned.
  */
int getsockopt (int socket, int level, int optname, void *optval, socklen_t *optlen_ptr);

/**
  * @brief  bind a name to a socket
  * @attention  Never doing operations on one socket in different MXOS threads
  * @note   Assigns the address specified by addr to the socket referred to by the file
  *         descriptor socket.
  * @param  socket: A file descriptor
  * @param  addr: Point to the target address to be binded
  * @param  length: This parameter containing the size of the buffer pointed to by addr
  * @retval On success, zero is returned.  On error, -1 is returned.
  */
int bind (int socket, struct sockaddr *addr, socklen_t length);

/**
  * @brief    Initiate a connection on a socket
  * @attention  Never doing operations on one socket in different MXOS threads
  * @details  The connect() system call connects the socket referred to by the file
  *           descriptor socket to the address specified by addr.
  * @param    socket: A file descriptor
  * @param    addr: Point to the target address to be binded
  * @param    length: This parameter containing the size of the buffer pointed to by addr
  * @retval   On success, zero is returned.  On error, -1 is returned.
  */
int connect (int socket, struct sockaddr *addr, socklen_t length);

/**
  * @brief    Listen for connections on a socket
  * @attention  Never doing operations on one socket in different MXOS threads
  * @details  listen() marks the socket referred to by socket as a passive socket,
  *           that is, as a socket that will be used to accept incoming connection
  *           requests using accept().
  * @param    socket: a file descriptor.
  * @param    n: Defines the maximum length to which the queue of pending
  *           connections for socket may grow. This parameter is not used in MXOS,
  *           use 0 is fine.
  * @retval   On success, zero is returned.  On error, -1 is returned.
  */
int listen (int socket, int n);

/**
  * @brief    Accept a connection on a socket
  * @attention  Never doing operations on one socket in different MXOS threads
  * @details  The accept() system call is used with connection-based socket types
  *           (SOCK_STREAM).  It extracts the first connection request on the queue
  *           of pending connections for the listening socket, sockfd, creates a 
  *           new connected socket, and returns a new file descriptor referring to 
  *           that socket.  The newly created socket is not in the listening state.  
  *           The original socket socket is unaffected by this call.
  * @param    socket: A file descriptor.
  * @param    addr: Point to the buffer to store the address of the accepted client.
  * @param    length_ptr: This parameter containing the size of the buffer pointed to
  *             by addr.
  * @retval   On success, zero is returned.  On error, -1 is returned.
  */
int accept (int socket, struct sockaddr *addr, socklen_t *length_ptr);

/**
  * @brief      Monitor multiple file descriptors, waiting until one or more of the 
  *             file descriptors become "ready" for some class of I/O operation 
  *             (e.g., input possible). 
  * @attention  Never doing operations on one socket in different MXOS threads
  * @note       A file descriptor is considered ready if it is possible to perform 
  *             the corresponding I/O operation (e.g., read()) without blocking.
  * @param      nfds: is the highest-numbered file descriptor in any of the three
  *             sets, plus 1. In MXOS, the mount of file descriptors is fewer, so  
  *             MXOS use the MAX number of these file descriptors inside, and this 
  *             parameter is cared.
  * @param      readfds: A file descriptor sets will be watched to see if characters 
  *             become available for reading
  * @param      writefds: A file descriptor sets will be watched to see if a write 
  *             will not block.
  * @param      exceptfds: A file descriptor sets will be watched for exceptions.
  * @param      timeout: The timeout argument specifies the interval that select() 
  *             should block waiting for a file descriptor to become ready. 
  *             If timeout is NULL (no timeout), select() can block indefinitely.   
  * @retval     On success, return the number of file descriptors contained in the 
  *             three returned descriptor sets (that is, the total number of bits 
  *             that are set in readfds, writefds, exceptfds) which may be zero if 
  *             the timeout expires before anything interesting happens.  On error, 
  *             -1 is returned, the file descriptor sets are unmodified, and timeout 
  *             becomes undefined.
  */
int select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds, struct timeval *timeout);


/**
  * @brief      Send a message on a socket
  * @attention  Never doing operations on one socket in different MXOS threads
  * @details    The send() call may be used only when the socket is in a connected
  *             state (so that the intended recipient is known). The only difference
  *             between send() and write() is the presence of flags.  With a zero
  *             flags argument, send() is equivalent to write().
  * @note       When the message does not fit into the send buffer of the socket,
  *             send() normally blocks, unless the socket has been placed in
  *             nonblocking I/O mode.  In nonblocking mode it would fail. The select()
  *             call may be used to determine when it is possible to send more data.        
  * @param      socket: A file descriptor.
  * @param      buffer: Point to the send data buffer.
  * @param      size: Length of the send data buffer.
  * @param      flags: Zero in MXOS.
  * @retval     On success, these calls return the number of bytes sent.  On error,
  *             -1 is returned,
  */
int send (int socket, const void *buffer, size_t size, int flags);

/**
  * @brief      Send a message on a socket
  * @attention  Never doing operations on one socket in different MXOS threads
  * @note       Refer send() for details.
  */
ssize_t write (int filedes, const void *buffer, size_t size);


/**
  * @brief      Send a message on a socket to a specific target address.
  * @attention  Never doing operations on one socket in different MXOS threads
  * @details    Refer send() for details. If sendto() is used on a connection-mode 
  *             (SOCK_STREAM, SOCK_SEQPACKET) socket, the arguments dest_addr and 
  *             addrlen are ignored. Otherwise, the address of the target is given by
  *             dest_addr with addrlen specifying its size. 
  * @param      socket: Refer send() for details.
  * @param      buffer: Refer send() for details.
  * @param      size: Refer send() for details.
  * @param      flags: Refer send() for details.
  * @param      addr: Point to the target address.
  * @param      length: This parameter containing the size of the buffer pointed to
  *             by addr.
  * @retval     On success, these calls return the number of bytes sent.  On error,
  *             -1 is returned,
  */
int sendto (int socket, const void *buffer, size_t size, int flags, const struct sockaddr *addr, socklen_t length);


/**
  * @brief      Receive a message from a socket.
  * @attention  Never doing operations on one socket in different MXOS threads
  * @details    If no messages are available at the socket, the receive calls wait
  *             for a message to arrive, unless the socket is nonblocking, in 
  *             which case the value -1 is returned.  The receive calls normally 
  *             return any data available, up to the requested amount, rather than 
  *             waiting for receipt of the full amount requested.
  * @param      socket: A file descriptor.
  * @param      buffer: Point to the send data buffer.
  * @param      size: Length of the send data buffer.
  * @param      flags: Zero in MXOS.
  * @retval     These calls return the number of bytes received, or -1 if an error
  *             occurred.
  *             When a stream socket peer has performed an orderly shutdown, the
  *             return value will be 0 (the traditional "end-of-file" return).
  *             The value 0 may also be returned if the requested number of bytes to
  *             receive from a stream socket was 0.
  */
int recv (int socket, void *buffer, size_t size, int flags);

/**
  * @brief      Receive a message from a socket.
  * @attention  Never doing operations on one socket in different MXOS threads
  * @note       Refer recv() for details.
  */
ssize_t read (int filedes, void *buffer, size_t size);


/**
  * @brief      Receive a message from a socket and get the source address.
  * @attention  Never doing operations on one socket in different MXOS threads
  * @details    If src_addr is not NULL, and the underlying protocol provides 
  *             the source address of the message, that source address is placed 
  *             in the buffer pointed to by src_addr.  In this case, addrlen is 
  *             a value-result argument.  Before the call, it should be 
  *             initialized to the size of the buffer associated with src_addr. 
  *             Upon return, addrlen is updated to contain the actual size of 
  *             the source address.  The returned address is truncated if the 
  *             buffer provided is too small; in this case, addrlen will return 
  *             a value greater than was supplied to the call.
  *             If the caller is not interested in the source address, src_addr
  *             should be specified as NULL and addrlen should be specified as 0.
  * @param      sockfd: Refer recv() for details.
  * @param      buf: Refer recv() for details.
  * @param      len: Refer recv() for details.
  * @param      flags: Refer recv() for details.
  * @param      src_addr: Point to the buffer to store the source address.
  * @param      addrlen: This parameter containing the size of the buffer pointed to 
  *             by src_addr.
  * @retval     These calls return the number of bytes received, or -1 if an 
  *             error occurred.
  *             When a stream socket peer has performed an orderly shutdown, the
  *             return value will be 0 (the traditional "end-of-file" return).
  *             The value 0 may also be returned if the requested number of bytes to
  *             receive from a stream socket was 0.
  */
int recvfrom (int socket, void *buffer, size_t size, int flags, struct sockaddr *addr, socklen_t *length_ptr);
 
/**
  * @brief      Close a file descriptor.
  * @attention  Never doing operations on one socket in different MXOS threads
  * @details    closes a file descriptor, so that it no longer refers to any
  *             file and may be reused.the resources associated with the
  *             open file description are freed.
  * @param      filedes: A file descriptor.
  * @retval     Returns zero on success.  On error, -1 is returned.
  */
int close (int filedes);

/**
  * @brief
  * @attention  Never doing operations on one socket in different MXOS threads
  * @details    The shutdown function shuts down the connection of socket socket.
  * 			The argument how specifies what action to perform.
  * @param      filedes: A file descriptor.
  * @param		how: 0 	 Stop receiving data for this socket. If further data arrives, reject it.
  *					 1 	 Stop trying to transmit data from this socket. Discard any data waiting to be sent.
  *					 	 Stop looking for acknowledgement of data already sent; don't retransmit it if it is lost.
  * @retval     Returns zero on success.  On error, -1 is returned.
  */
int shutdown(int filedes, int how);

/**
  * @brief      The ioctl function performs the generic I/O operation command on filedes.
  * @attention  Never doing operations on one socket in different MXOS threads
  * @details    A third argument is usually present, either a single number or a pointer to a structure.
  * 			The meaning of this argument.
  * @param      filedes: A file descriptor.
  * @param		command:Command
  * @retval     The returned value, and any error codes depends upon the command used.
  * 			Often -1 is returned for a failure.
  */
int ioctl(int filedes, int command, ...);

/**
  * @brief      The fcntl function performs the operation specified by command on the file descriptor filedes.
  * @attention  Never doing operations on one socket in different MXOS threads
  * @details    Some commands require additional arguments to be supplied. These additional arguments and
  * 			the return value and error conditions are given in the detailed descriptions of the individual commands.
  * @param      filedes: A file descriptor.
  * @param		command:Command
  * @retval     The returned value, and any error codes depends upon the command used.
  * 			Often -1 is returned for a failure.
  */
int fcntl(int filedes, int command, ...);


/**
  * @}
  */


/** @defgroup MXOS_SOCKET_GROUP_2 MXOS Socket Tool Functions
  * @brief Provide APIs for MXOS Socket Tool functions
  * @{
  */

/**
  * @brief      converts the Internet host address from IPv4 numbers-and-dots 
  *             notation into binary data in network byte order
  * @note       If the input is invalid, INADDR_NONE (usually -1) is returned. 
  * @param      name: Internet host address from IPv4 numbers-and-dots.
  * @retval     Returns zero on success.  On error, -1 is returned.
  */
uint32_t inet_addr (const char *name);

/**
  * @brief      Converts the Internet host address in, given in network byte 
  *             order, to a string in IPv4 dotted-decimal notation. 
  * @note       The returned string is stored in a given buffer, and the buffer 
  *             size should larger than 16.
  * @param      s: Point to a buffer to store the returned string in IPv4 
  *             dotted-decimal
  * @param      x: the Internet host address in.
  * @retval     Returns the same value as param s.
  */
char *inet_ntoa (struct in_addr addr);

/** @brief      Get the IP address from a host name. 
  * 
  * @note       Different to stand BSD function type, this function in MXOS do
  *             not return a buffer that contain the result, but write the result
  *             to a buffer provided by application. Also this function simplify
  *             the return value compared to the standard BSD version. 
  *             This function runs under block mode.
  *
  * @param      name: This parameter is either a hostname, or an IPv4 address in
  *             standard dot notation.
  * @param      addr: Point to a buffer to store the returned string in IPv4 
  *             dotted-decimal
  * @param      addrLen: This parameter containing the size of the buffer pointed 
  *             to by addr, 16 is recommended.
  * @retval     kNoerr or kGeneralErr
  */
struct hostent* gethostbyname(const char *name);

/**
 * @brief      converts an address *src from network format (usually a struct
 *             in_addr or some other binary form, in network byte order) to
 *             presentation format (suitable for external display purposes).
 *
 * @param      af:   AF_INET or AF_INET6
 * @param      src:  network address
 * @param      dst:  buffer holds the result
 * @param      size: size of the buffer, INET_ADDRSTRLEN and INET6_ADDRSTRLEN
 *                   define the maximum size required to convert an address of
 *                   the respective type.
 * @retval     NULL if a system error occurs, or returns a pointer to the
 *             destination string
 */

const char * inet_ntop(int af, const void * src, char * dst, socklen_t size);

/**
 * @brief      converts a presentation format address (that is, printable form
 *             as held in a character string) to network format (usually a struct
 *             in_addr or some other internal binary representation, in network byte order).
 *
 * @param      af:   AF_INET or AF_INET6
 * @param      src:  point to the presentation format address
 * @param      dst:  buffer holds the result
 * @retval     1 if the address was valid for the specified address family, or
 *             0 if the address was not parseable in the specified address family, or
 *             -1 if some system error occurred
 */
int inet_pton(int af, const char * src, void * dst);

/**
 * Translates the name of a service location (for example, a host name) and/or
 * a service name and returns a set of socket addresses and associated
 * information to be used in creating a socket with which to address the
 * specified service.
 * Memory for the result is allocated internally and must be freed by calling
 * freeaddrinfo()!
 *
 * Due to a limitation in dns_gethostbyname, only the first address of a
 * host is returned.
 * Also, service names are not supported (only port numbers)!
 *
 * @param nodename descriptive name or address string of the host
 *                 (may be NULL -> local address)
 * @param servname port number as string of NULL
 * @param hints structure containing input values that set socktype and protocol
 * @param res pointer to a pointer where to store the result (set to NULL on failure)
 * @return 0 on success, non-zero on failure
 */
int getaddrinfo(const char *nodename, const char *servname, const struct addrinfo *hints, struct addrinfo **res);

/**
 * Frees one or more addrinfo structures returned by getaddrinfo(), along with
 * any additional storage associated with those structures. If the ai_next field
 * of the structure is not null, the entire list of structures is freed.
 *
 * @param ai struct addrinfo to free
 */
void freeaddrinfo(struct addrinfo *ai);


int getpeername (int s, struct sockaddr *name, socklen_t *namelen);
int getsockname (int s, struct sockaddr *name, socklen_t *namelen);


/** @brief      Set TCP keep-alive mechanism parameters. 
 *
 *  @details    When TCP data is not transimitting for a certain time (defined by seconds), 
 *              MXOS send keep-alive package over the TCP socket, and the remote device  
 *              should return the keep-alive back to MXOS. This is a basic TCP function 
 *              deployed on every TCP/IP stack and application's interaction is not required. 
 *              If the remote device doesn't return the keep-alive package, MXOS add 1 to an 
 *              internal counter, and close the current socket connection once this count has 
 *              reached the maxErrNum (defined in parm: maxErrNum). 
 *
 *  @param      inMaxErrNum: The max possible count that the remote device doesn't return the 
 *              keep-alive package. If remote device returns, the internal count is cleared 
 *              to 0. 
 *  @param      inSeconds: The time interval between two keep-alive package
 *
 *  @retval     kNoerr or kGeneralErr
 */
void set_tcp_keepalive(int inMaxErrNum, int inSeconds);


/** @brief      Get TCP keep-alive mechanism parameters. Refer to @ref set_tcp_keepalive
 *
 *  @param      outMaxErrNum: Point to the address that store the maxErrNumber.
 *  @param      outSeconds: Point to the address that store the time interval between two
 *              keep-alive package.
 *
 *  @retval     kNoerr or kGeneralErr
 */
void get_tcp_keepalive(int *outMaxErrNum, int *outSeconds);


/* SSL */
/** @brief      Used to set the ssl protocol version for both ssl client and ssl server 
 *
 *  @note       This function should be called before ssl is ready to function (before
 *              ssl_connect and ssl_accept is called by application ).
 *
 *  @param      version: SSL protocol version, Refer SSL_VERSION for details.
 *
 *  @retval     void
 */
void ssl_set_client_version( ssl_version_type_t version );


/** @brief      Get the internal socket fire description
 *
 *  @note       This function should be called after ssl connection is established.
 *
 *  @param      ssl: SSL handler.
 *
 *  @retval     File descriptor for the SSL connection.
 */
int ssl_socket( mxos_ssl_t ssl );


/** @brief      Used by the SSL server. Set the certificate and private key for a SSL server. 
 *
 * @details     This function is called on the server side to set it's certifact and private key.
 *              It must be called before ssl_accept. These two arguments must be global
 *              string buffer, library will not create a copy for them.
 *
 *  @param      _cert_pem: Point to the certificate string in PEM format.
 *  @param      private_key_pem: Point to the private key string in PEM format.
 *
 *  @retval     void
 */
void ssl_set_cert(const char *_cert_pem, const char *private_key_pem);

/** @brief      SSL client create a SSL connection.
 *
 * @details    This function is called on the client side and initiates an SSL/TLS handshake with a 
 *              server.  When this function is called, the underlying communication channel has already 
 *              been set up. This function is called after TCP 3-way handshak finished. 
 *
 *  @param      fd: The fd number for a connected TCP socket.
 *  @param      calen: the string length of ca. 0=do not verify server's certificate.
 *  @param      ca: pointer to the CA certificate string, used to verify server's certificate.
 *  @param      errno: return the errno if ssl_connect return NULL.
 *
 *  @retval     return the SSL context pointer on success or NULL for fail.
 */
mxos_ssl_t ssl_connect(int fd, int calen, char*ca, int *errno);

/** @brief      SSL client create a SSL connection.
 *
 * @details    This function is called on the client side and initiates an SSL/TLS handshake with a 
 *              server.  When this function is called, the underlying communication channel has already 
 *              been set up. This function is called after TCP 3-way handshak finished. 
 *
 *  @param      fd: The fd number for a connected TCP socket.
 *  @param      calen: the string length of ca. 0=do not verify server's certificate.
 *  @param      ca: pointer to the CA certificate string, used to verify server's certificate.
 *  @param      sni_servername: sni server name.
 *  @param      errno: return the errno if ssl_connect return NULL.
 *
 *  @retval     return the SSL context pointer on success or NULL for fail.
 */
mxos_ssl_t ssl_connect_sni(int fd, int calen, char*ca, char *sni_servername, int *errno);

/** @brief      SSL Server accept a SSL connection
 *
 *  @param      fd: The fd number for a connected TCP socket.
 *
 *  @retval     return the SSL context pointer on success or NULL for fail.
 */
mxos_ssl_t ssl_accept(int fd);


/** @brief      SSL send data
 *
 *  @param      ssl: Point to the SSL context.
 *  @param      data: Point to send data.
 *  @param      len: data length.
 *
 *  @retval     On success, these calls return the number of bytes sent.  On error,
 *             -1 is returned,
 */
int ssl_send(mxos_ssl_t ssl, void* data, size_t len);

/** @brief      SSL receive data
 *
 *  @param      ssl: Point to the SSL context.
 *  @param      data: Point to buffer to receive SSL data.
 *  @param      len: max receive buffer length.
 *
 *  @retval     On success, these calls return the number of bytes received.  On error,
 *             -1 is returned,
 */
int ssl_recv(mxos_ssl_t ssl, void* data, size_t len);

/** @brief      Close the SSL session, release resource.
 *
 *  @param      ssl: Point to the SSL context.
 *
 *  @retval     kNoerr or kGeneralErr
 */
int ssl_close(mxos_ssl_t ssl);


int ssl_pending(void*ssl);
int ssl_get_error(void* ssl, int ret);
void ssl_set_using_nonblock(void* ssl, int nonblock);


/**
  * @}
  */




/**
  * @}
  */

/**
  * @}
  */
#ifdef __cplusplus
}
#endif

#endif /*__MXOS_SOCKET_H__*/



