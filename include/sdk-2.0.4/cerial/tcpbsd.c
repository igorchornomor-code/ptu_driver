/*****************************************************************************
*****              (C)2014 FLIR Commercial Systems, Inc.                 *****
*****                       All Rights Reserved.                         *****
*****                                                                    *****
*****     This source data and code (the "Code") may NOT be distributed  *****
*****     without the express prior written permission consent from of   *****
*****     FLIR Commercial Systems, Inc. ("FLIR").  FLIR PROVIDES THIS    *****
*****     CODE ON AN "AS IS" BASIS FOR USE BY RECIPIENT AT ITS OWN       *****
*****     RISK.  FLIR DISCLAIMS ALL WARRANTIES, WHETHER EXPRESS, IMPLIED *****
*****     OR STATUTORY, INCLUDING WITHOUT LIMITATION ANY IMPLIED         *****
*****     WARRANTIES OF TITLE, NON-INFRINGEMENT OF THIRD PARTY RIGHTS,   *****
*****     MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.          *****
*****     FLIR Commercial Systems, Inc. reserves the right to make       *****
*****     changes without further notice to the Code or any content      *****
*****     herein including to improve reliability, function or design.   *****
*****     FLIR Commercial Systems, Inc. shall not assume any liability   *****
*****     arising from the application or use of this code, data or      *****
*****     function.                                                      *****
*****                                                                    *****
*****     FLIR Commercial Systems, Inc.                                  *****
*****     Motion Control Systems                                         *****
*****     www.flir.com/mcs                                               *****
*****     mcs-support@flir.com                                           *****
*****************************************************************************/
/**
 * \file tcpbsd.c
 * \brief Cerial driver for TCP/IP using BSD style sockets
 */
#include <stdlib.h>
#include <assert.h>
#include <limits.h>
#include <sys/types.h>
#ifndef _MSC_VER
#include <sys/time.h>
#endif
#ifdef WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#else
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#endif
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include "cerial.h"

/* compile-time translation of standard calls */
#ifdef WIN32
#define FD_TYPE                     SOCKET
#define SOCK_CLOSE(...)             closesocket(__VA_ARGS__)
#define SOCK_IOCTL(s,cmd,argp)      ioctlsocket(s,cmd,(u_long *)argp)
#define SOCK_ERRNO                  WSAGetLastError()
#else
#define FD_TYPE                     int
#define SOCK_CLOSE(...)             close(__VA_ARGS__)
#define SOCK_IOCTL(...)             ioctl(__VA_ARGS__)
#define SOCK_ERRNO                  errno
#endif

/* also required for Win32 and maybe other systems */
#ifndef INVALID_SOCKET
#define INVALID_SOCKET              -1
#endif

/* Our read()/write() implementation uses signed integers to represent the
 * number of bytes to read/write or have read/written. Our interfacing uses
 * type casting without range checking or splitting a request into smaller
 * requests. Nomally (s)size_t is big enough to hold an int and read/write
 * requests are small, so just throw warnings instead of trying to make this
 * driver work on the edge cases.
 */
#if (INTMAXUL > SIZE_MAXUL) || (INT_MAXUL > SSIZE_MAXUL)
#warning    "size_t and/or ssize_t are not large enough for all possible "
            "read/write requests. Castings may fail."
#endif

/* our handle definition */
struct tbs {
    FD_TYPE fd;
    int timeout;
};
#define GET_TBS(vtsb)   ((struct tbs *)(vtsb))
#define GET_FD(vtsb)    (GET_TBS(vtsb)->fd)

/* debugging by dumping read/write bytes to stdin/stdout. */
#ifdef TCPBSD_DEBUG
#include <stdio.h>

static void tbs_hexprint(char prefix, const void *buf, size_t count){
    const unsigned char *b = (const unsigned char *)buf;
    size_t i;

    for(i = 0; i < count; i++){
        if((i % 8) == 0){
            printf("\n%c ", prefix);
        }
        printf("%02X", b[i]);
        if((i + 1) %8){
            printf(" ");
        }
    }
    fflush(stdout);
}
#endif /* TCPBSD_DEBUG */



/* add unsigned milliseconds to timeval a */
static void tbs_time_addms(struct timeval *a, unsigned int ms){
    a->tv_usec += (ms * 1000);
    a->tv_sec  += a->tv_usec / 1000000;
    a->tv_usec %= 1000000;
}


/* saturated difference between timeval a and timeval b
 * sets timeval diff to the difference if it is not NULL
 * returns difference in milliseconds
 */
static int tbs_time_diff(struct timeval *a, struct timeval *b,
        struct timeval *diff){
    /* difference */
    if(a->tv_usec < b->tv_usec){
        a->tv_sec   -= 1;
        a->tv_usec  += 1000000;
    }
    diff->tv_usec = a->tv_usec - b->tv_usec;
    diff->tv_sec  = a->tv_sec  - b->tv_sec;

    /* saturate at zero if things went negative on us */
    if(diff->tv_sec < 0){
        diff->tv_sec = 0;
        diff->tv_usec = 0;
    }

    /* return time in milliseconds */
    return (diff->tv_sec * 1000) + ((diff->tv_usec + 500) / 1000);
}


/* get current local time and put it in timeval tv.
 * this function is equivalent to gettimeofday(tv, NULL)
 */
static int tbs_time(struct timeval *tv){
#ifdef WIN32
    FILETIME ft;
    ULARGE_INTEGER time;

    /* load current time */
    GetSystemTimeAsFileTime(&ft);
    time.LowPart = ft.dwLowDateTime;
    time.HighPart = ft.dwHighDateTime;

    /* stuff into a tv
     * note: Windows file time is in 10e-7 seconds
     */
    tv->tv_sec  = (long)(time.QuadPart / 10000000ULL);
    tv->tv_usec = (long)(time.QuadPart % 10000000ULL);

    return 0;
#else
    return gettimeofday(tv, NULL);
#endif
}


void *tbs_open(const char *pathname, int flags){
    FD_TYPE             fd = INVALID_SOCKET;
    int                 nb, conrc;
    struct tbs          *ptbs;
    struct addrinfo     hints, *res = NULL, *rp;
    char                *pndup = NULL, *node, *service;

    /* Windows requires odd things to load the sockets layer */
#ifdef WIN32
    {
        WSADATA wsad;

        if(WSAStartup(MAKEWORD(2,2), &wsad)){
            return NULL;
        }
    }
#endif

    /* allocate a descriptor */
    if((ptbs = malloc(sizeof(struct tbs))) == NULL){
        errno = ENOMEM;
        goto err;
    }

    /* split the name into the hostname and port (name/service)
     * if the split fails, then pathname is used as a full hostname and port
     * 4000 is used as the service.
     */
    if((pndup = strdup(pathname)) == NULL){
        goto err;
    }
    node = pndup;
    service = strchr(pndup, ':');
    if(service){
        /* service string is one past the colon */
        *service = '\0';
        service++;
    } else {
        /* no service described; use port 4000 */
        service = "4000";
    }

    /* lookup records and connect to them in order */
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = 0;
    hints.ai_flags = 0;
    if(getaddrinfo(node, service, &hints, &res)){
        /* failed lookup */
        goto err;
    }
    /* try every record */
    for(rp = res, conrc = -1; rp != NULL && conrc == -1; rp = rp->ai_next){
        fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if(fd != INVALID_SOCKET &&
                (conrc = connect(fd, rp->ai_addr, rp->ai_addrlen)) == -1){
            /* failed to connect */
            SOCK_CLOSE(fd);
        }
    }
    if(conrc == -1){
        /* couldn't connect to any record */
        goto err;
    }

    /* set non-blocking */
    nb = 1;
    if(SOCK_IOCTL(fd, FIONBIO, &nb)){
        goto err;
    }

    /* complete and return the descriptor */
    freeaddrinfo(res);
    free(pndup);
    ptbs->fd = fd;
    ptbs->timeout = -1;
    return ptbs;

err:
    if(res){
        freeaddrinfo(res);
    }
    if(pndup){
        free(pndup);
    }
    if(ptbs){
        free(ptbs);
    }
    if(fd != INVALID_SOCKET){
        SOCK_CLOSE(fd);
    }
#ifdef WIN32
    WSACleanup();
#endif
    return NULL;
}


int tbs_read(void *vtsb, void *buf, int count){
    struct tbs      *tbs = GET_TBS(vtsb);
    FD_TYPE         fd;
    int             bytes, nfds, timeleft;
    fd_set          rfds;
    struct timeval  deadline, now, timeout, *pto;

    fd = tbs->fd;
    bytes = 0;

    /* negative reads are bad and cannot be typecast into size_t */
    assert(count >= 0);

    /* calculate the deadline for our read */
    if(tbs->timeout >= 0){
        if(tbs_time(&now)){
            goto err;
        }
        deadline = now;
        tbs_time_addms(&deadline, tbs->timeout);
        tbs_time_diff(&deadline, &now, &timeout);
        pto = &timeout;
    } else {
        /* no timeout */
        pto = NULL;
    }
    /* selection */
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);
    nfds = fd + 1;
    do {
        /* block for data */
        if(select(nfds, &rfds, NULL, NULL, pto) == -1){
            goto err;
        }
        if(FD_ISSET(fd, &rfds)){
            /* read */
            bytes += (int)recv(fd, ((char *)buf) + bytes,
                    (size_t)(count - bytes), 0);
        }
        /* mark read again */
        FD_SET(fd, &rfds);
        /* calculate next timeout */
        if(pto){
            if(tbs_time(&now)){
                goto err;
            }
            timeleft = tbs_time_diff(&deadline, &now, pto);
        }
    } while(bytes < count && (pto == NULL || timeleft > 0));

#ifdef TCPBSD_DEBUG
    if(bytes > 0){
        tbs_hexprint('<', buf, bytes);
    }
#endif

err:
    return bytes;
}


int tbs_write(void *vtsb, const void *buf, int count){
    struct tbs      *tbs = GET_TBS(vtsb);
    FD_TYPE         fd;
    int             bytes, nfds, timeleft;
    fd_set          wfds;
    struct timeval  deadline, now, timeout, *pto;

    fd = tbs->fd;
    bytes = 0;

    /* negative reads are bad and cannot be typecast into size_t */
    assert(count >= 0);

    /* calculate the deadline for our read */
    if(tbs->timeout >= 0){
        if(tbs_time(&now)){
            goto err;
        }
        deadline = now;
        tbs_time_addms(&deadline, tbs->timeout);
        tbs_time_diff(&deadline, &now, &timeout);
        pto = &timeout;
    } else {
        /* no timeout */
        pto = NULL;
    }
    /* selection */
    FD_ZERO(&wfds);
    FD_SET(fd, &wfds);
    nfds = fd + 1;
    do {
        /* block for data */
        if(select(nfds, NULL, &wfds, NULL, pto) == -1){
            goto err;
        }
        if(FD_ISSET(fd, &wfds)){
            /* write */
            bytes += (int)send(fd, ((char *)buf) + bytes,
                    (size_t)(count - bytes), 0);
        }
        /* mark write again */
        FD_SET(fd, &wfds);
        /* calculate next timeout */
        if(pto){
            if(tbs_time(&now)){
                goto err;
            }
            timeleft = tbs_time_diff(&deadline, &now, pto);
        }
    } while(bytes < count && (pto == NULL || timeleft > 0));

#ifdef TCPBSD_DEBUG
    if(bytes > 0){
        tbs_hexprint('>', buf, bytes);
    }
#endif

err:
    return bytes;
}


int tbs_ioctl(void *vtsb, int request, void *arg){
    switch(request){
    case CERIAL_IOCTL_TIMEOUT_SET:
        GET_TBS(vtsb)->timeout = *((int *)arg);
        break;
    default:
        errno = EINVAL;
        return -1;
    }

    return 0;
}


int tbs_close(void *vtsb){
    FD_TYPE fd = GET_FD(vtsb);

    SOCK_CLOSE(fd);
    free(vtsb);

#ifdef WIN32
    WSACleanup();
#endif

    return 0;
}


char *tbs_strerror(char *buf, size_t buflen){
    if(buflen < 1){
        return "strerror buffer too short";
    }

    strncpy(buf, strerror(errno), buflen);
    buf[buflen - 1] = 0;

    return buf;
}


/* describe our driver */
#ifndef _MSC_VER
const struct cerial_driver tcpbsd = {
    .open = tbs_open,
    .read = tbs_read,
    .write = tbs_write,
    .ioctl = tbs_ioctl,
    .close = tbs_close,
    .strerror = tbs_strerror
};
#else
/* MS C++ compiler: let's hope the structure doesn't change */
const struct cerial_driver tcpbsd = {
    tbs_open,
    tbs_read,
    tbs_write,
    tbs_ioctl,
    tbs_close,
    tbs_strerror
};
#endif

