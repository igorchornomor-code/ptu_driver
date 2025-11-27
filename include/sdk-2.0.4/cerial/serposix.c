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
 * \file serposix.c
 * POSIX serial driver as a cerial implementation.
 *
 * Tested on: Linux (Ubuntu 11.10)
 */

#include <stdlib.h>
#include <assert.h>
#include <limits.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include "cerial.h"


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

/** serposix handle */
struct csp {
    int fd;         /**< file descriptor */
    int timeout;    /**< current timeout */
};
#define GET_CSP(vcsp)   ((struct csp *)(vcsp))
#define GET_FD(vcsp)    (GET_CSP(vcsp)->fd)

/* Serial debugging by dumping read/write bytes to stdin/stdout. */
#ifdef SERPOSIX_DEBUG
#include <stdio.h>

static void csp_hexprint(char prefix, const void *buf, size_t count){
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
#endif /* SERPOSIX_DEBUG */



/* add unsigned milliseconds to timeval a */
static void csp_time_addms(struct timeval *a, unsigned int ms){
    a->tv_usec += (ms * 1000);
    a->tv_sec  += a->tv_usec / 1000000;
    a->tv_usec %= 1000000;
}


/* saturated difference between timeval a and timeval b
 * sets timeval diff to the difference if it is not NULL
 * returns difference in milliseconds
 */
static int csp_time_diff(struct timeval *a, struct timeval *b,
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


void *csp_open(const char *pathname, int flags){
    struct termios  tio;
    int             fd = -1, fopts, dtr;
    struct csp      *pcsp;

    /* allocate a descriptor */
    if((pcsp = malloc(sizeof(struct csp))) == NULL){
        errno = ENOMEM;
        goto err;
    }

    /* open the port */
    fopts = O_RDWR | O_NOCTTY | O_NONBLOCK;
    if((fd = open(pathname, fopts)) < 0){
        goto err;
    }

    /* change port settings to 9600 8N1 */
    if(tcgetattr(fd, &tio) < 0){
        goto err;
    }
    tio.c_iflag = IGNBRK | IGNPAR;
    tio.c_oflag = 0;
    tio.c_cflag = CS8 | CLOCAL | CREAD;
    tio.c_lflag = 0;
    cfsetispeed(&tio, B9600);
    cfsetospeed(&tio, B9600);

    /* flush and set new settings */
    if(tcflush(fd, TCIOFLUSH) || tcsetattr(fd, TCSANOW, &tio)){
        goto err;
    }

    /* try setting (fail is OK) DTR and RTS
     * This is for some serial converters that use these lines as take/give
     * bus access control.
     */
    dtr = TIOCM_DTR | TIOCM_RTS;
    ioctl(fd, TIOCMBIS, &dtr);

    /* complete and return the descriptor */
    pcsp->fd = fd;
    pcsp->timeout = -1;
    return pcsp;

err:
    if(pcsp){
        free(pcsp);
    }
    if(fd >= 0){
        close(fd);
    }
    return NULL;
}


int csp_read(void *vcsp, void *buf, int count){
    struct csp      *csp = GET_CSP(vcsp);
    int             fd, bytes, nfds, timeleft;
    fd_set          rfds;
    struct timeval  deadline, now, timeout, *pto;

    fd = csp->fd;
    bytes = 0;

    /* negative reads are bad and cannot be typecast into size_t */
    assert(count >= 0);

    /* calculate the deadline for our read */
    if(csp->timeout >= 0){
        if(gettimeofday(&now, NULL)){
            goto err;
        }
        deadline = now;
        csp_time_addms(&deadline, csp->timeout);
        csp_time_diff(&deadline, &now, &timeout);
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
            bytes += (int)read(fd, buf + bytes, (size_t)(count - bytes));
        }
        /* mark read again */
        FD_SET(fd, &rfds);
        /* calculate next timeout */
        if(pto){
            if(gettimeofday(&now, NULL)){
                goto err;
            }
            timeleft = csp_time_diff(&deadline, &now, pto);
        }
    } while(bytes < count && (pto == NULL || timeleft > 0));

#ifdef SERPOSIX_DEBUG
    if(retc > 0){
        csp_hexprint('<', buf, retc);
    }
#endif

err:
    return bytes;
}


int csp_write(void *vcsp, const void *buf, int count){
    struct csp      *csp = GET_CSP(vcsp);
    int             fd, bytes, nfds, timeleft;
    fd_set          wfds;
    struct timeval  deadline, now, timeout, *pto;

    fd = csp->fd;
    bytes = 0;

    /* negative reads are bad and cannot be typecast into size_t */
    assert(count >= 0);

    /* calculate the deadline for our read */
    if(csp->timeout >= 0){
        if(gettimeofday(&now, NULL)){
            goto err;
        }
        deadline = now;
        csp_time_addms(&deadline, csp->timeout);
        csp_time_diff(&deadline, &now, &timeout);
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
            bytes += (int)write(fd, buf + bytes, (size_t)(count - bytes));
        }
        /* mark write again */
        FD_SET(fd, &wfds);
        /* calculate next timeout */
        if(pto){
            if(gettimeofday(&now, NULL)){
                goto err;
            }
            timeleft = csp_time_diff(&deadline, &now, pto);
        }
    } while(bytes < count && (pto == NULL || timeleft > 0));

#ifdef SERPOSIX_DEBUG
    if(retc > 0){
        csp_hexprint('>', buf, retc);
    }
#endif

err:
    return bytes;
}


static int csp_baud_set(int fd, int baud){
    struct termios  tio;
    int             bconst;

    if(tcgetattr(fd, &tio)){
        return -1;
    }

    switch(baud){
    case 0:         bconst = B0;        break;
    case 50:        bconst = B50;       break;
    case 75:        bconst = B75;       break;
    case 110:       bconst = B110;      break;
    case 134:       bconst = B134;      break;
    case 150:       bconst = B150;      break;
    case 200:       bconst = B200;      break;
    case 300:       bconst = B300;      break;
    case 600:       bconst = B600;      break;
    case 1200:      bconst = B1200;     break;
    case 1800:      bconst = B1800;     break;
    case 2400:      bconst = B2400;     break;
    case 4800:      bconst = B4800;     break;
    case 9600:      bconst = B9600;     break;
    case 19200:     bconst = B19200;    break;
    case 38400:     bconst = B38400;    break;
    case 57600:     bconst = B57600;    break;
    case 115200:    bconst = B115200;   break;
    case 230400:    bconst = B230400;   break;
    default:        errno = EINVAL;     return -1;
    }

    if(cfsetispeed(&tio, bconst) || tcsetattr(fd, TCSANOW, &tio)){
        return -1;
    }
    return 0;
}


static int csp_baud_get(int fd, int *baud){
    struct termios  tio;
    int             n;

    if(tcgetattr(fd, &tio)){
        return -1;
    }

    switch(cfgetispeed(&tio)){
    case B0:        n = 0;      break;
    case B50:       n = 50;     break;
    case B75:       n = 75;     break;
    case B110:      n = 110;    break;
    case B134:      n = 134;    break;
    case B150:      n = 150;    break;
    case B200:      n = 200;    break;
    case B300:      n = 300;    break;
    case B600:      n = 600;    break;
    case B1200:     n = 1200;   break;
    case B1800:     n = 1800;   break;
    case B2400:     n = 2400;   break;
    case B4800:     n = 4800;   break;
    case B9600:     n = 9600;   break;
    case B19200:    n = 19200;  break;
    case B38400:    n = 38400;  break;
    case B57600:    n = 57600;  break;
    case B115200:   n = 115200; break;
    case B230400:   n = 230400; break;
    default:        errno = EINVAL; return -1;
    }

    *baud = n;
    return 0;
}


int csp_ioctl(void *vcsp, int request, void *arg){
    int fd = GET_FD(vcsp);

    switch(request){
    case CERIAL_IOCTL_BAUDRATE_GET:
        return csp_baud_get(fd, (int *)arg);
    case CERIAL_IOCTL_BAUDRATE_SET:
        return csp_baud_set(fd, *((int *)arg));
    case CERIAL_IOCTL_TIMEOUT_SET:
        GET_CSP(vcsp)->timeout = *((int *)arg);
        break;
    case CERIAL_IOCTL_FLUSH_INPUT:
        return tcflush(fd, TCIFLUSH);
    default:
        errno = EINVAL;
        return -1;
    }

    return 0;
}


int csp_close(void *vcsp){
    int fd = GET_FD(vcsp);

    close(fd);
    free(vcsp);

    return 0;
}


char *csp_strerror(char *buf, size_t buflen){
    if(buflen < 1){
        return "strerror buffer too short";
    }

    strncpy(buf, strerror(errno), buflen);
    buf[buflen - 1] = 0;

    return buf;
}


/* describe our driver */
const struct cerial_driver serposix = {
    .open = csp_open,
    .read = csp_read,
    .write = csp_write,
    .ioctl = csp_ioctl,
    .close = csp_close,
    .strerror = csp_strerror
};

