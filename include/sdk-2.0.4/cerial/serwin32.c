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
 * \file serwin32.c
 * \brief Cerial driver for Windows serial devices
 */

#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include "cerial.h"

/** serwin32 handle */
struct csw {
    HANDLE port;    /**< serial port file handle */
    char *path;     /**< path used to open the serial port */
};


/* Serial debugging by dumping read/write bytes to stdin/stdout. */
#ifdef SERWIN32_DEBUG
#include <stdio.h>

static void csw_hexprint(char prefix, const void *buf, size_t count){
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
#endif /* SERWIN32_DEBUG */


void *csw_open(const char *pathname, int flags){
    struct csw  *h;
    int         pnlen;
    DCB         dcb;

    pnlen = strlen(pathname);
    h = NULL;

    /* allocate a handle */
    if((h = malloc(sizeof(struct csw))) == NULL){
        goto err;
    }
    if((h->path = malloc(pnlen + 16)) == NULL){
        goto err;
    }

    /* if the port hasn't been prefixed with \\.\ then do so */
    *h->path = '\0';
    if(strncasecmp(pathname, "COM", 3) == 0){
        strcpy(h->path, "\\\\.\\");
    }
    strncat(h->path, pathname, pnlen - 16);

    /* open the port */
    h->port = CreateFileA(
            h->path,
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            NULL
            );
    if(h->port == INVALID_HANDLE_VALUE){
        goto err;
    }

    /* default to 9600 8N1 with DTR */
    if(!GetCommState(h->port, &dcb)){
        goto err;
    }
    dcb.DCBlength = sizeof(dcb);
    dcb.BaudRate = 9600;
    dcb.ByteSize = 8;
    dcb.fBinary = TRUE;
    dcb.Parity = NOPARITY;
    dcb.fParity = FALSE;
    dcb.StopBits = ONESTOPBIT;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fNull = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    if(!SetCommState(h->port, &dcb)){
        goto err;
    }

    return h;

err:
    if(h){
        if(h->path){
            free(h->path);
        }
        free(h);
    }

    return NULL;
}


int csw_read(void *vh, void *buf, int count){
    struct csw  *h = (struct csw *)vh;
    DWORD       bread;

    if(!ReadFile(h->port, buf, count, &bread, NULL)){
        return -1;
    }

#ifdef SERWIN32_DEBUG
    if(bread > 0){
        csw_hexprint('<', buf, bread);
    }
#endif

    return (int)bread;
}


int csw_write(void *vh, const void *buf, int count){
    struct csw  *h = (struct csw *)vh;
    DWORD       bwrit;

    if(!WriteFile(h->port, buf, count, &bwrit, NULL)){
        return -1;
    }

#ifdef SERWIN32_DEBUG
    if(bwrit > 0){
        csw_hexprint('>', buf, bwrit);
    }
#endif

    return (int)bwrit;
}


static int csw_baud_set(struct csw *h, int baud){
    DCB dcb;

    if(GetCommState(h->port, &dcb)){
        dcb.BaudRate = baud;
        if(SetCommState(h->port, &dcb)){
            return 0;
        }
    }

    return -1;
}


static int csw_baud_get(struct csw *h, int *baud){
    DCB dcb;

    if(GetCommState(h->port, &dcb)){
        return (int)dcb.BaudRate;
    } else {
        return -1;
   }
}


/* set total read and write timeout */
static int csw_set_timeout(struct csw *h, int timeout){
    COMMTIMEOUTS cto;

    if(timeout < 0){
        /* make negative timeouts to no timeout */
        timeout = 0;
    }

    /* no interval timeout */
    cto.ReadIntervalTimeout = 0;

    /* no multiplier will be used for every byte. this timeout is absolute. */
    cto.ReadTotalTimeoutMultiplier = 0;
    cto.WriteTotalTimeoutMultiplier = 0;

    /* and share the timeout for read and write */
    cto.ReadTotalTimeoutConstant = timeout;
    cto.WriteTotalTimeoutConstant = timeout;

    if(!SetCommTimeouts(h->port, &cto)){
        return -1;
    } else {
        return 0;
    }
}


int csw_ioctl(void *vh, int request, void *arg){
    struct csw *h = (struct csw *)vh;

    switch(request){
    case CERIAL_IOCTL_BAUDRATE_GET:
        return csw_baud_get(h, (int *)arg);
    case CERIAL_IOCTL_BAUDRATE_SET:
        return csw_baud_set(h, *((int *)arg));
    case CERIAL_IOCTL_TIMEOUT_SET:
        return csw_set_timeout(h, *((int *)arg));
    default:
        return -1;
    }

    return 0;
}


int csw_close(void *vh){
    struct csw *h = (struct csw *)vh;

    /* disable event notification */
    SetCommMask(h->port, 0);
    /* drop DTR */
    EscapeCommFunction(h->port, CLRDTR);
    /* purge any outstanding reads/writes and close device handle */
    PurgeComm(h->port, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR |
            PURGE_RXCLEAR);
    /* close */
    CloseHandle(h->port);

    /* free memory */
    free(h->path);
    h->path = NULL;
    free(h);

    return 0;
}


char *csw_strerror(char *buf, size_t buflen){
    DWORD errnum;
    char *crnl;

    /* format */
    errnum = GetLastError();
    if(!FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM, 0, errnum, 0, buf,
                buflen, NULL)){
        snprintf(buf, buflen, "0x%08X", (unsigned)errnum);
    }

    /* terminate the string at the first \r or \n, if any */
    if((crnl = strchr(buf, '\r')) || (crnl = strchr(buf, '\n'))){
        *crnl = '\0';
    }

    return buf;
}


/* describe our driver */
#ifndef _MSC_VER
const struct cerial_driver serwin32 = {
    .open = csw_open,
    .read = csw_read,
    .write = csw_write,
    .ioctl = csw_ioctl,
    .close = csw_close,
    .strerror = csw_strerror
};
#else
/* MS C++ compiler: let's hope the structure doesn't change */
const struct cerial_driver serwin32 = {
    csw_open,
    csw_read,
    csw_write,
    csw_ioctl,
    csw_close,
    csw_strerror
};
#endif

