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
 * \file cerstdio.c
 * \brief Cerial driver for C Standard I/O 
 *
 * Standard C library I/O driver as a cerial implementation. Some POSIX
 * functions are required.
 *
 * Either one or two files are opened by cerstdio_open(). These files to be
 * opened should be seperated by a colon (":"). The first file is for input and
 * the second file is for output. If only one file is given, then that file is
 * used for both input and output. The output file is created if it doesn't
 * exist. A single dash may be used to read/write standard in/out.
 *
 * Read calls will return -1 on end-of-file and error indication.
 *
 * Output is immediately flushed on every write() call.
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include "cerial.h"

#define IN      0
#define OUT     1

void *cerstdio_open(const char *pathname, int flags){
    FILE **io = NULL;
    char *dup = NULL, *in_name, *out_name;
    void *rv = NULL;

    if((io = calloc(2, sizeof(FILE *))) == NULL){
        goto err;
    }

    /* open the in and out files */
    if((dup = strdup(pathname)) == NULL){
        goto err;
    }
    in_name = strtok(dup, ":");
    out_name = strtok(NULL, ":");
    if(!in_name && !out_name){
        goto err;
    }
    if(!out_name){
        if(!strncmp(in_name, "-", 2)){
            io[IN] = stdin;
            io[OUT] = stdout;
        } else {
            if((io[OUT] = io[IN] = fopen(in_name, "r+")) == NULL){
                goto err;
            }
        }
    } else {
        if((io[IN] = fopen(in_name, "r")) == NULL){
            if(!strncmp(in_name, "-", 2)){
                io[IN] = stdin;
            } else {
                goto err;
            }
        }
        if((io[OUT] = fopen(out_name, "w+")) == NULL){
            if(!strncmp(out_name, "-", 2)){
                io[OUT] = stdin;
            } else {
                goto err;
            }
            goto err;
        }
    }

    rv = (void *)io;
err:
    if(dup){
        free(dup);
    }
    if(!rv){
        /* failure cleanup */
        if(io){
            if(io[IN]){
                fclose(io[IN]);
            }
            if(io[OUT] && io[OUT] != io[IN]){
                fclose(io[OUT]);
            }
            /* and free FILE * memory */
            free(io);
        }
    }
    return rv;
}


int cerstdio_read(void *vio, void *buf, int count){
    FILE    **io = vio;
    size_t  rcnt;

    rcnt = fread(buf, 1, count, io[IN]);
    if(rcnt == 0 && (ferror(io[IN]) || ferror(io[IN]))){
        return -1;
    } else {
        return rcnt;
    }
}


int cerstdio_write(void *vio, const void *buf, int count){
    FILE    **io = vio;
    size_t  rcnt;

    rcnt = fwrite(buf, 1, count, io[OUT]);
    fflush(io[OUT]);

    if(rcnt == 0 && (ferror(io[OUT]) || ferror(io[OUT]))){
        return -1;
    } else {
        return rcnt;
    }
}


int cerstdio_ioctl(void *vio, int request, void *arg){
    switch(request){
    default:
        errno = EINVAL;
        return -1;
    }

    return 0;
}


int cerstdio_close(void *vio){
    FILE **io = vio;

    if(io[OUT] != io[IN]){
        fclose(io[OUT]);
    }
    fclose(io[IN]);

    return 0;
}


char *cerstdio_strerror(char *buf, size_t buflen){
    if(buflen < 1){
        return "strerror buffer too short";
    }

    strncpy(buf, strerror(errno), buflen);
    buf[buflen - 1] = 0;

    return buf;
}


/* describe our driver */
#ifndef _MSC_VER
const struct cerial_driver cerstdio = {
    .open = cerstdio_open,
    .read = cerstdio_read,
    .write = cerstdio_write,
    .ioctl = cerstdio_ioctl,
    .close = cerstdio_close,
    .strerror = cerstdio_strerror
};
#else
/* MS C++ compiler: let's hope the structure doesn't change */
const struct cerial_driver cerstdio = {
    cerstdio_open,
    cerstdio_read,
    cerstdio_write,
    cerstdio_ioctl,
    cerstdio_close,
    cerstdio_strerror
};
#endif

