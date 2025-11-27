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
 * \file cerial.c
 * \brief Cerial I/O abstraction interface.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cerial.h"

/**
 * \def SERIAL_DRV
 * Defines a pointer to the serial driver for the target platform.
 */
#ifdef WIN32
#include "serwin32.h"
#define SERIAL_DRV  (&serwin32)
#else
#include "serposix.h"
#define SERIAL_DRV  (&serposix)
#endif
/* stdio driver */
#include "cerstdio.h"
/* TCP driver */
#include "tcpbsd.h"


/** Open a Cerial device
 *
 * The path provided selects both the driver and the path passed to the driver.
 * The system's serial driver is selected by default. Selecting a different
 * Cerial driver can be done at run-time by prefixing the path with the driver
 * class followed by a semicolon. Supported classes are:
 *  \li serial  (serial devices)
 *  \li tcp     (TCP/IP devices)
 *  \li file    (file I/O)
 *
 * For example, selecting a TCP/IP device:
 * \code
 * ceropen(cer, "tcp:4.22.31.3", CERIAL_FLAGS_NONE);
 * \endcode
 *
 * ceropen() takes an pre-allocated struct cerial to avoid malloc()/free(). It
 * is the user's responsibility to free this memory and to call cerclose()
 * before this memory is freed.
 *
 * \param cer       Pre-allocated cerial handle to initalize.
 * \param pathname  Device path to open.
 * \param flag      Driver flags.
 *
 * \returns zero (0) on success and -1 on error. cerstrerror() may be called
 * on error for a detailed description.
 */
int ceropen(cerialh cer, const char *pathname, int flags){
    char        *dup = NULL, *type;
    const char  *new_pathname;
    int         rv = -1;

    /* make sure we don't call on uninit memory in the error case */
    cer->drv = NULL;

    /* the first word before a colon is the cerial driver class
     * default to serial class
     */
    cer->drv = SERIAL_DRV;
    new_pathname = pathname;
    if((dup = strdup(pathname)) == NULL){
        goto err;
    }
    type = strchr(dup, ':');
    if(type){
        *type = '\0';
        new_pathname = type + 1;
        if(!strncmp(dup, "file", 5)){
            cer->drv = &cerstdio;
        } else if(!strncmp(dup, "serial", 7)){
            cer->drv = SERIAL_DRV;
        } else if(!strncmp(dup, "tcp", 4)){
            cer->drv = &tcpbsd;
        }
    }

    if((cer->port = cer->drv->open(new_pathname, flags)) == NULL){
        goto err;
    }

    rv = 0;
err:
    if(dup){
        free(dup);
    }
    return rv;
}


/** Read bytes from a Cerial device.
 *
 * \param cer   Cerial device handle.
 * \param buf   Buffer to read into.
 * \param count Maximum number of bytes to read into buf.
 *
 * \returns number of bytes read. This may be less than the requested count or
 * negative on error.
 */
int cerread(cerialh cer, void *buf, int count){
    return cer->drv->read(cer->port, buf, count);
}


/** Write bytes to a Cerial device.
 *
 * \param cer   Cerial device handle.
 * \param buf   Buffer to write from.
 * \param count Requested number of bytes to write from buf.
 *
 * \returns number of bytes written. This may be less than the requested
 * count or negative on error.
 */
int cerwrite(cerialh cer, const void *buf, int count){
    return cer->drv->write(cer->port, buf, count);
}


/** I/O Control
 *
 * cerioctl() allows manipulation of the underlaying driver.
 *
 * \param cer       Cerial device handle.
 * \param request   I/O request.
 * \param arg       I/O request argument.
 *
 * \returns Zero (0) on success.
 */
int cerioctl(cerialh cer, int request, void *arg){
    return cer->drv->ioctl(cer->port, request, arg);
}


/** Close a Cerial device.
 *
 * \param cer   Cerial device handle.
 *
 * \returns Zero (0) on success.
 */
int cerclose(cerialh cer){
    if(cer->port){
        return cer->drv->close(cer->port);
    } else {
        return -1;
    }
}


/** Get a string description of a Cerial device's error.
 *
 * Example:
 * \code
 * {
 *     char err[128];
 *     printf("Error: %s\n", cerstrerror(cer, err, sizeof(err)));
 * }
 * \endcode
 *
 * \param cer   Cerial device handle.
 * \param estr  String buffer to write the error into.
 * \param size  Maximum length of estr.
 *
 * \returns a pointer to the error string. This may be the same pointer as
 * estr, but it also may be a pointer to static memory.
 */
const char *cerstrerror(cerialh cer, char *estr, size_t size){
    if(cer->drv){
        return cer->drv->strerror(estr, size);
    } else {
        snprintf(estr, size, "Unknown");
        estr[size-1] = '\0';
        return estr;
    }
}


/** Writes the character c, cast as an unsigned char, to cerial.
 *
 * \param cer   Cerial device handle.
 * \param c     The character to write.
 *
 * \returns the character c or -1 on error.
 */
int cerputc(cerialh cer, int c){
    unsigned char ch = (unsigned char)c;

    if(cer->drv->write(cer->port, &ch, 1) != 1){
        return -1;
    }

    return c;
}


/** Reads a character, cast to an int, from cerial.
 *
 * \param cer   Cerial device handle.
 *
 * \returns a read character cast as an int or -1 on error.
 */
int cergetc(cerialh cer){
    unsigned char ch;

    if(cer->drv->read(cer->port, &ch, 1) != 1){
        return -1;
    }

    return (int)ch;
}


/** Write a complete block of data out.
 *
 * A driver's write is allowed to return with a short write. This function will
 * re-call the driver as many times as necessary to write out the entire block
 * of data. This function will only short-write when the driver's write fails
 * entirely (< 0 returned).
 *
 * \param cer   Cerial device handle.
 * \param buf   Buffer to write out.
 * \param count Number of bytes to write.
 *
 * \returns the number of bytes written.
 */
unsigned int cerblockwrite(cerialh cer, void *buf, int count){
    unsigned char   *b = (unsigned char *)buf;
    int             written, rc;

    written = 0;
    do {
        rc = cer->drv->write(cer->port, &b[written], count - written);
        if(rc > 0){
            written += rc;
        }
    } while(rc >= 0 && written < count);

    return written;
}

