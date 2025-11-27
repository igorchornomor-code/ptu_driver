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
 * \file estrap.c
 * \brief Example strap code
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include "estrap.h"
#include "cpi.h"

/* buffered file handle to print too. this is normally stdout or stderr. */
#define PRINT_HANDLE        stdout

/* string constants for e/w/iprintf */
const char *estrap_epstr = "[!] ";
const char *estrap_wpstr = "[~] ";
const char *estrap_ipstr = "[*] ";


static void print_help(const char *progname){
    printf( "Usage: %s -p <port> [optional arguments]\n"
            "Options:\n"
            " -h            - This help message.\n"
            " -p <port>     - Connect to serial/socket <port>.\n"
            " -b <baud>     - Use baud rate <baud> (default: 9600).\n",
            progname);
}


/** Strap into the example harness.
 *
 * \param argc  Argument count, as passed into main().
 * \param argv  Argument vector, as passed into main().
 *
 * \returns a Cerial handle or NULL on error.
 */
struct cerial *estrap_in(int argc, char *argv[]){
    int             c, baud, timeout, try, rc;
    char            errstr[128], *portname, *progname;
    struct cerial   *cer;
    uint16_t        status;

    cer = NULL;

    /* get options */
    baud = 9600;
    portname = NULL;
    progname = argv[0];
    while((c = getopt(argc, argv, "hp:b:")) != -1){
        switch(c){
        case 'h':
            print_help(progname);
            goto err;
            break;
        case 'p':
            portname = optarg;
            break;
        case 'b':
            baud = atoi(optarg);
            break;
        default:
            eprintf("Options are malformed.\n");
            goto err;
        }
    }
    if(!portname){
        eprintf("Port option is required.\n");
        goto err;
    }

    if((cer = malloc(sizeof(struct cerial))) == NULL){
        eprintf("Out of memory.\n");
        goto err;
    }

    /* open a port */
    if(ceropen(cer, portname, 0)){
        eprintf("Failed to open %s: %s\n",
                portname, cerstrerror(cer, errstr, sizeof(errstr)));
        goto err;
    }

    /* set baudrate
     * ignore errors since not all devices are serial ports
     */
    cerioctl(cer, CERIAL_IOCTL_BAUDRATE_SET, &baud);

    /* flush any characters already buffered */
    cerioctl(cer, CERIAL_IOCTL_FLUSH_INPUT, NULL);

    /* set five second timeout */
    timeout = 5000;
    if(cerioctl(cer, CERIAL_IOCTL_TIMEOUT_SET, &timeout)){
        wprintf("cerial: timeout ioctl not supported\n");
    }

    /* sync and lock */
    try = 0;
    do {
        try++;
    } while(try <= 3 && (cpi_resync(cer) || cpi_ptcmd(cer, &status, OP_NOOP)));
    if(try > 3){
        eprintf("Cannot communicate with PTU.\n");
        goto err;
    }

    /* immediately execute commands
     * (slave mode should be opt-in)
     */
    if((rc = cpi_ptcmd(cer, &status, OP_EXEC_MODE_SET,
                    (cpi_enum)CPI_IMMEDIATE_MODE))){
        die("Set Immediate Mode failed: %s\n", cpi_strerror(rc));
    }

    /* all good */
    return cer;

err:
    if(cer){
        cerclose(cer);
        free(cer);
    }
    return NULL;
}


/** Exit out of the example harness. */
void estrap_out(struct cerial *cer){
    cerclose(cer);
}


/** Print a message to standard out. */
int estrap_printf(const char *prefix, const char *fmt, ...){
    va_list ap;
    int     rc;

    va_start(ap, fmt);
    fwrite(prefix, 1, strlen(prefix), PRINT_HANDLE);
    rc = vfprintf(PRINT_HANDLE, fmt, ap);
    va_end(ap);

    return rc;
}


/** Print a message to standard out and exit(1) from the program.
 * This function never returns.
 */
void estrap_die(const char *fmt, ...){
    va_list ap;

    va_start(ap, fmt);
    fwrite(estrap_epstr, 1, strlen(estrap_epstr), PRINT_HANDLE);
    vfprintf(PRINT_HANDLE, fmt, ap);
    va_end(ap);

    exit(1);
}

