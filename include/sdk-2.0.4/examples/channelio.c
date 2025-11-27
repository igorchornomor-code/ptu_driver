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
#include <stdio.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "estrap.h"


static void print_hex(const char *name, unsigned char *data, int len){
    unsigned char   *end = data + len;
    int             i;

    iprintf("read(%s):", name);
    i = 0;
    while(data < end){
        printf(" %02X (%c)", *data, isprint(*data) ? *data : '.');
        i++;
        data++;
        if((i % 8) == 0){
            printf("\n");
            iprintf("read(%s):", name);
        }
    }
    printf("\n");
}


static void read_channel(struct cerial *cer, cpi_enum channel){
    unsigned char   buf[32];
    int             count;

    if((count = cpi_ch_read(cer, channel, buf, 32)) < 0){
        die("cpi_ch_write(): %s\n", cpi_strerror(count));
    }
    print_hex(channel == CPI_CHA ? "CHA" : "CHB", buf, count);
}


int main(int argc, char *argv[]){
    struct cerial   *cer;
    char            *hello = "Hello World!";
    int             count;

    if((cer = estrap_in(argc, argv)) == NULL){
        return 1;
    }

    /* write hello to CHA */
    if((count = cpi_ch_write(cer, CPI_CHA, hello, (int)strlen(hello))) < 0){
        die("cpi_ch_write(): %s\n", cpi_strerror(count));
    }
    iprintf("Wrote %d bytes of \"%s\" to CHA.\n", count, hello);

    /* read back some data on CHA and CHB */
    read_channel(cer, CPI_CHA);
    read_channel(cer, CPI_CHB);

    estrap_out(cer);
    return 0;
}

