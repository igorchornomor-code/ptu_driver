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
#include <time.h>
#include "estrap.h"

int main(int argc, char *argv[]){
    struct cerial   *cer;
    uint16_t        status;
    uint8_t         echo;
    int             rc, cmdcnt, pos;
    time_t          start;

    if((cer = estrap_in(argc, argv)) == NULL){
        return 1;
    }

    /* 10 second test with cpi_ptcmd() */
    iprintf("10 second poll with cpi_ptcmd...\n");
    for(start = time(NULL), cmdcnt = 0; (time(NULL) - start) < 10; cmdcnt++){
        if((rc = cpi_ptcmd(cer, &status, OP_PAN_CURRENT_POS_GET, &pos))){
            die("cpi_ptcmd failed: %s\n", cpi_strerror(rc));
        }
    }
    iprintf("%d commands executed (%.1f/sec)\n", cmdcnt, cmdcnt / 10.0);

    /* 10 second test with piped commands (split cpi_send()/cpi_recv()) */
    iprintf("10 second poll with cpi_send/cpi_recv...\n");
    for(start = time(NULL), cmdcnt = 0; (time(NULL) - start) < 10; cmdcnt++){
        if((rc = cpi_send(cer, 0x00, OP_PAN_CURRENT_POS_GET))){
            die("cpi_send failed: %s\n", cpi_strerror(rc));
        }
        if(cmdcnt > 0){
            /* collect previous command's response */
            if((rc = cpi_recv(cer, &echo, &status,
                            OP_PAN_CURRENT_POS_GET, &pos))){
                die("cpi_recv failed: %s\n", cpi_strerror(rc));
            }
        }
    }
    /* last response */
    if((rc = cpi_recv(cer, &echo, &status,
                    OP_PAN_CURRENT_POS_GET, &pos))){
        die("cpi_recv failed: %s\n", cpi_strerror(rc));
    }
    iprintf("%d commands executed (%.1f/sec)\n", cmdcnt, cmdcnt / 10.0);

    estrap_out(cer);
    return 0;
}

