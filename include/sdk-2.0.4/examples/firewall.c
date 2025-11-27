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
#include "estrap.h"

int main(int argc, char *argv[]){
    struct cerial   *cer;
    uint16_t        status;
    unsigned int    n, nrules;
    char            rule[64];

    if((cer = estrap_in(argc, argv)) == NULL){
        return 1;
    }

    /* install a firewall for port 80 and 4000 access only */
    if(cpi_ptcmd(cer, &status, OP_NET_FIREWALL_FLUSH)){
        die("NFF failed\n");
    }
    if(cpi_ptcmd(cer, &status, OP_NET_FIREWALL_PUSH, "PAT80") ||
            cpi_ptcmd(cer, &status, OP_NET_FIREWALL_PUSH, "PAT4000")){
        die("NFU failed\n");
    }
    if(cpi_ptcmd(cer, &status, OP_NET_FIREWALL_APPLY)){
        die("NFA failed\n");
    }

    /* read back the rules */
    iprintf("Installed firewall rules:\n");
    if(cpi_ptcmd(cer, &status, OP_NET_FIREWALL_COUNT, &nrules)){
        die("NFC failed\n");
    }
    for(n = 0; n < nrules; n++){
        if(cpi_ptcmd(cer, &status, OP_NET_FIREWALL_INDEX_GET, n,
                    (int)sizeof(rule), NULL, rule)){
            die("NFI failed\n");
        }
        iprintf("Rule #%d: %s\n", n, rule);
    }

    estrap_out(cer);
    return 0;
}

