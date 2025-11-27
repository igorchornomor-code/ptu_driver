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
    unsigned int    serial_num;
    char            verstr[64], modelstr[64];

    if((cer = estrap_in(argc, argv)) == NULL){
        return 1;
    }

    if(cpi_ptcmd(cer, &status, OP_SW_VERSION_SHORT_GET,
                (int)sizeof(verstr), NULL, verstr)){
        die("version failed\n");
    }
    if(cpi_ptcmd(cer, &status, OP_MODEL_GET,
                (int)sizeof(modelstr), NULL, modelstr)){
        die("model failed\n");
    }
    if(cpi_ptcmd(cer, &status, OP_SERIAL_GET, &serial_num)){
        die("serial failed\n");
    }

    iprintf("SDK v%s (compile-time query)\n", CPI_VERSION_STR);
    iprintf("SDK v%s (run-time query)\n", cpi_version_str());
    iprintf("%s #%u running v%s\n", modelstr, serial_num, verstr);

    estrap_out(cer);
    return 0;
}

