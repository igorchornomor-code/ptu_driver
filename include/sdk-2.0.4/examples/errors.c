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
#include "estrap.h"

static void printe(int err){
    iprintf("Test Error: %s: %s (%d)\n",
            CPI_EISPTU(err) ? "PTU" : "CPI",
            cpi_strerror(err),
            err);
}


int main(int argc, char *argv[]){
    /* verify error API */
    if(CPI_EMASK(CPI_EMAKEPTU(CPI_ERR_GENERAL)) != CPI_ERR_GENERAL){
        die("CPI_EMASK failed to mask PTU error type\n");
    }
    if(CPI_EISPTU(CPI_ERR_GENERAL)){
        die("CPI_EISPTU incorrectly identified CPI error as a PTU error\n");
    }
    if(!CPI_EISPTU((CPI_EMAKEPTU(CPI_ERR_GENERAL)))){
        die("CPI_EISPTU failed to identify PTU error\n");
    }

    /* a basic error */
    printe(CPI_ERR_GENERAL);
    /* PTU version of the same error */
    printe(CPI_EMAKEPTU(CPI_ERR_GENERAL));
    /* masked out PTU version of the error */
    printe(CPI_EMASK(CPI_EMAKEPTU(CPI_ERR_GENERAL)));
    /* and other errors */
    printe(CPI_ERR_NO_MEM);
    printe(CPI_ERR_UNREACH);
    printe(CPI_ERR_LOST_SYNC);

    iprintf("OK\n");
    return 0;
}

