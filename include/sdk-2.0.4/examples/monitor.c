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
#define NR_ITERATIONS	2
#define PAN_HOME		0
#define TILT_HOME		0

/* blocking go-to-position */
void ptu_go_to(struct cerial *cer, int pan, int tilt){
    uint16_t status;

    if(cpi_ptcmd(cer, &status, OP_PAN_DESIRED_POS_SET, pan) ||
            cpi_ptcmd(cer, &status, OP_TILT_DESIRED_POS_SET, tilt))
	{
        die("Failed to go to min position.\n");
    }
    if(cpi_block_until(cer, NULL, NULL, OP_PAN_CURRENT_POS_GET, pan) ||
            cpi_block_until(cer, NULL, NULL, OP_TILT_CURRENT_POS_GET, tilt))
	{
        die("Blocking for reset completion failed.\n");
    }
}
/* Reset PTU All */
void ptu_reset(struct cerial *cer, cpi_reset_type type){
    uint16_t status;

    if (cpi_ptcmd(cer, &status, OP_RESET, type) )
	{
        die("Failed to execute Reset.\n");
    }
    if (cpi_block_until(cer, NULL, NULL, OP_PAN_CURRENT_POS_GET, PAN_HOME) ||
        cpi_block_until(cer, NULL, NULL, OP_TILT_CURRENT_POS_GET, TILT_HOME)){
        die("Blocking failed.\n");
    }

}

int main(int argc, char *argv[]){
    struct cerial   *cer;
    uint16_t        status;
    int             pn, px, tn, tx, pu, tu;
	int i = 0;

	if((cer = estrap_in(argc, argv)) == NULL){
        return 1;
    }
// Set terse mode
    if(cpi_ptcmd(cer, &status, OP_FEEDBACK_SET, CPI_ASCII_FEEDBACK_TERSE)){
        die("Failed to set feedback mode.\n");
    }

    /* read min/max position and speed */
    if(cpi_ptcmd(cer, &status, OP_PAN_MAX_POSITION, &px) ||
            cpi_ptcmd(cer, &status, OP_PAN_MIN_POSITION, &pn) ||
            cpi_ptcmd(cer, &status, OP_TILT_MAX_POSITION, &tx) ||
            cpi_ptcmd(cer, &status, OP_TILT_MIN_POSITION, &tn) ||
            cpi_ptcmd(cer, &status, OP_PAN_UPPER_SPEED_LIMIT_GET, &pu) ||
            cpi_ptcmd(cer, &status, OP_TILT_UPPER_SPEED_LIMIT_GET, &tu)){
        die("Basic unit queries failed.\n");
    }
	/* Test reset */
	ptu_reset(cer, CPI_RESET_ALL);

    /* set desired speed to half the upper speed */
    if(cpi_ptcmd(cer, &status, OP_PAN_DESIRED_SPEED_SET, pu / 2) ||
            cpi_ptcmd(cer, &status, OP_TILT_DESIRED_SPEED_SET, tu / 2)){
        die("Setting PS/TS failed.\n");
    }

    /* cycle twice between min and max positions */
	ptu_go_to(cer, pn, tn);
    ptu_go_to(cer, px, tx);
    ptu_go_to(cer, pn, tn);
    ptu_go_to(cer, px, tx);

/* cycle between pre-defined positions */
	pn = -5000;
	px = 5000;
	tn = -1000;
	tx = 1000;

	for ( i=0; i < NR_ITERATIONS; i++)
	{
		ptu_go_to(cer, pn, tn);
		ptu_go_to(cer, px, tx);
	}
    /* and go home */
    ptu_go_to(cer, 0, 0);

    estrap_out(cer);
    return 0;
}

