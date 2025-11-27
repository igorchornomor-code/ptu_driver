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
#include <math.h>
#include <unistd.h>
#include "estrap.h"

#define PI 3.14159
#define US_TIMEOUT 10000
#define CMD_FREQUENCY (1000000 / US_TIMEOUT)
#define CYCLES 5
#define AMPLITUDE 500
#define FREQUENCY .5
#define GAIN      30

typedef struct
{
  double dFrequency;
  unsigned int uiAmp;
  double dGain;
  unsigned int uiMaxSpeed;
  int iInitPos;
} sine_t;

/* protos */
int ptu_next_cmd(struct cerial *cer, sine_t *psSine);

int main(int argc, char *argv[]){
    struct cerial   *psCer;
    sine_t sSine;
    int iParm = 0;
    uint16_t status;

    printf("This example will run a sine wave on the pan axis for "
            "%d cycles.\n", CYCLES);
    printf("Amplitude: %d, Frequency: %f\n", AMPLITUDE, FREQUENCY);

    if((psCer = estrap_in(argc, argv)) == NULL){
      return 1;
    }

    /* Sine wave frequency */
    sSine.dFrequency = FREQUENCY;
    /* Sine wave amplitude in PTU positions */
    sSine.uiAmp = AMPLITUDE;
    /* Gain for P controller */
    sSine.dGain = GAIN;

    /* set the PTU to pure velocity mode */
    if(cpi_ptcmd(psCer, &status, OP_SPEED_CONTROL_MODE_SET,
                (cpi_enum)CPI_CONTROL_PURE_VELOCITY)) {
      estrap_die("Unable to set control mode\n");
    }

    /* Get the initial position */
    if(cpi_ptcmd(psCer, &status, OP_PAN_CURRENT_POS_GET, &sSine.iInitPos)) {
      estrap_die("Unable to get position\n");
    }

    /* get the maximum speed */
    if(cpi_ptcmd(psCer, &status, OP_PAN_UPPER_SPEED_LIMIT_GET,
                &sSine.uiMaxSpeed)) {
      estrap_die("Unable to get max speed\n");
    }

    /* Loop */
    while(ptu_next_cmd(psCer, &sSine)) {
      if(usleep(US_TIMEOUT)) {
        printf("USLEEP FAILED\n");
      }
    }

    /* set our speed back to 0 */
    iParm = 0;
    if(cpi_ptcmd(psCer, &status, OP_PAN_DESIRED_SPEED_SET, iParm)) {
      estrap_die("Unable to set speed\n");
    }

    estrap_out(psCer);
    return 0;
}

int ptu_next_cmd(struct cerial *psCer, sine_t *psSine) {
  uint16_t status;
  int iPos;
  int iNextSpeed, iDesSpeed;
  int iDesPos;
  static int iTick = 0;

  if(cpi_ptcmd(psCer, &status, OP_PAN_CURRENT_POS_GET, &iPos)) {
    estrap_die("Unable to query position\n");
  }

  /* what speed should we be moving this tick */
  iDesSpeed = psSine->uiAmp * 2 * PI / (CMD_FREQUENCY / psSine->dFrequency) *
    cos(2 * PI * iTick / (CMD_FREQUENCY / psSine->dFrequency)) * CMD_FREQUENCY;
  /* Where should we be this tick */
  iDesPos = psSine->uiAmp *
      sin(2 * PI * iTick / (CMD_FREQUENCY / psSine->dFrequency)) +
      psSine->iInitPos;

  /* Use a simple feed forward + proportional
     controller to keep the position on track */
  iNextSpeed = iDesSpeed + (iDesPos - iPos) * psSine->dGain;

  /* deadband low speeds so the unit stays responsive */
  if(fabs(iNextSpeed) < CMD_FREQUENCY) {
    iNextSpeed = 0;
  }

  if(fabs(iNextSpeed) > psSine->uiMaxSpeed) {
    if(iNextSpeed > 0){
      iNextSpeed = psSine->uiMaxSpeed;
    } else {
      iNextSpeed = -psSine->uiMaxSpeed;
    }
  }

  /* Set the speed */
  if(cpi_ptcmd(psCer, &status, OP_PAN_DESIRED_SPEED_SET, iNextSpeed)) {
    estrap_die("Unable to set speed %d\n", (int)status);
  }


  iTick++;
  /* Reset tick counter after CYCLES cycles */
  if(iTick == (CMD_FREQUENCY / psSine->dFrequency) * CYCLES) {
    iTick = 0;
  }

  return iTick;
}
