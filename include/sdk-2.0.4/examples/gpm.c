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
#include <string.h>
#include "estrap.h"

/* dPan,dTilt in degrees, dAlt in meters */
typedef struct {
  double dLat;
  double dLon;
  double dAlt;
  double dPan;
  double dTilt;
} llapt_t;

/* test landmarks, and the theoretical angles to point to them */
#define NUM_LANDMARKS 4
llapt_t psLandmarks[NUM_LANDMARKS] = {
  { .01,     0,    0,               0,     -.005002208 },
  { .01,  -.01,  450,  -45.1921788453,   15.9953101503 },
  { -.01, -.01, -333,   -134.807798833, -11.9895116501 },
  { 0,     .01,  300,   90.0000025328,   15.0772371287 }
};

/* blocking go-to-position */
int ptu_go_to(struct cerial *cer, int pan, int tilt){
    uint16_t status;
    int rc;

    if((rc = cpi_ptcmd(cer, &status, OP_PAN_DESIRED_POS_SET, pan)) ||
       (rc = cpi_ptcmd(cer, &status, OP_TILT_DESIRED_POS_SET, tilt))){
      return rc;
    }
    if((rc = cpi_block_until(cer, NULL, NULL, OP_PAN_CURRENT_POS_GET, pan)) ||
            cpi_block_until(cer, NULL, NULL, OP_TILT_CURRENT_POS_GET, tilt)){
      return rc;
    }
    return CPI_ERR_OK;
}


static int print_landmark(cerialh cer, int idx){
    uint16_t status;
    int rc, ridx;
    char name[32];
    double lat, lng, alt;
    int panpos, tiltpos;
    double pointing_error;

    rc = cpi_ptcmd(cer, &status, OP_GEO_LAND_IDX_GET, idx,
            &ridx,
            32, NULL, name,
            &lat, &lng, &alt,
            &panpos, &tiltpos,
            &pointing_error);
    if(rc == CPI_ERR_OK){
        iprintf("{%d} %s %f %f %0.1f meters @ (%d,%d) %f%%\n",
                ridx, name, lat, lng, alt, panpos, tiltpos, pointing_error);
    } else {
        estrap_die("Landmark get failed (%d) with %s\n",
                rc, cpi_strerror(rc));
    }

    return rc;
}


int main(int argc, char *argv[]){
    struct cerial   *psCer;
    uint16_t        uwStatus;
    int             iPan, iTilt;
    double          dPanRes, dTiltRes;
    int             i;
    char            pcName[16];
    double          dCalQual;
    int             iStatus;
    int             iTimeout = 16000;
    double          dLat = 0, dLon = 0, dAlt = 0;
    double          dRoll, dPitch, dYaw;
    int             iPanMaxSpeed, iTiltMaxSpeed;

    iprintf("This example shows how to use basic GPM functions.\n");

    /* initialize the serial interface */
    if((psCer = estrap_in(argc, argv)) == NULL){
      return 1;
    }

    /* GPM Calibration takes a while, so we are going to set the timeout higher */
    cerioctl(psCer, CERIAL_IOCTL_TIMEOUT_SET, &iTimeout);

    /* clear out any old landmarks */
    if(cpi_ptcmd(psCer, &uwStatus, OP_GEO_LAND_CLEAR)){
      estrap_die("OP_GEO_LAND_CLEAR failed\n");
    }
    
    /* get the ptu resolution so we can convert the angles to
       ptu positions */
    if((iStatus = cpi_ptcmd(psCer, &uwStatus,
			    OP_PAN_RESOLUTION,
			    &dPanRes)) ||
       (iStatus = cpi_ptcmd(psCer, &uwStatus,
			    OP_TILT_RESOLUTION,
			    &dTiltRes))) {
      estrap_die("Failed to get resolution (%d) %s\n", iStatus,
		 cpi_strerror((iStatus)));
    }

    /* resolutions are in arc-seconds, but we want degrees */
    dPanRes /= 3600;
    dTiltRes /= 3600;

    /* Set our position */
    if((iStatus = cpi_ptcmd(psCer, &uwStatus,
			    OP_GEO_LAT_SET,
			    dLat)) || 
       (iStatus = cpi_ptcmd(psCer, &uwStatus,
			    OP_GEO_LON_SET,
			    dLon)) ||
       (iStatus = cpi_ptcmd(psCer, &uwStatus,
			    OP_GEO_ALT_SET,
			    dAlt))) {
      estrap_die("Failed to Set Pos (%d) with %s\n",
		 iStatus,
		 cpi_strerror(iStatus));
    } else {
      iprintf("POS  Lat: %f  Lon: %f  Alt: %f\n", dLat, dLon, dAlt);
    }

    /* Set our speed to max so this doesn't take forever */
    if((iStatus = cpi_ptcmd(psCer, &uwStatus, 
			    OP_PAN_UPPER_SPEED_LIMIT_GET,
			    &iPanMaxSpeed)) ||
       (iStatus = cpi_ptcmd(psCer, &uwStatus,
			    OP_TILT_UPPER_SPEED_LIMIT_GET,
			    &iTiltMaxSpeed))||
       (iStatus = cpi_ptcmd(psCer, &uwStatus, 
			    OP_PAN_DESIRED_SPEED_SET,
			    iPanMaxSpeed)) ||
       (iStatus = cpi_ptcmd(psCer, &uwStatus,
			    OP_TILT_DESIRED_SPEED_SET,
			    iTiltMaxSpeed))) {
      estrap_die("Failed to Speed (%d) with %s\n",
		 iStatus,
		 cpi_strerror(iStatus));
    }

    
    iprintf("Create some landmarks for calibration\n");
    for(i = 0; i < NUM_LANDMARKS; i++) {
      iPan = psLandmarks[i].dPan / dPanRes;
      iTilt = psLandmarks[i].dTilt / dTiltRes;

      /* landmark name */
      snprintf(pcName, 16, "LM%d", i);

      iprintf("Pointing to %s\n", pcName);

      if((iStatus = ptu_go_to(psCer, iPan, iTilt))) {
	  estrap_die("Failed to go to (%d, %d) with (%d) %s\n", 
		     iPan,
		     iTilt,
		     iStatus,
		     cpi_strerror(iStatus));
	}

      /* set the landmark */
      if((iStatus = cpi_ptcmd(psCer, &uwStatus,
			      OP_GEO_LAND_ADD, 
			      pcName,
			      psLandmarks[i].dLat,
			      psLandmarks[i].dLon,
			      psLandmarks[i].dAlt))){
	estrap_die("OP_GEO_LAND_ADD failed (%d) with %s\n", 
	    iStatus, 
	    cpi_strerror(iStatus));
      }
    
    }

    /* print out the landmarks */
    iprintf("Landmarks:\n");
    for(i = 0; i < NUM_LANDMARKS; i++) {
        print_landmark(psCer, i);
    }

    /* calibrate */
    iprintf("Calibrating\n");
    if((iStatus = cpi_ptcmd(psCer, &uwStatus,
			    OP_GEO_CAL))) {
      estrap_die("Cal failed (%d) with %s\n", 
		 iStatus, 
		 cpi_strerror(iStatus));    
    }

    /* Get Cal quality */
    if((iStatus = cpi_ptcmd(psCer, &uwStatus,
			    OP_GEO_CAL_QUALITY_GET, &dCalQual))) {
      estrap_die("Cal Qual Get failed (%d) with %s\n", 
		 iStatus, 
		 cpi_strerror(iStatus));
    } else {
      iprintf("Quality: %.9f\n", dCalQual);
    }

    /* Get the calibrated pose  */
    if((iStatus = cpi_ptcmd(psCer, &uwStatus,
			    OP_GEO_PTU_ROLL_GET,
			    &dRoll)) || 
       (iStatus = cpi_ptcmd(psCer, &uwStatus,
			    OP_GEO_PTU_PITCH_GET,
			    &dPitch)) ||
       (iStatus = cpi_ptcmd(psCer, &uwStatus,
			    OP_GEO_PTU_YAW_GET,
			    &dYaw))) {
      estrap_die("Failed to get Pose (%d) with %s\n",
		 iStatus,
		 cpi_strerror(iStatus));
    } else {
      iprintf("POSE  Roll: %f  Pitch: %f  Yaw: %f\n", dRoll, dPitch, dYaw);
    }

    /* Show landmarks again with quality */
    iprintf("Landmarks now have error filled in:\n");
    for(i = 0; i < NUM_LANDMARKS; i++) {
        print_landmark(psCer, i);
    }

    /* point to a generic location */
    iprintf("Now pointing to (.015, .011, 300)\n");
    if((iStatus = cpi_ptcmd(psCer, &uwStatus,
			    OP_GEO_POINT_LLA,
			    .015,
			    .011,
			    300.0))) {
      estrap_die("Pointing failed with (%d) %s\n", 
		 iStatus, 
		 cpi_strerror(iStatus));
      
    }
	
    estrap_out(psCer);
    return 0;
}

