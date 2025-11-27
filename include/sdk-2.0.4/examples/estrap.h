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
#ifndef ESTRAP_H
#define ESTRAP_H

/** \file estrap.h */

/* include the core CPI components */
#include "cpi.h"

struct cerial *estrap_in(int argc, char *argv[]);
void estrap_out(struct cerial *cer);

/* error, warning, and info printf() equivalents
 * all write to stdout by default (see estrap.c to redirect)
 */
extern const char *estrap_epstr, *estrap_wpstr, *estrap_ipstr;
int estrap_printf(const char *prefix, const char *fmt, ...);

/** \brief error printf() */
#define eprintf(...) estrap_printf(estrap_epstr, ##__VA_ARGS__)
/** \brief warning printf() */
#define wprintf(...) estrap_printf(estrap_wpstr, ##__VA_ARGS__)
/** \brief information printf() */
#define iprintf(...) estrap_printf(estrap_ipstr, ##__VA_ARGS__)

void estrap_die(const char *fmt, ...);
/** \brief shorthand for estrap_die() */
#define die(fmt, ...)       estrap_die(fmt, ##__VA_ARGS__)

#endif /* ESTRAP_H */

