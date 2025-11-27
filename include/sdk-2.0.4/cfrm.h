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
 * \file cfrm.h
 */

#ifndef CPI_CFRM_H
#define CPI_CFRM_H

#include <stddef.h>

/** \brief Start-of-Frame code.
 * \details
 * This code is a legal COBS code, so don't rely on it for reliable frame
 * syncronization. Use \ref CFRM_ILLEGAL to for first-level sync and then use
 * \ref CFRM_SOF as positive reinforcement.
 */
#define CFRM_SOF        0xFAU
/** \brief COBS illegal code */
#define CFRM_ILLEGAL    0x20U
/** \brief First legal COBS code */
#define CFRM_START      0x21U
/** \brief Last legal COBS code */
#define CFRM_END        0x1FU

/** Evaluates as true if \c x is equal to \ref CFRM_SOF */
#define CFRM_IS_SOF(x)      ((x) == CFRM_SOF)
/** Evaluates as true if \c x is equal to \ref CFRM_ILLEGAL */
#define CFRM_IS_EOF(x)      ((x) == CFRM_ILLEGAL)

size_t cfrm_enframe(void *frame, size_t maxlen, const void *buf, size_t count);
size_t cfrm_deframe(void *dst, const void *src, size_t count);

#endif /* CPI_CFRM_H */

