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
#ifndef CPI_CPI_H
#define CPI_CPI_H

#include <stdint.h>
#include <stdarg.h>
#include "cerial.h"
#include "opcodes.h"
#include "enums.h"
#include "error.h"

/**
 * \file cpi.h
 */

/** version major (a in a.x.x) */
#define CPI_VER_MAJOR                   2
/** version minor (a in x.a.x) */
#define CPI_VER_MINOR                   0
/** version revision (a in x.x.a) */
#define CPI_VER_REV                     4

/**
 * \brief version encoding
 * \details
 *
 * The SDK's version number can be represented as a single integer that can be
 * compared with a single comparison. This macro encodes an integer such that
 * newer versions are a larger number.
 *
 * \ref CPI_VERSION is encoded with this macro.
 *
 * Example usage:
 * \code
 * if(CPI_VERSION < CPI_VER_ENCODE(2.1.3)){
 *    printf("This code requires FLIR MCS SDK v2.1.3 or later.\n");
 *    exit(1);
 * }
 * \endcode
 */
#define CPI_VER_ENCODE(ma, mi, rev)     (((ma) << 16) | ((mi) << 5) | (rev))

/** SDK version (integer encoded) */
#define CPI_VERSION \
    CPI_VER_ENCODE(CPI_VER_MAJOR, CPI_VER_MINOR, CPI_VER_REV)

/** \cond */
#define CPI_STR(x)                  #x
#define CPI_XSTR(x)                 CPI_STR(x)
/** \endcond */
/** CPI_VERSION as a string literal */
#define CPI_VERSION_STR \
    CPI_XSTR(CPI_VER_MAJOR) "." \
    CPI_XSTR(CPI_VER_MINOR) "." \
    CPI_XSTR(CPI_VER_REV)


/* SDK function prototypes */
int cpi_vpack(void *buf, int count, uint8_t echo, cpi_opcode op, va_list *ap);
int cpi_vunpack(void *buf, int count, uint8_t *echo, uint16_t *status,
        cpi_opcode op, va_list *ap);
int cpi_pack(void *buf, int count, uint8_t echo, cpi_opcode op, ...);
int cpi_unpack(void *buf, int count, uint8_t *echo, uint16_t *status,
        cpi_opcode op, ...);

int cpi_vsend(cerialh cer, uint8_t echo, cpi_opcode op, va_list *ap);
int cpi_vrecv(cerialh cer, uint8_t *echo, uint16_t *status, cpi_opcode op,
        va_list *ap);
int cpi_send(cerialh cer, uint8_t echo, cpi_opcode op, ...);
int cpi_recv(cerialh cer, uint8_t *echo, uint16_t *status, cpi_opcode op, ...);

int cpi_resync(cerialh cer);
int cpi_ptcmd(cerialh cer, uint16_t *status, cpi_opcode op, ...);
int cpi_block_until(cerialh cer,
        int (*yield)(void *), void *yparam,
        cpi_opcode op, int value);
int cpi_ch_write(cerialh cer, int channel, void *buf, int len);
int cpi_ch_read(cerialh cer, int channel, void *buf, int len);
char *cpi_strerror(int error);
int cpi_version(void);
const char *cpi_version_str(void);

#endif /* CPI_CPI_H */

