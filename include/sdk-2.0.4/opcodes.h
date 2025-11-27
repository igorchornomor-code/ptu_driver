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
#ifndef CPI_OPCODE_H
#define CPI_OPCODE_H

/**
 * \file opcodes.h
 */

/* opcode enum */
#include "openum.h"
typedef enum cpi_opcode cpi_opcode;

/* Basic wire coding types. */
/** Integer specification */
#define CPI_T_INT       'i'
/** Unsigned integer specification */
#define CPI_T_UNSIGNED  'u'
/** Double (floating point) specification */
#define CPI_T_DOUBLE    'd'
/** String specification */
#define CPI_T_STR       's'
/** Enumerated type specification
 * This is wire coded as an integer (CPI_T_INT).
 */
#define CPI_T_ENUM      'e'
/** Binary string specification
 * This is wire coded as n string (CPI_T_STR).
 */
#define CPI_T_BINARY    'b'

/* Status bits */
/** An error occured. */
#define CPI_STATUS_ERR      1

/* Opcode lookup table.
 */
struct cpi_opent {
    cpi_opcode op;  /**< opcode */
    char *retspec;  /**< function return specification */
    char *argspec;  /**< function argument specification */
};
extern struct cpi_opent cpi_optable[];

#ifdef CPI_CONFIG_USE_OP_LKUP
/* search based lookup */
struct cpi_opent *cpi_op_lkup(cpi_opcode op);
#else
/* direct lookup */
#define cpi_op_lkup(op) (&cpi_optable[op])
#endif

/* Opcode by string lookup table. */
cpi_opcode cpi_opname_search(const char *);

#endif /* CPI_OPCODES_H */

