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

#ifndef CPI_ENUMS_H
#define CPI_ENUMS_H

/**
 * \file enums.h
 * \brief Common enumerations and defines.
 *
 * All enumerations are wire encoded as integers. Because the C specification
 * allows the compiler to decide if an enum is representated as a char or as an
 * int, all enumerations must be explicitly cast to int. For clarity's sake,
 * cpi_enum is a typedef to int.
 *
 * Example:
 * \code
 * cpi_ptcmd(cer, &status, OP_EXEC_MODE_SET, (cpi_enum)CPI_IMMEDIATE_MODE);
 * \endcode
 */

typedef int cpi_enum;
enum cpi_mpower {
    CPI_POWER_HIGH, CPI_POWER_REGULAR, CPI_POWER_LOW, CPI_POWER_OFF
};
enum cpi_enable {
    CPI_DISABLE, CPI_ENABLE
};
// enum cpi_stepmode {
//     CPI_STEP_FULL, CPI_STEP_HALF, CPI_STEP_QUARTER, CPI_STEP_AUTO
// };
enum cpi_channel {
    CPI_HOST, CPI_CHA, CPI_CHB
};

/* control mode */
enum cpi_control { CPI_CONTROL_INDEPENDENT = 1, CPI_CONTROL_PURE_VELOCITY };
/* feedback verbose/terse */
enum cpi_feedback { CPI_ASCII_FEEDBACK_TERSE, CPI_ASCII_FEEDBACK_VERBOSE };
/* limits type */
enum cpi_limits { CPI_LIMITS_FACTORY, CPI_LIMITS_USER, CPI_LIMITS_DISABLED };
/* IP Mode */
enum cpi_ip_mode { CPI_NET_IP_STATIC, CPI_NET_IP_DYNAMIC };
/* OPIO_OUT values */
enum cpi_opio_out { CPI_OPIO_OUT_LOW, CPI_OPIO_OUT_HIGH };
/* reset type */
typedef enum _cpi_reset_type {
    CPI_RESET_DEFAULT, CPI_RESET_NONE, CPI_RESET_ALL, CPI_RESET_PAN,
    CPI_RESET_TILT
}cpi_reset_type;

typedef enum cpi_stepmode {
    CPI_STEP_FULL, CPI_STEP_HALF, CPI_STEP_QUARTER, CPI_STEP_AUTO
} cpi_stepmode;

/* execution types */
enum cpi_execution_type { CPI_IMMEDIATE_MODE, CPI_SLAVE_MODE };
/* Halts */
enum cpi_halt {
    CPI_HALT_ALL,
    CPI_HALT_PAN,
    CPI_HALT_TILT
};
/* serial config */
enum cpi_parity { CPI_PARITY_EVEN, CPI_PARITY_NONE, CPI_PARITY_ODD };
enum cpi_host_startup { CPI_HOST_STARTUP_OFF, CPI_HOST_STARTUP_ON };
enum cpi_handshake {
    CPI_HANDSHAKE_FULL, CPI_HANDSHAKE_NONE, CPI_HANDSHAKE_HW, CPI_HANDSHAKE_SW
};

/* these defines are for ISM_STAB_ENABLE */
/* bit-mapped axes */
#define CPI_ISM_PAN                 0x01
#define CPI_ISM_TILT                0x02
/* flags */
enum cpi_ism_mode {
    CPI_ISM_MODE_DEFAULT,
    CPI_ISM_MODE_VERTICAL,
    CPI_ISM_MODE_INIT_ZERO
};

/* CPI_Monitor status */
#define CPI_MONITOR_EVENT_NO_START  0x01
#define CPI_MONITOR_EVENT_START     0x02
#define CPI_MONITOR_EVENT_STOP      0x04
#define CPI_MONITOR_EVENT_CH_INT    0x08
#define CPI_MONITOR_EVENT_ERROR     0x10
#define CPI_MONITOR_EVENT_FAULT     0x20

/* GPM Status */
enum cpi_gpm_status {
    CPI_GPM_NONE,
    CPI_GPM_UNINITIALIZED,
    CPI_GPM_READY,
    CPI_GPM_CALIBRATING,
    CPI_GPM_ERROR,
    CPI_GPM_CALIBRATION_FAILED
};

/* GPM pointing modes */
enum cpi_gpm_point_mode_t {
    CPI_GPM_POINT_LEGAL_ONLY,
    CPI_GPM_POINT_CLOSEST,
    CPI_GPM_POINT_NUM_MODES
};

/* ISM Status */
enum cpi_ism_status {
    CPI_ISM_STATUS_NONE,
    CPI_ISM_STATUS_READY,
    CPI_ISM_STATUS_RUN,
    CPI_ISM_STATUS_CALIBRATING,
    CPI_ISM_STATUS_ERROR,
    CPI_ISM_STATUS_CALIBRATION_FAILED,
    CPI_ISM_STATUS_BUSY
};

/* ISM cmd modes */
enum cpi_ism_cmd_mode {
    CPI_ISM_CMD_MODE_PTU,
    CPI_ISM_CMD_MODE_STAB,
    CPI_ISM_CMD_MODE_HEADING
};

enum cpi_ism_los_mode {
    CPI_ISM_LOS_CLOSEST,
    CPI_ISM_LOS_IN_BOUNDS,
    CPI_ISM_LOS_NUM_MODES
};

enum cpi_ism_gyro_firmware_status {
    CPI_ISM_GYRO_FIRMWARE_NONE,
    CPI_ISM_GYRO_FIRMWARE_OK,
    CPI_ISM_GYRO_FIRMWARE_UPGRADE,
    CPI_ISM_GYRO_FIRMWARE_NOT_SUPPORTED
};

#endif /* CPI_ENUMS_H */

