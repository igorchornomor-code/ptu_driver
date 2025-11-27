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
 * \file cerial.h
 */

#ifndef CERIAL_H
#define CERIAL_H

#include <stddef.h>


/** No flags */
#define CERIAL_FLAGS_NONE               0


/**
 * \page cerial_ioctls Cerial IOCTL Requests
 *
 * These a few standard Cerial ioctl requests. They are cover the most basic
 * control found in the standard SDK distribution. Custom requests are allowed
 * and should be at or above \ref CERIAL_IOCTL_USER_RESERVED.
 *
 * \section ci_General General
 *
 * \li \ref CERIAL_IOCTL_TIMEOUT_SET
 * \li \ref CERIAL_IOCTL_FLUSH_INPUT
 *
 * \section ci_SerialRequests Serial Requests
 *
 * \li \ref CERIAL_IOCTL_BAUDRATE_GET
 * \li \ref CERIAL_IOCTL_BAUDRATE_SET
 *
 *
 */
/**
 * \def CERIAL_IOCTL_BAUDRATE_SET
 * \brief Set the baudrate of a serial device.
 * \details
 * The argument is a pointer to an integer specifying the baudrate.
 *
 * This ioctl is only valid for serial devices. Devices that do not support
 * baud rates should return an invalid request error.
 *
 * \def CERIAL_IOCTL_BAUDRATE_GET
 * \brief Get the baudrate of a serial device.
 * \details
 * The argument is a pointer to an integer that is filled with the current
 * baudrate on success.
 *
 * This ioctl is only valid for serial devices. Devices that do not support
 * baud rates should return an invalid request error.
 *
 * \def CERIAL_IOCTL_TIMEOUT_SET
 * \brief Set an I/O timeout.
 * \details
 * An I/O timeout specifies the maximum time an I/O operation should take. If
 * a timeout occurs, then the operation should abort with a partial read/write.
 *
 * The argument is a pointer to an integer specifying a timeout in milliseconds.
 *
 * \def CERIAL_IOCTL_FLUSH_INPUT
 * \brief Remove all received charcters from the input buffer(s).
 * \details
 * This requests clears the input buffer without reading it out.
 *
 * \def CERIAL_IOCTL_USER_RESERVED
 * \brief Smallest request number for user-defined requests. Requests below
 *        this is reserved for standard cerial requests.
 */
#define CERIAL_IOCTL_BAUDRATE_GET       0
#define CERIAL_IOCTL_BAUDRATE_SET       1
#define CERIAL_IOCTL_TIMEOUT_SET        2
#define CERIAL_IOCTL_FLUSH_INPUT        3
#define CERIAL_IOCTL_USER_RESERVED      32768


/** \struct cerial_driver
 *
 * This structure describes a complete driver. All members must be defined.
 *
 * Driver interaction is UNIX-like. open() opens a new device, read() and
 * write() perform basic I/O, ioctl() offers control without dedicated function
 * calls, and close() cleans up the device and associate resources.
 *
 * Functions that return integers will return < 0 in error. The error value is
 * driver dependent, but not time or instance dependent. That is, given a
 * specific error, a nonrandom error number is returned. Multiple error
 * conditions can result in the same error number (ex: all errors result in
 * -1).
 */
struct cerial_driver {
    /** Open a device.
     *
     * The first argument is a name to the device (ex: "//./COM1" or
     * "/dev/ttyS0").
     *
     * The second argument are bit-vector of flags. 0 sets no flags.
     *
     * \returns NULL on error or a valid device-sepcific "port." This may be a
     * pointer to a file descriptor, a Windows HANDLE, or some other internal
     * definition. This pointer is often stored in cerial.port.
     */
    void *(*open)(const char *, int);

    /** Read bytes from a device.
     *
     * The first argument is a valid port handle.
     *
     * The second argument is the buffer to read bytes into.
     *
     * The third argument is the maximum number of bytes to read.
     *
     * \returns the number of bytes read. This may be less than the number of
     * bytes requested and may often occur in non-blocking mode. Less than zero
     * (0) is returned on error.
     */
    int (*read)(void *, void *, int);

    /** Write bytes to a device.
     *
     * The first argument is a valid port handle.
     *
     * The second argument is the buffer to write bytes from.
     *
     * The third argument is the maximum number of bytes to write.
     *
     * \returns the number of bytes written. This may be less than the number
     * of bytes requested. Less than zero (0) is returned on error.
     */
    int (*write)(void *, const void *, int);

    /** I/O Control
     *
     * The first argument is a valid port handle.
     *
     * The second argument is the device request.
     *
     * The last argument is a device and request dependent argument and may
     * not be required for every request.
     *
     * \returns zero (0) on success.
     */
    int (*ioctl)(void *, int, void *);

    /** Close a device.
     *
     * The only argument is a valid port handle.
     *
     * \returns zero (0) on success.
     */
    int (*close)(void *);

    /** Get a string description of the last error.
     * Call this function to get a description of the last recorded error.
     *
     * There is no standard error numbers or strings. Different implementations
     * may return different error messages. This call is to help users identify
     * problems, so be descriptive.
     *
     * Calling this function is thread specific. It is intended to be a common
     * interface to POSIX errno/strerror() and Win32's GetLastError(). Other
     * platforms may build their own error system using a thread-local
     * variable or by other means. Calling other cerial functions may change
     * the last-saved error (although clearing to "OK" is bad practice).
     *
     * The first argument is a character buffer to store the error message.
     * The second argument is the size of the buffer.
     *
     * A null character is guaranteed to be written into this buffer if
     * size >= 1. The maximum message length therefore is size - 1.
     *
     * \returns the first argument. This function must not return NULL.
     *
     * Example usage:
     *
     * \code
     * if(cerial->open(...)){
     *     printf("Error: %s\n", cerial->strerror(bufarray, sizeof(bufarray)));
     * }
     * \endcode
     */
    char *(*strerror)(char *, size_t);
};

/** \struct cerial
 * struct cerial is a complete handle to a driver and the port returned
 * from drv->open().
 */
struct cerial {
    void                        *port;  /**< driver-specific port handle */
    const struct cerial_driver  *drv;   /**< driver's interface */
};
typedef struct cerial * cerialh;

/* A basic driver wrapper. generally expands to cerh->drv(cerh->port, ...). */
int ceropen(cerialh, const char *, int);
int cerread(cerialh, void *, int);
int cerwrite(cerialh, const void *, int);
int cerioctl(cerialh, int, void *);
int cerclose(cerialh);
const char *cerstrerror(cerialh, char *, size_t);
/* utility functions */
int cerputc(cerialh, int);
int cergetc(cerialh);
unsigned int cerblockwrite(cerialh, void *, int);

#endif /* CERIAL_H */

