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
 * \file cpi.c
 * \brief SDK interface
 */

#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "cpi.h"
#include "cfrm.h"

/**
 * \brief Maximum frame size.
 * \details
 * Frames are allocated on the stack, so don't make this excessively large.
 */
#define CPI_MAX_FRAME       128

#ifdef LITTLE_ENDIAN
/** Generic byte-swap
 * This is only necessary on little-endian architectures and only when larger
 * types can be pointed to by a character pointer.
 */
static void cpi_bswap(char *bp, int width){
    unsigned char   tmp;
    int             head, tail;

    /* this function works with multiple of 2 widths */
    assert(width % 2 == 0);

    for(head = 0, tail = (width - 1); head < tail; head++, tail--){
        tmp = bp[head];
        bp[head] = bp[tail];
        bp[tail] = tmp;
    }
}
#endif /* LITTLE_ENDIAN */


/** Copy generic memory into a length limited frame */
static int cpi_frmcpy_in(void *in, int width, void *frame, int *idx, int len){
    if((*idx + width) > len){
        return CPI_ERR_NO_MEM;
    }

    memcpy(((char *)frame) + *idx, in, width);
    *idx += width;

    return CPI_ERR_OK;
}


/** Copy generic memory out from a length limited frame */
static int cpi_frmcpy_out(void *out, int width, void *frame, int *idx, int len){
    if((*idx + width) > len){
        return CPI_ERR_NO_MEM; /* OOM */
    }

    memcpy(out, ((char *)frame) + *idx, width);
    *idx += width;

    return CPI_ERR_OK;
}


/** Encode multi-byte integers.
 * For little-endian architectures, this swaps the input data.
 * For big-endian architectures, this function mostly copies data and
 * advances pointers.
 */
static int cpi_encode(void *in, int width, void *frame, int *idx, int len){
#ifdef LITTLE_ENDIAN
    if(width > 1){
        cpi_bswap(in, width);
    }
#endif

    return cpi_frmcpy_in(in, width, frame, idx, len);
}


/** Inverse of cpi_encode. */
static int cpi_decode(void *out, int width, void *frame, int *idx, int len){
    int rc;

    rc = cpi_frmcpy_out(out, width, frame, idx, len);

#ifdef LITTLE_ENDIAN
    if(width > 1){
        cpi_bswap(out, width);
    }
#endif

    return rc;
}


/** (Re)Establish frame synchronization.
 *
 * Call this function in the event bytes were lost, are switching into binary
 * mode, or if you want to be reasonably sure the next frame will arrive
 * intact.
 *
 * This function sends a few end-of-frame bytes to clear out the ASCII and
 * binary parser. It then synchronizes commands and responses with OP_NOOP.
 *
 * \param cer   Cerial handle
 *
 * \returns an error code (CPI_ERR_OK on success).
 */
int cpi_resync(cerialh cer){
    static const char flush[8] = {
        CFRM_ILLEGAL, CFRM_ILLEGAL, CFRM_ILLEGAL, CFRM_ILLEGAL,
        CFRM_ILLEGAL, CFRM_ILLEGAL, CFRM_ILLEGAL, CFRM_ILLEGAL
    };
    int     rc, is_noop;
    uint8_t byte;

    is_noop = 0;

    /* flush PTU's input */
    if(cerwrite(cer, flush, sizeof(flush)) != sizeof(flush)){
        return CPI_ERR_WRITE;
    }

    /* Send a valid no-operation command
     * Use 0xF for echo because cpi_ptcmd uses, and checks for, 0x0 through 0xE.
     * This way the user can call cpi_ptcmd() and get an error if cpi_ptcmd() 
     * accidently picks up our OP_NOOP return frame.
     */
    if((rc = cpi_send(cer, 0x0F, OP_NOOP))){
        return rc;
    }

    /* resync our input the command's SOF, echo and EOF 
     * keep checking frams until we see our OP_NOOP frame 
     * if we missed that frame, data should be flushed and 
     * cerread will timeout and we return CPI_ERR_LOST_SYNC.
     */
    do { 
      while((rc = cerread(cer, &byte, 1)) == 1 && !CFRM_IS_SOF(byte)){}
      if(rc != 1){
        return CPI_ERR_LOST_SYNC;
      }
      // Get second byte of frame. This byte is meaningless.
      if(cerread(cer, &byte, 1) != 1){    
	return CPI_ERR_LOST_SYNC;
      }
      // Get echo byte
      if(cerread(cer, &byte, 1) != 1){
	return CPI_ERR_LOST_SYNC;
      }
      // Make sure it's the echo for our OP_NOOP
      if((byte & 0x0F) == 0x0F){
	is_noop = 1;  // This frame is our OP_NOOP
      }
      while((rc = cerread(cer, &byte, 1)) == 1 && !CFRM_IS_EOF(byte)){}
      if(rc != 1){
        return CPI_ERR_LOST_SYNC;
      }
    } while(is_noop == 0);

    return CPI_ERR_OK;
}


/** Pack an opcode and its arguments into buf.
 *
 * \c buf is encoded from an argument list of types according to \c op.
 *
 * \param buf       Buffer to write into
 * \param count     Maximum number of bytes to write (size of \c buf)
 * \param echo      Echo field
 * \param op        Opcode
 * \param ap        Argument list
 *
 * \returns number of bytes packed into buf or -1 on error.
 */
int cpi_vpack(void *buf, int count, uint8_t echo, cpi_opcode op, va_list *ap){
    int                 rc, fidx;
    const char          *spec;
    char                *str;
    int                 slen;
    struct cpi_opent    *ent;
    /* native types */
    int             i;
    unsigned int    ui;
    /* wire types */
    uint16_t    u16;
    int32_t     i32;
    uint32_t    u32;
    double      dbl;

    /* get op entry */
    if((ent = cpi_op_lkup(op)) == NULL){
        return CPI_ERR_INVAL;
    }

    /* setup frame pointer */
    fidx = 0;

    /* insert echo */
    if((rc = cpi_encode(&echo, sizeof(echo), buf, &fidx, count))){
        goto err;
    }

    /* insert opcode */
    u16 = (uint16_t)op;
    if((rc = cpi_encode(&u16, sizeof(u16), buf, &fidx, count))){
        goto err;
    }

    /* encode arguments */
    spec = ent->argspec;
    while(!rc && *spec){
        /* encode arg */
        switch(*spec){
        case CPI_T_INT:
        case CPI_T_ENUM:
            i = va_arg(*ap, int);
            i32 = (int32_t)i;
            rc = cpi_encode(&i32, sizeof(i32), buf, &fidx, count);
            break;
        case CPI_T_UNSIGNED:
            ui = va_arg(*ap, unsigned int);
            u32 = (uint32_t)ui;
            rc = cpi_encode(&u32, sizeof(u32), buf, &fidx, count);
            break;
        case CPI_T_DOUBLE:
            dbl = va_arg(*ap, double);
            rc = cpi_encode(&dbl, sizeof(dbl), buf, &fidx, count);
            break;
        case CPI_T_STR:
        case CPI_T_BINARY:
            if(*spec == CPI_T_BINARY){
                slen = va_arg(*ap, int);
                str = va_arg(*ap, char *);
            } else {
                str = va_arg(*ap, char *);
                slen = strlen(str);
            }
            if(slen > UINT16_MAX){
                /* string is too long to represent */
                rc = CPI_ERR_NO_MEM;
                break;
            }
            /* length */
            u16 = (uint16_t)slen;
            if((rc = cpi_encode(&u16, sizeof(u16), buf, &fidx, count))){
                break;
            }
            /* bytes (no encoding, just plain raw copy) */
            rc = cpi_frmcpy_in(str, (int)slen, buf, &fidx, count);
            break;
        default:
            /* unknown type */
            rc = CPI_ERR_INVAL;
        }
        /* next arg */
        spec++;
    }

    if(!rc){
        return fidx;
    }
err:
    return rc;
}


/** Unpack binary stream according to an opcode and its returns.
 *
 * \c buf is decoded into an argument list of types according to \c op.
 * \c status is updated with the unpacked status code.
 *
 * \param buf       Buffer to read from
 * \param count     Number of bytes in buf
 * \param echo      Echo field from PTU
 * \param status    Status code from PTU
 * \param op        Opcode to use for decoding
 * \param ap        Argument list to decode into
 *
 * \returns number of bytes unpacked from buf or < 0 on error.
 */
int cpi_vunpack(void *buf, int count, uint8_t *echo, uint16_t *status,
        cpi_opcode op, va_list *ap){
    int                 rc, fidx, maxlen, *plen;
    const char          *spec;
    uint16_t            u16;
    int32_t             i32;
    uint32_t            u32;
    double              dbl;
    char                *str;
    struct cpi_opent    *ent;

    /* get op entry */
    if((ent = cpi_op_lkup(op)) == NULL){
        return CPI_ERR_INVAL;
    }

    /* setup frame pointer */
    fidx = 0;

    /* decode
     * First three bytes are the echo byte and status code.
     */
    if((rc = cpi_decode(echo, sizeof(uint8_t), buf, &fidx, count))){
        goto err;
    }
    if((rc = cpi_decode(&u16, sizeof(u16), buf, &fidx, count))){
        goto err;
    }
    *status = u16;
    if(u16 & CPI_STATUS_ERR){
        /* An error code is passed back instead of typical retspec.
         * We directly return this error as our function's return code.
         */
        if((rc = cpi_decode(&i32, sizeof(i32), buf, &fidx, count))){
            goto err;
        }
        rc = (int)i32;
        /* PTU error codes should all be negative, but if they're not then map
         * them to negative numbers.
         */
        if(rc > 0){
            rc = -rc;
        }
        if(rc == 0){
            rc = -1;
        }
        /* mark the error as a PTU error code */
        rc = CPI_EMAKEPTU(rc);

        goto err;
    }

    /* call was successful; decode arguments */
    spec = ent->retspec;
    while(!rc && *spec){
        /* encode arg */
        switch(*spec){
        case CPI_T_INT:
        case CPI_T_ENUM:
            if(!(rc = cpi_decode(&i32, sizeof(i32), buf, &fidx, count))){
                *va_arg(*ap, int *) = (int)i32;
            }
            break;
        case CPI_T_UNSIGNED:
            if(!(rc = cpi_decode(&u32, sizeof(u32), buf, &fidx, count))){
                *va_arg(*ap, unsigned int *) = (unsigned int)u32;
            }
            break;
        case CPI_T_DOUBLE:
            if(!(rc = cpi_decode(&dbl, sizeof(dbl), buf, &fidx, count))){
                *va_arg(*ap, double *) = dbl;
            }
            break;
        case CPI_T_STR:
        case CPI_T_BINARY:
            maxlen = va_arg(*ap, int);
            plen = va_arg(*ap, int *);
            str = va_arg(*ap, char *);
            /* adjust for null terminator */
            if(*spec == CPI_T_STR){
                maxlen--;
            }
            /* length */
            if((rc = cpi_decode(&u16, sizeof(u16), buf, &fidx, count))){
                /* oops */
                break;
            }
            /* NULL means the user doesn't care for the length */
            if(plen){
                *plen = (int)u16;
            }
            /* is there enough room in the user's buffer? */
            if(maxlen < u16){
                rc = CPI_ERR_NO_MEM;
                goto err;
            }
            /* raw bytes */
            rc = cpi_frmcpy_out(str, (int)u16, buf, &fidx, count);
            if(*spec == CPI_T_STR){
                /* null terminator */
                str[u16] = '\0';
            }
            break;
        default:
            /* unknown type */
            rc = CPI_ERR_INVAL;
        }
        /* next arg */
        spec++;
    }

    if(!rc){
        return fidx;
    }
err:
    return rc;
}


/** Pack an opcode and its arguments into buf.
 *
 * This function wraps cpi_vpack(). See cpi_vpack() for details.
 */
int cpi_pack(void *buf, int count, uint8_t echo, cpi_opcode op, ...){
    va_list ap;
    int     rc;

    va_start(ap, op);
    rc = cpi_vpack(buf, count, echo, op, &ap);
    va_end(ap);

    return rc;
}


/** Unpack binary stream according to an opcode and its returns.
 *
 * This function wraps cpi_vunpack(). See cpi_vunpack() for details.
 */
int cpi_unpack(void *buf, int count, uint8_t *echo, uint16_t *status,
        cpi_opcode op, ...){
    va_list ap;
    int     rc;

    va_start(ap, op);
    rc = cpi_vunpack(buf, count, echo, status, op, &ap);
    va_end(ap);

    return rc;
}


/** Send a pan-tilt command but do not receive the reply.
 *
 * Use this function when sending multiple commands before receiving multiple
 * responses.
 *
 * \returns error or CPI_ERR_OK on success
 *
 * \param cer       Cerial handle
 * \param echo      Echo
 * \param op        Opcode for receive decoding
 * \param ap        Argument list
 *
 * \returns an error code (CPI_ERR_OK on success).
 */
int cpi_vsend(cerialh cer, uint8_t echo, cpi_opcode op, va_list *ap){
    unsigned char   frame[CPI_MAX_FRAME], cmd[CPI_MAX_FRAME];
    size_t          flen;
    int             rc;

    /* create frame */
    if((rc = cpi_vpack(cmd, sizeof(cmd), echo, op, ap)) < 0){
        return rc;
    }
    if((flen = cfrm_enframe(frame, sizeof(frame), cmd, (size_t)rc)) == 0){
        return CPI_ERR_GENERAL;
    }

    /* send frame */
    if(cerblockwrite(cer, frame, flen) < flen){
        /* failure to write? user's function must have errored */
        return CPI_ERR_WRITE;
    }

    return CPI_ERR_OK;
}


/** Receive one command reply from the pan-tilt.
 *
 * Use this function when sending multiple commands before receiving multiple
 * responses.
 *
 * The opcode used to receive a reply must match the command. Incorrect
 * decoding or decode errors will occur if opcodes are mismatched.
 *
 * \param cer       Cerial handle
 * \param echo      Pointer to echo storage
 * \param status    Pointer to status code storage
 * \param op        Opcode for receive decoding
 * \param ap        Argument list
 *
 * \returns an error code (CPI_ERR_OK on success).
 */
int cpi_vrecv(cerialh cer, uint8_t *echo, uint16_t *status, cpi_opcode op,
        va_list *ap){
    unsigned char   frame[CPI_MAX_FRAME], cmd[CPI_MAX_FRAME];
    size_t          flen, clen;
    int             rc = 0;
	int				offset = -1;
    /* get the reply */
    for(flen = 0; flen < sizeof(frame)
					&& cerread(cer, &frame[flen], 1) == 1
					&& !CFRM_IS_EOF(frame[flen]);
		flen++)
	{
		if (CFRM_IS_SOF(frame[flen]) && (offset < 0))
			offset = flen;
	}
    if(CFRM_IS_SOF(frame[offset]))
	{
		if (( !CFRM_IS_EOF(frame[flen])))// && (op != OP_RESET))
		{
			/* oops! we lost sync! */
			return CPI_ERR_LOST_SYNC;
		}
	}
	else
	{
			/* oops! we lost sync! */
			return CPI_ERR_LOST_SYNC;
	}

    /* decode reply */
    if((clen = cfrm_deframe(cmd, frame + offset + 1, flen - offset - 1)) == 0){
        return CPI_ERR_GENERAL;
    }
    if((rc = cpi_vunpack(cmd, clen, echo, status, op, ap)) < 0){
        return rc;
    }

    return CPI_ERR_OK;
}


/** Send a pan-tilt command but do not receive the reply.
 *
 * This function wraps cpi_vsend(). See cpi_vsend() for details.
 */
int cpi_send(cerialh cer, uint8_t echo, cpi_opcode op, ...){
    va_list ap;
    int     rc;

    va_start(ap, op);
    rc = cpi_vsend(cer, echo, op, &ap);
    va_end(ap);

    return rc;
}


/** Receive one command reply from the pan-tilt.
 *
 * This function wraps cpi_vrecv(). See cpi_vrecv() for details.
 */
int cpi_recv(cerialh cer, uint8_t *echo, uint16_t *status, cpi_opcode op, ...){
    va_list ap;
    int     rc;

    va_start(ap, op);
    rc = cpi_vrecv(cer, echo, status, op, &ap);
    va_end(ap);

    return rc;
}


/** Send a pan-tilt command and receive the reply.
 *
 * This function is the basic interface to the pan-tilt. It wraps cpi_send()
 * and cpi_recv() into one call.
 *
 * Arguments must be of the types that the opcode specification \ref cpi_optable
 * prescribes. Returns must also be of the same types that the opcode return
 * specification prescribes, except they must be pointers to the type. Put
 * another way, arguments are pass by value and returns are pass by reference.
 *
 * For example, an opcode that takes an integer and returns a double should
 * be called by cpi_ptcmd() as:
 * \code
 * int index = 10;
 * double distance;
 *
 * cpi_ptcmd(cer, &status, OP_GEO_DIST_LANDMARK, index, &distance);
 * \endcode
 *
 * \param cer       Cerial handle
 * \param status    Pointer to status code storage
 * \param op        Opcode to execute
 *
 * \returns an error code (CPI_ERR_OK on success).
 */
int cpi_ptcmd(cerialh cer, uint16_t *status, cpi_opcode op, ...){
    va_list          ap;
    int              rc;
    uint8_t          echo_ret;
    static uint8_t   echo = 0;

    echo++;
    // cpi_resync uses echo = 0xF as special value. use 0x0 to 0xE
    if(echo >=0xF) echo = 0;

    va_start(ap, op);
    if((rc = cpi_vsend(cer, echo, op, &ap)) == 0)
	{
        rc = cpi_vrecv(cer, &echo_ret, status, op, &ap);

        if(!rc && (echo_ret & 0x0F) != echo)
		{
            /* echo field mismatch */
            rc = CPI_ERR_LOST_SYNC;
        }
    }
    va_end(ap);
    return rc;
}


/** Poll the PTU until an integer-returning opcode returns a specific value.
 *
 * \param cer       Cerial handle
 * \param yield     Callback if the value is not yet reached.
 * \param yparam    Callback parameter (user defined void pointer).
 * \param op        Opcode to poll
 * \param value     Return value to compare against.
 *
 * Yield callback is optional (set to NULL). If yield returns non-zero, then
 * this function will terminate with its return value.
 *
 * Example:
 * \code
 * cpi_block_until(cer, NULL, NULL, OP_PAN_CURRENT_POS_GET, 200);
 * \endcode
 */
int cpi_block_until(cerialh cer,
        int (*yield)(void *), void *yparam,
        cpi_opcode op, int value){
    int                 rc, qval;
    uint16_t            status;
    struct cpi_opent    *ent;

    /* get op entry */
    if((ent = cpi_op_lkup(op)) == NULL){
        return CPI_ERR_INVAL;
    }

    /* assert integer type return */
    if(!((ent->retspec[0] == CPI_T_INT ||
                    ent->retspec[0] == CPI_T_UNSIGNED) &&
                ent->retspec[1] == '\0')){
        return CPI_ERR_INVAL;
    }

    do {
        /* execute command */
        if((rc = cpi_ptcmd(cer, &status, op, &qval))){
            goto err;
        }
        /* user callback */
        if(qval != value && yield && (rc = (*yield)(yparam)) != 0){
            goto err;
        }
    } while(qval != value);

err:
    return rc;
}

/** Write bytes to an auxiliary channel.
 *
 * Channel I/O is for auxiliary serial channels, like CHA and CHB. This
 * function uses OP_CH_TX functions to execute I/O. Error codes are negative,
 * so test the return value of this function for negative values rather than
 * a constant.
 *
 * \param cer       Cerial handle
 * \param channel   Channel identifier (CHA, CHB, etc)
 * \param buf       Pointer to byte buffer
 * \param len       Length of buffer (number of bytes to write).
 *
 * \returns the number of bytes written or an error code.
 */
int cpi_ch_write(cerialh cer, int channel, void *buf, int len){
    uint16_t status;
    int      rc, count;

    if(channel < CPI_CHA || channel > CPI_CHB){
        return CPI_ERR_INVAL;
    }

    if((rc = cpi_ptcmd(cer, &status, OP_CH_TX, channel, len, buf, &count))){
        return rc;
    }

    return count;
}


/** Read bytes from an auxiliary channel.
 *
 * Channel I/O is for auxiliary serial channels, like CHA and CHB. This
 * function uses OP_CH_RX functions to execute I/O. Error codes are negative,
 * so test the return value of this function for negative values rather than
 * a constant.
 *
 * \param cer       Cerial handle
 * \param channel   Channel identifier (CHA, CHB, etc)
 * \param buf       Pointer to byte buffer
 * \param len       Length of buffer (number of bytes to read).
 *
 * \returns the number of bytes read or an error code.
 */
int cpi_ch_read(cerialh cer, int channel, void *buf, int len){
    uint16_t status;
    int      rc, count;

    if(channel < CPI_CHA || channel > CPI_CHB){
        return CPI_ERR_INVAL;
    }

    if((rc = cpi_ptcmd(cer, &status, OP_CH_RX, channel, len, len, &count,
                    buf))){
        return rc;
    }

    return count;
}


/** Get a string describing a CPI or PTU error code.
 *
 * \param error     Error code to be described.
 *
 * \returns a valid string pointer
 *
 * The returned string is read only. Reentrant behavior is undefined.
 */
char *cpi_strerror(int error){
    static char *estrs[] = {
        "OK",
        "general error",
        "argument is out of bounds",
        "axis error",
        "vane error",
        "invalid argument",
        "timeout",
        "try again",
        "out of memory",
        "prerequisite not met",
        "read error",
        "write error",
        "callback reinterpreted",
        "callback failed",
        "base speed is too low",
        "canceled",
        "move fault",
        "illegal command in constant velocity mode",
        "invalid with zero speed",
        "network unreachable",
        "illegal command during stabilization",
        "gyro failed to program",
        "failed to converge",
        "not enough landmarks",
        "not implemented",
        "not enabled",
        "illegal command",
        "cable disconnected",
        "no entry",
        "continuous mode not supported",
        "illegal unit ID",
        "state corrupted; defaults restored",
        "state corrupted; save failed",
        "invalid preset",
        "unmapped error",
        "illegal without limits",
        "acceleration table exceeded",
        "illegal with continuous mode",
        "lost protocol synchrony"
    };
    static char *unknown = "unknown error";

    /* all errors are less than CPI_ERR_OK */
    if(error > CPI_ERR_OK){
        error = CPI_ERR_OK;
    }

    /* map to positive space */
    error = abs(CPI_EMASK(error));

    /* return error's string */
    if(error >= (sizeof(estrs) / sizeof(estrs[0]))){
        return unknown;
    } else {
        return estrs[error];
    }
}


/** Get this library's version as an integer.
 *
 * Use this function to get run-time access to versioning information. Compile-
 * time access should use \ref CPI_VERSION.
 *
 * \returns a constant integer containing \ref CPI_VERSION.
 */
int cpi_version(void){
    return CPI_VERSION;
}


/** Get this library's version as a string.
 *
 * Use this function to get run-time access to versioning information. Compile-
 * time access should use \ref CPI_VERSION_STR.
 *
 * \returns a constant string containing \ref CPI_VERSION_STR.
 */
const char *cpi_version_str(void){
    return CPI_VERSION_STR;
}

