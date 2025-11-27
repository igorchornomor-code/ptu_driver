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
 * \file cfrm.c
 * \brief Framing layer.
 *
 * See \ref Framing for an overview of framing.
 */

#include <stdint.h>
#include <stdbool.h>
#include "cfrm.h"
#include "crc_cfrm.h"


/** Maximum encoded size of count raw bytes.
 *
 * Use this function to determine the minimum size of dst in cfrm_encode().
 *
 * \returns the number of bytes dst must be for cfrm_encode().
 */
static size_t cfrm_max_encoded_size(size_t count){
    return count + (count / 255) + 1;
}


/** Encode frame payload with modified COBS.
 *
 * The destination buffer must be big enough to hold the worst-case encoding
 * of src and fcs.
 *
 * \param dst       Destination buffer.
 * \param src       Source buffer.
 * \param count     Number of bytes to encode from src.
 * \param fcs       Frame-check-sequence to be encoded.
 *
 * \returns the final encoded length.
 */
static size_t cfrm_encode(void *dst, const void *src, size_t count,
        uint16_t fcs){
    unsigned char   *dptr = (unsigned char *)dst;
    unsigned char   *sptr = (unsigned char *)src, *send = sptr + count;
    unsigned char   *cptr, code, fcs_bytes[2];
    size_t          len;
    bool            fcs_load;

    /* setup code */
    cptr = dptr++;
    code = CFRM_START;
    len = 1;

    /* setup fcs */
    fcs_load = true;
    fcs_bytes[0] = (unsigned char)(fcs >> 8);
    fcs_bytes[1] = (unsigned char)(fcs & 0xFF);

    while(sptr != send){
        /* start next block when we encouter an illegal code */
        if(*sptr == CFRM_ILLEGAL){
            *cptr = code;
            cptr = dptr++;
            code = CFRM_START;
        } else {
            /* write data and advance code */
            *dptr++ = *sptr;
            code++;
            /* max lenth; encode next block */
            if(code == CFRM_END){
                *cptr = code;
                cptr = dptr++;
                code = CFRM_START;
                len++;
            }
        }
        /* one byte down */
        len++;
        sptr++;
        /* redirect sptr to the fcs when at the end of src */
        if(sptr == send && fcs_load){
            sptr = fcs_bytes;
            send = sptr + 2;
            fcs_load = false;
        }
    }

    /* finish last block */
    *cptr = code;

    /* length */
    return len;
}


/** Decode frame payload from modified COBS.
 *
 * The destination buffer must be at least as big as the source.
 *
 * CFRM_ILLEGAL MUST NOT BE PRESENT IN SRC. Use the illegal code to indicate
 * end-of-frame before passing data to this function.
 *
 * \returns The number of bytes in dst or zero on error.
 */
static size_t cfrm_decode(void *dst, const void *src, size_t count){
    unsigned char   *dptr = (unsigned char *)dst;
    unsigned char   *sptr = (unsigned char *)src;
    unsigned char   ubi, code;
    size_t          bsize, dlen;

    /* count is unsigned. do not underflow count! */
    if(count == 0){
        return 0;
    }

    dlen = 0;
    while(count != 0){
        /* get the block code */
        code = *sptr++;
        count--;
        /* remove the block code and it's count.
         * verify we have enough data to decode the full block
         */
        bsize = (size_t)(code - CFRM_START);
        if(count < bsize){
            /* oops! must be a runt frame */
            return 0;
        }
        count -= bsize;
        dlen += bsize;
        /* copy out block data */
        for(ubi = CFRM_START; ubi != code; ubi++){
            *dptr++ = *sptr++;
        }
        /* The last code does not have an implicit CFRM_ILLEGAL.
         * The last code block as an implicit CFRM_ILLEGAL that isn't real.
         */
        if(count && code != CFRM_END){
            *dptr++ = CFRM_ILLEGAL;
            dlen++;
        }
    }

    return dlen;
}


/** Enframe data.
 *
 * \param frame     Destination frame storage.
 * \param maxlen    Maximum length of the frame parameter.
 * \param buf       Source data to enframe.
 * \param count     Number of bytes in data to enframe.
 *
 * \returns the number of bytes written to frame or zero (0) on error.
 */
size_t cfrm_enframe(void *frame, size_t maxlen, const void *buf, size_t count){
    unsigned char   *fptr = (unsigned char *)frame;
    uint16_t        fcs;
    size_t          len;

    /* Is frame buffer is big enough?
     * Additional bytes for SOF, encoded FCS, and EOF.
     */
    if(maxlen < (2 + cfrm_max_encoded_size(2 + count))){
        return 0;
    }

    /* SOF marker */
    *fptr = CFRM_SOF;
    len = 1;

    /* inner frame */
    fcs = crc_cfrm(buf, count);
    len += cfrm_encode(&fptr[len], buf, count, fcs);

    /* EOF marker */
    fptr[len] = CFRM_ILLEGAL;
    len++;

    return len;
}


/** Deframe data.
 *
 * \c dst must be at least as big as \c src.
 * \c src must not contain \ref CFRM_SOF or \ref CFRM_ILLEGAL. This restriction
 * is only valid for \c count bytes in \c src.
 *
 * \param dst       Destination storage.
 * \param src       Frame data (without \ref CFRM_SOF or \ref CFRM_ILLEGAL).
 * \param count     Number of bytes in src.
 *
 * \returns Number of bytes in dst or zero on error.
 *
 */
size_t cfrm_deframe(void *dst, const void *src, size_t count){
    unsigned char   *dptr = (unsigned char *)dst;
    uint16_t        fcs_frame;
    size_t          len;

    /* decode */
    if((len = cfrm_decode(dst, src, count)) == 0){
        return 0;
    }

    /* calc FCS */
    len -= 2;
    fcs_frame = (((uint16_t)dptr[len]) << 8) | dptr[len + 1];

    if(fcs_frame == crc_cfrm(dst, len)){
        return len;
    } else {
        return 0;
    }
}

