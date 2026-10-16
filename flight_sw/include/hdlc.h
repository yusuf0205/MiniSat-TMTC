/*
 * hdlc.h - AX.25/HDLC-style framing with real bit stuffing.
 *
 * Frame on the wire (bits, MSB of each byte first):
 *   0x7E | dest(7) | src(7) | ctrl | pid | info(CCSDS packet) | FCS(2) | 0x7E
 * Everything between the flags is bit-stuffed: a 0 is inserted after
 * every five consecutive 1s, so the data can never look like a flag.
 * The FCS is CRC-16/CCITT-FALSE over dest..info (flags excluded).
 */
#ifndef HDLC_H
#define HDLC_H

#include <stdint.h>
#include <stddef.h>

#define HDLC_MAX_FRAME   300            /* bytes between flags (unstuffed) */

/* -------- transmit side: append frames to a packed bit stream -------- */
typedef struct {
    uint8_t *buf;        /* packed bits, MSB first        */
    size_t   cap_bits;
    size_t   nbits;
    int      overflow;
} bitwriter_t;

void   bw_init(bitwriter_t *w, uint8_t *buf, size_t cap_bytes);
void   bw_put_bit(bitwriter_t *w, int bit);
/* Encode one frame (addresses + info) and append its stuffed bits. */
size_t hdlc_encode_frame(bitwriter_t *w, const uint8_t dest[7], const uint8_t src[7],
                         const uint8_t *info, size_t info_len);
/* Pad with idle 1s to a byte boundary; returns bytes used. */
size_t bw_finish(bitwriter_t *w);

/* -------- receive side: bit-level state machine -------- */
typedef void (*hdlc_frame_cb)(const uint8_t *frame, size_t len, void *user);

typedef struct {
    uint8_t  bytes[HDLC_MAX_FRAME + 4];
    size_t   nbits;          /* de-stuffed bits collected in the current frame */
    int      ones;           /* run of consecutive 1s                           */
    int      in_frame;
    /* statistics */
    uint32_t frames_ok, fcs_errors, aborts, runts;
    hdlc_frame_cb cb;
    void    *user;
} hdlc_rx_t;

void hdlc_rx_init(hdlc_rx_t *rx, hdlc_frame_cb cb, void *user);
void hdlc_rx_bit(hdlc_rx_t *rx, int bit);
void hdlc_rx_bytes(hdlc_rx_t *rx, const uint8_t *packed, size_t nbytes);

/* Build a 7-byte AX.25 address from a call-sign (<=6 chars) and SSID. */
void ax25_addr(uint8_t out[7], const char *call, uint8_t ssid);

#endif
