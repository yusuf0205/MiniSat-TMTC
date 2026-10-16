/*
 * hdlc.c - bit-stuffed HDLC framing (transmit) and flag-hunting deframer (receive).
 */
#include <string.h>
#include "hdlc.h"
#include "codec.h"
#include "icd.h"

/* ------------------------------ TX ------------------------------ */
void bw_init(bitwriter_t *w, uint8_t *buf, size_t cap_bytes)
{
    w->buf = buf; w->cap_bits = cap_bytes * 8; w->nbits = 0; w->overflow = 0;
    memset(buf, 0, cap_bytes);
}

void bw_put_bit(bitwriter_t *w, int bit)
{
    if (w->nbits >= w->cap_bits) { w->overflow = 1; return; }
    if (bit) w->buf[w->nbits >> 3] |= (uint8_t)(0x80u >> (w->nbits & 7));
    w->nbits++;
}

static void put_flag(bitwriter_t *w)
{
    for (int i = 7; i >= 0; i--) bw_put_bit(w, (HDLC_FLAG >> i) & 1);
}

static void put_stuffed_byte(bitwriter_t *w, uint8_t b, int *ones)
{
    for (int i = 7; i >= 0; i--) {
        int bit = (b >> i) & 1;
        bw_put_bit(w, bit);
        if (bit) {
            if (++(*ones) == 5) { bw_put_bit(w, 0); *ones = 0; }   /* the stuffed 0 */
        } else {
            *ones = 0;
        }
    }
}

size_t hdlc_encode_frame(bitwriter_t *w, const uint8_t dest[7], const uint8_t src[7],
                         const uint8_t *info, size_t info_len)
{
    uint8_t frame[HDLC_MAX_FRAME];
    size_t n = 0;
    if (AX25_HDR_LEN + info_len + FRAME_FCS_LEN > sizeof frame) return 0;

    memcpy(frame + n, dest, 7); n += 7;
    memcpy(frame + n, src, 7);  n += 7;
    frame[n++] = AX25_CTRL_UI;
    frame[n++] = AX25_PID_NONE;
    memcpy(frame + n, info, info_len); n += info_len;
    uint16_t fcs = crc16_ccitt(frame, n);          /* flags are NOT included */
    frame[n++] = (uint8_t)(fcs >> 8);
    frame[n++] = (uint8_t)fcs;

    size_t start = w->nbits;
    int ones = 0;
    put_flag(w);
    for (size_t i = 0; i < n; i++) put_stuffed_byte(w, frame[i], &ones);
    put_flag(w);
    return w->nbits - start;
}

size_t bw_finish(bitwriter_t *w)
{
    while (w->nbits & 7) bw_put_bit(w, 1);           /* idle line = marking (1s) */
    return w->nbits >> 3;
}

void ax25_addr(uint8_t out[7], const char *call, uint8_t ssid)
{
    size_t i = 0;
    for (; i < 6 && call[i]; i++) out[i] = (uint8_t)(call[i] << 1);   /* AX.25 shifts chars left */
    for (; i < 6; i++) out[i] = (uint8_t)(' ' << 1);
    out[6] = (uint8_t)(0x60 | ((ssid & 0x0F) << 1));
}

/* ------------------------------ RX ------------------------------ */
void hdlc_rx_init(hdlc_rx_t *rx, hdlc_frame_cb cb, void *user)
{
    memset(rx, 0, sizeof *rx);
    rx->cb = cb; rx->user = user;
}

static void rx_append(hdlc_rx_t *rx, int bit)
{
    if (!rx->in_frame) return;
    if (rx->nbits >= (HDLC_MAX_FRAME + 1) * 8) {   /* too long: drop and wait for next flag */
        rx->in_frame = 0; rx->nbits = 0; rx->aborts++;
        return;
    }
    size_t byte = rx->nbits >> 3;
    if ((rx->nbits & 7) == 0) rx->bytes[byte] = 0;
    if (bit) rx->bytes[byte] |= (uint8_t)(0x80u >> (rx->nbits & 7));
    rx->nbits++;
}

static void rx_end_of_frame(hdlc_rx_t *rx)
{
    /* The buffer holds the frame plus the first 7 bits of the closing flag
     * (0 + six 1s). Remove them. */
    if (rx->in_frame && rx->nbits >= 7) {
        size_t bits = rx->nbits - 7;
        size_t len = bits / 8;
        if (bits < 8) {
            /* back-to-back flags or idle-fill leftovers: nothing between them */
        } else if ((bits & 7) || len < AX25_HDR_LEN + FRAME_FCS_LEN + 1) {
            rx->runts++;
        } else {
            uint16_t fcs = crc16_ccitt(rx->bytes, len - 2);
            uint16_t got = (uint16_t)((rx->bytes[len - 2] << 8) | rx->bytes[len - 1]);
            if (fcs == got) { rx->frames_ok++; if (rx->cb) rx->cb(rx->bytes, len - 2, rx->user); }
            else rx->fcs_errors++;
        }
    }
    rx->in_frame = 1;          /* a closing flag is also the next opening flag */
    rx->nbits = 0;
}

void hdlc_rx_bit(hdlc_rx_t *rx, int bit)
{
    if (bit) {
        rx->ones++;
        if (rx->ones > 6) {                        /* 7+ ones: abort / idle line */
            if (rx->in_frame && rx->nbits > 8) rx->aborts++;
            rx->in_frame = 0; rx->nbits = 0;
            return;
        }
        rx_append(rx, 1);
        return;
    }
    /* bit == 0 */
    if (rx->ones == 5) { rx->ones = 0; return; }   /* stuffed zero: throw it away */
    if (rx->ones == 6) {                           /* 01111110 = flag             */
        rx->ones = 0;                              /* (this last 0 is not stored) */
        rx_end_of_frame(rx);
        return;
    }
    rx->ones = 0;
    rx_append(rx, 0);
}

void hdlc_rx_bytes(hdlc_rx_t *rx, const uint8_t *packed, size_t nbytes)
{
    for (size_t i = 0; i < nbytes; i++)
        for (int b = 7; b >= 0; b--)
            hdlc_rx_bit(rx, (packed[i] >> b) & 1);
}
