/*
 * test_main.c - unit tests for the flight software building blocks.
 * Tiny framework: CHECK() counts passes/failures, exit code = failures.
 */
#include <stdio.h>
#include <string.h>
#include "codec.h"
#include "hdlc.h"
#include "ccsds.h"
#include "obc.h"

static int passed, failed;
#define CHECK(cond, name) do { if (cond) passed++; else { failed++; printf("FAIL: %s (line %d)\n", name, __LINE__); } } while (0)

/* ---------------- CRC ---------------- */
static void test_crc(void)
{
    CHECK(crc16_ccitt((const uint8_t *)"123456789", 9) == 0x29B1, "CRC-16/CCITT-FALSE check value 0x29B1");
    uint8_t b[] = {0x15, 0x00, 0xC0, 0x00, 0x00, 0x03, 0x00, 0x03};
    uint16_t c1 = crc16_ccitt(b, sizeof b);
    b[3] ^= 0x01;                                     /* flip one bit */
    CHECK(crc16_ccitt(b, sizeof b) != c1, "CRC detects a single flipped bit");
}

/* ---------------- big-endian cursor ---------------- */
static void test_codec(void)
{
    uint8_t buf[16]; cursor_t c; cur_init(&c, buf, sizeof buf);
    put_u16(&c, 4500); put_u32(&c, 0xDEADBEEF); put_f32(&c, -1.5f); put_i16(&c, -2);
    CHECK(buf[0] == 0x11 && buf[1] == 0x94, "4500 encodes big-endian as 11 94");
    cursor_t r; cur_init(&r, buf, c.pos);
    CHECK(get_u16(&r) == 4500, "u16 round trip");
    CHECK(get_u32(&r) == 0xDEADBEEF, "u32 round trip");
    CHECK(get_f32(&r) == -1.5f, "f32 round trip");
    CHECK(get_i16(&r) == -2, "i16 round trip");
    CHECK(!r.error, "no cursor error on exact read");
    (void)get_u8(&r);
    CHECK(r.error, "cursor refuses to read past the end");
}

/* ---------------- HDLC ---------------- */
static uint8_t got[400]; static size_t got_len; static int got_frames;
static void cb(const uint8_t *f, size_t n, void *u) { (void)u; memcpy(got, f, n); got_len = n; got_frames++; }

static void test_hdlc(void)
{
    uint8_t dest[7], src[7]; ax25_addr(dest, "GROUND", 0); ax25_addr(src, "MINSAT", 1);
    /* worst-case data: lots of 0x7E and 0xFF bytes that need stuffing */
    uint8_t info[64]; for (int i = 0; i < 64; i++) info[i] = (i % 2) ? 0x7E : 0xFF;
    uint8_t bits[1024]; bitwriter_t w; bw_init(&w, bits, sizeof bits);
    for (int k = 0; k < 3; k++) hdlc_encode_frame(&w, dest, src, info, sizeof info);
    size_t nb = bw_finish(&w);
    CHECK(w.nbits > 3 * (16 + 64 + 2 + 2) * 8, "stuffing added extra bits");

    hdlc_rx_t rx; hdlc_rx_init(&rx, cb, NULL); got_frames = 0;
    hdlc_rx_bytes(&rx, bits, nb);
    CHECK(got_frames == 3, "three back-to-back frames received");
    CHECK(got_len == 16 + 64 && memcmp(got + 16, info, 64) == 0, "payload survives stuffing/de-stuffing");

    /* corrupt one bit in the middle: FCS must catch it */
    bits[40] ^= 0x10;
    hdlc_rx_init(&rx, cb, NULL); got_frames = 0;
    hdlc_rx_bytes(&rx, bits, nb);
    CHECK(got_frames == 2 && rx.fcs_errors + rx.runts + rx.aborts >= 1, "corrupted frame is dropped, others survive");
}

/* ---------------- CCSDS ---------------- */
static void test_ccsds(void)
{
    uint8_t pkt[64];
    size_t n = ccsds_build_tc(pkt, sizeof pkt, 0x110, 5, NULL, 0x0011, (const uint8_t *)"\x0F\xA0", 2);
    CHECK(n == 6 + 2 + 2 + 2, "real-time TC length");
    CHECK(pkt[0] == 0x11 && pkt[1] == 0x10, "TC packet id: type=1, sec=0, APID 0x110");
    CHECK((size_t)((pkt[4] << 8) | pkt[5]) == n - 6 - 1, "length field = bytes after header - 1 (CRC included)");
    ccsds_packet_t p;
    CHECK(ccsds_parse(pkt, n, &p) == PKT_OK, "parse real-time TC");
    CHECK(p.hdr.apid == 0x110 && p.hdr.seq_count == 5 && p.cmd_id == 0x0011 && p.data_len == 2, "fields decoded");
    CHECK(p.hdr.seq_flags == 3, "standalone grouping flags");

    cuc_time_t tag = ms_to_cuc(123456);
    n = ccsds_build_tc(pkt, sizeof pkt, 0x100, 16383, &tag, 0x0001, NULL, 0);
    CHECK(pkt[0] == 0x19, "time-tagged TC sets the secondary header flag");
    CHECK(ccsds_parse(pkt, n, &p) == PKT_OK && p.has_time && cuc_to_ms(p.time) == 123456, "time tag round trip");
    CHECK(p.hdr.seq_count == 16383, "14-bit sequence count max");

    pkt[n - 3] ^= 0x40;
    CHECK(ccsds_parse(pkt, n, &p) == PKT_ERR_CRC, "corrupted packet rejected by CRC");
    CHECK(ccsds_parse(pkt, n - 1, &p) == PKT_ERR_LENGTH, "truncated packet rejected by length");
}

/* ---------------- TLE ---------------- */
static void test_tle(void)
{
    const char *l1 = "1 25544U 98067A   08264.51782528 -.00002182  00000-0 -11606-4 0  2927";
    const char *l2 = "2 25544  51.6416 247.4627 0006703 130.5360 325.0288 15.72125391563537";
    CHECK(tle_line_checksum_ok(l1), "TLE line 1 checksum");
    CHECK(tle_line_checksum_ok(l2), "TLE line 2 checksum");
    char bad[70]; memcpy(bad, l1, 70); bad[20] = '9';
    CHECK(!tle_line_checksum_ok(bad), "corrupted TLE line detected");
}

int main(void)
{
    test_crc(); test_codec(); test_hdlc(); test_ccsds(); test_tle();
    printf("%d passed, %d failed\n", passed, failed);
    return failed;
}
