/*
 * ccsds.c - CCSDS space packet build and parse with CRC checking.
 */
#include <string.h>
#include "ccsds.h"
#include "codec.h"
#include "icd.h"

uint64_t cuc_to_ms(cuc_time_t t)
{
    return (uint64_t)t.coarse * 1000u + ((uint64_t)t.fine * 1000u + 32768u) / 65536u;   /* rounded */
}

cuc_time_t ms_to_cuc(uint64_t ms)
{
    cuc_time_t t;
    t.coarse = (uint32_t)(ms / 1000u);
    t.fine = (uint16_t)(((ms % 1000u) * 65536u) / 1000u);
    return t;
}

static void put_cuc(cursor_t *c, cuc_time_t t)
{
    put_u8(c, CUC_PFIELD);
    put_u32(c, t.coarse);
    put_u16(c, t.fine);
}

static void put_primary(cursor_t *c, uint8_t type, uint8_t sec, uint16_t apid,
                        uint16_t seq, uint16_t data_field_len)
{
    uint16_t id = (uint16_t)((0u << 13) | ((type & 1u) << 12) | ((sec & 1u) << 11) | (apid & 0x7FFu));
    uint16_t sc = (uint16_t)((CCSDS_SEQ_STANDALONE << 14) | (seq % CCSDS_SEQ_MOD));
    put_u16(c, id);
    put_u16(c, sc);
    put_u16(c, (uint16_t)(data_field_len - 1));      /* "length minus one" */
}

static size_t finish_crc(cursor_t *c)
{
    uint16_t crc = crc16_ccitt(c->buf, c->pos);      /* header + data field so far */
    put_u16(c, crc);
    return c->error ? 0 : c->pos;
}

size_t ccsds_build_tm(uint8_t *out, size_t cap, uint16_t apid, uint16_t seq,
                      cuc_time_t obc_time, const uint8_t *src, size_t src_len)
{
    if (src_len > MAX_TM_SOURCE_LEN) return 0;
    cursor_t c; cur_init(&c, out, cap);
    put_primary(&c, CCSDS_TYPE_TM, 1, apid, seq, (uint16_t)(TM_SEC_HDR_LEN + src_len + PKT_CRC_LEN));
    put_cuc(&c, obc_time);
    put_bytes(&c, src, src_len);
    return finish_crc(&c);
}

size_t ccsds_build_tc(uint8_t *out, size_t cap, uint16_t apid, uint16_t seq,
                      const cuc_time_t *time_tag, uint16_t cmd_id,
                      const uint8_t *params, size_t params_len)
{
    cursor_t c; cur_init(&c, out, cap);
    size_t tt = time_tag ? TC_TIMETAG_LEN : 0;
    put_primary(&c, CCSDS_TYPE_TC, time_tag ? 1 : 0, apid, seq,
                (uint16_t)(tt + 2 + params_len + PKT_CRC_LEN));
    if (time_tag) { put_cuc(&c, *time_tag); put_u8(&c, 0x00); }   /* + reserved byte */
    put_u16(&c, cmd_id);
    put_bytes(&c, params, params_len);
    return finish_crc(&c);
}

int ccsds_parse(const uint8_t *pkt, size_t len, ccsds_packet_t *p)
{
    memset(p, 0, sizeof *p);
    if (len < CCSDS_PRI_HDR_LEN + PKT_CRC_LEN + 1) return PKT_ERR_SHORT;

    cursor_t c; cur_init(&c, (uint8_t *)pkt, len);
    uint16_t id = get_u16(&c), sc = get_u16(&c);
    p->hdr.version     = (uint8_t)(id >> 13);
    p->hdr.type        = (uint8_t)((id >> 12) & 1u);
    p->hdr.sec_hdr     = (uint8_t)((id >> 11) & 1u);
    p->hdr.apid        = id & 0x7FFu;
    p->hdr.seq_flags   = (uint8_t)(sc >> 14);
    p->hdr.seq_count   = sc & 0x3FFFu;
    p->hdr.data_length = get_u16(&c);

    if (p->hdr.version != 0) return PKT_ERR_VERSION;
    size_t total = CCSDS_PRI_HDR_LEN + (size_t)p->hdr.data_length + 1u;
    if (total != len) return PKT_ERR_LENGTH;

    uint16_t crc = crc16_ccitt(pkt, len - 2);
    uint16_t got = (uint16_t)((pkt[len - 2] << 8) | pkt[len - 1]);
    if (crc != got) return PKT_ERR_CRC;

    size_t end = len - 2;                         /* stop before the CRC */
    if (p->hdr.sec_hdr) {
        if (get_u8(&c) != CUC_PFIELD) return PKT_ERR_TIME;
        p->time.coarse = get_u32(&c);
        p->time.fine = get_u16(&c);
        p->has_time = 1;
        if (p->hdr.type == CCSDS_TYPE_TC) (void)get_u8(&c);   /* reserved byte */
    }
    if (p->hdr.type == CCSDS_TYPE_TC) {
        if (c.pos + 2 > end) return PKT_ERR_LENGTH;
        p->cmd_id = get_u16(&c);
    }
    if (c.error || c.pos > end) return PKT_ERR_LENGTH;
    p->data = pkt + c.pos;
    p->data_len = end - c.pos;
    return PKT_OK;
}
