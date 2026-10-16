/*
 * ccsds.h - CCSDS Space Packet (CCSDS 133.0-B-2) build/parse.
 *
 * Primary header (6 bytes):
 *   version(3) type(1) sec_hdr_flag(1) APID(11) | seq_flags(2) seq_count(14) | data_length(16)
 * data_length = (number of bytes after the primary header) - 1, and in this
 * ICD the 2-byte packet CRC IS counted inside the data field.
 *
 * TC: optional 8-byte time tag (CUC + reserved) when sec_hdr_flag = 1.
 * TM: always a 7-byte CUC secondary header (OBC time).
 */
#ifndef CCSDS_H
#define CCSDS_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint32_t coarse;     /* whole seconds since mission epoch */
    uint16_t fine;       /* 1/65536 s                         */
} cuc_time_t;

typedef struct {
    uint8_t  version, type, sec_hdr;
    uint16_t apid;
    uint8_t  seq_flags;
    uint16_t seq_count;
    uint16_t data_length;          /* raw field value */
} ccsds_hdr_t;

typedef struct {
    ccsds_hdr_t hdr;
    int         has_time;
    cuc_time_t  time;              /* TC time tag or TM OBC time */
    uint16_t    cmd_id;            /* TC only */
    const uint8_t *data;           /* TC: params after cmd id; TM: source data */
    size_t      data_len;
} ccsds_packet_t;

/* Result codes for parsing */
#define PKT_OK            0
#define PKT_ERR_SHORT     1
#define PKT_ERR_LENGTH    2
#define PKT_ERR_CRC       3
#define PKT_ERR_VERSION   4
#define PKT_ERR_TIME      5

uint64_t cuc_to_ms(cuc_time_t t);
cuc_time_t ms_to_cuc(uint64_t ms);

size_t ccsds_build_tm(uint8_t *out, size_t cap, uint16_t apid, uint16_t seq,
                      cuc_time_t obc_time, const uint8_t *src, size_t src_len);
size_t ccsds_build_tc(uint8_t *out, size_t cap, uint16_t apid, uint16_t seq,
                      const cuc_time_t *time_tag, uint16_t cmd_id,
                      const uint8_t *params, size_t params_len);
int    ccsds_parse(const uint8_t *pkt, size_t len, ccsds_packet_t *out);

#endif
