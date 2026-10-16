/*
 * obc.h - MiniSat On-Board Computer: TC pipeline, scheduler, telemetry,
 * on-board storage, FDIR and simulated subsystems.
 */
#ifndef OBC_H
#define OBC_H

#include <stdint.h>
#include <stddef.h>
#include "icd.h"
#include "ccsds.h"
#include "hdlc.h"

#define TICK_MS            10
#define SCHED_SLOTS        16
#define STORE_SLOTS        600
#define TXQ_SLOTS          256
#define SSTM_SAMPLE_BYTES  12
#define SSTM_PER_PACKET    16

typedef struct { uint16_t len; uint8_t bytes[MAX_PACKET_LEN]; } pkt_buf_t;

typedef struct {
    int        used;
    uint64_t   exec_ms;
    uint16_t   apid, seq, cmd_id;
    uint32_t   tag_coarse;
    uint16_t   params_len;
    uint8_t    params[MAX_PACKET_LEN];
} sched_entry_t;

typedef struct { uint16_t apid; cuc_time_t t; uint16_t len; uint8_t data[MAX_TM_SOURCE_LEN]; } store_entry_t;

typedef struct { pkt_buf_t q[TXQ_SLOTS]; int head, count; uint32_t dropped; } txq_t;

typedef struct {
    /* EPS */
    double soc;               /* 0..1 */
    double batt_mv, solar_mw, load_mw;
    int    rail_payload, rail_wheel, mtq_on;
    int    eclipse;
    uint16_t safe_mv;
    uint32_t low_v_ms;        /* how long voltage has been under the threshold */
    /* ADCS */
    double wheel_rpm; int16_t wheel_cmd;
    double mag_x_nt;
    double temp_c;
    /* COMMS */
    uint8_t tx_power_dbm;
} subsys_t;

typedef struct {
    int      active;
    uint8_t  rate_hz;
    uint32_t end_ms, next_ms, samples;
    uint8_t  buf[SSTM_PER_PACKET * SSTM_SAMPLE_BYTES];
    int      nbuf;
    cuc_time_t first_t;
} sstm_t;

typedef struct {
    uint64_t now_ms;
    uint8_t  mode;
    uint16_t acc_cnt, exec_cnt, rej_cnt;
    int32_t  last_tc_seq[4];          /* per TC APID, -1 = none yet */
    uint16_t tm_seq[8];               /* per TM APID                */
    sched_entry_t sched[SCHED_SLOTS];
    store_entry_t store[STORE_SLOTS];
    int      store_head, store_count;
    uint32_t store_overwritten;
    int      store_overflow_reported;
    txq_t    tx_hi, tx_lo;
    double   tx_budget_bits;
    uint32_t dump_remaining;          /* stored packets still to play back */
    uint8_t  hk_period_s;
    uint64_t next_hk_ms;
    sstm_t   sstm;
    subsys_t ss;
    char     tle[2][70];
    double   tle_epoch;
    uint32_t rng;
    /* link */
    hdlc_rx_t rx;
    uint8_t  addr_ground[7], addr_sat[7];
    /* statistics for the report */
    uint32_t pkt_crc_err, pkt_other_err, tc_received;
    uint32_t frames_sent;
} obc_t;

void obc_init(obc_t *o);
/* Feed received uplink bits (packed, MSB first). Complete TCs are executed. */
void obc_receive_bits(obc_t *o, const uint8_t *packed, size_t nbytes);
/* Advance one TICK_MS step; frames transmitted during the tick are appended to 'out'. */
void obc_tick(obc_t *o, bitwriter_t *out);

/* TLE helper, also used by unit tests */
int  tle_line_checksum_ok(const char *line69);

#endif
