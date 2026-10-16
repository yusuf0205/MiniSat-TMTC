/*
 * obc.c - the On-Board Computer application.
 *
 *  Uplink bits -> HDLC deframer -> CCSDS parse -> TC validation pipeline
 *      -> acceptance ACK -> (execute now | store in time-tag schedule)
 *      -> execution ACK
 *  Every tick: subsystem physics, FDIR, scheduler, SSTM, housekeeping,
 *  and the transmitter (rate-limited to LINK_BITRATE_BPS, two priorities).
 */
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "obc.h"
#include "codec.h"

/* ====================================================================== */
/*  Small helpers                                                          */
/* ====================================================================== */
static uint32_t rng_next(obc_t *o)              /* xorshift32: deterministic noise */
{
    uint32_t x = o->rng; x ^= x << 13; x ^= x >> 17; x ^= x << 5; o->rng = x; return x;
}
static double noise(obc_t *o, double amp)
{
    return amp * (((double)(rng_next(o) % 20001u) / 10000.0) - 1.0);
}
static cuc_time_t now_cuc(const obc_t *o) { return ms_to_cuc(o->now_ms); }

static int tc_apid_index(uint16_t apid)
{
    switch (apid) {
    case APID_TC_OBC:   return 0;
    case APID_TC_ADCS:  return 1;
    case APID_TC_EPS:   return 2;
    case APID_TC_COMMS: return 3;
    default:            return -1;
    }
}
static int tm_apid_index(uint16_t apid) { return (apid >= 0x200 && apid <= 0x207) ? apid - 0x200 : 7; }

/* ====================================================================== */
/*  Transmit queues                                                        */
/* ====================================================================== */
static void txq_push(txq_t *q, const uint8_t *pkt, size_t len)
{
    if (q->count == TXQ_SLOTS) {                 /* full: drop the oldest */
        q->head = (q->head + 1) % TXQ_SLOTS; q->count--; q->dropped++;
    }
    pkt_buf_t *slot = &q->q[(q->head + q->count) % TXQ_SLOTS];
    slot->len = (uint16_t)len;
    memcpy(slot->bytes, pkt, len);
    q->count++;
}

static void send_tm(obc_t *o, uint16_t apid, cuc_time_t t, const uint8_t *src, size_t len, int high)
{
    uint8_t pkt[MAX_PACKET_LEN];
    int i = tm_apid_index(apid);
    size_t n = ccsds_build_tm(pkt, sizeof pkt, apid, o->tm_seq[i], t, src, len);
    if (!n) return;
    o->tm_seq[i] = (uint16_t)((o->tm_seq[i] + 1) % CCSDS_SEQ_MOD);
    txq_push(high ? &o->tx_hi : &o->tx_lo, pkt, n);
}

static void store_put(obc_t *o, uint16_t apid, cuc_time_t t, const uint8_t *d, size_t len);

/* 'keep' = also record a copy on board, so the ground still learns about
 * it from the next dump even if nobody was listening when it happened. */
static void send_ack(obc_t *o, uint8_t type, uint16_t apid, uint16_t seq, uint16_t cmd,
                     uint8_t code, uint32_t tag, int keep)
{
    uint8_t b[16]; cursor_t c; cur_init(&c, b, sizeof b);
    put_u8(&c, type); put_u16(&c, apid); put_u16(&c, seq); put_u16(&c, cmd);
    put_u8(&c, code); put_u32(&c, tag);
    send_tm(o, APID_TM_ACK, now_cuc(o), b, c.pos, 1);
    if (keep) store_put(o, APID_TM_ACK, now_cuc(o), b, c.pos);
}

static void send_event(obc_t *o, uint8_t evt, uint32_t value)
{
    uint8_t b[8]; cursor_t c; cur_init(&c, b, sizeof b);
    put_u8(&c, evt); put_u32(&c, value);
    send_tm(o, APID_TM_EVENT, now_cuc(o), b, c.pos, 1);
    if (evt != EVT_STM_OVERFLOW) store_put(o, APID_TM_EVENT, now_cuc(o), b, c.pos);   /* events are always kept */
}

/* ====================================================================== */
/*  On-board storage (stored telemetry, ring buffer)                       */
/* ====================================================================== */
static void store_put(obc_t *o, uint16_t apid, cuc_time_t t, const uint8_t *d, size_t len)
{
    if (o->store_count == STORE_SLOTS) {         /* full: overwrite the oldest */
        o->store_head = (o->store_head + 1) % STORE_SLOTS;
        o->store_count--;
        o->store_overwritten++;
        if (!o->store_overflow_reported) { o->store_overflow_reported = 1; send_event(o, EVT_STM_OVERFLOW, 0); }
    }
    store_entry_t *e = &o->store[(o->store_head + o->store_count) % STORE_SLOTS];
    e->apid = apid; e->t = t; e->len = (uint16_t)len; memcpy(e->data, d, len);
    o->store_count++;
}

/* ====================================================================== */
/*  TLE                                                                    */
/* ====================================================================== */
int tle_line_checksum_ok(const char *line)
{
    int sum = 0;
    for (int i = 0; i < 68; i++) {
        char ch = line[i];
        if (ch >= '0' && ch <= '9') sum += ch - '0';
        else if (ch == '-') sum += 1;
    }
    if (line[68] < '0' || line[68] > '9') return 0;
    return (sum % 10) == (line[68] - '0');
}

/* ====================================================================== */
/*  Command table: validation (at acceptance) and execution                */
/* ====================================================================== */
typedef int (*validate_fn)(obc_t *, const uint8_t *, size_t);
typedef int (*execute_fn)(obc_t *, const uint8_t *, size_t);

typedef struct {
    uint16_t    apid, cmd_id;
    uint16_t    min_len, max_len;
    int         allowed_in_safe;
    validate_fn validate;
    execute_fn  execute;
} cmd_def_t;

/* ---- validators: return ERR_NONE or ERR_BAD_PARAM ---- */
static int v_none(obc_t *o, const uint8_t *p, size_t n) { (void)o; (void)p; (void)n; return ERR_NONE; }
static int v_hk_period(obc_t *o, const uint8_t *p, size_t n) { (void)o; (void)n; return (p[0] >= 1 && p[0] <= 60) ? ERR_NONE : ERR_BAD_PARAM; }
static int v_mode(obc_t *o, const uint8_t *p, size_t n) { (void)o; (void)n; return p[0] <= 1 ? ERR_NONE : ERR_BAD_PARAM; }
static int v_get(obc_t *o, const uint8_t *p, size_t n) { (void)o; (void)n; return (p[0] >= 1 && p[0] <= 6) ? ERR_NONE : ERR_BAD_PARAM; }
static int v_wheel(obc_t *o, const uint8_t *p, size_t n)
{
    (void)o; (void)n; int16_t rpm = (int16_t)((p[0] << 8) | p[1]);
    return (rpm >= -6000 && rpm <= 6000) ? ERR_NONE : ERR_BAD_PARAM;
}
static int v_sstm(obc_t *o, const uint8_t *p, size_t n)
{
    (void)o; (void)n;
    uint32_t dur = ((uint32_t)p[1] << 24) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 8) | p[4];
    return (p[0] >= 1 && p[0] <= 50 && dur >= 100 && dur <= 60000) ? ERR_NONE : ERR_BAD_PARAM;
}
static int v_bool(obc_t *o, const uint8_t *p, size_t n) { (void)o; (void)n; return p[0] <= 1 ? ERR_NONE : ERR_BAD_PARAM; }
static int v_rail(obc_t *o, const uint8_t *p, size_t n) { (void)o; (void)n; return (p[0] >= 1 && p[0] <= 2 && p[1] <= 1) ? ERR_NONE : ERR_BAD_PARAM; }
static int v_safe_mv(obc_t *o, const uint8_t *p, size_t n)
{
    (void)o; (void)n; uint16_t mv = (uint16_t)((p[0] << 8) | p[1]);
    return (mv >= 6200 && mv <= 7400) ? ERR_NONE : ERR_BAD_PARAM;
}
static int v_txp(obc_t *o, const uint8_t *p, size_t n) { (void)o; (void)n; return (p[0] >= 20 && p[0] <= 33) ? ERR_NONE : ERR_BAD_PARAM; }

/* ---- executors: return ERR_NONE or ERR_EXEC (+ any reply TM) ---- */
static int x_noop(obc_t *o, const uint8_t *p, size_t n) { (void)o; (void)p; (void)n; return ERR_NONE; }

static int x_dump(obc_t *o, const uint8_t *p, size_t n)
{
    (void)n;
    /* Playback is paced by the transmitter (see transmitter()), so a big
     * dump never floods the transmit queue. */
    o->dump_remaining = (uint16_t)((p[0] << 8) | p[1]);
    return ERR_NONE;
}

static void dump_feed(obc_t *o)
{
    while (o->dump_remaining > 0 && o->store_count > 0 && o->tx_lo.count < 2) {
        store_entry_t *e = &o->store[o->store_head];
        send_tm(o, e->apid, e->t, e->data, e->len, 0);     /* original time stamp kept */
        o->store_head = (o->store_head + 1) % STORE_SLOTS;
        o->store_count--;
        o->dump_remaining--;
        o->store_overflow_reported = 0;
    }
    if (o->store_count == 0) o->dump_remaining = 0;
}

static int x_hk_period(obc_t *o, const uint8_t *p, size_t n) { (void)n; o->hk_period_s = p[0]; return ERR_NONE; }

static int x_tle(obc_t *o, const uint8_t *p, size_t n)
{
    (void)n;
    char l1[70], l2[70];
    memcpy(l1, p, 69); l1[69] = 0;
    memcpy(l2, p + 69, 69); l2[69] = 0;
    if (l1[0] != '1' || l2[0] != '2') return ERR_EXEC;
    if (!tle_line_checksum_ok(l1) || !tle_line_checksum_ok(l2)) return ERR_EXEC;
    memcpy(o->tle[0], l1, 70); memcpy(o->tle[1], l2, 70);
    char epoch[15]; memcpy(epoch, l1 + 18, 14); epoch[14] = 0;      /* YYDDD.DDDDDDDD */
    o->tle_epoch = atof(epoch);
    send_event(o, EVT_TLE_UPDATED, (uint32_t)o->tle_epoch);
    return ERR_NONE;
}

static int x_clear_sched(obc_t *o, const uint8_t *p, size_t n)
{
    (void)p; (void)n;
    for (int i = 0; i < SCHED_SLOTS; i++) o->sched[i].used = 0;
    return ERR_NONE;
}

static void enter_safe(obc_t *o, uint32_t why_mv);

static int x_mode(obc_t *o, const uint8_t *p, size_t n)
{
    (void)n;
    if (p[0] == MODE_SAFE) { enter_safe(o, (uint32_t)o->ss.batt_mv); return ERR_NONE; }
    /* leaving SAFE is only allowed with a healthy battery (hysteresis) */
    if (o->ss.batt_mv < o->ss.safe_mv + 300) return ERR_EXEC;
    if (o->mode != MODE_NOMINAL) { o->mode = MODE_NOMINAL; send_event(o, EVT_SAFE_EXIT, (uint32_t)o->ss.batt_mv); }
    return ERR_NONE;
}

static int x_get(obc_t *o, const uint8_t *p, size_t n)
{
    (void)n;
    float v = 0;
    switch (p[0]) {
    case PARAM_WHEEL_RPM: v = (float)o->ss.wheel_rpm; break;
    case PARAM_BATT_MV:   v = (float)o->ss.batt_mv; break;
    case PARAM_MAG_X_NT:  v = (float)o->ss.mag_x_nt; break;
    case PARAM_TLE_EPOCH: v = (float)o->tle_epoch; break;
    case PARAM_TX_POWER:  v = (float)o->ss.tx_power_dbm; break;
    case PARAM_SAFE_MV:   v = (float)o->ss.safe_mv; break;
    default: return ERR_EXEC;
    }
    uint8_t b[8]; cursor_t c; cur_init(&c, b, sizeof b);
    put_u8(&c, p[0]); put_f32(&c, v);
    send_tm(o, APID_TM_ONREQ, now_cuc(o), b, c.pos, 1);
    return ERR_NONE;
}

static int x_wheel(obc_t *o, const uint8_t *p, size_t n)
{
    (void)n;
    if (!o->ss.rail_wheel) return ERR_EXEC;          /* hardware not powered */
    o->ss.wheel_cmd = (int16_t)((p[0] << 8) | p[1]);
    return ERR_NONE;
}

static int x_sstm(obc_t *o, const uint8_t *p, size_t n)
{
    (void)n;
    if (o->sstm.active) return ERR_EXEC;             /* no overlapping sessions */
    uint32_t dur = ((uint32_t)p[1] << 24) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 8) | p[4];
    o->sstm.active = 1;
    o->sstm.rate_hz = p[0];
    o->sstm.end_ms = (uint32_t)(o->now_ms + dur);
    o->sstm.next_ms = (uint32_t)o->now_ms;
    o->sstm.samples = 0;
    o->sstm.nbuf = 0;
    return ERR_NONE;
}

static int x_mtq(obc_t *o, const uint8_t *p, size_t n) { (void)n; o->ss.mtq_on = p[0]; return ERR_NONE; }

static int x_rail(obc_t *o, const uint8_t *p, size_t n)
{
    (void)n;
    if (p[0] == 1) o->ss.rail_payload = p[1];
    else { o->ss.rail_wheel = p[1]; if (!p[1]) o->ss.wheel_cmd = 0; }
    return ERR_NONE;
}
static int x_safe_mv(obc_t *o, const uint8_t *p, size_t n) { (void)n; o->ss.safe_mv = (uint16_t)((p[0] << 8) | p[1]); return ERR_NONE; }
static int x_txp(obc_t *o, const uint8_t *p, size_t n) { (void)n; o->ss.tx_power_dbm = p[0]; return ERR_NONE; }

static const cmd_def_t CMDS[] = {
    /* apid           cmd                    min  max  safe  validate     execute   */
    { APID_TC_OBC,   CMD_NO_OP,             0,   0,   1,    v_none,      x_noop        },
    { APID_TC_OBC,   CMD_DUMP_STM,          2,   2,   1,    v_none,      x_dump        },
    { APID_TC_OBC,   CMD_SET_HK_PERIOD,     1,   1,   1,    v_hk_period, x_hk_period   },
    { APID_TC_OBC,   CMD_TLE_UPLOAD,        138, 138, 0,    v_none,      x_tle         },
    { APID_TC_OBC,   CMD_CLEAR_SCHEDULE,    0,   0,   1,    v_none,      x_clear_sched },
    { APID_TC_OBC,   CMD_SET_MODE,          1,   1,   1,    v_mode,      x_mode        },
    { APID_TC_ADCS,  CMD_ADCS_GET_PARAM,    1,   1,   1,    v_get,       x_get         },
    { APID_TC_ADCS,  CMD_ADCS_SET_WHEEL,    2,   2,   0,    v_wheel,     x_wheel       },
    { APID_TC_ADCS,  CMD_ADCS_SSTM_START,   5,   5,   0,    v_sstm,      x_sstm        },
    { APID_TC_ADCS,  CMD_ADCS_MTQ_ONOFF,    1,   1,   0,    v_bool,      x_mtq         },
    { APID_TC_EPS,   CMD_EPS_SET_RAIL,      2,   2,   0,    v_rail,      x_rail        },
    { APID_TC_EPS,   CMD_EPS_SET_SAFE_MV,   2,   2,   1,    v_safe_mv,   x_safe_mv     },
    { APID_TC_COMMS, CMD_COMMS_TX_POWER,    1,   1,   0,    v_txp,       x_txp         },
};
#define NCMDS (sizeof CMDS / sizeof CMDS[0])

static const cmd_def_t *find_cmd(uint16_t apid, uint16_t id)
{
    for (size_t i = 0; i < NCMDS; i++) if (CMDS[i].apid == apid && CMDS[i].cmd_id == id) return &CMDS[i];
    return NULL;
}

/* ====================================================================== */
/*  The TC pipeline                                                        */
/* ====================================================================== */
static void reject(obc_t *o, const ccsds_packet_t *p, uint8_t code)
{
    o->rej_cnt++;
    send_ack(o, ACK_ACCEPT_FAIL, p->hdr.apid, p->hdr.seq_count, p->cmd_id, code,
             p->has_time ? p->time.coarse : 0, 0);
}

static void execute_and_report(obc_t *o, const cmd_def_t *d, uint16_t apid, uint16_t seq,
                               uint32_t tag, const uint8_t *params, size_t n)
{
    int r = d->execute(o, params, n);
    int keep = tag != 0;                 /* time-tagged: usually runs out of contact */
    if (r == ERR_NONE) { o->exec_cnt++; send_ack(o, ACK_EXEC_OK, apid, seq, d->cmd_id, ERR_NONE, tag, keep); }
    else               { o->rej_cnt++;  send_ack(o, ACK_EXEC_FAIL, apid, seq, d->cmd_id, (uint8_t)r, tag, keep); }
}

static void handle_packet(obc_t *o, const uint8_t *pkt, size_t len)
{
    ccsds_packet_t p;
    int r = ccsds_parse(pkt, len, &p);
    /* Gate 1: packet CRC. A corrupted packet can't be trusted at all, so
     * it is discarded silently (ground will see no ACK and retry). */
    if (r == PKT_ERR_CRC) { o->pkt_crc_err++; return; }
    if (r != PKT_OK) { o->pkt_other_err++; reject(o, &p, ERR_BAD_LENGTH); return; }
    if (p.hdr.type != CCSDS_TYPE_TC) { o->pkt_other_err++; return; }
    o->tc_received++;

    /* Gate 2: APID routing table */
    int ai = tc_apid_index(p.hdr.apid);
    if (ai < 0) { reject(o, &p, ERR_BAD_APID); return; }

    /* Gate 3: duplicate / replay check (same sequence count twice in a row) */
    if (o->last_tc_seq[ai] == (int32_t)p.hdr.seq_count) { reject(o, &p, ERR_DUPLICATE); return; }

    /* Gate 4: known command with the right parameter length */
    const cmd_def_t *d = find_cmd(p.hdr.apid, p.cmd_id);
    if (!d) { reject(o, &p, ERR_BAD_CMD); return; }
    if (p.data_len < d->min_len || p.data_len > d->max_len) { reject(o, &p, ERR_BAD_LENGTH); return; }

    /* Gate 5: allowed in the current mode? */
    if (o->mode == MODE_SAFE && !d->allowed_in_safe) { reject(o, &p, ERR_MODE); return; }

    /* Gate 6: parameter ranges (checked at acceptance, before anything moves) */
    int v = d->validate(o, p.data, p.data_len);
    if (v != ERR_NONE) { reject(o, &p, (uint8_t)v); return; }

    /* Gate 7: time tag sanity and a free schedule slot */
    int slot = -1;
    if (p.has_time) {
        uint64_t when = cuc_to_ms(p.time);
        if (when + 1000 < o->now_ms) { reject(o, &p, ERR_TIME_PAST); return; }
        for (int i = 0; i < SCHED_SLOTS; i++) if (!o->sched[i].used) { slot = i; break; }
        if (slot < 0) { reject(o, &p, ERR_SCHED_FULL); return; }
    }

    /* Accepted: remember the sequence count, count it, tell the ground. */
    o->last_tc_seq[ai] = p.hdr.seq_count;
    o->acc_cnt++;
    uint32_t tag = p.has_time ? p.time.coarse : 0;
    send_ack(o, ACK_ACCEPT_OK, p.hdr.apid, p.hdr.seq_count, p.cmd_id, ERR_NONE, tag, 0);

    if (p.has_time) {
        sched_entry_t *e = &o->sched[slot];
        e->used = 1; e->exec_ms = cuc_to_ms(p.time);
        e->apid = p.hdr.apid; e->seq = p.hdr.seq_count; e->cmd_id = p.cmd_id;
        e->tag_coarse = tag; e->params_len = (uint16_t)p.data_len;
        memcpy(e->params, p.data, p.data_len);
    } else {
        execute_and_report(o, d, p.hdr.apid, p.hdr.seq_count, 0, p.data, p.data_len);
    }
}

static void on_frame(const uint8_t *frame, size_t len, void *user)
{
    obc_t *o = (obc_t *)user;
    if (len <= AX25_HDR_LEN) return;
    if (memcmp(frame, o->addr_sat, 7) != 0) return;      /* not addressed to us */
    handle_packet(o, frame + AX25_HDR_LEN, len - AX25_HDR_LEN);
}

/* ====================================================================== */
/*  Scheduler: run time-tagged commands when their time comes              */
/* ====================================================================== */
static void scheduler_poll(obc_t *o)
{
    for (int i = 0; i < SCHED_SLOTS; i++) {
        sched_entry_t *e = &o->sched[i];
        if (!e->used || o->now_ms < e->exec_ms) continue;   /* ">=" never misses a slot */
        e->used = 0;
        const cmd_def_t *d = find_cmd(e->apid, e->cmd_id);
        if (!d) continue;
        if (o->mode == MODE_SAFE && !d->allowed_in_safe) {      /* re-check at execution */
            o->rej_cnt++;
            send_ack(o, ACK_EXEC_FAIL, e->apid, e->seq, e->cmd_id, ERR_ABORTED_SAFE, e->tag_coarse, 1);
            continue;
        }
        execute_and_report(o, d, e->apid, e->seq, e->tag_coarse, e->params, e->params_len);
    }
}

/* ====================================================================== */
/*  Subsystem physics and FDIR                                             */
/* ====================================================================== */
#define ORBIT_S       5400.0
#define SUNLIT_S      3300.0
#define BATT_MWH      20000.0

static void enter_safe(obc_t *o, uint32_t mv)
{
    if (o->mode == MODE_SAFE) return;
    o->mode = MODE_SAFE;
    o->ss.rail_payload = 0;               /* shed load */
    o->ss.wheel_cmd = 0;
    o->ss.mtq_on = 0;
    if (o->sstm.active) o->sstm.active = 0;
    send_event(o, EVT_SAFE_ENTER, mv);
}

static void physics(obc_t *o, double dt_s)
{
    subsys_t *s = &o->ss;
    double t = (double)o->now_ms / 1000.0;
    double phase = fmod(t, ORBIT_S);
    s->eclipse = phase >= SUNLIT_S;
    s->solar_mw = s->eclipse ? 0.0 : 7000.0;

    s->load_mw = 2600.0                                         /* OBC + receiver   */
               + (s->rail_payload ? 3800.0 : 0.0)
               + (s->rail_wheel ? 300.0 + 0.12 * fabs(s->wheel_rpm) : 0.0)
               + (s->mtq_on ? 500.0 : 0.0)
               + 30.0 * (s->tx_power_dbm - 20);                 /* bigger PA, more power */

    double net_mw = s->solar_mw * 0.92 - s->load_mw;            /* charge efficiency */
    s->soc += net_mw * dt_s / 3600.0 / BATT_MWH;
    if (s->soc > 1.0) s->soc = 1.0;
    if (s->soc < 0.0) s->soc = 0.0;
    s->batt_mv = 6000.0 + 2400.0 * s->soc - 0.04 * s->load_mw + (s->eclipse ? 0 : 60);

    /* reaction wheel: rate-limited first-order response, coasts down unpowered */
    double target = s->rail_wheel ? s->wheel_cmd : 0.0;
    double tau = s->rail_wheel ? 0.8 : 30.0;
    double dw = (target - s->wheel_rpm) * dt_s / tau;
    double lim = 1500.0 * dt_s;
    if (dw > lim) dw = lim;
    if (dw < -lim) dw = -lim;
    s->wheel_rpm += dw;

    /* magnetometer: field changes around the orbit, magnetorquer adds a bias */
    s->mag_x_nt = 28000.0 * sin(2.0 * M_PI * 2.0 * t / ORBIT_S) + (s->mtq_on ? 2500.0 : 0.0) + noise(o, 60.0);

    /* temperature: warms in sunlight, cools in eclipse */
    double t_target = s->eclipse ? -8.0 : 28.0;
    s->temp_c += (t_target - s->temp_c) * dt_s / 900.0;

    /* FDIR: battery under threshold for 3 s continuously -> SAFE mode */
    if (s->batt_mv < s->safe_mv) s->low_v_ms += (uint32_t)(dt_s * 1000.0); else s->low_v_ms = 0;
    if (o->mode == MODE_NOMINAL && s->low_v_ms >= 3000) enter_safe(o, (uint32_t)s->batt_mv);
}

/* ====================================================================== */
/*  Housekeeping and SSTM                                                  */
/* ====================================================================== */
static size_t build_hk(obc_t *o, uint8_t *b, size_t cap)
{
    subsys_t *s = &o->ss;
    int used = 0; for (int i = 0; i < SCHED_SLOTS; i++) used += o->sched[i].used;
    cursor_t c; cur_init(&c, b, cap);
    put_u8(&c, o->mode);
    put_u16(&c, (uint16_t)s->batt_mv);
    put_u8(&c, (uint8_t)(s->soc * 100.0 + 0.5));
    put_u16(&c, (uint16_t)s->solar_mw);
    put_u16(&c, (uint16_t)s->load_mw);
    put_i16(&c, (int16_t)lround(s->wheel_rpm));
    put_i16(&c, s->wheel_cmd);
    put_u8(&c, (uint8_t)(s->rail_payload | (s->rail_wheel << 1) | (s->mtq_on << 2)));
    put_i16(&c, (int16_t)lround(s->mag_x_nt / 10.0));
    put_u16(&c, o->acc_cnt); put_u16(&c, o->exec_cnt); put_u16(&c, o->rej_cnt);
    put_u8(&c, (uint8_t)used);
    put_u16(&c, (uint16_t)o->store_count);
    put_u8(&c, (uint8_t)o->sstm.active);
    put_u8(&c, (uint8_t)s->eclipse);
    put_i16(&c, (int16_t)lround(s->temp_c * 10.0));
    put_u8(&c, s->tx_power_dbm);
    return c.pos;
}

static void sstm_flush(obc_t *o)
{
    if (o->sstm.nbuf == 0) return;
    store_put(o, APID_TM_SSTM, o->sstm.first_t, o->sstm.buf, (size_t)o->sstm.nbuf * SSTM_SAMPLE_BYTES);
    o->sstm.nbuf = 0;
}

static void sstm_task(obc_t *o)
{
    sstm_t *x = &o->sstm;
    if (!x->active) return;
    if (o->now_ms >= x->next_ms) {
        if (x->nbuf == 0) x->first_t = now_cuc(o);
        cursor_t c; cur_init(&c, x->buf + x->nbuf * SSTM_SAMPLE_BYTES, SSTM_SAMPLE_BYTES);
        put_u32(&c, (uint32_t)o->now_ms);
        put_i16(&c, (int16_t)lround(o->ss.mag_x_nt / 10.0));
        put_i16(&c, (int16_t)lround(o->ss.wheel_rpm));
        put_i16(&c, (int16_t)lround(o->ss.temp_c * 10.0));
        put_u16(&c, (uint16_t)o->ss.batt_mv);
        x->nbuf++; x->samples++;
        x->next_ms += 1000u / x->rate_hz;          /* rate validated 1..50, never 0 */
        if (x->nbuf == SSTM_PER_PACKET) sstm_flush(o);
    }
    if (o->now_ms >= x->end_ms) {
        sstm_flush(o);
        x->active = 0;
        send_event(o, EVT_SSTM_DONE, x->samples);
    }
}

/* ====================================================================== */
/*  Transmitter: two priority queues, limited by the link bit rate         */
/* ====================================================================== */
static void transmitter(obc_t *o, bitwriter_t *out, double dt_s)
{
    o->tx_budget_bits += LINK_BITRATE_BPS * dt_s;
    if (o->tx_budget_bits > 4000) o->tx_budget_bits = 4000;
    for (;;) {
        if (!o->tx_hi.count) dump_feed(o);
        txq_t *q = o->tx_hi.count ? &o->tx_hi : (o->tx_lo.count ? &o->tx_lo : NULL);
        if (!q) break;
        pkt_buf_t *pb = &q->q[q->head];
        double cost = (AX25_HDR_LEN + pb->len + FRAME_FCS_LEN) * 8.0 * 1.03 + 16.0;
        if (cost > o->tx_budget_bits) break;
        size_t before = out->nbits;
        hdlc_encode_frame(out, o->addr_ground, o->addr_sat, pb->bytes, pb->len);
        if (out->overflow) { out->nbits = before; out->overflow = 0; break; }
        o->tx_budget_bits -= (double)(out->nbits - before);
        q->head = (q->head + 1) % TXQ_SLOTS; q->count--;
        o->frames_sent++;
    }
}

/* ====================================================================== */
/*  Public API                                                             */
/* ====================================================================== */
void obc_init(obc_t *o)
{
    memset(o, 0, sizeof *o);
    o->mode = MODE_NOMINAL;
    for (int i = 0; i < 4; i++) o->last_tc_seq[i] = -1;
    o->hk_period_s = 10;
    o->rng = 0x12345678u;
    o->ss.soc = 0.62;
    o->ss.rail_wheel = 1;
    o->ss.safe_mv = 6600;
    o->ss.temp_c = 20.0;
    o->ss.tx_power_dbm = 27;
    ax25_addr(o->addr_ground, "GROUND", 0);
    ax25_addr(o->addr_sat, "MINSAT", 1);
    hdlc_rx_init(&o->rx, on_frame, o);
    physics(o, 0.0);
}

void obc_receive_bits(obc_t *o, const uint8_t *packed, size_t nbytes)
{
    hdlc_rx_bytes(&o->rx, packed, nbytes);
}

void obc_tick(obc_t *o, bitwriter_t *out)
{
    double dt = TICK_MS / 1000.0;
    o->now_ms += TICK_MS;
    physics(o, dt);
    scheduler_poll(o);
    sstm_task(o);
    if (o->now_ms >= o->next_hk_ms) {
        uint8_t hk[64];
        size_t n = build_hk(o, hk, sizeof hk);
        send_tm(o, APID_TM_HK, now_cuc(o), hk, n, 0);       /* real-time copy        */
        store_put(o, APID_TM_STORED_HK, now_cuc(o), hk, n);  /* stored copy for dump  */
        o->next_hk_ms = o->now_ms + (uint64_t)o->hk_period_s * 1000u;
    }
    transmitter(o, out, dt);
}
