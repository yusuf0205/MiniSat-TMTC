/*
 * obc_sim_main.c - runs the flight software on a PC, in lock-step with the
 * ground-station simulator (software-in-the-loop).
 *
 * stdin  (from ground): records  [u32 t_ms][u32 nbytes][nbytes of uplink bits]
 * stdout (to ground)  : records  [u32 t_ms][u32 nbytes][nbytes of downlink bits]
 * For each record the OBC first runs its 10 ms ticks up to t_ms (collecting
 * the downlink bits it transmits), then receives the uplink bits.
 *
 * argv[1] (optional): truth CSV, written once per simulated second
 * argv[2] (optional): final statistics (JSON)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "obc.h"

static int read_exact(void *dst, size_t n)
{
    return fread(dst, 1, n, stdin) == n;
}
static uint32_t be32(const uint8_t *b) { return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3]; }
static void wr32(uint32_t v) { uint8_t b[4] = {(uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v}; fwrite(b, 1, 4, stdout); }

static obc_t g_obc;                                /* big: keep it off the stack */
static uint8_t g_up[1 << 16], g_down[1 << 17];

int main(int argc, char **argv)
{
    FILE *truth = argc > 1 ? fopen(argv[1], "w") : NULL;
    obc_t *o = &g_obc;
    obc_init(o);
    if (truth) fprintf(truth, "t_s,mode,batt_mv,soc_pct,solar_mw,load_mw,wheel_rpm,wheel_cmd,eclipse,temp_c,mag_x_nt,stored,acc,exec,rej,sched\n");

    uint8_t hdr[8];
    while (read_exact(hdr, 8)) {
        uint32_t t_ms = be32(hdr), n = be32(hdr + 4);
        if (n > sizeof g_up || !read_exact(g_up, n)) break;

        bitwriter_t w; bw_init(&w, g_down, sizeof g_down);
        while (o->now_ms < t_ms) {
            obc_tick(o, &w);
            if (truth && o->now_ms % 1000 == 0) {
                int used = 0; for (int i = 0; i < SCHED_SLOTS; i++) used += o->sched[i].used;
                fprintf(truth, "%llu,%d,%.0f,%.1f,%.0f,%.0f,%.1f,%d,%d,%.2f,%.0f,%d,%u,%u,%u,%d\n",
                        (unsigned long long)(o->now_ms / 1000), o->mode, o->ss.batt_mv, o->ss.soc * 100,
                        o->ss.solar_mw, o->ss.load_mw, o->ss.wheel_rpm, o->ss.wheel_cmd, o->ss.eclipse,
                        o->ss.temp_c, o->ss.mag_x_nt, o->store_count, o->acc_cnt, o->exec_cnt, o->rej_cnt, used);
            }
        }
        size_t nb = bw_finish(&w);
        wr32(t_ms); wr32((uint32_t)nb); fwrite(g_down, 1, nb, stdout); fflush(stdout);

        obc_receive_bits(o, g_up, n);
    }

    if (argc > 2) {
        FILE *f = fopen(argv[2], "w");
        if (f) {
            fprintf(f, "{\"frames_ok\":%u,\"frame_fcs_errors\":%u,\"frame_aborts\":%u,\"frame_runts\":%u,"
                       "\"pkt_crc_errors\":%u,\"pkt_other_errors\":%u,\"tc_received\":%u,"
                       "\"accepted\":%u,\"executed\":%u,\"rejected\":%u,\"frames_sent\":%u,"
                       "\"txq_dropped\":%u,\"store_overwritten\":%u}\n",
                    o->rx.frames_ok, o->rx.fcs_errors, o->rx.aborts, o->rx.runts,
                    o->pkt_crc_err, o->pkt_other_err, o->tc_received,
                    o->acc_cnt, o->exec_cnt, o->rej_cnt, o->frames_sent,
                    o->tx_hi.dropped + o->tx_lo.dropped, o->store_overwritten);
            fclose(f);
        }
    }
    if (truth) fclose(truth);
    return 0;
}
