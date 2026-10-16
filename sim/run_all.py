"""
run_all.py - run every simulation and produce the results folder.

    python3 sim/run_all.py          (after building flight_sw)

Outputs (in results/): CSV tables, PNG plots and summary.json.
"""
import csv
import json
import os
import random
import struct
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "ground"))
import scenarios                          # noqa: E402
import link                               # noqa: E402
from icd import ICD                       # noqa: E402

OUT = os.path.join(HERE, "..", "results")
os.makedirs(OUT, exist_ok=True)
plt.rcParams.update({"figure.dpi": 130, "font.size": 9, "axes.grid": True, "grid.alpha": 0.3})


def write_csv(name, rows):
    if not rows:
        return
    keys = [k for k in rows[0].keys() if not k.startswith("_")]
    with open(os.path.join(OUT, name), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def shade(ax, truth):
    """Grey = eclipse, green = ground pass, red = SAFE mode."""
    for a, b in scenarios.PASSES:
        ax.axvspan(a / 3600, b / 3600, color="tab:green", alpha=0.18, lw=0)
    for key, color in (("eclipse", "0.6"), ("mode_safe", "tab:red")):
        start = None
        for r in truth:
            on = r["eclipse"] == "1" if key == "eclipse" else r["mode"] == "0"
            t = int(r["t_s"]) / 3600
            if on and start is None:
                start = t
            if not on and start is not None:
                ax.axvspan(start, t, color=color, alpha=0.15 if key == "eclipse" else 0.10, lw=0)
                start = None
        if start is not None:
            ax.axvspan(start, int(truth[-1]["t_s"]) / 3600, color=color, alpha=0.1, lw=0)


# ====================================================================== mission
def mission():
    g, plan = scenarios.mission_day(OUT)
    truth = list(csv.DictReader(open(os.path.join(OUT, "truth.csv"))))
    events = sorted({(e["t_s"], e["event"], e["value"]) for e in g.events})
    events = [{"t_s": t, "event": e, "value": v} for t, e, v in events]
    write_csv("tc_log.csv", g.tc_log)
    write_csv("hk_received.csv", sorted(g.hk, key=lambda h: (h["t_s"], h["source"])))
    write_csv("events.csv", events)
    write_csv("sstm_samples.csv", g.sstm)
    write_csv("onrequest_tm.csv", g.onreq)
    write_csv("operations_plan.csv", plan)

    rt = [h for h in g.hk if h["source"] == "RT"]
    dump = [h for h in g.hk if h["source"] == "DUMP"]
    ts = [int(r["t_s"]) / 3600 for r in truth]

    # ---- Figure 1: mission overview
    fig, ax = plt.subplots(3, 1, figsize=(10, 7.5), sharex=True)
    shade(ax[0], truth)
    ax[0].plot(ts, [float(r["batt_mv"]) for r in truth], "k-", lw=0.8, label="True battery voltage (simulator)")
    ax[0].plot([h["t_s"] / 3600 for h in dump], [h["batt_mv"] for h in dump], ".", ms=2, color="tab:blue",
               label="Learned later from stored-TM dumps")
    ax[0].plot([h["t_s"] / 3600 for h in rt], [h["batt_mv"] for h in rt], "o", ms=3, color="tab:orange",
               label="Seen live during passes")
    ax[0].axhline(6600, color="tab:red", ls="--", lw=1, label="Safe-mode threshold 6600 mV")
    ax[0].set_ylabel("Battery (mV)")
    ax[0].legend(loc="lower left", fontsize=7, ncol=2)
    ax[0].set_title("A 6.4-hour mission day: green = ground pass, grey = eclipse, red = SAFE mode")

    shade(ax[1], truth)
    ax[1].plot(ts, [float(r["load_mw"]) / 1000 for r in truth], color="tab:purple", lw=0.8, label="Power used (W)")
    ax[1].plot(ts, [float(r["solar_mw"]) / 1000 for r in truth], color="goldenrod", lw=0.8, label="Solar power (W)")
    ax[1].set_ylabel("Power (W)")
    ax[1].legend(loc="upper right", fontsize=7)

    shade(ax[2], truth)
    ax[2].plot(ts, [float(r["wheel_rpm"]) for r in truth], "k-", lw=0.8, label="Wheel speed (truth)")
    ax[2].plot([h["t_s"] / 3600 for h in dump], [h["wheel_rpm"] for h in dump], ".", ms=2, color="tab:blue",
               label="From dumps")
    for r in g.tc_log:
        if r["time_tag_s"] and r.get("exec_obc_s"):
            ok = r["exec"] == "OK"
            ax[2].axvline(r["exec_obc_s"] / 3600, color="tab:green" if ok else "tab:red", lw=1, ls=":")
            ax[2].text(r["exec_obc_s"] / 3600, 3600, ("TT ok" if ok else "TT aborted"), fontsize=6, rotation=90,
                       color="tab:green" if ok else "tab:red", va="top")
    ax[2].set_ylabel("Wheel (rpm)")
    ax[2].set_xlabel("Mission time (hours)")
    ax[2].legend(loc="lower right", fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig1_mission_overview.png"))
    plt.close(fig)

    # ---- Figure 2: SSTM wheel step test
    fig, ax = plt.subplots(figsize=(8, 3.6))
    s = [x for x in g.sstm if x["t_s"] < 40]
    ax.plot([x["t_s"] for x in s], [x["wheel_rpm"] for x in s], ".-", ms=3, lw=0.8,
            label=f"SSTM samples at 20 Hz ({len(s)} samples)")
    hk_early = [h for h in rt if h["t_s"] < 40]
    ax.plot([h["t_s"] for h in hk_early], [h["wheel_rpm"] for h in hk_early], "s", ms=6, color="tab:orange",
            label="Normal housekeeping (every 10 s)")
    for t, lab in ((23.1, "SET +3000 rpm"), (29.1, "SET -2000 rpm")):
        ax.axvline(t, color="tab:red", ls="--", lw=1)
        ax.text(t + 0.2, -1200 if t < 25 else 2500, lab, fontsize=7, color="tab:red")
    ax.set_xlabel("Mission time (s)")
    ax.set_ylabel("Wheel speed (rpm)")
    ax.set_title("Why super-sampled telemetry exists: the 10 s housekeeping misses the whole manoeuvre")
    ax.legend(fontsize=7, loc="lower left")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig2_sstm_wheel_step.png"))
    plt.close(fig)

    # ---- Figure 3: every command and what happened to it
    fig, ax = plt.subplots(figsize=(9, 8))
    names, colors, labels = [], [], []
    for r in g.tc_log:
        if r["accept"] != "OK":
            c, lab = "tab:red", f"REJECTED at acceptance: {r['accept_code']}"
        elif r["exec"] == "OK":
            c, lab = "tab:green", "accepted + executed"
        elif r["exec"] == "FAIL":
            c, lab = "tab:orange", f"accepted, execution FAILED: {r['exec_code']}"
        else:
            c, lab = "0.6", "accepted, no execution report"
        names.append(r["name"])
        colors.append(c)
        labels.append(lab)
    y = list(range(len(names)))[::-1]
    ax.barh(y, [1] * len(names), color=colors)
    for yi, lab in zip(y, labels):
        ax.text(0.02, yi, lab, va="center", fontsize=6.5, color="white" if lab.startswith(("REJ", "acc")) else "k")
    ax.set_yticks(y)
    ax.set_yticklabels(names, fontsize=6.5)
    ax.set_xticks([])
    ax.set_title(f"All {len(names)} telecommands sent in the mission and their verification result")
    ax.grid(False)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig3_tc_outcomes.png"))
    plt.close(fig)

    stats = json.load(open(os.path.join(OUT, "obc_stats_mission.json")))
    summary = {
        "mission_duration_h": round(int(truth[-1]["t_s"]) / 3600, 2),
        "passes": len(scenarios.PASSES),
        "telecommands_sent": len(g.tc_log),
        "accepted": sum(r["accept"] == "OK" for r in g.tc_log),
        "rejected_at_acceptance": sum(r["accept"] == "FAIL" for r in g.tc_log),
        "executed_ok": sum(r["exec"] == "OK" for r in g.tc_log),
        "execution_failed": sum(r["exec"] == "FAIL" for r in g.tc_log),
        "rejection_reasons": sorted({r["accept_code"] for r in g.tc_log if r["accept"] == "FAIL"}),
        "hk_packets_live": len(rt),
        "hk_packets_from_dumps": len(dump),
        "hk_live_coverage_pct": round(100 * len({round(h["t_s"]) for h in rt}) / max(len({round(h["t_s"]) for h in g.hk}), 1), 1),
        "sstm_samples": len(g.sstm),
        "events": events,
        "min_battery_mv": min(float(r["batt_mv"]) for r in truth),
        "obc_counters": stats,
    }
    return summary


# ====================================================================== BER
def frame_bits(pkt_len):
    """Exact number of bits on the wire for a frame, stuffing included (typical data)."""
    return len(link.hdlc_frame_bits(link.ADDR_SAT, link.ADDR_GROUND, bytes(pkt_len)))


def ber():
    rows = scenarios.ber_sweep(OUT)
    write_csv("ber_sweep.csv", rows)
    tc_bits = frame_bits(6 + 2 + 2)                       # NO_OP packet
    ack_bits = frame_bits(6 + 7 + 12 + 2)                 # acceptance ACK packet
    hk_bits = frame_bits(6 + 7 + 29 + 2)                  # housekeeping packet
    bers = [r["ber"] for r in rows]
    fine = [i * 1e-5 for i in range(0, 101)]
    fig, ax = plt.subplots(figsize=(8, 4.2))
    ax.plot([b * 1e3 for b in fine], [100 * (1 - b) ** hk_bits for b in fine], "-", color="tab:orange", lw=1,
            label=f"Theory: one {hk_bits}-bit HK frame survives  (1-BER)^{hk_bits}")
    ax.plot([b * 1e3 for b in bers], [r["hk_received_pct"] for r in rows], "o", color="tab:orange",
            label="Measured: housekeeping packets received")
    ax.plot([b * 1e3 for b in fine], [100 * (1 - b) ** (tc_bits + ack_bits) for b in fine], "-", color="tab:blue",
            lw=1, label=f"Theory: command + its ACK both survive  (1-BER)^{tc_bits + ack_bits}")
    ax.plot([b * 1e3 for b in bers], [r["tc_ok_first_try_pct"] for r in rows], "s", color="tab:blue",
            label="Measured: command confirmed on first try")
    ax.plot([b * 1e3 for b in bers], [r["tc_ok_after_retries_pct"] for r in rows], "D-", color="tab:green",
            label="Measured: command confirmed after automatic retries")
    ax.set_xlabel("Bit error rate (errors per 1000 bits)")
    ax.set_ylabel("Success (%)")
    ax.set_ylim(0, 105)
    ax.set_title("Noisy radio link: CRC throws bad frames away, retries win them back")
    ax.legend(fontsize=7, loc="lower left")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig4_ber_sweep.png"))
    plt.close(fig)
    return {"rows": rows, "tc_frame_bits": tc_bits, "ack_frame_bits": ack_bits, "hk_frame_bits": hk_bits}


# ====================================================================== CRC strength
def crc_experiment(trials=200000, seed=3):
    """How good is CRC-16 at catching damage? Corrupt random 44-byte packets
    and count how often the damage is NOT detected."""
    table = []
    for i in range(256):
        c = i << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
        table.append(c)

    def crc(data):
        c = 0xFFFF
        for b in data:
            c = ((c << 8) & 0xFFFF) ^ table[((c >> 8) ^ b) & 0xFF]
        return c

    assert crc(b"123456789") == 0x29B1
    rnd = random.Random(seed)
    res = {}
    for label, nflips in (("1 bit flipped", 1), ("2 bits flipped", 2), ("3 bits flipped", 3),
                          ("burst of 16 bits", "burst16"), ("heavy damage (8-20 random bits)", "heavy")):
        missed = 0
        n = trials if nflips == "heavy" else trials // 4
        for _ in range(n):
            data = bytearray(rnd.getrandbits(8) for _ in range(42))
            pkt = bytes(data) + struct.pack(">H", crc(data))
            bad = bytearray(pkt)
            nbits = len(bad) * 8
            if nflips == "burst16":
                start = rnd.randrange(nbits - 16)
                positions = [start] + [start + 15] + [start + k for k in range(1, 15) if rnd.random() < 0.5]
            elif nflips == "heavy":
                positions = rnd.sample(range(nbits), rnd.randint(8, 20))
            else:
                positions = rnd.sample(range(nbits), nflips)
            for p in positions:
                bad[p // 8] ^= 0x80 >> (p % 8)
            if crc(bad[:-2]) == struct.unpack(">H", bad[-2:])[0]:
                missed += 1
        res[label] = {"trials": n, "undetected": missed}
        print(f"CRC experiment: {label}: {missed} undetected out of {n}")
    return res


if __name__ == "__main__":
    summary = {"mission": mission()}
    print("mission done")
    summary["ber"] = ber()
    summary["crc"] = crc_experiment()
    with open(os.path.join(OUT, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary["mission"], indent=2)[:2000])
