"""
scenarios.py - the two simulation campaigns.

1. mission_day(): ~6.4 hours (4+ orbits), 5 ground passes, a realistic
   operations plan: checkout, SSTM step test, deliberately bad commands,
   TLE upload, time-tagged commands that run out of contact, a battery
   drain that triggers FDIR safe mode, recovery, and stored-TM dumps.
2. ber_sweep(): the same link at increasing bit-error rates, to measure
   how framing + CRC + retries cope with a noisy radio.
"""
import json
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "ground"))

from station import GroundStation          # noqa: E402
from icd import ICD                        # noqa: E402

OBC, ADCS, EPS, COMMS = (ICD["APID_TC_OBC"], ICD["APID_TC_ADCS"], ICD["APID_TC_EPS"], ICD["APID_TC_COMMS"])
TLE1 = "1 25544U 98067A   08264.51782528 -.00002182  00000-0 -11606-4 0  2927"
TLE2 = "2 25544  51.6416 247.4627 0006703 130.5360 325.0288 15.72125391563537"
PASSES = [(0, 600), (5800, 6400), (11300, 11900), (16900, 17500), (22300, 22900)]


def mission_day(out_dir):
    g = GroundStation(passes=PASSES, truth_csv=os.path.join(out_dir, "truth.csv"),
                      stats_json=os.path.join(out_dir, "obc_stats_mission.json"), seed=7)
    plan = []                                   # operator's notes, for the report

    def at(t, note):
        g.run_until(t)
        plan.append({"t_s": t, "note": note})

    # ---------------- Pass 1: checkout ----------------
    at(5, "Link check and first readings")
    g.send("No-op link check", OBC, ICD["CMD_NO_OP"])
    g.send("Read battery voltage", ADCS, ICD["CMD_ADCS_GET_PARAM"], bytes([ICD["PARAM_BATT_MV"]]))
    g.send("Read magnetometer X", ADCS, ICD["CMD_ADCS_GET_PARAM"], bytes([ICD["PARAM_MAG_X_NT"]]))

    at(20, "Wheel step test recorded at 20 Hz by SSTM")
    g.send("SSTM 20 Hz for 15 s", ADCS, ICD["CMD_ADCS_SSTM_START"], struct.pack(">BI", 20, 15000))
    at(23, "Wheel to +3000 rpm")
    g.send("Wheel +3000 rpm", ADCS, ICD["CMD_ADCS_SET_WHEEL"], struct.pack(">h", 3000))
    at(26, "A second SSTM while the first is still running must fail")
    g.send("Second SSTM while one runs", ADCS, ICD["CMD_ADCS_SSTM_START"], struct.pack(">BI", 10, 2000))
    at(29, "Wheel to -2000 rpm")
    g.send("Wheel -2000 rpm", ADCS, ICD["CMD_ADCS_SET_WHEEL"], struct.pack(">h", -2000))

    at(45, "Robustness: commands that must be refused")
    g.send("Wheel 9000 rpm (out of range)", ADCS, ICD["CMD_ADCS_SET_WHEEL"], struct.pack(">h", 9000))
    g.send("Unknown command 0x00FF", ADCS, 0x00FF)
    g.send("Wrong APID 0x150", 0x150, ICD["CMD_NO_OP"])
    g.send("SSTM rate 0 Hz (invalid)", ADCS, ICD["CMD_ADCS_SSTM_START"], struct.pack(">BI", 0, 1000))
    noop = g.send("No-op (original)", OBC, ICD["CMD_NO_OP"])
    g.run_for(3)
    g.send("No-op replayed (same sequence count)", OBC, ICD["CMD_NO_OP"], seq=noop["seq"])

    at(60, "Orbit data upload")
    g.send("TLE upload (valid)", OBC, ICD["CMD_TLE_UPLOAD"], (TLE1 + TLE2).encode())
    bad = TLE1[:20] + "9" + TLE1[21:]
    g.send("TLE upload (1 digit corrupted)", OBC, ICD["CMD_TLE_UPLOAD"], (bad + TLE2).encode())
    g.send("Read TLE epoch", ADCS, ICD["CMD_ADCS_GET_PARAM"], bytes([ICD["PARAM_TLE_EPOCH"]]))

    at(75, "Payload on (heavy load) + time-tagged plan for out-of-contact")
    g.send("Payload rail ON", EPS, ICD["CMD_EPS_SET_RAIL"], bytes([1, 1]))
    g.send("TT: magnetorquer ON at t=2000 s", ADCS, ICD["CMD_ADCS_MTQ_ONOFF"], bytes([1]), time_tag_s=2000)
    g.send("TT: magnetorquer OFF at t=2600 s", ADCS, ICD["CMD_ADCS_MTQ_ONOFF"], bytes([0]), time_tag_s=2600)
    g.send("TT: wheel 1500 rpm at t=3000 s", ADCS, ICD["CMD_ADCS_SET_WHEEL"], struct.pack(">h", 1500), time_tag_s=3000)
    g.send("TT: wheel 4000 rpm at t=9000 s", ADCS, ICD["CMD_ADCS_SET_WHEEL"], struct.pack(">h", 4000), time_tag_s=9000)
    g.send("TT: tag already in the past", OBC, ICD["CMD_NO_OP"], time_tag_s=10)

    at(120, "Dump stored telemetry")
    g.send("Dump stored TM", OBC, ICD["CMD_DUMP_STM"], struct.pack(">H", 1000))

    # ---------------- Passes 2..5: routine operations ----------------
    for i, (a, b) in enumerate(PASSES[1:], start=2):
        at(a + 10, f"Pass {i}: dump everything recorded since the last pass")
        g.send(f"Pass {i}: dump stored TM", OBC, ICD["CMD_DUMP_STM"], struct.pack(">H", 1000))
        g.run_for(60)
        hk = g.latest_hk()
        if hk and hk["mode"] == ICD["MODE_SAFE"]:
            plan.append({"t_s": g.t_ms / 1000, "note": f"Pass {i}: satellite found in SAFE mode ({hk['batt_mv']} mV)"})
            g.send(f"Pass {i}: wheel 1000 rpm while SAFE", ADCS, ICD["CMD_ADCS_SET_WHEEL"], struct.pack(">h", 1000))
            g.send(f"Pass {i}: request NOMINAL mode", OBC, ICD["CMD_SET_MODE"], bytes([1]))
            g.run_for(10)
        else:
            g.send(f"Pass {i}: read battery", ADCS, ICD["CMD_ADCS_GET_PARAM"], bytes([ICD["PARAM_BATT_MV"]]))
            g.send(f"Pass {i}: wheel 2000 rpm", ADCS, ICD["CMD_ADCS_SET_WHEEL"], struct.pack(">h", 2000))
        g.run_until(b)

    g.run_until(PASSES[-1][1] + 5)
    g.close()
    return g, plan


def ber_sweep(out_dir, bers=(0, 1e-5, 3e-5, 1e-4, 2e-4, 4e-4, 7e-4, 1e-3), duration_s=1200):
    rows = []
    for k, ber in enumerate(bers):
        stats = os.path.join(out_dir, f"_stats_ber_{k}.json")
        g = GroundStation(passes=None, ber_up=ber, ber_down=ber, seed=100 + k, stats_json=stats,
                          ack_timeout_s=3.0, max_retries=4)
        g.send("HK every 1 s", OBC, ICD["CMD_SET_HK_PERIOD"], bytes([1]))
        g.run_for(5)
        sent = 0
        while g.t_ms < (duration_s - 20) * 1000:
            g.send("No-op", OBC, ICD["CMD_NO_OP"])
            sent += 1
            g.run_for(2.5)
        g.run_for(20)
        g.close()
        with open(stats) as f:
            st = json.load(f)
        os.remove(stats)
        noops = [r for r in g.tc_log if r["name"] == "No-op"]
        first = sum(1 for r in noops if r["accept"] == "OK" and r["tries"] == 1)
        final = sum(1 for r in noops if r["accept"] == "OK")
        tries = sum(r["tries"] for r in noops)
        rt = [h for h in g.hk if h["source"] == "RT"]
        hk_rx = len(rt)
        hk_expected = max(h["seq"] for h in rt) - min(h["seq"] for h in rt) + 1   # from sequence counts
        rows.append({
            "ber": ber, "tc_sent": sent, "tc_transmissions": tries,
            "tc_ok_first_try_pct": 100 * first / sent, "tc_ok_after_retries_pct": 100 * final / sent,
            "uplink_frames_ok": st["frames_ok"], "uplink_frames_bad": st["frame_fcs_errors"] + st["frame_runts"] + st["frame_aborts"],
            "hk_received_pct": 100 * min(hk_rx / hk_expected, 1.0),
            "down_fcs_errors": g.deframer.fcs_errors, "bits_flipped": g.bits_flipped_up + g.bits_flipped_down,
        })
        print(f"BER {ber:g}: first-try {rows[-1]['tc_ok_first_try_pct']:.1f}%  after retries "
              f"{rows[-1]['tc_ok_after_retries_pct']:.1f}%  HK {rows[-1]['hk_received_pct']:.1f}%")
    return rows
