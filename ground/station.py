"""
station.py - the ground station simulator.

* Starts the real C flight software (obc_sim) as a child process and runs
  it in lock-step (software-in-the-loop).
* Models the radio link: ground passes (link only up when the satellite is
  in view), 9600 bit/s uplink, and random bit errors (BER) both ways.
* Sends telecommands, retries when no acceptance ACK arrives, and decodes
  every kind of telemetry into tables for analysis.
"""
import os
import struct
import subprocess

import numpy as np

import link
from icd import ICD, ERR_NAMES, CMD_NAMES, EVT_NAMES

HERE = os.path.dirname(os.path.abspath(__file__))
OBC_BIN = os.path.join(HERE, "..", "flight_sw", "build", "obc_sim")

HK_FIELDS = ">BHBHHhhBhHHHBHBBhB"
HK_NAMES = ["mode", "batt_mv", "soc_pct", "solar_mw", "load_mw", "wheel_rpm", "wheel_cmd", "rails",
            "mag_x_10nt", "acc", "exec", "rej", "sched", "stored", "sstm_active", "eclipse", "temp_c10",
            "tx_power"]


class GroundStation:
    def __init__(self, passes=None, ber_up=0.0, ber_down=0.0, seed=1, truth_csv=None, stats_json=None,
                 ack_timeout_s=6.0, max_retries=3):
        self.ack_timeout_s, self.max_retries = ack_timeout_s, max_retries
        self.passes = passes                     # list of (start_s, end_s); None = always in view
        self.ber_up, self.ber_down = ber_up, ber_down
        self.rng = np.random.default_rng(seed)
        args = [OBC_BIN, truth_csv or os.devnull, stats_json or os.devnull]
        self.proc = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self.t_ms = 0
        self.deframer = link.Deframer()
        self.seq = {}
        self.up_queue = []                       # list of bit lists waiting for the uplink
        self.up_budget = 0.0
        self.pending = {}                        # (apid, seq) -> tc record awaiting acceptance
        # result tables
        self.tc_log, self.hk, self.events, self.onreq, self.sstm = [], [], [], [], []
        self.frames_down = 0
        self.bits_flipped_up = self.bits_flipped_down = 0
        self.bits_sent_up = self.bits_sent_down = 0

    # ------------------------------------------------------------ link helpers
    def in_view(self, t_s=None):
        t_s = self.t_ms / 1000 if t_s is None else t_s
        return self.passes is None or any(a <= t_s < b for a, b in self.passes)

    def _corrupt(self, packed: bytes, ber: float, direction: str) -> bytes:
        nbits = len(packed) * 8
        if direction == "up":
            self.bits_sent_up += nbits
        else:
            self.bits_sent_down += nbits
        if ber <= 0 or nbits == 0:
            return packed
        nflip = self.rng.binomial(nbits, ber)
        if nflip == 0:
            return packed
        arr = bytearray(packed)
        for pos in self.rng.choice(nbits, size=nflip, replace=False):
            arr[pos // 8] ^= 0x80 >> (pos % 8)
        if direction == "up":
            self.bits_flipped_up += nflip
        else:
            self.bits_flipped_down += nflip
        return bytes(arr)

    # ------------------------------------------------------------ commanding
    def _queue_packet(self, pkt: bytes):
        self.up_queue.append(link.hdlc_frame_bits(link.ADDR_SAT, link.ADDR_GROUND, pkt))

    def send(self, name, apid, cmd_id, params=b"", time_tag_s=None, seq=None):
        """Queue a TC. Returns its log record (filled in as ACKs arrive)."""
        if seq is None:
            seq = self.seq.get(apid, 0)
            self.seq[apid] = (seq + 1) % ICD["CCSDS_SEQ_MOD"]
        tt = None if time_tag_s is None else int(time_tag_s * 1000)
        pkt = link.build_tc(apid, seq, cmd_id, params, tt)
        rec = {"name": name, "apid": apid, "seq": seq, "cmd": CMD_NAMES.get(cmd_id, hex(cmd_id)),
               "time_tag_s": time_tag_s, "sent_s": self.t_ms / 1000, "tries": 1,
               "accept": None, "accept_code": None, "accept_s": None,
               "exec": None, "exec_code": None, "exec_s": None, "_pkt": pkt}
        self.tc_log.append(rec)
        self.pending[(apid, seq)] = rec
        self._queue_packet(pkt)
        return rec

    def _retry_check(self):
        now = self.t_ms / 1000
        for key, rec in list(self.pending.items()):
            if rec["accept"] is not None:
                del self.pending[key]
                continue
            if now - rec["sent_s"] >= self.ack_timeout_s * rec["tries"] and self.in_view():
                if rec["tries"] > self.max_retries:
                    rec["accept"] = "LOST"
                    del self.pending[key]
                else:
                    rec["tries"] += 1
                    self._queue_packet(rec["_pkt"])          # same packet, same sequence count

    # ------------------------------------------------------------ telemetry
    def _on_tm(self, tm):
        apid, t_s, d = tm["apid"], tm["t_ms"] / 1000, tm["data"]
        if apid in (ICD["APID_TM_HK"], ICD["APID_TM_STORED_HK"]) and len(d) == struct.calcsize(HK_FIELDS):
            row = dict(zip(HK_NAMES, struct.unpack(HK_FIELDS, d)))
            row.update(t_s=t_s, seq=tm["seq"], source="RT" if apid == ICD["APID_TM_HK"] else "DUMP", rx_s=self.t_ms / 1000)
            self.hk.append(row)
        elif apid == ICD["APID_TM_ACK"]:
            typ, tapid, tseq, cmd, code, tag = struct.unpack(">BHHHBI", d)
            rec = next((r for r in reversed(self.tc_log) if r["apid"] == tapid and r["seq"] == tseq), None)
            if rec is None:
                return
            now = self.t_ms / 1000
            if typ == ICD["ACK_ACCEPT_OK"] and rec["accept"] in (None, "LOST"):
                rec.update(accept="OK", accept_code="NONE", accept_s=now)
            elif typ == ICD["ACK_ACCEPT_FAIL"]:
                if code == ICD["ERR_DUPLICATE"] and rec["tries"] > 1:
                    # our retry hit the duplicate check: the FIRST copy was accepted,
                    # only its ACK was lost on the way down.
                    if rec["accept"] is None:
                        rec.update(accept="OK", accept_code="NONE(dup-retry)", accept_s=now)
                elif rec["accept"] is None:
                    rec.update(accept="FAIL", accept_code=ERR_NAMES.get(code, code), accept_s=now)
            elif typ == ICD["ACK_EXEC_OK"]:
                rec.update(exec="OK", exec_code="NONE", exec_s=now, exec_obc_s=t_s)
            elif typ == ICD["ACK_EXEC_FAIL"]:
                rec.update(exec="FAIL", exec_code=ERR_NAMES.get(code, code), exec_s=now, exec_obc_s=t_s)
        elif apid == ICD["APID_TM_EVENT"]:
            evt, val = struct.unpack(">BI", d)
            self.events.append({"t_s": t_s, "event": EVT_NAMES.get(evt, evt), "value": val})
        elif apid == ICD["APID_TM_ONREQ"]:
            pid, val = struct.unpack(">Bf", d)
            self.onreq.append({"t_s": t_s, "param": pid, "value": val})
        elif apid == ICD["APID_TM_SSTM"]:
            for i in range(0, len(d), 12):
                ms, mag, rpm, temp, mv = struct.unpack(">IhhhH", d[i:i + 12])
                self.sstm.append({"t_s": ms / 1000, "mag_x_nt": mag * 10, "wheel_rpm": rpm,
                                  "temp_c": temp / 10, "batt_mv": mv})

    # ------------------------------------------------------------ time stepping
    def step(self, dt_ms):
        target = self.t_ms + dt_ms
        view = self.in_view()
        bits = []
        if view:
            self.up_budget = min(self.up_budget + ICD["LINK_BITRATE_BPS"] * dt_ms / 1000, 6000)
            while self.up_queue and len(self.up_queue[0]) <= self.up_budget:
                fb = self.up_queue.pop(0)
                self.up_budget -= len(fb)
                bits += fb
        up = self._corrupt(link.pack_bits(bits), self.ber_up, "up") if bits else b""
        self.proc.stdin.write(struct.pack(">II", target, len(up)) + up)
        self.proc.stdin.flush()
        hdr = self.proc.stdout.read(8)
        t, n = struct.unpack(">II", hdr)
        down = self.proc.stdout.read(n)
        self.t_ms = target
        if view and down:
            for frame in self.deframer.feed(self._corrupt(down, self.ber_down, "down")):
                self.frames_down += 1
                if frame[:7] != link.ADDR_GROUND:
                    continue
                tm = link.parse_tm(frame[16:])
                if tm:
                    self._on_tm(tm)
        self._retry_check()

    def run_until(self, t_s):
        while self.t_ms < t_s * 1000:
            dt = 100 if self.in_view() else 1000      # fine steps only while in contact
            self.step(min(dt, int(t_s * 1000) - self.t_ms))

    def run_for(self, s):
        self.run_until(self.t_ms / 1000 + s)

    def latest_hk(self):
        rt = [h for h in self.hk if h["source"] == "RT"]
        return rt[-1] if rt else None

    def close(self):
        self.proc.stdin.close()
        self.proc.wait(timeout=30)
        for r in self.tc_log:
            r.pop("_pkt", None)
