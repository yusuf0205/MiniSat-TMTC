"""
link.py - an independent Python implementation of the link and packet layers.

It is written separately from the C code on purpose: if the ground station
and the flight software both work against icd.h and still understand each
other end-to-end, the ICD is unambiguous.
"""
import struct
from icd import ICD

FLAG_BITS = [0, 1, 1, 1, 1, 1, 1, 0]


# ----------------------------------------------------------------- CRC
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


# ----------------------------------------------------------------- AX.25 addresses
def ax25_addr(call: str, ssid: int) -> bytes:
    call = call.ljust(6)[:6]
    return bytes((ord(ch) << 1) & 0xFF for ch in call) + bytes([0x60 | ((ssid & 0x0F) << 1)])


ADDR_GROUND = ax25_addr("GROUND", 0)
ADDR_SAT = ax25_addr("MINSAT", 1)


# ----------------------------------------------------------------- HDLC encode
def hdlc_frame_bits(dest: bytes, src: bytes, info: bytes) -> list:
    body = dest + src + bytes([ICD["AX25_CTRL_UI"], ICD["AX25_PID_NONE"]]) + info
    body += struct.pack(">H", crc16_ccitt(body))
    bits = list(FLAG_BITS)
    ones = 0
    for byte in body:
        for i in range(7, -1, -1):
            b = (byte >> i) & 1
            bits.append(b)
            if b:
                ones += 1
                if ones == 5:
                    bits.append(0)          # stuffed zero
                    ones = 0
            else:
                ones = 0
    bits += FLAG_BITS
    return bits


def pack_bits(bits: list) -> bytes:
    bits = bits + [1] * ((-len(bits)) % 8)      # idle 1s to a byte boundary
    out = bytearray()
    for i in range(0, len(bits), 8):
        v = 0
        for b in bits[i:i + 8]:
            v = (v << 1) | b
        out.append(v)
    return bytes(out)


# ----------------------------------------------------------------- HDLC decode
class Deframer:
    """Bit-level flag hunter, the mirror of hdlc_rx_bit() in C."""

    def __init__(self):
        self.buf = []
        self.ones = 0
        self.in_frame = False
        self.frames = []
        self.fcs_errors = 0
        self.runts = 0

    def _end(self):
        if self.in_frame and len(self.buf) >= 7:
            bits = self.buf[:-7]
            if len(bits) >= 8:
                if len(bits) % 8 or len(bits) // 8 < ICD["AX25_HDR_LEN"] + 3:
                    self.runts += 1
                else:
                    data = bytes(int("".join(map(str, bits[i:i + 8])), 2) for i in range(0, len(bits), 8))
                    if crc16_ccitt(data[:-2]) == struct.unpack(">H", data[-2:])[0]:
                        self.frames.append(data[:-2])
                    else:
                        self.fcs_errors += 1
        self.in_frame = True
        self.buf = []

    def feed(self, packed: bytes):
        for byte in packed:
            for i in range(7, -1, -1):
                b = (byte >> i) & 1
                if b:
                    self.ones += 1
                    if self.ones > 6:
                        self.in_frame = False
                        self.buf = []
                        continue
                    if self.in_frame:
                        self.buf.append(1)
                else:
                    if self.ones == 5:
                        self.ones = 0
                        continue
                    if self.ones == 6:
                        self.ones = 0
                        self._end()
                        continue
                    self.ones = 0
                    if self.in_frame:
                        self.buf.append(0)
                if len(self.buf) > 8 * 400:
                    self.in_frame = False
                    self.buf = []
        out, self.frames = self.frames, []
        return out


# ----------------------------------------------------------------- CCSDS
def cuc(ms: int) -> bytes:
    coarse, rem = divmod(int(ms), 1000)
    fine = (rem * 65536) // 1000
    return struct.pack(">BIH", ICD["CUC_PFIELD"], coarse, fine)


def cuc_to_ms(coarse: int, fine: int) -> int:
    return coarse * 1000 + (fine * 1000 + 32768) // 65536


def build_tc(apid: int, seq: int, cmd_id: int, params: bytes = b"", time_tag_ms=None) -> bytes:
    sec = time_tag_ms is not None
    data = (cuc(time_tag_ms) + b"\x00" if sec else b"") + struct.pack(">H", cmd_id) + params
    length = len(data) + 2 - 1                              # CRC is inside the data field
    pid = (0 << 13) | (1 << 12) | (int(sec) << 11) | (apid & 0x7FF)
    psc = (ICD["CCSDS_SEQ_STANDALONE"] << 14) | (seq % ICD["CCSDS_SEQ_MOD"])
    pkt = struct.pack(">HHH", pid, psc, length) + data
    return pkt + struct.pack(">H", crc16_ccitt(pkt))


def parse_tm(pkt: bytes):
    """Return dict or None (bad CRC / malformed)."""
    if len(pkt) < 6 + 7 + 2:
        return None
    pid, psc, length = struct.unpack(">HHH", pkt[:6])
    if 6 + length + 1 != len(pkt):
        return None
    if crc16_ccitt(pkt[:-2]) != struct.unpack(">H", pkt[-2:])[0]:
        return None
    if (pid >> 12) & 1 != 0:
        return None
    p, coarse, fine = struct.unpack(">BIH", pkt[6:13])
    return {"apid": pid & 0x7FF, "seq": psc & 0x3FFF, "t_ms": cuc_to_ms(coarse, fine), "data": pkt[13:-2]}
