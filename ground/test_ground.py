"""
test_ground.py - checks that the Python ground station and the C flight
software agree on the ICD, byte for byte.   Run:  python3 ground/test_ground.py
"""
import os
import struct
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import link                                   # noqa: E402
from icd import ICD                           # noqa: E402
from station import GroundStation, OBC_BIN    # noqa: E402

passed = failed = 0


def check(cond, name):
    global passed, failed
    if cond:
        passed += 1
    else:
        failed += 1
        print("FAIL:", name)


# 1. same CRC as the C code
check(link.crc16_ccitt(b"123456789") == 0x29B1, "CRC check value")

# 2. same packet bytes as the C unit test expects (APID 0x110, seq 5, cmd 0x0011, params 0FA0)
pkt = link.build_tc(0x110, 5, 0x0011, b"\x0F\xA0")
check(pkt[:6] == bytes([0x11, 0x10, 0xC0, 0x05, 0x00, 0x05]), "TC primary header bytes")
check(len(pkt) == 12, "TC length")

# 3. HDLC round trip in Python with nasty data
d = link.Deframer()
info = bytes([0x7E, 0xFF] * 20)
frames = d.feed(link.pack_bits(link.hdlc_frame_bits(link.ADDR_GROUND, link.ADDR_SAT, info) * 1))
check(len(frames) == 1 and frames[0][16:] == info, "HDLC stuffing round trip")

# 4. constants really come from icd.h
check(ICD["APID_TC_ADCS"] == 0x110 and ICD["CMD_ADCS_SET_WHEEL"] == 0x11, "ICD constants parsed from icd.h")

# 5. end to end through the real C flight software
if os.path.exists(OBC_BIN):
    g = GroundStation()
    r = g.send("noop", ICD["APID_TC_OBC"], ICD["CMD_NO_OP"])
    w = g.send("wheel", ICD["APID_TC_ADCS"], ICD["CMD_ADCS_SET_WHEEL"], struct.pack(">h", 1234))
    g.run_until(12)
    g.close()
    check(r["accept"] == "OK" and r["exec"] == "OK", "end-to-end NO_OP accepted and executed")
    check(w["exec"] == "OK" and abs(g.latest_hk()["wheel_rpm"] - 1234) <= 1, "wheel reached commanded speed")
else:
    print("(skipping end-to-end test: build flight_sw first)")

print(f"{passed} passed, {failed} failed")
sys.exit(failed)
