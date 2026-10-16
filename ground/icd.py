"""
icd.py - the ground station reads its constants straight from the flight
software's icd.h, so the two sides can never use different numbers
(the #1 lesson from real TM/TC work: one source of truth).
"""
import os
import re

_HERE = os.path.dirname(os.path.abspath(__file__))
ICD_H = os.path.join(_HERE, "..", "flight_sw", "include", "icd.h")


def _load(path=ICD_H):
    values = {}
    pat = re.compile(r"^\s*#define\s+([A-Z0-9_]+)\s+(.+?)\s*(/\*.*)?$")
    with open(path) as f:
        for line in f:
            m = pat.match(line)
            if not m or m.group(1) == "ICD_H":
                continue
            expr = m.group(2)
            for k, v in values.items():                     # allow references to earlier names
                expr = re.sub(rf"\b{k}\b", str(v), expr)
            try:
                values[m.group(1)] = int(eval(expr, {"__builtins__": {}}))
            except Exception:
                pass
    return values


ICD = _load()
globals().update(ICD)

# Human-readable names for logs and plots
ERR_NAMES = {v: k[4:] for k, v in ICD.items() if k.startswith("ERR_")}
CMD_NAMES = {v: k[4:] for k, v in ICD.items() if k.startswith("CMD_")}
EVT_NAMES = {v: k[4:] for k, v in ICD.items() if k.startswith("EVT_")}
ACK_NAMES = {ICD["ACK_ACCEPT_OK"]: "ACCEPT_OK", ICD["ACK_ACCEPT_FAIL"]: "ACCEPT_FAIL",
             ICD["ACK_EXEC_OK"]: "EXEC_OK", ICD["ACK_EXEC_FAIL"]: "EXEC_FAIL"}
