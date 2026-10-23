#!/usr/bin/env python3
"""Crank / power-cycle recovery monitor.

Captures the EPS LKA_STATE sequence + steer fault flags + controls/panda state
with timestamps so we can see exactly what happens after an engine crank
(relay drops panda+EPS power, then restores) and why openpilot shows
"steering temporarily unavailable" on re-engage.

Logs transitions (not every frame) to keep it light, plus a periodic heartbeat.

Usage (on EON):
  PYTHONPATH=/data/openpilot:/data/openpilot/pyextra python3 /tmp/crank_monitor.py [duration_s]
"""
import os
import sys
import time
import cereal.messaging as messaging

LKA_NAMES = {1: "standby", 3: "state3", 5: "active", 9: "temp_fault2", 17: "state17", 25: "temp_fault"}


def decode_lka(dat):
    # EPS_STATUS LKA_STATE : 31|7@0+  (big-endian, top 7 bits of byte 3)
    try:
        return (dat[3] >> 1) & 0x7F
    except Exception:
        return None


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 900.0
    ts = time.strftime("%Y%m%d_%H%M%S")
    outdir = "/data/media/0/realdata"
    os.makedirs(outdir, exist_ok=True)
    path = f"{outdir}/crank_{ts}.log"
    f = open(path, "w")

    def emit(msg):
        line = f"{time.time():.3f}  {msg}"
        f.write(line + "\n")
        f.flush()
        print(line)

    sm = messaging.SubMaster(["carState", "controlsState", "pandaStates"])
    sub_can = messaging.sub_sock("can", timeout=100)

    emit(f"monitor start, logging to {path}")

    last = {}
    last_lka = None
    last_hb = 0.0
    last_can_rx = time.time()
    t0 = time.time()
    while time.time() - t0 < dur:
        sm.update(20)

        # EPS LKA_STATE straight off the bus (0x262, byte3>>1)
        for m in messaging.drain_sock(sub_can, wait_for_one=False):
            for c in m.can:
                if c.address == 0x262 and c.src == 0:
                    last_can_rx = time.time()
                    lka = decode_lka(c.dat)
                    if lka is None:
                        continue
                    if lka != last_lka:
                        emit(f"LKA_STATE: {last_lka} -> {lka} ({LKA_NAMES.get(lka, '?')})  raw_byte3={c.dat[3]}")
                        last_lka = lka

        cur = {
            "state": str(sm["carState"] and sm["controlsState"].state),
            "enabled": sm["controlsState"].enabled,
            "active": sm["controlsState"].active,
            "steerWarning": bool(getattr(sm["carState"], "steerWarning", False)),
            "steerError": bool(getattr(sm["carState"], "steerError", False)),
            "canValid": bool(sm["carState"].canValid),
            "nPandas": len(sm["pandaStates"]),
            "ctrlAllowed": any(ps.controlsAllowed for ps in sm["pandaStates"]) if len(sm["pandaStates"]) else False,
            "safety": str(sm["pandaStates"][0].safetyModel) if len(sm["pandaStates"]) else "none",
            "csAlive": sm.alive["carState"],
        }
        for k, v in cur.items():
            if last.get(k) != v:
                emit(f"{k}: {last.get(k)} -> {v}")
        last = cur

        now = time.time()
        if now - last_hb >= 5.0:
            last_hb = now
            can_age = now - last_can_rx
            emit(f"HB  state={cur['state']} en={cur['enabled']} act={cur['active']} "
                 f"steerWarn={cur['steerWarning']} LKA={last_lka} ctrlAllowed={cur['ctrlAllowed']} "
                 f"nPandas={cur['nPandas']} canValid={cur['canValid']} can_age={can_age:.1f}s")

    emit("monitor end")
    f.close()


if __name__ == "__main__":
    main()
