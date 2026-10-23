#!/usr/bin/env python3
import argparse
import os
import time
from collections import Counter
from datetime import datetime
from typing import Optional, TextIO

import cereal.messaging as messaging
from common.realtime import sec_since_boot


WATCH_IDS = (0x25, 0x260, 0x262, 0x2E4, 0x191)


def _emit(msg: str, log_fp: Optional[TextIO]) -> None:
  print(msg)
  if log_fp is not None:
    log_fp.write(msg + "\n")
    log_fp.flush()


def _default_log_path() -> str:
  ts = datetime.now().strftime("%Y%m%d_%H%M%S")
  return f"/data/media/0/realdata/steering_watch_{ts}.log"


def watch(duration: float, bus: int, print_every: float, log_path: Optional[str]) -> None:
  sm = messaging.SubMaster(["carState", "controlsState", "carControl", "can"], poll=["carState"])
  sendcan_sock = messaging.sub_sock("sendcan", timeout=20)
  log_fp: Optional[TextIO] = None

  can_counts = Counter()
  can_unique = {addr: set() for addr in WATCH_IDS}
  sendcan_counts = Counter()
  sendcan_unique = {0x2E4: set(), 0x191: set()}
  state_counts = Counter()
  alert_counts = Counter()

  angle_min, angle_max = 1e9, -1e9
  rate_min, rate_max = 1e9, -1e9
  drv_torque_min, drv_torque_max = 1e9, -1e9
  eps_torque_min, eps_torque_max = 1e9, -1e9
  actuator_steer_min, actuator_steer_max = 1e9, -1e9
  steering_pressed_seen = False
  steer_warning_seen = False
  controls_active_seen = False

  if log_path:
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    log_fp = open(log_path, "w", encoding="utf-8")
    _emit(f"log_path: {log_path}", log_fp)
    _emit(f"start_time: {datetime.now().isoformat()}", log_fp)
    _emit(f"duration_s: {duration}", log_fp)
    _emit(f"watch_bus: {bus}", log_fp)

  t0 = sec_since_boot()
  last_print = t0

  interrupted = False
  try:
    while (sec_since_boot() - t0) < duration:
      sm.update(50)

      cs = sm["carState"]
      ct = sm["controlsState"]
      cc = sm["carControl"]

      angle_min = min(angle_min, float(cs.steeringAngleDeg))
      angle_max = max(angle_max, float(cs.steeringAngleDeg))
      rate_min = min(rate_min, float(cs.steeringRateDeg))
      rate_max = max(rate_max, float(cs.steeringRateDeg))
      drv_torque_min = min(drv_torque_min, float(cs.steeringTorque))
      drv_torque_max = max(drv_torque_max, float(cs.steeringTorque))
      eps_torque_min = min(eps_torque_min, float(cs.steeringTorqueEps))
      eps_torque_max = max(eps_torque_max, float(cs.steeringTorqueEps))
      actuator_steer_min = min(actuator_steer_min, float(cc.actuators.steer))
      actuator_steer_max = max(actuator_steer_max, float(cc.actuators.steer))

      steering_pressed_seen |= bool(cs.steeringPressed)
      steer_warning_seen |= bool(cs.steerWarning)
      controls_active_seen |= bool(ct.active)
      state_counts[str(ct.state)] += 1
      if ct.alertType:
        alert_counts[str(ct.alertType)] += 1

      if sm.updated["can"]:
        for pkt in sm["can"]:
          if pkt.src == bus and pkt.address in can_unique:
            can_counts[pkt.address] += 1
            can_unique[pkt.address].add(pkt.dat.hex())

      for m in messaging.drain_sock(sendcan_sock, wait_for_one=False):
        for pkt in m.sendcan:
          if pkt.address in sendcan_unique:
            sendcan_counts[pkt.address] += 1
            sendcan_unique[pkt.address].add(pkt.dat.hex())

      now = sec_since_boot()
      if now - last_print >= print_every:
        _emit(
          f"t={now - t0:5.1f}s "
          f"state={ct.state} "
          f"en={int(ct.enabled)} "
          f"angle={cs.steeringAngleDeg:7.2f} "
          f"rate={cs.steeringRateDeg:7.2f} "
          f"drvT={cs.steeringTorque:6.1f} "
          f"epsT={cs.steeringTorqueEps:6.1f} "
          f"pressed={int(cs.steeringPressed)} "
          f"warn={int(cs.steerWarning)} "
          f"active={int(ct.active)} "
          f"actSteer={cc.actuators.steer:6.3f} "
          f"can2e4={can_counts[0x2E4]} "
          f"sc2e4={sendcan_counts[0x2E4]} "
          f"alert={ct.alertType}",
          log_fp,
        )
        last_print = now

      time.sleep(0.01)
  except KeyboardInterrupt:
    interrupted = True
  finally:
    if log_fp is not None:
      _emit(f"capture_interrupted: {interrupted}", log_fp)

  _emit("\n=== steering watch summary ===", log_fp)
  for addr in WATCH_IDS:
    _emit(f"can[{hex(addr)}] count={can_counts[addr]:5d} unique={len(can_unique[addr])}", log_fp)
  for addr in (0x2E4, 0x191):
    _emit(f"sendcan[{hex(addr)}] count={sendcan_counts[addr]:5d} unique={len(sendcan_unique[addr])}", log_fp)

  _emit(f"angle min/max: {angle_min:.3f} / {angle_max:.3f}", log_fp)
  _emit(f"rate min/max: {rate_min:.3f} / {rate_max:.3f}", log_fp)
  _emit(f"driver torque min/max: {drv_torque_min:.3f} / {drv_torque_max:.3f}", log_fp)
  _emit(f"eps torque min/max: {eps_torque_min:.3f} / {eps_torque_max:.3f}", log_fp)
  _emit(f"actuator steer min/max: {actuator_steer_min:.5f} / {actuator_steer_max:.5f}", log_fp)
  _emit(f"steeringPressed seen: {steering_pressed_seen}", log_fp)
  _emit(f"steerWarning seen: {steer_warning_seen}", log_fp)
  _emit(f"controls active seen: {controls_active_seen}", log_fp)
  _emit(f"state counts: {dict(state_counts)}", log_fp)
  _emit(f"alert counts: {dict(alert_counts)}", log_fp)

  for addr in WATCH_IDS:
    vals = sorted(can_unique[addr])
    if vals:
      _emit(f"{hex(addr)} unique_head: {', '.join(vals[:8])}", log_fp)
  for addr in (0x2E4, 0x191):
    vals = sorted(sendcan_unique[addr])
    if vals:
      _emit(f"sendcan {hex(addr)} unique_head: {', '.join(vals[:8])}", log_fp)
  if log_fp is not None:
    _emit(f"end_time: {datetime.now().isoformat()}", log_fp)
    log_fp.close()


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Watch steering command and steering sensor consistency")
  parser.add_argument("--duration", type=float, default=900.0, help="capture duration in seconds")
  parser.add_argument("--bus", type=int, default=0, help="CAN bus index to inspect for car-side frames")
  parser.add_argument("--print-every", type=float, default=1.0, help="console print interval in seconds")
  parser.add_argument("--log", type=str, default="", help="optional log output file path")
  args = parser.parse_args()

  log_path = args.log.strip() or _default_log_path()
  watch(args.duration, args.bus, args.print_every, log_path)
