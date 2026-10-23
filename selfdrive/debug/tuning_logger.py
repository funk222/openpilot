#!/usr/bin/env python3
"""Lightweight lateral-tuning data logger for the single-CAN Corolla rig.

Writes a CSV of vehicle state + openpilot's desired lateral signals so we can
fit steerRatio / angleOffset / tire stiffness (angle) and steerActuatorDelay /
gains (timing) offline.

Keep the subscriber set small and drain promptly to avoid the "evicting all
subscribers" overload that tripped commIssue with steering_watch.

Usage (on EON):
  PYTHONPATH=/data/openpilot:/data/openpilot/pyextra \
    python3 /tmp/tuning_logger.py [duration_s]
"""
import os
import sys
import time
import cereal.messaging as messaging


def g(obj, name, default=0.0):
    try:
        v = getattr(obj, name)
        return v
    except Exception:
        return default


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 1800.0
    ts = time.strftime("%Y%m%d_%H%M%S")
    outdir = "/data/media/0/realdata"
    os.makedirs(outdir, exist_ok=True)
    path = f"{outdir}/tuning_{ts}.csv"

    sm = messaging.SubMaster([
        "carState", "controlsState", "lateralPlan",
        "carControl", "liveParameters", "liveLocationKalman",
    ])

    cols = [
        "t", "vEgo", "aEgo",
        "steerAngleDeg", "steerRateDeg", "steerTorque", "steerTorqueEps", "steerPressed",
        "yawRate",                 # from liveLocationKalman (rad/s, +left)
        "curvature", "desiredCurvature", "desiredCurvatureRate",
        "steerAngleDesiredDeg",    # controlsState
        "lp_curvature",            # lateralPlan first curvature
        "cc_steer", "cc_steerAngleDeg", "latActive", "enabled",
        "lparam_steerRatio", "lparam_angleOffsetDeg", "lparam_angleOffsetAvgDeg", "lparam_stiffness",
    ]

    f = open(path, "w")
    f.write(",".join(cols) + "\n")
    print(f"logging to {path}")

    t0 = time.time()
    last_write = 0.0
    n = 0
    try:
        while time.time() - t0 < dur:
            sm.update(20)
            if not sm.updated["carState"]:
                continue
            now = time.time()
            if now - last_write < 0.02:   # cap ~50 Hz
                continue
            last_write = now

            cs = sm["carState"]
            ctl = sm["controlsState"]
            lpn = sm["lateralPlan"]
            cc = sm["carControl"]
            lpar = sm["liveParameters"]
            llk = sm["liveLocationKalman"]

            yaw = 0.0
            try:
                av = llk.angularVelocityCalibrated.value
                yaw = av[2] if av and len(av) >= 3 else 0.0
            except Exception:
                pass

            lp_curv = 0.0
            try:
                cvs = lpn.curvatures
                lp_curv = cvs[0] if cvs and len(cvs) else 0.0
            except Exception:
                pass

            acts = cc.actuators
            row = [
                f"{now - t0:.3f}",
                f"{g(cs,'vEgo'):.4f}", f"{g(cs,'aEgo'):.4f}",
                f"{g(cs,'steeringAngleDeg'):.3f}", f"{g(cs,'steeringRateDeg'):.3f}",
                f"{g(cs,'steeringTorque'):.3f}", f"{g(cs,'steeringTorqueEps'):.3f}",
                f"{int(bool(g(cs,'steeringPressed',False)))}",
                f"{yaw:.5f}",
                f"{g(ctl,'curvature'):.6f}", f"{g(ctl,'desiredCurvature'):.6f}", f"{g(ctl,'desiredCurvatureRate'):.6f}",
                f"{g(ctl,'steeringAngleDesiredDeg'):.3f}",
                f"{lp_curv:.6f}",
                f"{g(acts,'steer'):.4f}", f"{g(acts,'steeringAngleDeg'):.3f}",
                f"{int(bool(g(cc,'latActive',False)))}", f"{int(bool(g(ctl,'enabled',False)))}",
                f"{g(lpar,'steerRatio'):.4f}", f"{g(lpar,'angleOffsetDeg'):.4f}",
                f"{g(lpar,'angleOffsetAverageDeg'):.4f}", f"{g(lpar,'stiffnessFactor'):.4f}",
            ]
            f.write(",".join(row) + "\n")
            n += 1
            if n % 250 == 0:
                f.flush()
                print(f"  {now - t0:6.1f}s  rows={n}  vEgo={g(cs,'vEgo'):.1f}  "
                      f"ang={g(cs,'steeringAngleDeg'):.1f}  yaw={yaw:.3f}  "
                      f"desCurv={g(ctl,'desiredCurvature'):.4f}")
    except KeyboardInterrupt:
        pass
    finally:
        f.flush()
        f.close()
        print(f"done. rows={n}  file={path}")


if __name__ == "__main__":
    main()
