# Custom 0.8.13: Corolla CAN + Odyssey Dynamics Profile

This document records the custom tuning used for the legacy conversion branch.

## Goal

- Keep **Toyota Corolla 2017 CAN/ECU signal layout** (for Arduino + Corolla steering hardware compatibility).
- Use **Odyssey-like vehicle dynamics parameters** as the initial steering profile.
- Leave final road tuning for later iteration.

## Files Changed

- `selfdrive/car/toyota/interface.py`

## What Changed

### 1) Corolla identity kept for protocol parsing

The branch still uses Toyota Corolla paths (`CAR.COROLLA`) for:

- DBC selection
- CAN parsing
- Toyota safety and control flow

This keeps compatibility with the Arduino ECU bridge that emits Toyota-format signals.

### 2) Corolla dynamics overridden for legacy build

Inside the Corolla parameter block, when `ToyotaFlags.SINGLE_CAN` is enabled, these values are applied:

- `wheelbase = 3.00`
- `steerRatio = 14.35`
- `mass = 4471 lb + cargo`
- `tire_stiffness_factor = 0.82`
- `centerToFront = wheelbase * 0.41`
- `steerActuatorDelay = 0.10`
- `lateralTuning.pid.kpV = [0.28]`
- `lateralTuning.pid.kiV = [0.08]`

These are reference-style values to emulate Odyssey-like behavior while preserving Toyota signaling.

### 3) Single-CAN validity check fix

`ret.canValid` logic was corrected:

- Single-CAN mode: only powertrain parser validity is required (`cp.can_valid`)
- Normal split-bus mode: both powertrain and camera parsers are required

## Arduino Matching Requirements

To keep runtime logic consistent, the Arduino ECU must continue to publish Toyota-formatted signals used by Toyota parsers, including at least:

- `WHEEL_SPEEDS`
- `STEER_ANGLE_SENSOR`
- `STEER_TORQUE_SENSOR`
- `PCM_CRUISE`
- `PCM_CRUISE_2`
- `BRAKE_MODULE`
- `GEAR_PACKET`

Do **not** substitute Honda message IDs/signal layouts into this branch. The dynamic profile is Odyssey-like, but the signal protocol is Toyota Corolla.

## Next Tuning Pass (Road Test)

After installation and basic validation, tune in small increments:

1. `steerRatio`
2. `kpV`
3. `kiV`
4. `steerActuatorDelay`

Keep one change per run and log behavior per speed band.
