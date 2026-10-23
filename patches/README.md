# Out-of-tree patches

These patches live outside the `panda` submodule (which points at an upstream
`commaai/panda` commit we cannot push to). Apply them after checking out the
submodule, then rebuild/flash the panda firmware.

## panda_toyota_single_can_no_fwd.patch

**Why:** This rig is a single physical CAN bus (grey panda over OBD-II, a real
Corolla EPS, and an Arduino Toyota simulator), with **nothing on bus 2**. Stock
Toyota safety mirrors every bus-0 frame to bus 2 (the forward-facing camera
bus). With no node on bus 2, each forwarded frame is never ACKed, so CAN2 keeps
retransmitting and jams the panda's TX path. The result: `canFwdErrs` explodes
(seen at ~2.4M) and openpilot's own bus-0 output (`0x2E4` steering, `0x343`,
`0x412`) never reaches the physical bus, so the EPS never gets steering commands
and is stuck at `LKA_STATE=3`.

**Fix:** Disable the `bus 0 -> bus 2` forward in `toyota_fwd_hook`. After this,
`canFwdErrs` stays 0, openpilot's `0x2E4` reaches the bus, the EPS advances to
`LKA_STATE=1` (standby) and, once engaged, `LKA_STATE=5` (active).

### Apply + rebuild + flash

```bash
cd panda
git apply ../patches/panda_toyota_single_can_no_fwd.patch   # or: patch -p1 < ...
cd ..
# rebuild panda firmware (needs arm-none-eabi-gcc + scons)
cd panda && scons -u -j4 && cd ..
# reboot the device; pandad reflashes the panda when the firmware
# signature changes (see selfdrive/pandad.py).
```

On the EON the patched source is also committed on the local panda submodule
branch `single-can-nofwd` for safe keeping.
