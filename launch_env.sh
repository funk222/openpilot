#!/usr/bin/bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

if [ -z "$REQUIRED_NEOS_VERSION" ]; then
  export REQUIRED_NEOS_VERSION="19.1"
fi

if [ -z "$SKIP_NEOS_UPDATE" ]; then
  export SKIP_NEOS_UPDATE="1"
fi

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="4"
fi

if [ -z "$PASSIVE" ]; then
  export PASSIVE="1"
fi

if [ -z "$SUPPRESS_UNOFFICIAL_HARDWARE_ALERT" ]; then
  export SUPPRESS_UNOFFICIAL_HARDWARE_ALERT="1"
fi

if [ -z "$SUPPRESS_CAMERA_MALFUNCTION_ALERT" ]; then
  export SUPPRESS_CAMERA_MALFUNCTION_ALERT="1"
fi

# Force car fingerprint for custom single-CAN Arduino setup
# This allows Corolla to be recognized even if CAN message pattern doesn't perfectly match OEM
if [ -z "$FINGERPRINT" ]; then
  export FINGERPRINT="TOYOTA COROLLA 2017"
fi

if [ -z "$SKIP_FW_QUERY" ]; then
  export SKIP_FW_QUERY="1"
fi

if [ -z "$ENABLE_STEERING_WATCH_AUTORUN" ]; then
  export ENABLE_STEERING_WATCH_AUTORUN="0"
fi

if [ -z "$STEERING_WATCH_DURATION" ]; then
  export STEERING_WATCH_DURATION="5400"
fi

if [ -z "$STEERING_WATCH_PRINT_EVERY" ]; then
  export STEERING_WATCH_PRINT_EVERY="1.0"
fi

export STAGING_ROOT="/data/safe_staging"
