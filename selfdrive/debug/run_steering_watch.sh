#!/usr/bin/bash
set -euo pipefail

cd /data/openpilot
mkdir -p /data/media/0/realdata
export PYTHONPATH="/data/openpilot:/data/openpilot/pyextra"

DURATION="${1:-1800}"
PRINT_EVERY="${2:-1.0}"
TS="$(date +%Y%m%d_%H%M%S)"
LOG="/data/media/0/realdata/steering_watch_${TS}.log"
PIDFILE="/data/media/0/realdata/steering_watch.pid"
ERRLOG="/data/media/0/realdata/steering_watch_${TS}.err"

nohup bash -c "
for i in \$(seq 1 300); do
  [ -e /dev/shm/carState ] && break
  sleep 1
done
python3 /data/openpilot/selfdrive/debug/steering_watch.py \
  --duration \"${DURATION}\" \
  --print-every \"${PRINT_EVERY}\" \
  --log \"${LOG}\"
" >"${ERRLOG}" 2>&1 &

echo $! > "${PIDFILE}"
echo "started_pid=$(cat "${PIDFILE}")"
echo "log_file=${LOG}"
echo "err_file=${ERRLOG}"
