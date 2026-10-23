#!/usr/bin/bash
set -euo pipefail

PIDFILE="/data/media/0/realdata/steering_watch.pid"

if [ ! -f "${PIDFILE}" ]; then
  echo "pidfile_not_found=${PIDFILE}"
  exit 1
fi

PID="$(cat "${PIDFILE}")"
if kill -0 "${PID}" 2>/dev/null; then
  kill "${PID}"
  echo "stopped_pid=${PID}"
else
  echo "pid_not_running=${PID}"
fi

rm -f "${PIDFILE}"
