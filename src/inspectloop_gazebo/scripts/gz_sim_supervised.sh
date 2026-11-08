#!/usr/bin/env bash
# Run `gz sim` in its own process group and take the whole group down on SIGINT/SIGTERM.
# The gz-tools ruby wrapper otherwise leaves the simulation server orphaned after Ctrl-C.
# Usage: gz_sim_supervised.sh <headless:true|false> <world.sdf> [extra gz sim args...]
set -u
headless="$1"; world="$2"; shift 2
args=(-r -v 2)
if [[ "${headless}" == "true" ]]; then
  args+=(-s --headless-rendering)   # server only, offscreen EGL rendering for camera/gpu_lidar
fi
setsid gz sim "${args[@]}" "$@" "${world}" &
pgid=$!
cleanup() {
  trap - INT TERM EXIT
  kill -INT -- "-${pgid}" 2>/dev/null
  for _ in $(seq 1 30); do
    kill -0 -- "-${pgid}" 2>/dev/null || exit 0
    sleep 0.1
  done
  kill -TERM -- "-${pgid}" 2>/dev/null
  sleep 1
  kill -KILL -- "-${pgid}" 2>/dev/null
  exit 0
}
trap cleanup INT TERM EXIT
wait "${pgid}"
