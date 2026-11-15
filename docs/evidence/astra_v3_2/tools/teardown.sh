#!/usr/bin/env bash
# Harness teardown: bounded SIGINT of the launch, then the project's cleanup + SIGTERM leftovers.
lp="$1"; ws="$(cd "$(dirname "$0")/../../../.." && pwd)"
kill -INT "$lp" 2>/dev/null
for _ in $(seq 150); do kill -0 "$lp" 2>/dev/null || break; sleep 0.2; done
kill -0 "$lp" 2>/dev/null && { echo "[teardown] launch ignored SIGINT for 30 s -> SIGTERM"; kill -TERM "$lp"; }
bash "$ws/scripts/cleanup_sim.sh"
pkill -TERM -f "component_container_isolated.*nav2_container" 2>/dev/null
pkill -TERM -f "inspectloop_gazebo/scripts/gz_sim_supervised.sh" 2>/dev/null
sleep 2; pkill -KILL -f "inspectloop_gazebo/worlds/warehouse.sdf" 2>/dev/null
left=$(ps -eo args | grep -cE "[w]arehouse.sdf|[n]av2_container|install/inspectloop_(safety|vision|mission)/lib")
echo "[teardown] leftover InspectLoop processes: $left"
