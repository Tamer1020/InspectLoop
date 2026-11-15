#!/usr/bin/env bash
# V3.2 F4 live test: unknown-ownership goal on the real Nav2 stack (no mission manager).
T="$(cd "$(dirname "$0")" && pwd)"; ws="$(cd "$T/../../../.." && pwd)"
out="${1:?out}"; mkdir -p "$out"; out="$(cd "$out" && pwd)"
source /opt/ros/jazzy/setup.bash; source "$ws/install/setup.bash"; set -u
if ps -eo args | grep -q "[i]nspectloop_gazebo/worlds/warehouse.sdf"; then echo "sim already running"; exit 4; fi
ros2 launch inspectloop_bringup inspectloop.launch.py headless:=true rviz:=false mission:=false \
  > "$out/launch.log" 2>&1 &
lp=$!
for _ in $(seq 900); do grep -q "lifecycle_manager_navigation.*Managed nodes are active" "$out/launch.log" && break; sleep 0.2; done
grep -q "lifecycle_manager_navigation.*Managed nodes are active" "$out/launch.log" || { echo "Nav2 not active"; bash "$T/teardown.sh" $lp; exit 3; }
sleep 3
{ echo "--- ros2 action info /navigate_to_pose (who shares the server)"; timeout 20 ros2 action info /navigate_to_pose; } > "$out/action_info.txt" 2>&1
cat "$out/action_info.txt"
for mode in fallback nofallback; do
  echo "=== $mode"
  timeout 120 python3 "$T/live_unknown_goal.py" "$mode" > "$out/$mode.txt" 2>&1; echo "harness exit $?" >> "$out/$mode.txt"
  grep -vE "^EVENT|^\[WARN\]" "$out/$mode.txt"
  sleep 3
done
bash "$T/teardown.sh" $lp > "$out/teardown.txt" 2>&1; cat "$out/teardown.txt"
