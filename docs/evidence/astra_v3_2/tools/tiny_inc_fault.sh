#!/usr/bin/env bash
# F3 live test: while the robot drives, replace the LiDAR with the audit's malformed scan
# (720 beams, angle_increment 1e-9). Mirrors scripts/fault_test.sh scan_nan.
T="$(cd "$(dirname "$0")" && pwd)"; ws="$(cd "$T/../../../.." && pwd)"
out="${1:?out}"; mkdir -p "$out"
source /opt/ros/jazzy/setup.bash; source "$ws/install/setup.bash"; set -u
log="$out/launch.log"
if ps -eo args | grep -q "[i]nspectloop_gazebo/worlds/warehouse.sdf"; then echo "sim already running"; exit 4; fi
ros2 launch inspectloop_bringup inspectloop.launch.py headless:=true rviz:=false \
  exit_on_mission_end:=true output_root:="$out" \
  mission_file:="$(ros2 pkg prefix inspectloop_mission)/share/inspectloop_mission/config/fault_single_gauge.yaml" \
  > "$log" 2>&1 &
lp=$!
until grep -q "\-> viewpoint" "$log" 2>/dev/null; do sleep 1; kill -0 $lp 2>/dev/null || { echo "launch ended early"; exit 1; }; done
sleep 4
pid="$(pgrep -f "ros_gz_bridge/parameter_bridge.*__node:=gz_lidar_bridge( |$)" | head -1)"
[[ -n "$pid" ]] || { echo "lidar bridge not found"; kill -INT $lp; exit 3; }
python3 "$ws/scripts/sample_odom.py" 8.0 > "$out/odom_speed.txt" 2>&1 & so=$!
python3 "$T/watch_gate_out.py" 8.0 > "$out/gate_out.txt" 2>&1 & go=$!
for _ in $(seq 150); do grep -q READY "$out/odom_speed.txt" 2>/dev/null && grep -q READY "$out/gate_out.txt" 2>/dev/null && break; sleep 0.1; done
sleep 1.0
python3 "$T/inject_tiny_inc.py" > "$out/injector.log" 2>&1 & inj=$!
for _ in $(seq 300); do grep -q FIRST_PUBLISH "$out/injector.log" 2>/dev/null && break; sleep 0.1; done
echo "[tiny_inc] $(date +%T.%N | cut -c1-12) injector publishing; killing gz_lidar_bridge pid $pid at window t+~1.x s" | tee "$out/fault.log"
kill -9 "$pid"
wait $so; wait $go
{ echo "--- /odom speed"; cat "$out/odom_speed.txt"; echo "--- /cmd_vel_safe (gate timer output)"; cat "$out/gate_out.txt"; } | tee -a "$out/fault.log"
wait $lp; rc=$?
kill -INT $inj 2>/dev/null
echo "[tiny_inc] launch exit code $rc" | tee -a "$out/fault.log"
