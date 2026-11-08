#!/usr/bin/env bash
# Fault-injection test for InspectLoop (headless).
#   scripts/fault_test.sh camera <output_dir>   camera bridge killed before the observation
#   scripts/fault_test.sh lidar  <output_dir>   LiDAR bridge killed while the robot drives
# Only the InspectLoop bridge process with the given node name is terminated.
fault="${1:?camera|lidar}"; out="${2:?output dir}"
ws="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$out"; out="$(cd "$out" && pwd)"
source /opt/ros/jazzy/setup.bash; source "$ws/install/setup.bash"   # (not set -u safe)
set -u
log="$out/launch.log"

ros2 launch inspectloop_bringup inspectloop.launch.py headless:=true rviz:=false \
  exit_on_mission_end:=true output_root:="$out" \
  mission_file:="$(ros2 pkg prefix inspectloop_mission)/share/inspectloop_mission/config/fault_single_gauge.yaml" \
  > "$log" 2>&1 &
launch_pid=$!

until grep -q "\-> viewpoint" "$log" 2>/dev/null; do
  sleep 1; kill -0 $launch_pid 2>/dev/null || { echo "launch ended early"; exit 1; }
done
case "$fault" in
  camera) sleep 2;  node="gz_camera_bridge" ;;
  lidar)  sleep 6;  node="gz_lidar_bridge" ;;
  *) echo "unknown fault $fault"; kill -INT $launch_pid; exit 2 ;;
esac
# the InspectLoop-specific node name is on the bridge command line (-r __node:=<name>)
pid="$(pgrep -f "ros_gz_bridge/parameter_bridge.*__node:=${node}( |$)" | head -1)"
[[ -n "$pid" ]] || { echo "bridge $node not found"; kill -INT $launch_pid; exit 3; }
echo "[fault_test] $(date +%T) injecting $fault fault: killing $node (pid $pid)" | tee -a "$out/fault.log"
if [[ "$fault" == "lidar" ]]; then
  # measure the robot speed just before and after the LiDAR disappears
  python3 "$ws/scripts/sample_odom.py" 3.0 > "$out/odom_speed.txt" 2>&1 &
  sampler=$!
  sleep 0.5
  kill -9 "$pid"
  echo "[fault_test] $(date +%T) killed at t+0.5s of the odom sample window" | tee -a "$out/fault.log"
  wait $sampler
  echo "[fault_test] robot speed around the fault (sample window starts 0.5 s before the kill):" | tee -a "$out/fault.log"
  tee -a "$out/fault.log" < "$out/odom_speed.txt"
else
  kill -9 "$pid"
fi
wait $launch_pid
echo "[fault_test] launch exit code $?" | tee -a "$out/fault.log"
