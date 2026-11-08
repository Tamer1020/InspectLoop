#!/usr/bin/env bash
# Fault-injection test for InspectLoop (headless).
#   scripts/fault_test.sh camera <output_dir>   camera bridge killed before the observation
#   scripts/fault_test.sh lidar  <output_dir>   LiDAR bridge killed while the robot drives
#   scripts/fault_test.sh scan_nan|scan_empty <dir>   LiDAR replaced by all-NaN / empty scans
#   scripts/fault_test.sh image_frozen|image_future <dir>  camera replaced by a frozen stream /
#                                                         future-stamped frames
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
wait_for() {   # wait_for <file> <pattern> <timeout_s>
  for _ in $(seq 1 $(( $3 * 10 ))); do grep -q "$2" "$1" 2>/dev/null && return 0; sleep 0.1; done
  echo "[fault_test] timeout waiting for '$2' in $1" | tee -a "$out/fault.log"; return 1
}
injector=""
case "$fault" in
  camera) sleep 2;  node="gz_camera_bridge" ;;
  lidar)  sleep 4;  node="gz_lidar_bridge" ;;
  scan_nan|scan_empty) sleep 4; node="gz_lidar_bridge"; injector="$fault" ;;
  image_frozen|image_future) sleep 1; node="gz_camera_bridge"; injector="$fault" ;;
  *) echo "unknown fault $fault"; kill -INT $launch_pid; exit 2 ;;
esac
# the InspectLoop-specific node name is on the bridge command line (-r __node:=<name>)
pid="$(pgrep -f "ros_gz_bridge/parameter_bridge.*__node:=${node}( |$)" | head -1)"
[[ -n "$pid" ]] || { echo "bridge $node not found"; kill -INT $launch_pid; exit 3; }
if [[ "$fault" == "lidar" || "$fault" == scan_* ]]; then
  # speed record starts BEFORE the fault (robot driving) and lasts 8 s
  python3 "$ws/scripts/sample_odom.py" 8.0 > "$out/odom_speed.txt" 2>&1 &
  sampler=$!
  wait_for "$out/odom_speed.txt" READY 15
  t_ready=$(date +%s.%N)
fi
t_inj=$(date +%s.%N)
if [[ -n "$injector" ]]; then
  python3 "$ws/scripts/publish_fault_stream.py" "$injector" 1.0 > "$out/injector.log" 2>&1 &
  inj=$!
  if [[ "$injector" == image_* ]]; then wait_for "$out/injector.log" CAPTURED 15; fi
  if [[ "$injector" == scan_* ]]; then wait_for "$out/injector.log" FIRST_PUBLISH 15; fi
fi
echo "[fault_test] $(date +%T.%N | cut -c1-12) injecting $fault fault: killing $node (pid $pid)" | tee -a "$out/fault.log"
kill -9 "$pid"
t_kill=$(date +%s.%N)
if [[ -n "${sampler:-}" ]]; then
  wait $sampler
  echo "[fault_test] robot speed; injector started at t+$(python3 -c "print(round($t_inj-$t_ready,1))") s, bridge killed at t+$(python3 -c "print(round($t_kill-$t_ready,1))") s of the window:" | tee -a "$out/fault.log"
  tee -a "$out/fault.log" < "$out/odom_speed.txt"
fi
wait $launch_pid
[[ -n "${inj:-}" ]] && kill -INT "$inj" 2>/dev/null
echo "[fault_test] launch exit code $?" | tee -a "$out/fault.log"
