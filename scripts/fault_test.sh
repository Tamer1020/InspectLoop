#!/usr/bin/env bash
# Fault-injection test for InspectLoop (headless).
#   scripts/fault_test.sh camera <output_dir>   camera bridge killed before the observation
#   scripts/fault_test.sh lidar  <output_dir>   LiDAR bridge killed while the robot drives
#   scripts/fault_test.sh scan_nan|scan_empty <dir>   LiDAR replaced by all-NaN / empty scans
#   scripts/fault_test.sh image_frozen|image_future <dir>  camera replaced by a frozen stream /
#                                                         future-stamped frames
#   scripts/fault_test.sh scan_front_nan <dir>  V3: live scan with every beam in +-35 deg NaN
#   scripts/fault_test.sh clock_reset <dir>     V3: Gazebo world TIME reset (sim clock jumps back)
#                                               just after the robot reached its first viewpoint
# Only the InspectLoop bridge process with the given node name is terminated.
fault="${1:?camera|lidar}"; out="${2:?output dir}"
ws="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$out"; out="$(cd "$out" && pwd)"
source /opt/ros/jazzy/setup.bash; source "$ws/install/setup.bash"   # (not set -u safe)
set -u
log="$out/launch.log"
# refuse to run next to another InspectLoop simulation (shared /clock, /odom would corrupt results)
if ps -eo args | grep -q "[i]nspectloop_gazebo/worlds/warehouse.sdf"; then
  echo "[fault_test] another InspectLoop simulation is running - stop it first" | tee "$out/fault.log"
  exit 4
fi

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
  image_frozen|image_future) node="gz_camera_bridge"; injector="$fault" ;;
  scan_front_nan) sleep 4; node="gz_lidar_bridge"; injector="$fault"
    # second bridge: the live Gazebo scan on /scan_live, relayed (with the blind sector) to /scan
    ros2 run ros_gz_bridge parameter_bridge \
      "/inspectloop/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" --ros-args \
      -r /inspectloop/scan:=/scan_live -r __node:=fault_scan_live_bridge > "$out/live_bridge.log" 2>&1 &
    live_bridge=$! ;;
  clock_reset) node=""
    wait_for "$log" "TERMINAL SUCCEEDED" 300 || { kill -INT $launch_pid; exit 5; }
    sleep 1.0 ;;
  *) echo "unknown fault $fault"; kill -INT $launch_pid; exit 2 ;;
esac
# the InspectLoop-specific node name is on the bridge command line (-r __node:=<name>)
if [[ -n "$node" ]]; then
  pid="$(pgrep -f "ros_gz_bridge/parameter_bridge.*__node:=${node}( |$)" | head -1)"
  [[ -n "$pid" ]] || { echo "bridge $node not found"; kill -INT $launch_pid; exit 3; }
fi
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
  if [[ "$injector" == image_* ]]; then wait_for "$out/injector.log" CAPTURED 30 || { kill -INT $launch_pid; exit 5; }; fi
  if [[ "$injector" == scan_* ]]; then wait_for "$out/injector.log" FIRST_PUBLISH 30 || { kill -INT $launch_pid; exit 5; }; fi
fi
if [[ "$fault" == clock_reset ]]; then
  echo "[fault_test] $(date +%T.%N | cut -c1-12) injecting clock_reset: world time reset" | tee -a "$out/fault.log"
  gz service -s /world/inspectloop_warehouse/control --reqtype gz.msgs.WorldControl \
    --reptype gz.msgs.Boolean --timeout 5000 --req 'reset: {time_only: true}' 2>&1 | tee -a "$out/fault.log"
else
  echo "[fault_test] $(date +%T.%N | cut -c1-12) injecting $fault fault: killing $node (pid $pid)" | tee -a "$out/fault.log"
  kill -9 "$pid"
fi
t_kill=$(date +%s.%N)
if [[ -n "${sampler:-}" ]]; then
  wait $sampler
  echo "[fault_test] robot speed; injector started at t+$(python3 -c "print(round($t_inj-$t_ready,1))") s, bridge killed at t+$(python3 -c "print(round($t_kill-$t_ready,1))") s of the window:" | tee -a "$out/fault.log"
  tee -a "$out/fault.log" < "$out/odom_speed.txt"
fi
wait $launch_pid
rc=$?
[[ -n "${inj:-}" ]] && kill -INT "$inj" 2>/dev/null
[[ -n "${live_bridge:-}" ]] && kill -INT "$live_bridge" 2>/dev/null
echo "[fault_test] launch exit code $rc" | tee -a "$out/fault.log"
