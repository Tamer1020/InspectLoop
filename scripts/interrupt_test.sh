#!/usr/bin/env bash
# F4 live test: SIGINT the mission manager while Nav2 is executing a goal.
# Expected: the active goal is cancelled AND its terminal state confirmed, the report is written
# (state ABORTED, 'interrupted'), and the robot stops.
out="${1:?output dir}"
ws="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$out"; out="$(cd "$out" && pwd)"
source /opt/ros/jazzy/setup.bash; source "$ws/install/setup.bash"
set -u
if ps -eo args | grep -q "[i]nspectloop_gazebo/worlds/warehouse.sdf"; then
  echo "another InspectLoop simulation is running - stop it first"; exit 4
fi
log="$out/launch.log"
ros2 launch inspectloop_bringup inspectloop.launch.py headless:=true rviz:=false \
  exit_on_mission_end:=true output_root:="$out" > "$log" 2>&1 &
launch_pid=$!
until grep -q "\-> viewpoint" "$log" 2>/dev/null; do sleep 1; done
sleep 5
python3 "$ws/scripts/sample_odom.py" 6.0 > "$out/odom_speed.txt" 2>&1 &
sampler=$!
for _ in $(seq 1 150); do grep -q READY "$out/odom_speed.txt" 2>/dev/null && break; sleep 0.1; done
mpid="$(ps -eo pid,args | grep "[i]nstall/inspectloop_mission/lib/inspectloop_mission/mission_manager" | awk '{print $1}' | head -1)"
echo "[interrupt_test] $(date +%T) SIGINT -> mission_manager pid $mpid" | tee "$out/interrupt.log"
kill -INT "$mpid"
wait $sampler
tee -a "$out/interrupt.log" < "$out/odom_speed.txt"
wait $launch_pid
rc=$?; echo "[interrupt_test] launch exit code $rc" | tee -a "$out/interrupt.log"
