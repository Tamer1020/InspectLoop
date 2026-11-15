#!/usr/bin/env bash
# F4 live test: SIGINT the mission manager the moment its Nth goal is logged SENT
# (inside the send -> acceptance window). The simulation keeps running afterwards so Nav2's own
# goal status and the robot speed can be observed; then everything is shut down.
T="$(cd "$(dirname "$0")" && pwd)"; ws="$(cd "$T/../../../.." && pwd)"
out="${1:?out}"; nth="${2:-1}"; mkdir -p "$out"
source /opt/ros/jazzy/setup.bash; source "$ws/install/setup.bash"; set -u
log="$out/launch.log"
if ps -eo args | grep -q "[i]nspectloop_gazebo/worlds/warehouse.sdf"; then echo "sim already running"; exit 4; fi
ros2 launch inspectloop_bringup inspectloop.launch.py headless:=true rviz:=false \
  exit_on_mission_end:=false output_root:="$out" > "$log" 2>&1 &
lp=$!
until [[ -s "$log" ]]; do sleep 0.2; done
k=0
while read -r line; do
  if [[ "$line" == *"] SENT"* ]]; then k=$((k+1)); [[ $k -ge $nth ]] && break; fi
done < <(timeout 600 tail -n +1 -F "$log" 2>/dev/null)
mpid="$(pgrep -f "install/inspectloop_mission/lib/inspectloop_mission/mission_manager" | head -1)"
kill -INT "$mpid"
echo "[sigint_at_dispatch] $(date +%T.%N | cut -c1-12) SIGINT -> mission_manager $mpid right after SENT #$nth" | tee "$out/interrupt.log"
for _ in $(seq 200); do kill -0 "$mpid" 2>/dev/null || break; sleep 0.1; done
kill -0 "$mpid" 2>/dev/null && echo "mission_manager STILL RUNNING after 20 s" | tee -a "$out/interrupt.log"
echo "[sigint_at_dispatch] mission_manager exited; sampling robot speed for 10 s" | tee -a "$out/interrupt.log"
python3 "$ws/scripts/sample_odom.py" 10.0 > "$out/odom_speed.txt" 2>&1
tee -a "$out/interrupt.log" < "$out/odom_speed.txt"
echo "--- Nav2 /navigate_to_pose goal status after the interrupt (2=EXECUTING 4=SUCCEEDED 5=CANCELED 6=ABORTED)" | tee -a "$out/interrupt.log"
timeout 10 ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable \
  /navigate_to_pose/_action/status action_msgs/msg/GoalStatusArray 2>&1 | grep -E "status:" | sort | uniq -c | tee -a "$out/interrupt.log"
{ echo "--- ros2 action info /navigate_to_pose"; timeout 20 ros2 action info /navigate_to_pose; } >> "$out/interrupt.log" 2>&1
bash "$T/teardown.sh" $lp >> "$out/interrupt.log" 2>&1
echo "[sigint_at_dispatch] teardown done" | tee -a "$out/interrupt.log"
