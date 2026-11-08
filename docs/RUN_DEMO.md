# Running the demo

Always start with:

```bash
cd ~/InspectLoop && source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

## A. Full autonomous gauge round (GUI + RViz)

```bash
ros2 launch inspectloop_bringup inspectloop.launch.py
```

What happens (recorded in `docs/evidence/demo_run_v2/`; v1 baseline in `docs/evidence/v1_demo_run/`):

1. Gazebo opens the warehouse, the robot spawns at (-10, 0); bridges, robot_state_publisher,
   `cmd_vel_gate` and `health_monitor` start.
2. After 5 s Nav2 + AMCL come up (map → odom from AMCL, initial pose from parameters).
3. After 10 s `mission_manager` waits for Nav2, `/gauge/read`, `/map`, TF and `/amcl_pose`.
4. **PG-11** – drives to the frontal viewpoint, waits until odometry shows the robot at rest,
   captures 5 fresh frames → **ACCEPT** 4.2 bar.
5. **PG-12** – frontal view: the hanging conduit box hides the dial → `OCCLUDED` → **MOVE_NEW**;
   the planner blocks that direction and picks the −45° viewpoint; there the dial is clear but
   too small for the needle-evidence test (`TOO_FAR`) → **MOVE_NEW** closer along the same line of
   sight (1.2 m) → **ACCEPT** 6.8 bar.
6. **PG-13** – a crate stack hides the gauge: `TAG_NOT_VISIBLE` → **RETRY_SAME** → **MOVE_NEW** (−30°)
   → RETRY_SAME → MOVE_NEW (+30°) → RETRY_SAME → **UNRESOLVED** (budget of 3 viewpoints spent).
7. **PG-14** – **ACCEPT** 8.7 bar, flagged **alarm HIGH** (limit 8.0 bar).
8. Return home; report written to `~/inspectloop_runs/<mission_id>/`.

In RViz: map, costmaps, LiDAR, AMCL particles, global/local plans, robot model, TF, the candidate
viewpoints (thin blue arrows), visited viewpoints coloured by decision (green ACCEPT, yellow
RETRY_SAME, orange MOVE_NEW, red UNRESOLVED), the result text at each gauge, the live camera and the
annotated gauge reading.

## B. Headless (no GUI), stops when the mission ends

```bash
ros2 launch inspectloop_bringup inspectloop.launch.py headless:=true rviz:=false exit_on_mission_end:=true
```

Takes ≈ 4 min wall time on the reference machine.

## C. Results

```bash
ls ~/inspectloop_runs/                        # one folder per mission
R=~/inspectloop_runs/<mission_id>
xdg-open $R/mission_report.html               # or open mission_report.md
ros2 run inspectloop_mission evaluate_run $R  # compare with simulator ground truth (offline only)
ros2 run inspectloop_mission generate_report $R   # re-render reports from mission_record.json
```

Run folder: `mission_record.json` (everything), `mission_report.{md,html,json}`,
`gauge_readings.jsonl`, `gauge_<id>.json`, `images/` (raw, annotated, rectified per observation).

## D. Watch it live

```bash
ros2 topic echo /mission/decisions     # every ACCEPT / RETRY_SAME / MOVE_NEW / UNRESOLVED + reason
ros2 topic echo /gauge/readings        # value, sigma, flags, quality metrics
ros2 topic echo /mission/status
ros2 topic echo /diagnostics           # health_monitor
```

## E. Fault injection

```bash
scripts/fault_test.sh camera /tmp/fault_camera   # camera bridge killed before the observation
scripts/fault_test.sh lidar  /tmp/fault_lidar    # LiDAR bridge killed while driving
```

Further modes (Astra F3/F5): `scan_nan`, `scan_empty` (LiDAR replaced by present-but-invalid
scans), `image_frozen`, `image_future` (camera replaced by a frozen / future-stamped stream), and
in V3 `scan_front_nan` (the *live* scan relayed with every beam inside ±35° set to NaN — the rest
stays valid, the rate stays healthy) and `clock_reset` (Gazebo world-time reset right after the
robot reached its first viewpoint, i.e. the simulation clock jumps backwards).
`scripts/interrupt_test.sh <dir>` sends SIGINT to the mission while Nav2 executes a goal.

Expected: camera → `NO_IMAGE` → RETRY_SAME → UNRESOLVED ("moving cannot fix a sensor fault");
LiDAR loss or invalid scans → gate `STOPPED_SCAN_FAULT` (robot stops, speeds logged to
`odom_speed.txt`) → mission ABORTED, report still written; front-sector blindness → gate
`BLOCKED_UNOBSERVED`, robot stops, mission ABORTED after 3 s; frozen / future-stamped images are
never used as evidence; a clock reset invalidates the observation in progress (`CLOCK_RESET`,
clock epoch changes) — Nav2 itself does not recover from a world-time reset, so navigation then
fails and the gauge ends `UNRESOLVED` / `NAVIGATION_FAILURE`. The script refuses to run while
another InspectLoop simulation is up. Recorded runs: `docs/evidence/astra_v3/faults/`.

Other useful switches: `mission:=false` (drive with RViz "Nav2 Goal"), `mission_file:=<yaml>`,
`output_root:=<dir>`, `camera_bridge:=false` (start without camera),
`ros2 service call /cmd_vel_gate/set_estop std_srvs/srv/SetBool "{data: true}"` (e-stop).

## F. Stopping

`Ctrl-C` in the launch terminal. The Gazebo server is run in its own process group and torn down
on shutdown. If a crash leaves processes behind: `scripts/cleanup_sim.sh` (only matches InspectLoop
command lines).
