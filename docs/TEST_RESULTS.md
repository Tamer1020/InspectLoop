# Test results

All results below were produced on 2026-11-08 on the development machine (WSL2, Ubuntu 24.04,
ROS 2 Jazzy, Gazebo Harmonic, 16 cores). Raw artefacts are in `docs/evidence/`. Nothing in this file
is estimated or copied from elsewhere; where something was not verified it says so.

## 1. Build

```
colcon build --symlink-install      -> Summary: 8 packages finished (no warnings with -Wall -Wextra -Wpedantic on the C++ package)
```

## 2. Automated tests (`colcon test`)

`colcon test-result --all` → **126 results, 0 errors, 0 failures, 0 skipped**
(120 test cases + 6 CTest wrapper entries). Re-verified on a fresh `git clone` of the repository
(clean `colcon build` + `colcon test`).

| Package | Framework | Cases | What is covered |
|---|---|---|---|
| inspectloop_safety | gtest (C++) | 18 | TopicMonitor (rate, stale, degraded, clock jump), velocity gate priorities, clamping, slow/stop zone, corridor clearance |
| inspectloop_vision | pytest | 35 | needle reading on textures and perspective/noise/blur renders, occlusion / hidden tag / off-image / no-frame flags, **σ calibration**, constants ↔ textures, **4 real Gazebo frames** |
| inspectloop_mission | pytest | 41 | decision policy (all 4 decisions, escalation, budgets), viewpoint planner (feasibility on the real map, hints, blocked directions), config validation, report + evaluator |
| inspectloop_gazebo | pytest | 7 | committed world/map/textures match the layout, overhanging occluder absent from map, gauges not buried in walls (regression), texture tag |
| inspectloop_description | pytest | 7 | URDF tree, frames, wheel joints, inertia validity, plugin/geometry consistency |
| inspectloop_navigation | pytest | 6 | AMCL initial pose = spawn, frames, robot radius, velocity limits, map file |
| inspectloop_bringup | pytest | 6 | every launch file builds a LaunchDescription |

### Vision accuracy and σ calibration (synthetic, ground truth exact)

* Undistorted textures: |error| < 0.02 bar for all four gauges.
* Effect of the bezel ellipse refinement (introduced after the first measurements): frontal synthetic
  views at 0.5× scale had errors of up to **−0.63 bar** with the homography alone; after moving the tag
  quiet zone to one full marker cell and adding the bezel fit, the same sweep gives **≤ 0.05 bar**
  except for tiny dials seen very obliquely (scale 0.2, strong skew), where errors up to 0.74 bar
  remain — those observations get σ above the acceptance limit.
* Calibration sweep (`test_sigma_is_calibrated_on_synthetic_views`): 96 observations (4 gauges ×
  4 scales × 3 perspectives × 2 blur levels, noise σ = 3 grey levels), none flagged;
  mean |error| 0.011 bar, max 0.072 bar; **100 % within 1σ, max normalised error 0.81σ** → the σ model
  is *conservative* on these conditions (it over-states uncertainty), which errs on the safe side.

## 3. Simulation: environment and sensors (headless)

| Check | Result |
|---|---|
| Gazebo world loads, robot spawns (`ros_gz_sim create`) | ✔ |
| `/scan` rate | 9.6–9.7 Hz (`ros2 topic hz`) |
| `/odom` frame ids | `odom` → `base_footprint` ✔ |
| `/camera/image_raw` | 640×480 rgb8, `frame_id = camera_optical_link` ✔ |
| TF odom → camera_optical_link | translation (0.255, 0, 0.180), optical rotation ✔ |
| Wheel odometry while driving | 0.40 m/s measured on `/odom` during navigation (fault test) |
| Nav2 lifecycle (localization + navigation incl. route/docking servers) | "Managed nodes are active" ✔ |

## 4. End-to-end gauge missions (ground truth only used afterwards)

Three complete missions with the final code, two headless (g2, g3) and one with the Gazebo GUI and
RViz (gui). Decisions were identical in all three:

| Gauge | Truth | Decision sequence | g2 | g3 (archived) | gui |
|---|---|---|---|---|---|
| PG-11 clear | 4.20 | ACCEPT | 4.187 ± 0.104 | 4.205 ± 0.108 | 4.208 ± 0.105 |
| PG-12 occluded from the front | 6.80 | MOVE_NEW → ACCEPT (at −45°) | 6.769 ± 0.158 | 6.792 ± 0.151 | 6.812 ± 0.162 |
| PG-13 behind crate | 3.10 | RETRY_SAME → MOVE_NEW → RETRY_SAME → MOVE_NEW → RETRY_SAME → UNRESOLVED | UNRESOLVED | UNRESOLVED | UNRESOLVED |
| PG-14 above alarm | 8.70 | ACCEPT, alarm HIGH | 8.688 ± 0.109 | 8.701 ± 0.107 | 8.697 ± 0.105 |
| max \|error\| of accepted | | | 0.031 | **0.0085** | 0.012 |
| within reported 2σ | | | 3/3 | 3/3 | 3/3 |

(± values are the reported 95 % half-widths = 2σ.)

Archived run g3 (`docs/evidence/demo_run/`): mission COMPLETED, 204 s sim / 234 s wall, 7 viewpoints
visited, 7/7 navigations succeeded on the first attempt, final position errors 0.037–0.156 m
(mean 0.083 m), yaw errors 0.14–0.19 rad, return home SUCCEEDED, launch exit code 0.

Per-observation evidence (g3):

| Obs | Viewpoint | Flags | Visibility | Needle conf | View angle | Dial radius | Decision |
|---|---|---|---|---|---|---|---|
| PG-11 #1 | V0 frontal 1.5 m | – | 0.997 | 0.98 | 21° | 71 px | ACCEPT |
| PG-12 #1 | V0 frontal 1.5 m | OCCLUDED, LOW_NEEDLE_CONTRAST | 0.008 | 0.005 | 21° | 70 px | MOVE_NEW (different_side) |
| PG-12 #2 | V3 −45° 1.5 m | – | 0.87 | 0.46 | 46° | 57 px | ACCEPT |
| PG-13 #1–6 | V0, V1 (−30°), V2 (+30°) | TAG_NOT_VISIBLE | – | – | – | – | RETRY_SAME/MOVE_NEW → UNRESOLVED |
| PG-14 #1 | V0 frontal 1.5 m | – | 0.995 | 0.96 | 20° | 71 px | ACCEPT |

The frontal "view angle" is ≈ 20° because the camera (0.18 m high, tilted up 14°) looks up at the dial.
The PG-12 acceptance at −45° passed with modest margins (visibility 0.87 vs 0.85 limit) — see
KNOWN_LIMITATIONS.md.

Images: `docs/evidence/demo_run/images/` (raw, annotated, rectified for every observation) and the
Gazebo GUI screenshot `docs/evidence/gui/gazebo_gui_pg12.png` (taken with the `/gui/screenshot`
service during the GUI run).

## 5. Failure handling

| Test | How | Observed result | Evidence |
|---|---|---|---|
| Physical occlusion (hanging conduit box, invisible to LiDAR/map) | PG-12 in every mission | OCCLUDED detected → MOVE_NEW → −45° view → ACCEPT, error ≤ 0.031 bar | `demo_run/` |
| No valid reading possible (crate) | PG-13 in every mission | 3 viewpoints, 6 observations → UNRESOLVED, no value published | `demo_run/` |
| **Camera failure** | `scripts/fault_test.sh camera` kills `gz_camera_bridge` while driving to PG-11 | health_monitor: `/camera/image_raw: STALE`; reader: `NO_IMAGE (no fresh frame within 5.0 s; frames received so far: 111)` — no stale frame reused; RETRY_SAME → **UNRESOLVED** "camera delivers no frames: moving cannot fix a sensor fault"; mission completed, exit 0 | `fault_camera/` |
| **LiDAR failure** while driving | `scripts/fault_test.sh lidar` kills `gz_lidar_bridge` | `cmd_vel_gate -> STOPPED_SCAN_FAULT`; measured speed **0.400 m/s → 0.000 m/s within 0.5–1.0 s**; `/scan: STALE`; Nav2 goal aborted; mission **ABORTED** "LiDAR stream failed – stopping for safety"; report written; exit 0 | `fault_lidar/` (`odom_speed.txt`) |
| System not ready | first integration run: mission subscribed `/amcl_pose` volatile and missed the latched message | mission ABORTED after 180 s with "missing /amcl_pose", report written (bug fixed: transient-local subscription) | log in this file's history |
| Vision node crash | earlier pipeline: an rclpy logging error killed the vision node mid-mission | mission recorded the inspection as failed and continued (bug fixed + service wrapped so exceptions return an error result) | – |

## 6. Bugs found by running the system (and fixed)

1. Panel at x = 11.89 m was inside the 0.2 m east wall (inner face 11.8) → camera saw only wall.
   Fixed in the layout; regression test added.
2. ArUco tag had an 8 px quiet zone; the dark panel border merged with the tag at low resolution,
   shifting corners by ~3.6 px and biasing readings by up to 0.6 bar. Fixed with a one-cell quiet zone.
3. Planner "move closer" sent the robot back along an occluded line of sight (run g1, archived in
   `docs/evidence/run_g1_before_fixes/`). Fixed: failed directions are blocked at every distance.
4. σ model double-counted foreshortening and rejected a correct 45° reading (g1: 6.81 bar, σ 0.17).
   Replaced by the minor-axis resolution term; calibration test added.
5. `gz sim` server orphaned after shutdown; launch exit code 1 from a double shutdown. Fixed with the
   supervised wrapper and a guarded exit handler (all final runs: exit 0, no leftover processes).

## 7. Not verified / limitations of the evidence

* RViz2 and the Gazebo GUI were started and ran the full mission (windows confirmed with `xwininfo`,
  Gazebo screenshot captured), but RViz rendering was **not visually inspected**: WSLg does not
  allow root-window capture, and RViz logged a GLSL link error for the indexed-colour map shader
  (WSL Mesa driver), so the costmap colouring may not render on this machine.
* The GitHub Actions workflow was written but **has not run** (no repository published from here).
* Only the scenarios above were tested; no statistics over many randomised worlds.
